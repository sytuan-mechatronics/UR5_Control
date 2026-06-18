"""Safe coordinate diagnosis for vision picks.

This tool does not command robot motion. It only:
1) verifies robot is already near SCAN_POSE,
2) captures RGB/depth + TCP pose,
3) computes p_cam / p_base from the current backend logic,
4) optionally pauses so operator can jog robot manually onto the same point,
5) compares computed coordinates against actual TCP coordinates,
6) writes JSON + Markdown logs with a subjective diagnosis.
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from vision.calibration import (
    camera_to_base,
    min_safe_camera_depth_m,
    pixel_to_camera_3d,
    sanitize_camera_depth_mm,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy.")
        raise SystemExit(0)


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def capture_valid_frames(camera: FemtoCamera, max_attempts: int = 8):
    for attempt in range(1, max_attempts + 1):
        rgb, depth, cam_ts = camera.get_frames_with_timestamp()
        rgb_nonzero = int(np.count_nonzero(rgb))
        depth_nonzero = int(np.count_nonzero(depth))
        print(
            f"Frame attempt {attempt}/{max_attempts}: "
            f"rgb_nonzero={rgb_nonzero}, depth_nonzero={depth_nonzero}, "
            f"shape_rgb={rgb.shape}, shape_depth={depth.shape}"
        )
        if rgb_nonzero > 0 and depth_nonzero > 0:
            return rgb, depth, cam_ts
        time.sleep(0.05)
    raise RuntimeError("Khong lay duoc frame hop le")


def resolve_intrinsics_for_frame(frame_w: int, frame_h: int) -> Dict[str, float]:
    sx = frame_w / float(config.CAM_CALIB_WIDTH)
    sy = frame_h / float(config.CAM_CALIB_HEIGHT)
    fx_scaled = config.CAM_FX * sx
    fy_scaled = config.CAM_FY * sy
    cx_scaled = config.CAM_CX * sx
    cy_scaled = config.CAM_CY * sy
    frame_cx = frame_w / 2.0
    frame_cy = frame_h / 2.0
    raw_center_err = abs(config.CAM_CX - frame_cx) + abs(config.CAM_CY - frame_cy)
    scaled_center_err = abs(cx_scaled - frame_cx) + abs(cy_scaled - frame_cy)

    if (
        (abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3)
        and raw_center_err + 5.0 < scaled_center_err
    ):
        return {
            "fx": config.CAM_FX,
            "fy": config.CAM_FY,
            "cx": config.CAM_CX,
            "cy": config.CAM_CY,
            "sx": sx,
            "sy": sy,
            "used_scale": False,
            "reason": (
                "Metadata baseline intrinsics co ve da stale: "
                "cx/cy goc hop ly hon tam frame hien tai, bo qua auto-scale."
            ),
        }

    return {
        "fx": fx_scaled,
        "fy": fy_scaled,
        "cx": cx_scaled,
        "cy": cy_scaled,
        "sx": sx,
        "sy": sy,
        "used_scale": abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3,
        "reason": "",
    }


def _round_list(values, ndigits: int = 6) -> List[float]:
    return [round(float(v), ndigits) for v in values]


def build_assessment(payload: Dict[str, object]) -> List[str]:
    findings: List[str] = []
    if payload.get("scanpose_joint_error_deg", 0.0) > payload.get("scanpose_tol_deg", 3.0):
        findings.append(
            "Robot chua dung dung SCAN_POSE luc chup, nen moi so sanh sau do co the bi lech."
        )

    ts_ms = float(payload.get("frame_pose_delta_ms", 0.0))
    if ts_ms > 100.0:
        findings.append(
            "Frame va TCP pose lech thoi gian qua 100ms, nghi van de dong bo camera/robot."
        )

    raw_depth = float(payload.get("raw_depth_mm", 0.0))
    safe_depth = float(payload.get("safe_depth_mm", 0.0))
    min_safe = float(payload.get("min_safe_depth_mm", 0.0))
    if raw_depth > 0 and safe_depth > raw_depth + 1.0:
        findings.append(
            "Depth do duoc nho hon standoff camera->TCP, backend phai clamp len. Thuong la dau hieu TCP/hand-eye chua khop hoac depth cham vao vung hole."
        )
    elif raw_depth > 0 and raw_depth < min_safe - 5.0:
        findings.append(
            "Raw depth nam duoi nguong an toan, can uu tien kiem tra lai vi tri TCP moi va ma tran hand-eye."
        )

    delta_vs_tcp = payload.get("delta_vs_tcp_m")
    if isinstance(delta_vs_tcp, list) and len(delta_vs_tcp) == 3:
        planar_mm = math.hypot(delta_vs_tcp[0], delta_vs_tcp[1]) * 1000.0
        dz_mm = delta_vs_tcp[2] * 1000.0
        if planar_mm > 180.0 and abs(dz_mm) < 30.0:
            findings.append(
                "Diem backend tinh ra lech XY rat lon so voi TCP tai luc chup trong khi Z gan bang nhau. Chu quan minh nghi manh la TCP/day tool hoac T_cam_to_tcp dang sai hon la do depth."
            )
        elif dz_mm > 20.0:
            findings.append(
                "Diem pick tinh ra nam cao hon TCP luc chup. Neu lap lai nhieu lan thi co kha nang truc Z hoac chieu offset dang bi nguoc."
            )

    expected_base = payload.get("expected_base_m")
    p_base = payload.get("p_base_m")
    if isinstance(expected_base, list) and isinstance(p_base, list):
        err = np.array(p_base, dtype=np.float64) - np.array(expected_base, dtype=np.float64)
        err_mm = err * 1000.0
        planar_err_mm = float(np.linalg.norm(err_mm[:2]))
        z_err_mm = abs(float(err_mm[2]))
        total_err_mm = float(np.linalg.norm(err_mm))

        if total_err_mm <= 15.0:
            findings.append(
                "Sai so tong nho, backend dang kha hop ly. Neu van pick loi thi uu tien xem gripper, CoG, hoac quang duong motion."
            )
        elif z_err_mm > max(25.0, planar_err_mm * 1.2):
            findings.append(
                "Sai so chu yeu o Z. Chu quan minh nghi depth, mat phan xa, anh sang, hoac vung lay depth dang khong trung diem cham that."
            )
        elif planar_err_mm > max(40.0, z_err_mm * 1.5):
            findings.append(
                "Sai so chu yeu o XY. Chu quan minh nghi TCP day lai chua dung, hand-eye sai he truc, hoac camera gan tool khong dung nhu luc calib."
            )
        else:
            findings.append(
                "Sai so ca XY va Z deu dang ke. Thuong la to hop cua TCP + hand-eye, khong chi rieng depth."
            )

    if not findings:
        findings.append(
            "Chua du du lieu de ket luan manh. Hay chay voi --teach-expected de co toa do thuc te tren cung diem."
        )
    return findings


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe diagnosis for vision-to-robot coordinate errors",
    )
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--scanpose-tol-deg", type=float, default=3.0)
    parser.add_argument(
        "--teach-expected",
        action="store_true",
        help="Pause after capture so operator jogs TCP onto the same point, then read actual TCP xyz",
    )
    parser.add_argument("--expected-x", type=float, help="Actual measured X in base frame (m)")
    parser.add_argument("--expected-y", type=float, help="Actual measured Y in base frame (m)")
    parser.add_argument("--expected-z", type=float, help="Actual measured Z in base frame (m)")
    parser.add_argument(
        "--log-dir",
        default="vision_coordinate_diagnosis/logs",
        help="Directory to save json/md diagnosis logs",
    )
    return parser.parse_args()


def read_expected_xyz(args: argparse.Namespace) -> Optional[List[float]]:
    if args.teach_expected:
        return None
    if args.expected_x is not None and args.expected_y is not None and args.expected_z is not None:
        return [args.expected_x, args.expected_y, args.expected_z]
    return None


def write_reports(log_dir: Path, stem: str, payload: Dict[str, object], assessment: List[str]) -> None:
    log_dir.mkdir(parents=True, exist_ok=True)
    json_path = log_dir / f"{stem}.json"
    md_path = log_dir / f"{stem}.md"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    lines = [
        "# Chan doan loi toa do vision pick",
        "",
        f"- Thoi gian: `{payload['timestamp']}`",
        f"- Robot IP: `{payload['robot_ip']}`",
        f"- Detections: `{payload['detections']}`",
        f"- Depth source: `{payload.get('depth_source', 'unknown')}`",
        "",
        "## So lieu chinh",
        "",
        f"- TCP luc chup: `{payload.get('tcp_at_capture_m')}`",
        f"- p_cam: `{payload.get('p_cam_m')}`",
        f"- p_base: `{payload.get('p_base_m')}`",
        f"- delta_vs_tcp: `{payload.get('delta_vs_tcp_m')}`",
        f"- Raw depth / safe depth / min safe (mm): `{payload.get('raw_depth_mm')}` / `{payload.get('safe_depth_mm')}` / `{payload.get('min_safe_depth_mm')}`",
        f"- Frame-pose delta (ms): `{payload.get('frame_pose_delta_ms')}`",
        "",
        "## Danh gia chu quan",
        "",
    ]
    lines.extend([f"- {item}" for item in assessment])
    expected_base = payload.get("expected_base_m")
    if expected_base is not None:
        lines.extend(
            [
                "",
                "## So sanh voi diem thuc te",
                "",
                f"- expected_base: `{expected_base}`",
                f"- error_mm: `{payload.get('error_mm')}`",
                f"- total_error_mm: `{payload.get('total_error_mm')}`",
            ]
        )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Da luu log JSON: {json_path}")
    print(f"Da luu bao cao MD: {md_path}")


def main() -> int:
    args = parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    print("=== DIAGNOSE PICK COORDINATE ERROR ===")
    print("Tool nay KHONG tu dong move robot.")
    print(f"Robot IP: {args.robot_ip}")
    confirm(
        "Robot dang dung gan SCAN_POSE, workspace clear, va neu can ban se jog tay TCP vao cung diem vision chon",
        args.yes,
    )

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
    expected_base = read_expected_xyz(args)

    payload: Dict[str, object] = {
        "timestamp": timestamp,
        "robot_ip": args.robot_ip,
        "scanpose_tol_deg": args.scanpose_tol_deg,
    }

    try:
        rtde.connect()
        camera.connect()

        current_joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_joints, config.SCAN_POSE_JOINTS)
        payload["scanpose_joint_error_deg"] = round(float(err_deg), 4)
        print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            payload["status"] = "scanpose_mismatch"
            assessment = build_assessment(payload)
            write_reports(Path(args.log_dir), timestamp, payload, assessment)
            print("Loi: robot chua dung dung SCAN_POSE.")
            return 1

        rgb, depth, cam_ts = capture_valid_frames(camera)
        tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
        frame_h, frame_w = depth.shape
        intr = resolve_intrinsics_for_frame(frame_w, frame_h)
        frame_center_uv = (frame_w / 2.0, frame_h / 2.0)
        detections = detector.detect(rgb)

        payload.update(
            {
                "status": "captured",
                "frame_shape_rgb": list(rgb.shape),
                "frame_shape_depth": list(depth.shape),
                "tcp_at_capture_m": _round_list(tcp_pose_at_capture),
                "frame_pose_delta_ms": round(abs(cam_ts - rtde_ts) * 1000.0, 3),
                "intrinsics": intr,
                "detections": len(detections),
                "min_safe_depth_mm": round(
                    min_safe_camera_depth_m(
                        np.array(config.T_CAM_TO_TCP, dtype=np.float64),
                        margin_below_tcp_m=config.PICK_MIN_DESCENT_M,
                    )
                    * 1000.0,
                    3,
                ),
            }
        )

        if intr["reason"]:
            print(f"Canh bao: {intr['reason']}")
        print(
            f"Intrinsics su dung: fx={intr['fx']:.2f}, fy={intr['fy']:.2f}, "
            f"cx={intr['cx']:.2f}, cy={intr['cy']:.2f}"
        )
        print(f"Timestamp sync: {payload['frame_pose_delta_ms']:.1f} ms")
        print(f"Detections: {len(detections)}")

        if not detections:
            payload["status"] = "no_detections"
            assessment = build_assessment(payload)
            write_reports(Path(args.log_dir), timestamp, payload, assessment)
            print("Khong phat hien phoi.")
            return 1

        target = detector.select_best_target(detections, depth, frame_center_uv)
        if target is None:
            payload["status"] = "no_valid_target_depth"
            payload["depth_debug_all"] = [
                {
                    "bbox": det.bbox,
                    "center": det.center,
                    "depth_debug": getattr(det, "depth_debug", {}),
                    "roi_debug": camera.analyze_depth_roi(depth, det.bbox),
                }
                for det in detections
            ]
            assessment = build_assessment(payload)
            write_reports(Path(args.log_dir), timestamp, payload, assessment)
            print("Co detections nhung khong co target hop le theo depth.")
            return 1

        depth_debug = getattr(target, "depth_debug", {})
        raw_depth_mm = 0.0
        depth_source = str(depth_debug.get("depth_source", "unknown"))
        for key in (
            "micro_median_depth_mm",
            "inner_median_depth_mm",
            "nearest_median_depth_mm",
            "bbox_median_depth_mm",
        ):
            value = float(depth_debug.get(key, 0.0) or 0.0)
            if value > 0:
                raw_depth_mm = value
                break

        if raw_depth_mm <= 0.0:
            raw_depth_mm = float(camera.get_reliable_depth(depth, target.bbox))

        safe_depth_mm, was_clamped, min_safe_depth_mm = sanitize_camera_depth_mm(
            raw_depth_mm,
            config.T_CAM_TO_TCP,
            margin_below_tcp_m=config.PICK_MIN_DESCENT_M,
        )

        u, v = target.center
        p_cam = pixel_to_camera_3d(
            u,
            v,
            safe_depth_mm,
            intr["fx"],
            intr["fy"],
            intr["cx"],
            intr["cy"],
        )
        p_base = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
        tcp_xyz = np.array(tcp_pose_at_capture[:3], dtype=np.float64)
        delta_vs_tcp = np.array(p_base, dtype=np.float64) - tcp_xyz

        payload.update(
            {
                "status": "target_computed",
                "target": {
                    "label": target.label,
                    "confidence": round(float(target.confidence), 4),
                    "bbox": [int(v) for v in target.bbox],
                    "center_px": [round(float(u), 3), round(float(v), 3)],
                },
                "depth_source": depth_source,
                "raw_depth_mm": round(float(raw_depth_mm), 3),
                "safe_depth_mm": round(float(safe_depth_mm), 3),
                "depth_was_clamped": bool(was_clamped),
                "min_safe_depth_mm": round(float(min_safe_depth_mm), 3),
                "depth_debug_target": {
                    "depth_debug": depth_debug,
                    "roi_debug": camera.analyze_depth_roi(depth, target.bbox),
                },
                "p_cam_m": _round_list(p_cam),
                "p_base_m": _round_list(p_base),
                "delta_vs_tcp_m": _round_list(delta_vs_tcp),
            }
        )

        print(
            f"Target: label={target.label}, conf={target.confidence:.3f}, "
            f"center=({u:.1f},{v:.1f}), raw_depth={raw_depth_mm:.1f} mm, safe_depth={safe_depth_mm:.1f} mm"
        )
        print(f"p_cam(m): {_round_list(p_cam, 4)}")
        print(f"p_base(m): {_round_list(p_base, 4)}")
        print(f"delta_vs_tcp(m): {_round_list(delta_vs_tcp, 4)}")

        if args.teach_expected:
            print("\n=== TEACH EXPECTED BASE ===")
            print("Hay dung freedrive/jog tay dua TCP cham DUNG diem vision vua chon.")
            print("Tool nay khong tu move, nen buoc nay an toan hon so voi auto-pick.")
            input("Nhan Enter khi TCP da cham dung diem de doc toa do thuc te...")
            expected_pose = rtde.get_tcp_pose()
            expected_base = list(expected_pose[:3])

        if expected_base is not None:
            error_vec = np.array(p_base, dtype=np.float64) - np.array(expected_base, dtype=np.float64)
            error_mm = error_vec * 1000.0
            payload["expected_base_m"] = _round_list(expected_base)
            payload["error_mm"] = _round_list(error_mm, 3)
            payload["total_error_mm"] = round(float(np.linalg.norm(error_mm)), 3)
            print(f"expected_base(m): {_round_list(expected_base, 4)}")
            print(f"error(mm): {_round_list(error_mm, 2)}")
            print(f"total_error(mm): {payload['total_error_mm']}")

        assessment = build_assessment(payload)
        payload["assessment"] = assessment
        write_reports(Path(args.log_dir), timestamp, payload, assessment)

        print("\nDanh gia chu quan:")
        for item in assessment:
            print(f"- {item}")
        return 0

    finally:
        try:
            camera.disconnect()
        except Exception:
            pass
        try:
            rtde.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
