"""Standalone test: from SCAN_POSE, detect part, touch part, return SCAN_POSE.

Expected operator flow:
1) Manually park robot at SCAN_POSE first.
2) Run this script.
3) Script captures image immediately, computes pick point, touches part,
   then returns to SCAN_POSE and exits.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from vision.calibration import (
    build_pick_approach_pose,
    camera_to_base,
    pixel_to_camera_3d,
    sanitize_camera_depth_mm,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera


WINDOW_PREVIEW = "SCANPOSE TOUCH PREVIEW"


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy test.")
        raise SystemExit(0)


def wait_steady(rtde: RTDEClient, label: str) -> bool:
    ok = rtde.wait_steady(
        timeout_s=config.RTDE_WAIT_TIMEOUT,
        threshold=config.RTDE_STEADY_THRESHOLD,
        motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
    )
    if not ok:
        print(f"Canh bao: timeout khi cho robot dung tai buoc {label}.")
    return ok


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def capture_valid_frames(camera: FemtoCamera, max_attempts: int = 8):
    """Capture frames and require non-empty RGB/depth data."""
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

    raise RuntimeError("Khong lay duoc frame hop le (RGB/Depth deu rong)")


def _fmt_xyz(values) -> str:
    return "[" + ", ".join(f"{float(v):.4f}" for v in values) + "]"


def _fmt_pose(values) -> str:
    return "[" + ", ".join(f"{float(v):.4f}" for v in values) + "]"


def log_target_estimate(
    target,
    raw_depth_mm: float,
    safe_depth_mm: float,
    tcp_pose_at_capture,
    p_cam,
    p_base,
    approach_pose,
    touch_pose,
) -> None:
    tcp_xyz = np.array(tcp_pose_at_capture[:3], dtype=np.float64)
    scan_xyz = np.array(config.SCAN_POSE_TCP[:3], dtype=np.float64)
    p_base_np = np.array(p_base, dtype=np.float64)
    delta_vs_tcp = p_base_np - tcp_xyz
    delta_vs_scan = p_base_np - scan_xyz

    payload = {
        "label": target.label,
        "confidence": round(float(target.confidence), 4),
        "bbox": [int(v) for v in target.bbox],
        "center_px": [round(float(target.center[0]), 2), round(float(target.center[1]), 2)],
        "raw_depth_mm": round(float(raw_depth_mm), 2),
        "safe_depth_mm": round(float(safe_depth_mm), 2),
        "tcp_at_capture_m": [round(float(v), 6) for v in tcp_pose_at_capture],
        "p_cam_m": [round(float(v), 6) for v in p_cam],
        "p_base_m": [round(float(v), 6) for v in p_base],
        "delta_vs_tcp_m": [round(float(v), 6) for v in delta_vs_tcp],
        "delta_vs_scanpose_m": [round(float(v), 6) for v in delta_vs_scan],
        "approach_pose_m_rad": [round(float(v), 6) for v in approach_pose],
        "touch_pose_m_rad": [round(float(v), 6) for v in touch_pose],
    }
    print(f"Target 3D estimate: {json.dumps(payload, ensure_ascii=True)}")


def resolve_intrinsics_for_frame(frame_w: int, frame_h: int):
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

    # Protect against stale width/height metadata in camera_intrinsics.json.
    # If raw intrinsics already align with the current frame center much better
    # than the scaled version, trust the raw values and flag metadata mismatch.
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
                "cx/cy goc da hop ly hon so voi tam frame hien tai, nen bo qua auto-scale."
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


def build_preview_image(
    rgb: np.ndarray,
    detections,
    target,
    raw_depth_mm: float,
    safe_depth_mm: float,
    depth_was_clamped: bool,
    frame_center_uv,
    p_cam,
    p_base,
    depth_diag,
) -> np.ndarray:
    preview = rgb.copy()
    h, w = preview.shape[:2]
    fc_u, fc_v = int(frame_center_uv[0]), int(frame_center_uv[1])

    cv2.drawMarker(
        preview,
        (fc_u, fc_v),
        (255, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=22,
        thickness=2,
    )

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        color = (0, 0, 255)
        thickness = 2
        if target is not None and det is target:
            color = (0, 255, 0)
            thickness = 3
        cv2.rectangle(preview, (x1, y1), (x2, y2), color, thickness)
        cx, cy = int(det.center[0]), int(det.center[1])
        cv2.circle(preview, (cx, cy), 5, color, -1)
        label = f"{det.label} {det.confidence:.2f}"
        cv2.putText(
            preview,
            label,
            (x1, max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )
    if target is not None and depth_diag:
        cx, cy = [int(v) for v in depth_diag.get("center_px", [0, 0])]
        cv2.circle(preview, (cx, cy), 8, (0, 255, 255), 2)

        for key, color in (
            ("center_roi_bounds", (0, 255, 255)),
            ("inner_roi_bounds", (255, 165, 0)),
        ):
            bounds = depth_diag.get(key)
            if bounds and len(bounds) == 4:
                x1, y1, x2, y2 = [int(v) for v in bounds]
                cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)

        for px, py in depth_diag.get("inner_valid_points_sample", []):
            cv2.circle(preview, (int(px), int(py)), 1, (255, 255, 0), -1)

    info_lines = [
        f"frame={w}x{h} detections={len(detections)}",
        (
            f"target_px=({target.center[0]:.1f}, {target.center[1]:.1f}) "
            f"raw_depth={raw_depth_mm:.1f}mm safe_depth={safe_depth_mm:.1f}mm"
            if target is not None and depth_was_clamped
            else f"target_px=({target.center[0]:.1f}, {target.center[1]:.1f}) depth={safe_depth_mm:.1f}mm"
        ) if target is not None else "target=None",
        (
            "depth_roi center={:.2f} inner={:.2f}".format(
                float((depth_diag or {}).get("center_stats", {}).get("ratio", 0.0)),
                float((depth_diag or {}).get("inner_stats", {}).get("ratio", 0.0)),
            )
            if target is not None and depth_diag
            else "depth_roi center=-- inner=--"
        ),
        f"p_cam={_fmt_xyz(p_cam)} m" if p_cam is not None else "p_cam=None",
        f"p_base={_fmt_xyz(p_base)} m" if p_base is not None else "p_base=None",
        "Enter/y: tiep tuc motion | q/Esc/n: huy",
    ]
    for idx, line in enumerate(info_lines):
        y = 28 + idx * 26
        cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 1, cv2.LINE_AA)

    return preview


def show_preview_and_confirm(preview: np.ndarray, force: bool, save_path: Path) -> None:
    save_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(save_path), preview)
    print(f"Da luu preview: {save_path}")

    try:
        cv2.namedWindow(WINDOW_PREVIEW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_PREVIEW, min(preview.shape[1], 1400), min(preview.shape[0], 900))
        cv2.imshow(WINDOW_PREVIEW, preview)
        cv2.waitKey(50)
    except cv2.error:
        print("Canh bao: khong mo duoc cua so preview, chi luu anh debug.")
        if not force:
            confirm("Da xem preview luu file. Tiep tuc motion?", force=False)
        return

    if force:
        print("Preview hien thi. Dang tiep tuc theo --yes.")
        cv2.waitKey(150)
        return

    print("Preview dang mo. Nhan y/Enter de tiep tuc, n/q/Esc de huy.")
    while True:
        cv2.imshow(WINDOW_PREVIEW, preview)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10, ord("y"), ord("Y")):
            print("Xac nhan preview: tiep tuc motion.")
            return
        if key in (27, ord("q"), ord("Q"), ord("n"), ord("N")):
            print("Da huy test sau buoc preview.")
            raise SystemExit(0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect from SCAN_POSE, touch part, return SCAN_POSE"
    )
    parser.add_argument(
        "--robot-ip",
        default=config.ROBOT_IP,
        help="Robot IP (default from config)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip interactive safety confirmation",
    )
    parser.add_argument(
        "--touch-offset-z",
        type=float,
        default=0.005,
        help="Offset Z (m) from estimated surface for touch pose (default: 0.005)",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=0.2,
        help="Pause time in seconds at touch pose",
    )
    parser.add_argument(
        "--scanpose-tol-deg",
        type=float,
        default=3.0,
        help="Max allowed joint error (deg) from SCAN_POSE before starting vision",
    )
    parser.add_argument(
        "--save-preview-dir",
        default="captures/scanpose_touch",
        help="Thu muc luu anh preview debug",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Khong mo cua so preview, chi in log va chay tiep",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== SCANPOSE TOUCH TEST ===")
    print(f"Robot IP: {args.robot_ip}")
    print("Flow: scanpose -> vision -> touch -> return scanpose -> stop")

    confirm(
        "Robot da dung san o SCAN_POSE, workspace clear, san sang test cham phoi",
        args.yes,
    )

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(
        args.robot_ip,
        port=config.URSCRIPT_PORT,
        timeout=config.URSCRIPT_TIMEOUT,
    )
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

    try:
        rtde.connect()
        urscript.connect()
        camera.connect()

        # Inform operator if robot is far from taught SCAN_POSE.
        current_j = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_j, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            print(
                "Loi: robot chua dung dung SCAN_POSE. "
                f"Tolerance={args.scanpose_tol_deg:.2f} deg, actual={err_deg:.2f} deg."
            )
            print("Hay dua robot ve SCAN_POSE roi chay lai tool.")
            return 1
        print("Xac nhan vi tri SCAN_POSE: OK")
        print(f"SCAN_POSE TCP in config: {[round(v, 4) for v in config.SCAN_POSE_TCP]}")

        rgb, depth, cam_ts = capture_valid_frames(camera)
        # Read TCP immediately after capture to minimize frame/pose time skew.
        tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
        scanpose_delta = [tcp_pose_at_capture[i] - config.SCAN_POSE_TCP[i] for i in range(6)]
        print(f"TCP so voi SCAN_POSE config: {[round(v, 4) for v in scanpose_delta]}")

        frame_h, frame_w = depth.shape
        intr = resolve_intrinsics_for_frame(frame_w, frame_h)
        fx_eff = intr["fx"]
        fy_eff = intr["fy"]
        cx_eff = intr["cx"]
        cy_eff = intr["cy"]
        if intr["used_scale"]:
            print(
                "Canh bao: do phan giai stream khac baseline calibration, "
                f"auto-scale intrinsics sx={intr['sx']:.3f}, sy={intr['sy']:.3f} "
                f"(baseline={int(config.CAM_CALIB_WIDTH)}x{int(config.CAM_CALIB_HEIGHT)}, "
                f"stream={frame_w}x{frame_h})"
            )
        elif intr["reason"]:
            print(f"Canh bao: {intr['reason']}")
        print(
            f"Intrinsics su dung: fx={fx_eff:.2f}, fy={fy_eff:.2f}, "
            f"cx={cx_eff:.2f}, cy={cy_eff:.2f}"
        )

        ts_diff = abs(cam_ts - rtde_ts)
        if ts_diff > 0.1:
            print(f"Canh bao: frame/pose lech {ts_diff * 1000:.0f} ms")
        else:
            print(f"Timestamp sync OK: {ts_diff * 1000:.1f} ms")

        detections = detector.detect(rgb)
        print(f"Detections: {len(detections)}")
        if not detections:
            print("Khong phat hien phoi. Se ve SCAN_POSE va ket thuc.")
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_no_detection")
            return 1

        frame_center_uv = (depth.shape[1] / 2.0, depth.shape[0] / 2.0)
        target = detector.select_best_target(detections, depth, frame_center_uv)
        if target is None:
            print("Co detections nhung khong co target hop le theo depth.")
            for idx, det in enumerate(detections, start=1):
                depth_diag = camera.analyze_depth_roi(depth, det.bbox)
                print(
                    f"Depth debug detection #{idx}: "
                    f"{json.dumps({'bbox': det.bbox, 'center': det.center, 'depth_debug': getattr(det, 'depth_debug', {}), 'roi_debug': depth_diag}, ensure_ascii=True)}"
                )
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_no_valid_target")
            return 1

        u, v = target.center
        depth_mm = camera.get_reliable_depth(depth, target.bbox)
        print(
            f"Target: label={target.label}, conf={target.confidence:.3f}, "
            f"center=({u:.1f},{v:.1f}), depth={depth_mm:.1f} mm"
        )
        if depth_mm <= 0:
            print("Depth khong hop le. Se ve SCAN_POSE va ket thuc.")
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_invalid_depth")
            return 1

        raw_depth_mm = depth_mm
        depth_mm, was_clamped, min_safe_depth_mm = sanitize_camera_depth_mm(
            depth_mm,
            config.T_CAM_TO_TCP,
            margin_below_tcp_m=config.PICK_MIN_DESCENT_M,
        )
        if was_clamped:
            clamp_delta_mm = abs(depth_mm - raw_depth_mm)
            print(
                "Canh bao: depth {:.1f} mm nho hon camera->TCP standoff, "
                "auto-clamp len {:.1f} mm de tranh robot di nguoc len.".format(
                    raw_depth_mm,
                    min_safe_depth_mm,
                )
            )
            if clamp_delta_mm > config.DEPTH_MAX_CLAMP_DELTA_MM:
                print(
                    "Loi: raw depth lech qua xa so voi safe depth "
                    f"({clamp_delta_mm:.1f}mm > {config.DEPTH_MAX_CLAMP_DELTA_MM:.1f}mm). "
                    "Depth capture dang sai, dung motion de tranh tinh pick sai."
                )
                depth_diag = camera.analyze_depth_roi(depth, target.bbox)
                print(f"Depth debug target: {json.dumps(depth_diag, ensure_ascii=True)}")
                urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                wait_steady(rtde, "return_scanpose_bad_depth_capture")
                return 1

        p_cam = pixel_to_camera_3d(
            u,
            v,
            depth_mm,
            fx_eff,
            fy_eff,
            cx_eff,
            cy_eff,
        )
        p_base = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
        print(
            f"p_cam(m)={ [round(vv, 4) for vv in p_cam] }, "
            f"p_base(m)={ [round(vv, 4) for vv in p_base] }"
        )
        depth_diag = camera.analyze_depth_roi(depth, target.bbox)
        print(f"Depth debug target: {json.dumps(depth_diag, ensure_ascii=True)}")

        preview = build_preview_image(
            rgb=rgb,
            detections=detections,
            target=target,
            raw_depth_mm=raw_depth_mm,
            safe_depth_mm=depth_mm,
            depth_was_clamped=was_clamped,
            frame_center_uv=frame_center_uv,
            p_cam=p_cam,
            p_base=p_base,
            depth_diag=depth_diag,
        )
        preview_path = (
            Path(args.save_preview_dir)
            / f"scanpose_touch_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        )
        if not args.no_preview:
            show_preview_and_confirm(preview, args.yes, preview_path)
        else:
            preview_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(preview_path), preview)
            print(f"Da luu preview: {preview_path}")

        approach_pose = build_pick_approach_pose(
            p_base,
            config.PICK_APPROACH_OFFSET_Z,
            tool_rx=config.TOOL_DOWN_RX,
            tool_ry=config.TOOL_DOWN_RY,
            tool_rz=config.TOOL_DOWN_RZ,
        )
        touch_pose = build_pick_approach_pose(
            p_base,
            args.touch_offset_z,
            tool_rx=config.TOOL_DOWN_RX,
            tool_ry=config.TOOL_DOWN_RY,
            tool_rz=config.TOOL_DOWN_RZ,
        )
        retreat_pose = build_pick_approach_pose(
            p_base,
            config.PICK_RETREAT_OFFSET_Z,
            tool_rx=config.TOOL_DOWN_RX,
            tool_ry=config.TOOL_DOWN_RY,
            tool_rz=config.TOOL_DOWN_RZ,
        )

        print(f"Approach pose: {approach_pose}")
        print(f"Touch pose:    {touch_pose}")
        log_target_estimate(
            target=target,
            raw_depth_mm=raw_depth_mm,
            safe_depth_mm=depth_mm,
            tcp_pose_at_capture=tcp_pose_at_capture,
            p_cam=p_cam,
            p_base=p_base,
            approach_pose=approach_pose,
            touch_pose=touch_pose,
        )

        if touch_pose[2] >= tcp_pose_at_capture[2]:
            print(
                "Loi: touch pose van nam cao hon TCP tai luc chup "
                f"({touch_pose[2]:.4f} >= {tcp_pose_at_capture[2]:.4f}). Dung de tranh robot di nguoc."
            )
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_upward_guard")
            return 1

        # Guard rail: skip motion if computed target is too far from current TCP.
        dx = approach_pose[0] - tcp_pose_at_capture[0]
        dy = approach_pose[1] - tcp_pose_at_capture[1]
        dz = touch_pose[2] - tcp_pose_at_capture[2]
        planar_dist = (dx * dx + dy * dy) ** 0.5
        if planar_dist > 0.25 or abs(dz) > 0.35:
            print(
                "Loi: target pose lech qua xa so voi TCP tai luc capture. "
                f"planar={planar_dist:.3f}m, dz={dz:.3f}m. Dung de tranh va cham."
            )
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_outlier_target")
            return 1

        urscript.move_linear(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        wait_steady(rtde, "approach")

        # Move slowly when touching the part surface.
        urscript.move_linear(touch_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
        wait_steady(rtde, "touch")

        if args.hold_s > 0:
            time.sleep(args.hold_s)

        urscript.move_linear(retreat_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        wait_steady(rtde, "retreat")

        urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
        wait_steady(rtde, "return_scanpose")

        print("Hoan tat test: da cham phoi va quay lai SCAN_POSE.")
        return 0

    except Exception as exc:
        print(f"Loi khi chay test: {exc}")
        # Best-effort return to SCAN_POSE before exit.
        try:
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_on_error")
        except Exception:
            pass
        return 1
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            camera.disconnect()
        except Exception:
            pass
        try:
            rtde.disconnect()
        except Exception:
            pass
        try:
            urscript.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
