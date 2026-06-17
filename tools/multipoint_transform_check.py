"""Collect multiple vision-vs-teach samples to diagnose transform error patterns.

For each sample:
1) Robot must be at SCAN_POSE.
2) Tool captures one frame and computes the current vision target.
3) Tool saves an annotated image with VISION_PICK marked.
4) Operator jogs TCP onto that exact point on the real part.
5) Tool reads TCP actual as expected_base and computes error.

After N samples the tool prints summary statistics:
- mean dx/dy/dz
- std dx/dy/dz
- mean total error

Use this to distinguish:
- fixed bias -> likely hand-eye / TCP / mounting
- high variance -> likely depth / vision / lighting
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
from vision.calibration import camera_to_base, pixel_to_camera_3d
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
from vision.tray_holes import (
    assign_pick_to_layout_hole,
    detect_tray_holes,
    match_tray_layout_to_detected_holes,
    snap_pick_to_nearest_hole,
)
from vision.tray_reference import refine_base_xy_with_checkerboard


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


def draw_overlay(frame_bgr, target, holes=None, layout_match=None):
    overlay = frame_bgr.copy()
    if holes:
        for hole in holes:
            hu, hv = [int(round(v)) for v in hole["center"]]
            hr = int(round(hole["radius_px"]))
            cv2.circle(overlay, (hu, hv), hr, (0, 200, 255), 2)
    if target is not None:
        x1, y1, x2, y2 = [int(v) for v in target.bbox]
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 180, 0), 2)
        pu, pv = int(round(target.pick_point[0])), int(round(target.pick_point[1]))
        cv2.circle(overlay, (pu, pv), 7, (0, 0, 255), -1)
        cv2.putText(overlay, "VISION_PICK", (pu + 10, pv - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        px1, py1, px2, py2 = [int(v) for v in target.pick_bbox]
        cv2.rectangle(overlay, (px1, py1), (px2, py2), (0, 255, 255), 2)
    if layout_match is not None:
        for idx, hole_uv in enumerate(layout_match["projected_holes_uv"], start=1):
            hu, hv = [int(round(v)) for v in hole_uv]
            cv2.circle(overlay, (hu, hv), 10, (255, 0, 255), 2)
            cv2.putText(overlay, f"H{idx}", (hu + 8, hv - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)
    return overlay


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect multi-point vision-vs-teach transform samples")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--samples", type=int, default=3, help="Number of samples to collect")
    parser.add_argument("--scanpose-tol-deg", type=float, default=3.0)
    parser.add_argument(
        "--raw-transform-only",
        action="store_true",
        help="Disable tray-hole snap and checkerboard XY refine to measure only pixel+depth+T_CAM_TO_TCP bias",
    )
    parser.add_argument("--output", default="logs/multipoint_transform_check.json", help="JSON output path")
    parser.add_argument("--image-dir", default="logs/multipoint_transform_check", help="Annotated image directory")
    return parser.parse_args()


def classify(errors_mm: np.ndarray) -> str:
    mean_vec = errors_mm.mean(axis=0)
    std_vec = errors_mm.std(axis=0)
    mean_norm = float(np.mean(np.linalg.norm(errors_mm, axis=1)))
    max_std = float(np.max(std_vec))
    mean_abs = np.abs(mean_vec)

    if mean_norm < 10.0:
        return "Sai so nho; transform co ve on, nghi van de van hanh hoac tieu chi pick."
    if max_std < 8.0 and np.max(mean_abs) > 15.0:
        return "Sai so co tinh lap lai; nghi T_CAM_TO_TCP, TCP active, hoac mounting camera/tool."
    if max_std > 15.0:
        return "Sai so dao dong lon; nghi depth, vision target, anh sang, hoac be mat phan xa."
    return "Sai so o muc trung gian; can them mau de tach depth bias voi calibration bias."


def main() -> int:
    args = parse_args()

    print("=== MULTIPOINT TRANSFORM CHECK ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Samples: {args.samples}")
    print(f"Raw transform only: {args.raw_transform_only}")
    confirm(
        "Robot da dung san o SCAN_POSE. Moi mau: tool chup anh, ban xem anh, jog TCP cham DUNG diem VISION_PICK, roi nhan Enter.",
        args.yes,
    )

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
    image_dir = Path(args.image_dir)
    image_dir.mkdir(parents=True, exist_ok=True)

    samples = []

    try:
        rtde.connect()
        camera.connect()

        for sample_idx in range(1, args.samples + 1):
            print(f"\n=== SAMPLE {sample_idx}/{args.samples} ===")
            input("Dat diem/phoi can do roi nhan Enter de chup...")

            current_joints = rtde.get_joint_positions()
            err_deg = joint_max_error_deg(current_joints, config.SCAN_POSE_JOINTS)
            print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
            if err_deg > args.scanpose_tol_deg:
                raise RuntimeError("Robot khong dung dung SCAN_POSE truoc khi chup.")

            rgb, depth, cam_ts = capture_valid_frames(camera)
            tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
            print(f"TCP at capture: {[round(v, 4) for v in tcp_pose_at_capture]}")
            print(f"Frame/pose delta: {abs(cam_ts - rtde_ts) * 1000.0:.1f} ms")

            frame_h, frame_w = depth.shape
            sx = frame_w / float(config.CAM_CALIB_WIDTH)
            sy = frame_h / float(config.CAM_CALIB_HEIGHT)
            fx_eff = config.CAM_FX * sx
            fy_eff = config.CAM_FY * sy
            cx_eff = config.CAM_CX * sx
            cy_eff = config.CAM_CY * sy
            print(
                f"Intrinsics used: fx={fx_eff:.2f}, fy={fy_eff:.2f}, "
                f"cx={cx_eff:.2f}, cy={cy_eff:.2f}"
            )

            detections = detector.detect(rgb)
            print(f"Detections: {len(detections)}")
            if not detections:
                raise RuntimeError("Khong phat hien phoi.")

            frame_center_uv = (frame_w / 2.0, frame_h / 2.0)
            target = detector.select_best_target(detections, depth, frame_center_uv)
            if target is None:
                raise RuntimeError("Co detections nhung khong co target hop le theo depth.")

            target = detector.refine_pick_point(rgb, target, depth)
            u, v = target.pick_point
            depth_mm, depth_bbox = detector.resolve_pick_depth(depth, target)
            if depth_mm <= 0:
                raise RuntimeError("Depth khong hop le.")

            holes = []
            layout_match = None
            hole_ref_enabled = config.TRAY_HOLE_REF_ENABLED and not args.raw_transform_only
            tray_ref_enabled = config.TRAY_REF_ENABLED and not args.raw_transform_only

            if hole_ref_enabled:
                holes = detect_tray_holes(
                    rgb,
                    min_radius_px=config.TRAY_HOLE_MIN_RADIUS_PX,
                    max_radius_px=config.TRAY_HOLE_MAX_RADIUS_PX,
                    min_dist_px=config.TRAY_HOLE_MIN_DIST_PX,
                )
                layout_match = match_tray_layout_to_detected_holes(
                    config.TRAY_LAYOUT_PATH,
                    holes,
                    max_reproj_error_px=config.TRAY_LAYOUT_MAX_REPROJ_ERR_PX,
                    max_candidate_holes=config.TRAY_LAYOUT_MAX_CANDIDATE_HOLES,
                )
                snapped_hole = None
                if layout_match is not None:
                    snapped_hole = assign_pick_to_layout_hole(
                        [u, v],
                        layout_match,
                        max_assign_dist_px=config.TRAY_LAYOUT_MAX_ASSIGN_DIST_PX,
                    )
                if snapped_hole is None:
                    snapped_hole = snap_pick_to_nearest_hole(
                        [u, v],
                        holes,
                        max_snap_dist_px=config.TRAY_HOLE_MAX_SNAP_DIST_PX,
                    )
                if snapped_hole is not None:
                    u, v = snapped_hole["center"]
                    target.pick_point = [u, v]

            print(
                f"Target: label={target.label}, conf={target.confidence:.3f}, "
                f"pick=({u:.1f},{v:.1f}), depth={depth_mm:.1f} mm, "
                f"bbox={target.bbox}, pick_bbox={target.pick_bbox}, depth_bbox={depth_bbox}, source={target.pick_source}"
            )

            p_cam = pixel_to_camera_3d(u, v, depth_mm, fx_eff, fy_eff, cx_eff, cy_eff)
            p_base_raw = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
            xy_source = "depth_only"
            if tray_ref_enabled:
                p_base_raw, xy_source = refine_base_xy_with_checkerboard(
                    rgb,
                    u,
                    v,
                    p_base_raw,
                    tcp_pose_at_capture,
                    config.T_CAM_TO_TCP,
                    fx_eff,
                    fy_eff,
                    cx_eff,
                    cy_eff,
                    config.TRAY_REF_INNER_CORNERS,
                    config.TRAY_REF_SQUARE_SIZE_M,
                )
            p_base = [
                p_base_raw[0] + config.PICK_OFFSET_X,
                p_base_raw[1] + config.PICK_OFFSET_Y,
                p_base_raw[2] + config.PICK_OFFSET_Z,
            ]

            image_path = image_dir / f"sample_{sample_idx:02d}.jpg"
            overlay = draw_overlay(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), target, holes, layout_match)
            cv2.imwrite(str(image_path), overlay)
            print(f"Saved annotated image: {image_path}")
            print(f"Computed p_base(m): {[round(vv, 4) for vv in p_base]}")
            print(f"xy_source: {xy_source}")
            print("Hay mo anh, jog TCP cham DUNG diem VISION_PICK tren phoi, roi quay lai day.")
            input("Nhan Enter khi TCP da cham dung diem de doc expected_base tu robot...")

            expected_pose = rtde.get_tcp_pose()
            expected_base = list(expected_pose[:3])
            error_mm = (np.array(p_base) - np.array(expected_base)) * 1000.0
            error_norm_mm = float(np.linalg.norm(error_mm))

            print(f"Expected base tu TCP actual (m): {[round(v, 4) for v in expected_base]}")
            print(
                f"error(mm): dx={error_mm[0]:.1f}, dy={error_mm[1]:.1f}, "
                f"dz={error_mm[2]:.1f}, norm={error_norm_mm:.1f}"
            )

            samples.append(
                {
                    "sample_index": sample_idx,
                    "image_path": str(image_path),
                    "tcp_at_capture": tcp_pose_at_capture,
                    "pick_uv": [float(u), float(v)],
                    "depth_mm": float(depth_mm),
                    "p_cam_m": [float(vv) for vv in p_cam],
                    "p_base_raw_m": [float(vv) for vv in p_base_raw],
                    "p_base_final_m": [float(vv) for vv in p_base],
                    "expected_base_m": [float(vv) for vv in expected_base],
                    "error_mm": [float(vv) for vv in error_mm],
                    "error_norm_mm": error_norm_mm,
                    "xy_source": xy_source,
                    "raw_transform_only": args.raw_transform_only,
                    "target_bbox": list(target.bbox),
                    "target_pick_bbox": list(target.pick_bbox),
                    "target_pick_source": target.pick_source,
                }
            )

            if sample_idx < args.samples:
                print("Dua robot ve lai SCAN_POSE truoc khi lay mau tiep theo.")
                input("Nhan Enter sau khi robot da o lai SCAN_POSE...")

        errors_mm = np.array([sample["error_mm"] for sample in samples], dtype=np.float64)
        norms_mm = np.array([sample["error_norm_mm"] for sample in samples], dtype=np.float64)
        mean_vec = errors_mm.mean(axis=0)
        std_vec = errors_mm.std(axis=0)
        summary = {
            "mean_error_mm": mean_vec.tolist(),
            "std_error_mm": std_vec.tolist(),
            "mean_norm_mm": float(np.mean(norms_mm)),
            "max_norm_mm": float(np.max(norms_mm)),
            "classification": classify(errors_mm),
        }

        print("\n=== SUMMARY ===")
        print(
            f"mean_error(mm): dx={mean_vec[0]:.1f}, dy={mean_vec[1]:.1f}, dz={mean_vec[2]:.1f}"
        )
        print(
            f"std_error(mm):  dx={std_vec[0]:.1f}, dy={std_vec[1]:.1f}, dz={std_vec[2]:.1f}"
        )
        print(f"mean_norm(mm):  {summary['mean_norm_mm']:.1f}")
        print(f"max_norm(mm):   {summary['max_norm_mm']:.1f}")
        print(f"Nhan dinh: {summary['classification']}")

        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps({"samples": samples, "summary": summary}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(f"Saved report: {output_path}")
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
