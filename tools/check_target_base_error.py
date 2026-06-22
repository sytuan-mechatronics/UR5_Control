"""Compare computed target base coordinate against a measured ground-truth base point.

Workflow:
1) Park robot at SCAN_POSE.
2) Place one known reference target/part in view.
3) Run this tool.
4) Either provide the measured base-frame XYZ manually, or use --teach-expected:
   after the image capture, jog the robot TCP onto the same point and let the tool
   read the expected base XYZ directly from RTDE.
5) The tool prints dx/dy/dz + total error.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from vision.calibration import apply_pick_correction, camera_to_base, pixel_to_camera_3d
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check vision-computed target base error against measured base XYZ")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--scanpose-tol-deg", type=float, default=3.0)
    parser.add_argument(
        "--teach-expected",
        action="store_true",
        help="After computing p_base, pause so operator can jog TCP to the same point and read expected XYZ from RTDE",
    )
    parser.add_argument(
        "--raw-transform-only",
        action="store_true",
        help="Disable checkerboard XY refine and measure only pixel+depth+T_CAM_TO_TCP bias",
    )
    parser.add_argument("--expected-x", type=float, help="Measured target X in base frame (m)")
    parser.add_argument("--expected-y", type=float, help="Measured target Y in base frame (m)")
    parser.add_argument("--expected-z", type=float, help="Measured target Z in base frame (m)")
    return parser.parse_args()


def read_expected_xyz(args: argparse.Namespace):
    if args.teach_expected:
        return None

    if args.expected_x is not None and args.expected_y is not None and args.expected_z is not None:
        return [args.expected_x, args.expected_y, args.expected_z]

    print("Nhap toa do base THUC TE cua diem dang do (don vi: m).")
    x = float(input("expected_x: ").strip())
    y = float(input("expected_y: ").strip())
    z = float(input("expected_z: ").strip())
    return [x, y, z]


def main() -> int:
    args = parse_args()

    print("=== CHECK TARGET BASE ERROR ===")
    print(f"Robot IP: {args.robot_ip}")
    confirm(
        "Robot da dung san o SCAN_POSE, workspace clear, va ban se do dung CUNG DIEM ma vision chon",
        args.yes,
    )
    expected_base = read_expected_xyz(args)
    if expected_base is not None:
        print(f"Expected base (m): {[round(v, 4) for v in expected_base]}")

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

    try:
        rtde.connect()
        camera.connect()

        current_joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_joints, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            print("Loi: robot chua dung dung SCAN_POSE.")
            return 1

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
            print("Khong phat hien phoi.")
            return 1

        frame_center_uv = (frame_w / 2.0, frame_h / 2.0)
        target = detector.select_best_target(detections, depth, frame_center_uv)
        if target is None:
            print("Co detections nhung khong co target hop le theo depth.")
            return 1

        target = detector.refine_pick_point(rgb, target, depth)
        u, v = target.pick_point
        depth_mm, depth_bbox = detector.resolve_pick_depth(depth, target)
        print(
            f"Target: label={target.label}, conf={target.confidence:.3f}, "
            f"bbox_center=({target.center[0]:.1f},{target.center[1]:.1f}), "
            f"pick=({u:.1f},{v:.1f}), depth={depth_mm:.1f} mm, "
            f"bbox={target.bbox}, pick_bbox={target.pick_bbox}, depth_bbox={depth_bbox}, source={target.pick_source}"
        )
        if depth_mm <= 0:
            print("Depth khong hop le.")
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
        p_base_raw = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
        xy_source = "depth_only"
        if config.TRAY_REF_ENABLED and not args.raw_transform_only:
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

        p_base, correction_meta = apply_pick_correction(p_base_raw)
        if args.teach_expected:
            print("\n=== TEACH EXPECTED BASE ===")
            print("Vision da chon diem sau tren phoi:")
            print(f"  pick_uv=({u:.1f}, {v:.1f})")
            print(f"  p_base_final(m)={[round(vv, 4) for vv in p_base]}")
            print("Hay jog robot de TCP cham DUNG diem nay tren phoi.")
            input("Nhan Enter khi TCP da cham dung diem de doc expected_base tu robot...")
            expected_pose = rtde.get_tcp_pose()
            expected_base = list(expected_pose[:3])
            print(f"Expected base tu TCP actual (m): {[round(v, 4) for v in expected_base]}")

        error_vec = np.array(p_base) - np.array(expected_base)
        error_mm = error_vec * 1000.0
        abs_error_mm = np.abs(error_mm)
        total_error_mm = float(np.linalg.norm(error_mm))

        print(f"p_cam(m): {[round(vv, 4) for vv in p_cam]}")
        print(f"p_base_raw(m): {[round(vv, 4) for vv in p_base_raw]}")
        print(f"pick_offset_base(m): {[round(vv, 4) for vv in correction_meta.get('final_offset', [0.0, 0.0, 0.0])]}")
        print(f"pick_correction_local(m): {[round(vv, 4) for vv in correction_meta.get('local_offset', [0.0, 0.0, 0.0])]} mode={correction_meta.get('mode', 'unknown')}")
        print(f"p_base_final(m): {[round(vv, 4) for vv in p_base]}")
        print(f"xy_source: {xy_source}")
        print(f"expected_base(m): {[round(vv, 4) for vv in expected_base]}")
        print(
            f"error(mm): dx={error_mm[0]:.1f}, dy={error_mm[1]:.1f}, dz={error_mm[2]:.1f}, "
            f"norm={total_error_mm:.1f}"
        )
        print(
            f"abs_error(mm): x={abs_error_mm[0]:.1f}, y={abs_error_mm[1]:.1f}, z={abs_error_mm[2]:.1f}"
        )

        print("\nNHAN XET:")
        print("- Neu dx,dy,dz gan nhu co dinh qua nhieu lan do: nghi calibration/TCP/tool mounting.")
        print("- Neu loi dao dong manh giua cac lan do cung mot diem: nghi depth/vision/anh sang.")
        print("- Neu loi chu yeu o Z: nghi depth truoc.")
        print("- Neu loi XY nhieu nhung Z on: nghi T_CAM_TO_TCP, intrinsics, hoac pick_uv.")
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
