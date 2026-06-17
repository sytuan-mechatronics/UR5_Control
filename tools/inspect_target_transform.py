"""Inspect vision -> base transform without sending any robot motion.

Use this to validate whether the computed `p_base` is geometrically plausible
before letting the robot move.
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
from vision.calibration import (
    axis_angle_to_rotation_matrix,
    camera_to_base,
    pixel_to_camera_3d,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
from vision.tray_reference import refine_base_xy_with_checkerboard


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy inspect.")
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
    parser = argparse.ArgumentParser(description="Inspect target transform without robot motion")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--scanpose-tol-deg", type=float, default=3.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== INSPECT TARGET TRANSFORM ===")
    print(f"Robot IP: {args.robot_ip}")

    confirm(
        "Robot da dung san o SCAN_POSE, workspace clear, se chi chup va tinh toa do, KHONG move",
        args.yes,
    )

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

        frame_center_uv = (depth.shape[1] / 2.0, depth.shape[0] / 2.0)
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
        p_base = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
        xy_source = "depth_only"
        if config.TRAY_REF_ENABLED:
            p_base, xy_source = refine_base_xy_with_checkerboard(
                rgb,
                u,
                v,
                p_base,
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
            p_base[0] + config.PICK_OFFSET_X,
            p_base[1] + config.PICK_OFFSET_Y,
            p_base[2] + config.PICK_OFFSET_Z,
        ]
        camera_origin_base = camera_to_base([0.0, 0.0, 0.0], tcp_pose_at_capture, config.T_CAM_TO_TCP)

        print(f"p_cam(m): {[round(vv, 4) for vv in p_cam]}")
        print(f"camera_origin_base(m): {[round(vv, 4) for vv in camera_origin_base]}")
        print(f"p_base(m): {[round(vv, 4) for vv in p_base]}")
        print(f"pick_offset_base(m): {[round(config.PICK_OFFSET_X, 4), round(config.PICK_OFFSET_Y, 4), round(config.PICK_OFFSET_Z, 4)]}")
        print(f"xy_source: {xy_source}")

        delta_base = np.array(p_base) - np.array(tcp_pose_at_capture[:3])
        print(f"delta target - tcp in base (m): {[round(vv, 4) for vv in delta_base.tolist()]}")
        print(f"distance tcp -> target (mm): {np.linalg.norm(delta_base) * 1000.0:.1f}")

        delta_cam_base = np.array(p_base) - np.array(camera_origin_base)
        print(f"delta target - camera_origin in base (m): {[round(vv, 4) for vv in delta_cam_base.tolist()]}")
        print(f"distance camera_origin -> target (mm): {np.linalg.norm(delta_cam_base) * 1000.0:.1f}")

        R_base_tcp = axis_angle_to_rotation_matrix(*tcp_pose_at_capture[3:6])
        tcp_x = R_base_tcp[:, 0]
        tcp_y = R_base_tcp[:, 1]
        tcp_z = R_base_tcp[:, 2]
        print(f"TCP axes in base: X={tcp_x.round(4).tolist()}, Y={tcp_y.round(4).tolist()}, Z={tcp_z.round(4).tolist()}")

        print("\nKET LUAN:")
        print("- Doi chieu p_base voi vi tri phoi ban do/estimate tren may that.")
        print("- Neu p_base lech hang tram mm so voi thuc te thi phai sua calibration/depth, khong sua motion.")
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
