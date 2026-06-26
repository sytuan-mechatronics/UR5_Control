"""Register single-part reference samples for one specific slot.

Use this when only one part is visible and you want a robust slot identity
for offset tuning.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from vision.femto_camera import FemtoCamera
from vision.single_slot_reference import append_single_slot_reference_sample


WINDOW_NAME = "SINGLE SLOT REFERENCE"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Register single-slot reference image")
    parser.add_argument("--slot-name", required=True, help="slot_1..slot_5")
    parser.add_argument("--image", default="", help="Use existing image instead of camera capture")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP for SCAN_POSE verification")
    parser.add_argument("--sample-name", default="", help="Optional sample name")
    parser.add_argument("--scanpose-tol-deg", type=float, default=config.TRAY_SLOT_SCANPOSE_TOL_DEG)
    parser.add_argument("--save-image-dir", default="captures/single_slot_reference")
    return parser.parse_args()


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def verify_scan_pose_and_capture_frame(robot_ip: str, scanpose_tol_deg: float):
    rtde = RTDEClient(robot_ip, frequency=config.RTDE_FREQUENCY)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
    rtde.connect()
    camera.connect()
    try:
        ok = rtde.wait_steady(
            timeout_s=config.RTDE_WAIT_TIMEOUT,
            threshold=config.RTDE_STEADY_THRESHOLD,
            motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
            motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
        )
        if not ok:
            raise RuntimeError("Robot chua dung yen hoan toan, khong cho phep chup single-slot reference")
        joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(joints, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE khi chup mau single-slot: {err_deg:.2f} deg")
        if err_deg > scanpose_tol_deg:
            raise RuntimeError(
                "Robot khong o dung SCAN_POSE khi chup mau single-slot. "
                f"tol={scanpose_tol_deg:.2f} deg, actual={err_deg:.2f} deg"
            )
        rgb, depth, _ = camera.get_frames_with_timestamp()
        del depth
        tcp_pose = rtde.get_tcp_pose()
        return rgb, joints, tcp_pose
    finally:
        try:
            camera.disconnect()
        finally:
            rtde.disconnect()


def main() -> int:
    args = parse_args()
    if args.image:
        image_path = Path(args.image)
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"Khong mo duoc anh: {image_path}")
        capture_joints = None
        capture_tcp = None
    else:
        image, capture_joints, capture_tcp = verify_scan_pose_and_capture_frame(
            args.robot_ip,
            args.scanpose_tol_deg,
        )

    save_dir = ROOT / args.save_image_dir
    save_dir.mkdir(parents=True, exist_ok=True)
    raw_image_path = save_dir / f"{args.slot_name}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    cv2.imwrite(str(raw_image_path), image)
    print(f"Da luu anh goc: {raw_image_path}")

    picked = {"point": None}

    def on_mouse(event, x, y, flags, param):
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            picked["point"] = (float(x), float(y))

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, min(image.shape[1], 1400), min(image.shape[0], 900))
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    while True:
        preview = image.copy()
        if picked["point"] is not None:
            u, v = picked["point"]
            cv2.circle(preview, (int(u), int(v)), 8, (0, 255, 0), -1)
            cv2.putText(preview, args.slot_name, (int(u) + 10, int(v) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            info = f"{args.slot_name} selected at ({u:.1f},{v:.1f}). Enter=save, click again=change, Esc=cancel"
        else:
            info = f"Click the visible part center for {args.slot_name}. Enter=save, Esc=cancel"
        cv2.putText(preview, info, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(preview, info, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW_NAME, preview)
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            print("Da huy tao single-slot reference.")
            return 0
        if key in (13, 10):
            if picked["point"] is None:
                print("Ban chua click diem phoi.")
                continue
            break

    cv2.destroyAllWindows()
    u, v = picked["point"]
    ref_path = append_single_slot_reference_sample(
        slot_name=args.slot_name,
        u=u,
        v=v,
        image_path=str(raw_image_path),
        sample_name=args.sample_name,
        scan_pose_joints=capture_joints,
        scan_pose_tcp=capture_tcp,
    )
    print(f"Da luu single-slot reference: {ref_path}")
    print(f"slot={args.slot_name} uv=({u:.1f}, {v:.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
