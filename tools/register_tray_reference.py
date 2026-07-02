"""Capture or load one tray image and label fixed slot IDs.

Usage:
1. Put robot/camera at the normal scan pose.
2. Run this tool.
3. Click slot 1 -> slot N in order on the image.
4. Press Enter to save.

When capturing from camera directly, this tool requires the robot to be
steady and already at SCAN_POSE before it will accept the sample.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import math

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from vision.femto_camera import FemtoCamera
from vision.tray_slot_reference import append_tray_slot_reference_sample


WINDOW_NAME = "TRAY SLOT REFERENCE"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Register tray slot reference image")
    parser.add_argument("--image", default="", help="Use existing image instead of camera capture")
    parser.add_argument("--slot-count", type=int, default=5, help="Number of slots to mark")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP for SCAN_POSE verification")
    parser.add_argument("--sample-name", default="", help="Optional sample name, vd: sample_03")
    parser.add_argument(
        "--scanpose-tol-deg",
        type=float,
        default=config.TRAY_SLOT_SCANPOSE_TOL_DEG,
        help="Max joint error from SCAN_POSE",
    )
    parser.add_argument(
        "--save-image-dir",
        default="captures/tray_reference",
        help="Where to save captured/annotated tray reference images",
    )
    return parser.parse_args()


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def verify_scan_pose_and_capture_frame(robot_ip: str, scanpose_tol_deg: float) -> tuple:
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
            raise RuntimeError("Robot chua dung yen hoan toan, khong cho phep chup tray reference")

        joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(joints, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE khi chup mau: {err_deg:.2f} deg")
        if err_deg > scanpose_tol_deg:
            raise RuntimeError(
                "Robot khong o dung SCAN_POSE khi chup mau. "
                f"tol={scanpose_tol_deg:.2f} deg, actual={err_deg:.2f} deg"
            )

        rgb, depth, _ = camera.get_frames_with_timestamp()
        tcp_pose = rtde.get_tcp_pose()
        del depth
        return rgb, "camera", joints, tcp_pose
    finally:
        try:
            camera.disconnect()
        finally:
            rtde.disconnect()


def draw_points(image, points, slot_count):
    preview = image.copy()
    for idx, point in enumerate(points, start=1):
        u, v = point
        cv2.circle(preview, (int(u), int(v)), 8, (0, 255, 0), -1)
        cv2.putText(
            preview,
            f"{idx}",
            (int(u) + 12, int(v) - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    if len(points) < slot_count:
        info = f"Click slot {len(points)+1}/{slot_count}. Enter=save, Backspace=undo, Esc=cancel"
    else:
        info = f"Da du {slot_count}/{slot_count} slot. Enter=save, Backspace=undo, Esc=cancel"
    cv2.putText(preview, info, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(preview, info, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    return preview


def main() -> int:
    args = parse_args()
    if args.image:
        image_path = Path(args.image)
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"Khong mo duoc anh: {image_path}")
        source_label = str(image_path)
        capture_joints = None
        capture_tcp = None
    else:
        image, source_label, capture_joints, capture_tcp = verify_scan_pose_and_capture_frame(
            args.robot_ip,
            args.scanpose_tol_deg,
        )

    save_dir = ROOT / args.save_image_dir
    save_dir.mkdir(parents=True, exist_ok=True)
    raw_image_path = save_dir / f"tray_reference_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    cv2.imwrite(str(raw_image_path), image)
    print(f"Da luu anh goc: {raw_image_path}")

    points = []

    def on_mouse(event, x, y, flags, param):
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < args.slot_count:
            points.append((float(x), float(y)))

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, min(image.shape[1], 1400), min(image.shape[0], 900))
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    while True:
        preview = draw_points(image, points, args.slot_count)
        cv2.imshow(WINDOW_NAME, preview)
        key = cv2.waitKey(30) & 0xFF
        if key in (8, 127):
            if points:
                points.pop()
        elif key in (27, ord("q"), ord("Q")):
            print("Da huy tao tray reference.")
            return 0
        elif key in (13, 10):
            if len(points) != args.slot_count:
                print(f"Can {args.slot_count} diem, hien tai moi co {len(points)} diem.")
                continue
            break

    annotated_path = save_dir / f"tray_reference_annotated_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    annotated = draw_points(image, points, args.slot_count)
    cv2.imwrite(str(annotated_path), annotated)
    cv2.destroyAllWindows()

    slots = [{"name": f"slot_{idx}", "u": point[0], "v": point[1]} for idx, point in enumerate(points, start=1)]
    ref_path = append_tray_slot_reference_sample(
        slots=slots,
        image_path=str(raw_image_path),
        image_width=int(image.shape[1]),
        image_height=int(image.shape[0]),
        sample_name=args.sample_name,
        scan_pose_joints=capture_joints,
        scan_pose_tcp=capture_tcp,
    )

    print(f"Source: {source_label}")
    print(f"Da luu anh annotated: {annotated_path}")
    print(f"Da luu tray reference JSON: {ref_path}")
    if args.sample_name:
        print(f"Sample name: {args.sample_name}")
    print("Slots:")
    for slot in slots:
        print(f"  {slot['name']}: ({slot['u']:.1f}, {slot['v']:.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
