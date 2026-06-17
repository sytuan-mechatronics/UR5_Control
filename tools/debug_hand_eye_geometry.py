"""Debug geometric interpretation of T_cam_to_tcp.

Use this to inspect where the camera is relative to TCP/base and why
the runtime derives a particular "min safe depth".
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1]))

import config
from vision.calibration import (
    camera_to_base,
    min_safe_camera_depth_m,
    pixel_to_camera_3d,
    tcp_pose_to_transform,
)


def fmt_vec(vec: np.ndarray, scale: float = 1.0) -> str:
    values = [round(float(v) * scale, 6) for v in vec]
    return str(values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect hand-eye geometry and implied camera/TCP relationship",
    )
    parser.add_argument(
        "--tcp",
        default=",".join(str(v) for v in config.SCAN_POSE_TCP),
        help="TCP pose x,y,z,rx,ry,rz in m/rad (default: SCAN_POSE_TCP)",
    )
    parser.add_argument("--u", type=float, default=None, help="Optional pixel u for sample projection")
    parser.add_argument("--v", type=float, default=None, help="Optional pixel v for sample projection")
    parser.add_argument("--depth-mm", type=float, default=None, help="Optional sample depth in mm")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    tcp_pose = [float(v.strip()) for v in args.tcp.split(",")]
    if len(tcp_pose) != 6:
        raise SystemExit("TCP must have 6 comma-separated values")

    T_cam_to_tcp = np.array(config.T_CAM_TO_TCP, dtype=np.float64)
    T_tcp_to_cam = np.linalg.inv(T_cam_to_tcp)
    T_base_tcp = tcp_pose_to_transform(tcp_pose)
    T_base_cam = T_base_tcp @ T_cam_to_tcp

    cam_origin_in_tcp = T_cam_to_tcp[:3, 3]
    tcp_origin_in_cam = (T_tcp_to_cam @ np.array([0.0, 0.0, 0.0, 1.0]))[:3]
    cam_origin_in_base = (T_base_cam @ np.array([0.0, 0.0, 0.0, 1.0]))[:3]

    cam_x_in_base = T_base_cam[:3, :3] @ np.array([1.0, 0.0, 0.0])
    cam_y_in_base = T_base_cam[:3, :3] @ np.array([0.0, 1.0, 0.0])
    cam_z_in_base = T_base_cam[:3, :3] @ np.array([0.0, 0.0, 1.0])

    print("=== Hand-Eye Geometry Debug ===")
    print(f"TCP pose used (base frame): {fmt_vec(np.array(tcp_pose))}")
    print(f"T_cam_to_tcp translation (camera origin in TCP): {fmt_vec(cam_origin_in_tcp)} m")
    print(f"T_cam_to_tcp translation (camera origin in TCP): {fmt_vec(cam_origin_in_tcp, 1000.0)} mm")
    print(f"TCP origin in camera frame: {fmt_vec(tcp_origin_in_cam)} m")
    print(f"TCP origin in camera frame: {fmt_vec(tcp_origin_in_cam, 1000.0)} mm")
    print(f"Camera origin in base frame: {fmt_vec(cam_origin_in_base)} m")
    print(f"Camera optical axis +Z in base frame: {fmt_vec(cam_z_in_base)}")
    print(f"Camera +X in base frame: {fmt_vec(cam_x_in_base)}")
    print(f"Camera +Y in base frame: {fmt_vec(cam_y_in_base)}")
    print(f"Min safe depth from current T_cam_to_tcp: {min_safe_camera_depth_m(T_cam_to_tcp, config.PICK_MIN_DESCENT_M) * 1000.0:.1f} mm")

    if args.u is not None and args.v is not None and args.depth_mm is not None:
        print("\n=== Sample Projection ===")
        p_cam = np.array(
            pixel_to_camera_3d(
                args.u,
                args.v,
                args.depth_mm,
                config.CAM_FX,
                config.CAM_FY,
                config.CAM_CX,
                config.CAM_CY,
            ),
            dtype=np.float64,
        )
        p_base = np.array(camera_to_base(p_cam.tolist(), tcp_pose, T_cam_to_tcp), dtype=np.float64)
        print(f"Input pixel/depth: u={args.u:.1f}, v={args.v:.1f}, depth={args.depth_mm:.1f} mm")
        print(f"p_cam:  {fmt_vec(p_cam)} m")
        print(f"p_base: {fmt_vec(p_base)} m")
        print(f"Delta vs TCP in base: {fmt_vec(p_base - np.array(tcp_pose[:3], dtype=np.float64))} m")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
