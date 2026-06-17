"""Minimal test: from SCAN_POSE, try a small Cartesian movel offset and return.

Purpose:
- Verify whether URScript `movel()` is actually executed by the controller
- Separate controller/TCP/reachability issues from vision/calibration issues
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
from robot.urscript_client import URScriptClient


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


def pose_error_mm(actual_pose, target_pose) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(target_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


def print_pose(label: str, pose) -> None:
    print(f"{label}: {[round(v, 4) for v in pose]}")


def print_pose_delta(label: str, actual_pose, target_pose) -> float:
    delta = [actual_pose[i] - target_pose[i] for i in range(6)]
    err_mm = pose_error_mm(actual_pose, target_pose)
    print_pose(f"{label} actual TCP", actual_pose)
    print(f"{label} delta: {[round(v, 4) for v in delta]} (pos_err={err_mm:.1f} mm)")
    return err_mm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test if robot accepts movel to a small Cartesian offset from SCAN_POSE"
    )
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP")
    parser.add_argument("--yes", action="store_true", help="Skip interactive confirmation")
    parser.add_argument(
        "--mode",
        choices=["movel", "movej-ik"],
        default="movel",
        help="Motion method to test",
    )
    parser.add_argument(
        "--orientation",
        choices=["current-tcp", "tool-down"],
        default="current-tcp",
        help="Orientation to use for target pose",
    )
    parser.add_argument("--dx", type=float, default=-0.02, help="Offset X in meters")
    parser.add_argument("--dy", type=float, default=0.0, help="Offset Y in meters")
    parser.add_argument("--dz", type=float, default=-0.02, help="Offset Z in meters")
    parser.add_argument("--vel", type=float, default=0.03, help="movel velocity in m/s")
    parser.add_argument("--accel", type=float, default=0.10, help="movel acceleration in m/s^2")
    parser.add_argument(
        "--reach-tol-mm",
        type=float,
        default=10.0,
        help="Max allowed position error after movel",
    )
    parser.add_argument(
        "--scanpose-tol-deg",
        type=float,
        default=3.0,
        help="Max allowed joint error from SCAN_POSE before starting",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== MOVEL OFFSET TEST ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Mode: {args.mode}")
    print(f"Orientation: {args.orientation}")
    print(f"Offset target: dx={args.dx:.3f} m, dy={args.dy:.3f} m, dz={args.dz:.3f} m")

    confirm(
        "Robot da dung san o SCAN_POSE, workspace clear, san sang test movel offset nho",
        args.yes,
    )

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(
        args.robot_ip,
        port=config.URSCRIPT_PORT,
        timeout=config.URSCRIPT_TIMEOUT,
    )

    try:
        rtde.connect()
        urscript.connect()
        urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
        print(f"Payload set: mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")

        current_joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_joints, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE: {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            print("Loi: robot chua dung dung SCAN_POSE. Hay dua robot ve SCAN_POSE roi chay lai.")
            return 1

        tcp0, _ = rtde.get_tcp_pose_with_timestamp()
        print_pose("Start TCP", tcp0)

        if args.orientation == "current-tcp":
            target_rx, target_ry, target_rz = tcp0[3], tcp0[4], tcp0[5]
        else:
            target_rx, target_ry, target_rz = (
                config.TOOL_DOWN_RX,
                config.TOOL_DOWN_RY,
                config.TOOL_DOWN_RZ,
            )

        target_pose = [
            tcp0[0] + args.dx,
            tcp0[1] + args.dy,
            tcp0[2] + args.dz,
            target_rx,
            target_ry,
            target_rz,
        ]
        print_pose("Target TCP", target_pose)

        if args.mode == "movel":
            urscript.move_linear(target_pose, accel=args.accel, vel=args.vel)
        else:
            urscript.move_joint_to_pose_ik(target_pose, accel=max(args.accel, 0.2), vel=max(args.vel, 0.1))
        wait_steady(rtde, f"offset_{args.mode}")
        tcp1, _ = rtde.get_tcp_pose_with_timestamp()
        err_mm = print_pose_delta(f"Offset move ({args.mode})", tcp1, target_pose)

        if err_mm > args.reach_tol_mm:
            print(
                f"\nKET LUAN: robot KHONG den duoc target offset nho bang mode {args.mode}.\n"
                "Neu movel fail nhung movej-ik pass: van de nam o linear path / movel.\n"
                "Neu ca hai cung fail: van de nam o target pose / TCP active / reachability / controller."
            )
            return 2

        print(f"\nOffset test ({args.mode}) dat yeu cau. Thu quay lai SCAN_POSE...")
        urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL * 0.3, vel=config.JOINT_VEL * 0.3)
        wait_steady(rtde, "return_scanpose")
        tcp2, _ = rtde.get_tcp_pose_with_timestamp()
        print_pose("Return TCP", tcp2)
        print("\nKET LUAN: robot chap nhan movel offset nho.")
        return 0

    except Exception as exc:
        print(f"Loi khi chay test: {exc}")
        return 1
    finally:
        try:
            urscript.disconnect()
        except Exception:
            pass
        try:
            rtde.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
