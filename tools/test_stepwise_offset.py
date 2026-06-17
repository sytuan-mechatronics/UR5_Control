"""Stepwise Cartesian reachability test from SCAN_POSE.

Move from current SCAN_POSE toward a target offset in small `movej(get_inverse_kin(...))`
steps. This helps identify at which distance the controller starts rejecting poses.
"""

import argparse
import math
import sys
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
    return rtde.wait_steady(
        timeout_s=config.RTDE_WAIT_TIMEOUT,
        threshold=config.RTDE_STEADY_THRESHOLD,
        motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
    )


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def pose_error_mm(actual_pose, target_pose) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(target_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


def motion_delta_mm(actual_pose, start_pose) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(start_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stepwise reachability test from SCAN_POSE")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dx", type=float, required=True, help="Final X offset in meters")
    parser.add_argument("--dy", type=float, required=True, help="Final Y offset in meters")
    parser.add_argument("--dz", type=float, required=True, help="Final Z offset in meters")
    parser.add_argument(
        "--step-mm",
        type=float,
        default=20.0,
        help="Position increment per step in millimeters",
    )
    parser.add_argument(
        "--reach-tol-mm",
        type=float,
        default=10.0,
        help="Allowed position error per step",
    )
    parser.add_argument(
        "--min-motion-ratio",
        type=float,
        default=0.7,
        help="Required fraction of commanded step that must be physically achieved",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    final_offset = np.array([args.dx, args.dy, args.dz], dtype=np.float64)
    step_m = max(args.step_mm / 1000.0, 1e-6)
    distance = float(np.linalg.norm(final_offset))
    steps = max(int(math.ceil(distance / step_m)), 1)

    print("=== STEPWISE OFFSET TEST ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Final offset: dx={args.dx:.3f} m, dy={args.dy:.3f} m, dz={args.dz:.3f} m")
    print(f"Distance: {distance * 1000.0:.1f} mm, steps: {steps}, step_mm: {args.step_mm:.1f}")

    confirm("Robot dang o SCAN_POSE, workspace clear, san sang test theo nhieu buoc nho", args.yes)

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(args.robot_ip, port=config.URSCRIPT_PORT, timeout=config.URSCRIPT_TIMEOUT)

    try:
        rtde.connect()
        urscript.connect()
        urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
        print(f"Payload set: mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")

        current_joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_joints, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE: {err_deg:.2f} deg")
        if err_deg > 3.0:
            print("Loi: robot chua dung dung SCAN_POSE.")
            return 1

        tcp0, _ = rtde.get_tcp_pose_with_timestamp()
        print(f"Start TCP: {[round(v, 4) for v in tcp0]}")

        orientation = tcp0[3:6]
        last_ok_step = 0
        last_ok_pose = tcp0

        for i in range(1, steps + 1):
            alpha = i / steps
            offset = final_offset * alpha
            target_pose = [
                tcp0[0] + float(offset[0]),
                tcp0[1] + float(offset[1]),
                tcp0[2] + float(offset[2]),
                orientation[0],
                orientation[1],
                orientation[2],
            ]
            print(f"\nStep {i}/{steps}: target={[round(v, 4) for v in target_pose]}")

            try:
                urscript.move_joint_to_pose_ik(target_pose, accel=0.2, vel=0.1)
                wait_steady(rtde, f"step_{i}")
                actual_pose, _ = rtde.get_tcp_pose_with_timestamp()
                err_mm = pose_error_mm(actual_pose, target_pose)
                moved_mm = motion_delta_mm(actual_pose, tcp0)
                commanded_mm = float(np.linalg.norm(offset) * 1000.0)
                print(f"Actual: {[round(v, 4) for v in actual_pose]}")
                print(f"err={err_mm:.1f} mm, moved={moved_mm:.1f} mm, commanded={commanded_mm:.1f} mm")
                motion_ok = moved_mm >= commanded_mm * args.min_motion_ratio
                if err_mm > args.reach_tol_mm or not motion_ok:
                    if err_mm > args.reach_tol_mm:
                        print(f"FAIL at step {i}: err {err_mm:.1f} mm > {args.reach_tol_mm:.1f} mm")
                    else:
                        print(
                            f"FAIL at step {i}: moved {moved_mm:.1f} mm < "
                            f"{args.min_motion_ratio * 100.0:.0f}% of commanded {commanded_mm:.1f} mm"
                        )
                    break
                last_ok_step = i
                last_ok_pose = actual_pose
            except Exception as exc:
                print(f"FAIL at step {i}: {exc}")
                break

        print("\n=== SUMMARY ===")
        print(f"last_ok_step: {last_ok_step}/{steps}")
        print(f"last_ok_distance_mm: {distance * 1000.0 * (last_ok_step / steps):.1f}")
        print(f"last_ok_pose: {[round(v, 4) for v in last_ok_pose]}")

        urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL * 0.3, vel=config.JOINT_VEL * 0.3)
        wait_steady(rtde, "return_scanpose")
        return 0

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
