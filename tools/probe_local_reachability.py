"""Probe small Cartesian offsets around SCAN_POSE to map local reachability.

Runs a sequence of tiny `movej(get_inverse_kin(...))` tests around the current
TCP pose and reports which axis directions are accepted by the controller.
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe tiny local reachability around SCAN_POSE")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--step-mm", type=float, default=5.0, help="Offset magnitude per probe")
    parser.add_argument("--reach-tol-mm", type=float, default=5.0, help="Position tolerance to count as pass")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    step_m = args.step_mm / 1000.0
    probes = [
        ("+X", [step_m, 0.0, 0.0]),
        ("-X", [-step_m, 0.0, 0.0]),
        ("+Y", [0.0, step_m, 0.0]),
        ("-Y", [0.0, -step_m, 0.0]),
        ("+Z", [0.0, 0.0, step_m]),
        ("-Z", [0.0, 0.0, -step_m]),
    ]

    print("=== LOCAL REACHABILITY PROBE ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Step: {args.step_mm:.1f} mm")

    confirm("Robot dang o SCAN_POSE, workspace clear, san sang probe local reachability", args.yes)

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(args.robot_ip, port=config.URSCRIPT_PORT, timeout=config.URSCRIPT_TIMEOUT)

    try:
        rtde.connect()
        urscript.connect()
        urscript.set_tcp(config.TCP_OFFSET)
        print(f"TCP set: offset={config.TCP_OFFSET}")
        urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)

        current_joints = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_joints, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE: {err_deg:.2f} deg")
        if err_deg > 3.0:
            print("Loi: robot chua dung dung SCAN_POSE.")
            return 1

        tcp0, _ = rtde.get_tcp_pose_with_timestamp()
        print(f"Start TCP: {[round(v, 4) for v in tcp0]}")

        results = []
        for name, (dx, dy, dz) in probes:
            target_pose = [
                tcp0[0] + dx,
                tcp0[1] + dy,
                tcp0[2] + dz,
                tcp0[3],
                tcp0[4],
                tcp0[5],
            ]
            print(f"\nProbe {name}: target={[round(v, 4) for v in target_pose]}")
            try:
                urscript.move_joint_to_pose_ik(target_pose, accel=0.2, vel=0.1)
                wait_steady(rtde, f"probe_{name}")
                actual_pose, _ = rtde.get_tcp_pose_with_timestamp()
                err_mm = pose_error_mm(actual_pose, target_pose)
                ok = err_mm <= args.reach_tol_mm
                print(f"Actual: {[round(v, 4) for v in actual_pose]}")
                print(f"err={err_mm:.1f} mm -> {'PASS' if ok else 'FAIL'}")
                results.append((name, ok, err_mm))
            except Exception as exc:
                print(f"Probe {name} exception: {exc}")
                results.append((name, False, None))

            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL * 0.3, vel=config.JOINT_VEL * 0.3)
            wait_steady(rtde, "return_scanpose")

        print("\n=== SUMMARY ===")
        for name, ok, err_mm in results:
            if err_mm is None:
                print(f"{name}: FAIL (exception)")
            else:
                print(f"{name}: {'PASS' if ok else 'FAIL'} err={err_mm:.1f} mm")
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
