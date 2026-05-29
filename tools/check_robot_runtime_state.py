"""Quick runtime diagnostics for UR controller state before motion tests."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.dashboard_client import DashboardClient
from robot.rtde_client import RTDEClient


def fmt_deg(joints):
    return [round(math.degrees(j), 2) for j in joints]


def max_joint_error_deg(current, target):
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def main() -> int:
    print("=== UR Runtime State Check ===")
    print(f"Robot IP: {config.ROBOT_IP}")

    dash = DashboardClient(config.ROBOT_IP, timeout=config.SOCKET_TIMEOUT)
    rtde = RTDEClient(config.ROBOT_IP, frequency=config.RTDE_FREQUENCY)

    try:
        dash.connect()
        rtde.connect()

        robotmode = dash.get_robotmode()
        safetystatus = dash.get_safety_status()
        program_state, program_state_raw = dash.get_program_state()
        tcp = rtde.get_tcp_pose()
        joints = rtde.get_joint_positions()

        print(f"robotmode: {robotmode}")
        print(f"safetystatus: {safetystatus}")
        print(f"programState: {program_state} ({program_state_raw})")
        print(f"TCP actual: {[round(v, 4) for v in tcp]}")
        print(f"Joints actual (deg): {fmt_deg(joints)}")
        print(f"SCAN_POSE target (deg): {fmt_deg(config.SCAN_POSE_JOINTS)}")
        print(f"Lech SCAN_POSE: {max_joint_error_deg(joints, config.SCAN_POSE_JOINTS):.2f} deg")

        print("\nInterpretation:")
        print("- safetystatus phai la NORMAL")
        print("- robotmode nen o RUNNING hoặc IDLE sau khi brake release")
        print("- neu programState = STOPPED thi van co the nhan script 30002,")
        print("  nhung neu pendant dang o Local/Manual hoặc bi rang buoc safety thi movel co the bi bo qua")
        return 0

    except Exception as exc:
        print(f"Loi khi doc runtime state: {exc}")
        return 1
    finally:
        try:
            rtde.disconnect()
        except Exception:
            pass
        try:
            dash.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
