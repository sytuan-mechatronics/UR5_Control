"""Capture two named robot poses and print env-ready config lines.

Default use case:
- Pose 1: a safer pre-pick joint approach
- Pose 2: an optional second intermediate point closer to the tray
"""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient


def fmt_joints_deg(joints):
    return [round(math.degrees(v), 2) for v in joints]


def fmt_values(values, digits=6):
    return ",".join(f"{v:.{digits}f}" for v in values)


def capture_pose(rtde: RTDEClient, name: str) -> dict:
    input(f"\n>>> Dua robot den '{name}', sau do nhan Enter de luu... ")
    tcp = rtde.get_tcp_pose()
    joints = rtde.get_joint_positions()
    pose = {
        "name": name,
        "joints_rad": [round(v, 6) for v in joints],
        "joints_deg": fmt_joints_deg(joints),
        "tcp_m_rad": [round(v, 6) for v in tcp],
    }
    print(f"\nDa luu '{name}':")
    print(f"  joints_rad: {pose['joints_rad']}")
    print(f"  joints_deg: {pose['joints_deg']}")
    print(f"  tcp_m_rad : {pose['tcp_m_rad']}")
    return pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture two new robot poses")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument(
        "--pose-1-name",
        default="PRE_PICK_APPROACH_JOINTS",
        help="Name for first pose",
    )
    parser.add_argument(
        "--pose-2-name",
        default="PRE_PICK_ALIGN_JOINTS",
        help="Name for second pose",
    )
    parser.add_argument(
        "--output",
        default="captured_two_poses.json",
        help="Output JSON path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("=== CAPTURE TWO POSES ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Pose 1: {args.pose_1_name}")
    print(f"Pose 2: {args.pose_2_name}")
    print("\nHuong dan:")
    print("1. Dua robot toi pose dau tien bang teach pendant/Freedrive.")
    print("2. Nhan Enter de luu.")
    print("3. Lap lai voi pose thu hai.")

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    try:
        rtde.connect()
        pose_1 = capture_pose(rtde, args.pose_1_name)
        pose_2 = capture_pose(rtde, args.pose_2_name)

        output = {
            "robot_ip": args.robot_ip,
            "poses": [pose_1, pose_2],
        }
        output_path = Path(args.output)
        output_path.write_text(json.dumps(output, ensure_ascii=True, indent=2), encoding="utf-8")

        print(f"\nDa luu JSON: {output_path}")
        print("\n=== ENV SNIPPETS ===")
        print(f"{args.pose_1_name}={fmt_values(pose_1['joints_rad'])}")
        print(f"{args.pose_2_name}={fmt_values(pose_2['joints_rad'])}")
        print("\n=== TCP REFERENCE ===")
        print(f"{args.pose_1_name.replace('JOINTS', 'TCP')}={fmt_values(pose_1['tcp_m_rad'])}")
        print(f"{args.pose_2_name.replace('JOINTS', 'TCP')}={fmt_values(pose_2['tcp_m_rad'])}")
        return 0
    except Exception as exc:
        print(f"Loi khi capture poses: {exc}")
        return 1
    finally:
        try:
            rtde.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
