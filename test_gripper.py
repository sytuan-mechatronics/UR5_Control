"""
Quick open/close tester for pneumatic gripper.
Usage:
  python test_gripper.py close
  python test_gripper.py open
  python test_gripper.py toggle --cycles 5 --hold-s 1.0
"""

import argparse
import sys
import time

import config
from core.pneumatic_gripper import PneumaticGripper, GripperError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quick pneumatic gripper test")
    parser.add_argument(
        "action",
        nargs="?",
        default="toggle",
        choices=["open", "close", "toggle"],
        help="Action to run (default: toggle)",
    )
    parser.add_argument("--port", default=config.GRIPPER_PORT, help="Serial port (default from config)")
    parser.add_argument("--baud", type=int, default=config.GRIPPER_BAUD, help="Baudrate")
    parser.add_argument("--cycles", type=int, default=3, help="Toggle cycles")
    parser.add_argument("--hold-s", type=float, default=1.0, help="Hold time between open/close")
    return parser


def run() -> int:
    args = build_parser().parse_args()

    g = PneumaticGripper(
        port=args.port,
        baud=args.baud,
        cmd_timeout_s=config.GRIPPER_CMD_TIMEOUT_S,
        grip_settle_s=config.GRIPPER_SETTLE_S,
        release_settle_s=config.GRIPPER_RELEASE_SETTLE_S,
        heartbeat_interval_s=config.GRIPPER_HEARTBEAT_S,
    )

    try:
        g.connect()

        if args.action == "close":
            result = g.close()
            print("close ->", result)
            return 0 if result.get("ok") else 2

        if args.action == "open":
            result = g.open()
            print("open  ->", result)
            return 0 if result.get("ok") else 2

        for idx in range(1, args.cycles + 1):
            close_result = g.close()
            print(f"toggle[{idx}] close ->", close_result)
            if not close_result.get("ok"):
                return 2
            time.sleep(args.hold_s)

            open_result = g.open()
            print(f"toggle[{idx}] open  ->", open_result)
            if not open_result.get("ok"):
                return 2
            time.sleep(args.hold_s)

        return 0

    except GripperError as err:
        print("gripper error:", err)
        return 2
    except Exception as err:
        print("unexpected error:", err)
        return 3
    finally:
        try:
            g.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(run())
