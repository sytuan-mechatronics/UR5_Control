"""
Hardware validation tool for Arduino-based pneumatic gripper.

This script helps verify the full chain:
PC2 -> /dev/gripper -> Arduino -> Relay -> Solenoid Valve -> Pneumatic Gripper.
"""

import argparse
import logging
import sys
import time
from typing import Callable, Dict, Any

import config
from core.pneumatic_gripper import PneumaticGripper, GripperError


LOGGER = logging.getLogger("test_pneumatic_gripper")


def _setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%H:%M:%S",
    )


def _print_result(label: str, result: Dict[str, Any]) -> None:
    LOGGER.info("%s -> ok=%s response=%s state=%s", label, result.get("ok"), result.get("response"), result.get("state"))


def action_status(gripper: PneumaticGripper, args: argparse.Namespace) -> int:
    state = gripper.get_state()
    LOGGER.info("status -> ok=%s gripping=%s raw=%s", state.get("ok"), state.get("gripping"), state.get("raw"))
    return 0


def action_open(gripper: PneumaticGripper, args: argparse.Namespace) -> int:
    result = gripper.open()
    _print_result("open", result)
    return 0 if result.get("ok") else 2


def action_close(gripper: PneumaticGripper, args: argparse.Namespace) -> int:
    result = gripper.close()
    _print_result("close", result)
    return 0 if result.get("ok") else 2


def action_pulse(gripper: PneumaticGripper, args: argparse.Namespace) -> int:
    LOGGER.info("pulse start: close %.3fs then open %.3fs", args.on_s, args.off_s)

    close_result = gripper.close()
    _print_result("pulse.close", close_result)
    if not close_result.get("ok"):
        return 2

    time.sleep(args.on_s)

    open_result = gripper.open()
    _print_result("pulse.open", open_result)
    if not open_result.get("ok"):
        return 2

    time.sleep(args.off_s)
    LOGGER.info("pulse done")
    return 0


def action_cycle(gripper: PneumaticGripper, args: argparse.Namespace) -> int:
    LOGGER.info(
        "cycle start: cycles=%d close_hold=%.3fs open_hold=%.3fs",
        args.cycles,
        args.on_s,
        args.off_s,
    )

    if args.pre_open:
        open_result = gripper.open()
        _print_result("cycle.pre_open", open_result)
        if not open_result.get("ok"):
            return 2
        time.sleep(0.2)

    for idx in range(1, args.cycles + 1):
        LOGGER.info("cycle %d/%d", idx, args.cycles)

        close_result = gripper.close()
        _print_result("cycle.close", close_result)
        if not close_result.get("ok"):
            return 2
        time.sleep(args.on_s)

        open_result = gripper.open()
        _print_result("cycle.open", open_result)
        if not open_result.get("ok"):
            return 2
        time.sleep(args.off_s)

    LOGGER.info("cycle done")
    return 0


def action_hold(gripper: PneumaticGripper, args: argparse.Namespace) -> int:
    LOGGER.info("hold start: close then hold for %.3fs", args.hold_s)

    close_result = gripper.close()
    _print_result("hold.close", close_result)
    if not close_result.get("ok"):
        return 2

    start = time.time()
    last_probe = 0.0
    while True:
        elapsed = time.time() - start
        if elapsed >= args.hold_s:
            break
        if elapsed - last_probe >= args.probe_interval_s:
            last_probe = elapsed
            try:
                state = gripper.get_state()
                LOGGER.info("hold probe t=%.1fs -> gripping=%s raw=%s", elapsed, state.get("gripping"), state.get("raw"))
            except GripperError as err:
                LOGGER.error("hold probe failed at t=%.1fs: %s", elapsed, err)
                if args.fail_fast:
                    return 2
        time.sleep(0.05)

    open_result = gripper.open()
    _print_result("hold.open", open_result)
    if not open_result.get("ok"):
        return 2

    LOGGER.info("hold done")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate pneumatic gripper hardware chain via Arduino serial gateway.",
    )
    parser.add_argument(
        "--action",
        choices=["status", "open", "close", "pulse", "cycle", "hold"],
        default="status",
        help="Test action to run.",
    )
    parser.add_argument("--port", default=config.GRIPPER_PORT, help="Serial device path (default from config).")
    parser.add_argument("--baud", type=int, default=config.GRIPPER_BAUD, help="Serial baudrate.")
    parser.add_argument(
        "--cmd-timeout-s",
        type=float,
        default=config.GRIPPER_CMD_TIMEOUT_S,
        help="Command response timeout in seconds.",
    )
    parser.add_argument(
        "--grip-settle-s",
        type=float,
        default=config.GRIPPER_SETTLE_S,
        help="Mechanical settle delay after close.",
    )
    parser.add_argument(
        "--release-settle-s",
        type=float,
        default=config.GRIPPER_RELEASE_SETTLE_S,
        help="Mechanical settle delay after open/release.",
    )
    parser.add_argument(
        "--heartbeat-s",
        type=float,
        default=config.GRIPPER_HEARTBEAT_S,
        help="Heartbeat period in seconds.",
    )
    parser.add_argument("--on-s", type=float, default=1.0, help="Hold time in close state for pulse/cycle.")
    parser.add_argument("--off-s", type=float, default=1.0, help="Hold time in open state for pulse/cycle.")
    parser.add_argument("--cycles", type=int, default=10, help="Number of cycles for action=cycle.")
    parser.add_argument("--hold-s", type=float, default=20.0, help="Hold time for action=hold.")
    parser.add_argument(
        "--probe-interval-s",
        type=float,
        default=2.0,
        help="State probe interval for action=hold.",
    )
    parser.add_argument(
        "--pre-open",
        action="store_true",
        help="Open once before cycle action starts.",
    )
    parser.add_argument(
        "--fail-fast",
        action="store_true",
        help="Stop immediately when a probe/read error occurs.",
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    _setup_logging(args.verbose)

    LOGGER.info(
        "config: action=%s port=%s baud=%d timeout=%.2fs settle=%.2fs release_settle=%.2fs heartbeat=%.2fs",
        args.action,
        args.port,
        args.baud,
        args.cmd_timeout_s,
        args.grip_settle_s,
        args.release_settle_s,
        args.heartbeat_s,
    )

    actions = {
        "status": action_status,
        "open": action_open,
        "close": action_close,
        "pulse": action_pulse,
        "cycle": action_cycle,
        "hold": action_hold,
    }  # type: Dict[str, Callable[[PneumaticGripper, argparse.Namespace], int]]

    gripper = PneumaticGripper(
        port=args.port,
        baud=args.baud,
        cmd_timeout_s=args.cmd_timeout_s,
        grip_settle_s=args.grip_settle_s,
        release_settle_s=args.release_settle_s,
        heartbeat_interval_s=args.heartbeat_s,
    )

    try:
        gripper.connect()
        LOGGER.info("connected: %s", gripper.is_connected)
        return actions[args.action](gripper, args)
    except KeyboardInterrupt:
        LOGGER.warning("interrupted by user")
        return 130
    except GripperError as err:
        LOGGER.error("gripper error: %s", err)
        return 2
    except Exception as err:
        LOGGER.exception("unexpected error: %s", err)
        return 3
    finally:
        try:
            gripper.disconnect()
        except Exception as err:
            LOGGER.warning("disconnect warning: %s", err)


if __name__ == "__main__":
    sys.exit(main())
