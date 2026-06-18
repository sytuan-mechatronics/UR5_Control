"""Check whether commanded TCP Z+ / Z- matches actual robot motion.

Safe workflow:
1) Park robot at a safe pose with free space above and below a small distance.
2) Run this tool.
3) Tool applies current TCP offset, moves only a tiny amount along base Z,
   reads actual TCP from RTDE, then returns to the start pose.

This isolates TCP-direction issues from vision/calibration issues.
"""

import argparse
import sys
from pathlib import Path

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


def fmt_pose(pose) -> str:
    return "[" + ", ".join(f"{float(v):.4f}" for v in pose) + "]"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test actual TCP Z direction with small +Z/-Z moves",
    )
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dz-mm", type=float, default=10.0, help="Absolute Z step in mm")
    parser.add_argument("--vel", type=float, default=0.02, help="Linear velocity m/s")
    parser.add_argument("--accel", type=float, default=0.08, help="Linear acceleration m/s^2")
    parser.add_argument(
        "--orientation",
        choices=["current", "tool-down"],
        default="current",
        help="Keep current TCP orientation or force tool-down",
    )
    return parser.parse_args()


def run_step(rtde: RTDEClient, urscript: URScriptClient, start_pose, dz_m: float, args) -> None:
    if args.orientation == "current":
        rx, ry, rz = start_pose[3], start_pose[4], start_pose[5]
    else:
        rx, ry, rz = config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ

    target_pose = [start_pose[0], start_pose[1], start_pose[2] + dz_m, rx, ry, rz]

    print(f"\nStart TCP:  {fmt_pose(start_pose)}")
    print(f"Target TCP: {fmt_pose(target_pose)}")
    print(f"Commanded dz = {dz_m * 1000.0:+.1f} mm")

    urscript.move_linear(target_pose, accel=args.accel, vel=args.vel)
    wait_steady(rtde, f"move_z_{dz_m:+.3f}")
    actual_pose, _ = rtde.get_tcp_pose_with_timestamp()
    actual_dz_mm = (actual_pose[2] - start_pose[2]) * 1000.0
    z_error_mm = (actual_pose[2] - target_pose[2]) * 1000.0

    print(f"Actual TCP: {fmt_pose(actual_pose)}")
    print(f"Actual dz  = {actual_dz_mm:+.1f} mm")
    print(f"Z error    = {z_error_mm:+.1f} mm")

    if dz_m < 0 and actual_dz_mm < 0:
        print("KET LUAN buoc nay: lenh Z am lam TCP Z giam. Chieu TCP Z DUNG.")
    elif dz_m > 0 and actual_dz_mm > 0:
        print("KET LUAN buoc nay: lenh Z duong lam TCP Z tang. Chieu TCP Z DUNG.")
    else:
        print("KET LUAN buoc nay: chieu TCP Z co dau hieu NGUOC hoac TCP active dang sai.")

    urscript.move_linear(start_pose, accel=args.accel, vel=args.vel)
    wait_steady(rtde, "return_start_pose")
    returned_pose, _ = rtde.get_tcp_pose_with_timestamp()
    print(f"Return TCP: {fmt_pose(returned_pose)}")


def main() -> int:
    args = parse_args()
    dz_m = abs(args.dz_mm) / 1000.0

    print("=== TCP Z DIRECTION TEST ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"TCP offset runtime: {config.TCP_OFFSET}")
    print(f"Payload runtime: mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")
    print(f"Test step: +/-{args.dz_mm:.1f} mm")

    confirm(
        "Robot dang o vi tri an toan, co du khoang trong de di xuong va di len them mot buoc nho",
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
        urscript.set_tcp(config.TCP_OFFSET)
        urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)

        start_pose, _ = rtde.get_tcp_pose_with_timestamp()

        print("\n=== TEST 1: Z am ===")
        run_step(rtde, urscript, start_pose, -dz_m, args)

        print("\n=== TEST 2: Z duong ===")
        start_pose, _ = rtde.get_tcp_pose_with_timestamp()
        run_step(rtde, urscript, start_pose, +dz_m, args)

        print("\nTong ket:")
        print("- Neu TEST 1 cho Actual dz am va TEST 2 cho Actual dz duong: TCP Z dang dung.")
        print("- Neu nguoc lai: active TCP hoac TCP offset dang sai chieu.")
        print("- Neu TCP Z dung nhung mat tool ban NHIN thay van di nguoc: diem TCP dat sai vi tri tren tool.")
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
