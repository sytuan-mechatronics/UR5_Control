"""Diagnose whether runtime set_tcp/set_payload causes Z-motion mismatch.

This tool runs a tiny Z descend/ascend test from the current pose and lets the
operator choose whether code should apply TCP and/or payload first.
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
        description="Test runtime TCP/payload influence on small Z moves",
    )
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dz-mm", type=float, default=10.0)
    parser.add_argument("--vel", type=float, default=0.02)
    parser.add_argument("--accel", type=float, default=0.08)
    parser.add_argument("--skip-set-tcp", action="store_true")
    parser.add_argument("--skip-set-payload", action="store_true")
    return parser.parse_args()


def run_step(rtde: RTDEClient, urscript: URScriptClient, start_pose, dz_m: float, accel: float, vel: float) -> None:
    target_pose = [start_pose[0], start_pose[1], start_pose[2] + dz_m, start_pose[3], start_pose[4], start_pose[5]]
    print(f"\nStart TCP:  {fmt_pose(start_pose)}")
    print(f"Target TCP: {fmt_pose(target_pose)}")
    print(f"Commanded dz = {dz_m * 1000.0:+.1f} mm")

    urscript.move_linear(target_pose, accel=accel, vel=vel)
    wait_steady(rtde, f"move_z_{dz_m:+.3f}")
    actual_pose, _ = rtde.get_tcp_pose_with_timestamp()
    actual_dz_mm = (actual_pose[2] - start_pose[2]) * 1000.0
    z_error_mm = (actual_pose[2] - target_pose[2]) * 1000.0

    print(f"Actual TCP: {fmt_pose(actual_pose)}")
    print(f"Actual dz  = {actual_dz_mm:+.1f} mm")
    print(f"Z error    = {z_error_mm:+.1f} mm")

    urscript.move_linear(start_pose, accel=accel, vel=vel)
    wait_steady(rtde, "return_start_pose")
    returned_pose, _ = rtde.get_tcp_pose_with_timestamp()
    print(f"Return TCP: {fmt_pose(returned_pose)}")


def main() -> int:
    args = parse_args()
    dz_m = abs(args.dz_mm) / 1000.0

    print("=== RUNTIME TCP/PAYLOAD Z TEST ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"skip_set_tcp     = {args.skip_set_tcp}")
    print(f"skip_set_payload = {args.skip_set_payload}")
    print(f"runtime TCP      = {config.TCP_OFFSET}")
    print(f"runtime payload  = mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")
    print(f"test step        = +/-{args.dz_mm:.1f} mm")

    confirm(
        "Robot dang o vi tri an toan, co du khoang trong de di xuong va di len them mot buoc nho",
        args.yes,
    )

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(args.robot_ip, port=config.URSCRIPT_PORT, timeout=config.URSCRIPT_TIMEOUT)

    try:
        rtde.connect()
        urscript.connect()

        if args.skip_set_tcp:
            print("Bo qua set_tcp() tu code. Dung TCP active hien co tren robot.")
        else:
            urscript.set_tcp(config.TCP_OFFSET)
            print(f"Da set TCP tu code: {config.TCP_OFFSET}")

        if args.skip_set_payload:
            print("Bo qua set_payload() tu code. Dung payload/CoG active hien co tren robot.")
        else:
            urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
            print(f"Da set payload tu code: mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")

        start_pose, _ = rtde.get_tcp_pose_with_timestamp()

        print("\n=== TEST 1: Z am ===")
        run_step(rtde, urscript, start_pose, -dz_m, args.accel, args.vel)

        print("\n=== TEST 2: Z duong ===")
        start_pose, _ = rtde.get_tcp_pose_with_timestamp()
        run_step(rtde, urscript, start_pose, +dz_m, args.accel, args.vel)

        print("\nGoi y doc ket qua:")
        print("- Neu skip-set-payload giup robot xuong duoc tot hon: nghi payload/CoG do code set.")
        print("- Neu skip-set-tcp giup robot xuong duoc tot hon: nghi TCP do code set.")
        print("- Neu chi khi bo ca hai moi tot: pendant va runtime dang lech ca TCP lan payload.")
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
