"""Manual Phase-1 test tool - basic UR5 motion + gripper response.

Run --preflight first to validate robot ports and gripper serial without
executing the full motion cycle.
"""

import argparse
import json
import socket
import sys
import time
import traceback
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from core.experiment_report_logger import experiment_report_logger
from core.job_store import JobStore
from core.pick_place import PickPlaceCycle, AbortException
from core.pneumatic_gripper import PneumaticGripper
from robot.dashboard_client import DashboardClient
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient


def check_tcp_port(host: str, port: int, timeout_s: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except Exception:
        return False


def run_preflight(robot_ip: str) -> int:
    print("\n=== PREFLIGHT PHASE 1 ===")
    robot_ports = [
        ("Dashboard", config.DASHBOARD_PORT),
        ("URScript", config.URSCRIPT_PORT),
        ("RTDE", config.RTDE_PORT),
    ]
    for name, port in robot_ports:
        ok = check_tcp_port(robot_ip, port)
        print(f"{name} port {port}: {'OK' if ok else 'FAIL'}")
        if not ok:
            print(f"Loi: khong ket noi duoc {name} port {port}.")
            return 1

    gripper = PneumaticGripper(
        port=config.GRIPPER_PORT,
        baud=config.GRIPPER_BAUD,
        cmd_timeout_s=config.GRIPPER_CMD_TIMEOUT_S,
        grip_settle_s=config.GRIPPER_SETTLE_S,
        release_settle_s=config.GRIPPER_RELEASE_SETTLE_S,
        heartbeat_interval_s=config.GRIPPER_HEARTBEAT_S,
    )
    try:
        gripper.connect()
        state = gripper.get_state()
        print(f"Gripper serial: OK (state={state})")
    except Exception as exc:
        print(f"Gripper serial: FAIL ({exc})")
        return 1
    finally:
        try:
            gripper.disconnect()
        except Exception:
            pass

    print("Preflight thanh cong. Co the chay test Phase 1 that.")
    return 0


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy test Phase 1.")
        raise SystemExit(0)


def print_summary(job_snapshot: dict, result: Optional[dict]) -> None:
    print("\n=== KET QUA PHASE 1 ===")
    if result is not None:
        print(f"status: {result.get('status')}")
        print(f"stage: {result.get('stage')}")
        print(f"cycle_duration_s: {result.get('cycle_duration_s')}")
        print(f"gripper_close_ms: {result.get('gripper_close_ms')}")
        print(f"gripper_open_ms: {result.get('gripper_open_ms')}")

    print("\n=== JOB SNAPSHOT ===")
    print(f"job_id: {job_snapshot.get('job_id')}")
    print(f"status: {job_snapshot.get('status')}")
    print(f"phase: {job_snapshot.get('phase')}")
    if job_snapshot.get("error"):
        print(f"error: {job_snapshot.get('error')}")


def print_logs(job_snapshot: dict, tail: int) -> None:
    logs = job_snapshot.get("log") or []
    if not logs:
        print("\nKhong co log nao duoc ghi.")
        return
    print(f"\n=== LOG CUOI ({min(len(logs), tail)} dong) ===")
    for line in logs[-tail:]:
        print(line)


def print_scenario_report(path: str, row: dict) -> None:
    print("\n=== LOG KICH BAN 1 ===")
    print(f"path: {path}")
    print(json.dumps(row, ensure_ascii=True, indent=2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test Phase 1 manual")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP")
    parser.add_argument("--station", default="phase1_manual_test", help="Station name")
    parser.add_argument(
        "--workflow-id",
        default=f"manual-phase1-{int(time.time())}",
        help="Workflow ID",
    )
    parser.add_argument("--yes", action="store_true", help="Skip safety confirmation")
    parser.add_argument("--log-tail", type=int, default=40, help="Tail lines to print")
    parser.add_argument("--preflight", action="store_true", help="Run preflight only")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("=== Phase 1 Manual Test ===")
    print(f"Robot IP: {args.robot_ip}")
    print("Mode: basic UR5 motion + gripper response")

    if args.preflight:
        return run_preflight(args.robot_ip)

    confirm("Robot o trang thai an toan, workspace clear, san sang chay", args.yes)

    dashboard = DashboardClient(args.robot_ip, port=config.DASHBOARD_PORT)
    urscript = URScriptClient(
        args.robot_ip,
        port=config.URSCRIPT_PORT,
        timeout=config.URSCRIPT_TIMEOUT,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    gripper = PneumaticGripper(
        port=config.GRIPPER_PORT,
        baud=config.GRIPPER_BAUD,
        cmd_timeout_s=config.GRIPPER_CMD_TIMEOUT_S,
        grip_settle_s=config.GRIPPER_SETTLE_S,
        release_settle_s=config.GRIPPER_RELEASE_SETTLE_S,
        heartbeat_interval_s=config.GRIPPER_HEARTBEAT_S,
    )

    job_store = JobStore()
    job = job_store.create_job(
        station=args.station,
        workflow_id=args.workflow_id,
        experiment_stage=1,
    )
    job_id = job["job_id"]
    print(f"\nDa tao job: {job_id} (experiment_stage=1)")

    result = None
    exit_code = 0
    try:
        dashboard.connect()
        urscript.connect()
        rtde.connect()
        gripper.connect()

        cycle = PickPlaceCycle(
            dashboard=dashboard,
            urscript=urscript,
            rtde=rtde,
            job_store=job_store,
            job_id=job_id,
            robot_ip=args.robot_ip,
            gripper=gripper,
        )
        result = cycle.run()

    except AbortException:
        exit_code = 130
        print("\nPhase 1 ABORTED by user.")
    except Exception as exc:
        exit_code = 1
        print(f"\nPhase 1 THAT BAI: {exc}")
        traceback.print_exc()
    finally:
        for client, name in [
            (gripper, "gripper"),
            (rtde, "rtde"),
            (urscript, "urscript"),
            (dashboard, "dashboard"),
        ]:
            try:
                client.disconnect()
            except Exception as err:
                print(f"Warning: {name} disconnect: {err}")

    job_snapshot = job_store.get_job(job_id) or {}
    report_path = None
    report_row = None
    try:
        report_path, report_row = experiment_report_logger.write_scenario1(job_snapshot, result or {})
    except Exception as err:
        print(f"Warning: ghi log kich ban 1 that bai: {err}")

    print_summary(job_snapshot, result)
    print_logs(job_snapshot, max(1, args.log_tail))
    if report_path and report_row:
        print_scenario_report(report_path, report_row)

    if exit_code == 0:
        print("\nPhase 1 test HOAN TAT.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
