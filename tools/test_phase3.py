"""Manual Phase-3 test tool - multi-shot pick-place up to max parts.

Expected flow:
  HOME -> SCAN_APPROACH -> SCAN_POSE
  LOOP until no parts detected (up to --max-parts):
    capture -> detect -> pick -> place -> return to SCAN_POSE
  HOME -> done

Run --preflight first to validate model/camera/ports without moving robot.
"""

import argparse
import json
import os
import socket
import sys
import time
import traceback
from pathlib import Path
from typing import Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from core.job_store import JobStore
from core.pick_place import PickPlaceCycle, AbortException
from core.pneumatic_gripper import PneumaticGripper
from robot.dashboard_client import DashboardClient
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from vision.detector import Detector


def check_tcp_port(host: str, port: int, timeout_s: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except Exception:
        return False


def run_preflight(robot_ip: str) -> int:
    print("\n=== PREFLIGHT PHASE 3 ===")

    model_path = config.YOLO_MODEL_PATH
    model_exists = os.path.isfile(model_path)
    print(f"YOLO_MODEL_PATH: {model_path}")
    print(f"Model exists: {model_exists}")
    if not model_exists:
        print("Loi: khong tim thay model. Dung lai preflight.")
        return 1

    try:
        detector = Detector(
            model_path=model_path,
            confidence=config.YOLO_CONFIDENCE,
            target_class=config.YOLO_TARGET_CLASS,
        )
        img = np.zeros((640, 640, 3), dtype=np.uint8)
        detections = detector.detect(img)
        print("Detector load: OK")
        print(f"Detector infer test: OK (detections={len(detections)})")
    except Exception as exc:
        print(f"Detector load/infer FAIL: {exc}")
        return 1

    robot_ports = [
        ("Dashboard", config.DASHBOARD_PORT),
        ("URScript", config.URSCRIPT_PORT),
        ("RTDE", config.RTDE_PORT),
    ]
    for name, port in robot_ports:
        ok = check_tcp_port(robot_ip, port)
        print(f"{name} port {port}: {'OK' if ok else 'FAIL'}")
        if not ok:
            print(f"Loi: khong ket noi duoc {name} port {port}. Kiem tra robot IP va trang thai nguon.")
            return 1

    try:
        from vision.femto_camera import FemtoCamera

        camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        camera.connect()
        try:
            rgb, depth, _ = camera.get_frames_with_timestamp()
            print(f"Camera stream: OK (rgb={rgb.shape}, depth={depth.shape})")
        finally:
            camera.disconnect()
    except Exception as exc:
        print(f"Camera stream: FAIL ({exc})")
        return 1

    print("Preflight thanh cong. Co the chay test Phase 3 that.")
    return 0


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy test Phase 3.")
        raise SystemExit(0)


def print_summary(job_snapshot: dict, result: Optional[dict]) -> None:
    print("\n=== KET QUA PHASE 3 ===")
    if result is not None:
        print(f"status: {result.get('status')}")
        print(f"stage: {result.get('stage')}")
        print(f"detected_objects: {result.get('detected_objects')}")
        print(f"parts_found: {result.get('parts_found')}")
        print(f"parts_picked: {result.get('parts_picked')}")

    print("\n=== JOB SNAPSHOT ===")
    print(f"job_id: {job_snapshot.get('job_id')}")
    print(f"status: {job_snapshot.get('status')}")
    print(f"phase: {job_snapshot.get('phase')}")
    print(f"parts_found: {job_snapshot.get('parts_found')}")
    print(f"parts_picked: {job_snapshot.get('parts_picked')}")
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test Phase 3 manual")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP")
    parser.add_argument("--station", default="phase3_manual_test", help="Station name")
    parser.add_argument(
        "--workflow-id",
        default=f"manual-phase3-{int(time.time())}",
        help="Workflow ID",
    )
    parser.add_argument("--yes", action="store_true", help="Skip safety confirmation")
    parser.add_argument("--log-tail", type=int, default=60, help="Tail lines to print")
    parser.add_argument("--preflight", action="store_true", help="Run preflight only")
    parser.add_argument("--max-parts", type=int, default=5, help="Expected max parts for validation")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== Phase 3 Manual Test ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Mode: full flow multi-shot (expected up to {args.max_parts} parts)")

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
        experiment_stage=3,
    )
    job_id = job["job_id"]
    print(f"\nDa tao job: {job_id} (experiment_stage=3)")

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
        print("\nPhase 3 ABORTED by user.")
    except Exception as exc:
        exit_code = 1
        print(f"\nPhase 3 THAT BAI: {exc}")
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
    print_summary(job_snapshot, result)
    print_logs(job_snapshot, max(1, args.log_tail))

    picked = int((result or {}).get("parts_picked") or 0)
    print(f"\nPicked summary: {picked}/{args.max_parts} (max expected)")

    if exit_code == 0:
        print("\nPhase 3 test HOAN TAT.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
