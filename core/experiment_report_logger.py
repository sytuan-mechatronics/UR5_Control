"""Official experiment scenario log writers for scenarios 1, 2, 3."""

from __future__ import annotations

import csv
import os
import threading
import time
from datetime import datetime, timezone
from typing import Dict, List, Tuple

import config


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _ensure_csv_header(path: str, fieldnames: List[str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", newline="", encoding="utf-8") as file_obj:
            csv.DictWriter(file_obj, fieldnames=fieldnames).writeheader()


SCENARIO1_FIELDS = [
    "job_id", "created_at", "completed_at", "duration_s",
    "station", "workflow_id", "experiment_stage",
    "ur5_joint_speed_rad_s", "ur5_linear_speed_m_s", "ur5_pick_approach_speed_m_s",
    "cycle_status", "cycle_time_s",
    "gripper_close_ms", "gripper_open_ms",
    "warning_count", "error_or_warning", "note",
]

SCENARIO2_FIELDS = [
    "job_id", "created_at", "completed_at", "duration_s",
    "station", "workflow_id", "experiment_stage",
    "ur5_joint_speed_rad_s", "ur5_linear_speed_m_s", "ur5_pick_approach_speed_m_s",
    "confidence_yolo11", "localization_error_mm",
    "target_x_m", "target_y_m", "target_z_m",
    "pick_result", "retry_count", "slot_position", "selected_slot",
    "cycle_status", "parts_found", "parts_picked",
    "error_or_warning", "note",
]

SCENARIO3_FIELDS = [
    "job_id", "created_at", "completed_at", "duration_s",
    "station", "workflow_id", "experiment_stage",
    "ur5_joint_speed_rad_s", "ur5_linear_speed_m_s", "ur5_pick_approach_speed_m_s",
    "scenario_case", "parts_found_initial", "parts_picked_total",
    "run_status", "tray_position", "pick_order",
    "pick_result", "retry_count", "confidence_yolo11",
    "part_duration_s", "error_or_warning", "note",
]


class ExperimentReportLogger:
    def __init__(self) -> None:
        self._lock = threading.Lock()

    @staticmethod
    def _speed_fields() -> Dict[str, float]:
        return {
            "ur5_joint_speed_rad_s": config.JOINT_VEL,
            "ur5_linear_speed_m_s": config.LINEAR_VEL,
            "ur5_pick_approach_speed_m_s": config.PICK_APPROACH_VEL,
        }

    @staticmethod
    def _warning_count(job_snapshot: Dict) -> int:
        logs = job_snapshot.get("log") or []
        return sum(1 for line in logs if "CANH BAO" in line or "WARNING" in line)

    @staticmethod
    def _error_or_warning(job_snapshot: Dict) -> str:
        error = (job_snapshot.get("error") or "").strip()
        if error:
            return error
        logs = job_snapshot.get("log") or []
        warns = [line for line in logs if "CANH BAO" in line or "WARNING" in line]
        if warns:
            return warns[-1]
        return ""

    @staticmethod
    def _completed_at(job_snapshot: Dict) -> float:
        return float(job_snapshot.get("updated_at") or time.time())

    def write_scenario1(self, job_snapshot: Dict, result: Dict) -> Tuple[str, Dict]:
        created_at = float(job_snapshot.get("created_at") or time.time())
        completed_at = self._completed_at(job_snapshot)
        row = {
            "job_id": job_snapshot.get("job_id", ""),
            "created_at": _iso(created_at),
            "completed_at": _iso(completed_at),
            "duration_s": round(completed_at - created_at, 3),
            "station": job_snapshot.get("station", ""),
            "workflow_id": job_snapshot.get("workflow_id", ""),
            "experiment_stage": job_snapshot.get("experiment_stage", 1),
            **self._speed_fields(),
            "cycle_status": job_snapshot.get("status", ""),
            "cycle_time_s": result.get("cycle_duration_s", ""),
            "gripper_close_ms": result.get("gripper_close_ms", ""),
            "gripper_open_ms": result.get("gripper_open_ms", ""),
            "warning_count": self._warning_count(job_snapshot),
            "error_or_warning": self._error_or_warning(job_snapshot),
            "note": "",
        }
        with self._lock:
            _ensure_csv_header(config.RESULTS_SCENARIO1_CSV, SCENARIO1_FIELDS)
            with open(config.RESULTS_SCENARIO1_CSV, "a", newline="", encoding="utf-8") as file_obj:
                csv.DictWriter(file_obj, fieldnames=SCENARIO1_FIELDS).writerow(row)
        return config.RESULTS_SCENARIO1_CSV, row

    def write_scenario2(self, job_snapshot: Dict, result: Dict) -> Tuple[str, Dict]:
        created_at = float(job_snapshot.get("created_at") or time.time())
        completed_at = self._completed_at(job_snapshot)
        target_base = result.get("target_base") or [None, None, None]
        row = {
            "job_id": job_snapshot.get("job_id", ""),
            "created_at": _iso(created_at),
            "completed_at": _iso(completed_at),
            "duration_s": round(completed_at - created_at, 3),
            "station": job_snapshot.get("station", ""),
            "workflow_id": job_snapshot.get("workflow_id", ""),
            "experiment_stage": job_snapshot.get("experiment_stage", 2),
            **self._speed_fields(),
            "confidence_yolo11": result.get("confidence", ""),
            "localization_error_mm": "",
            "target_x_m": target_base[0] if len(target_base) > 0 else "",
            "target_y_m": target_base[1] if len(target_base) > 1 else "",
            "target_z_m": target_base[2] if len(target_base) > 2 else "",
            "pick_result": result.get("pick_result", ""),
            "retry_count": result.get("retries_used", 0),
            "slot_position": result.get("selected_slot", ""),
            "selected_slot": result.get("selected_slot", ""),
            "cycle_status": job_snapshot.get("status", ""),
            "parts_found": result.get("parts_found", 0),
            "parts_picked": result.get("parts_picked", 0),
            "error_or_warning": self._error_or_warning(job_snapshot),
            "note": "",
        }
        with self._lock:
            _ensure_csv_header(config.RESULTS_SCENARIO2_CSV, SCENARIO2_FIELDS)
            with open(config.RESULTS_SCENARIO2_CSV, "a", newline="", encoding="utf-8") as file_obj:
                csv.DictWriter(file_obj, fieldnames=SCENARIO2_FIELDS).writerow(row)
        return config.RESULTS_SCENARIO2_CSV, row

    def write_scenario3(self, job_snapshot: Dict, result: Dict, scenario_case: str = "") -> Tuple[str, List[Dict]]:
        created_at = float(job_snapshot.get("created_at") or time.time())
        completed_at = self._completed_at(job_snapshot)
        common = {
            "job_id": job_snapshot.get("job_id", ""),
            "created_at": _iso(created_at),
            "completed_at": _iso(completed_at),
            "duration_s": round(completed_at - created_at, 3),
            "station": job_snapshot.get("station", ""),
            "workflow_id": job_snapshot.get("workflow_id", ""),
            "experiment_stage": job_snapshot.get("experiment_stage", 3),
            **self._speed_fields(),
            "scenario_case": scenario_case,
            "parts_found_initial": result.get("parts_found", 0),
            "parts_picked_total": result.get("parts_picked", 0),
            "run_status": job_snapshot.get("status", ""),
            "error_or_warning": self._error_or_warning(job_snapshot),
            "note": "",
        }
        records = result.get("pick_records") or []
        rows: List[Dict] = []
        if records:
            for rec in records:
                rows.append({
                    **common,
                    "tray_position": rec.get("tray_position", ""),
                    "pick_order": rec.get("pick_index", ""),
                    "pick_result": "success" if rec.get("grip_success") else "fail",
                    "retry_count": rec.get("retries_used", ""),
                    "confidence_yolo11": rec.get("confidence", ""),
                    "part_duration_s": rec.get("part_duration_s", ""),
                })
        else:
            rows.append({
                **common,
                "tray_position": "",
                "pick_order": "",
                "pick_result": "no_pick",
                "retry_count": "",
                "confidence_yolo11": "",
                "part_duration_s": "",
            })

        with self._lock:
            _ensure_csv_header(config.RESULTS_SCENARIO3_CSV, SCENARIO3_FIELDS)
            with open(config.RESULTS_SCENARIO3_CSV, "a", newline="", encoding="utf-8") as file_obj:
                writer = csv.DictWriter(file_obj, fieldnames=SCENARIO3_FIELDS)
                writer.writerows(rows)
        return config.RESULTS_SCENARIO3_CSV, rows


experiment_report_logger = ExperimentReportLogger()
