"""
Job Result Logger — Phase 3 pick outcome CSV writer for system experiments.

Only writes to CSV when a job completes normally (status="done").
Aborted or errored jobs are silently discarded from the buffer.

Outputs:
  results/job_summary.csv     — one row per completed job
  results/pick_detections.csv — one row per pick attempt (all cycles)
"""

import csv
import logging
import os
import threading
import time
from datetime import datetime, timezone
from typing import Dict, List, Optional

import config

logger = logging.getLogger(__name__)


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _ensure_csv_header(path: str, fieldnames: List[str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=fieldnames).writeheader()


SUMMARY_FIELDS = [
    "job_id", "created_at", "completed_at", "duration_s",
    "station", "workflow_id", "experiment_stage",
    "parts_target", "first_scan_count", "parts_picked", "success_rate_pct",
    "first_attempt_picks", "retry_picks", "all_fail_cycles",
]

DETECTION_FIELDS = [
    "job_id", "pick_index", "cycle_number",
    "confidence", "depth_mm",
    "pick_u", "pick_v",
    "retries_used", "grip_success",
]


class JobResultLogger:
    """
    Thread-safe per-job buffer that flushes to CSV on normal completion.

    Typical lifecycle per job:
        start_job()                    # Phase 3 begins
        set_first_scan_count()         # after first detection cycle
        record_pick() × N              # after each grip attempt (success or fail)
        finalize()  OR  discard()      # done vs abort/error
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._buffers: Dict[str, Dict] = {}

    def start_job(
        self,
        job_id: str,
        created_at: float,
        station: str,
        workflow_id: str,
        experiment_stage: int,
    ) -> None:
        with self._lock:
            self._buffers[job_id] = {
                "created_at": created_at,
                "station": station,
                "workflow_id": workflow_id,
                "experiment_stage": experiment_stage,
                "first_scan_count": None,
                "pick_records": [],
                "_counter": 0,
            }
        logger.debug("JobLogger: started buffer for %s", job_id)

    def set_first_scan_count(self, job_id: str, count: int) -> None:
        with self._lock:
            buf = self._buffers.get(job_id)
            if buf is not None and buf["first_scan_count"] is None:
                buf["first_scan_count"] = count

    def record_pick(
        self,
        job_id: str,
        cycle_number: int,
        confidence: float,
        depth_mm: float,
        pick_u: float,
        pick_v: float,
        retries_used: int,
        grip_success: bool,
    ) -> None:
        with self._lock:
            buf = self._buffers.get(job_id)
            if buf is None:
                return
            buf["_counter"] += 1
            buf["pick_records"].append({
                "pick_index": buf["_counter"],
                "cycle_number": cycle_number,
                "confidence": round(float(confidence), 4),
                "depth_mm": round(float(depth_mm), 1),
                "pick_u": round(float(pick_u), 1),
                "pick_v": round(float(pick_v), 1),
                "retries_used": retries_used,
                "grip_success": grip_success,
            })

    def finalize(self, job_id: str, parts_picked: int, completed_at: Optional[float] = None) -> None:
        with self._lock:
            buf = self._buffers.pop(job_id, None)
        if buf is None:
            logger.warning("JobLogger: finalize called for unknown/already-finalized job %s", job_id)
            return

        if completed_at is None:
            completed_at = time.time()

        records = buf["pick_records"]
        first_attempt_picks = sum(1 for r in records if r["grip_success"] and r["retries_used"] == 0)
        retry_picks         = sum(1 for r in records if r["grip_success"] and r["retries_used"] > 0)
        all_fail_cycles     = sum(1 for r in records if not r["grip_success"])

        summary_row = {
            "job_id":              job_id,
            "created_at":          _iso(buf["created_at"]),
            "completed_at":        _iso(completed_at),
            "duration_s":          round(completed_at - buf["created_at"], 1),
            "station":             buf["station"],
            "workflow_id":         buf["workflow_id"],
            "experiment_stage":    buf["experiment_stage"],
            "parts_target":        config.PICK_TARGET_COUNT,
            "first_scan_count":    buf["first_scan_count"] if buf["first_scan_count"] is not None else "",
            "parts_picked":        parts_picked,
            "success_rate_pct":    round(parts_picked / max(config.PICK_TARGET_COUNT, 1) * 100, 1),
            "first_attempt_picks": first_attempt_picks,
            "retry_picks":         retry_picks,
            "all_fail_cycles":     all_fail_cycles,
        }

        try:
            _ensure_csv_header(config.RESULTS_JOB_SUMMARY_CSV, SUMMARY_FIELDS)
            with open(config.RESULTS_JOB_SUMMARY_CSV, "a", newline="", encoding="utf-8") as f:
                csv.DictWriter(f, fieldnames=SUMMARY_FIELDS).writerow(summary_row)
            logger.info("JobLogger: summary row written for job %s", job_id)
        except Exception as exc:
            logger.error("JobLogger: failed to write summary for %s: %s", job_id, exc)

        if records:
            try:
                _ensure_csv_header(config.RESULTS_PICK_DETECTIONS_CSV, DETECTION_FIELDS)
                with open(config.RESULTS_PICK_DETECTIONS_CSV, "a", newline="", encoding="utf-8") as f:
                    writer = csv.DictWriter(f, fieldnames=DETECTION_FIELDS)
                    for rec in records:
                        writer.writerow({"job_id": job_id, **rec})
                logger.info(
                    "JobLogger: %d detection rows written for job %s",
                    len(records), job_id,
                )
            except Exception as exc:
                logger.error("JobLogger: failed to write detections for %s: %s", job_id, exc)

    def discard(self, job_id: str) -> None:
        with self._lock:
            self._buffers.pop(job_id, None)
        logger.debug("JobLogger: discarded buffer for %s (abort/error)", job_id)


# Module-level singleton — import this directly in pick_place.py
job_result_logger = JobResultLogger()
