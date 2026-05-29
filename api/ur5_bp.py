"""
Flask Blueprint for UR5 REST API.
Endpoints for job execution, status tracking, and abort.
"""

import logging
import threading
import time
import requests
from flask import Blueprint, request, jsonify
from typing import Tuple

from core.job_store import JobStore
from core.pick_place import PickPlaceCycle
import config


logger = logging.getLogger(__name__)

ur5_bp = Blueprint("ur5", __name__, url_prefix="/api/ur5")

# Global job store (initialized by app)
job_store: JobStore = None
active_job_lock = threading.Lock()
active_job_id: str = None


def set_job_store(store: JobStore) -> None:
    """Set the global job store (called by app.py)."""
    global job_store
    job_store = store


def _run_job(job_id: str) -> None:
    """
    Run pick-place cycle for job (in separate thread).
    
    Args:
        job_id: Job ID to execute
    """
    global active_job_id
    
    try:
        logger.info(f"Starting job thread for {job_id}")
        
        cycle = PickPlaceCycle(
            robot_ip=config.ROBOT_IP,
            job_store=job_store,
            job_id=job_id
        )

        result = cycle.run()
        job_snapshot = job_store.get_job(job_id) or {}
        
        logger.info(f"Job {job_id} completed: {result}")

        # Call PC1 callback if enabled
        if config.PC1_CALLBACK_ENABLED:
            _notify_pc1_done(
                job_id,
                success=True,
                result=result,
                workflow_id=job_snapshot.get("workflow_id"),
                status=job_snapshot.get("status", "done"),
                experiment_stage=job_snapshot.get("experiment_stage"),
            )

    except Exception as e:
        logger.error(f"Job {job_id} failed: {e}")
        job_snapshot = job_store.get_job(job_id) or {}
        
        # Call PC1 callback with error
        if config.PC1_CALLBACK_ENABLED:
            _notify_pc1_done(
                job_id,
                success=False,
                error=str(e),
                workflow_id=job_snapshot.get("workflow_id"),
                status=job_snapshot.get("status", "error"),
                experiment_stage=job_snapshot.get("experiment_stage"),
            )

    finally:
        with active_job_lock:
            if active_job_id == job_id:
                active_job_id = None
        logger.info(f"Job {job_id} thread finished")


def _notify_pc1_done(
    job_id: str,
    success: bool,
    result: dict = None,
    error: str = None,
    workflow_id: str = None,
    status: str = None,
    experiment_stage: int = None,
) -> None:
    """
    Thông báo PC1 rằng job đã xong.

    Retry tối đa 5 lần với exponential backoff (1s, 2s, 4s, 8s, 16s).
    Nếu PC1 đang bận hoặc mạng bị ngắt ngắn, tránh mất callback
    khiến PC1 không biết UR5 đã done → workflow bị kẹt.
    """
    import time as _time  # tránh shadow biến time ở scope ngoài

    headers = {}
    if config.PC1_WEBHOOK_SECRET:
        headers["X-UR5-Secret"] = config.PC1_WEBHOOK_SECRET

    payload = {"job_id": job_id, "success": success}
    if workflow_id:
        payload["workflow_id"] = workflow_id
    if status:
        payload["status"] = status
    if isinstance(experiment_stage, int):
        payload["experiment_stage"] = experiment_stage
    if result:
        payload["result"] = result
    if error:
        payload["error"] = error

    max_attempts = 5
    for attempt in range(max_attempts):
        try:
            resp = requests.post(
                config.PC1_UR5_DONE_URL,
                json=payload,
                headers=headers,
                timeout=5.0,
            )
            if resp.status_code < 400:
                logger.info(
                    "PC1 callback OK (attempt %d/%d, status %d)",
                    attempt + 1, max_attempts, resp.status_code
                )
                return
            logger.warning(
                "PC1 callback HTTP %d (attempt %d/%d)",
                resp.status_code, attempt + 1, max_attempts
            )
        except Exception as exc:
            logger.warning(
                "PC1 callback failed (attempt %d/%d): %s",
                attempt + 1, max_attempts, exc
            )

        if attempt < max_attempts - 1:
            delay = 2 ** attempt  # 1s, 2s, 4s, 8s (sau lần cuối không sleep)
            logger.debug("Retrying PC1 callback in %ds...", delay)
            _time.sleep(delay)

    logger.error(
        "PC1 callback failed after %d attempts, job_id=%s — "
        "PC1 workflow có thể bị kẹt!",
        max_attempts, job_id
    )


# ==================== ENDPOINTS ====================


@ur5_bp.route("/execute", methods=["POST"])
def execute_job() -> Tuple[dict, int]:
    """
    Execute pick-place cycle.
    
        Request body (minimal):
        {
            "phase": 1|2|3
        }

        Request body (extended):
        {
            "phase": 1|2|3,
            "station": str,
            "workflow_id": str
        }
    
    Response: (202 Accepted)
    {
      "job_id": str,
      "status": "accepted"
    }
    
    Error: (400 Bad Request / 409 Conflict)
    """
    global active_job_id

    # Validate request
    data = request.get_json() or {}
    station = str(data.get("station") or "ur5_station").strip() or "ur5_station"
    workflow_id = str(data.get("workflow_id") or f"manual-{int(time.time())}").strip()
    requested_stage = data.get("phase", data.get("experiment_stage", config.EXPERIMENT_STAGE))

    try:
        experiment_stage = int(requested_stage)
    except (TypeError, ValueError):
        return {
            "error": "experiment_stage must be an integer in [1,2,3]"
        }, 400

    if experiment_stage not in (1, 2, 3):
        return {
            "error": "experiment_stage must be one of [1,2,3]"
        }, 400

    # Check if another job is already running
    with active_job_lock:
        if active_job_id is not None:
            logger.warning(f"Job {active_job_id} already running, rejecting new job")
            return {
                "error": f"Job {active_job_id} already running",
                "active_job_id": active_job_id
            }, 409

        # Create new job
        job = job_store.create_job(station, workflow_id, experiment_stage=experiment_stage)
        job_id = job["job_id"]
        active_job_id = job_id

    # Start job in background thread
    thread = threading.Thread(target=_run_job, args=(job_id,), daemon=True)
    thread.start()

    logger.info(f"Job {job_id} accepted and started")

    return {
        "job_id": job_id,
        "status": "accepted",
        "experiment_stage": experiment_stage,
    }, 202


@ur5_bp.route("/status/<job_id>", methods=["GET"])
def get_status(job_id: str) -> Tuple[dict, int]:
    """
    Get job status.
    
    Response: (200 OK)
    {
      "job_id": str,
      "status": "accepted|running|done|error|aborted|aborting",
      "phase": str,
      "station": str,
      "workflow_id": str,
      "parts_found": int,
      "parts_picked": int,
      "error": str or null,
      "created_at": float,
      "updated_at": float,
      "log": list[str]
    }
    
    Error: (404 Not Found)
    """
    job = job_store.get_job(job_id)

    if job is None:
        logger.warning(f"Job {job_id} not found")
        return {
            "error": f"Job {job_id} not found"
        }, 404

    return job, 200


@ur5_bp.route("/abort/<job_id>", methods=["POST"])
def abort_job(job_id: str) -> Tuple[dict, int]:
    """
    Request job abort.
    
    Response: (200 OK)
    {
      "aborted": true
    }
    
    Error: (404 Not Found)
    """
    success = job_store.abort_job(job_id)

    if not success:
        logger.warning(f"Cannot abort job {job_id}")
        return {
            "error": f"Job {job_id} not found or cannot be aborted"
        }, 404

    logger.info(f"Abort request processed for {job_id}")

    return {
        "aborted": True,
        "job_id": job_id
    }, 200


@ur5_bp.route("/health", methods=["GET"])
def health() -> Tuple[dict, int]:
    """
    Health check - does NOT connect to robot.
    
    Response: (200 OK)
    {
      "status": "ok",
      "robot_ip": str,
      "pc2_port": int
    }
    """
    return {
        "status": "ok",
        "robot_ip": config.ROBOT_IP,
        "pc2_port": config.PC2_PORT
    }, 200
