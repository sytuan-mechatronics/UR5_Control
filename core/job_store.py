"""
Job Store - Thread-safe in-memory job storage.
Tracks status of pick-place cycles.
"""

import logging
import time
import threading
import uuid
from typing import Dict, Optional, List


logger = logging.getLogger(__name__)


class JobStore:
    """Thread-safe job storage."""

    def __init__(self):
        """Initialize job store."""
        self.jobs: Dict[str, Dict] = {}
        self.lock = threading.Lock()
        self.abort_events: Dict[str, threading.Event] = {}

    def create_job(self, station: str, workflow_id: str, experiment_stage: int = 1) -> Dict:
        """
        Create new job.
        
        Args:
            station: Station/location name
            workflow_id: Workflow ID from PC1
            experiment_stage: Current experiment stage (1..3)
            
        Returns:
            Job dict with structure:
            {
              "job_id": str (8-char hex),
              "status": "accepted",
              "phase": "initializing",
              "station": str,
              "workflow_id": str,
              "experiment_stage": int,
              "parts_found": 0,
              "parts_picked": 0,
              "error": None,
              "created_at": float (timestamp),
              "updated_at": float (timestamp),
              "log": list[str],
            }
        """
        job_id = uuid.uuid4().hex[:8]
        now = time.time()

        job = {
            "job_id": job_id,
            "status": "accepted",
            "phase": "initializing",
            "station": station,
            "workflow_id": workflow_id,
            "experiment_stage": experiment_stage,
            "parts_found": 0,
            "parts_picked": 0,
            "error": None,
            "created_at": now,
            "updated_at": now,
            "log": [],
        }

        with self.lock:
            self.jobs[job_id] = job
            self.abort_events[job_id] = threading.Event()

        logger.info(f"Created job {job_id} for workflow {workflow_id}")
        return job

    def get_job(self, job_id: str) -> Optional[Dict]:
        """
        Get job by ID.
        
        Args:
            job_id: Job ID
            
        Returns:
            Job dict or None if not found
        """
        with self.lock:
            return self.jobs.get(job_id)

    def update_job(self, job_id: str, **kwargs) -> None:
        """
        Update job fields.
        
        Args:
            job_id: Job ID
            **kwargs: Fields to update
            
        Raises:
            KeyError: If job not found
        """
        with self.lock:
            if job_id not in self.jobs:
                raise KeyError(f"Job {job_id} not found")

            job = self.jobs[job_id]
            
            # Update allowed fields
            for key in ["status", "phase", "parts_found", "parts_picked", "error"]:
                if key in kwargs:
                    job[key] = kwargs[key]

            # Always update timestamp
            job["updated_at"] = time.time()

            logger.debug(f"Job {job_id} updated: {kwargs}")

    def append_log(self, job_id: str, message: str) -> None:
        """
        Append log message to job.
        
        Args:
            job_id: Job ID
            message: Log message
        """
        with self.lock:
            if job_id not in self.jobs:
                return

            job = self.jobs[job_id]
            
            # Keep only last 100 log entries
            job["log"].append(f"[{time.time():.1f}] {message}")
            if len(job["log"]) > 100:
                job["log"] = job["log"][-100:]

        logger.info(f"Job {job_id}: {message}")

    def set_phase(self, job_id: str, phase: str) -> None:
        """
        Set current phase and log it.
        
        Args:
            job_id: Job ID
            phase: Phase name
        """
        self.update_job(job_id, phase=phase)
        self.append_log(job_id, f"Phase: {phase}")

    def abort_job(self, job_id: str) -> bool:
        """
        Request job abort.
        
        Args:
            job_id: Job ID
            
        Returns:
            True if job exists and abort was set, False if job not found
        """
        with self.lock:
            if job_id not in self.jobs:
                return False

            job = self.jobs[job_id]
            if job["status"] not in ["accepted", "running"]:
                logger.warning(f"Cannot abort job {job_id} with status {job['status']}")
                return False

            self.abort_events[job_id].set()
            self.jobs[job_id]["status"] = "aborting"

        logger.warning(f"Abort requested for job {job_id}")
        return True

    def is_aborted(self, job_id: str) -> bool:
        """
        Check if job abort was requested.
        
        Args:
            job_id: Job ID
            
        Returns:
            True if abort event is set
        """
        with self.lock:
            if job_id not in self.abort_events:
                return False
            return self.abort_events[job_id].is_set()

    def get_abort_event(self, job_id: str) -> Optional[threading.Event]:
        """
        Get abort event (for waiting).
        
        Args:
            job_id: Job ID
            
        Returns:
            threading.Event or None
        """
        with self.lock:
            return self.abort_events.get(job_id)

    def list_jobs(self) -> List[Dict]:
        """
        Get all jobs.
        
        Returns:
            List of job dicts
        """
        with self.lock:
            return list(self.jobs.values())

    def list_active_jobs(self) -> List[Dict]:
        """
        Get active jobs (status: accepted or running).
        
        Returns:
            List of job dicts with status in [accepted, running]
        """
        with self.lock:
            return [
                job for job in self.jobs.values()
                if job["status"] in ["accepted", "running"]
            ]

    def cleanup_old_jobs(self, max_age_seconds: float = 86400) -> None:
        """
        Remove jobs older than max_age_seconds (for memory management).
        
        Args:
            max_age_seconds: Age threshold in seconds (default 24 hours)
        """
        now = time.time()
        
        with self.lock:
            to_delete = [
                job_id for job_id, job in self.jobs.items()
                if (now - job["updated_at"] > max_age_seconds and
                    job["status"] in ["done", "error", "aborted"])
            ]

            for job_id in to_delete:
                del self.jobs[job_id]
                del self.abort_events[job_id]

        if to_delete:
            logger.info(f"Cleaned up {len(to_delete)} old jobs")
