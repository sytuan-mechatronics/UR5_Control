"""
PC2 Flask Application - UR5 Robot Control Server.
Receives commands from PC1 via REST API and orchestrates pick-place operations.
"""

import logging
import logging.handlers
import sys
import atexit
import threading
import time
from pathlib import Path
from typing import Dict

from flask import Flask, request
from api.ur5_bp import ur5_bp, set_job_store, set_robot_clients
from core.job_store import JobStore
from core.pneumatic_gripper import PneumaticGripper, GripperError
from robot.dashboard_client import DashboardClient
from robot.urscript_client import URScriptClient
from robot.rtde_client import RTDEClient
import config


# ==================== LOGGING SETUP ====================

def setup_logging() -> None:
    """Configure logging to console and file."""
    log_level = getattr(logging, config.LOG_LEVEL.upper(), logging.INFO)

    # Create logs directory if needed
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)

    # Root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    console_handler.setFormatter(console_formatter)
    root_logger.addHandler(console_handler)

    # File handler
    file_handler = logging.handlers.RotatingFileHandler(
        log_dir / "pc2_ur5.log",
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(log_level)
    file_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    file_handler.setFormatter(file_formatter)
    root_logger.addHandler(file_handler)

    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)


# ==================== FLASK APP ====================


class RobotConnectionManager:
    """Manage shared robot client connections with thread-safe reconnect."""

    # REFACTORED
    def __init__(
        self,
        dashboard: DashboardClient,
        urscript: URScriptClient,
        rtde: RTDEClient,
        max_reconnect_attempts: int = 3,
        reconnect_delay_s: float = 1.0,
    ) -> None:
        self.dashboard = dashboard
        self.urscript = urscript
        self.rtde = rtde
        self.max_reconnect_attempts = max_reconnect_attempts
        self.reconnect_delay_s = reconnect_delay_s
        self._lock = threading.Lock()

    def _dashboard_connected(self) -> bool:
        socket_obj = getattr(self.dashboard, "socket", None)
        return socket_obj is not None and socket_obj.fileno() != -1

    def _urscript_connected(self) -> bool:
        socket_obj = getattr(self.urscript, "socket", None)
        return socket_obj is not None and socket_obj.fileno() != -1

    def _rtde_connected(self) -> bool:
        return getattr(self.rtde, "client", None) is not None

    def _reconnect_dashboard(self) -> None:
        try:
            self.dashboard.disconnect()
        except Exception:
            pass
        self.dashboard.connect()

    def _reconnect_urscript(self) -> None:
        try:
            self.urscript.disconnect()
        except Exception:
            pass
        self.urscript.connect()

    def _reconnect_rtde(self) -> None:
        try:
            self.rtde.disconnect()
        except Exception:
            pass
        self.rtde.connect()

    # REFACTORED
    def ensure_connected(self) -> bool:
        """Ensure shared clients are alive; reconnect with retry if needed."""
        with self._lock:
            for attempt in range(1, self.max_reconnect_attempts + 1):
                try:
                    dashboard_ok = self._dashboard_connected()
                    urscript_ok = self._urscript_connected()
                    rtde_ok = self._rtde_connected()

                    if dashboard_ok:
                        try:
                            self.dashboard.get_robotmode()
                        except Exception:
                            dashboard_ok = False

                    if rtde_ok:
                        try:
                            self.rtde.get_joint_speeds()
                        except Exception:
                            rtde_ok = False

                    if dashboard_ok and urscript_ok and rtde_ok:
                        return True

                    if not dashboard_ok:
                        logging.warning("Dashboard connection is down; reconnecting")
                        self._reconnect_dashboard()

                    if not urscript_ok:
                        logging.warning("URScript connection is down; reconnecting")
                        self._reconnect_urscript()

                    if not rtde_ok:
                        logging.warning("RTDE connection is down/unhealthy; reconnecting")
                        # Selective reconnect: keep dashboard/urscript if still alive.
                        self._reconnect_rtde()

                    self.dashboard.get_robotmode()
                    self.rtde.get_joint_speeds()
                    if not self._urscript_connected():
                        raise RuntimeError("URScript socket still unavailable after reconnect")
                    return True
                except Exception as err:
                    logging.error(
                        "ensure_connected attempt %d/%d failed: %s",
                        attempt,
                        self.max_reconnect_attempts,
                        err,
                    )
                    if attempt < self.max_reconnect_attempts:
                        time.sleep(self.reconnect_delay_s)
            return False

    # REFACTORED
    def get_status(self) -> Dict[str, object]:
        """Return current robot connection health for /health endpoint."""
        with self._lock:
            dashboard_ok = self._dashboard_connected()
            urscript_ok = self._urscript_connected()
            rtde_ok = self._rtde_connected()
            if rtde_ok:
                try:
                    self.rtde.get_joint_speeds()
                except Exception:
                    rtde_ok = False

            return {
                "dashboard_connected": dashboard_ok,
                "urscript_connected": urscript_ok,
                "rtde_connected": rtde_ok,
                "all_connected": dashboard_ok and urscript_ok and rtde_ok,
            }

def create_app() -> Flask:
    """Create and configure Flask application."""
    app = Flask(__name__)

    # Initialize job store
    job_store = JobStore()
    set_job_store(job_store)

    # Register blueprints
    app.register_blueprint(ur5_bp)

    # Request logging
    @app.before_request
    def log_request() -> None:
        """Log incoming request."""
        logging.info(
            f"{request.method} {request.path} - "
            f"Client: {request.remote_addr}"
        )

    @app.after_request
    def log_response(response):
        """Log response."""
        logging.info(
            f"{request.method} {request.path} - "
            f"Status: {response.status_code}"
        )
        return response

    @app.errorhandler(404)
    def not_found(e):
        """Handle 404 errors."""
        return {"error": "Endpoint not found"}, 404

    @app.errorhandler(500)
    def internal_error(e):
        """Handle 500 errors."""
        logging.error(f"Internal error: {e}")
        return {"error": "Internal server error"}, 500

    return app


# ==================== MAIN ====================

if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)

    # REFACTORED
    _shared_dashboard = DashboardClient(config.ROBOT_IP)
    _shared_urscript = URScriptClient(config.ROBOT_IP)
    _shared_rtde = RTDEClient(config.ROBOT_IP, frequency=10.0)
    _connection_manager = RobotConnectionManager(
        dashboard=_shared_dashboard,
        urscript=_shared_urscript,
        rtde=_shared_rtde,
    )
    _shared_gripper = PneumaticGripper(
        port=config.GRIPPER_PORT,
        baud=config.GRIPPER_BAUD,
        cmd_timeout_s=config.GRIPPER_CMD_TIMEOUT_S,
        grip_settle_s=config.GRIPPER_SETTLE_S,
        release_settle_s=config.GRIPPER_RELEASE_SETTLE_S,
        heartbeat_interval_s=config.GRIPPER_HEARTBEAT_S,
    )  # REFACTORED
    try:
        _shared_gripper.connect()  # REFACTORED
        logger.info("Pneumatic gripper connected on %s", config.GRIPPER_PORT)
    except GripperError as err:
        logger.error("Pneumatic gripper connect FAILED: %s", err)

    atexit.register(_shared_gripper.disconnect)  # REFACTORED

    logger.info("=" * 70)
    logger.info("PC2 UR5 Robot Control Server")
    logger.info("=" * 70)
    logger.info(f"Robot IP: {config.ROBOT_IP}")
    logger.info(f"PC2 Server: {config.PC2_HOST}:{config.PC2_PORT}")
    logger.info(f"PC1 Callback: {config.PC1_UR5_DONE_URL if config.PC1_CALLBACK_ENABLED else 'Disabled'}")
    logger.info("=" * 70)

    app = create_app()
    # REFACTORED
    set_robot_clients(  # REFACTORED
        dashboard=_shared_dashboard,
        urscript=_shared_urscript,
        rtde=_shared_rtde,
        manager=_connection_manager,
        gripper=_shared_gripper,
    )

    try:
        logger.info(f"Starting Flask server on {config.PC2_HOST}:{config.PC2_PORT}")
        app.run(
            host=config.PC2_HOST,
            port=config.PC2_PORT,
            debug=False,
            use_reloader=False,
            threaded=True,  # REFACTORED
        )
    except Exception as e:
        logger.error(f"Failed to start server: {e}")
        sys.exit(1)
