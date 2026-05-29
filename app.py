"""
PC2 Flask Application - UR5 Robot Control Server.
Receives commands from PC1 via REST API and orchestrates pick-place operations.
"""

import logging
import logging.handlers
import sys
from pathlib import Path

from flask import Flask, request
from api.ur5_bp import ur5_bp, set_job_store
from core.job_store import JobStore
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

    logger.info("=" * 70)
    logger.info("PC2 UR5 Robot Control Server")
    logger.info("=" * 70)
    logger.info(f"Robot IP: {config.ROBOT_IP}")
    logger.info(f"PC2 Server: {config.PC2_HOST}:{config.PC2_PORT}")
    logger.info(f"PC1 Callback: {config.PC1_UR5_DONE_URL if config.PC1_CALLBACK_ENABLED else 'Disabled'}")
    logger.info("=" * 70)

    app = create_app()

    try:
        logger.info(f"Starting Flask server on {config.PC2_HOST}:{config.PC2_PORT}")
        app.run(
            host=config.PC2_HOST,
            port=config.PC2_PORT,
            debug=False,
            use_reloader=False
        )
    except Exception as e:
        logger.error(f"Failed to start server: {e}")
        sys.exit(1)
