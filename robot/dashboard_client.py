"""
Dashboard Client for UR5 (port 29999).
Handles safety checks, power on, brake release.
"""

import socket
import logging
from typing import Tuple, Dict, Optional


logger = logging.getLogger(__name__)


class DashboardClient:
    """TCP client for UR Dashboard Server (port 29999)."""

    def __init__(self, ip: str, port: int = 29999, timeout: float = 5.0):
        """
        Initialize Dashboard Client.
        
        Args:
            ip: Robot IP address
            port: Dashboard port (default 29999)
            timeout: Socket timeout in seconds
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None

    def connect(self) -> None:
        """Connect to Dashboard Server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            logger.info(f"Connected to Dashboard at {self.ip}:{self.port}")
        except Exception as e:
            logger.error(f"Failed to connect to Dashboard: {e}")
            raise

    def disconnect(self) -> None:
        """Disconnect from Dashboard Server."""
        if self.socket:
            try:
                self.socket.close()
                logger.info("Disconnected from Dashboard")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            finally:
                self.socket = None

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

    def send_command(self, cmd: str) -> str:
        """
        Send command to Dashboard and receive response.
        
        Args:
            cmd: Command string (e.g., "robotmode")
            
        Returns:
            Response string from robot
            
        Raises:
            RuntimeError: If socket is not connected
        """
        if not self.socket:
            raise RuntimeError("Not connected to Dashboard Server")

        try:
            # Ensure command ends with newline
            if not cmd.endswith("\n"):
                cmd += "\n"

            self.socket.sendall(cmd.encode("utf-8"))
            response = self.socket.recv(4096).decode("utf-8").strip()
            logger.debug(f"Command: {cmd.strip()} -> Response: {response}")
            return response
        except Exception as e:
            logger.error(f"Error sending command '{cmd.strip()}': {e}")
            raise

    def get_robotmode(self) -> str:
        """
        Get current robot mode.
        
        Returns:
            Robot mode string (POWER_OFF, POWER_ON, IDLE, RUNNING, etc.)
        """
        response = self.send_command("robotmode")
        # Response format: "robotmode: RUNNING" or similar
        if ":" in response:
            return response.split(":", 1)[1].strip()
        return response.strip()

    def get_safety_status(self) -> str:
        """
        Get current safety status.
        
        Returns:
            Safety status string (NORMAL, PROTECTIVE_STOP, etc.)
        """
        response = self.send_command("safetystatus")
        if ":" in response:
            return response.split(":", 1)[1].strip()
        return response.strip()

    def precheck_ready(self) -> Dict[str, str]:
        """
        Perform safety check before running.
        
        Returns:
            Dict with robotmode and safetystatus
            
        Raises:
            RuntimeError: If robot is not in safe state
        """
        robotmode = self.get_robotmode()
        safetystatus = self.get_safety_status()

        logger.info(f"Robot Mode: {robotmode}, Safety Status: {safetystatus}")

        # Check safety
        if "PROTECTIVE_STOP" in safetystatus or "EMERGENCY" in safetystatus:
            raise RuntimeError(f"Robot in unsafe state: {safetystatus}")

        result = {
            "robotmode": robotmode,
            "safetystatus": safetystatus,
            "program_state": self.get_program_state()[0],
        }

        return result

    def get_program_state(self) -> Tuple[str, str]:
        """
        Get current program state.
        
        Returns:
            Tuple of (parsed_state, raw_response)
            parsed_state: PLAYING|PAUSED|STOPPED|UNKNOWN
        """
        response = self.send_command("programState")
        raw = response
        
        if "PLAYING" in response:
            return "PLAYING", raw
        elif "PAUSED" in response:
            return "PAUSED", raw
        elif "STOPPED" in response:
            return "STOPPED", raw
        else:
            return "UNKNOWN", raw

    def power_on(self) -> str:
        """
        Power on the robot.
        
        Returns:
            Response from robot
        """
        logger.info("Powering on robot...")
        response = self.send_command("power on")
        logger.info(f"Power on response: {response}")
        return response

    def power_off(self) -> str:
        """
        Power off the robot.
        
        Returns:
            Response from robot
        """
        logger.info("Powering off robot...")
        response = self.send_command("power off")
        logger.info(f"Power off response: {response}")
        return response

    def brake_release(self) -> str:
        """
        Release robot brakes.
        
        Returns:
            Response from robot
        """
        logger.info("Releasing brakes...")
        response = self.send_command("brake release")
        logger.info(f"Brake release response: {response}")
        return response

    def prepare_to_run(self) -> Dict[str, str]:
        """
        Prepare robot for operation: power on and release brakes.
        
        Returns:
            Dict with power_on_reply and brake_release_reply
            
        Raises:
            RuntimeError: If any step fails
        """
        logger.info("Preparing robot to run...")

        try:
            power_reply = self.power_on()
            brake_reply = self.brake_release()

            result = {
                "power_on_reply": power_reply,
                "brake_release_reply": brake_reply,
            }

            logger.info("Robot preparation complete")
            return result

        except Exception as e:
            logger.error(f"Failed to prepare robot: {e}")
            raise
