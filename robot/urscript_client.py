"""
URScript Client for UR5 (port 30002).
Sends motion commands and gripper commands via URScript.
"""

import socket
import logging
from typing import List, Optional


logger = logging.getLogger(__name__)


class URScriptClient:
    """TCP client for URScript commands (port 30002)."""

    def __init__(self, ip: str, port: int = 30002, timeout: float = 10.0):
        """
        Initialize URScript Client.
        
        Args:
            ip: Robot IP address
            port: URScript port (default 30002)
            timeout: Socket timeout in seconds
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None

    def connect(self) -> None:
        """Connect to URScript Server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            logger.info(f"Connected to URScript at {self.ip}:{self.port}")
        except Exception as e:
            logger.error(f"Failed to connect to URScript: {e}")
            raise

    def disconnect(self) -> None:
        """Disconnect from URScript Server."""
        if self.socket:
            try:
                self.socket.close()
                logger.info("Disconnected from URScript")
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

    def send_raw(self, script_text: str) -> None:
        """
        Send raw URScript command.
        
        Args:
            script_text: URScript command string
            
        Raises:
            RuntimeError: If not connected
        """
        if not self.socket:
            raise RuntimeError("Not connected to URScript Server")

        try:
            # Ensure command ends with newline
            if not script_text.endswith("\n"):
                script_text += "\n"

            self.socket.sendall(script_text.encode("utf-8"))
            logger.debug(f"Sent URScript: {script_text.strip()}")

        except Exception as e:
            logger.error(f"Error sending URScript: {e}")
            raise

    def send_once(self, script_text: str) -> None:
        """
        Send a complete URScript payload over a short-lived socket.

        For CB3 controllers, motion programs are often more reliable when the TCP
        connection is opened, the full script is sent, and the socket is closed
        immediately after delivery.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.ip, self.port))
            try:
                if not script_text.endswith("\n"):
                    script_text += "\n"
                sock.sendall(script_text.encode("utf-8"))
                logger.debug("Sent one-shot URScript program")
            finally:
                sock.close()
        except Exception as e:
            logger.error(f"Error sending one-shot URScript: {e}")
            raise

    def send_program(
        self,
        body_lines: List[str],
        program_name: str = "external_cmd",
        one_shot: bool = True,
    ) -> None:
        """
        Send a complete URScript program block.

        CB-series controllers are more reliable when motion commands are sent as a
        full program (`def ... end`) instead of a single standalone line.

        Args:
            body_lines: URScript statements without indentation/newlines.
            program_name: Program function name.
            one_shot: Send over a fresh socket and close immediately after send.
        """
        if not body_lines:
            raise ValueError("body_lines must not be empty")

        script_lines = [f"def {program_name}():"]
        for line in body_lines:
            line = line.rstrip()
            if line:
                script_lines.append(f"  {line}")
        script_lines.append("end")
        script_text = "\n".join(script_lines) + "\n"
        if one_shot:
            self.send_once(script_text)
        else:
            self.send_raw(script_text)

    def move_joint(
        self,
        joints: List[float],
        accel: float = 1.0,
        vel: float = 0.8
    ) -> None:
        """
        Move to joint position using movej.
        
        Args:
            joints: [j1, j2, j3, j4, j5, j6] in radians
            accel: Acceleration in rad/s²
            vel: Velocity in rad/s
        """
        # Format: movej([j1,j2,j3,j4,j5,j6], a=accel, v=vel)
        joints_str = ",".join(f"{j:.6f}" for j in joints)
        logger.info(f"movej to joints: {joints}")
        self.send_program(
            [f"movej([{joints_str}], a={accel}, v={vel})"],
            program_name="external_movej",
            one_shot=True,
        )

    def move_linear(
        self,
        pose: List[float],
        accel: float = 0.3,
        vel: float = 0.1
    ) -> None:
        """
        Move to Cartesian pose using movel.
        
        Args:
            pose: [x, y, z, rx, ry, rz] in meters and radians
            accel: Acceleration in m/s²
            vel: Velocity in m/s
        """
        # Format: movel(p[x,y,z,rx,ry,rz], a=accel, v=vel)
        pose_str = ",".join(f"{p:.6f}" for p in pose)
        logger.info(f"movel to pose: {pose}")
        self.send_program(
            [f"movel(p[{pose_str}], a={accel}, v={vel})"],
            program_name="external_movel",
            one_shot=True,
        )

    def move_joint_to_pose_ik(
        self,
        pose: List[float],
        accel: float = 0.5,
        vel: float = 0.3,
    ) -> None:
        """
        Move to Cartesian pose using movej(get_inverse_kin(...)).

        This is useful when a straight-line `movel` path is rejected but the
        target pose itself may still be reachable through joint-space motion.

        Args:
            pose: [x, y, z, rx, ry, rz] in meters and radians
            accel: Joint acceleration-like parameter for movej
            vel: Joint velocity-like parameter for movej
        """
        pose_str = ",".join(f"{p:.6f}" for p in pose)
        logger.info(f"movej(get_inverse_kin) to pose: {pose}")
        self.send_program(
            [f"movej(get_inverse_kin(p[{pose_str}]), a={accel}, v={vel})"],
            program_name="external_movej_ik",
            one_shot=True,
        )

    def move_linear_offset(
        self,
        current_pose: List[float],
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        accel: float = 0.3,
        vel: float = 0.1
    ) -> None:
        """
        Move with offset from current pose.
        
        Args:
            current_pose: Current [x, y, z, rx, ry, rz]
            dx, dy, dz: Offset in meters
            accel: Acceleration in m/s²
            vel: Velocity in m/s
        """
        # Calculate new pose
        new_pose = [
            current_pose[0] + dx,
            current_pose[1] + dy,
            current_pose[2] + dz,
            current_pose[3],
            current_pose[4],
            current_pose[5],
        ]
        logger.info(f"Moving with offset: dx={dx}, dy={dy}, dz={dz}")
        self.move_linear(new_pose, accel=accel, vel=vel)

    def set_tcp(self, tcp_pose: List[float]) -> None:
        """
        Set TCP offset.
        
        Args:
            tcp_pose: [x, y, z, rx, ry, rz] offset from tool flange
        """
        pose_str = ",".join(f"{p:.6f}" for p in tcp_pose)
        logger.info(f"Setting TCP: {tcp_pose}")
        self.send_program(
            [f"set_tcp(p[{pose_str}])"],
            program_name="external_set_tcp",
            one_shot=True,
        )

    def set_payload(self, mass_kg: float, center_of_mass: List[float] = None) -> None:
        """
        Set payload mass and center of mass.
        
        Args:
            mass_kg: Payload mass in kg
            center_of_mass: [x, y, z] offset, if None use [0, 0, 0]
        """
        if center_of_mass is None:
            center_of_mass = [0.0, 0.0, 0.0]

        com_str = ",".join(f"{c:.6f}" for c in center_of_mass)
        logger.info(f"Setting payload: {mass_kg}kg")
        self.send_program(
            [f"set_payload({mass_kg}, [{com_str}])"],
            program_name="external_set_payload",
            one_shot=True,
        )
