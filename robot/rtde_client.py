"""
RTDE Client for UR5 (port 30004).
Reads real-time robot state: TCP pose, joint positions, speeds.
"""

import logging
import time
from typing import List, Optional, Tuple

try:
    import rtde_receive
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning("ur-rtde not installed. Install with: pip install ur-rtde")


logger = logging.getLogger(__name__)


class RTDEClient:
    """Real-Time Data Exchange (RTDE) client for UR5."""

    def __init__(self, ip: str, port: int = 30004, frequency: float = 10.0):
        """
        Initialize RTDE Client.
        
        Args:
            ip: Robot IP address
            port: RTDE port (default 30004)
            frequency: Communication frequency in Hz
        """
        if not RTDE_AVAILABLE:
            raise RuntimeError(
                "ur-rtde library not available. "
                "Install with: pip install ur-rtde"
            )

        self.ip = ip
        self.port = port
        self.frequency = frequency
        self.client = None

    def connect(self) -> None:
        """Connect to RTDE server."""
        try:
            self.client = rtde_receive.RTDEReceiveInterface(
                self.ip,
                frequency=self.frequency
            )
            logger.info(f"Connected to RTDE at {self.ip}:{self.port}")
        except Exception as e:
            logger.error(f"Failed to connect to RTDE: {e}")
            raise

    def disconnect(self) -> None:
        """Disconnect from RTDE server."""
        if self.client:
            try:
                self.client.disconnect()
                logger.info("Disconnected from RTDE")
            except Exception as e:
                logger.error(f"Error disconnecting from RTDE: {e}")
            finally:
                self.client = None

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

    def _check_connected(self) -> None:
        """Raise error if not connected."""
        if not self.client:
            raise RuntimeError("Not connected to RTDE server")

    def get_tcp_pose(self) -> List[float]:
        """
        Get current TCP pose.
        
        Returns:
            [x, y, z, rx, ry, rz] in meters and radians
        """
        self._check_connected()
        try:
            pose = self.client.getActualTCPPose()
            logger.debug(f"TCP Pose: {pose}")
            return list(pose)
        except Exception as e:
            logger.error(f"Error reading TCP pose: {e}")
            raise

    def get_tcp_pose_with_timestamp(self) -> Tuple[List[float], float]:
        """
        Đọc TCP pose và ghi lại thời điểm đọc (wall clock).

        Dùng kết hợp với FemtoCamera.get_frames_with_timestamp() để
        kiểm tra frame/pose không lệch nhau quá 100ms.

        Returns:
            Tuple (pose, timestamp_s):
              pose        — [x, y, z, rx, ry, rz] (m, rad)
              timestamp_s — time.time() ngay sau khi đọc RTDE
        """
        pose = self.get_tcp_pose()
        # Ghi timestamp NGAY SAU khi đọc xong RTDE buffer
        timestamp_s = time.time()
        return pose, timestamp_s

    def get_joint_positions(self) -> List[float]:
        """
        Get current joint positions.
        
        Returns:
            [j1, j2, j3, j4, j5, j6] in radians
        """
        self._check_connected()
        try:
            joints = self.client.getActualQ()
            logger.debug(f"Joint Positions: {joints}")
            return list(joints)
        except Exception as e:
            logger.error(f"Error reading joint positions: {e}")
            raise

    def get_joint_speeds(self) -> List[float]:
        """
        Get current joint velocities.
        
        Returns:
            [dj1, dj2, dj3, dj4, dj5, dj6] in rad/s
        """
        self._check_connected()
        try:
            speeds = self.client.getActualQd()
            logger.debug(f"Joint Speeds: {speeds}")
            return list(speeds)
        except Exception as e:
            logger.error(f"Error reading joint speeds: {e}")
            raise

    def get_tool_analog_input(self, channel: int = 0) -> float:
        """
        Get tool analog input value (voltage, 0-10V).
        Used for OnRobot RG width feedback via analog output.

        Args:
            channel: Analog input channel (0 or 1)

        Returns:
            Voltage value (0.0 - 10.0 V)
        """
        self._check_connected()
        try:
            if channel == 0:
                return float(self.client.getActualToolAnalogInput0())
            else:
                return float(self.client.getActualToolAnalogInput1())
        except Exception as e:
            logger.error(f"Error reading tool analog input {channel}: {e}")
            return 0.0

    def get_digital_input(self, index: int) -> bool:
        """
        Get digital input state.
        Used for OnRobot RG grip-detected signal via digital output.

        Args:
            index: Digital input index (0-7 tool inputs via bit mask)

        Returns:
            True if input is HIGH
        """
        self._check_connected()
        try:
            # getActualDigitalInputBits returns bitmask
            bits = self.client.getActualDigitalInputBits()
            return bool(bits & (1 << index))
        except Exception as e:
            logger.error(f"Error reading digital input {index}: {e}")
            return False

    def is_steady(self, threshold: float = 0.001) -> bool:
        """
        Check if robot is steady (all joints below speed threshold).
        
        Args:
            threshold: Speed threshold in rad/s
            
        Returns:
            True if all joints are steady, False otherwise
        """
        try:
            speeds = self.get_joint_speeds()
            is_steady = all(abs(speed) < threshold for speed in speeds)
            if not is_steady:
                logger.debug(f"Robot not steady. Max speed: {max(abs(s) for s in speeds):.6f} rad/s")
            return is_steady
        except Exception as e:
            logger.error(f"Error checking if steady: {e}")
            return False

    def wait_steady(
        self,
        timeout_s: float = 30.0,
        poll_interval: float = 0.05,
        threshold: float = 0.001,
        motion_start_timeout: float = 2.0,
        motion_start_threshold: float = 0.005
    ) -> bool:
        """
        Block until robot is steady or timeout.

        Gồm 2 giai đoạn:
          1. Đợi robot BẮT ĐẦU di chuyển (tránh false-positive ngay sau gửi lệnh)
          2. Đợi robot DỪNG HẲN (all joint_speed < threshold)

        Lý do cần giai đoạn 1: UR5 CB-series có độ trễ 0.1-0.5s giữa lúc
        gửi movej/movel và lúc joint_speed tăng lên > 0. Nếu poll ngay,
        is_steady() trả True giả và bước tiếp theo chạy sai timing.

        Args:
            timeout_s: Thời gian tối đa đợi robot dừng (sau khi đã bắt đầu)
            poll_interval: Khoảng thời gian giữa mỗi lần poll (s)
            threshold: Ngưỡng tốc độ "đứng yên" (rad/s)
            motion_start_timeout: Thời gian tối đa chờ robot bắt đầu move (s)
            motion_start_threshold: Ngưỡng tốc độ "đã bắt đầu" (rad/s)

        Returns:
            True nếu robot đã steady, False nếu timeout
        """
        # ── Giai đoạn 1: Đợi robot bắt đầu di chuyển ──────────────────────
        logger.debug(
            f"Waiting for motion to start (timeout={motion_start_timeout}s, "
            f"threshold={motion_start_threshold} rad/s)..."
        )
        start = time.time()
        motion_started = False
        while time.time() - start < motion_start_timeout:
            try:
                speeds = self.get_joint_speeds()
                if any(abs(s) > motion_start_threshold for s in speeds):
                    motion_started = True
                    logger.debug(
                        f"Motion started (max speed: "
                        f"{max(abs(s) for s in speeds):.4f} rad/s)"
                    )
                    break
            except Exception:
                pass
            time.sleep(poll_interval)

        if not motion_started:
            # Pose đã đúng (robot không cần di chuyển) HOẶC lệnh chưa được
            # xử lý — trong cả hai trường hợp kiểm tra ngay is_steady().
            logger.debug(
                "Motion did not start within timeout — "
                "robot may already be at target pose"
            )

        # ── Giai đoạn 2: Đợi robot dừng hẳn ────────────────────────────────
        logger.info(f"Waiting for robot to be steady (timeout={timeout_s}s)...")
        start_time = time.time()

        while time.time() - start_time < timeout_s:
            if self.is_steady(threshold=threshold):
                elapsed = time.time() - start_time
                logger.info(f"Robot is steady (waited {elapsed:.2f}s)")
                return True
            time.sleep(poll_interval)

        elapsed = time.time() - start_time
        logger.warning(f"Timeout waiting for robot to be steady ({elapsed:.2f}s)")
        return False
