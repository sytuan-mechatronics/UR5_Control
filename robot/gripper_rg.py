"""

OnRobot RG Gripper Control via URScript.
Handles opening, closing, and grip detection.

── NO_GRIPPER MODE / ENABLE FLAG ────────────────────────────────
GRIPPER_ENABLED = True   # False → bỏ qua gripper, chạy motion only
# Đọc từ env: GRIPPER_ENABLED=false

── URCap Version & Syntax ─────────────────────────────────────────────────────
File này dùng cú pháp OnRobot URCap v5.16.0:

    rg_grip(tool=0, force=<N>, width=<mm>)

    Tham số:
        tool              — index gripper (0 = single, 0/1 = dual setup)
        force             — lực gắp (N), RG2: 3–40N, RG6: 3–120N
        width             — độ mở target (mm), RG2: 0–110mm, RG6: 0–160mm

── Grip Detection ─────────────────────────────────────────────────────────────
Port 30002 là fire-and-forget, robot không trả response về PC2.
Ba phương pháp phát hiện grip (set GRIPPER_GRIP_DETECT_METHOD trong config):

  "timeout"        — đơn giản nhất, đợi cố định rồi coi OK
    "width_feedback" — đọc rg_get_status(tool=0) qua RTDE output register
                                         0=idle, 1=gripping, 2=no_object, 3=object_lost
  "digital_output" — đọc digital input UR5 được nối từ grip-detected signal
                     của RG (cần cấu hình wiring thêm)
"""

import logging
import time
from typing import Dict, Optional, TYPE_CHECKING

from .urscript_client import URScriptClient
import config

if TYPE_CHECKING:
    from .rtde_client import RTDEClient


logger = logging.getLogger(__name__)


class GripperRG:
    """OnRobot RG2/RG6 gripper controller — URCap v5.16.0 syntax."""

    RG_STATUS_REGISTER = 24
    RG_WIDTH_REGISTER = 25

    def __init__(self, urscript_client: URScriptClient, tool_index: int = 0, enabled: bool = True):
        """
        Initialize gripper controller.

        Args:
            urscript_client: URScriptClient instance (dùng chung, không tạo mới)
            tool_index: Gripper tool index (0 = single gripper setup)
            enabled: True nếu gripper hoạt động, False → bỏ qua toàn bộ lệnh gripper
        """
        self.urscript_client = urscript_client
        self.tool_index = tool_index
        self.enabled = enabled and not config.IS_SIMULATION
        self.simulation_mode = config.IS_SIMULATION or not self.enabled

    def _result(self, action: str, ok: bool, **extra) -> Dict:
        """Build a normalized response payload for logging/UI observers."""
        payload = {
            "ok": ok,
            "action": action,
            "simulated": self.simulation_mode,
            "mode": "simulation" if self.simulation_mode else "hardware",
            "tool": self.tool_index,
            "timestamp": time.time(),
        }
        payload.update(extra)
        return payload

    def _simulate_delay(self, delay_s: float) -> float:
        """Sleep in simulation mode and return elapsed seconds."""
        start = time.time()
        time.sleep(max(delay_s, 0.0))
        return time.time() - start

    def open(self, width_mm: int = None) -> Dict:
        """
        Open gripper.

        Args:
            width_mm: Target opening width in mm.
                      None → dùng GRIPPER_OPEN_WIDTH từ config.
        """
        if width_mm is None:
            width_mm = config.GRIPPER_OPEN_WIDTH

        if self.simulation_mode:
            elapsed = self._simulate_delay(config.GRIPPER_SIM_OPEN_DELAY_S)
            logger.info("Gripper simulated open to %smm (%.3fs)", width_mm, elapsed)
            return self._result(
                "open",
                True,
                target_width_mm=width_mm,
                elapsed_s=round(elapsed, 3),
                message="Simulated gripper open completed",
            )

        # Dùng force thấp khi mở để không bị shock
        script = f"rg_grip(tool={self.tool_index}, force=5, width={width_mm})"
        logger.info(f"Opening gripper to {width_mm}mm")
        self.urscript_client.send_raw(script)
        return self._result(
            "open",
            True,
            target_width_mm=width_mm,
            message="Hardware gripper open command sent",
        )

    def close(self, force_n: int = None, width_mm: int = None) -> Dict:
        """
        Close gripper with specified force.

        Args:
            force_n: Grip force in Newtons. None → dùng GRIPPER_CLOSE_FORCE.
            width_mm: Target width in mm. None → dùng GRIPPER_CLOSE_WIDTH.
        """
        if force_n is None:
            force_n = config.GRIPPER_CLOSE_FORCE
        if width_mm is None:
            width_mm = config.GRIPPER_CLOSE_WIDTH

        if self.simulation_mode:
            elapsed = self._simulate_delay(config.GRIPPER_SIM_CLOSE_DELAY_S)
            logger.info(
                "Gripper simulated close force=%sN width=%smm (%.3fs)",
                force_n,
                width_mm,
                elapsed,
            )
            return self._result(
                "close",
                True,
                force_n=force_n,
                target_width_mm=width_mm,
                elapsed_s=round(elapsed, 3),
                message="Simulated gripper close completed",
            )

        script = f"rg_grip(tool={self.tool_index}, force={force_n}, width={width_mm})"
        logger.info(f"Closing gripper with force={force_n}N, width={width_mm}mm")
        self.urscript_client.send_raw(script)
        return self._result(
            "close",
            True,
            force_n=force_n,
            target_width_mm=width_mm,
            message="Hardware gripper close command sent",
        )

    def get_actual_width(self, rtde_client: Optional["RTDEClient"] = None) -> Optional[float]:
        """
        Read current gripper width using rg_get_width(tool=<idx>).

        The value is written into an RTDE output integer register by a short
        URScript probe and then read back from RTDE.

        Args:
            rtde_client: RTDEClient instance, required to read output register.

        Returns:
            Width in mm if available, otherwise None.
        """
        if self.simulation_mode:
            logger.warning("Gripper disabled — get_actual_width returns None")
            return None
        if rtde_client is None:
            logger.warning("get_actual_width requires rtde_client")
            return None

        script = (
            f"def rg_probe_width():\n"
            f"  width_mm = rg_get_width(tool={self.tool_index})\n"
            f"  write_output_integer_register({self.RG_WIDTH_REGISTER}, round(width_mm))\n"
            f"end\n"
            f"rg_probe_width()"
        )

        try:
            self.urscript_client.send_raw(script)
            time.sleep(0.03)
            width_mm = self._read_output_int_register(rtde_client, self.RG_WIDTH_REGISTER)
            if width_mm is None:
                return None
            return float(width_mm)
        except Exception as e:
            logger.warning(f"Error reading rg_get_width: {e}")
            return None

    def wait_grip_detected(
        self,
        rtde_client: Optional["RTDEClient"] = None,
        timeout_s: float = None,
        poll_interval: float = 0.05
    ) -> bool:
        """
        Đợi cho đến khi grip được detected hoặc timeout.

        Phương pháp được chọn qua config.GRIPPER_GRIP_DETECT_METHOD:

          "timeout"        — Đợi cố định 0.5s sau lệnh close, luôn trả True.
                             Dùng khi chưa có feedback hardware.

          "width_feedback" — Đọc actual width qua RTDE tool analog input.
                             OnRobot RG nối analog output (0-10V) → tool
                             analog input của UR5.
                             Grip OK nếu:
                               phoi_diameter - tolerance
                               ≤ actual_width
                               ≤ phoi_diameter + tolerance
                             Miss nếu actual_width ≈ 0 (đóng hoàn toàn).

          "digital_output" — Đọc digital input UR5 từ grip-detected signal
                             của RG (cần wiring riêng).

        Args:
            rtde_client: RTDEClient instance (bắt buộc với width_feedback
                         và digital_output, bỏ qua với timeout).
            timeout_s: Thời gian timeout (s). None → config.GRIPPER_TIMEOUT_S.
            poll_interval: Khoảng poll (s).

        Returns:
            True nếu grip detected, False nếu timeout hoặc miss detected.
        """
        if self.simulation_mode:
            elapsed = self._simulate_delay(config.GRIPPER_SIM_DETECT_DELAY_S)
            logger.info(
                "Gripper simulated grip-detected after %.3fs",
                elapsed,
            )
            return True
        if timeout_s is None:
            timeout_s = config.GRIPPER_TIMEOUT_S

        method = config.GRIPPER_GRIP_DETECT_METHOD
        logger.info(
            f"Waiting for grip detection (method={method}, timeout={timeout_s}s)"
        )

        if method == "width_feedback":
            return self._wait_grip_width_feedback(
                rtde_client, timeout_s, poll_interval
            )
        elif method == "digital_output":
            return self._wait_grip_digital_output(
                rtde_client, timeout_s, poll_interval
            )
        else:
            # "timeout" — fallback mặc định
            return self._wait_grip_timeout(timeout_s=0.5)

    # ── Private methods ──────────────────────────────────────────────────────

    def _wait_grip_timeout(self, timeout_s: float = 0.5) -> bool:
        """
        Phương pháp đơn giản: đợi cố định rồi coi là OK.
        Dùng khi chưa có feedback hardware.
        """
        time.sleep(timeout_s)
        logger.info(f"Grip assumed detected (timeout method, waited {timeout_s}s)")
        return True

    def _wait_grip_width_feedback(
        self,
        rtde_client,
        timeout_s: float,
        poll_interval: float
    ) -> bool:
        """
        Phương pháp width_feedback:
          - Gọi rg_get_status(tool=<idx>) qua URScript probe
          - Đọc status từ RTDE output integer register
          - 1 = gripping (có vật)      -> detected
          - 2 = no_object (miss)       -> fail
          - 3 = object_lost            -> fail
          - 0 = idle                   -> continue polling
        """
        if rtde_client is None:
            logger.error(
                "width_feedback method requires rtde_client. "
                "Falling back to timeout method."
            )
            return self._wait_grip_timeout()

        logger.info("Width feedback via rg_get_status(tool=0)")

        start = time.time()
        while time.time() - start < timeout_s:
            try:
                status = self._read_rg_status(rtde_client)
                if status is None:
                    time.sleep(poll_interval)
                    continue

                logger.debug(f"rg_get_status={status}")

                if status == 1:
                    actual_width = self.get_actual_width(rtde_client)
                    if actual_width is not None:
                        logger.info(
                            f"Grip detected via rg_get_status (status=1, width={actual_width:.1f}mm)"
                        )
                    else:
                        logger.info("Grip detected via rg_get_status (status=1)")
                    return True

                if status == 2:
                    logger.warning("Grip MISS via rg_get_status (status=2: no_object)")
                    return False

                if status == 3:
                    logger.warning("Grip FAIL via rg_get_status (status=3: object_lost)")
                    return False

            except Exception as e:
                logger.warning(f"Error reading width feedback: {e}")

            time.sleep(poll_interval)

        logger.warning(
            f"Grip detection timeout ({timeout_s}s) via width_feedback"
        )
        return False

    def _read_rg_status(self, rtde_client) -> Optional[int]:
        """Probe rg_get_status(tool=<idx>) and read it from RTDE output register."""
        script = (
            f"def rg_probe_status():\n"
            f"  write_output_integer_register({self.RG_STATUS_REGISTER}, rg_get_status(tool={self.tool_index}))\n"
            f"end\n"
            f"rg_probe_status()"
        )
        self.urscript_client.send_raw(script)
        time.sleep(0.03)
        return self._read_output_int_register(rtde_client, self.RG_STATUS_REGISTER)

    @staticmethod
    def _read_output_int_register(rtde_client, register_index: int) -> Optional[int]:
        """Read RTDE output int register with compatibility fallback names."""
        try:
            if hasattr(rtde_client, "client") and rtde_client.client is not None:
                reader = getattr(rtde_client.client, "getOutputIntRegister", None)
                if callable(reader):
                    return int(reader(register_index))
            reader = getattr(rtde_client, "get_output_int_register", None)
            if callable(reader):
                return int(reader(register_index))
        except Exception as e:
            logger.warning(f"Error reading RTDE output int register {register_index}: {e}")
        return None

    def _wait_grip_digital_output(
        self,
        rtde_client,
        timeout_s: float,
        poll_interval: float
    ) -> bool:
        """
        Phương pháp digital_output:
          - Đọc digital input pin GRIPPER_DIGITAL_OUTPUT_PIN từ RTDE
          - Pin HIGH = grip detected (theo wiring của RG signal output)
        """
        if rtde_client is None:
            logger.error(
                "digital_output method requires rtde_client. "
                "Falling back to timeout method."
            )
            return self._wait_grip_timeout()

        pin = config.GRIPPER_DIGITAL_OUTPUT_PIN
        logger.info(f"Digital output: polling pin {pin}")

        start = time.time()
        while time.time() - start < timeout_s:
            try:
                if rtde_client.get_digital_input(pin):
                    elapsed = time.time() - start
                    logger.info(
                        f"Grip detected via digital input pin {pin} "
                        f"(after {elapsed:.2f}s)"
                    )
                    return True
            except Exception as e:
                logger.warning(f"Error reading digital input: {e}")

            time.sleep(poll_interval)

        logger.warning(
            f"Grip detection timeout ({timeout_s}s) via digital_output pin {pin}"
        )
        return False

    def stop(self) -> None:
        """Stop gripper motion immediately (send zero force)."""
        if self.simulation_mode:
            logger.info("Gripper stop in simulation mode")
            return
        script = f"rg_grip(tool={self.tool_index}, force=0, width=0)"
        logger.info("Stopping gripper")
        self.urscript_client.send_raw(script)

    def is_enabled(self) -> bool:
        """Return True if gripper is enabled."""
        return not self.simulation_mode

    @classmethod
    def try_create(cls, urscript_client, tool_index=0) -> "GripperRG":
        """
        Thử tạo gripper instance.
        Nếu config.GRIPPER_ENABLED = False → tạo instance disabled ngay.
        Nếu True → tạo bình thường.
        Không raise exception ở đây, lỗi thực sự sẽ xảy ra lúc open()/close().
        """
        enabled = not getattr(config, "IS_SIMULATION", False)
        return cls(urscript_client, tool_index=tool_index, enabled=enabled)
