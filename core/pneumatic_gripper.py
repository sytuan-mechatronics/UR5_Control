"""
pneumatic_gripper.py - PC2 Flask Server
=======================================
Class PneumaticGripper: control pneumatic gripper via Arduino Uno R3.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Set, Dict, Any

import serial


logger = logging.getLogger(__name__)

# Protocol constants (must match Arduino firmware)
_CMD_GRIP = b"1"
_CMD_RELEASE = b"0"
_CMD_STATUS = b"?"
_CMD_KEEPALIVE = b"K"

_RESP_GRIP_OK = "GRIP_OK"
_RESP_GRIP_ALREADY = "GRIP_ALREADY"
_RESP_RELEASE_OK = "RELEASE_OK"
_RESP_RELEASE_ALREADY = "RELEASE_ALREADY"

_GRIP_SUCCESS_RESPONSES = {_RESP_GRIP_OK, _RESP_GRIP_ALREADY}
_RELEASE_SUCCESS_RESPONSES = {_RESP_RELEASE_OK, _RESP_RELEASE_ALREADY}


class GripperError(Exception):
    """Raised for all pneumatic gripper errors."""


class PneumaticGripper:
    """Thread-safe serial client for Arduino-controlled pneumatic gripper."""

    def __init__(
        self,
        port: str = "/dev/gripper",
        baud: int = 9600,
        cmd_timeout_s: float = 3.0,
        grip_settle_s: float = 0.5,
        release_settle_s: float = 0.3,
        heartbeat_interval_s: float = 3.0,
    ) -> None:
        self._port = port
        self._baud = baud
        self._cmd_timeout_s = cmd_timeout_s
        self._grip_settle_s = grip_settle_s
        self._release_settle_s = release_settle_s
        self._heartbeat_interval = heartbeat_interval_s

        self._ser = None  # type: Optional[serial.Serial]
        self._lock = threading.Lock()
        self._connected = False

        self._hb_thread = None  # type: Optional[threading.Thread]
        self._hb_stop = threading.Event()

        self._grip_state = None  # type: Optional[bool]

    def connect(self) -> None:
        """Open serial and start heartbeat thread."""
        if self._connected:
            logger.warning("PneumaticGripper already connected, skip connect()")
            return

        logger.info("PneumaticGripper opening serial %s @ %d", self._port, self._baud)
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=self._cmd_timeout_s,
                write_timeout=2.0,
            )
        except serial.SerialException as err:
            raise GripperError("Cannot open serial {}: {}".format(self._port, err)) from err

        # Arduino Uno auto-resets when serial opens.
        time.sleep(2.0)
        self._reset_buffer()

        self._connected = True

        self._hb_stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="gripper-heartbeat",
            daemon=True,
        )
        self._hb_thread.start()
        logger.info("PneumaticGripper connected")

    def disconnect(self) -> None:
        """Stop heartbeat, try open gripper, then close serial."""
        if not self._connected:
            return

        self._hb_stop.set()
        if self._hb_thread and self._hb_thread.is_alive():
            self._hb_thread.join(timeout=5.0)

        try:
            self.open()
        except GripperError:
            pass

        if self._ser and self._ser.is_open:
            self._ser.close()

        self._connected = False
        logger.info("PneumaticGripper disconnected")

    def close(self) -> Dict[str, Any]:
        """Close gripper."""
        result = self._send_command(_CMD_GRIP, _GRIP_SUCCESS_RESPONSES)
        if result["ok"]:
            self._grip_state = True
            time.sleep(self._grip_settle_s)
        result["state"] = self._grip_state
        return result

    def open(self) -> Dict[str, Any]:
        """Open gripper."""
        result = self._send_command(_CMD_RELEASE, _RELEASE_SUCCESS_RESPONSES)
        if result["ok"]:
            self._grip_state = False
            time.sleep(self._release_settle_s)
        result["state"] = self._grip_state
        return result

    def get_state(self) -> Dict[str, Any]:
        """Query state from Arduino without changing relay."""
        self._check_connected()
        with self._lock:
            self._flush_read_buffer()
            self._ser.write(_CMD_STATUS)
            response = self._read_line()

        gripping = response == "STATE:1"
        self._grip_state = gripping
        return {"ok": True, "gripping": gripping, "raw": response}

    def _heartbeat_loop(self) -> None:
        """Send keepalive periodically to prevent Arduino watchdog release."""
        while not self._hb_stop.is_set():
            self._hb_stop.wait(timeout=self._heartbeat_interval)
            if self._hb_stop.is_set():
                break

            if not self._connected or not self._ser or not self._ser.is_open:
                continue

            try:
                with self._lock:
                    self._ser.write(_CMD_KEEPALIVE)
                    try:
                        self._ser.readline()
                    except serial.SerialTimeoutException:
                        pass
            except serial.SerialException as err:
                logger.warning("PneumaticGripper heartbeat serial error: %s", err)

    def _send_command(self, cmd: bytes, success_responses: Set[str]) -> Dict[str, Any]:
        """Write command and read a single response line, thread-safe."""
        self._check_connected()

        with self._lock:
            self._flush_read_buffer()
            try:
                self._ser.write(cmd)
            except serial.SerialException as err:
                raise GripperError("Serial write failed: {}".format(err)) from err

            response = self._read_line()

        ok = response in success_responses
        if not ok:
            logger.error("PneumaticGripper command %r unexpected response: %s", cmd, response)

        return {"ok": ok, "response": response, "state": self._grip_state}

    def _read_line(self) -> str:
        """Read one line with serial timeout and raise explicit GripperError."""
        try:
            raw = self._ser.readline()
        except serial.SerialException as err:
            raise GripperError("Serial read failed (USB unplugged?): {}".format(err)) from err

        if not raw:
            raise GripperError(
                "Arduino no response after {}s timeout. Check USB/firmware.".format(self._cmd_timeout_s)
            )

        return raw.decode("ascii", errors="replace").strip()

    def _flush_read_buffer(self) -> None:
        """Drop buffered bytes before command to avoid stale responses."""
        if self._ser and self._ser.in_waiting:
            _ = self._ser.read(self._ser.in_waiting)

    def _reset_buffer(self) -> None:
        """Drain boot message after Arduino reset."""
        time.sleep(0.1)
        if self._ser and self._ser.in_waiting:
            _ = self._ser.read(self._ser.in_waiting)

    def _check_connected(self) -> None:
        if not self._connected or not self._ser or not self._ser.is_open:
            raise GripperError("PneumaticGripper is not connected. Call connect() first.")

    def __enter__(self) -> "PneumaticGripper":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._connected and bool(self._ser) and self._ser.is_open

    @property
    def grip_state(self) -> Optional[bool]:
        return self._grip_state
