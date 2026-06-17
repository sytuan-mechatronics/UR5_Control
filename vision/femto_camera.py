"""
Orbbec Femto Mega Camera Interface.
Captures aligned RGB and Depth frames.

Installation:
  pip install pyorbbecsdk

If pyorbbecsdk is not available, ensure:
  1. Orbbec SDK is installed on the system
  2. Python bindings are in PYTHONPATH
"""

import logging
import time
import numpy as np
from typing import Tuple, Optional

import config

try:
    import ob
    ORBBEC_AVAILABLE = True
except ImportError:
    try:
        import pyorbbecsdk as ob
        ORBBEC_AVAILABLE = True
    except ImportError:
        ORBBEC_AVAILABLE = False


logger = logging.getLogger(__name__)


def _first_attr(obj, names):
    """Return first existing attribute name from candidates."""
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def _call_any(obj, method_names, *args, **kwargs):
    """Call first available method from candidates."""
    method = _first_attr(obj, method_names)
    if method is None:
        raise AttributeError(f"None of methods exist: {method_names}")
    return method(*args, **kwargs)


def _enum_value(module_obj, enum_names, member_names):
    """Resolve enum member across SDK naming variants."""
    enum_cls = _first_attr(module_obj, enum_names)
    if enum_cls is None:
        return None
    for member in member_names:
        if hasattr(enum_cls, member):
            return getattr(enum_cls, member)
    return None


class FemtoCamera:
    """Orbbec Femto Mega camera interface."""

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        transport: Optional[str] = None,
        ip: Optional[str] = None,
        net_port: Optional[int] = None,
    ):
        """
        Initialize camera.
        
        Args:
            width: Image width in pixels (default 1280)
            height: Image height in pixels (default 720)
            transport: auto|usb|lan
            ip: Camera IP when using LAN
            net_port: Orbbec network port
        """
        if not ORBBEC_AVAILABLE:
            raise RuntimeError(
                "pyorbbecsdk not available. "
                "Install Orbbec SDK and ensure Python bindings are available."
            )

        self.width = width
        self.height = height
        self.transport = (transport or config.CAMERA_TRANSPORT or "auto").strip().lower()
        if self.transport not in ("auto", "usb", "lan"):
            self.transport = "auto"
        self.ip = (ip if ip is not None else config.CAMERA_IP).strip()
        self.net_port = int(net_port if net_port is not None else config.CAMERA_NET_PORT)
        self.pipeline: Optional[ob.Pipeline] = None
        self.config: Optional[ob.Config] = None
        self.device_info = {}

    def _query_devices(self, ctx):
        """Enumerate both USB and LAN devices when SDK supports it."""
        try:
            _call_any(ctx, ["enable_net_device_enumeration"], True)
            time.sleep(0.5)
        except Exception:
            pass
        return _call_any(ctx, ["query_devices", "queryDevices"])

    def _get_device_ip(self, devices, idx: int) -> str:
        try:
            return str(_call_any(devices, ["get_device_ip_address_by_index"], idx))
        except Exception:
            return ""

    def _get_device_conn(self, devices, idx: int) -> str:
        try:
            return str(_call_any(devices, ["get_device_connection_type_by_index"], idx))
        except Exception:
            return "unknown"

    def _select_device(self, ctx):
        """Select device according to configured transport/IP."""
        devices = self._query_devices(ctx)
        count = devices.get_count()

        if count == 0 and self.transport != "lan":
            raise RuntimeError("Không phát hiện thiết bị Orbbec nào")

        selected_device = None
        selected_meta = None

        for idx in range(count):
            device = devices.get_device_by_index(idx)
            info = device.get_device_info()
            dev_ip = self._get_device_ip(devices, idx)
            conn = self._get_device_conn(devices, idx)
            is_lan = bool(dev_ip)

            if self.transport == "usb" and is_lan:
                continue
            if self.transport == "lan" and not is_lan:
                continue
            if self.ip and dev_ip and dev_ip != self.ip:
                continue

            selected_device = device
            selected_meta = {
                "name": info.get_name(),
                "serial": info.get_serial_number(),
                "firmware": info.get_firmware_version(),
                "ip": dev_ip,
                "connection_type": conn,
            }
            break

        if selected_device is None and self.transport == "lan" and self.ip:
            try:
                selected_device = _call_any(
                    ctx,
                    ["create_net_device"],
                    self.ip,
                    self.net_port,
                )
                selected_meta = {
                    "name": "Orbbec network device",
                    "serial": "",
                    "firmware": "",
                    "ip": self.ip,
                    "connection_type": "LAN",
                }
            except Exception as exc:
                raise RuntimeError(
                    f"Không mở được camera LAN tại {self.ip}:{self.net_port}: {exc}"
                ) from exc

        if selected_device is None:
            suffix = f", ip={self.ip}" if self.ip else ""
            raise RuntimeError(
                f"Không tìm thấy camera theo transport={self.transport}{suffix}"
            )

        return selected_device, selected_meta

    def connect(self) -> None:
        """Connect and start camera."""
        try:
            logger.info("Initializing Orbbec Femto Mega camera...")

            ctx = ob.Context()
            device, self.device_info = self._select_device(ctx)

            logger.info(
                "Selected camera transport=%s ip=%s conn=%s name=%s serial=%s",
                self.transport,
                self.device_info.get("ip", ""),
                self.device_info.get("connection_type", ""),
                self.device_info.get("name", ""),
                self.device_info.get("serial", ""),
            )

            # Create pipeline
            self.pipeline = ob.Pipeline(device)

            # Create config
            self.config = ob.Config()

            # Configure color stream
            color_profile = None
            try:
                color_sensor = _enum_value(
                    ob,
                    ["SensorType", "OBSensorType"],
                    ["COLOR", "COLOR_SENSOR"],
                )
                if color_sensor is None:
                    raise RuntimeError("Color sensor enum not found")

                profiles = _call_any(
                    self.pipeline,
                    ["get_stream_profile_list", "getStreamProfileList"],
                    color_sensor,
                )

                # New SDK style: profile_list object with getters.
                if hasattr(profiles, "get_video_stream_profile"):
                    rgb_fmt = _enum_value(ob, ["Format", "OBFormat"], ["RGB", "RGB888"])
                    try:
                        color_profile = profiles.get_video_stream_profile(
                            self.width,
                            self.height,
                            rgb_fmt,
                            0,
                        )
                        logger.info(
                            f"Requested color profile: {self.width}x{self.height} → OK"
                        )
                    except Exception:
                        logger.warning(
                            f"Profile {self.width}x{self.height} không tìm thấy, "
                            "dùng default. Kiểm tra CAMERA_WIDTH/HEIGHT trong config."
                        )
                        color_profile = profiles.get_default_video_stream_profile()
                else:
                    # Legacy style: iterable profiles.
                    for profile in profiles:
                        w = _call_any(profile, ["get_width", "getWidth"])
                        h = _call_any(profile, ["get_height", "getHeight"])
                        if w == self.width and h == self.height:
                            color_profile = profile
                            break
                if color_profile is None:
                    if hasattr(profiles, "get_default_video_stream_profile"):
                        color_profile = profiles.get_default_video_stream_profile()
                    elif profiles:
                        color_profile = profiles[0]

                if color_profile is not None:
                    w = _call_any(color_profile, ["get_width", "getWidth"])
                    h = _call_any(color_profile, ["get_height", "getHeight"])
                    if w != self.width or h != self.height:
                        logger.warning(
                            f"Requested {self.width}x{self.height} not found, "
                            f"using default: {w}x{h}"
                        )
            except Exception as e:
                logger.warning(f"Could not configure color profile: {e}")

            if color_profile:
                _call_any(self.config, ["enable_stream", "enableStream"], color_profile)
                w = _call_any(color_profile, ["get_width", "getWidth"])
                h = _call_any(color_profile, ["get_height", "getHeight"])
                logger.info(f"Color stream: {w}x{h}")

            # Configure depth stream
            depth_profile = None
            try:
                depth_sensor = _enum_value(
                    ob,
                    ["SensorType", "OBSensorType"],
                    ["DEPTH", "DEPTH_SENSOR"],
                )
                if depth_sensor is None:
                    raise RuntimeError("Depth sensor enum not found")

                profiles = _call_any(
                    self.pipeline,
                    ["get_stream_profile_list", "getStreamProfileList"],
                    depth_sensor,
                )
                if hasattr(profiles, "get_default_video_stream_profile"):
                    depth_profile = profiles.get_default_video_stream_profile()
                elif profiles:
                    depth_profile = profiles[0]
            except Exception as e:
                logger.warning(f"Could not configure depth profile: {e}")

            if depth_profile:
                _call_any(self.config, ["enable_stream", "enableStream"], depth_profile)
                w = _call_any(depth_profile, ["get_width", "getWidth"])
                h = _call_any(depth_profile, ["get_height", "getHeight"])
                logger.info(f"Depth stream: {w}x{h}")

            # Optional alignment setup (API differs by SDK version).
            try:
                align_mode = _enum_value(
                    ob,
                    ["AlignMode", "OBAlignMode"],
                    ["ALIGN_D2C", "HW_MODE", "SW_MODE"],
                )
                if align_mode is not None:
                    _call_any(self.config, ["set_align_mode", "setAlignMode"], align_mode)
            except Exception as e:
                logger.warning(f"Could not set align mode: {e}")

            # Prefer full frame set (color+depth together) when SDK supports it.
            try:
                full_frame_require = _enum_value(
                    ob,
                    ["FrameAggregateOutputMode", "OBFrameAggregateOutputMode"],
                    ["FULL_FRAME_REQUIRE"],
                )
                if full_frame_require is not None:
                    _call_any(
                        self.config,
                        ["set_frame_aggregate_output_mode", "setFrameAggregateOutputMode"],
                        full_frame_require,
                    )
            except Exception as e:
                logger.warning(f"Could not set frame aggregate mode: {e}")

            # Start pipeline
            self.pipeline.start(self.config)
            logger.info("Camera connected and started")

        except Exception as e:
            logger.error(f"Failed to connect camera: {e}")
            raise

    def disconnect(self) -> None:
        """Disconnect and stop camera."""
        if self.pipeline:
            try:
                self.pipeline.stop()
                logger.info("Camera disconnected")
            except Exception as e:
                logger.error(f"Error disconnecting camera: {e}")
            finally:
                self.pipeline = None

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

    def get_aligned_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get aligned RGB and Depth frames.
        
        Returns:
            Tuple of (rgb_array, depth_array)
            - rgb_array: Shape (H, W, 3), uint8, BGR format
            - depth_array: Shape (H, W), uint16, depth in mm
            
        Raises:
            RuntimeError: If camera is not connected
        """
        if not self.pipeline:
            raise RuntimeError("Camera not connected")

        try:
            # Wait for frameset with retries; camera often needs warmup cycles.
            frameset = None
            for _ in range(8):
                frameset = _call_any(self.pipeline, ["wait_for_frames", "waitForFrames"], 1000)
                if frameset:
                    break
            if not frameset:
                raise RuntimeError("No frames received from camera after retries")

            # Get color frame
            color_frame = _call_any(frameset, ["get_color_frame", "getColorFrame"])
            if color_frame:
                rgb_data = _call_any(color_frame, ["get_data", "data"])
                rgb_array = np.frombuffer(rgb_data, dtype=np.uint8)
                color_h = _call_any(color_frame, ["get_height", "getHeight"])
                color_w = _call_any(color_frame, ["get_width", "getWidth"])
                rgb_array = rgb_array.reshape(
                    (color_h, color_w, 3)
                )
                logger.debug(f"RGB frame: {rgb_array.shape}")
            else:
                logger.warning("No color frame in frameset")
                rgb_array = np.zeros((self.height, self.width, 3), dtype=np.uint8)

            # Get depth frame
            depth_frame = _call_any(frameset, ["get_depth_frame", "getDepthFrame"])
            if depth_frame:
                depth_data = _call_any(depth_frame, ["get_data", "data"])
                depth_array = np.frombuffer(depth_data, dtype=np.uint16)
                depth_h = _call_any(depth_frame, ["get_height", "getHeight"])
                depth_w = _call_any(depth_frame, ["get_width", "getWidth"])
                depth_array = depth_array.reshape(
                    (depth_h, depth_w)
                )
                # Convert depth to millimeters if SDK reports a scale factor.
                try:
                    depth_scale = float(_call_any(depth_frame, ["get_depth_scale", "getDepthScale"]))
                    if depth_scale > 0 and abs(depth_scale - 1.0) > 1e-6:
                        depth_array = (depth_array.astype(np.float32) * depth_scale).astype(np.uint16)
                except Exception:
                    pass
                logger.debug(f"Depth frame: {depth_array.shape}, range: {depth_array.min()}-{depth_array.max()}")
            else:
                raise RuntimeError("No depth frame in frameset")

            return rgb_array, depth_array

        except Exception as e:
            logger.error(f"Error capturing frames: {e}")
            raise

    def get_frames_with_timestamp(
        self
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Chụp frame RGB+Depth và ghi lại thời điểm nhận frame (wall clock).

        Mục đích: so sánh với timestamp của RTDE để phát hiện
        frame/pose lệch nhau quá 100ms (robot rung nhẹ giữa 2 lần đọc).

        Returns:
            (rgb_array, depth_array, timestamp_s)
            timestamp_s: time.time() ngay sau khi pipeline trả frame — đơn vị giây.
        """
        rgb, depth = self.get_aligned_frames()
        # Ghi timestamp NGAY SAU khi nhận frame, trước bất kỳ xử lý nào
        timestamp_s = time.time()
        return rgb, depth, timestamp_s

    def get_reliable_depth(
        self,
        depth_array: np.ndarray,
        bbox: list,
        min_valid_ratio: float = 0.0,
    ) -> float:
        """
        Get reliable depth at bbox center using sliding-window median.

        For glossy plastic parts, center pixel often returns 0 (invalid) due to IR reflection.
        This method uses a local window around bbox center and returns median(valid_depths).
        """
        _ = min_valid_ratio  # Backward-compatible parameter kept for older call sites.
        x1, y1, x2, y2 = [int(v) for v in bbox]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        h, w = depth_array.shape
        bbox_w = max(1, x2 - x1)
        bbox_h = max(1, y2 - y1)

        def _clip_window(center_x: int, center_y: int, half: int):
            row_start = max(0, center_y - half)
            row_end = min(h, center_y + half + 1)
            col_start = max(0, center_x - half)
            col_end = min(w, center_x + half + 1)
            return row_start, row_end, col_start, col_end

        def _median_valid(region: np.ndarray) -> float:
            valid = region[region > 0]
            if valid.size == 0:
                return 0.0
            return float(np.median(valid))

        def _valid_stats(region: np.ndarray):
            valid = region[region > 0]
            if valid.size == 0:
                return 0.0, 0.0, 0.0
            return (
                float(np.median(valid)),
                float(valid.size) / float(region.size),
                float(np.max(valid) - np.min(valid)),
            )

        def _nearest_valid_stats(region: np.ndarray, center_x: int, center_y: int, origin_x: int, origin_y: int):
            valid_rows, valid_cols = np.nonzero(region > 0)
            if valid_rows.size == 0:
                return 0.0, 0, float("inf"), 0.0

            global_x = valid_cols + origin_x
            global_y = valid_rows + origin_y
            distances = np.sqrt((global_x - center_x) ** 2 + (global_y - center_y) ** 2)
            order = np.argsort(distances)
            take = min(len(order), max(config.DEPTH_NEAREST_MIN_SAMPLES, 1))
            picked = order[:take]
            picked_depths = region[valid_rows[picked], valid_cols[picked]].astype(np.float32)
            return (
                float(np.median(picked_depths)),
                int(picked_depths.size),
                float(distances[picked[-1]]) if picked.size > 0 else float("inf"),
                float(np.max(picked_depths) - np.min(picked_depths)),
            )

        base_half = max(0, int(config.DEPTH_WINDOW_HALF))
        max_half = max(base_half, min(24, max(4, min(bbox_w, bbox_h) // 4)))

        tried_halves = []
        for half in sorted(set([base_half, 4, 6, 8, 12, 16, max_half])):
            row_start, row_end, col_start, col_end = _clip_window(cx, cy, half)
            window = depth_array[row_start:row_end, col_start:col_end]
            if window.size == 0:
                continue
            median_depth = _median_valid(window)
            tried_halves.append(half)
            if median_depth > 0:
                logger.debug(
                    "Depth window median: %.1fmm (center=(%d,%d), half=%d, valid=%d/%d)",
                    median_depth,
                    cx,
                    cy,
                    half,
                    int(np.count_nonzero(window > 0)),
                    int(window.size),
                )
                return median_depth

        # Fallback 1: use central crop of bbox (more tolerant than single center window).
        inner_margin_x = max(1, int(bbox_w * 0.2))
        inner_margin_y = max(1, int(bbox_h * 0.2))
        inner_x1 = max(0, x1 + inner_margin_x)
        inner_y1 = max(0, y1 + inner_margin_y)
        inner_x2 = min(w, x2 - inner_margin_x)
        inner_y2 = min(h, y2 - inner_margin_y)
        if inner_x2 > inner_x1 and inner_y2 > inner_y1:
            inner_roi = depth_array[inner_y1:inner_y2, inner_x1:inner_x2]
            inner_depth, inner_valid_ratio, inner_spread = _valid_stats(inner_roi)
            if (
                inner_depth > 0
                and inner_valid_ratio >= max(min_valid_ratio, config.DEPTH_INNER_MIN_VALID_RATIO)
                and inner_spread <= config.DEPTH_INNER_MAX_SPREAD_MM
            ):
                logger.debug(
                    "Depth fallback inner ROI median: %.1fmm (valid=%d/%d, ratio=%.3f, spread=%.1fmm)",
                    inner_depth,
                    int(np.count_nonzero(inner_roi > 0)),
                    int(inner_roi.size),
                    inner_valid_ratio,
                    inner_spread,
                )
                return inner_depth
            nearest_depth, nearest_samples, nearest_max_dist, nearest_spread = _nearest_valid_stats(
                inner_roi,
                cx,
                cy,
                inner_x1,
                inner_y1,
            )
            if (
                nearest_depth > 0
                and nearest_samples >= config.DEPTH_NEAREST_MIN_SAMPLES
                and nearest_max_dist <= config.DEPTH_NEAREST_MAX_CENTER_DIST_PX
                and nearest_spread <= config.DEPTH_NEAREST_MAX_SPREAD_MM
            ):
                logger.debug(
                    "Depth fallback nearest-valid cluster: %.1fmm (samples=%d, max_dist=%.1fpx, spread=%.1fmm)",
                    nearest_depth,
                    nearest_samples,
                    nearest_max_dist,
                    nearest_spread,
                )
                return nearest_depth

        # Fallback 2: use entire bbox median if any valid depth exists.
        bbox_x1 = max(0, x1)
        bbox_y1 = max(0, y1)
        bbox_x2 = min(w, x2)
        bbox_y2 = min(h, y2)
        if bbox_x2 > bbox_x1 and bbox_y2 > bbox_y1:
            bbox_roi = depth_array[bbox_y1:bbox_y2, bbox_x1:bbox_x2]
            bbox_depth, bbox_valid_ratio, bbox_spread = _valid_stats(bbox_roi)
            if (
                bbox_depth > 0
                and bbox_valid_ratio >= max(min_valid_ratio, config.DEPTH_BBOX_MIN_VALID_RATIO)
                and bbox_spread <= config.DEPTH_BBOX_MAX_SPREAD_MM
            ):
                logger.debug(
                    "Depth fallback bbox ROI median: %.1fmm (valid=%d/%d, ratio=%.3f, spread=%.1fmm)",
                    bbox_depth,
                    int(np.count_nonzero(bbox_roi > 0)),
                    int(bbox_roi.size),
                    bbox_valid_ratio,
                    bbox_spread,
                )
                return bbox_depth

        logger.warning(
            "No valid depth around bbox center or ROI: %s (tried_halves=%s)",
            bbox,
            tried_halves,
        )
        return 0.0

    def analyze_depth_roi(
        self,
        depth_array: np.ndarray,
        bbox: list,
    ) -> dict:
        """Return debug metrics for depth quality around a detection bbox."""
        x1, y1, x2, y2 = [int(v) for v in bbox]
        h, w = depth_array.shape
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        def _stats(region: np.ndarray) -> dict:
            if region.size == 0:
                return {
                    "size": 0,
                    "valid": 0,
                    "ratio": 0.0,
                    "min_mm": 0.0,
                    "median_mm": 0.0,
                    "max_mm": 0.0,
                }
            valid = region[region > 0]
            if valid.size == 0:
                return {
                    "size": int(region.size),
                    "valid": 0,
                    "ratio": 0.0,
                    "min_mm": 0.0,
                    "median_mm": 0.0,
                    "max_mm": 0.0,
                }
            return {
                "size": int(region.size),
                "valid": int(valid.size),
                "ratio": float(valid.size) / float(region.size),
                "min_mm": float(np.min(valid)),
                "median_mm": float(np.median(valid)),
                "max_mm": float(np.max(valid)),
            }

        bbox_roi = depth_array[y1:y2, x1:x2]
        half = max(3, int(min(max(1, x2 - x1), max(1, y2 - y1), 12) // 2))
        c_y1 = max(0, cy - half)
        c_y2 = min(h, cy + half)
        c_x1 = max(0, cx - half)
        c_x2 = min(w, cx + half)
        center_roi = depth_array[c_y1:c_y2, c_x1:c_x2]
        inner_margin_x = max(1, int(max(1, x2 - x1) * config.DEPTH_INNER_MARGIN_RATIO))
        inner_margin_y = max(1, int(max(1, y2 - y1) * config.DEPTH_INNER_MARGIN_RATIO))
        i_x1 = max(0, x1 + inner_margin_x)
        i_y1 = max(0, y1 + inner_margin_y)
        i_x2 = min(w, x2 - inner_margin_x)
        i_y2 = min(h, y2 - inner_margin_y)
        inner_roi = depth_array[i_y1:i_y2, i_x1:i_x2] if i_x2 > i_x1 and i_y2 > i_y1 else np.empty((0, 0), dtype=depth_array.dtype)
        valid_points = np.argwhere(inner_roi > 0)
        valid_points_global = (
            [[int(i_x1 + col), int(i_y1 + row)] for row, col in valid_points[:250]]
            if valid_points.size > 0
            else []
        )

        return {
            "bbox": [x1, y1, x2, y2],
            "center_px": [cx, cy],
            "bbox_stats": _stats(bbox_roi),
            "center_roi_bounds": [c_x1, c_y1, c_x2, c_y2],
            "center_stats": _stats(center_roi),
            "inner_roi_bounds": [i_x1, i_y1, i_x2, i_y2],
            "inner_stats": _stats(inner_roi),
            "inner_valid_points_sample": valid_points_global,
            "config": {
                "inner_margin_ratio": config.DEPTH_INNER_MARGIN_RATIO,
                "inner_min_valid_ratio": config.DEPTH_INNER_MIN_VALID_RATIO,
                "inner_max_spread_mm": config.DEPTH_INNER_MAX_SPREAD_MM,
                "inner_relaxed_min_valid_ratio": config.DEPTH_INNER_RELAXED_MIN_VALID_RATIO,
                "inner_relaxed_max_spread_mm": config.DEPTH_INNER_RELAXED_MAX_SPREAD_MM,
                "nearest_min_samples": config.DEPTH_NEAREST_MIN_SAMPLES,
                "nearest_max_center_dist_px": config.DEPTH_NEAREST_MAX_CENTER_DIST_PX,
                "nearest_max_spread_mm": config.DEPTH_NEAREST_MAX_SPREAD_MM,
                "nearest_relaxed_max_center_dist_px": config.DEPTH_NEAREST_RELAXED_MAX_CENTER_DIST_PX,
                "nearest_relaxed_max_spread_mm": config.DEPTH_NEAREST_RELAXED_MAX_SPREAD_MM,
            },
        }
