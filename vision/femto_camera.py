"""
Orbbec Femto Mega Camera Interface.
Captures aligned RGB and Depth frames.

Installation:
  Ensure `import ob` works in the project environment.

The project provides an `ob.py` compatibility shim that loads the
installed Orbbec binding or the bundled runtime from `vendor/`.
"""

import logging
import time
import numpy as np
from typing import Tuple, Optional

try:
    import ob
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


def _pick_video_profile(profiles, width: int, height: int, format_candidates):
    """Pick the requested profile when possible; otherwise fall back safely."""
    if hasattr(profiles, "get_video_stream_profile"):
        for fmt in format_candidates:
            if fmt is None:
                continue
            try:
                return profiles.get_video_stream_profile(width, height, fmt, 0)
            except Exception:
                continue
        try:
            return profiles.get_default_video_stream_profile()
        except Exception:
            return None

    for profile in profiles:
        w = _call_any(profile, ["get_width", "getWidth"])
        h = _call_any(profile, ["get_height", "getHeight"])
        if w == width and h == height:
            return profile

    if hasattr(profiles, "__getitem__"):
        try:
            return profiles[0]
        except Exception:
            return None
    return None


class FemtoCamera:
    """Orbbec Femto Mega camera interface."""

    def __init__(self, width: int = 1280, height: int = 720):
        """
        Initialize camera.
        
        Args:
            width: Image width in pixels (default 1280)
            height: Image height in pixels (default 720)
        """
        if not ORBBEC_AVAILABLE:
            raise RuntimeError(
                "`import ob` is not available. "
                "Install Orbbec SDK or use the bundled runtime."
            )

        self.width = width
        self.height = height
        self.pipeline: Optional[ob.Pipeline] = None
        self.config: Optional[ob.Config] = None

    def connect(self) -> None:
        """Connect and start camera."""
        try:
            logger.info("Initializing Orbbec Femto Mega camera...")

            # Create pipeline
            self.pipeline = ob.Pipeline()

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

                color_profile = _pick_video_profile(
                    profiles,
                    self.width,
                    self.height,
                    [
                        _enum_value(ob, ["Format", "OBFormat"], ["RGB", "RGB888"]),
                        _enum_value(ob, ["Format", "OBFormat"], ["BGR"]),
                        _enum_value(ob, ["Format", "OBFormat"], ["MJPG"]),
                    ],
                )
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
                depth_profile = _pick_video_profile(
                    profiles,
                    self.width,
                    self.height,
                    [
                        _enum_value(ob, ["Format", "OBFormat"], ["Y16"]),
                    ],
                )
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
        min_valid_ratio: float = 0.1
    ) -> float:
        """
        Get reliable depth value from region of interest.
        
        Args:
            depth_array: Depth frame (H, W) in mm
            bbox: [x1, y1, x2, y2] in pixels
            min_valid_ratio: Minimum ratio of valid pixels required (0-1)
            
        Returns:
            Median depth in mm, or 0.0 if insufficient valid data
        """
        x1, y1, x2, y2 = [int(v) for v in bbox]
        
        # Clip to frame bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(depth_array.shape[1], x2)
        y2 = min(depth_array.shape[0], y2)

        # Extract ROI
        roi = depth_array[y1:y2, x1:x2]
        
        if roi.size == 0:
            logger.warning("Invalid bbox")
            return 0.0

        # Remove zeros (depth holes, common on white surfaces)
        valid = roi[roi > 0]
        
        if len(valid) == 0:
            logger.warning("No valid depth data in ROI")
            return 0.0

        # Kiểm tra tỉ lệ pixel hợp lệ tối thiểu
        valid_ratio = len(valid) / roi.size
        if valid_ratio < min_valid_ratio:
            logger.warning(
                f"Insufficient valid depth data: {valid_ratio:.1%} < {min_valid_ratio:.1%}"
            )
            return 0.0

        # ── Phương pháp percentile cho phôi trụ trắng đứng ──────────────────────
        # Tình huống depth map trong bbox:
        #   Mặt trên phôi: depth = 0 (hole do phản chiếu) → đã loại ở trên
        #   Viền phôi:     depth = 400mm (gần nhất, đúng)
        #   Nền khay:      depth = 450mm (xa hơn, không phải phôi)
        # → Lấy percentile thấp để ưu tiên điểm gần nhất (đỉnh phôi/viền phôi)
        # thay vì median (dễ bị kéo về phía nền khay)
        depth_10th = float(np.percentile(valid, 10))
        depth_30th = float(np.percentile(valid, 30))

        if (depth_30th - depth_10th) < 20.0:  # vùng ổn định ≤ 20mm
            # 10th–30th gần nhau → có cụm điểm đồng nhất ở đỉnh phôi
            logger.debug(
                f"ROI depth (percentile stable): {depth_10th:.1f}mm "
                f"(10th–30th spread: {depth_30th - depth_10th:.1f}mm, "
                f"valid ratio: {valid_ratio:.1%})"
            )
            return depth_10th

        # Fallback: loại outlier cao (nền khay), lấy median của foreground
        bg_threshold = float(np.percentile(valid, 40))
        foreground = valid[valid <= bg_threshold]
        if len(foreground) < 5:
            logger.warning("Fallback foreground too small, returning 10th percentile")
            return depth_10th

        median_depth = float(np.median(foreground))
        logger.debug(
            f"ROI depth (percentile fallback): {median_depth:.1f}mm "
            f"(valid ratio: {valid_ratio:.1%})"
        )
        return median_depth
