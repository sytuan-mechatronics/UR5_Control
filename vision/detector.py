"""
YOLO Object Detection for Phôi (parts).
Uses Ultralytics YOLOv8.
"""

import logging
import numpy as np
from typing import List, Dict, Optional
from pathlib import Path

import config

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


logger = logging.getLogger(__name__)


class Detection:
    """Single detection result."""

    def __init__(
        self,
        bbox: list,
        confidence: float,
        label: str,
        area: int
    ):
        """
        Initialize detection.
        
        Args:
            bbox: [x1, y1, x2, y2] in pixels
            confidence: Detection confidence (0-1)
            label: Class label
            area: Bounding box area in pixels²
        """
        self.bbox = bbox
        self.x1, self.y1, self.x2, self.y2 = bbox
        self.confidence = confidence
        self.label = label
        self.area = area
        self.center = [
            (self.x1 + self.x2) / 2.0,
            (self.y1 + self.y2) / 2.0
        ]
        self.depth_debug = {}

    def __repr__(self) -> str:
        return (
            f"Detection(label={self.label}, confidence={self.confidence:.2f}, "
            f"bbox={self.bbox}, center={self.center})"
        )

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "bbox": self.bbox,
            "center": self.center,
            "confidence": self.confidence,
            "label": self.label,
            "area": self.area,
        }


class Detector:
    """YOLO detector for phôi objects."""

    MICRO_DEPTH_WINDOW_SIZE = 6

    def __init__(
        self,
        model_path: str,
        confidence: float = 0.5,
        target_class: str = "phoi"
    ):
        """
        Initialize detector.
        
        Args:
            model_path: Path to YOLO model file (.pt)
            confidence: Detection confidence threshold (0-1)
            target_class: Target class name to detect
        """
        if not YOLO_AVAILABLE:
            raise RuntimeError(
                "ultralytics not available. "
                "Install with: pip install ultralytics"
            )

        self.model_path = model_path
        self.confidence = confidence
        self.target_class = target_class
        self.model = None

        # Try to load model
        self._load_model()

    def _load_model(self) -> None:
        """Load YOLO model."""
        model_file = Path(self.model_path)
        
        if not model_file.exists():
            logger.warning(f"Model file not found: {self.model_path}")
            logger.info("Will attempt to load from Ultralytics hub on first use")
            # Don't raise error - model will be auto-downloaded on first detect()
            return

        try:
            self.model = YOLO(self.model_path)
            logger.info(f"Loaded YOLO model: {self.model_path}")
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise

    def _extract_center_micro_roi_depth(
        self,
        depth_array: np.ndarray,
        bbox: list,
        frame_w: int,
        frame_h: int,
    ) -> tuple:
        """Estimate depth from a tiny ROI centered on bbox center.

        Physical rationale:
        - YOLO bbox is square-ish, but the target top face is circular.
        - Sampling the whole bbox includes tray background around the 4 corners.
        - For a tall cylinder inside a tray hole, that background can be ~50 mm deeper,
          which is unacceptable for robot Z targeting.

        Returns:
            (median_depth_mm, valid_ratio, roi_bounds)
            roi_bounds = (x1, y1, x2, y2) after clamping
        """
        if depth_array is None or depth_array.size == 0:
            logger.debug("Depth array is empty while extracting center micro ROI")
            return 0.0, 0.0, None

        bx1, by1, bx2, by2 = [int(v) for v in bbox]
        cx = int(round((bx1 + bx2) / 2.0))
        cy = int(round((by1 + by2) / 2.0))

        half = max(1, int(self.MICRO_DEPTH_WINDOW_SIZE // 2))
        roi_x1 = max(0, cx - half)
        roi_y1 = max(0, cy - half)
        roi_x2 = min(frame_w, cx + half)
        roi_y2 = min(frame_h, cy + half)

        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            logger.debug(
                "Micro ROI invalid after clamping: bbox=%s center=(%d,%d) roi=(%d,%d,%d,%d)",
                bbox,
                cx,
                cy,
                roi_x1,
                roi_y1,
                roi_x2,
                roi_y2,
            )
            return 0.0, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

        try:
            micro_roi = depth_array[roi_y1:roi_y2, roi_x1:roi_x2]
            if micro_roi.size == 0:
                logger.debug("Micro ROI is empty after slicing: bbox=%s", bbox)
                return 0.0, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

            valid_depths = micro_roi[micro_roi > 0]
            valid_ratio = float(valid_depths.size) / float(micro_roi.size)
            if valid_depths.size == 0:
                logger.debug(
                    "No valid depth in center micro ROI: bbox=%s roi=(%d,%d,%d,%d)",
                    bbox,
                    roi_x1,
                    roi_y1,
                    roi_x2,
                    roi_y2,
                )
                return 0.0, valid_ratio, (roi_x1, roi_y1, roi_x2, roi_y2)

            median_depth = float(np.median(valid_depths))
            return median_depth, valid_ratio, (roi_x1, roi_y1, roi_x2, roi_y2)
        except Exception as exc:
            logger.warning("Micro ROI depth extraction failed for bbox=%s: %s", bbox, exc)
            return 0.0, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

    def _extract_inner_roi_depth(
        self,
        depth_array: np.ndarray,
        bbox: list,
        frame_w: int,
        frame_h: int,
    ) -> tuple:
        """Estimate depth from an inner crop of the bbox when center micro ROI has holes."""
        bx1, by1, bx2, by2 = [int(v) for v in bbox]
        bbox_w = max(1, bx2 - bx1)
        bbox_h = max(1, by2 - by1)
        margin_x = max(1, int(bbox_w * config.DEPTH_INNER_MARGIN_RATIO))
        margin_y = max(1, int(bbox_h * config.DEPTH_INNER_MARGIN_RATIO))
        roi_x1 = max(0, bx1 + margin_x)
        roi_y1 = max(0, by1 + margin_y)
        roi_x2 = min(frame_w, bx2 - margin_x)
        roi_y2 = min(frame_h, by2 - margin_y)
        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            return 0.0, 0.0, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

        inner_roi = depth_array[roi_y1:roi_y2, roi_x1:roi_x2]
        if inner_roi.size == 0:
            return 0.0, 0.0, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

        valid_depths = inner_roi[inner_roi > 0]
        valid_ratio = float(valid_depths.size) / float(inner_roi.size)
        if valid_depths.size == 0:
            return 0.0, valid_ratio, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

        median_depth = float(np.median(valid_depths))
        spread_mm = float(np.max(valid_depths) - np.min(valid_depths))
        return median_depth, valid_ratio, spread_mm, (roi_x1, roi_y1, roi_x2, roi_y2)

    def _extract_nearest_valid_depth(
        self,
        depth_array: np.ndarray,
        bbox: list,
        frame_w: int,
        frame_h: int,
    ) -> tuple:
        """Use nearest valid depth samples around bbox center when center ROI has holes."""
        bx1, by1, bx2, by2 = [int(v) for v in bbox]
        bbox_w = max(1, bx2 - bx1)
        bbox_h = max(1, by2 - by1)
        margin_x = max(1, int(bbox_w * config.DEPTH_INNER_MARGIN_RATIO))
        margin_y = max(1, int(bbox_h * config.DEPTH_INNER_MARGIN_RATIO))
        roi_x1 = max(0, bx1 + margin_x)
        roi_y1 = max(0, by1 + margin_y)
        roi_x2 = min(frame_w, bx2 - margin_x)
        roi_y2 = min(frame_h, by2 - margin_y)
        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            return 0.0, 0, float("inf"), 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

        inner_roi = depth_array[roi_y1:roi_y2, roi_x1:roi_x2]
        valid_rows, valid_cols = np.nonzero(inner_roi > 0)
        if valid_rows.size == 0:
            return 0.0, 0, float("inf"), 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)

        center_x = int(round((bx1 + bx2) / 2.0))
        center_y = int(round((by1 + by2) / 2.0))
        global_x = valid_cols + roi_x1
        global_y = valid_rows + roi_y1
        distances = np.sqrt((global_x - center_x) ** 2 + (global_y - center_y) ** 2)
        order = np.argsort(distances)
        take = min(len(order), max(config.DEPTH_NEAREST_MIN_SAMPLES, 1))
        picked = order[:take]
        picked_depths = inner_roi[valid_rows[picked], valid_cols[picked]].astype(np.float32)
        return (
            float(np.median(picked_depths)),
            int(picked_depths.size),
            float(distances[picked[-1]]) if picked.size > 0 else float("inf"),
            float(np.max(picked_depths) - np.min(picked_depths)),
            (roi_x1, roi_y1, roi_x2, roi_y2),
        )

    def _extract_bbox_depth(
        self,
        depth_array: np.ndarray,
        bbox: list,
        frame_w: int,
        frame_h: int,
    ) -> tuple:
        """Use the whole bbox when depth is stable enough across the detected top face."""
        bx1, by1, bx2, by2 = [int(v) for v in bbox]
        roi_x1 = max(0, bx1)
        roi_y1 = max(0, by1)
        roi_x2 = min(frame_w, bx2)
        roi_y2 = min(frame_h, by2)
        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            return 0.0, 0.0, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)
        bbox_roi = depth_array[roi_y1:roi_y2, roi_x1:roi_x2]
        valid_depths = bbox_roi[bbox_roi > 0]
        valid_ratio = float(valid_depths.size) / float(max(1, bbox_roi.size))
        if valid_depths.size == 0:
            return 0.0, valid_ratio, 0.0, (roi_x1, roi_y1, roi_x2, roi_y2)
        return (
            float(np.median(valid_depths)),
            valid_ratio,
            float(np.max(valid_depths) - np.min(valid_depths)),
            (roi_x1, roi_y1, roi_x2, roi_y2),
        )

    def detect(self, rgb_image: np.ndarray) -> List[Detection]:
        """
        Detect objects in image.
        
        Args:
            rgb_image: RGB image (H, W, 3), uint8
            
        Returns:
            List of Detection objects, sorted by confidence (descending)
        """
        if self.model is None:
            try:
                self.model = YOLO(self.model_path)
            except Exception as e:
                logger.error(f"Failed to load model: {e}")
                return []

        try:
            # Run inference
            results = self.model(rgb_image, conf=self.confidence, verbose=False)
            detections = []

            # Process results
            for result in results:
                if result.boxes is None:
                    continue

                for i, box in enumerate(result.boxes):
                    # Get coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Get confidence
                    conf = float(box.conf[0].cpu().numpy())

                    # Get class name
                    class_id = int(box.cls[0].cpu().numpy())
                    label = result.names[class_id] if class_id in result.names else str(class_id)

                    # Filter by target class
                    if label.lower() != self.target_class.lower():
                        continue

                    # Calculate area
                    area = (x2 - x1) * (y2 - y1)

                    detection = Detection(
                        bbox=[x1, y1, x2, y2],
                        confidence=conf,
                        label=label,
                        area=area
                    )
                    detections.append(detection)

            # Sort by confidence descending
            detections.sort(key=lambda d: d.confidence, reverse=True)

            logger.info(
                f"Detected {len(detections)} {self.target_class} objects"
            )

            return detections

        except Exception as e:
            logger.error(f"Error during detection: {e}")
            return []

    def select_best_target(
        self,
        detections: List[Detection],
        depth_array: np.ndarray,
        frame_center_uv: tuple,
    ) -> Optional[Detection]:
        """
        Select best target from detections based on depth quality and proximity.
        
        Scoring:
            score = 0.4 * depth_quality + 0.6 * (1 - normalized_distance_to_center)
        
        Args:
            detections: List of Detection objects
            depth_array: Depth frame (H, W) in mm
            frame_center_uv: (u, v) center of frame
            
        Returns:
            Best Detection or None if no valid target
        """
        if not detections:
            logger.info("No detections to select from")
            return None

        frame_h, frame_w = depth_array.shape
        frame_cx, frame_cy = frame_center_uv

        best_detection = None
        best_score = -1.0

        # Max possible distance in frame
        max_dist = np.sqrt(frame_w**2 + frame_h**2) / 2.0

        for detection in detections:
            x1, y1, x2, y2 = [int(v) for v in detection.bbox]
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(frame_w, x2)
            y2 = min(frame_h, y2)
            if x2 <= x1 or y2 <= y1:
                logger.debug(f"Invalid bbox after clipping: {detection.bbox}")
                continue

            roi = depth_array[y1:y2, x1:x2]
            if roi.size == 0:
                logger.debug(f"Empty ROI for detection: {detection}")
                continue

            median_depth = 0.0
            valid_ratio = 0.0
            micro_depth, micro_valid_ratio, micro_bounds = self._extract_center_micro_roi_depth(
                depth_array,
                detection.bbox,
                frame_w,
                frame_h,
            )
            depth_source = "none"
            spread_mm = 0.0
            inner_bounds = None
            inner_valid_ratio = 0.0
            inner_spread_mm = 0.0
            inner_depth = 0.0
            nearest_depth = 0.0
            nearest_samples = 0
            nearest_max_dist = float("inf")
            nearest_spread_mm = 0.0
            nearest_bounds = None
            bbox_depth = 0.0
            bbox_valid_ratio = 0.0
            bbox_spread_mm = 0.0
            bbox_bounds = None

            # Prefer the most stable region of the detected top face, not just the center.
            inner_depth, inner_valid_ratio, inner_spread_mm, inner_bounds = self._extract_inner_roi_depth(
                depth_array,
                detection.bbox,
                frame_w,
                frame_h,
            )
            if (
                inner_depth > 0
                and inner_valid_ratio >= config.DEPTH_INNER_MIN_VALID_RATIO
                and inner_spread_mm <= config.DEPTH_INNER_MAX_SPREAD_MM
            ):
                median_depth = inner_depth
                valid_ratio = inner_valid_ratio
                depth_source = "inner"
                spread_mm = inner_spread_mm
            elif (
                inner_depth > 0
                and inner_valid_ratio >= config.DEPTH_INNER_RELAXED_MIN_VALID_RATIO
                and inner_spread_mm <= config.DEPTH_INNER_RELAXED_MAX_SPREAD_MM
            ):
                median_depth = inner_depth
                valid_ratio = inner_valid_ratio
                depth_source = "inner_relaxed"
                spread_mm = inner_spread_mm
            else:
                nearest_depth, nearest_samples, nearest_max_dist, nearest_spread_mm, nearest_bounds = self._extract_nearest_valid_depth(
                    depth_array,
                    detection.bbox,
                    frame_w,
                    frame_h,
                )
                if (
                    nearest_depth > 0
                    and nearest_samples >= config.DEPTH_NEAREST_MIN_SAMPLES
                    and nearest_max_dist <= config.DEPTH_NEAREST_MAX_CENTER_DIST_PX
                    and nearest_spread_mm <= config.DEPTH_NEAREST_MAX_SPREAD_MM
                ):
                    median_depth = nearest_depth
                    valid_ratio = max(
                        inner_valid_ratio,
                        float(nearest_samples) / float(max(1, (nearest_bounds[2] - nearest_bounds[0]) * (nearest_bounds[3] - nearest_bounds[1]))),
                    )
                    depth_source = "nearest"
                    spread_mm = nearest_spread_mm
                elif (
                    nearest_depth > 0
                    and nearest_samples >= config.DEPTH_NEAREST_MIN_SAMPLES
                    and nearest_max_dist <= config.DEPTH_NEAREST_RELAXED_MAX_CENTER_DIST_PX
                    and nearest_spread_mm <= config.DEPTH_NEAREST_RELAXED_MAX_SPREAD_MM
                ):
                    median_depth = nearest_depth
                    valid_ratio = max(
                        inner_valid_ratio,
                        float(nearest_samples) / float(max(1, (nearest_bounds[2] - nearest_bounds[0]) * (nearest_bounds[3] - nearest_bounds[1]))),
                    )
                    depth_source = "nearest_relaxed"
                    spread_mm = nearest_spread_mm
                else:
                    bbox_depth, bbox_valid_ratio, bbox_spread_mm, bbox_bounds = self._extract_bbox_depth(
                        depth_array,
                        detection.bbox,
                        frame_w,
                        frame_h,
                    )
                    if (
                        bbox_depth > 0
                        and bbox_valid_ratio >= config.DEPTH_BBOX_MIN_VALID_RATIO
                        and bbox_spread_mm <= config.DEPTH_BBOX_MAX_SPREAD_MM
                    ):
                        median_depth = bbox_depth
                        valid_ratio = bbox_valid_ratio
                        depth_source = "bbox"
                        spread_mm = bbox_spread_mm

            if median_depth <= 0:
                detection.depth_debug = {
                    "selected": False,
                    "depth_source": "none",
                    "micro_roi": micro_bounds,
                    "micro_valid_ratio": micro_valid_ratio,
                    "micro_median_depth_mm": micro_depth,
                    "inner_roi": inner_bounds,
                    "inner_valid_ratio": inner_valid_ratio,
                    "inner_median_depth_mm": inner_depth,
                    "inner_spread_mm": inner_spread_mm,
                    "nearest_roi": nearest_bounds,
                    "nearest_samples": nearest_samples,
                    "nearest_median_depth_mm": nearest_depth,
                    "nearest_max_center_dist_px": nearest_max_dist,
                    "nearest_spread_mm": nearest_spread_mm,
                    "bbox_roi": bbox_bounds,
                    "bbox_valid_ratio": bbox_valid_ratio,
                    "bbox_median_depth_mm": bbox_depth,
                    "bbox_spread_mm": bbox_spread_mm,
                }
                logger.warning(
                    "No valid depth for detection: %s micro_roi=%s micro_valid=%.3f inner_roi=%s inner_valid=%.3f inner_spread=%.1f nearest_samples=%d nearest_dist=%.1f nearest_spread=%.1f bbox_valid=%.3f bbox_spread=%.1f",
                    detection,
                    micro_bounds,
                    micro_valid_ratio,
                    inner_bounds,
                    inner_valid_ratio,
                    inner_spread_mm,
                    nearest_samples,
                    nearest_max_dist,
                    nearest_spread_mm,
                    bbox_valid_ratio,
                    bbox_spread_mm,
                )
                continue

            depth_quality = min(1.0, valid_ratio)
            if depth_source == "inner":
                depth_quality *= 0.9
            elif depth_source == "inner_relaxed":
                depth_quality *= 0.82
            elif depth_source == "nearest":
                depth_quality *= 0.8
            elif depth_source == "nearest_relaxed":
                depth_quality *= 0.72
            elif depth_source == "bbox":
                depth_quality *= 0.68

            # Calculate distance to frame center
            dist_to_center = np.sqrt(
                (detection.center[0] - frame_cx)**2 +
                (detection.center[1] - frame_cy)**2
            )
            normalized_dist = dist_to_center / max_dist

            # Calculate score
            score = 0.4 * depth_quality + 0.6 * (1.0 - normalized_dist)

            logger.debug(
                f"Detection {detection.label}: depth_quality={depth_quality:.2f}, "
                f"dist_norm={normalized_dist:.2f}, score={score:.3f}, "
                f"median_depth={median_depth:.1f}mm, source={depth_source}, "
                f"micro_roi={micro_bounds}, inner_roi={inner_bounds}"
            )
            detection.depth_debug = {
                "selected": False,
                "depth_source": depth_source,
                "micro_roi": micro_bounds,
                "micro_valid_ratio": micro_valid_ratio,
                "micro_median_depth_mm": micro_depth,
                "inner_roi": inner_bounds,
                "inner_valid_ratio": inner_valid_ratio,
                "inner_median_depth_mm": inner_depth,
                "inner_spread_mm": inner_spread_mm,
                "nearest_roi": nearest_bounds,
                "nearest_samples": nearest_samples,
                "nearest_median_depth_mm": nearest_depth,
                "nearest_max_center_dist_px": nearest_max_dist,
                "nearest_spread_mm": nearest_spread_mm,
                "bbox_roi": bbox_bounds,
                "bbox_valid_ratio": bbox_valid_ratio,
                "bbox_median_depth_mm": bbox_depth,
                "bbox_spread_mm": bbox_spread_mm,
                "depth_quality": depth_quality,
                "dist_to_center_px": dist_to_center,
                "score": score,
                "selected_depth_mm": median_depth,
            }

            if score > best_score:
                best_score = score
                best_detection = detection

        if best_detection:
            best_detection.depth_debug["selected"] = True
            logger.info(f"Selected best target: {best_detection} (score={best_score:.3f})")
        else:
            logger.warning("No valid target with depth data found")

        return best_detection
