"""
YOLO Object Detection for Phôi (parts).
Uses Ultralytics YOLOv8.
"""

import logging
import numpy as np
from typing import List, Dict, Optional, Tuple
from pathlib import Path
import cv2

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
        self.pick_point = list(self.center)
        self.pick_bbox = list(self.bbox)
        self.pick_source = "bbox_center"

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
            "pick_point": self.pick_point,
            "pick_bbox": self.pick_bbox,
            "pick_source": self.pick_source,
            "confidence": self.confidence,
            "label": self.label,
            "area": self.area,
        }


class Detector:
    """YOLO detector for phôi objects."""

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
        camera_module=None  # Will be injected, for now None
    ) -> Optional[Detection]:
        """
        Select best target from detections based on depth quality and proximity.
        
        Scoring:
            score = 0.4 * depth_quality + 0.6 * (1 - normalized_distance_to_center)
        
        Args:
            detections: List of Detection objects
            depth_array: Depth frame (H, W) in mm
            frame_center_uv: (u, v) center of frame
            camera_module: Camera module for depth extraction (optional)
            
        Returns:
            Best Detection or None if no valid target
        """
        if not detections:
            logger.info("No detections to select from")
            return None

        # Import locally to avoid circular dependency
        if camera_module is None:
            from . import femto_camera
            camera_module = femto_camera

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

            valid_ratio = float(np.count_nonzero(roi > 0)) / float(roi.size)

            # Calculate depth quality
            try:
                # Use a simple camera instance for get_reliable_depth method
                # We can call it directly from femto_camera module
                median_depth = camera_module.FemtoCamera.get_reliable_depth(
                    None,  # No self needed for static-like usage
                    depth_array,
                    [x1, y1, x2, y2],
                    min_valid_ratio=0.1
                )
            except Exception:
                # Fallback: direct calculation
                valid = roi[roi > 0]
                if len(valid) == 0:
                    median_depth = 0.0
                else:
                    median_depth = float(np.median(valid))

            if median_depth <= 0:
                logger.debug(f"No valid depth for detection: {detection}")
                continue

            depth_quality = min(1.0, valid_ratio)

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
                f"dist_norm={normalized_dist:.2f}, score={score:.3f}"
            )

            if score > best_score:
                best_score = score
                best_detection = detection

        if best_detection:
            logger.info(f"Selected best target: {best_detection} (score={best_score:.3f})")
        else:
            logger.warning("No valid target with depth data found")

        return best_detection

    def refine_pick_point(
        self,
        rgb_image: np.ndarray,
        detection: Detection,
        depth_array: Optional[np.ndarray] = None,
        window_radius_px: int = 6,
    ) -> Detection:
        """
        Refine pick point inside YOLO bbox using bright-object contour centroid.

        This is intended for white phoi on dark background, where bbox center can
        be noticeably biased from the true object center.
        """
        x1, y1, x2, y2 = [int(v) for v in detection.bbox]
        h, w = rgb_image.shape[:2]
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return detection

        roi = rgb_image[y1:y2, x1:x2]
        if roi.size == 0:
            return detection

        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return detection

        roi_cx = (x2 - x1) / 2.0
        roi_cy = (y2 - y1) / 2.0
        best_contour = None
        best_score = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 40.0:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 1e-6:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            dist2 = (cx - roi_cx) ** 2 + (cy - roi_cy) ** 2
            score = (-area, dist2)
            if best_score is None or score < best_score:
                best_score = score
                best_contour = contour

        if best_contour is None:
            return detection

        moments = cv2.moments(best_contour)
        if moments["m00"] <= 1e-6:
            return detection

        cx = float(moments["m10"] / moments["m00"])
        cy = float(moments["m01"] / moments["m00"])
        global_u = x1 + cx
        global_v = y1 + cy

        bx, by, bw, bh = cv2.boundingRect(best_contour)
        pick_bbox = [x1 + bx, y1 + by, x1 + bx + bw, y1 + by + bh]

        detection.pick_point = [global_u, global_v]
        detection.pick_bbox = pick_bbox
        detection.pick_source = "contour_centroid"

        if depth_array is not None:
            du = int(round(global_u))
            dv = int(round(global_v))
            x1d = max(0, du - window_radius_px)
            y1d = max(0, dv - window_radius_px)
            x2d = min(depth_array.shape[1], du + window_radius_px + 1)
            y2d = min(depth_array.shape[0], dv + window_radius_px + 1)
            if x2d > x1d and y2d > y1d:
                detection.pick_bbox = [x1d, y1d, x2d, y2d]

        return detection

    def resolve_pick_depth(
        self,
        depth_array: np.ndarray,
        detection: Detection,
        camera_module=None,
        search_radii_px: Optional[List[int]] = None,
    ) -> Tuple[float, list]:
        """
        Resolve reliable depth for pick point with progressive fallback.

        Order:
        1. Small/medium windows around refined pick point
        2. Full YOLO bbox
        """
        if camera_module is None:
            from . import femto_camera
            camera_module = femto_camera

        if search_radii_px is None:
            search_radii_px = [6, 10, 16, 24]

        h, w = depth_array.shape
        pu, pv = detection.pick_point
        cx = int(round(pu))
        cy = int(round(pv))

        tried_bboxes = []
        for radius in search_radii_px:
            x1 = max(0, cx - radius)
            y1 = max(0, cy - radius)
            x2 = min(w, cx + radius + 1)
            y2 = min(h, cy + radius + 1)
            bbox = [x1, y1, x2, y2]
            tried_bboxes.append(bbox)
            depth_mm = camera_module.FemtoCamera.get_reliable_depth(None, depth_array, bbox, min_valid_ratio=0.02)
            if depth_mm > 0:
                detection.pick_bbox = bbox
                detection.pick_source = f"{detection.pick_source}+depth_r{radius}"
                return depth_mm, bbox

        depth_mm = camera_module.FemtoCamera.get_reliable_depth(None, depth_array, detection.bbox, min_valid_ratio=0.02)
        if depth_mm > 0:
            detection.pick_bbox = list(detection.bbox)
            detection.pick_source = f"{detection.pick_source}+depth_bbox"
            return depth_mm, detection.pick_bbox

        logger.warning("No valid depth around refined pick point or full bbox")
        return 0.0, detection.pick_bbox
