"""Tray contour detection and layout projection."""

from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from vision.tray_layout import load_tray_layout


def order_quad_points(points: np.ndarray) -> np.ndarray:
    pts = np.array(points, dtype=np.float32).reshape(4, 2)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(sums)]   # top-left
    ordered[2] = pts[np.argmax(sums)]   # bottom-right
    ordered[1] = pts[np.argmin(diffs)]  # top-right
    ordered[3] = pts[np.argmax(diffs)]  # bottom-left
    return ordered


def detect_tray_quad(rgb_image: np.ndarray, min_area_px: float = 80000.0) -> Optional[np.ndarray]:
    """Detect the tray outer quadrilateral from RGB image."""
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_quad = None
    best_area = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area_px:
            continue
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        if len(approx) == 4:
            quad = order_quad_points(approx[:, 0, :])
        else:
            rect = cv2.minAreaRect(contour)
            quad = order_quad_points(cv2.boxPoints(rect))
        quad_area = cv2.contourArea(quad.astype(np.float32))
        if quad_area < min_area_px:
            continue
        if best_area is None or quad_area > best_area:
            best_area = quad_area
            best_quad = quad
    return best_quad


def project_tray_layout(layout_path: str, tray_quad_current: np.ndarray) -> Optional[dict]:
    """Project tray holes from template layout to current tray quadrilateral."""
    layout = load_tray_layout(Path(layout_path))
    tray_corners = layout.get("tray_corners_uv", [])
    holes_uv = layout.get("holes_uv", [])
    if len(tray_corners) != 4 or len(holes_uv) != 5:
        return None

    src = order_quad_points(np.array([[p["u"], p["v"]] for p in tray_corners], dtype=np.float32))
    dst = order_quad_points(np.array(tray_quad_current, dtype=np.float32))
    H = cv2.getPerspectiveTransform(src, dst)
    hole_pts = np.array([[[p["u"], p["v"]]] for p in holes_uv], dtype=np.float32)
    projected = cv2.perspectiveTransform(hole_pts, H).reshape(-1, 2)
    return {
        "homography": H,
        "tray_quad": dst.tolist(),
        "projected_holes_uv": projected.tolist(),
        "layout": layout,
    }
