"""Tray hole detection and tray-layout matching helpers."""

from itertools import combinations, permutations
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from vision.tray_layout import load_tray_layout


def detect_tray_holes(
    rgb_image: np.ndarray,
    min_radius_px: int = 10,
    max_radius_px: int = 40,
    min_dist_px: int = 30,
) -> List[dict]:
    """Detect approximately circular tray holes in the current RGB frame."""
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 1.5)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=min_dist_px,
        param1=100,
        param2=18,
        minRadius=min_radius_px,
        maxRadius=max_radius_px,
    )

    if circles is None:
        return []

    holes = []
    for circle in np.round(circles[0, :]).astype(int):
        u, v, r = int(circle[0]), int(circle[1]), int(circle[2])
        holes.append({"center": [float(u), float(v)], "radius_px": float(r)})
    return holes


def snap_pick_to_nearest_hole(
    pick_uv: List[float],
    holes: List[dict],
    max_snap_dist_px: float = 80.0,
) -> Optional[dict]:
    """Snap pick point to nearest detected tray hole if close enough."""
    if not holes:
        return None

    pu, pv = pick_uv
    best = None
    best_dist = None
    for hole in holes:
        hu, hv = hole["center"]
        dist = float(np.hypot(hu - pu, hv - pv))
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best = hole

    if best is None or best_dist is None or best_dist > max_snap_dist_px:
        return None

    return {
        "center": best["center"],
        "radius_px": best["radius_px"],
        "snap_dist_px": best_dist,
    }


def _centers_to_array(holes: List[dict]) -> np.ndarray:
    return np.array([hole["center"] for hole in holes], dtype=np.float32)


def match_tray_layout_to_detected_holes(
    layout_path: str,
    detected_holes: List[dict],
    max_reproj_error_px: float = 30.0,
    max_candidate_holes: int = 8,
) -> Optional[dict]:
    """Match 5-hole tray layout to currently detected holes using similarity transform."""
    layout = load_tray_layout(Path(layout_path))
    layout_holes = layout.get("holes_uv", [])
    if len(layout_holes) != 5 or len(detected_holes) < 5:
        return None

    if max_candidate_holes >= 5 and len(detected_holes) > max_candidate_holes:
        detected_holes = sorted(
            detected_holes,
            key=lambda hole: hole.get("radius_px", 0.0),
            reverse=True,
        )[:max_candidate_holes]

    template_pts = np.array([[hole["u"], hole["v"]] for hole in layout_holes], dtype=np.float32)
    detected_pts_all = _centers_to_array(detected_holes)

    best = None
    best_err = None
    n = len(detected_holes)
    for idxs in combinations(range(n), 5):
        subset = detected_pts_all[list(idxs)]
        for perm in permutations(range(5)):
            dst = subset[list(perm)]
            M, _inliers = cv2.estimateAffinePartial2D(template_pts, dst, method=cv2.LMEDS)
            if M is None:
                continue
            projected = cv2.transform(template_pts.reshape(1, -1, 2), M).reshape(-1, 2)
            err = float(np.mean(np.linalg.norm(projected - dst, axis=1)))
            if err > max_reproj_error_px:
                continue
            if best_err is None or err < best_err:
                matched_detected = [detected_holes[list(idxs)[perm_idx]] for perm_idx in perm]
                best_err = err
                best = {
                    "affine_2x3": M,
                    "projected_holes_uv": projected.tolist(),
                    "matched_detected_holes": matched_detected,
                    "reproj_error_px": err,
                    "layout": layout,
                }
    return best


def assign_pick_to_layout_hole(
    pick_uv: List[float],
    layout_match: dict,
    max_assign_dist_px: float = 100.0,
) -> Optional[dict]:
    """Assign pick point to nearest projected layout hole."""
    if layout_match is None:
        return None

    pu, pv = pick_uv
    best_idx = None
    best_dist = None
    projected = layout_match["projected_holes_uv"]
    for idx, hole_uv in enumerate(projected):
        hu, hv = hole_uv
        dist = float(np.hypot(hu - pu, hv - pv))
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_idx = idx

    if best_idx is None or best_dist is None or best_dist > max_assign_dist_px:
        return None

    hole_meta = layout_match["layout"]["holes_uv"][best_idx]
    return {
        "id": int(hole_meta["id"]),
        "center": projected[best_idx],
        "assign_dist_px": best_dist,
        "reproj_error_px": float(layout_match["reproj_error_px"]),
    }
