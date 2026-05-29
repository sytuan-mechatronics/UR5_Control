"""Tray reference helpers using checkerboard pose for XY correction."""

from typing import List, Optional, Tuple

import cv2
import numpy as np

from vision.calibration import camera_to_base


def build_checkerboard_object_points(inner_corners: Tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = inner_corners
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid
    objp *= square_size_m
    return objp


def detect_checkerboard_pose(
    rgb_image: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    inner_corners: Tuple[int, int],
    square_size_m: float,
) -> Optional[dict]:
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        inner_corners,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    obj_points = build_checkerboard_object_points(inner_corners, square_size_m)
    ok, rvec, tvec = cv2.solvePnP(obj_points, corners_refined, K, dist)
    if not ok:
        return None

    R_target2cam, _ = cv2.Rodrigues(rvec)
    centroid = corners_refined.reshape(-1, 2).mean(axis=0)
    return {
        "rvec": rvec,
        "tvec": tvec.reshape(3),
        "R_target2cam": R_target2cam,
        "corners": corners_refined.reshape(-1, 2),
        "centroid_uv": [float(centroid[0]), float(centroid[1])],
    }


def pixel_to_plane_camera_3d(
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    R_target2cam: np.ndarray,
    t_target2cam: np.ndarray,
) -> Optional[List[float]]:
    """Intersect camera ray with checkerboard plane (Z=0 in target frame)."""
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
    ray /= np.linalg.norm(ray)

    plane_point = t_target2cam.reshape(3)
    plane_normal = R_target2cam[:, 2]
    denom = float(np.dot(plane_normal, ray))
    if abs(denom) < 1e-6:
        return None

    scale = float(np.dot(plane_normal, plane_point) / denom)
    if scale <= 0:
        return None

    point_cam = ray * scale
    return point_cam.tolist()


def refine_base_xy_with_checkerboard(
    rgb_image: np.ndarray,
    u: float,
    v: float,
    p_base_depth: List[float],
    tcp_pose_at_capture: List[float],
    T_cam_to_tcp,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    inner_corners: Tuple[int, int],
    square_size_m: float,
) -> Tuple[List[float], str]:
    tray_ref = detect_checkerboard_pose(
        rgb_image,
        fx,
        fy,
        cx,
        cy,
        inner_corners,
        square_size_m,
    )
    if tray_ref is None:
        return p_base_depth, "depth_only_no_checkerboard"

    return refine_base_xy_with_tray_pose(
        u,
        v,
        p_base_depth,
        tcp_pose_at_capture,
        T_cam_to_tcp,
        fx,
        fy,
        cx,
        cy,
        tray_ref,
    )


def refine_base_xy_with_tray_pose(
    u: float,
    v: float,
    p_base_depth: List[float],
    tcp_pose_at_capture: List[float],
    T_cam_to_tcp,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    tray_ref: dict,
) -> Tuple[List[float], str]:
    """Refine XY using a pre-detected tray reference pose."""

    plane_point_cam = pixel_to_plane_camera_3d(
        u,
        v,
        fx,
        fy,
        cx,
        cy,
        tray_ref["R_target2cam"],
        tray_ref["tvec"],
    )
    if plane_point_cam is None:
        return p_base_depth, "depth_only_bad_plane_intersection"

    plane_base = camera_to_base(plane_point_cam, tcp_pose_at_capture, T_cam_to_tcp)
    refined = [plane_base[0], plane_base[1], p_base_depth[2]]
    return refined, "tray_checkerboard_xy"
