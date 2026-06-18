"""
Camera to Robot Calibration and Coordinate Transformation.
Converts pixel coordinates to robot base frame using hand-eye calibration.
"""

import logging
import numpy as np
from typing import List, Tuple

import config


logger = logging.getLogger(__name__)


def pixel_to_camera_3d(
    u: float,
    v: float,
    depth_mm: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float
) -> List[float]:
    """
    Deproject pixel coordinate to 3D camera frame.
    
    Args:
        u, v: Pixel coordinates
        depth_mm: Depth value in mm
        fx, fy: Focal lengths in pixels
        cx, cy: Principal point in pixels
        
    Returns:
        [X_cam, Y_cam, Z_cam] in meters
    """
    depth_m = depth_mm / 1000.0
    
    x_cam = (u - cx) * depth_m / fx
    y_cam = (v - cy) * depth_m / fy
    z_cam = depth_m
    
    logger.debug(f"Pixel ({u:.0f}, {v:.0f}) @ {depth_mm:.1f}mm -> 3D ({x_cam:.4f}, {y_cam:.4f}, {z_cam:.4f})m")
    
    return [x_cam, y_cam, z_cam]


def axis_angle_to_rotation_matrix(
    rx: float,
    ry: float,
    rz: float
) -> np.ndarray:
    """
    Convert UR5 axis-angle representation to 3x3 rotation matrix.
    
    Uses Rodrigues' rotation formula.
    axis_angle = [rx, ry, rz], where magnitude is rotation angle
    and direction is rotation axis.
    
    Args:
        rx, ry, rz: Axis-angle components in radians
        
    Returns:
        3x3 rotation matrix
    """
    angle = np.sqrt(rx**2 + ry**2 + rz**2)
    
    if angle < 1e-6:
        # Small angle, use first-order approximation
        return np.eye(3)
    
    # Normalized axis
    axis = np.array([rx, ry, rz]) / angle
    
    # Rodrigues formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    
    return R


def tcp_pose_to_transform(tcp_pose: List[float]) -> np.ndarray:
    """
    Convert UR5 TCP pose to 4x4 homogeneous transform matrix.
    
    Args:
        tcp_pose: [x, y, z, rx, ry, rz] in meters and radians
        
    Returns:
        4x4 transform matrix T_base_to_tcp
    """
    x, y, z, rx, ry, rz = tcp_pose
    
    # Position vector
    position = np.array([x, y, z])
    
    # Rotation matrix from axis-angle
    R = axis_angle_to_rotation_matrix(rx, ry, rz)
    
    # Construct 4x4 transform
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = position
    
    logger.debug(f"TCP pose -> 4x4 transform")
    
    return T


def camera_to_base(
    point_cam_3d: List[float],
    tcp_pose_at_capture: List[float],
    T_cam_to_tcp: np.ndarray
) -> List[float]:
    """
    Transform 3D point from camera frame to robot base frame.
    
    Transformation chain:
        point_base = T_base_tcp @ T_tcp_cam @ point_cam
    
    Args:
        point_cam_3d: [X_cam, Y_cam, Z_cam] in camera frame (meters)
        tcp_pose_at_capture: TCP pose [x, y, z, rx, ry, rz] when photo was taken
        T_cam_to_tcp: 4x4 hand-eye calibration matrix (T_tcp_cam)
        
    Returns:
        [X_base, Y_base, Z_base] in robot base frame (meters)
    """
    # Convert point to homogeneous coordinates
    point_cam_h = np.array([point_cam_3d[0], point_cam_3d[1], point_cam_3d[2], 1.0])
    
    # Step 1: Camera -> TCP
    point_tcp_h = T_cam_to_tcp @ point_cam_h
    
    # Step 2: TCP -> Base
    T_base_tcp = tcp_pose_to_transform(tcp_pose_at_capture)
    point_base_h = T_base_tcp @ point_tcp_h
    
    # Extract position
    point_base = [point_base_h[0], point_base_h[1], point_base_h[2]]
    
    logger.debug(
        f"Camera 3D {point_cam_3d} -> Base {point_base}"
    )
    
    return point_base


def camera_origin_to_base(
    tcp_pose_at_capture: List[float],
    T_cam_to_tcp: np.ndarray,
) -> List[float]:
    """Return camera origin in base frame for the current capture pose."""
    return camera_to_base([0.0, 0.0, 0.0], tcp_pose_at_capture, T_cam_to_tcp)


def min_safe_camera_depth_m(
    T_cam_to_tcp: np.ndarray,
    margin_below_tcp_m: float = 0.02,
) -> float:
    """
    Compute the minimum camera depth that still places the target below TCP.

    Why this matters:
    - Camera is mounted above the TCP and looks downward.
    - If measured depth is shorter than the camera->TCP standoff along optical Z,
      the estimated target will land above TCP and robot may move upward.

    Args:
        T_cam_to_tcp: 4x4 hand-eye matrix mapping camera frame -> TCP frame.
        margin_below_tcp_m: extra distance to keep target below TCP.

    Returns:
        Minimum valid depth in meters along camera optical axis.
    """
    T_tcp_to_cam = np.linalg.inv(T_cam_to_tcp)
    tcp_origin_in_cam = T_tcp_to_cam @ np.array([0.0, 0.0, 0.0, 1.0])
    tcp_depth_in_cam = float(tcp_origin_in_cam[2])
    return tcp_depth_in_cam + margin_below_tcp_m


def sanitize_camera_depth_mm(
    depth_mm: float,
    T_cam_to_tcp: np.ndarray,
    margin_below_tcp_m: float = 0.02,
) -> Tuple[float, bool, float]:
    """
    Clamp depth if it would place the target above TCP.

    Returns:
        (sanitized_depth_mm, was_clamped, min_safe_depth_mm)
    """
    if not config.DEPTH_TCP_STANDOFF_CLAMP_ENABLED:
        return depth_mm, False, 0.0
    min_safe_depth_mm = min_safe_camera_depth_m(
        T_cam_to_tcp,
        margin_below_tcp_m=margin_below_tcp_m,
    ) * 1000.0
    if depth_mm < min_safe_depth_mm:
        return min_safe_depth_mm, True, min_safe_depth_mm
    return depth_mm, False, min_safe_depth_mm


def build_pick_approach_pose(
    point_base: List[float],
    offset_z: float,
    tool_rx: float = -2.04842,
    tool_ry: float = -2.026713,
    tool_rz: float = 0.31989
) -> List[float]:
    """
    Build TCP pose for picking approach.
    
    Args:
        point_base: [X, Y, Z] in base frame
        offset_z: Z offset above point (positive = higher)
        tool_rx, tool_ry, tool_rz: Tool orientation (axis-angle)
        
    Returns:
        [x, y, z, rx, ry, rz] pose for movel command
    """
    pose = [
        point_base[0],
        point_base[1],
        point_base[2] + offset_z,
        tool_rx,
        tool_ry,
        tool_rz
    ]
    
    logger.debug(f"Pick approach pose: {pose}")
    
    return pose


def estimate_gripper_opening_width(
    object_width_pixels: float,
    depth_mm: float,
    focal_length_pixels: float,
    gripper_width_range: Tuple[int, int] = (0, 110)
) -> int:
    """
    Estimate required gripper opening width based on object size in image.
    
    This is a heuristic; actual gripper opening should be determined by:
    1. Manual measurement of object
    2. Close-loop feedback from gripper depth or force
    
    Args:
        object_width_pixels: Object width in image (pixels)
        depth_mm: Object depth from camera (mm)
        focal_length_pixels: Camera focal length (pixels)
        gripper_width_range: (min_mm, max_mm) gripper opening range
        
    Returns:
        Estimated gripper opening width (mm)
    """
    # Project object width to 3D
    depth_m = depth_mm / 1000.0
    object_width_m = (object_width_pixels * depth_m) / focal_length_pixels
    object_width_mm = object_width_m * 1000.0
    
    # Add safety margin
    gripper_opening = object_width_mm + 10  # 10mm margin
    
    # Clamp to gripper range
    min_opening, max_opening = gripper_width_range
    gripper_opening = max(min_opening, min(max_opening, gripper_opening))
    
    logger.info(
        f"Estimated gripper opening: {gripper_opening:.0f}mm "
        f"(object: {object_width_mm:.0f}mm)"
    )
    
    return int(gripper_opening)


def compute_hand_eye_calibration_error(
    detected_points_cam: List[List[float]],
    known_points_base: List[List[float]],
    tcp_poses_at_capture: List[List[float]],
    T_cam_to_tcp_estimate: np.ndarray
) -> float:
    """
    Compute hand-eye calibration error (for validation).
    
    Args:
        detected_points_cam: List of [X, Y, Z] in camera frame
        known_points_base: List of ground-truth [X, Y, Z] in base frame
        tcp_poses_at_capture: List of TCP poses when each point was captured
        T_cam_to_tcp_estimate: Estimated hand-eye calibration matrix
        
    Returns:
        Mean reprojection error in meters
    """
    if len(detected_points_cam) != len(known_points_base):
        raise ValueError("Mismatched point lists")
    
    errors = []
    
    for i, (p_cam, p_base_true, tcp_pose) in enumerate(
        zip(detected_points_cam, known_points_base, tcp_poses_at_capture)
    ):
        # Transform detected point
        p_base_estimated = camera_to_base(p_cam, tcp_pose, T_cam_to_tcp_estimate)
        
        # Calculate error
        error = np.linalg.norm(np.array(p_base_estimated) - np.array(p_base_true))
        errors.append(error)
        
        logger.debug(
            f"Point {i}: error = {error:.4f}m, "
            f"true = {p_base_true}, estimated = {p_base_estimated}"
        )
    
    mean_error = np.mean(errors)
    logger.info(f"Mean hand-eye calibration error: {mean_error:.6f}m")
    
    return mean_error
