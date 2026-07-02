"""
Camera to Robot Calibration and Coordinate Transformation.
Converts pixel coordinates to robot base frame using hand-eye calibration.
"""

import logging
import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple

import config


logger = logging.getLogger(__name__)
_PICK_CORRECTION_CACHE = {
    "path": None,
    "mtime_ns": None,
    "data": None,
}


def normalize_slot_name(name: str) -> str:
    """Normalize slot aliases like slot4 -> slot_4."""
    text = str(name or "").strip().lower()
    if not text:
        return ""
    if text.startswith("slot") and len(text) > 4 and text[4:].isdigit():
        return f"slot_{text[4:]}"
    return text


def _pick_correction_file() -> Path:
    raw_path = str(getattr(config, "PICK_CORRECTION_MAP_PATH", "pick_correction_map.json")).strip()
    path = Path(raw_path)
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[1] / raw_path
    return path


def _load_pick_correction_map() -> Dict:
    path = _pick_correction_file()
    if not path.exists():
        return {"enabled": False, "reason": f"missing:{path.name}", "points": []}

    stat = path.stat()
    cache_key = str(path)
    if (
        _PICK_CORRECTION_CACHE["path"] == cache_key
        and _PICK_CORRECTION_CACHE["mtime_ns"] == stat.st_mtime_ns
        and _PICK_CORRECTION_CACHE["data"] is not None
    ):
        return _PICK_CORRECTION_CACHE["data"]

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        data = {"enabled": False, "reason": f"json_error:{exc}", "points": []}
        _PICK_CORRECTION_CACHE.update({"path": cache_key, "mtime_ns": stat.st_mtime_ns, "data": data})
        return data

    parsed_points = []
    for idx, point in enumerate(raw.get("points", []), start=1):
        try:
            parsed_points.append(
                {
                    "name": normalize_slot_name(point.get("name", f"P{idx}")),
                    "x": float(point["x"]),
                    "y": float(point["y"]),
                    "dx": float(point.get("dx", 0.0)),
                    "dy": float(point.get("dy", 0.0)),
                    "dz": float(point.get("dz", 0.0)),
                    "u": float(point["u"]) if "u" in point else None,
                    "v": float(point["v"]) if "v" in point else None,
                }
            )
        except Exception:
            continue

    data = {
        "enabled": bool(parsed_points),
        "reason": "ok" if parsed_points else "no_points",
        "points": parsed_points,
        "path": str(path),
        "meta": raw.get("meta", {}),
    }
    _PICK_CORRECTION_CACHE.update({"path": cache_key, "mtime_ns": stat.st_mtime_ns, "data": data})
    return data


def compute_pick_correction_offset(
    p_base_raw: List[float],
    forced_slot: str = "",
    pick_uv: List[float] = None,
) -> Tuple[List[float], Dict[str, object]]:
    """
    Compute local XY/Z correction from a sparse workspace calibration map.

    The correction map is defined in robot base frame using raw transformed target
    coordinates before global PICK_OFFSET_X/Y/Z is applied.
    """
    global_offset = [
        float(config.PICK_OFFSET_X),
        float(config.PICK_OFFSET_Y),
        float(config.PICK_OFFSET_Z),
    ]
    meta = {
        "enabled": bool(getattr(config, "PICK_CORRECTION_ENABLED", False)),
        "mode": "global_only",
        "path": "",
        "point_count": 0,
        "forced_slot": normalize_slot_name(forced_slot or ""),
        "pick_uv": [float(pick_uv[0]), float(pick_uv[1])] if pick_uv is not None else [],
        "local_offset": [0.0, 0.0, 0.0],
        "global_offset": global_offset,
        "final_offset": global_offset[:],
        "used_points": [],
    }

    if not getattr(config, "PICK_CORRECTION_ENABLED", False):
        meta["mode"] = "disabled"
        return global_offset, meta

    correction_map = _load_pick_correction_map()
    meta["path"] = correction_map.get("path", "")
    points = correction_map.get("points", [])
    meta["point_count"] = len(points)

    if not correction_map.get("enabled", False):
        meta["mode"] = correction_map.get("reason", "no_points")
        return global_offset, meta

    px = float(p_base_raw[0])
    py = float(p_base_raw[1])
    exact_tol_m = float(config.PICK_CORRECTION_EXACT_MM) / 1000.0
    max_radius_m = max(0.0, float(config.PICK_CORRECTION_MAX_RADIUS_MM) / 1000.0)
    max_pixel_dist = max(0.0, float(getattr(config, "PICK_CORRECTION_PIXEL_MAX_DIST_PX", 0.0)))
    neighbor_count = max(1, int(config.PICK_CORRECTION_NEIGHBORS))
    power = max(0.1, float(config.PICK_CORRECTION_POWER))
    strategy = str(getattr(config, "PICK_CORRECTION_STRATEGY", "pixel_slot")).strip().lower()

    scored = []
    for point in points:
        dist = float(np.hypot(px - point["x"], py - point["y"]))
        scored.append((dist, point))

    scored.sort(key=lambda item: item[0])

    forced_slot = normalize_slot_name(forced_slot)
    if forced_slot:
        forced_point = next((point for point in points if point["name"] == forced_slot), None)
        if forced_point is None:
            meta["mode"] = "global_only_forced_slot_missing"
            return global_offset, meta

        forced_dist = float(np.hypot(px - forced_point["x"], py - forced_point["y"]))
        if max_radius_m > 0.0 and forced_dist > max_radius_m:
            meta["mode"] = "global_only_forced_slot_outside_radius"
            meta["used_points"] = [{"name": forced_point["name"], "dist_mm": round(forced_dist * 1000.0, 3)}]
            return global_offset, meta

        local_offset = [forced_point["dx"], forced_point["dy"], forced_point.get("dz", 0.0)]
        final_offset = [global_offset[i] + local_offset[i] for i in range(3)]
        meta.update(
            {
                "mode": "forced_slot",
                "selected_slot": forced_point["name"],
                "local_offset": local_offset,
                "final_offset": final_offset,
                "used_points": [{"name": forced_point["name"], "dist_mm": round(forced_dist * 1000.0, 3)}],
            }
        )
        return final_offset, meta

    if strategy == "pixel_slot" and pick_uv is not None:
        pu = float(pick_uv[0])
        pv = float(pick_uv[1])
        pixel_scored = []
        for point in points:
            if point.get("u") is None or point.get("v") is None:
                continue
            pixel_dist = float(np.hypot(pu - point["u"], pv - point["v"]))
            pixel_scored.append((pixel_dist, point))

        pixel_scored.sort(key=lambda item: item[0])
        if pixel_scored:
            nearest_dist_px, nearest_point = pixel_scored[0]
            if max_pixel_dist > 0.0 and nearest_dist_px > max_pixel_dist:
                meta["mode"] = "global_only_pixel_outside_radius"
                meta["used_points"] = [{"name": nearest_point["name"], "dist_px": round(nearest_dist_px, 3)}]
                return global_offset, meta

            local_offset = [
                nearest_point["dx"],
                nearest_point["dy"],
                nearest_point.get("dz", 0.0),
            ]
            final_offset = [global_offset[i] + local_offset[i] for i in range(3)]
            meta.update(
                {
                    "mode": "pixel_slot",
                    "selected_slot": nearest_point["name"],
                    "local_offset": local_offset,
                    "final_offset": final_offset,
                    "used_points": [{"name": nearest_point["name"], "dist_px": round(nearest_dist_px, 3)}],
                }
            )
            return final_offset, meta

    if scored and scored[0][0] <= exact_tol_m:
        point = scored[0][1]
        local_offset = [point["dx"], point["dy"], point.get("dz", 0.0)]
        final_offset = [global_offset[i] + local_offset[i] for i in range(3)]
        meta.update(
            {
                "mode": "exact",
                "local_offset": local_offset,
                "final_offset": final_offset,
                "used_points": [{"name": point["name"], "dist_mm": round(scored[0][0] * 1000.0, 3)}],
            }
        )
        return final_offset, meta

    if strategy in ("nearest", "slot_only"):
        if not scored:
            meta["mode"] = "global_only_no_points"
            return global_offset, meta

        nearest_dist, nearest_point = scored[0]
        if max_radius_m > 0.0 and nearest_dist > max_radius_m:
            meta["mode"] = "global_only_outside_radius"
            return global_offset, meta

        local_offset = [
            nearest_point["dx"],
            nearest_point["dy"],
            nearest_point.get("dz", 0.0),
        ]
        final_offset = [global_offset[i] + local_offset[i] for i in range(3)]
        meta.update(
            {
                "mode": "slot_only",
                "local_offset": local_offset,
                "final_offset": final_offset,
                "selected_slot": nearest_point["name"],
                "used_points": [{"name": nearest_point["name"], "dist_mm": round(nearest_dist * 1000.0, 3)}],
            }
        )
        return final_offset, meta

    nearby = scored[:neighbor_count]
    if max_radius_m > 0.0:
        nearby = [item for item in nearby if item[0] <= max_radius_m]

    if not nearby:
        meta["mode"] = "global_only_no_neighbors"
        return global_offset, meta

    weights = []
    for dist, point in nearby:
        weight = 1.0 / max(dist, 1e-6) ** power
        weights.append((weight, point, dist))

    weight_sum = sum(weight for weight, _, _ in weights)
    local_offset = [0.0, 0.0, 0.0]
    used_points = []
    for weight, point, dist in weights:
        ratio = weight / weight_sum
        local_offset[0] += point["dx"] * ratio
        local_offset[1] += point["dy"] * ratio
        local_offset[2] += point.get("dz", 0.0) * ratio
        used_points.append(
            {
                "name": point["name"],
                "dist_mm": round(dist * 1000.0, 3),
                "weight": round(ratio, 4),
            }
        )

    final_offset = [global_offset[i] + local_offset[i] for i in range(3)]
    meta.update(
        {
            "mode": "idw",
            "local_offset": local_offset,
            "final_offset": final_offset,
            "used_points": used_points,
        }
    )
    return final_offset, meta


def apply_pick_correction(
    p_base_raw: List[float],
    forced_slot: str = "",
    pick_uv: List[float] = None,
) -> Tuple[List[float], Dict[str, object]]:
    """Apply global pick offset plus optional local correction-map offset."""
    final_offset, meta = compute_pick_correction_offset(
        p_base_raw,
        forced_slot=forced_slot,
        pick_uv=pick_uv,
    )
    p_base = [float(p_base_raw[i]) + float(final_offset[i]) for i in range(3)]
    return p_base, meta


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


def resolve_intrinsics_for_frame(
    frame_w: int,
    frame_h: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    calib_w: float,
    calib_h: float,
) -> Dict[str, float]:
    """
    Scale camera intrinsics to the current frame size, while protecting against
    stale calibration width/height metadata.
    """
    sx = frame_w / float(calib_w)
    sy = frame_h / float(calib_h)

    fx_scaled = fx * sx
    fy_scaled = fy * sy
    cx_scaled = cx * sx
    cy_scaled = cy * sy

    frame_cx = frame_w / 2.0
    frame_cy = frame_h / 2.0
    raw_center_err = abs(cx - frame_cx) + abs(cy - frame_cy)
    scaled_center_err = abs(cx_scaled - frame_cx) + abs(cy_scaled - frame_cy)

    if (
        (abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3)
        and raw_center_err + 5.0 < scaled_center_err
    ):
        return {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "sx": sx,
            "sy": sy,
            "used_scale": False,
            "reason": (
                "Metadata baseline intrinsics co ve da stale: "
                "cx/cy goc da hop ly hon so voi tam frame hien tai, nen bo qua auto-scale."
            ),
        }

    return {
        "fx": fx_scaled,
        "fy": fy_scaled,
        "cx": cx_scaled,
        "cy": cy_scaled,
        "sx": sx,
        "sy": sy,
        "used_scale": abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3,
        "reason": "",
    }


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


def clamp_pick_z_sequence(
    scan_z: float,
    point_z: float,
    approach_offset_z: float,
    touch_offset_z: float,
    retreat_offset_z: float,
    min_descent_mm: float = 5.0,
) -> Tuple[float, float, float]:
    """
    Clamp Z sequence so the robot does not rise above the current scan height
    before moving toward the target.
    """
    min_descent_m = min_descent_mm / 1000.0
    max_working_z = scan_z - min_descent_m
    touch_z = point_z + touch_offset_z
    approach_z = min(point_z + approach_offset_z, max_working_z)
    retreat_z = min(point_z + retreat_offset_z, max_working_z)
    return approach_z, touch_z, retreat_z


def build_lateral_pre_approach_pose(
    point_base: List[float],
    tcp_pose_at_capture: List[float],
    approach_offset_z: float,
    tool_rx: float = -2.04842,
    tool_ry: float = -2.026713,
    tool_rz: float = 0.31989,
) -> List[float]:
    """
    Build a pre-approach pose that keeps the current scan height while moving
    laterally above the target XY.
    """
    del approach_offset_z
    return [
        point_base[0],
        point_base[1],
        tcp_pose_at_capture[2],
        tool_rx,
        tool_ry,
        tool_rz,
    ]


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
