"""
Configuration module for PC2 UR5 control system.
Reads all settings from environment variables with default values.
"""

import os
import json
from pathlib import Path
from typing import List, Tuple
import numpy as np


def _load_local_env() -> None:
    """Load .env from repo root without requiring python-dotenv."""
    env_path = Path(__file__).resolve().parent / ".env"
    if not env_path.exists():
        return

    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            continue
        if value and len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        os.environ[key] = value


_load_local_env()


# ==================== ROBOT CONNECTIVITY ====================

ROBOT_IP = os.getenv("ROBOT_IP", "192.168.125.11")
ROBOT_TYPE = "UR5 CB3"
POLYSCOPE = "3.15.5"  # CB3, RTDE supported
DASHBOARD_PORT = int(os.getenv("DASHBOARD_PORT", "29999"))
URSCRIPT_PORT = int(os.getenv("URSCRIPT_PORT", "30002"))
RTDE_PORT = int(os.getenv("RTDE_PORT", "30004"))


# ==================== FLASK PC2 SERVER ====================

PC2_HOST = os.getenv("PC2_HOST", "0.0.0.0")
PC2_PORT = int(os.getenv("PC2_PORT", "5001"))


# ==================== EXPERIMENT STAGE ====================

_EXPERIMENT_STAGE_RAW = str(os.getenv("EXPERIMENT_STAGE", "1")).strip()
try:
    EXPERIMENT_STAGE = int(_EXPERIMENT_STAGE_RAW)
except ValueError:
    EXPERIMENT_STAGE = 1

if EXPERIMENT_STAGE not in (1, 2, 3):
    EXPERIMENT_STAGE = 1

EXPERIMENT_STAGE_LABELS = {
    1: "static_motion_only",
    2: "motion_plus_vision",
    3: "full_flow_sim_grip",
}


# ==================== PC1 CALLBACK CONFIGURATION ====================

PC1_BASE_URL = os.getenv("PC1_BASE_URL", "http://192.168.1.100:5000")
PC1_UR5_DONE_URL = f"{PC1_BASE_URL}/api/workflow/ur5/done"
PC1_CALLBACK_ENABLED = os.getenv("PC1_CALLBACK_ENABLED", "False").lower() == "true"
PC1_WEBHOOK_SECRET = os.getenv("PC1_WEBHOOK_SECRET", "")


# ==================== MOTION PARAMETERS ====================

# Joint motion (movej)
JOINT_ACCEL = float(os.getenv("JOINT_ACCEL", "1.0"))      # rad/s²
JOINT_VEL = float(os.getenv("JOINT_VEL", "0.8"))          # rad/s

# Linear motion (movel)
LINEAR_ACCEL = float(os.getenv("LINEAR_ACCEL", "0.3"))    # m/s²
LINEAR_VEL = float(os.getenv("LINEAR_VEL", "0.1"))        # m/s
PICK_APPROACH_VEL = float(os.getenv("PICK_APPROACH_VEL", "0.05"))  # m/s (slow when approaching)


def _parse_xyz(env_key: str, default: List[float]) -> List[float]:
    """Parse comma-separated XYZ vector from environment variable."""
    value = os.getenv(env_key)
    if value:
        try:
            parts = [float(x.strip()) for x in value.split(",")]
            if len(parts) == 3:
                return parts
        except ValueError:
            return default
    return default


def _parse_int_pair(env_key: str, default: Tuple[int, int]) -> Tuple[int, int]:
    value = os.getenv(env_key)
    if value:
        try:
            parts = [int(x.strip()) for x in value.split(",")]
            if len(parts) == 2:
                return parts[0], parts[1]
        except ValueError:
            return default
    return default


PAYLOAD_MASS_KG = float(os.getenv("PAYLOAD_MASS_KG", "1.3"))
PAYLOAD_COG = _parse_xyz("PAYLOAD_COG", [0.0, 0.0, 0.10])


# ==================== GEOMETRY & PICKING ====================

PICK_APPROACH_OFFSET_Z = float(os.getenv("PICK_APPROACH_OFFSET_Z", "0.15"))  # m, 150mm above part
PICK_FINAL_OFFSET_Z = float(os.getenv("PICK_FINAL_OFFSET_Z", "0.005"))      # m, 5mm above surface
PICK_RETREAT_OFFSET_Z = float(os.getenv("PICK_RETREAT_OFFSET_Z", "0.15"))   # m, lift up after grip
MAX_TARGET_PLANAR_DIST_M = float(os.getenv("MAX_TARGET_PLANAR_DIST_M", "0.35"))
MAX_TARGET_DZ_DIST_M = float(os.getenv("MAX_TARGET_DZ_DIST_M", "0.35"))
PICK_OFFSET_X = float(os.getenv("PICK_OFFSET_X", "0.0"))                    # m, base-frame fine tune
PICK_OFFSET_Y = float(os.getenv("PICK_OFFSET_Y", "0.0"))                    # m, base-frame fine tune
PICK_OFFSET_Z = float(os.getenv("PICK_OFFSET_Z", "0.0"))                    # m, base-frame fine tune
TRAY_REF_ENABLED = os.getenv("TRAY_REF_ENABLED", "False").lower() == "true"
TRAY_REF_TWO_SHOT = os.getenv("TRAY_REF_TWO_SHOT", "False").lower() == "true"
TRAY_REF_INNER_CORNERS = _parse_int_pair("TRAY_REF_INNER_CORNERS", (6, 9))
TRAY_REF_SQUARE_SIZE_M = float(os.getenv("TRAY_REF_SQUARE_SIZE_M", "0.02"))
TRAY_HOLE_REF_ENABLED = os.getenv("TRAY_HOLE_REF_ENABLED", "False").lower() == "true"
TRAY_LAYOUT_PATH = os.getenv("TRAY_LAYOUT_PATH", "tray_layout.json")
TRAY_HOLE_MIN_RADIUS_PX = int(os.getenv("TRAY_HOLE_MIN_RADIUS_PX", "10"))
TRAY_HOLE_MAX_RADIUS_PX = int(os.getenv("TRAY_HOLE_MAX_RADIUS_PX", "40"))
TRAY_HOLE_MIN_DIST_PX = int(os.getenv("TRAY_HOLE_MIN_DIST_PX", "30"))
TRAY_HOLE_MAX_SNAP_DIST_PX = float(os.getenv("TRAY_HOLE_MAX_SNAP_DIST_PX", "80"))
TRAY_LAYOUT_MAX_REPROJ_ERR_PX = float(os.getenv("TRAY_LAYOUT_MAX_REPROJ_ERR_PX", "30"))
TRAY_LAYOUT_MAX_ASSIGN_DIST_PX = float(os.getenv("TRAY_LAYOUT_MAX_ASSIGN_DIST_PX", "100"))
TRAY_LAYOUT_MAX_CANDIDATE_HOLES = int(os.getenv("TRAY_LAYOUT_MAX_CANDIDATE_HOLES", "8"))


# ==================== GRIPPER ONROBOT RG ====================

GRIPPER_MODEL = "OnRobot RG6 V2"
GRIPPER_URCAP_VERSION = "5.16.0"

# Single switch for hardware/simulation behavior.
# True  -> simulation mode (no physical gripper command)
# False -> real hardware mode
IS_SIMULATION = os.getenv("IS_SIMULATION", "False").lower() == "true"

# Backward-compatible alias used in existing code paths.
GRIPPER_ENABLED = not IS_SIMULATION

GRIPPER_OPEN_WIDTH = int(os.getenv("GRIPPER_OPEN_WIDTH", "140"))      # mm, initial suggestion
GRIPPER_CLOSE_FORCE = int(os.getenv("GRIPPER_CLOSE_FORCE", "40"))     # N
GRIPPER_CLOSE_WIDTH = int(os.getenv("GRIPPER_CLOSE_WIDTH", "0"))      # mm, fully closed
GRIPPER_TIMEOUT_S = float(os.getenv("GRIPPER_TIMEOUT_S", "3.0"))      # s, timeout for grip detection
GRIPPER_MAX_WIDTH_MM = int(os.getenv("GRIPPER_MAX_WIDTH_MM", "160"))   # RG6 default=160mm (RG2=110mm)

# Simulated gripper timing (used when IS_SIMULATION=True)
GRIPPER_SIM_OPEN_DELAY_S = float(os.getenv("GRIPPER_SIM_OPEN_DELAY_S", "0.45"))
GRIPPER_SIM_CLOSE_DELAY_S = float(os.getenv("GRIPPER_SIM_CLOSE_DELAY_S", "0.65"))
GRIPPER_SIM_DETECT_DELAY_S = float(os.getenv("GRIPPER_SIM_DETECT_DELAY_S", "0.25"))

# URScript syntax reference:
#   rg_grip(tool=0, force=<N>, width=<mm>)
#   rg_get_status(tool=0) -> 0/1/2/3
#   rg_get_width(tool=0)  -> actual width (mm)
# CB3 + URSoftware 3.15.5: RTDE supported.

# Grip detection method:
#   "timeout"        — đợi cố định 0.5s rồi coi là OK (đơn giản nhất)
#   "width_feedback" — đọc rg_get_status(tool=0) qua RTDE output register
#                      0=idle, 1=gripping, 2=no_object, 3=object_lost
#   "digital_output" — đọc digital input UR5 từ gripper signal
GRIPPER_GRIP_DETECT_METHOD = os.getenv("GRIPPER_GRIP_DETECT_METHOD", "timeout")
GRIPPER_DIGITAL_OUTPUT_PIN = int(os.getenv("GRIPPER_DIGITAL_OUTPUT_PIN", "0"))  # tool digital input index
GRIPPER_PHOI_DIAMETER_MM = float(os.getenv("GRIPPER_PHOI_DIAMETER_MM", "30"))   # đường kính thực đo
GRIPPER_WIDTH_TOLERANCE_MM = float(os.getenv("GRIPPER_WIDTH_TOLERANCE_MM", "5"))  # sai số chấp nhận


# ==================== RETRY LOGIC ====================

MAX_PICK_RETRIES = int(os.getenv("MAX_PICK_RETRIES", "3"))


# ==================== YOLO DETECTION ====================

YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", "ur5.pt")
YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE", "0.5"))
YOLO_TARGET_CLASS = os.getenv("YOLO_TARGET_CLASS", "phoi")


# ==================== CAMERA PARAMETERS ====================

# Default runtime stream resolution should follow the calibration baseline
# unless the operator explicitly overrides it in .env.
CAMERA_WIDTH = None
CAMERA_HEIGHT = None
DEPTH_HOLE_FILL = os.getenv("DEPTH_HOLE_FILL", "True").lower() == "true"


# ==================== FIXED JOINT POSITIONS (Cartesian teaching) ====================

def _parse_joint_array(env_key: str, default: List[float]) -> List[float]:
    """Parse comma-separated joint values from environment variable."""
    value = os.getenv(env_key)
    if value:
        try:
            return [float(x.strip()) for x in value.split(",")]
        except ValueError:
            return default
    return default


HOME_JOINTS = _parse_joint_array(
    "HOME_JOINTS",
    [-0.000503, -1.016982, -0.477012, -1.754268, 1.530797, -0.000192]
)

SCAN_APPROACH_JOINTS = _parse_joint_array(
    "SCAN_APPROACH_JOINTS",
    [-0.000767, -1.1577, -1.240295, -1.798434, 1.530749, -0.00018]
)

SCAN_POSE_JOINTS = _parse_joint_array(
    "SCAN_POSE_JOINTS",
    [-0.03954, -1.582997, -1.547223, -1.29303, 1.584148, -0.000299]
)


# ==================== FIXED CARTESIAN POSITIONS (Place position) ====================

def _parse_pose_array(env_key: str, default: List[float]) -> List[float]:
    """Parse comma-separated pose values [x,y,z,rx,ry,rz] from environment variable."""
    value = os.getenv(env_key)
    if value:
        try:
            return [float(x.strip()) for x in value.split(",")]
        except ValueError:
            return default
    return default


# TCP pose at SCAN_POSE (measured on robot and used as Cartesian reference).
# If needed, override via env SCAN_POSE_TCP with 6 comma-separated values.
SCAN_POSE_TCP = _parse_pose_array(
    "SCAN_POSE_TCP",
    [0.592493, -0.13308, 0.180722, -2.031953, 2.10491, -0.305354]
)

PICK_APPROACH_CART_STATIC = _parse_pose_array(
    "PICK_APPROACH_CART_STATIC",
    [0.592493, -0.13308, 0.330722, -2.031953, 2.10491, -0.305354]
)


PLACE_APPROACH_CART = _parse_pose_array(
    "PLACE_APPROACH_CART",
    [-0.061814, 0.640666, 0.129714, -2.852838, -0.392402, 0.049224]
)

PLACE_POINT_CART = _parse_pose_array(
    "PLACE_POINT_CART",
    [-0.062042, 0.63978, 0.06777, -2.952537, -0.406208, 0.03285]
)

PLACE_RETREAT_CART = _parse_pose_array(
    "PLACE_RETREAT_CART",
    [-0.061814, 0.640666, 0.129714, -2.852838, -0.392402, 0.049224]
)


# ==================== TOOL ORIENTATION ====================

# Lấy từ tcp_m_rad của SCAN_POSE thực tế (robot_poses.json)
# Dùng trong build_pick_approach_pose() để set orientation gripper
# khi di chuyển đến pick point tính từ camera
TOOL_DOWN_RX = float(os.getenv("TOOL_DOWN_RX", "-2.031953"))
TOOL_DOWN_RY = float(os.getenv("TOOL_DOWN_RY", "2.10491"))
TOOL_DOWN_RZ = float(os.getenv("TOOL_DOWN_RZ", "-0.305354"))


# ==================== HAND-EYE CALIBRATION ====================

def _parse_homogeneous_matrix(env_key: str, default: np.ndarray) -> np.ndarray:
    """Parse 4x4 homogeneous matrix from environment variable (16 comma-separated values)."""
    value = os.getenv(env_key)
    if value:
        try:
            elements = [float(x.strip()) for x in value.split(",")]
            if len(elements) == 16:
                return np.array(elements).reshape(4, 4)
        except ValueError:
            pass
    return default


# Hand-eye calibration result (method=PARK, recollected after camera/tool remount on 2026-05-29)
# Previous matrix became invalid after mechanical remove/reinstall.
_DEFAULT_T_CAM_TO_TCP = np.array([
    [ 0.959769,  0.229463,  0.161836,  0.079477],
    [-0.252183,  0.957871,  0.137430, -0.134596],
    [-0.123483, -0.172714,  0.977201, -0.138800],
    [ 0.0,       0.0,       0.0,       1.0      ],
], dtype=np.float64)

T_CAM_TO_TCP = _parse_homogeneous_matrix("T_CAM_TO_TCP", _DEFAULT_T_CAM_TO_TCP)


# ==================== CAMERA INTRINSICS (Orbbec Femto Mega) ====================

def _load_camera_intrinsics_defaults() -> dict:
    """Load camera intrinsics defaults from camera_intrinsics.json if available."""
    defaults = {
        "fx": 1114.278564453125,
        "fy": 1114.118408203125,
        "cx": 937.609375,
        "cy": 518.2891845703125,
        "width": 1280.0,
        "height": 720.0,
    }
    intrinsics_path = Path(__file__).resolve().parent / "camera_intrinsics.json"
    if not intrinsics_path.exists():
        return defaults

    try:
        with intrinsics_path.open("r", encoding="utf-8") as file_obj:
            data = json.load(file_obj)
        for key in ("fx", "fy", "cx", "cy", "width", "height"):
            if key in data:
                defaults[key] = float(data[key])
    except Exception:
        pass

    return defaults


_CAM_INTRINSICS_DEFAULTS = _load_camera_intrinsics_defaults()

if CAMERA_WIDTH is None:
    CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", str(int(_CAM_INTRINSICS_DEFAULTS["width"]))))
if CAMERA_HEIGHT is None:
    CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", str(int(_CAM_INTRINSICS_DEFAULTS["height"]))))

CAM_FX = float(os.getenv("CAM_FX", str(_CAM_INTRINSICS_DEFAULTS["fx"])))    # focal length X
CAM_FY = float(os.getenv("CAM_FY", str(_CAM_INTRINSICS_DEFAULTS["fy"])))    # focal length Y
CAM_CX = float(os.getenv("CAM_CX", str(_CAM_INTRINSICS_DEFAULTS["cx"])))    # principal point X
CAM_CY = float(os.getenv("CAM_CY", str(_CAM_INTRINSICS_DEFAULTS["cy"])))    # principal point Y
CAM_CALIB_WIDTH = float(os.getenv("CAM_CALIB_WIDTH", str(_CAM_INTRINSICS_DEFAULTS["width"])))
CAM_CALIB_HEIGHT = float(os.getenv("CAM_CALIB_HEIGHT", str(_CAM_INTRINSICS_DEFAULTS["height"])))


# ==================== TIMEOUT & SAFETY ====================

RTDE_FREQUENCY = float(os.getenv("RTDE_FREQUENCY", "10.0"))  # Hz
RTDE_STEADY_THRESHOLD = float(os.getenv("RTDE_STEADY_THRESHOLD", "0.001"))  # rad/s
RTDE_WAIT_TIMEOUT = float(os.getenv("RTDE_WAIT_TIMEOUT", "30.0"))  # s
# Timeout đợi robot bắt đầu di chuyển sau khi gửi lệnh
# CB-series có độ trễ ~0.1-0.5s trước khi joint_speed > 0
RTDE_MOTION_START_TIMEOUT = float(os.getenv("RTDE_MOTION_START_TIMEOUT", "2.0"))  # s
RTDE_MOTION_START_THRESHOLD = float(os.getenv("RTDE_MOTION_START_THRESHOLD", "0.005"))  # rad/s

SOCKET_TIMEOUT = float(os.getenv("SOCKET_TIMEOUT", "5.0"))  # s
URSCRIPT_TIMEOUT = float(os.getenv("URSCRIPT_TIMEOUT", "10.0"))  # s


# ==================== LOGGING ====================

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
