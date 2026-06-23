"""
Configuration module for PC2 UR5 control system.
Reads all settings from environment variables with default values.
"""

import os
import json
from pathlib import Path
from typing import List, Tuple
import numpy as np

try:
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover - optional at runtime
    load_dotenv = None


_CONFIG_DIR = Path(__file__).resolve().parent
if load_dotenv is not None:
    load_dotenv(_CONFIG_DIR / ".env")


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


def _parse_vector3(env_key: str, default):
    """Parse 3 comma-separated floats from environment, else keep default."""
    value = os.getenv(env_key)
    if not value:
        return list(default)
    try:
        parts = [float(x.strip()) for x in value.split(",")]
    except ValueError:
        return list(default)
    if len(parts) != 3:
        return list(default)
    return parts


# Payload/CoG used by external URScript motion commands.
# Default CoG is an estimate for a long tool assembly and should be refined on robot.
PAYLOAD_MASS_KG = float(os.getenv("PAYLOAD_MASS_KG", "1.0"))
PAYLOAD_COG = _parse_vector3("PAYLOAD_COG", [0.0, 0.0, 0.16])
SKIP_SET_TCP = os.getenv("SKIP_SET_TCP", "False").lower() == "true"
SKIP_SET_PAYLOAD = os.getenv("SKIP_SET_PAYLOAD", "False").lower() == "true"
TCP_OFFSET = [
    float(os.getenv("TCP_OFFSET_X_M", "-0.00115")),
    float(os.getenv("TCP_OFFSET_Y_M", "0.00987")),
    float(os.getenv("TCP_OFFSET_Z_M", "0.31535")),
    float(os.getenv("TCP_OFFSET_RX_RAD", "0.0185")),
    float(os.getenv("TCP_OFFSET_RY_RAD", "-0.0294")),
    float(os.getenv("TCP_OFFSET_RZ_RAD", "3.1303")),
]


# ==================== GEOMETRY & PICKING ====================

PICK_APPROACH_OFFSET_Z = float(os.getenv("PICK_APPROACH_OFFSET_Z", "0.15"))  # m, 150mm above part
PICK_FINAL_OFFSET_Z = float(os.getenv("PICK_FINAL_OFFSET_Z", "0.005"))      # m, 5mm above surface
PICK_RETREAT_OFFSET_Z = float(os.getenv("PICK_RETREAT_OFFSET_Z", "0.15"))   # m, lift up after grip
# Bù trừ vật lý tay kẹp khí nén: hạ thêm từ đỉnh phôi xuống giữa thân để ôm chắc.
# Giá trị âm = hạ xuống. Calibrate từng bước, bắt đầu với -0.025.
GRASP_Z_OFFSET = float(os.getenv("GRASP_Z_OFFSET", "-0.025"))
PICK_MAX_PLANAR_DELTA_M = float(os.getenv("PICK_MAX_PLANAR_DELTA_M", "0.25"))
PICK_MAX_APPROACH_LIFT_M = float(os.getenv("PICK_MAX_APPROACH_LIFT_M", "0.03"))
PICK_MAX_FINAL_Z_ABOVE_CAPTURE_M = float(os.getenv("PICK_MAX_FINAL_Z_ABOVE_CAPTURE_M", "0.005"))
PICK_MAX_FINAL_Z_ABOVE_SCAN_M = float(os.getenv("PICK_MAX_FINAL_Z_ABOVE_SCAN_M", "0.005"))
PICK_MIN_DESCENT_M = float(os.getenv("PICK_MIN_DESCENT_M", "0.02"))
PICK_MIN_FINAL_BELOW_CAMERA_M = float(os.getenv("PICK_MIN_FINAL_BELOW_CAMERA_M", "0.02"))
PICK_OFFSET_X = float(os.getenv("PICK_OFFSET_X", "0.066"))
PICK_OFFSET_Y = float(os.getenv("PICK_OFFSET_Y", "0.080"))
PICK_OFFSET_Z = float(os.getenv("PICK_OFFSET_Z", "-0.088"))
PICK_CORRECTION_ENABLED = os.getenv("PICK_CORRECTION_ENABLED", "False").lower() == "true"
PICK_CORRECTION_MAP_PATH = os.getenv("PICK_CORRECTION_MAP_PATH", "pick_correction_map.json").strip()
PICK_CORRECTION_STRATEGY = os.getenv("PICK_CORRECTION_STRATEGY", "pixel_slot").strip().lower()
if PICK_CORRECTION_STRATEGY not in ("pixel_slot", "slot_only", "nearest", "idw"):
    PICK_CORRECTION_STRATEGY = "pixel_slot"
PICK_CORRECTION_NEIGHBORS = int(os.getenv("PICK_CORRECTION_NEIGHBORS", "4"))
PICK_CORRECTION_POWER = float(os.getenv("PICK_CORRECTION_POWER", "2.0"))
PICK_CORRECTION_EXACT_MM = float(os.getenv("PICK_CORRECTION_EXACT_MM", "3.0"))
PICK_CORRECTION_MAX_RADIUS_MM = float(os.getenv("PICK_CORRECTION_MAX_RADIUS_MM", "180.0"))
PICK_CORRECTION_PIXEL_MAX_DIST_PX = float(os.getenv("PICK_CORRECTION_PIXEL_MAX_DIST_PX", "180.0"))


# ==================== GRIPPER ONROBOT RG ====================

GRIPPER_MODEL = "OnRobot RG6 V2"
GRIPPER_URCAP_VERSION = "5.16.0"

# Single switch for hardware/simulation behavior.
# True  -> simulation mode (no physical gripper command)
# False -> real hardware mode
IS_SIMULATION = os.getenv("IS_SIMULATION", "False").lower() == "true"

# Backward-compatible alias used in existing code paths.
GRIPPER_ENABLED = not IS_SIMULATION

# REFACTORED: Pneumatic gripper serial configuration
GRIPPER_PORT = os.getenv("GRIPPER_PORT", "/dev/gripper")
GRIPPER_BAUD = int(os.getenv("GRIPPER_BAUD", "9600"))
GRIPPER_CMD_TIMEOUT_S = float(os.getenv("GRIPPER_CMD_TIMEOUT_S", "3.0"))
GRIPPER_SETTLE_S = float(os.getenv("GRIPPER_SETTLE_S", "0.5"))
# Settle time when opening gripper (5/2 valve reverse + cylinder exhaust complete)
GRIPPER_RELEASE_SETTLE_S = float(os.getenv("GRIPPER_RELEASE_SETTLE_S", "0.3"))
GRIPPER_HEARTBEAT_S = float(os.getenv("GRIPPER_HEARTBEAT_S", "3.0"))

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
# Max outer pick cycles (safety ceiling — loop normally ends when tray is empty).
MAX_PICK_CYCLES = int(os.getenv("MAX_PICK_CYCLES", "20"))
# Extra sleep after wait_steady() before flushing camera and capturing scan frame.
# Gives the camera pipeline time to buffer frames taken when robot was stationary.
SCAN_SETTLE_SLEEP_S = float(os.getenv("SCAN_SETTLE_SLEEP_S", "0.3"))
# Pixel radius around a successfully picked UV to exclude from future scans.
# Prevents re-picking the same empty slot due to residual detection.
PICKED_EXCLUSION_RADIUS_PX = int(os.getenv("PICKED_EXCLUSION_RADIUS_PX", "100"))


# ==================== YOLO DETECTION ====================

YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", "models/phoi.pt")
YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE", "0.5"))
YOLO_TARGET_CLASS = os.getenv("YOLO_TARGET_CLASS", "phoi")


# ==================== CAMERA PARAMETERS ====================

CAMERA_TRANSPORT = os.getenv("CAMERA_TRANSPORT", "auto").strip().lower()
if CAMERA_TRANSPORT not in ("auto", "usb", "lan"):
    CAMERA_TRANSPORT = "auto"
CAMERA_IP = os.getenv("CAMERA_IP", "").strip()
CAMERA_NET_PORT = int(os.getenv("CAMERA_NET_PORT", "8090"))
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "1920"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "1080"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))
CAMERA_LAN_COLOR_FORMATS = [
    x.strip().upper()
    for x in os.getenv("CAMERA_LAN_COLOR_FORMATS", "MJPG,BGR,RGB,YUYV,NV12,NV21,I420").split(",")
    if x.strip()
]
CAMERA_USB_COLOR_FORMATS = [
    x.strip().upper()
    for x in os.getenv("CAMERA_USB_COLOR_FORMATS", "RGB,BGR,MJPG,YUYV,NV12,NV21,I420").split(",")
    if x.strip()
]
CAMERA_LAN_WAIT_TIMEOUT_MS = int(os.getenv("CAMERA_LAN_WAIT_TIMEOUT_MS", "500"))
CAMERA_USB_WAIT_TIMEOUT_MS = int(os.getenv("CAMERA_USB_WAIT_TIMEOUT_MS", "1000"))
CAMERA_LAN_FRAME_RETRIES = int(os.getenv("CAMERA_LAN_FRAME_RETRIES", "4"))
CAMERA_USB_FRAME_RETRIES = int(os.getenv("CAMERA_USB_FRAME_RETRIES", "8"))
# Frames to discard before each real capture to drain stale pipeline buffer on LAN transport.
CAMERA_LAN_WARMUP_FRAMES = int(os.getenv("CAMERA_LAN_WARMUP_FRAMES", "2"))
# LAN FPS profile. Default 15 to reduce SDK buffer pressure.
# On a dedicated gigabit link you can raise this to 30.
CAMERA_LAN_FPS = int(os.getenv("CAMERA_LAN_FPS", "15"))
DEPTH_HOLE_FILL = os.getenv("DEPTH_HOLE_FILL", "True").lower() == "true"
DEPTH_WINDOW_HALF = int(os.getenv("DEPTH_WINDOW_HALF", "2"))
DEPTH_INNER_MARGIN_RATIO = float(os.getenv("DEPTH_INNER_MARGIN_RATIO", "0.22"))
DEPTH_INNER_MIN_VALID_RATIO = float(os.getenv("DEPTH_INNER_MIN_VALID_RATIO", "0.05"))
DEPTH_INNER_MAX_SPREAD_MM = float(os.getenv("DEPTH_INNER_MAX_SPREAD_MM", "60.0"))
DEPTH_MAX_CLAMP_DELTA_MM = float(os.getenv("DEPTH_MAX_CLAMP_DELTA_MM", "80.0"))
DEPTH_NEAREST_MIN_SAMPLES = int(os.getenv("DEPTH_NEAREST_MIN_SAMPLES", "48"))
DEPTH_NEAREST_MAX_CENTER_DIST_PX = float(os.getenv("DEPTH_NEAREST_MAX_CENTER_DIST_PX", "18.0"))
DEPTH_NEAREST_MAX_SPREAD_MM = float(os.getenv("DEPTH_NEAREST_MAX_SPREAD_MM", "12.0"))
DEPTH_INNER_RELAXED_MIN_VALID_RATIO = float(os.getenv("DEPTH_INNER_RELAXED_MIN_VALID_RATIO", "0.03"))
DEPTH_INNER_RELAXED_MAX_SPREAD_MM = float(os.getenv("DEPTH_INNER_RELAXED_MAX_SPREAD_MM", "8.0"))
DEPTH_NEAREST_RELAXED_MAX_CENTER_DIST_PX = float(os.getenv("DEPTH_NEAREST_RELAXED_MAX_CENTER_DIST_PX", "30.0"))
DEPTH_NEAREST_RELAXED_MAX_SPREAD_MM = float(os.getenv("DEPTH_NEAREST_RELAXED_MAX_SPREAD_MM", "4.0"))
DEPTH_BBOX_MIN_VALID_RATIO = float(os.getenv("DEPTH_BBOX_MIN_VALID_RATIO", "0.20"))
DEPTH_BBOX_MAX_SPREAD_MM = float(os.getenv("DEPTH_BBOX_MAX_SPREAD_MM", "25.0"))
DEPTH_TCP_STANDOFF_CLAMP_ENABLED = os.getenv("DEPTH_TCP_STANDOFF_CLAMP_ENABLED", "False").lower() == "true"


# ==================== TRAY REFERENCE ====================

TRAY_REF_ENABLED = os.getenv("TRAY_REF_ENABLED", "False").lower() == "true"
TRAY_REF_INNER_CORNERS = (6, 9)
TRAY_REF_SQUARE_SIZE_M = float(os.getenv("TRAY_REF_SQUARE_SIZE_M", "0.02"))


# ==================== ROBOT POSE DEFAULTS ====================

def _load_robot_pose_defaults() -> dict:
    """Load taught robot poses from robot_poses.json when available.

    Missing entries fall back to hard-coded defaults below so runtime does not break
    if the JSON is incomplete.
    """
    defaults = {
        "HOME_JOINTS": [-0.095581, -1.485115, -0.124507, -1.549033, 1.607623, 0.000156],
        "SCAN_APPROACH_JOINTS": [-0.094671, -1.502198, -0.954787, -1.580273, 1.577496, 0.000132],
        "SCAN_POSE_JOINTS": [-0.0937, -1.589512, -1.398178, -1.480151, 1.575771, 0.00012],
        "SCAN_POSE_TCP": [0.564878, -0.157992, 0.201213, 2.152263, 1.979381, 0.237215],
        "PLACE_APPROACH_CART": [-0.177861, 0.555789, 0.209864, 0.779606, -2.971455, -0.395008],
        "PLACE_POINT_CART": [-0.182785, 0.565214, 0.109697, 0.798319, -3.033674, -0.135877],
        "PLACE_RETREAT_CART": [-0.177856, 0.555788, 0.209869, 0.779512, -2.971445, -0.39503],
        "TOOL_DOWN": [2.152263, 1.979381, 0.237215],
    }

    poses_path = Path(__file__).resolve().parent / "robot_poses.json"
    if not poses_path.exists():
        return defaults

    try:
        data = json.loads(poses_path.read_text(encoding="utf-8"))

        if "HOME" in data and data["HOME"].get("joints_rad"):
            defaults["HOME_JOINTS"] = [float(x) for x in data["HOME"]["joints_rad"]]

        if "SCAN_APPROACH_JOINTS" in data and data["SCAN_APPROACH_JOINTS"].get("joints_rad"):
            defaults["SCAN_APPROACH_JOINTS"] = [
                float(x) for x in data["SCAN_APPROACH_JOINTS"]["joints_rad"]
            ]

        if "SCAN_POSE_JOINTS" in data and data["SCAN_POSE_JOINTS"].get("joints_rad"):
            defaults["SCAN_POSE_JOINTS"] = [float(x) for x in data["SCAN_POSE_JOINTS"]["joints_rad"]]
        if "SCAN_POSE_JOINTS" in data and data["SCAN_POSE_JOINTS"].get("tcp_m_rad"):
            defaults["SCAN_POSE_TCP"] = [float(x) for x in data["SCAN_POSE_JOINTS"]["tcp_m_rad"]]
            defaults["TOOL_DOWN"] = [float(x) for x in data["SCAN_POSE_JOINTS"]["tcp_m_rad"][3:6]]

        if "SCAN_POSE" in data and data["SCAN_POSE"].get("joints_rad"):
            defaults["SCAN_POSE_JOINTS"] = [float(x) for x in data["SCAN_POSE"]["joints_rad"]]
        if "SCAN_POSE" in data and data["SCAN_POSE"].get("tcp_m_rad"):
            defaults["SCAN_POSE_TCP"] = [float(x) for x in data["SCAN_POSE"]["tcp_m_rad"]]
            defaults["TOOL_DOWN"] = [float(x) for x in data["SCAN_POSE"]["tcp_m_rad"][3:6]]

        if "SCAN_POSE_TCP" in data and data["SCAN_POSE_TCP"].get("tcp_m_rad"):
            defaults["SCAN_POSE_TCP"] = [float(x) for x in data["SCAN_POSE_TCP"]["tcp_m_rad"]]
            defaults["TOOL_DOWN"] = [float(x) for x in data["SCAN_POSE_TCP"]["tcp_m_rad"][3:6]]

        if "PLACE_APPROACH_CART" in data and data["PLACE_APPROACH_CART"].get("tcp_m_rad"):
            defaults["PLACE_APPROACH_CART"] = [
                float(x) for x in data["PLACE_APPROACH_CART"]["tcp_m_rad"]
            ]

        if "PLACE_POINT_CART" in data and data["PLACE_POINT_CART"].get("tcp_m_rad"):
            defaults["PLACE_POINT_CART"] = [float(x) for x in data["PLACE_POINT_CART"]["tcp_m_rad"]]

        if "PLACE_RETREAT_CART" in data and data["PLACE_RETREAT_CART"].get("tcp_m_rad"):
            defaults["PLACE_RETREAT_CART"] = [
                float(x) for x in data["PLACE_RETREAT_CART"]["tcp_m_rad"]
            ]
        elif "PLACE_APPROACH_CART" in data and data["PLACE_APPROACH_CART"].get("tcp_m_rad"):
            # Safe fallback when retreat is intentionally same as place approach.
            defaults["PLACE_RETREAT_CART"] = [
                float(x) for x in data["PLACE_APPROACH_CART"]["tcp_m_rad"]
            ]
    except Exception:
        pass

    return defaults


_ROBOT_POSE_DEFAULTS = _load_robot_pose_defaults()


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
    _ROBOT_POSE_DEFAULTS["HOME_JOINTS"]
)

SCAN_APPROACH_JOINTS = _parse_joint_array(
    "SCAN_APPROACH_JOINTS",
    _ROBOT_POSE_DEFAULTS["SCAN_APPROACH_JOINTS"]
)

SCAN_POSE_JOINTS = _parse_joint_array(
    "SCAN_POSE_JOINTS",
    _ROBOT_POSE_DEFAULTS["SCAN_POSE_JOINTS"]
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
    _ROBOT_POSE_DEFAULTS["SCAN_POSE_TCP"]
)


PLACE_APPROACH_CART = _parse_pose_array(
    "PLACE_APPROACH_CART",
    _ROBOT_POSE_DEFAULTS["PLACE_APPROACH_CART"]
)

PLACE_POINT_CART = _parse_pose_array(
    "PLACE_POINT_CART",
    _ROBOT_POSE_DEFAULTS["PLACE_POINT_CART"]
)

PLACE_RETREAT_CART = _parse_pose_array(
    "PLACE_RETREAT_CART",
    _ROBOT_POSE_DEFAULTS["PLACE_RETREAT_CART"]
)


# ==================== TOOL ORIENTATION ====================

# Lấy từ tcp_m_rad của SCAN_POSE thực tế (robot_poses.json)
# Dùng trong build_pick_approach_pose() để set orientation gripper
# khi di chuyển đến pick point tính từ camera
TOOL_DOWN_RX = float(os.getenv("TOOL_DOWN_RX", str(_ROBOT_POSE_DEFAULTS["TOOL_DOWN"][0])))
TOOL_DOWN_RY = float(os.getenv("TOOL_DOWN_RY", str(_ROBOT_POSE_DEFAULTS["TOOL_DOWN"][1])))
TOOL_DOWN_RZ = float(os.getenv("TOOL_DOWN_RZ", str(_ROBOT_POSE_DEFAULTS["TOOL_DOWN"][2])))


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


def _load_hand_eye_defaults() -> np.ndarray:
    """Load default T_cam_to_tcp from hand_eye_result.json when available."""
    fallback = np.array([
        [0.9951500711354427, -0.044091332319025234, 0.08793344263395714, 0.1449751753706647],
        [0.02209516212173082, 0.971268628037997, 0.23695792031499208, -0.10261883832654922],
        [-0.09585478459597346, -0.2338657875866499, 0.9675322494193863, -0.4189851310270788],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)

    result_path = Path(__file__).resolve().parent / "hand_eye_result.json"
    if not result_path.exists():
        return fallback

    try:
        with result_path.open("r", encoding="utf-8") as file_obj:
            data = json.load(file_obj)
        matrix = data.get("T_cam_to_tcp")
        if matrix is None:
            return fallback
        arr = np.array(matrix, dtype=np.float64)
        if arr.shape != (4, 4):
            return fallback
        return arr
    except Exception:
        return fallback


_DEFAULT_T_CAM_TO_TCP = _load_hand_eye_defaults()

T_CAM_TO_TCP = _parse_homogeneous_matrix("T_CAM_TO_TCP", _DEFAULT_T_CAM_TO_TCP)


# ==================== CAMERA INTRINSICS (Orbbec Femto Mega) ====================

def _load_camera_intrinsics_defaults() -> dict:
    """Load camera intrinsics defaults from camera_intrinsics.json if available."""
    defaults = {
        "fx": 1114.278564453125,
        "fy": 1114.118408203125,
        "cx": 937.609375,
        "cy": 518.2891845703125,
        "width": 1920.0,
        "height": 1080.0,
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

CAM_FX = float(os.getenv("CAM_FX", str(_CAM_INTRINSICS_DEFAULTS["fx"])))    # focal length X
CAM_FY = float(os.getenv("CAM_FY", str(_CAM_INTRINSICS_DEFAULTS["fy"])))    # focal length Y
CAM_CX = float(os.getenv("CAM_CX", str(_CAM_INTRINSICS_DEFAULTS["cx"])))    # principal point X
CAM_CY = float(os.getenv("CAM_CY", str(_CAM_INTRINSICS_DEFAULTS["cy"])))    # principal point Y
CAM_CALIB_WIDTH = float(os.getenv("CAM_CALIB_WIDTH", str(_CAM_INTRINSICS_DEFAULTS["width"])))
CAM_CALIB_HEIGHT = float(os.getenv("CAM_CALIB_HEIGHT", str(_CAM_INTRINSICS_DEFAULTS["height"])))


def _maybe_fix_stale_intrinsics_baseline() -> Tuple[float, float]:
    """Repair stale width/height metadata when env overrides old baseline only.

    Common field issue:
    - fx/fy/cx/cy already updated for 1920x1080
    - but CAM_CALIB_WIDTH/HEIGHT remain overridden as 1280x720 in shell
    This causes false auto-scaling and wrong pick coordinates.
    """
    global CAM_CALIB_WIDTH, CAM_CALIB_HEIGHT

    default_w = float(_CAM_INTRINSICS_DEFAULTS["width"])
    default_h = float(_CAM_INTRINSICS_DEFAULTS["height"])
    if default_w <= 0 or default_h <= 0:
        return CAM_CALIB_WIDTH, CAM_CALIB_HEIGHT

    ratio_fx = CAM_FX / float(_CAM_INTRINSICS_DEFAULTS["fx"]) if _CAM_INTRINSICS_DEFAULTS["fx"] else 1.0
    ratio_fy = CAM_FY / float(_CAM_INTRINSICS_DEFAULTS["fy"]) if _CAM_INTRINSICS_DEFAULTS["fy"] else 1.0
    ratio_cx = CAM_CX / float(_CAM_INTRINSICS_DEFAULTS["cx"]) if _CAM_INTRINSICS_DEFAULTS["cx"] else 1.0
    ratio_cy = CAM_CY / float(_CAM_INTRINSICS_DEFAULTS["cy"]) if _CAM_INTRINSICS_DEFAULTS["cy"] else 1.0

    looks_like_same_intrinsics = all(abs(v - 1.0) < 0.02 for v in (ratio_fx, ratio_fy, ratio_cx, ratio_cy))
    stale_baseline = (
        abs(CAM_CALIB_WIDTH - default_w) > 1e-3
        or abs(CAM_CALIB_HEIGHT - default_h) > 1e-3
    )

    if looks_like_same_intrinsics and stale_baseline:
        print(
            "Canh bao config: CAM_CALIB_WIDTH/HEIGHT dang bi env override cu "
            f"({CAM_CALIB_WIDTH:.0f}x{CAM_CALIB_HEIGHT:.0f}) trong khi fx/fy/cx/cy "
            f"khop voi baseline file {default_w:.0f}x{default_h:.0f}. "
            "Tu dong sua ve baseline moi de tranh auto-scale sai."
        )
        CAM_CALIB_WIDTH = default_w
        CAM_CALIB_HEIGHT = default_h

    return CAM_CALIB_WIDTH, CAM_CALIB_HEIGHT


CAM_CALIB_WIDTH, CAM_CALIB_HEIGHT = _maybe_fix_stale_intrinsics_baseline()


# ==================== TIMEOUT & SAFETY ====================

RTDE_FREQUENCY = float(os.getenv("RTDE_FREQUENCY", "10.0"))  # Hz
RTDE_STEADY_THRESHOLD = float(os.getenv("RTDE_STEADY_THRESHOLD", "0.001"))  # rad/s
RTDE_WAIT_TIMEOUT = float(os.getenv("RTDE_WAIT_TIMEOUT", "35.0"))  # s
# Timeout đợi robot bắt đầu di chuyển sau khi gửi lệnh
# CB-series có độ trễ ~0.1-0.5s trước khi joint_speed > 0
RTDE_MOTION_START_TIMEOUT = float(os.getenv("RTDE_MOTION_START_TIMEOUT", "3.0"))  # s
RTDE_MOTION_START_THRESHOLD = float(os.getenv("RTDE_MOTION_START_THRESHOLD", "0.005"))  # rad/s
CB3_MOTION_PRE_WAIT_SLEEP_S = float(os.getenv("CB3_MOTION_PRE_WAIT_SLEEP_S", "0.5"))  # s

SOCKET_TIMEOUT = float(os.getenv("SOCKET_TIMEOUT", "5.0"))  # s
URSCRIPT_TIMEOUT = float(os.getenv("URSCRIPT_TIMEOUT", "10.0"))  # s


# ==================== LOGGING ====================

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
