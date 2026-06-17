import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
import config
from vision.calibration import min_safe_camera_depth_m

print("=== Kiểm tra T_CAM_TO_TCP ===")
T = np.array(config.T_CAM_TO_TCP)
R = T[:3, :3]
t = T[:3, 3]

det = np.linalg.det(R)
orth_err = np.max(np.abs(R @ R.T - np.eye(3)))

print(f"det(R)        = {det:.6f}  (phải ≈ 1.0)")
print(f"Ortho error   = {orth_err:.6f}  (phải < 0.01)")
print(f"Translation   = {t} m")
print(f"  |t|         = {np.linalg.norm(t)*100:.1f} cm")

if abs(det - 1.0) < 0.01 and orth_err < 0.01:
    print("✓ R hợp lệ")
else:
    print("✗ R KHÔNG hợp lệ — calibration có vấn đề, cần chạy lại")

if np.max(np.abs(T[3] - np.array([0.0, 0.0, 0.0, 1.0]))) < 1e-9:
    print("✓ Homogeneous row cuối hợp lệ")
else:
    print("✗ Hàng cuối ma trận 4x4 không hợp lệ")

print(f"Translation   = {t * 1000.0} mm")
print(f"Min safe depth= {min_safe_camera_depth_m(T, config.PICK_MIN_DESCENT_M) * 1000.0:.1f} mm")

print("\n=== Kiểm tra Camera Intrinsics ===")
cx = config.CAM_CX
w = config.CAMERA_WIDTH
print(f"cx = {cx:.1f}, image_width = {w}")
print(f"cx lệch tâm = {cx - w/2:.1f} px")
if abs(cx - w/2) > 100:
    print("⚠ cx lệch nhiều — cần xác minh bằng ảnh thực tế")
else:
    print("✓ cx trong khoảng bình thường")

print("\n=== Kiểm tra TOOL_DOWN vs SCAN_POSE ===")
scan_tcp = list(config.SCAN_POSE_TCP)
tool_down = [config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ]
print(f"SCAN_POSE tcp rx,ry,rz = {scan_tcp[3:]}")
print(f"TOOL_DOWN rx,ry,rz     = {tool_down}")
if tool_down != scan_tcp[3:]:
    print("⚠ TOOL_DOWN khác SCAN_POSE orientation — nên đồng bộ lại")
else:
    print("✓ TOOL_DOWN đồng bộ với SCAN_POSE")

print("\n=== Kiểm tra hướng pick Z ===")
scan_z = float(config.SCAN_POSE_TCP[2])
approach_z = scan_z - abs(config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET) + config.PICK_APPROACH_OFFSET_Z
final_z = scan_z + config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET
print(f"SCAN_POSE z           = {scan_z:.6f} m")
print(f"Approach offset z     = {config.PICK_APPROACH_OFFSET_Z:.6f} m")
print(f"Final offset z        = {(config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET):.6f} m")
print(f"Example approach z    = {approach_z:.6f} m")
print(f"Example final z       = {final_z:.6f} m")
print(f"Max approach lift     = {config.PICK_MAX_APPROACH_LIFT_M:.6f} m")
print(f"Max final above scan  = {config.PICK_MAX_FINAL_Z_ABOVE_SCAN_M:.6f} m")
if final_z <= scan_z + config.PICK_MAX_FINAL_Z_ABOVE_SCAN_M:
    print("✓ Final pick pose mặc định không bị đội lên quá SCAN_POSE")
else:
    print("✗ Final pick pose mặc định đang cao hơn SCAN_POSE — cần kiểm tra lại dấu offset")
