import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
import config

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
scan_rx = config.SCAN_POSE_JOINTS  # joints, không phải TCP
scan_tcp = [-0.876127, -0.102961, 0.23565, -2.04842, -2.026713, 0.31989]
tool_down = [config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ]
print(f"SCAN_POSE tcp rx,ry,rz = {scan_tcp[3:]}")
print(f"TOOL_DOWN rx,ry,rz     = {tool_down}")
if tool_down != scan_tcp[3:]:
    print("⚠ TOOL_DOWN khác SCAN_POSE orientation — nên đồng bộ lại")
