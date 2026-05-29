# CHECKLIST: Thông số cần để hoàn thiện PC2

Tài liệu này liệt kê **toàn bộ thông số cần chuẩn bị** trước khi chạy PC2 production.

## 1️⃣ ROBOT POSITIONS (Dạy bằng Teach Pendant)

### Để dạy:
1. Bật robot, mở UR Teach Pendant
2. Move to position → Note down [j1, j2, j3, j4, j5, j6] (radian)
3. Lặp lại cho mỗi vị trí dưới đây

### Vị trí cần dạy:

| # | Thông số | Giá trị | Ghi chú |
|---|----------|--------|---------|
| 1 | `HOME_JOINTS` | [?, ?, ?, ?, ?, ?] | Vị trí home an toàn, ngoài workspace |
| 2 | `SCAN_APPROACH_JOINTS` | [?, ?, ?, ?, ?, ?] | Tiếp cận khay, joint move |
| 3 | `SCAN_POSE_JOINTS` | [?, ?, ?, ?, ?, ?] | Chụp ảnh bao quát khay từ trên xuống |
| 4 | `PLACE_APPROACH_CART` | [x, y, z+0.15, rx, ry, rz] | 150mm trên băng tải |
| 5 | `PLACE_POINT_CART` | [x, y, z, rx, ry, rz] | Tiếp xúc với băng tải |
| 6 | `PLACE_RETREAT_CART` | [x, y, z+0.15, rx, ry, rz] | 150mm trên, sau khi mở gripper |

### Lưu ý:
- **Joint angles**: Radian, từ UR pendant → copy [j1, j2, j3, j4, j5, j6]
- **Cartesian**: Mét + Radian, format [x, y, z, rx, ry, rz]
- **Tool orientation khi chụp ảnh** (SCAN_POSE_JOINTS):
  - Tool phải **thẳng đứng** nhìn xuống khay
  - Thường là rx = π (180°), ry = 0, rz = 0

### Cách dạy nhanh:
```bash
# Trên pendant: MOVE > Position
# Copy từ status bar: "Cartesian: x=0.5, y=0.2, z=0.4, rx=3.14, ry=0.0, rz=0.0"
# Hoặc từ "Joint: j1=..., j2=..., ..."

# Paste vào config.py:
export SCAN_POSE_JOINTS="0.0,-1.5708,0.0,-1.5708,0.0,0.0"
```

---

## 2️⃣ HAND-EYE CALIBRATION (Từ camera)

### Kết quả: `T_CAM_TO_TCP` (4x4 matrix)

**Phương pháp:**

#### Option A: Tính toán từ đo lường (nhanh, ~30 phút)
```
1. Đặt charuco board tại vị trí cố định trong workspace
2. Từ vị trí khác nhau, chụp ảnh board bằng camera
3. Tính tọa độ board trong base frame bằng FK robot
4. Giải phương trình: T_cam_to_tcp = solve(p_cam, p_base, tcp_poses)
```

**Công cụ:**
- OpenCV: `cv2.solvePnP()` để tính từ ảnh → 3D
- numpy: Matrix inversion để giải calibration

**Ví dụ code:**
```python
import numpy as np
from scipy.optimize import least_squares
import cv2

# Input:
# - detected_points_cam: list of [x, y, z] in camera frame
# - known_points_base: list of [x, y, z] in base frame (đo bằng FK)
# - tcp_poses: list of TCP pose khi chụp ảnh

# Giải:
T_cam_to_tcp = calibrate_hand_eye(
    detected_points_cam,
    known_points_base,
    tcp_poses
)

print(f"T_CAM_TO_TCP = {T_cam_to_tcp.tolist()}")
```

#### Option B: Dùng công cụ Kalibr (chính xác, ~1 tiếng)
```bash
# Kalibr: https://github.com/ethz-asl/kalibr
# Chụp video board, tự động calibrate
kalibr_calibrate_cameras --bag data.bag --models pinhole-radtan --target target.yaml
```

#### Option C: Giả sử identity (quick start, có sai)
```python
T_CAM_TO_TCP = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
]
# ⚠️ Chỉ dùng để test, sẽ có error 5-10cm
```

### Kết quả lưu:
```bash
export T_CAM_TO_TCP="1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1"
# hoặc giá trị thực tế từ calibration
```

---

## 3️⃣ CAMERA INTRINSICS (Từ Orbbec SDK hoặc calibration)

### Từ Orbbec SDK:
```python
import pyorbbecsdk as ob

device = ob.Device()
sensor = device.getSensorByType(ob.SensorType.RGB)
intrinsics = sensor.getIntrinsics()

print(f"FX: {intrinsics.fx}")
print(f"FY: {intrinsics.fy}")
print(f"CX: {intrinsics.cx}")
print(f"CY: {intrinsics.cy}")
```

### Típical Orbbec Femto Mega (1280x720):
```
CAM_FX = 605.0
CAM_FY = 605.0
CAM_CX = 640.0
CAM_CY = 360.0
```

### Hoặc từ calibration file:
```bash
# Nếu có file calibration.yaml
CAM_FX = 605.0  # Từ file
CAM_FY = 605.0
CAM_CX = 640.0
CAM_CY = 360.0
```

### Lưu vào .env:
```bash
export CAM_FX="605.0"
export CAM_FY="605.0"
export CAM_CX="640.0"
export CAM_CY="360.0"
```

---

## 4️⃣ YOLO MODEL (Phôi detection)

### Tùy chọn:

#### Option A: Download pretrained model
```bash
# YOLOv8 pretrained (nhưng không phải là "phôi")
mkdir -p models
cd models

# YOLOv8n: nhỏ, nhanh
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt

# YOLOv8s: balance tốc độ-độ chính xác
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8s.pt
```

#### Option B: Train model phôi riêng
```bash
# Dataset: Ảnh phôi từ camera, bbox annotations
# Format: YOLO format (txt files)

from ultralytics import YOLO

# Load pretrained
model = YOLO('yolov8s.pt')

# Train trên dataset phôi
results = model.train(
    data='phoi_dataset/data.yaml',
    epochs=100,
    imgsz=640,
    device=0,  # GPU
)

# Export
model.export(format='pt')  # → phoi.pt
```

#### Option C: Dùng model generic ban đầu (rồi fine-tune sau)
```bash
# Dùng COCO pretrained + threshold cao
# Sẽ không tốt nhưng có thể test
export YOLO_CONFIDENCE="0.7"  # Ngưỡng cao → ít false positive
```

### Lưu model:
```bash
# Copy file vào:
ur5.pt

# Update config:
export YOLO_MODEL_PATH="ur5.pt"
export YOLO_TARGET_CLASS="phoi"  # Tên lớp trong model
export YOLO_CONFIDENCE="0.5"      # Confidence threshold
```

---

## 5️⃣ GRIPPER PARAMETERS (OnRobot RG)

### Cần đo/kiểm tra:

| Thông số | Giá trị | Ghi chú |
|----------|--------|---------|
| `GRIPPER_OPEN_WIDTH` | 110 (mm) | Mở hoàn toàn (RG2 max 110mm, RG6 max 160mm) |
| `GRIPPER_CLOSE_FORCE` | 40-50 (N) | Lực gắp, phôi trụ ~40N |
| `GRIPPER_CLOSE_WIDTH` | 0 (mm) | Đóng hoàn toàn |
| `GRIPPER_TIMEOUT_S` | 3.0 (s) | Timeout detect grip |

### Kiểm tra:
1. Cẩm gripper, power on
2. Thử mở/đóng bằng pendant
3. Đo kích thước phôi → set `GRIPPER_OPEN_WIDTH` phù hợp
4. Thử gắp phôi → adjust `GRIPPER_CLOSE_FORCE`

### Lưu vào .env:
```bash
export GRIPPER_OPEN_WIDTH="110"
export GRIPPER_CLOSE_FORCE="40"
export GRIPPER_CLOSE_WIDTH="0"
export GRIPPER_TIMEOUT_S="3.0"
```

---

## 6️⃣ NETWORK & CONNECTIVITY

### Kiểm tra:

| Thông số | Giá trị | Kiểm tra |
|----------|--------|---------|
| `ROBOT_IP` | 192.168.125.11 | `ping 192.168.125.11` |
| Firewall ports | 29999, 30002, 30004 | `netstat -tuln \| grep 29999` |
| USB camera | /dev/video0 (Linux) | `lsusb \| grep Orbbec` |
| Network mask | 192.168.125.x/24 | Router config |

### Setup:
```bash
# Linux
sudo ip addr add 192.168.125.100/24 dev eth0

# Windows: Adapter settings → Static IP 192.168.125.100/24
```

### Lưu vào .env:
```bash
export ROBOT_IP="192.168.125.11"
```

---

## 7️⃣ PC1 CALLBACK (Optional)

Nếu muốn PC2 báo kết quả về PC1:

| Thông số | Giá trị | Ghi chú |
|----------|--------|---------|
| `PC1_BASE_URL` | http://192.168.1.100:5000 | IP + port PC1 |
| `PC1_CALLBACK_ENABLED` | True/False | Bật/tắt callback |
| `PC1_WEBHOOK_SECRET` | "secret123" | Khớp với UR5_WEBHOOK_SECRET ở PC1 |

### Lưu vào .env:
```bash
export PC1_BASE_URL="http://192.168.1.100:5000"
export PC1_CALLBACK_ENABLED="True"
export PC1_WEBHOOK_SECRET="secret123"
```

---

## 8️⃣ MOTION PARAMETERS (Optional tuning)

Default thường ổn, nhưng có thể tune:

| Thông số 10 | Default | Range | Ghi chú |
|----------|---------|-------|---------|
| `JOINT_VEL` | 0.8 rad/s | 0.1-1.5 | Nhanh hơn → riskier |
| `JOINT_ACCEL` | 1.0 rad/s² | 0.5-2.0 | Tăng accel → mạnh hơn |
| `LINEAR_VEL` | 0.1 m/s | 0.05-0.3 | Chậm nếu motion nhạy |
| `PICK_APPROACH_VEL` | 0.05 m/s | 0.02-0.1 | Vô cùng chậm khi xuống |
| `MAX_PICK_RETRIES` | 3 | 2-5 | Số lần retry grip |

### Lưu vào .env:
```bash
export JOINT_VEL="0.8"
export JOINT_ACCEL="1.0"
export LINEAR_VEL="0.1"
export PICK_APPROACH_VEL="0.05"
export MAX_PICK_RETRIES="3"
```

---

## 📋 CHECKLIST TRIỂN KHAI

### Tuần 1: Chuẩn bị cơ bản
- [ ] UR5 lên mạng, ping được
- [ ] Camera USB detect, test capture
- [ ] Gripper power on, thử mở/đóng
- [ ] UR pendant bật được
- [ ] Python 3.8+ trên PC2

### Tuần 2: Dạy vị trí
- [ ] Dạy `HOME_JOINTS` (và note down)
- [ ] Dạy `SCAN_APPROACH_JOINTS`
- [ ] Dạy `SCAN_POSE_JOINTS` (camera thẳng xuống khay)
- [ ] Dạy `PLACE_*_CART` (3 pose: approach, point, retreat)

### Tuần 3: Camera & Calibration
- [ ] Cài Orbbec SDK
- [ ] Đo camera intrinsics (hoặc dùng default)
- [ ] Làm hand-eye calibration (→ `T_CAM_TO_TCP`)
- [ ] Test camera capture + depth align

### Tuần 4: YOLO & Gripper
- [ ] Chuẩn bị hoặc train YOLO model phôi
- [ ] Đo kích thước phôi → `GRIPPER_OPEN_WIDTH`
- [ ] Test gripper force (gắp phôi không té)

### Tuần 5: Integration & Test
- [ ] Setup .env với tất cả thông số
- [ ] Test `python app.py` → server start
- [ ] Test health check: `curl /api/ur5/health`
- [ ] Dry run (không phôi): test motion paths
- [ ] Live run (có phôi): full pick-place cycle
- [ ] Fine-tune gripper force nếu cần

---

## 🔧 TEMPLATE .env CHO MỌI NGƯỜI

```bash
# ==================== ROBOT ====================
ROBOT_IP=192.168.125.11
DASHBOARD_PORT=29999
URSCRIPT_PORT=30002
RTDE_PORT=30004

# ==================== POSITIONS (TỪNG NGƯỜI ĐIỀN) ====================
HOME_JOINTS=0.0,-1.5708,0.0,-1.5708,0.0,0.0
SCAN_APPROACH_JOINTS=0.0,-1.5708,0.0,-1.5708,0.0,0.0
SCAN_POSE_JOINTS=0.0,-1.5708,0.0,-1.5708,0.0,0.0

PLACE_APPROACH_CART=0.5,0.0,0.3,3.14159,0.0,0.0
PLACE_POINT_CART=0.5,0.0,0.15,3.14159,0.0,0.0
PLACE_RETREAT_CART=0.5,0.0,0.3,3.14159,0.0,0.0

# ==================== CALIBRATION ====================
T_CAM_TO_TCP=1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1  # Identity (temporary)
CAM_FX=605.0
CAM_FY=605.0
CAM_CX=640.0
CAM_CY=360.0

# ==================== GRIPPER ====================
GRIPPER_OPEN_WIDTH=110
GRIPPER_CLOSE_FORCE=40
GRIPPER_CLOSE_WIDTH=0

# ==================== YOLO ====================
YOLO_MODEL_PATH=ur5.pt
YOLO_CONFIDENCE=0.5
YOLO_TARGET_CLASS=phoi

# ==================== PC1 CALLBACK (optional) ====================
PC1_CALLBACK_ENABLED=False
PC1_BASE_URL=http://192.168.1.100:5000
PC1_WEBHOOK_SECRET=
```

---

## ❓ FAQ

### Q: Có thể chạy mà không có hand-eye calibration?
A: Có nhưng sẽ sai 5-10cm. Dùng identity matrix: `T_CAM_TO_TCP = [[1,0,0,0], ...]` để test. Sau đó fine-tune khi có real calibration.

### Q: Nếu không có model YOLO?
A: Có thể dùng COCO pretrained (nhưng detect cái gì cũng được). Tốt nhất là train model phôi riêng (30 min nếu có dataset).

### Q: Gripper force bao nhiêu là đủ?
A: Phôi trụ nhẹ thường 30-50N. Thử từ 40N, tăng dần. Nếu phôi té thì tăng force.

### Q: Có thể copy positions từ robot khác?
A: **KHÔNG**. Mỗi robot có DH params khác nhau. Phải dạy lại.

### Q: Bao lâu là xong?
A: Nếu tất cả có sẵn (robot, camera, gripper): **2-3 tuần**
- Chuẩn bị: 2-3 ngày
- Dạy vị trí: 1 ngày
- Calibration: 2-3 ngày
- Test & fine-tune: 3-5 ngày

---

## 📞 KHI CẦN HỖ TRỢ

Nếu bị stuck ở thông số nào, cung cấp:
1. **Ảnh/video**: Khay, phôi, gripper, camera
2. **Log error**: Output từ `python app.py`
3. **Đặc tính phôi**: Kích thước (mm), trọng lượng, vật liệu
4. **Gripper loại**: RG2 hay RG6?
5. **UR version**: CB 3.1, CB 5.x?
