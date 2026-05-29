# Orbbec Runtime Bundle (Linux x86_64)

This folder contains runtime binaries copied from local `pyorbbecsdk_repo` so teammates can clone the main repository and run without rebuilding Orbbec SDK.

## Included

- `linux-x86_64/pyorbbecsdk.cpython-38-x86_64-linux-gnu.so`
- `linux-x86_64/libOrbbecSDK.so*`
- `linux-x86_64/OrbbecSDKConfig.xml`
- `linux-x86_64/extensions/*`

## Usage

1. Keep this folder in the repository.
2. Set `LD_LIBRARY_PATH` to include:
   - `vendor/orbbec_runtime/linux-x86_64`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/depthengine`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/filters`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/frameprocessor`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/firmwareupdater`
3. Ensure Python version is compatible with `pyorbbecsdk.cpython-38-...so`.

Example:

```bash
export LD_LIBRARY_PATH="$PWD/vendor/orbbec_runtime/linux-x86_64:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/depthengine:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/filters:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/frameprocessor:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/firmwareupdater:$LD_LIBRARY_PATH"
```
# UR5 Control Server (PC2)

Server Flask dieu khien UR5 + camera Orbbec Femto Mega + YOLO detection + gripper OnRobot RG.

Muc tieu: PC1 goi REST API sang PC2, PC2 thuc thi chu trinh pick-place theo tung giai doan thu nghiem (phase 1/2/3).

## 1. Tong quan

He thong hien tai ho tro 3 muc van hanh:

- Phase 1 (static_motion_only): chi test motion + gripper simulation, khong dung camera/YOLO.
- Phase 2 (motion_plus_vision): motion + camera + YOLO + tinh toa do pick, nhung khong gap thuc.
- Phase 3 (full_flow_sim_grip): full luong scan -> pick -> place, co retry logic, callback PC1 (neu bat).

API cho phep:

- tao job moi
- theo doi trang thai/phase/log
- abort job dang chay
- health check

Luu y: hien tai design chi cho phep 1 job chay tai 1 thoi diem.

## 2. Cau truc thu muc

```text
Ur5_control/
├── app.py
├── config.py
├── requirements.txt
├── README.md
├── QUICKSTART.md
├── ARCHITECTURE.md
├── DEPLOYMENT.md
├── PARAMETERS_CHECKLIST.md
├── example_api_usage.py
│
├── api/
│   ├── __init__.py
│   └── ur5_bp.py
│
├── core/
│   ├── __init__.py
│   ├── job_store.py
│   └── pick_place.py
│
├── robot/
│   ├── __init__.py
│   ├── dashboard_client.py
│   ├── urscript_client.py
│   ├── rtde_client.py
│   └── gripper_rg.py
│
├── vision/
│   ├── __init__.py
│   ├── femto_camera.py
│   ├── detector.py
│   └── calibration.py
│
├── tools/
│   ├── collect_dataset.py
│   ├── get_camera_intrinsics.py
│   ├── hand_eye_calibration.py
│   ├── read_robot_pose.py
│   ├── test_motion.py
│   └── validate_config.py
│
├── models/
├── logs/
└── dataset/
```

## 3. Yeu cau he thong

### Phan cung

- UR5 CB-series (da test theo profile CB3)
- Gripper OnRobot RG2/RG6
- Orbbec Femto Mega
- PC Linux (khuyen nghi), Python 3.8+

### Cong UR5 su dung

- 29999: Dashboard (safety/power/brake)
- 30002: URScript (movej/movel/gripper command)
- 30004: RTDE (doc trang thai robot theo thoi gian thuc)

## 4. Cai dat nhanh

### 4.1 Tao moi truong

```bash
cd /path/to/Ur5_control
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 4.2 Tao file env

```bash
cp .env.example .env
```

### 4.3 Cai SDK camera

Code hien tai su dung Orbbec Python binding. Neu chua co, camera se fail khi vao phase can vision.

Kiem tra nhanh:

```bash
python -c "import ob; print('Orbbec binding OK')"
```

Neu loi import, cai Orbbec SDK/binding phu hop may cua ban.

### 4.4 Model YOLO la bat buoc cho phase 2/3

Du an dang de `.gitignore` bo qua file `*.pt`, vi vay doi tac clone repo se KHONG co san `models/phoi.pt`.

Can chuan bi model thu cong truoc khi chay phase 2 hoac phase 3:

1. Copy file model vao `models/phoi.pt`
2. Hoac doi duong dan trong `.env`:
  - `YOLO_MODEL_PATH=/duong_dan/toi_model_cua_ban.pt`

Kiem tra nhanh truoc khi run server:

```bash
ls -lh models/phoi.pt
```

Neu thieu model:

- Phase 1 van chay duoc
- Phase 2/3 se khong detect duoc object dung cach

## 5. Cau hinh quan trong

Cac bien quan trong trong `.env`:

### Ket noi

- ROBOT_IP (mac dinh 192.168.125.11)
- DASHBOARD_PORT, URSCRIPT_PORT, RTDE_PORT
- PC2_HOST, PC2_PORT

### Chon che do chay

- EXPERIMENT_STAGE=1|2|3
- IS_SIMULATION=True|False

Goi y:

- test an toan ban dau: EXPERIMENT_STAGE=1 va IS_SIMULATION=True
- chay full that: EXPERIMENT_STAGE=3 va IS_SIMULATION=False (sau khi da verify)

### Motion

- JOINT_ACCEL, JOINT_VEL
- LINEAR_ACCEL, LINEAR_VEL
- PICK_APPROACH_VEL

### Gripper

- GRIPPER_OPEN_WIDTH
- GRIPPER_CLOSE_FORCE
- GRIPPER_CLOSE_WIDTH
- GRIPPER_TIMEOUT_S
- GRIPPER_GRIP_DETECT_METHOD (timeout | width_feedback | digital_output)

### Vision

- YOLO_MODEL_PATH
- YOLO_CONFIDENCE
- YOLO_TARGET_CLASS
- CAMERA_WIDTH, CAMERA_HEIGHT

Luu y:

- `YOLO_MODEL_PATH` phai tro toi file model ton tai thuc te.
- De chia se cho doi tac, nen gui kem file model qua link noi bo (Drive/NAS/Release) va huong dan copy ve `models/phoi.pt`.

### Calibration

- T_CAM_TO_TCP (4x4, 16 gia tri)
- CAM_FX, CAM_FY, CAM_CX, CAM_CY

## 6. Chay server

```bash
python app.py
```

Server start theo host/port trong config (mac dinh `0.0.0.0:5001`).

Health check:

```bash
curl http://localhost:5001/api/ur5/health
```

Vi du response:

```json
{
  "status": "ok",
  "robot_ip": "192.168.125.11",
  "pc2_port": 5001
}
```

## 7. REST API

Base path: `/api/ur5`

### 7.1 Tao job

`POST /api/ur5/execute`

Body toi thieu:

```json
{
  "phase": 1
}
```

Body day du:

```json
{
  "phase": 3,
  "station": "khay_MiR_01",
  "workflow_id": "wf_001"
}
```

Ghi chu:

- `phase` va `experiment_stage` deu duoc chap nhan, gia tri phai trong [1,2,3].
- Neu dang co job chay se tra 409.

Response thanh cong (202):

```json
{
  "job_id": "a1b2c3d4",
  "status": "accepted",
  "experiment_stage": 3
}
```

### 7.2 Xem trang thai

`GET /api/ur5/status/<job_id>`

Tra ve toan bo snapshot job:

- status: accepted | running | done | error | aborting | aborted
- phase: phase hien tai
- parts_found, parts_picked
- error
- log (toi da 100 dong gan nhat)

### 7.3 Abort

`POST /api/ur5/abort/<job_id>`

Response:

```json
{
  "aborted": true,
  "job_id": "a1b2c3d4"
}
```

### 7.4 Health

`GET /api/ur5/health`

Chi check service-level (khong ket noi truc tiep den robot).

## 8. Luong hoat dong theo phase

### Phase 1

- Connect dashboard + urscript + rtde
- Move HOME -> SCAN -> PICK_APPROACH_STATIC -> PLACE_APPROACH -> HOME
- Gripper close/open o che do simulation
- Khong su dung camera/YOLO

### Phase 2

- Them camera + YOLO
- Detect object, transform pixel/depth -> base pose
- Move den pick approach target (khong pick/place that)
- Return HOME

### Phase 3

- Full pick-place loop
- Initial scan dem tong parts
- Moi part:
  - scan lai + chon target tot nhat
  - tinh toa do 3D
  - move pick approach -> descend -> grip
  - retry toi da MAX_PICK_RETRIES neu grip fail
  - move place approach -> place -> retreat
- Cuoi chu trinh: return HOME

## 9. Callback ve PC1 (tuy chon)

Bat qua env:

- PC1_CALLBACK_ENABLED=True
- PC1_BASE_URL
- PC1_WEBHOOK_SECRET (neu can)

Khi job xong, PC2 POST den:

- `{PC1_BASE_URL}/api/workflow/ur5/done`

Co retry exponential backoff toi da 5 lan.

## 10. Bo script tools

Trong thu muc `tools/` co cac script ho tro calibration/kiem tra:

- `get_camera_intrinsics.py`: doc intrinsics tu Orbbec va ghi `camera_intrinsics.json`
- `collect_dataset.py`: thu du lieu anh
- `hand_eye_calibration.py`: hand-eye calibration bang checkerboard + RTDE
- `read_robot_pose.py`: doc pose robot
- `test_motion.py`: test motion profile
- `validate_config.py`: validate nhanh config/calibration

Vi du:

```bash
python tools/validate_config.py
python tools/get_camera_intrinsics.py
```

## 11. Test API tu PC1

Da co script mau:

```bash
python example_api_usage.py --example basic --pc2-url http://localhost:5001
```

Cac mode:

- basic
- abort
- concurrent

## 12. Troubleshooting nhanh

### Khong ket noi duoc robot

- Kiem tra ping ROBOT_IP
- Kiem tra 3 cong 29999/30002/30004
- Kiem tra robot da power on + brake release

### Loi camera binding

- Kiem tra import Orbbec binding
- Kiem tra camera USB3, quyen truy cap device

### YOLO model khong ton tai

- Dat model dung duong dan YOLO_MODEL_PATH (vd: models/phoi.pt)

### Grip fail lien tuc

- Tang GRIPPER_CLOSE_FORCE
- Kiem tra GRIPPER_GRIP_DETECT_METHOD
- Kiem tra setup co khi va kich thuoc phoi

### Toa do pick sai

- Re-check T_CAM_TO_TCP
- Re-check CAM_FX/FY/CX/CY
- Re-check huong tool TOOL_DOWN_RX/RY/RZ

## 13. Luu y van hanh an toan

- Luon test o phase 1 truoc khi phase 3
- Luon co nguoi giam sat khi chay robot that
- Xac nhan workspace clear, khong co vat can
- Verify lai cac pose day bang teach pendant truoc khi cho chay tu dong

## 14. Tai lieu lien quan

- QUICKSTART.md
- ARCHITECTURE.md
- DEPLOYMENT.md
- PARAMETERS_CHECKLIST.md

## License

Internal use only.