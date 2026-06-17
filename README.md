# UR5 Control Server (PC2)

PC2 Flask server dieu khien UR5, camera Orbbec Femto Mega, YOLO detection, va gripper khi nhap qua REST API.

Trang thai code hien tai:

- Runtime chinh dang dung `PneumaticGripper` qua serial Arduino (`core/pneumatic_gripper.py`).
- `robot/gripper_rg.py` van con trong repo nhu code tham khao / duong lui cho OnRobot RG, nhung khong phai runtime mac dinh.
- Robot clients (`Dashboard`, `URScript`, `RTDE`) duoc tao 1 lan, tai su dung giua cac jobs, va co reconnect logic.
- Camera intrinsics co the duoc nap tu `camera_intrinsics.json` neu file ton tai.
- Hand-eye calibration mac dinh da duoc cap nhat bang ket qua do thuc te.

Muc tieu he thong: PC1 goi REST API sang PC2, PC2 thuc thi job pick-place theo cac muc thu nghiem an toan tu stage 1 den stage 3.

## 1. Tong quan he thong

He thong hien tai ho tro 3 che do van hanh:

- Stage 1: `static_motion_only`
  - Motion co ban tren robot
  - Khong phu thuoc vision stack
  - Dung de test ket noi, teach point, motion profile, va luong co ban
- Stage 2: `motion_plus_vision`
  - Them camera + YOLO + transform toa do
  - Tinh pick target tu anh/depth
  - Khong thuc hien pick-place full cycle
- Stage 3: `full_flow_sim_grip`
  - Full luong scan -> detect -> transform -> pick -> place -> retry
  - Co cap nhat job state, log, callback PC1 neu bat

API hien tai cho phep:

- tao job moi
- theo doi trang thai / phase / log
- abort job dang chay
- health check

Gioi han hien tai:

- chi cho phep 1 job chay tai 1 thoi diem
- health endpoint khong chu dong mo ket noi moi, chi bao tinh trang cua shared clients hien co

## 2. Kien truc runtime hien tai

Luong runtime chinh:

1. `app.py` khoi tao logging va shared clients
2. `DashboardClient`, `URScriptClient`, `RTDEClient` duoc tao 1 lan
3. `PneumaticGripper` mo serial tai startup neu co hardware
4. `RobotConnectionManager` theo doi va reconnect robot clients khi can
5. `api/ur5_bp.py` nhan request, tao job, spawn thread xu ly
6. `core/pick_place.py` thuc thi chu trinh stage 1/2/3

Thanh phan chinh:

- `app.py`: startup Flask, logging, shared clients, inject dependencies vao API layer
- `api/ur5_bp.py`: REST endpoints `/execute`, `/status/<job_id>`, `/abort/<job_id>`, `/health`
- `core/pick_place.py`: logic motion, vision, retry, gripper actions, callback updates
- `core/job_store.py`: in-memory job store va log snapshot
- `core/pneumatic_gripper.py`: serial client toi Arduino relay/solenoid
- `vision/femto_camera.py`: camera frames + timestamp
- `vision/detector.py`: YOLO inference + target selection
- `vision/calibration.py`: pixel -> camera -> base transform utilities

## 3. Cau truc thu muc

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
├── camera_intrinsics.json            # co the co, duoc nap lam default
├── hand_eye_result.json              # ket qua calibration luu tam/tham khao
├── test_gripper.py                   # test nhanh open/close/toggle gripper
│
├── api/
│   ├── __init__.py
│   └── ur5_bp.py
│
├── core/
│   ├── __init__.py
│   ├── job_store.py
│   ├── pick_place.py
│   └── pneumatic_gripper.py
│
├── robot/
│   ├── __init__.py
│   ├── dashboard_client.py
│   ├── urscript_client.py
│   ├── rtde_client.py
│   └── gripper_rg.py                 # legacy/reference path for OnRobot RG
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
│   ├── test_phase2.py
│   ├── test_pneumatic_gripper.py
│   ├── test_scanpose_touch.py
│   └── validate_config.py
│
├── models/
├── logs/
└── dataset/
```

## 4. Yeu cau he thong

### Phan cung

- UR5 CB-series (da test theo profile CB3 / PolyScope 3.15.5)
- Camera Orbbec Femto Mega
- Pneumatic gripper dieu khien qua Arduino + relay + solenoid
- PC Linux, Python 3.8+

Neu ban van can OnRobot RG:

- repo van con `robot/gripper_rg.py`
- nhung runtime mac dinh trong `app.py` hien dang dung `PneumaticGripper`

### Cong UR5 duoc su dung

- `29999`: Dashboard
- `30002`: URScript
- `30004`: RTDE

### Python packages quan trong

- `flask`
- `numpy`
- `opencv-python`
- `ur-rtde`
- `ultralytics`
- `pyserial`
- Orbbec Python binding / `pyorbbecsdk`

## 5. Cai dat nhanh

### 5.1 Tao moi truong

```bash
cd /path/to/Ur5_control
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 5.2 Tao file env

```bash
cp .env.example .env
```

### 5.3 Cai camera SDK / binding

Kiem tra nhanh:

```bash
python -c "import pyorbbecsdk as ob; print('Orbbec binding OK')"
```

Neu loi import, stage 2/3 se khong vao duoc vision path.

### 5.4 Chuan bi YOLO model

Du an khong commit file `*.pt`. Ban phai copy model thu cong vao `models/phoi.pt` hoac set:

```bash
export YOLO_MODEL_PATH=/duong_dan/model_cua_ban.pt
```

Kiem tra nhanh:

```bash
ls -lh models/phoi.pt
```

## 6. Cau hinh hien tai

Tat ca config duoc doc tu `config.py`, va da co default tu bo du lieu do thuc te trong repo.

### 6.1 Ket noi he thong

- `ROBOT_IP`
- `DASHBOARD_PORT`
- `URSCRIPT_PORT`
- `RTDE_PORT`
- `PC2_HOST`, `PC2_PORT`

### 6.2 Experiment stage va simulation

- `EXPERIMENT_STAGE=1|2|3`
- `IS_SIMULATION=True|False`

Nhan xet:

- `EXPERIMENT_STAGE` duoc dung trong runtime va co map label:
  - `1 -> static_motion_only`
  - `2 -> motion_plus_vision`
  - `3 -> full_flow_sim_grip`
- `IS_SIMULATION` duoc dung de bieu thi che do mo phong / disable hardware path o cac code path tuong thich.
- Runtime chinh hien van khoi tao `PneumaticGripper`; vi vay neu khong co hardware, can kiem tra startup log va deployment mode cua ban.

### 6.3 Motion va pose

Joint poses:

- `HOME_JOINTS`
- `SCAN_APPROACH_JOINTS`
- `SCAN_POSE_JOINTS`

Cartesian poses:

- `SCAN_POSE_TCP`
- `PLACE_APPROACH_CART`
- `PLACE_POINT_CART`
- `PLACE_RETREAT_CART`

Motion tuning:

- `JOINT_ACCEL`, `JOINT_VEL`
- `LINEAR_ACCEL`, `LINEAR_VEL`
- `PICK_APPROACH_VEL`
- `PICK_APPROACH_OFFSET_Z`
- `PICK_FINAL_OFFSET_Z`
- `PICK_RETREAT_OFFSET_Z`
- `MAX_PICK_RETRIES`

### 6.4 Pneumatic gripper config

Runtime dang dung `core/pneumatic_gripper.py` voi cac env chinh:

- `GRIPPER_PORT` (mac dinh `/dev/gripper`)
- `GRIPPER_BAUD` (mac dinh `9600`)
- `GRIPPER_CMD_TIMEOUT_S`
- `GRIPPER_SETTLE_S`
- `GRIPPER_RELEASE_SETTLE_S` (mac dinh `0.3`, delay sau open/release de xylanh xa khi on dinh)
- `GRIPPER_HEARTBEAT_S`

Protocol serial hien tai:

- `1` -> grip / close
- `0` -> release / open
- `?` -> query state
- `K` -> keepalive

Expected responses tu firmware:

- `GRIP_OK`
- `GRIP_ALREADY`
- `RELEASE_OK`
- `RELEASE_ALREADY`

Ghi chu:

- khi `connect()`, Arduino Uno co the auto-reset khi serial mo
- class co heartbeat thread de giu watchdog cua firmware khong nha relay ngoai y muon
- `disconnect()` co co gang `open()` gripper truoc khi dong serial
- `open()` co settle delay rieng (`GRIPPER_RELEASE_SETTLE_S`) de tranh move som khi ngong kep chua mo het

### 6.4.1 Motion tool va CB3 settle notes

- `tools/test_motion.py` la tool test tay, khong quan ly power/brake qua Dashboard.
- Dieu kien truoc khi chay: operator xac nhan robot da POWER ON + RUNNING tren PolyScope.
- Trong runtime pick-place (`core/pick_place.py`), sau moi lan `prepare_to_run()` co them `time.sleep(1.5)` de cho CB3 brake release settle truoc khi gui lenh URScript.

### 6.5 Legacy OnRobot RG config

Trong `config.py` van con cac bien cho RG / code cu:

- `GRIPPER_OPEN_WIDTH`
- `GRIPPER_CLOSE_FORCE`
- `GRIPPER_CLOSE_WIDTH`
- `GRIPPER_TIMEOUT_S`
- `GRIPPER_GRIP_DETECT_METHOD`
- `GRIPPER_DIGITAL_OUTPUT_PIN`

Nhung luong runtime mac dinh hien tai khong su dung `robot/gripper_rg.py`.

### 6.6 Vision va calibration

YOLO:

- `YOLO_MODEL_PATH`
- `YOLO_CONFIDENCE`
- `YOLO_TARGET_CLASS`

Camera stream:

- `CAMERA_WIDTH`
- `CAMERA_HEIGHT`
- `DEPTH_HOLE_FILL`

Intrinsics:

- `CAM_FX`, `CAM_FY`, `CAM_CX`, `CAM_CY`
- `CAM_CALIB_WIDTH`, `CAM_CALIB_HEIGHT`

Hand-eye:

- `T_CAM_TO_TCP`

Tool orientation:

- `TOOL_DOWN_RX`
- `TOOL_DOWN_RY`
- `TOOL_DOWN_RZ`

Luu y quan trong:

- neu `camera_intrinsics.json` co mat trong root repo, `config.py` se tu dong nap file nay lam default
- `SCAN_POSE_TCP` da co gia tri do thuc te, dung lam reference Cartesian cho scan pose
- hand-eye default hien tai da la ket qua calibration co quality tot, khong con la identity placeholder

## 7. Chay server

```bash
python app.py
```

Khi startup, app se:

1. setup logging vao console + `logs/pc2_ur5.log`
2. tao shared Dashboard / URScript / RTDE clients
3. tao `RobotConnectionManager`
4. co gang connect `PneumaticGripper`
5. inject cac shared clients vao blueprint API

Startup log se hien thong tin tuong tu:

```text
======================================================================
PC2 UR5 Robot Control Server
======================================================================
Robot IP: 192.168.125.11
PC2 Server: 0.0.0.0:5001
PC1 Callback: Disabled
======================================================================
```

## 8. REST API

Base path: `/api/ur5`

### 8.1 Execute job

`POST /api/ur5/execute`

Request toi thieu:

```json
{
  "phase": 1
}
```

Request day du:

```json
{
  "phase": 3,
  "station": "khay_MiR_01",
  "workflow_id": "wf_001"
}
```

Ghi chu:

- `phase` va `experiment_stage` deu duoc chap nhan
- gia tri hop le chi trong `[1, 2, 3]`
- neu dang co 1 job chay, API tra `409`

Response thanh cong:

```json
{
  "job_id": "a1b2c3d4",
  "status": "accepted",
  "experiment_stage": 3
}
```

### 8.2 Xem trang thai job

`GET /api/ur5/status/<job_id>`

Tra ve snapshot tu `JobStore`, gom:

- `job_id`
- `status`
- `phase`
- `station`
- `workflow_id`
- `parts_found`
- `parts_picked`
- `error`
- `created_at`
- `updated_at`
- `log`

### 8.3 Abort job

`POST /api/ur5/abort/<job_id>`

Response:

```json
{
  "aborted": true,
  "job_id": "a1b2c3d4"
}
```

### 8.4 Health check

`GET /api/ur5/health`

Response hien tai:

```json
{
  "status": "ok",
  "robot_ip": "192.168.125.11",
  "pc2_port": 5001,
  "robot_connection": {
    "available": true,
    "dashboard_connected": true,
    "urscript_connected": true,
    "rtde_connected": true,
    "all_connected": true
  }
}
```

Ghi chu:

- endpoint nay khong mo ket noi moi
- no chi doc tinh trang cua shared clients / connection manager hien co

## 9. Luong hoat dong theo stage

### Stage 1 - static motion only

Luong co ban trong `PickPlaceCycle`:

- precheck safety qua Dashboard
- prepare robot (power / brake)
- move `HOME_JOINTS`
- move `SCAN_POSE_JOINTS`
- move `PICK_APPROACH_CART_STATIC`
- gripper close/open
- move `PLACE_APPROACH_CART`
- return home

Muc dich:

- verify motion, ket noi, timing, gripper chain
- khong can camera va YOLO

### Stage 2 - motion + vision + pick-place 1 phoi

Luong chinh:

- precheck + prepare robot
- open gripper
- home -> scan approach -> scan pose
- chup frame + doc tcp timestamp
- YOLO detect -> select best target
- transform pixel -> camera -> base
- build approach_pose va final_pose
- move approach -> descend -> close gripper (GAP PHOI)
- retreat sau pick
- move place approach -> place point -> open gripper (THA PHOI)
- place retreat
- return home

Muc dich:

- validate toan bo pipeline: camera, hand-eye, transform, gripper, place
- pick-place 1 phoi de xac nhan he thong truoc khi chay phase 3 (5 phoi)
- tien de de tich hop voi MiR

### Stage 3 - full flow

Luong chinh:

- open gripper
- home -> scan approach -> scan pose
- initial scan dem tong so phoi
- moi lan pick:
  - scan lai + doc tcp timestamp gan frame timestamp
  - select target tot nhat
  - lay depth tin cay
  - transform sang `p_base`
  - build `approach_pose` va `final_pose`
  - move approach -> descend -> close gripper
  - neu fail, retry toi da `MAX_PICK_RETRIES`
  - move place approach -> place point -> retreat
- cuoi chu trinh return home

Co cap nhat vao job store:

- `phase`
- `parts_found`
- `parts_picked`
- log chi tiet

## 10. Callback ve PC1

Bat callback bang:

- `PC1_CALLBACK_ENABLED=True`
- `PC1_BASE_URL=http://...`
- `PC1_WEBHOOK_SECRET=` neu can

Khi job xong, PC2 goi:

- `{PC1_BASE_URL}/api/workflow/ur5/done`

Retry policy hien tai:

- toi da 5 lan
- exponential backoff: `1s, 2s, 4s, 8s, 16s`

Payload co the gom:

- `job_id`
- `success`
- `workflow_id`
- `status`
- `experiment_stage`
- `result`
- `error`

## 11. Tools va test scripts

### Runtime validation

- `python tools/validate_config.py`
  - kiem tra `T_CAM_TO_TCP`
  - kiem tra orthogonality / determinant rotation
  - kiem tra `CAM_CX` so voi `CAMERA_WIDTH`
  - canh bao neu `TOOL_DOWN_*` khac huong scan pose reference

### Camera / calibration

- `python tools/get_camera_intrinsics.py`
  - doc intrinsics tu Orbbec
  - ghi `camera_intrinsics.json`
- `python tools/hand_eye_calibration.py`
  - hand-eye calibration bang checkerboard + RTDE
- `python tools/read_robot_pose.py`
  - doc va luu taught poses

### Motion / workflow debug

- `python tools/test_motion.py`
  - test sequence motion theo taught poses
- `python tools/test_phase2.py`
  - test rieng flow stage 2
- `python tools/test_scanpose_touch.py`
  - verify SCAN_POSE / TCP reference / touch logic

### Gripper tests

- `python test_gripper.py close`
- `python test_gripper.py open`
- `python test_gripper.py toggle --cycles 5 --hold-s 1.0`

Tool phan cung day du hon:

- `python tools/test_pneumatic_gripper.py --action status`
- `python tools/test_pneumatic_gripper.py --action open`
- `python tools/test_pneumatic_gripper.py --action close`
- `python tools/test_pneumatic_gripper.py --action cycle --cycles 10`
- `python tools/test_pneumatic_gripper.py --action hold --hold-s 20`

`tools/test_pneumatic_gripper.py` duoc dung de verify full chain:

- PC2 -> `/dev/gripper`
- Arduino
- relay
- solenoid valve
- pneumatic gripper

## 12. Van de thuong gap

### Khong ket noi duoc robot

- ping `ROBOT_IP`
- kiem tra 3 cong `29999 / 30002 / 30004`
- dam bao robot da power on + brake release
- goi `/api/ur5/health` de xem `robot_connection`

### Health tra `available=false`

- app chua inject connection manager
- hoac shared client chua duoc khoi tao / da loi

### Khong mo duoc gripper serial

- kiem tra `GRIPPER_PORT`
- kiem tra symlink `/dev/gripper`
- kiem tra quyen serial device
- kiem tra Arduino co reset khi mo cong serial hay khong

### Gripper co serial nhung khong phan hoi dung protocol

- kiem tra firmware Arduino co tra `GRIP_OK`, `RELEASE_OK`, `STATE:1/0`
- chay `tools/test_pneumatic_gripper.py --action status --verbose`

### YOLO model khong ton tai

- dat dung model vao `YOLO_MODEL_PATH`
- kiem tra model file thuc su ton tai trong filesystem

### Toa do pick sai

- re-check `T_CAM_TO_TCP`
- re-check `CAM_FX/FY/CX/CY`
- re-check `TOOL_DOWN_RX/RY/RZ`
- re-check `SCAN_POSE_TCP`
- chay lai `tools/validate_config.py`

### Stage 2/3 co detection nhung target_pose sai

- so sanh timestamp camera va RTDE
- xac minh TCP tai luc chup frame
- xac minh `camera_intrinsics.json` co dung resolution thuc te

## 13. Luu y van hanh an toan

- luon test stage 1 truoc khi chay stage 3
- luon co nguoi giam sat khi chay robot that
- dung gan E-stop khi test motion / gripper
- xac nhan workspace clear truoc khi bat dau
- verify lai taught poses bang teach pendant truoc khi cho chay tu dong
- test gripper rieng truoc khi cho pick-place full cycle

## 14. Tai lieu lien quan

- `QUICKSTART.md`
- `ARCHITECTURE.md`
- `DEPLOYMENT.md`
- `PARAMETERS_CHECKLIST.md`

## License

Internal use only.