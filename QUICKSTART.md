# Quick Start Guide - PC2 UR5 Server

Tai lieu nay la luong khoi dong nhanh theo runtime hien tai cua project.

## 1. Setup Moi Truong

```bash
cd /path/to/Ur5_control
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 2. Tao File Cấu Hình

```bash
cp .env.example .env
nano .env
```

Cap nhat toi thieu cac bien sau:

```env
ROBOT_IP=192.168.125.11
EXPERIMENT_STAGE=1
IS_SIMULATION=False

# Runtime gripper hien tai (Arduino + relay + pneumatic)
GRIPPER_PORT=/dev/gripper
GRIPPER_BAUD=9600

# Vision
YOLO_MODEL_PATH=models/phoi.pt
CAMERA_TRANSPORT=auto
CAMERA_IP=
CAMERA_WIDTH=1920
CAMERA_HEIGHT=1080
```

Neu chua san sang vision/model, van co the test Stage 1 truoc.

## 3. Kiem Tra Nhanh Truoc Khi Run

```bash
python tools/validate_config.py
python test_gripper.py open
python test_gripper.py close
```

Neu can test phan cung gripper chi tiet hon:

```bash
python tools/test_pneumatic_gripper.py --action status
```

## 4. Run Server

```bash
python app.py
```

Neu startup OK, log se co dang:

```text
======================================================================
PC2 UR5 Robot Control Server
======================================================================
Robot IP: 192.168.125.11
PC2 Server: 0.0.0.0:5001
PC1 Callback: Disabled
======================================================================
```

## 5. Health Check

Mo terminal khac:

```bash
curl http://localhost:5001/api/ur5/health
```

Response hien tai gom trang thai ket noi robot clients:

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

## 6. Chay Job Theo Stage

### Stage 1 (an toan, motion co ban)

```bash
curl -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{
    "phase": 1,
    "station": "khay_test",
    "workflow_id": "wf_stage1"
  }'
```

### Stage 2 (motion + vision)

```bash
curl -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{
    "phase": 2,
    "station": "khay_test",
    "workflow_id": "wf_stage2"
  }'
```

### Stage 3 (full flow)

```bash
curl -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{
    "phase": 3,
    "station": "khay_test",
    "workflow_id": "wf_stage3"
  }'
```

Response tao job:

```json
{
  "job_id": "a1b2c3d4",
  "status": "accepted",
  "experiment_stage": 1
}
```

## 7. Theo Doi Job

```bash
curl http://localhost:5001/api/ur5/status/a1b2c3d4
```

Abort neu can:

```bash
curl -X POST http://localhost:5001/api/ur5/abort/a1b2c3d4
```

## 8. Danh Sach Du Lieu Can Chuan Bi

1. Robot taught poses:
   - `HOME_JOINTS`
   - `SCAN_APPROACH_JOINTS`
   - `SCAN_POSE_JOINTS`
   - `PLACE_APPROACH_CART`
   - `PLACE_POINT_CART`
   - `PLACE_RETREAT_CART`
2. Hand-eye calibration: `T_CAM_TO_TCP`
3. Camera intrinsics: `CAM_FX`, `CAM_FY`, `CAM_CX`, `CAM_CY`
4. YOLO model: `models/phoi.pt`
5. Gripper serial path: `GRIPPER_PORT`

## 9. Common Issues

| Issue | Solution |
|-------|----------|
| Cannot connect robot | Check ping/IP and ports 29999/30002/30004 |
| `/health` all_connected=false | Check robot power/brake and shared connections in app log |
| Orbbec binding not found | Install `pyorbbecsdk` and verify import |
| YOLO model missing | Put model at `models/phoi.pt` or set `YOLO_MODEL_PATH` |
| Serial gripper not responding | Check `/dev/gripper`, permission, Arduino firmware protocol |
| Pick coordinates unstable | Re-check `T_CAM_TO_TCP`, intrinsics, `TOOL_DOWN_*` |

## 10. Test Scripts Nhanh

```bash
python example_api_usage.py --example basic --pc2-url http://localhost:5001
python tools/test_motion.py
python tools/test_phase2.py
```

## 11. Tai Lieu Lien Quan

- [README.md](README.md)
- [DEPLOYMENT.md](DEPLOYMENT.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [PARAMETERS_CHECKLIST.md](PARAMETERS_CHECKLIST.md)
