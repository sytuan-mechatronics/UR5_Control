# Quick Start Guide - PC2 UR5 Server

**Để chạy PC2 trong 5 phút:**

## 1. Clone / Setup
```bash
cd /path/to/ur5_backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 2. Configure Robot
```bash
cp .env.example .env
nano .env
```

**Chỉnh sửa các dòng này với giá trị thực tế:**
```
ROBOT_IP=192.168.125.11              # IP của UR5 của bạn
YOLO_MODEL_PATH=ur5.pt               # Model phôi trong thư mục gốc repo
SCAN_POSE_JOINTS=...                 # Từ teach pendant
PLACE_APPROACH_CART=...              # Từ teach pendant
PLACE_POINT_CART=...                 # Từ teach pendant
```

## 3. Run Server
```bash
python app.py
```

Nếu successful, sẽ thấy:
```
================================================================================
PC2 UR5 Robot Control Server
================================================================================
Robot IP: 192.168.125.11
PC2 Server: 0.0.0.0:5001
PC1 Callback: Disabled
================================================================================
INFO - Starting Flask server on 0.0.0.0:5001
```

## 4. Test Health Check
```bash
# Mở terminal khác
curl http://localhost:5001/api/ur5/health
```

Response:
```json
{
  "status": "ok",
  "robot_ip": "192.168.125.11",
  "pc2_port": 5001
}
```

## 5. Submit Job
```bash
curl -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{
    "station": "khay_test",
    "workflow_id": "workflow_001"
  }'
```

Response:
```json
{
  "job_id": "a1b2c3d4",
  "status": "accepted"
}
```

## 6. Check Status
```bash
curl http://localhost:5001/api/ur5/status/a1b2c3d4
```

## Next Steps

### Data ngay sẽ cần (trước khi chạy):

1. **Robot positions** (dùng teach pendant):
   - [ ] `HOME_JOINTS`: Vị trí home an toàn
   - [ ] `SCAN_APPROACH_JOINTS`: Tiếp cận khay
   - [ ] `SCAN_POSE_JOINTS`: Chụp ảnh bao quát
   - [ ] `PLACE_APPROACH_CART`: 150mm trên băng tải
   - [ ] `PLACE_POINT_CART`: Tiếp xúc băng tải
   - [ ] `PLACE_RETREAT_CART`: 150mm trên sau khi mở

2. **Hand-eye calibration**:
   - [ ] Run hand-eye calibration procedure
   - [ ] Get `T_CAM_TO_TCP` matrix (4x4)
   - [ ] Paste vào config

3. **YOLO model**:
   - [ ] Train hoặc download YOLOv8 model phôi
   - [ ] Lưu vào `ur5.pt` hoặc cập nhật `YOLO_MODEL_PATH` đúng vị trí file
   - [ ] Test detection trên ảnh sample

4. **Camera intrinsics**:
   - [ ] Lấy từ Orbbec SDK hoặc calibration
   - [ ] Cập nhật `CAM_FX, CAM_FY, CAM_CX, CAM_CY`

### Common Issues

| Issue | Solution |
|-------|----------|
| Cannot connect to robot | Check robot IP, firewall, network cable |
| Orbbec SDK not found | `python -c "import ob"` để diagnose |
| YOLO model not found | Đặt file vào `ur5.pt` hoặc sửa `YOLO_MODEL_PATH` |
| Grip failed repeatedly | Tăng `GRIPPER_CLOSE_FORCE`, kiểm tra phôi |
| Coordinate out of workspace | Check hand-eye calibration, YOLO detection |

### Full Example

Run Python script test:
```bash
python example_api_usage.py --example basic
```

### Production Setup

Xem [DEPLOYMENT.md](DEPLOYMENT.md) cho:
- Systemd service
- Docker deployment
- Gunicorn WSGI
- Nginx reverse proxy

### Documentation

- [README.md](README.md) - Full docs
- [DEPLOYMENT.md](DEPLOYMENT.md) - Production deployment
- [example_api_usage.py](example_api_usage.py) - API examples

### Support

Liên hệ robotics team nếu có vấn đề.
