# Deployment Guide - PC2 UR5 Server

Tai lieu deploy production cho runtime hien tai cua project.

Scope tai lieu nay:

- Deploy service Flask dieu khien UR5 tren Linux
- Van hanh voi shared robot clients + reconnect manager
- Runtime gripper theo duong pneumatic serial (Arduino)
- Health check va quan tri service

## 1. Chon Kieu Deploy

### Option A: Bare Metal Linux (khuyen nghi cho robot cell)

Uu diem:

- It layer trung gian
- Truy cap serial/USB de hon
- Debug truc tiep tren host

Nhuoc diem:

- Quan tri dependencies theo host

### Option B: Docker

Uu diem:

- Moi truong dong goi, de tai lap

Nhuoc diem:

- USB/serial passthrough phuc tap hon
- Co them overhead va van de permission

Khuyen nghi van hanh robot that: uu tien Bare Metal.

## 2. Bare Metal Deployment (Ubuntu 20.04+)

### 2.1 Prerequisites

```bash
sudo apt-get update
sudo apt-get install -y \
  python3 python3-pip python3-venv \
  git build-essential curl

# Neu can camera SDK build/runtime
sudo apt-get install -y libusb-1.0-0-dev
```

### 2.2 Lay source code

```bash
sudo mkdir -p /opt/ur5_control
sudo chown -R $USER:$USER /opt/ur5_control
cd /opt/ur5_control
git clone https://github.com/hquy55869-maker/Ur5_Control.git .
```

### 2.3 Python environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 2.4 Camera binding / SDK

Kiem tra binding:

```bash
python -c "import pyorbbecsdk as ob; print('Orbbec binding OK')"
```

Neu loi, can cai Orbbec SDK/binding phu hop kernel va may.

### 2.5 Gripper serial device

Runtime hien tai dung `GRIPPER_PORT` (mac dinh `/dev/gripper`).

Kiem tra serial devices:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
```

Vi du tao udev rule de co symlink on dinh:

```bash
sudo tee /etc/udev/rules.d/99-gripper.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="gripper", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Sau do:

```bash
ls -l /dev/gripper
```

### 2.6 Tao file .env

```bash
cp .env.example .env
nano .env
```

Gia tri toi thieu nen set:

```env
ROBOT_IP=192.168.125.11
PC2_HOST=0.0.0.0
PC2_PORT=5001

EXPERIMENT_STAGE=1
IS_SIMULATION=False

GRIPPER_PORT=/dev/gripper
GRIPPER_BAUD=9600
GRIPPER_CMD_TIMEOUT_S=3.0
GRIPPER_SETTLE_S=0.5
GRIPPER_HEARTBEAT_S=3.0

YOLO_MODEL_PATH=models/phoi.pt
LOG_LEVEL=INFO
```

### 2.7 Validate truoc khi bat service

```bash
source .venv/bin/activate
python tools/validate_config.py
python test_gripper.py open
python test_gripper.py close
```

Neu can test gripper sau hon:

```bash
python tools/test_pneumatic_gripper.py --action status --verbose
```

## 3. Systemd Service

Tao service file:

```bash
sudo nano /etc/systemd/system/ur5-pc2.service
```

Noi dung de xuat:

```ini
[Unit]
Description=UR5 Robot Control Server (PC2)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=ur5
Group=ur5
WorkingDirectory=/opt/ur5_control
EnvironmentFile=/opt/ur5_control/.env
Environment="PATH=/opt/ur5_control/.venv/bin"
ExecStart=/opt/ur5_control/.venv/bin/python /opt/ur5_control/app.py
Restart=always
RestartSec=5
TimeoutStopSec=20

# Hardening (co the dieu chinh theo thuc te)
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=full
ProtectHome=false
ReadWritePaths=/opt/ur5_control/logs

[Install]
WantedBy=multi-user.target
```

Enable/start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable ur5-pc2
sudo systemctl start ur5-pc2
sudo systemctl status ur5-pc2 --no-pager
```

Logs:

```bash
sudo journalctl -u ur5-pc2 -f
```

## 4. Health Check Va API Smoke Test

### 4.1 Health endpoint

```bash
curl -s http://127.0.0.1:5001/api/ur5/health | jq .
```

Expected shape:

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

### 4.2 Stage 1 smoke

```bash
curl -X POST http://127.0.0.1:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{"phase":1,"station":"deploy_smoke","workflow_id":"wf_smoke_001"}'
```

### 4.3 Theo doi job

```bash
curl http://127.0.0.1:5001/api/ur5/status/<job_id>
```

Abort khi can:

```bash
curl -X POST http://127.0.0.1:5001/api/ur5/abort/<job_id>
```

## 5. Docker (tham khao)

Luu y quan trong:

- camera USB va serial gripper can passthrough dung
- can cap quyen `/dev/bus/usb` va `/dev/gripper`/`/dev/tty*`

Vi du `docker-compose.yml` toi thieu:

```yaml
services:
  ur5-pc2:
    build: .
    container_name: ur5-pc2
    ports:
      - "5001:5001"
    env_file:
      - .env
    volumes:
      - ./logs:/app/logs
      - /dev/bus/usb:/dev/bus/usb
      - /dev/gripper:/dev/gripper
    privileged: true
    restart: always
```

Neu truoc mat uu tien stability robot cell, nen dung bare metal.

## 6. Gunicorn / Nginx

Runtime hien tai la Flask app tu `app.py` voi shared clients khoi tao trong process chinh.

Khuyen nghi:

- Khong chay nhieu workers cho runtime control robot
- Neu dung gunicorn, de `workers=1`, `threads=1`

Vi du gunicorn command:

```bash
gunicorn -w 1 --threads 1 -b 0.0.0.0:5001 "app:create_app()"
```

Trong thuc te, run truc tiep `python app.py` qua systemd de it rui ro hon voi lifecycle hardware.

## 7. Logging Va Rotation

App tu ghi log vao:

- `logs/pc2_ur5.log`

Tao logrotate:

```bash
sudo tee /etc/logrotate.d/ur5-pc2 >/dev/null <<'EOF'
/opt/ur5_control/logs/*.log {
    daily
    rotate 14
    compress
    delaycompress
    missingok
    notifempty
    copytruncate
}
EOF
```

## 8. Security Checklist

- Gioi han mang chi cho PC1/PC2/robot subnet can thiet
- Khong expose cong 5001 ra Internet cong khai
- Dung firewall allowlist IP
- Bao ve file `.env` (chua endpoint callback, secrets)
- Dat quyen toi thieu cho user chay service

## 9. Troubleshooting

### 9.1 `robot_connection.all_connected=false`

- Kiem tra robot da power on + brake release
- Kiem tra ping toi `ROBOT_IP`
- Kiem tra cong 29999/30002/30004
- Xem logs systemd

### 9.2 Gripper serial fail

- Kiem tra `/dev/gripper` co ton tai va dung permission
- Kiem tra firmware Arduino co protocol `1/0/?/K`
- Test bang `tools/test_pneumatic_gripper.py`

### 9.3 Vision fail o stage 2/3

- Kiem tra import `pyorbbecsdk`
- Kiem tra model file `YOLO_MODEL_PATH`
- Kiem tra intrinsics va hand-eye (`tools/validate_config.py`)

## 10. Rollback

Rollback nhanh ve commit truoc:

```bash
cd /opt/ur5_control
git fetch --all
git checkout <stable_commit>
source .venv/bin/activate
pip install -r requirements.txt
sudo systemctl restart ur5-pc2
```

Sau rollback, luon re-run smoke test stage 1 truoc khi van hanh.
