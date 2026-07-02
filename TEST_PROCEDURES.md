# Quy trinh test UR5 Control System

> Luon chay theo thu tu: Stage 1 → Stage 2 → Stage 3. Khong skip stage.

---

## Muc luc

1. [Kiem tra truoc khi bat dau](#1-kiem-tra-truoc-khi-bat-dau)
2. [Test camera doc lap](#2-test-camera-doc-lap)
3. [Test gripper doc lap](#3-test-gripper-doc-lap)
4. [Stage 1 — Static motion](#4-stage-1--static-motion)
5. [Stage 2 — Vision + pick don](#5-stage-2--vision--pick-don)
6. [Stage 3 — Full loop nhieu phoi](#6-stage-3--full-loop-nhieu-phoi)
7. [Kiem tra log va job status](#7-kiem-tra-log-va-job-status)
8. [Xu ly su co thuong gap](#8-xu-ly-su-co-thuong-gap)

---

## 1. Kiem tra truoc khi bat dau

### 1.1 Moi truong Python

```bash
cd /path/to/Ur5_Control
source .venv/bin/activate
python3 tools/validate_config.py
```

Ket qua mong doi:
- `T_CAM_TO_TCP`: rotation orthogonal OK, determinant = 1.0
- `CAM_CX / CAM_CY`: nam trong kich thuoc anh
- `TOOL_DOWN_RX/RY/RZ`: khong co warning sai huong

### 1.2 Ket noi mang

```bash
# Robot
ping -c 3 192.168.125.11

# Camera LAN
ping -c 3 192.168.125.10

# NIC speed
ethtool $(ip route get 192.168.125.10 | awk '{print $5; exit}') | grep Speed
# Mong doi: Speed: 1000Mb/s
```

### 1.3 Cong robot

```bash
nc -zv 192.168.125.11 29999   # Dashboard
nc -zv 192.168.125.11 30002   # URScript
nc -zv 192.168.125.11 30004   # RTDE
```

### 1.4 Model YOLO

```bash
ls -lh models/phoi.pt
python3 -c "from ultralytics import YOLO; m = YOLO('models/phoi.pt'); print('YOLO OK, classes:', m.names)"
```

### 1.5 Orbbec SDK

```bash
python3 -c "import pyorbbecsdk as ob; print('Orbbec SDK OK')"
```

---

## 2. Test camera doc lap

**Muc dich:** Xac nhan stream LAN on dinh truoc khi cho robot chay.

```bash
python3 test_camera_view.py --transport lan --ip 192.168.125.10
```

**Tieu chi pass** (theo doi it nhat 60 giay):

| Chi so | Yeu cau |
|--------|---------|
| FPS | >= 10 |
| Reconnects | 0 sau 60s |
| Stream stale | < 500ms |
| Depth frame | Hien thi dung, khong den |

**Neu FPS thap hoac stale cao:**
- Kiem tra lai `Speed: 1000Mb/s`
- Kiem tra PoE injector cap nguoi du (PoE 160s >= 15W)
- Thu giam `CAMERA_LAN_FPS` xuong 8 trong `.env`

---

## 3. Test gripper doc lap

**Muc dich:** Xac nhan toan bo chain PC2 → serial → Arduino → relay → solenoid.

### 3.1 Kiem tra trang thai

```bash
python3 tools/test_pneumatic_gripper.py --action status --verbose
```

Mong doi: `STATE:0` (mo) hoac `STATE:1` (dong)

### 3.2 Test dong/mo

```bash
python3 test_gripper.py close
# Kiem tra: gripper dong, nghe tieng khi nen
python3 test_gripper.py open
# Kiem tra: gripper mo hoan toan
```

### 3.3 Test cycle

```bash
python3 test_gripper.py toggle --cycles 5 --hold-s 1.0
```

Mong doi: 5 chu ky dong/mo, khong co loi timeout, khong co GRIP_FAIL response.

### 3.4 Test do ben

```bash
python3 tools/test_pneumatic_gripper.py --action hold --hold-s 20
```

Mong doi: giu 20 giay, kem khong truot, ap suat on dinh.

---

## 4. Stage 1 — Static motion

**Muc dich:** Verify motion, ket noi, timing, gripper chain. Khong can camera.

**Dieu kien truoc:**
- Robot POWER ON + BRAKE RELEASE tren PolyScope
- Workspace clear, khong co vat can
- Nguoi giam sat dung canh E-stop

### 4.1 Khoi dong server

```bash
python3 app.py
```

Kiem tra startup log:
```
Robot IP: 192.168.125.11
PC2 Server: 0.0.0.0:5001
```

### 4.2 Health check

```bash
curl -s http://localhost:5001/api/ur5/health | python3 -m json.tool
```

Mong doi: `"all_connected": true`

### 4.3 Chay Stage 1

```bash
curl -s -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{"phase": 1}' | python3 -m json.tool
```

Ghi lai `job_id` tu response.

### 4.4 Theo doi trang thai

```bash
JOB_ID="<job_id_o_tren>"
watch -n 1 "curl -s http://localhost:5001/api/ur5/status/$JOB_ID | python3 -m json.tool"
```

**Tieu chi pass Stage 1:**
- `status: "done"`
- Robot di chuyen theo: HOME → SCAN_APPROACH → SCAN_POSE → PICK_APPROACH_STATIC → HOME
- Gripper dong va mo khong co loi
- Khong co exception trong log

**Neu fail:** Xem `log` trong response status, dung `DEPLOYMENT.md` de debug.

---

## 5. Stage 2 — Vision + pick don

**Muc dich:** Validate toan bo pipeline camera + YOLO + transform + pick-place 1 phoi.

**Dieu kien truoc:**
- Stage 1 pass
- Camera stream on dinh (FPS >= 10, Reconnects = 0)
- 1 phoi nam trong vung nhin cua camera tai SCAN_POSE
- `pick_correction_map.json` da co gia tri do thuc te

### 5.1 Xac nhan camera tai scan pose

Thu cong di robot den SCAN_POSE bang teach pendant, chup anh kiem tra:
```bash
python3 test_camera_view.py --transport lan --ip 192.168.125.10 --save-frame /tmp/scan_check.png
```

Kiem tra anh: phoi ro net, nam trong ~80% trung tam anh.

### 5.2 Chay Stage 2

```bash
curl -s -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{"phase": 2}' | python3 -m json.tool
```

**Tieu chi pass Stage 2:**
- `status: "done"`
- `parts_found >= 1`
- `parts_picked == 1`
- Robot ha xuong dung vi tri phoi, grip, nang len, dat vao place point
- Khong co `"depth_failed"`, `"validation_failed"` trong log

**Neu toa do sai > 15mm:**
1. Kiem tra TCP pose tai luc chup: log hien thi `cam_ts` va `rtde_ts`, delta phai < 100ms
2. Chay lai `tools/validate_config.py`
3. Thu tang `SCAN_SETTLE_SLEEP_S=0.5` trong `.env`
4. Thu tang `CAMERA_LAN_WARMUP_FRAMES=3`

**Neu grip truot nhung toa do dung:**
1. Kiem tra `GRASP_Z_OFFSET` — dich xuong them neu can (giam gia tri, e.g. `-0.030`)
2. Kiem tra ap suat khi nen
3. Kiem tra PICK_APPROACH_VEL — giam toc do ap sat

---

## 6. Stage 3 — Full loop nhieu phoi

**Muc dich:** Chay vong lap tu dong pick-place toan bo khay cho den khi khay rong.

**Dieu kien truoc:**
- Stage 2 pass
- 5 phoi dat vao khay dung cac vi tri slot
- `pick_correction_map.json` co du 5 diem voi dx/dy/dz da do thuc te

### 6.1 Xac nhan correction map

```bash
python3 -c "
import json
with open('pick_correction_map.json') as f:
    m = json.load(f)
print('So diem:', len(m['points']))
for p in m['points']:
    print('{}: dx={} dy={} dz={}'.format(p['name'], p['dx'], p['dy'], p['dz']))
"
```

Gia tri dx/dy trong khoang -0.025 den +0.025 (25mm) la binh thuong.

### 6.2 Chay Stage 3

```bash
curl -s -X POST http://localhost:5001/api/ur5/execute \
  -H "Content-Type: application/json" \
  -d '{"phase": 3}' | python3 -m json.tool
```

**Tieu chi pass Stage 3:**
- `status: "done"`
- `parts_picked == 5` (hoac = tong so phoi trong khay)
- Khong co cycle nao dat `MAX_PICK_CYCLES=20`
- Moi phoi duoc dat dung place_point, khong bi roi
- Vong lap ket thuc voi `detections=0` (khay rong)

**Theo doi theo thoi gian thuc:**
```bash
JOB_ID="<job_id>"
while true; do
    STATUS=$(curl -s http://localhost:5001/api/ur5/status/$JOB_ID)
    echo $STATUS | python3 -m json.tool | grep -E '"status"|"parts_picked"|"parts_found"|"phase"'
    echo "---"
    sleep 3
done
```

**Neu mot so phoi khong duoc pick:**
- Kiem tra log xem co `"grip_fail"` hoac `"validation_failed"` khong
- Neu grip fail nhieu: giam `PICK_APPROACH_VEL`, dieu chinh `GRASP_Z_OFFSET`
- Neu bi skip (exclusion): giam `PICKED_EXCLUSION_RADIUS_PX=65` trong `.env`

**Neu vong lap chay qua 20 cycle (MAX_PICK_CYCLES):**
- Log se ghi warning: `MAX_PICK_CYCLES reached`
- Tang `MAX_PICK_CYCLES=30` neu can nhieu cycle hon
- Dieu tra nguyen nhan grip fail lien tiep

---

## 7. Kiem tra log va job status

### Xem log real-time

```bash
tail -f logs/pc2_ur5.log | grep -E "ERROR|WARNING|pick_cycle|grip|detect|flush"
```

### Cac message log quan trong

| Log message | Y nghia |
|-------------|---------|
| `LAN transport detected: using 10fps` | Camera dung LAN profile |
| `Flushing N stale frames` | Buffer drain truoc capture |
| `cam_ts delta: Xms` | Do tre giua frame va TCP pose |
| `pick_cycle N/20` | Bat dau cycle thu N |
| `grip_success=True` | Gap thanh cong |
| `Quick retry X/3` | Dang thu lai grip |
| `grip_fail: returning to scan` | Het retry, quay lai scan |
| `detections=0 after exclusion` | Tat ca phoi da gap |
| `tray empty, stopping` | Ket thuc vong lap |

### Doc job log day du

```bash
curl -s http://localhost:5001/api/ur5/status/<job_id> | python3 -c "
import sys, json
j = json.load(sys.stdin)
for line in j.get('log', []):
    print(line)
"
```

---

## 8. Xu ly su co thuong gap

### Camera stream stale tro lai sau khi fix

Trieu chung: `Stream stale > 1000ms`, Reconnects tang

Kiem tra:
```bash
# NIC co giam toc khong?
ethtool <interface> | grep Speed
# Thu reset camera
python3 -c "import pyorbbecsdk as ob; p = ob.Pipeline(); p.stop()"
```

Neu van stale: giam FPS xuong 8, tang `CAMERA_LAN_WAIT_TIMEOUT_MS=700`

---

### Robot khong ket noi duoc sau reboot

```bash
python3 -c "
import socket
for port in [29999, 30002, 30004]:
    try:
        s = socket.create_connection(('192.168.125.11', port), timeout=3)
        s.close()
        print(f'Port {port}: OK')
    except Exception as e:
        print(f'Port {port}: FAIL - {e}')
"
```

Neu Dashboard (29999) OK nhung RTDE (30004) fail: robot chua vao RUNNING mode, phai bam resume tren PolyScope.

---

### Phoi duoc detect nhung grip lien tuc fail

Uu tien kiem tra theo thu tu:
1. `GRASP_Z_OFFSET`: thu giam them 5mm (-0.005)
2. `PICK_APPROACH_VEL`: giam xuong 0.05 m/s
3. Ap suat khi nen: kiem tra pressure gauge
4. Kiem tra pick correction map: dx/dy co vuot qua 30mm khong?

---

### Toa do pick sai nhieu (> 20mm)

1. Kiem tra frame timestamp vs TCP timestamp trong log
   - Neu delta > 150ms: tang `SCAN_SETTLE_SLEEP_S=0.5`, tang `CAMERA_LAN_WARMUP_FRAMES=3`
2. Chay lai validate:
   ```bash
   python3 tools/validate_config.py
   ```
3. Kiem tra `SCAN_POSE_TCP` co khop voi vi tri thuc te khong:
   ```bash
   python3 tools/read_robot_pose.py
   ```
4. Thu chay Stage 2 voi 1 phoi, quan sat vi tri robot ha xuong, do sai lech, cap nhat correction map.

---

### Abort job dang chay khi can thiet

```bash
curl -s -X POST http://localhost:5001/api/ur5/abort/<job_id> | python3 -m json.tool
```

Robot se dung sau lenh hien tai va return home (neu abort duoc xu ly trong pick cycle).

**Khan cap:** Nhan E-stop tren PolyScope hoac nhan E-stop vat ly.
