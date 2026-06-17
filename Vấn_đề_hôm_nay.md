# Vấn đề hôm nay

Tai lieu nay tong hop cac su co da gap trong buoi lam viec hom nay, cach chan doan, cach sua, va trang thai hien tai cua du an.

Muc tieu:

- luu lai cac loi da gap de lan sau khong bi lap lai
- ghi ro nhung thay doi moi da ap dung vao code
- tach ro van de nao da sua xong, van de nao con phu thuoc phan cung/mang

---

## 1. Tong quan ket qua hom nay

Hom nay da xu ly 4 nhom van de chinh:

1. Moi truong Python, thu vien, va binding camera Orbbec
2. Phan biet va ho tro 2 nhanh ket noi camera: `USB` va `LAN`
3. Cap nhat ma tran `hand-eye` moi sau khi thay gripper
4. Dong bo bo `camera_intrinsics` moi vao runtime va tai lieu

Trang thai hien tai:

- du an da co `.venv` va cac package Python chinh da duoc cai
- `ultralytics`, `ur-rtde`, `opencv`, `flask`, `pyserial`, `scipy` da import duoc
- binding Orbbec da duoc noi vao runtime bundle trong repo
- code runtime da ho tro chon camera theo `CAMERA_TRANSPORT=auto|usb|lan`
- ma tran `T_CAM_TO_TCP` mac dinh da duoc thay bang ma tran moi
- intrinsics hien tai `1920x1080` da duoc dong bo vao `config.py` va file mau

---

## 2. Su co: moi truong Python va setup thu vien

### 2.1 Hien tuong

Can doc lai repo va setup lai de chay du an. Ban dau chua co moi truong Python san sang trong repo.

### 2.2 Nguyen nhan

- chua co `.venv` rieng cho project
- `requirements.txt` can `ultralytics`, `ur-rtde`, `opencv-python`, `pyserial`, `scipy`, `flask`, `requests`
- vision stack can them `torch/torchvision`

### 2.3 Cach xu ly

Da tao:

- `/.venv`

Da cai:

- `flask==2.3.3`
- `requests==2.31.0`
- `numpy==1.24.4`
- `opencv-python`
- `scipy==1.10.1`
- `python-dotenv==1.0.1`
- `pyserial==3.5`
- `ur-rtde==1.6.3`
- `torch==2.4.1+cpu`
- `torchvision==0.19.1+cpu`
- `ultralytics==8.4.67`

### 2.4 Ghi chu quan trong

Ban `requirements.txt` hien tai cho `ultralytics>=8.3.0` se keo them `torch`.

Neu cai truc tiep tu PyPI ma khong canh gioi, `pip` co the keo ban `CUDA` rat nang. Trong buoi nay da chuyen qua cai:

- `torch`
- `torchvision`

theo kenh `CPU-only` de setup nhe hon.

### 2.5 Trang thai hien tai

Da xac nhan import duoc:

- `flask`
- `requests`
- `numpy`
- `cv2`
- `scipy`
- `serial`
- `rtde_control`, `rtde_receive`, `rtde_io`
- `ultralytics`

---

## 3. Su co: nghi ngo du an co dung YOLO11 hay khong

### 3.1 Hien tuong

Can kiem tra xem du an da dung `YOLO11` chua.

### 3.2 Ket qua kiem tra

Da ra soat:

- [vision/detector.py](./vision/detector.py)
- [config.py](./config.py)
- [requirements.txt](./requirements.txt)
- tai lieu `README`, `QUICKSTART`, `PARAMETERS_CHECKLIST`, `DEPLOYMENT`

### 3.3 Ket luan

Du an hien tai:

- dung API generic `from ultralytics import YOLO`
- khong co tham chieu rieng toi `yolo11`, `yolov11`, `yolo11*.pt`
- comment trong [vision/detector.py](./vision/detector.py) van mo ta theo kieu `YOLOv8`

Nghia la:

- code **co the** chay voi model YOLO moi neu `ultralytics` ho tro
- nhung **khong co dau hieu nao cho thay du an dang khoa chat vao YOLO11**

### 3.4 Huong su dung

Neu muon dung model moi:

- dat model `.pt` moi vao duong dan phu hop
- doi `YOLO_MODEL_PATH`

Nhung ve mat code, repo hien chi dung `Ultralytics YOLO` noi chung.

---

## 4. Su co: `vision/detector.py` co loi cu phap

### 4.1 Hien tuong

Khi import stack vision, xuat hien `SyntaxError`.

### 4.2 Nguyen nhan

Trong [vision/detector.py](./vision/detector.py), ham `select_best_target()` co docstring bi lap:

- co 2 lan khoi `Returns:`
- dong mo/ket thuc docstring khong dung

### 4.3 Cach sua

Da xoa phan docstring lap du thua.

### 4.4 Trang thai hien tai

Da `py_compile` pass cho:

- [vision/detector.py](./vision/detector.py)

va import stack vision da qua duoc buoc nay.

---

## 5. Su co: binding Orbbec khong on dinh giua cac script

### 5.1 Hien tuong

Co script import duoc camera, co script lai bao:

- `pyorbbecsdk không có`
- `Chưa cài pyorbbecsdk trong môi trường hiện tại`

Du repo da co runtime bundle trong:

- [vendor/orbbec_runtime](./vendor/orbbec_runtime)

### 5.2 Nguyen nhan

Binding camera co 3 cach import khac nhau trong repo:

1. `import ob`
2. `import pyorbbecsdk as ob`
3. fallback / import truc tiep theo script

Ngoai ra, khi chay script theo duong dan tuyet doi, vi du:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/hand_eye_calibration.py
```

Python chi uu tien thu muc `tools/`, nen khong thay duoc wrapper o root repo.

### 5.3 Cach sua

Da them:

- [ob.py](./ob.py)
- [pyorbbecsdk.py](./pyorbbecsdk.py)
- [sitecustomize.py](./sitecustomize.py)

Muc dich:

- `ob.py`: shim cho `import ob`
- `pyorbbecsdk.py`: wrapper de `import pyorbbecsdk` cung dung duoc runtime bundle
- `sitecustomize.py`: hook startup bo sung runtime path trong truong hop Python load tu root repo

Da sua them cac script tools de tu dua repo root vao `sys.path`:

- [tools/hand_eye_calibration.py](./tools/hand_eye_calibration.py)
- [tools/get_camera_intrinsics.py](./tools/get_camera_intrinsics.py)

### 5.4 Trang thai hien tai

Da xac nhan:

- `import ob` OK
- `import pyorbbecsdk as ob` OK khi chay tu root repo
- `tools/hand_eye_calibration.py` khong con chet ngay tu constructor vi ly do thieu binding
- `tools/get_camera_intrinsics.py --help` chay duoc bang `/usr/bin/python3`

---

## 6. Su co: `get_camera_intrinsics.py` van roi vao nhanh fallback sai

### 6.1 Hien tuong

Khi chay:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/get_camera_intrinsics.py
```

script in:

```text
pyorbbecsdk không có, dùng giá trị mặc định Femto Mega
```

du thuc te binding da load duoc qua `ob`.

### 6.2 Nguyen nhan

Trong [tools/get_camera_intrinsics.py](./tools/get_camera_intrinsics.py):

- da dat `SDK = "ob"` neu import qua shim `ob`
- nhung `main()` lai chi chap nhan:

```python
if SDK == "pyorbbecsdk":
```

nen script tu roi xuong nhanh fallback sai.

### 6.3 Cach sua

Da sua thanh:

```python
if SDK in {"ob", "pyorbbecsdk"}:
```

### 6.4 Trang thai hien tai

Loi logic nay da duoc xu ly.

Neu van loi khi chay, thi do la loi mo camera that / quyen mang / USB, khong con la loi bind-wrapper nua.

---

## 7. Su co: can ho tro ro rang 2 nhanh camera `USB` va `LAN`

### 7.1 Hien tuong

Trong qua trinh debug camera, co luc dang test `USB`, co luc lai chuyen qua `LAN`. Code cu de nhieu script tu tuyen chon camera theo cach khong thong nhat, nen rat de gap:

- script USB nhay sang nhanh LAN
- script LAN lai in goi y debug USB
- `test_camera_view.py` va `tools/get_camera_intrinsics.py` khong cung mot giao dien cau hinh

### 7.2 Nguyen nhan

- config camera chua tach bach `transport`
- mot so script co tham so `ip` mac dinh, lam code de roi vao nhanh LAN ngoai y muon
- runtime chinh va tools chua cung dung mot cach mo camera

### 7.3 Cach xu ly

Da cap nhat:

- [config.py](./config.py)
- [vision/femto_camera.py](./vision/femto_camera.py)
- [test_camera_view.py](./test_camera_view.py)
- [tools/get_camera_intrinsics.py](./tools/get_camera_intrinsics.py)
- [tools/hand_eye_calibration.py](./tools/hand_eye_calibration.py)
- [README.md](./README.md)
- [QUICKSTART.md](./QUICKSTART.md)
- [.env.example](./.env.example)

Them cac bien cau hinh:

- `CAMERA_TRANSPORT=auto|usb|lan`
- `CAMERA_IP`
- `CAMERA_NET_PORT`

Them cach chay ro rang:

USB:

```bash
python3 test_camera_view.py --backend orbbec --transport usb
python3 tools/get_camera_intrinsics.py --transport usb
```

LAN:

```bash
python3 test_camera_view.py --backend orbbec --transport lan --ip 192.168.125.10
python3 tools/get_camera_intrinsics.py --transport lan --ip 192.168.125.10
```

### 7.4 Ghi chu thuc te

Neu camera khong ping duoc tren LAN, loi nam o:

- subnet/IP
- switch/PoE/day LAN
- camera chua o dung IP

khong nam o YOLO hay xu ly anh.

### 7.5 Trang thai hien tai

Code da ho tro day du ca `USB` va `LAN`.

Van de con lai khi test camera that neu co se thuoc:

- ket noi mang
- quyen USB / enumerate device
- cau hinh IP camera

---

## 8. Su co: cap nhat ma tran `hand-eye` moi sau khi thay gripper

### 8.1 Hien tuong

Sau khi thay gripper moi, can dung ma tran `T_CAM_TO_TCP` moi thay cho ma tran cu.

### 8.2 Cach xu ly

Da cap nhat default hand-eye trong [config.py](./config.py) theo [hand_eye_result.json](./hand_eye_result.json):

```text
[[ 0.976265,  0.168094,  0.136569,  0.108484],
 [-0.171848,  0.984992,  0.016089, -0.128004],
 [-0.131814, -0.039177,  0.990500, -0.147013],
 [ 0.000000,  0.000000,  0.000000,  1.000000]]
```

### 8.3 Luu y

Ban than file `hand_eye_result.json` chi chua ma tran cuoi, khong chua:

- so pose da chup
- consistency error
- raw data

nen ve mat traceability thi van con thieu.

### 8.4 Trang thai hien tai

Runtime mac dinh dang dung ma tran moi.

---

## 9. Su co: cap nhat `camera_intrinsics` moi vao runtime

### 9.1 Hien tuong

Can dong bo intrinsics that cua camera vao runtime thay vi dung bo mac dinh Femto Mega.

### 9.2 Cach xu ly

Da cap nhat:

- [camera_intrinsics.json](./camera_intrinsics.json)
- [config.py](./config.py)
- [.env.example](./.env.example)
- [PARAMETERS_CHECKLIST.md](./PARAMETERS_CHECKLIST.md)

Bo gia tri moi:

- `fx = 1114.278564453125`
- `fy = 1114.118408203125`
- `cx = 937.609375`
- `cy = 518.2891845703125`
- `width = 1920`
- `height = 1080`

### 9.3 Trang thai hien tai

Runtime da doc bo intrinsics moi nay theo mac dinh.

---

## 10. Su co: cap nhat bo `robot_poses.json` nhieu lan va can dong bo vao runtime

### 10.1 Hien tuong

Trong buoi nay da co nhieu lan ghi lai pose bang:

- [tools/read_robot_pose.py](./tools/read_robot_pose.py)

Ban dau `robot_poses.json` thieu:

- `SCAN_POSE_JOINTS`
- `PLACE_RETREAT_CART`
- `PLACE_POINT_CART`

Sau do da ghi bo pose cuoi cung day du hon.

### 10.2 Nguyen nhan de gay nham

- ten pose nhap bang tay de bi sai
- co lan operator dang dinh nhap `SCAN_POSE_JOINTS` nhung lai luu nham vao key khac
- `config.py` co fallback cu, nen nhin qua de tuong runtime chua cap nhat

### 10.3 Cach xu ly

Da sua [config.py](./config.py) de:

- doc default tu [robot_poses.json](./robot_poses.json)
- fallback an toan neu file JSON chua du key

Da dong bo [.env.example](./.env.example) theo bo pose cuoi.

### 10.4 Bo pose runtime hien tai

Joint poses:

- `HOME_JOINTS = [0.000767, -1.030642, -0.524792, -1.877612, 1.575267, 0.000096]`
- `SCAN_APPROACH_JOINTS = [-0.054147, -1.318838, -1.252148, -1.603742, 1.616455, 0.000012]`
- `SCAN_POSE_JOINTS = [-0.054434, -1.448646, -1.614485, -1.27647, 1.616491, 0.000036]`

Cartesian poses:

- `SCAN_POSE_TCP = [0.56045, -0.127764, 0.221925, -1.963961, 2.052368, -0.355677]`
- `PLACE_APPROACH_CART = [-0.190914, 0.523067, 0.250281, -2.764561, -0.780959, 0.108603]`
- `PLACE_POINT_CART = [-0.219838, 0.570151, 0.12614, -2.911071, -0.82381, 0.049533]`
- `PLACE_RETREAT_CART = [-0.190871, 0.523073, 0.250271, -2.764585, -0.780922, 0.108531]`

Tool orientation:

- `TOOL_DOWN = [-1.963961, 2.052368, -0.355677]`

### 10.5 Trang thai hien tai

Bo pose runtime hien tai da khop voi [robot_poses.json](./robot_poses.json).

---

## 11. Su co: `tools/test_motion.py` ket noi duoc nhung robot khong di chuyen

### 11.1 Hien tuong

Khi chay:

```bash
python3 tools/test_motion.py
```

script:

- ket noi duoc `Dashboard + RTDE + URScript`
- gui lenh `HOME`
- nhung robot dung nguyen vi tri
- RTDE doc joint thuc te khong doi
- script fail voi sai so joint lon

Log dien hinh:

```text
Dừng tại: [121.49, -73.2, -31.66, -85.38, 120.84, 0.0]°
Sai so joint HOME: 121.45°
RuntimeError: HOME: robot khong den dung joint target
```

### 11.2 Nguyen nhan gan nhat trong code

[robot/urscript_client.py](./robot/urscript_client.py) truoc do gui motion theo dang mot dong lenh trần:

- `movej(...)`
- `movel(...)`

Kieu nay tren UR5 CB3 rat de roi vao tinh huong:

- socket ket noi duoc
- script duoc gui di
- nhung controller khong thuc thi motion on dinh

### 11.3 Cach xu ly da ap dung

Da sua [robot/urscript_client.py](./robot/urscript_client.py):

1. Them `send_program(...)`
   - dong goi lenh thanh chuong trinh URScript day du dang `def ... end`

2. Them `send_once(...)`
   - moi lenh motion mo socket rieng
   - gui chuong trinh
   - dong socket ngay

3. Chuyen `move_joint()`, `move_linear()`, `set_tcp()`, `set_payload()` sang dung kieu gui moi

Da sua [tools/test_motion.py](./tools/test_motion.py):

- them `ensure_joint_target_reached()`
- them `ensure_tcp_target_reached()`

Muc dich:

- khong con bao thanh cong gia chi vi robot dang dung yen
- sau moi buoc phai do joint/TCP thuc te co toi dich khong

### 11.4 Ket luan tam thoi

Sau khi sua code, neu robot van:

- ket noi duoc
- nhung dung yen

thi kha nang cao loi nam o:

- controller/policy nhan lenh motion tu port `30002`
- che do remote/local tren robot
- safety/program state tren teach pendant

chung khong con chu yeu la bug Python nua.

### 11.5 Trang thai hien tai

Code da duoc siết de chan doan dung hon.

Bai can kiem tiep tren robot that:

- pendant co popup gi khi gui `movej`
- `robotmode`
- `safetystatus`
- `programState`

---

## 12. Tong ket trang thai hien tai

Nhung gi da on:

- moi truong Python da setup xong
- binding Orbbec da duoc hop nhat
- runtime da ho tro `USB/LAN`
- hand-eye moi da vao runtime
- intrinsics moi da vao runtime
- bo pose moi da vao runtime
- `test_motion.py` da co chan doan that, khong con bao thanh cong gia

Nhung gi con phu thuoc phan cung / controller:

- camera LAN co ping duoc hay khong
- camera USB/LAN co duoc SDK mo that hay khong
- UR5 co thuc thi motion tren port `30002` hay bi controller chan

---

## 13. Lenh nen dung sau buoi nay

Kich hoat moi truong:

```bash
cd /home/tuan/Downloads/Ur5_Control-main
source .venv/bin/activate
```

Test camera USB:

```bash
python3 test_camera_view.py --backend orbbec --transport usb
```

Test camera LAN:

```bash
python3 test_camera_view.py --backend orbbec --transport lan --ip 192.168.125.10
```

Doc intrinsics:

```bash
python3 tools/get_camera_intrinsics.py --transport usb
python3 tools/get_camera_intrinsics.py --transport lan --ip 192.168.125.10
```

Test motion:

```bash
python3 tools/test_motion.py
```

---

## 7. Su co: `test_camera_view.py` va `get_camera_intrinsics.py` chua tach ro USB/LAN

### 7.1 Hien tuong

Can su dung camera qua:

- `USB`
- `LAN`

nhung tool test ban dau chua tach ro hai che do nay. Co luc script tu dong roi vao nhanh `LAN` vi tham so `ip` mac dinh, co luc thong bao loi toan bo theo huong USB.

### 7.2 Nguyen nhan

- [test_camera_view.py](./test_camera_view.py) ban dau tron chung logic `USB` va `LAN`
- `--ip` tung co gia tri mac dinh, de den viec script tu hieu la dang dung `LAN`
- thong bao debug chu yeu nhac `USB`

### 7.3 Cach sua

Da them vao:

- [test_camera_view.py](./test_camera_view.py)
- [tools/get_camera_intrinsics.py](./tools/get_camera_intrinsics.py)

tham so:

- `--transport {auto,usb,lan}`
- `--ip`

va da dua logic:

- `USB`: tim thiet bi USB
- `LAN`: tim camera theo `IP`, hoac tao network device neu can
- `auto`: cho phep detect tu do

### 7.4 Trang thai hien tai

Lenh dung:

USB:

```bash
python3 test_camera_view.py --backend orbbec --transport usb
python3 tools/get_camera_intrinsics.py --transport usb
```

LAN:

```bash
python3 test_camera_view.py --backend orbbec --transport lan --ip 192.168.125.10
python3 tools/get_camera_intrinsics.py --transport lan --ip 192.168.125.10
```

---

## 8. Su co: runtime chinh chua dung duoc cau hinh camera USB/LAN

### 8.1 Hien tuong

Khong chi tool test, ma runtime chinh cua du an cung can biet:

- dang dung `USB` hay `LAN`
- neu `LAN` thi camera dang o `IP` nao

Ban dau [vision/femto_camera.py](./vision/femto_camera.py) chua co bo tham so nay.

### 8.2 Cach sua

Da them vao [config.py](./config.py):

- `CAMERA_TRANSPORT`
- `CAMERA_IP`
- `CAMERA_NET_PORT`

Da cap nhat [vision/femto_camera.py](./vision/femto_camera.py) de:

- enumerate duoc `USB` va `LAN`
- loc thiet bi theo `transport`
- neu `LAN` thi uu tien camera dung `IP`
- runtime chinh trong [core/pick_place.py](./core/pick_place.py) tu dong huong theo config nay

### 8.3 Trang thai hien tai

Da xac nhan:

- `FemtoCamera transport` doc dung tu config
- `FemtoCamera ip` doc dung tu config
- `FemtoCamera port` doc dung tu config

---

## 9. Su co: camera LAN khong ping duoc / khong mo duoc

### 9.1 Hien tuong

Da gap cac loi:

- `Send control transfer failed`
- `Get device in recovery mode failed`
- camera LAN khong ping duoc
- SDK co the bi treo neu goi truc tiep `create_net_device()`

### 9.2 Nguyen nhan chinh

Van de o lop mang/phu kien, khong phai o lop YOLO:

- `PC` va camera khong cung subnet
- `enp2s0` dang o `192.168.125.100`, trong khi camera de `192.168.1.10`
- hoac camera chua cau hinh Ethernet dung

### 9.3 Cach xu ly / bai hoc rut ra

Da thong nhat:

- khong dung `IPv6`
- nen dua camera ve cung dai `192.168.125.x` voi robot va PC

So do de xuat:

- `PC`: `192.168.125.100`
- `Robot`: `192.168.125.11`
- `Camera`: `192.168.125.10`

### 9.4 Trang thai hien tai

Code da ho tro `LAN`, nhung neu `ping` khong thong thi:

- van de van la mang / camera mode / IP
- khong phai bug Python

---

## 10. Su co: hand-eye calibration moi sau khi thay gripper

### 10.1 Hien tuong

Ban yeu cau:

- dung ma tran hand-eye moi
- vi da thay gripper moi

### 10.2 Kiem tra

Da doc:

- [hand_eye_result.json](./hand_eye_result.json)
- ma tran default cu trong [config.py](./config.py)

Da so sanh va thay:

- ma tran moi lech dang ke so voi ma tran cu
- dieu nay phu hop voi viec da thay gripper / thay doi hinh hoc TCP-camera

### 10.3 Cach cap nhat

Da thay `_DEFAULT_T_CAM_TO_TCP` trong [config.py](./config.py) bang ma tran moi:

```text
[[ 0.976265,  0.168094,  0.136569,  0.108484],
 [-0.171848,  0.984992,  0.016089, -0.128004],
 [-0.131814, -0.039177,  0.990500, -0.147013],
 [ 0.000000,  0.000000,  0.000000,  1.000000]]
```

### 10.4 Trang thai hien tai

Runtime mac dinh hien tai da dung ma tran moi.

Luu y:

- file [hand_eye_result.json](./hand_eye_result.json) van nen duoc giu de doi chieu
- nhung phan runtime chinh se doc ma tran moi tu [config.py](./config.py) neu `.env` khong override

---

## 11. Su co: can cap nhat bo camera intrinsics moi

### 11.1 Hien tuong

Da co bo intrinsics moi trong:

- [camera_intrinsics.json](./camera_intrinsics.json)

voi:

- `fx = 1114.278564453125`
- `fy = 1114.118408203125`
- `cx = 937.609375`
- `cy = 518.2891845703125`
- `width = 1920`
- `height = 1080`

nhung mot so tai lieu va file mau van con:

- `605 / 640 / 360`
- `1280x720`

### 11.2 Nguyen nhan

Bo intrinsics cu trong:

- [.env.example](./.env.example)
- [PARAMETERS_CHECKLIST.md](./PARAMETERS_CHECKLIST.md)

chua duoc dong bo lai voi file `camera_intrinsics.json`.

### 11.3 Cach sua

Da cap nhat:

- [config.py](./config.py)
- [.env.example](./.env.example)
- [PARAMETERS_CHECKLIST.md](./PARAMETERS_CHECKLIST.md)

Trong `config.py`, default intrinsics da thanh:

- `1920x1080`
- `fx/fy/cx/cy` moi

### 11.4 Trang thai hien tai

Runtime doc ra:

- `CAM_FX = 1114.278564453125`
- `CAM_FY = 1114.118408203125`
- `CAM_CX = 937.609375`
- `CAM_CY = 518.2891845703125`
- `CAM_CALIB_WIDTH = 1920`
- `CAM_CALIB_HEIGHT = 1080`

---

## 12. Su co: `hand_eye_calibration.py` chet ngay vi bao thieu `pyorbbecsdk`

### 12.1 Hien tuong

Khi chay:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/hand_eye_calibration.py
```

script bao:

```text
RuntimeError: Chưa cài pyorbbecsdk trong môi trường hiện tại
```

### 12.2 Nguyen nhan

Script duoc chay tu thu muc `tools/`, nen:

- khong thay wrapper o root repo
- chi import truc tiep `pyorbbecsdk`

### 12.3 Cach sua

Da sua [tools/hand_eye_calibration.py](./tools/hand_eye_calibration.py):

- them repo root vao `sys.path`
- uu tien `import ob`
- fallback `import pyorbbecsdk as ob`
- doc `CAMERA_TRANSPORT`, `CAMERA_IP`, `CAMERA_NET_PORT` tu [config.py](./config.py)

### 12.4 Trang thai hien tai

Da xac nhan:

- file import duoc
- `ob is not None`

Neu con loi nua khi chay tool nay, loi tiep theo se la:

- mo camera that
- detect checkerboard
- network/USB quyen truy cap

chu khong con la loi “thieu pyorbbecsdk”.

---

## 13. Cac file da duoc cap nhat hom nay

File moi:

- [ob.py](./ob.py)
- [pyorbbecsdk.py](./pyorbbecsdk.py)
- [sitecustomize.py](./sitecustomize.py)
- [Vấn_đề_hôm_nay.md](./Vấn_đề_hôm_nay.md)

File da sua:

- [config.py](./config.py)
- [vision/detector.py](./vision/detector.py)
- [vision/femto_camera.py](./vision/femto_camera.py)
- [test_camera_view.py](./test_camera_view.py)
- [tools/get_camera_intrinsics.py](./tools/get_camera_intrinsics.py)
- [tools/hand_eye_calibration.py](./tools/hand_eye_calibration.py)
- [.env.example](./.env.example)
- [README.md](./README.md)
- [QUICKSTART.md](./QUICKSTART.md)
- [PARAMETERS_CHECKLIST.md](./PARAMETERS_CHECKLIST.md)

---

## 14. Cach chay dung sau khi da sua

### 14.1 Kich hoat moi truong

```bash
cd /home/tuan/Downloads/Ur5_Control-main
source .venv/bin/activate
```

### 14.2 Test camera USB

```bash
python3 test_camera_view.py --backend orbbec --transport usb
python3 tools/get_camera_intrinsics.py --transport usb
```

### 14.3 Test camera LAN

```bash
python3 test_camera_view.py --backend orbbec --transport lan --ip 192.168.125.10
python3 tools/get_camera_intrinsics.py --transport lan --ip 192.168.125.10
```

### 14.4 Chay hand-eye calibration

```bash
python3 tools/hand_eye_calibration.py
```

Luu y:

- dung `camera_intrinsics.json` da update
- dam bao camera thuc su mo duoc theo transport dang chon

---

## 15. Nhung diem can tiep tuc kiem tra

1. Xac nhan camera that dang ket noi theo `USB` hay `LAN`
2. Neu `LAN`, xac minh:
   - ping thong
   - camera va PC cung subnet
   - camera da o mode Ethernet
3. Chay lai:
   - `tools/get_camera_intrinsics.py`
   - `tools/hand_eye_calibration.py`
4. Sau khi xac minh camera on dinh, test lai:
   - `tools/test_scanpose_touch.py`
   - `tools/test_phase2.py`

---

## 16. Ket luan

Phan lon loi hom nay khong nam o YOLO hay logic pick-place, ma nam o:

- moi truong Python
- binding camera
- su khac nhau giua cach chay script tu root repo va tu `tools/`
- viec chua tach ro `USB` va `LAN`
- du lieu calibration / intrinsics chua dong bo

Nhung thay doi moi da giai quyet duoc:

- setup Python va vision stack
- bo wrapper camera bundle trong repo
- tach ro camera `USB/LAN`
- update `T_CAM_TO_TCP` moi
- update `camera_intrinsics` moi

Phan con lai can tiep tuc xac nhan tren phan cung that:

- camera co mo duoc on dinh khong
- mang LAN co thong khong
- checkerboard detect co on dinh khong
- hand-eye moi co van hanh tot tren robot that khong
