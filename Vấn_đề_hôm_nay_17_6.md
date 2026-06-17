# Vấn đề hôm nay 17/6

Tai lieu nay tong hop cac van de da gap trong buoi test ngay 17/06, nhat la nhom van de lien quan den:

- hand-eye calibration `camera -> TCP`
- depth ROI khi nhin phoi trang trong khay den
- tool debug `scanpose_touch`
- logic chan an toan de robot khong di nguoc len hoac lao sai huong

Muc tieu:

- ghi lai logics da sua trong code
- tach ro van de nao da duoc xac nhan
- chi ra van de nao con phai test tiep tren phan cung that

---

## 1. Tong ket nhanh

Hom nay da xu ly va xac nhan 6 nhom van de chinh:

1. Cap nhat ma tran hand-eye moi tu ket qua calibration 18 pose
2. Xac dinh ro su khac nhau giua `raw depth` va `safe/clamped depth`
3. Chung minh duoc depth camera co luc hong o tam phoi, nhung khong phai luc nao cung sai toan bo
4. Cai tien logic lay depth tu bbox phoi de khong con phu thuoc cung vao tam phoi
5. Bo sung log chi tiet de backend in ra toa do phoi ma no dang tinh
6. Xac dinh thu muc hien tai khong co git metadata day du, nen chua the push len GitHub truc tiep

Trang thai hien tai:

- `hand_eye_result.json` da duoc cap nhat bang ket qua calibration moi
- ma tran moi hop ly hon ma tran cu ve chieu sau camera-TCP
- `test_scanpose_touch.py` da co log `Target 3D estimate`
- `test_scanpose_touch.py` da hien thi ro `raw_depth`, `safe_depth`, `p_cam`, `p_base`
- logic chon depth da duoc mo rong: `inner`, `inner_relaxed`, `nearest`, `nearest_relaxed`, `bbox`
- robot van duoc chan an toan neu `touch_pose` van nam cao hon TCP luc chup

---

## 2. Van de: ma tran hand-eye cu day robot di nguoc len

### 2.1 Hien tuong

O cac lan test dau, log cho thay:

- `raw depth` quanh `230-243 mm`
- nhung backend lai ep thanh `safe depth ~386.7 mm`
- robot tinh ra `touch pose` cao hon TCP luc chup

He qua:

- robot co nguy co di nguoc len thay vi di xuong cham phoi

### 2.2 Chan doan

Da them tool debug hinh hoc:

- [tools/debug_hand_eye_geometry.py](./tools/debug_hand_eye_geometry.py)

Ket qua voi ma tran cu:

- `TCP origin in camera frame z ~ 366.7 mm`
- `min safe depth ~ 386.7 mm`

Dieu nay khong phu hop voi co khi that:

- tool dai khoang `30 cm`
- camera gan tren tool, cach tool khoang `9.5 cm`

Ket luan:

- ma tran cu dat camera qua xa so voi TCP theo truc nhin
- day la ly do logic runtime cu phai ep depth len de tranh robot di nguoc

### 2.3 Cach xu ly

Da thu lai hand-eye calibration voi:

- `18 poses`
- sau do cap nhat vao [hand_eye_result.json](./hand_eye_result.json)

### 2.4 Ket qua

Ma tran moi:

```text
T_cam_to_tcp t = [64.7, -242.6, -162.1] mm
TCP origin in camera frame z = 193.4 mm
min safe depth = 213.4 mm
```

So voi ma tran cu:

- hop ly hon nhieu
- khong con ep depth len > `380 mm`
- phu hop hon voi kich thuoc co khi that

---

## 3. Van de: nham lan giua `raw depth` va `safe depth`

### 3.1 Hien tuong

Preview cu hien:

- `depth = 386.7 mm`

nhung do khong phai so camera do duoc, ma la so da bi code `clamp`.

### 3.2 Nguyen nhan

Logic cu:

- neu `raw depth` nho hon `min safe depth`
- code tu dong nang gia tri depth len
- roi van hien thi depth da nang len tren preview

Dieu nay gay hieu nham rang:

- camera tra depth > `300 mm`

trong khi thuc te camera chi do `~230-240 mm`.

### 3.3 Cach xu ly

Da sua [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py):

- tach ro `raw_depth_mm`
- tach ro `safe_depth_mm`
- neu chenh lech qua xa, dung motion ngay

Nguong hien tai:

- `DEPTH_MAX_CLAMP_DELTA_MM = 80`

### 3.4 Ket qua

Neu `raw depth` va `safe depth` lech qua xa, script in:

```text
Depth capture dang sai, dung motion de tranh tinh pick sai.
```

Dieu nay giup debug dung nguyen nhan, khong bi `safe depth` che mat `raw depth`.

---

## 4. Van de: depth bi hole o tam phoi

### 4.1 Hien tuong

Nhieu frame co:

- `bbox_stats` rat dep
- `inner_stats` co depth kha on
- nhung `center_stats.valid = 0`

Tuc la:

- tam phoi bi mat depth
- vi tri quanh tam van co depth

### 4.2 Ket luan

Day khong phai loi YOLO chinh.

Cung khong phai luc nao camera cung "sai depth" toan bo.

Van de that:

- mat phoi trang / phan xa
- IR/depth sensor mat du lieu ngay vung tam
- nhung van do duoc depth o vung xung quanh

### 4.3 Cach xu ly trong code

Da cai tien logic chon depth trong:

- [vision/detector.py](./vision/detector.py)
- [vision/femto_camera.py](./vision/femto_camera.py)

Tu logic cu:

- chi uu tien micro ROI tam

sang logic moi:

1. `inner`
2. `inner_relaxed`
3. `nearest`
4. `nearest_relaxed`
5. `bbox`

Tuc la:

- khong con bat buoc phai thay dung tam phoi moi cho phep lay depth
- backend se uu tien vung dinh phoi on dinh nhat trong bbox

### 4.4 Cac nguong da mo rong

Trong [config.py](./config.py):

- `DEPTH_INNER_MIN_VALID_RATIO = 0.05`
- `DEPTH_INNER_RELAXED_MIN_VALID_RATIO = 0.03`
- `DEPTH_BBOX_MIN_VALID_RATIO = 0.20`
- `DEPTH_NEAREST_RELAXED_MAX_CENTER_DIST_PX = 30`

Muc tieu:

- giam truong hop bo qua target chi vi tam phoi bi hole
- van giu guard de tranh dung depth qua loang

---

## 5. Van de: can nhin thay ro ROI depth va pixel valid

### 5.1 Hien tuong

Chi nhin log text thi kho thay ngay:

- vung nao la center ROI
- vung nao la inner ROI
- pixel depth hop le dang nam o dau

### 5.2 Cach xu ly

Da sua [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py):

- ve `center ROI`
- ve `inner ROI`
- ve sample cac pixel valid trong `inner ROI`
- ve tam bbox

Preview hien tai giup phan biet nhanh:

- do nhan sai
- do anh sang
- do hole dung ngay tam phoi

---

## 6. Van de: can log ra toa do phoi ma backend tinh duoc

### 6.1 Hien tuong

Truoc day chi co:

- `p_cam`
- `p_base`

nhung chua du de doi chieu nhanh voi thuc te.

### 6.2 Cach xu ly

Da them block log:

```text
Target 3D estimate: {...}
```

vao [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py)

Trong do co:

- `bbox`
- `center_px`
- `raw_depth_mm`
- `safe_depth_mm`
- `tcp_at_capture_m`
- `p_cam_m`
- `p_base_m`
- `delta_vs_tcp_m`
- `delta_vs_scanpose_m`
- `approach_pose_m_rad`
- `touch_pose_m_rad`

### 6.3 Y nghia

Giu log nay de doi chieu:

- backend dang nghi phoi nam o dau trong base frame
- no lech bao xa so voi TCP luc chup
- no lech bao xa so voi `SCAN_POSE`

---

## 7. Ket qua test cuoi ngay voi ma tran moi

### 7.1 Frame dep nhat

Da co frame ma:

- `raw_depth = safe_depth = 239 mm`
- `center_stats.ratio = 1.0`
- `inner_stats.ratio = 1.0`

Tuc la:

- depth da sach
- khong con bi hole o tam
- co the danh gia duoc logic transform ma khong bi depth che mat

### 7.2 Toa do backend tinh ra

Log:

```text
p_cam = [0.0489, -0.0485, 0.2390]
p_base = [0.8497, -0.3227, 0.1612]
tcp_at_capture = [0.6045, -0.1496, 0.1627]
delta_vs_tcp = [+0.2452, -0.1731, -0.0015]
```

### 7.3 Ket luan

Du depth da dep:

- backend van tinh phoi lech kha xa theo `XY`
- khoang `24.5 cm` theo `X`
- khoang `17.3 cm` theo `Y`
- va gan nhu cung do cao voi TCP

He qua:

- `touch_pose.z` van cao hon TCP luc chup
- guard van chan

Day la dau hieu cho thay:

- hand-eye moi da tot hon hand-eye cu rat nhieu
- nhung transform `camera -> TCP -> base` van chua khop hoan toan voi setup that, dac biet o `XY`

---

## 8. Nghi ngo ve TCP va hand-eye

Trong buoi nay da kiem tra va thao luan:

- neu `TCP` day sai, toan bo hand-eye cung sai he quy chieu
- neu `raw depth` dung nhung transform van bat hop ly, phai nghi ngo `T_cam_to_tcp`

Trang thai hien tai:

- TCP da duoc day lai
- hand-eye da thu lai 18 pose
- ma tran moi da hop ly hon ro ret

Nhung:

- can tiep tuc test de xac nhan phan `XY`
- chua the ket luan da dung hoan toan

---

## 9. Cong cu moi da them trong buoi nay

Da them / nang cap:

- [tools/debug_hand_eye_geometry.py](./tools/debug_hand_eye_geometry.py)
- [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py)
- [vision/detector.py](./vision/detector.py)
- [vision/femto_camera.py](./vision/femto_camera.py)
- [tools/validate_config.py](./tools/validate_config.py)

Cong dung:

- soi hinh hoc `camera <-> TCP`
- in `min safe depth`
- debug ROI depth
- in toa do phoi backend tinh duoc
- chan motion neu point tinh ra van bat hop ly

---

## 10. Van de Git/GitHub

### 10.1 Hien tuong

Ban muon:

- tong hop file hom nay
- up lai folder len GitHub

### 10.2 Ket qua kiem tra

Thu muc hien tai co:

- `/.git`

nhung thu muc nay dang rong, khong co:

- `HEAD`
- `objects`
- `refs`

Nghia la:

- day khong phai git repo hoan chinh
- khong the `git status`, `git commit`, `git push` truc tiep tu day

### 10.3 Ket luan

Hien tai:

- da tao duoc tai lieu tong hop trong repo
- nhung chua the day len GitHub tu thu muc nay neu khong khoi phuc repo git that

Can lam mot trong 2 cach:

1. mo dung ban repo co `.git` day du roi copy cac file da sua vao do
2. hoac `git init` lai, add remote, commit, push bang tay

---

## 11. Ket luan cuoi ngay

Nhung gi da xac nhan chac chan:

- depth camera khong phai luc nao cung sai; co frame rat sach
- tam phoi bi hole la van de lap lai, nhung da co logic depth moi de giam anh huong
- ma tran hand-eye moi tot hon ma tran cu ro ret
- hand-eye cu la mot nguyen nhan lon cua loi "robot di nguoc len"
- hien tai depth debug, ROI debug, va target 3D estimate da du de tiep tuc test co he thong

Nhung gi con can test tiep:

- xac nhan phan `XY` cua hand-eye moi
- so lai `p_base` backend tinh ra voi vi tri phoi that tren ban may
- xac dinh xem co can thu them 1 lan calibration nua voi bo pose da trai rong hon khong

---

## 12. File lien quan trong buoi nay

- [hand_eye_result.json](./hand_eye_result.json)
- [config.py](./config.py)
- [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py)
- [tools/debug_hand_eye_geometry.py](./tools/debug_hand_eye_geometry.py)
- [tools/validate_config.py](./tools/validate_config.py)
- [vision/detector.py](./vision/detector.py)
- [vision/femto_camera.py](./vision/femto_camera.py)
- [robot_poses.json](./robot_poses.json)
