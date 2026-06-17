# Vấn đề hôm nay 1

Tai lieu nay la ban cap nhat bo sung cho [Vấn_đề_hôm_nay.md](./Vấn_đề_hôm_nay.md), tap trung vao cac thay doi moi nhat lien quan den:

- motion test tren UR5
- logic cac phase 1, 2, 3
- camera Orbbec LAN viewer
- tool `test_scanpose_touch.py` de nhin ro camera dang thay gi

Muc tieu cua file nay:

- ghi lai dung nhung loi vua gap
- giai thich nguyen nhan thuc te
- chi ro file nao da sua
- de ngay mai len lab co the test lai nhanh, khong lap lai vong debug cu

---

## 1. Tong ket nhanh ket qua cap nhat nay

Da xu ly 5 cum van de chinh:

1. `tools/test_motion.py` gui lenh nhung UR5 khong chay
2. Don dep file `tools/test_motion11.py` va xac nhan logic phase khong bi anh huong
3. `test_camera_view.py` ket noi duoc camera LAN nhung stream bi dung sau vai frame
4. `test_camera_view.py` chua the hien ro khi stream dang bi stale / dong bang
5. `tools/test_scanpose_touch.py` chua cho nhin truc tiep camera dang thay gi truoc khi robot touch

Trang thai hien tai:

- `tools/test_motion.py` da chay duoc
- `tools/test_motion11.py` da xoa
- logic `phase 1`, `phase 2`, `phase 3` trong runtime chinh van giu dung
- `test_camera_view.py` da co ho tro `LAN + port + stale detect + auto reconnect`
- `tools/test_scanpose_touch.py` da co preview anh, bbox, target, `p_cam`, `p_base`, va hoi xac nhan truoc khi motion

---

## 2. Su co: `tools/test_motion.py` gui lenh nhung robot khong chay

### 2.1 Hien tuong

Khi chay:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/test_motion.py
```

robot in ra:

- da ket noi `RTDE + URScript`
- da gui lenh `HOME`
- nhung joint thuc te khong doi
- sau do bao:

```text
HOME: robot khong den dung joint target
```

### 2.2 Nguyen nhan

Ban `tools/test_motion.py` luc do:

- co `RTDEClient`
- co `URScriptClient`
- nhung khong co buoc `Dashboard.prepare_to_run()`

Trong khi do controller UR CB3 o lab lai nhay cam voi thu tu:

1. safety / ready check
2. power/brake release settle
3. sau do moi nen nhan `movej` / `movel`

Neu bo qua buoc nay:

- socket URScript van nhan
- script van duoc gui
- nhung robot co the khong thuc thi motion

### 2.3 Cach sua

Da sua [tools/test_motion.py](./tools/test_motion.py):

- them `DashboardClient`
- them `prepare_robot_for_motion()`
- goi `dashboard.precheck_ready()`
- goi `dashboard.prepare_to_run()`
- them `time.sleep(1.5)` sau `prepare_to_run()`
- giu nguyen cac buoc verify:
  - `ensure_joint_target_reached()`
  - `ensure_tcp_target_reached()`

### 2.4 Ket qua

Sau sua, `tools/test_motion.py` da chay duoc.

Day la ket luan quan trong:

- loi khong nam o teach point
- loi khong nam o `movej/movel` syntax
- loi nam o buoc chuan bi trang thai chay cua controller

---

## 3. Su co: ton tai song song `test_motion.py` va `test_motion11.py`

### 3.1 Hien tuong

Trong repo co 2 file:

- [tools/test_motion.py](./tools/test_motion.py)
- `tools/test_motion11.py`

Gay roi khi van hanh vi:

- nguoi dung khong biet nen chay file nao
- 2 file co logic rat gan nhau nhung khong hoan toan dong bo

### 3.2 Cach xu ly

Sau khi `tools/test_motion.py` da duoc sua de chay on dinh:

- giu lai [tools/test_motion.py](./tools/test_motion.py)
- xoa `tools/test_motion11.py`

### 3.3 Ket qua

Hien tai repo chi con 1 file motion test chinh:

- [tools/test_motion.py](./tools/test_motion.py)

Dieu nay giup tranh nham lan trong buoi test lab.

---

## 4. Kiem tra lai logic cac phase sau khi sua `test_motion.py`

### 4.1 Muc tieu kiem tra

Can xac nhan xem viec sua `tools/test_motion.py` co vo tinh lam thay doi logic runtime chinh hay khong.

### 4.2 Ket qua ra soat

Da kiem tra [core/pick_place.py](./core/pick_place.py), [tools/test_phase2.py](./tools/test_phase2.py), [tools/test_phase3.py](./tools/test_phase3.py).

Ket luan:

- logic `phase 1` khong doi
- logic `phase 2` khong doi
- logic `phase 3` khong doi

### 4.3 Phase 1 hien tai

`phase 1 = static_motion_only`

Luong:

1. `HOME`
2. `SCAN_APPROACH`
3. `SCAN_POSE`
4. `PLACE_APPROACH`
5. `PLACE_POINT`
6. `PLACE_RETREAT`
7. `HOME`

Khong dung vision, khong grip, chi kiem tra motion.

### 4.4 Phase 2 hien tai

`phase 2 = motion_plus_vision`

Luong hien tai trong code:

1. prepare robot
2. mo gripper
3. `HOME`
4. `SCAN_APPROACH`
5. `SCAN_POSE`
6. chup anh + YOLO + depth + transform
7. di toi `approach`
8. di xuong `final pose`
9. grip
10. retreat
11. `PLACE_APPROACH`
12. `PLACE_POINT`
13. release
14. `PLACE_RETREAT`
15. `HOME`

Luu y:

- day la full single pick-place
- khong phai chi detect roi cham

Tool tien de de kiem calibration van la:

- [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py)

### 4.5 Phase 3 hien tai

`phase 3 = full_flow_sim_grip`

Luong:

1. prepare robot
2. mo gripper
3. `HOME`
4. `SCAN_APPROACH`
5. `SCAN_POSE`
6. quet lan 1 de dem so phoi
7. lap lai theo multi-shot:
   - quay lai scan
   - chup anh moi
   - detect target moi
   - tinh toa do moi
   - pick
   - place
   - quay lai scan
8. het phoi thi ve `HOME`

Ket luan:

- viec sua `tools/test_motion.py` khong lam sai logic phase
- runtime chinh da co `dashboard.prepare_to_run()` va `sleep(1.5)` san

---

## 5. Su co: `test_camera_view.py` LAN ping duoc nhung khong mo duoc dung cach

### 5.1 Hien tuong

Ban dau:

- camera LAN ping duoc
- nhung `test_camera_view.py` van khong mo duoc hoac mo khong on dinh

Co luc gap:

```text
No route to host
```

ve sau ket noi duoc nhung stream khong song dung.

### 5.2 Nguyen nhan

Tool viewer cu co vai diem yeu:

- hard-code net port theo kieu cu
- chua debug ro `USB` va `LAN`
- khi LAN stream dung thi van giu anh cu, trong nhu dang "capture"
- khong thong bao ro la stream da stale

### 5.3 Cach sua

Da sua [test_camera_view.py](./test_camera_view.py):

- them `import config`
- them `--transport {auto,usb,lan}`
- them `--ip`
- them `--port`
- su dung `config.CAMERA_NET_PORT`
- khi dung LAN:
  - thu port cau hinh truoc
  - fallback `8090` neu can
- thong bao debug ro hon khi mo camera that bai

### 5.4 Ket luan ky thuat quan trong

`ping` thanh cong chi co nghia:

- may tinh thay camera tren mang

`ping` **khong** co nghia:

- Orbbec network service dang mo dung
- SDK se stream on dinh

Do do, viec:

```bash
nc -vz 192.168.125.10 8090
```

van quan trong de check dich vu TCP.

---

## 6. Su co: camera mo duoc nhung khong hien video that, chi giong nhu "capture"

### 6.1 Hien tuong

Sau khi ket noi duoc camera LAN, cua so viewer mo ra nhung anh khong nhuc nhich.

Tren overlay xuat hien:

- `Frames: 8`
- `STALE frame: 39622 ms`

### 6.2 Dien giai dung

Day khong phai la do OpenCV chi chup 1 tam anh.

No co nghia la:

- camera da stream duoc vai frame dau
- sau do luong frame moi bi dung
- viewer dang giu frame cuoi cung de hien thi

Nen nhin bang mat se thay giong "capture".

### 6.3 Cach sua trong code

Da sua [test_camera_view.py](./test_camera_view.py):

- retry lay `frameset` thay vi cho 1 lan roi bo
- neu SDK ho tro thi bat `FULL_FRAME_REQUIRE`
- theo doi:
  - `last_frame_wall_time`
  - `frame_counter`
- overlay them:
  - `Frames: ...`
  - `STALE frame: ... ms`
  - `Reconnects: ...`
- them `reopen()` cho backend
- neu stale qua nguong thi tu dong reconnect

### 6.4 Tham so moi da them

Trong [test_camera_view.py](./test_camera_view.py):

- `--reconnect-stale-ms`
- `--reconnect-delay-s`

Vi du:

```bash
python3 test_camera_view.py \
  --backend orbbec \
  --transport lan \
  --ip 192.168.125.10 \
  --port 8090 \
  --reconnect-stale-ms 2500
```

### 6.5 Ket luan

Neu sau cap nhat ma van co pattern:

- nhan vai frame dau
- stale
- reconnect
- lai stale

thi loi luc do khong con nam o code viewer nua, ma nam o:

- nhanh LAN cua Orbbec SDK
- camera network streaming
- bandwidth / mode Ethernet / firmware / switch

---

## 7. Su co: `test_scanpose_touch.py` khong cho nhin camera dang thay gi

### 7.1 Hien tuong

Khi chay:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/test_scanpose_touch.py
```

tool chi in log:

- so detections
- target center
- depth
- `p_cam`
- `p_base`

nhung khong co preview anh de nguoi dung xac nhan:

- camera dang thay gi
- YOLO dang box vao dau
- target nao dang duoc chon

### 7.2 Hau qua

Rat kho xac dinh nhanh loi dang nam o:

- YOLO detect sai phoi
- target selection sai
- depth tai bbox sai
- transform `camera -> base` sai

### 7.3 Cach sua

Da nang cap [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py):

- them `cv2`
- them preview window:
  - `SCANPOSE TOUCH PREVIEW`
- ve tat ca bbox detections
- target duoc chon se to mau xanh
- ve tam bbox
- ve tam frame
- in ngay tren anh:
  - kich thuoc frame
  - so detections
  - `target_px`
  - `depth`
  - `p_cam`
  - `p_base`
- luu preview vao:
  - `captures/scanpose_touch/...`
- hoi nguoi van hanh xac nhan truoc khi robot motion

### 7.4 Tham so moi

Da them:

- `--save-preview-dir`
- `--no-preview`

### 7.5 Gia tri van hanh

Day la cap nhat rat quan trong cho buoi calibration / phase 2 test, vi:

- truoc khi robot ha xuong, co the nhin thay camera dang chon dung phoi hay khong
- neu bbox / center lech, co the dung ngay, khong cho robot motion

---

## 8. Su co: `test_scanpose_touch.py` chan target vi lech qua xa

### 8.1 Hien tuong

Tool in:

```text
Loi: target pose lech qua xa so voi TCP tai luc capture. planar=0.256m, dz=-0.053m. Dung de tranh va cham.
```

### 8.2 Y nghia dung

Day khong phai la tool bi hong.

Day la guard rail chu dong de chan truong hop:

- target tinh ra qua xa vi tri TCP hien tai
- co nguy co do sai detect / sai transform / sai depth
- neu van cho robot di thi de va cham

### 8.3 Nguyen nhan kha nang cao trong case nay

Theo log:

- frame `1920x1080`
- calibration baseline `1280x720`
- intrinsics dang duoc auto-scale
- detections = 2
- target co the chua phai phoi ban muon cham

Nen can nhin preview de biet:

1. box nao dang duoc chon
2. diem center co nam dung tren phoi khong
3. depth tai box do co hop ly khong

### 8.4 Ket luan

Guard rail nay nen giu lai.

Khong duoc xoa vo dieu kien, vi no la lop bao ve an toan trong giai doan calibration.

---

## 9. File da thay doi trong dot cap nhat nay

Da sua:

- [tools/test_motion.py](./tools/test_motion.py)
- [test_camera_view.py](./test_camera_view.py)
- [tools/test_scanpose_touch.py](./tools/test_scanpose_touch.py)

Da xoa:

- `tools/test_motion11.py`

Da xac nhan logic:

- [core/pick_place.py](./core/pick_place.py)
- [tools/test_phase2.py](./tools/test_phase2.py)
- [tools/test_phase3.py](./tools/test_phase3.py)

---

## 10. Trang thai hien tai de tiep tuc test tai lab

### 10.1 Motion

Co the test motion co ban bang:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/test_motion.py
```

### 10.2 Camera viewer

Co the debug camera LAN bang:

```bash
python3 test_camera_view.py \
  --backend orbbec \
  --transport lan \
  --ip 192.168.125.10 \
  --port 8090 \
  --reconnect-stale-ms 2500
```

### 10.3 Preview touch test

Co the test phase 2 tien de bang:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/test_scanpose_touch.py
```

Neu chi muon luu preview:

```bash
/usr/bin/python3 /home/tuan/Downloads/Ur5_Control-main/tools/test_scanpose_touch.py --no-preview
```

---

## 11. Ke hoach debug hop ly tiep theo

Thu tu uu tien:

1. Chay `tools/test_motion.py` de xac nhan robot di dung taught points
2. Chay `tools/test_scanpose_touch.py` de xem preview camera, bbox, target
3. Neu target chon sai:
   - xem lai model
   - xem lai select target
   - xem lai depth
4. Neu target chon dung nhung `p_base` van lech:
   - xem lai `T_CAM_TO_TCP`
   - xem lai `camera_intrinsics`
5. Neu camera LAN van stale lien tuc:
   - uu tien test USB de tach bai toan mang ra khoi bai toan vision

---

## 12. Ket luan cuoi cung

Trong dot cap nhat nay, co 3 diem quan trong nhat:

1. `tools/test_motion.py` da duoc sua dung goc loi controller readiness, nen da chay duoc.
2. Logic runtime cua `phase 1`, `phase 2`, `phase 3` khong bi pha vo boi sua doi nay.
3. Cong cu vision test da manh hon ro ret:
   - `test_camera_view.py` cho biet stream LAN co stale hay khong
   - `tools/test_scanpose_touch.py` cho nhin truc tiep camera thay gi truoc khi robot motion

Ket qua la:

- viec debug ngoai lab se nhanh hon
- de tach loi motion, loi camera, loi detect, va loi transform hon
- giai doan test calibration / touch part an toan hon
