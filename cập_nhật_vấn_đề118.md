# Cập nhật vấn đề 18/6

Tai lieu nay tong hop lai cac van de da gap trong buoi debug ngay 18/06 quanh 4 nhom chinh:

- TCP runtime cua UR5
- hand-eye calibration `T_cam_to_tcp`
- logic tinh toa do phoi `p_cam -> p_base`
- motion test `SCAN_POSE -> approach -> touch`

Muc tieu cua tai lieu:

- ghi ro van de nao da duoc xac nhan
- tach ro loi motion voi loi vision/calibration
- ghi lai cac nham lan da gap de tranh lap lai
- chot huong xu ly dung cho cac buoi test tiep theo

---

## 1. Tong ket nhanh

Ket luan quan trong nhat cua buoi nay:

1. TCP moi cua robot da duoc cap nhat va test lai, motion da di dung den pose duoc tinh.
2. Logic nhan ma tran `camera -> tcp -> base` trong code khong bi dao chieu hay nhan sai thu tu.
3. Hand-eye calibration cu khong con dong bo sau khi doi TCP, can calib lai.
4. Sau khi thu hand-eye PARK moi va test lai, robot da thuc hien dung `approach` va `touch` voi sai so gan nhu 0 mm trong bai test.
5. Van de "robot len qua cao roi moi ha xuong" den tu logic build pose va offset approach, khong phai do TCP bi sai.
6. Van de "diem gap lech ve 1 phia" co the do systematic bias va nen xu ly bang `PICK_OFFSET_X/Y/Z`, nhung phai nho don vi la `met`.
7. Da tung gap loi nhap offset `-0.5` thay vi `-0.005`, gay lech target toi `500 mm`.

---

## 2. TCP moi da xac nhan

TCP moi tren pendant sau khi setup lai:

```text
X  = -1.15 mm
Y  =  9.87 mm
Z  = 315.35 mm
RX = 0.0185 rad
RY = -0.0294 rad
RZ = 3.1303 rad
```

Da doi sang `met` va cap nhat vao runtime config:

```python
TCP_OFFSET = [
    -0.00115,
    0.00987,
    0.31535,
    0.0185,
    -0.0294,
    3.1303,
]
```

### 2.1 Dieu can nho

`TCP_OFFSET` va `PICK_APPROACH_OFFSET_Z` la 2 khai niem khac nhau:

- `TCP_OFFSET`
  - mo ta vi tri TCP so voi flange/tool
  - la tham so co khi
  - don vi `met`
- `PICK_APPROACH_OFFSET_Z`
  - mo ta robot se dung cao hon diem gap bao nhieu truoc khi ha xuong
  - la tham so motion planning
  - don vi `met`

Vi du:

- `TCP_OFFSET_X = -0.00115` nghia la `-1.15 mm`
- `PICK_APPROACH_OFFSET_Z = 0.15` nghia la `150 mm`

Day la ly do da tung co nham lan: thay `0.15` trong code va nghi no lien quan den TCP. Thuc te khong phai.

---

## 3. Van de cu: robot khong xuong touch pose

Ban dau, log cho thay:

- `approach_pose` di dung
- `touch_pose` tinh ra thap hon `approach_pose`
- nhung actual TCP tai buoc `touch` van dung nguyen o cao do `approach`

### 3.1 Nhan dinh ban dau

Co 2 nhom nghi ngo chinh:

1. motion race condition / URScript runtime issue
2. TCP / payload / CoG / hand-eye / target estimation co van de

### 3.2 Huong sua da ap dung

Da sua theo huong:

- bundle `set_tcp + set_payload + move*` trong cung mot URScript program
- tang `RTDE_MOTION_START_TIMEOUT`
- them `CB3_MOTION_PRE_WAIT_SLEEP_S`
- thay hardcode `motion_start_timeout=0.5`

Ket qua sau khi fix motion stack:

- robot da co the xuong dung `touch_pose`
- `Approach delta` va `Touch delta` co the ve gan `0 mm`

=> Ket luan: motion runtime truoc do co van de that, nhung sau fix thi phan motion da on.

---

## 4. Kiem tra logic ma tran

### 4.1 Logic runtime hien tai

Code runtime bien doi diem theo chuoi:

```text
point_base = T_base_tcp @ T_cam_to_tcp @ point_cam
```

Trong [vision/calibration.py]:

- `point_cam_3d` duoc dua ve homogeneous
- nhan `T_cam_to_tcp`
- sau do nhan `T_base_tcp` duoc suy ra tu `ActualTCPPose`

### 4.2 Ket luan

Khong tim thay loi:

- dao chieu `cam->tcp` thanh `tcp->cam`
- quen invert ma tran
- nhan sai thu tu
- sai logic axis-angle -> rotation matrix

Ly do ket luan duoc dieu nay:

- neu sai logic ma tran, `approach` da se lech rat lon
- nhung trong test tot nhat, `Approach pos_err` va `Touch pos_err` deu gan `0 mm`

Nghia la:

- runtime dang di den dung pose no tinh
- neu pose sai, loi nam o `target estimation`, `calibration quality`, hoac `offset`
- khong nam o phep nhan ma tran

---

## 5. Hand-eye calibration va van de dong bo voi TCP

### 5.1 Diem quan trong

Tool calibration trong repo dung `ActualTCPPose()` cua UR:

- nghia la `T_cam_to_tcp` duoc solve theo `TCP hien hanh`
- neu doi TCP thi hand-eye cu khong con la cung tham chieu

Day la mot diem rat quan trong da duoc xac nhan trong buoi nay:

- doi TCP xong ma giu nguyen `hand_eye_result.json` cu thi ve ban chat da doi bai toan hinh hoc

### 5.2 Kiem tra hand-eye cu

Da co luc hand-eye cu cho ket qua geometry co ve hop ly co khi, nhung sau khi doi TCP thi khong nen tin no cho production nua.

### 5.3 Thu calib lai

Da thu thu thap `20 poses` va tool tra ra:

```text
Method PARK:
Translation = [-0.1137, 0.0455, -0.2387] m
```

Da cap nhat vao `hand_eye_result.json`.

### 5.4 Danh gia hinh hoc

Sau khi cap nhat, `tools/debug_hand_eye_geometry.py` cho:

- `T_cam_to_tcp translation = [-113.7, 45.5, -238.7] mm`
- `TCP origin in camera frame z ~ 236.8 mm`
- `Camera optical axis +Z in base` huong xuong rat ro

Day la dau hieu hinh hoc hop ly.

### 5.5 Kiem tra thuc nghiem

Sau khi dung hand-eye PARK moi va motion stack moi, test `scanpose_touch` cho ket qua:

- `Approach pos_err = 0.0 mm`
- `Touch pos_err = 0.0 mm`
- `Retreat pos_err = 0.0 mm`

=> Trong vung lam viec scan hien tai, hand-eye moi dang hoat dong rat tot.

---

## 6. Van de "robot len qua cao roi moi ha xuong"

### 6.1 Hien tuong

Co giai doan robot di len cao roi moi di ngang den phoi, tao cam giac motion khong tu nhien va kho so sanh voi vi tri gap.

### 6.2 Nguyen nhan

Truoc do, logic build pose thuong la:

```text
approach_z = point_z + PICK_APPROACH_OFFSET_Z
```

Voi:

```text
PICK_APPROACH_OFFSET_Z = 0.15
```

neu `point_z + 0.15` cao hon `SCAN_POSE`, robot se nang len roi moi di ngang.

### 6.3 Cach sua dung

Da dua logic clamp vao helper dung chung:

```text
clamp_pick_z_sequence()
```

Nguyen tac:

- khong cho `approach_z` vuot qua `scan_z - 5 mm`
- `approach`, `touch`, `retreat` duoc sap xep Z an toan

Them vao do:

- `build_lateral_pre_approach_pose()`
  - di ngang o cao do hien tai cua `SCAN_POSE`
  - khong tu y nhac cao hon roi moi di ngang

### 6.4 Ket qua sau fix

Trong test tot nhat:

- `scan_z ~ 0.21049`
- `approach_z ~ 0.20549`
- `touch_z ~ 0.20175`

Nghia la robot chi ha ngan tu scan xuong target, khong con "ngoc len" cao bat thuong nua.

---

## 7. Van de lech diem gap

### 7.1 Hien tuong

Du motion di dung pose, ban van thay:

- vi tri gap lech so voi phoi
- nhieu lan lech cung mot phia

### 7.2 Nhung nguyen nhan da phat hien

#### a. Dung `bbox_center`

Trong mot giai doan, `Detector` bi thieu `refine_pick_point()` va `resolve_pick_depth()`, khien nhieu tool fallback ve:

```text
pick_source = bbox_center
```

Day la nguyen nhan rat manh gay lech vi tri gap, vi:

- YOLO bbox khong nhat thiet nam dung tam mat tren cua phoi
- bbox co the an ca nen xung quanh
- center bbox khong phai tam co khi can gap

#### b. Intrinsics khong dong nhat giua cac tool

Co tool scale `fx/fy/cx/cy` truc tiep theo frame size, co tool co logic bao ve metadata stale.

Da thong nhat bang helper chung:

```text
resolve_intrinsics_for_frame()
```

#### c. Approach/test flow va production flow tinh target khac nhau

Da co luc:

- `tool test` cho ket qua tot
- nhung `core/pick_place.py` van dung logic cu

Da sua de flow production cung dung:

- `refine_pick_point()`
- `resolve_pick_depth()`
- `resolve_intrinsics_for_frame()`
- `clamp_pick_z_sequence()`

### 7.3 Fix chuan da ap dung

Da khui phuc pipeline target refinement:

- contour trong bbox duoc dung de tim `contour_centroid`
- neu tim thay contour hop ly:
  - `pick_point = contour_centroid`
  - `pick_source = contour_centroid`
- neu khong:
  - fallback ve `bbox_center`

Da them lay depth theo `pick_bbox` chat hon quanh diem gap.

---

## 8. Van de systematic bias va `PICK_OFFSET_X/Y/Z`

### 8.1 Khi nao nen dung offset

Neu sau nhieu lan test:

- robot luon lech cung mot huong
- do lech gan nhu co dinh
- calibration/motion da on

thi nen dung `PICK_OFFSET_X/Y/Z` de bu sai so co tinh he thong.

### 8.2 Don vi rat quan trong

`PICK_OFFSET_*` dung don vi `met`.

Vi du:

- `0.005` = `5 mm`
- `-0.003` = `-3 mm`
- `-0.5` = `-500 mm`

### 8.3 Su co da gap

Da tung nhap:

```python
PICK_OFFSET_Y = -0.5
```

thay vi:

```python
PICK_OFFSET_Y = -0.005
```

He qua:

- target bi day lech hon `62 cm`
- log cho thay:
  - `delta_vs_tcp_m` lech Y rat lon
  - `planar=0.629m`
- he thong chan an toan khong cho robot chay tiep

Day la mot bai hoc quan trong:

- khi bu offset theo mm phai doi sang `met`
- nhap sai 2 chu so thap phan co the day target lech hang tram mm

### 8.4 Da cap nhat code de de debug hon

Da them vao config:

```python
PICK_OFFSET_X
PICK_OFFSET_Y
PICK_OFFSET_Z
```

Da them log ro hon trong tool va flow chinh:

- `p_base_raw`
- `pick_offset_base`
- `p_base` sau khi bu offset

Muc dich:

- de phan biet sai so den tu transform goc hay do offset bo sung
- de tinh trung binh sai so va bu offset co he thong

---

## 9. Loi config `.env`

Trong buoi nay da xac nhan:

- workspace hien tai khong co file `.env`
- chi co `.env.example`

Vi vay:

- neu sua gia tri bang cach sua `.env.example` thi runtime khong doc
- neu `config.py` dang dung `os.getenv(..., default)` thi no se fallback ve default trong code

Day giai thich vi sao co luc user noi "da doi offset nhung toa do gap van khong doi":

- runtime van dang dung default
- hoac dang doc gia tri khac voi gia tri user nghi da sua

---

## 10. Cac loi tool da gap trong buoi nay

### 10.1 `Detector` thieu API cu

`tools/inspect_target_transform.py` tung chet vi:

```text
AttributeError: 'Detector' object has no attribute 'refine_pick_point'
```

Nguyen nhan:

- `vision/detector.py` khong con day du method ma nhieu tool debug dang goi

Da sua:

- bo sung lai `refine_pick_point()`
- bo sung lai `resolve_pick_depth()`

### 10.2 `config` thieu bien `TRAY_REF_*`

`inspect_target_transform.py` tung chet vi:

```text
AttributeError: module 'config' has no attribute 'TRAY_REF_ENABLED'
```

Da sua:

- cho tool fallback ve gia tri mac dinh an toan neu config thieu

### 10.3 Sai chu ky helper `resolve_intrinsics_for_frame`

Sau khi doi helper thanh dung chung, `test_scanpose_touch.py` tung goi theo API cu va bi loi:

```text
missing 6 required positional arguments
```

Da sua lai call site cho dung chu ky moi.

---

## 11. Cac fix da ap dung vao code

### 11.1 `vision/calibration.py`

Da them:

- `resolve_intrinsics_for_frame()`
- `clamp_pick_z_sequence()`
- `build_lateral_pre_approach_pose()`

### 11.2 `vision/detector.py`

Da them / khui phuc:

- contour-based `pick_point` refinement
- `refine_pick_point()`
- `resolve_pick_depth()`

### 11.3 `tools/test_scanpose_touch.py`

Da sua:

- dung helper intrinsics chung
- dung clamp Z sequence
- in `p_base_raw`, `pick_offset_base`, `p_base`

### 11.4 `tools/inspect_target_transform.py`

Da sua:

- fallback config an toan
- dung helper intrinsics chung

### 11.5 `tools/view_scanpose_target.py`

Da sua:

- dung helper intrinsics chung
- overlay va log theo `pick_point`

### 11.6 `tools/move_to_vision_target.py`

Da sua:

- dung helper intrinsics chung
- pre-approach / approach / touch dong bo voi pipeline moi

### 11.7 `core/pick_place.py`

Da sua:

- dung `refine_pick_point()`
- dung `resolve_pick_depth()`
- dung `resolve_intrinsics_for_frame()`
- dung `clamp_pick_z_sequence()`
- them log `PickOffset: raw_base / offset / final_base`

---

## 12. Danh gia trang thai hien tai

### 12.1 Da on

- TCP moi da cap nhat dung vao runtime
- motion stack da on hon
- hand-eye PARK moi cho ket qua test touch rat tot trong vung scan
- logic ma tran runtime khong bi dao chieu
- robot da co the di den `approach` va `touch` voi sai so cuc nho

### 12.2 Con can theo doi

- systematic bias XY/Z qua nhieu lan test
- do on dinh cua `contour_centroid` tren nhieu phoi / nhieu vi tri khay
- su can thiet cua `PICK_OFFSET_X/Y/Z`
- quality thuc te cua hand-eye tren nhieu diem khac nhau, khong chi 1 vung scan

---

## 13. Quy trinh tiep tuc duoc khuyen nghi

### Buoc 1. Test khong offset

Dat:

```text
PICK_OFFSET_X = 0.0
PICK_OFFSET_Y = 0.0
PICK_OFFSET_Z = 0.0
```

Chay:

```bash
python3 tools/view_scanpose_target.py
python3 tools/inspect_target_transform.py --yes
python3 tools/test_scanpose_touch.py --touch-mode movel-bundled
```

### Buoc 2. Thu 5-10 lan

Moi lan ghi lai:

- vi tri phoi thuc te
- `p_base_raw`
- lech thuc te cua robot so voi diem can gap
- `pick_source`

### Buoc 3. Neu lech co tinh co he thong

Tinh trung binh:

- `mean_error_x`
- `mean_error_y`
- `mean_error_z`

Dat:

```text
PICK_OFFSET_X = -mean_error_x
PICK_OFFSET_Y = -mean_error_y
PICK_OFFSET_Z = -mean_error_z
```

Nho:

- don vi la `met`
- `5 mm = 0.005`

### Buoc 4. Test lai

Muc tieu:

- `approach/touch` van dung pose
- vi tri gap bam hon vao thuc te
- sai so con lai nho va khong con lech cung mot huong ro ret

---

## 14. Nhung dieu KHONG nen lam nua

- Khong nhap offset theo mm ma quen doi sang `met`
- Khong sua tay ma tran hand-eye theo cam tinh
- Khong tiep tuc nghi ngo "nhan ma tran sai chieu" khi motion da bam pose gan nhu tuyet doi
- Khong ket luan offset co tac dung neu runtime van dang doc `0.0`
- Khong dua checkerboard/two-shot thanh giai phap chinh neu setup tray-layout on dinh hon

---

## 15. Ket luan cuoi cung

Buoi debug nay da xac nhan duoc 3 dieu cot loi:

1. Loi motion stack ban dau da duoc sua, robot hien tai co the di dung `approach` va `touch`.
2. Logic transform va hand-eye runtime khong bi sai chieu; van de con lai neu co chu yeu la chat luong target va systematic bias.
3. Offset la huong bu dung khi sai so co tinh co he thong, nhung phai quan ly dung don vi va log ro `raw -> offset -> final`.

Trang thai hien tai cua he thong da tot hon ro ret so voi dau buoi:

- motion dung hon
- target estimation ro hon
- config va log ro hon
- du du lieu de bu offset bai ban neu can

Huong dung tu nay tro di la:

- giu TCP co dinh
- giu hand-eye PARK moi
- test nhieu mau
- chi bu offset nho theo `met`
- va dung cac tool debug da duoc dong bo de so sanh `p_base_raw`, `p_base`, va lech thuc te.
