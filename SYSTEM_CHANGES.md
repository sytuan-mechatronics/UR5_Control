# System Changes

Tai lieu nay tom tat cac thay doi gan day cua he thong `UR5 Control Server`, cac tool moi da duoc them, cac loi da gap trong qua trinh debug, va huong sua hien tai.

## 1. Muc tieu cap nhat

- Dong bo tai lieu voi code dang chay thuc te.
- Giam loi motion khi robot di tu `SCAN_POSE` den vi tri phoi.
- Cai thien do chinh xac vision khi phoi nghieng, khay dich chuyen, hoac depth co hole.
- Them bo tool debug de tach rieng tung lop loi: robot motion, transform, camera, target refinement, tray reference.

## 2. Thay doi chinh trong he thong

### 2.1 Vision pipeline

- Model mac dinh doi sang `ur5.pt`.
- Runtime camera chuan hoa ve `import ob`; repo co `ob.py` shim + bundled Orbbec runtime.
- Target khong con dung `bbox center` thuong, ma uu tien:
  - `contour_centroid` cua phoi
  - depth lay theo vung nho quanh `pick_point`
  - neu depth hole thi no se no rong ROI tung muc va fallback ve bbox depth
- Luong pick debug hien tai:
  - `SCAN_POSE -> pre_approach -> approach -> touch -> return SCAN_POSE`
- `pre_approach` duoc giu cung cao do voi `SCAN_POSE`; robot khong duoc tu y nhac cao hon roi moi di ngang.
- `approach/touch` co clamp logic de tranh tinh sai theo huong nguoc len tren.

### 2.2 Motion va safety

- Script test va flow chinh them nhieu guard:
  - dung ngay neu `wait_steady()` timeout
  - dung ngay neu robot khong den duoc pose dich
  - dung ngay neu target xa bat thuong so voi TCP luc capture
- Da them `payload` va `center of gravity` vao cau hinh.
- Da them gioi han an toan:
  - `MAX_TARGET_PLANAR_DIST_M`
  - `MAX_TARGET_DZ_DIST_M`

### 2.3 Config va env

- `config.py` da duoc sua de nap `.env` tai root repo mot cach on dinh.
- Da them cac bien:
  - `PICK_OFFSET_X/Y/Z`
  - `TOOL_DOWN_RX/RY/RZ`
  - `PAYLOAD_MASS_KG`, `PAYLOAD_COG`
  - `TRAY_REF_*`
  - `TRAY_HOLE_REF_*`
  - `TRAY_LAYOUT_PATH`
  - `TRAY_LAYOUT_MAX_REPROJ_ERR_PX`
  - `TRAY_LAYOUT_MAX_ASSIGN_DIST_PX`
  - `TRAY_LAYOUT_MAX_CANDIDATE_HOLES`

## 3. Tool moi

### 3.1 Motion va robot debug

- `tools/test_movel_offset.py`
  - test offset nho de tach bai toan `movel/controller/TCP/reachability`.
- `tools/inspect_target_transform.py`
  - chi tinh `p_cam`, `camera_origin_base`, `p_base`, khong cho robot chay.
- `tools/test_scanpose_touch.py`
  - tool debug chinh:
    - xac nhan `SCAN_POSE`
    - chup frame
    - detect phoi
    - tinh target
    - di `pre_approach -> approach -> touch`
    - quay lai `SCAN_POSE`

### 3.2 Vision debug

- `tools/view_scanpose_target.py`
  - mo cua so anh, ve bbox, `pick_point`, ROI depth, overlay `p_cam/p_base`, va nguon target refinement.
- `tools/view_tray_pose.py`
  - debug tray-layout/hole matching, khong cho robot chay.
- `tools/annotate_tray_layout.py`
  - tao `tray_layout.json` bang cach click cac diem mau tren anh khay.

### 3.3 Vision modules moi

- `vision/tray_layout.py`
  - doc/ghi `tray_layout.json`.
- `vision/tray_holes.py`
  - detect lo khay, match layout 5 lo, gan phoi vao lo gan nhat.
- `vision/tray_reference.py`
  - nhanh checkerboard/two-shot de debug.
- `vision/tray_pose.py`
  - nhanh detect bien ngoai khay; da thu nghiem nhung hien tai khong uu tien.

## 4. Loi da gap trong qua trinh debug

### 4.1 Motion va controller

- Robot khong di dung target, hoac dung im du da gui `movel`.
- `get_inverse_kin` khong tim thay nghiem.
- `path sanity check failed`.
- `position deviates from path`.
- `protective stop`.

Nguyen nhan thuc te da gap:

- pose vision tinh ra qua xa hoac theo huong controller khong chap nhan
- robot o gan vung singularity
- payload/CoG/TCP/chinh sach motion chua phu hop
- luong di chuyen chua duoc tach ro `di ngang truoc, ha xuong sau`

### 4.2 Vision va transform

- Dung `bbox center` lam diem pick dan den truot phoi khi phoi nghieng.
- Depth hole lam `depth=0`.
- `approach_z` tung bi tinh cao hon `SCAN_POSE`, gay hien tuong robot di nguoc len tren.
- Checkerboard `two-shot` co the keo target sai hang chuc mm neu tam bang dat khong that su dong phang voi khay.

### 4.3 Config va van hanh

- `.env` truoc day tung khong an vao runtime.
- Gia tri joint placeholder trong `.env` tung lam robot roi khoi pose that.
- Nhiem vu debug tray pose theo bien ngoai khay lech nhieu so voi layout lo.

## 5. Huong sua da ap dung

### 5.1 Da sua

- Dung `contour_centroid` thay cho `bbox center`.
- Them fallback depth theo nhieu kich thuoc ROI.
- Chuyen flow debug ve `SCAN_POSE -> pre_approach -> approach -> touch`.
- Them payload, safety guard, va log chi tiet.
- Them `PICK_OFFSET_X/Y/Z` de fine-tune nho trong base-frame.
- Them logic khong cho robot tiep tuc motion khi timeout/protective stop.
- Them nhanh tham chieu theo khay:
  - `checkerboard/two-shot` de debug
  - `tray_hole_layout` cho khay co 5 lo co dinh

### 5.2 Khong uu tien nua

- Dung offset lon de bu loi toan cuc.
- Dung checkerboard `two-shot` cho setup hien tai neu bang marker dat roi / khong on dinh.
- Dung contour ngoai cua khay lam tham chieu chinh neu no khong bam dung geometry that.

## 6. Huong sua uu tien hien tai

Thu tu uu tien dang khuyen nghi:

1. Xac dinh `SCAN_POSE` va motion on dinh bang `tools/test_scanpose_touch.py`.
2. Dung `tools/view_scanpose_target.py` de kiem tra:
   - `pick_point`
   - `source=...`
   - `depth`
   - `p_base`
3. Neu khay co 5 lo co dinh:
   - tao `tray_layout.json`
   - match layout 5 lo trong runtime
   - gan phoi vao lo gan nhat
4. Chi khi sai so con nho va co tinh co dinh moi dung `PICK_OFFSET_X/Y/Z` de fine-tune vai mm.

## 7. Khuyen nghi van hanh

- Tool debug uu tien:
  - `python3 tools/view_scanpose_target.py`
  - `python3 tools/test_scanpose_touch.py --yes`
- Neu debug tray holes/layout:
  - `python3 tools/annotate_tray_layout.py --capture`
  - `python3 tools/view_tray_pose.py`
- Neu can tach rieng transform:
  - `python3 tools/inspect_target_transform.py --yes`

## 8. Ket luan

He thong da duoc cai thien ro o 3 lop:

- Motion an toan hon va dung test dung luc khi robot khong on dinh.
- Vision dung `pick_point` tot hon va depth robust hon.
- Them bo tool debug va nhanh tham chieu theo khay/lo de giam phu thuoc vao offset toan cuc.

Huong uu tien de tiep tuc la: `detect phoi + match layout 5 lo cua khay`, thay vi tiep tuc dua vao checkerboard hoac offset lon.
