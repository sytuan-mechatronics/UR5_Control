# Vấn đề hôm nay 18/06

## Mục tiêu

Làm rõ vì sao UR5 trong flow `scanpose -> vision -> touch/pick` có hiện tượng:
- không đi xuống `touch pose` như mong đợi
- có lúc nhìn như đi lên
- dù điểm vision trên ảnh (`p_cam`, `p_base`) đã được xác nhận là đúng

File này tổng hợp để người khác như Claude vào có thể hiểu ngay:
- hiện tượng thực tế
- những gì đã kiểm tra
- những gì đã loại trừ
- nguyên nhân đang nghi nhất
- các file đã thay đổi hôm nay

---

## Hiện tượng thực tế

### 1. Vision đã nhận diện đúng

Trên ảnh preview:
- bbox đúng phôi
- tâm chọn đúng vùng cần gắp
- `p_cam` và `p_base` hiển thị trên ảnh được xác nhận là đúng theo thực tế

Ví dụ log:

```text
Target: label=phoi, conf=0.930, center=(1019.5,350.0), depth=276.0 mm
```

### 2. Touch pose có Z thấp hơn approach pose về mặt số

Ví dụ:

```text
Approach pose: [0.628267..., -0.291111..., 0.382555..., 2.152263, 1.979381, 0.237215]
Touch pose:    [0.628267..., -0.291111..., 0.237555..., 2.152263, 1.979381, 0.237215]
```

Tức là:
- `touch_z < approach_z`
- logic tạo pose theo `base Z` là đúng

### 3. Nhưng robot thực tế không đi xuống touch pose

Log sau khi thêm `actual TCP` cho từng bước:

```text
Approach actual TCP: [0.6218, -0.2782, 0.3638, 2.1522, 1.9793, 0.2374]
Approach delta:      [-0.0067, 0.0143, -0.0193, -0.0000, -0.0001, 0.0002] (pos_err=25.0 mm)

Touch actual TCP: [0.6217, -0.2782, 0.3638, 2.1521, 1.9795, 0.2373]
Touch delta:      [-0.0067, 0.0144, 0.1257, -0.0001, 0.0002, 0.0001] (pos_err=126.7 mm)

Retreat actual TCP: [0.6217, -0.2782, 0.3637, 2.1523, 1.9795, 0.2372]
Retreat delta:      [-0.0068, 0.0143, -0.0193, -0.0000, 0.0002, 0.0000] (pos_err=25.0 mm)
```

Ý nghĩa:
- robot tới gần `approach`
- gần như không xuống thêm ở bước `touch`
- rồi `retreat` gần như trùng lại với `approach`

---

## Những gì đã kiểm tra

### 1. Đã cập nhật TCP offset mới

TCP offset operator cung cấp:

```text
X=-9.04 mm
Y=9.05 mm
Z=325.1 mm
Rx=0.0185 rad
Ry=-0.0294 rad
Rz=3.1303 rad
Payload=1 kg
```

Đã đưa vào runtime config:
- `TCP_OFFSET = [-0.00904, 0.00905, 0.3251, 0.0185, -0.0294, 3.1303]`
- `PAYLOAD_MASS_KG = 1.0`

### 2. Đã cập nhật hand-eye

Có thử vài phiên bản `hand_eye_result.json`.

Bản đang giữ lại là bản cũ hợp lý hơn về cơ khí:

```text
T_cam_to_tcp translation = [-52.3, 48.5, -271.7] mm
```

Lý do giữ bản này:
- rotation hợp lệ
- XY không vô lý như bản mới bị lệch ngang rất lớn
- khớp cơ khí với mô tả camera/tool tốt hơn

### 3. Đã test chiều TCP Z riêng

Dùng tool:
- `tools/test_tcp_z_direction.py`

Kết quả tại `SCAN_POSE`:
- lệnh `Z -10 mm` không cho thấy dấu ngược
- lệnh `Z +10 mm` làm `TCP Z` tăng đúng

Kết luận:
- `TCP Z` không bị đảo dấu
- `base Z` mà code dùng là đúng chiều

### 4. Đã test touch bằng 2 mode

Đã test:
- `movel`
- `movej(get_inverse_kin(...))`

Kết quả:
- cả hai mode đều không đưa robot xuống `touch pose`
- nên lỗi không phải riêng do `movel`

### 5. Trên teach pendant thao tác xuống rất mượt

Đây là manh mối rất quan trọng:
- pendant jog xuống mượt
- nhưng chạy từ code không xuống được

Kết luận:
- không phải do robot kẹt cơ khí
- không phải do tư thế đó bản thân robot không xuống được
- lỗi nằm ở sự khác nhau giữa `runtime settings` do code áp vào và trạng thái/tool đang dùng trên pendant

---

## Những giả thuyết đã loại trừ hoặc giảm mức nghi ngờ

### 1. Không còn nghi vision là thủ phạm chính

Vì:
- operator xác nhận `p_cam` và `p_base` trên ảnh là đúng
- nhận diện đúng phôi

### 2. Không còn nghi trục Z bị đảo dấu toàn bộ

Vì test `TCP Z direction` cho thấy chiều Z của TCP là đúng.

### 3. Không còn nghi riêng `movel`

Vì đổi sang `movej-ik` mà vẫn không xuống được.

### 4. Không còn nghi “do pose touch tính sai dấu Z”

Vì:
- `touch_pose.z` luôn nhỏ hơn `approach_pose.z`
- tức về số học, lệnh vẫn là đi xuống

---

## Nguyên nhân đang nghi nhất

Tại thời điểm viết file này, nguyên nhân nghi nhất là:

### 1. Runtime setting từ code khác với tool state đang vận hành mượt trên pendant

Cụ thể nghi:
- `set_tcp(...)` từ code
- `set_payload(...)` từ code
- đặc biệt là `PAYLOAD_COG` hiện vẫn chỉ là giá trị ước lượng:

```text
PAYLOAD_COG = [0.0, 0.0, 0.16]
```

Đây chưa phải CoG thật do operator dạy/đo trên robot.

Nếu pendant đang dùng:
- TCP đúng
- payload/CoG đúng

nhưng code lại set lại bằng giá trị khác, thì motion từ code có thể cư xử khác hẳn pendant.

### 2. Bộ pose vận hành và TCP active có thể vẫn chưa hoàn toàn đồng bộ theo cùng trạng thái tool

Operator có nói đã dạy lại pose sau TCP mới.
Tuy nhiên vẫn cần cảnh giác:
- nếu `read_robot_pose.py` hoặc file `robot_poses.json` chưa được chụp/lưu lại đầy đủ đúng tại trạng thái active tool hiện tại
- thì code có thể vẫn đang dùng dữ liệu pose không khớp hoàn toàn với runtime TCP

### 3. CoG/payload rất có thể là thủ phạm lớn

Lý do:
- pendant chạy mượt
- code chạy không mượt
- code hiện luôn set lại payload/CoG
- CoG chưa phải số thật

Đây là nghi phạm mạnh nhất còn lại.

---

## Hướng xử lý tiếp theo

### Ưu tiên 1: kiểm tra ảnh hưởng của payload/CoG

Nên thêm hoặc test các option:
- `--skip-set-payload`
- hoặc `--payload-kg 0`

Mục tiêu:
- nếu bỏ `set_payload` mà robot xuống được như pendant, chốt ngay lỗi nằm ở `payload/CoG`

### Ưu tiên 2: kiểm tra ảnh hưởng của set_tcp từ code

Nên thêm option:
- `--skip-set-tcp`

Mục tiêu:
- nếu bỏ `set_tcp` mà robot chạy giống pendant, chốt lỗi nằm ở TCP runtime do code set

### Ưu tiên 3: chụp lại đầy đủ tất cả pose vận hành thật sau TCP mới

Theo `PARAMETERS_CHECKLIST.md`, các điểm trong `read_robot_pose.py` là các điểm thật của chu trình:
- `HOME`
- `SCAN_APPROACH_JOINTS`
- `SCAN_POSE_JOINTS`
- `PLACE_APPROACH_CART`
- `PLACE_POINT_CART`
- `PLACE_RETREAT_CART`

Cần chắc chắn:
- `robot_poses.json` có đủ tất cả key trên
- được capture lại sau khi dạy TCP mới
- đúng trạng thái tool active hiện dùng trong pendant

### Ưu tiên 4: giữ nguyên vision/hand-eye hiện tại, không lật dấu bừa

Vì:
- `p_cam/p_base` đã được xác nhận đúng
- không có bằng chứng cho việc cần lật dấu cả ma trận

Không nên:
- tự ý đảo dấu Z trong `T_cam_to_tcp`
- tự ý lật transform `cam->tcp` sang `tcp->cam`

---

## Các file đã thay đổi hôm nay

### 1. Cấu hình tool/TCP/payload

- [config.py](/home/tuan/Downloads/Ur5_Control-main/config.py)
  - thêm `TCP_OFFSET`
  - thêm cờ `DEPTH_TCP_STANDOFF_CLAMP_ENABLED`
  - thêm `PICK_MIN_FINAL_BELOW_CAMERA_M`
  - giữ `PAYLOAD_MASS_KG`
  - giữ `PAYLOAD_COG` tạm thời ở giá trị ước lượng

### 2. Runtime áp TCP/payload

- [core/pick_place.py](/home/tuan/Downloads/Ur5_Control-main/core/pick_place.py)
  - runtime set `TCP` và `payload`
  - đổi guard an toàn từ so với `TCP z` sang so với `camera origin`

### 3. Logic calibration / transform

- [vision/calibration.py](/home/tuan/Downloads/Ur5_Control-main/vision/calibration.py)
  - thêm `camera_origin_to_base(...)`
  - đổi `sanitize_camera_depth_mm(...)` để mặc định không auto-clamp theo TCP standoff nữa

### 4. Tool test scanpose touch

- [tools/test_scanpose_touch.py](/home/tuan/Downloads/Ur5_Control-main/tools/test_scanpose_touch.py)
  - log `Approach actual TCP`
  - log `Touch actual TCP`
  - log `Retreat actual TCP`
  - thêm `pos_err`
  - thêm `--touch-mode movel|movej-ik`
  - đổi guard an toàn theo camera thay vì TCP

### 5. Tool test chiều TCP Z

- [tools/test_tcp_z_direction.py](/home/tuan/Downloads/Ur5_Control-main/tools/test_tcp_z_direction.py)
  - tool mới để xác minh chiều `TCP Z` có bị ngược không

### 6. Client URScript

- [robot/urscript_client.py](/home/tuan/Downloads/Ur5_Control-main/robot/urscript_client.py)
  - thêm `move_joint_to_pose_ik(...)`

### 7. Tool chẩn đoán tọa độ vision

- [vision_coordinate_diagnosis/diagnose_pick_coordinate_error.py](/home/tuan/Downloads/Ur5_Control-main/vision_coordinate_diagnosis/diagnose_pick_coordinate_error.py)
- [vision_coordinate_diagnosis/HUONG_DAN.md](/home/tuan/Downloads/Ur5_Control-main/vision_coordinate_diagnosis/HUONG_DAN.md)

Nhóm file này dùng để:
- log `p_cam`, `p_base`, `delta_vs_tcp`
- so sánh với điểm thật do operator jog tay
- đánh giá nghi `depth`, `TCP`, `hand-eye`

---

## Kết luận ngắn cho Claude

Tại thời điểm này:
- vision point đúng
- hand-eye đang dùng là bản cũ hợp lý hơn
- TCP Z đúng chiều
- touch pose tính đúng chiều xuống
- nhưng robot từ code không xuống được, dù pendant xuống mượt

=> Vấn đề còn lại nghi mạnh nhất là:
- `runtime TCP/payload/CoG do code set không khớp với trạng thái tool đang dùng mượt trên pendant`

Đặc biệt:
- `PAYLOAD_COG` chưa phải số thật
- cần test `skip-set-payload` và có thể cả `skip-set-tcp`

Không nên tiếp tục sửa:
- vision
- đảo dấu hand-eye
- đổi chiều trục Z

trước khi chốt ảnh hưởng của:
- `payload`
- `CoG`
- `set_tcp`

