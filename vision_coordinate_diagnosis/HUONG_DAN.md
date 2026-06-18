# Hướng dẫn chẩn đoán lỗi tọa độ vision pick

File tool chính:
- [diagnose_pick_coordinate_error.py](/home/tuan/Downloads/Ur5_Control-main/vision_coordinate_diagnosis/diagnose_pick_coordinate_error.py)

Mục tiêu:
- Không cho robot tự lao vào phôi.
- Chỉ chụp ảnh, tính tọa độ backend, cho người vận hành jog tay đến đúng điểm thực.
- So sánh `tọa độ backend tính` với `tọa độ thực tế`.
- Ghi log ra file và đưa nhận xét chủ quan.

## Khi nào nên dùng

Nên dùng khi có một trong các dấu hiệu sau:
- Robot nhận diện xong nhưng đi lên cao bất thường.
- Robot lao lệch theo XY.
- Depth nhìn có vẻ đúng nhưng điểm pick backend tính ra vẫn xa thực tế.
- Nghi ngờ TCP mới dạy sai.
- Nghi ngờ ma trận hand-eye sai dấu, sai trục, sai đơn vị.

## Cách chạy an toàn nhất

1. Đưa robot về `SCAN_POSE` bằng cách bạn vẫn làm hằng ngày.
2. Đặt 1 phôi cố định trong vùng nhìn của camera.
3. Đảm bảo vùng làm việc trống, để nếu bạn cần jog tay thì không vướng.
4. Chạy tool:

```bash
python3 vision_coordinate_diagnosis/diagnose_pick_coordinate_error.py --robot-ip 192.168.125.11 --teach-expected
```

5. Tool sẽ:
- kiểm tra robot có đang gần `SCAN_POSE` không
- chụp RGB/depth
- nhận diện phôi
- tính `p_cam`, `p_base`, `delta_vs_tcp`
- dừng lại để bạn jog tay TCP đến đúng điểm mà vision vừa chọn
- đọc tọa độ thực tế từ robot
- ghi log `.json` và `.md`
- in nhận xét chủ quan

## Log được lưu ở đâu

Mặc định log nằm trong:

```bash
vision_coordinate_diagnosis/logs/
```

Mỗi lần chạy sẽ có:
- `YYYYMMDD_HHMMSS.json`
- `YYYYMMDD_HHMMSS.md`

## Cách đọc kết quả

### 1. `raw_depth_mm`

- Nếu `raw_depth_mm` thấp bất thường hoặc hay bằng `0`, nghi depth hole, mặt phản xạ, hoặc vùng lấy depth chưa đúng.
- Nếu `safe_depth_mm` bị clamp cao hơn nhiều so với `raw_depth_mm`, nghi `TCP/hand-eye` hoặc depth đang chạm vùng sai.

### 2. `delta_vs_tcp_m`

- Nếu `delta_vs_tcp_m` lệch XY rất lớn, ví dụ vài chục cm, trong khi robot đang gần ngay phía trên phôi, nghi mạnh `TCP` hoặc `T_cam_to_tcp`.
- Nếu `delta_vs_tcp_m[2]` dương nhiều, backend đang nghĩ điểm pick ở phía trên TCP. Đây là dấu hiệu rất đáng nghi.

### 3. `error_mm`

Đây là sai số thật giữa:
- tọa độ backend tính ra
- tọa độ bạn jog tay chạm đúng vào phôi

Đọc như sau:
- Sai chủ yếu ở `Z`: nghi depth trước.
- Sai chủ yếu ở `X/Y`: nghi `TCP`, `hand-eye`, hoặc hướng trục camera.
- Sai cả `X/Y/Z`: thường là tổ hợp `TCP + hand-eye`, không chỉ riêng depth.

## Nhận xét thực tế cho case hôm nay

Dựa trên log và ma trận hiện tại trong repo:
- Depth có lúc lỗi hole, nhưng không còn là nghi phạm chính trong mọi lần test.
- Có những lần depth đã khá sạch nhưng `p_base` vẫn lệch XY lớn.
- Với ma trận hiện tại, camera đang bị suy ra lệch ngang khá nhiều so với TCP.
- Vì vậy nghi ngờ lớn nhất hiện tại vẫn là:
  1. `TCP mới dạy chưa đúng tâm/tool thật`
  2. `hand-eye dùng với TCP mới nhưng pose lấy calib chưa thật sạch`
  3. `camera gắn thực tế không giống đúng cấu hình lúc calib`

## Khuyến nghị

- Chạy tool này ít nhất 3 lần với cùng 1 phôi, cùng 1 vị trí.
- Nếu sai số lặp lại gần như giống nhau:
  nghi `TCP/hand-eye`.
- Nếu sai số thay đổi mạnh mỗi lần:
  nghi `depth/ánh sáng/bề mặt`.
- Nếu muốn an toàn hơn nữa:
  chỉ dùng tool này để đo, không bật flow auto-pick cho đến khi sai số xuống mức chấp nhận được.
