# Báo Cáo Kết Quả Thực Nghiệm — Hệ Thống UR5 Pick-and-Place

**Ngày thực nghiệm:** 26/06/2026  
**Hệ thống:** Robot UR5 CB3 + Camera Orbbec Femto Mega + YOLO11 + Pneumatic Gripper  
**Thông số chung:**
- Tốc độ khớp: 0.8 rad/s
- Tốc độ tuyến tính: 0.1 m/s
- Tốc độ tiếp cận pick: 0.05 m/s

---

## 1. Kịch Bản 1 — Chuyển Động Cơ Bản (Phase 1)

**Mục đích:** Kiểm tra kết nối, motion profile, và hoạt động gripper không cần camera.  
**Công cụ:** `python tools/test_phase1.py`

### Bảng 1.1 — Kết quả tổng hợp Phase 1

| Chỉ số | Giá trị |
|---|---|
| Số lần chạy | 8 |
| Trạng thái | 8/8 thành công (`done`) |
| Số lỗi / cảnh báo | 0 |
| Tỉ lệ thành công | **100%** |

> **Nhận xét:** Toàn bộ 8 lần chạy Phase 1 đều hoàn thành thành công, không ghi nhận bất kỳ lỗi hay cảnh báo nào. Kết quả này xác nhận hệ thống cơ học (robot, gripper, kết nối Dashboard/URScript/RTDE) hoạt động ổn định và đạt điều kiện tiên quyết để tiến sang Phase 2 có tích hợp vision.

---

### Bảng 1.2 — Thống kê thời gian chu trình (cycle time)

| | Tất cả 8 lần (s) | 6 lần ổn định (s) |
|---|---|---|
| Trung bình (mean) | 41.45 | **36.13** |
| Độ lệch chuẩn (std) | 9.86 | 0.62 |
| Nhỏ nhất (min) | 35.48 | 35.48 |
| Lớn nhất (max) | 57.46 | 37.02 |

> **Nhận xét:** Hai lần chạy đầu (57.3s và 57.5s) có thời gian dài hơn đáng kể so với 6 lần còn lại do bao gồm bước chuẩn bị hệ thống lần đầu (brake release settle, khởi động kết nối). Khi loại bỏ 2 lần warm-up, 6 lần còn lại cho thấy chu trình rất ổn định: **36.13 ± 0.62s**, với độ lệch chuẩn chỉ bằng **1.7%** của giá trị trung bình. Kết quả này đủ để dùng làm baseline thời gian chuyển động cho các phase sau.

---

### Bảng 1.3 — Thống kê gripper

| Chỉ số | Trung bình | Độ lệch chuẩn |
|---|---|---|
| Thời gian đóng gripper (ms) | 511.5 | 0.2 |
| Thời gian mở gripper (ms) | 314.5 | 0.1 |

> **Nhận xét:** Gripper pneumatic hoạt động cực kỳ nhất quán với độ lệch chuẩn dưới 0.2ms — tương đương sai số đo lường. Thời gian đóng (511.5ms) lớn hơn thời gian mở (314.5ms) do hành trình kẹp cần thắng lực lò xo hồi vị. Hai giá trị này ổn định xuyên suốt 8 lần chạy, cho thấy van solenoid và cơ cấu khí nén hoạt động đáng tin cậy.

---

## 2. Kịch Bản 2 — Phát Hiện + Pick-Place 1 Phôi (Phase 2)

**Mục đích:** Xác nhận pipeline đầy đủ: camera → YOLO → hand-eye transform → pick-place 1 phôi.  
**Công cụ:** `python tools/test_phase2.py`

### Bảng 2.1 — Kết quả tổng hợp Phase 2

| Chỉ số | Giá trị |
|---|---|
| Số lần chạy | 21 |
| Số slot kiểm tra | 5 (slot_1 → slot_5) |
| Kết quả pick | 21/21 `success` |
| Số lần retry | 0 |
| Tỉ lệ thành công | **100%** |

> **Nhận xét:** Pipeline tích hợp vision đạt tỉ lệ thành công 100% trên 21 lần chạy, phủ đều cả 5 vị trí slot trên khay. Không có lần nào cần retry, chứng tỏ toàn bộ chuỗi xử lý từ phát hiện phôi, chuyển đổi tọa độ pixel sang không gian robot (hand-eye calibration), đến gắp và đặt phôi đều hoạt động chính xác và nhất quán ở tất cả các vị trí kiểm tra.

---

### Bảng 2.2 — Thống kê thời gian chu trình Phase 2 (giây)

| Chỉ số | Giá trị (s) |
|---|---|
| Trung bình | 82.14 |
| Độ lệch chuẩn | 1.92 |
| Nhỏ nhất | 79.53 |
| Lớn nhất | 86.40 |

> **Nhận xét:** Thời gian chu trình Phase 2 (~82s) tăng khoảng **46 giây** so với Phase 1 (~36s). Phần tăng thêm này tương ứng với các bước: di chuyển đến scan pose, chụp frame camera, YOLO inference, tính toán tọa độ pick, và thực hiện place. Độ lệch chuẩn chỉ **1.92s (2.3% mean)** cho thấy thời gian xử lý vision ổn định, không bị ảnh hưởng bởi biến thiên vị trí phôi giữa các lần chạy.

---

### Bảng 2.3 — Thống kê độ tin cậy YOLO11 Phase 2

| Chỉ số | Giá trị |
|---|---|
| Trung bình | 0.9322 |
| Độ lệch chuẩn | 0.0057 |
| Nhỏ nhất | 0.9201 |
| Lớn nhất | 0.9433 |

> **Nhận xét:** Model YOLO11 cho confidence ổn định cao trong toàn bộ 21 lần chạy, với giá trị trung bình **0.9322** và tất cả kết quả đều vượt ngưỡng 0.92. Độ lệch chuẩn rất nhỏ (0.0057) phản ánh điều kiện ánh sáng và góc nhìn camera được duy trì nhất quán tại scan pose. Không có lần nào phát hiện dưới ngưỡng confidence tối thiểu, chứng tỏ model hoạt động đáng tin cậy trong điều kiện thực nghiệm.

---

### Bảng 2.4 — Kết quả theo từng slot (Phase 2)

| Slot | Số lần chạy | Confidence trung bình | Thời gian TB (s) | Kết quả |
|---|---|---|---|---|
| slot_1 | 4 | 0.9345 | 81.20 | 4/4 success |
| slot_2 | 4 | 0.9278 | 84.21 | 4/4 success |
| slot_3 | 5 | 0.9335 | 82.57 | 5/5 success |
| slot_4 | 4 | 0.9316 | 82.97 | 4/4 success |
| slot_5 | 4 | 0.9334 | 79.63 | 4/4 success |
| **Tổng** | **21** | **0.9322** | **82.14** | **21/21** |

> **Nhận xét:** Cả 5 slot đều đạt tỉ lệ thành công 100%, xác nhận hệ thống hoạt động đáng tin cậy tại mọi vị trí trong vùng làm việc. Về thời gian, **slot_5** nhanh nhất (79.63s) trong khi **slot_2** chậm nhất (84.21s) — chênh lệch ~4.6s có thể do khoảng cách quỹ đạo pick-to-place khác nhau giữa các vị trí. Confidence YOLO phân bố đều qua các slot (0.9278–0.9345), không có vị trí nào khó phát hiện hơn, chứng tỏ phôi được nhận diện tốt bất kể vị trí trên khay.

---

## 3. Kịch Bản 3 — Vòng Lặp Tự Động 5 Phôi (Phase 3)

**Mục đích:** Vòng lặp pick-place tự động cho đến khi khay rỗng (tối đa 5 phôi/lần chạy).  
**Công cụ:** `python tools/test_phase3.py`

### Bảng 3.1 — Kết quả tổng hợp Phase 3 (10 job)

| Chỉ số | Giá trị |
|---|---|
| Số job chạy | 10 |
| Phôi mục tiêu / job | 5 |
| Tổng số picks | 50 |
| Tỉ lệ thành công | **100% (10/10 job, 50/50 picks)** |
| Lần pick đầu thành công | 50/50 (100%) |
| Số lần retry | 0 |
| Chu kỳ thất bại toàn phần | 0 |

> **Nhận xét:** Phase 3 là kịch bản thực nghiệm phức tạp nhất với vòng lặp tự động nhiều phôi, cơ chế rescan, exclusion list vị trí đã gắp, và in-place retry. Hệ thống hoàn thành 10/10 job và 50/50 picks thành công ngay từ lần thử đầu tiên (first-attempt success 100%), không kích hoạt bất kỳ cơ chế retry nào. Kết quả này cho thấy toàn bộ pipeline hoạt động đủ chính xác để vận hành tự động liên tục mà không cần can thiệp thủ công.

---

### Bảng 3.2 — Thống kê thời gian job Phase 3 (giây, 5 phôi/job)

| Chỉ số | Giá trị (s) |
|---|---|
| Trung bình | 230.21 |
| Độ lệch chuẩn | 40.37 |
| Nhỏ nhất | 208.90 |
| Lớn nhất | 341.20 |

> **Nhận xét:** Giá trị độ lệch chuẩn lớn (40.37s) chủ yếu do **job đầu tiên (341.2s)** kéo lên — cao hơn ~63% so với các job sau. Nếu loại trừ job đầu, 9 job còn lại có thời gian trong khoảng **208.9 – 242.3s** với trung bình ~218s và ổn định hơn nhiều. Thời gian job đầu dài có thể do: khởi động stream camera LAN, flush buffer frame đầu tiên, và settle sau lần kết nối đầu. Từ job thứ 2 trở đi, hệ thống đạt thời gian ổn định, tương đương **~43–44 giây/phôi** (218s / 5 phôi).

---

### Bảng 3.3 — Thời gian xử lý từng phôi (4 job có log chi tiết)

| Job ID | Thời gian TB / phôi (s) | Tổng job (s) |
|---|---|---|
| 36a12b4c (lần 1) | 9.215 | 341.2 |
| e1b1ec74 (lần 2) | 6.953 | 242.3 |
| 565d0028 (lần 3) | 6.462 | 230.1 |
| eae796af (lần 4) | 6.111 | 215.8 |
| **Trung bình** | **7.185** | — |

> **Nhận xét:** Thời gian xử lý trung bình mỗi phôi giảm dần qua các job: từ **9.215s** (job 1) xuống **6.111s** (job 4) — giảm khoảng **33.7%**. Xu hướng này phản ánh hiện tượng "warm-up" của hệ thống, trong đó stream camera LAN ổn định hơn (ít frame stale hơn), pipeline xử lý không còn overhead khởi tạo. Từ job thứ 3–4, thời gian/phôi hội tụ quanh **6.1–6.5s**, đây có thể xem là năng lực xử lý thực của hệ thống ở trạng thái ổn định.

---

### Bảng 3.4 — Thống kê confidence YOLO11 và độ sâu (depth) — tất cả 50 picks

| Chỉ số | Confidence YOLO11 | Depth (mm) |
|---|---|---|
| Trung bình | 0.9319 | 253.6 |
| Độ lệch chuẩn | 0.0078 | 9.7 |
| Nhỏ nhất | 0.9181 | 233 |
| Lớn nhất | 0.9497 | 270 |
| Grip success | 50/50 (100%) | — |
| Retries used | 0 | — |

> **Nhận xét:** Confidence YOLO trong Phase 3 (**0.9319 ± 0.0078**) gần như đồng nhất với Phase 2 (**0.9322 ± 0.0057**), cho thấy việc tăng số phôi từ 1 lên 5 không ảnh hưởng đến chất lượng phát hiện. Về độ sâu, khoảng biến thiên **233–270mm** (std = 9.7mm) phản ánh chiều cao phôi trên khay không hoàn toàn đồng đều — điều bình thường trong điều kiện thực tế. Hệ thống xử lý thành công toàn bộ dải độ sâu này mà không cần điều chỉnh thêm, xác nhận tính robustness của bước tính tọa độ 3D từ depth map.

---

## 4. Tổng Hợp So Sánh Các Kịch Bản

### Bảng 4.1 — So sánh tổng quan 3 phase

| Chỉ số | Phase 1 | Phase 2 | Phase 3 |
|---|---|---|---|
| Mục tiêu | Chuyển động cơ bản | Vision + 1 phôi | Vòng lặp 5 phôi |
| Số lần chạy | 8 | 21 | 10 job (50 picks) |
| Tỉ lệ thành công | **100%** | **100%** | **100%** |
| Thời gian TB (s) | 36.13* | 82.14 | 230.21 |
| Confidence YOLO | — | 0.9322 | 0.9319 |
| Retry count | — | 0 | 0 |
| Lỗi / cảnh báo | 0 | 0 | 0 |

*6 lần ổn định, sau khi bỏ 2 lần warm-up đầu

> **Nhận xét:** Cả 3 phase đều đạt tỉ lệ thành công 100% với 0 lần retry và 0 lỗi, cho thấy hệ thống được tích hợp và kiểm tra tốt ở mọi cấp độ phức tạp. Thời gian chu trình tăng dần theo mức độ phức tạp: Phase 1 (~36s) → Phase 2 (~82s, +46s cho vision) → Phase 3 (~230s cho 5 phôi, tương đương ~43s/phôi). Confidence YOLO duy trì ổn định giữa Phase 2 và Phase 3 (chênh lệch chỉ 0.0003), xác nhận mô hình hoạt động nhất quán bất kể số lượng phôi cần xử lý.

---

### Bảng 4.2 — Thông số chung tất cả thực nghiệm

| Thông số | Giá trị |
|---|---|
| Tốc độ khớp | 0.8 rad/s |
| Tốc độ tuyến tính | 0.1 m/s |
| Tốc độ tiếp cận pick | 0.05 m/s |
| Thời gian đóng gripper | 511.5 ms |
| Thời gian mở gripper | 314.5 ms |
| Model phát hiện | YOLO11 |
| Ngưỡng confidence tối thiểu quan sát | 0.9181 |
| Khoảng cách phát hiện trung bình | 253.6 mm |

> **Nhận xét:** Toàn bộ thực nghiệm được thực hiện với cùng bộ thông số tốc độ và cấu hình, đảm bảo tính nhất quán khi so sánh kết quả giữa các phase. Tốc độ tiếp cận pick (0.05 m/s) thấp hơn tốc độ tuyến tính chung (0.1 m/s) nhằm giảm thiểu lực tác động khi gripper tiếp xúc phôi. Confidence tối thiểu quan sát được (0.9181) vẫn cao hơn ngưỡng chấp nhận thông thường (0.8–0.85), cho thấy còn dư địa để điều chỉnh ngưỡng phát hiện nếu cần mở rộng sang điều kiện ánh sáng khó hơn.

---

## 5. Nhận Xét Tổng Kết

1. **Độ tin cậy cao toàn hệ thống:** Tất cả 3 phase đều đạt tỉ lệ thành công 100% (tổng cộng 79 lần chạy / 50 picks), không có retry, không có lỗi — cho thấy hệ thống sẵn sàng hoạt động ở chế độ tự động.

2. **YOLO11 ổn định và nhất quán:** Confidence dao động rất nhỏ (std ≈ 0.006–0.008) trong khoảng 0.918–0.950 trên cả Phase 2 và Phase 3, bất kể vị trí slot hay số lượng phôi. Mô hình đủ robust với biến thiên chiều cao phôi (depth 233–270mm).

3. **Hiện tượng warm-up ở Phase 3:** Job đầu tiên (341.2s, 9.2s/phôi) dài hơn đáng kể so với các job sau (208–215s, ~6.1–6.5s/phôi). Hệ thống đạt trạng thái ổn định từ job thứ 3–4, cần tính đến khi đánh giá thời gian chu trình trong điều kiện vận hành thực.

4. **Gripper pneumatic đáng tin cậy:** Thời gian đóng/mở nhất quán (std < 0.2ms), không có sự cố cơ học trong toàn bộ 50 picks. Đây là thành phần hoạt động ổn định nhất của hệ thống.

5. **Tiềm năng cải thiện thời gian:** Phase 3 ổn định đạt ~6.1s/phôi ở trạng thái warm-up. Với tốc độ robot hiện tại (0.1 m/s tuyến tính), vẫn còn dư địa tăng tốc nếu yêu cầu thông lượng cao hơn, đồng thời cần đánh giá tác động đến độ chính xác pick.
