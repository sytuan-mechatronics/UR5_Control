# Runtime Setting Diagnosis

Mục tiêu của folder này:
- tách riêng việc test `set_tcp()` và `set_payload()` khỏi vision
- xem chính runtime setting từ code có làm robot chạy khác pendant hay không

## File chính

- [test_runtime_settings_z.py](/home/tuan/Downloads/Ur5_Control-main/runtime_setting_diagnosis/test_runtime_settings_z.py)

## Ý nghĩa 2 cờ test

- `--skip-set-payload`
  - code sẽ không gọi `set_payload(...)`
  - robot dùng payload/CoG đang active trên pendant

- `--skip-set-tcp`
  - code sẽ không gọi `set_tcp(...)`
  - robot dùng TCP đang active trên pendant

## Quy trình test chuẩn

1. Để robot ở vị trí an toàn, có chỗ đi lên/xuống thêm 10 mm.
2. Giữ nguyên tool/TCP/payload trên pendant ở trạng thái bạn thấy jog mượt.
3. Chạy 4 kịch bản theo đúng thứ tự này.

### Test 1: mặc định

```bash
python3 runtime_setting_diagnosis/test_runtime_settings_z.py --robot-ip 192.168.125.11 --dz-mm 10
```

### Test 2: bỏ set payload

```bash
python3 runtime_setting_diagnosis/test_runtime_settings_z.py --robot-ip 192.168.125.11 --dz-mm 10 --skip-set-payload
```

### Test 3: bỏ set tcp

```bash
python3 runtime_setting_diagnosis/test_runtime_settings_z.py --robot-ip 192.168.125.11 --dz-mm 10 --skip-set-tcp
```

### Test 4: bỏ cả hai

```bash
python3 runtime_setting_diagnosis/test_runtime_settings_z.py --robot-ip 192.168.125.11 --dz-mm 10 --skip-set-payload --skip-set-tcp
```

## Cách đọc kết quả

Mỗi test nhìn 3 dòng chính:
- `Start TCP`
- `Target TCP`
- `Actual dz`

## Kết luận

- Nếu bỏ `set_payload` mà motion xuống tốt hơn:
  - nghi mạnh `payload/CoG` do code set sai

- Nếu bỏ `set_tcp` mà motion xuống tốt hơn:
  - nghi mạnh `TCP` do code set sai hoặc khác active TCP trên pendant

- Nếu chỉ khi bỏ cả hai mới tốt:
  - runtime code đang lệch pendant ở cả TCP và payload

- Nếu cả 4 test đều như nhau:
  - lỗi không nằm ở runtime `set_tcp/set_payload`, cần quay lại kiểm tra motion path hoặc pose vận hành

