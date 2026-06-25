# Codex Task Prompt — UR5 Pick-Place Bug Fix

## Bối cảnh hệ thống

Đây là hệ thống điều khiển UR5 CB3 (PolyScope 3.15.5) để pick-place phôi tự động.
Stack: Flask PC2 server → URScriptClient → UR5 CB3.

Camera: Orbbec Femto Mega (hand-eye calibrated).
Gripper: Pneumatic, điều khiển qua Arduino serial.
Motion: URScript gửi qua TCP port 30002 dạng `def...end` blocks.

---

## Bug hiện tại

Robot **không xuống touch/pick pose** mặc dù:
- Vision detect đúng phôi
- `p_base` đã được xác nhận đúng theo thực tế
- `touch_pose.Z < approach_pose.Z` (logic tính đúng chiều xuống)
- Pendant jog tay xuống bình thường

Log thực tế:
```
approach_pose Z = 0.383  → actual TCP Z = 0.364  (err ~19mm, chấp nhận được)
touch_pose    Z = 0.238  → actual TCP Z = 0.364  (err 126mm, ROBOT KHÔNG NHÚC NHÍCH)
retreat_pose  Z = 0.383  → actual TCP Z = 0.364  (trùng approach, bỏ qua touch)
```

---

## Nguyên nhân gốc đã phân tích

### 1. Race condition (xác suất ~65%)

Trong `urscript_client.py`, tất cả lệnh dùng `one_shot=True`:
mỗi lệnh = một `def...end` block gửi qua socket riêng rồi đóng.

Vấn đề:
- CB3 có độ trễ 0.3–0.8s để parse và bắt đầu execute program mới
- `wait_steady(motion_start_timeout=0.5)` bị hardcode 0.5s tại TẤT CẢ call sites
- Khi touch move vừa được gửi, robot chưa kịp bắt đầu di chuyển
- `wait_steady` timeout sau 0.5s (thấy robot đứng yên), return luôn
- Lệnh retreat được gửi ngay → CB3 override/hủy touch program đang chờ
- Robot thực hiện retreat, touch bị bỏ qua hoàn toàn

### 2. TCP/Payload không persist (xác suất ~20%)

`set_tcp()` và `set_payload()` được gửi là hai program riêng:
- Program 1: `def external_set_tcp(): set_tcp(...) end` → đóng socket
- Program 2: `def external_movel(): movel(...) end` → CB3 reset TCP về pendant value

Nếu pendant TCP ≠ config TCP, và CoG sai → dynamics controller kích soft safety stop
khi thực hiện chuyển động thẳng đứng xuống, robot đứng im.

`PAYLOAD_COG` hiện là `[0.0, 0.0, 0.16]` — giá trị ước lượng, chưa đo thật.

---

## Các file cần sửa

```
robot/urscript_client.py
core/pick_place.py
config.py
tools/test_scanpose_touch.py   (thêm flag test)
```

---

## Yêu cầu thay đổi chi tiết

### Task 1: `robot/urscript_client.py` — Thêm method bundle settings+motion

Thêm method mới `move_linear_with_settings()` vào class `URScriptClient`.
Method này gửi `set_tcp`, `set_payload`, `movel` trong **một program duy nhất**
để đảm bảo settings có hiệu lực trong cùng execution context với motion command.

```python
def move_linear_with_settings(
    self,
    pose: List[float],
    tcp_offset: List[float],
    payload_kg: float,
    payload_cog: List[float],
    accel: float = 0.3,
    vel: float = 0.1,
) -> None:
    """
    Move movel trong cùng program với set_tcp và set_payload.

    Trên CB3, set_tcp/set_payload gửi riêng không persist sang program movel tiếp theo.
    Method này bundle tất cả vào một def...end block để đảm bảo đồng nhất.

    Args:
        pose: [x, y, z, rx, ry, rz] meters + radians
        tcp_offset: [x, y, z, rx, ry, rz] TCP offset
        payload_kg: payload mass kg
        payload_cog: [x, y, z] center of gravity
        accel: linear acceleration m/s²
        vel: linear velocity m/s
    """
    tcp_str = ",".join(f"{v:.6f}" for v in tcp_offset)
    cog_str = ",".join(f"{v:.6f}" for v in payload_cog)
    pose_str = ",".join(f"{p:.6f}" for p in pose)

    logger.info(f"move_linear_with_settings to pose: {pose}")
    self.send_program(
        [
            f"set_tcp(p[{tcp_str}])",
            f"set_payload({payload_kg:.4f}, [{cog_str}])",
            f"movel(p[{pose_str}], a={accel}, v={vel})",
        ],
        program_name="external_movel_full",
        one_shot=True,
    )
```

Tương tự, thêm `move_joint_with_settings()` cho movej:

```python
def move_joint_with_settings(
    self,
    joints: List[float],
    tcp_offset: List[float],
    payload_kg: float,
    payload_cog: List[float],
    accel: float = 1.0,
    vel: float = 0.8,
) -> None:
    """
    Move movej trong cùng program với set_tcp và set_payload.
    """
    tcp_str = ",".join(f"{v:.6f}" for v in tcp_offset)
    cog_str = ",".join(f"{v:.6f}" for v in payload_cog)
    joints_str = ",".join(f"{j:.6f}" for j in joints)

    logger.info(f"move_joint_with_settings to joints: {joints}")
    self.send_program(
        [
            f"set_tcp(p[{tcp_str}])",
            f"set_payload({payload_kg:.4f}, [{cog_str}])",
            f"movej([{joints_str}], a={accel}, v={vel})",
        ],
        program_name="external_movej_full",
        one_shot=True,
    )
```

---

### Task 2: `core/pick_place.py` — Sửa motion_start_timeout và dùng method mới

#### 2a. Thay TẤT CẢ `motion_start_timeout=0.5` bằng `config.RTDE_MOTION_START_TIMEOUT`

Tìm và replace toàn bộ trong file:
```python
# Cũ (hardcode):
self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT, motion_start_timeout=0.5)

# Mới (dùng config):
self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT, motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT)
```

Có duy nhất 2 chỗ dùng `motion_start_timeout=10.0` trong error/abort cleanup — **giữ nguyên**.

#### 2b. Thêm `time.sleep` trước `wait_steady` sau bước touch/pick final pose

Tìm các bước descent (approach → final_pose) và thêm sleep:

```python
# Bước descend to pick
self.urscript.move_linear(final_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
time.sleep(0.5)  # CB3 parse latency — đảm bảo motion bắt đầu trước wait_steady
self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT, motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT)
```

Thêm tương tự cho bước `PLACE_POINT_CART`.

#### 2c. Xóa `_apply_runtime_tool_settings()` riêng biệt, thay bằng bundle trong từng motion

Bỏ (hoặc giữ lại nhưng không gọi nữa):
```python
def _apply_runtime_tool_settings(self) -> None:
    self.urscript.set_tcp(config.TCP_OFFSET)       # ← không persist
    self.urscript.set_payload(...)                  # ← không persist
```

Thay các lệnh `move_joint` và `move_linear` quan trọng bằng version bundle:

```python
# Thay:
self.urscript.move_joint(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)

# Bằng:
self.urscript.move_joint_with_settings(
    config.HOME_JOINTS,
    tcp_offset=config.TCP_OFFSET,
    payload_kg=config.PAYLOAD_MASS_KG,
    payload_cog=config.PAYLOAD_COG,
    accel=config.JOINT_ACCEL,
    vel=config.JOINT_VEL,
)
```

Áp dụng tương tự cho các bước:
- `SCAN_APPROACH_JOINTS`
- `SCAN_POSE_JOINTS`
- `approach_pose` (move_linear_with_settings)
- `final_pose` (move_linear_with_settings)
- `PLACE_APPROACH_CART`
- `PLACE_POINT_CART`
- `PLACE_RETREAT_CART`
- retreat về approach sau pick

**Lưu ý:** Giữ lại `_apply_runtime_tool_settings()` trong log để trace, nhưng đánh dấu deprecated:
```python
def _apply_runtime_tool_settings(self) -> None:
    """DEPRECATED: set_tcp/payload không persist sang program riêng trên CB3.
    Dùng move_*_with_settings() thay thế. Giữ lại để log thông tin tool config."""
    self._log(
        "Tool config (bundled vào từng motion): tcp={}, mass={:.3f}kg, cog={}".format(
            [round(float(v), 4) for v in config.TCP_OFFSET],
            config.PAYLOAD_MASS_KG,
            [round(float(v), 4) for v in config.PAYLOAD_COG],
        )
    )
```

---

### Task 3: `config.py` — Thêm config cho skip flags và tăng RTDE timeout

#### 3a. Tăng default `RTDE_MOTION_START_TIMEOUT`

```python
# Cũ:
RTDE_MOTION_START_TIMEOUT = float(os.getenv("RTDE_MOTION_START_TIMEOUT", "2.0"))

# Mới:
RTDE_MOTION_START_TIMEOUT = float(os.getenv("RTDE_MOTION_START_TIMEOUT", "3.0"))
```

#### 3b. Thêm flag để test isolation (dùng trong tools)

```python
# Debug/test flags — không dùng trong production runtime
SKIP_SET_TCP = os.getenv("SKIP_SET_TCP", "False").lower() == "true"
SKIP_SET_PAYLOAD = os.getenv("SKIP_SET_PAYLOAD", "False").lower() == "true"
```

#### 3c. Thêm sleep config cho CB3 parse latency

```python
# CB3 URScript parse latency — thời gian chờ sau khi gửi program trước wait_steady
# Đặc biệt cần thiết cho short-distance moves (touch/place) mà CB3 chậm start
CB3_MOTION_PRE_WAIT_SLEEP_S = float(os.getenv("CB3_MOTION_PRE_WAIT_SLEEP_S", "0.5"))
```

---

### Task 4: `tools/test_scanpose_touch.py` — Thêm CLI flags để test isolation

Thêm argparse flags:

```python
parser.add_argument(
    "--skip-set-payload",
    action="store_true",
    help="Bỏ qua set_payload() để test ảnh hưởng của payload/CoG sai",
)
parser.add_argument(
    "--skip-set-tcp",
    action="store_true",
    help="Bỏ qua set_tcp() để test ảnh hưởng của TCP runtime conflict",
)
parser.add_argument(
    "--motion-start-timeout",
    type=float,
    default=None,
    help="Override RTDE_MOTION_START_TIMEOUT (s). Default: dùng config.",
)
parser.add_argument(
    "--pre-wait-sleep",
    type=float,
    default=None,
    help="Sleep (s) trước wait_steady sau touch command. Default: dùng config.",
)
parser.add_argument(
    "--touch-mode",
    choices=["movel", "movej-ik", "movel-bundled"],
    default="movel-bundled",
    help="Mode gửi touch command. movel-bundled = bundle settings+movel 1 program.",
)
```

Logic sử dụng flags:

```python
motion_start_timeout = args.motion_start_timeout or config.RTDE_MOTION_START_TIMEOUT
pre_wait_sleep = args.pre_wait_sleep if args.pre_wait_sleep is not None else config.CB3_MOTION_PRE_WAIT_SLEEP_S

# Apply settings
if not args.skip_set_tcp:
    urscript.set_tcp(config.TCP_OFFSET)
    logger.info("set_tcp applied")
else:
    logger.warning("SKIP_SET_TCP: bỏ qua set_tcp()")

if not args.skip_set_payload:
    urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
    logger.info("set_payload applied: %.2fkg, cog=%s", config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
else:
    logger.warning("SKIP_SET_PAYLOAD: bỏ qua set_payload()")

# Touch command
if args.touch_mode == "movel-bundled":
    urscript.move_linear_with_settings(
        touch_pose,
        tcp_offset=config.TCP_OFFSET,
        payload_kg=config.PAYLOAD_MASS_KG,
        payload_cog=config.PAYLOAD_COG,
        accel=config.LINEAR_ACCEL,
        vel=config.PICK_APPROACH_VEL,
    )
elif args.touch_mode == "movel":
    urscript.move_linear(touch_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
elif args.touch_mode == "movej-ik":
    urscript.move_joint_to_pose_ik(touch_pose, accel=0.5, vel=0.3)

time.sleep(pre_wait_sleep)
rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT, motion_start_timeout=motion_start_timeout)
```

---

## Thứ tự test sau khi apply fix

```bash
# Test A: Chỉ tăng timeout — xác nhận race condition
python tools/test_scanpose_touch.py --motion-start-timeout 5.0 --pre-wait-sleep 0.5

# Test B: Bundle mode (fix chính)
python tools/test_scanpose_touch.py --touch-mode movel-bundled

# Test C: Bỏ payload — isolate CoG issue
python tools/test_scanpose_touch.py --skip-set-payload --motion-start-timeout 5.0

# Test D: Bỏ cả TCP và payload — baseline pendant state
python tools/test_scanpose_touch.py --skip-set-tcp --skip-set-payload --motion-start-timeout 5.0
```

Đọc log `Touch actual TCP Z` sau mỗi test:
- Nếu Test A fix được → race condition là thủ phạm
- Nếu Test B fix được → TCP/payload isolation là thủ phạm  
- Nếu Test C fix được → CoG sai là thủ phạm
- Nếu Test D fix được nhưng B không fix → TCP offset trong config sai

---

## KHÔNG làm những việc sau

- Không đảo dấu trong `T_cam_to_tcp` (hand-eye đã được xác nhận)
- Không lật transform `cam->tcp` thành `tcp->cam`
- Không sửa `pixel_to_camera_3d()` hay `camera_to_base()`
- Không thay đổi `TOOL_DOWN_RX/RY/RZ`
- Không sửa logic detection/depth trong `detector.py` hay `femto_camera.py`
- Không thêm bất kỳ workaround nào liên quan đến đảo dấu Z

---

## Ràng buộc code

- Giữ nguyên tất cả type hints
- Giữ nguyên logging pattern (`logger.info`, `logger.warning`, `logger.error`)
- Không break backward compat với các call site cũ của `move_linear()` và `move_joint()`
- Method mới là additive, không replace method cũ
- Tất cả string format dùng f-string, không dùng `.format()` mới trong method mới
- Giữ nguyên `one_shot=True` trên tất cả `send_program()` calls
