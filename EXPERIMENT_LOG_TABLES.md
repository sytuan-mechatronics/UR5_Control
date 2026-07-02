# Bang 4 Log Thuc Nghiem

Tai lieu nay chi giu dung `4 bang`, moi bang ung voi `1 log`.

## 1. Log Workflow Tong MIR + UR5

File de xuat:

- `workflow_report.csv` hoac file log tong ben PC1 / workflow controller

Khi ghi:

- ghi `1 dong` sau moi lan bam `Start` va workflow ket thuc

| Ten cot | Y nghia | Don vi / gia tri |
|---|---|---|
| `run_id` | Ma lan chay tong | chuoi |
| `workflow_id` | Ma workflow | chuoi |
| `scenario_case` | Truong hop thuc nghiem tong, vd `5a`, `5b`, `5c` | chuoi |
| `mir_travel_time_s` | Tong thoi gian MIR di chuyen | `s` |
| `mir_stop_error_m` | Sai so dung cua MIR so voi diem dich | `m` |
| `feed_wait_time_s` | Thoi gian cho cap phoi neu co | `s` |
| `vision_confirm_result` | Vision co xac nhan phoi hay khong | `true/false` hoac chuoi mo ta |
| `vision_confidence` | Do tin cay xac nhan vision | `0..1` |
| `ur5_cycle_time_s` | Tong thoi gian phan UR5 thuc hien | `s` |
| `ur5_parts_found` | So phoi tim thay | so nguyen |
| `ur5_parts_picked` | So phoi gap thanh cong | so nguyen |
| `workflow_total_time_s` | Tong thoi gian workflow tu dau den cuoi | `s` |
| `final_status` | Trang thai ket thuc | `completed`, `timeout`, `error`, `aborted` |
| `error_note` | Loi hoac canh bao chinh | chuoi |

## 2. Log Kich Ban 1 / Phase 1

File dang dung:

- `results/scenario1_phase1.csv`

Tool tao log:

- `python3 tools/test_phase1.py`

Khi ghi:

- ghi `1 dong` cho moi lan chay phase 1

| Ten cot | Y nghia | Don vi / gia tri |
|---|---|---|
| `job_id` | Ma job | chuoi |
| `created_at` | Thoi diem tao job | ISO UTC |
| `completed_at` | Thoi diem ket thuc | ISO UTC |
| `duration_s` | Tong thoi gian job | `s` |
| `station` | Tram / nguon test | chuoi |
| `workflow_id` | Ma workflow hoac ma lan test | chuoi |
| `experiment_stage` | Stage thuc nghiem | `1` |
| `cycle_status` | Trang thai ket thuc chu trinh | `done`, `error`, `aborted` |
| `cycle_time_s` | Thoi gian 1 chu trinh motion | `s` |
| `gripper_close_ms` | Thoi gian dong gripper | `ms` |
| `gripper_open_ms` | Thoi gian mo gripper | `ms` |
| `warning_count` | So canh bao trong job log | so nguyen |
| `error_or_warning` | Loi/canh bao quan trong nhat | chuoi |
| `note` | Ghi chu thu cong | chuoi |

## 3. Log Kich Ban 2 / Phase 2

File dang dung:

- `results/scenario2_phase2.csv`

Tool tao log:

- `python3 tools/test_phase2.py`

Khi ghi:

- ghi `1 dong` cho moi lan chay phase 2

| Ten cot | Y nghia | Don vi / gia tri |
|---|---|---|
| `job_id` | Ma job | chuoi |
| `created_at` | Thoi diem tao job | ISO UTC |
| `completed_at` | Thoi diem ket thuc | ISO UTC |
| `duration_s` | Tong thoi gian job | `s` |
| `station` | Tram / nguon test | chuoi |
| `workflow_id` | Ma workflow hoac ma lan test | chuoi |
| `experiment_stage` | Stage thuc nghiem | `2` |
| `confidence_yolo11` | Do tin cay phat hien YOLO | `0..1` |
| `localization_error_mm` | Sai so dinh vi thuc te | `mm` |
| `target_x_m` | Toa do target X trong he base | `m` |
| `target_y_m` | Toa do target Y trong he base | `m` |
| `target_z_m` | Toa do target Z trong he base | `m` |
| `pick_result` | Ket qua gap | `success`, `no_detection`, `invalid_depth_zero`, ... |
| `retry_count` | So lan thu lai | so nguyen |
| `slot_position` | Vi tri slot cua phoi duoc nhan dien | chuoi |
| `selected_slot` | Slot duoc chon neu co mapping correction | chuoi |
| `cycle_status` | Trang thai ket thuc | `done`, `error`, `aborted` |
| `parts_found` | So phoi detect duoc | so nguyen |
| `parts_picked` | So phoi gap thanh cong | so nguyen |
| `error_or_warning` | Loi/canh bao quan trong | chuoi |
| `note` | Ghi chu thu cong | chuoi |

## 4. Log Kich Ban 3 / Phase 3

File dang dung:

- `results/scenario3_phase3.csv`

Tool tao log:

- `python3 tools/test_phase3.py --scenario-case 3a`

Khi ghi:

- ghi `nhieu dong` cho `1 lan chay`
- moi dong ung voi `1 phoi`

| Ten cot | Y nghia | Don vi / gia tri |
|---|---|---|
| `job_id` | Ma job | chuoi |
| `created_at` | Thoi diem tao job | ISO UTC |
| `completed_at` | Thoi diem ket thuc | ISO UTC |
| `duration_s` | Tong thoi gian ca lan chay | `s` |
| `station` | Tram / nguon test | chuoi |
| `workflow_id` | Ma workflow hoac ma lan test | chuoi |
| `experiment_stage` | Stage thuc nghiem | `3` |
| `scenario_case` | Truong hop con `3a`, `3b`, `3c`, `3d` | chuoi |
| `parts_found_initial` | So phoi tim thay o lan scan dau | so nguyen |
| `parts_picked_total` | Tong so phoi gap thanh cong | so nguyen |
| `run_status` | Trang thai lan chay | `done`, `error`, `aborted` |
| `tray_position` | Vi tri khay/slot cua phoi | chuoi |
| `pick_order` | Thu tu gap | so nguyen |
| `pick_result` | Ket qua cua phoi do | `success`, `fail`, `no_pick` |
| `retry_count` | So lan thu lai cua phoi do | so nguyen |
| `confidence_yolo11` | Confidence cua phoi do | `0..1` |
| `part_duration_s` | Thoi gian xu ly rieng cua phoi do | `s` |
| `error_or_warning` | Loi/canh bao quan trong | chuoi |
| `note` | Ghi chu thu cong | chuoi |
