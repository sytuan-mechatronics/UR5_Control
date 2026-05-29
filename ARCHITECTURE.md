# Architecture & Design - PC2 UR5 Control System

Tài liệu này giải thích kiến trúc code, design patterns, và flow chính của hệ thống.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         PC1 (Main PC)                       │
│                  (Workflow orchestration)                    │
└────────────────────────┬────────────────────────────────────┘
                         │
                    REST API (POST /api/ur5/execute)
                         │
┌────────────────────────▼────────────────────────────────────┐
│                       PC2 (Flask Server)                     │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Flask Application (app.py)              │   │
│  │  ├─ ur5_bp: REST API endpoints                       │   │
│  │  ├─ job_store: In-memory job database (thread-safe) │   │
│  │  └─ _run_job(job_id): Main orchestrator thread      │   │
│  └──────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            Core: PickPlaceCycle (pick_place.py)      │   │
│  │  ├─ run(): Main pick-place state machine             │   │
│  │  ├─ _check_abort(): Check for abort signal           │   │
│  │  └─ _set_phase(): Update job phase                   │   │
│  └──────────────────────────────────────────────────────┘   │
│  ┌────────────────┬──────────────────┬──────────────────┐   │
│  │   Robot        │     Vision       │   Controllers    │   │
│  │  Subsystem     │    Subsystem     │    (Abstract)    │   │
│  ├────────────────┼──────────────────┼──────────────────┤   │
│  │ dashboard_     │ femto_camera.py  │ config.py        │   │
│  │ client.py      │ (Orbbec Femto)   │ (All settings)   │   │
│  │ (Port 29999)   │                  │                  │   │
│  │                │ detector.py      │ job_store.py     │   │
│  │ urscript_      │ (YOLO v8)        │ (Job database)   │   │
│  │ client.py      │                  │                  │   │
│  │ (Port 30002)   │ calibration.py   │                  │   │
│  │                │ (Hand-eye calib) │                  │   │
│  │ rtde_client.py │                  │                  │   │
│  │ (Port 30004)   │                  │                  │   │
│  │                │                  │                  │   │
│  │ gripper_rg.py  │                  │                  │   │
│  │ (Gripper cmd)  │                  │                  │   │
│  └────────────────┴──────────────────┴──────────────────┘   │
└──────────────────────┬───────────────────────────────────────┘
                       │ Robot network
                       │ (ports 29999, 30002, 30004)
┌──────────────────────▼────────────────────────────────────────┐
│                    UR5 Robot Controller                        │
│  ├─ Dashboard Server (29999): Safety, power, brake            │
│  ├─ URScript Server (30002): Motion commands                  │
│  └─ RTDE Server (30004): Real-time state feedback             │
└────────────────────────────────────────────────────────────────┘
                       │ USB
┌──────────────────────▼────────────────────────────────────────┐
│         Orbbec Femto Mega Camera (eye-in-hand)                │
└────────────────────────────────────────────────────────────────┘
```

## Core State Machine: PickPlaceCycle.run()

```
START
  │
  ├─► INITIALIZING
  │   └─ Connect: dashboard, urscript, rtde, camera
  │   └─ Safety checks
  │   └─ Power on, brake release
  │   └─ Gripper open
  │
  ├─► MOVING_TO_HOME
  │   └─ movej(HOME_JOINTS)
  │   └─ wait_steady()
  │
  ├─► MOVING_TO_SCAN_APPROACH
  │   └─ movej(SCAN_APPROACH_JOINTS)
  │   └─ wait_steady()
  │
  ├─► MOVING_TO_SCAN_POSE
  │   └─ movej(SCAN_POSE_JOINTS)
  │   └─ wait_steady()
  │
  ├─► INITIAL_SCAN
  │   └─ capture_frame() + detect()
  │   └─ Nếu parts_found == 0 → NO_PARTS_FOUND → END
  │
  ├─► MAIN PICK LOOP (for each part or timeout)
  │   │
  │   ├─► SCANNING_BEFORE_PICK_{i}
  │   │   └─ Chụp ảnh + phát hiện
  │   │   └─ Chọn phôi tốt nhất từ depth + YOLO score
  │   │   └─ Tính tọa độ: pixel → camera 3D → base frame
  │   │
  │   ├─► MOVING_TO_PICK_APPROACH_{i}
  │   │   └─ movel(approach_pose)
  │   │   └─ wait_steady()
  │   │
  │   ├─► PICKING_{i} [with retry logic]
  │   │   ├─ movel(final_pose, slow_vel)  # Descend
  │   │   ├─ gripper.close()
  │   │   ├─ wait_grip_detected()
  │   │   │
  │   │   ├─ Nếu grip OK:
  │   │   │  └─ Continue to retreat
  │   │   │
  │   │   └─ Nếu grip FAIL:
  │   │      ├─ Nếu retry < MAX_PICK_RETRIES:
  │   │      │  ├─ movel(approach_pose)  # Retreat
  │   │      │  ├─ gripper.open()
  │   │      │  └─ [loop back to PICKING with retry++]
  │   │      │
  │   │      └─ Nếu retry >= MAX_PICK_RETRIES:
  │   │         └─ Raise RuntimeError("Grip failed")
  │   │
  │   ├─► RETREATING_AFTER_PICK_{i}
  │   │   └─ movel(approach_pose)
  │   │   └─ wait_steady()
  │   │
  │   ├─► MOVING_TO_PLACE_{i}
  │   │   └─ movel(PLACE_APPROACH_CART)
  │   │   └─ movel(PLACE_POINT_CART, slow_vel)
  │   │   └─ wait_steady()
  │   │
  │   ├─► PLACING_{i}
  │   │   └─ gripper.open()
  │   │   └─ sleep(0.3)  # Settle
  │   │
  │   ├─► RETREATING_AFTER_PLACE_{i}
  │   │   └─ movel(PLACE_RETREAT_CART)
  │   │   └─ wait_steady()
  │   │
  │   ├─► [Return to SCAN_POSE for next part]
  │   │   └─ movej(SCAN_POSE_JOINTS)
  │   │   └─ parts_picked += 1
  │   │
  │   └─ [End of loop]
  │
  ├─► RETURNING_HOME
  │   └─ movej(HOME_JOINTS)
  │   └─ wait_steady()
  │
  ├─► DONE
  │   └─ Update job status = "done"
  │   └─ Cleanup: disconnect all
  │   └─ Return result
  │
  └─► [Exception handlers]
      ├─ ABORTED (AbortException)
      │  └─ gripper.open() + movej(HOME)
      │  └─ status = "aborted"
      │
      └─ ERROR (any Exception)
         └─ gripper.open() + movej(HOME)
         └─ status = "error"
         └─ Save error message
```

## Thread Safety & Concurrency

### Design:
- **Single job at a time**: Chỉ 1 job chạy trên robot cùng lúc
- **Per-job thread**: Mỗi job run trong thread riêng
- **Shared JobStore**: Tất cả threads access JobStore qua lock

### Implementation:

```python
# app.py
ur5_bp.execute_job()  # API endpoint
  ├─ Check: is active_job running? (with lock)
  ├─ Create job in job_store
  ├─ Set active_job = job_id
  ├─ Spawn thread: _run_job(job_id)
  └─ Return 202 immediately

# In background thread
_run_job(job_id)
  ├─ Create PickPlaceCycle
  ├─ cycle.run()  # Main logic
  └─ Finally: Set active_job = None

# JobStore (core/job_store.py)
class JobStore:
  lock = threading.Lock()  # Protect all operations
  
  update_job(job_id, **kwargs):
    with lock:
      jobs[job_id].update(**kwargs)  # Atomic update
```

### Safety properties:
1. ✓ No race condition: All job updates locked
2. ✓ No double-run: Active job check locked
3. ✓ Clean abort: AbortException caught, cleanup in finally block

## Coordinate Transformation Pipeline

**Objective**: Từ pixel phát hiện trong ảnh → tọa độ robot base frame

```
Pixel (u, v) in image
         │
         │ + Depth (Z_cam from depth frame)
         │
         ▼
Camera 3D point: pixel_to_camera_3d()
    [X_cam, Y_cam, Z_cam]  ← formula
    
         │
         │ + T_cam_to_tcp (hand-eye calibration, 4x4 matrix)
         │ + TCP pose at capture (T_base_tcp from RTDE)
         │
         ▼
camera_to_base()
    T_base_cam = T_base_tcp @ T_cam_to_tcp
    [X_base, Y_base, Z_base]

         │
         │ + Offset Z (PICK_APPROACH_OFFSET_Z)
         │ + Tool orientation (TOOL_DOWN_RX/RY/RZ)
         │
         ▼
build_pick_approach_pose()
    [X_base, Y_base, Z_base + offset, RX, RY, RZ]
    
         │
         ▼
    movel(pose) command to robot
```

**Critical path**: Hand-eye calibration chính xác → Chỉ có 1-2mm error là OK

## Motion Profiles

### Phases:

| Phase | vel (rad/s) | accel (rad/s²) | Purpose |
|-------|-------------|----------------|---------|
| Approach | 0.8 | 1.0 | Nhanh từ home đến scan |
| Scan | 0.8 | 1.0 | Nhanh từ scan approach → scan pose |
| Move to pick approach | 0.1 | 0.3 | Chậm, an toàn |
| Final pick descent | 0.05 | 0.3 | Rất chậm, sensitive |
| Move to place | 0.1 | 0.3 | Chậm, an toàn |
| Place descent | 0.05 | 0.3 | Rất chậm, sensitive |

### Safety constraints:
- RTDE monitoring: Tất cả motion phải có rtde.wait_steady() trước khi next step
- Timeout: 30s max per motion (configurable)
- Gripper timeout: 3s max for grip detection

## Error Handling

### Exception hierarchy:

```
Exception
├─ AbortException (custom)
│  └─ Caught in PickPlaceCycle.run()
│     → Graceful shutdown: gripper.open() + home
│     → status = "aborted"
│
├─ RuntimeError (robot not ready, grip failed, etc.)
│  └─ Caught in PickPlaceCycle.run()
│     → Attempt cleanup
│     → status = "error"
│     → Save error message to job
│
└─ Other (network, library bugs)
   └─ Caught in PickPlaceCycle.run()
      → Try cleanup in finally block
      → status = "error"
```

### Cleanup strategy:

```python
finally:
  try:
    camera.disconnect()
  except:
    log error
  
  try:
    rtde.disconnect()
  except:
    log error
  
  try:
    urscript.disconnect()
  except:
    log error
  
  try:
    dashboard.disconnect()
  except:
    log error
```

→ Ensures clean shutdown even if multiple errors

## API Flow Diagram

```
                    PC1 Request
                        │
                        ▼
            POST /api/ur5/execute
                (job_id, station, workflow_id)
                        │
                        ▼
        ur5_bp.execute_job() [Synchronous endpoint]
                │
                ├─ Validate input
                ├─ Check if active_job exists
                │  ├─ Yes: Return 409 Conflict
                │  └─ No: Create job in job_store
                ├─ Spawn thread: _run_job()
                └─ Return 202 Accepted immediately
                        │
                        ▼
    Background thread: _run_job(job_id)
                │
                ├─ PickPlaceCycle.run()
                │  ├─ Motion loop
                │  ├─ Error handling
                │  └─ Return result dict
                │
                ├─ Update job: status="done" or "error"
                │
                └─ [If PC1_CALLBACK_ENABLED]
                   POST PC1_UR5_DONE_URL
                   (job_id, success, result/error)
                        │
                        ▼
            PC1 receives callback
```

## Performance Considerations

1. **YOLO inference**: ~100-200ms per frame (GPU), ~500ms (CPU)
   - Bottleneck if CPU-only
   - Solution: Use GPU (CUDA/RTX) or smaller model

2. **RTDE polling**: 10Hz = 100ms per read
   - Safe threshold for steady check: 30s typical

3. **Motion times**:
   - Home → Scan: ~5-10s
   - Scan + detect: ~0.5s
   - Approach + pick + retreat: ~10-15s
   - Move to place + place: ~10-15s
   - Total per part: ~30-40s average

4. **Memory**:
   - JobStore: ~1KB per job
   - Images (RGB + depth): ~2-3MB per frame
   - YOLO model: ~100-200MB (in memory)
   - Total baseline: ~300MB

## Design Patterns Used

1. **Context Manager** (robot/*.py):
   ```python
   with DashboardClient(ip) as client:
       client.connect()  # Auto
       # use client
       client.disconnect()  # Auto
   ```

2. **Singleton pattern** (job_store):
   - Shared instance across all threads
   - All access synchronized via lock

3. **State Machine** (PickPlaceCycle):
   - Clear phase transitions
   - Logged for debugging

4. **Template Method** (app.py):
   - execute_job: High-level flow
   - _run_job: Implementation

5. **Observer pattern** (job_store):
   - Consumers poll job status
   - No push notifications needed

## Future Improvements

1. **Async I/O**: Replace sync socket with async
2. **YOLO tracking**: Persistent object tracking between frames
3. **Adaptive motion**: Slow down near objects based on depth
4. **Force feedback**: Monitor gripper force, adjust grasp
5. **Multi-robot**: Support multiple UR5 instances
6. **ML pipeline**: Learn optimal pick points from failures
