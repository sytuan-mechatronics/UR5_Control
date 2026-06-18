"""Move robot from SCAN_POSE to the current vision target and return.

Purpose:
- Capture one frame at SCAN_POSE
- Compute the same target point used by vision
- Save an annotated image so the operator can see which point was selected
- Move robot to that target using a guarded pre-approach -> approach -> touch flow

This is a verification tool, not a production cycle.
"""

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from vision.calibration import (
    build_lateral_pre_approach_pose,
    camera_to_base,
    pixel_to_camera_3d,
    resolve_intrinsics_for_frame,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
from vision.tray_holes import (
    assign_pick_to_layout_hole,
    detect_tray_holes,
    match_tray_layout_to_detected_holes,
    snap_pick_to_nearest_hole,
)
from vision.tray_reference import refine_base_xy_with_checkerboard


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy tool.")
        raise SystemExit(0)


def wait_steady(rtde: RTDEClient, label: str) -> bool:
    ok = rtde.wait_steady(
        timeout_s=config.RTDE_WAIT_TIMEOUT,
        threshold=config.RTDE_STEADY_THRESHOLD,
        motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
    )
    if not ok:
        print(f"Canh bao: timeout khi cho robot dung tai buoc {label}.")
    return ok


def require_steady(rtde: RTDEClient, label: str) -> None:
    if not wait_steady(rtde, label):
        raise RuntimeError(f"Robot khong on dinh sau buoc {label}.")


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def pose_position_error_mm(actual_pose, target_pose) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(target_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


def print_pose_delta(label: str, actual_pose, target_pose) -> None:
    delta = [actual_pose[i] - target_pose[i] for i in range(6)]
    print(f"{label} actual TCP: {[round(v, 4) for v in actual_pose]}")
    print(f"{label} delta:      {[round(v, 4) for v in delta]} (pos_err={pose_position_error_mm(actual_pose, target_pose):.1f} mm)")


def assert_pose_reached(label: str, actual_pose, target_pose, tolerance_mm: float = 15.0) -> None:
    err_mm = pose_position_error_mm(actual_pose, target_pose)
    if err_mm > tolerance_mm:
        raise RuntimeError(
            f"{label} khong den duoc target pose. "
            f"Sai so vi tri = {err_mm:.1f} mm > {tolerance_mm:.1f} mm."
        )


def clamp_pick_z_sequence(
    scan_z: float,
    point_z: float,
    approach_offset_z: float,
    touch_offset_z: float,
    retreat_offset_z: float,
    min_descent_mm: float = 5.0,
) -> Tuple[float, float, float]:
    min_descent_m = min_descent_mm / 1000.0
    max_working_z = scan_z - min_descent_m
    touch_z = point_z + touch_offset_z
    approach_z = min(point_z + approach_offset_z, max_working_z)
    retreat_z = min(point_z + retreat_offset_z, max_working_z)
    return approach_z, touch_z, retreat_z


def capture_valid_frames(camera: FemtoCamera, max_attempts: int = 8):
    for attempt in range(1, max_attempts + 1):
        rgb, depth, cam_ts = camera.get_frames_with_timestamp()
        rgb_nonzero = int(np.count_nonzero(rgb))
        depth_nonzero = int(np.count_nonzero(depth))
        print(
            f"Frame attempt {attempt}/{max_attempts}: "
            f"rgb_nonzero={rgb_nonzero}, depth_nonzero={depth_nonzero}, "
            f"shape_rgb={rgb.shape}, shape_depth={depth.shape}"
        )
        if rgb_nonzero > 0 and depth_nonzero > 0:
            return rgb, depth, cam_ts
        time.sleep(0.05)
    raise RuntimeError("Khong lay duoc frame hop le (RGB/Depth deu rong)")


def draw_overlay(frame_bgr, target, holes=None, layout_match=None):
    overlay = frame_bgr.copy()
    if holes:
        for hole in holes:
            hu, hv = [int(round(v)) for v in hole["center"]]
            hr = int(round(hole["radius_px"]))
            cv2.circle(overlay, (hu, hv), hr, (0, 200, 255), 2)

    if target is not None:
        x1, y1, x2, y2 = [int(v) for v in target.bbox]
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 180, 0), 2)
        pu, pv = int(round(target.pick_point[0])), int(round(target.pick_point[1]))
        cv2.circle(overlay, (pu, pv), 7, (0, 0, 255), -1)
        cv2.putText(overlay, "VISION_PICK", (pu + 10, pv - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        px1, py1, px2, py2 = [int(v) for v in target.pick_bbox]
        cv2.rectangle(overlay, (px1, py1), (px2, py2), (0, 255, 255), 2)

    if layout_match is not None:
        for idx, hole_uv in enumerate(layout_match["projected_holes_uv"], start=1):
            hu, hv = [int(round(v)) for v in hole_uv]
            cv2.circle(overlay, (hu, hv), 10, (255, 0, 255), 2)
            cv2.putText(overlay, f"H{idx}", (hu + 8, hv - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)
    return overlay


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move robot to the current vision target from SCAN_POSE")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--yes", action="store_true", help="Skip interactive confirmation")
    parser.add_argument(
        "--target-stage",
        choices=["pre-approach", "approach", "touch"],
        default="touch",
        help="Deepest point the robot should reach before returning",
    )
    parser.add_argument("--touch-offset-z", type=float, default=0.0, help="Offset Z (m) relative to estimated part surface")
    parser.add_argument("--hold-s", type=float, default=0.5, help="Pause time at deepest reached pose")
    parser.add_argument("--scanpose-tol-deg", type=float, default=3.0)
    parser.add_argument("--reach-tol-mm", type=float, default=15.0)
    parser.add_argument(
        "--raw-transform-only",
        action="store_true",
        help="Disable tray-hole snap and checkerboard XY refine to verify raw image-to-base transform",
    )
    parser.add_argument("--save-path", default="logs/move_to_vision_target.jpg", help="Annotated image path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== MOVE TO VISION TARGET ===")
    print(f"Robot IP: {args.robot_ip}")
    print(f"Target stage: {args.target_stage}")

    confirm(
        "Robot da dung san o SCAN_POSE, workspace clear, va ban cho phep robot di chuyen toi diem vision hien tai",
        args.yes,
    )

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(args.robot_ip, port=config.URSCRIPT_PORT, timeout=config.URSCRIPT_TIMEOUT)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

    try:
        rtde.connect()
        urscript.connect()
        camera.connect()
        urscript.set_tcp(config.TCP_OFFSET)
        print(f"TCP set: offset={config.TCP_OFFSET}")
        urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
        print(f"Payload set: mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")

        current_j = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_j, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            print("Loi: robot chua dung dung SCAN_POSE. Hay dua robot ve SCAN_POSE roi chay lai tool.")
            return 1

        rgb, depth, cam_ts = capture_valid_frames(camera)
        tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
        ts_diff_ms = abs(cam_ts - rtde_ts) * 1000.0
        print(f"TCP at capture: {[round(v, 4) for v in tcp_pose_at_capture]}")
        print(f"Frame/pose delta: {ts_diff_ms:.1f} ms")

        frame_h, frame_w = depth.shape
        intr = resolve_intrinsics_for_frame(
            frame_w,
            frame_h,
            config.CAM_FX,
            config.CAM_FY,
            config.CAM_CX,
            config.CAM_CY,
            config.CAM_CALIB_WIDTH,
            config.CAM_CALIB_HEIGHT,
        )
        fx_eff = intr["fx"]
        fy_eff = intr["fy"]
        cx_eff = intr["cx"]
        cy_eff = intr["cy"]
        print(f"Intrinsics used: fx={fx_eff:.2f}, fy={fy_eff:.2f}, cx={cx_eff:.2f}, cy={cy_eff:.2f}")
        if intr["reason"]:
            print(f"Canh bao intrinsics: {intr['reason']}")

        detections = detector.detect(rgb)
        print(f"Detections: {len(detections)}")
        if not detections:
            print("Khong phat hien phoi.")
            return 1

        frame_center_uv = (frame_w / 2.0, frame_h / 2.0)
        target = detector.select_best_target(detections, depth, frame_center_uv)
        if target is None:
            print("Co detections nhung khong co target hop le theo depth.")
            return 1

        target = detector.refine_pick_point(rgb, target, depth)
        u, v = target.pick_point
        depth_mm, depth_bbox = detector.resolve_pick_depth(depth, target)
        if depth_mm <= 0:
            print("Depth khong hop le.")
            return 1

        holes = []
        layout_match = None
        hole_ref_enabled = config.TRAY_HOLE_REF_ENABLED and not args.raw_transform_only
        tray_ref_enabled = config.TRAY_REF_ENABLED and not args.raw_transform_only

        if hole_ref_enabled:
            holes = detect_tray_holes(
                rgb,
                min_radius_px=config.TRAY_HOLE_MIN_RADIUS_PX,
                max_radius_px=config.TRAY_HOLE_MAX_RADIUS_PX,
                min_dist_px=config.TRAY_HOLE_MIN_DIST_PX,
            )
            layout_match = match_tray_layout_to_detected_holes(
                config.TRAY_LAYOUT_PATH,
                holes,
                max_reproj_error_px=config.TRAY_LAYOUT_MAX_REPROJ_ERR_PX,
                max_candidate_holes=config.TRAY_LAYOUT_MAX_CANDIDATE_HOLES,
            )
            snapped_hole = None
            if layout_match is not None:
                snapped_hole = assign_pick_to_layout_hole(
                    [u, v],
                    layout_match,
                    max_assign_dist_px=config.TRAY_LAYOUT_MAX_ASSIGN_DIST_PX,
                )
            if snapped_hole is None:
                snapped_hole = snap_pick_to_nearest_hole(
                    [u, v],
                    holes,
                    max_snap_dist_px=config.TRAY_HOLE_MAX_SNAP_DIST_PX,
                )
            if snapped_hole is not None:
                u, v = snapped_hole["center"]
                target.pick_point = [u, v]

        print(
            f"Target: label={target.label}, conf={target.confidence:.3f}, "
            f"pick=({u:.1f},{v:.1f}), depth={depth_mm:.1f} mm, "
            f"source={target.pick_source}, depth_bbox={depth_bbox}"
        )

        p_cam = pixel_to_camera_3d(u, v, depth_mm, fx_eff, fy_eff, cx_eff, cy_eff)
        p_base = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
        xy_source = "depth_only"
        if tray_ref_enabled:
            p_base, xy_source = refine_base_xy_with_checkerboard(
                rgb,
                u,
                v,
                p_base,
                tcp_pose_at_capture,
                config.T_CAM_TO_TCP,
                fx_eff,
                fy_eff,
                cx_eff,
                cy_eff,
                config.TRAY_REF_INNER_CORNERS,
                config.TRAY_REF_SQUARE_SIZE_M,
            )
        p_base = [
            p_base[0] + config.PICK_OFFSET_X,
            p_base[1] + config.PICK_OFFSET_Y,
            p_base[2] + config.PICK_OFFSET_Z,
        ]
        print(f"p_cam(m): {[round(vv, 4) for vv in p_cam]}")
        print(f"p_base(m): {[round(vv, 4) for vv in p_base]}  xy_source={xy_source}")

        save_path = Path(args.save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        overlay = draw_overlay(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), target, holes, layout_match)
        cv2.imwrite(str(save_path), overlay)
        print(f"Saved annotated image: {save_path}")

        current_scan_z = tcp_pose_at_capture[2]
        approach_z, touch_z, retreat_z = clamp_pick_z_sequence(
            current_scan_z,
            p_base[2],
            config.PICK_APPROACH_OFFSET_Z,
            args.touch_offset_z,
            config.PICK_RETREAT_OFFSET_Z,
        )
        pre_approach_pose = build_lateral_pre_approach_pose(
            p_base,
            tcp_pose_at_capture,
            config.PICK_APPROACH_OFFSET_Z,
            tool_rx=config.TOOL_DOWN_RX,
            tool_ry=config.TOOL_DOWN_RY,
            tool_rz=config.TOOL_DOWN_RZ,
        )
        approach_pose = [p_base[0], p_base[1], approach_z, config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ]
        touch_pose = [p_base[0], p_base[1], touch_z, config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ]
        retreat_pose = [p_base[0], p_base[1], retreat_z, config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ]

        if touch_pose[2] >= current_scan_z - 0.005:
            raise RuntimeError(
                "Touch pose khong nam THAP hon SCAN_POSE. "
                f"scan_z={current_scan_z:.4f}m, touch_z={touch_pose[2]:.4f}m."
            )
        if not (pre_approach_pose[2] >= approach_pose[2] >= touch_pose[2]):
            raise RuntimeError(
                "Thu tu Z cua pre_approach/approach/touch khong hop le. "
                f"pre={pre_approach_pose[2]:.4f}, approach={approach_pose[2]:.4f}, touch={touch_pose[2]:.4f}"
            )

        dx = pre_approach_pose[0] - tcp_pose_at_capture[0]
        dy = pre_approach_pose[1] - tcp_pose_at_capture[1]
        dz = pre_approach_pose[2] - tcp_pose_at_capture[2]
        planar_dist = (dx * dx + dy * dy) ** 0.5
        if planar_dist > config.MAX_TARGET_PLANAR_DIST_M or abs(dz) > config.MAX_TARGET_DZ_DIST_M:
            raise RuntimeError(
                "Target pose lech qua xa so voi TCP tai luc capture. "
                f"planar={planar_dist:.3f}m, dz={dz:.3f}m, "
                f"limits=({config.MAX_TARGET_PLANAR_DIST_M:.3f}m, {config.MAX_TARGET_DZ_DIST_M:.3f}m)."
            )

        print(f"Pre-approach pose: {[round(v, 4) for v in pre_approach_pose]}")
        print(f"Approach pose:     {[round(v, 4) for v in approach_pose]}")
        print(f"Touch pose:        {[round(v, 4) for v in touch_pose]}")

        urscript.move_linear(pre_approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        require_steady(rtde, "pre_approach")
        actual_pre, _ = rtde.get_tcp_pose_with_timestamp()
        print_pose_delta("Pre-approach", actual_pre, pre_approach_pose)
        assert_pose_reached("Pre-approach", actual_pre, pre_approach_pose, tolerance_mm=args.reach_tol_mm)

        if args.target_stage in ("approach", "touch"):
            urscript.move_linear(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            require_steady(rtde, "approach")
            actual_approach, _ = rtde.get_tcp_pose_with_timestamp()
            print_pose_delta("Approach", actual_approach, approach_pose)
            assert_pose_reached("Approach", actual_approach, approach_pose, tolerance_mm=args.reach_tol_mm)

        if args.target_stage == "touch":
            urscript.move_linear(touch_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
            require_steady(rtde, "touch")
            actual_touch, _ = rtde.get_tcp_pose_with_timestamp()
            print_pose_delta("Touch", actual_touch, touch_pose)
            assert_pose_reached("Touch", actual_touch, touch_pose, tolerance_mm=args.reach_tol_mm)

        if args.hold_s > 0:
            time.sleep(args.hold_s)

        if args.target_stage == "touch":
            urscript.move_linear(retreat_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            require_steady(rtde, "retreat")
        if args.target_stage in ("approach", "touch"):
            urscript.move_linear(pre_approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            require_steady(rtde, "retreat_pre_approach")
        urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
        require_steady(rtde, "return_scanpose")

        print("Hoan tat: robot da di den diem vision va quay lai SCAN_POSE.")
        return 0

    except Exception as exc:
        print(f"Loi khi chay tool: {exc}")
        try:
            if urscript.socket is not None and rtde.client is not None:
                urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                wait_steady(rtde, "return_scanpose_on_error")
        except Exception:
            pass
        return 1
    finally:
        try:
            camera.disconnect()
        except Exception:
            pass
        try:
            urscript.disconnect()
        except Exception:
            pass
        try:
            rtde.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
