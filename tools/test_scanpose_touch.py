"""Standalone test: from SCAN_POSE, detect part, touch part, return SCAN_POSE.

Expected operator flow:
1) Manually park robot at SCAN_POSE first.
2) Run this script.
3) Script captures image immediately, computes pick point, touches part,
   then returns to SCAN_POSE and exits.
"""

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from vision.calibration import (
    build_lateral_pre_approach_pose,
    build_pick_approach_pose,
    camera_to_base,
    pixel_to_camera_3d,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
from vision.tray_holes import (
    assign_pick_to_layout_hole,
    detect_tray_holes,
    match_tray_layout_to_detected_holes,
    snap_pick_to_nearest_hole,
)
from vision.tray_reference import (
    detect_checkerboard_pose,
    refine_base_xy_with_checkerboard,
    refine_base_xy_with_tray_pose,
)


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Da huy test.")
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
        raise RuntimeError(
            f"Robot khong on dinh sau buoc {label}. "
            "Dung test de tranh tiep tuc motion khi robot dang protective stop hoac chua den dich."
        )


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def pose_position_error_mm(actual_pose, target_pose) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(target_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


def print_pose_delta(label: str, actual_pose, target_pose) -> None:
    delta = [actual_pose[i] - target_pose[i] for i in range(6)]
    print(
        f"{label} actual TCP: {[round(v, 4) for v in actual_pose]}"
    )
    print(
        f"{label} delta:      {[round(v, 4) for v in delta]} "
        f"(pos_err={pose_position_error_mm(actual_pose, target_pose):.1f} mm)"
    )


def assert_pose_reached(label: str, actual_pose, target_pose, tolerance_mm: float = 15.0) -> None:
    err_mm = pose_position_error_mm(actual_pose, target_pose)
    if err_mm > tolerance_mm:
        raise RuntimeError(
            f"{label} khong den duoc target pose. "
            f"Sai so vi tri = {err_mm:.1f} mm > {tolerance_mm:.1f} mm. "
            "Kha nang robot bo qua movel, pose khong reachable, hoac active TCP/feature tren robot khong dung."
        )


def assert_joint_pose_reached(
    rtde: RTDEClient,
    label: str,
    target_joints,
    tolerance_deg: float = 3.0,
) -> None:
    current_j = rtde.get_joint_positions()
    err_deg = joint_max_error_deg(current_j, target_joints)
    print(f"{label} joint error: {err_deg:.2f} deg")
    if err_deg > tolerance_deg:
        raise RuntimeError(
            f"{label} khong den duoc target joint pose. "
            f"Sai so toi da = {err_deg:.2f} deg > {tolerance_deg:.2f} deg."
        )


def clamp_pick_z_sequence(
    scan_z: float,
    point_z: float,
    approach_offset_z: float,
    touch_offset_z: float,
    retreat_offset_z: float,
    min_descent_mm: float = 5.0,
) -> Tuple[float, float, float]:
    """Clamp pick Z sequence so robot never goes above SCAN_POSE before descending."""
    min_descent_m = min_descent_mm / 1000.0
    max_working_z = scan_z - min_descent_m
    touch_z = point_z + touch_offset_z
    approach_z = min(point_z + approach_offset_z, max_working_z)
    retreat_z = min(point_z + retreat_offset_z, max_working_z)
    return approach_z, touch_z, retreat_z


def capture_valid_frames(camera: FemtoCamera, max_attempts: int = 8):
    """Capture frames and require non-empty RGB/depth data."""
    last = None
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
        last = (rgb, depth, cam_ts)
        time.sleep(0.05)

    raise RuntimeError("Khong lay duoc frame hop le (RGB/Depth deu rong)")


def capture_tray_reference_shot(
    camera: FemtoCamera,
    rtde: RTDEClient,
    fx_eff: float,
    fy_eff: float,
    cx_eff: float,
    cy_eff: float,
):
    print("\n=== SHOT 1: TRAY REFERENCE ===")
    print("Dat checkerboard len khay, dam bao camera nhin ro bang.")
    input("Nhan Enter de chup tray reference...")
    rgb_ref, _depth_ref, _cam_ts_ref = capture_valid_frames(camera)
    tcp_pose_ref, _ = rtde.get_tcp_pose_with_timestamp()
    tray_ref = detect_checkerboard_pose(
        rgb_ref,
        fx_eff,
        fy_eff,
        cx_eff,
        cy_eff,
        config.TRAY_REF_INNER_CORNERS,
        config.TRAY_REF_SQUARE_SIZE_M,
    )
    if tray_ref is None:
        raise RuntimeError("Khong detect duoc checkerboard trong shot tray reference.")

    print(
        f"Tray reference OK: centroid_uv="
        f"{[round(v, 1) for v in tray_ref['centroid_uv']]}"
    )
    return tray_ref, tcp_pose_ref


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect from SCAN_POSE, touch part, return SCAN_POSE"
    )
    parser.add_argument(
        "--robot-ip",
        default=config.ROBOT_IP,
        help="Robot IP (default from config)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip interactive safety confirmation",
    )
    parser.add_argument(
        "--touch-offset-z",
        type=float,
        default=0.0,
        help="Offset Z (m) from estimated surface for touch pose (default: 0.0)",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=0.2,
        help="Pause time in seconds at touch pose",
    )
    parser.add_argument(
        "--scanpose-tol-deg",
        type=float,
        default=3.0,
        help="Max allowed joint error (deg) from SCAN_POSE before starting vision",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== SCANPOSE TOUCH TEST ===")
    print(f"Robot IP: {args.robot_ip}")
    print("Flow: scanpose -> vision -> touch -> return scanpose -> stop")

    confirm(
        "Robot da dung san o SCAN_POSE, workspace clear, san sang test cham phoi",
        args.yes,
    )

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )

    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(
        args.robot_ip,
        port=config.URSCRIPT_PORT,
        timeout=config.URSCRIPT_TIMEOUT,
    )
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

    try:
        rtde.connect()
        urscript.connect()
        camera.connect()
        urscript.set_payload(config.PAYLOAD_MASS_KG, config.PAYLOAD_COG)
        print(f"Payload set: mass={config.PAYLOAD_MASS_KG:.3f} kg, cog={config.PAYLOAD_COG}")

        # Inform operator if robot is far from taught SCAN_POSE.
        current_j = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_j, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            print(
                "Loi: robot chua dung dung SCAN_POSE. "
                f"Tolerance={args.scanpose_tol_deg:.2f} deg, actual={err_deg:.2f} deg."
            )
            print("Hay dua robot ve SCAN_POSE roi chay lai tool.")
            return 1
        print("Xac nhan vi tri SCAN_POSE: OK")
        print(f"SCAN_POSE TCP in config: {[round(v, 4) for v in config.SCAN_POSE_TCP]}")

        preview_rgb, preview_depth, _preview_ts = capture_valid_frames(camera)
        frame_h, frame_w = preview_depth.shape
        sx = frame_w / float(config.CAM_CALIB_WIDTH)
        sy = frame_h / float(config.CAM_CALIB_HEIGHT)
        fx_eff = config.CAM_FX * sx
        fy_eff = config.CAM_FY * sy
        cx_eff = config.CAM_CX * sx
        cy_eff = config.CAM_CY * sy
        if abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3:
            print(
                "Canh bao: do phan giai stream khac baseline calibration, "
                f"auto-scale intrinsics sx={sx:.3f}, sy={sy:.3f} "
                f"(baseline={int(config.CAM_CALIB_WIDTH)}x{int(config.CAM_CALIB_HEIGHT)}, "
                f"stream={frame_w}x{frame_h})"
            )
        print(
            f"Intrinsics su dung: fx={fx_eff:.2f}, fy={fy_eff:.2f}, "
            f"cx={cx_eff:.2f}, cy={cy_eff:.2f}"
        )

        tray_ref = None
        tray_ref_tcp_pose = None
        if config.TRAY_REF_ENABLED and config.TRAY_REF_TWO_SHOT:
            tray_ref, tray_ref_tcp_pose = capture_tray_reference_shot(
                camera,
                rtde,
                fx_eff,
                fy_eff,
                cx_eff,
                cy_eff,
            )
            print("\n=== SHOT 2: PART IMAGE ===")
            print("Bo checkerboard ra, dat phoi vao khay, giu robot nguyen SCAN_POSE.")
            input("Nhan Enter de chup phoi...")

        rgb, depth, cam_ts = capture_valid_frames(camera)
        # Read TCP immediately after capture to minimize frame/pose time skew.
        tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
        scanpose_delta = [tcp_pose_at_capture[i] - config.SCAN_POSE_TCP[i] for i in range(6)]
        print(f"TCP so voi SCAN_POSE config: {[round(v, 4) for v in scanpose_delta]}")
        scanpose_tcp_err_mm = pose_position_error_mm(tcp_pose_at_capture, config.SCAN_POSE_TCP)
        if scanpose_tcp_err_mm > 25.0:
            raise RuntimeError(
                "TCP thuc te tai luc capture lech qua xa SCAN_POSE_TCP trong config. "
                f"Sai so = {scanpose_tcp_err_mm:.1f} mm > 25.0 mm. "
                "Kiem tra lai taught pose/TCP active truoc khi chay vision."
            )
        if tray_ref_tcp_pose is not None:
            tray_tcp_drift_mm = pose_position_error_mm(tcp_pose_at_capture, tray_ref_tcp_pose)
            print(f"TCP drift giua shot tray va shot phoi: {tray_tcp_drift_mm:.1f} mm")
            if tray_tcp_drift_mm > 2.0:
                raise RuntimeError(
                    "Robot da xê dich giua shot tray reference va shot phoi. "
                    f"TCP drift={tray_tcp_drift_mm:.1f} mm > 2.0 mm."
                )

        ts_diff = abs(cam_ts - rtde_ts)
        if ts_diff > 0.1:
            print(f"Canh bao: frame/pose lech {ts_diff * 1000:.0f} ms")
        else:
            print(f"Timestamp sync OK: {ts_diff * 1000:.1f} ms")

        detections = detector.detect(rgb)
        print(f"Detections: {len(detections)}")
        if not detections:
            print("Khong phat hien phoi. Se ve SCAN_POSE va ket thuc.")
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_no_detection")
            return 1

        frame_center_uv = (depth.shape[1] / 2.0, depth.shape[0] / 2.0)
        target = detector.select_best_target(detections, depth, frame_center_uv)
        if target is None:
            print("Co detections nhung khong co target hop le theo depth.")
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_no_valid_target")
            return 1

        target = detector.refine_pick_point(rgb, target, depth)
        u, v = target.pick_point
        depth_mm, depth_bbox = detector.resolve_pick_depth(depth, target)
        hole_source = None
        if config.TRAY_HOLE_REF_ENABLED:
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
                if snapped_hole is not None:
                    u, v = snapped_hole["center"]
                    hole_source = (
                        f"tray_layout_hole(id={snapped_hole['id']},"
                        f"assign={snapped_hole['assign_dist_px']:.1f}px,"
                        f"reproj={snapped_hole['reproj_error_px']:.1f}px)"
                    )
            if snapped_hole is None:
                snapped_hole = snap_pick_to_nearest_hole(
                    [u, v],
                    holes,
                    max_snap_dist_px=config.TRAY_HOLE_MAX_SNAP_DIST_PX,
                )
                if snapped_hole is not None:
                    u, v = snapped_hole["center"]
                    hole_source = (
                        f"tray_hole_snap(r={snapped_hole['radius_px']:.1f}px,"
                        f"dist={snapped_hole['snap_dist_px']:.1f}px)"
                    )
        print(
            f"Target: label={target.label}, conf={target.confidence:.3f}, "
            f"pick=({u:.1f},{v:.1f}), depth={depth_mm:.1f} mm, "
            f"source={target.pick_source}, depth_bbox={depth_bbox}"
        )
        if hole_source is not None:
            print(f"Hole source={hole_source}")
        if depth_mm <= 0:
            print("Depth khong hop le. Se ve SCAN_POSE va ket thuc.")
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            wait_steady(rtde, "return_scanpose_invalid_depth")
            return 1

        p_cam = pixel_to_camera_3d(
            u,
            v,
            depth_mm,
            fx_eff,
            fy_eff,
            cx_eff,
            cy_eff,
        )
        p_base_depth_only = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
        p_base = list(p_base_depth_only)
        xy_source = "depth_only"
        if tray_ref is not None:
            p_base, xy_source = refine_base_xy_with_tray_pose(
                u,
                v,
                p_base,
                tcp_pose_at_capture,
                config.T_CAM_TO_TCP,
                fx_eff,
                fy_eff,
                cx_eff,
                cy_eff,
                tray_ref,
            )
            xy_source = f"{xy_source}_two_shot"
        elif config.TRAY_REF_ENABLED:
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
        print(
            f"p_cam(m)={ [round(vv, 4) for vv in p_cam] }, "
            f"p_base(m)={ [round(vv, 4) for vv in p_base] }"
        )
        print(f"p_base_depth_only(m)={ [round(vv, 4) for vv in p_base_depth_only] }")
        delta_xy_refine_mm = [
            round((p_base[i] - p_base_depth_only[i]) * 1000.0, 1) for i in range(3)
        ]
        print(f"delta_refine_vs_depth_only(mm)={delta_xy_refine_mm}")
        print(
            "Pick offset base(m)="
            f"{[round(config.PICK_OFFSET_X, 4), round(config.PICK_OFFSET_Y, 4), round(config.PICK_OFFSET_Z, 4)]}"
        )
        print(f"XY source={xy_source}")
        camera_origin_in_base = camera_to_base([0.0, 0.0, 0.0], tcp_pose_at_capture, config.T_CAM_TO_TCP)
        print(
            f"camera_origin_base(m)={ [round(vv, 4) for vv in camera_origin_in_base] }, "
            f"cam_to_target_dist={np.linalg.norm(np.array(p_cam)) * 1000.0:.1f} mm"
        )

        current_scan_z = tcp_pose_at_capture[2]
        approach_z, touch_z, retreat_z = clamp_pick_z_sequence(
            current_scan_z,
            p_base[2],
            config.PICK_APPROACH_OFFSET_Z,
            args.touch_offset_z,
            config.PICK_RETREAT_OFFSET_Z,
        )

        approach_pose = [
            p_base[0],
            p_base[1],
            approach_z,
            config.TOOL_DOWN_RX,
            config.TOOL_DOWN_RY,
            config.TOOL_DOWN_RZ,
        ]
        touch_pose = [
            p_base[0],
            p_base[1],
            touch_z,
            config.TOOL_DOWN_RX,
            config.TOOL_DOWN_RY,
            config.TOOL_DOWN_RZ,
        ]
        retreat_pose = [
            p_base[0],
            p_base[1],
            retreat_z,
            config.TOOL_DOWN_RX,
            config.TOOL_DOWN_RY,
            config.TOOL_DOWN_RZ,
        ]
        guard_pre_approach_pose = build_lateral_pre_approach_pose(
            p_base,
            tcp_pose_at_capture,
            config.PICK_APPROACH_OFFSET_Z,
            tool_rx=config.TOOL_DOWN_RX,
            tool_ry=config.TOOL_DOWN_RY,
            tool_rz=config.TOOL_DOWN_RZ,
        )
        pre_approach_pose = guard_pre_approach_pose

        if touch_pose[2] >= current_scan_z - 0.005:
            raise RuntimeError(
                "Touch pose khong nam THAP hon SCAN_POSE. "
                f"scan_z={current_scan_z:.4f}m, touch_z={touch_pose[2]:.4f}m. "
                "Dung test vi target dang bi tinh sai huong (co nguy co robot chay nguoc len tren)."
            )
        if not (pre_approach_pose[2] >= approach_pose[2] >= touch_pose[2]):
            raise RuntimeError(
                "Thu tu Z cua pre_approach/approach/touch khong hop le. "
                f"pre={pre_approach_pose[2]:.4f}, approach={approach_pose[2]:.4f}, touch={touch_pose[2]:.4f}"
            )

        # Guard rail: skip motion if computed target is too far from current TCP.
        dx = guard_pre_approach_pose[0] - tcp_pose_at_capture[0]
        dy = guard_pre_approach_pose[1] - tcp_pose_at_capture[1]
        dz = guard_pre_approach_pose[2] - tcp_pose_at_capture[2]
        planar_dist = (dx * dx + dy * dy) ** 0.5
        if planar_dist > config.MAX_TARGET_PLANAR_DIST_M or abs(dz) > config.MAX_TARGET_DZ_DIST_M:
            print(
                "Loi: target pose lech qua xa so voi TCP tai luc capture. "
                f"planar={planar_dist:.3f}m, dz={dz:.3f}m, "
                f"limits=({config.MAX_TARGET_PLANAR_DIST_M:.3f}m, {config.MAX_TARGET_DZ_DIST_M:.3f}m). "
                "Dung de tranh va cham."
            )
            urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            require_steady(rtde, "return_scanpose_outlier_target")
            return 1

        print(f"Pre-approach:  {pre_approach_pose}")
        print(f"Approach pose: {approach_pose}")
        print(f"Touch pose:    {touch_pose}")

        urscript.move_linear(pre_approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        require_steady(rtde, "pre_approach")
        actual_pre_approach_pose, _ = rtde.get_tcp_pose_with_timestamp()
        print_pose_delta("Pre-approach", actual_pre_approach_pose, pre_approach_pose)
        assert_pose_reached("Pre-approach", actual_pre_approach_pose, pre_approach_pose)

        urscript.move_linear(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        require_steady(rtde, "approach")
        actual_approach_pose, _ = rtde.get_tcp_pose_with_timestamp()
        print_pose_delta("Approach", actual_approach_pose, approach_pose)
        assert_pose_reached("Approach", actual_approach_pose, approach_pose)

        # Move slowly when touching the part surface.
        urscript.move_linear(touch_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
        require_steady(rtde, "touch")
        actual_touch_pose, _ = rtde.get_tcp_pose_with_timestamp()
        print_pose_delta("Touch", actual_touch_pose, touch_pose)
        assert_pose_reached("Touch", actual_touch_pose, touch_pose)

        if args.hold_s > 0:
            time.sleep(args.hold_s)

        urscript.move_linear(retreat_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        require_steady(rtde, "retreat")
        urscript.move_linear(pre_approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
        require_steady(rtde, "retreat_pre_approach")

        urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
        require_steady(rtde, "return_scanpose")

        print("Hoan tat test: robot da den touch pose va quay lai SCAN_POSE.")
        return 0

    except Exception as exc:
        print(f"Loi khi chay test: {exc}")
        # Best-effort return to SCAN_POSE before exit.
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
            rtde.disconnect()
        except Exception:
            pass
        try:
            urscript.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
