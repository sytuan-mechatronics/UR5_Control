"""Visualize SCAN_POSE detection result with image overlay and target coordinates.

Operator flow:
1) Manually park robot at SCAN_POSE.
2) Run this tool.
3) The tool captures one frame, detects the target, shows RGB window with overlays,
   and prints pixel/depth/p_cam/p_base plus planned poses to terminal.
4) Press q/ESC to close, or s to save the annotated frame.
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
from vision.calibration import build_lateral_pre_approach_pose, camera_to_base, pixel_to_camera_3d
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
from vision.tray_holes import (
    assign_pick_to_layout_hole,
    detect_tray_holes,
    match_tray_layout_to_detected_holes,
    snap_pick_to_nearest_hole,
)
from vision.tray_reference import detect_checkerboard_pose, refine_base_xy_with_checkerboard


WINDOW_NAME = "SCAN_POSE Target View"


def joint_max_error_deg(current, target) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def pose_position_error_mm(actual_pose, target_pose) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(target_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize target detection and coordinates at SCAN_POSE")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP")
    parser.add_argument("--scanpose-tol-deg", type=float, default=3.0, help="Max allowed joint error from SCAN_POSE")
    parser.add_argument("--touch-offset-z", type=float, default=0.0, help="Touch offset relative to estimated surface (m)")
    parser.add_argument("--save-dir", default="captures", help="Directory for saved annotated image")
    return parser.parse_args()


def draw_overlay(image_bgr, detections, target, depth_mm, tcp_pose, p_cam, p_base, pre_approach_pose, approach_z, touch_z, tray_ref=None, xy_source="depth_only", holes=None, snapped_hole=None, layout_match=None):
    frame = image_bgr.copy()
    h, w = frame.shape[:2]
    cx_frame, cy_frame = int(w / 2), int(h / 2)
    cv2.drawMarker(frame, (cx_frame, cy_frame), (255, 255, 0), cv2.MARKER_CROSS, 24, 2)

    for det in detections:
        color = (0, 180, 0)
        thickness = 2
        if target is det:
            color = (0, 0, 255)
            thickness = 3
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        label = f"{det.label} {det.confidence:.2f}"
        cv2.putText(frame, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

    if target is not None:
        bbox_u, bbox_v = int(round(target.center[0])), int(round(target.center[1]))
        u, v = int(round(target.pick_point[0])), int(round(target.pick_point[1]))
        cv2.circle(frame, (bbox_u, bbox_v), 5, (255, 0, 255), 2)
        cv2.circle(frame, (u, v), 6, (0, 0, 255), -1)
        cv2.line(frame, (cx_frame, cy_frame), (u, v), (0, 255, 255), 2)
        px1, py1, px2, py2 = [int(vv) for vv in target.pick_bbox]
        cv2.rectangle(frame, (px1, py1), (px2, py2), (0, 255, 255), 2)
        if holes:
            for hole in holes:
                hu, hv = [int(round(vv)) for vv in hole["center"]]
                hr = int(round(hole["radius_px"]))
                cv2.circle(frame, (hu, hv), hr, (0, 200, 255), 2)
        if layout_match is not None:
            for idx, hole_uv in enumerate(layout_match["projected_holes_uv"], start=1):
                hu, hv = [int(round(vv)) for vv in hole_uv]
                cv2.circle(frame, (hu, hv), 10, (255, 120, 0), 2)
                cv2.putText(frame, str(idx), (hu + 8, hv - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 120, 0), 2, cv2.LINE_AA)
        if snapped_hole is not None:
            hu, hv = [int(round(vv)) for vv in snapped_hole["center"]]
            cv2.drawMarker(frame, (hu, hv), (0, 200, 255), cv2.MARKER_CROSS, 20, 2)
        if tray_ref is not None:
            board_cx, board_cy = [int(round(vv)) for vv in tray_ref["centroid_uv"]]
            cv2.circle(frame, (board_cx, board_cy), 8, (255, 165, 0), 2)
            for corner in tray_ref["corners"]:
                cu, cv = int(round(corner[0])), int(round(corner[1]))
                cv2.circle(frame, (cu, cv), 2, (255, 165, 0), -1)

        text_lines = [
            f"bbox_center=({target.center[0]:.1f}, {target.center[1]:.1f}) pick=({target.pick_point[0]:.1f}, {target.pick_point[1]:.1f})",
            f"pick_source={target.pick_source} depth={depth_mm:.1f}mm",
            f"xy_source={xy_source}",
            f"p_cam=({p_cam[0]:.4f}, {p_cam[1]:.4f}, {p_cam[2]:.4f}) m",
            f"p_base=({p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}) m",
            f"pick_offset=({config.PICK_OFFSET_X:.4f}, {config.PICK_OFFSET_Y:.4f}, {config.PICK_OFFSET_Z:.4f}) m",
            f"scan_tcp=({tcp_pose[0]:.4f}, {tcp_pose[1]:.4f}, {tcp_pose[2]:.4f}) m",
            f"pre_z={pre_approach_pose[2]:.4f} approach_z={approach_z:.4f} touch_z={touch_z:.4f}",
        ]
        y = 28
        for line in text_lines:
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
            y += 24

    help_text = "q/ESC: thoat | s: luu anh"
    cv2.putText(frame, help_text, (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return frame


def main() -> int:
    args = parse_args()

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

    try:
        rtde.connect()
        camera.connect()

        current_j = rtde.get_joint_positions()
        err_deg = joint_max_error_deg(current_j, config.SCAN_POSE_JOINTS)
        print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
        if err_deg > args.scanpose_tol_deg:
            raise RuntimeError(
                f"Robot chua dung dung SCAN_POSE. tolerance={args.scanpose_tol_deg:.2f} deg, actual={err_deg:.2f} deg"
            )

        rgb, depth, cam_ts = capture_valid_frames(camera)
        tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
        print(f"TCP so voi SCAN_POSE config: {[round(tcp_pose_at_capture[i] - config.SCAN_POSE_TCP[i], 4) for i in range(6)]}")
        print(f"TCP vs SCAN_POSE pos err: {pose_position_error_mm(tcp_pose_at_capture, config.SCAN_POSE_TCP):.1f} mm")

        frame_h, frame_w = depth.shape
        sx = frame_w / float(config.CAM_CALIB_WIDTH)
        sy = frame_h / float(config.CAM_CALIB_HEIGHT)
        fx_eff = config.CAM_FX * sx
        fy_eff = config.CAM_FY * sy
        cx_eff = config.CAM_CX * sx
        cy_eff = config.CAM_CY * sy
        print(f"Timestamp delta: {abs(cam_ts - rtde_ts) * 1000.0:.1f} ms")
        print(f"Intrinsics su dung: fx={fx_eff:.2f}, fy={fy_eff:.2f}, cx={cx_eff:.2f}, cy={cy_eff:.2f}")

        detections = detector.detect(rgb)
        print(f"Detections: {len(detections)}")
        frame_center_uv = (depth.shape[1] / 2.0, depth.shape[0] / 2.0)
        target = detector.select_best_target(detections, depth, frame_center_uv)

        image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if target is None:
            annotated = draw_overlay(image_bgr, detections, None, 0.0, tcp_pose_at_capture, [0, 0, 0], [0, 0, 0], tcp_pose_at_capture, tcp_pose_at_capture[2], tcp_pose_at_capture[2])
            print("Khong co target hop le theo depth.")
        else:
            target = detector.refine_pick_point(rgb, target, depth)
            u, v = target.pick_point
            depth_mm, depth_bbox = detector.resolve_pick_depth(depth, target)
            p_cam = pixel_to_camera_3d(u, v, depth_mm, fx_eff, fy_eff, cx_eff, cy_eff)
            holes = []
            snapped_hole = None
            layout_match = None
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
                    p_cam = pixel_to_camera_3d(u, v, depth_mm, fx_eff, fy_eff, cx_eff, cy_eff)
            p_base = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
            tray_ref = None
            xy_source = "depth_only"
            if config.TRAY_REF_ENABLED:
                tray_ref = detect_checkerboard_pose(
                    rgb,
                    fx_eff,
                    fy_eff,
                    cx_eff,
                    cy_eff,
                    config.TRAY_REF_INNER_CORNERS,
                    config.TRAY_REF_SQUARE_SIZE_M,
                )
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
            camera_origin_in_base = camera_to_base([0.0, 0.0, 0.0], tcp_pose_at_capture, config.T_CAM_TO_TCP)
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

            print(
                f"Target: label={target.label}, conf={target.confidence:.3f}, "
                f"bbox_center=({target.center[0]:.1f}, {target.center[1]:.1f}), "
                f"pick=({u:.1f}, {v:.1f}), depth={depth_mm:.1f} mm, "
                f"bbox={target.bbox}, pick_bbox={target.pick_bbox}, depth_bbox={depth_bbox}, source={target.pick_source}"
            )
            print(f"p_cam(m)={ [round(vv, 4) for vv in p_cam] }")
            print(f"camera_origin_base(m)={ [round(vv, 4) for vv in camera_origin_in_base] }")
            print(f"p_base(m)={ [round(vv, 4) for vv in p_base] }")
            print(f"pick_offset_base(m)={ [round(config.PICK_OFFSET_X, 4), round(config.PICK_OFFSET_Y, 4), round(config.PICK_OFFSET_Z, 4)] }")
            print(f"xy_source={xy_source}")
            if snapped_hole is not None:
                if "id" in snapped_hole:
                    print(
                        "hole_source=tray_layout_hole "
                        f"id={snapped_hole['id']} center={ [round(v, 1) for v in snapped_hole['center']] } "
                        f"assign_px={snapped_hole['assign_dist_px']:.1f} reproj_px={snapped_hole['reproj_error_px']:.1f}"
                    )
                else:
                    print(f"hole_source=tray_hole_snap center={ [round(v, 1) for v in snapped_hole['center']] } radius_px={snapped_hole['radius_px']:.1f} dist_px={snapped_hole['snap_dist_px']:.1f}")
            print(f"delta target - tcp in base (m)={ [round(p_base[i] - tcp_pose_at_capture[i], 4) for i in range(3)] }")
            print(f"distance tcp -> target (mm)={np.linalg.norm(np.array(p_base) - np.array(tcp_pose_at_capture[:3])) * 1000.0:.1f}")
            print(f"Pre-approach pose={ [round(vv, 4) for vv in pre_approach_pose] }")
            print(f"Approach z={approach_z:.4f}, touch z={touch_z:.4f}, retreat z={retreat_z:.4f}")

            annotated = draw_overlay(
                image_bgr,
                detections,
                target,
                depth_mm,
                tcp_pose_at_capture,
                p_cam,
                p_base,
                pre_approach_pose,
                approach_z,
                touch_z,
                tray_ref=tray_ref,
                xy_source=xy_source,
                holes=holes,
                snapped_hole=snapped_hole,
                layout_match=layout_match,
            )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.imshow(WINDOW_NAME, annotated)

        save_dir = Path(args.save_dir)
        while True:
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                save_dir.mkdir(parents=True, exist_ok=True)
                out_path = save_dir / f"scanpose_target_{int(time.time())}.png"
                cv2.imwrite(str(out_path), annotated)
                print(f"Da luu anh: {out_path}")

        cv2.destroyAllWindows()
        return 0
    finally:
        try:
            camera.disconnect()
        except Exception:
            pass
        try:
            rtde.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
