"""Phase-3 scan-only diagnostic tool.

Purpose:
- Move robot to SCAN_POSE when needed
- Capture one frame at SCAN_POSE
- Detect all visible parts
- Show which tray sample/slot each part maps to
- Print p_base_raw / p_base / correction info without moving down to pick

Use this before Phase 3 to verify:
- the captured tray sample is reasonable
- slot identification is stable
- per-slot correction is being applied as expected
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from vision.calibration import (
    _load_pick_correction_map,
    apply_pick_correction,
    camera_to_base,
    normalize_slot_name,
    pixel_to_camera_3d,
    resolve_intrinsics_for_frame,
    sanitize_camera_depth_mm,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera
from vision.single_slot_reference import load_single_slot_reference
from vision.tray_slot_reference import load_tray_slot_reference, resolve_selected_slot_for_target


WINDOW_NAME = "PHASE3 SLOT SCAN"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan-only checker for Phase 3 tray sample / slot matching")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP, help="Robot IP")
    parser.add_argument(
        "--scanpose-tol-deg",
        type=float,
        default=float(getattr(config, "TRAY_SLOT_SCANPOSE_TOL_DEG", 3.0)),
        help="Skip move if max joint error from SCAN_POSE is within this tolerance",
    )
    parser.add_argument("--yes", action="store_true", help="Skip preview confirmation")
    parser.add_argument("--no-preview", action="store_true", help="Do not open preview window")
    parser.add_argument(
        "--save-dir",
        default="captures/phase3_slot_scan",
        help="Directory to save annotated preview and JSON summary",
    )
    parser.add_argument(
        "--fail-on-duplicate-slot",
        action="store_true",
        help="Return exit code 2 if 2 detections map to the same slot",
    )
    parser.add_argument(
        "--fail-on-unmatched",
        action="store_true",
        help="Return exit code 3 if any detection cannot map to a slot",
    )
    return parser.parse_args()


def print_runtime_snapshot() -> None:
    correction_map = _load_pick_correction_map()
    tray_ref = load_tray_slot_reference()
    single_slot_ref = load_single_slot_reference()
    print(f"Runtime config file: {Path(config.__file__).resolve()}")
    print(f"PICK_CORRECTION_ENABLED: {config.PICK_CORRECTION_ENABLED}")
    print(f"PICK_CORRECTION_MAP_PATH: {config.PICK_CORRECTION_MAP_PATH}")
    print(f"PICK_CORRECTION_POINTS: {len(correction_map.get('points', []))}")
    print(f"TRAY_SLOT_REFERENCE_ENABLED: {config.TRAY_SLOT_REFERENCE_ENABLED}")
    print(f"TRAY_SLOT_REFERENCE_PATH: {config.TRAY_SLOT_REFERENCE_PATH}")
    print(f"TRAY_SLOT_REFERENCE_SAMPLES: {len(tray_ref.get('samples', []))}")
    print(f"SINGLE_SLOT_REFERENCE_ENABLED: {config.SINGLE_SLOT_REFERENCE_ENABLED}")
    print(f"SINGLE_SLOT_REFERENCE_PATH: {config.SINGLE_SLOT_REFERENCE_PATH}")
    print(f"SINGLE_SLOT_REFERENCE_SAMPLES: {len(single_slot_ref.get('samples', []))}")
    print(
        "GLOBAL_PICK_OFFSET(m): "
        f"{[round(float(config.PICK_OFFSET_X), 4), round(float(config.PICK_OFFSET_Y), 4), round(float(config.PICK_OFFSET_Z), 4)]}"
    )


def joint_max_error_deg(current: List[float], target: List[float]) -> float:
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


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


def move_robot_to_scan_pose(rtde: RTDEClient, urscript: URScriptClient) -> None:
    print("Dang dua robot ve HOME...")
    urscript.move_joint(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
    wait_steady(rtde, "move_home")

    print("Dang dua robot den SCAN_APPROACH...")
    urscript.move_joint(config.SCAN_APPROACH_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
    wait_steady(rtde, "move_scan_approach")

    print("Dang dua robot den SCAN_POSE...")
    urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
    wait_steady(rtde, "move_scan_pose")


def ensure_robot_at_scan_pose(rtde: RTDEClient, urscript: URScriptClient, scanpose_tol_deg: float) -> None:
    current_j = rtde.get_joint_positions()
    err_deg = joint_max_error_deg(current_j, config.SCAN_POSE_JOINTS)
    print(f"Lech SCAN_POSE hien tai (max joint error): {err_deg:.2f} deg")
    if err_deg <= scanpose_tol_deg:
        print("Robot da o SCAN_POSE trong tolerance. Bo qua auto move.")
        return
    print("Robot chua o SCAN_POSE. Bat dau auto move ve SCAN_POSE...")
    move_robot_to_scan_pose(rtde, urscript)


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


def pose_position_error_mm(actual_pose: List[float], target_pose: List[float]) -> float:
    delta = np.array(actual_pose[:3], dtype=np.float64) - np.array(target_pose[:3], dtype=np.float64)
    return float(np.linalg.norm(delta) * 1000.0)


def _format_short_xyz(values: List[float]) -> str:
    return "[" + ", ".join(f"{float(v):.3f}" for v in values[:3]) + "]"


def _draw_preview(rgb: np.ndarray, detections: List[object], rows: List[Dict[str, object]], sample_name: str) -> np.ndarray:
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    h, _ = image.shape[:2]
    row_by_idx = {int(row["det_index"]): row for row in rows}

    for idx, det in enumerate(detections, start=1):
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        row = row_by_idx.get(idx, {})
        slot_name = str(row.get("selected_slot") or row.get("matched_slot") or "?")
        ok = bool(slot_name and slot_name != "?")
        color = (0, 180, 0) if ok else (0, 0, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        u, v = [int(round(val)) for val in det.pick_point]
        cv2.circle(image, (u, v), 5, color, -1)
        label = f"#{idx} {slot_name} {det.confidence:.2f}"
        cv2.putText(image, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

    head = [
        f"sample={sample_name or 'n/a'}",
        f"detections={len(detections)}",
        "green=matched red=unmatched",
    ]
    y = 26
    for line in head:
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 1, cv2.LINE_AA)
        y += 24

    help_text = "y/Enter: tiep tuc | q/ESC: huy | anh da duoc luu"
    cv2.putText(image, help_text, (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def _show_preview_and_confirm(image: np.ndarray, path: Path, force_yes: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)
    print(f"Da luu preview: {path}")
    if force_yes:
        return

    cv2.imshow(WINDOW_NAME, image)
    print("Preview dang mo. Nhan y/Enter de dong, q/Esc de huy.")
    while True:
        key = cv2.waitKey(50) & 0xFF
        if key in (13, 10, ord("y")):
            break
        if key in (27, ord("q"), ord("n")):
            cv2.destroyAllWindows()
            raise SystemExit(0)
    cv2.destroyAllWindows()


def _print_table(rows: List[Dict[str, object]]) -> None:
    if not rows:
        print("Khong co dong nao de in.")
        return

    headers = [
        "idx",
        "conf",
        "pick_uv",
        "depth_mm",
        "matched_slot",
        "selected_slot",
        "sample",
        "match_err_px",
        "mode",
        "p_base_raw(x,y,z)",
        "p_base(x,y,z)",
    ]
    body = []
    for row in rows:
        body.append(
            [
                str(row["det_index"]),
                f"{float(row['confidence']):.3f}",
                f"({row['pick_u']:.1f},{row['pick_v']:.1f})",
                f"{row['safe_depth_mm']:.1f}",
                str(row.get("matched_slot") or ""),
                str(row.get("selected_slot") or ""),
                str(row.get("sample_name") or ""),
                "" if row.get("match_err_px") is None else str(row.get("match_err_px")),
                str(row.get("correction_mode") or ""),
                _format_short_xyz(row["p_base_raw_m"]),
                _format_short_xyz(row["p_base_m"]),
            ]
        )

    widths = [len(h) for h in headers]
    for item in body:
        for idx, cell in enumerate(item):
            widths[idx] = max(widths[idx], len(cell))

    def fmt_line(values: List[str]) -> str:
        return " | ".join(values[i].ljust(widths[i]) for i in range(len(values)))

    print("\n=== PHASE 3 SLOT SCAN ===")
    print(fmt_line(headers))
    print("-+-".join("-" * width for width in widths))
    for row in body:
        print(fmt_line(row))


def main() -> int:
    args = parse_args()
    print_runtime_snapshot()

    detector = Detector(
        model_path=config.YOLO_MODEL_PATH,
        confidence=config.YOLO_CONFIDENCE,
        target_class=config.YOLO_TARGET_CLASS,
    )
    rtde = RTDEClient(args.robot_ip, frequency=config.RTDE_FREQUENCY)
    urscript = URScriptClient(args.robot_ip, port=config.URSCRIPT_PORT, timeout=config.URSCRIPT_TIMEOUT)
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    try:
        rtde.connect()
        urscript.connect()
        camera.connect()

        ensure_robot_at_scan_pose(rtde, urscript, args.scanpose_tol_deg)

        rgb, depth, cam_ts = capture_valid_frames(camera)
        tcp_pose_at_capture, rtde_ts = rtde.get_tcp_pose_with_timestamp()
        print(f"TCP so voi SCAN_POSE config: {[round(tcp_pose_at_capture[i] - config.SCAN_POSE_TCP[i], 4) for i in range(6)]}")
        print(f"TCP vs SCAN_POSE pos err: {pose_position_error_mm(tcp_pose_at_capture, config.SCAN_POSE_TCP):.1f} mm")

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
        print(f"Timestamp sync: {abs(cam_ts - rtde_ts) * 1000.0:.1f} ms")
        print(f"Intrinsics su dung: fx={fx_eff:.2f}, fy={fy_eff:.2f}, cx={cx_eff:.2f}, cy={cy_eff:.2f}")
        if intr.get("reason"):
            print(f"Canh bao intrinsics: {intr['reason']}")

        detections = detector.detect(rgb)
        detections = sorted(detections, key=lambda det: (float(det.center[0]), float(det.center[1])))
        print(f"Detections: {len(detections)}")
        if not detections:
            print("Khong phat hien phoi nao.")
            return 1

        rows: List[Dict[str, object]] = []
        duplicate_slots: Dict[str, int] = {}
        unmatched_count = 0
        common_sample_name = ""

        for det_index, det in enumerate(detections, start=1):
            det = detector.refine_pick_point(rgb, det, depth)
            u, v = det.pick_point
            raw_depth_mm, depth_bbox = detector.resolve_pick_depth(depth, det)
            safe_depth_mm = raw_depth_mm
            safe_depth_mm, was_clamped, min_safe_depth_mm = sanitize_camera_depth_mm(
                safe_depth_mm,
                config.T_CAM_TO_TCP,
                margin_below_tcp_m=config.PICK_MIN_DESCENT_M,
            )
            if was_clamped:
                print(
                    f"Detection #{det_index}: depth {raw_depth_mm:.1f}mm duoc clamp len {min_safe_depth_mm:.1f}mm"
                )

            p_cam = pixel_to_camera_3d(
                u,
                v,
                safe_depth_mm,
                fx_eff,
                fy_eff,
                cx_eff,
                cy_eff,
            )
            p_base_raw = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
            matched_slot, slot_match_meta = resolve_selected_slot_for_target(detections, det)
            forced_slot = normalize_slot_name(matched_slot)
            p_base, correction_meta = apply_pick_correction(
                p_base_raw,
                forced_slot=forced_slot,
                pick_uv=[u, v],
            )

            sample_name = str(slot_match_meta.get("sample_name") or "")
            if sample_name and not common_sample_name:
                common_sample_name = sample_name

            match_assignments = slot_match_meta.get("assignments") or {}
            slot_entry = match_assignments.get(det_index - 1) or {}
            selected_slot = str(correction_meta.get("selected_slot") or correction_meta.get("forced_slot") or "")
            if selected_slot:
                duplicate_slots[selected_slot] = duplicate_slots.get(selected_slot, 0) + 1
            else:
                unmatched_count += 1

            row = {
                "det_index": det_index,
                "label": det.label,
                "confidence": round(float(det.confidence), 4),
                "bbox": [int(vv) for vv in det.bbox],
                "center_px": [round(float(det.center[0]), 2), round(float(det.center[1]), 2)],
                "pick_u": round(float(u), 2),
                "pick_v": round(float(v), 2),
                "pick_source": str(det.pick_source),
                "raw_depth_mm": round(float(raw_depth_mm), 2),
                "safe_depth_mm": round(float(safe_depth_mm), 2),
                "depth_bbox": [int(vv) for vv in depth_bbox] if depth_bbox is not None else None,
                "p_cam_m": [round(float(vv), 6) for vv in p_cam],
                "p_base_raw_m": [round(float(vv), 6) for vv in p_base_raw],
                "p_base_m": [round(float(vv), 6) for vv in p_base],
                "matched_slot": matched_slot,
                "selected_slot": selected_slot,
                "sample_name": sample_name,
                "match_reason": str(slot_match_meta.get("reason") or ""),
                "match_err_px": slot_entry.get("error_px", slot_match_meta.get("best_mean_error_px")),
                "second_best_slot": str(slot_match_meta.get("second_best_slot") or ""),
                "second_best_error_px": slot_match_meta.get("second_best_error_px"),
                "correction_mode": str(correction_meta.get("mode") or ""),
                "global_plus_local_offset_m": [round(float(vv), 6) for vv in correction_meta.get("final_offset", [0.0, 0.0, 0.0])],
                "local_offset_m": [round(float(vv), 6) for vv in correction_meta.get("local_offset", [0.0, 0.0, 0.0])],
            }
            rows.append(row)

            print(
                f"#{det_index} Target: conf={det.confidence:.3f}, "
                f"pick=({u:.1f},{v:.1f}), depth={safe_depth_mm:.1f} mm, "
                f"matched_slot={matched_slot or 'n/a'}, selected_slot={selected_slot or 'n/a'}, sample={sample_name or 'n/a'}"
            )
            print(
                f"   p_cam(m)={[round(vv, 4) for vv in p_cam]}, "
                f"p_base_raw(m)={[round(vv, 4) for vv in p_base_raw]}, "
                f"p_base(m)={[round(vv, 4) for vv in p_base]}"
            )
            print(
                f"   pick_offset_base(m)={[round(vv, 4) for vv in correction_meta.get('final_offset', [0.0, 0.0, 0.0])]}, "
                f"pick_correction_local(m)={[round(vv, 4) for vv in correction_meta.get('local_offset', [0.0, 0.0, 0.0])]} "
                f"mode={correction_meta.get('mode', 'unknown')}"
            )
            print(
                f"   match_meta: reason={slot_match_meta.get('reason')} "
                f"mean_err_px={slot_match_meta.get('best_mean_error_px')} "
                f"max_err_px={slot_match_meta.get('best_max_error_px')} "
                f"scale={slot_match_meta.get('used_scale')} rot_deg={slot_match_meta.get('rotation_deg')}"
            )
            if slot_match_meta.get("override_source"):
                single_meta = slot_match_meta.get("single_slot_meta") or {}
                print(
                    f"   override: source={slot_match_meta.get('override_source')} "
                    f"tray_slot={slot_match_meta.get('tray_slot_name', '')} "
                    f"final_slot={matched_slot or 'n/a'} "
                    f"single_sample={single_meta.get('sample_name', '')} "
                    f"single_err_px={single_meta.get('error_px')}"
                )
            if slot_match_meta.get("second_best_slot"):
                print(
                    f"   second_best={slot_match_meta.get('second_best_slot')} "
                    f"err_px={slot_match_meta.get('second_best_error_px')}"
                )

        _print_table(rows)

        duplicate_slot_names = sorted([slot for slot, count in duplicate_slots.items() if count > 1])
        status = "ok"
        if duplicate_slot_names:
            status = "duplicate_slots"
        elif unmatched_count > 0:
            status = "unmatched_slots"

        print("\n=== DANH GIA ===")
        print(f"sample_duoc_chon: {common_sample_name or 'n/a'}")
        print(f"so_phoi_detect: {len(rows)}")
        print(f"so_slot_khong_match: {unmatched_count}")
        print(f"slot_bi_trung: {duplicate_slot_names if duplicate_slot_names else 'khong'}")
        if duplicate_slot_names:
            print("Canh bao: co it nhat 2 phoi dang trung cung 1 slot. Mau anh hoac offset slot dang nham.")
        elif unmatched_count:
            print("Canh bao: co phoi chua map duoc vao slot. Can xem lai bo mau / nguong match.")
        else:
            print("Tot: moi phoi dang map vao 1 slot rieng.")

        preview = _draw_preview(rgb, detections, rows, common_sample_name)
        preview_path = save_dir / f"phase3_slot_scan_{stamp}.jpg"
        _show_preview_and_confirm(preview, preview_path, args.yes or args.no_preview)

        payload = {
            "timestamp": stamp,
            "status": status,
            "scan_pose_tcp_at_capture": [round(float(v), 6) for v in tcp_pose_at_capture],
            "sample_name": common_sample_name,
            "rows": rows,
            "duplicate_slots": duplicate_slot_names,
            "unmatched_count": unmatched_count,
        }
        json_path = save_dir / f"phase3_slot_scan_{stamp}.json"
        json_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
        print(f"Da luu JSON: {json_path}")

        if duplicate_slot_names and args.fail_on_duplicate_slot:
            return 2
        if unmatched_count > 0 and args.fail_on_unmatched:
            return 3
        return 0
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
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
