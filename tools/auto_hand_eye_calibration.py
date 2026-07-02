"""Automatic hand-eye calibration using a checkerboard and UR motion.

This tool starts from the robot pose taught manually by the operator,
then moves through a safe set of nearby poses, captures checkerboard
observations automatically, and solves T_cam_to_tcp.

Safety notes:
- Ensure the checkerboard is rigidly fixed in the workspace.
- Ensure the robot path around the manually taught start pose is fully collision-free.
- Stand beside the E-stop during the whole run.
"""

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.dashboard_client import DashboardClient
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from tools.hand_eye_calibration import (
    CHECKERBOARD_INNER_CORNERS,
    OrbbecColorCamera,
    build_checkerboard_object_points,
    compute_hand_eye,
)


@dataclass
class CalibrationSample:
    pose_index: int
    target_pose: List[float]
    actual_pose: List[float]
    rvec: np.ndarray
    tvec: np.ndarray
    R_gripper2base: np.ndarray
    t_gripper2base: np.ndarray
    R_target2cam: np.ndarray
    t_target2cam: np.ndarray
    reproj_error_px: float
    board_area_ratio: float
    center_offset_ratio: float
    quality_score: float
    image_path: str = ""


@dataclass
class MethodScore:
    name: str
    T_cam_to_tcp: np.ndarray
    mean_error_mm: float
    max_error_mm: float


def confirm(message: str, force: bool) -> None:
    if force:
        return
    answer = input(f"\n[XAC NHAN] {message}\nTiep tuc? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        raise SystemExit(0)


def fmt_pose(pose: Sequence[float]) -> List[float]:
    return [round(float(v), 4) for v in pose]


def max_tcp_error_mm(current: Sequence[float], target: Sequence[float]) -> float:
    return max(abs((float(current[i]) - float(target[i])) * 1000.0) for i in range(3))


def max_rot_error_deg(current: Sequence[float], target: Sequence[float]) -> float:
    return max(abs(math.degrees(float(current[i]) - float(target[i]))) for i in range(3, 6))


def wait_steady(rtde: RTDEClient, label: str, timeout_s: Optional[float] = None) -> bool:
    timeout_s = timeout_s if timeout_s is not None else config.RTDE_WAIT_TIMEOUT
    ok = rtde.wait_steady(
        timeout_s=timeout_s,
        threshold=config.RTDE_STEADY_THRESHOLD,
        motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
    )
    if ok:
        print(f"  Robot da dung han tai {label}")
    else:
        print(f"  Timeout waiting for {label}")
    return ok


def require_steady_or_exit(rtde: RTDEClient, label: str, timeout_s: float) -> None:
    if not wait_steady(rtde, label=label, timeout_s=timeout_s):
        raise SystemExit(f"Robot khong dung on dinh tai {label}. Huy de tranh lay mau sai.")


def prepare_robot_for_motion(dashboard: DashboardClient) -> None:
    status = dashboard.precheck_ready()
    print(
        "Dashboard status:"
        f" mode={status['robotmode']},"
        f" safety={status['safetystatus']},"
        f" program={status['program_state']}"
    )
    dashboard.prepare_to_run()
    time.sleep(1.5)


def build_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def generate_pose_sequence(
    base_pose: Sequence[float],
    xy_step: float,
    z_step: float,
    rot_step_rad: float,
) -> List[List[float]]:
    """Generate a diverse-but-conservative set of calibration poses around base pose."""
    bx, by, bz, brx, bry, brz = [float(v) for v in base_pose]
    half_xy = xy_step * 0.5
    diag_xy = xy_step * 0.7
    half_z = z_step * 0.5
    half_rot = rot_step_rad * 0.6
    diag_rot = rot_step_rad * 0.85
    offsets = [
        # Baseline
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        # Pure translation (keep only a few so we save budget for richer rotations)
        (xy_step, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, xy_step, 0.0, 0.0, 0.0, 0.0),
        (0.0, -xy_step, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, z_step, 0.0, 0.0, 0.0),
        (0.0, 0.0, -z_step, 0.0, 0.0, 0.0),
        # Pure orientation
        (0.0, 0.0, 0.0, rot_step_rad, 0.0, 0.0),
        (0.0, 0.0, 0.0, -rot_step_rad, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, rot_step_rad, 0.0),
        (0.0, 0.0, 0.0, 0.0, -rot_step_rad, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, rot_step_rad),
        (0.0, 0.0, 0.0, 0.0, 0.0, -rot_step_rad),
        # Coupled translation + multi-axis tilt
        (diag_xy, half_xy, 0.0, diag_rot, half_rot, 0.0),
        (-diag_xy, -half_xy, 0.0, -diag_rot, -half_rot, 0.0),
        (half_xy, diag_xy, 0.0, -half_rot, diag_rot, 0.0),
        (-half_xy, -diag_xy, 0.0, half_rot, -diag_rot, 0.0),
        # Coupled z + tilt to change viewing angle more strongly
        (0.0, 0.0, half_z, half_rot, half_rot, half_rot),
        (0.0, 0.0, -half_z, -half_rot, -half_rot, -half_rot),
    ]
    poses = []
    for dx, dy, dz, drx, dry, drz in offsets:
        poses.append([bx + dx, by + dy, bz + dz, brx + drx, bry + dry, brz + drz])
    return poses


def pose_is_distinct(current_pose: Sequence[float], previous_pose: Optional[Sequence[float]]) -> bool:
    if previous_pose is None:
        return True
    pos_delta_mm = np.linalg.norm(
        np.array(current_pose[:3], dtype=np.float64) - np.array(previous_pose[:3], dtype=np.float64)
    ) * 1000.0
    rot_delta_deg = np.linalg.norm(
        np.degrees(
            np.array(current_pose[3:6], dtype=np.float64) - np.array(previous_pose[3:6], dtype=np.float64)
        )
    )
    return bool(pos_delta_mm >= 3.0 or rot_delta_deg >= 2.0)


def compute_sample_quality(
    obj_points: np.ndarray,
    corners_refined: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    frame_shape: Tuple[int, int],
) -> Dict[str, float]:
    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, dist)
    projected = projected.reshape(-1, 2)
    observed = corners_refined.reshape(-1, 2)
    reproj_error_px = float(np.mean(np.linalg.norm(projected - observed, axis=1)))

    h, w = frame_shape
    min_xy = observed.min(axis=0)
    max_xy = observed.max(axis=0)
    board_area_px = max(float((max_xy[0] - min_xy[0]) * (max_xy[1] - min_xy[1])), 0.0)
    board_area_ratio = board_area_px / float(w * h)

    board_center = observed.mean(axis=0)
    frame_center = np.array([w / 2.0, h / 2.0], dtype=np.float64)
    center_offset_px = float(np.linalg.norm(board_center - frame_center))
    frame_diag = float(np.hypot(w, h))
    center_offset_ratio = center_offset_px / max(frame_diag, 1.0)

    quality_score = (
        board_area_ratio * 1000.0
        - reproj_error_px * 12.0
        - center_offset_ratio * 25.0
    )
    return {
        "reproj_error_px": reproj_error_px,
        "board_area_ratio": board_area_ratio,
        "center_offset_ratio": center_offset_ratio,
        "quality_score": quality_score,
    }


def compute_consistency_stats(T_cam_to_tcp: np.ndarray, data) -> Tuple[float, float]:
    R_g2b, t_g2b, _, t_t2c = data
    positions_in_base = []
    for i in range(len(R_g2b)):
        T_base_tcp = build_transform(R_g2b[i], t_g2b[i].flatten())
        p_cam = np.append(t_t2c[i].flatten(), 1.0)
        p_tcp = T_cam_to_tcp @ p_cam
        p_base = T_base_tcp @ p_tcp
        positions_in_base.append(p_base[:3])

    positions = np.array(positions_in_base, dtype=np.float64)
    mean_pos = positions.mean(axis=0)
    errors_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000.0
    return float(np.mean(errors_mm)), float(np.max(errors_mm))


def is_identity_transform(T: np.ndarray, tol: float = 1e-6) -> bool:
    return np.allclose(T, np.eye(4), atol=tol)


def rank_method_results(results: Dict[str, np.ndarray], data) -> List[MethodScore]:
    scored: List[MethodScore] = []
    for name, T in results.items():
        if is_identity_transform(T):
            continue
        mean_mm, max_mm = compute_consistency_stats(T, data)
        scored.append(
            MethodScore(
                name=name,
                T_cam_to_tcp=np.asarray(T, dtype=np.float64),
                mean_error_mm=mean_mm,
                max_error_mm=max_mm,
            )
        )
    scored.sort(key=lambda item: (item.mean_error_mm, item.max_error_mm))
    return scored


def save_best_result_by_consistency(results: Dict[str, np.ndarray], data) -> Tuple[str, np.ndarray]:
    ranked = rank_method_results(results, data)
    if ranked:
        best = ranked[0]
    else:
        fallback_name = list(results.keys())[0]
        best = MethodScore(
            name=fallback_name,
            T_cam_to_tcp=np.asarray(results[fallback_name], dtype=np.float64),
            mean_error_mm=float("inf"),
            max_error_mm=float("inf"),
        )
        print("\n[WARN] Tat ca methods suy bien, tam luu method dau tien.")

    output = {
        "method": best.name,
        "T_cam_to_tcp": best.T_cam_to_tcp.tolist(),
        "note": "4x4 homogeneous transform, units: meters",
    }
    with (ROOT / "hand_eye_result.json").open("w", encoding="utf-8") as file_obj:
        json.dump(output, file_obj, indent=2)

    print("\n=== Method Ranking (by consistency) ===")
    if ranked:
        for item in ranked:
            print(
                f"  {item.name}: mean={item.mean_error_mm:.1f}mm, "
                f"max={item.max_error_mm:.1f}mm"
            )

    print("\nDa luu vao hand_eye_result.json")
    print(f"T_CAM_TO_TCP (method={best.name}):")
    print(best.T_cam_to_tcp)
    print("\n--- Copy doan nay vao config.py ---")
    print("T_CAM_TO_TCP = [")
    for row in best.T_cam_to_tcp.tolist():
        print(f"    {[round(value, 6) for value in row]},")
    print("]")
    print(
        f"\nConsistency error (marker in base): mean={best.mean_error_mm:.1f}mm, "
        f"max={best.max_error_mm:.1f}mm"
    )
    print("Tot neu mean < 5mm, chap nhan duoc neu < 10mm")
    return best.name, best.T_cam_to_tcp


def select_best_samples(samples: List[CalibrationSample], min_keep: int, max_keep: int) -> List[CalibrationSample]:
    if len(samples) <= min_keep:
        return samples

    reproj_values = np.array([s.reproj_error_px for s in samples], dtype=np.float64)
    median_reproj = float(np.median(reproj_values))
    mad_reproj = float(np.median(np.abs(reproj_values - median_reproj)))
    reproj_limit = median_reproj + max(0.35, 2.5 * mad_reproj)

    area_values = np.array([s.board_area_ratio for s in samples], dtype=np.float64)
    median_area = float(np.median(area_values))
    area_floor = max(0.0015, median_area * 0.55)

    filtered = [
        s for s in samples
        if s.reproj_error_px <= reproj_limit and s.board_area_ratio >= area_floor
    ]
    if len(filtered) < min_keep:
        filtered = sorted(samples, key=lambda s: s.quality_score, reverse=True)[:min_keep]
    else:
        filtered = sorted(filtered, key=lambda s: s.quality_score, reverse=True)

    max_keep = max(min_keep, max_keep)
    return filtered[: min(max_keep, len(filtered))]


def save_sample_log(
    samples: List[CalibrationSample],
    selected_pose_indices: List[int],
    path: Path,
) -> None:
    payload = {
        "selected_pose_indices": selected_pose_indices,
        "sample_count": len(samples),
        "samples": [
            {
                "pose_index": s.pose_index,
                "selected": s.pose_index in selected_pose_indices,
                "target_pose": [round(float(v), 6) for v in s.target_pose],
                "actual_pose": [round(float(v), 6) for v in s.actual_pose],
                "marker_t_cam_m": [round(float(v), 6) for v in s.t_target2cam.tolist()],
                "tcp_t_base_m": [round(float(v), 6) for v in s.t_gripper2base.tolist()],
                "reproj_error_px": round(float(s.reproj_error_px), 4),
                "board_area_ratio": round(float(s.board_area_ratio), 6),
                "center_offset_ratio": round(float(s.center_offset_ratio), 6),
                "quality_score": round(float(s.quality_score), 4),
                "image_path": s.image_path,
            }
            for s in samples
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def capture_checkerboard_sample(
    camera: OrbbecColorCamera,
    obj_points: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    max_attempts: int,
    save_dir: Optional[Path],
    sample_idx: int,
) -> Optional[Tuple[np.ndarray, np.ndarray, Dict[str, float], str]]:
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for attempt in range(1, max_attempts + 1):
        frame = camera.read()
        if frame is None:
            print(f"    Attempt {attempt}/{max_attempts}: no frame")
            time.sleep(0.15)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD_INNER_CORNERS,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            print(f"    Attempt {attempt}/{max_attempts}: checkerboard not found")
            time.sleep(0.15)
            continue

        corners_refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria,
        )
        ok, rvec, tvec = cv2.solvePnP(obj_points, corners_refined, K, dist)
        if not ok:
            print(f"    Attempt {attempt}/{max_attempts}: solvePnP failed")
            time.sleep(0.15)
            continue

        if save_dir is not None:
            frame_draw = frame.copy()
            cv2.drawChessboardCorners(
                frame_draw,
                CHECKERBOARD_INNER_CORNERS,
                corners_refined,
                found,
            )
            cv2.drawFrameAxes(frame_draw, K, dist, rvec, tvec, 0.03)
            out_path = save_dir / f"auto_hand_eye_pose_{sample_idx:02d}.jpg"
            cv2.imwrite(str(out_path), frame_draw)
            image_path = str(out_path)
        else:
            image_path = ""

        quality = compute_sample_quality(
            obj_points,
            corners_refined,
            rvec,
            tvec,
            K,
            dist,
            gray.shape,
        )

        return rvec, tvec, quality, image_path

    return None


def load_intrinsics() -> Tuple[np.ndarray, np.ndarray, dict]:
    with (ROOT / "camera_intrinsics.json").open("r", encoding="utf-8") as file_obj:
        intr = json.load(file_obj)

    K = np.array(
        [
            [intr["fx"], 0.0, intr["cx"]],
            [0.0, intr["fy"], intr["cy"]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist, intr


def collect_auto_calibration_data(args: argparse.Namespace):
    K, dist, intr = load_intrinsics()
    obj_points = build_checkerboard_object_points()
    save_dir = None if args.no_save_frames else (ROOT / args.save_dir)
    log_path = ROOT / args.log_path
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)

    with DashboardClient(
        args.robot_ip,
        port=config.DASHBOARD_PORT,
    ) as dashboard, RTDEClient(
        args.robot_ip,
        frequency=config.RTDE_FREQUENCY,
    ) as rtde, URScriptClient(
        args.robot_ip,
        port=config.URSCRIPT_PORT,
        timeout=config.URSCRIPT_TIMEOUT,
    ) as urscript, OrbbecColorCamera(
        width=int(intr.get("width", config.CAMERA_WIDTH)),
        height=int(intr.get("height", config.CAMERA_HEIGHT)),
        fps=int(config.CAMERA_FPS),
        transport=config.CAMERA_TRANSPORT,
        ip=config.CAMERA_IP,
        net_port=config.CAMERA_NET_PORT,
    ) as camera:
        prepare_robot_for_motion(dashboard)
        confirm(
            "Checkerboard da duoc co dinh chac chan, workspace clear. "
            "Ban da dua robot toi pose bat dau mong muon bang tay",
            args.force,
        )
        if not args.force:
            input(
                "\nDat robot tai pose bat dau bang tay/Freedrive, "
                "roi nhan Enter khi robot dung yen hoan toan..."
            )
        require_steady_or_exit(rtde, "manual_start_pose", timeout_s=args.wait_timeout)
        time.sleep(args.settle_s)
        base_pose = rtde.get_tcp_pose()
        print(f"Base pose for calibration: {fmt_pose(base_pose)}")

        target_poses = generate_pose_sequence(
            base_pose,
            xy_step=args.xy_step,
            z_step=args.z_step,
            rot_step_rad=math.radians(args.rot_step_deg),
        )
        target_poses = target_poses[: args.max_poses]

        samples: List[CalibrationSample] = []
        last_recorded_pose = None

        for idx, target_pose in enumerate(target_poses, start=1):
            print(f"\n[Pose {idx}/{len(target_poses)}] target={fmt_pose(target_pose)}")
            urscript.move_linear_with_settings(
                target_pose,
                tcp_offset=config.TCP_OFFSET,
                payload_kg=config.PAYLOAD_MASS_KG,
                payload_cog=config.PAYLOAD_COG,
                accel=args.linear_accel,
                vel=args.linear_vel,
            )
            wait_steady(rtde, f"pose_{idx}", timeout_s=args.wait_timeout)
            time.sleep(args.settle_s)

            actual_pose = rtde.get_tcp_pose()
            pos_err_mm = max_tcp_error_mm(actual_pose, target_pose)
            rot_err_deg = max_rot_error_deg(actual_pose, target_pose)
            print(f"  actual={fmt_pose(actual_pose)}")
            print(f"  target_error: pos={pos_err_mm:.1f} mm, rot={rot_err_deg:.2f} deg")

            if not pose_is_distinct(actual_pose, last_recorded_pose):
                print("  Skip: pose qua giong mau truoc, tranh duplicate.")
                continue

            sample = capture_checkerboard_sample(
                camera,
                obj_points,
                K,
                dist,
                max_attempts=args.capture_attempts,
                save_dir=save_dir,
                sample_idx=idx,
            )
            if sample is None:
                print("  Skip: khong detect duoc checkerboard o pose nay.")
                continue

            rvec, tvec, quality, image_path = sample
            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec.flatten()
            rvec_tcp = np.array(actual_pose[3:6], dtype=np.float64)
            R_gripper2base, _ = cv2.Rodrigues(rvec_tcp)
            t_gripper2base = np.array(actual_pose[0:3], dtype=np.float64)
            samples.append(
                CalibrationSample(
                    pose_index=idx,
                    target_pose=list(target_pose),
                    actual_pose=list(actual_pose),
                    rvec=rvec.copy(),
                    tvec=tvec.copy(),
                    R_gripper2base=R_gripper2base.copy(),
                    t_gripper2base=t_gripper2base.copy(),
                    R_target2cam=R_target2cam.copy(),
                    t_target2cam=t_target2cam.copy(),
                    reproj_error_px=quality["reproj_error_px"],
                    board_area_ratio=quality["board_area_ratio"],
                    center_offset_ratio=quality["center_offset_ratio"],
                    quality_score=quality["quality_score"],
                    image_path=image_path,
                )
            )
            last_recorded_pose = actual_pose

            print("  OK")
            print(f"    Marker trong cam: t={t_target2cam.round(4)}")
            print(f"    TCP trong base:   t={t_gripper2base.round(4)}")
            print(
                "    Quality:"
                f" reproj={quality['reproj_error_px']:.3f}px,"
                f" area={quality['board_area_ratio']:.4f},"
                f" center_offset={quality['center_offset_ratio']:.3f},"
                f" score={quality['quality_score']:.2f}"
            )

        print("\nTra robot ve pose bat dau...")
        urscript.move_linear_with_settings(
            base_pose,
            tcp_offset=config.TCP_OFFSET,
            payload_kg=config.PAYLOAD_MASS_KG,
            payload_cog=config.PAYLOAD_COG,
            accel=args.linear_accel,
            vel=args.linear_vel,
        )
        wait_steady(rtde, "return_start_pose", timeout_s=args.wait_timeout)

    count = len(samples)
    print(f"\nThu thap xong {count} mau hop le")
    if count < 4:
        print("Qua it mau hop le, can it nhat 4 va nen co 10-18 mau.")
        return None

    min_keep = max(4, min(args.min_keep, count))
    max_keep = max(min_keep, min(args.max_keep, count))
    selected_samples = select_best_samples(samples, min_keep=min_keep, max_keep=max_keep)
    selected_pose_indices = [sample.pose_index for sample in selected_samples]
    print(f"Chon {len(selected_samples)}/{count} mau de solve: {selected_pose_indices}")
    save_sample_log(samples, selected_pose_indices, log_path)
    print(f"Da luu sample log: {log_path}")

    return (
        [s.R_gripper2base for s in selected_samples],
        [s.t_gripper2base.reshape(3, 1) for s in selected_samples],
        [s.R_target2cam for s in selected_samples],
        [s.t_target2cam.reshape(3, 1) for s in selected_samples],
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Automatic hand-eye calibration around a manually taught start pose")
    parser.add_argument("--robot-ip", default=config.ROBOT_IP)
    parser.add_argument("--max-poses", type=int, default=18, help="So pose muc tieu toi da de thu")
    parser.add_argument("--xy-step", type=float, default=0.03, help="Buoc dich XY quanh pose bat dau (m)")
    parser.add_argument("--z-step", type=float, default=0.02, help="Buoc dich Z quanh pose bat dau (m)")
    parser.add_argument("--rot-step-deg", type=float, default=10.0, help="Buoc nghiêng orientation (deg)")
    parser.add_argument("--joint-accel", type=float, default=max(config.JOINT_ACCEL * 0.25, 0.15))
    parser.add_argument("--joint-vel", type=float, default=max(config.JOINT_VEL * 0.2, 0.10))
    parser.add_argument("--linear-accel", type=float, default=max(config.LINEAR_ACCEL * 0.35, 0.08))
    parser.add_argument("--linear-vel", type=float, default=max(config.LINEAR_VEL * 0.35, 0.03))
    parser.add_argument("--wait-timeout", type=float, default=max(config.RTDE_WAIT_TIMEOUT, 35.0))
    parser.add_argument("--settle-s", type=float, default=0.35, help="Thoi gian doi sau khi robot dung han")
    parser.add_argument("--capture-attempts", type=int, default=4, help="So lan thu chup checkerboard moi pose")
    parser.add_argument("--save-dir", default="captures/auto_hand_eye", help="Thu muc luu anh annotate")
    parser.add_argument("--log-path", default="logs/auto_hand_eye_samples.json", help="File JSON log chat luong tung pose")
    parser.add_argument("--min-keep", type=int, default=8, help="So mau toi thieu giu lai de solve sau khi loc")
    parser.add_argument("--max-keep", type=int, default=12, help="So mau toi da giu lai de solve sau khi loc")
    parser.add_argument("--no-save-frames", action="store_true", help="Khong luu anh annotate")
    parser.add_argument("--force", action="store_true", help="Bo qua xac nhan tay truoc khi chay")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("=== AUTO HAND-EYE CALIBRATION ===")
    print(f"Robot IP: {args.robot_ip}")
    print("Start pose source: manual by operator")
    print(
        "Steps: "
        f"xy_step={args.xy_step:.3f}m, z_step={args.z_step:.3f}m, rot_step={args.rot_step_deg:.1f}deg"
    )
    data = collect_auto_calibration_data(args)
    if not data:
        return 1

    results = compute_hand_eye(data)
    if not results:
        print("Khong tinh duoc hand-eye.")
        return 1

    save_best_result_by_consistency(results, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
