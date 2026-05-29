"""
Pick-Place Cycle Orchestrator.
Main logic for autonomous pick-place operations.
"""

import logging
import time
import traceback
import json
from typing import Dict, Optional

from robot.dashboard_client import DashboardClient
from robot.urscript_client import URScriptClient
from robot.rtde_client import RTDEClient
from robot.gripper_rg import GripperRG
from vision.femto_camera import FemtoCamera
from vision.detector import Detector
from vision.calibration import (
    pixel_to_camera_3d,
    camera_to_base,
    build_pick_approach_pose,
    build_lateral_pre_approach_pose,
)
from vision.tray_holes import (
    assign_pick_to_layout_hole,
    detect_tray_holes,
    match_tray_layout_to_detected_holes,
    snap_pick_to_nearest_hole,
)
from vision.tray_reference import refine_base_xy_with_checkerboard
from core.job_store import JobStore
import config


logger = logging.getLogger(__name__)


class AbortException(Exception):
    """Raised when job is aborted."""
    pass


def _clamp_pick_z_sequence(
    scan_z: float,
    point_z: float,
    approach_offset_z: float,
    final_offset_z: float,
    retreat_offset_z: float,
    min_descent_mm: float = 5.0,
):
    """Keep pick poses below SCAN_POSE so the arm does not move upward first."""
    min_descent_m = min_descent_mm / 1000.0
    max_working_z = scan_z - min_descent_m
    approach_z = min(point_z + approach_offset_z, max_working_z)
    final_z = point_z + final_offset_z
    retreat_z = min(point_z + retreat_offset_z, max_working_z)
    return approach_z, final_z, retreat_z


class PickPlaceCycle:
    """Orchestrates a complete pick-place cycle."""

    def __init__(self, robot_ip: str, job_store: JobStore, job_id: str):
        """
        Initialize pick-place cycle.
        
        Args:
            robot_ip: Robot IP address
            job_store: JobStore instance
            job_id: Job ID to manage
        """
        self.robot_ip = robot_ip
        self.job_store = job_store
        self.job_id = job_id

        # Initialize clients (not connected yet)
        self.dashboard = DashboardClient(robot_ip)
        self.urscript = URScriptClient(robot_ip)
        self.rtde = RTDEClient(robot_ip)
        self.gripper = GripperRG.try_create(self.urscript)
        self.camera = None
        self.detector = None

        self.tcp_pose_at_capture = None

    def _effective_intrinsics(self, frame_width: int, frame_height: int):
        """Scale camera intrinsics to the runtime frame resolution."""
        sx = frame_width / float(config.CAM_CALIB_WIDTH)
        sy = frame_height / float(config.CAM_CALIB_HEIGHT)
        return (
            config.CAM_FX * sx,
            config.CAM_FY * sy,
            config.CAM_CX * sx,
            config.CAM_CY * sy,
            sx,
            sy,
        )

    def _log_transform_debug(
        self,
        u: float,
        v: float,
        depth_mm: float,
        p_cam,
        tcp_pose,
        p_base,
        xy_source: str,
        fx_eff: float,
        fy_eff: float,
        cx_eff: float,
        cy_eff: float,
        frame_width: int,
        frame_height: int,
    ) -> None:
        """Emit enough data to separate pixel/depth error from base-transform error."""
        camera_origin_base = camera_to_base([0.0, 0.0, 0.0], tcp_pose, config.T_CAM_TO_TCP)
        self._log(
            "VisionDebug: "
            f"frame={frame_width}x{frame_height}, "
            f"intrinsics=(fx={fx_eff:.2f},fy={fy_eff:.2f},cx={cx_eff:.2f},cy={cy_eff:.2f}), "
            f"pick_uv=({u:.1f},{v:.1f}), depth_mm={depth_mm:.1f}"
        )
        self._log(f"VisionDebug: p_cam(m)={[round(vv, 4) for vv in p_cam]}")
        self._log(f"VisionDebug: tcp_pose(m,rad)={[round(vv, 4) for vv in tcp_pose]}")
        self._log(f"VisionDebug: camera_origin_base(m)={[round(vv, 4) for vv in camera_origin_base]}")
        self._log(f"VisionDebug: p_base_raw(m)={[round(vv, 4) for vv in p_base]}, xy_source={xy_source}")

    def _check_abort(self) -> None:
        """Check if job was aborted, raise if so."""
        if self.job_store.is_aborted(self.job_id):
            logger.warning(f"Job {self.job_id} abort requested")
            raise AbortException("Job aborted by user")

    def _set_phase(self, phase: str) -> None:
        """Set current phase in job store."""
        self.job_store.set_phase(self.job_id, phase)

    def _log(self, message: str) -> None:
        """Log message to job store."""
        self.job_store.append_log(self.job_id, message)

    def _log_gripper_event(self, event: Dict) -> None:
        """Store structured gripper response for web/UI observers."""
        try:
            self._log(f"GripperEvent: {json.dumps(event, ensure_ascii=True)}")
        except Exception:
            self._log(f"GripperEvent: {event}")

    def _ensure_vision_stack(self) -> None:
        """Lazy-init camera and detector so phase 1 can run without vision stack."""
        if self.camera is None:
            self.camera = FemtoCamera(
                width=config.CAMERA_WIDTH,
                height=config.CAMERA_HEIGHT,
            )
        if self.detector is None:
            self.detector = Detector(
                model_path=config.YOLO_MODEL_PATH,
                confidence=config.YOLO_CONFIDENCE,
                target_class=config.YOLO_TARGET_CLASS,
            )

    def _apply_payload_settings(self) -> None:
        """Apply payload mass/CoG before motion commands."""
        try:
            self.urscript.set_payload(
                config.PAYLOAD_MASS_KG,
                config.PAYLOAD_COG,
            )
            self._log(
                f"Payload set: mass={config.PAYLOAD_MASS_KG:.3f}kg, cog={config.PAYLOAD_COG}"
            )
        except Exception as exc:
            logger.warning(f"Failed to set payload: {exc}")
            self._log(f"WARNING: set_payload failed: {exc}")

    def _disconnect_all(self) -> None:
        """Disconnect all clients safely."""
        try:
            if self.camera is not None:
                self.camera.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting camera: {e}")

        try:
            self.rtde.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting RTDE: {e}")

        try:
            self.urscript.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting URScript: {e}")

        try:
            self.dashboard.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting Dashboard: {e}")

    def _run_phase_1_static(self, stage_label: str) -> Dict[str, object]:
        """Phase 1: Static motion only, no camera/YOLO, no physical grip."""
        detected_objects = 0
        try:
            self._set_phase("phase1_initializing")
            self.dashboard.connect()
            self.urscript.connect()
            self.rtde.connect()

            self.dashboard.precheck_ready()
            self.dashboard.prepare_to_run()
            self._apply_payload_settings()

            self._set_phase("phase1_moving_home")
            self.urscript.move_joint(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("phase1_moving_scan")
            self.urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("phase1_moving_pick_approach")
            self.urscript.move_linear(
                config.PICK_APPROACH_CART_STATIC,
                accel=config.LINEAR_ACCEL,
                vel=config.LINEAR_VEL,
            )
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("phase1_simulated_grip")
            self._log_gripper_event(
                self.gripper.close(
                    force_n=config.GRIPPER_CLOSE_FORCE,
                    width_mm=config.GRIPPER_CLOSE_WIDTH,
                )
            )
            self._log_gripper_event(self.gripper.open(config.GRIPPER_OPEN_WIDTH))

            self._set_phase("phase1_moving_place_approach")
            self.urscript.move_linear(
                config.PLACE_APPROACH_CART,
                accel=config.LINEAR_ACCEL,
                vel=config.LINEAR_VEL,
            )
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("phase1_returning_home")
            self.urscript.move_joint(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("done")
            self.job_store.update_job(self.job_id, status="done")
            return {
                "status": "success",
                "stage": "Phase 1 completed",
                "experiment_stage": 1,
                "experiment_label": stage_label,
                "detected_objects": detected_objects,
                "parts_found": 0,
                "parts_picked": 0,
            }
        except Exception:
            self.job_store.update_job(self.job_id, status="error")
            raise
        finally:
            self._disconnect_all()

    def _run_phase_2_motion_vision(self, stage_label: str) -> Dict[str, object]:
        """Phase 2: Motion + vision + coordinate transform, no gripping."""
        detected_objects = 0
        target_pose = None
        try:
            self._ensure_vision_stack()

            self._set_phase("phase2_initializing")
            self.dashboard.connect()
            self.urscript.connect()
            self.rtde.connect()
            self.camera.connect()

            self.dashboard.precheck_ready()
            self.dashboard.prepare_to_run()
            self._apply_payload_settings()

            self._set_phase("phase2_moving_home")
            self.urscript.move_joint(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("phase2_moving_scan")
            self.urscript.move_joint(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("phase2_vision_detect")
            rgb, depth, cam_ts = self.camera.get_frames_with_timestamp()
            detections = self.detector.detect(rgb)
            detected_objects = len(detections)
            self.job_store.update_job(self.job_id, parts_found=detected_objects)
            self._log(f"Phase2 detections: {detected_objects}")
            frame_h, frame_w = depth.shape
            fx_eff, fy_eff, cx_eff, cy_eff, sx, sy = self._effective_intrinsics(frame_w, frame_h)
            if abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3:
                self._log(
                    "CANH BAO: runtime frame khac calibration baseline, "
                    f"auto-scale intrinsics sx={sx:.3f}, sy={sy:.3f} "
                    f"(baseline={int(config.CAM_CALIB_WIDTH)}x{int(config.CAM_CALIB_HEIGHT)}, "
                    f"frame={frame_w}x{frame_h})"
                )

            if not detections:
                self._log("Khong phat hien object nao. Robot se return home.")
            else:
                frame_center_uv = (frame_w / 2.0, frame_h / 2.0)
                target = self.detector.select_best_target(detections, depth, frame_center_uv)
                if target is None:
                    self._log("Co detection nhung select_best_target tra None (tat ca depth khong hop le).")
                else:
                    target = self.detector.refine_pick_point(rgb, target, depth)
                    u, v = target.pick_point
                    depth_mm, depth_bbox = self.detector.resolve_pick_depth(depth, target)
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
                        if snapped_hole is None:
                            snapped_hole = snap_pick_to_nearest_hole(
                                [u, v],
                                holes,
                                max_snap_dist_px=config.TRAY_HOLE_MAX_SNAP_DIST_PX,
                            )
                        if snapped_hole is not None:
                            u, v = snapped_hole["center"]
                            if "id" in snapped_hole:
                                self._log(
                                    f"Hole source: tray_layout_hole id={snapped_hole['id']} "
                                    f"center=({u:.1f},{v:.1f}) assign={snapped_hole['assign_dist_px']:.1f}px "
                                    f"reproj={snapped_hole['reproj_error_px']:.1f}px"
                                )
                            else:
                                self._log(
                                    f"Hole source: tray_hole_snap center=({u:.1f},{v:.1f}) "
                                    f"r={snapped_hole['radius_px']:.1f}px dist={snapped_hole['snap_dist_px']:.1f}px"
                                )
                    self._log(
                        f"Target: label={target.label}, pick=({u:.1f},{v:.1f}), "
                        f"depth={depth_mm:.1f}mm, source={target.pick_source}, depth_bbox={depth_bbox}"
                    )
                    if depth_mm <= 0:
                        self._log("Depth = 0 tai target bbox, bo qua tinh toa do pick.")
                    else:
                        p_cam = pixel_to_camera_3d(
                            u,
                            v,
                            depth_mm,
                            fx_eff,
                            fy_eff,
                            cx_eff,
                            cy_eff,
                        )
                        tcp_pose, rtde_ts = self.rtde.get_tcp_pose_with_timestamp()
                        ts_diff = abs(cam_ts - rtde_ts)
                        if ts_diff > 0.1:
                            self._log(f"CANH BAO: frame/pose lech {ts_diff * 1000:.0f}ms — toa do co the khong chinh xac")
                        else:
                            self._log(f"Timestamp sync OK: delta={ts_diff * 1000:.1f}ms")
                        p_base = camera_to_base(p_cam, tcp_pose, config.T_CAM_TO_TCP)
                        xy_source = "depth_only"
                        if config.TRAY_REF_ENABLED:
                            p_base, xy_source = refine_base_xy_with_checkerboard(
                                rgb,
                                u,
                                v,
                                p_base,
                                tcp_pose,
                                config.T_CAM_TO_TCP,
                                fx_eff,
                                fy_eff,
                                cx_eff,
                                cy_eff,
                                config.TRAY_REF_INNER_CORNERS,
                                config.TRAY_REF_SQUARE_SIZE_M,
                            )
                        self._log_transform_debug(
                            u,
                            v,
                            depth_mm,
                            p_cam,
                            tcp_pose,
                            p_base,
                            xy_source,
                            fx_eff,
                            fy_eff,
                            cx_eff,
                            cy_eff,
                            frame_w,
                            frame_h,
                        )
                        p_base = [
                            p_base[0] + config.PICK_OFFSET_X,
                            p_base[1] + config.PICK_OFFSET_Y,
                            p_base[2] + config.PICK_OFFSET_Z,
                        ]
                        self._log(f"XY source: {xy_source}")
                        target_pose = build_pick_approach_pose(
                            p_base,
                            config.PICK_APPROACH_OFFSET_Z,
                            tool_rx=config.TOOL_DOWN_RX,
                            tool_ry=config.TOOL_DOWN_RY,
                            tool_rz=config.TOOL_DOWN_RZ,
                        )
                        self._log(f"target_pose={target_pose}")

            self._check_abort()

            if target_pose is not None:
                self._set_phase("phase2_moving_pick_approach")
                self.urscript.move_linear(
                    target_pose,
                    accel=config.LINEAR_ACCEL,
                    vel=config.LINEAR_VEL,
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
            else:
                self._log("target_pose la None, bo qua buoc move pick approach.")

            self._check_abort()

            self._set_phase("phase2_returning_home")
            self.urscript.move_joint(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            self._set_phase("done")
            self.job_store.update_job(self.job_id, status="done")
            return {
                "status": "success",
                "stage": "Phase 2 completed",
                "experiment_stage": 2,
                "experiment_label": stage_label,
                "detected_objects": detected_objects,
                "parts_found": detected_objects,
                "parts_picked": 0,
                "target_pose": target_pose,
            }
        except AbortException:
            self._set_phase("aborted")
            self._log("Phase 2 bi abort boi nguoi dung.")
            self.job_store.update_job(self.job_id, status="aborted")
            raise
        except Exception:
            self.job_store.update_job(self.job_id, status="error")
            raise
        finally:
            self._disconnect_all()

    def run(self) -> Dict[str, int]:
        """
        Execute complete pick-place cycle.
        
        Returns:
            {"parts_found": int, "parts_picked": int}
            
        Raises:
            Exception: On unrecoverable errors
            AbortException: If job was aborted
        """
        logger.info(f"Starting pick-place cycle for job {self.job_id}")
        self.job_store.update_job(self.job_id, status="running")

        job_snapshot = self.job_store.get_job(self.job_id) or {}
        stage = int(job_snapshot.get("experiment_stage") or config.EXPERIMENT_STAGE)
        stage_label = config.EXPERIMENT_STAGE_LABELS.get(stage, "unknown")
        self._log(f"ExperimentStage: {stage} ({stage_label})")

        if stage == 1:
            return self._run_phase_1_static(stage_label)
        if stage == 2:
            return self._run_phase_2_motion_vision(stage_label)

        self._ensure_vision_stack()

        try:
            # ============ INITIALIZATION ============
            self._set_phase("initializing")
            self._log("Connecting to robot...")

            self.dashboard.connect()
            self.urscript.connect()
            self.rtde.connect()
            self.camera.connect()

            self._log("All systems connected")

            # Safety check
            self._log("Performing safety check...")
            self.dashboard.precheck_ready()
            self._log("Safety check passed")

            # Prepare robot
            self._log("Preparing robot (power on, brake release)...")
            self.dashboard.prepare_to_run()
            self._log("Robot ready")
            self._apply_payload_settings()

            # Open gripper
            self._log("Opening gripper...")
            self._log_gripper_event(self.gripper.open(config.GRIPPER_OPEN_WIDTH))
            time.sleep(0.5)  # Wait for gripper to open

            # ============ HOME ============
            self._set_phase("moving_to_home")
            self._log("Moving to home position...")
            self.urscript.move_joint(
                config.HOME_JOINTS,
                accel=config.JOINT_ACCEL,
                vel=config.JOINT_VEL
            )
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
            self._log("At home")

            # ============ SCAN APPROACH ============
            self._set_phase("moving_to_scan_approach")
            self._log("Moving to scan approach position...")
            self.urscript.move_joint(
                config.SCAN_APPROACH_JOINTS,
                accel=config.JOINT_ACCEL,
                vel=config.JOINT_VEL
            )
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
            self._log("At scan approach")

            # ============ SCAN POSE ============
            self._set_phase("moving_to_scan_pose")
            self._log("Moving to scan position...")
            self.urscript.move_joint(
                config.SCAN_POSE_JOINTS,
                accel=config.JOINT_ACCEL,
                vel=config.JOINT_VEL
            )
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
            self._log("At scan position")

            # ============ INITIAL SCAN ============
            self._set_phase("initial_scan")
            self._log("Capturing initial scan...")
            # Initial scan chỉ cần đếm số lượng parts, không cần TCP pose
            rgb, depth, _ = self.camera.get_frames_with_timestamp()
            detections = self.detector.detect(rgb)

            parts_found = len(detections)
            self.job_store.update_job(self.job_id, parts_found=parts_found)
            self._log(f"Found {parts_found} part(s)")

            if parts_found == 0:
                self._set_phase("no_parts_found")
                self._log("No parts found, cycle complete")
                self._log("Returning to home...")
                self.urscript.move_joint(
                    config.HOME_JOINTS,
                    accel=config.JOINT_ACCEL,
                    vel=config.JOINT_VEL
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                self._log("At home")
                self._set_phase("done")
                self.job_store.update_job(self.job_id, status="done")
                return {
                    "status": "success",
                    "stage": "Phase 3 completed",
                    "experiment_stage": 3,
                    "experiment_label": stage_label,
                    "detected_objects": 0,
                    "parts_found": 0,
                    "parts_picked": 0,
                }

            # ============ MAIN PICK LOOP ============
            parts_picked = 0
            frame_center_uv = (config.CAMERA_WIDTH / 2.0, config.CAMERA_HEIGHT / 2.0)

            for pick_attempt in range(parts_found):
                self._check_abort()

                # ---- Scan before pick ----
                self._set_phase(f"scanning_before_pick_{pick_attempt}")
                self._log(f"Scanning for pick #{pick_attempt + 1}...")

                # ── Chụp frame + đọc TCP pose càng đồng thời càng tốt ────────────
                # camera.waitForFrames() blocking → RTDE đọc ngay sau đó
                # 2 timestamp đều dùng time.time() → có thể so sánh trực tiếp
                rgb, depth, cam_ts = self.camera.get_frames_with_timestamp()
                self.tcp_pose_at_capture, rtde_ts = self.rtde.get_tcp_pose_with_timestamp()

                ts_diff = abs(cam_ts - rtde_ts)
                if ts_diff > 0.1:
                    logger.warning(
                        "Frame/pose timestamp mismatch: %.3fs — "
                        "robot may have moved between capture and pose read",
                        ts_diff
                    )
                    self._log(
                        f"WARNING: frame/pose lệch {ts_diff*1000:.0f}ms — "
                        "có thể cần scan lại"
                    )

                detections = self.detector.detect(rgb)
                if not detections:
                    self._log("No more parts detected, ending cycle")
                    break

                target = self.detector.select_best_target(
                    detections,
                    depth,
                    frame_center_uv
                )

                if target is None:
                    self._log("No valid target with depth, skipping")
                    continue

                self._log(f"Target selected: {target.label} @ {target.bbox}")

                # ---- Calculate pick poses ----
                x1, y1, x2, y2 = target.bbox
                target = self.detector.refine_pick_point(rgb, target, depth)
                frame_h, frame_w = depth.shape
                fx_eff, fy_eff, cx_eff, cy_eff, sx, sy = self._effective_intrinsics(frame_w, frame_h)
                if abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3:
                    self._log(
                        "CANH BAO: runtime frame khac calibration baseline, "
                        f"auto-scale intrinsics sx={sx:.3f}, sy={sy:.3f} "
                        f"(baseline={int(config.CAM_CALIB_WIDTH)}x{int(config.CAM_CALIB_HEIGHT)}, "
                        f"frame={frame_w}x{frame_h})"
                    )
                u, v = target.pick_point
                depth_mm, _ = self.detector.resolve_pick_depth(depth, target)
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
                    if snapped_hole is None:
                        snapped_hole = snap_pick_to_nearest_hole(
                            [u, v],
                            holes,
                            max_snap_dist_px=config.TRAY_HOLE_MAX_SNAP_DIST_PX,
                        )
                    if snapped_hole is not None:
                        u, v = snapped_hole["center"]
                        if "id" in snapped_hole:
                            self._log(
                                f"Hole source: tray_layout_hole id={snapped_hole['id']} "
                                f"center=({u:.1f},{v:.1f}) assign={snapped_hole['assign_dist_px']:.1f}px "
                                f"reproj={snapped_hole['reproj_error_px']:.1f}px"
                            )
                        else:
                            self._log(
                                f"Hole source: tray_hole_snap center=({u:.1f},{v:.1f}) "
                                f"r={snapped_hole['radius_px']:.1f}px dist={snapped_hole['snap_dist_px']:.1f}px"
                            )

                if depth_mm <= 0:
                    self._log(f"No valid depth for target, skipping")
                    continue

                self._log(f"Target depth: {depth_mm:.1f}mm")

                # Transform to base frame
                p_cam = pixel_to_camera_3d(
                    u, v, depth_mm,
                    fx_eff, fy_eff,
                    cx_eff, cy_eff
                )

                p_base = camera_to_base(
                    p_cam,
                    self.tcp_pose_at_capture,
                    config.T_CAM_TO_TCP
                )
                xy_source = "depth_only"
                if config.TRAY_REF_ENABLED:
                    p_base, xy_source = refine_base_xy_with_checkerboard(
                        rgb,
                        u,
                        v,
                        p_base,
                        self.tcp_pose_at_capture,
                        config.T_CAM_TO_TCP,
                        fx_eff,
                        fy_eff,
                        cx_eff,
                        cy_eff,
                        config.TRAY_REF_INNER_CORNERS,
                        config.TRAY_REF_SQUARE_SIZE_M,
                    )
                self._log_transform_debug(
                    u,
                    v,
                    depth_mm,
                    p_cam,
                    self.tcp_pose_at_capture,
                    p_base,
                    xy_source,
                    fx_eff,
                    fy_eff,
                    cx_eff,
                    cy_eff,
                    frame_w,
                    frame_h,
                )
                p_base = [
                    p_base[0] + config.PICK_OFFSET_X,
                    p_base[1] + config.PICK_OFFSET_Y,
                    p_base[2] + config.PICK_OFFSET_Z,
                ]
                self._log(f"XY source: {xy_source}")

                pre_approach_pose = build_lateral_pre_approach_pose(
                    p_base,
                    self.tcp_pose_at_capture,
                    config.PICK_APPROACH_OFFSET_Z,
                    tool_rx=config.TOOL_DOWN_RX,
                    tool_ry=config.TOOL_DOWN_RY,
                    tool_rz=config.TOOL_DOWN_RZ
                )
                approach_z, final_z, retreat_z = _clamp_pick_z_sequence(
                    self.tcp_pose_at_capture[2],
                    p_base[2],
                    config.PICK_APPROACH_OFFSET_Z,
                    config.PICK_FINAL_OFFSET_Z,
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

                final_pose = [
                    p_base[0],
                    p_base[1],
                    final_z,
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

                self._log(f"Pre-approach pose: {pre_approach_pose}")
                self._log(f"Pick pose: {approach_pose}")

                current_scan_z = self.tcp_pose_at_capture[2]
                if final_pose[2] >= current_scan_z - 0.005:
                    raise RuntimeError(
                        "Computed final pick pose is not below SCAN_POSE. "
                        f"scan_z={current_scan_z:.4f}, final_z={final_pose[2]:.4f}. "
                        "Abort to avoid moving upward away from the part."
                    )
                if not (pre_approach_pose[2] >= approach_pose[2] >= final_pose[2]):
                    raise RuntimeError(
                        "Computed pick Z ordering is invalid. "
                        f"pre={pre_approach_pose[2]:.4f}, "
                        f"approach={approach_pose[2]:.4f}, "
                        f"final={final_pose[2]:.4f}"
                    )

                # ---- Move to approach ----
                for retry in range(config.MAX_PICK_RETRIES):
                    self._check_abort()

                    if retry > 0:
                        self._log(f"Retry pick #{retry + 1}/{config.MAX_PICK_RETRIES}")

                    self._set_phase(f"moving_to_pick_pre_approach_{pick_attempt}_retry_{retry}")
                    self._log("Moving to pick pre-approach...")
                    self.urscript.move_linear(
                        pre_approach_pose,
                        accel=config.LINEAR_ACCEL,
                        vel=config.LINEAR_VEL
                    )
                    self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                    self._log("At pick pre-approach")

                    self._set_phase(f"moving_to_pick_approach_{pick_attempt}_retry_{retry}")
                    self._log("Moving to pick approach...")
                    self.urscript.move_linear(
                        approach_pose,
                        accel=config.LINEAR_ACCEL,
                        vel=config.LINEAR_VEL
                    )
                    self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                    self._log("At pick approach")

                    # ---- Final descent ----
                    self._set_phase(f"picking_{pick_attempt}_retry_{retry}")
                    self._log("Descending to part...")
                    self.urscript.move_linear(
                        final_pose,
                        accel=config.LINEAR_ACCEL,
                        vel=config.PICK_APPROACH_VEL
                    )
                    self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                    self._log("At part surface")

                    # ---- Grip ----
                    self._log("Gripping part...")
                    self._log_gripper_event(
                        self.gripper.close(
                            force_n=config.GRIPPER_CLOSE_FORCE,
                            width_mm=config.GRIPPER_CLOSE_WIDTH
                        )
                    )

                    grip_ok = self.gripper.wait_grip_detected(
                        rtde_client=self.rtde,
                        timeout_s=config.GRIPPER_TIMEOUT_S
                    )

                    if grip_ok:
                        self._log("Part gripped successfully")
                        break
                    else:
                        self._log("Grip detection failed")
                        
                        if retry < config.MAX_PICK_RETRIES - 1:
                            self._log("Retreating to retry...")
                            self.urscript.move_linear(
                                retreat_pose,
                                accel=config.LINEAR_ACCEL,
                                vel=config.LINEAR_VEL
                            )
                            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                            self._log_gripper_event(self.gripper.open(config.GRIPPER_OPEN_WIDTH))
                            time.sleep(0.3)
                        else:
                            raise RuntimeError(
                                f"Grip failed after {config.MAX_PICK_RETRIES} retries"
                            )
                else:
                    continue  # Retried, continue to next part

                # Successfully gripped, continue to place

                # ---- Retreat after pick ----
                self._set_phase(f"retreating_after_pick_{pick_attempt}")
                self._log("Retreating with part...")
                self.urscript.move_linear(
                    retreat_pose,
                    accel=config.LINEAR_ACCEL,
                    vel=config.LINEAR_VEL
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                self._log("Part retreated safely")

                # ---- Move to place ----
                self._set_phase(f"moving_to_place_{pick_attempt}")
                self._log("Moving to place position...")
                self.urscript.move_linear(
                    config.PLACE_APPROACH_CART,
                    accel=config.LINEAR_ACCEL,
                    vel=config.LINEAR_VEL
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                self._log("At place approach")

                # ---- Descend for place ----
                self._log("Descending to conveyor...")
                self.urscript.move_linear(
                    config.PLACE_POINT_CART,
                    accel=config.LINEAR_ACCEL,
                    vel=config.PICK_APPROACH_VEL
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                self._log("At conveyor")

                # ---- Release ----
                self._set_phase(f"placing_{pick_attempt}")
                self._log("Releasing part...")
                self._log_gripper_event(self.gripper.open(config.GRIPPER_OPEN_WIDTH))
                time.sleep(0.3)  # Wait for part to settle
                self._log("Part released")

                # ---- Retreat after place ----
                self._set_phase(f"retreating_after_place_{pick_attempt}")
                self._log("Retreating from conveyor...")
                self.urscript.move_linear(
                    config.PLACE_RETREAT_CART,
                    accel=config.LINEAR_ACCEL,
                    vel=config.LINEAR_VEL
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
                self._log("Safe retreat")

                parts_picked += 1
                self.job_store.update_job(self.job_id, parts_picked=parts_picked)

                # ---- Return to scan for next part ----
                self._log("Returning to scan position...")
                self.urscript.move_joint(
                    config.SCAN_POSE_JOINTS,
                    accel=config.JOINT_ACCEL,
                    vel=config.JOINT_VEL
                )
                self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)

            # ============ FINISH ============
            self._set_phase("returning_home")
            self._log("Returning to home...")
            self.urscript.move_joint(
                config.HOME_JOINTS,
                accel=config.JOINT_ACCEL,
                vel=config.JOINT_VEL
            )
            self.rtde.wait_steady(timeout_s=config.RTDE_WAIT_TIMEOUT)
            self._log("At home")

            self._set_phase("done")
            self._log(f"Cycle complete: {parts_picked}/{parts_found} parts picked")
            self.job_store.update_job(self.job_id, status="done")

            return {
                "status": "success",
                "stage": "Phase 3 completed",
                "experiment_stage": 3,
                "experiment_label": stage_label,
                "detected_objects": parts_found,
                "parts_found": parts_found,
                "parts_picked": parts_picked,
            }

        except AbortException as e:
            logger.warning(f"Job {self.job_id} aborted: {e}")
            self._set_phase("aborted")
            self._log(f"Cycle aborted: {e}")

            # Cleanup: open gripper and return home
            try:
                self._log("Aborting: opening gripper...")
                self._log_gripper_event(self.gripper.open(config.GRIPPER_OPEN_WIDTH))
                time.sleep(0.3)

                self._log("Aborting: returning to home...")
                self.urscript.move_joint(
                    config.HOME_JOINTS,
                    accel=config.JOINT_ACCEL,
                    vel=config.JOINT_VEL
                )
                self.rtde.wait_steady(timeout_s=10.0)
            except Exception as cleanup_err:
                logger.error(f"Error during abort cleanup: {cleanup_err}")

            self.job_store.update_job(self.job_id, status="aborted")
            raise

        except Exception as e:
            logger.error(f"Error in pick-place cycle: {e}\n{traceback.format_exc()}")
            self._set_phase("error")
            self._log(f"Error: {str(e)}")
            self.job_store.update_job(
                self.job_id,
                status="error",
                error=str(e)
            )

            # Cleanup attempt
            try:
                logger.info("Attempting cleanup after error...")
                self._log_gripper_event(self.gripper.open(config.GRIPPER_OPEN_WIDTH))
                time.sleep(0.3)
                self.urscript.move_joint(
                    config.HOME_JOINTS,
                    accel=config.JOINT_ACCEL,
                    vel=config.JOINT_VEL
                )
                self.rtde.wait_steady(timeout_s=10.0)
            except Exception as cleanup_err:
                logger.error(f"Error during error cleanup: {cleanup_err}")

            raise

        finally:
            # Always cleanup connections
            try:
                self.camera.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting camera: {e}")

            try:
                self.rtde.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting RTDE: {e}")

            try:
                self.urscript.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting URScript: {e}")

            try:
                self.dashboard.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting Dashboard: {e}")

            logger.info(f"Cleanup complete for job {self.job_id}")
