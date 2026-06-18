"""
Pick-Place Cycle Orchestrator.
Main logic for autonomous pick-place operations.
"""

import json
import logging
import time
import traceback
import math
from typing import Dict, List, Optional

import config
from core.job_store import JobStore
from core.pneumatic_gripper import PneumaticGripper, GripperError
from robot.dashboard_client import DashboardClient
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient
from vision.calibration import (
    clamp_pick_z_sequence,
    camera_origin_to_base,
    camera_to_base,
    pixel_to_camera_3d,
    resolve_intrinsics_for_frame,
    sanitize_camera_depth_mm,
)
from vision.detector import Detector
from vision.femto_camera import FemtoCamera


logger = logging.getLogger(__name__)


class AbortException(Exception):
    """Raised when job is aborted."""


class PickPlaceCycle:
    """Orchestrates a complete pick-place cycle."""

    def __init__(
        self,
        dashboard: DashboardClient,
        urscript: URScriptClient,
        rtde: RTDEClient,
        job_store: JobStore,
        job_id: str,
        robot_ip: str = "",
        gripper: Optional[PneumaticGripper] = None,
    ) -> None:
        self.robot_ip = robot_ip
        self.job_store = job_store
        self.job_id = job_id

        # Clients are injected externally; this class does not own connection lifecycle.
        self.dashboard = dashboard
        self.urscript = urscript
        self.rtde = rtde
        self.gripper = gripper

        self.camera = None  # type: Optional[FemtoCamera]
        self.detector = None  # type: Optional[Detector]
        self._camera_connected = False

        self.tcp_pose_at_capture = None

    def _validate_pick_motion(
        self,
        tcp_pose_at_capture,
        approach_pose,
        final_pose,
    ) -> None:
        """Reject implausible pick poses before robot moves toward the part."""
        dx = approach_pose[0] - tcp_pose_at_capture[0]
        dy = approach_pose[1] - tcp_pose_at_capture[1]
        planar_dist = math.hypot(dx, dy)
        approach_lift = approach_pose[2] - tcp_pose_at_capture[2]
        final_vs_scan = final_pose[2] - config.SCAN_POSE_TCP[2]
        descent_span = approach_pose[2] - final_pose[2]
        camera_origin_base = camera_origin_to_base(tcp_pose_at_capture, config.T_CAM_TO_TCP)
        final_below_camera = camera_origin_base[2] - final_pose[2]

        if planar_dist > config.PICK_MAX_PLANAR_DELTA_M:
            raise RuntimeError(
                "Target lech qua xa theo XY: planar={:.3f}m > {:.3f}m".format(
                    planar_dist,
                    config.PICK_MAX_PLANAR_DELTA_M,
                )
            )
        if approach_lift > config.PICK_MAX_APPROACH_LIFT_M:
            raise RuntimeError(
                "Approach pose bi day len qua cao: dz={:.3f}m > {:.3f}m".format(
                    approach_lift,
                    config.PICK_MAX_APPROACH_LIFT_M,
                )
            )
        if final_vs_scan > config.PICK_MAX_FINAL_Z_ABOVE_SCAN_M:
            raise RuntimeError(
                "Final pick pose nam cao hon SCAN_POSE: dz={:.3f}m > {:.3f}m".format(
                    final_vs_scan,
                    config.PICK_MAX_FINAL_Z_ABOVE_SCAN_M,
                )
            )
        if final_below_camera < config.PICK_MIN_FINAL_BELOW_CAMERA_M:
            raise RuntimeError(
                "Final pick pose chua nam duoi camera du an toan: clearance={:.3f}m < {:.3f}m".format(
                    final_below_camera,
                    config.PICK_MIN_FINAL_BELOW_CAMERA_M,
                )
            )
        if descent_span < config.PICK_MIN_DESCENT_M:
            raise RuntimeError(
                "Approach/final khong tao duoc huong di xuong: descent={:.3f}m < {:.3f}m".format(
                    descent_span,
                    config.PICK_MIN_DESCENT_M,
                )
            )

        self._log(
            "Pick motion validated: planar={:.3f}m, approach_lift={:.3f}m, "
            "final_vs_scan={:.3f}m, below_camera={:.3f}m, descent={:.3f}m".format(
                planar_dist,
                approach_lift,
                final_vs_scan,
                final_below_camera,
                descent_span,
            )
        )

    def _check_abort(self) -> None:
        if self.job_store.is_aborted(self.job_id):
            logger.warning("Job %s abort requested", self.job_id)
            raise AbortException("Job aborted by user")

    def _set_phase(self, phase: str) -> None:
        self.job_store.set_phase(self.job_id, phase)

    def _log(self, message: str) -> None:
        self.job_store.append_log(self.job_id, message)

    def _log_gripper_event(self, event: Dict) -> None:
        try:
            self._log("GripperEvent: {}".format(json.dumps(event, ensure_ascii=True)))
        except Exception:
            self._log("GripperEvent: {}".format(event))

    def _gripper_close(self) -> Dict[str, object]:
        if self.gripper is None:
            raise RuntimeError("Pneumatic gripper is not initialized")
        try:
            result = self.gripper.close()
        except GripperError as err:
            raise RuntimeError("Gripper loi phan cung: {}".format(err)) from err

        self._log_gripper_event(result)
        if not result.get("ok", False):
            raise RuntimeError("Grip khong xac nhan: {}".format(result.get("response")))
        return result

    def _gripper_open(self) -> Dict[str, object]:
        if self.gripper is None:
            raise RuntimeError("Pneumatic gripper is not initialized")
        try:
            result = self.gripper.open()
        except GripperError as err:
            raise RuntimeError("Gripper loi phan cung: {}".format(err)) from err

        self._log_gripper_event(result)
        if not result.get("ok", False):
            raise RuntimeError("Release khong xac nhan: {}".format(result.get("response")))
        return result

    def _ensure_vision_stack(self) -> None:
        if self.camera is None:
            self.camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)

        if not self._camera_connected:
            self.camera.connect()
            self._camera_connected = True

        if self.detector is None:
            self.detector = Detector(
                model_path=config.YOLO_MODEL_PATH,
                confidence=config.YOLO_CONFIDENCE,
                target_class=config.YOLO_TARGET_CLASS,
            )

    def _apply_runtime_tool_settings(self) -> None:
        """DEPRECATED: set_tcp/payload duoc bundle vao tung motion tren CB3."""
        self._log(
            "Tool config (bundled vao tung motion): tcp={}, mass={:.3f}kg, cog={}".format(
                [round(float(v), 4) for v in config.TCP_OFFSET],
                config.PAYLOAD_MASS_KG,
                [round(float(v), 4) for v in config.PAYLOAD_COG],
            )
        )

    def _wait_steady_default(self) -> bool:
        return self.rtde.wait_steady(
            timeout_s=config.RTDE_WAIT_TIMEOUT,
            motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        )

    def _wait_steady_cleanup(self) -> bool:
        return self.rtde.wait_steady(timeout_s=10.0, motion_start_timeout=10.0)

    def _move_joint_runtime(self, joints: List[float], accel: float, vel: float) -> None:
        self.urscript.move_joint_with_settings(
            joints,
            tcp_offset=config.TCP_OFFSET,
            payload_kg=config.PAYLOAD_MASS_KG,
            payload_cog=config.PAYLOAD_COG,
            accel=accel,
            vel=vel,
        )

    def _move_linear_runtime(self, pose: List[float], accel: float, vel: float) -> None:
        self.urscript.move_linear_with_settings(
            pose,
            tcp_offset=config.TCP_OFFSET,
            payload_kg=config.PAYLOAD_MASS_KG,
            payload_cog=config.PAYLOAD_COG,
            accel=accel,
            vel=vel,
        )

    def _disconnect_all(self) -> None:
        """No-op for compatibility. Connection lifecycle is managed externally."""
        logger.warning("_disconnect_all() called but lifecycle is externally managed")

    def _run_phase_1_static(self, stage_label: str) -> Dict[str, object]:
        """Phase 1: Motion-only validation through all taught points."""
        try:
            self._set_phase("phase1_initializing")
            self.dashboard.precheck_ready()
            self.dashboard.prepare_to_run()
            time.sleep(1.5)  # CB3 brake release settle — URScript dropped if sent too early
            self._apply_runtime_tool_settings()
            self._check_abort()

            self._set_phase("phase1_moving_home")
            self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase1_moving_scan_approach")
            self._move_joint_runtime(config.SCAN_APPROACH_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase1_moving_scan_pose")
            self._move_joint_runtime(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase1_moving_place_approach")
            self._move_linear_runtime(config.PLACE_APPROACH_CART, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase1_moving_place_point")
            self._move_linear_runtime(config.PLACE_POINT_CART, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
            time.sleep(config.CB3_MOTION_PRE_WAIT_SLEEP_S)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase1_moving_place_retreat")
            self._move_linear_runtime(config.PLACE_RETREAT_CART, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase1_returning_home")
            self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()

            self._set_phase("done")
            self.job_store.update_job(self.job_id, status="done")
            return {
                "status": "success",
                "stage": "Phase 1 completed",
                "experiment_stage": 1,
                "experiment_label": stage_label,
                "detected_objects": 0,
                "parts_found": 0,
                "parts_picked": 0,
            }
        except AbortException:
            self._set_phase("aborted")
            self._log("Phase 1 aborted.")
            self.job_store.update_job(self.job_id, status="aborted")
            raise
        except Exception:
            self._set_phase("error")
            self.job_store.update_job(self.job_id, status="error")
            raise

    def _run_phase_2_motion_vision(self, stage_label: str) -> Dict[str, object]:
        """Phase 2: Full motion + vision + single pick-place."""
        detected_objects = 0
        approach_pose = None
        final_pose = None

        try:
            self._ensure_vision_stack()

            self._set_phase("phase2_initializing")
            self.dashboard.precheck_ready()
            self.dashboard.prepare_to_run()
            time.sleep(1.5)  # CB3 brake release settle — URScript dropped if sent too early
            self._apply_runtime_tool_settings()

            self._set_phase("phase2_opening_gripper")
            self._gripper_open()
            self._check_abort()

            self._set_phase("phase2_moving_home")
            self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase2_moving_scan_approach")
            self._move_joint_runtime(config.SCAN_APPROACH_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase2_moving_scan_pose")
            self._move_joint_runtime(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase2_vision_detect")
            rgb, depth, cam_ts = self.camera.get_frames_with_timestamp()
            tcp_pose_at_capture, rtde_ts = self.rtde.get_tcp_pose_with_timestamp()
            detections = self.detector.detect(rgb)
            detected_objects = len(detections)
            self.job_store.update_job(self.job_id, parts_found=detected_objects)
            self._log("Phase2 detections: {}".format(detected_objects))

            if detections:
                frame_center_uv = (config.CAMERA_WIDTH / 2.0, config.CAMERA_HEIGHT / 2.0)
                target = self.detector.select_best_target(detections, depth, frame_center_uv)
                if target is None:
                    self._log("Co detection nhung select_best_target tra None (tat ca depth khong hop le).")
                    for det in detections:
                        self._log(
                            "DepthDebug: {}".format(
                                json.dumps(
                                    {
                                        "bbox": det.bbox,
                                        "center": det.center,
                                        "depth_debug": getattr(det, "depth_debug", {}),
                                        "roi_debug": self.camera.analyze_depth_roi(depth, det.bbox),
                                    },
                                    ensure_ascii=True,
                                )
                            )
                        )
                else:
                    target = self.detector.refine_pick_point(rgb, target, depth)
                    u, v = target.pick_point
                    depth_mm, depth_bbox = self.detector.resolve_pick_depth(depth, target)
                    self._log(
                        "Target: label={}, center=({:.1f},{:.1f}), pick=({:.1f},{:.1f}), depth={:.1f}mm, source={}, depth_bbox={}".format(
                            target.label,
                            target.center[0],
                            target.center[1],
                            u,
                            v,
                            depth_mm,
                            target.pick_source,
                            depth_bbox,
                        )
                    )
                    if depth_mm > 0:
                        raw_depth_mm = depth_mm
                        depth_mm, was_clamped, min_safe_depth_mm = sanitize_camera_depth_mm(
                            depth_mm,
                            config.T_CAM_TO_TCP,
                            margin_below_tcp_m=config.PICK_MIN_DESCENT_M,
                        )
                        if was_clamped:
                            self._log(
                                "CANH BAO: depth {:.1f}mm nho hon camera->TCP standoff, "
                                "tu dong nang len {:.1f}mm de tranh pick nguoc chieu.".format(
                                    raw_depth_mm,
                                    min_safe_depth_mm,
                                )
                            )
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
                        if intr["reason"]:
                            self._log("IntrinsicsWarning: {}".format(intr["reason"]))
                        p_cam = pixel_to_camera_3d(
                            u,
                            v,
                            depth_mm,
                            intr["fx"],
                            intr["fy"],
                            intr["cx"],
                            intr["cy"],
                        )
                        ts_diff = abs(cam_ts - rtde_ts)
                        if ts_diff > 0.1:
                            self._log("CANH BAO: frame/pose lech {:.0f}ms".format(ts_diff * 1000.0))
                        else:
                            self._log("Timestamp sync OK: delta={:.1f}ms".format(ts_diff * 1000.0))

                        p_base_raw = camera_to_base(p_cam, tcp_pose_at_capture, config.T_CAM_TO_TCP)
                        p_base = [
                            p_base_raw[0] + config.PICK_OFFSET_X,
                            p_base_raw[1] + config.PICK_OFFSET_Y,
                            p_base_raw[2] + config.PICK_OFFSET_Z,
                        ]
                        self._log(
                            "DepthDebugTarget: {}".format(
                                json.dumps(
                                    self.camera.analyze_depth_roi(depth, target.bbox),
                                    ensure_ascii=True,
                                )
                            )
                        )
                        self._log(
                            "PickOffset: raw_base={}, offset={}, final_base={}".format(
                                [round(float(v), 6) for v in p_base_raw],
                                [
                                    round(float(config.PICK_OFFSET_X), 6),
                                    round(float(config.PICK_OFFSET_Y), 6),
                                    round(float(config.PICK_OFFSET_Z), 6),
                                ],
                                [round(float(v), 6) for v in p_base],
                            )
                        )
                        approach_z, final_z, _ = clamp_pick_z_sequence(
                            tcp_pose_at_capture[2],
                            p_base[2],
                            config.PICK_APPROACH_OFFSET_Z,
                            config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET,
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
                        self._validate_pick_motion(
                            tcp_pose_at_capture,
                            approach_pose,
                            final_pose,
                        )
                        self._log("approach_pose={}".format(approach_pose))
                        self._log("final_pose={}".format(final_pose))
                    else:
                        self._log("Depth = 0 tai target bbox, bo qua tinh toa do pick.")
            else:
                self._log("Khong phat hien object nao. Robot se return home.")

            self._check_abort()

            if approach_pose is None or final_pose is None:
                self._set_phase("phase2_returning_home")
                self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_default()
                self._set_phase("done")
                self.job_store.update_job(self.job_id, status="done", parts_picked=0)
                return {
                    "status": "success",
                    "stage": "Phase 2 completed",
                    "experiment_stage": 2,
                    "experiment_label": stage_label,
                    "detected_objects": detected_objects,
                    "parts_found": detected_objects,
                    "parts_picked": 0,
                    "target_pose": None,
                }

            self._set_phase("phase2_moving_pick_approach")
            self._move_linear_runtime(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase2_descending_to_pick")
            self._move_linear_runtime(final_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
            time.sleep(config.CB3_MOTION_PRE_WAIT_SLEEP_S)
            self._wait_steady_default()

            self._set_phase("phase2_gripping")
            self._gripper_close()

            self._set_phase("phase2_retreating_after_pick")
            self._move_linear_runtime(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            self._wait_steady_default()
            self._check_abort()

            self._set_phase("phase2_moving_place_approach")
            self._move_linear_runtime(config.PLACE_APPROACH_CART, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            self._wait_steady_default()

            self._set_phase("phase2_descending_to_place")
            self._move_linear_runtime(config.PLACE_POINT_CART, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
            time.sleep(config.CB3_MOTION_PRE_WAIT_SLEEP_S)
            self._wait_steady_default()

            self._set_phase("phase2_releasing")
            self._gripper_open()
            time.sleep(0.3)

            self._set_phase("phase2_retreating_after_place")
            self._move_linear_runtime(config.PLACE_RETREAT_CART, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
            self._wait_steady_default()

            self._set_phase("phase2_returning_home")
            self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()

            self._set_phase("done")
            self.job_store.update_job(self.job_id, status="done", parts_picked=1)
            return {
                "status": "success",
                "stage": "Phase 2 completed",
                "experiment_stage": 2,
                "experiment_label": stage_label,
                "detected_objects": detected_objects,
                "parts_found": detected_objects,
                "parts_picked": 1,
                "target_pose": final_pose,
            }

        except AbortException:
            self._set_phase("aborted")
            self._log("Phase 2 aborted: cleanup...")
            try:
                self._gripper_open()
                self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_cleanup()
            except Exception as cleanup_err:
                logger.error("Abort cleanup error: %s", cleanup_err)
            self.job_store.update_job(self.job_id, status="aborted")
            raise

        except Exception:
            self._set_phase("error")
            try:
                self._gripper_open()
                self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_cleanup()
            except Exception as cleanup_err:
                logger.error("Error cleanup: %s", cleanup_err)
            self.job_store.update_job(self.job_id, status="error")
            raise

        finally:
            if self._camera_connected:
                try:
                    self.camera.disconnect()
                    self._camera_connected = False
                except Exception as err:
                    logger.error("Camera disconnect error: %s", err)

    def run(self) -> Dict[str, int]:
        """Execute complete pick-place cycle."""
        logger.info("Starting pick-place cycle for job %s", self.job_id)
        self.job_store.update_job(self.job_id, status="running")

        job_snapshot = self.job_store.get_job(self.job_id) or {}
        stage = int(job_snapshot.get("experiment_stage") or config.EXPERIMENT_STAGE)
        stage_label = config.EXPERIMENT_STAGE_LABELS.get(stage, "unknown")
        self._log("ExperimentStage: {} ({})".format(stage, stage_label))

        if stage == 1:
            return self._run_phase_1_static(stage_label)
        if stage == 2:
            return self._run_phase_2_motion_vision(stage_label)

        self._ensure_vision_stack()

        try:
            self._set_phase("initializing")
            self._log("Verifying robot readiness...")

            self._log("Performing safety check...")
            self.dashboard.precheck_ready()
            self._log("Safety check passed")

            self._log("Preparing robot (power on, brake release)...")
            self.dashboard.prepare_to_run()
            time.sleep(1.5)  # CB3 brake release settle — URScript dropped if sent too early
            self._apply_runtime_tool_settings()
            self._log("Robot ready")

            self._log("Opening gripper...")
            self._gripper_open()
            time.sleep(0.5)

            self._set_phase("moving_to_home")
            self._log("Moving to home position...")
            self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._log("At home")

            self._set_phase("moving_to_scan_approach")
            self._log("Moving to scan approach position...")
            self._move_joint_runtime(config.SCAN_APPROACH_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._log("At scan approach")

            self._set_phase("moving_to_scan_pose")
            self._log("Moving to scan position...")
            self._move_joint_runtime(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._log("At scan position")

            self._set_phase("initial_scan")
            self._log("Capturing initial scan...")
            rgb, depth, _ = self.camera.get_frames_with_timestamp()
            detections = self.detector.detect(rgb)

            parts_found = len(detections)
            self.job_store.update_job(self.job_id, parts_found=parts_found)
            self._log("Found {} part(s)".format(parts_found))

            if parts_found == 0:
                self._set_phase("no_parts_found")
                self._log("No parts found, cycle complete")
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

            parts_picked = 0
            frame_center_uv = (config.CAMERA_WIDTH / 2.0, config.CAMERA_HEIGHT / 2.0)

            for pick_attempt in range(parts_found):
                self._check_abort()

                self._set_phase("scanning_before_pick_{}".format(pick_attempt))
                self._log("Scanning for pick #{}...".format(pick_attempt + 1))

                rgb, depth, cam_ts = self.camera.get_frames_with_timestamp()
                self.tcp_pose_at_capture, rtde_ts = self.rtde.get_tcp_pose_with_timestamp()

                ts_diff = abs(cam_ts - rtde_ts)
                if ts_diff > 0.1:
                    logger.warning(
                        "Frame/pose timestamp mismatch: %.3fs - robot may have moved between capture and pose read",
                        ts_diff,
                    )
                    self._log("WARNING: frame/pose lech {:.0f}ms".format(ts_diff * 1000.0))

                detections = self.detector.detect(rgb)
                if not detections:
                    self._log("No more parts detected, ending cycle")
                    break

                target = self.detector.select_best_target(detections, depth, frame_center_uv)
                if target is None:
                    for det in detections:
                        self._log(
                            "DepthDebug: {}".format(
                                json.dumps(
                                    {
                                        "bbox": det.bbox,
                                        "center": det.center,
                                        "depth_debug": getattr(det, "depth_debug", {}),
                                        "roi_debug": self.camera.analyze_depth_roi(depth, det.bbox),
                                    },
                                    ensure_ascii=True,
                                )
                            )
                        )
                    self._log("No valid target with depth, skipping")
                    continue

                target = self.detector.refine_pick_point(rgb, target, depth)
                self._log("Target selected: {} @ {}".format(target.label, target.bbox))

                u, v = target.pick_point
                depth_mm, depth_bbox = self.detector.resolve_pick_depth(depth, target)

                if depth_mm <= 0:
                    self._log("No valid depth for target, skipping")
                    continue

                raw_depth_mm = depth_mm
                depth_mm, was_clamped, min_safe_depth_mm = sanitize_camera_depth_mm(
                    depth_mm,
                    config.T_CAM_TO_TCP,
                    margin_below_tcp_m=config.PICK_MIN_DESCENT_M,
                )
                if was_clamped:
                    self._log(
                        "WARNING: depth {:.1f}mm < camera->TCP standoff, clamp -> {:.1f}mm "
                        "de tranh robot di nguoc len".format(
                            raw_depth_mm,
                            min_safe_depth_mm,
                        )
                    )

                self._log(
                    "Target detail: center=({:.1f},{:.1f}), pick=({:.1f},{:.1f}), depth={:.1f}mm, source={}, depth_bbox={}".format(
                        target.center[0],
                        target.center[1],
                        u,
                        v,
                        depth_mm,
                        target.pick_source,
                        depth_bbox,
                    )
                )
                self._log("Target depth: {:.1f}mm".format(depth_mm))
                self._log(
                    "DepthDebugTarget: {}".format(
                        json.dumps(
                            self.camera.analyze_depth_roi(depth, target.bbox),
                            ensure_ascii=True,
                        )
                    )
                )

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
                if intr["reason"]:
                    self._log("IntrinsicsWarning: {}".format(intr["reason"]))
                p_cam = pixel_to_camera_3d(u, v, depth_mm, intr["fx"], intr["fy"], intr["cx"], intr["cy"])
                p_base_raw = camera_to_base(p_cam, self.tcp_pose_at_capture, config.T_CAM_TO_TCP)
                p_base = [
                    p_base_raw[0] + config.PICK_OFFSET_X,
                    p_base_raw[1] + config.PICK_OFFSET_Y,
                    p_base_raw[2] + config.PICK_OFFSET_Z,
                ]
                self._log(
                    "PickOffset: raw_base={}, offset={}, final_base={}".format(
                        [round(float(v), 6) for v in p_base_raw],
                        [
                            round(float(config.PICK_OFFSET_X), 6),
                            round(float(config.PICK_OFFSET_Y), 6),
                            round(float(config.PICK_OFFSET_Z), 6),
                        ],
                        [round(float(v), 6) for v in p_base],
                    )
                )

                approach_z, final_z, _ = clamp_pick_z_sequence(
                    self.tcp_pose_at_capture[2],
                    p_base[2],
                    config.PICK_APPROACH_OFFSET_Z,
                    config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET,
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
                self._validate_pick_motion(
                    self.tcp_pose_at_capture,
                    approach_pose,
                    final_pose,
                )

                self._log("Pick pose: {}".format(approach_pose))

                grip_success = False
                for retry in range(config.MAX_PICK_RETRIES):
                    self._check_abort()

                    if retry > 0:
                        self._log("Retry pick #{}/{}".format(retry + 1, config.MAX_PICK_RETRIES))

                    self._set_phase("moving_to_pick_approach_{}_retry_{}".format(pick_attempt, retry))
                    self._log("Moving to pick approach...")
                    self._move_linear_runtime(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
                    self._wait_steady_default()
                    self._log("At pick approach")

                    self._set_phase("picking_{}_retry_{}".format(pick_attempt, retry))
                    self._log("Descending to part...")
                    self._move_linear_runtime(final_pose, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
                    time.sleep(config.CB3_MOTION_PRE_WAIT_SLEEP_S)
                    self._wait_steady_default()
                    self._log("At part surface")

                    self._log("Gripping part...")
                    try:
                        self._gripper_close()
                        self._log("Part gripped successfully")
                        grip_success = True
                        break
                    except RuntimeError as grip_err:
                        self._log("Grip attempt {} failed: {}".format(retry + 1, grip_err))
                        if retry < config.MAX_PICK_RETRIES - 1:
                            self._log("Retreating to retry...")
                            self._move_linear_runtime(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
                            self._wait_steady_default()
                            self._gripper_open()
                            time.sleep(0.3)
                        else:
                            raise RuntimeError("Grip failed after {} retries".format(config.MAX_PICK_RETRIES))

                if not grip_success:
                    continue

                self._set_phase("retreating_after_pick_{}".format(pick_attempt))
                self._log("Retreating with part...")
                self._move_linear_runtime(approach_pose, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
                self._wait_steady_default()
                self._log("Part retreated safely")

                self._set_phase("moving_to_place_{}".format(pick_attempt))
                self._log("Moving to place position...")
                self._move_linear_runtime(config.PLACE_APPROACH_CART, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
                self._wait_steady_default()
                self._log("At place approach")

                self._log("Descending to conveyor...")
                self._move_linear_runtime(config.PLACE_POINT_CART, accel=config.LINEAR_ACCEL, vel=config.PICK_APPROACH_VEL)
                time.sleep(config.CB3_MOTION_PRE_WAIT_SLEEP_S)
                self._wait_steady_default()
                self._log("At conveyor")

                self._set_phase("placing_{}".format(pick_attempt))
                self._log("Releasing part...")
                self._gripper_open()
                time.sleep(0.3)
                self._log("Part released")

                self._set_phase("retreating_after_place_{}".format(pick_attempt))
                self._log("Retreating from conveyor...")
                self._move_linear_runtime(config.PLACE_RETREAT_CART, accel=config.LINEAR_ACCEL, vel=config.LINEAR_VEL)
                self._wait_steady_default()
                self._log("Safe retreat")

                parts_picked += 1
                self.job_store.update_job(self.job_id, parts_picked=parts_picked)

                self._log("Returning to scan position...")
                self._move_joint_runtime(config.SCAN_APPROACH_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_default()
                self._move_joint_runtime(config.SCAN_POSE_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_default()

            self._set_phase("returning_home")
            self._log("Returning to home...")
            self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
            self._wait_steady_default()
            self._log("At home")

            self._set_phase("done")
            self._log("Cycle complete: {}/{} parts picked".format(parts_picked, parts_found))
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

        except AbortException as err:
            logger.warning("Job %s aborted: %s", self.job_id, err)
            self._set_phase("aborted")
            self._log("Cycle aborted: {}".format(err))

            try:
                self._log("Aborting: opening gripper...")
                self._gripper_open()
                time.sleep(0.3)

                self._log("Aborting: returning to home...")
                self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_cleanup()
            except Exception as cleanup_err:
                logger.error("Error during abort cleanup: %s", cleanup_err)

            self.job_store.update_job(self.job_id, status="aborted")
            raise

        except Exception as err:
            logger.error("Error in pick-place cycle: %s\n%s", err, traceback.format_exc())
            self._set_phase("error")
            self._log("Error: {}".format(err))
            self.job_store.update_job(self.job_id, status="error", error=str(err))

            try:
                logger.info("Attempting cleanup after error...")
                self._gripper_open()
                time.sleep(0.3)
                self._move_joint_runtime(config.HOME_JOINTS, accel=config.JOINT_ACCEL, vel=config.JOINT_VEL)
                self._wait_steady_cleanup()
            except Exception as cleanup_err:
                logger.error("Error during error cleanup: %s", cleanup_err)

            raise

        finally:
            if self._camera_connected:
                try:
                    self.camera.disconnect()
                    self._camera_connected = False
                except Exception as err:
                    logger.error("Camera disconnect error: %s", err)
