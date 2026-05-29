"""Hand-eye calibration tool using a checkerboard and UR RTDE."""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rtde_receive

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import ob
except ImportError:
    ob = None


ROBOT_IP = "192.168.125.11"
CHECKERBOARD_INNER_CORNERS = (6, 9)  # (cols, rows)
SQUARE_SIZE_M = 0.02  # 20mm
RAW_DATA_PATH = ROOT / "hand_eye_raw_data.json"

from vision.femto_camera import FemtoCamera


class OrbbecColorCamera:
    """Minimal Orbbec color stream reader for calibration snapshots."""

    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self._camera = None

    def open(self):
        if ob is None:
            raise RuntimeError(
                "Khong import duoc module ob/pyorbbecsdk. "
                "Kiem tra lai runtime Orbbec trong repo."
            )
        self._camera = FemtoCamera(width=self.width, height=self.height)
        self._camera.connect()
        return self

    def read(self):
        if self._camera is None:
            return None
        rgb, _depth = self._camera.get_aligned_frames()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def close(self):
        if self._camera is not None:
            try:
                self._camera.disconnect()
            except Exception:
                pass
            finally:
                self._camera = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()


def build_checkerboard_object_points():
    """Build checkerboard 3D points in target frame (Z=0 plane)."""
    cols, rows = CHECKERBOARD_INNER_CORNERS
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid
    objp *= SQUARE_SIZE_M
    return objp


def collect_calibration_data():
    """Collect robot and marker poses for hand-eye calibration."""
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    obj_points = build_checkerboard_object_points()

    with open("camera_intrinsics.json", encoding="utf-8") as file_obj:
        intr = json.load(file_obj)

    K = np.array(
        [[intr["fx"], 0, intr["cx"]],
         [0, intr["fy"], intr["cy"]],
         [0, 0, 1]],
        dtype=np.float64,
    )
    dist = np.zeros((5, 1), dtype=np.float64)

    camera = OrbbecColorCamera(
        width=int(intr.get("width", 1280)),
        height=int(intr.get("height", 720)),
    )
    camera.open()

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []
    raw_samples = []

    pose_count = 0
    print("\nHướng dẫn thu thập data (checkerboard):")
    print("  - Di chuyển robot đến pose mới (Freedrive)")
    print("  - Robot PHẢI đứng yên hoàn toàn trước khi nhấn Enter")
    print("  - Cần tối thiểu 8 pose, lý tưởng là 12-15 pose")
    print("  - Thay đổi NHIỀU: xoay wrist, nghiêng sang trái/phải/trước/sau")
    print("  - Dùng checkerboard inner corners = 6x9, square = 20mm")
    print("  - Không scale/resize ảnh trước khi detect")
    print("  - Ctrl+C khi đủ pose\n")

    try:
        while True:
            input(f"[Pose {pose_count + 1}] Nhấn Enter khi robot đứng yên...")
            time.sleep(0.2)

            frame = camera.read()
            if frame is None:
                print("Không đọc được ảnh camera, thử lại")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                CHECKERBOARD_INNER_CORNERS,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if not found:
                print("Không detect được checkerboard 6x9, bỏ qua pose này")
                cv2.imshow("Calibration View", frame)
                cv2.waitKey(800)
                continue

            refine_criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners_refined = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                refine_criteria,
            )

            ok, rvec, tvec = cv2.solvePnP(obj_points, corners_refined, K, dist)
            if not ok:
                print("solvePnP thất bại, bỏ qua pose này")
                continue

            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec.flatten()

            tcp = rtde_r.getActualTCPPose()
            rvec_tcp = np.array(tcp[3:6], dtype=np.float64)
            R_gripper2base, _ = cv2.Rodrigues(rvec_tcp)
            t_gripper2base = np.array(tcp[0:3], dtype=np.float64)

            R_gripper2base_list.append(R_gripper2base)
            t_gripper2base_list.append(t_gripper2base.reshape(3, 1))
            R_target2cam_list.append(R_target2cam)
            t_target2cam_list.append(t_target2cam.reshape(3, 1))
            raw_samples.append(
                {
                    "pose_index": pose_count + 1,
                    "tcp_pose": [float(v) for v in tcp],
                    "r_gripper2base": R_gripper2base.tolist(),
                    "t_gripper2base": t_gripper2base.tolist(),
                    "r_target2cam": R_target2cam.tolist(),
                    "t_target2cam": t_target2cam.tolist(),
                }
            )

            pose_count += 1

            RAW_DATA_PATH.write_text(
                json.dumps(
                    {
                        "robot_ip": ROBOT_IP,
                        "checkerboard_inner_corners": CHECKERBOARD_INNER_CORNERS,
                        "square_size_m": SQUARE_SIZE_M,
                        "pose_count": pose_count,
                        "samples": raw_samples,
                    },
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )

            frame_draw = frame.copy()
            cv2.drawChessboardCorners(
                frame_draw,
                CHECKERBOARD_INNER_CORNERS,
                corners_refined,
                found,
            )
            cv2.drawFrameAxes(frame_draw, K, dist, rvec, tvec, 0.03)
            cv2.imshow("Calibration View", frame_draw)
            cv2.waitKey(500)

            print(f"  ✓ Pose {pose_count} OK")
            print(f"    Marker trong cam: t={t_target2cam.round(4)}")
            print(f"    TCP trong base:   t={t_gripper2base.round(4)}")
    except KeyboardInterrupt:
        print(f"\nThu thập xong {pose_count} poses")
    finally:
        camera.close()
        cv2.destroyAllWindows()

    if pose_count < 4:
        print("Quá ít pose, cần ít nhất 4 (khuyến nghị 8+)")
        return None

    return (
        R_gripper2base_list,
        t_gripper2base_list,
        R_target2cam_list,
        t_target2cam_list,
    )


def compute_hand_eye(data):
    """Compute hand-eye calibration with several OpenCV methods."""
    R_g2b, t_g2b, R_t2c, t_t2c = data

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    }

    results = {}
    for name, method in methods.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_g2b, t_g2b, R_t2c, t_t2c, method=method
            )
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()
            results[name] = T
            print(f"\nMethod {name}:")
            print(f"  Translation: {t.flatten().round(4)} m")
            print(f"  Rotation:\n{R.round(4)}")
        except Exception as exc:
            print(f"Method {name} thất bại: {exc}")

    return results


def verify_calibration(T_cam_to_tcp, data):
    """Compute consistency error: marker should project to same base-frame position across all poses."""
    R_g2b, t_g2b, _, t_t2c = data
    positions_in_base = []

    for i in range(len(R_g2b)):
        T_base_tcp = np.eye(4)
        T_base_tcp[:3, :3] = R_g2b[i]
        T_base_tcp[:3, 3] = t_g2b[i].flatten()

        p_cam = np.append(t_t2c[i].flatten(), 1)
        p_tcp = T_cam_to_tcp @ p_cam
        p_base = T_base_tcp @ p_tcp
        positions_in_base.append(p_base[:3])

    positions = np.array(positions_in_base)
    mean_pos = positions.mean(axis=0)
    errors = np.linalg.norm(positions - mean_pos, axis=1)

    print(
        f"\nConsistency error (marker in base): mean={np.mean(errors) * 1000:.1f}mm, "
        f"max={np.max(errors) * 1000:.1f}mm"
    )
    print("Tốt nếu mean < 5mm, chấp nhận được nếu < 10mm")


def _is_identity(T, tol=1e-6):
    """Return True if T is effectively the identity matrix (degenerate/failed result)."""
    return np.allclose(T, np.eye(4), atol=tol)


def save_results(results):
    """Save the best hand-eye transform to JSON and print config output."""
    # Prefer PARK then HORAUD then ANDREFF — skip identity (failed) results
    preference = ["PARK", "HORAUD", "ANDREFF", "TSAI"]
    best_name = None
    for name in preference:
        if name in results and not _is_identity(results[name]):
            best_name = name
            break
    if best_name is None:
        best_name = list(results.keys())[0]
        print("\n[WARN] Tất cả methods cho kết quả suy biến — kiểm tra lại data")
    T = results[best_name]

    output = {
        "method": best_name,
        "T_cam_to_tcp": T.tolist(),
        "note": "4x4 homogeneous transform, units: meters",
    }

    with open("hand_eye_result.json", "w", encoding="utf-8") as file_obj:
        json.dump(output, file_obj, indent=2)

    print("\nĐã lưu vào hand_eye_result.json")
    print(f"Raw samples đã lưu tại: {RAW_DATA_PATH}")
    print(f"T_CAM_TO_TCP (method={best_name}):")
    print(T)

    print("\n--- Copy đoạn này vào config.py ---")
    rows = [str([round(value, 6) for value in row]) for row in T.tolist()]
    print("T_CAM_TO_TCP = [")
    for row in rows:
        print(f"    {row},")
    print("]")


if __name__ == "__main__":
    print("=== Hand-Eye Calibration Tool (Checkerboard 6x9, 20mm) ===\n")
    data = collect_calibration_data()
    if data:
        results = compute_hand_eye(data)
        if results:
            save_results(results)
            preference = ["PARK", "HORAUD", "ANDREFF", "TSAI"]
            best_name = next(
                (n for n in preference if n in results and not _is_identity(results[n])),
                list(results.keys())[0],
            )
            verify_calibration(results[best_name], data)
