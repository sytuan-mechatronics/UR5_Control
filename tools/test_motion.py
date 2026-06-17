"""Manual motion test for UR5 joint positions.

Run this with the robot clear, standing beside the E-stop, and confirm each step.
"""

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from robot.dashboard_client import DashboardClient
from robot.rtde_client import RTDEClient
from robot.urscript_client import URScriptClient


ROBOT_IP = config.ROBOT_IP


def fmt_deg(joints):
    return [round(math.degrees(j), 2) for j in joints]


def max_joint_error_deg(current, target):
    return max(abs(math.degrees(current[i] - target[i])) for i in range(6))


def max_tcp_error_mm(current, target):
    return max(abs((current[i] - target[i]) * 1000.0) for i in range(3))


def confirm(message):
    answer = input(f"\n[XÁC NHẬN] {message}\nTiếp tục? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Hủy.")
        raise SystemExit(0)


def wait_steady(rtde, label="", timeout_s=None):
    timeout_s = timeout_s if timeout_s is not None else config.RTDE_WAIT_TIMEOUT
    ok = rtde.wait_steady(
        timeout_s=timeout_s,
        threshold=config.RTDE_STEADY_THRESHOLD,
        motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
    )
    if not ok:
        print(f"  ⚠ Timeout khi chờ {label}")
    else:
        joints = rtde.get_joint_positions()
        print(f"  Dừng tại: {fmt_deg(joints)}°")
    return ok


def ensure_steady_or_confirm(rtde, label: str) -> None:
    ok = wait_steady(rtde, label=label)
    if not ok:
        answer = input("  ⚠ Robot chua dung han. Tiep tuc? [y/N]: ").strip().lower()
        if answer not in ("y", "yes"):
            raise SystemExit(1)


def move_joints(urscript, joints, accel, vel):
    urscript.move_joint(joints, accel=accel, vel=vel)


def move_pose(urscript, pose, accel, vel):
    urscript.move_linear(pose, accel=accel, vel=vel)


def ensure_joint_target_reached(rtde, target, label: str, tol_deg: float = 3.0) -> None:
    current = rtde.get_joint_positions()
    err_deg = max_joint_error_deg(current, target)
    print(f"  Sai so joint {label}: {err_deg:.2f}°")
    if err_deg > tol_deg:
        raise RuntimeError(
            f"{label}: robot khong den dung joint target "
            f"(error={err_deg:.2f}°, tol={tol_deg:.2f}°)"
        )


def ensure_tcp_target_reached(rtde, target, label: str, tol_mm: float = 15.0) -> None:
    current = rtde.get_tcp_pose()
    err_mm = max_tcp_error_mm(current, target)
    print(f"  Sai so TCP {label}: {err_mm:.1f} mm")
    if err_mm > tol_mm:
        raise RuntimeError(
            f"{label}: robot khong den dung TCP target "
            f"(error={err_mm:.1f} mm, tol={tol_mm:.1f} mm)"
        )


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

    status_after = dashboard.precheck_ready()
    print(
        "Dashboard ready:"
        f" mode={status_after['robotmode']},"
        f" safety={status_after['safetystatus']},"
        f" program={status_after['program_state']}"
    )


def main():
    print("Ket noi Dashboard + RTDE + URScript...")
    with DashboardClient(
        ROBOT_IP,
        port=config.DASHBOARD_PORT,
    ) as dashboard, RTDEClient(
        ROBOT_IP,
        frequency=config.RTDE_FREQUENCY,
    ) as rtde, URScriptClient(
        ROBOT_IP,
        port=config.URSCRIPT_PORT,
        timeout=config.URSCRIPT_TIMEOUT,
    ) as urscript:
        print("OK\n")
        prepare_robot_for_motion(dashboard)

        current = rtde.get_joint_positions()
        home = config.HOME_JOINTS
        print("=== Vị trí hiện tại ===")
        print(f"Joints (deg): {fmt_deg(current)}")
        print(f"HOME (deg):   {fmt_deg(home)}")

        diff_deg = max_joint_error_deg(current, home)
        print(f"Lệch HOME: {diff_deg:.2f}°")
        if diff_deg > 10:
            print("\n⚠ Robot KHÔNG ở gần HOME")
            print("  Hãy đưa robot về HOME bằng tay (Freedrive) trước")
            confirm("Robot đã ở vị trí an toàn, workspace clear")
        else:
            confirm("Robot gần HOME, workspace clear, sẵn sàng test")

        print("\n[Test 1] Di chuyển về HOME_JOINTS...")
        confirm("Gửi lệnh HOME")
        move_joints(urscript, config.HOME_JOINTS, config.JOINT_ACCEL, config.JOINT_VEL)
        ensure_steady_or_confirm(rtde, label="HOME")
        ensure_joint_target_reached(rtde, config.HOME_JOINTS, "HOME")
        print("  ✓ Đã về HOME")

        confirm("Tiếp tục → SCAN_APPROACH_JOINTS")
        print("\n[Test 2] Di chuyển đến SCAN_APPROACH...")
        move_joints(
            urscript,
            config.SCAN_APPROACH_JOINTS,
            config.JOINT_ACCEL * 0.5,
            config.JOINT_VEL * 0.5,
        )
        ensure_steady_or_confirm(rtde, label="SCAN_APPROACH")
        ensure_joint_target_reached(rtde, config.SCAN_APPROACH_JOINTS, "SCAN_APPROACH")
        print("  ✓ Đã đến SCAN_APPROACH")

        confirm("Tiếp tục → SCAN_POSE (robot sẽ nhìn xuống khay)")
        print("\n[Test 3] Di chuyển đến SCAN_POSE...")
        move_joints(
            urscript,
            config.SCAN_POSE_JOINTS,
            config.JOINT_ACCEL * 0.3,
            config.JOINT_VEL * 0.3,
        )
        ensure_steady_or_confirm(rtde, label="SCAN_POSE")
        ensure_joint_target_reached(rtde, config.SCAN_POSE_JOINTS, "SCAN_POSE")
        tcp = rtde.get_tcp_pose()
        print(f"  TCP thực tế: {[round(v, 4) for v in tcp]}")
        print(f"  Joint mục tiêu: {fmt_deg(config.SCAN_POSE_JOINTS)}°")
        print("  ✓ Đã đến SCAN_POSE")

        confirm("Tiếp tục → test PLACE_APPROACH_CART")
        print("\n[Test 4] Di chuyển đến PLACE_APPROACH_CART...")
        move_pose(
            urscript,
            config.PLACE_APPROACH_CART,
            config.LINEAR_ACCEL,
            config.LINEAR_VEL,
        )
        ensure_steady_or_confirm(rtde, label="PLACE_APPROACH")
        ensure_tcp_target_reached(rtde, config.PLACE_APPROACH_CART, "PLACE_APPROACH")
        tcp = rtde.get_tcp_pose()
        print(f"  TCP thực tế: {[round(v, 4) for v in tcp]}")
        print(f"  TCP mục tiêu: {[round(v, 4) for v in config.PLACE_APPROACH_CART]}")
        print("  ✓ Đã đến PLACE_APPROACH_CART")

        confirm("Tiếp tục → test PLACE_POINT_CART")
        print("\n[Test 5] Di chuyển đến PLACE_POINT_CART...")
        move_pose(
            urscript,
            config.PLACE_POINT_CART,
            config.LINEAR_ACCEL,
            config.PICK_APPROACH_VEL,
        )
        ensure_steady_or_confirm(rtde, label="PLACE_POINT")
        ensure_tcp_target_reached(rtde, config.PLACE_POINT_CART, "PLACE_POINT")
        tcp = rtde.get_tcp_pose()
        print(f"  TCP thực tế: {[round(v, 4) for v in tcp]}")
        print(f"  TCP mục tiêu: {[round(v, 4) for v in config.PLACE_POINT_CART]}")
        print("  ✓ Đã đến PLACE_POINT_CART")

        confirm("Tiếp tục → test PLACE_RETREAT_CART")
        print("\n[Test 6] Di chuyển đến PLACE_RETREAT_CART...")
        move_pose(
            urscript,
            config.PLACE_RETREAT_CART,
            config.LINEAR_ACCEL,
            config.LINEAR_VEL,
        )
        ensure_steady_or_confirm(rtde, label="PLACE_RETREAT")
        ensure_tcp_target_reached(rtde, config.PLACE_RETREAT_CART, "PLACE_RETREAT")
        tcp = rtde.get_tcp_pose()
        print(f"  TCP thực tế: {[round(v, 4) for v in tcp]}")
        print(f"  TCP mục tiêu: {[round(v, 4) for v in config.PLACE_RETREAT_CART]}")
        print("  ✓ Đã đến PLACE_RETREAT_CART")

        confirm("Robot sẽ quay về HOME với tốc độ vừa phải")
        print("\n[Return HOME] Quay về HOME_JOINTS...")
        move_joints(
            urscript,
            config.HOME_JOINTS,
            config.JOINT_ACCEL * 0.6,
            config.JOINT_VEL * 0.6,
        )
        ensure_steady_or_confirm(rtde, label="HOME return")
        ensure_joint_target_reached(rtde, config.HOME_JOINTS, "HOME return")
        print("  ✓ Đã về HOME")

    print("\n=== Test hoàn thành ===")
    print("Nếu robot đi đúng 6 điểm và tự quay HOME → sequence poses OK")


if __name__ == "__main__":
    main()
