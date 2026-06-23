"""
Manual Phase 3 pick script — bypass vision, dung toa do tu pick_correction_map.json.

Dung khi:
  - Camera khong on dinh / MiR dock lech nhe
  - Can fallback nhanh de quay video demo

Setup (lam 1 lan truoc khi chay):
  1. Jog robot den SCAN_POSE, sau do ha xuong cham vao mat phoi (bat ky slot)
  2. Doc TCP Z tu teach pendant hoac: python3 tools/read_robot_pose.py
  3. Dat gia tri do duoc vao .env: MANUAL_PART_Z=<gia_tri>
     Vi du: MANUAL_PART_Z=0.020

Chay:
  python3 tools/test_phase3_manual.py              # gap tat ca 5 slot
  python3 tools/test_phase3_manual.py --dry-run    # in toa do, khong di chuyen
  python3 tools/test_phase3_manual.py --slots 1,3  # chi gap slot 1 va 3
  python3 tools/test_phase3_manual.py --slot 2     # test rieng slot 2

Chinh Z neu sai:
  Neu robot ha xuong truot hoac chua cham: dieu chinh MANUAL_PART_Z trong .env
  Giam (vi du tu 0.020 xuong 0.010) de xuong them, tang de bo len.
"""

import argparse
import json
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
from core.pneumatic_gripper import PneumaticGripper

# ---------------------------------------------------------------------------
# Lay MANUAL_PART_Z tu .env
# ---------------------------------------------------------------------------
import os
_MANUAL_PART_Z_STR = os.getenv("MANUAL_PART_Z", "")

MANUAL_PART_Z_CONFIGURED = bool(_MANUAL_PART_Z_STR)
MANUAL_PART_Z = float(_MANUAL_PART_Z_STR) if MANUAL_PART_Z_CONFIGURED else 0.0


# ---------------------------------------------------------------------------
# Doc correction map va tinh toa do pick cho tung slot
# ---------------------------------------------------------------------------

def load_slot_targets():
    """
    Tinh toa do pick day du cho tung slot tu pick_correction_map.json.

    Cong thuc:
        pick_x = map.x + map.dx + config.PICK_OFFSET_X
        pick_y = map.y + map.dy + config.PICK_OFFSET_Y
        part_z = MANUAL_PART_Z + map.dz
        approach_z = part_z + PICK_APPROACH_OFFSET_Z
        touch_z    = part_z + PICK_FINAL_OFFSET_Z + GRASP_Z_OFFSET
    """
    map_path = Path(ROOT) / config.PICK_CORRECTION_MAP_PATH
    if not map_path.exists():
        raise FileNotFoundError(f"Khong tim thay correction map: {map_path}")

    with open(map_path) as f:
        data = json.load(f)

    orient = [config.TOOL_DOWN_RX, config.TOOL_DOWN_RY, config.TOOL_DOWN_RZ]
    slots = []

    for p in data["points"]:
        pick_x = p["x"] + p["dx"] + config.PICK_OFFSET_X
        pick_y = p["y"] + p["dy"] + config.PICK_OFFSET_Y
        part_z = MANUAL_PART_Z + p.get("dz", 0.0)

        approach_z = part_z + config.PICK_APPROACH_OFFSET_Z
        touch_z    = part_z + config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET
        retreat_z  = part_z + config.PICK_RETREAT_OFFSET_Z

        slots.append({
            "name":     p["name"],
            "uv":       (p.get("u", 0), p.get("v", 0)),
            "approach": [pick_x, pick_y, approach_z] + orient,
            "touch":    [pick_x, pick_y, touch_z]    + orient,
            "retreat":  [pick_x, pick_y, retreat_z]  + orient,
            "part_z":   part_z,
        })

    return slots


# ---------------------------------------------------------------------------
# Helpers motion (dung cung cach test_motion.py)
# ---------------------------------------------------------------------------

def move_joints(urscript, joints, accel=None, vel=None):
    urscript.move_joint_with_settings(
        joints,
        tcp_offset=config.TCP_OFFSET,
        payload_kg=config.PAYLOAD_MASS_KG,
        payload_cog=config.PAYLOAD_COG,
        accel=accel or config.JOINT_ACCEL,
        vel=vel or config.JOINT_VEL,
    )


def move_pose(urscript, pose, accel=None, vel=None):
    urscript.move_linear_with_settings(
        pose,
        tcp_offset=config.TCP_OFFSET,
        payload_kg=config.PAYLOAD_MASS_KG,
        payload_cog=config.PAYLOAD_COG,
        accel=accel or config.LINEAR_ACCEL,
        vel=vel or config.LINEAR_VEL,
    )


def wait_steady(rtde, label="", timeout_s=None):
    ok = rtde.wait_steady(
        timeout_s=timeout_s or config.RTDE_WAIT_TIMEOUT,
        threshold=config.RTDE_STEADY_THRESHOLD,
        motion_start_timeout=config.RTDE_MOTION_START_TIMEOUT,
        motion_start_threshold=config.RTDE_MOTION_START_THRESHOLD,
    )
    if not ok:
        print(f"  [WARN] Timeout doi robot {label}")
    return ok


def fmt(pose):
    xyz = [round(v * 1000, 1) for v in pose[:3]]
    return f"XYZ={xyz} mm"


def confirm(msg):
    ans = input(f"\n[XAC NHAN] {msg} — Tiep tuc? [y/N]: ").strip().lower()
    if ans not in ("y", "yes"):
        print("Huy.")
        sys.exit(0)


# ---------------------------------------------------------------------------
# Dry-run: in toa do, khong di chuyen
# ---------------------------------------------------------------------------

def dry_run(slots, slot_ids):
    print("\n=== DRY RUN — khong di chuyen ===")
    if not MANUAL_PART_Z_CONFIGURED:
        print("[WARN] MANUAL_PART_Z chua duoc set trong .env — dang dung Z=0.0 (sai!)")
        print("       Hay do Z that va set: MANUAL_PART_Z=<gia_tri>")
    print(f"\nMANUAL_PART_Z        = {MANUAL_PART_Z:.4f} m ({MANUAL_PART_Z*1000:.1f} mm)")
    print(f"PICK_OFFSET_X        = {config.PICK_OFFSET_X:.4f}")
    print(f"PICK_OFFSET_Y        = {config.PICK_OFFSET_Y:.4f}")
    print(f"PICK_APPROACH_OFFSET = {config.PICK_APPROACH_OFFSET_Z:.3f} m")
    print(f"GRASP_Z_OFFSET       = {config.GRASP_Z_OFFSET:.3f} m")

    for s in slots:
        if s["name"] not in slot_ids:
            continue
        print(f"\n--- {s['name']} ---")
        print(f"  Approach : {fmt(s['approach'])}")
        print(f"  Touch    : {fmt(s['touch'])}")
        print(f"  Retreat  : {fmt(s['retreat'])}")
        desc = s['part_z'] + config.PICK_APPROACH_OFFSET_Z - (s['part_z'] + config.PICK_FINAL_OFFSET_Z + config.GRASP_Z_OFFSET)
        print(f"  Descent  : {desc*1000:.1f} mm (min={config.PICK_MIN_DESCENT_M*1000:.0f} mm)")
        if desc < config.PICK_MIN_DESCENT_M:
            print(f"  [WARN] Descent < min! Kiem tra lai MANUAL_PART_Z.")

    print("\nDry run xong. Them --confirm de chay that.")


# ---------------------------------------------------------------------------
# Chay that: pick 5 slot
# ---------------------------------------------------------------------------

def run_manual_pick(slots, slot_ids, no_confirm=False):
    if not MANUAL_PART_Z_CONFIGURED:
        print("\n[LOI] MANUAL_PART_Z chua duoc set trong .env!")
        print("      Jog robot cham vao mat phoi, doc TCP Z, sau do:")
        print("      echo 'MANUAL_PART_Z=<gia_tri>' >> .env")
        print("      Vi du: MANUAL_PART_Z=0.020")
        sys.exit(1)

    print(f"\nKet noi robot {config.ROBOT_IP}...")
    dashboard = DashboardClient(config.ROBOT_IP, config.DASHBOARD_PORT)
    urscript  = URScriptClient(config.ROBOT_IP, config.URSCRIPT_PORT)
    rtde      = RTDEClient(config.ROBOT_IP, config.RTDE_PORT)
    gripper   = None

    try:
        dashboard.connect()
        urscript.connect()
        rtde.connect()
        print("Robot ket noi OK")

        try:
            gripper = PneumaticGripper(config.GRIPPER_PORT, config.GRIPPER_BAUD)
            gripper.connect()
            print("Gripper ket noi OK")
        except Exception as ge:
            print(f"[WARN] Khong ket noi duoc gripper: {ge}")
            print("       Script tiep tuc nhung KHONG thao tac gripper thuc su.")

        # Precheck + prepare
        status = dashboard.precheck_ready()
        print(f"Robot mode: {status.get('robotmode')}, safety: {status.get('safetystatus')}")
        dashboard.prepare_to_run()
        time.sleep(1.5)  # CB3 brake release settle

        if not no_confirm:
            confirm("Robot da POWER ON + RUNNING. Workspace clear?")

        # Mo gripper truoc
        print("\n[1] Mo gripper...")
        if gripper:
            gripper.open()
            time.sleep(config.GRIPPER_RELEASE_SETTLE_S)

        # HOME
        print("[2] Di chuyen ve HOME...")
        move_joints(urscript, config.HOME_JOINTS)
        wait_steady(rtde, "HOME")

        # SCAN APPROACH
        print("[3] Di chuyen den SCAN_APPROACH...")
        move_joints(urscript, config.SCAN_APPROACH_JOINTS)
        wait_steady(rtde, "SCAN_APPROACH")

        # SCAN POSE
        print("[4] Di chuyen den SCAN_POSE...")
        move_joints(urscript, config.SCAN_POSE_JOINTS)
        wait_steady(rtde, "SCAN_POSE")
        tcp = rtde.get_tcp_pose()
        print(f"    TCP tai scan pose: {fmt(tcp)}")

        parts_picked = 0

        for s in slots:
            if s["name"] not in slot_ids:
                continue

            print(f"\n{'='*50}")
            print(f"  GAP: {s['name']}")
            print(f"  Approach : {fmt(s['approach'])}")
            print(f"  Touch    : {fmt(s['touch'])}")
            print(f"{'='*50}")

            if not no_confirm:
                confirm(f"Chuan bi gap {s['name']}")

            grip_success = False

            for attempt in range(config.MAX_PICK_RETRIES):
                if attempt > 0:
                    print(f"  [Retry {attempt+1}/{config.MAX_PICK_RETRIES}]")

                # Approach
                print(f"  -> Approach...")
                move_pose(urscript, s["approach"], vel=config.LINEAR_VEL)
                wait_steady(rtde, "approach")

                # Descend
                print(f"  -> Ha xuong ({fmt(s['touch'])})...")
                move_pose(urscript, s["touch"], vel=config.PICK_APPROACH_VEL)
                wait_steady(rtde, "touch")

                # Grip
                print(f"  -> Grip...")
                grip_ok = False
                if gripper:
                    try:
                        result = gripper.close()
                        grip_ok = result.get("success", False)
                        print(f"     Grip result: {result}")
                    except Exception as ge:
                        print(f"     [WARN] Grip exception: {ge}")
                else:
                    print("     [WARN] Khong co gripper — gia lap OK")
                    grip_ok = True

                if grip_ok:
                    grip_success = True
                    break
                else:
                    print(f"  -> Grip fail — retreat va thu lai...")
                    move_pose(urscript, s["retreat"], vel=config.LINEAR_VEL)
                    wait_steady(rtde, "retreat after fail")
                    if gripper:
                        gripper.open()
                    time.sleep(0.3)

            if not grip_success:
                print(f"  [WARN] Het {config.MAX_PICK_RETRIES} lan thu — bo qua {s['name']}")
                # Retreat ve scan pose
                move_pose(urscript, s["approach"], vel=config.LINEAR_VEL)
                wait_steady(rtde, "approach after all fail")
                move_joints(urscript, config.SCAN_APPROACH_JOINTS)
                wait_steady(rtde, "SCAN_APPROACH")
                move_joints(urscript, config.SCAN_POSE_JOINTS)
                wait_steady(rtde, "SCAN_POSE")
                continue

            # Retreat
            print(f"  -> Retreat...")
            move_pose(urscript, s["retreat"], vel=config.LINEAR_VEL)
            wait_steady(rtde, "retreat")

            # PLACE
            print(f"  -> Di den PLACE_APPROACH...")
            move_pose(urscript, config.PLACE_APPROACH_CART, vel=config.LINEAR_VEL)
            wait_steady(rtde, "PLACE_APPROACH")

            print(f"  -> Di den PLACE_POINT...")
            move_pose(urscript, config.PLACE_POINT_CART, vel=config.PICK_APPROACH_VEL)
            wait_steady(rtde, "PLACE_POINT")

            print(f"  -> Mo gripper (tha phoi)...")
            if gripper:
                gripper.open()
                time.sleep(config.GRIPPER_RELEASE_SETTLE_S)

            print(f"  -> Retreat place...")
            move_pose(urscript, config.PLACE_RETREAT_CART, vel=config.LINEAR_VEL)
            wait_steady(rtde, "PLACE_RETREAT")

            parts_picked += 1
            print(f"  [OK] {s['name']} gap xong ({parts_picked} phoi da gap)")

            # Quay ve scan pose cho slot tiep theo
            print(f"  -> Quay ve SCAN_APPROACH...")
            move_joints(urscript, config.SCAN_APPROACH_JOINTS)
            wait_steady(rtde, "SCAN_APPROACH")
            print(f"  -> SCAN_POSE...")
            move_joints(urscript, config.SCAN_POSE_JOINTS)
            wait_steady(rtde, "SCAN_POSE")

        # Return HOME
        print(f"\n[XONG] Da gap {parts_picked}/{len(slot_ids)} slot")
        print("Di chuyen ve HOME...")
        move_joints(urscript, config.SCAN_APPROACH_JOINTS)
        wait_steady(rtde, "SCAN_APPROACH")
        move_joints(urscript, config.HOME_JOINTS)
        wait_steady(rtde, "HOME")
        print("Hoan thanh.")

    finally:
        try:
            dashboard.disconnect()
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
        if gripper:
            try:
                gripper.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Manual Phase 3 pick — bypass vision")
    parser.add_argument("--dry-run",   action="store_true", help="In toa do, khong di chuyen")
    parser.add_argument("--slots",     default="",          help="Danh sach slot, vi du: 1,3,5 (mac dinh: tat ca)")
    parser.add_argument("--slot",      type=int, default=0, help="Chi chay 1 slot (1-5)")
    parser.add_argument("--no-confirm",action="store_true", help="Bo qua xac nhan tung buoc (chay lien tuc)")
    args = parser.parse_args()

    slots = load_slot_targets()
    all_names = [s["name"] for s in slots]

    if args.slot:
        slot_name = f"slot_{args.slot}"
        if slot_name not in all_names:
            print(f"[LOI] slot_{args.slot} khong ton tai. Co: {all_names}")
            sys.exit(1)
        selected = [slot_name]
    elif args.slots:
        selected = [f"slot_{n.strip()}" for n in args.slots.split(",")]
        unknown = [s for s in selected if s not in all_names]
        if unknown:
            print(f"[LOI] Khong tim thay: {unknown}. Co: {all_names}")
            sys.exit(1)
    else:
        selected = all_names

    print(f"\nSe xu ly: {selected}")
    print(f"MANUAL_PART_Z = {MANUAL_PART_Z:.4f} m  ({'DA SET' if MANUAL_PART_Z_CONFIGURED else 'CHUA SET — can set truoc khi chay that'})")

    if args.dry_run:
        dry_run(slots, set(selected))
    else:
        run_manual_pick(slots, set(selected), no_confirm=args.no_confirm)


if __name__ == "__main__":
    main()
