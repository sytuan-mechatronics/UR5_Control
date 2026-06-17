# Chạy: python tools/read_robot_pose.py
# Nhấn Enter để chụp pose hiện tại, Ctrl+C để thoát

import json
import math

import rtde_receive


ROBOT_IP = "192.168.125.11"


def main():
    print(f"Kết nối tới robot {ROBOT_IP}...")
    r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("Kết nối thành công!")
    print("Di chuyển robot đến vị trí cần ghi, nhấn Enter để lưu pose")
    print("Đặt tên cho pose khi được hỏi. Ctrl+C để thoát.\n")

    poses = {}

    while True:
        try:
            input(">>> Nhấn Enter khi robot đã ở đúng vị trí...")

            tcp = r.getActualTCPPose()
            joints = r.getActualQ()

            name = input("Tên pose này (VD: HOME, SCAN_POSE, PLACE_POINT): ").strip()
            if not name:
                print("Bỏ qua (tên trống)")
                continue

            poses[name] = {
                "joints_rad": [round(v, 6) for v in joints],
                "joints_deg": [round(math.degrees(v), 2) for v in joints],
                "tcp_m_rad": [round(v, 6) for v in tcp],
            }

            print(f"\n✓ Đã lưu '{name}':")
            print(f"  Joints (rad): {poses[name]['joints_rad']}")
            print(f"  Joints (deg): {poses[name]['joints_deg']}")
            print(f"  TCP (m,rad):  {poses[name]['tcp_m_rad']}")
            print()

            # Lưu ra file ngay mỗi lần nhấn Enter
            with open("robot_poses.json", "w", encoding="utf-8") as file_obj:
                json.dump(poses, file_obj, ensure_ascii=False, indent=2)
            print("  -> Đã lưu vào robot_poses.json\n")

        except KeyboardInterrupt:
            print("\n\nTất cả poses đã lưu:")
            print(json.dumps(poses, indent=2))
            print("\nFile: robot_poses.json")
            break


if __name__ == "__main__":
    main()
