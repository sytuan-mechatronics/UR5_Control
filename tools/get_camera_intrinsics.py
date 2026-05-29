"""Read Orbbec camera intrinsics and save them to camera_intrinsics.json.

Run this after connecting the Femto Mega to the PC over USB.
"""

import json
import sys

try:
    import ob
    SDK = "ob"
except ImportError:
    SDK = None


def get_intrinsics_orbbec():
    ctx = ob.Context()
    devices = ctx.query_devices()
    if devices.get_count() == 0:
        raise RuntimeError("Không phát hiện thiết bị Orbbec nào")

    device = devices.get_device_by_index(0)
    sensor = device.get_sensor_list().get_sensor_by_type(ob.OBSensorType.COLOR_SENSOR)
    if sensor is None:
        raise RuntimeError("Không tìm thấy COLOR_SENSOR trên thiết bị Orbbec")

    profile = sensor.get_stream_profile_list().get_default_video_stream_profile()
    intr = profile.get_intrinsic()
    return {
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "cx": float(intr.cx),
        "cy": float(intr.cy),
        "width": int(profile.get_width()),
        "height": int(profile.get_height()),
    }


def main():
    if SDK == "ob":
        try:
            intr = get_intrinsics_orbbec()
        except Exception as exc:
            print(f"[LỖI] Không lấy được intrinsics từ Orbbec SDK: {exc}")
            sys.exit(1)

        print("Camera Intrinsics:")
        print(f"  FX = {intr['fx']:.4f}")
        print(f"  FY = {intr['fy']:.4f}")
        print(f"  CX = {intr['cx']:.4f}")
        print(f"  CY = {intr['cy']:.4f}")
        print(f"  Resolution: {intr['width']}x{intr['height']}")

        with open("camera_intrinsics.json", "w", encoding="utf-8") as file_obj:
            json.dump(intr, file_obj, indent=2)
        print("\nĐã lưu vào camera_intrinsics.json")
        return

    print("Không import được module ob, dùng giá trị mặc định Femto Mega:")
    print("  FX=605, FY=605, CX=640, CY=360 (1280x720)")
    print("  Cần cài SDK để lấy giá trị chính xác")
    sys.exit(1)


if __name__ == "__main__":
    main()
