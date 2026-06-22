"""Read Orbbec camera intrinsics and save them to camera_intrinsics.json.

Supports both USB and LAN transport.
"""

import argparse
import json
import sys
from pathlib import Path

# Allow running this script via absolute path while still finding repo-local wrappers.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import config

try:
    import ob
    SDK = "ob"
except ImportError:
    try:
        import pyorbbecsdk as ob
        SDK = "pyorbbecsdk"
    except ImportError:
        SDK = None


def get_intrinsics_pyorbbecsdk(transport="auto", ip="", net_port=8090):
    ctx = ob.Context()
    try:
        ctx.enable_net_device_enumeration(True)
    except Exception:
        pass

    device = None
    devices = ctx.query_devices()
    for idx in range(devices.get_count()):
        cand = devices.get_device_by_index(idx)
        try:
            dev_ip = devices.get_device_ip_address_by_index(idx)
        except Exception:
            dev_ip = ""

        is_lan = bool(dev_ip)
        if transport == "usb" and is_lan:
            continue
        if transport == "lan" and not is_lan:
            continue
        if ip and dev_ip and dev_ip != ip:
            continue
        device = cand
        break

    if device is None and transport == "lan" and ip:
        device = ctx.create_net_device(ip, net_port)

    if device is None:
        raise RuntimeError("Không phát hiện thiết bị Orbbec phù hợp")

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


def parse_args():
    parser = argparse.ArgumentParser(description="Read Orbbec camera intrinsics")
    parser.add_argument(
        "--transport",
        choices=["auto", "usb", "lan"],
        default=config.CAMERA_TRANSPORT,
    )
    parser.add_argument("--ip", default=config.CAMERA_IP, help="IP camera khi dùng LAN")
    parser.add_argument("--port", type=int, default=config.CAMERA_NET_PORT, help="Port camera khi dùng LAN")
    return parser.parse_args()


def main():
    args = parse_args()
    if SDK in {"ob", "pyorbbecsdk"}:
        try:
            intr = get_intrinsics_pyorbbecsdk(args.transport, args.ip, args.port)
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

    print("pyorbbecsdk không có, dùng giá trị mặc định Femto Mega:")
    print("  FX=605, FY=605, CX=640, CY=360 (1280x720)")
    print("  Cần cài SDK để lấy giá trị chính xác")
    sys.exit(1)


if __name__ == "__main__":
    main()
