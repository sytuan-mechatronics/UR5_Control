"""
test_camera_view.py
───────────────────
Xem live stream từ camera Orbbec Femto Mega.
Hiển thị: RGB, Depth colormap, thông tin pixel khi di chuột.

Yêu cầu:
    pip install opencv-python numpy

Orbbec SDK (chọn 1):
    pip install pyorbbecsdk          ← SDK chính thức Python
    pip install open3d               ← có thể dùng thay thế
    Hoặc dùng OpenCV VideoCapture   ← fallback nếu không có SDK

Chạy:
    python test_camera_view.py
    python test_camera_view.py --backend opencv     # ép dùng OpenCV
    python test_camera_view.py --backend orbbec --transport usb
    python test_camera_view.py --backend orbbec --transport lan --ip 192.168.125.10
    python test_camera_view.py --save-dir captures  # lưu ảnh vào thư mục
    python test_camera_view.py --width 1280 --height 720

Phím tắt khi đang chạy:
    s       → chụp và lưu ảnh RGB + Depth
    d       → bật/tắt hiển thị depth map
    i       → bật/tắt overlay thông tin pixel
    h       → bật/tắt histogram depth
    r       → reset colormap depth về mặc định
    1..5    → đổi colormap depth
    q / ESC → thoát
"""

import os
import time
import argparse

import cv2
import numpy as np

import config

# ─────────────────────────────────────────────────────────────────────────────
# Hằng số
# ─────────────────────────────────────────────────────────────────────────────

WINDOW_RGB   = "Femto Mega — RGB (nhấn H để xem phím tắt)"
WINDOW_DEPTH = "Femto Mega — Depth"

DEPTH_COLORMAPS = {
    "1": (cv2.COLORMAP_JET,      "JET"),
    "2": (cv2.COLORMAP_TURBO,    "TURBO"),
    "3": (cv2.COLORMAP_PLASMA,   "PLASMA"),
    "4": (cv2.COLORMAP_HOT,      "HOT"),
    "5": (cv2.COLORMAP_VIRIDIS,  "VIRIDIS"),
}
DEFAULT_COLORMAP_KEY = "2"

# Depth hợp lệ cho Femto Mega (mm)
DEPTH_MIN_MM = 300
DEPTH_MAX_MM = 4000

# ─────────────────────────────────────────────────────────────────────────────
# Backend Orbbec SDK
# ─────────────────────────────────────────────────────────────────────────────

class OrbbecBackend:
    """Dùng pyorbbecsdk để lấy RGB + Depth aligned."""

    NAME = "pyorbbecsdk"

    def __init__(
        self,
        width=1280,
        height=720,
        fps=30,
        ip="",
        transport="auto",
        net_port=None,
    ):
        import pyorbbecsdk as ob          # noqa: import bên trong để lazy
        self._ob   = ob
        self.width  = width
        self.height = height
        self.fps    = fps
        self._ip = (ip or "").strip()
        self.transport = (transport or "auto").strip().lower()
        self.net_port = int(net_port if net_port is not None else config.CAMERA_NET_PORT)
        self._pipeline = None
        self._config   = None
        self.intrinsics = {}
        self._selected_net_port = None
        self.last_frame_wall_time = 0.0
        self.frame_counter = 0
        self.active_transport = self.transport
        self.color_format_name = ""
        self.wait_timeout_ms = int(config.CAMERA_USB_WAIT_TIMEOUT_MS)
        self.frame_retries = int(config.CAMERA_USB_FRAME_RETRIES)

    def _candidate_net_ports(self):
        ports = [self.net_port]
        if 8090 not in ports:
            ports.append(8090)
        return ports

    def _query_devices(self, ctx):
        try:
            ctx.enable_net_device_enumeration(True)
            time.sleep(0.5)
        except Exception:
            pass
        return ctx.query_devices()

    def _is_lan_mode(self):
        return self.active_transport == "lan"

    def _preferred_color_formats(self):
        if self._is_lan_mode():
            return list(config.CAMERA_LAN_COLOR_FORMATS)
        return list(config.CAMERA_USB_COLOR_FORMATS)

    def _pick_color_profile(self, profile_list):
        """Pick the best available color profile for current SDK/device."""
        ob = self._ob
        preferred_formats = self._preferred_color_formats()

        for fmt_name in preferred_formats:
            fmt = getattr(ob.OBFormat, fmt_name, None)
            if fmt is None:
                continue
            try:
                profile = profile_list.get_video_stream_profile(
                    self.width, self.height, fmt, self.fps
                )
                print(f"  Color fmt: {fmt_name}")
                return profile
            except Exception:
                continue

        # Fallback to default stream profile if requested profile not found
        profile = profile_list.get_default_video_stream_profile()
        try:
            fmt_name = profile.get_format().name
        except Exception:
            fmt_name = str(profile.get_format())
        print(f"  Color fmt: {fmt_name} (default profile)")
        return profile

    def _pick_depth_profile(self, profile_list):
        """Pick the best available depth profile for current SDK/device."""
        ob = self._ob
        y16 = getattr(ob.OBFormat, "Y16", None)
        if y16 is not None:
            try:
                return profile_list.get_video_stream_profile(
                    self.width, self.height, y16, self.fps
                )
            except Exception:
                pass
        return profile_list.get_default_video_stream_profile()

    def _frame_to_bgr(self, color_frame):
        """Convert Orbbec color frame to OpenCV BGR image."""
        ob = self._ob
        h = color_frame.get_height()
        w = color_frame.get_width()
        raw = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
        fmt = color_frame.get_format()

        if fmt == getattr(ob.OBFormat, "RGB", object()):
            rgb = raw.reshape((h, w, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if fmt == getattr(ob.OBFormat, "BGR", object()):
            return raw.reshape((h, w, 3))

        if fmt == getattr(ob.OBFormat, "MJPG", object()):
            img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError("Decode MJPG frame failed")
            return img

        if fmt == getattr(ob.OBFormat, "YUYV", object()) or fmt == getattr(ob.OBFormat, "YUY2", object()):
            yuyv = raw.reshape((h, w, 2))
            return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

        if fmt == getattr(ob.OBFormat, "NV12", object()):
            nv12 = raw.reshape((h * 3 // 2, w))
            return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

        if fmt == getattr(ob.OBFormat, "NV21", object()):
            nv21 = raw.reshape((h * 3 // 2, w))
            return cv2.cvtColor(nv21, cv2.COLOR_YUV2BGR_NV21)

        if fmt == getattr(ob.OBFormat, "I420", object()):
            i420 = raw.reshape((h * 3 // 2, w))
            return cv2.cvtColor(i420, cv2.COLOR_YUV2BGR_I420)

        # Generic fallback for unrecognized format: try 3-channel packed RGB/BGR.
        if raw.size == h * w * 3:
            packed = raw.reshape((h, w, 3))
            return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)

        raise RuntimeError(f"Unsupported color frame format: {fmt}")

    def open(self):
        ob = self._ob
        ctx = ob.Context()
        devices = self._query_devices(ctx)
        if devices.get_count() == 0 and self.transport != "lan":
            raise RuntimeError("Không tìm thấy thiết bị Orbbec nào")

        selected_device = None
        selected_info = None

        for idx in range(devices.get_count()):
            device = devices.get_device_by_index(idx)
            info = device.get_device_info()
            try:
                dev_ip = devices.get_device_ip_address_by_index(idx)
            except Exception:
                dev_ip = ""
            try:
                conn = str(devices.get_device_connection_type_by_index(idx))
            except Exception:
                conn = "unknown"

            is_lan = bool(dev_ip)
            if self.transport == "usb" and is_lan:
                continue
            if self.transport == "lan" and not is_lan:
                continue
            if self._ip and dev_ip and dev_ip != self._ip:
                continue

            selected_device = device
            selected_info = (info, dev_ip, conn)
            break

        if selected_device is None and self.transport == "lan" and self._ip:
            last_exc = None
            for port in self._candidate_net_ports():
                try:
                    selected_device = ctx.create_net_device(self._ip, port)
                    self._selected_net_port = port
                    selected_info = (None, self._ip, "LAN")
                    break
                except Exception as exc:
                    last_exc = exc

            if selected_device is None:
                tried = ", ".join(str(p) for p in self._candidate_net_ports())
                raise RuntimeError(
                    f"Không mở được camera LAN tại {self._ip} "
                    f"(đã thử port: {tried}): {last_exc}"
                ) from last_exc

        if selected_device is None:
            raise RuntimeError(
                f"Không tìm thấy camera theo transport={self.transport!r}"
                + (f", ip={self._ip}" if self._ip else "")
            )

        if selected_info and selected_info[0] is not None:
            info, dev_ip, conn = selected_info
            print(f"  Thiết bị : {info.get_name()}")
            print(f"  Serial   : {info.get_serial_number()}")
            print(f"  Firmware : {info.get_firmware_version()}")
            print(f"  Kết nối  : {conn}")
            if dev_ip:
                print(f"  IP       : {dev_ip}")
        else:
            shown_port = self._selected_net_port or self.net_port
            print(f"  Thiết bị : Orbbec LAN @ {self._ip}:{shown_port}")

        self.active_transport = "lan" if (selected_info and selected_info[1]) or self.transport == "lan" else "usb"
        self.wait_timeout_ms = (
            int(config.CAMERA_LAN_WAIT_TIMEOUT_MS)
            if self._is_lan_mode()
            else int(config.CAMERA_USB_WAIT_TIMEOUT_MS)
        )
        self.frame_retries = (
            int(config.CAMERA_LAN_FRAME_RETRIES)
            if self._is_lan_mode()
            else int(config.CAMERA_USB_FRAME_RETRIES)
        )
        if self._is_lan_mode():
            self.fps = int(config.CAMERA_LAN_FPS)
            print(f"  LAN mode: dung {self.fps}fps profile (CAMERA_LAN_FPS)")

        self._pipeline = ob.Pipeline(selected_device)
        self._config   = ob.Config()

        # Bật stream màu và depth từ profile list để tương thích nhiều SDK version.
        color_profiles = self._pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        color_profile = self._pick_color_profile(color_profiles)
        try:
            self.color_format_name = color_profile.get_format().name
        except Exception:
            self.color_format_name = "unknown"
        self._config.enable_stream(color_profile)

        depth_profiles = self._pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
        depth_profile = self._pick_depth_profile(depth_profiles)
        self._config.enable_stream(depth_profile)

        # Align depth → color
        align_mode = getattr(ob.OBAlignMode, "HW_MODE", None)
        if align_mode is not None:
            self._config.set_align_mode(align_mode)

        # Skip FULL_FRAME_REQUIRE on LAN: waits for both streams simultaneously,
        # causing multi-second stalls when color/depth packets arrive out of sync.
        if not self._is_lan_mode():
            try:
                agg_mode = getattr(ob, "OBFrameAggregateOutputMode", None)
                if agg_mode is not None and hasattr(agg_mode, "FULL_FRAME_REQUIRE"):
                    self._config.set_frame_aggregate_output_mode(agg_mode.FULL_FRAME_REQUIRE)
            except Exception:
                pass

        self._pipeline.start(self._config)

        # Lấy intrinsics màu
        try:
            profile  = (self._pipeline
                        .get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
                        .get_default_video_stream_profile())
            intr = profile.get_intrinsic()
            self.intrinsics = {
                "fx": float(intr.fx),
                "fy": float(intr.fy),
                "cx": float(intr.cx),
                "cy": float(intr.cy),
                "width":  self.width,
                "height": self.height,
            }
        except Exception:
            self.intrinsics = {}

        return self

    def read(self):
        """
        Trả về (rgb_bgr, depth_mm) hoặc (None, None).
        rgb_bgr   : numpy array (H, W, 3) uint8
        depth_mm  : numpy array (H, W)    uint16, đơn vị mm
        """
        frameset = None
        color_frame = None
        depth_frame = None

        for _ in range(self.frame_retries):
            frameset = self._pipeline.wait_for_frames(self.wait_timeout_ms)
            if frameset is None:
                continue

            color_frame = frameset.get_color_frame()
            depth_frame = frameset.get_depth_frame()
            if color_frame is not None and depth_frame is not None:
                break

        if frameset is None or color_frame is None or depth_frame is None:
            return None, None

        bgr = self._frame_to_bgr(color_frame)

        # Depth (mm)
        depth_data = np.frombuffer(
            depth_frame.get_data(), dtype=np.uint16
        ).reshape((depth_frame.get_height(),
                   depth_frame.get_width()))

        # Scale nếu cần (Femto Mega đôi khi trả về 0.1mm đơn vị)
        scale = depth_frame.get_depth_scale()
        if scale and abs(scale - 1.0) > 0.01:
            depth_data = (depth_data.astype(np.float32) * scale).astype(np.uint16)

        self.last_frame_wall_time = time.time()
        self.frame_counter += 1

        return bgr, depth_data

    def close(self):
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            finally:
                self._pipeline = None
                self._config = None

    def reopen(self):
        self.close()
        time.sleep(0.3)
        self.open()
        return self

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()


class OpenCVBackend:
    """Fallback backend using default webcam via OpenCV."""

    NAME = "opencv"

    def __init__(self, width=1280, height=720):
        self.width = width
        self.height = height
        self.intrinsics = {}
        self._cap = None

    def open(self):
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            raise RuntimeError("Không mở được camera qua OpenCV VideoCapture(0)")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        print("  Thiết bị : OpenCV camera index 0")
        return self

    def read(self):
        if self._cap is None:
            return None, None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None, None
        depth_dummy = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint16)
        return frame, depth_dummy

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()


# ─────────────────────────────────────────────────────────────────────────────
# Hàm chọn backend tự động
# ─────────────────────────────────────────────────────────────────────────────

def _orbbec_diagnostics():
    """Collect detailed Orbbec SDK/device diagnostics for clear error reporting."""
    result = {
        "sdk_installed": False,
        "device_count": 0,
        "devices": [],
        "error": None,
    }
    try:
        import pyorbbecsdk as ob
        result["sdk_installed"] = True
        ctx = ob.Context()
        try:
            ctx.enable_net_device_enumeration(True)
            time.sleep(0.5)
        except Exception:
            pass
        devices = ctx.query_devices()
        count = devices.get_count()
        result["device_count"] = count

        for idx in range(count):
            try:
                dev = devices.get_device_by_index(idx)
                info = dev.get_device_info()
                result["devices"].append({
                    "name": info.get_name(),
                    "serial": info.get_serial_number(),
                    "firmware": info.get_firmware_version(),
                    "ip": getattr(devices, "get_device_ip_address_by_index", lambda *_: "")(idx),
                })
            except Exception as exc:
                result["devices"].append({"error": str(exc)})

    except ImportError:
        result["error"] = "pyorbbecsdk_not_installed"
    except Exception as exc:
        result["error"] = str(exc)

    return result


def auto_select_backend(force=None, width=1280, height=720, ip="", transport="auto", net_port=None):
    if transport == "lan":
        if not ip:
            raise RuntimeError("Transport LAN yêu cầu --ip")
        shown_port = int(net_port if net_port is not None else config.CAMERA_NET_PORT)
        print(f"[Backend] LAN mode: {ip}:{shown_port}")
        return OrbbecBackend(width, height, fps=config.CAMERA_FPS, ip=ip, transport="lan", net_port=net_port)
    if transport == "usb":
        print("[Backend] USB mode")
        if force == "opencv":
            return OpenCVBackend(width, height)
        return OrbbecBackend(width, height, fps=config.CAMERA_FPS, transport="usb")

    # ── Ép buộc backend cụ thể ────────────────────────────────────
    if force == "opencv":
        print("[Backend] Ép dùng OpenCV VideoCapture")
        return OpenCVBackend(width, height)

    if force == "orbbec":
        print("[Backend] Ép dùng pyorbbecsdk (Orbbec SDK)")
        return OrbbecBackend(width, height, fps=config.CAMERA_FPS, ip=ip, transport="auto", net_port=net_port)

    # ── Tự động detect ────────────────────────────────────────────
    diag = _orbbec_diagnostics()
    count = diag["device_count"]

    if not diag["sdk_installed"]:
        print("[WARN] Chưa cài SDK Orbbec (pyorbbecsdk) → fallback OpenCV")
        print()
        print("Cài SDK nếu chưa có:")
        print("  pip install pyorbbecsdk")
        print()
        return OpenCVBackend(width, height)

    if count == 0:
        print("[WARN] Không phát hiện thiết bị Orbbec Femto Mega → fallback OpenCV")
        if diag["error"]:
            print(f"  Chi tiết SDK: {diag['error']}")
        print()
        print("Kiểm tra:")
        print("  • Femto Mega đã cắm cáp USB3 chưa?")
        print("  • Đèn nguồn camera có sáng không?")
        print("  • Linux: sudo chmod 666 /dev/bus/usb/*/*")
        print("           lsusb | grep -i orbbec")
        print("  • Windows: Device Manager -> tìm \"Orbbec\"")
        print()
        print("Cài SDK nếu chưa có:")
        print("  pip install pyorbbecsdk")
        print()
        return OpenCVBackend(width, height)

    # Tìm thấy thiết bị Orbbec
    print(f"[Backend] Phát hiện {count} thiết bị Orbbec → dùng Orbbec SDK")
    for idx, dev in enumerate(diag["devices"], start=1):
        if "error" in dev:
            print(f"  [{idx}] <error reading device info>: {dev['error']}")
        else:
            print(
                f"  [{idx}] {dev['name']} | SN={dev['serial']} | FW={dev['firmware']} | IP={dev.get('ip','')}"
            )
    return OrbbecBackend(width, height, fps=config.CAMERA_FPS, ip=ip, transport="auto", net_port=net_port)


# ─────────────────────────────────────────────────────────────────────────────
# Xử lý depth map
# ─────────────────────────────────────────────────────────────────────────────

def depth_to_colormap(depth_mm, colormap_id, min_mm=DEPTH_MIN_MM, max_mm=DEPTH_MAX_MM):
    """
    Chuyển depth (uint16, mm) → ảnh màu để hiển thị.
    Vùng depth=0 (hole) hiển thị màu đen.
    """
    mask_valid = (depth_mm > min_mm) & (depth_mm < max_mm)

    # Normalize về [0, 255]
    depth_f = depth_mm.astype(np.float32)
    depth_f = np.clip(depth_f, min_mm, max_mm)
    depth_norm = ((depth_f - min_mm) / (max_mm - min_mm) * 255).astype(np.uint8)

    colored = cv2.applyColorMap(depth_norm, colormap_id)

    # Vùng không hợp lệ → đen
    colored[~mask_valid] = 0

    return colored


def depth_histogram(depth_mm, width=320, height=100):
    """Vẽ histogram phân phối depth."""
    valid = depth_mm[(depth_mm > DEPTH_MIN_MM) & (depth_mm < DEPTH_MAX_MM)]
    if len(valid) == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)

    hist, edges = np.histogram(valid, bins=64,
                               range=(DEPTH_MIN_MM, DEPTH_MAX_MM))
    hist_img = np.zeros((height, width, 3), dtype=np.uint8)
    max_val  = hist.max() if hist.max() > 0 else 1

    bar_w = width // len(hist)
    for i, val in enumerate(hist):
        bar_h = int(val / max_val * (height - 20))
        x1 = i * bar_w
        x2 = x1 + bar_w - 1
        cv2.rectangle(hist_img,
                      (x1, height - bar_h - 1),
                      (x2, height - 1),
                      (0, 200, 255), -1)

    # Label min/max
    cv2.putText(hist_img, f"{DEPTH_MIN_MM}mm", (2, height - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    cv2.putText(hist_img, f"{DEPTH_MAX_MM}mm",
                (width - 60, height - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    return hist_img


# ─────────────────────────────────────────────────────────────────────────────
# Overlay thông tin lên ảnh
# ─────────────────────────────────────────────────────────────────────────────

def draw_overlay_rgb(frame, mouse_uv, depth_mm, show_info,
                     fps, backend_name, intrinsics, extra_lines=None):
    """Vẽ crosshair + thông tin lên ảnh RGB."""
    h, w = frame.shape[:2]
    out  = frame.copy()

    # Crosshair tại vị trí chuột
    u, v = mouse_uv
    if 0 <= u < w and 0 <= v < h:
        cv2.line(out, (u - 15, v), (u + 15, v), (0, 255, 0), 1)
        cv2.line(out, (u, v - 15), (u, v + 15), (0, 255, 0), 1)
        cv2.circle(out, (u, v), 5, (0, 255, 0), 1)

        if show_info:
            # Giá trị pixel RGB
            b, g, r = out[v, u]
            pixel_text = f"RGB({r},{g},{b})"
            cv2.putText(out, pixel_text, (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Depth tại pixel đó (nếu có)
            if depth_mm is not None:
                d = int(depth_mm[v, u])
                depth_text = (f"Depth: {d}mm ({d/1000:.3f}m)"
                              if d > 0 else "Depth: ---")
                cv2.putText(out, depth_text, (u + 10, v + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                # Tọa độ 3D nếu có intrinsics
                if intrinsics and d > 0:
                    fx = intrinsics.get("fx", 605)
                    fy = intrinsics.get("fy", 605)
                    cx = intrinsics.get("cx", w / 2)
                    cy = intrinsics.get("cy", h / 2)
                    dm = d / 1000.0
                    X  = (u - cx) * dm / fx
                    Y  = (v - cy) * dm / fy
                    Z  = dm
                    xyz_text = f"3D: ({X:.3f}, {Y:.3f}, {Z:.3f}) m"
                    cv2.putText(out, xyz_text, (u + 10, v + 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (255, 200, 0), 1)

    if show_info:
        # Header thông tin chung
        lines = [
            f"Backend: {backend_name}",
            f"Reso: {w}x{h}",
            f"FPS: {fps:.1f}",
        ]
        if intrinsics:
            lines.append(
                f"fx={intrinsics.get('fx',0):.1f}  "
                f"fy={intrinsics.get('fy',0):.1f}  "
                f"cx={intrinsics.get('cx',0):.1f}  "
                f"cy={intrinsics.get('cy',0):.1f}"
            )
        if extra_lines:
            lines.extend(extra_lines)
        for i, line in enumerate(lines):
            cv2.putText(out, line, (8, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 0), 3)
            cv2.putText(out, line, (8, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (220, 220, 220), 1)

    # Hướng dẫn phím tắt (góc dưới)
    shortcuts = "S:lưu  D:depth  I:info  H:hist  1-5:colormap  Q:thoát"
    cv2.putText(out, shortcuts, (8, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
    cv2.putText(out, shortcuts, (8, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    return out


# ─────────────────────────────────────────────────────────────────────────────
# Lưu ảnh
# ─────────────────────────────────────────────────────────────────────────────

def save_frame(rgb_bgr, depth_mm, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")

    # Lưu RGB
    rgb_path = os.path.join(save_dir, f"rgb_{ts}.jpg")
    cv2.imwrite(rgb_path, rgb_bgr,
                [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f"  Đã lưu RGB  : {rgb_path}")

    # Lưu Depth (PNG 16-bit để giữ giá trị mm)
    if depth_mm is not None:
        depth_path = os.path.join(save_dir, f"depth_{ts}.png")
        cv2.imwrite(depth_path, depth_mm)
        print(f"  Đã lưu Depth: {depth_path}")

        # Thống kê depth
        valid = depth_mm[(depth_mm > DEPTH_MIN_MM) & (depth_mm < DEPTH_MAX_MM)]
        if len(valid) > 0:
            print(f"  Depth stats : min={valid.min()}mm  "
                  f"mean={int(valid.mean())}mm  max={valid.max()}mm  "
                  f"holes={np.sum(depth_mm == 0)}")

    return rgb_path


# ─────────────────────────────────────────────────────────────────────────────
# Vòng lặp chính
# ─────────────────────────────────────────────────────────────────────────────

def run(args):
    backend = auto_select_backend(
        force  = args.backend,
        width  = args.width,
        height = args.height,
        ip     = args.ip,
        transport = args.transport,
        net_port = args.port,
    )

    print(f"\nĐang mở camera ({backend.NAME})...")
    try:
        backend.open()
    except Exception as exc:
        print(f"\n[LỖI] Không mở được camera:\n  {exc}")
        print("\nGợi ý debug:")
        if args.transport == "lan":
            print(f"  • Kiểm tra ping camera: ping {args.ip}")
            print(f"  • Kiểm tra TCP port: nc -vz {args.ip} {args.port}")
            print("  • Kiểm tra PC và camera cùng subnet")
            print("  • Kiểm tra camera đã ở mode Ethernet và đúng IP")
            print("  • Nếu camera không mở được dù ping OK: ping chỉ xác nhận ICMP, chưa xác nhận Orbbec SDK service")
        else:
            print("  • Kiểm tra USB đã cắm chưa: lsusb | grep -i orbbec")
            print("  • Kiểm tra quyền USB (Linux): sudo chmod 666 /dev/bus/usb/*/*")
        print("  • Thử backend khác: python test_camera_view.py --backend opencv")
        return

    print("Camera OK — đang hiển thị...\n")

    # Trạng thái UI
    show_depth     = True
    show_info      = True
    show_histogram = False
    mouse_uv       = (0, 0)

    colormap_key = DEFAULT_COLORMAP_KEY
    colormap_id, colormap_name = DEPTH_COLORMAPS[colormap_key]

    # FPS tracking
    fps_times = []
    fps       = 0.0

    # Mouse callback
    def on_mouse(event, x, y, flags, param):
        nonlocal mouse_uv
        mouse_uv = (x, y)

    cv2.namedWindow(WINDOW_RGB,   cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_RGB,  min(args.width,  1280),
                                  min(args.height,  720))
    mouse_callback_ready = False
    mouse_callback_failed = False

    if show_depth:
        cv2.namedWindow(WINDOW_DEPTH, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_DEPTH, min(args.width,  1280),
                                       min(args.height,  720))

    last_rgb_bgr  = None
    last_depth_mm = None
    stale_frame_count = 0
    reconnect_attempts = 0

    try:
        while True:
            rgb_bgr, depth_mm = backend.read()

            if rgb_bgr is None:
                # Frame drop — dùng frame cũ
                if last_rgb_bgr is None:
                    time.sleep(0.03)
                    continue
                stale_frame_count += 1
                rgb_bgr  = last_rgb_bgr
                depth_mm = last_depth_mm
            else:
                stale_frame_count = 0
                last_rgb_bgr  = rgb_bgr
                last_depth_mm = depth_mm

            # ── FPS ──────────────────────────────────────────────────
            t1 = time.perf_counter()
            fps_times.append(t1)
            fps_times = [t for t in fps_times if t1 - t < 2.0]
            fps = len(fps_times) / 2.0

            stale_ms = 0
            last_ts = getattr(backend, "last_frame_wall_time", 0.0)
            if last_ts > 0:
                stale_ms = int((time.time() - last_ts) * 1000.0)

            if stale_ms >= args.reconnect_stale_ms:
                reconnect_attempts += 1
                print(
                    f"[WARN] Stream stale {stale_ms} ms, thu reconnect "
                    f"{reconnect_attempts}..."
                )
                try:
                    backend.reopen()
                    stale_frame_count = 0
                    last_rgb_bgr = None
                    last_depth_mm = None
                    fps_times = []
                    print("[INFO] Reconnect camera OK")
                    continue
                except Exception as exc:
                    print(f"[WARN] Reconnect camera FAIL: {exc}")
                    time.sleep(args.reconnect_delay_s)

            extra_lines = [
                f"Frames: {getattr(backend, 'frame_counter', 0)}",
                f"Reconnects: {reconnect_attempts}",
                f"Transport active: {getattr(backend, 'active_transport', backend.NAME)}",
                f"Color fmt: {getattr(backend, 'color_format_name', '?')}",
            ]
            if stale_frame_count > 0:
                extra_lines.append(f"STALE frame: {stale_ms} ms")

            # ── Vẽ RGB ───────────────────────────────────────────────
            rgb_out = draw_overlay_rgb(
                rgb_bgr, mouse_uv, depth_mm,
                show_info, fps,
                backend.NAME, backend.intrinsics,
                extra_lines=extra_lines,
            )
            cv2.imshow(WINDOW_RGB, rgb_out)

            if not mouse_callback_ready and not mouse_callback_failed:
                try:
                    cv2.setMouseCallback(WINDOW_RGB, on_mouse)
                    mouse_callback_ready = True
                except cv2.error:
                    # Một số backend GUI không trả window handle hợp lệ cho callback.
                    mouse_callback_failed = True
                    print("[WARN] Không gắn được mouse callback, tiếp tục chạy không có tracking chuột.")

            # ── Vẽ Depth ─────────────────────────────────────────────
            if show_depth:
                depth_colored = depth_to_colormap(
                    depth_mm, colormap_id)

                # Crosshair trên depth
                u, v = mouse_uv
                h_d, w_d = depth_colored.shape[:2]
                if 0 <= u < w_d and 0 <= v < h_d:
                    cv2.line(depth_colored,
                             (u-15,v), (u+15,v), (255,255,255), 1)
                    cv2.line(depth_colored,
                             (u,v-15), (u,v+15), (255,255,255), 1)
                    d_val = int(depth_mm[v, u])
                    label = f"{d_val}mm" if d_val > 0 else "---"
                    cv2.putText(depth_colored, label,
                                (u+8, v-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255), 1)

                # Label colormap
                cv2.putText(depth_colored,
                            f"Colormap: {colormap_name}  "
                            f"[{DEPTH_MIN_MM}-{DEPTH_MAX_MM}mm]",
                            (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 0, 0), 3)
                cv2.putText(depth_colored,
                            f"Colormap: {colormap_name}  "
                            f"[{DEPTH_MIN_MM}-{DEPTH_MAX_MM}mm]",
                            (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (220, 220, 220), 1)

                # Histogram
                if show_histogram:
                    hist_img = depth_histogram(depth_mm)
                    h_d2, w_d2 = depth_colored.shape[:2]
                    hw, ww = hist_img.shape[:2]
                    y1 = h_d2 - hw - 8
                    x1 = 8
                    depth_colored[y1:y1+hw, x1:x1+ww] = hist_img

                cv2.imshow(WINDOW_DEPTH, depth_colored)

            # ── Xử lý phím ───────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), 27):          # Q hoặc ESC
                print("Thoát.")
                break

            elif key == ord('s'):              # Lưu ảnh
                save_frame(rgb_bgr, depth_mm, args.save_dir)

            elif key == ord('d'):              # Bật/tắt depth
                show_depth = not show_depth
                if show_depth:
                    cv2.namedWindow(WINDOW_DEPTH, cv2.WINDOW_NORMAL)
                else:
                    cv2.destroyWindow(WINDOW_DEPTH)
                print(f"Depth: {'ON' if show_depth else 'OFF'}")

            elif key == ord('i'):              # Bật/tắt info
                show_info = not show_info
                print(f"Info overlay: {'ON' if show_info else 'OFF'}")

            elif key == ord('h'):              # Bật/tắt histogram
                show_histogram = not show_histogram
                print(f"Histogram: {'ON' if show_histogram else 'OFF'}")

            elif key == ord('r'):              # Reset colormap
                colormap_key  = DEFAULT_COLORMAP_KEY
                colormap_id, colormap_name = DEPTH_COLORMAPS[colormap_key]
                print(f"Colormap reset: {colormap_name}")

            elif chr(key) in DEPTH_COLORMAPS: # Đổi colormap (1-5)
                colormap_key  = chr(key)
                colormap_id, colormap_name = DEPTH_COLORMAPS[colormap_key]
                print(f"Colormap: {colormap_name}")

    except KeyboardInterrupt:
        print("\nCtrl+C — thoát.")
    finally:
        backend.close()
        cv2.destroyAllWindows()

    # In intrinsics lần cuối để tiện copy
    if backend.intrinsics:
        print("\n─── Camera Intrinsics (copy vào config.py) ───")
        for k, v in backend.intrinsics.items():
            print(f"  {k.upper():8s} = {v}")
        print()

        # Lưu ra file JSON
        import json
        path = "camera_intrinsics.json"
        with open(path, "w") as f:
            json.dump(backend.intrinsics, f, indent=2)
        print(f"  Đã lưu intrinsics → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Live view camera Orbbec Femto Mega (RGB + Depth)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--backend",
                   choices=["auto", "orbbec", "opencv"],
                   default="auto",
                   help="Backend camera (mặc định: auto)")
    p.add_argument("--width",  type=int, default=1280,
                   help="Chiều rộng frame (mặc định: 1280)")
    p.add_argument("--height", type=int, default=720,
                   help="Chiều cao frame (mặc định: 720)")
    p.add_argument("--save-dir", default="captures",
                   help="Thư mục lưu ảnh khi nhấn S (mặc định: captures/)")
    p.add_argument(
        "--ip",
        default="",
        help="IP camera khi kết nối LAN (vd: 192.168.125.10)",
    )
    p.add_argument(
        "--transport",
        choices=["auto", "usb", "lan"],
        default="auto",
        help="Chọn nhánh kết nối camera",
    )
    p.add_argument(
        "--port",
        type=int,
        default=config.CAMERA_NET_PORT,
        help="Port network device của Orbbec khi dùng LAN",
    )
    p.add_argument(
        "--reconnect-stale-ms",
        type=int,
        default=2500,
        help="Tu dong reconnect neu khong co frame moi qua nguong nay",
    )
    p.add_argument(
        "--reconnect-delay-s",
        type=float,
        default=1.0,
        help="Thoi gian cho truoc khi thu reconnect lai sau khi fail",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    print("=" * 60)
    print("  Femto Mega Camera Viewer")
    print("=" * 60)
    print(f"  Backend  : {args.backend}")
    print(f"  Transport: {args.transport}")
    print(f"  IP       : {args.ip or '-'}")
    print(f"  Port     : {args.port}")
    print(f"  Reconnect: stale>{args.reconnect_stale_ms}ms")
    print(f"  Reso     : {args.width}x{args.height}")
    print(f"  Save dir : {args.save_dir}/")
    print("=" * 60)

    run(args)
