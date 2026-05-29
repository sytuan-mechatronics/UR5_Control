"""Annotate tray corners + 5 hole centers and save them to tray_layout.json.

Usage:
  python3 tools/annotate_tray_layout.py --image path/to/image.png
  python3 tools/annotate_tray_layout.py --capture

Controls:
  left click  : add point
  u           : undo last point
  s           : save when enough points
  q / ESC     : quit without saving
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from vision.femto_camera import FemtoCamera
from vision.tray_layout import save_tray_layout


WINDOW_NAME = "Annotate Tray Layout"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Click 4 tray corners + 5 holes and save tray_layout.json")
    parser.add_argument("--image", help="Existing image path to annotate")
    parser.add_argument("--capture", action="store_true", help="Capture one image from camera")
    parser.add_argument("--output", default="tray_layout.json", help="Output JSON path")
    parser.add_argument("--notes", default="", help="Optional notes saved to JSON")
    return parser.parse_args()


def capture_from_camera():
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
    try:
        camera.connect()
        rgb, _depth, _ts = camera.get_frames_with_timestamp()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    finally:
        try:
            camera.disconnect()
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    if not args.image and not args.capture:
        raise SystemExit("Can --image hoac --capture")

    if args.image:
        frame = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if frame is None:
            raise SystemExit(f"Khong doc duoc anh: {args.image}")
    else:
        frame = capture_from_camera()

    display = frame.copy()
    tray_corners = []
    holes = []

    def redraw():
        nonlocal display
        display = frame.copy()
        for idx, (u, v) in enumerate(tray_corners, start=1):
            cv2.circle(display, (int(round(u)), int(round(v))), 7, (255, 120, 0), -1)
            cv2.putText(
                display,
                f"C{idx}",
                (int(round(u)) + 10, int(round(v)) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 200, 0),
                2,
                cv2.LINE_AA,
            )
        if len(tray_corners) == 4:
            pts = [(int(round(u)), int(round(v))) for u, v in tray_corners]
            for idx in range(4):
                cv2.line(display, pts[idx], pts[(idx + 1) % 4], (255, 120, 0), 2)
        for idx, (u, v) in enumerate(holes, start=1):
            cv2.circle(display, (int(round(u)), int(round(v))), 7, (0, 0, 255), -1)
            cv2.putText(
                display,
                f"H{idx}",
                (int(round(u)) + 10, int(round(v)) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        if len(tray_corners) < 4:
            guide = "Click 4 tray corners theo chieu kim dong ho | u: undo | q/ESC: quit"
        else:
            guide = "Click 5 hole centers | u: undo | s: save | q/ESC: quit"
        cv2.putText(display, guide, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(display, guide, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 1, cv2.LINE_AA)

    def on_mouse(event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(tray_corners) < 4:
                tray_corners.append([float(x), float(y)])
            elif len(holes) < 5:
                holes.append([float(x), float(y)])
            redraw()

    redraw()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    while True:
        cv2.imshow(WINDOW_NAME, display)
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            cv2.destroyAllWindows()
            return 1
        if key == ord("u"):
            if holes:
                holes.pop()
            elif tray_corners:
                tray_corners.pop()
            redraw()
        if key == ord("s"):
            if len(tray_corners) != 4 or len(holes) != 5:
                print(f"Can dung 4 goc + 5 lo, hien tai co {len(tray_corners)} goc va {len(holes)} lo")
                continue
            output_path = Path(args.output)
            save_tray_layout(
                output_path,
                [frame.shape[1], frame.shape[0]],
                tray_corners,
                holes,
                notes=args.notes,
            )
            print(f"Da luu tray layout: {output_path.resolve()}")
            break

    cv2.destroyAllWindows()
    time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
