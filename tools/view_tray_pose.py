"""Debug tray-hole layout matching on the current camera frame."""

import argparse
import os
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from vision.femto_camera import FemtoCamera
from vision.tray_holes import detect_tray_holes, match_tray_layout_to_detected_holes


WINDOW_NAME = "Tray Hole Layout View"


def parse_args():
    parser = argparse.ArgumentParser(description="View detected tray contour and projected 5-hole layout")
    parser.add_argument("--layout", default=config.TRAY_LAYOUT_PATH)
    parser.add_argument(
        "--save-path",
        default="logs/view_tray_pose.jpg",
        help="Fallback path to save annotated image when GUI window cannot be opened",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Do not open OpenCV window; save annotated image and exit",
    )
    return parser.parse_args()


def draw(frame_bgr, holes, layout_match):
    overlay = frame_bgr.copy()
    if holes:
        for hole in holes:
            hu, hv = [int(round(v)) for v in hole["center"]]
            hr = int(round(hole["radius_px"]))
            cv2.circle(overlay, (hu, hv), hr, (0, 200, 255), 2)

    if layout_match is not None:
        for hole in layout_match["projected_holes_uv"]:
            hu, hv = int(round(hole[0])), int(round(hole[1]))
            cv2.circle(overlay, (hu, hv), 10, (255, 0, 255), 2)
        for idx, hole in enumerate(layout_match["projected_holes_uv"], start=1):
            hu, hv = int(round(hole[0])), int(round(hole[1]))
            cv2.putText(overlay, f"H{idx}", (hu + 8, hv - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2, cv2.LINE_AA)

    guide = "q/ESC: thoat | detected holes: orange | matched layout holes: magenta"
    cv2.putText(overlay, guide, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(overlay, guide, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 1, cv2.LINE_AA)
    return overlay


def main():
    args = parse_args()
    camera = FemtoCamera(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
    try:
        print("Connecting camera...")
        camera.connect()
        rgb, _depth, _ts = camera.get_frames_with_timestamp()
        frame_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        holes = detect_tray_holes(
            rgb,
            min_radius_px=config.TRAY_HOLE_MIN_RADIUS_PX,
            max_radius_px=config.TRAY_HOLE_MAX_RADIUS_PX,
            min_dist_px=config.TRAY_HOLE_MIN_DIST_PX,
        )
        layout_match = match_tray_layout_to_detected_holes(
            args.layout,
            holes,
            max_reproj_error_px=config.TRAY_LAYOUT_MAX_REPROJ_ERR_PX,
            max_candidate_holes=config.TRAY_LAYOUT_MAX_CANDIDATE_HOLES,
        )

        print(f"detected_holes={len(holes)}")
        if holes:
            print(f"holes={[[round(v, 1) for v in hole['center']] + [round(hole['radius_px'], 1)] for hole in holes]}")
        print(f"layout_match={layout_match is not None}")
        if layout_match is not None:
            print(f"reproj_error_px={layout_match['reproj_error_px']:.1f}")
            print(
                "projected_holes_uv="
                f"{[[round(v, 1) for v in pt] for pt in layout_match['projected_holes_uv']]}"
            )

        overlay = draw(frame_bgr, holes, layout_match)
        save_path = Path(args.save_path)
        headless_env = not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY"))
        if args.no_gui or headless_env:
            save_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(save_path), overlay)
            print(f"Saved annotated image to: {save_path}")
            if headless_env and not args.no_gui:
                print("GUI display not detected; skipped cv2.imshow().")
            return 0

        try:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            while True:
                cv2.imshow(WINDOW_NAME, overlay)
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord("q")):
                    break
            cv2.destroyAllWindows()
        except cv2.error as exc:
            save_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(save_path), overlay)
            print(f"OpenCV GUI unavailable: {exc}")
            print(f"Saved annotated image to: {save_path}")
        return 0
    finally:
        try:
            camera.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
