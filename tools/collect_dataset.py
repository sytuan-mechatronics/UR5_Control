"""Collect raw dataset images for phoi detection.

Use while robot is parked at SCAN_POSE and camera points to the tray.

Examples:
  python tools/collect_dataset.py
  python tools/collect_dataset.py --backend orbbec
  python tools/collect_dataset.py --backend opencv --target-count 300
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2


# Allow importing camera backends from project root when this script runs in tools/.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from test_camera_view import auto_select_backend  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect raw images for dataset labeling",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--backend", choices=["auto", "orbbec", "opencv"], default="auto")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--save-dir", default="dataset/raw_images")
    parser.add_argument("--target-count", type=int, default=500)
    parser.add_argument("--delay-s", type=float, default=0.5)
    parser.add_argument("--prefix", default="phoi")
    parser.add_argument("--bg-prefix", default="background")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    return parser.parse_args()


def count_existing_images(save_dir: str) -> int:
    return len([
        file_name for file_name in os.listdir(save_dir)
        if file_name.lower().endswith(".jpg")
    ])


def build_image_name(prefix: str) -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return f"{prefix}_{stamp}.jpg"


def append_metadata(meta_path: str, record: dict) -> None:
    with open(meta_path, "a", encoding="utf-8") as file_obj:
        file_obj.write(json.dumps(record, ensure_ascii=True) + "\n")


def save_capture(frame, save_dir: str, prefix: str, jpeg_quality: int) -> str:
    file_name = build_image_name(prefix)
    file_path = os.path.join(save_dir, file_name)
    ok = cv2.imwrite(file_path, frame, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    if not ok:
        raise RuntimeError("cv2.imwrite failed")
    return file_path


def main():
    args = parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    meta_path = os.path.join(args.save_dir, "captures.jsonl")

    print("=" * 64)
    print("  Dataset Collection - Phoi")
    print("=" * 64)
    print(f"  Backend     : {args.backend}")
    print(f"  Resolution  : {args.width}x{args.height}")
    print(f"  Save dir    : {args.save_dir}")
    print(f"  Target      : {args.target_count} images")
    print(f"  Auto delay  : {args.delay_s}s")
    print("=" * 64)

    existing = count_existing_images(args.save_dir)
    print(f"Da co san: {existing} anh")
    print("\nPhim tat:")
    print("  c  -> chup 1 anh phoi")
    print("  a  -> bat/tat chup tu dong")
    print("  n  -> chup 1 anh background")
    print("  q  -> thoat")

    backend = auto_select_backend(force=args.backend, width=args.width, height=args.height)

    try:
        backend.open()
    except Exception as exc:
        print(f"\n[Loi] Khong mo duoc camera: {exc}")
        return 1

    auto_mode = False
    last_auto_ts = 0.0

    window_name = "Dataset Collection - Camera View"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, min(args.width, 1280), min(args.height, 720))

    try:
        while True:
            frame, _depth = backend.read()
            if frame is None:
                key = cv2.waitKey(10) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            total_now = count_existing_images(args.save_dir)
            remain = max(args.target_count - total_now, 0)

            info = f"Da co: {total_now}/{args.target_count} | Con lai: {remain} | Auto: {'ON' if auto_mode else 'OFF'}"
            cv2.putText(
                frame,
                info,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow(window_name, frame)

            now = time.time()
            if auto_mode and (now - last_auto_ts) >= args.delay_s:
                try:
                    path = save_capture(frame, args.save_dir, args.prefix, args.jpeg_quality)
                    last_auto_ts = now
                    append_metadata(
                        meta_path,
                        {
                            "file": os.path.basename(path),
                            "label": args.prefix,
                            "mode": "auto",
                            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                            "backend": backend.NAME,
                            "width": int(frame.shape[1]),
                            "height": int(frame.shape[0]),
                        },
                    )
                    print(f"Auto: {path}")
                except Exception as exc:
                    print(f"[WARN] Khong luu duoc anh auto: {exc}")

            key = cv2.waitKey(30) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("c"):
                try:
                    path = save_capture(frame, args.save_dir, args.prefix, args.jpeg_quality)
                    append_metadata(
                        meta_path,
                        {
                            "file": os.path.basename(path),
                            "label": args.prefix,
                            "mode": "manual",
                            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                            "backend": backend.NAME,
                            "width": int(frame.shape[1]),
                            "height": int(frame.shape[0]),
                        },
                    )
                    print(f"Chup: {path}")
                except Exception as exc:
                    print(f"[WARN] Khong luu duoc anh: {exc}")

            elif key == ord("a"):
                auto_mode = not auto_mode
                if auto_mode:
                    last_auto_ts = time.time()
                print(f"Auto mode: {'ON' if auto_mode else 'OFF'}")

            elif key == ord("n"):
                try:
                    path = save_capture(frame, args.save_dir, args.bg_prefix, args.jpeg_quality)
                    append_metadata(
                        meta_path,
                        {
                            "file": os.path.basename(path),
                            "label": args.bg_prefix,
                            "mode": "manual_background",
                            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                            "backend": backend.NAME,
                            "width": int(frame.shape[1]),
                            "height": int(frame.shape[0]),
                        },
                    )
                    print(f"Background: {path}")
                except Exception as exc:
                    print(f"[WARN] Khong luu duoc anh background: {exc}")

    except KeyboardInterrupt:
        print("\nDung boi Ctrl+C")
    finally:
        backend.close()
        cv2.destroyAllWindows()

    print(f"\nXong. Tong cong {count_existing_images(args.save_dir)} anh trong {args.save_dir}")
    print(f"Metadata: {meta_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
