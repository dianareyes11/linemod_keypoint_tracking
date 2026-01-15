#!/usr/bin/env python3
import sys

import cv2
import numpy as np


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 scripts/sample_hsv.py <image_path>")
        return 2

    image_path = sys.argv[1]
    bgr = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"Failed to read image: {image_path}")
        return 1

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    samples = []

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        pixel = hsv[y, x].astype(int)
        samples.append(pixel)
        bgr_pixel = bgr[y, x].astype(int)
        print(f"sample {len(samples)} at ({x},{y}) BGR={bgr_pixel.tolist()} HSV={pixel.tolist()}")

    window = "Click milk-carton pixels (press q when done)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)

    while True:
        cv2.imshow(window, bgr)
        key = cv2.waitKey(20) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()

    if not samples:
        print("No samples collected.")
        return 0

    samples_np = np.array(samples, dtype=np.int32)
    lower = samples_np.min(axis=0)
    upper = samples_np.max(axis=0)
    print("\nHSV range from samples:")
    print(f"lower color range: [ {float(lower[0])}, {float(lower[1])}, {float(lower[2])}, 0. ]")
    print(f"upper color range: [ {float(upper[0])}, {float(upper[1])}, {float(upper[2])}, 0. ]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
