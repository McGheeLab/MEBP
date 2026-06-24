"""
Well-finder visual debugger (detection R&D).

Runs the SHIPPING detector — ``WellDetector.detect_filled_wells`` +
``WellDetector.fit_well_grid`` (see SupportClasses/VisionDetector.py) — on a
mosaic image and writes an annotated overlay so you can eyeball the result while
developing/tuning well detection against the labeled training corpus
(config/hardware/well_training/ and config/hardware/mosaics/).

Usage:
    python "coding plans/well_detection_rnd/well_finder_viz.py" [IMAGE] [ROWS] [COLS]
    # defaults: config/hardware/mosaics/24.png  4 6  → 24_annotated.png

This is a dev tool, not part of the app. No duplicated algorithm — it calls the
same code the GUI uses, so what you see here is what the app detects.
"""

from __future__ import annotations

import os
import sys

# Make the repo root importable when run as a standalone script.
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")))

import numpy as np
import cv2

from SupportClasses.VisionDetector import WellDetector


def annotate(image_path, n_rows, n_cols, out_path):
    img = cv2.imread(image_path)
    if img is None:
        print("could not read", image_path)
        return
    dets = WellDetector.detect_filled_wells(img)
    centers = [(d.center_px[0], d.center_px[1], d.radius_px) for d in dets]
    aff, assign = WellDetector.fit_well_grid(
        [(c[0], c[1]) for c in centers], n_rows, n_cols)
    print(f"blobs detected: {len(dets)}")
    if aff is None:
        print("grid fit failed")
        return
    A, t = aff[:, :2], aff[:, 2]
    radii = [centers[idx][2] for idx in assign.values()]
    rmed = float(np.median(radii)) if radii else 20.0
    vis = img.copy()
    n_det = 0
    for i in range(n_rows):
        for j in range(n_cols):
            name = f"{chr(ord('A') + i)}{j + 1}"
            if (i, j) in assign:
                cx, cy, r = centers[assign[(i, j)]]
                col = (0, 255, 0)
                n_det += 1
            else:
                p = A @ np.array([j, i]) + t
                cx, cy, r = float(p[0]), float(p[1]), rmed
                col = (0, 165, 255)
            cv2.circle(vis, (int(cx), int(cy)), int(r), col, 4)
            cv2.drawMarker(vis, (int(cx), int(cy)), (0, 0, 255),
                           cv2.MARKER_CROSS, 30, 4)
            cv2.putText(vis, name, (int(cx) - 40, int(cy) - int(r) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 4)
    print(f"resolved {len(assign)}/{n_rows*n_cols} "
          f"(detected={n_det}, filled-from-grid={n_rows*n_cols-n_det})")
    cv2.imwrite(out_path, vis)
    print("wrote", out_path)


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "config/hardware/mosaics/24.png"
    rows = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    cols = int(sys.argv[3]) if len(sys.argv) > 3 else 6
    annotate(path, rows, cols,
             "coding plans/well_detection_rnd/_annotated_preview.png")
