"""
One-shot regeneration of the 24-well plate calibration after the orientation
flip (plate_flip_180=True). The saved mosaic is ground truth; this RELABELS the
already-detected well centres (reference_markers) for the new orientation,
stores them DIRECTLY as calibrated_positions (no warp), and DELETES the bogus
warp/affine that absorbed the 180° flip as a 79mm correction.

Run from the project root:  python tools_regen_plate_orientation.py
Writes settings.json + config/hardware/last_calibration.json in place
(.bak-orientremap backups already made).
"""

import json
import os

from SupportClasses.WellPlate import WellPlate
from SupportClasses.MosaicWellRemap import label_positions

SIGN = (-1.0, -1.0)  # plate_flip_180=True (ME3B V1 180° mount)
SETTINGS = "settings.json"
SNAPSHOT = os.path.join("config", "hardware", "last_calibration.json")


def regen_cal(cal: dict, plate) -> dict:
    """Return a corrected copy of a `calibration` dict."""
    markers = cal.get("reference_markers") or {}
    positions = [(float(v[0]), float(v[1])) for v in markers.values()]
    if len(positions) < 3:
        raise SystemExit(
            "reference_markers has <3 positions — cannot relabel; "
            "run a Mosaic scan / Map wells first.")
    results = label_positions(positions, plate, SIGN)
    rows = int(plate.rows)
    cols = int(plate.cols)
    if len(results) != rows * cols:
        print(f"  WARNING: matched {len(results)}/{rows*cols} wells")

    out = dict(cal)
    out["plate_flip_180"] = True
    out["reference_markers"] = {n: [x, y] for n, (x, y) in results.items()}
    out["calibrated_positions"] = {n: [x, y] for n, (x, y) in results.items()}
    if "A1" in results:
        out["taught_a1"] = list(results["A1"])
    corner_well = f"{chr(ord('A') + rows - 1)}{cols}"  # e.g. D6
    out["corner_well"] = corner_well
    if corner_well in results:
        out["taught_corner"] = list(results[corner_well])
    tw = out.get("third_well")
    if tw and tw in results:
        out["taught_third"] = list(results[tw])
    # The bogus warp/affine absorbed the 180° flip — drop them entirely.
    out.pop("plate_warp", None)
    out.pop("mosaic_affine", None)
    # Identity alignment (calibrated_positions is authoritative now).
    out["offset_x"] = 0.0
    out["offset_y"] = 0.0
    out["rotation"] = 0.0
    out["scale"] = 1.0
    return out, results


def main():
    plate = WellPlate.load("24")

    # settings.json
    with open(SETTINGS, "r", encoding="utf-8") as f:
        settings = json.load(f)
    cal = settings.get("calibration")
    if not cal:
        raise SystemExit("settings.json has no 'calibration' section")
    new_cal, results = regen_cal(cal, plate)
    settings["calibration"] = new_cal
    with open(SETTINGS, "w", encoding="utf-8") as f:
        json.dump(settings, f, indent=2)
    print(f"settings.json: re-derived {len(results)} wells; "
          f"A1={tuple(round(v) for v in results['A1'])}; "
          "dropped plate_warp + mosaic_affine.")

    # config/hardware/last_calibration.json
    if os.path.exists(SNAPSHOT):
        with open(SNAPSHOT, "r", encoding="utf-8") as f:
            snap = json.load(f)
        scal = snap.get("calibration")
        if scal:
            new_scal, _ = regen_cal(scal, plate)
            snap["calibration"] = new_scal
            # keep the fingerprint orientation stamp consistent
            fp = snap.get("fingerprint")
            if isinstance(fp, dict):
                fp["plate_flip_180"] = True
            with open(SNAPSHOT, "w", encoding="utf-8") as f:
                json.dump(snap, f, indent=2)
            print("last_calibration.json: re-derived snapshot; "
                  "dropped plate_warp + mosaic_affine.")

    # Verification
    print("\nNew labeling (corners):")
    for n in ("A1", "A6", "D1", "D6"):
        if n in results:
            print(f"  {n} = ({results[n][0]:,.0f}, {results[n][1]:,.0f}) um")


if __name__ == "__main__":
    main()
