# MEBP v7.5.x — Needle Location quick-move button

## Objective

Add a **"Go to needle location"** quick-move button to the Calibration →
**Needle Location** tab so the operator can drive the stage straight to the
approximate spot where the needle sits in both side cameras, instead of hand-
jogging it back into frame every time.

The needle side-cameras (`NEEDLE_X` / `NEEDLE_Y`) are bolted to the frame, so the
centered needle position is a **stable per-machine datum**. We persist it and let
the operator return to it with one click, ready to re-center.

User decisions (2026-06-16):
- **Location source:** auto-save the centered position from each successful
  *Center & Save*, **plus** a manual *Set current as location* button to seed a
  fresh machine before the first calibration.
- **Z behavior:** retract Z to the Fast-Move (Safe) Z → travel XY → **lower to
  the saved needle-cam Z** so the needle re-enters both side views.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | New quick-move + set-current buttons on the Needle Location tab; `_needle_loc_goto` / `_needle_loc_set_current` / `_needle_loc_store_xy` / `_needle_loc_update_goto_ui`; persist absolute XY on Center & Save. |
| `gui/pages/hardware/device_profile.py` | New `needle_loc_xy_um` field on `DeviceProfile` (round-trips to the device JSON alongside `needle_cam_z`). |
| `tests/test_v75x_needle_location_quick_move.py` | New tests. |

## Design

- **Storage key:** `settings.get/set("device_profile.needle_loc_xy_um")` →
  `[x_um, y_um]` in the **absolute Prior stage µm** frame (so the value survives
  restarts — the ProScan keeps its absolute frame across power cycles). Mirrors
  how `device_profile.needle_cam_z` is handled.
- **DeviceProfile:** `needle_loc_xy_um: Optional[list] = None` added to the
  dataclass + `to_dict` / `from_dict` / `from_settings` / `apply_to_settings`
  so it travels with the saved device profile.
- **Quick-move** reuses the existing `CalibrationPage._safe_navigate_to(...)`
  (→ `StageController.safe_travel_to`): retract Z to `_safe_z` → wait → fast XY →
  wait → lower to the needle-cam Z (converted user→zero-ref via
  `user_z_to_zref`). No new motion/coordinate math.
- **Z target** = `controller.get_needle_cam_z_user()` → `user_z_to_zref(...)`.
  If the needle-cam Z is not captured yet, the move stays at Safe Z
  (`lower_z=False`) — XY-only, still safe.
- **Safety gate:** with ZP connected and no `_safe_z` set, the retract would fall
  back to zero-ref `0` (the bottom datum on ME3B V1) = a crash, so the go-to is
  blocked with a message directing to the Needle Offset tab (mirrors the Plate
  Location `_ploc_run_queue` gate).

## Implementation Steps

- [x] Add `needle_loc_xy_um` to `DeviceProfile` (dataclass + 4 bridge methods).
- [x] Init `self._needle_loc_xy_um` from settings in `_build_needle_location_tab`.
- [x] Add the "Go to needle location" + "Set current as location" buttons + a
      saved-location caption label to the wizard panel.
- [x] `_needle_loc_store_xy` (persist + refresh UI), `_needle_loc_update_goto_ui`
      (enable/label), `_needle_loc_set_current`, `_needle_loc_goto`.
- [x] Persist absolute XY in `_needle_loc_center_and_save`.
- [x] Tests — `tests/test_v75x_needle_location_quick_move.py` (17) + offscreen
      tab-build smoke (loads saved loc → enables button → renders label).
- [ ] Real-HW verification on ME3B V1.

## Testing Notes

- `_needle_loc_store_xy` writes `device_profile.needle_loc_xy_um` and enables the
  go-to button; reload reads it back.
- `_needle_loc_goto` calls `safe_travel_to` with the saved absolute XY and the
  needle-cam Z (zero-ref); stays at Safe Z when no needle-cam Z.
- Go-to blocked when ZP connected + Safe Z unset.
- `DeviceProfile` round-trips `needle_loc_xy_um`.

## Issues & Decisions

- Stored in the **absolute** stage frame (not the zero-ref `needle_origin_um`)
  because the quick-move feeds `safe_travel_to`, which takes absolute µm and is
  power-cycle-stable on the ProScan.
- **Needs real-HW verification on ME3B V1.**
