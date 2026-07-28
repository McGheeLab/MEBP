# MEBP v7.5.x — Per-Camera Rotation-vs-Stage Calibration + Mirrored View + Monitor Camera Role

## Objective

Three operator requests (2026-07-24):

1. **Per-camera rotation calibration on Hardware Setup → Cameras.** Every
   camera slot should carry a calibration that says what the camera's
   rotation is **relative to the microscope stage**:
   - The **microscope** camera views along the Z axis, so its rotation is
     in the stage **XY plane** (nominal: axis-aligned, 0°).
   - The **needle** cameras are mounted at **~45° to the stage X or Y
     axis** (nominal: ±45°).
2. **New "Monitor" camera** — a camera that rests on the stage and
   overviews the entire operation. Becomes a first-class `CameraRole`
   with its own (4th) camera slot.
3. **Per-camera "Mirrored view" option** (follow-up) — a checkbox to
   declare that a camera shows a mirrored (left↔right flipped) view. A
   mirror reverses image handedness, which the rotation (a proper
   rotation) can never express — so it is a separate flag. flip-x plus an
   arbitrary rotation together span **every** possible camera orientation
   (the full O(2) improper set), so one boolean + θ is complete.

## Current state (why this is mostly surfacing, not new math)

The measurement + storage infrastructure already exists from
`MEBP_v75x_CAL_LIVEVIEW_AND_CAMERA_ROTATION.md` and
`MEBP_v75x_REANCHOR_MOSAIC_AND_CAMERA_ORIENTATION.md`:

- `PixelCalibrationDialog` measures the in-plane rotation vs the stage
  (`result_rotation_deg`, via `plus_column_direction_deg`) by moving the
  stage and phase-correlating the image displacement. It already has
  X / Y / ±45° move-direction presets.
- `CameraManager.get/set_rotation_deg(slot)` is the live value used by
  `pixel_to_stage_offset` (click→stage mapping applies `R(θ)`).
- `CameraCalibrationStore.set_rotation/get_rotation` persists it per
  device identity (rotation-only write preserves µm/px siblings);
  `hardware_setup._restore_calibration_for_slot` already restores it on
  slot assignment, independent of µm/px.
- The **microscope** already has a "Calibrate orientation…" button + a
  rotation readout — but it lives on the Objective Calibration card, and
  the **needle** cameras only get a rotation as a side effect of their
  µm/px calibration (shown inline as `@ N°` on the needle card). There is
  no uniform per-camera rotation readout/calibration on the slot cards.

## Design

### Monitor role (4th camera)

- `CameraRole.MONITOR = "monitor"` — rests on the stage, overviews the
  whole operation. Added to `SINGLETON_CAMERA_ROLES` (one monitor at a
  time), the slot-row Role combo ("Monitor (overview)"), and
  `_role_badge_props`.
- `MAX_LIVE_CAMERAS` 3 → **4**. `from_dict` already pads shorter
  `camera_roles` lists with UNASSIGNED, so legacy 3-entry configs migrate
  for free; unknown role strings still degrade to UNASSIGNED on older
  builds reading a newer config.
- `CameraManager` default `max_cameras` and the `gui/app.py` construction
  now source `MAX_LIVE_CAMERAS` (they were hardcoded `3` in two places
  with a "keep in sync" comment). All Hardware Setup slot loops already
  iterate the built widget lists, so the 4th row flows through detect /
  save-load setup / previews / correction strips untouched.

### Per-slot rotation strip (Camera Detection & Assignment)

Each slot mini-card gains a row-3 strip:

- **Readout label** — `Rotation vs stage: 44.7° (Δ −0.3° from nominal
  45°) — side view, ~45° to the stage X/Y axes` (green when calibrated,
  dim + "not calibrated" otherwise). Role-aware nominal:
  - MICROSCOPE / MONITOR → axis-aligned nominals {0°, ±90°, 180°}
  - NEEDLE_X / NEEDLE_Y → diagonal nominals {±45°, ±135°}
  - The Δ is a sanity hint only — never used for motion.
- **"⟳ Calibrate rotation…" button** — launches the existing
  `PixelCalibrationDialog` for THIS slot, takes **only**
  `result_rotation_deg` (µm/px is deliberately untouched — the needle /
  objective µm/px calibrations own that), pushes it live
  (`set_rotation_deg`) and persists rotation-only per device identity
  (`CameraCalibrationStore.set_rotation`). Enabled while the slot's
  camera is running.
- **Microscope special case:** when the calibrated slot holds the
  MICROSCOPE role, the fresh rotation is also synced into every
  per-objective calibration (mount rotation is a property of the CAMERA,
  not the objective) — same rule the objective card's own "Calibrate
  orientation…" applies. That sync is extracted into a new public
  `ObjectiveCalibrationCard.adopt_camera_rotation(rotation_deg)` reused
  by both paths.

### Mirrored view

- `CameraCalibrationStore.get_mirrored` / `set_mirrored` — a `mirrored`
  bool sibling of `rotation_deg` in the per-identity entry (stored only
  when True; False pops the key → legacy entries byte-identical;
  preserves µm/px / rotation / image-correction siblings).
- `CameraManager._mirrored[]` + `get_mirrored` / `set_mirrored`
  (getattr-guarded read for `__new__` test doubles).
- `pixel_to_stage_offset` applies a horizontal parity flip
  (`dx_px → −dx_px`) BEFORE scaling/rotation when the slot is mirrored.
  Default False = byte-identical identity mapping. flip-x + R(θ) spans
  O(2), so the full mirrored+rotated orientation set is reachable.
- Hardware Setup slot card: a "⇄ Mirrored view" checkbox (row 4; enabled
  once the slot has a source — no live feed needed, it's a declaration),
  `_on_toggle_mirror` / `_apply_slot_mirror` (push live + persist
  rotation/µm/px-preserving), restored via `_restore_calibration_for_slot`
  and reflected in the rotation readout ("· mirrored view").
- **Display is left un-mirrored** — like rotation, only the click→stage
  mapping is corrected (keeps captures/detection/mosaic untouched).
- **Not synced to the objective store** — mirror is purely a
  camera-identity property; it survives objective swaps because the
  objective push never touches `set_mirrored`.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/HardwareConfig.py` | `CameraRole.MONITOR`; added to `SINGLETON_CAMERA_ROLES`; `MAX_LIVE_CAMERAS = 4` |
| `SupportClasses/CameraCalibrationStore.py` | `get_mirrored` / `set_mirrored` (sibling of `rotation_deg`) |
| `gui/widgets/camera_manager.py` | default `max_cameras` from `MAX_LIVE_CAMERAS`; `_mirrored[]` + `get/set_mirrored`; horizontal parity flip in `pixel_to_stage_offset` |
| `gui/app.py` | `CameraManager()` (slot count = `MAX_LIVE_CAMERAS` default) |
| `gui/pages/hardware_setup.py` | slot rows from `MAX_LIVE_CAMERAS`; Monitor role combo item + badge; rotation helpers `role_nominal_rotations` / `nominal_rotation_delta` / `role_rotation_hint`; per-slot rotation strip + `_refresh_slot_rotation_displays` + `_on_calibrate_slot_rotation` / `_apply_slot_rotation`; per-slot **Mirrored view** checkbox + `_on_toggle_mirror` / `_apply_slot_mirror`; mirror restore in `_restore_calibration_for_slot` |
| `gui/pages/hardware/objective_calibration_card.py` | per-objective rotation sync extracted to public `adopt_camera_rotation` (reused by its own orientation handler) |
| `tests/test_v75x_camera_rotation_cal_and_monitor.py` | new suite (rotation + monitor + mirror) |

## Implementation Steps

- [x] Explore existing rotation/orientation infrastructure + role plumbing
- [x] `CameraRole.MONITOR` + singleton + `MAX_LIVE_CAMERAS = 4`
- [x] `CameraManager` / `app.py` slot count from `MAX_LIVE_CAMERAS`
- [x] Slot rows: 4th row, Monitor role combo item, Monitor badge
- [x] Rotation nominal helpers (pure, module-level, testable)
- [x] Per-slot rotation strip UI + refresh wiring
- [x] `_on_calibrate_slot_rotation` / `_apply_slot_rotation` (rotation-only)
- [x] `ObjectiveCalibrationCard.adopt_camera_rotation` refactor
- [x] Mirrored-view flag: store + manager parity-flip + slot checkbox + restore
- [x] Tests + affected-suite regression run
- [x] Finalize this plan + CLAUDE.md table row

## Testing Notes

- New `tests/test_v75x_camera_rotation_cal_and_monitor.py`:
  - MONITOR enum value/serialization round-trip; singleton enforcement;
    legacy 3-entry `camera_roles` pads to 4; `MAX_LIVE_CAMERAS == 4`.
  - `nominal_rotation_delta` picks the nearest role nominal (needle 44°
    → Δ −1° from 45°; microscope 91° → Δ +1° from 90°; wrap cases).
  - Offscreen page build: 4 slot rows with rotation labels/buttons;
    Monitor in the role combo; readout flips not-calibrated → value.
  - `_apply_slot_rotation`: pushes to manager, persists rotation-only
    (µm/px sibling preserved), microscope slot syncs per-objective via
    the card's `adopt_camera_rotation`.
  - **Mirror:** `pixel_to_stage_offset` negates x only when mirrored;
    composes correctly with rotation (flip-x → R(90°): (10,0)px →
    (0,−10)µm); per-slot; getattr-guarded for doubles. Store round-trip
    (True persists, False pops the key, siblings preserved). Page:
    checkbox per slot, `_apply_slot_mirror` pushes live + persists +
    updates readout, restores on slot assignment.
- **Result: 35 new tests green (22 rotation/monitor + 13 mirror).**
  Regression green:
  `test_v75x_camera_calibration_store` / `camera_cal_liveview` /
  `camera_rotation` / `test_v74x_objective_calibration` /
  `test_v744_calibration_revision` (one hard-coded 3-entry serialization
  expectation updated to be `MAX_LIVE_CAMERAS`-agnostic) /
  `reanchor_mosaic_and_camera_orientation` / `camera_image_correction` /
  `camera_hardware_controls` / `camera_async_open_and_context_width` /
  `fluorescence_mosaic` / `rosette_ink_pickup_well` / `plate_types` /
  `needle_max_flow_and_pickup_rates` / `ink_location_assignments` /
  `ink_location_well_colors` / `ink_well_type_and_subtype`.
  `test_v75x_spheroid_picker_scaling` has 5 PRE-EXISTING errors
  (`CameraManager.__new__` fake missing `_rotation_deg`, from concurrent
  uncommitted WIP in the working tree — confirmed present with this
  change stashed).
- **Needs real-HW verification on ME3B V1/V3**: assign + start each
  camera → Calibrate rotation → plausible angles (microscope ≈ 0°-ish,
  needle cams ≈ ±45°-ish); toggle **Mirrored view** on a camera whose
  feed is flipped → live-view clicks drive the stage the correct way;
  readouts + mirror restore after restart; a 4th (Monitor) camera can be
  assigned, started, previewed.

## Issues & Decisions

- **Rotation-only on the slot strip** — the slot button deliberately does
  NOT touch µm/px: the microscope's µm/px is per-objective
  (`ObjectiveCalibrationStore`) and the needle µm/px has its own
  calibrate flow. One measurement dialog, two commit policies.
- **Nominal Δ is informational** — nothing consumes it; the calibrated θ
  itself keeps flowing through the existing `pixel_to_stage_offset` path.
- **Monitor rotation semantics** — the monitor camera rests on the stage
  (overview). Its rotation calibration is available/informational
  (axis-aligned nominal) but nothing gates on it.
- `from_dict` role padding makes the 3→4 slot migration free; no
  settings.json migration required.
- **Mirror is a separate flag, not part of rotation** — a mirror is an
  improper (handedness-reversing) transform; a proper rotation cannot
  represent it. flip-x + R(θ) generates the full O(2), so a single
  horizontal-mirror boolean + the continuous θ covers every mounted
  orientation, mirrored or not. Horizontal (left↔right) is the standard
  meaning; a vertical mirror = 180° rotation + horizontal, already
  reachable.
- **Mirror lives only in the identity store** (not the objective store) —
  it's a physical camera/optics property, so it survives objective swaps
  without a per-objective copy (nothing in the objective push touches
  `set_mirrored`).

## Addendum (2026-07-24) — mirror flips the DISPLAY at the frame source

Operator: *"if i select mirror camera, it needs to mirror the output of the
camera feed so what i see is not mirrored."* The initial cut applied the mirror
as a click→stage flip (`pixel_to_stage_offset` `dx→−dx`) + a mosaic
`_orient_tile` flip, leaving the displayed feed mirrored. Reworked so the mirror
is a **horizontal flip at the frame source**, correcting the DISPLAY:

- **`CameraWidget`**: `_mirrored` + `set_mirrored` + `mirrored`;
  `_maybe_mirror_frame(frame)` = `cv2.flip(frame, 1)` when set, shared by
  `_grab_frame` (applied *before* the raw-frame cache, so display + the cached
  frame used for detection/mosaic/click-mapping are all un-mirrored) and the
  direct `capture_fresh_frame` path (the mosaic scanner bypasses `_grab_frame`).
  Geometric — unlike brightness/gamma, which stay display-only *after* the cache.
- **`CameraManager.get/set_mirrored`** now **delegate to the widget** (like the
  image-correction knobs); the local `_mirrored[]` is a fallback for no-widget
  slots / test doubles. `_widget` made `getattr`-safe for `__new__` doubles.
- **`pixel_to_stage_offset`**: mirror term **removed** (frame arrives
  un-mirrored → clicks map with rotation only).
- **`MosaicBuilder`**: `frame_mirrored` param + the mirror in `_orient_tile`
  **removed** — the builder receives an un-mirrored frame and applies rotation
  only. `_ploc_microscope_frame_orientation()` returns rotation only.
- Net: `mirror` and `rotation` are now independent and orthogonal — mirror
  corrects handedness at the source (display + everything), rotation stays a
  downstream `R(θ)` (display stays rotated, mapping/mosaic corrected). One
  horizontal mirror still covers all handedness (a vertical mirror = 180° +
  horizontal).
- Tests reworked: `TestMirrorNotInMapping` (mapping ignores mirror),
  `TestCameraWidgetMirror` (`_maybe_mirror_frame` flips/no-ops),
  `TestManagerMirrorDelegates`; `_orient_tile`/builder mirror tests dropped.
  49 green across the two suites; camera-store/liveview/correction/hw-controls
  (80) + mosaic/fluorescence (79) green.
- **Needs real-HW check:** select Mirrored view on a mirrored camera → the live
  feed reads un-mirrored, and clicks + mosaic mapping stay correct.
