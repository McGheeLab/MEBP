# MEBP v7.16 — the live-view orientation is separate from the measured one

## Objective

Operator report:

> 1. I setup the camera live view so that the image is showing a correctly
>    rotated and mirrored view
> 2. I then go into the objective calibration for the mosaic builder, and once it
>    is done it always flips the camera back to the wrong orientation for the
>    live view.
>
> if we need to, we can separate the mosaic settings from the camera live view
> settings.

Their diagnosis was right, and their proposed remedy is the one implemented.

## Root cause

ONE `(rotation_deg, mirrored, flip_y)` triple per camera identity in
`CameraCalibrationStore` served three consumers:

| Consumer | What it uses the triple for |
|---|---|
| `CameraFeedView._sync_auto_orientation` | what the operator SEES |
| `MosaicBuilder._orient_tile` | orienting every mosaic tile into stage axes |
| `CameraManager.pixel_to_stage_offset` | mapping every live-view click to stage µm |

…and it had **TWO writers**:

1. the per-slot **Flip X / Flip Y / Rotation** controls on Hardware Setup →
   Cameras (`_apply_slot_mirror` / `_apply_slot_flip_y` /
   `_apply_slot_rotation_value`) — a *viewing preference*;
2. `objective_calibration_card._on_calibrate_clicked`, which pushes
   `derive_camera_stage_orientation`'s measured `flip_x` / `flip_y` /
   `rotation_deg` into the live manager **and** the store — the *measured
   camera→stage matrix*.

(2) overwrote (1). So the operator's exact workflow — set the view up, then
calibrate — always ended with the view flipped back. Reproduced against the real
`HardwareSetupPage` + `CameraManager` offscreen.

**No precedence rule could have fixed this, because the two are different
quantities.** The measured triple makes the display *stage-aligned* (image +X →
stage +X). On a rig with `plate_flip_180 = True` the plate then reads 180° from
the A1-top-left convention every other plate view uses — so the orientation the
operator wants to LOOK at is legitimately not the one the mosaic NEEDS.

### Why the split is safe

The display was already decoupled from the geometry at the plumbing level, which
is what makes this a small change rather than a rework:

* `CameraFeedView._widget_to_image` → `ViewGeometry.to_image(...)` inverts this
  view's own transform, so `clicked` emits **RAW frame** coordinates;
* `pixel_to_stage_offset` then applies the **measured** orientation to those raw
  coordinates;
* overlays (`set_bore_markers`, `set_reference_markers`, target rings) go stage
  µm → raw px → display via the same `true_xform`.

So moving the display does not move where a click lands. Pinned by
`TestTheDisplayCannotMoveTheGeometry`, which asserts `pixel_to_stage_offset` is
byte-identical across 4 flip combos × 5 rotations of the *view*, with a companion
assertion that the *measured* mirror still does change it (otherwise that test
could pass because nothing maps clicks at all).

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/CameraCalibrationStore.py` | Schema 1.2 → **1.3**. NEW `get_view_orientation` / `set_view_orientation` / `clear_view_orientation` storing a nested `view_orientation: {rotation_deg, flip_x, flip_y}`. NEW `_migrate_12_to_13`. |
| `gui/widgets/camera_manager.py` | NEW per-slot `_view_orient` cache + `display_orientation` / `has_display_orientation` / `set_display_orientation` / `clear_display_orientation`. `restore_calibration_from_store` restores **and clears** it. `full_orientation` docstring now states it is geometry only. |
| `gui/widgets/camera_feed_view.py` | `_sync_auto_orientation` reads `display_orientation`, falling back to `full_orientation`. |
| `gui/pages/hardware_setup.py` | The three slot controls now commit through NEW `_commit_slot_view_orientation` (display only). NEW `_slot_view_orientation_now` / `_slot_view_orientation_note`. `_push_slot_view_orientation` + the control sync in `_refresh_slot_rotation_displays` read the view. `_restore_calibration_for_slot` restores/clears the view. Tooltips rewritten. |
| `gui/pages/calibration.py` | `_ploc_apply_feed_orientation` prefers `display_orientation` for the plate-location feed. |
| `gui/widgets/capture_controller.py` | 🐞 unrelated defect found en route (see below). |
| `tests/test_v716_live_view_vs_measured_orientation.py` | NEW (28 tests). |
| `tests/test_v75x_camera_rotation_cal_and_monitor.py` | 6 tests updated to the new contract + 5 added (48 total). |

**Deliberately NOT changed:** `objective_calibration_card`. Once the view is a
separate field, that card's existing writes *are* geometry writes and are
correct — the fix needed no change there at all.

## Ownership after this change

* **Measured** (`rotation_deg` / `mirrored` / `flip_y`) — written only by a
  measurement: `ScaleFovCalibrationDialog` (via the objective card),
  `PixelCalibrationDialog` (⟳ Rotation…), `camera_rotation_align_dialog`
  (⊾ Square up mount…), `mosaic_calibration_dialog`'s flip buttons. Read by
  `MosaicBuilder._orient_tile`, `pixel_to_stage_offset` /
  `stage_offset_to_pixel`, `MosaicCalibration.resolve_camera_orientation`,
  `capture_controller`, `cell_targeting_workflow`, the square-up dialog.
* **Live view** (`view_orientation`) — written only by the per-slot Flip X /
  Flip Y / Rotation controls. Read only by `CameraFeedView` (`auto_orient`), the
  slot previews, and the plate-location feed.

## Design decisions

* **Absolute, not a delta.** The view is what the operator wants to see, stored
  outright. A delta on top of the measurement would let a re-measurement change
  the look, which is the defect.
* **Unset ⇒ fall back to the measurement.** Byte-identical to pre-v7.16 for any
  camera the operator never customises; no migration needed for neutral cameras.
* **A neutral view is stored EXPLICITLY** (it does not pop the key). A deliberate
  "no flips, no rotation" must survive a later measurement; popping would let the
  measured orientation take the view back over — the same bug, harder to see.
* **The first edit seeds from what is on screen.** Discovered by measuring the
  real page, not by reading the code: before any preference exists the controls
  show the measured fallback, so ticking Flip X must mean "what I'm looking at,
  plus flip X". Committing only the named field would silently drop the measured
  flip Y and zero the rotation, and the picture would jump for no stated reason.
* **Malformed stored view ⇒ report ABSENT, not neutral.** Falling back to the
  measurement is recoverable; forcing "no flips" would show a mirrored feed as if
  it were fine.
* **⚠ Restore CLEARS as well as sets.** A slot's `CameraWidget` outlives the
  source assigned to it, so a camera with no view preference must actively hand
  the view back to the measurement — otherwise reassigning a slot leaves the new
  camera showing the previous one's rotation. Same hazard as the v7.16 crop
  restore; pinned in both the manager and the page.
* **The readout names both.** "Rotation vs stage: 180°" would otherwise read as a
  claim about the feed. A `· live view: flip X, +180.0°` clause appears only when
  a preference exists.

## Migration

`_migrate_12_to_13` seeds `view_orientation` from each camera's existing
geometry, so **every camera looks exactly as it did before the upgrade** and the
two evolve independently from then on. A camera whose geometry is fully neutral
gets nothing written (absent ⇒ falls back to neutral anyway), so those entries
stay byte-identical. Never re-seeds (guarded on the key already existing), so it
cannot overwrite a view the operator has since changed.

## 🐞 Unrelated defect found while tracing consumers (fixed)

`capture_controller._orientation` did
`rot, fx, fy, _out = resolve_camera_orientation(...)` — unpacking **four** values
from a 3-tuple (`_resolve_orientation` is the 4-tuple one, with the mosaic's
display-only output rotation appended). The `ValueError` landed in the bare
`except` below, so whenever no `_orientation_fn` was supplied **every capture was
stamped with a neutral orientation**, silently claiming `pixels_stage_aligned`
for a rotated camera. Verified before/after: `(0.0, False, False)` →
`(-90.0, True, False)`. `test_v714_capture_core` 47 green.

## Implementation Steps

- [x] Store: schema 1.3 + view accessors + migration
- [x] Manager: `_view_orient` cache + display-orientation API + store restore
- [x] `CameraFeedView`: read `display_orientation`
- [x] Hardware Setup: controls commit display only; sync + readout + tooltips
- [x] Calibration page: plate-location feed reads the view
- [x] Fix the `capture_controller` 4-tuple unpack
- [x] New test suite + update the 6 tests whose contract changed
- [x] Mutation-verify
- [x] Regression sweep

## Testing Notes

**NEW `tests/test_v716_live_view_vs_measured_orientation.py` — 28 tests**,
driving the PRODUCTION `CameraCalibrationStore`, `CameraManager`,
`CameraFeedView` and `HardwareSetupPage` (a stand-in that reads the right
accessor proves nothing about the widget the operator looks at).

**6/6 mutations CAUGHT**, each a real source edit reverted afterwards:

| # | Mutation | Result |
|---|---|---|
| M1 | feed reads `full_orientation` again (**the original bug**) | 2 failures |
| M2 | slot controls write geometry again (two writers for one number) | 3 failures |
| M3 | the view setter pops a neutral view (measurement retakes it) | 2 failures |
| M4 | migration dropped (existing views silently change on upgrade) | 2 failures |
| M5 | restore sets but never CLEARS (slot inherits previous camera's view) | 2 failures |
| M6 | the mosaic resolver reads the VIEW instead of the measurement | 1 failure |

⚠ **My own smoke assertion was wrong and it is what surfaced the seeding
semantic** — I asserted the first edit would commit only the named field; the
real page (correctly) seeds from the orientation in force. Now pinned by
`TestTheFirstEditSeedsFromWhatIsOnScreen` rather than left as undocumented
behaviour.

**Regression — 538 green across 4 per-suite batches** plus a `gui.app` import
smoke and an offscreen `HardwareSetupPage` walkthrough of all three steps of the
operator's workflow:

* live-view/orientation-audit/square-up/square-crop — 214
* camera-store/scale-FOV/unified-mosaic/image-correction/hw-controls — 162
* mosaic-orientation-adjust+remap/mapping-orient/reanchor/rotation-cal — 115
  (1 pre-existing failure, see below)
* plate-location-click-rim/tucsen-mosaic-FOV/objective-cal/suite-hygiene — 115
* capture-core — 47

**One pre-existing failure, PROVED not ours:**
`test_v75x_mapping_camera_orient_and_rosette_z::TestOrientTile::test_center_pixel_is_invariant_under_rotation`
(`0 not greater than 128`) — in `MosaicBuilder._orient_tile`, a file this diff
does not touch. Re-run with this change **stashed**: still fails.

## Needs GUI/HW verification on ME3B V1, IN ORDER

1. **Nothing looks different on launch** — every camera's feed comes up exactly
   as it did before the upgrade (this is the migration; if a feed changed
   orientation, the seeding is wrong and nothing below is trusted).
2. Tick **Flip X** / **Flip Y** / set **Rotation °** on the microscope slot until
   the live view reads the way you want. The readout gains a
   `· live view: …` clause.
3. **The fix:** run **Mosaic & Camera Calibration** (objective calibration) to
   completion → **the live view must not move**, and the three controls must keep
   their values. The `Rotation vs stage:` figure in the readout *should* change —
   that is the measurement landing.
4. Confirm the mosaic still stitches correctly (the measurement reached the
   geometry): a small scan with no dark seams or wrong-side tiles.
5. **Confirm clicks still land:** click a feature on the live view and drive to
   it — the needle must land on it, at every live-view rotation you try. This is
   the safety claim; a constant offset here means the display leaked into the
   geometry.
6. Restart → both orientations return, on that camera only.
7. Reassign the slot to a different camera that has no view preference → the feed
   must fall back to that camera's measured orientation, **not** inherit the
   previous camera's rotation.
8. Plate Location's live feed should match the Hardware Setup preview.

## Issues & Decisions

* Considered and rejected: leaving the slot controls writing both geometry and
  the view. It fixes the reported symptom (the calibration would run after) but
  keeps a hand-typed viewing angle silently moving the mosaic geometry, and makes
  the outcome depend on the order the operator happens to work in.
* The needle side cams are unaffected in substance: their mount direction
  (`column_dir_deg`) was already a separate measured field, and the square-up
  dialog deliberately keeps reading `full_orientation` because it needs `det F`
  to derive its target angle.
