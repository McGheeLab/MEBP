# MEBP v7.13 — Needle Bore Wizard Redesign

## Objective

Rework the Needle Location tab's bore-calibration wizard into an operator-friendly
workflow (operator: *"the wizard must be much better workflow. it needs to be user
friendly. all videos should be as large as possible"*), with four operator-specified
changes:

1. **Videos as large as possible** — full tab restructure: one big persistent camera
   pane (side cams for steps 1–2, microscope for steps 3–4) beside a compact wizard
   column; no scroll area burying the wizard.
2. **Bore offsets relative to the camera field of view** (operator: *"these offsets
   are not relative to the xy stage, they are relative to the camera field of
   view"*) — the microscope click method becomes THE bore-offset method; the old
   side-camera stage-centring group is retired from view (kept alive hidden as the
   cross-validator); no parking requirement beyond "all bores visible in frame";
   drift is a live readout, not a modal ambush.
3. **Step 2: the plate bottom is contestable** — measure the plate TOP by jog +
   capture, then type a "bottom is X mm below top" offset auto-filled from the
   selected plate type; the bottom is derived, tagged `source="estimated"`.
4. **Bore dots overlay** — after offsets exist, a toggle shows a colored dot per
   bore on the microscope image.
5. (Operator, mid-session): step 4's focal scan must produce **the constant offset
   between the objective focal plane (f, 0–10000 µm) and the needle tip Z** —
   f=5000 µm paired with z=1.050 mm ⇒ one constant f–z offset, verified by its
   constancy across ladder rungs. *"All further downstream calculations … we can
   figure out later"* — consumption of the constant is explicitly deferred.

## Files Modified

| File | Change |
|---|---|
| `gui/widgets/needle_bore_wizard.py` | Flow overhaul (Back/Next, `step_changed`, `showEvent`, inline refusals, drift readout, session anchoring), step-2 rebuild, bore-dot push + toggle, f–z result panel, datum writer rework, `microscope_pane()`/`set_step_panel()` host APIs |
| `gui/pages/calibration.py` | Needle Location tab restructure (camera `QStackedWidget` + wizard column + hidden legacy group + Advanced-Z expander), step-1 panel extraction, new `_zoff_apply_plate_bottom_z` single-writer, `_needle_loc_show_camera_pane` |
| `gui/widgets/camera_feed_view.py` | `set_bore_markers` + `_bore_marker_raw_px` + `_draw_bore_markers` (calibrated-inverse projection, change-gated) |
| `SupportClasses/PlateBottomOptical.py` | New pure `focus_needle_offset_mm(rungs, focal_sign)` — the rung-median f–z constant |
| `gui/widgets/plate_level_wizard.py` | `_objective_name()`; `_focal_sign` reads the datum with the fixed key convention |
| `tests/test_v710_needle_bore_wizard.py` | Modal assertions → inline-refusal contract; +4 new tests (59 total) |
| `tests/test_v79_bore_offset_calibration.py` | One test updated: the side-camera group is retired from view (hidden parent pinned) |
| `tests/test_v75x_plate_z_autocal_tab.py` | `TestNeedleLocationScrolls` (old layout) → `TestNeedleLocationBigPane` (new layout) |
| `tests/test_v713_step2_plate_bottom_from_top.py` | NEW (20) |
| `tests/test_v713_bore_dot_overlay.py` | NEW (16) |
| `tests/test_v713_needle_loc_tab_layout.py` | NEW (19) |

## Implementation Steps

- [x] **Foundations (additive):** `CalibrationPage._zoff_apply_plate_bottom_z(zref, source)`
  — single writer for a COMPUTED plate bottom (`_zoff_set_plate_bottom_z` captures the
  *current* Z and could not be reused); `CameraFeedView.set_bore_markers`;
  `PlateBottomOptical.focus_needle_offset_mm`.
- [x] **Step 2 rebuild:** "Set plate top = current Z" (emits `calibration_changed`
  explicitly — `_set_top_z` does not emit, and without the emit the controller never
  learns the new top); offset spinbox auto-filled from
  `HardwareConfig.plate_z_offsets()` `bottom − top` (both are mm below the same
  fiducial, so the difference is the physical distance, fiducial-independent) with a
  "no stored value — enter from datasheet" hint and a typed-value-wins guard; derived
  bottom preview + explicit Apply (tagged `estimated`); "Set fast-move (safe) Z";
  the old "Auto-fill all from plate type" kept as secondary.
- [x] **Pure helper** `plate_bottom_zref_from_top(top, offset, z_up_sign)` =
  `top − z_up_sign×offset`. ⚠ Uses `z_up_sign()`, NOT `print_z_dir()` — that one is
  *derived from* the taught top/bottom pair, so using it here reads a stale sign from
  a previous plate's references (circular). After applying,
  `derive_z_up_sign(top, bottom)` reproduces `z_up_sign()` by construction.
- [x] **Flow overhaul:** Back/Next row (navigation stays free; a blocked destination
  renders its refusal inline); opens on the **first incomplete step** (was hard-coded
  step 3); one bold instruction line + one status line replace the 4–6 competing
  text surfaces; ~20 refusal `QMessageBox`es → sticky inline refusals (`_refuse`;
  the three *decision* dialogs are kept); commit button disabled with the first
  blocker as its tooltip (`_commit_blockers`, computed in refresh — the operator
  never has to click to find out); live drift readout (green/red, cached position at
  3 Hz so no serial cost; the CLICK gate still re-reads fresh); park is now optional
  — `_on_start_session` ("Start measuring") anchors the session-reference XY at the
  current position, so a hand-jogged stage keeps the stationarity guarantee the
  store math depends on; strip shows ⚠ for an in-progress uncommitted session.
- [x] **🐞 SAFETY FIX — `showEvent` re-arm:** `hideEvent` disarms the print floor and
  nothing re-armed it on re-show until a strip button happened to be clicked; steps
  3/4 jog the needle toward the glass. `showEvent` now re-enters the current step
  (arming the floor on steps 3/4) and re-emits `step_changed` so the host pane syncs.
  Mutation-confirmed CAUGHT.
- [x] **Bore dots overlay:** `CameraFeedView.set_bore_markers(markers)` —
  `(label, dx_um, dy_um, color)` in CAMERA-CENTRE-relative stage µm (pto space), so
  the dot is FIXED in frame and needs no stage tracking; change-gated (3–7 Hz pushes
  are free when unchanged). ⚠ Projection through
  `CameraManager.stage_offset_to_pixel` (the exact inverse of the click path) —
  NEVER the naive identity divide, which is wrong on a rotated/mirrored camera (the
  v7.8 `to_px` lesson; `_draw_reference_markers`' identity projection is a known
  flaw deliberately not copied). Mutation-confirmed CAUGHT (identity-degraded
  projection fails the round-trip test).
- [x] **Wizard marker push:** session clicks render as dots immediately (instant
  visual feedback — previously a click's only record was a text label); committed
  offsets render via `needle_camera_offset + offset_um(k)` (exact by
  `pto(P_k) = pto(P_0) + offset_um(k)`), falling back to each bore's `stage_um`
  provenance ONLY when the invariant `stage_um(k) − stage_um(0) ≈ offset_um(k)`
  holds — the legacy side-camera path wrote ABSOLUTE stage µm into the same field
  (opposite difference sense), and projecting those would draw plausibly-wrong dots.
  Toggle checkbox in the microscope pane header, auto-ON at commit, hidden when no
  source derives. Datum dot green, then blue/yellow/pink/teal cycling.
- [x] **Step 4 f–z result:** `focus_needle_offset_mm` — every usable rung pairs a
  fitted tip focus with the needle Z read back at that moment, each an independent
  estimate of the constant in `needle_z = focal_sign×f/1000 + offset`; median +
  spread, constancy across rungs IS the verification. Headline label
  `_s4_result`: "Focal-plane ↔ needle-tip offset: +X.XXX mm (spread Y µm over N
  margins)", yellow above `DEFAULT_SPREAD_TOL_UM`.
- [x] **🐞 Datum key mismatch FIXED:** `_write_focus_datum` passed the composite
  `"cam|objective"` (`_ploc_camera_objective_key()`) as the CAMERA half of the
  `PlateFocusDatumStore` key with an empty objective, while the only reader
  (`plate_level_wizard._focal_sign`) passed the bare camera identity — **the keys
  could never match**, so the stored datum was dead on arrival. Both now resolve
  (bare identity, `camera_config.current_objective_name`). Mutation-confirmed
  CAUGHT (reverting the writer fails the key-agreement test). Also: the writer now
  persists the **rung-median offset** (n independent tip pairs beat the single
  hand-focused on-glass pair; `focal_sign_and_offset` remains the sign source and
  the fallback), stamps `zero_z_mm` (making `is_stale()` live — the epoch was never
  written before) and the turret position.
- [x] **Tab restructure** (`_build_needle_location_tab`): banner + horizontal
  splitter [`_needle_loc_camera_stack` (stretch 3) | wizard column (stretch 1, min
  s(360))]. Stack page 0 = the side-cam pair (unchanged widgets/attribute names),
  page 1 = `wiz.microscope_pane()` (built by the wizard, parented hidden until the
  host mounts it — no leak, no floating window in standalone builds). Wizard's
  `step_changed` → `_needle_loc_show_camera_pane` (steps `bores`/`touchoff` → page
  1). Step-1's real controls (pick grid, goto/set/last-known, Z checkboxes, action
  row — all `_needle_loc_*` names unchanged) extracted into a panel mounted INSIDE
  the wizard's step-1 page via new `wiz.set_step_panel(...)`. `_build_z_offset_content`
  demoted to a collapsed "Advanced Z references…" `QToolButton` expander (its
  internal scroll retained). The mic groupbox's `s(300)` min-height dropped — the
  big pane provides the height.
- [x] **Legacy side-camera group retired from view, kept alive:**
  `_build_bore_offset_group()` and every `_bore_cal_*` method byte-identical
  (cross-validator; `bore_offsets_changed → _bore_cal_apply_stored/_bore_cal_refresh`
  wiring kept — that is still the store→needle re-apply path). Mounted inside a
  **hidden parent QWidget**: `_bore_cal_refresh` calls `setVisible(True)` on it for
  a multi-bore needle, and an unparented widget would float as a top-level window.
- [x] Tests + regression (below).

## Testing Notes

**New:** `test_v713_step2_plate_bottom_from_top.py` (20 — both polarities incl. the
self-consistency `derive_z_up_sign(top, bottom) == z_up_sign`; autofill/hint/
typed-wins/force; apply tags `estimated`; top capture emits; the untagged app.py
re-push cannot erase the source tag — pinned on the real `StageController`),
`test_v713_bore_dot_overlay.py` (16 — round-trip through a rotated+mirrored fake
manager implementing the real forward/inverse pair; a companion test pins that the
naive identity projection is >50 px wrong on that camera; change-gating; the 3-way
stored-dots derivation incl. legacy absolute-provenance rejection; toggle
behaviour), `test_v713_needle_loc_tab_layout.py` (19 — `focus_needle_offset_mm`
math incl. the sign-flip mutation and median-vs-outlier; **writer key == reader
key** for the focus datum; `_write_focus_datum` writes the rung-median offset +
`zero_z_mm` and `is_stale` fires; real-page layout: stack 2 pages, step drives the
index, step-1 controls inside the wizard page, legacy group parented+hidden+not-a-
window, no burying scroll, wizard opens on first incomplete step, `showEvent`
re-arms the floor + re-emits `step_changed`, Back/Next walk the order).

**Updated:** `test_v710_needle_bore_wizard.py` — 5 modal assertions → the inline
contract (each also asserts `_FakeMB.warned == []`: no modal on a refusal), +4 new
(drift readout red/green, session anchoring at the current position, gated session
refusal, commit-button disabled-with-tooltip) → 59 green.
`test_v79_bore_offset_calibration.py` — one test updated deliberately: the group's
own visible flag flips while `isVisibleTo(page)` stays False, `isWindow()` False
(pins the hidden-parent requirement). `test_v75x_plate_z_autocal_tab.py` —
`TestNeedleLocationScrolls` (pinned the old scrolled layout) replaced by
`TestNeedleLocationBigPane` (video pane wider than the wizard column at 1400×700;
no scroll area above the wizard; side-cam floor kept; the mic feed gets pane height
on step 3).

**Mutations confirmed CAUGHT (4/4, each a real source edit reverted in a finally):**
step-2 polarity sign flipped → bottom lands on the wrong side of the top ·
bore-dot projection degraded to the naive identity · `showEvent` re-arm removed →
floor stays off after a page revisit · datum key fix reverted → writer and reader
keys diverge.

**Regression, run per-suite** (the repo's documented cross-suite camera hang):
v710 wizard/gate/focus-roi/sign/square-up/orientation-audit, v711
bottom-wizard/optical/worker/level-wizard/level-worker/datum/host-contract/focus-
state, v712 phase0/builder-ui, v713 andor-raw-stats/cell-targeting-surface-z, v79
bore-offset (93), v731 jog-nav/integration, ~25 `CalibrationPage`-importing v75x
suites (needle-location-quick-move, cal-z-envelope, needle-offset-z-side-view,
plate-location×4, plate-types, autocal×2, mosaic×4, reanchor, rosette-reanchor,
single-well, startup-well-map, last-known-cal, camera×3, needle-cam-mount,
needle-center-direction, pump-compliance, spheroid-sink, template-reregister,
suite-hygiene) — **all green**, plus a `gui.app` import smoke and an offscreen
`CalibrationPage` build driving the stack through the wizard steps.

**Four pre-existing failures PROVED not ours:**
1. `test_v75x_plate_template_reregister::test_scan_well_guarded_without_rosette` —
   calls `_ploc_scan_rosette_well`, which exists 0× anywhere (documented in the
   v7.12 plan row).
2. `test_v75x_pump_compliance_and_backlash::test_tabs_restructured` and
3. `test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices` — both expect
   the tab title "Plate Z Auto-Cal"; the operator's uncommitted work renamed it
   "Plate Bed Level" (documented in the v7.12 plan row).
4. `test_v75x_plate_mosaic::TestMosaicSettings::test_dialog_round_trip_and_defaults`
   — an uncommitted 8-line `avg_frames` change in `gui/dialogs/mosaic_settings_dialog.py`
   (verified by `git diff`; the test file is unmodified). The suite's
   `TestManualAlignPage` hang and `test_real_24_well_mosaic` CV failure are the
   long-documented environmental items; the page-touching classes were run
   individually and are green.

## Issues & Decisions

- **"Relative to the camera" was already the math** — `offset_um(k) = pto(P_k) −
  pto(P_0)` is a camera-frame difference and stage position cancels
  (`NeedleBoreCalibrationStore.py:141-198`, untouched). What was stage-flavoured was
  the UX: the mandatory park at the saved needle location, and the drift modal. The
  redesign made the session anchor explicit ("Start measuring" at ANY position) and
  the drift a readout — the stationarity requirement is unchanged and still refuses
  clicks above `MAX_STAGE_DRIFT_UM`.
- **Operator decisions (AskUserQuestion, 4 + 1 follow-up):** camera clicks only,
  side-camera group unmounted · plate top by jog+capture · auto-fill from plate-type
  offsets then editable · full layout restructure · (follow-up) step 4's deliverable
  is the f–z constant itself; downstream consumption deferred.
- **`print_z_dir()` circularity** — the step-2 derivation must use `z_up_sign()`;
  `print_z_dir()` is derived from the very top/bottom pair being established.
- **`stage_um` provenance is polluted** across store generations (wizard writes pto,
  legacy side-camera wrote absolute µm into the same field) — the overlay's
  invariant check is the guard; it can never be "tidied away".
- **The wizard opens where the work is** (`_initial_step`), not on a hard-coded
  step 3; construction commands nothing (floor arming moved to
  `showEvent`/`go_to_step`, pinned by test).
- **`_zoff_apply_plate_bottom_z` is a NEW single writer**, not a reuse:
  `_zoff_set_plate_bottom_z` captures the current stage Z and takes no value. The
  AST host-accessor contract (`test_v711_host_accessor_contract`) passes because the
  method exists on the real page.
- **Deliberately NOT done:** consuming the f–z constant downstream (operator:
  *"figure out later"*); per-well surface deltas via the plate-level wizard; the
  reference-marker identity-projection flaw in `_draw_reference_markers` (pre-existing,
  separate fix — the new bore-dot path does it right).

## ⚠ Needs real-HW verification on ME3B V1, IN ORDER

1. Open Calibration → Needle Location: the side-cam pair fills the big left pane;
   the wizard column reads step 1 with the centring controls inside it.
2. Next → step 3: the pane switches to the microscope feed at full size; "Go to
   needle at survey height" parks and starts a session, OR jog by hand and press
   "Start measuring" — the drift readout goes green.
3. Click each bore tip: a colored dot lands under every click immediately; jog XY
   deliberately >5 µm and confirm clicks are refused inline (status + red drift
   line, no popup) and re-anchoring re-clicks cleanly.
4. Save bore offsets → the dots persist (toggle ON), then drive each bore to one
   target and check DIRECTION, not just distance (the standing v7.10 check).
5. Step 2 on a plate type with stored offsets: the bottom-offset box auto-fills;
   capture the plate top with the tip touching the top surface; Apply → the bottom
   reads "(estimated)"; cross-check against a contact touch-off on the same well.
6. Step 4 optical ladder: the headline reads a constant f–z offset whose spread over
   the margins is ≤ ~25 µm; restart the app and confirm the datum record survives
   (`config/hardware/plate_focus_datum.json` carries `zero_z_mm`, the bare camera
   identity and the objective name in its key).
7. Resize the window to the bench monitor's real size: no horizontal scrollbars in
   the wizard column, the video stays the biggest thing on screen.
