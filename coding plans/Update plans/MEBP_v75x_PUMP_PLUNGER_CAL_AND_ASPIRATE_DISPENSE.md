# MEBP v7.5.x — Pump Plunger Calibration + ASPIRATE/DISPENSE Vocabulary

**Status:** in progress · **Needs real-HW verification on ME3B V1**

## Objective

1. Give each syringe pump (P1/P2/P3) a **plunger zero/max calibration** that mirrors the Z stage's
   "Set Bottom / Set Top" (`StageController.apply_z_setup`):
   - **ZERO = plunger all the way IN = syringe empty** (= the dispensed datum → fill 0).
   - **MAX = plunger all the way OUT = syringe full** (= the aspirated extreme).
   One capture flow sets the datum, **derives** the dispense/aspirate direction, and sets the soft
   limits. The pump position readout is reframed into a **fill level** (0 = empty → capacity = full).
2. Make the plunger vocabulary unambiguous **everywhere**: **ASPIRATE** = draw fluid IN (toward
   max/full), **DISPENSE** = push fluid OUT (toward zero/empty). Eliminate the scattered synonyms
   **extrude / expel / eject / draw** in favour of aspirate/dispense (UI, comments, docstrings, public
   identifiers, and persisted config keys with back-compat shims). We do **not** introduce "inject";
   `aspirate`/`dispense` are already the dominant correct terms.

## Canonical convention (single source of truth)

Lives atop `StageController.move_pump_uL` + a CLAUDE.md bullet:

```
ASPIRATE = draw fluid IN  (plunger toward MAX/full; raises fill).
DISPENSE = push fluid OUT (plunger toward ZERO/empty; lowers fill).
Volume-delta sign: + = dispense, − = aspirate.
Fill/position: 0 = empty (fully dispensed), capacity = full (fully aspirated).
Per-pump dispense/aspirate DIRECTION is OWNED by the calibration (apply_pump_setup),
the same way z_up_sign owns Z. Reserved terms NOT renamed:
  'prime'   = start-of-print pre-flow lead-in (its own concept)
  'retract' = Z safe-travel raise / pump pressure-relief (overloaded; Z dominates)
```

## Two non-negotiable invariants

- **Direction-ownership re-validation:** owning direction retires the per-pump `_flip_sign`/`axis_flip`
  for *calibrated* pumps. A wrong derived sign reverses fluid flow → **re-run pump calibration on each
  machine after deploy.** Uncalibrated pumps keep the legacy path, so nothing changes until a pump is
  calibrated.
- **Preserve `CommandType` value `"extrude"`:** `PrintExecutionLogger` persists `cmd.type.value` into
  exec-log manifests. Renaming the member to `DISPENSE` must keep `value="extrude"` for forensic
  comparability.

## Files modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | `apply_pump_setup` / `apply_pump_convention` / `capture_current_pump_raw` / `pump_aspirate_sign`+setter / `pump_dir_sign` / fill helpers (`pump_capacity_uL`, `raw_to_pump_fill_uL`, `pump_fill_uL`, `pump_fill_uL_to_raw`); direction ownership in `move_pump_relative`; canonical docstring on `move_pump_uL`; `_pump_aspirate_sign`/`_pump_setup` init; wire `pump_dir_sign_provider` into ZPJogHandler |
| (same, ZPJogHandler) | jog deltas use `pump_dir_sign` provider instead of `_flip_sign` for pumps |
| `SupportClasses/SafetyLimits.py` | `update_from_hardware_config(..., skip_pumps=())` clobber guard; sign-comment wording |
| `gui/pages/hardware/device_profile.py` | `pump_setup` field across dataclass / `to_dict` / `from_dict` / `from_settings` / `apply_to_settings` |
| `main.py` | `apply_pump_convention(...)` restore after `apply_z_convention` |
| `gui/pages/hardware_setup.py` | "Pump Plunger Setup" group + capture handlers; clobber-guard the hw-config pump-limit overwrite |
| `gui/pages/hardware/control_panel.py` | pump readout reframed to fill µL + bar ranges 0→capacity |
| `SupportClasses/{PrintManager,PickAndPlaceManager,WellSetup,PrintPlanOfAction,PrintTrajectoryPlanner,PhysicalModels}.py` | vocabulary: extrude/expel/eject/draw → aspirate/dispense (strings, comments, identifiers, config keys + shims) |
| `gui/pages/workflows/*`, `gui/pages/print_setup_legacy.py`, `INSTRUCTIONS.md`, `CLAUDE.md` | UI strings + docs |
| `tests/test_v75x_pump_plunger_setup.py` (new) | calibration + fill + persistence + direction tests |

## Implementation steps

- [x] 0. This plan doc.
- [x] 1. StageController pump calibration backend + canonical docstring.
- [x] 2. Direction ownership (`pump_dir_sign` in `move_pump_relative` + ZPJogHandler).
- [x] 3. `DeviceProfile.pump_setup` (×5 methods incl. dataclass field).
- [x] 4. `main.py` restore + `SafetyLimits.update_from_hardware_config` clobber guard.
- [x] 5. GUI "Pump Plunger Setup" group + handlers.
- [x] 6. control_panel fill-µL readout + bar ranges.
- [x] 7. Tests (`tests/test_v75x_pump_plunger_setup.py`, 19 incl. offscreen GUI smoke).
- [x] 8. Vocabulary B1 (strings/comments/docstrings → aspirate/dispense).
- [x] 9. Vocabulary B2 (public identifiers: `CommandType.DISPENSE` value-kept, `post_dispense_needles`, `dispense_uL` + alias, stress-test idents).
- [x] 10. Vocabulary B3 (config keys: `dispense_rate_uL_s` + translating `from_dict` shim).
- [x] 11. CLAUDE.md convention bullet + plan-table row; verification.

## Verification results

- `tests/test_v75x_pump_plunger_setup.py` — 19/19 green (backend round-trip both polarities,
  fill conversion, direction guard, owned-direction motion mapping, persistence, offscreen page
  smoke + capture flow).
- Touched suites green: pump-settle/spheroid/zp-envelope/xbox-speed/jog-nav (88), print-setup/
  multi-object/cell-targeting/simple-pm/workflow-settings/quick-print/stress (198 across the batch),
  print-execution-logging/path-barrier/print-z/feedrate (50), trajectory-regen/print-z-plate-bottom.
- Pre-existing unrelated failures (present in HEAD, NOT introduced here): `test_v726_print_execution`
  (`_make_mock_plate` 3-tuple bug, errors=4) and `test_v73_trajectory_planner` (MagicMock `<` compare,
  errors=6) — both confirmed identical error counts on the unmodified HEAD files.
- Legacy `config/prtcfg.json` with the old `eject_rate_uL_s` key still loads via the shim.
- No stale references to renamed symbols; core modules import cleanly.

## Testing notes

- `python -m pytest tests/test_v75x_pump_plunger_setup.py` + touched suites (pick-place, print,
  well-setup, z-axis-setup, quick-print, stress). Loading an old `config/prtcfg.json` with the legacy
  `eject_rate_uL_s` key must still work (back-compat shim).
- Offscreen page-build smoke for the Pump sub-page.
- Real-HW: per pump, jog all-in → Set Dispensed (fill 0); jog all-out → Set Aspirated (fill≈capacity,
  direction_ok green); confirm a manual aspirate raises fill and dispense lowers it; soft limits stop
  over-aspirate past full / over-dispense past empty; restart restores values.

## Issues & decisions

- **Soft-limit envelope = EXACT captured extremes (no margin).** Deviates from the initial plan's
  "±2% margin" idea: expanding the envelope beyond the captured *mechanical* extremes would command the
  plunger past its hard stops. Mirrors Z (exact min/max). Setup-mode jog uses `bypass_safety=True`.
- **`get_pump_position_uL` contract unchanged** (it returns `mm_to_uL(raw − zero)`, a delta the
  print/extrude path relies on). Fill is a separate, additive readout (`pump_fill_uL`).

### Addendum — pump jog buttons read ASPIRATE / DISPENSE on the side panels

Operator follow-on: on every jog **side** panel (NOT the Hardware Setup page), the pump up/down
buttons now read **Aspirate** (top — draws fluid IN, raises fill, emits a negative step) and
**Dispense** (bottom — pushes fluid OUT, lowers fill, emits a positive step) instead of the
`P{n} ▲/▼` extend/retract arrows. The Hardware Setup page (its `control_panel` left panel + the
Device-tab `stage_panel` jog) keeps the raw ▲/▼ arrows.

- `JogButtonArray` gains `pump_action_labels: bool = False` + `_build_pump_action_columns` /
  `_action_btn` (wider word buttons + a `P{n}` column header). The emitted
  `jog_pump_requested(pump, distance)` sign convention is unchanged (`+` = dispense, `−` = aspirate);
  only which button carries which sign + the label differ, so consumers are untouched. With the
  calibration's owned direction, Aspirate physically raises the fill readout and Dispense lowers it.
- `HardwareControlPanel` gains `pump_action_labels` (threaded to its `JogButtonArray`). Hardware
  Setup creates it plainly (`HardwareControlPanel()` → arrows); `StandardJogContextPanel` (jog page +
  all workflow side panels + calibration) creates it with `pump_action_labels=True`; the Settings
  page's standalone `JogButtonArray` also passes `pump_action_labels=True`.
- Verified offscreen (both variants: labels + sign mapping) + jog/workflow/calibration GUI suites
  green (140).

---

## Addendum — Plunger Setup relocated to the Device (Stage) page, one block per pump

Operator request: move the syringe (plunger) **zero & max location selection**
to the **first calibration page, directly under the Z setup section**, with each
syringe as its **own block** ordered **Z, P1, P2, P3**, and have capturing the
zero/max **update + save the calibration extents**.

- **Moved** the plunger setup off `HardwareSetupPage`'s **Pump** sub-page onto
  the **Device (Stage)** sub-page (`StageHardwarePanel`, the first HW sub-page),
  placed immediately **after `_build_z_axis_setup_group()`** so the page reads
  Z Axis Setup → **P1 → P2 → P3** plunger blocks → per-axis safety-limit grid.
- **One block per pump** (no pump combo): new
  `StageHardwarePanel._build_pump_plunger_setup_block(pump)` ×3, with per-pump
  `_pump_setup_dispensed_btns` / `_pump_setup_aspirated_btns` /
  `_pump_setup_status_lbls` / `_pump_setup_dispensed_raw` dicts and per-pump
  handlers `_pump_setup_capture_dispensed(pump)` / `_pump_setup_capture_aspirated(pump)`
  (each captures its own extremes independently; arming is per-block).
- **Update + save the calibration extents:** `_pump_setup_persist(pump)` calls
  `apply_pump_setup` (derives direction + sets `safety_limits.p{n}_min/max`),
  mirrors the captured extents into the visible `spin_p_mins/spin_p_maxs`
  spinboxes, persists `zero_position` + `safety_limits.p{n}_min/max` +
  `device_profile.pump_setup`, and calls `_persist_active_profile()` (same
  persistence the Z setup uses — `_persist_active_profile` now reached directly
  on the panel that owns it, no cross-page `self._stage_panel` hop).
- **Removed** `_build_pump_plunger_setup_group` + the combo-based handlers from
  `hardware_setup.py`; the Pump sub-page now only carries Pump Channels + Pump
  Timing. No other call sites referenced the old symbols (repo-wide grep).
- Tests: `tests/test_v75x_pump_plunger_setup.py::TestPumpPlungerSetupPanel`
  rebuilt to drive `StageHardwarePanel` (blocks-exist / capture+persist /
  per-pump independence / extents-spinboxes-updated / cancel) — 20 green;
  z-display / ink-location / camera-image-correction page suites green (43).
  **Needs real-HW verification on ME3B V1.**
