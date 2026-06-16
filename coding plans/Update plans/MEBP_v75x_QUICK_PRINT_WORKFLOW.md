# MEBP v7.5.x — Quick Print Workflow

## Objective

Add a no-frills **Quick Print** workflow to the Workflows mode: the operator picks
one object (a built-in simple shape or a saved print from the library), clicks one
well, and presses **Print** — the system prints it. The previously-stubbed
`quick_print` tile (rendered "Coming soon") becomes a functional page.

The design **mirrors the standard print flow exactly** so motion is correct by
construction: it builds a single-well `PrintJob` with the same
`build_well_plate_job()` helper Print Setup uses, then runs it through a fresh
`PrintManager` in discrete mode (the same path `app._on_monitor_start` falls into
when a job has no plan/waypoints). No new motion or coordinate math.

## Scope (confirmed with user)

- **Object source = Both** — a dropdown of built-in simple shapes (Dot / Circle /
  Meander-filled disc) *and* saved print objects from `config/prints`.
- **Controls = a few knobs** — Object, Size (for simple shapes), Pump, Flow (µL/s),
  Print-Z (mm). Travel Z auto-uses the calibrated safe Z; Print-Z defaults from
  calibration (`plate_bottom_z` → `plate_top_z`).

## Coordinate / unit contract (verified)

- `build_well_plate_job(well_positions, path_points, ...)` — `well_positions` is
  `[(name, x_mm, y_mm)]` in **zero-ref mm**; `path_points` are **mm relative to well
  center** (`SupportClasses/PrintManager.py:500`).
- Discrete executor: `MOVE_XY {x,y}` → `move_xy_absolute(x, y, from_zero_ref=True)`
  (`PrintManager.py:2106`); `TRAVEL_UP` → `travel_z_height`,
  `MOVE_Z`/`TRAVEL_DOWN` → `print_z_height` (zero-ref mm).
- The page receives well positions via `set_calibration_data(plate, well_positions,
  safe_z)` as **absolute stage µm**; `controller.zero_position` is in µm.
- **Well center** = calibrated position when available, converted to zero-ref mm
  (`(wx_um - zero["x"]) / 1000`), else geometric `plate.get_well_position(name)`
  (already A1-relative mm = zero-ref mm). Both feed `build_well_plate_job` identically.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/workflows/quick_print_workflow.py` | **New** — `QuickPrintWorkflowPage` (header, config row, `WellPlateNavigator`, run row; geometry→path via `GeometryEngine`, job via `build_well_plate_job`, run via `PrintManager` + `_PrintBridge`) |
| `gui/pages/workflows_mode.py` | Import `QuickPrintWorkflowPage`; add `elif tile.workflow_id == "quick_print"` branch instantiating it `(controller, settings)` |
| `gui/pages/workflows/workflow_picker.py` | `quick_print` tile `enabled=True`; description → "Drop one object in a well and print — no setup." |
| `tests/test_v75x_quick_print_workflow.py` | **New** — 15 tests |
| `CLAUDE.md` | Added row to Existing Update Plans table |

## Reused (no reinvention)

- `build_well_plate_job` / `PrintManager.load_job`/`start`/`abort` / `PrintSettings`
  / `PrintState` — `SupportClasses/PrintManager.py`
- `GeometryEngine.PrintObject.from_dict` + `generate_object_trajectory` (XY columns
  only) — handles every saved object type; csv-sourced objects use their persisted
  trajectory. Default-needle fallback `NeedleSpec(gauge=22, od_um=718, id_um=413,
  wall_um=152)` (matches `print_objects._extract_needle_syringe`) so simple shapes
  work before hardware is configured.
- `PrintFileManager.list_files` / `load` / `PrintFileData.objects`
- `WellPlate.get_well_position`
- `gui/widgets/jog_well_plate.py::WellPlateNavigator` (`well_clicked`)
- `gui/widgets/standard_jog_context.py::StandardJogContextPanel` (left context panel)
- Page pattern / thread-bridge / `set_*` hooks mirror
  `gui/pages/workflows/spheroid_pickup_workflow.py`

## Implementation Steps

- [x] Confirm reused-component APIs (units, signatures, PrintState, needle/syringe build)
- [x] Create `QuickPrintWorkflowPage` (UI + geometry pipeline + job build + run/abort)
- [x] Register page in `workflows_mode.py` (import + `elif` branch)
- [x] Enable the `quick_print` tile in `workflow_picker.py`
- [x] Write `tests/test_v75x_quick_print_workflow.py`
- [x] Run tests + headless wiring check
- [x] Update plan doc + CLAUDE.md table row

## Testing Notes

- `python -m unittest tests.test_v75x_quick_print_workflow` — **15 pass** (offscreen
  Qt). Covers: headless construction + simple-shape combo entries; circle/dot/meander
  and a saved-object dict each compile to a non-empty XY path; input object dict not
  mutated; well-center prefers calibrated (→ zero-ref mm) and falls back to geometric;
  Print-button gating; `build_well_plate_job` offsets the path by the well center;
  settings carry safe-Z / Print-Z / flow.
- Headless wiring: `WorkflowsModePage` registers `quick_print` → `QuickPrintWorkflowPage`,
  tile enabled, fanout (`set_hardware_config`/`set_calibration_data`/`set_z_references`)
  does not raise, title delegates to the page.
- Regression: `test_v731_jog_navigation`, `test_v731_integration`,
  `test_v75x_plate_centering` — **83 pass**.
- Manual (real/sim hardware): connect + calibrate (sets safe Z + well positions) →
  Workflows → Quick Print → pick Simple: Circle + a well → Print → needle travels,
  lowers to Print-Z, traces circle while extruding, retracts; Abort raises Z. Repeat
  with a saved print. Edge cases: no safe Z → warning; uncalibrated well → geometric
  fallback; not connected → Print disabled.

## Issues & Decisions

- **Combo userData as string, not tuple** — `QComboBox.findData` does not match Python
  tuples (returns -1), which broke selection restore + path generation. Switched to
  `"kind:ref"` strings parsed with `split(":", 1)` (file names may contain ':').
- **`PrintObject.from_dict` mutates its argument** (pops `trajectory`) — the helper
  deep-copies each object dict first; covered by `test_obj_dict_input_not_mutated`.
- **Own PrintManager instance** — the page creates its own `PrintManager(controller)`
  per run (mirrors spheroid's own-executor pattern). `start()` spawns its own daemon
  thread and `_start_recorder()` safely no-ops when no recorder is attached.
- **Meander = filled circular meander** (filled circle) rather than a square raster, so
  the fill stays inside a round well.
- **Discrete execution** chosen over hybrid: simpler/predictable for a single object and
  it needs no `PrintPlanOfAction`; identical to the legacy "Send to Monitor → discrete"
  path.
