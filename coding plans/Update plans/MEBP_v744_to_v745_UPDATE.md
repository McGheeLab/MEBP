# MEBP v7.4.4 → v7.4.5 Update Plan

## Objective

Replace the one-line "Format:" combo on the Hardware Setup → Plate sub-page with a full SolidWorks-style parametric well-plate designer. Users can:

- Place wells of any size, anywhere — Single Well or Grid Pattern tools
- Name each well freely (A1, B5, or arbitrary strings) — Grid tool defaults to ANSI labels
- Drag wells with the mouse; the solver re-arranges the rest of the sketch to honor constraints
- Add geometric constraints (Lock, Distance, Horizontal/Vertical, Concentric, Equal-Ø, Coincident)
- Save custom plates with a name; coexist with the 6 standard formats as starting templates
- See live DOF status (under-/well-/over-constrained) in a status bar at the bottom of the sub-page

Downstream consumers (Calibration, Jog, Print Setup, Print execution) treat custom plates identically to standards.

Branch: continues on `Version-7.4.2` for now; bump to `Version-7.4.5` at completion per CLAUDE.md checklist.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/WellPlate.py` | `WellPlate.format: int | str` (custom plates use `"custom:<name>"`); new `WellPlate.from_wells(name, wells, **meta)` factory bypassing the grid `__post_init__`; new `WellPlate.load(name_or_format)` polymorphic factory. Existing `from_format(int)` preserved unchanged so legacy callers keep working. `__post_init__` guards the grid auto-fill behind `if not self._wells`. |
| `SupportClasses/HardwareConfig.py` | New `plate_name: str = ""` field. New `active_plate_key` property returning `plate_name if plate_name else plate_format`. `to_dict()` / `from_dict()` migration: `plate_name` written + read alongside `plate_format` for one release cycle. Validation tolerates custom plate names. |
| `SupportClasses/PlateDesign.py` (new) | Editable sketch document: `Entity`, `Point`, `Line`, `Circle`, `Well`, `Group`, `PlateOutline`, `PlateDesign` dataclasses + tagged `Constraint`. `PlateDesign.compile() -> WellPlate` resolves all entities to absolute positions and emits a `WellPlate` via `from_wells`. JSON serialization round-trip. `PlateDesign.from_standard_format(int)` factory that clones a standard format into an editable design. |
| `SupportClasses/PlateSketchSolver.py` (new) | scipy `least_squares` Levenberg-Marquardt solver. Free-variable vector indexer, residual assembly per constraint kind, analytic Jacobians where short, finite-diff fallback otherwise. Drag-pin lifecycle for mouse interactions. `SolveReport` (status / residual / rank-deficit / conflicting constraints). |
| `gui/pages/hardware/plate_designer.py` (new) | `PlateDesignerWidget(QWidget)` composite: header (picker combo + Save / Save As / Delete buttons + Fit / Grid / Snap toggles) + toolbar column + central canvas + properties panel + DOF status bar. Owns `PlateDesign` + solver lifecycle. Mediates file IO for `config/hardware/plates/user/<name>.json`. |
| `gui/widgets/plate_designer_canvas.py` (new) | `PlateDesignerCanvas(QGraphicsView)` with custom scene items (`PlateOutlineItem`, `GridItem`, `WellItem`, `ConstraintMarkerItem`, `SnapIndicatorItem`). Tool state machine (Select / Single Well / Grid / Add Constraint / Lock / Delete). Drag with solver pin (33 ms debounce). Scroll-wheel zoom + middle-mouse pan. Distance / Ø / lock marker rendering. |
| `gui/pages/hardware_setup.py` | Replace `_sub_layouts["plate"]` block (lines 676–692) with `PlateDesignerWidget` instantiation. Special-case `"plate"` in `_setup_ui()` sub-page scaffold so the canvas fills the viewport (no inner scroll). `_rebuild_config()` writes `plate_name`. `_apply_config_to_ui()` calls `self._plate_designer.load_plate(active_key)`. |
| `gui/widgets/well_plate_view.py` | Read radius per-well from `WellInfo.diameter` instead of `WellPlate.well_diameter` so custom plates with varied well sizes render correctly. |
| `tests/test_v745_plate_design.py` (new) | Round-trip serialization, `PlateDesign.compile() → WellPlate` correctness, `from_standard_format()` equivalence to `WellPlate.from_format(int)`. |
| `tests/test_v745_plate_sketch_solver.py` (new) | Solver convergence on hand-built sketches, DOF detection (well/under/over), drag-pin lifecycle, conflicting-constraint reporting. |
| `tests/test_v745_well_plate_load.py` (new) | `WellPlate.load(6)` equivalent to `WellPlate.from_format(6)`; `WellPlate.load("custom-name")` round-trip; backward-compat with `from_format` alias. |
| `coding plans/Architectures/ARCHITECTURE_V745.md` *(new — at completion)* | Standard architecture-doc delta for the new version. |
| `CLAUDE.md` | New entry in *Existing Update Plans* table on completion; current-version bump. |

## Implementation Steps

- [x] Read ARCHITECTURE_V742.md + key source files (WellPlate, HardwareConfig, hardware_setup, well_plate_view)
- [x] Create this update plan
- [x] Backend: generalize `WellPlate` — `format: int|str`, `from_wells()`, `load()`, guarded `__post_init__`
- [x] Backend: `HardwareConfig.plate_name` + `active_plate_key` + JSON round-trip migration
- [x] Backend: `PlateDesign` data model + JSON round-trip + `compile() → WellPlate` + `from_standard_format(int)`
- [x] Backend: `PlateSketchSolver` — LM + Jacobians + DOF detection + drag-pin lifecycle
- [x] UI: `PlateDesignerCanvas` — scene + items + tool state machine + drag + zoom/pan
- [x] UI: `PlateDesignerWidget` — header + toolbar + canvas + properties panel + status bar
- [x] Integrate: replace combo in `hardware_setup.py`, scroll-scaffold special-case, `_rebuild_config()` / `_apply_config_to_ui()` updates
- [x] Render: WellPlateView reads per-well diameter via `WellInfo.diameter` (custom plates)
- [x] Tests: unit tests for design, solver, load factory (`tests/test_v745_plate_design.py`, `test_v745_plate_sketch_solver.py`, `test_v745_canvas_history.py`)
- [x] Manual verification: launch app, walk Plate sub-page golden path
- [x] Architecture doc `ARCHITECTURE_V745.md` (delta against v742)
- [x] Update `CLAUDE.md` (version bumped to v7.4.6 alongside v745+v746 plan entries)

## Testing Notes

### Unit tests

```bash
python -m unittest discover tests/ -p "test_v745_*.py"
python -m unittest discover tests/  # full suite, regression check
```

- Solver converges to ground truth on hand-built 12-well plate.
- DOF detection: well-/under-/over-determined hand-built cases each return the expected status.
- Drag-pin: pinning A1 to (10, 10) on a fully-constrained grid produces the expected translation of all linked wells.
- `WellPlate.load(6)` returns a plate equivalent to `WellPlate.from_format(6)`.
- `WellPlate.load("test-plate")` round-trips via save → load.
- `PlateDesign.from_standard_format(96).compile()` yields a plate whose A1..H12 positions match `WellPlate.from_format(96)` exactly.
- Existing test suite passes unmodified — verifies `from_format` alias and that `__post_init__` guard does not regress standard formats.

### Manual end-to-end

```bash
python3 main.py
```

1. Hardware Setup → Plate sub-page → designer renders, picker shows 6 standards.
2. Pick `96-well` → 96 wells appear in proper ANSI positions.
3. Switch to Single Well tool → click at (50, 50) → new well appears (default Ø).
4. Drag a well → siblings stay put when under-constrained; DOF bar shows amber.
5. Multi-select two wells → Add Constraint → Distance H = 15 mm → solver moves one; DOF bar turns green.
6. Save As "test-plate" → file appears in `config/hardware/plates/user/test-plate.json`.
7. Switch picker to `12-well` then back to `test-plate` → reloads identically.
8. Quit + relaunch → `HardwareConfig.plate_name == "test-plate"`; designer reopens to the saved plate.
9. Navigate to Calibration page → A1 auto-calibration uses the custom plate's A1 position.
10. Navigate to Print Setup → well picker shows the custom names.
11. Delete `test-plate` from the picker → confirm dialog → file removed.

## Issues & Decisions

- **Wells own a center `Point` rather than embedding x/y.** Every constraint operates on points/lines/circles, never on `Well` directly. This makes "distance between two wells" representable as `distance_pp(w1.center, w2.center)` — no special-cased well-to-well constraints needed. The `Well` is a labeled wrapper around a center-point + diameter.
- **WellPlate kept backward-compatible.** `from_format(int)` works exactly as before — no JSON file lookups for standard formats. `load(int|str)` adds the new polymorphic entry point. Existing callers (`WellSetup`, `calibration.py`, `jog_control.py`, etc.) keep working unchanged via the alias. Only new code that needs to support custom plate names calls `load(cfg.active_plate_key)`.
- **No bundled JSON files for standards.** Standards are computed in-code via `PlateDesign.from_standard_format(int)` (which clones from `PLATE_DEFINITIONS`). Avoids maintaining 6 JSON files that mirror the dict, and removes a file-IO dependency at app startup. Only user-created custom plates are file-backed.
- **Solver uses squared distance form.** Residuals like `|p1-p2|² - d²` avoid the `1/r → ∞` blowup when two points temporarily coincide during dragging.
- **Drag uses a transient "ghost" constraint.** On mouseDown, push a `drag_ghost` constraint with high weight (1000); on mouseMove, update its target to the snapped cursor position and re-solve from the previous solution (LM converges in 1–3 iterations on warm start). On mouseUp, pop the ghost.
- **DOF status bar surfaces solver state.** Green ✓ when well-determined; amber ⚠ when underdetermined (sketch is free to drift); red ✗ when over-constrained or inconsistent (highlights the top conflicting constraints). Failing to converge does not mutate the design — last-good positions remain on screen.
- **Per-well diameter.** Each `Well` stores its own diameter, defaulting to a plate-level value for the grid tool. `WellPlate.well_diameter` becomes `0.0` for custom plates ("varies — see WellInfo"); downstream code that previously read `plate.well_diameter` migrates to `well.diameter` from `WellInfo`. `well_plate_view.py` updated to render per-well.
- **Shipping scope.** v7.4.5 ships Select / Single Well / Grid tools and the 8 constraint kinds in the table below. Circle Pattern, Line/Construction Line, and the more exotic constraints (Tangent, Symmetric, Equal-Length, Point-on-Line/Circle) defer to a follow-up update along with Undo/Redo via `QUndoStack`.

### Constraint Kinds Shipped in v7.4.5

| Kind | Accepts | Residual | DOF |
|---|---|---|---|
| `ground` | 1 Point | `[px, py]` | 2 |
| `fix` | 1 Point | `[px-x0, py-y0]` (snapshot at lock) | 2 |
| `coincident_pp` | 2 Points | `[Δx, Δy]` | 2 |
| `distance_pp` | 2 Points, value d | `[\|p1-p2\|² - d²]` | 1 |
| `horizontal` | 2 Points | `[Δy]` | 1 |
| `vertical` | 2 Points | `[Δx]` | 1 |
| `concentric` | 2 Circles | `[Δcx, Δcy]` | 2 |
| `equal_radius` | 2 Wells/Circles | `[r1 - r2]` | 1 |
| `drag_ghost` | 1 Point, target (tx,ty) | `[w(px-tx), w(py-ty)]` (transient, w=1000) | "soft" 2 |
