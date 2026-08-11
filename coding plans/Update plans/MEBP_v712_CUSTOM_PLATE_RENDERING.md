# MEBP v7.12 — Custom parametric plates render (and calibrate) correctly everywhere

## Objective

Operator report: *"our new way to custom build plates now breaks many of the
pages that show a plate layout. For example the most recent plate we made has 6
wells in a grid, and then 2 places where there is a ring layout of wells. In
many of the workflow pages, and even the jog page, the layout does not render
correctly."*

Make every surface that draws a well plate read the plate's real geometry, and
fix the same root cause where it breaks **motion** rather than drawing.

---

## Root cause

Every custom plate is built through `WellPlate.from_wells()`
(`SupportClasses/WellPlate.py:238`), which deliberately stores

- `well_spacing_x = well_spacing_y = 0.0`
- `well_diameter = 0.0` — "varies, see WellInfo per well"
- `rows` / `cols` = `max(row)+1` / `max(col)+1`, a **pseudo-grid**

The operator's plate `plt_7863c425dc27` ("Custom 6 insert with rosette") is a
2x3 grid of 28 mm inserts at 40 mm pitch plus two 3-well rings of 5.5 mm bores.
Compiled, that is `rows=3, cols=5` for **12 wells**, with **C1/C2/C3 all
carrying `(row=1, col=1)`** and D1/D2/D3 all carrying `(row=1, col=3)`.

The real geometry is intact on every `WellInfo` (`x`, `y`, `diameter`). Only the
plate-level aggregates are meaningless. Consumers therefore split in two, and
both camps already existed in the tree:

| Reads | Result |
|---|---|
| per-well `x` / `y` / `diameter` | correct — `WellPlateView`, `JogWorkspaceView`, plate-library thumbnails |
| `rows`/`cols` cells, `well_diameter`, `well_spacing_*` | broken — navigator, print monitor, parts of calibration |

Verified by rendering the production widgets offscreen against the operator's
actual plate: `WellPlateNavigator` drew **8 circles for 12 wells** — each 3-well
ring collapsed into one — all at a single radius, on a phantom 3x5 grid with
mislabelled A/B/C row headers.

**The same pseudo-grid broke motion.** `CalibrationPage` synthesised well names
as `f"A{cols}"` / `f"{ROW_LABELS[rows-1]}{cols}"`, asking for `"A5"` and `"C5"` —
wells that do not exist, so `get_well_position` raised `KeyError` and 3-well
auto-calibration and Go-To-Corner could not run on any custom plate at all.

### Checked and NOT a bug

The plate's geometry compiles correctly. Its wells span 108 x 68 mm and sit on
the 127.76 x 85.48 mm footprint with margins 10.44 / 9.32 / 9.56 / 7.92 mm.
An early reading that suggested otherwise had used the wrong origin: the storage
frame is A1-relative, so `PlateBoundary.extent_a1()` returns
`(-17.05, -13.67, 110.71, 71.81)`, not `(0, 0, 127.76, 85.48)`. `compile()` and
the builder agree.

---

## Design decisions (operator)

1. **Geometric layout for ALL plates** — one code path, no `is_custom` branch.
   A regular plate's real geometry *is* a regular grid, so standard plates come
   out unchanged while custom plates come out right.
2. **The footprint is read from the existing `PlateDocument.boundary`**, not
   duplicated onto `WellPlate`. *"when we defined the plate, the plate gets a
   footprint."* Two homes for one fact is what this change is removing, not
   adding.
3. **Full scope** — rendering, the calibration motion bugs, and a sweep with
   shared helpers rather than per-widget copies.

---

## Files modified

### New

| File | Purpose |
|---|---|
| `gui/widgets/plate_layout.py` | `PlateTransform` + `fit_wells` + `wells_from_plate` — ONE aspect-preserving mm→px fit for every small plate widget. Pure arithmetic, no Qt, so it is testable without a `QApplication`. |
| `tests/test_v712_custom_plate_rendering.py` | 70 tests. |

### Shared helpers

| File | Change |
|---|---|
| `SupportClasses/WellPlate.py` | NEW `well_diameter_of(name)` (per-well → plate-level → largest), `representative_well_diameter`, `nearest_neighbour_pitch_mm()` (median nearest-neighbour distance; falls back to the declared spacing on a regular plate, byte-identical there), `extreme_well(dx, dy)` (support function over real centres), `calibration_triangle()`. `_max_well_radius_mm` now delegates. |
| `SupportClasses/PlateDocumentStore.py` | NEW `plate_footprint_extent_mm(key)` + `_doc_footprint_extent_mm(doc)`. Resolves a doc id / display name / `custom:` tag / plate-type id / format int, and returns the outline **in the compiled plate's A1-well frame**. Returns None when nothing resolves so callers keep their fallback. |

### Renderers

| File | Change |
|---|---|
| `gui/widgets/jog_well_plate.py` | `WellPlateNavigator` rewritten onto the geometric layout: real `x`/`y`, per-well radius with a 4 px floor, nearest-centre hit test, headers gated on real spacing and positioned from real well centres, proximity gate sized from the measured pitch. Raster preview now uses the current well's own centre and radius. |
| `gui/pages/print_monitor.py` | `PlateOverviewWidget` sizes each well by its own diameter (was `self._wells[0]["diameter"]` for all) and fits from the full extent including radii and the minimum corner (was `max_x`/`max_y` only, so wells ran off every edge). Headers gated on real spacing. |
| `gui/widgets/well_plate_view.py` | Delegates its two inline diameter resolutions to `WellPlate`. |

### Consumers

| File | Change |
|---|---|
| `gui/pages/calibration.py` | `_get_calibration_wells` / `_pick_third_well` / `_corner_well` derived from the real layout. NEW module-level `_well_diameter_mm(plate, name)` / `_well_pitch_mm(plate)` replacing 13 zero-collapsed `well_diameter` / `well_spacing` reads. The expected-circle radius is now sized per well. |
| `gui/pages/jog_control.py` | `_update_xz_well_under_needle` resolves each well's own diameter — the plate-level 0.0 made `abs(...) <= 0` never match, so the XZ side view never showed a well. |
| `gui/widgets/standard_jog_context.py` | Hardware-Info plate line counts real wells and prints a diameter range (was `rows * cols` and `well_diameter` — "15-well · 0.00 mm Ø" for this 12-well plate). |
| `gui/app.py` | Passes `representative_well_diameter` to `print_monitor.setup_plate`, which skips the XY-detail and YZ well geometry entirely when it receives 0. |
| `gui/widgets/jog_workspace_view.py`, `gui/pages/print_builder_sketch.py` | Delegate their copies of the fallback chain. |

**Deliberately NOT folded in:** `mosaic_well_mapping_dialog._main_well_diameter_mm`
is the **median of non-sub wells**, a different rule chosen for fitting rosette
parents — unifying it onto the max-based chain would change behaviour. Its
`_grid_pitch_mm` already has its own custom-plate fallback.

---

## Implementation steps

- [x] `WellPlate` helpers (diameter, pitch, extreme well, calibration triangle)
- [x] `plate_footprint_extent_mm` in the correct A1-well frame
- [x] `gui/widgets/plate_layout.py`
- [x] `WellPlateNavigator` rewritten
- [x] `PlateOverviewWidget` fixed
- [x] `app.py` / `jog_control` / `standard_jog_context`
- [x] Calibration well-name synthesis + radii sweep
- [x] De-duplicate the diameter fallback copies
- [x] Tests + mutation verification
- [x] Regression sweep

---

## Issues & decisions

**A frame bug in the first cut of the fix.** `PlateBoundary.extent_a1()` is
measured from the document's A1 **datum**; every `WellPlate` coordinate is
measured from the A1 **well**. On a standard plate those coincide, which is why
the mistake was invisible there — but the operator placed their grid seed at
(7.388, 9.894), so handing a renderer the raw extent shifted the outline off the
wells by exactly that much. The offset is now recovered empirically (ask the
compiled plate where one evaluated well ended up) rather than by re-deriving
whatever anchor `compile()` chose, so it holds for a rosette-only document that
has no grid to name A1 from. Pinned by
`test_footprint_is_in_the_A1_WELL_frame_not_the_A1_datum_frame`, plus a
companion asserting the shift is a no-op on a standard plate.

**The calibration helpers are free functions, not methods.** They started as
`CalibrationPage` methods and broke 4 tests in
`test_v75x_plate_location_manual_click_rim`: that suite drives handlers through
`SimpleNamespace` stubs carrying only the attributes a handler touches, and a
new bound helper breaks every such call site (the hazard CLAUDE.md already
records for `_reserve_warning`). They now take the plate explicitly, which the
stubs already supply. Pinned by
`test_helpers_are_free_functions_so_stub_pages_keep_working`.

**`wells_from_plate` skips wells with no `x`/`y` rather than defaulting them.**
It feeds `paintEvent`, which must not raise; and defaulting to the origin would
stack every well on one point — the exact failure this module exists to prevent.
A plate whose wells carry no geometry returns `[]` and the caller draws nothing,
which is the honest outcome. Found by
`test_v75x_fluorescence_mosaic::test_set_raster_grid_paints`, whose stub plate
carried only `row`/`col`; that stub now carries real coordinates, because a
plate without them can no longer describe something this widget can draw.

**Two of the first-round tests were too weak and mutations caught them.** The
print-monitor test asserted on `widget._wells` (the model), which says nothing
about the radius the painter uses — rewritten to probe a **rendered pixel** that
must be background beside a 5.5 mm bore but would be inside it were the bore
drawn at the 28 mm insert's radius. The proximity-gate test could not see its
mutation because the `<= 0` fallback rescued it; it now uses a fixture where the
pitch and diameter gates genuinely differ (4 mm wells on a 9 mm pitch), with the
probe kept inside half the pitch so the neighbouring well does not become the
nearest one.

---

## Testing notes

`tests/test_v712_custom_plate_rendering.py` — **70 tests, green.** The fixture
rebuilds the operator's layout in code (`PlateDocument.new_plate` + `add_grid` +
two `add_ring`s) rather than reading `config/`, so the suite does not depend on
one machine's files. `TestTheFixtureReproducesTheDefect` guards the guard: if
the fixture ever stopped being a pseudo-grid plate with mixed diameters, every
test below it would pass vacuously.

**6 / 6 mutations confirmed CAUGHT** (each a real source edit reverted in a
`finally`):

| Mutation | Caught by |
|---|---|
| restore the rows/cols cell layout | 4 navigator tests |
| restore the uniform first-well radius | `test_a_small_well_is_PAINTED_small` |
| restore the `f"A{cols}"` name synthesis | `test_calibration_wells_exist_on_a_custom_plate` |
| drop the A1-well frame shift | `test_footprint_is_in_the_A1_WELL_frame…` |
| restore the plate-level `well_diameter` in the jog XZ view | `test_xz_well_under_needle_finds_a_custom_plate_well` |
| restore the `well_spacing` proximity gate | `test_proximity_gate_uses_the_pitch_not_the_well_size` |

**Standard plates are pinned unchanged** throughout: 24-well still teaches
`A1 / A6 / D6` with corner `D6` and third `A6`; 96-well still `A1 / A12 / H12`,
`H12`, `A12`; both keep their headers, their well counts, A1 top-left and a
single well radius.

**Regression, run per-suite (~700 tests green):** plate-doc-geometry (73),
plate-builder-UI (103), plate-identity-and-stores (42), plate-design (15),
plate-sketch-solver (6), plate-designer (16), plate-types (38),
well-type-presets (18), plate-well-detection (36), plate-centering (6),
predict-well (10), mosaic-orientation-remap (14), startup-well-map (6),
calibration-revision (20), last-known-calibration (21), click-rim (27),
workflow-toggle (21), single-well-mosaic-reregister (26), plate-z-autocal-tab
(16), z-retract (22), jog-navigation (28), fluorescence-mosaic (35),
spheroid-survey-tab (49), spheroid-page-integration (50),
quick-print-trajectory-view (13), quick-print-zones (54), context-panel (21),
responsive-context-panel (15), suite-hygiene (10), plus `test_v75x_plate_mosaic`
(110, excluding the documented `TestManualAlignPage` hang).

Plus a `gui.app` import smoke and an end-to-end render: `WellPlate.load(<doc
id>)` → 12 wells → both workflow navigators report 12 distinct centres, 2
distinct radii, headers off.

### Pre-existing failures, PROVED not from this change

| Failure | Evidence |
|---|---|
| `test_v75x_plate_mosaic::test_real_24_well_mosaic` | The legacy blob detector; `VisionDetector.py` is untouched by this diff. Documented in CLAUDE.md. |
| `test_v75x_plate_mosaic::TestMosaicSettings::test_dialog_round_trip_and_defaults` | Fails on a `full_res_scan` key: **0x in HEAD, 4x in the worktree's `mosaic_settings_dialog.py`** — a file absent from this diff. Operator's uncommitted work. |
| `test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices` | Expects "Plate Z Auto-Cal"; the operator's uncommitted v7.11 work renamed the tab "Plate Bed Level" (0x in HEAD, 1x in the worktree, at line 11488 — nowhere near this diff). Already documented in CLAUDE.md. |
| `test_v79_cell_targeting_setup_page::test_the_real_saved_profile_reproduces_its_exact_volume` | Golden-file test over the operator's own `config/workflows/cell_targeting/__last__.json`, re-saved with `push_depth` 0.1 → 0.400052 (the exact 4.0x seen: 0.003142 vs 0.012568). Already documented in CLAUDE.md. |
| `test_v75x_quick_print_workflow` (2) | `_speed_pct_spin` was retired by the v7.6 two-parameter rework; `test_enabled_when_ready` blocks on a `PrintReadiness` object evaluated before the test sets `_plate`/`_selected_well`. Both live in `quick_print_workflow.py` / `PrintReadiness.py`, neither in this diff. |

---

## Needs GUI verification on ME3B V1

In order:

1. Open the custom plate in **Quick Print** and **Fluorescence Mosaic** — each
   ring must show **three separately clickable** small wells, visibly smaller
   than the six inserts, with no row/column headers.
2. Click one ring well and confirm the stage travels to **that** well and not a
   neighbour (the hit test now picks the nearest centre, which is what makes a
   small well inside a large neighbour's circle reachable).
3. Jog across the plate and confirm the **XZ side view** shows a well under the
   needle — over an insert and over a bore, at visibly different widths.
4. Check the Jog page's Hardware-Info line reads **12-well · 5.50–28.00 mm Ø**,
   not "15-well · 0.00 mm".
5. On the Calibration page run **Go To Corner** and **3-well auto-calibration** —
   both currently raise `KeyError` on this plate.
6. Run a print and confirm the **monitor's plate overview** shows all 12 wells
   inside the widget with the rings drawn small.
7. Confirm a **standard 24-well plate** looks exactly as it did before.

## Note

The same pseudo-grid collapse affects the v1 rosette plates already on disk
(`plate-24_Rosette in A2&A3.json` — sub-wells `A2.a/.b/.c` share their parent's
row/col), so this repairs those too. No data migration: every plate file is
untouched, only the renderers changed.
