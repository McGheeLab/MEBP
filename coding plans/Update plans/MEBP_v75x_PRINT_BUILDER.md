# MEBP v7.5.x — Print Builder mode + draw-to-print Sketch tool

## Objective

Make building print trajectories easier and give it a dedicated home.

1. **New top-level "Print Builder" mode page**, inserted in the sidebar
   *before* Printing (the natural build → print flow).
2. **Relocate** the three build-time sub-pages out of Printing mode into Print
   Builder: **Image Import** (the former *Helper Functions* image-stack →
   raster toolpath page), **Hardware** (read-only HW summary), **Print
   Settings** (display toggles). Printing mode is now run-focused (Setup /
   Monitor / Results).
3. **New flagship "Sketch" sub-page** — draw a print directly from vector
   primitives (line / rect / circle / ellipse / polygon), optionally filled,
   stacked across N Z-layers, with a live toolpath preview. No image prep.
4. The Sketch (and Image Import) output is **baked into a `csv_import` print
   object** and handed to **Print Setup's custom-prints area** via the existing
   `print_file_created` contract (Print Setup stays in Printing mode).

## Files Modified / Added

| File | Change |
|------|--------|
| `SupportClasses/SketchTrajectory.py` | **NEW** — `Sketch` / `SketchShape` model + `compile_to_trajectory()` → Nx7. Reuses `GeometryEngine` generators (outline + meander-fill) + a scanline polygon fill. Returns `CompiledSketch` (traj + per-waypoint pump_states for preview + stats). |
| `SupportClasses/PrintFileManager.py` | **NEW fn** `save_trajectory_as_print_object()` — writes `config/prints/{name}.csv` + `.json` (`object_type="csv_import"`), auto-incrementing the name; shared by Sketch + Image Import. |
| `gui/widgets/sketch_canvas.py` | **NEW** — `SketchCanvas` (custom-painted `QWidget`): direct draw/move/resize, snap, pan/zoom, undo/redo. Bound to the shared `Sketch` model. Not the constraint-solver canvas. |
| `gui/pages/print_builder_sketch.py` | **NEW** — `SketchPage`: tool palette + canvas + properties (exact sizes, fill, pump, Z-stack) + live `_PathPreviewWidget` + **Send to Print Setup**. Emits `print_file_created`. |
| `gui/pages/print_builder.py` | **NEW** — `PrintBuilderPage(ModePage)` (vertical strip): Sketch / Image Import / Hardware / Print Settings. Houses the relocated `PrintingHardwarePage` + `PrintingSettingsPage` classes. |
| `gui/pages/printing_mode.py` | Dropped Helper/Hardware/Settings sub-pages + their classes/accessors; now 3 sub-pages (Setup/Monitor/Results). |
| `gui/app.py` | Registered Print Builder as main page **index 3** (Printing→4, Workflows→5, Settings→6): `menu_items`, `pages`, `btn_map`, `titles`/`context_titles`. Rewired `print_file_created` from the Print Builder authoring pages to `_on_print_created` (renamed from `_on_helper_print_created`); **fixed** the latent `_load_file_by_name` bug → `_load_print_file` (resolved via `setup_page._legacy.tab_objects`); bumped its post-create nav to index 4. |
| `tests/test_v75x_sketch_trajectory.py` | **NEW** — 8 unit tests for the compiler. |

## Implementation Steps

- [x] Add `save_trajectory_as_print_object()` shared helper.
- [x] Create `SketchTrajectory.py` (model + compiler, GeometryEngine reuse, scanline polygon fill, serpentine Z-stack, lift/move/lower travel).
- [x] Unit tests (8) — all passing.
- [x] `SketchCanvas` widget (tools, draw/move/resize handles, snap, pan/zoom, undo).
- [x] `SketchPage` sub-page (toolbar + canvas + props + live preview + send).
- [x] `PrintBuilderPage` container; relocate `PrintingHardwarePage` + `PrintingSettingsPage`.
- [x] Trim `printing_mode.py` to Setup/Monitor/Results.
- [x] Wire Print Builder into `app.py` nav (index 3) + rewire signals + fix `_load_print_file` bug + nav-index bump.
- [x] Keep top-bar title consistent with the "Image Import" tab (`PrintBuilderPage.get_sub_page_title`).
- [x] DRY: `helper_functions._load_as_print_object` now calls the shared save helper.
- [x] Data-path verification: compile → `save_trajectory_as_print_object` → `import_csv_trajectory` round-trips (Nx7 preserved); offscreen full `MainWindow` builds with 7 pages, Print Builder at index 3 / Printing at 4, all titles correct.
- [x] Live GUI draw → send interaction (user-verified across multiple sessions).
- [x] Architecture doc + README + CLAUDE.md version bump → V7.5.0 / Version-7.5.0 branch.

## Testing Notes

- `python3 -m unittest tests.test_v75x_sketch_trajectory -v` → 8/8 OK (Nx7 shape,
  monotonic `t`, per-layer Z, pump-column routing P1/P2/P3, travel segments have
  zero flow, monotonic plunger displacement, empty-sketch guard, no-hardware
  fallback).
- Offscreen smoke: `PrintBuilderPage()` builds 4 sub-pages; a filled circle
  compiles to a ~481-waypoint toolpath and the preview stats render.
- `import gui.app` OK (nav rewiring imports cleanly).
- Full suite: `QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests`.
- Manual: Print Builder → Sketch → draw circle, toggle Fill, set 3 layers →
  **Send to Print Setup** → object appears as a `csv_import` custom object in
  Print Setup and previews; Image Import still works; all main nav buttons + top
  titles correct.

## Sketch enhancements (iteration 2 — user feedback)

After the first live test the user requested four additions, all implemented:

1. **Paint-bucket fill of enclosed regions** — new `Tool.FILL`. Click inside an
   area enclosed by shapes/lines → `SketchTrajectory.compute_fill_region()`
   rasterizes all outlines (scipy `ndimage`, watertight via Bresenham +
   dilation), flood-labels the free space, rejects open/edge-touching regions,
   and returns a serpentine meander. Stored as a baked `kind="region"`
   `SketchShape` (its toolpath in `points`); the compiler emits it directly.
   `SketchCanvas.fill_result` signal drives a status hint when not enclosed.
2. **Snap to existing object borders** — object snap (default ON, toggle in the
   palette): `SketchCanvas._snap()` snaps to other shapes' vertices/centers
   (exact) and nearest boundary points (edges) within tolerance, with a yellow
   cross-hair marker; grid snap remains as a fallback. Excludes the shape being
   moved/resized.
3. **Per-object thickness (printed bead width)** — `SketchShape.line_width_mm`.
   Renders the stroke proportionally thick; the compiler lays multiple
   side-by-side passes (`_pass_offsets`) for thick outlines (concentric circles
   / inset-outset rects / scaled ellipses / parallel lines) and a bead-based
   deposition model (`vol_per_mm = line_spacing × layer_height` per pass) so
   volume stays coherent. Editable per shape in the properties panel.
4. **Full dark theme** — scoped QSS on the Sketch page styles all `QToolButton`s
   (normal/hover/checked), the toolbar background, and `primaryBtn`/`dangerBtn`.

Tests: extended `test_v75x_sketch_trajectory.py` to **12** (fill-region inside /
open / region-compiles / thick-multipass) — all green. Offscreen smoke confirms
fill in/out, edge snap, and multi-pass waypoint growth.

Known limitation: paint-bucket fill assumes simple (convex-ish) sections — a
region with interior holes is filled as one pass set (no per-span travel yet);
arbitrary-polygon *outline* thickening is single-pass.

## Sketch enhancements (iteration 3 — UI polish + needle basis)

1. **Right-panel polish** — the Sketch properties/preview column now matches the
   Hardware Setup look: dark `base` panel, frosted "lighter box" sections with
   blue-on-dark title pills via the shared `gui/styles.build_section_title_style`;
   scoped QSS keeps the scroll area transparent so the cards sit on the dark
   background. Column widened (~360 px) for the padded cards.
2. **Needle-diameter basis** — `SketchPage.set_hardware_config` reads the active
   needle's `od_mm` and uses it as the raster step (`Sketch.line_spacing_mm`) and
   the default bead/outline width for new shapes (`SketchCanvas.set_default_line_width`).
   The Print-parameters card shows "Needle Ø X.XX mm"; "Fill gap" relabeled
   "Raster step". Falls back to the prior 0.4 mm default until a needle is set.
3. **Standard-well reference** — `SketchCanvas.set_reference_well()` draws a dashed
   mauve well outline (Ø from `WellPlate.load(active_plate_key).well_diameter`) at
   the origin, labeled, and framed by `fit_view`, so users size drawings against a
   real well. Shows from the plate format even without a needle.

## Sketch enhancements (iteration 4 — raster as the main view)

The compiled **toolpath raster is now the main canvas view**; the input shapes
became a thin **editable overlay** on top:

- `SketchCanvas.set_toolpath(trajectory, pump_states)` groups segments into
  colored print runs / dashed-grey travel runs (computed once) and draws them in
  world space under the shapes. The page pushes the compiled result here in
  `_recompute_preview` instead of to a side widget.
- `_draw_shape` is now overlay-only: a slim dashed boundary (solid + handles when
  selected) — no bead-width fill (the raster shows real thickness/fill).
  `region` shapes render as a dashed bounding outline so they stay selectable.
- The side `_PathPreviewWidget` was removed from the right panel (import dropped);
  stats remain in the bottom status row. Raster updates on a 120 ms debounce, so
  the overlay moves live during drags and the raster catches up on release.

## Issues & Decisions

- **Engine choice:** built a lightweight custom-painted canvas instead of reusing
  `PlateDesignerCanvas`/`PlateSketchSolver` — the user wants "direct & freeform",
  and the constraint solver is overkill for authoring a toolpath. Geometry still
  reuses `GeometryEngine` generators, so shapes match the rest of the pipeline.
- **Output contract:** Sketch *bakes* a trajectory (csv_import) rather than
  emitting editable parametric objects, matching the user's request that builder
  output land in Print Setup's custom-prints area.
- **Latent bug fixed:** the old helper-functions handler called a nonexistent
  `_load_file_by_name`; corrected to `_load_print_file` and made it resolve the
  objects tab through the wizard's `_legacy` page.
- **Polygon fill** uses a simple even-odd scanline (good for simple polygons);
  outline polygons always work.
- **Pump volume:** the compiler is volume-accurate when a needle + syringe are
  available (pulled from `HardwareConfig`), else falls back to a monotonic
  `flow_factor` so the pump column and preview stay valid.
- **Version label:** code carries `v7.5.x` comments while CLAUDE.md still reads
  V7.4.8 — confirm the exact version/branch with the user before finalizing the
  architecture/README docs.
