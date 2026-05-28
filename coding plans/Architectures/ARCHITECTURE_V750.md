# MEBP v7.5.0 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.5.0 | May 2026**

> **Delta against [`ARCHITECTURE_V748.md`](ARCHITECTURE_V748.md).**
> Sections not mentioned here are unchanged.

---

## 1. What's New in v7.5.0

A new top-level **Print Builder** mode page makes authoring print trajectories
explicit and easy, and replaces the side-preview model with a **raster-as-main-view**:

1. **Print Builder main page** (sidebar **index 3**, inserted *before* Printing —
   the natural build → print flow). It relocates the three build-time sub-pages
   out of Printing mode (which is now run-focused: Setup / Monitor / Results):
   - **Sketch** *(new)* — draw-to-print canvas.
   - **Image Import** — the former *Helper Functions* image-stack → raster.
   - **Hardware** — read-only HW summary.
   - **Print Settings** — display toggles.
2. **Sketch tool.** Vector primitives (line / rect / circle / ellipse / polygon) +
   paint-bucket fill of enclosed regions, multi-layer Z-stack, object/border
   snapping, per-shape **bead width** (multi-pass outlines), driven by the
   active **needle diameter** (raster step + default bead). A dashed **standard
   well** outline is drawn at the origin for scale.
3. **Raster-as-main-view.** The compiled toolpath renders directly on the canvas
   (colored by pump, dashed grey travel); the input shapes are a slim
   **editable overlay** on top. The old side preview is gone.
4. **Csv-import contract preserved.** Both Sketch and Image Import bake their
   output through a shared `save_trajectory_as_print_object()` helper into
   `config/prints/` as a `csv_import` print object, which Print Setup's custom-
   prints area picks up via `print_file_created` (a latent
   `_load_file_by_name` → `_load_print_file` routing bug is fixed in `app.py`).

---

## 2. Backend Changes

### `SupportClasses/SketchTrajectory.py` *(new)*

```python
@dataclass
class SketchShape:
    kind: str                       # "line"|"rect"|"circle"|"ellipse"|"polygon"|"region"
    cx, cy, radius, rx, ry, width, height: float
    points: list[tuple[float, float]]  # line/polygon vertices; region holds the
                                        # baked meander toolpath
    filled: bool = False
    line_width_mm: float = 0.4      # printed bead width (multi-pass + render)
    pump_index: int = 0
    color: str

@dataclass
class Sketch:
    shapes: list[SketchShape]
    z_start_mm, layer_height_mm, num_layers: ...
    print_speed_mm_s, travel_speed_mm_s, travel_clearance_mm: ...
    line_spacing_mm: float           # the raster step / per-pass bead
    outline_points_per_mm: float
    flow_factor: float

def compile_to_trajectory(sketch, needle=None, syringe=None) -> CompiledSketch:
    """Per layer × per shape: _shape_paths() emits one path per side-by-side
    pass for thick outlines (`_pass_offsets`); fills/regions emit a single
    meander. Stitched with lift→move→lower travel between paths, time-
    parameterized at print/travel speed. Pump column = cumulative plunger
    displacement; per-pass bead = sketch.line_spacing_mm so volume stays
    coherent across multi-pass outlines and fills.
    """
```

`compute_fill_region(shapes, click_xy, spacing_mm)` — paint-bucket:
rasterizes outlines (Bresenham + `ndimage.binary_dilation` to seal),
`ndimage.label` finds the free-space component at the click; rejects
edge-touching (open) regions; returns a serpentine meander as the
`region` shape's `points`. Requires scipy.

### `SupportClasses/PrintFileManager.py`

```python
def save_trajectory_as_print_object(
    trajectory, base_name="PrintBuilder", description="", color="#89b4fa",
    author="Print Builder", source="PrintBuilder", object_name="Sketch_1",
    prints_dir=DEFAULT_PRINTS_DIR, extra_params=None,
) -> str:
    """Bake an Nx7 trajectory into config/prints/{name}.csv + .json with
    object_type='csv_import' (auto-incremented name). Shared by both the
    Sketch page and the Image Import page so the resulting object shows up
    in Print Setup's custom-prints area unchanged."""
```

---

## 3. GUI Changes

### `gui/pages/print_builder.py` *(new)* — `PrintBuilderPage(ModePage)`

Vertical icon strip with four sub-pages. The relocated `PrintingHardwarePage`
(wraps `HardwareSummaryWidget`) and `PrintingSettingsPage` (display toggles)
classes were moved here from `printing_mode.py`. `get_sub_page_title()` returns
the registered tab label (so "Image Import" reads correctly in the top bar
even though `HelperFunctionsPage.get_page_title()` returns "Helper Functions").

### `gui/widgets/sketch_canvas.py` *(new)* — `SketchCanvas(QWidget)`

Lightweight custom-painted canvas (deliberately *not* the constraint-solving
`PlateDesignerCanvas`). Direct-manipulation: drag to create, drag to move /
resize handles, snap (grid + object/border), pan/zoom, undo/redo.

Paint order: grid → axes → reference-well outline → **toolpath** (`_tp_runs`,
colored print runs + dashed grey travel) → shape **overlay** (slim dashed
boundary + handles when selected) → in-progress preview → snap marker.

Key API (mostly setters from the page):

```python
set_sketch(sketch)
set_tool(Tool)                       # SELECT, LINE, RECT, CIRCLE, ELLIPSE, POLYGON, FILL
set_active_pump(int)
set_default_line_width(mm)           # new shapes inherit (needle Ø)
set_reference_well(diameter_mm)      # dashed mauve well at origin
set_object_snap(bool)
set_toolpath(trajectory, pump_states)  # the raster shown in main view
```

`Signals: sketch_changed, selection_changed(int), tool_changed, fill_result(bool)`.

### `gui/pages/print_builder_sketch.py` *(new)* — `SketchPage(QWidget)`

Tool palette | canvas | properties panel (frosted "lighter box" cards via
`build_section_title_style` — Hardware-Setup look). The right panel hosts the
per-shape and print-parameters cards and the **Send to Print Setup** button.
There is **no side preview** — the canvas itself shows the raster.

`set_hardware_config(config)` pulls `needle.od_mm` and sets
`Sketch.line_spacing_mm` (raster step) + canvas default line width to it; loads
the active plate via `WellPlate.load(active_plate_key)` and passes its
`well_diameter` to `set_reference_well`. The Print-parameters card surfaces
"Needle Ø X.XX mm".

### `gui/pages/printing_mode.py` *(trimmed)*

`PrintingModePage` is now 3 sub-pages (Setup / Monitor / Results). The
`PrintingHardwarePage` and `PrintingSettingsPage` classes were moved to
`gui/pages/print_builder.py`. Helper Functions is no longer registered here.

### `gui/app.py` — navigation

```
0 Hardware Setup · 1 Calibration · 2 Jog · 3 Print Builder · 4 Printing
5 Workflows · 6 Settings
```

Updated `menu_items`, `pages`, `btn_map`, `titles` / `context_titles` in
lock-step. `_on_helper_print_created` → `_on_print_created`: routes
`print_file_created` from both Print Builder authoring pages (Sketch + Image
Import) into Print Setup; resolves `tab_objects` via either the wizard's
`_legacy` page or directly; calls `_load_print_file` (fixed); navigates to the
new Printing index (4).

---

## 4. Folder Structure (delta)

```
SupportClasses/
  SketchTrajectory.py           # NEW: Sketch / SketchShape + compile_to_trajectory + compute_fill_region

gui/
  pages/
    print_builder.py            # NEW: PrintBuilderPage (ModePage)
    print_builder_sketch.py     # NEW: SketchPage (palette + canvas + props + send)
    printing_mode.py            # trimmed to 3 sub-pages
  widgets/
    sketch_canvas.py            # NEW: SketchCanvas (raster main view + editable overlay)

tests/
  test_v75x_sketch_trajectory.py  # NEW: 12 tests (compile, fill region, multi-pass, region kind, layers, ...)
```

---

## 5. Key Algorithms

**Bead-coherent multi-pass.** A thick outline of width `line_width_mm` is laid
as `n = round(line_width / bead)` side-by-side passes, each a bead-wide deposit
(`_pass_offsets` returns centred perpendicular offsets). Each pass's deposited
volume = `bead × layer_height` per mm, so total volume = total path length ×
that — coherent whether the user thickens an outline, fills a shape, or fills
a region.

**Paint-bucket fill.** Rasterize all shape outlines into a binary grid with
Bresenham line marking + `ndimage.binary_dilation(1)` to seal seams.
`ndimage.label(~border)` labels the free-space components; the click's
component is rejected if it touches the grid edge (open region / outside),
otherwise serpentine-scanlined at `line_spacing_mm` into the region's
toolpath. Stored as a baked `region` `SketchShape`.

**Object snap.** `SketchCanvas._snap()` snaps to other shapes' exact vertices /
centers first (line endpoints, polygon vertices, rect corners, circle/ellipse
centers, rect center), then to the nearest point on each shape's boundary
within a screen-pixel tolerance. Grid snap remains as a fallback. The shape
being edited is excluded.

---

## 6. Design Principles (delta)

- **Authoring vs running** — Print Builder is the *authoring* home (Sketch +
  Image Import + Hardware ref + display toggles); Printing mode is *running*
  (Setup / Monitor / Results). Builder output flows into Setup as a baked
  `csv_import` object.
- **Raster as the source of truth on the canvas** — the user sees exactly
  what will print. Shapes are an editable boundary overlay, never the rendered
  bead.
- **Needle-diameter basis** — fill / outline width / raster step default from
  the active needle's OD; users can still tune.

---

## 7. Catch-Up Bundle

This release also bundles the staged v7.4.3 – v7.4.8 increments that
accumulated on the Version-7.4.2 branch (per CLAUDE.md). See their update plans
(`coding plans/Update plans/MEBP_v744_to_v745`…`v747_to_v748_UPDATE.md`) and
architecture docs (`ARCHITECTURE_V745`…`V748.md`) for the per-version detail.
