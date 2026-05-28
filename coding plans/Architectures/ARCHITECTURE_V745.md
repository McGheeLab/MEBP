# MEBP v7.4.5 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.5 | May 2026**

> **Delta against [`ARCHITECTURE_V742.md`](ARCHITECTURE_V742.md).** Sections
> not mentioned here are unchanged.

---

## 1. What's New in v7.4.5

The Hardware Setup → Plate sub-page gains a full **SolidWorks-style
parametric well-plate designer**, replacing the previous one-line
format combo. Users can now author arbitrary plate geometries:

- Place wells of any size, anywhere — Single Well or Grid Pattern tools
- Name each well freely (A1, B5, or arbitrary strings)
- Drag wells with the mouse; the solver re-arranges the rest of the
  sketch to honor active constraints
- Add geometric constraints (Lock, Distance, Horizontal/Vertical,
  Concentric, Equal-Ø, Coincident)
- Save custom plates by name; coexist with the 6 standard formats as
  bundled starting templates
- Live DOF status (under-/well-/over-constrained) in a bottom status bar

Downstream consumers (Calibration, Jog, Print Setup, Print execution)
treat custom plates identically to standards via the new polymorphic
`WellPlate.load` factory.

---

## 2. Backend Additions

### `SupportClasses/PlateDesign.py` (new)

The editable sketch document. Separate from the runtime `WellPlate`
artifact every other page consumes — the designer compiles a
`PlateDesign` into a `WellPlate` via `compile()`.

```python
@dataclass
class PlateDesign:
    schema_version: str = "1.0"
    name: str
    outline: PlateOutline
    entities: dict[EntityId, Entity]
    constraints: list[Constraint]
    a1_offset_x: float
    a1_offset_y: float
    well_depth_default_mm: float
    def compile(self) -> WellPlate: ...
    def add_well(...) -> Well: ...
    def add_grid(...) -> Group: ...
    def add_circle_pattern(...) -> Group: ...
    @classmethod
    def from_standard_format(cls, well_count: int) -> "PlateDesign": ...
```

Entities are `Point`, `Line`, `Circle`, `Well`, `Group`. **Wells own a
center `Point` rather than embedding x/y directly** — every geometric
constraint operates on points/lines/circles, and `Well` is a labeled
wrapper around a center + diameter.

`Constraint` is a tagged dataclass:

```python
@dataclass
class Constraint:
    id: int
    kind: str              # see CONSTRAINT_KINDS
    refs: list[EntityId]
    value: Optional[float] = None
    weight: float = 1.0
    snapshot: Optional[tuple[float, float]] = None
```

### `SupportClasses/PlateSketchSolver.py` (new)

`scipy.optimize.least_squares` with Levenberg-Marquardt for fully- or
over-determined systems; falls back to trust-region-reflective (`trf`)
for under-determined sketches.

```python
class PlateSketchSolver:
    def solve(self) -> SolveReport: ...
    def begin_drag(self, point_id, target_xy) -> SolveReport: ...
    def update_drag(self, target_xy) -> SolveReport: ...
    def end_drag(self) -> SolveReport: ...
```

**DOF detection** computes `rank(J)` from the optimum Jacobian:

- `cost ≈ 0 ∧ rank == n_free` → `WELL_DETERMINED`
- `cost ≈ 0 ∧ rank < n_free` → `UNDER_DETERMINED`
- `cost > 0`                  → `INCONSISTENT` (top-3 conflicting refs reported)

**Drag** is implemented as a transient `drag_ghost` constraint pushed
onto the design's constraint list with `weight=1000` (large enough to
dominate other constraints during drag). On `mouseUp` the ghost is
popped.

Constraint kinds shipped in v7.4.5: `ground`, `fix`, `coincident_pp`,
`distance_pp`, `horizontal`, `vertical`, `concentric`, `equal_radius`,
`drag_ghost`. Pre-solve passes handle `equal_radius` (write-through)
and treat `concentric` between Wells identically to `coincident_pp`
on their centers.

### `SupportClasses/WellPlate.py` (generalized)

```python
@dataclass
class WellPlate:
    format: int | str         # "custom:<name>" for designed plates
    # ... rest unchanged
    @classmethod
    def from_wells(cls, name, wells, **meta) -> "WellPlate": ...
    @classmethod
    def load(cls, name_or_format: int | str) -> "WellPlate": ...
```

- `from_format(int)` preserved unchanged. Existing callers (`WellSetup`,
  `calibration.py`, `jog_control.py`, etc.) keep working.
- `__post_init__` skips grid auto-fill when `_wells` is already supplied.
- New `WellPlate.load(name_or_format)` accepts standard ints, stringified
  ints, raw custom names, or `"custom:<name>"`-prefixed encodings.
- `plate_width`/`plate_height`/`get_bounding_box`/`get_well_area_bounds_*`
  fall back to per-well extents (and `_max_well_radius_mm()`) when the
  plate is custom.

### `SupportClasses/HardwareConfig.py`

```python
@dataclass
class HardwareConfig:
    plate_format: int = 24        # legacy field, still honored
    plate_name: str = ""          # v7.4.5; takes precedence when non-empty

    @property
    def active_plate_key(self) -> int | str:
        return self.plate_name if self.plate_name else self.plate_format
```

`to_dict()` / `from_dict()` write/read `plate_name` alongside the
legacy `plate_format` for one release cycle. `validate()` flags missing
custom-plate files explicitly.

---

## 3. UI Additions

### `gui/pages/hardware/plate_designer.py` (new)

`PlateDesignerWidget(QWidget)` — top-level composite that owns the
`PlateDesign` document and routes events. Layout:

```
PlateDesignerWidget
├── header — picker combo + Save / Save As / Delete + Fit / Snap toggle
├── splitter (toolbar | canvas | properties panel)
└── DOF status bar
```

### `gui/widgets/plate_designer_canvas.py` (new)

`PlateDesignerCanvas(QGraphicsView)` with:

- **Scene items**: `PlateOutlineItem` (offset so A1 sits at design (0,0)
  inset by `a1_offset_x/y` within the outline), `WellItem`,
  `WellLabelItem` (per-well name labels), background grid via
  `drawBackground` (1 mm minor / 10 mm major).
- **Tool state machine**: `Tool.SELECT` / `DRAW_SINGLE_WELL` / `DRAW_GRID`
  / `ADD_CONSTRAINT`. Cursor changes per tool.
- **Drag**: solver-pin with 33 ms debounce (`QTimer.singleShot`).
- **Zoom/pan**: scroll-wheel zoom centered on cursor; middle-mouse pan.
- **Selection**: purple-border (Catppuccin mauve); click / Ctrl-click /
  empty-click-clears.
- **Keyboard shortcuts**: `S` Select, `W` Single Well, `G` Grid,
  `Esc` cancel, `F` fit view, `Del` delete selection.

### `gui/pages/hardware_setup.py` integration

- Lines 676–698 (`_sub_layouts["plate"]`): combo replaced with a
  `PlateDesignerWidget` mount. The original `QComboBox` is kept hidden
  as a backward-compat shim for legacy `currentData()` readers.
- `_rebuild_config()`: writes both `plate_format` and `plate_name` from
  the designer, keeping the hidden shim combo synced.
- `_apply_config_to_ui()`: calls `self._plate_designer.load_plate(
  self._config.active_plate_key)`.

---

## 4. Storage Layout

```
config/hardware/plates/
  user/                    # user-saved designs (writable)
    my-coverslip-array.json
```

Standard formats remain in-code via `PLATE_DEFINITIONS`; cloning into a
designable plate happens through `PlateDesign.from_standard_format(int)`
(no JSON files for standards).

Custom plate JSON schema:

```jsonc
{
  "schema_version": "1.0",
  "name": "my-coverslip-array",
  "outline": { "kind": "rect", "width": 127.76, "height": 85.48, ... },
  "entities": {
    "1": {"type": "Point", "id": 1, "x": 0, "y": 0, "fixed": true},
    "2": {"type": "Well", "id": 2, "name": "A1", "center": 1, ... }
  },
  "constraints": [ {"id": 1, "kind": "ground", "refs": [1]}, ... ],
  "a1_offset_x": 14.38, "a1_offset_y": 11.24
}
```

---

## 5. Updated Core Design Principles

(Additions to the v7.4.2 list.)

22. **Custom plates are first-class.** Every consumer that needs plate
    geometry routes through `WellPlate.load(cfg.active_plate_key)` —
    standards and custom designs use the same code path. Legacy
    `from_format(int)` is kept as a thin alias.

23. **Sketcher document vs. runtime artifact.** `PlateDesign` is the
    editable document the user manipulates; `WellPlate` is the
    immutable runtime artifact every downstream page consumes.
    `PlateDesign.compile()` is the one-way bridge.

24. **Wells own a center Point.** Every constraint operates on points,
    lines, or circles. `Well` is a labeled wrapper around a center
    `Point` + diameter; this makes "distance between two wells"
    expressible as `distance_pp(w1.center, w2.center)` with no
    special-cased well-to-well constraint kinds.

---

## 6. Sections Unchanged

All earlier architecture sections not mentioned above apply verbatim.
