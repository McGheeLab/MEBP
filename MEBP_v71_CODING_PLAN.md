# MEBP v7.1 — Print System Overhaul: Detailed Coding Plan

## Table of Contents
1. [Executive Summary](#1-executive-summary)
2. [New File Structure](#2-new-file-structure)
3. [Phase 1 — Physical Models & Data Layer](#3-phase-1--physical-models--data-layer)
4. [Phase 2 — Parametric Geometry Engine](#4-phase-2--parametric-geometry-engine)
5. [Phase 3 — Trajectory Planner & Motion Control](#5-phase-3--trajectory-planner--motion-control)
6. [Phase 4 — Flow Physics & Safety](#6-phase-4--flow-physics--safety)
7. [Phase 5 — Print Setup GUI (3 Tabs)](#7-phase-5--print-setup-gui-3-tabs)
8. [Phase 6 — Print Monitor Page](#8-phase-6--print-monitor-page)
9. [Phase 7 — Print Recording & Replay](#9-phase-7--print-recording--replay)
10. [Phase 8 — Integration & Migration](#10-phase-8--integration--migration)
11. [Dependency Summary](#11-dependency-summary)
12. [Implementation Order & Session Breakdown](#12-implementation-order--session-breakdown)

---

## 1. Executive Summary

This plan replaces the current `PrintSetupPage` (single-page, pattern-based) with a
multi-tab workspace-oriented print system.  The overhaul touches **backend data models**,
**geometry generation**, **motion control algorithms**, **flow physics**, **three new GUI
tabs**, a **new Print Monitor page**, and an **automatic recording system**.

### Current State (v7.0)
- `print_setup.py` (1060 lines): File/WellPlate/Pattern source tabs + 2D canvas + context panel
- `PrintManager.py` (1469 lines): PrintJob/PrintCommand + sequential executor + queue
- `WellPlate.py` (315 lines): ANSI/SLAS plate geometry + 4 path generators
- Execution model: discrete command list, no time parameterization, no trajectory tracking

### Target State (v7.1)
- Workspace-oriented setup with syringe/needle/ink physical models
- Parametric print objects in (x,y,z,p1,p2,p3,t) with needle-aware geometry
- CSV trajectory import with time interpolation
- Multi-projection visualization (XY large, ZY right, XZ below) at both plate and well scale
- Real-time needle tracking with Kalman-filtered motion control
- Well bottom plane detection via 3-point teach
- Print recording to file with replay capability

---

## 2. New File Structure

```
SupportClasses/
│── PhysicalModels.py        ★ NEW — Syringe, Needle, Ink, FluidColumn, RosetteInsert
│── GeometryEngine.py        ★ NEW — Parametric print objects + needle-aware paths
│── TrajectoryPlanner.py     ★ NEW — Time-parameterized paths, interpolation, CSV import
│── MotionController.py      ★ NEW — Kalman filter, PID, feedforward tracking
│── FlowPhysics.py           ★ NEW — Pressure/flow calculations, safety thresholds
│── PrintRecorder.py         ★ NEW — Auto-record prints, replay data
│── WellSetup.py             ★ NEW — Well assignments, rosettes, plane fitting, roles
│── PrintManager.py          ✏ MODIFIED — New command types, trajectory execution mode
│── WellPlate.py             ✏ MODIFIED — Add well depth, bottom offset per well
│── StageController.py       ✏ MODIFIED — Add trajectory execution API, rate testing
│── XYStage.py               ✏ MODIFIED — JSON-based controller protocol mapping
│── SafetyLimits.py          ✏ MODIFIED — Add flow-rate limits per needle
│── Settings.py              ✏ MODIFIED — New settings sections

config/
│── controllers/
│   │── proscan_ii.json      ★ NEW — ProScan II command map
│   │── proscan_iii.json     ★ NEW — ProScan III command map
│── hardware/
│   │── needles.json         ★ NEW — Needle gauge catalog (16G–32G+)
│   │── syringes.json        ★ NEW — Hamilton syringe catalog

gui/pages/
│── print_workspace.py       ★ NEW — Tab 1: Setup Workspace
│── print_objects.py         ★ NEW — Tab 2: Setup Print Files / Objects
│── print_well_setup.py      ★ NEW — Tab 3: Well Setup & Assignment
│── print_monitor.py         ★ NEW — Print Monitor page (separate from setup)
│── print_setup.py           ✏ MODIFIED — Becomes thin wrapper hosting 3 tabs

gui/widgets/
│── projection_canvas.py     ★ NEW — Multi-view canvas (XY large, ZY right, XZ below)
│── well_plate_view.py       ★ NEW — Interactive top-down plate view with color coding
│── syringe_display.py       ★ NEW — Visual syringe fill-level / position indicator
│── trajectory_view.py       ★ NEW — Real-time needle path + future waypoints
```

Total: **16 new files** (12 Python + 4 JSON), **7 modified files**

---

## 3. Phase 1 — Physical Models & Data Layer

### File: `SupportClasses/PhysicalModels.py`

Purpose: Dataclasses representing the physical hardware configuration, fluid management,
and well insert geometry.

### 3a. Core Hardware Models

Needle and syringe specifications are stored as **user-editable JSON files**
so users can add new gauges or syringe sizes without modifying Python code.

**File: `config/hardware/needles.json`**
```json
{
    "description": "Needle gauge specifications (ASTM standard values)",
    "needles": {
        "16": {"od_um": 1651, "id_um": 1194, "wall_um": 229},
        "17": {"od_um": 1473, "id_um": 1067, "wall_um": 203},
        "18": {"od_um": 1270, "id_um":  838, "wall_um": 216},
        "19": {"od_um": 1067, "id_um":  686, "wall_um": 191},
        "20": {"od_um":  908, "id_um":  603, "wall_um": 152},
        "21": {"od_um":  819, "id_um":  514, "wall_um": 152},
        "22": {"od_um":  718, "id_um":  413, "wall_um": 152},
        "23": {"od_um":  641, "id_um":  337, "wall_um": 152},
        "25": {"od_um":  515, "id_um":  260, "wall_um": 127},
        "26": {"od_um":  464, "id_um":  260, "wall_um": 102},
        "27": {"od_um":  413, "id_um":  210, "wall_um": 102},
        "28": {"od_um":  362, "id_um":  184, "wall_um":  89},
        "30": {"od_um":  312, "id_um":  159, "wall_um":  76},
        "32": {"od_um":  235, "id_um":  108, "wall_um":  64}
    }
}
```

**File: `config/hardware/syringes.json`**
```json
{
    "description": "Hamilton 1700-series Half-Height Gastight syringes",
    "manufacturer": "Hamilton",
    "series": "1700",
    "plunger_type": "UHMWPE X-style BFP",
    "stroke_length_mm": 30.0,
    "notes": "Compatible with Hamilton PSD/4 Syringe Pumps",
    "syringes": {
        "25":   {"part_number": "1702", "barrel_id_mm": 1.030},
        "50":   {"part_number": "1705", "barrel_id_mm": 1.457},
        "100":  {"part_number": "1710", "barrel_id_mm": 2.060},
        "250":  {"part_number": "1725", "barrel_id_mm": 3.256},
        "500":  {"part_number": "1750", "barrel_id_mm": 4.606},
        "1000": {"part_number": "1001", "barrel_id_mm": 6.513}
    }
}
```

**Python loaders:**
```python
def load_needle_catalog(path: str = "config/hardware/needles.json") -> dict:
    """Load needle gauge specs from JSON. Returns {gauge_int: NeedleSpec}."""
    with open(path) as f:
        data = json.load(f)
    return {int(g): NeedleSpec(gauge=int(g), **dims)
            for g, dims in data["needles"].items()}

def load_syringe_catalog(path: str = "config/hardware/syringes.json") -> dict:
    """Load syringe specs from JSON. Returns {volume_uL: SyringeSpec}."""
    with open(path) as f:
        data = json.load(f)
    stroke = data["stroke_length_mm"]
    return {int(v): SyringeSpec(volume_uL=int(v), stroke_length_mm=stroke, **s)
            for v, s in data["syringes"].items()}

@dataclass
class NeedleSpec:
    """Physical needle specification — loaded from needles.json."""
    gauge: int                      # 16–32+
    od_um: float                    # Outer diameter (µm)
    id_um: float                    # Inner diameter (µm)
    wall_um: float                  # Wall thickness (µm)
    length_inches: float = 1.0      # 1.0 or 2.0 (user-selected in GUI)
    num_channels: int = 1           # 1 for single, up to 3 for multi-channel
    channel_pump_map: dict = None   # e.g. {1: "P1", 2: "P2"} for multi-channel

    @property
    def length_mm(self) -> float:
        return self.length_inches * 25.4

    @property
    def id_mm(self) -> float:
        return self.id_um / 1000.0

    @property
    def od_mm(self) -> float:
        return self.od_um / 1000.0

@dataclass
class SyringeSpec:
    """Hamilton 1700-series syringe — loaded from syringes.json."""
    volume_uL: int                  # 25, 50, 100, 250, 500, 1000
    stroke_length_mm: float = 30.0
    barrel_id_mm: float = 0.0
    part_number: str = ""

    @property
    def uL_per_mm(self) -> float:
        return self.volume_uL / self.stroke_length_mm

    @property
    def mm_per_uL(self) -> float:
        return self.stroke_length_mm / self.volume_uL

    @property
    def cross_section_area_mm2(self) -> float:
        """Barrel cross-sectional area for flow calculations."""
        return math.pi * (self.barrel_id_mm / 2) ** 2
```

### 3b. Ink & Fluid Column Model

The physical setup is: **mineral oil → buffer → ink** inside the syringe+tubing+needle.
The oil stays in the syringe as a hydraulic medium. A buffer layer separates oil
from ink to prevent mixing. The ink is drawn up into the needle/tubing from a well.

```python
@dataclass
class InkSpec:
    """Printing material specification."""
    name: str
    ink_type: str                   # "granular", "cells", "media", "hydrogel", "buffer", "custom"
    viscosity_cP: float = 1.0
    granule_diameter_um: float = 0  # For granular inks
    cell_diameter_um: float = 0     # For cell inks
    density_g_mL: float = 1.0
    color: str = "#a6e3a1"          # Display color (hex)

    def can_flow_through(self, needle: NeedleSpec) -> str:
        """Check if ink can pass through needle. Returns status string."""
        # Free flow:       needle_ID > 4 × max(granule, cell) diameter
        # Risk of clogging: 1× < needle_ID < 4× → warning
        # Pick-and-place:  needle_ID < 1× → can only aspirate/deposit one at a time
        ...

class PrintingMode(Enum):
    """How a pump delivers ink during a print run."""
    INCREMENTAL = "incremental"
    # Pick up a small volume of ink before each well/object.
    # Workflow: aspirate ink → print → (optionally waste/wash) → repeat
    # Pros: Fresh ink each time, less waste
    # Cons: Slower, more needle travel

    CONTINUOUS = "continuous"
    # Fill the entire syringe (or a large portion) with ink upfront.
    # Workflow: fill syringe → print many wells → refill when empty
    # Pros: Faster, fewer interruptions
    # Cons: Ink sits in syringe longer, may settle

@dataclass
class FluidColumn:
    """
    Tracks the layered fluid state inside one syringe + tubing + needle.

    Physical layout (from plunger tip to needle tip):
        [=== mineral oil ===][== buffer ==][=== ink ===] → needle tip

    All lengths in mm of plunger travel (convertible to µL via syringe spec).
    """
    oil_volume_uL: float = 0.0       # Mineral oil (hydraulic medium, always present)
    buffer_volume_uL: float = 0.0    # Buffer separating oil from ink
    ink_volume_uL: float = 0.0       # Current ink loaded
    ink_spec: InkSpec | None = None   # What ink is currently loaded
    dead_volume_uL: float = 2.0      # Tubing + needle dead volume (measured)

    @property
    def total_volume_uL(self) -> float:
        return self.oil_volume_uL + self.buffer_volume_uL + self.ink_volume_uL

    def can_dispense(self, amount_uL: float) -> bool:
        """Check if enough ink remains to dispense the requested amount."""
        return self.ink_volume_uL >= amount_uL

    def dispense(self, amount_uL: float):
        """Record dispensing ink (plunger pushes oil→buffer→ink out)."""
        self.ink_volume_uL = max(0, self.ink_volume_uL - amount_uL)

    def aspirate_ink(self, amount_uL: float, ink: InkSpec):
        """Record aspirating ink into needle (plunger pulls back)."""
        self.ink_volume_uL += amount_uL
        self.ink_spec = ink

    def waste_ink(self):
        """Eject all ink to waste (push until only buffer remains)."""
        self.ink_volume_uL = 0
        self.ink_spec = None

    def refresh_buffer(self, buffer_uL: float, buffer_ink: InkSpec):
        """Reset buffer layer (waste old buffer+ink, aspirate fresh buffer)."""
        self.ink_volume_uL = 0
        self.ink_spec = None
        self.buffer_volume_uL = buffer_uL

@dataclass
class PumpLoadout:
    """Configuration and state for one pump channel."""
    pump_id: str                        # "P1", "P2", "P3"
    syringe: SyringeSpec | None = None
    fluid_column: FluidColumn = field(default_factory=FluidColumn)
    printing_mode: PrintingMode = PrintingMode.INCREMENTAL
    current_position_mm: float = 0.0    # Tracked plunger position (mm)

    @property
    def current_position_uL(self) -> float:
        if self.syringe:
            return self.current_position_mm * self.syringe.uL_per_mm
        return 0.0

    @property
    def remaining_ink_uL(self) -> float:
        return self.fluid_column.ink_volume_uL

    @property
    def current_ink(self) -> InkSpec | None:
        return self.fluid_column.ink_spec
```

### 3c. Rosette Well Insert Model

A rosette insert subdivides a standard well into multiple sub-wells arranged in
a circular pattern. Used for: ink reservoirs, sorted cell collection, wash/waste/buffer
stations. The same physical insert geometry is reused for different purposes.

```python
@dataclass
class RosetteSubWell:
    """One sub-well within a rosette insert."""
    index: int                      # 0-based position in rosette
    angle_deg: float                # Angular position (0° = 12 o'clock)
    radial_offset_mm: float         # Distance from well center to sub-well center
    diameter_mm: float              # Sub-well opening diameter
    depth_mm: float                 # Sub-well depth
    z_offset_mm: float = 0.0       # Z offset of sub-well bottom vs well bottom

@dataclass
class RosetteInsert:
    """
    A rosette insert that fits into a standard well.

    Physical object: a cylindrical plug with N sub-wells arranged in a
    ring pattern, optionally with a center sub-well.

    The same physical insert can serve different roles:
    - Ink rosette: each sub-well holds a different ink
    - Sort rosette: each sub-well collects a different cell type
    - Service rosette: wash + waste + buffer in one well
    """
    name: str
    well_format: int                # Which plate format this fits (6, 12, 24...)
    num_subwells: int               # Total sub-wells including center
    has_center_well: bool = True    # Whether there's a center sub-well
    ring_radius_mm: float = 0.0     # Radius of the sub-well ring
    subwell_diameter_mm: float = 0.0
    subwell_depth_mm: float = 0.0
    insert_z_offset_mm: float = 0.0 # How much the insert raises the bottom

    subwells: list[RosetteSubWell] = field(default_factory=list)

    @classmethod
    def create_standard(cls, name: str, well_format: int,
                        num_ring: int, has_center: bool = True,
                        **kwargs) -> "RosetteInsert":
        """
        Create a rosette with evenly-spaced sub-wells.

        Automatically calculates sub-well positions from the parent
        well diameter and number of sub-wells requested.
        """
        ...

    def get_subwell_xy(self, index: int) -> tuple[float, float]:
        """Get (x, y) offset of sub-well relative to parent well center."""
        sw = self.subwells[index]
        rad = math.radians(sw.angle_deg)
        return (sw.radial_offset_mm * math.sin(rad),
                sw.radial_offset_mm * math.cos(rad))

    def get_subwell_z_bottom(self, index: int) -> float:
        """Get Z position of sub-well bottom (for needle depth targeting)."""
        return self.insert_z_offset_mm + self.subwells[index].z_offset_mm
```

### 3d. Workspace Config

```python
@dataclass
class WorkspaceConfig:
    """Complete physical configuration for a print session."""
    needle: NeedleSpec
    pumps: dict[str, PumpLoadout]       # {"P1": ..., "P2": ..., "P3": ...}
    plate_format: int                   # 6, 12, 24, 48, 96
    rosette_library: dict[str, RosetteInsert]  # Named rosette definitions
    ink_library: dict[str, InkSpec]     # All available inks
    buffer_ink: InkSpec | None = None   # The buffer material (e.g. DPBS)
    print_settings: dict = field(default_factory=dict)
```

### Key Design Decisions:
- **Needle and syringe catalogs in JSON files** (`config/hardware/`) — users can
  add new gauge sizes or syringe models by editing JSON, no Python changes needed.
  The GUI dropdowns are populated from these files at startup.
- **Controller protocols in JSON files** (`config/controllers/`) — same principle,
  adding a new XY controller type = adding a JSON file.
- **Fluid column model** tracks oil/buffer/ink layers per syringe — the GUI can
  visualize this as a stacked bar in the syringe display widget
- **Printing mode per pump** — incremental (aspirate before each print) vs.
  continuous (fill syringe upfront) — independently selectable for each pump
- **Rosette inserts** are geometry-only; their *purpose* (ink, sort, service)
  is defined by the well assignment in Tab 3, not by the insert itself
- All volumes displayed in **µL** in the GUI; all plunger positions stored in **mm**
  internally and converted via `SyringeSpec.uL_per_mm`

### Tasks:
- [ ] P1.1: Create `PhysicalModels.py` with all dataclasses + JSON loaders
- [ ] P1.2: Create `config/hardware/needles.json` — all gauges 16G–32G (ASTM values)
- [ ] P1.3: Create `config/hardware/syringes.json` — Hamilton 1700-series (6 sizes)
- [ ] P1.4: `load_needle_catalog()` + `load_syringe_catalog()` JSON loaders
- [ ] P1.5: `InkSpec.can_flow_through()` — free-flow / clogging / pick-and-place
- [ ] P1.6: `FluidColumn` — oil/buffer/ink tracking with dispense/aspirate/waste/refresh
- [ ] P1.7: `PrintingMode` enum (incremental vs continuous)
- [ ] P1.8: `RosetteInsert` + `RosetteSubWell` geometry with auto-layout
- [ ] P1.9: `WorkspaceConfig` with full serialization to/from JSON
- [ ] P1.10: Unit tests — µL↔mm conversions, JSON loading, fluid column state,
             rosette geometry, ink compatibility checks

---

## 4. Phase 2 — Parametric Geometry Engine

### File: `SupportClasses/GeometryEngine.py`

Purpose: Generate print objects as time-parameterized trajectories accounting for
needle diameter and ink properties.

### Print Object Types:

#### 2D Objects (single Z layer):
| Object   | Parameters | Notes |
|----------|-----------|-------|
| Point    | (cx, cy) | Single deposition point, dwell time based on volume |
| Line     | (x1,y1)→(x2,y2), width | Single pass or multi-pass with line spacing = needle_OD × (1 - overlap) |
| Circle   | center, radius | Discrete segments, spacing = needle_OD |
| Square   | center, side_length | 4 lines with corner handling |
| Triangle | center, side_length | 3 lines |
| Spiral   | center, max_radius, spacing | Archimedes spiral, pitch = needle_OD × (1-overlap) |
| Ellipse  | center, a, b | Parametric ellipse |

#### 3D Objects (multi-layer):
| Object   | Parameters | Notes |
|----------|-----------|-------|
| Sphere (shell)   | center, radius, layer_h | Circular cross-sections varying by Z |
| Sphere (solid)   | center, radius, layer_h | Filled circles (meander/spiral fill) per layer |
| Cube (shell)     | center, side, layer_h | Square perimeters stacked |
| Cube (solid)     | center, side, layer_h | Filled squares per layer |
| Cylinder (shell) | center, r, height, layer_h | Circular perimeters stacked |
| Cylinder (solid) | center, r, height, layer_h | Filled circles stacked |
| Ellipsoid (shell/solid) | center, a, b, c, layer_h | Elliptical cross-sections |

### Core Calculations:

```python
@dataclass
class PrintObject:
    """A parametric print object defined in (x,y,z,p1,p2,p3,t) space."""
    name: str
    object_type: str                    # "point", "line", "circle", etc.
    params: dict                        # Type-specific parameters
    position: tuple[float,float,float]  # (x,y,z) offset within well
    ink_assignments: dict               # {"P1": InkSpec, ...}
    color: str = "#a6e3a1"              # Display color

    # Generated trajectory
    trajectory: np.ndarray | None = None  # Nx7 array: [x,y,z,p1,p2,p3,t]

def generate_object_trajectory(
    obj: PrintObject,
    needle: NeedleSpec,
    syringe_map: dict[str, SyringeSpec],
    overlap_fraction: float = 0.0,      # 0.0 = no overlap, 0.2 = 20% overlap
) -> np.ndarray:
    """
    Generate the (x,y,z,p1,p2,p3,t) trajectory for a print object.

    Line spacing = needle_OD * (1 - overlap_fraction)
    Extrusion rate = cross_section_area * linear_speed (volume conservation)
    Time parameterization based on target print speed.

    Returns Nx7 numpy array.
    """
```

### Extrusion Volume Model:
The key insight: **volume in = volume out**.

```
Deposited cross-section ≈ needle_OD × layer_height  (rectangular approximation)
Volume per mm of travel = needle_OD × layer_height   (mm³/mm = mm²)
Required flow rate = travel_speed × needle_OD × layer_height  (mm³/s)
Pump rate (mm/s) = flow_rate_mm3_per_s / syringe.cross_section_area_mm2
Pump rate (µL/s) = flow_rate_mm3_per_s  (1 mm³ = 1 µL)
```

**Important:** We do NOT apply a granular packing fraction correction to the
extrusion rate. The system deposits ink *into* a granular support medium that
already exists in the well — it does not extrude granular material itself.
The granule/cell size in `InkSpec` is used only for:
- Needle compatibility checks (jamming vs. free-flow vs. pick-and-place)
- Shear stress safety calculations
- Flow regime classification
The volumetric extrusion rate is purely based on the deposited filament
geometry and the fluid properties of the ink being dispensed.

### Tasks:
- [ ] P2.1: `PrintObject` dataclass with trajectory storage
- [ ] P2.2: 2D generators: point, line, circle, square, triangle, spiral, ellipse
- [ ] P2.3: 3D generators: sphere, cube, cylinder, ellipsoid (shell + solid variants)
- [ ] P2.4: Needle-aware line spacing calculator
- [ ] P2.5: Volume-conserving extrusion rate calculator
- [ ] P2.6: Time parameterization for each object type
- [ ] P2.7: `PrintCollection` — a set of positioned/colored print objects for one well
- [ ] P2.8: Unit tests for geometry + extrusion calculations

---

## 5. Phase 3 — Trajectory Planner & Motion Control

### File: `SupportClasses/TrajectoryPlanner.py`

Purpose: Convert parametric objects into executable time-interpolated waypoints.

```python
@dataclass
class Waypoint:
    """A single point in the trajectory with all axis positions and timestamp."""
    t: float        # seconds from print start
    x: float        # µm (XY stage)
    y: float        # µm (XY stage)
    z: float        # mm (Z axis)
    p1: float       # mm (pump 1 position)
    p2: float       # mm (pump 2 position)
    p3: float       # mm (pump 3 position)

class TrajectoryPlanner:
    """
    Converts PrintObject trajectories into executable waypoint sequences.

    Responsibilities:
    1. Merge multiple PrintObject trajectories into a single timeline
    2. Insert travel moves (Z-up, XY travel, Z-down) between objects
    3. Handle retract/prime sequences at segment boundaries
    4. Interpolate at fixed timestep for the motion controller
    5. Import CSV trajectories: (x,y,z,p1,p2,p3,t) format
    """

    def plan_well_print(
        self,
        print_collection: PrintCollection,
        workspace: WorkspaceConfig,
        well_bottom_z: float,       # Z offset for this specific well
    ) -> list[Waypoint]:
        """Generate complete waypoint list for printing one well."""
        ...

    def import_csv_trajectory(self, filepath: str) -> np.ndarray:
        """Load (x,y,z,p1,p2,p3,t) from CSV file.  Validates columns."""
        ...

    def interpolate_trajectory(
        self,
        raw_waypoints: np.ndarray,
        dt: float = 0.05,          # 50ms interpolation step
    ) -> np.ndarray:
        """
        Resample trajectory at fixed timestep using cubic spline
        interpolation.  Handles sharp corners by detecting high
        curvature and inserting deceleration zones.
        """
        ...
```

### File: `SupportClasses/MotionController.py`

Purpose: Real-time tracking controller that sends velocity commands to keep
the needle on the planned trajectory.

#### Control Architecture:

```
                    ┌─────────────────────────────────────────────┐
   Trajectory  ──►  │  Feedforward   │  Kalman     │  Correction  │ ──► Stage
   (desired)        │  Velocity Calc │  State Est  │  Output      │     Commands
                    └───────┬────────┴──────┬──────┴──────────────┘
                            │               │
                   Planned velocity    Position feedback
                   from trajectory     from position poller
```

#### Strategy 1: Kalman-Filtered Predictive Controller (PRIMARY)

```python
class KalmanMotionController:
    """
    Uses a Kalman filter to predict the required velocity commands
    based on:
    - Current estimated position (filtered from noisy position reads)
    - Desired trajectory position at current + lookahead time
    - History of N previous command→response pairs
    - Knowledge of the stage's response latency

    State vector: [x, y, vx, vy, ax, ay]  (position, velocity, acceleration)
    Measurement: [x, y] from position poller
    Control input: [vx_cmd, vy_cmd] sent to stage

    The filter accounts for:
    - Communication latency (XY stage: 300ms–1000ms round-trip)
    - Command processing delay
    - Stage acceleration/deceleration profiles
    """

    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.state = np.zeros(6)        # [x, y, vx, vy, ax, ay]
        self.P = np.eye(6) * 1000       # State covariance
        self.Q = ...                     # Process noise (tuned)
        self.R = ...                     # Measurement noise (tuned)
        self.F = ...                     # State transition matrix
        self.H = np.zeros((2, 6))       # Measurement matrix [x, y]
        self.H[0, 0] = 1; self.H[1, 1] = 1

        self.lookahead_steps: int = 5   # How far ahead to look on trajectory
        self.command_history: deque = deque(maxlen=50)

    def update(
        self,
        measured_pos: tuple[float, float],
        desired_trajectory: list[Waypoint],
        current_time: float,
    ) -> tuple[float, float]:
        """
        Returns (vx_command, vy_command) to send to XY stage.

        1. Kalman predict step (propagate state forward)
        2. Kalman update step (correct with measurement)
        3. Compute lookahead target from trajectory
        4. Calculate feedforward velocity + correction
        5. Apply rate limiting for stage safety
        """
        ...

    def reset(self):
        """Reset filter state (call when starting new print)."""
        ...
```

#### Strategy 2: Feedforward + PID with Auto-Tuning (SECONDARY)

```python
class PIDMotionController:
    """
    Classical PID with feedforward for trajectory tracking.

    Feedforward: velocity directly from trajectory derivative
    PID: corrects for position error

    Auto-tuning via relay feedback (Ziegler-Nichols):
    - Run a simple oscillation test on each axis
    - Measure ultimate gain Ku and period Tu
    - Compute Kp, Ki, Kd from ZN formulas
    - Store tuned values in settings.json
    """

    def __init__(self):
        self.kp = 0.5   # Proportional (conservative default)
        self.ki = 0.01   # Integral
        self.kd = 0.02   # Derivative
        self.max_output = 5000  # Max velocity command (µm/s)
        self.integral_limit = 1000  # Anti-windup

    def auto_tune(self, controller: StageController) -> dict:
        """
        Run Ziegler-Nichols relay auto-tune sequence.
        Moves stage in small oscillation, measures response,
        computes optimal PID gains.

        Returns: {"kp": float, "ki": float, "kd": float}
        """
        ...
```

#### Strategy 3: Model Predictive Control (ADVANCED/FUTURE)

```python
class MPCMotionController:
    """
    Model Predictive Control — optimizes velocity commands over
    a prediction horizon considering:
    - Stage dynamics model (acceleration limits)
    - Communication delay
    - Path constraints (sharp corners)
    - Position bounds (safety limits)

    Uses scipy.optimize.minimize with a receding horizon.
    More computationally expensive but handles corners better.
    """
    ...
```

### Corner Handling Strategy:
Sharp corners require deceleration → stop → re-accelerate.
The trajectory planner pre-processes the path to:
1. Detect corners (angle between consecutive segments > threshold)
2. Insert deceleration zones before corners
3. Insert dwell/stop at corner vertices
4. Insert acceleration zones after corners

### XY Stage Rate Testing:
```python
def test_stage_command_rate(controller: StageController) -> dict:
    """
    Measure actual stage responsiveness by commanding small, safe
    movements and comparing commanded vs actual positions over time.

    ⚠️ SAFETY: Uses LOW velocity and SMALL displacements only.
    The stage must be connected and clear of obstacles.

    Procedure:
    1. Record current position as home
    2. Command a series of small moves (e.g. ±500 µm) at LOW speed
       (10% of max, typically ~1000 µm/s)
    3. After each move command, poll position at various intervals
       (100ms, 200ms, 333ms, 500ms, 1000ms)
    4. Measure: time from command sent → movement detected,
       time from command → position settled,
       position accuracy at each polling interval
    5. Return home position

    Results:
    {
        "command_response_ms": float,    # Time until stage starts moving
        "settle_time_ms": float,         # Time until position stabilizes
        "min_reliable_poll_ms": int,     # Fastest poll that gives stable reads
        "max_command_hz": float,         # Effective command rate
        "position_accuracy_um": float,   # Typical overshoot/undershoot
        "avg_round_trip_ms": float,      # Send command + read position
    }

    Store results in settings.json for motion controller tuning.
    """
```

### Tasks:
- [ ] P3.1: `Waypoint` dataclass, `TrajectoryPlanner` class
- [ ] P3.2: CSV import with (x,y,z,p1,p2,p3,t) validation
- [ ] P3.3: Cubic spline interpolation at fixed timestep
- [ ] P3.4: Sharp corner detection + deceleration zone insertion
- [ ] P3.5: Travel move insertion (Z-up → XY move → Z-down)
- [ ] P3.6: `KalmanMotionController` — state estimation + command generation
- [ ] P3.7: `PIDMotionController` — feedforward + PID + auto-tune
- [ ] P3.8: Stage command rate tester (add to Settings page)
- [ ] P3.9: `StageController` API additions: `set_velocity_xy()`, `get_latency_stats()`
- [ ] P3.10: Unit tests: interpolation accuracy, corner handling, Kalman convergence

---

## 6. Phase 4 — Flow Physics & Safety

### File: `SupportClasses/FlowPhysics.py`

Purpose: Calculate safe flow rates and expected pressures for needle/syringe/ink combos.

### Pressure Model (Hagen-Poiseuille for laminar flow):

```
ΔP = (128 × µ × L × Q) / (π × d⁴)

Where:
  ΔP = pressure drop (Pa)
  µ  = dynamic viscosity (Pa·s) = viscosity_cP × 0.001
  L  = needle length (m)
  Q  = volumetric flow rate (m³/s)
  d  = needle inner diameter (m)
```

### Reynolds Number Check:
```
Re = (ρ × v × d) / µ
v  = Q / (π × (d/2)²)

Laminar:    Re < 2100  (Hagen-Poiseuille valid)
Transition: 2100 < Re < 4000
Turbulent:  Re > 4000  (need different model)
```

### Safety Thresholds:

```python
@dataclass
class FlowSafetyResult:
    """Result of flow safety calculation."""
    max_safe_flow_rate_uL_s: float    # Maximum flow before pressure limit
    pressure_at_requested_rate_Pa: float
    pressure_limit_Pa: float          # Configurable (default ~200 kPa for syringes)
    reynolds_number: float
    flow_regime: str                  # "laminar", "transition", "turbulent"
    is_safe: bool
    warnings: list[str]               # e.g. "Near jamming threshold for granular ink"

def calculate_flow_safety(
    needle: NeedleSpec,
    syringe: SyringeSpec,
    ink: InkSpec,
    requested_flow_rate_uL_s: float,
    pressure_limit_Pa: float = 200_000,  # 200 kPa default
) -> FlowSafetyResult:
    """
    Calculate whether a flow rate is safe for the given hardware combo.

    Also checks:
    - Granular ink: warn if needle ID < 4× granule size
    - Cell ink: warn if needle ID < 4× cell diameter
    - Shear stress on cells: τ = (32 × µ × Q) / (π × d³)
    """
    ...
```

### Granular Flow Considerations:
```
Jamming regime:       needle_ID < granule_diameter       → pick-and-place only
Intermittent regime:  granule_diameter < needle_ID < 4×  → risk of clogging
Free-flow regime:     needle_ID > 4 × granule_diameter   → continuous extrusion OK

For cells: same ratios apply, with typical cell diameters:
  - HEK293: ~15 µm
  - MSC: ~20 µm
  - Hepatocytes: ~25 µm
  - Organoids: 100–500 µm
```

### Tasks:
- [ ] P4.1: `FlowPhysics.py` with Hagen-Poiseuille calculator
- [ ] P4.2: Reynolds number computation
- [ ] P4.3: Granular/cell flow regime classification
- [ ] P4.4: Cell shear stress calculation
- [ ] P4.5: `FlowSafetyResult` dataclass with `is_safe` and `warnings`
- [ ] P4.6: Integration with `SafetyLimits.py` — add per-needle flow limits
- [ ] P4.7: Unit tests with known needle/syringe combos

---

## 7. Phase 5 — Print Setup GUI (3 Tabs)

### Restructured `print_setup.py`:
The existing file becomes a thin shell hosting a `QTabWidget` with 3 tabs.

```python
class PrintSetupPage(QWidget):
    """Print setup with 3 workflow tabs."""

    def __init__(self, controller, settings, parent=None):
        ...
        self.workspace_config: WorkspaceConfig = None

        self.tabs = QTabWidget()
        self.tab_workspace = WorkspaceTab(controller, settings)
        self.tab_objects = PrintObjectsTab(controller, settings)
        self.tab_wells = WellSetupTab(controller, settings)

        self.tabs.addTab(self.tab_workspace, "1. Workspace")
        self.tabs.addTab(self.tab_objects, "2. Print Objects")
        self.tabs.addTab(self.tab_wells, "3. Well Setup")
```

---

### Tab 1: Setup Workspace (`gui/pages/print_workspace.py`)

**Purpose:** Configure the physical setup before designing prints.

**Layout:**
```
┌──────────────────────────────────────────────────────────┐
│  Plate Format     │  [6] [12] [24] [48] [96]  dropdown  │
├───────────────────┼──────────────────────────────────────┤
│  Needle Config    │  Gauge: [22▾]  Length: [1.0"▾]       │
│                   │  Channels: [1▾]  Channel→Pump map    │
│                   │  ID: 413 µm  OD: 718 µm  (auto)     │
├───────────────────┼──────────────────────────────────────┤
│  Pump Loadout     │                                      │
│  ┌──────────────────────────────────────────────────────┐│
│  │ P1: Syringe [100µL▾]  Mode [Incremental▾]           ││
│  │     Fluid: [===oil===][=buf=][==ink==]→ tip          ││
│  │     Ink loaded: 8.2 µL of "Hydrogel A"              ││
│  │ P2: Syringe [250µL▾]  Mode [Continuous▾]            ││
│  │     Fluid: [=======oil=======][buf][====cells====]→  ││
│  │     Ink loaded: 142 µL of "MSC Cells"               ││
│  │ P3: Syringe [50µL▾]   Mode [Incremental▾]           ││
│  │     Fluid: [===oil===][=buf=][ ]→ tip (empty)       ││
│  └──────────────────────────────────────────────────────┘│
│  Buffer material: [DPBS▾]  Dead volume: [2.0] µL        │
├───────────────────┼──────────────────────────────────────┤
│  Ink Library      │  [+Add] [Edit] [Delete]              │
│  (persistent)     │  Name | Type | Viscosity | Granule Ø │
│                   │  Hydrogel A | hydrogel | 50 cP | —   │
│                   │  MSC Cells  | cells    | 1.2 cP| 20µm│
│                   │  DPBS       | buffer   | 1.0 cP| —   │
│                   │  ... scrollable table ...             │
├───────────────────┼──────────────────────────────────────┤
│  Rosette Library  │  [+New] [Edit] [Delete]              │
│  (insert defs)    │  Name | Subwells | Fits | Depth      │
│                   │  Ink-6  | 6+center | 24w | 8mm       │
│                   │  Sort-4 | 4+center | 24w | 6mm       │
│                   │  ... editable table ...               │
├───────────────────┼──────────────────────────────────────┤
│  Print Settings   │  Travel Z | Layer H | Print Speed    │
│  (defaults)       │  Z Feed | Pump Feed | Retract/Prime  │
├───────────────────┼──────────────────────────────────────┤
│  Compatibility    │  ✅ P1: Hydrogel A flows freely      │
│  Report           │  ⚠️ P2: Cells near jamming limit    │
│  (auto-computed)  │  ✅ P3: (no ink loaded)              │
│                   │  Max safe rate P1: 2.3 µL/s          │
│                   │  Max safe rate P2: 0.8 µL/s          │
└───────────────────┴──────────────────────────────────────┘
```

**Context Panel:** Save/Load workspace configs, syringe catalog quick reference,
rosette insert editor with visual preview.

### Key Interactions:
- **Syringe selection** from dropdown (6 Hamilton sizes) → auto-populates barrel ID
  and µL/mm ratio. No custom syringe entry needed.
- **Printing mode per pump** — Incremental means the system will aspirate a small
  volume before each well/object; Continuous means fill the syringe upfront and
  print many wells before refilling.
- **Fluid column display** — a horizontal stacked bar per pump showing oil (gray),
  buffer (light blue), ink (ink's display color). Updates in real time during prints.
- **Buffer material** — selected from ink library, used as the separator between
  oil and ink. The system aspirates buffer from a designated buffer well.
- **Rosette library** — define insert geometries once, then assign to wells in Tab 3.
  Each rosette is geometry-only; the *role* of each sub-well is set in Tab 3.

### Tasks:
- [ ] P5.1: `WorkspaceTab` widget with all config sections
- [ ] P5.2: Needle gauge dropdown with auto-calculated dimensions display
- [ ] P5.3: Pump loadout editor (syringe from catalog, printing mode, fluid column bar)
- [ ] P5.4: Ink library manager (add/edit/delete, persisted in settings.json)
- [ ] P5.5: Rosette library manager (add/edit/delete with visual geometry preview)
- [ ] P5.6: Buffer material selector + dead volume setting
- [ ] P5.7: Auto-compatibility report (calls `FlowPhysics.calculate_flow_safety()`)
- [ ] P5.8: Save/Load workspace JSON
- [ ] P5.9: Signal to propagate workspace config to other tabs when changed

---

### Tab 2: Print Objects (`gui/pages/print_objects.py`)

**Purpose:** Design parametric print objects and build print collections.

**Main Content Layout:**
```
┌──────────────────────────────────┬────────────────────────────┐
│  Object Designer                 │  Well Preview              │
│  ┌────────────────────────────┐  │  ┌──────────────┬─────────┐│
│  │ Type: [Cylinder (solid)▾]  │  │  │              │  ZY     ││
│  │ Radius: [2.0] mm          │  │  │  XY Top-Down │  Side   ││
│  │ Height: [3.0] mm          │  │  │  (large)     │  View   ││
│  │ Fill: [Spiral▾]           │  │  │              │  (tall) ││
│  │ Overlap: [10]%            │  │  │    ┌────┐    │  ┌───┐  ││
│  │ Ink: [P1: Hydrogel A▾]   │  │  │    │    │    │  │   │  ││
│  │ [Generate Preview]        │  │  │    └────┘    │  └───┘  ││
│  └────────────────────────────┘  │  ├──────────────┴─────────┤│
│                                  │  │  XZ Bottom View (wide) ││
│  Object Library:                 │  │  ┌───────────────────┐ ││
│  ┌────────────────────────────┐  │  │  │     ▓▓▓▓▓▓▓▓▓     │ ││
│  │ ● Scaffold_v1 (Cyl,solid) │  │  │  └───────────────────┘ ││
│  │ ● CellRing (Circle)       │  │  └────────────────────────┘│
│  │ ● MediaDrop (Point)       │  │                            │
│  │ [+Add to Library] [Del]   │  │  [▶ Simulate Print]       │
│  └────────────────────────────┘  │  [Import CSV Trajectory]   │
│                                  │                            │
│  Print Collection for Well:      │                            │
│  ┌────────────────────────────┐  │                            │
│  │ 1. Scaffold_v1 @ (0,0,0)  │  │                            │
│  │ 2. CellRing @ (0,0,3.1)   │  │                            │
│  │ 3. MediaDrop @ (1,0,3.2)  │  │                            │
│  │ [+Add] [↑] [↓] [Del]     │  │                            │
│  └────────────────────────────┘  │                            │
└──────────────────────────────────┴────────────────────────────┘
```

**The Well Preview** uses the `ProjectionCanvas` widget in L-shaped layout
(XY large top-left, ZY tall right, XZ wide below) showing:
- Well boundary (circle with correct diameter from plate spec)
- All objects in the print collection, color-coded by ink
- Needle path as lines within the well

**CSV Import** loads (x,y,z,p1,p2,p3,t) files as custom trajectory objects.

**Simulate Print** plays back the trajectory in the preview at configurable speed.

**Context Panel:** Object type reference, print collection manager, color legend.

### Tasks:
- [ ] P5.10: `PrintObjectsTab` widget layout
- [ ] P5.11: Object designer form (type dropdown, parameters, ink assignment)
- [ ] P5.12: Parametric object preview generation (calls `GeometryEngine`)
- [ ] P5.13: Object library (add/edit/delete/duplicate)
- [ ] P5.14: Print collection builder (ordered list of objects with positions)
- [ ] P5.15: `ProjectionCanvas` widget — L-shaped layout: XY large (top-left), ZY tall (right), XZ wide (bottom)
- [ ] P5.16: CSV trajectory import and display
- [ ] P5.17: Print simulation playback (animated preview)
- [ ] P5.18: Color coding by ink assignment

---

### Tab 3: Well Setup (`gui/pages/print_well_setup.py`)

**Purpose:** Assign print collections to wells, detect well bottom plane,
assign well roles (ink, wash, waste, buffer, sorted cells), attach rosette
inserts, and configure ink pickup workflows. This is where the full plate
layout is defined before printing.

**Main Content Layout:**
```
┌──────────────────────────────────────┬──────────────────────┐
│  Interactive Plate View (XY)         │  ZY Side Projection  │
│  ┌────────────────────────────────┐  │  (tall, shows well   │
│  │ ● ● ● ● ● ● ● ● ● ● ● ● │  │  bottoms + rosette   │
│  │ ● ● 🟢🟢● ● ● ● ● ● ● ● │  │  z-offsets + needle)  │
│  │ ● ● 🟢🟢● ● ● ● ● ● ● ● │  │  ┌──────────────────┐│
│  │ ● ● ● ● ● ● ● ● 🔵🔵● ● │  │  │   ▼ needle tip    ││
│  │ ● ● ● ● ● ● ● ● 🔵🔵● ● │  │  │  ┌─┐  ┌─┐  ┌─┐  ││
│  │ ● ● ● ● ● ● ● ● ● ● ● ● │  │  │  └─┘  └─┘  └─┘  ││
│  │ ● ● ● ● ● ● ● ● ● ● ● ● │  │  │  well bottoms     ││
│  │ 🟡🟣● ● ● ● ● ● ● ● 🔴🟤│  │  └──────────────────┘│
│  └────────────────────────────────┘  │                      │
│  ▒▒▒▒ = selected (rubber band)       │                      │
│  Legend: 🟢Print 🔵Ink 🟡Wash        │                      │
│          🔴Waste 🟣Buffer 🟤Sort      │  Needle: ✛ (live)    │
│  [3 wells selected]                  │                      │
├──────────────────────────────────────┴──────────────────────┤
│  XZ Bottom Projection (wide, shows well bottoms + needle)   │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         ▼ needle tip                                    │ │
│  │    ┌─┐  ┌─┐  ┌─┐  ┌─┐  ┌─┐  ┌─┐  ┌─┐  ┌─┐          │ │
│  │    └─┘  └─┘  └─┘  └─┘  └─┘  └─┘  └─┘  └─┘          │ │
│  └────────────────────────────────────────────────────────┘ │
├────────────────────────────────────┴───────────────────────────┤
│  Selection Actions:   (applies to all selected wells)         │
│  ┌───────────────────────────────────────────────────────────┐│
│  │ Set Role: [Print▾] [Ink 1▾] [Ink 2▾] [Wash▾] [Waste▾]   ││
│  │           [Buffer▾] [Sorted 1▾] [Sorted 2▾] [Empty▾]     ││
│  │ Assign Print: [Scaffold_v1▾] [+Append] [Replace] [Clear] ││
│  │ Attach Rosette: [None▾] [Ink-6▾] [Sort-4▾]               ││
│  │ Sub-well roles: (shown when rosette attached)             ││
│  │   ┌─────────────────────────────────────────────┐         ││
│  │   │ Center: [Ink 1: Hydrogel A ▾]               │         ││
│  │   │ Ring 1: [Ink 2: MSC Cells  ▾]               │         ││
│  │   │ Ring 2: [Ink 3: Collagen   ▾]               │         ││
│  │   │ Ring 3: [Wash ▾]                            │         ││
│  │   │ Ring 4: [Waste ▾]                           │         ││
│  │   │ Ring 5: [Buffer ▾]                          │         ││
│  │   └─────────────────────────────────────────────┘         ││
│  │ [Apply to Selected] [Reset Selected] [Select All Print]   ││
│  └───────────────────────────────────────────────────────────┘│
├───────────────────────────────────────────────────────────────┤
│  Well Bottom Detection:                                       │
│  Jog needle to glass in 3+ wells → Calculate plane            │
│  [A1: ✅ z=-0.12] [D6: ✅ z=-0.08] [H12: ✅ z=-0.15]        │
│  Plane: z = -0.12 + 0.0003x - 0.0002y  R²=0.998             │
│  [Teach Current Well] [Calculate Plane] [Clear Points]        │
├───────────────────────────────────────────────────────────────┤
│  Well Assignment Summary:  (scrollable, synced with plate)    │
│  Well │ Role     │ Insert   │ Subwell Roles  │ Print       │ Z │
│  A1   │ Print    │ —        │ —              │ Scaffold_v1 │+.02│
│  A2   │ Print    │ —        │ —              │ Scaffold_v1 │+.01│
│  G1   │ Ink      │ Ink-6    │ 🔵🔵🔵🔵🔵🔵🔵│ —          │-.05│
│  G12  │ Service  │ Sort-4   │ 🟡🔴🟣🟤🟤  │ —           │-.03│
│  H1   │ Wash     │ —        │ —              │ —           │-.04│
│  H12  │ Waste    │ —        │ —              │ —           │-.06│
│  [Save Layout] [Load Layout] [Auto-assign Pattern...]        │
└───────────────────────────────────────────────────────────────┘
```

### Multi-Select Well Interaction:

The plate view is a fully interactive `QGraphicsScene`-based widget supporting:

```python
class WellPlateView(QGraphicsView):
    """
    Interactive top-down well plate visualization with multi-select.

    Selection methods:
    1. Click        — select single well (clears previous selection)
    2. Ctrl+Click   — toggle individual well in/out of selection
    3. Shift+Click  — range select (rectangle from last click to current)
    4. Rubber-band  — click-drag to draw selection rectangle
    5. Row/Col header click — select entire row or column
    6. Ctrl+A       — select all wells

    Visual feedback:
    - Selected wells get a bright highlight border
    - Hover shows tooltip with well name, role, assigned prints
    - Wells color-coded by role (consistent color scheme across app)
    - Rosette wells show a tiny rosette icon overlay
    - Print wells show a mini-preview of assigned geometry

    Signals:
    - selection_changed(list[str])  — emits list of selected well names
    - well_double_clicked(str)      — opens detail editor for one well
    - context_menu_requested(list[str], QPoint) — right-click menu
    """

    # Right-click context menu on selection:
    #   "Assign Print..." → submenu of available PrintCollections
    #   "Set Role" → submenu of roles
    #   "Attach Rosette..." → submenu of rosette library
    #   "Clear Assignments"
    #   "Reset to Empty"
    #   ─────────────
    #   "Select All with Same Role"
    #   "Select All with Same Print"
```

### Well Roles — Expanded:

```python
class WellRole(Enum):
    """Role assigned to a well or sub-well within a rosette."""
    EMPTY = "empty"                 # Unused / unassigned
    PRINT = "print"                 # Receives print objects from Tab 2
    INK = "ink"                     # Ink reservoir — pump aspirates from here
    WASH = "wash"                   # Wash station — needle jiggles randomly to clean
    WASTE = "waste"                 # Waste deposit — eject buffer+old ink to reset needle
    BUFFER = "buffer"               # Buffer pickup — aspirate buffer material
    SORTED_CELLS = "sorted"         # Deposit sorted/picked cells
```

### Well Role Behaviors (Automated Workflows):

Each role triggers specific automated behaviors during print execution:

```python
@dataclass
class WashBehavior:
    """
    Wash well: needle moves randomly within well to scrub off debris.

    The wash sequence:
    1. Travel to wash well at travel Z
    2. Lower needle into wash fluid to wash_depth
    3. Execute random XY jiggle pattern within well radius for wash_duration
    4. Raise needle to travel Z
    """
    wash_depth_mm: float = 2.0          # How deep into wash fluid
    wash_duration_s: float = 5.0        # How long to jiggle
    jiggle_radius_mm: float = 2.0       # Random motion radius
    jiggle_speed_mm_s: float = 5.0      # Speed of random motion
    num_jiggle_points: int = 20         # Number of random waypoints

@dataclass
class WasteBehavior:
    """
    Waste well: eject material to reset needle conditions.

    The waste sequence:
    1. Travel to waste well at travel Z
    2. Lower needle to waste_depth
    3. Push pump to eject waste_volume (old ink + some buffer)
    4. Dwell for drip_time
    5. Raise needle to travel Z

    This clears the old ink from the needle so fresh ink or
    buffer can be loaded. The amount ejected depends on the
    dead volume + any remaining ink in the fluid column.
    """
    waste_depth_mm: float = 1.0
    waste_volume_uL: float = 0.0        # 0 = auto (eject all ink + dead volume)
    eject_rate_uL_s: float = 1.0
    drip_time_s: float = 2.0

@dataclass
class BufferBehavior:
    """
    Buffer well: aspirate fresh buffer material.

    The buffer sequence:
    1. Travel to buffer well at travel Z
    2. Lower needle to buffer_depth
    3. Pull pump to aspirate buffer_volume
    4. Dwell for settle_time
    5. Raise needle to travel Z

    Buffer separates mineral oil from ink in the fluid column.
    Refreshing buffer is done after wasting old ink, before
    picking up new ink.
    """
    buffer_depth_mm: float = 3.0
    buffer_volume_uL: float = 5.0       # Amount of buffer to aspirate
    aspirate_rate_uL_s: float = 0.5
    settle_time_s: float = 1.0

@dataclass
class InkPickupBehavior:
    """
    Ink well: aspirate ink into needle.

    Behavior depends on printing mode:

    INCREMENTAL mode:
        1. Travel to ink well (or rosette sub-well) at travel Z
        2. Lower needle to ink_depth
        3. Pull pump to aspirate pickup_volume_uL
        4. Dwell for settle_time
        5. Raise needle to travel Z
        → Repeat before each print well/object

    CONTINUOUS mode:
        1. Travel to ink well at travel Z
        2. Lower needle to ink_depth
        3. Pull pump to aspirate fill_volume_uL (large amount or full syringe)
        4. Dwell for settle_time
        5. Raise needle to travel Z
        → Print many wells, refill when remaining ink < threshold

    For rosette sub-wells, the XY target is the sub-well center
    (offset from parent well center by rosette geometry).
    """
    ink_depth_mm: float = 3.0
    pickup_volume_uL: float = 2.0       # Per-pickup for incremental mode
    fill_volume_uL: float = 0.0         # For continuous mode (0 = fill syringe)
    refill_threshold_uL: float = 5.0    # Refill when remaining ink below this
    aspirate_rate_uL_s: float = 0.5
    settle_time_s: float = 1.0

@dataclass
class SortedCellBehavior:
    """
    Sorted cell well: deposit picked/sorted cells.

    The deposit sequence:
    1. Travel to sorted cell well (or rosette sub-well) at travel Z
    2. Lower needle to deposit_depth
    3. Push pump to eject deposit_volume
    4. Dwell for settle_time
    5. Raise needle to travel Z
    """
    deposit_depth_mm: float = 1.0
    deposit_volume_uL: float = 1.0
    eject_rate_uL_s: float = 0.5
    settle_time_s: float = 1.0
```

### Full Ink Change Workflow (Automated Sequence):

When the system needs to switch inks (e.g., pump P1 currently has Ink A,
needs Ink B for the next print group), the following sequence executes
automatically:

```
1. WASTE   → Travel to waste well → eject old ink + contaminated buffer
2. WASH    → Travel to wash well → jiggle needle to clean exterior
3. BUFFER  → Travel to buffer well → aspirate fresh buffer
4. INK     → Travel to ink well → aspirate new ink
5. (optional) WASTE → small waste shot to prime new ink to needle tip
6. PRINT   → Travel to print well → execute print
```

This sequence is configurable per pump and stored as a `ServiceSequence`:

```python
@dataclass
class ServiceSequence:
    """Configurable sequence of service operations between prints."""
    steps: list[str] = field(default_factory=lambda: [
        "waste", "wash", "buffer", "ink"
    ])
    # Each step references a well role; the planner finds the nearest
    # well (or rosette sub-well) with that role.
    # Steps can be reordered, repeated, or removed.
    # E.g., for continuous mode with same ink: just ["ink"] (top-up only)
    # E.g., for full ink change: ["waste", "wash", "buffer", "ink", "waste"]
```

### Well Assignment Data Model:

```python
@dataclass
class WellAssignment:
    """Complete assignment for a single well."""
    well_name: str
    role: WellRole = WellRole.EMPTY
    role_index: int = 0                 # For numbered roles: Ink 1, Ink 2, Sorted 1...
    ink_spec: InkSpec | None = None     # Which ink this well contains (for INK wells)

    # Print assignment (for PRINT wells)
    print_collections: list[str] = field(default_factory=list)  # Ordered list of print names
    print_offsets: list[tuple[float,float,float]] = field(default_factory=list)  # Per-print XYZ offsets

    # Rosette insert (optional — for any role)
    rosette: RosetteInsert | None = None
    subwell_roles: list[WellRole] = field(default_factory=list)      # Role per sub-well
    subwell_inks: list[InkSpec | None] = field(default_factory=list) # Ink per sub-well (for INK sub-wells)
    subwell_labels: list[str] = field(default_factory=list)          # User labels

    # Behaviors (populated based on role)
    wash_behavior: WashBehavior | None = None
    waste_behavior: WasteBehavior | None = None
    buffer_behavior: BufferBehavior | None = None
    ink_pickup: InkPickupBehavior | None = None
    sorted_deposit: SortedCellBehavior | None = None

    # Calibration
    z_offset: float = 0.0               # From plane fit
    manually_taught_z: float | None = None  # Override from manual teach

    # Display
    color: str = "#585b70"

# Color scheme (consistent across all views):
ROLE_COLORS = {
    WellRole.EMPTY:        "#585b70",   # Gray (Catppuccin surface2)
    WellRole.PRINT:        "#a6e3a1",   # Green
    WellRole.INK:          "#89b4fa",   # Blue
    WellRole.WASH:         "#f9e2af",   # Yellow
    WellRole.WASTE:        "#f38ba8",   # Red
    WellRole.BUFFER:       "#cba6f7",   # Mauve/Purple
    WellRole.SORTED_CELLS: "#fab387",   # Peach/Orange
}
```

### Tasks:
- [ ] P5.19: `WellSetupTab` widget layout
- [ ] P5.20: `WellPlateView` — `QGraphicsView`-based interactive plate widget
- [ ] P5.21: Multi-select: click, Ctrl+click, Shift+click, rubber-band drag, row/col headers
- [ ] P5.22: Right-click context menu on selection (assign, clear, reset, select-similar)
- [ ] P5.23: Projection views — ZY tall (right of plate), XZ wide (below plate) with well bottoms, rosette z-offsets, needle
- [ ] P5.24: Real-time needle position overlay (from Xbox controller jog)
- [ ] P5.25: `WellBottomDetector` — teach points + plane fitting (numpy lstsq)
- [ ] P5.26: Selection Actions panel — role assignment, print assignment, rosette attachment
- [ ] P5.27: Rosette sub-well role editor (shown when rosette attached to selected well)
- [ ] P5.28: `WellAssignment` dataclass with all role-specific behaviors
- [ ] P5.29: Well role behaviors: `WashBehavior`, `WasteBehavior`, `BufferBehavior`,
             `InkPickupBehavior`, `SortedCellBehavior`
- [ ] P5.30: `ServiceSequence` — configurable waste→wash→buffer→ink workflow
- [ ] P5.31: Color coding scheme (`ROLE_COLORS`) + legend widget
- [ ] P5.32: Well assignment summary table (scrollable, synced with plate selection)
- [ ] P5.33: Auto-assign patterns (fill row/col/block, checkerboard, etc.)
- [ ] P5.34: Print list per well — append, insert, reorder, remove individual prints
- [ ] P5.35: Save/Load full well setup to JSON (including all behaviors and rosettes)

---

## 8. Phase 6 — Print Monitor Page

### File: `gui/pages/print_monitor.py`

**Purpose:** A dedicated page (separate nav button) for monitoring active prints.
Added to the left menu as a 6th page icon (e.g. "📡 Monitor").

**Layout:**
```
┌─────────────────────────────────────────────────────────────────┐
│ PRINT MONITOR                                                   │
├───────────────────────┬─────────────────────────────────────────┤
│  Full Plate Overview  │  Current Well Detail                    │
│  (miniature Tab 3     │  ┌─────────────────────┬──────────────┐│
│   style, showing      │  │                     │   ZY View    ││
│   progress per well)  │  │   XY View (large)   │   (tall)     ││
│                       │  │                     │              ││
│  🟩🟩🟨⬜⬜⬜⬜⬜⬜⬜⬜⬜│  │   needle ✛          │   needle ✛   ││
│  🟩🟩🟨⬜⬜⬜⬜⬜⬜⬜⬜⬜│  │   path ---          │   path ---   ││
│  ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜│  │                     │              ││
│  ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜│  ├─────────────────────┴──────────────┤│
│  ...                  │  │   XZ View (wide, below)             ││
│                       │  │   needle ✛   path ---               ││
│  🟩=done 🟨=active    │  └─────────────────────────────────────┘│
│  ⬜=pending 🔵=ink     │  Path: ──── completed  ╌╌╌╌ upcoming  │
│                       │  Next 20 waypoints shown as dashed     │
├───────────────────────┼─────────────────────────────────────────┤
│  Syringe Status       │  Print Progress                        │
│  ┌─────┐ ┌─────┐ ┌──┐│  Job: Scaffold_Batch_1                 │
│  │░░░░░│ │▓▓░░░│ │▓▓││  Well: B3 (4 / 24)                    │
│  │░░░░░│ │▓▓░░░│ │▓▓││  Layer: 5 / 12                        │
│  │▓▓▓▓▓│ │▓▓▓▓▓│ │▓▓││  Step: 1,247 / 8,400                 │
│  │▓▓▓▓▓│ │▓▓▓▓▓│ │▓▓││  Time: 4:32 / ~12:00                 │
│  │ P1  │ │ P2  │ │P3││  [⏸ Pause] [⏹ Abort]                  │
│  │72 µL│ │45 µL│ │8 ││  ░░░░░░░░░░░░████████████ 52%         │
│  └─────┘ └─────┘ └──┘│                                         │
│  Needle: 22G × 1"    │  Tracking error: 12 µm (avg)           │
│  ID: 413 µm          │  Controller: Kalman (locked)            │
└───────────────────────┴─────────────────────────────────────────┘
```

### Real-Time Needle Display:
- Position dot with crosshair in all three projections
- Completed path as solid colored lines
- Next N waypoints as dashed lines (configurable, default N=20)
- Tracking error indicator (distance from planned position)
- Updated at position poller rate (~3 Hz)

### Syringe Display Widget:
```python
class SyringeDisplayWidget(QWidget):
    """
    Visual representation of a syringe showing the fluid column:

    ┌──────────────────┐
    │  ░░░░░░░░░░░░░░  │  ← mineral oil (gray)
    │  ░░░░░░░░░░░░░░  │
    │  ████████████████  │  ← buffer (light blue)
    │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │  ← ink (ink's display color)
    │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │
    └───────┤├─────────┘  ← needle tip
         P1 │ 100µL
     Ink: 42.3 µL Hydrogel A
     Buf: 5.0 µL  Mode: Incremental

    Shows:
    - Stacked fluid column (oil / buffer / ink) as colored bars
      proportional to volume within the syringe
    - Current plunger position
    - Total capacity + current ink volume in µL
    - Ink name + color
    - Printing mode indicator (Incremental / Continuous)
    - Flow rate indicator (small animated bar during active printing)
    - Warning icon if ink is low or buffer is depleted
    """
```

### Tasks:
- [ ] P6.1: Add "📡 Monitor" button to left menu + page index in `app.py`
- [ ] P6.2: `PrintMonitorPage` layout with plate overview + detail panels
- [ ] P6.3: Miniature plate overview with per-well progress coloring
- [ ] P6.4: L-shaped projection detail view — XY large, ZY right, XZ below — with real-time needle position
- [ ] P6.5: Dashed-line upcoming waypoint rendering (next N waypoints)
- [ ] P6.6: `SyringeDisplayWidget` — fill level bar with µL readout
- [ ] P6.7: Needle info display (gauge, ID, length)
- [ ] P6.8: Print progress panel (well, layer, step, time, ETA)
- [ ] P6.9: Tracking error display (avg distance from planned path)
- [ ] P6.10: Pause/Abort controls mirrored from print setup

---

## 9. Phase 7 — Print Recording & Replay

### File: `SupportClasses/PrintRecorder.py`

Purpose: Automatically log every print execution for historical tracking and replay.

```python
class PrintRecorder:
    """
    Records print execution data to timestamped files.

    Recorded data per print:
    - Workspace config (needle, syringe, inks)
    - Well assignments and plane fit
    - Planned trajectory (all waypoints)
    - Actual positions (timestamped from position poller)
    - Commands sent and timing
    - Tracking errors
    - Final status (completed/aborted/error)

    File format: JSON + CSV pair
    - {timestamp}_meta.json  → workspace, settings, well assignments
    - {timestamp}_data.csv   → t, planned_x, planned_y, planned_z,
                                actual_x, actual_y, actual_z,
                                p1, p2, p3, tracking_error
    """

    def __init__(self, output_dir: str = "print_records"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self._recording = False
        self._current_meta = {}
        self._current_data = []

    def start_recording(self, workspace: WorkspaceConfig,
                        well_setup: dict, job_name: str):
        """Begin recording a new print."""
        ...

    def record_sample(self, t: float, planned: Waypoint,
                      actual_xy: tuple, actual_zp: tuple,
                      tracking_error: float):
        """Record one timestep of data."""
        ...

    def stop_recording(self, status: str):
        """Finalize and save the recording."""
        ...

    @staticmethod
    def load_recording(meta_path: str) -> tuple[dict, pd.DataFrame]:
        """Load a recording for replay or analysis."""
        ...
```

### Integration with PrintHistory:
The existing `PrintHistory.py` tracks summary stats. `PrintRecorder` adds full
trajectory-level data. They complement each other:
- `PrintHistory`: "How many prints have we done? Success rate?"
- `PrintRecorder`: "Show me the actual path of print #47 vs planned."

### Tasks:
- [ ] P7.1: `PrintRecorder` class with start/record/stop
- [ ] P7.2: JSON metadata writer (workspace, settings, assignments)
- [ ] P7.3: CSV data writer (planned vs actual positions over time)
- [ ] P7.4: Integration: auto-start recording when print begins
- [ ] P7.5: Integration: auto-stop recording on complete/abort/error
- [ ] P7.6: Recording browser in Print Monitor (list past prints, load for display)
- [ ] P7.7: Replay visualization — overlay past print data on projections

---

## 10. Phase 8 — Integration & Migration

### Changes to Existing Files:

#### `PrintManager.py` — Modifications:
- [ ] P8.1: Add `CommandType.TRAJECTORY` — execute a full trajectory with motion controller
- [ ] P8.2: Add `TrajectoryExecutor` — runs the motion control loop using `MotionController`
- [ ] P8.3: Modify print loop to use trajectory execution instead of discrete commands
- [ ] P8.4: Integrate `PrintRecorder` auto-start/stop
- [ ] P8.5: Update `PrintJob` to hold `WorkspaceConfig` reference
- [ ] P8.6: Add service sequence executor — waste→wash→buffer→ink automation
- [ ] P8.7: Add `FluidColumn` state tracking — update after each aspirate/dispense
- [ ] P8.8: Add ink change detection — trigger service sequence when pump needs different ink
- [ ] P8.9: Add incremental vs continuous mode handling in the print loop

#### `WellPlate.py` — Modifications:
- [ ] P8.10: Add `well_depth_mm` to `PLATE_DEFINITIONS` (typical well depths per format)
- [ ] P8.11: Add per-well `bottom_z_offset` storage (from plane fit)
- [ ] P8.12: Add `rosette_insert` field to `WellInfo` for attached rosette geometry
- [ ] P8.13: Add 384-well plate definition

#### `XYStage.py` — Modifications (JSON-Based Controller Protocol Mapping):

Instead of hardcoding ProScan II/III commands in Python, controller protocols
are defined in **JSON map files** — one per controller type. The software uses
a set of abstract command names (`position_query`, `move_absolute`, etc.) and
the JSON file maps each to the controller-specific command syntax. This makes
it trivial to add support for future controllers (e.g., ASI MS-2000, Zaber)
by simply adding a new JSON file.

**File location:** `config/controllers/`

**Example: `config/controllers/proscan_iii.json`**
```json
{
    "controller_name": "Prior ProScan III",
    "manufacturer": "Prior Scientific",
    "protocol_version": "III",
    "communication": {
        "line_terminator_tx": "\\r\\n",
        "line_terminator_rx": "\\r\\n",
        "default_baud_rate": 38400,
        "byte_size": 8,
        "stop_bits": 1,
        "timeout_s": 1.0,
        "encoding": "ascii"
    },
    "detection": {
        "wake_command": "STAGE",
        "wake_delay_ms": 100,
        "firmware_query": "V",
        "identify_tokens": ["E", "R", "ProScan"],
        "notes": "Send STAGE first to wake, then V to get firmware string"
    },
    "commands": {
        "position_query":       {"cmd": "P",              "response": "{x},{y},{z}"},
        "move_absolute":        {"cmd": "G {x},{y}",      "response": "R"},
        "move_relative":        {"cmd": "GR {dx},{dy}",   "response": "R"},
        "set_velocity":         {"cmd": "VS,{vx},{vy}",   "response": "R"},
        "set_max_speed":        {"cmd": "SMS,{speed}",    "response": "R"},
        "set_acceleration":     {"cmd": "SAS,{accel}",    "response": "R"},
        "set_jerk":             {"cmd": "SCS,{jerk}",     "response": "R"},
        "set_home":             {"cmd": "Z",              "response": "R"},
        "stop":                 {"cmd": "I",              "response": "R"},
        "firmware_version":     {"cmd": "V",              "response": "{version}"},
        "stage_type":           {"cmd": "STAGE",          "response": "{type}"}
    },
    "parameters": {
        "speed_range": [1, 100],
        "acceleration_range": [1, 100],
        "jerk_range": [1, 100],
        "position_units": "microsteps",
        "position_range_x": [-100000, 100000],
        "position_range_y": [-100000, 100000]
    }
}
```

**Example: `config/controllers/proscan_ii.json`**
```json
{
    "controller_name": "Prior ProScan II",
    "manufacturer": "Prior Scientific",
    "protocol_version": "II",
    "communication": {
        "line_terminator_tx": "\\r",
        "line_terminator_rx": "\\r",
        "default_baud_rate": 38400,
        "byte_size": 8,
        "stop_bits": 1,
        "timeout_s": 1.0,
        "encoding": "ascii"
    },
    "detection": {
        "wake_command": null,
        "wake_delay_ms": 0,
        "firmware_query": "V",
        "identify_tokens": ["ProScan", "II"],
        "notes": "No STAGE command; go straight to firmware query with CR only"
    },
    "commands": {
        "position_query":       {"cmd": "P",              "response": "{x},{y},{z}"},
        "move_absolute":        {"cmd": "G,{x},{y}",      "response": "R"},
        "move_relative":        {"cmd": "GR,{dx},{dy}",   "response": "R"},
        "set_velocity":         {"cmd": "VS,{vx},{vy}",   "response": "R"},
        "set_max_speed":        {"cmd": "SMS,{speed}",    "response": "R"},
        "set_acceleration":     {"cmd": "SAS,{accel}",    "response": "R"},
        "set_jerk":             null,
        "set_home":             {"cmd": "Z",              "response": "R"},
        "stop":                 {"cmd": "I",              "response": "R"},
        "firmware_version":     {"cmd": "V",              "response": "{version}"},
        "stage_type":           null
    },
    "parameters": {
        "speed_range": [1, 100],
        "acceleration_range": [1, 100],
        "jerk_range": null,
        "position_units": "microsteps",
        "position_range_x": [-100000, 100000],
        "position_range_y": [-100000, 100000]
    }
}
```

**Python loader:**
```python
class ControllerProtocol:
    """
    Loads a controller JSON map and provides command formatting.

    Usage:
        proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
        cmd = proto.format_command("move_absolute", x=1000, y=2000)
        # → "G 1000,2000"
        terminator = proto.tx_terminator  # → b"\r\n"
    """

    def __init__(self, config: dict):
        self._config = config
        self._commands = config["commands"]
        comm = config["communication"]
        self._tx_term = comm["line_terminator_tx"].encode().decode(
            "unicode_escape").encode("ascii")
        self._rx_term = comm["line_terminator_rx"].encode().decode(
            "unicode_escape").encode("ascii")

    @classmethod
    def load(cls, filepath: str) -> "ControllerProtocol":
        with open(filepath) as f:
            return cls(json.load(f))

    @property
    def tx_terminator(self) -> bytes:
        return self._tx_term

    def format_command(self, command_name: str, **kwargs) -> str | None:
        """Format a command with parameters. Returns None if unsupported."""
        entry = self._commands.get(command_name)
        if entry is None:
            return None  # Command not supported by this controller
        return entry["cmd"].format(**kwargs)

    def has_command(self, command_name: str) -> bool:
        return self._commands.get(command_name) is not None

    def get_detection_info(self) -> dict:
        return self._config["detection"]

    def get_parameter(self, name: str):
        return self._config["parameters"].get(name)
```

The `XYStageManager` loads the appropriate JSON on construction:
```python
class XYStageManager:
    def __init__(self, simulate=False, settings=None,
                 controller_json="config/controllers/proscan_iii.json"):
        self._protocol = ControllerProtocol.load(controller_json)
        ...

    def send_command(self, command: str) -> str | None:
        encoded = command.encode("ascii") + self._protocol.tx_terminator
        self.spo.write(encoded)

    def move_stage_to_position(self, x, y, fast=False):
        cmd = self._protocol.format_command("move_absolute", x=int(x), y=int(y))
        if cmd:
            self.send_command(cmd)

    def set_jerk(self, jerk: int):
        cmd = self._protocol.format_command("set_jerk", jerk=jerk)
        if cmd is None:
            logger.debug("set_jerk not supported by this controller")
            return
        self.send_command(cmd)
```

Auto-detection tries each JSON file in `config/controllers/`, sends the
detection sequence, and selects the first one that matches.

- [ ] P8.14: `ControllerProtocol` class — JSON loader + command formatter
- [ ] P8.15: Create `config/controllers/proscan_iii.json` command map
- [ ] P8.16: Create `config/controllers/proscan_ii.json` command map
- [ ] P8.17: Refactor `XYStageManager.__init__()` to accept + load controller JSON
- [ ] P8.18: Refactor `send_command()` to use `protocol.tx_terminator`
- [ ] P8.19: Refactor all movement/query methods to use `protocol.format_command()`
- [ ] P8.20: Handle `None` returns for unsupported commands (graceful fallback)
- [ ] P8.21: Auto-detect controller by iterating JSON files and testing detection sequences
- [ ] P8.22: Add controller selector to Settings page (dropdown of JSON files + Auto-detect)
- [ ] P8.23: Store selected controller JSON path in `settings.json`

#### `StageController.py` — Modifications:
- [ ] P8.24: Add `send_velocity_xy(vx, vy)` for continuous motion control
- [ ] P8.25: Add `get_position_with_timestamp()` for Kalman filter input
- [ ] P8.26: Add `test_command_rate()` — safe small-movement test method
- [ ] P8.27: Pass controller JSON path through to `XYStageManager`

#### `SafetyLimits.py` — Modifications:
- [ ] P8.28: Add `max_flow_rate_uL_s` per pump (computed from FlowPhysics)
- [ ] P8.29: Add `clamp_flow_rate()` method

#### `Settings.py` — Modifications:
- [ ] P8.30: Add sections: `workspace`, `ink_library`, `rosette_library`, `well_setup`, `motion_controller`
- [ ] P8.31: Add stage rate test results storage
- [ ] P8.32: Add fluid column state persistence (save/restore between sessions)
- [ ] P8.33: Add `controller_json` setting path + protocol auto-detect result

#### `app.py` — Modifications:
- [ ] P8.34: Add Print Monitor page (6th page) to navigation
- [ ] P8.35: Pass `WorkspaceConfig` between pages via shared state
- [ ] P8.36: Update `_create_pages()` to instantiate new page types

---

## 11. Dependency Summary

### Existing Dependencies (no changes):
- PySide6, pygame, pyserial

### New Dependencies:
| Package | Purpose | Install |
|---------|---------|---------|
| `numpy` | Array math, Kalman filter, trajectory interpolation | `pip install numpy` |
| `scipy` | Spline interpolation (`scipy.interpolate`), plane fitting (`scipy.linalg`) | `pip install scipy` |
| `pandas` | CSV trajectory import, recording data management | `pip install pandas` |

All three are standard scientific Python packages and likely already installed.
No exotic dependencies required.

---

## 12. Implementation Order & Session Breakdown

### Session A — Foundation (Physical Models + Flow Physics)
**Files:** `PhysicalModels.py`, `FlowPhysics.py`, `config/hardware/needles.json`, `config/hardware/syringes.json`
**Tasks:** P1.1–P1.10, P4.1–P4.7
**Estimated size:** ~900 lines Python + JSON config files
**Why first:** Everything else depends on these data structures. Includes
`FluidColumn`, `RosetteInsert`, `PrintingMode`, JSON-loaded needle/syringe
catalogs, and all the flow/pressure safety calculations.

### Session B — Geometry Engine
**Files:** `GeometryEngine.py`
**Tasks:** P2.1–P2.9
**Estimated size:** ~800 lines
**Why second:** Print objects needed before GUI tabs can preview them.

### Session C — Tab 1 (Workspace) GUI
**Files:** `print_workspace.py`
**Tasks:** P5.1–P5.9
**Estimated size:** ~600 lines
**Why third:** First visible GUI. Syringe catalog selector, pump loadout
with fluid column bars, ink library, rosette library, printing mode selector,
compatibility report. Can test immediately with simulators.

### Session D — Tab 2 (Print Objects) GUI + Projection Canvas
**Files:** `print_objects.py`, `projection_canvas.py`
**Tasks:** P5.10–P5.18
**Estimated size:** ~900 lines
**Why fourth:** Object designer needs workspace config from Tab 1.
Triple-projection canvas (XY large, ZY right, XZ below) is reused by Tab 3 and Monitor.

### Session E — Trajectory Planner + CSV Import
**Files:** `TrajectoryPlanner.py`
**Tasks:** P3.1–P3.5
**Estimated size:** ~500 lines
**Why fifth:** Connects geometry engine to executable paths.

### Session F — Motion Controllers (Kalman + PID)
**Files:** `MotionController.py`
**Tasks:** P3.6–P3.10
**Estimated size:** ~700 lines
**Why sixth:** Core algorithm work; needs trajectory planner outputs.

### Session G — Tab 3 (Well Setup) — Well Plate View + Multi-Select
**Files:** `print_well_setup.py`, `WellSetup.py`, `well_plate_view.py`
**Tasks:** P5.19–P5.35
**Estimated size:** ~1400 lines
**Why seventh:** Largest GUI component. `QGraphicsView`-based interactive
plate with rubber-band/shift/ctrl multi-select, right-click context menus,
rosette sub-well editor, well role behaviors (wash/waste/buffer/ink/sort),
service sequence configuration, plane fitting, assignment summary table.
**May split into G1 (plate view + multi-select) and G2 (behaviors + workflows).**

### Session H — Print Monitor Page
**Files:** `print_monitor.py`, `syringe_display.py`, `trajectory_view.py`
**Tasks:** P6.1–P6.10
**Estimated size:** ~900 lines
**Includes:** Fluid column syringe visualization, real-time L-shaped projection view,
plate progress overview, upcoming waypoint dashed lines, tracking error display.

### Session I — Recording + Integration + Migration
**Files:** `PrintRecorder.py` + modifications to existing files + controller JSON maps
**Tasks:** P7.1–P7.7, P8.1–P8.36
**Estimated size:** ~900 lines of new code + ~500 lines of modifications + JSON config files
**Includes:** Auto-record prints, modify `PrintManager` for trajectory execution +
service sequences + fluid column tracking, JSON-based controller protocol mapping in
`XYStage.py` (proscan_ii.json, proscan_iii.json), auto-detect, update `StageController`
with velocity API, wire up all pages in `app.py`, persist all new state.

### Total Estimated New Code: ~8,500 lines across 16 new files (12 Python + 4 JSON) + 7 modified files

### Dependency Chain:
```
Session A (PhysicalModels + FlowPhysics + hardware JSON)
    ├── Session B (GeometryEngine) ← needs NeedleSpec, InkSpec
    │       ├── Session D (Tab 2 GUI) ← needs PrintObject
    │       └── Session E (TrajectoryPlanner) ← needs trajectories
    │               └── Session F (MotionController) ← needs waypoints
    ├── Session C (Tab 1 GUI) ← needs all PhysicalModels + JSON catalogs
    ├── Session G (Tab 3 GUI) ← needs RosetteInsert, WellRole, behaviors
    ├── Session H (Monitor GUI) ← needs FluidColumn for syringe display
    └── Session I (Integration) ← needs everything above
        ├── Controller JSON maps (proscan_ii.json, proscan_iii.json)
        ├── ControllerProtocol loader + auto-detect (XYStage.py)
        ├── Service sequence executor (PrintManager.py)
        └── Settings persistence for all new models
```

---

## Appendix A — Syringe Reference Data

Hamilton 1700 Series (all 30mm stroke, Half-Height, UHMWPE plunger):

| Volume (µL) | PN Base | Barrel ID (mm) | µL per mm | mm per µL |
|-------------|---------|-----------------|-----------|-----------|
| 25          | 1702    | 1.030           | 0.833     | 1.200     |
| 50          | 1705    | 1.457           | 1.667     | 0.600     |
| 100         | 1710    | 2.060           | 3.333     | 0.300     |
| 250         | 1725    | 3.256           | 8.333     | 0.120     |
| 500         | 1750    | 4.606           | 16.667    | 0.060     |
| 1000        | 1001    | 6.513           | 33.333    | 0.030     |

*Barrel ID computed from: ID = sqrt(4 × V / (π × L)) where V = volume, L = 30mm*

## Appendix B — Needle Gauge Reference

| Gauge | OD (µm) | ID (µm) | Wall (µm) |
|-------|---------|---------|-----------|
| 16    | 1651    | 1194    | 229       |
| 17    | 1473    | 1067    | 203       |
| 18    | 1270    | 838     | 216       |
| 19    | 1067    | 686     | 191       |
| 20    | 908     | 603     | 152       |
| 21    | 819     | 514     | 152       |
| 22    | 718     | 413     | 152       |
| 23    | 641     | 337     | 152       |
| 25    | 515     | 260     | 127       |
| 26    | 464     | 260     | 102       |
| 27    | 413     | 210     | 102       |
| 28    | 362     | 184     | 89        |
| 30    | 312     | 159     | 76        |
| 32    | 235     | 108     | 64        |
