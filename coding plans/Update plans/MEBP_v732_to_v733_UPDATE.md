# MEBP v7.3.2 → v7.3.3 — Mode-Based Navigation & Pick-and-Place

**Objective:** Restructure the application into mode-based navigation (Printing mode groups existing print pages as sub-pages with right-side icon nav) and introduce a new Pick-and-Place mode for target identification, image stitching, and automated liquid handling operations (spheroid pickup, trypsin cell extraction, fluorescent tagging).

**Branch:** `Version-7.3.3`
**Base:** `Version-7.3.2`
**Status:** Complete (Features 1-10)
**Date:** 2026-03-13

---

## Architecture Overview

### Current Navigation (v7.3.2)

```
Left sidebar (60px, 9 buttons):
  0: 🔧 Hardware Setup
  1: 📊 Dashboard
  2: 🕹️ Jog Control
  3: 📐 Calibration
  4: 🖨️ Print Setup
  5: 📈 Print Monitor
  6: 📋 Print Results
  7: 🧰 Helper Functions
  8: ⚙️ Settings
```

### New Navigation (v7.3.3)

```
Left sidebar (60px, mode/page buttons):
  0: 🔧 Hardware Setup
  1: 📊 Dashboard
  2: 🕹️ Jog Control
  3: 📐 Calibration
  4: 🖨️ Printing        ← MODE (opens with right-side sub-nav)
  5: 🔬 Pick & Place     ← MODE (opens with right-side sub-nav)
  6: ⚙️ Settings

When a MODE page is active, a right-side icon column appears:

Printing mode (right sidebar):
  ┌──┐
  │🖨️│ Print Setup
  │📈│ Print Monitor
  │📋│ Print Results
  │🧰│ Helpers
  └──┘

Pick & Place mode (right sidebar):
  ┌──┐
  │⚙️│ Operation Setup (select mode + configure)
  │🎯│ Target Selection (camera feed overlays + marking → auto-queue)
  │▶️│ Execution (run operations, context panel shows queue)
  └──┘
```

### Key Design Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | Mode pages are container widgets with internal QStackedWidget + right sidebar | Keeps existing sub-page classes unchanged; only wrapping changes |
| 2 | Right sidebar is 48px column of 40x40 icon buttons | Compact, always visible when in a mode, doesn't compete with left sidebar |
| 3 | Existing print pages (4-7) become children of PrintingModePage | Minimal code changes to existing pages — they keep their interfaces |
| 4 | Pick & Place uses safe_travel_to() for ALL inter-well moves | Mandatory Safe Z protocol: raise Z → wait → XY → wait → lower Z |
| 5 | Image stitcher uses stage coordinates for pixel-perfect alignment | Camera frame position = stage XY at capture time; no feature matching needed |
| 6 | Same-well moves retract Z by configurable `intra_well_retract_mm` (default 1mm) | Still waits for Z confirmation before XY move |
| 7 | Pick & Place operations are queue-based (OperationQueue → OperationExecutor) | Allows batch definition, pause/resume, and operation-level retry |

---

## Feature Summary

| # | Feature | Scope | Status |
|---|---------|-------|--------|
| 1 | Mode-based navigation refactor | app.py, new ModePage base class | ✓ Done |
| 2 | Printing mode container | New gui/pages/printing_mode.py wrapping pages 4-7 | ✓ Done |
| 3 | Pick & Place backend | New SupportClasses/PickAndPlaceManager.py | ✓ Done |
| 4 | Image stitcher backend | New SupportClasses/ImageStitcher.py | ✓ Done |
| 5 | Pick & Place target selection page | New gui/pages/pp_target_selection.py | ✓ Done |
| 6 | Pick & Place operation queue page | SUPERSEDED by pp_operation_setup.py (config-first) | ✓ Replaced |
| 7 | Pick & Place execution page | New gui/pages/pp_execution.py | ✓ Done |
| 8 | Pick & Place mode container | New gui/pages/pick_place_mode.py | ✓ Done |
| 9 | P&P restructure: config-first, target overlays, auto-queue | pp_operation_setup.py, target_overlay_camera_view.py, camera fix | ✓ Done |
| 10 | Camera µm/px calibration + relaxed needle detection | VisionDetector, pixel_calibration_dialog, calibration page UI | ✓ Done |

---

## 1. Mode-Based Navigation Refactor

### Objective
Restructure `app.py` navigation from a flat list of 9 pages to a mode-aware system where certain sidebar entries (Printing, Pick & Place) are **mode pages** that contain sub-pages with a right-side icon column.

### Current State
- `app.py` uses `QStackedWidget` (`_page_stack`) with 9 pages indexed 0-8
- Left sidebar has 9 buttons mapped to page indices via `_on_menu_click()`
- Context panel (left extra box) per page via `_context_stack`
- Pages are flat — no nesting or sub-navigation

### Implementation Steps

- [x] **1.1** Create `gui/pages/mode_page.py` — abstract `ModePage(QWidget)` base class:
  - Contains: `QHBoxLayout` → [main content `QStackedWidget`] + [right sidebar `QVBoxLayout`]
  - Right sidebar: 48px wide, column of 40x40 `QPushButton` icons (Catppuccin surface0 bg)
  - Methods: `add_sub_page(icon, title, widget)`, `switch_to(index)`, `get_active_sub_page()`
  - Signal: `sub_page_changed(int)` — emitted when sub-page switches
  - Each sub-page button highlights when active (mauve border, like left sidebar)
  - Delegates `on_status_update()` to active sub-page
  - Delegates `get_context_widget()` to active sub-page
  - Delegates `set_hardware_config()` to ALL sub-pages

- [x] **1.2** Refactor `app.py::_create_pages()`:
  - Pages list becomes: [HardwareSetup(0), Dashboard(1), Jog(2), Calibration(3), PrintingMode(4), PickPlaceMode(5), Settings(6)]
  - `PrintingModePage` wraps: PrintSetupPage, PrintMonitorPage, PrintResultsPage, HelperFunctionsPage
  - `PickPlaceModePage` wraps: PPTargetSelectionPage, PPOperationQueuePage, PPExecutionPage
  - Total main pages: 7 (down from 9)

- [x] **1.3** Update `app.py` navigation:
  - `_on_menu_click()` button map updated for new indices
  - `_navigate_to()` updated: when switching to a mode page, shows right sidebar; when switching to a non-mode page, hides any visible right sidebar
  - Context panel routing: mode pages delegate to their active sub-page's context widget
  - `_context_stack` entries for mode pages update dynamically when sub-page changes

- [x] **1.4** Update `app.py` wiring:
  - `_wire_job_pipeline()` — now accesses print pages through `PrintingModePage.sub_pages[0]` (setup) and `[1]` (monitor)
  - `_wire_print_manager_to_monitor()` — same indirect access
  - `_send_job_to_monitor()` — switches PrintingMode to monitor sub-page, then navigates to PrintingMode
  - `_on_print_completed_v726()` — accesses results through `PrintingModePage.sub_pages[2]`
  - `_on_helper_print_created()` — accesses print objects through PrintingMode
  - Page gating: mode pages gated as before (hardware must be valid)
  - Calibration data wiring → jog page remains unchanged (still page index 2 → 2)

- [x] **1.5** Update page gating indices in `_update_page_gating()`

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/mode_page.py` | **NEW** — ModePage base class with right sidebar sub-navigation |
| `gui/app.py` | Refactored page creation, navigation, wiring for mode-based system |

---

## 2. Printing Mode Container

### Objective
Wrap existing Print Setup, Print Monitor, Print Results, and Helper Functions pages into a `PrintingModePage` container that provides right-side sub-page navigation.

### Implementation Steps

- [x] **2.1** Create `gui/pages/printing_mode.py`:
  - `PrintingModePage(ModePage)` — instantiates and adds 4 sub-pages
  - Sub-page icons: 🖨️ Setup, 📈 Monitor, 📋 Results, 🧰 Helpers
  - Default sub-page: Print Setup (index 0)
  - Exposes convenience accessors: `setup_page`, `monitor_page`, `results_page`, `helpers_page`
  - Forwards `set_hardware_config()` to all sub-pages
  - Forwards `on_status_update()` to active sub-page

- [x] **2.2** Expose sub-page access for wiring:
  - `printing_mode.setup_page` → PrintSetupPage instance
  - `printing_mode.monitor_page` → PrintMonitorPage instance
  - `printing_mode.results_page` → PrintResultsPage instance
  - `printing_mode.helpers_page` → HelperFunctionsPage instance

- [x] **2.3** Wire job pipeline through printing mode:
  - `job_ready` signal → `_send_job_to_monitor()` auto-switches to monitor sub-page
  - Monitor execution signals pass through unchanged
  - Results notification works via `printing_mode.results_page`

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/printing_mode.py` | **NEW** — Printing mode container with 4 sub-pages |

---

## 3. Pick & Place Backend — PickAndPlaceManager

### Objective
Backend engine for all pick-and-place operations. Handles operation definitions, queue management, and execution with Safe Z protocol enforcement.

### Data Model

```python
@dataclass
class PickPlaceTarget:
    """A target identified in the stitched image."""
    target_id: str                    # Unique ID (e.g., "T001")
    x_um: float                       # Stage X in µm
    y_um: float                       # Stage Y in µm
    well_name: str                    # Which well this target is in
    pixel_x: int                      # Pixel X in stitched image
    pixel_y: int                      # Pixel Y in stitched image
    size_um: float = 0.0              # Estimated target diameter in µm
    label: str = ""                   # User label
    selected: bool = True             # Whether included in operations

class OperationType(Enum):
    SPHEROID_PICKUP = "spheroid_pickup"
    TRYPSIN_CELL_PICKUP = "trypsin_cell_pickup"
    FLUORESCENT_TAGGING = "fluorescent_tagging"

@dataclass
class SpheroidPickupConfig:
    """Config for spheroid pickup mode."""
    spheroid_diameter_um: float = 200.0
    safety_factor: float = 1.5        # Volume multiplier
    pickup_bore: str = "P1"           # Which pump bore to use
    pickup_speed_uL_s: float = 1.0    # Aspiration speed
    release_speed_uL_s: float = 1.0   # Dispensing speed
    # Computed: volume = (4/3)π(d/2)³ * safety_factor (converted to µL)

@dataclass
class TrypsinPickupConfig:
    """Config for trypsin cell pickup mode."""
    trypsin_volume_uL: float = 5.0
    dwell_time_s: float = 120.0       # Wait time after trypsin addition
    single_bore: bool = True          # True = 1 bore, False = 2 bores
    trypsin_bore: str = "P1"          # Bore for trypsin delivery
    extraction_bore: str = "P1"       # Bore for cell extraction (same if single)
    extraction_volume_uL: float = 6.0 # Volume to extract (trypsin + cells)
    push_speed_uL_s: float = 2.0
    pull_speed_uL_s: float = 1.0

@dataclass
class FluorescentTaggingConfig:
    """Config for fluorescent tagging mode."""
    num_bores: int = 1                 # 1, 2, or 3 bores
    dye_configs: list = field(default_factory=list)  # Per-bore DyeConfig
    dwell_time_s: float = 300.0        # Incubation time
    # If a bore is designated as waste collector:
    waste_bore: str | None = None      # e.g., "P3" — skip going to waste wells
    use_waste_bore_mode: bool = False   # Use bore as local waste collector

@dataclass
class DyeConfig:
    """Config for a single fluorescent dye bore."""
    bore: str = "P1"                   # Pump bore
    dye_well: str = ""                 # Well containing this dye
    dye_name: str = ""                 # e.g., "DAPI", "GFP", "mCherry"
    volume_uL: float = 2.0            # Volume per target
    color: str = "#89b4fa"             # Display color

@dataclass
class PickPlaceOperation:
    """A single pick-and-place operation in the queue."""
    op_id: str                         # Unique ID
    op_type: OperationType
    source_target: PickPlaceTarget     # Where to pick from
    dest_target: PickPlaceTarget | None  # Where to place (None for in-place ops)
    config: SpheroidPickupConfig | TrypsinPickupConfig | FluorescentTaggingConfig
    status: str = "pending"            # pending, running, completed, failed, skipped
    error_msg: str = ""

class OperationQueue:
    """Ordered queue of PickPlaceOperations."""
    operations: list[PickPlaceOperation]
    def add(op), remove(op_id), reorder(op_id, new_index)
    def get_pending() -> list
    def mark_running(op_id), mark_completed(op_id), mark_failed(op_id, msg)
```

### Execution Engine

```python
class PickPlaceExecutor:
    """Executes operations from the queue using StageController."""

    def __init__(self, controller: StageController, hw_config: HardwareConfig):
        self.controller = controller
        self.hw_config = hw_config
        self._safe_z_mm: float = 5.0
        self._intra_well_retract_mm: float = 1.0
        self._current_well: str = ""

    def execute_queue(self, queue: OperationQueue,
                      pause_event, on_progress, on_op_complete):
        """Execute all pending operations in order."""
        # For each op:
        #   1. Navigate to source (safe Z if different well, small retract if same)
        #   2. Perform pickup/delivery
        #   3. Navigate to dest (safe Z always — different well)
        #   4. Perform release/extraction
        #   5. Service sequence if needed (waste/wash/buffer)

    def _safe_move_to(self, target: PickPlaceTarget):
        """Move to target with appropriate Z protocol."""
        if target.well_name != self._current_well:
            # INTER-WELL: full safe Z protocol
            self.controller.safe_travel_to(
                target_x_um=target.x_um,
                target_y_um=target.y_um,
                safe_z_mm=self._safe_z_mm,
                target_z_mm=self._operating_z_mm,
            )
        else:
            # INTRA-WELL: small retract + wait + XY + wait + lower
            self.controller.move_z_relative(-self._intra_well_retract_mm)
            self.controller.wait_for_z_arrival(current_z - intra_retract)
            self.controller.move_xy_absolute(target.x_um, target.y_um)
            self.controller.wait_for_xy_arrival(target.x_um/1000, target.y_um/1000)
            self.controller.move_z_relative(self._intra_well_retract_mm)
            self.controller.wait_for_z_arrival(operating_z)
        self._current_well = target.well_name

    def _execute_spheroid_pickup(self, op: PickPlaceOperation): ...
    def _execute_trypsin_pickup(self, op: PickPlaceOperation): ...
    def _execute_fluorescent_tagging(self, op: PickPlaceOperation): ...
```

### Implementation Steps

- [x] **3.1** Create `SupportClasses/PickAndPlaceManager.py` with data model classes
- [x] **3.2** Implement `OperationQueue` with add/remove/reorder/status tracking
- [x] **3.3** Implement `PickPlaceExecutor` with safe Z protocol enforcement
- [x] **3.4** Implement `_execute_spheroid_pickup()`:
  - Calculate volume from spheroid diameter: `V = (4/3)π(d/2)³ * safety_factor` → µL
  - Move to source target (safe Z protocol)
  - Lower needle to operating Z
  - Aspirate volume via `controller.move_pump_uL(bore, -volume)`
  - Raise, safe travel to dest
  - Dispense same volume via `controller.move_pump_uL(bore, +volume)`

- [x] **3.5** Implement `_execute_trypsin_pickup()`:
  - **Single bore mode:**
    1. Load trypsin from trypsin well → safe travel to target
    2. Dispense trypsin into target well
    3. Wait `dwell_time_s` (timer with progress callback)
    4. Aspirate `extraction_volume_uL` (trypsin + cells)
    5. Safe travel to dest → dispense
  - **Dual bore mode:**
    1. Load trypsin on `trypsin_bore` → safe travel to target
    2. Dispense trypsin into target well
    3. Wait `dwell_time_s`
    4. Aspirate on `extraction_bore` (separate bore, positioned at target)
    5. Safe travel to dest → dispense from extraction bore

- [x] **3.6** Implement `_execute_fluorescent_tagging()`:
  - **Single bore mode (1 bore):**
    1. Pickup dye from dye well → safe travel to target
    2. Deposit dye → wait `dwell_time_s`
    3. Aspirate dye from target
    4. Safe travel to waste → dispense waste
    5. Safe travel to buffer → wash → next cycle
  - **Multi bore mode (2-3 bores, all dyes):**
    1. Load each bore from its dye well (safe travel between)
    2. For each target: deposit each bore's dye → dwell → aspirate each
    3. Waste/wash/buffer between targets
  - **Waste bore mode (1 bore = waste collector):**
    1. Load dye bores from dye wells
    2. Deposit dye at target → dwell
    3. Aspirate spent dye using waste bore (skip traveling to waste well)
    4. Continue to next target until waste bore is full
    5. Then safe travel to waste to empty waste bore + refill dye bores

- [x] **3.7** Add pause/resume/abort support (threading.Event pattern, same as PrintManager)
- [x] **3.8** Add operation-level callbacks: `on_op_started(op)`, `on_op_completed(op)`, `on_op_failed(op, msg)`

### Files Modified
| File | Change |
|------|--------|
| `SupportClasses/PickAndPlaceManager.py` | **NEW** — Full pick & place data model + execution engine |

---

## 4. Image Stitcher Backend

### Objective
Build large composite images of wells by stitching together camera frames captured at known stage positions. The key insight: **stage coordinates ARE the alignment** — no feature matching needed.

### Algorithm

```
For each captured frame:
  1. Record stage position (x_um, y_um) at capture time
  2. Convert frame pixels to µm using camera's micron_per_pixel
  3. Place frame in composite image at correct position
  4. Handle overlapping regions (newest frame wins, or blending)

Composite coordinate system:
  - Origin: top-left of bounding box of all captured frames
  - Scale: micron_per_pixel (same as camera)
  - Each pixel represents exactly 1 camera pixel worth of stage space
```

### Implementation Steps

- [x] **4.1** Create `SupportClasses/ImageStitcher.py`:
  ```python
  class StitchedImage:
      """A composite image built from camera frames at known positions."""

      def __init__(self, micron_per_pixel: float):
          self.um_per_px = micron_per_pixel
          self._tiles: list[Tile] = []     # (frame, stage_x_um, stage_y_um)
          self._composite: np.ndarray | None = None
          self._origin_x_um: float = 0     # Stage X of composite top-left
          self._origin_y_um: float = 0     # Stage Y of composite top-left
          self._dirty: bool = True

      def add_tile(self, frame: np.ndarray, stage_x_um: float, stage_y_um: float):
          """Add a camera frame captured at the given stage position."""
          # stage_x_um, stage_y_um = center of the frame in stage coords

      def get_composite(self) -> np.ndarray:
          """Return the stitched composite image (lazy rebuild if dirty)."""

      def stage_to_pixel(self, x_um: float, y_um: float) -> tuple[int, int]:
          """Convert stage coordinates to pixel coordinates in composite."""
          px = int((x_um - self._origin_x_um) / self.um_per_px)
          py = int((y_um - self._origin_y_um) / self.um_per_px)
          return (px, py)

      def pixel_to_stage(self, px: int, py: int) -> tuple[float, float]:
          """Convert composite pixel coordinates to stage coordinates."""
          x_um = self._origin_x_um + px * self.um_per_px
          y_um = self._origin_y_um + py * self.um_per_px
          return (x_um, y_um)

      def clear(self):
          """Reset the stitcher."""
  ```

- [x] **4.2** Implement `_rebuild_composite()`:
  - Compute bounding box of all tiles (min/max stage coords + frame sizes)
  - Allocate composite array
  - For each tile: compute pixel offset from origin, paste into composite
  - Overlap strategy: latest tile overwrites (simple, deterministic)

- [x] **4.3** Implement auto-scan pattern generator:
  ```python
  def generate_scan_pattern(
      well_center_x_um, well_center_y_um,
      well_diameter_um, frame_width_px, frame_height_px,
      um_per_px, overlap_fraction=0.2
  ) -> list[tuple[float, float]]:
      """Generate raster scan positions to cover a well."""
      # Returns list of (stage_x_um, stage_y_um) positions
      # Serpentine pattern for efficiency
  ```

- [x] **4.4** Implement manual capture mode: `add_current_frame()` — captures frame + records current stage position
- [x] **4.5** Thread-safe: stitcher runs on background thread, emits `composite_updated` signal

### Files Modified
| File | Change |
|------|--------|
| `SupportClasses/ImageStitcher.py` | **NEW** — Stage-coordinate-based image stitcher |

---

## 5. Pick & Place Target Selection Page

### Objective
GUI page where users view the live camera feed / stitched image, click to mark targets, and manage the target list. This is the primary interaction surface for defining what to pick and where to place.

### Layout

```
┌─────────────────────────────────────────────────────────────┐
│  ┌───────────────────────────────┬────────────────────────┐ │
│  │                               │  Target List           │ │
│  │   Camera / Stitched Image     │  ┌──────────────────┐  │ │
│  │   View (interactive)          │  │ T001 (120µm) ✓   │  │ │
│  │                               │  │ T002 (95µm)  ✓   │  │ │
│  │   [click to mark targets]     │  │ T003 (110µm) ✓   │  │ │
│  │   [scroll to zoom]            │  │ ...               │  │ │
│  │   [drag to pan]               │  └──────────────────┘  │ │
│  │                               │                        │ │
│  │                               │  [Remove] [Clear All]  │ │
│  └───────────────────────────────┴────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ Scan Controls: [Manual Capture] [Auto-Scan Well ▾]    │  │
│  │ View: ○ Live Camera  ● Stitched  │ Targets: 3 marked  │  │
│  └────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘

Context panel (left):
  - Well selector (which well to scan)
  - Scan settings (overlap %, coverage)
  - Target size filter (min/max µm)
  - Auto-detect settings (threshold, min confidence)
  - Export target list
```

### Implementation Steps

- [x] **5.1** Create `gui/pages/pp_target_selection.py`:
  - `PPTargetSelectionPage(QWidget)` with splitter: image view (left) + target list (right)
  - Image view: custom `QWidget` with `paintEvent` for stitched image + overlays
  - Mouse interaction: click → add target at stage coordinates, right-click → remove
  - Scroll → zoom, middle-drag → pan

- [x] **5.2** Implement `StitchedImageView(QWidget)`:
  - Displays either live camera feed OR stitched composite
  - Draws target markers (colored circles at target locations)
  - Draws camera FOV rectangle when showing stitched view
  - Coordinate transform: widget pixels ↔ stage µm (via stitcher's pixel_to_stage)
  - Signals: `target_added(x_um, y_um)`, `target_removed(target_id)`

- [x] **5.3** Implement target list widget:
  - `QListWidget` with target entries (ID, size, well, coordinates)
  - Checkbox per target (include/exclude from operations)
  - Click to highlight in image view
  - Double-click to navigate stage to target (with safe Z)

- [x] **5.4** Implement scan controls:
  - "Manual Capture" button → captures current frame + stage position → adds to stitcher
  - "Auto-Scan Well" dropdown → select well → generates scan pattern → executes scan
  - Auto-scan: moves stage through pattern, captures at each position, builds composite
  - Progress bar during auto-scan
  - Safe Z between wells, small retract within well during scan

- [x] **5.5** Implement target auto-detection (optional):
  - For spheroids: circle detection on stitched image (similar to WellDetector)
  - User confirms/edits detected targets
  - Configurable min/max size filter

- [x] **5.6** Wire to `StageController` for:
  - Current position display
  - Navigate-to-target (safe travel)
  - Camera frame capture

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/pp_target_selection.py` | **NEW** — Target selection page with stitched image view |

---

## 6. Pick & Place Operation Queue Page

### Objective
GUI page where users define pick-and-place operations from the marked targets, configure operation parameters, and arrange the execution order.

### Layout

```
┌────────────────────────────────────────────────────────────┐
│  Operation Mode: [● Spheroid] [○ Trypsin] [○ Fluorescent] │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Mode-Specific Configuration Panel                   │  │
│  │  (changes based on selected mode)                    │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Operation Queue                                     │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │ 1. Spheroid T001 → Well B3 (pending)           │  │  │
│  │  │ 2. Spheroid T002 → Well B3 (pending)           │  │  │
│  │  │ 3. Trypsin  T003 → Well C1 (pending)           │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  │  [Add from Targets] [Remove] [Move Up] [Move Down]   │  │
│  │  [Clear Queue]                                       │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  [Send to Execution ▶]                               │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

### Implementation Steps

- [x] **6.1** Create `gui/pages/pp_operation_queue.py`:
  - `PPOperationQueuePage(QWidget)` with mode selector, config panel, queue list
  - Radio buttons for operation type selection

- [x] **6.2** Implement mode-specific config panels:
  - **Spheroid:** diameter spinbox, safety factor spinbox, bore selector, speed spinboxes
  - **Trypsin:** volume spinbox, dwell time, single/dual bore toggle, bore selectors, extraction volume
  - **Fluorescent:** bore count (1-3), per-bore dye config (well selector, color, volume), dwell time, waste bore toggle

- [x] **6.3** Implement operation queue widget:
  - QListWidget showing operations with icons (by type) and status indicators
  - Add: select targets from target list → create operations with current mode config
  - Destination well/position picker for each operation
  - Drag-reorder or move up/down buttons

- [x] **6.4** "Send to Execution" → packages OperationQueue and emits signal to execution page

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/pp_operation_queue.py` | **NEW** — Operation queue page with mode configs |

---

## 7. Pick & Place Execution Page

### Objective
GUI page that runs the operation queue with real-time monitoring, similar to Print Monitor but for pick-and-place operations.

### Layout

```
┌────────────────────────────────────────────────────────────┐
│  ┌────────────────────────┬─────────────────────────────┐  │
│  │  Plate Overview        │  Operation Progress         │  │
│  │  (well plate with      │  ┌───────────────────────┐  │  │
│  │   needle position +    │  │ Op 1: Spheroid T001   │  │  │
│  │   target markers)      │  │ Status: ✓ Complete    │  │  │
│  │                        │  │ Op 2: Spheroid T002   │  │  │
│  │                        │  │ Status: ▶ Running...  │  │  │
│  │                        │  │ Op 3: Trypsin T003    │  │  │
│  │                        │  │ Status: ○ Pending     │  │  │
│  │                        │  └───────────────────────┘  │  │
│  └────────────────────────┴─────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Progress: ████████░░░░ 2/3 (67%)  |  ETA: 3:45      │  │
│  │  Current: Moving to target T002... Dwell: 45s remain  │  │
│  │  [▶ Start] [⏸ Pause] [⏹ Abort]                       │  │
│  └────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

### Implementation Steps

- [x] **7.1** Create `gui/pages/pp_execution.py`:
  - `PPExecutionPage(QWidget)` with plate view, operation list, progress bar, controls

- [x] **7.2** Implement operation progress list:
  - Per-operation status: pending (○), running (▶), completed (✓), failed (✗)
  - Running operation shows sub-step detail (e.g., "Dispensing trypsin...", "Dwelling: 45s")
  - Dwell time countdown display

- [x] **7.3** Wire to `PickPlaceExecutor`:
  - Start → launches executor in daemon thread
  - Pause/Resume via threading.Event
  - Abort → sets abort flag
  - Thread-safe signal bridge (same pattern as PrintManager → PrintMonitor)

- [x] **7.4** Plate overview widget showing:
  - All wells with roles (source, dest, waste, wash, buffer, dye)
  - Current needle position (crosshair)
  - Target markers with completion status
  - Active path line showing planned route

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/pp_execution.py` | **NEW** — Execution monitoring page |

---

## 8. Pick & Place Mode Container

### Objective
Wrap the three Pick & Place sub-pages into a `PickPlaceModePage` container, analogous to `PrintingModePage`.

### Implementation Steps

- [x] **8.1** Create `gui/pages/pick_place_mode.py`:
  - `PickPlaceModePage(ModePage)` — adds 3 sub-pages
  - **Restructured sub-page order (v7.3.3 Phase 2):**
    - ⚙️ Operation Setup (index 0) — config-first workflow
    - 🎯 Target Selection (index 1) — camera feed with target overlays
    - ▶️ Execution (index 2) — run queue, context shows queue list
  - Exposes: `setup_page`, `target_page`, `execution_page`

- [x] **8.2** Wire inter-page data flow (restructured):
  - `setup_page.config_changed` → `target_page.set_operation_config()` + `execution_page.set_operation_config()`
  - `target_page.queue_changed` → `execution_page.update_queue()` (continuous, auto-built)
  - Targets auto-create PickPlaceOperations using current config when added

- [x] **8.3** Wire to `app.py`:
  - Hardware config propagation
  - Controller access (StageController, camera)
  - Safe Z config from settings/calibration

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/pick_place_mode.py` | **NEW** — Pick & Place mode container with 3 sub-pages (restructured: Setup → Target → Execution) |

---

## Safe Z Protocol — Critical Requirements

**EVERY movement between wells MUST follow the Safe Z protocol:**

```
1. Retract Z to safe_z_mm
2. WAIT for Z arrival confirmation (wait_for_z_arrival)
3. Move XY to target
4. WAIT for XY arrival confirmation (wait_for_xy_arrival)
5. Lower Z to operating height
6. WAIT for Z arrival confirmation
```

**For same-well movements:**
```
1. Retract Z by intra_well_retract_mm (default 1mm)
2. WAIT for Z arrival confirmation
3. Move XY to new position within well
4. WAIT for XY arrival confirmation
5. Lower Z back to operating height
6. WAIT for Z arrival confirmation
```

**Implementation:** All movements route through `PickPlaceExecutor._safe_move_to()` which enforces this protocol. Direct `move_xy_absolute()` calls without Z retract are NEVER used.

---

## Implementation Order

1. **Feature 1** — Mode-based navigation refactor (foundation for everything)
2. **Feature 2** — Printing mode container (proves the mode system works with existing pages)
3. **Feature 4** — Image stitcher backend (no GUI dependency)
4. **Feature 3** — Pick & Place backend (no GUI dependency)
5. **Feature 5** — Target selection page (needs stitcher + controller)
6. **Feature 6** — Operation queue page (needs backend data model)
7. **Feature 7** — Execution page (needs executor)
8. **Feature 8** — Pick & Place mode container (wraps 5-7)

---

## Testing Notes

- [ ] Verify Printing mode sub-page navigation works (right sidebar icons switch pages)
- [ ] Verify job pipeline still works through PrintingModePage wrapper
- [ ] Verify hardware config propagation reaches all sub-pages
- [ ] Verify context panel switches correctly for mode sub-pages
- [ ] Verify image stitcher produces correct composite from known positions
- [ ] Verify stage_to_pixel / pixel_to_stage round-trips correctly
- [ ] Verify auto-scan pattern covers entire well with specified overlap
- [ ] Verify target click → stage coordinate extraction is accurate
- [ ] Verify spheroid volume calculation is correct
- [ ] Verify Safe Z protocol is followed for ALL inter-well moves
- [ ] Verify same-well retract protocol (1mm up, wait, XY, wait, down)
- [ ] Verify trypsin dual-bore mode uses correct bores
- [ ] Verify fluorescent waste-bore mode skips waste well travel
- [ ] Verify pause/resume/abort work during operations
- [ ] Verify dwell time countdown is accurate
- [ ] Run existing test suite (v7.3.2 tests should still pass)

---

## Issues & Decisions

| # | Issue | Decision | Date |
|---|-------|----------|------|
| 1 | How to handle right sidebar alongside existing context panel? | Right sidebar is INSIDE the mode page widget, not managed by app.py. Context panel (left) still managed by app.py, delegates to active sub-page | 2026-03-13 |
| 2 | Image stitching alignment method? | Pure stage-coordinate-based (no feature matching). Camera um_per_px * frame_size → exact placement. Requires accurate calibration. | 2026-03-13 |
| 3 | Safe Z enforcement — how to prevent bypassing? | PickPlaceExecutor._safe_move_to() is the ONLY way to move. No public method exposes direct XY move without Z check. | 2026-03-13 |
| 4 | Intra-well retract amount? | Configurable, default 1mm. Still follows wait-for-Z-before-XY rule. | 2026-03-13 |
| 5 | Fluorescent waste bore capacity tracking? | Track cumulative volume on waste bore; trigger waste-empty when approaching syringe capacity (from HardwareConfig pump spec) | 2026-03-13 |
| 6 | Camera feed not showing on target selection page when started from calibration | Root cause: CameraFeedView scaled frames to 0×0 when hidden in QStackedWidget. Fixed: store `_last_qimage`, skip render when hidden, re-render in showEvent | 2026-03-13 |
| 7 | Pick & Place page order — operation queue as separate page? | Eliminated separate queue page. Operation type selection is FIRST page (PPOperationSetupPage), targets auto-create operations when added. Queue shared to execution context panel. | 2026-03-13 |
| 8 | Camera µm/px calibration — where to put it? | "Calibrate µm/px" button on hardware page's Live Camera Sources section. Opens modal dialog (PixelCalibrationDialog) using CameraManager + StageController. Stage moved known distance, phase correlation measures pixel displacement. | 2026-03-13 |
| 9 | Needle detection too strict when µm/px is wrong? | Added "Relaxed Detect" mode (±80% tolerance or unconstrained). Detection pauses for interactive accept/reject with +/- radius adjustment. Edge refinement via radial gradient + Kasa circle fit. Accepted needle + known gauge → implied µm/px emitted to hardware page. | 2026-03-13 |
| 10 | Hardware page name/notes font too large | Removed oversized font styles (13pt/12pt), set minWidth=350 for readability | 2026-03-13 |

---

## 9. Pick & Place Restructuring (v7.3.3 Phase 2)

### Objective
Restructure Pick & Place sub-pages so operation type selection comes first, targets are selected over the live camera feed with overlays, and the queue is auto-built and shared with execution.

### Changes

- [x] **9.1** Created `gui/pages/pp_operation_setup.py` (NEW) — Operation type radio buttons (Spheroid/Trypsin/Fluorescent) + mode-specific config stacked widget + `config_changed` signal
- [x] **9.2** Created `gui/widgets/target_overlay_camera_view.py` (NEW) — CameraFeedView subclass that draws target circles at stage coordinates on the live feed with size/ID labels
- [x] **9.3** Modified `gui/pages/pp_target_selection.py` — Replaced CameraFeedView with TargetOverlayCameraView, added `queue_changed` signal and internal `OperationQueue`, `set_operation_config()` slot, auto-creates operations when targets are added
- [x] **9.4** Modified `gui/pages/pp_execution.py` — `get_context_widget()` now returns queue display panel with count + list, `update_queue()` for continuous updates, `set_operation_config()` updates pending ops
- [x] **9.5** Rewritten `gui/pages/pick_place_mode.py` — New sub-page order: Setup(0) → Target Selection(1) → Execution(2), wired config_changed + queue_changed signals
- [x] **9.6** Fixed CameraFeedView hidden-widget rendering — Store `_last_qimage`, skip render when not visible, re-render in showEvent
- [x] **9.7** Target list shows XY coordinates: `"T001  (1234.5, 5678.9)  (200µm)  A1"`

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/pp_operation_setup.py` | **NEW** — Operation type + config first page |
| `gui/widgets/target_overlay_camera_view.py` | **NEW** — CameraFeedView subclass with target overlays |
| `gui/widgets/camera_feed_view.py` | Fixed hidden-widget rendering (store _last_qimage, showEvent re-render) |
| `gui/pages/pp_target_selection.py` | TargetOverlayCameraView, queue auto-building, operation config |
| `gui/pages/pp_execution.py` | Context panel queue display, update_queue, set_operation_config |
| `gui/pages/pick_place_mode.py` | Restructured: Setup → Target Selection → Execution, signal wiring |

---

## 10. Camera µm/px Calibration + Relaxed Needle Detection (v7.3.3 Phase 3)

### Objective
Add empirical camera µm/px calibration via stage-move phase correlation, and relaxed needle detection with interactive edge refinement and accept/reject UI.

### Changes

- [x] **10.1** `SupportClasses/VisionDetector.py` — Added `measure_pixel_displacement()` (cv2.phaseCorrelate with Hanning window)
- [x] **10.2** `SupportClasses/VisionDetector.py` — Added `NeedleDetector._fit_circle_kasa()` (algebraic least-squares circle fit)
- [x] **10.3** `SupportClasses/VisionDetector.py` — Added `NeedleDetector.refine_to_edge()` (radial gradient at 72 angles → Kasa fit, outlier filtering)
- [x] **10.4** `SupportClasses/VisionDetector.py` — Added `NeedleDetector.detect_needle_relaxed()` (±80% tolerance or unconstrained search)
- [x] **10.5** Created `gui/dialogs/__init__.py` + `gui/dialogs/pixel_calibration_dialog.py` (NEW) — Modal wizard: capture → move stage → capture → phase correlate → compute µm/px
- [x] **10.6** `gui/pages/hardware_setup.py` — Added `set_controller()`, `set_calibrated_um_per_px()`, `_on_calibrate_umpx()`, "Calibrate µm/px" button in Live Camera Sources section
- [x] **10.7** `gui/app.py` — Wired StageController to hardware page, connected `um_per_px_calibrated` signal from calibration → hardware page
- [x] **10.8** `gui/widgets/detection_worker.py` — Added `NEEDLE_DETECT_RELAXED` mode + `_detect_needle_relaxed()` method
- [x] **10.9** `gui/widgets/detection_overlay.py` — Added adjustable blue circle support: `set_adjustable_needle()`, `adjust_radius()`, `get_adjustable_circle()`, `clear_adjustable()`
- [x] **10.10** `gui/pages/calibration.py` — Added "Relaxed Detect" button, Accept/Reject/+/- button row, interactive mode with edge refinement, needle-based µm/px computation + `um_per_px_calibrated` signal

### Files Modified
| File | Change |
|------|--------|
| `SupportClasses/VisionDetector.py` | `measure_pixel_displacement()`, `detect_needle_relaxed()`, `refine_to_edge()`, `_fit_circle_kasa()` |
| `gui/dialogs/__init__.py` | **NEW** — Dialogs package |
| `gui/dialogs/pixel_calibration_dialog.py` | **NEW** — Stage-move phase correlation calibration dialog |
| `gui/pages/hardware_setup.py` | `set_controller()`, `set_calibrated_um_per_px()`, calibrate button, wider name/notes fields |
| `gui/app.py` | Wire controller to hardware page, connect um_per_px_calibrated signal |
| `gui/widgets/detection_worker.py` | `NEEDLE_DETECT_RELAXED` mode + handler |
| `gui/widgets/detection_overlay.py` | Adjustable blue circle for interactive needle verification |
| `gui/pages/calibration.py` | `um_per_px_calibrated` signal, relaxed detect toggle, accept/reject/adjust UI, edge refinement |
