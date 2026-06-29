"""
PickAndPlaceManager.py — Backend for pick-and-place operations.

v7.3.3: Data model and execution engine for three operation modes:
  1. Spheroid pickup — volume-based aspiration/dispensing
  2. Trypsin cell pickup — single or dual bore with dwell time
  3. Fluorescent tagging — multi-bore dye application with wash cycles

All inter-well movements enforce the Safe Z protocol:
  raise Z → wait → XY → wait → lower Z

All intra-well movements use a shorter retract:
  retract 1mm → wait → XY → wait → lower 1mm
"""

from __future__ import annotations

import logging
import math
import random
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from types import SimpleNamespace
from typing import Callable, Optional

logger = logging.getLogger(__name__)


def _settled_pump_move(ctrl, pump, volume_uL, rate_uL_s=None):
    """v7.5.x: discrete, blocking pump actuation with the controller's
    configured settle dwell (Hardware Setup → Pump). Every pick/place pump
    move is discrete (a clear next step follows), so it brackets the move
    with the settle time and blocks until the pump finishes. Falls back to a
    plain move on older controllers / fakes that lack the ``settle`` kwarg."""
    try:
        ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate_uL_s, settle=True)
    except TypeError:
        ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate_uL_s)


# ════════════════════════════════════════════════════════════════════
#  DATA MODEL
# ════════════════════════════════════════════════════════════════════

@dataclass
class PickPlaceTarget:
    """A target identified in the stitched image."""
    target_id: str                    # Unique ID (e.g., "T001")
    x_um: float                       # Stage X in µm
    y_um: float                       # Stage Y in µm
    well_name: str                    # Which well this target is in
    pixel_x: int = 0                  # Pixel X in stitched image
    pixel_y: int = 0                  # Pixel Y in stitched image
    size_um: float = 0.0              # Estimated target diameter in µm
    label: str = ""                   # User label
    selected: bool = True             # Whether included in operations

    def to_dict(self) -> dict:
        return {
            "target_id": self.target_id,
            "x_um": self.x_um,
            "y_um": self.y_um,
            "well_name": self.well_name,
            "pixel_x": self.pixel_x,
            "pixel_y": self.pixel_y,
            "size_um": self.size_um,
            "label": self.label,
            "selected": self.selected,
        }

    @staticmethod
    def from_dict(d: dict) -> PickPlaceTarget:
        return PickPlaceTarget(**d)


class OperationType(Enum):
    SPHEROID_PICKUP = "spheroid_pickup"
    TRYPSIN_CELL_PICKUP = "trypsin_cell_pickup"
    FLUORESCENT_TAGGING = "fluorescent_tagging"
    CELL_TARGET_REMOVAL = "cell_target_removal"
    CELL_LABELING = "cell_labeling"


class OperationStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


# ── Operation Configs ────────────────────────────────────────────

@dataclass
class SpheroidPickupConfig:
    """Config for spheroid pickup mode."""
    spheroid_diameter_um: float = 200.0
    safety_factor: float = 1.5        # Volume multiplier
    pickup_bore: str = "P1"           # Which pump bore to use
    pickup_speed_uL_s: float = 1.0    # Aspiration speed
    release_speed_uL_s: float = 1.0   # Dispensing speed
    # Needle Z heights for the pick / place, expressed as a height ABOVE the
    # calibrated plate bottom (mm, + = up). They are deliberately independent:
    # the pick is usually just off the glass to grab a settled spheroid, while
    # the place sits higher to release it. Resolved to a zero-ref Z by the GUI
    # via StageController.print_height_to_zref and pushed to the executor.
    pick_z_offset_mm: float = 0.10
    place_z_offset_mm: float = 0.50
    # Optional pauses (s) the needle holds after aspirating at the pick and
    # after dispensing at the place — lets a spheroid settle into / out of the
    # bore. Default 0.0 (``_dwell(0)`` returns immediately → no behaviour change).
    pick_dwell_s: float = 0.0
    place_dwell_s: float = 0.0

    def compute_volume_uL(self) -> float:
        """Compute pickup volume from spheroid diameter.

        V = (4/3)π(d/2)³ * safety_factor, converted from µm³ to µL.
        1 µL = 1e9 µm³
        """
        r_um = self.spheroid_diameter_um / 2.0
        vol_um3 = (4.0 / 3.0) * math.pi * (r_um ** 3)
        vol_uL = vol_um3 / 1e9  # µm³ → µL
        return vol_uL * self.safety_factor

    def to_dict(self) -> dict:
        return {
            "spheroid_diameter_um": self.spheroid_diameter_um,
            "safety_factor": self.safety_factor,
            "pickup_bore": self.pickup_bore,
            "pickup_speed_uL_s": self.pickup_speed_uL_s,
            "release_speed_uL_s": self.release_speed_uL_s,
            "pick_z_offset_mm": self.pick_z_offset_mm,
            "place_z_offset_mm": self.place_z_offset_mm,
            "pick_dwell_s": self.pick_dwell_s,
            "place_dwell_s": self.place_dwell_s,
        }


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
    trypsin_well: str = ""            # Well containing trypsin solution
    dest_well: str = ""               # Destination well for cells

    def to_dict(self) -> dict:
        return {
            "trypsin_volume_uL": self.trypsin_volume_uL,
            "dwell_time_s": self.dwell_time_s,
            "single_bore": self.single_bore,
            "trypsin_bore": self.trypsin_bore,
            "extraction_bore": self.extraction_bore,
            "extraction_volume_uL": self.extraction_volume_uL,
            "push_speed_uL_s": self.push_speed_uL_s,
            "pull_speed_uL_s": self.pull_speed_uL_s,
            "trypsin_well": self.trypsin_well,
            "dest_well": self.dest_well,
        }


@dataclass
class CellRemovalConfig:
    """Config for cell targeting & removal (trypsinize-in-place then extract).

    The needle is loaded with a cell-release reagent (e.g. trypsin), driven to a
    cell-removal location just off the plate bottom, where it SLOWLY pushes in a
    small column of reagent (the needle's inner cross-section area × a short
    push depth), waits a user-defined incubation time for the cells to release,
    then QUICKLY pulls up a multiple of that volume (reagent + released cells),
    travels to a placing location and dispenses there.

    The push volume is intentionally derived from the needle bore so it is "a
    small amount" relative to the needle (``cross_section_area_mm² × depth``);
    the GUI stamps the resolved value into ``release_volume_uL`` so the executor
    has a concrete number even without a needle in hand.
    """
    reagent_bore: str = "P1"             # bore holding / dispensing the reagent
    # Push (reagent delivery): volume = needle inner area × release_depth_mm.
    release_depth_mm: float = 0.10       # "needle inner area × 0.1 mm" column
    release_volume_uL: float = 0.0       # resolved push volume (µL); see above
    extract_multiplier: float = 2.0      # pull volume = multiplier × push volume
    dwell_time_s: float = 60.0           # incubation after pushing reagent in
    push_speed_uL_s: float = 0.5         # SLOW push (and the gentle final dispense)
    pull_speed_uL_s: float = 5.0         # FAST extraction pull
    # Needle Z heights, as a height ABOVE the calibrated plate bottom (mm, +=up).
    removal_z_offset_mm: float = 0.10    # at the cell-removal location (off glass)
    place_z_offset_mm: float = 0.50      # at the placing location

    def compute_release_volume_uL(self, needle=None) -> float:
        """Push (reagent-release) volume in µL.

        When a needle is supplied, the volume is the bore column
        ``cross_section_area_mm² × release_depth_mm`` (1 mm³ == 1 µL). Otherwise
        falls back to the pre-resolved ``release_volume_uL`` the GUI stamped in.
        """
        if needle is not None:
            try:
                area = float(needle.cross_section_area_mm2)
                if area > 0:
                    return area * float(self.release_depth_mm)
            except (TypeError, ValueError, AttributeError):
                pass
        return float(self.release_volume_uL or 0.0)

    def compute_extract_volume_uL(self, needle=None) -> float:
        """Fast-pull (extraction) volume = ``extract_multiplier × push``."""
        return self.compute_release_volume_uL(needle) * float(self.extract_multiplier)

    def to_dict(self) -> dict:
        return {
            "reagent_bore": self.reagent_bore,
            "release_depth_mm": self.release_depth_mm,
            "release_volume_uL": self.release_volume_uL,
            "extract_multiplier": self.extract_multiplier,
            "dwell_time_s": self.dwell_time_s,
            "push_speed_uL_s": self.push_speed_uL_s,
            "pull_speed_uL_s": self.pull_speed_uL_s,
            "removal_z_offset_mm": self.removal_z_offset_mm,
            "place_z_offset_mm": self.place_z_offset_mm,
        }


@dataclass
class CellLabelingConfig:
    """Config for cell labeling / staining (deposit a stain, incubate, remove).

    Unlike Cell Targeting & Removal there is NO placement step: the operator
    picks the regions to stain and the stain reagent, and for each region the
    needle:

      1. loads the stain from its reagent well,
      2. travels to the region (just off the plate bottom),
      3. SLOWLY deposits a small column of stain (the needle's inner
         cross-section area × a short deposit depth),
      4. waits a user-defined incubation time — the headline knob: how long the
         stain is allowed to develop on the cells,
      5. SLOWLY aspirates a multiple of the deposited volume back up (the stain
         plus a little surrounding fluid), then
      6. travels to the waste well and dispenses it (there is no place target).

    Both the deposit AND the aspirate are intentionally slow so the stain is
    delivered and withdrawn gently. The pump is volume-balanced per region
    (load − deposit − aspirate + waste-dispense = 0), so it never drifts.

    The deposit volume is derived from the needle bore so it is "a small amount"
    relative to the needle (``cross_section_area_mm² × depth``); the GUI stamps
    the resolved value into ``deposit_volume_uL`` so the executor has a concrete
    number even without a needle in hand.
    """
    stain_bore: str = "P1"               # bore holding / dispensing the stain
    # Deposit (stain delivery): volume = needle inner area × deposit_depth_mm.
    deposit_depth_mm: float = 0.10       # "needle inner area × 0.1 mm" column
    deposit_volume_uL: float = 0.0       # resolved deposit volume (µL); see above
    aspirate_multiplier: float = 2.0     # aspirate-back = multiplier × deposit
    stain_dwell_time_s: float = 300.0    # incubation — how long the stain develops
    deposit_speed_uL_s: float = 0.5      # SLOW deposition
    aspirate_speed_uL_s: float = 0.5     # SLOW aspiration (both slow by request)
    # Needle Z height at the stain region, as a height ABOVE the calibrated
    # plate bottom (mm, += up). Resolved to a zero-ref Z by the GUI and pushed
    # to the executor as ``pick_z_mm`` (same contract as the other handlers).
    label_z_offset_mm: float = 0.10

    def compute_deposit_volume_uL(self, needle=None) -> float:
        """Deposit (stain) volume in µL.

        When a needle is supplied, the volume is the bore column
        ``cross_section_area_mm² × deposit_depth_mm`` (1 mm³ == 1 µL). Otherwise
        falls back to the pre-resolved ``deposit_volume_uL`` the GUI stamped in.
        """
        if needle is not None:
            try:
                area = float(needle.cross_section_area_mm2)
                if area > 0:
                    return area * float(self.deposit_depth_mm)
            except (TypeError, ValueError, AttributeError):
                pass
        return float(self.deposit_volume_uL or 0.0)

    def compute_aspirate_volume_uL(self, needle=None) -> float:
        """Stain-removal aspirate volume = ``aspirate_multiplier × deposit``."""
        return self.compute_deposit_volume_uL(needle) * float(
            self.aspirate_multiplier)

    def to_dict(self) -> dict:
        return {
            "stain_bore": self.stain_bore,
            "deposit_depth_mm": self.deposit_depth_mm,
            "deposit_volume_uL": self.deposit_volume_uL,
            "aspirate_multiplier": self.aspirate_multiplier,
            "stain_dwell_time_s": self.stain_dwell_time_s,
            "deposit_speed_uL_s": self.deposit_speed_uL_s,
            "aspirate_speed_uL_s": self.aspirate_speed_uL_s,
            "label_z_offset_mm": self.label_z_offset_mm,
        }


@dataclass
class DyeConfig:
    """Config for a single fluorescent dye bore."""
    bore: str = "P1"                   # Pump bore
    dye_well: str = ""                 # Well containing this dye
    dye_name: str = ""                 # e.g., "DAPI", "GFP", "mCherry"
    volume_uL: float = 2.0            # Volume per target
    color: str = "#89b4fa"             # Display color

    def to_dict(self) -> dict:
        return {
            "bore": self.bore,
            "dye_well": self.dye_well,
            "dye_name": self.dye_name,
            "volume_uL": self.volume_uL,
            "color": self.color,
        }


@dataclass
class FluorescentTaggingConfig:
    """Config for fluorescent tagging mode."""
    num_bores: int = 1                 # 1, 2, or 3 bores
    dye_configs: list[DyeConfig] = field(default_factory=list)
    dwell_time_s: float = 300.0        # Incubation time
    waste_bore: Optional[str] = None   # e.g., "P3" — local waste collector
    use_waste_bore_mode: bool = False   # Use bore as local waste collector
    waste_well: str = ""               # Well to deposit waste
    buffer_well: str = ""              # Buffer wash well
    wash_well: str = ""                # Wash well

    def to_dict(self) -> dict:
        return {
            "num_bores": self.num_bores,
            "dye_configs": [d.to_dict() for d in self.dye_configs],
            "dwell_time_s": self.dwell_time_s,
            "waste_bore": self.waste_bore,
            "use_waste_bore_mode": self.use_waste_bore_mode,
            "waste_well": self.waste_well,
            "buffer_well": self.buffer_well,
            "wash_well": self.wash_well,
        }


# ── Operation ────────────────────────────────────────────────────

@dataclass
class PickPlaceOperation:
    """A single pick-and-place operation in the queue."""
    op_id: str
    op_type: OperationType
    source_target: PickPlaceTarget     # Where to pick from / operate on
    dest_target: Optional[PickPlaceTarget] = None  # Where to place
    config: SpheroidPickupConfig | TrypsinPickupConfig | FluorescentTaggingConfig | CellRemovalConfig | CellLabelingConfig = field(
        default_factory=SpheroidPickupConfig)
    status: OperationStatus = OperationStatus.PENDING
    error_msg: str = ""
    sub_step: str = ""                 # Current sub-step description

    @staticmethod
    def make_id() -> str:
        return f"OP-{uuid.uuid4().hex[:8].upper()}"


# ════════════════════════════════════════════════════════════════════
#  OPERATION QUEUE
# ════════════════════════════════════════════════════════════════════

class OperationQueue:
    """Ordered queue of PickPlaceOperations."""

    def __init__(self):
        self._operations: list[PickPlaceOperation] = []

    @property
    def operations(self) -> list[PickPlaceOperation]:
        return list(self._operations)

    def __len__(self) -> int:
        return len(self._operations)

    def add(self, op: PickPlaceOperation):
        """Add an operation to the end of the queue."""
        self._operations.append(op)
        logger.info(f"Queue: added {op.op_id} ({op.op_type.value})")

    def remove(self, op_id: str):
        """Remove an operation by ID."""
        self._operations = [o for o in self._operations if o.op_id != op_id]

    def reorder(self, op_id: str, new_index: int):
        """Move an operation to a new position in the queue."""
        op = next((o for o in self._operations if o.op_id == op_id), None)
        if op is None:
            return
        self._operations.remove(op)
        new_index = max(0, min(new_index, len(self._operations)))
        self._operations.insert(new_index, op)

    def get_pending(self) -> list[PickPlaceOperation]:
        """Return all pending operations."""
        return [o for o in self._operations
                if o.status == OperationStatus.PENDING]

    def mark_running(self, op_id: str):
        for o in self._operations:
            if o.op_id == op_id:
                o.status = OperationStatus.RUNNING
                break

    def mark_completed(self, op_id: str):
        for o in self._operations:
            if o.op_id == op_id:
                o.status = OperationStatus.COMPLETED
                break

    def mark_failed(self, op_id: str, msg: str = ""):
        for o in self._operations:
            if o.op_id == op_id:
                o.status = OperationStatus.FAILED
                o.error_msg = msg
                break

    def clear(self):
        self._operations.clear()

    def get_progress(self) -> tuple[int, int]:
        """Return (completed_count, total_count)."""
        completed = sum(1 for o in self._operations
                        if o.status == OperationStatus.COMPLETED)
        return (completed, len(self._operations))


# ════════════════════════════════════════════════════════════════════
#  EXECUTION ENGINE
# ════════════════════════════════════════════════════════════════════

class PickPlaceExecutor:
    """Executes operations from the queue using StageController.

    All movements enforce the Safe Z protocol:
      - Inter-well: raise to safe_z → wait → XY → wait → lower
      - Intra-well: retract by intra_well_retract_mm → wait → XY → wait → lower

    Threading: designed to run in a daemon thread with pause/abort support.
    """

    def __init__(self, controller, hw_config=None):
        """
        Args:
            controller: StageController instance.
            hw_config: HardwareConfig for pump µL↔mm conversion.
        """
        self.controller = controller
        self.hw_config = hw_config

        # Safe Z protocol parameters
        self.safe_z_mm: float = 5.0
        self.operating_z_mm: float = 0.0   # Default Z height for operations
        # Spheroid pick / place can use distinct operating heights (zero-ref
        # mm). When set, the spheroid handler uses pick_z_mm at the source and
        # place_z_mm at the destination; None falls back to operating_z_mm.
        self.pick_z_mm: Optional[float] = None
        self.place_z_mm: Optional[float] = None
        self.intra_well_retract_mm: float = 1.0
        self.z_timeout_s: float = 15.0
        self.xy_timeout_s: float = 30.0

        # Service well positions (ABSOLUTE stage µm). Populated by the workflow
        # page from the calibrated well positions + the Hardware Setup reagent
        # locations (which well each ink_type oil/wash/waste/buffer maps to).
        self.waste_well_pos: Optional[tuple[float, float]] = None
        self.oil_well_pos: Optional[tuple[float, float]] = None
        self.wash_well_pos: Optional[tuple[float, float]] = None
        self.buffer_well_pos: Optional[tuple[float, float]] = None
        # Cell-release reagent (e.g. trypsin) reagent well — used by the
        # CELL_TARGET_REMOVAL handler to load the needle before each removal.
        # ABSOLUTE stage µm; dip Z in zero-ref mm.
        self.reagent_well_pos: Optional[tuple[float, float]] = None
        self.reagent_dip_z_mm: Optional[float] = None

        # ── Prep routine (run once before the pick & place loop) ──────
        # The needle is conditioned before picking: DISPENSE a needle of oil to
        # waste, ASPIRATE a needle of fresh oil, wash, then load buffer. All
        # volumes are in multiples of one needle's internal bore volume.
        self.do_prep: bool = False
        self.prep_bore: str = "P1"           # pump/bore used for prep aspirate/dispense
        self.needle_volume_uL: float = 0.0   # "1 needle's worth" (bore cylinder)
        self.oil_needles: float = 1.0        # dispensed to waste AND aspirated from oil
        self.buffer_needles: float = 1.0     # aspirated from the buffer well
        self.prep_rate_uL_s: float = 1.0     # aspirate/dispense flow during prep
        self.service_z_mm: Optional[float] = None  # dip Z at service wells (zero-ref)
        # Wash = dip + jiggle Z up/down + random XY jiggle about the well centre.
        self.wash_cycles: int = 3
        self.wash_z_amplitude_mm: float = 0.5      # how far up/down each jiggle
        self.wash_xy_amplitude_um: float = 200.0   # random XY radius about centre
        self.wash_dwell_s: float = 0.3             # settle between jiggles

        # ── Post-clean routine (run once after the loop, e.g. cell removal) ──
        # "Needle waste, wash, and reset": DISPENSE residual to waste, wash, then
        # reload buffer so the needle ends conditioned. Reuses the same service
        # wells / needle volume as the prep. Default off (spheroid path unchanged).
        self.do_post_clean: bool = False
        # v7.5.x: renamed from post_expel_needles (DISPENSE = push fluid out).
        self.post_dispense_needles: float = 1.0  # needles of residual dispensed to waste

        # ── Print cleanup routine (Quick Print: after the print finishes) ──
        # "Waste, wash, reset oil": DISPENSE a multiple of the needle to waste,
        # wash, then refill OIL to reset the syringe's oil level. When
        # ``cleanup_oil_baseline_uL`` is set, the oil step returns the plunger to
        # that pre-run position (a genuine "reset the amount of oil in the
        # syringe"); otherwise it aspirates ``cleanup_oil_needles`` × a needle.
        self.cleanup_waste_needles: float = 6.0
        self.cleanup_oil_needles: float = 1.0   # fixed fallback oil aspirate
        self.cleanup_oil_baseline_uL: Optional[float] = None  # pre-run plunger µL
        # v7.5.x: "reset syringe to initial condition" mode. When True the waste
        # step DISPENSES the live leftover (current plunger − baseline = the
        # unprinted ink + buffer) PLUS a small oil flush margin, then the oil
        # step returns the plunger to ``cleanup_oil_baseline_uL`` (its pre-run /
        # initial position). So the run ends with the syringe back to its
        # initial state: pure oil at the initial fill. ``cleanup_waste_needles``
        # is ignored in this mode (waste is computed live).
        self.cleanup_reset_to_initial: bool = False
        self.cleanup_oil_margin_uL: float = 0.0  # small oil flushed past the tip

        # State
        self._current_well: str = ""
        self._abort_flag = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # Not paused initially

        # Callbacks
        self.on_op_started: Optional[Callable] = None
        self.on_op_completed: Optional[Callable] = None
        self.on_op_failed: Optional[Callable] = None
        self.on_sub_step: Optional[Callable] = None
        self.on_dwell_tick: Optional[Callable] = None

    # ── Queue execution ──────────────────────────────────────────

    def execute_queue(
        self,
        queue: OperationQueue,
        on_progress: Optional[Callable] = None,
    ) -> bool:
        """Execute all pending operations in the queue.

        Args:
            queue: The operation queue to execute.
            on_progress: Callback(completed, total, message).

        Returns:
            True if all operations completed, False if aborted/failed.
        """
        self._abort_flag.clear()
        pending = queue.get_pending()
        total = len(pending)

        if total == 0:
            logger.info("PickPlaceExecutor: no pending operations")
            return True

        logger.info(f"PickPlaceExecutor: starting {total} operations")

        try:
            # One-time needle conditioning before the pick & place loop.
            if self.do_prep:
                self.run_prep()

            for i, op in enumerate(pending):
                # Check abort
                if self._abort_flag.is_set():
                    logger.info("PickPlaceExecutor: aborted")
                    return False

                # Check pause
                self._pause_event.wait()

                # Mark running
                queue.mark_running(op.op_id)
                op.sub_step = "Starting..."
                if self.on_op_started:
                    self.on_op_started(op)

                try:
                    self._execute_operation(op)
                    queue.mark_completed(op.op_id)
                    if self.on_op_completed:
                        self.on_op_completed(op)
                except AbortException:
                    logger.info(f"Operation {op.op_id} aborted")
                    return False
                except Exception as e:
                    logger.error(f"Operation {op.op_id} failed: {e}", exc_info=True)
                    queue.mark_failed(op.op_id, str(e))
                    if self.on_op_failed:
                        self.on_op_failed(op, str(e))

                if on_progress:
                    completed, _ = queue.get_progress()
                    on_progress(completed, total,
                                f"Completed {completed}/{total}")

            # One-time needle clean-up after the loop (waste → wash → buffer).
            if self.do_post_clean and not self._abort_flag.is_set():
                self.run_post_clean()

            logger.info("PickPlaceExecutor: all operations complete")
            return True
        finally:
            # SAFETY: always leave the needle at the safe travel height on ANY
            # exit (completion / abort / error). The spheroid op ends at place_z
            # (needle DOWN) with no retract of its own, so without this the
            # needle stays parked in a well — a crash waiting for the next move
            # or a manual jog. Mirrors the print path's end-at-safe-Z guarantee.
            self._retract_to_safe_z()

    def _retract_to_safe_z(self):
        """Best-effort raise the needle to the safe travel height.

        Polarity-safe and RAISE-ONLY via ``StageController.ensure_retracted_to``
        (a confirmed no-op when the needle is already retracted or the ZP board
        is disconnected). Never raises — runs in the ``execute_queue`` finally on
        every exit path.
        """
        safe_z = getattr(self, "safe_z_mm", None)
        ctrl = self.controller
        if safe_z is None or ctrl is None:
            return
        # Only use ensure_retracted_to — it is raise-only / polarity-safe. A bare
        # move_z_absolute(safe_z) is NOT (it would DESCEND if the needle were
        # already above safe_z), so we deliberately do NOT fall back to it; a
        # controller without ensure_retracted_to just skips the end retract.
        if not hasattr(ctrl, "ensure_retracted_to"):
            return
        try:
            ctrl.ensure_retracted_to(safe_z)
        except Exception as e:
            logger.warning(f"PickPlaceExecutor: end-of-run retract failed: {e}")

    def pause(self):
        """Pause execution after current sub-step completes."""
        self._pause_event.clear()
        logger.info("PickPlaceExecutor: paused")

    def resume(self):
        """Resume paused execution."""
        self._pause_event.set()
        logger.info("PickPlaceExecutor: resumed")

    def abort(self):
        """Abort execution."""
        self._abort_flag.set()
        self._pause_event.set()  # Unblock pause wait
        logger.info("PickPlaceExecutor: abort requested")

    # ── Operation dispatch ───────────────────────────────────────

    def _execute_operation(self, op: PickPlaceOperation):
        """Dispatch to the appropriate operation handler."""
        if op.op_type == OperationType.SPHEROID_PICKUP:
            self._execute_spheroid_pickup(op)
        elif op.op_type == OperationType.TRYPSIN_CELL_PICKUP:
            self._execute_trypsin_pickup(op)
        elif op.op_type == OperationType.FLUORESCENT_TAGGING:
            self._execute_fluorescent_tagging(op)
        elif op.op_type == OperationType.CELL_TARGET_REMOVAL:
            self._execute_cell_removal(op)
        elif op.op_type == OperationType.CELL_LABELING:
            self._execute_cell_labeling(op)
        else:
            raise ValueError(f"Unknown operation type: {op.op_type}")

    # ── Spheroid Pickup ──────────────────────────────────────────

    def _execute_spheroid_pickup(self, op: PickPlaceOperation):
        """Execute a spheroid pickup operation.

        1. Navigate to source target (safe Z protocol)
        2. Lower to operating Z
        3. Aspirate computed volume
        4. Raise + safe travel to destination
        5. Dispense same volume
        """
        cfg: SpheroidPickupConfig = op.config
        volume_uL = cfg.compute_volume_uL()
        source = op.source_target
        dest = op.dest_target

        # Pick / place use independent operating heights when supplied.
        pick_z = self.pick_z_mm if self.pick_z_mm is not None else self.operating_z_mm
        place_z = self.place_z_mm if self.place_z_mm is not None else self.operating_z_mm

        logger.info(
            f"Spheroid pickup: {source.target_id} → {dest.target_id if dest else 'N/A'}, "
            f"diameter={cfg.spheroid_diameter_um}µm, volume={volume_uL:.4f}µL, "
            f"pick_z={pick_z:.3f} place_z={place_z:.3f} mm")

        # 1. Move to source (lower to the pick height)
        self._set_sub_step(op, f"Moving to source {source.target_id}")
        self._safe_move_to(source, target_z_mm=pick_z)
        self._check_abort()

        # 2. Aspirate (move_pump_uL takes µL/s and clamps the flow rate itself)
        self._set_sub_step(op, f"Aspirating {volume_uL:.4f} µL")
        _settled_pump_move(self.controller, cfg.pickup_bore, -volume_uL,
                                     rate_uL_s=cfg.pickup_speed_uL_s)
        self._check_abort()

        # 2b. Optional pick pause (let the spheroid settle into the bore).
        if getattr(cfg, "pick_dwell_s", 0.0):
            self._dwell(op, float(cfg.pick_dwell_s), "Pick pause")
            self._check_abort()

        # 3. Move to destination (lower to the place height)
        if dest:
            self._set_sub_step(op, f"Moving to dest {dest.target_id}")
            self._safe_move_to(dest, target_z_mm=place_z)
            self._check_abort()

            # 4. Dispense
            self._set_sub_step(op, f"Dispensing {volume_uL:.4f} µL")
            _settled_pump_move(self.controller, cfg.pickup_bore, volume_uL,
                                         rate_uL_s=cfg.release_speed_uL_s)

            # 4b. Optional place pause (let the spheroid release from the bore).
            if getattr(cfg, "place_dwell_s", 0.0):
                self._check_abort()
                self._dwell(op, float(cfg.place_dwell_s), "Place pause")

        self._set_sub_step(op, "Complete")

    # ── Cell Targeting & Removal ─────────────────────────────────

    def _execute_cell_removal(self, op: PickPlaceOperation):
        """Execute one cell targeting & removal operation.

        Sequence (per target — the needle prep + the post-clean bracket the
        whole loop, run once each by ``execute_queue``):

          1. Load the cell-release reagent (trypsin): safe-travel to the reagent
             well + draw the push volume.
          2. Travel to the cell-removal location, lower to the removal Z
             (just off the plate bottom).
          3. SLOWLY push the reagent column in.
          4. Wait the user-defined incubation time.
          5. QUICKLY pull up ``extract_multiplier`` × the pushed volume
             (reagent + released cells).
          6. Travel to the placing location, lower to the place Z.
          7. Gently dispense the extracted volume.

        The pump is volume-balanced over the op (load − push − pull + dispense
        = 0), so it never drifts. Heights are pushed in by the GUI:
        ``pick_z_mm`` = the removal height, ``place_z_mm`` = the place height
        (both zero-ref mm; None falls back to ``operating_z_mm`` — same contract
        as the spheroid handler).
        """
        cfg: CellRemovalConfig = op.config
        source = op.source_target
        dest = op.dest_target
        bore = cfg.reagent_bore

        needle = (getattr(self.hw_config, "needle", None)
                  if self.hw_config is not None else None)
        push_uL = cfg.compute_release_volume_uL(needle)
        pull_uL = push_uL * float(cfg.extract_multiplier)

        # Pick / place use independent operating heights when supplied.
        removal_z = self.pick_z_mm if self.pick_z_mm is not None else self.operating_z_mm
        place_z = self.place_z_mm if self.place_z_mm is not None else self.operating_z_mm

        logger.info(
            "Cell removal: %s → %s, push=%.5f µL (slow %.2f µL/s), pull=%.5f µL "
            "(fast %.2f µL/s), dwell=%.0fs, removal_z=%.3f place_z=%.3f mm",
            source.target_id, dest.target_id if dest else "N/A",
            push_uL, cfg.push_speed_uL_s, pull_uL, cfg.pull_speed_uL_s,
            cfg.dwell_time_s, removal_z, place_z)

        # 1. Load the cell-release reagent from its well.
        if self.reagent_well_pos is not None and push_uL > 0:
            self._set_sub_step(
                op, f"Loading {push_uL:.4f} µL of cell-release reagent")
            if not self._safe_move_to_well(
                    "__reagent__", target_z_mm=self.reagent_dip_z_mm):
                raise RuntimeError(
                    "Cell removal: reagent well not configured/resolved")
            self._check_abort()
            _settled_pump_move(self.controller, bore, -push_uL,
                                         rate_uL_s=cfg.push_speed_uL_s)
            self._check_abort()

        # 2. Travel to the cell-removal location (lower to the removal Z).
        self._set_sub_step(op, f"Moving to removal target {source.target_id}")
        self._safe_move_to(source, target_z_mm=removal_z)
        self._check_abort()

        # 3. Slowly push the reagent in.
        if push_uL > 0:
            self._set_sub_step(op, f"Releasing {push_uL:.4f} µL (slow)")
            _settled_pump_move(self.controller, bore, +push_uL,
                                         rate_uL_s=cfg.push_speed_uL_s)
            self._check_abort()

        # 4. Incubate.
        self._dwell(op, cfg.dwell_time_s, "Cell-release incubation")
        self._check_abort()

        # 5. Quickly pull up the extraction volume (reagent + cells).
        if pull_uL > 0:
            self._set_sub_step(op, f"Extracting {pull_uL:.4f} µL (fast)")
            _settled_pump_move(self.controller, bore, -pull_uL,
                                         rate_uL_s=cfg.pull_speed_uL_s)
            self._check_abort()

        # 6. Travel to the placing location (lower to the place Z).
        if dest:
            self._set_sub_step(op, f"Moving to placement {dest.target_id}")
            self._safe_move_to(dest, target_z_mm=place_z)
            self._check_abort()

            # 7. Gently dispense the extracted cells.
            if pull_uL > 0:
                self._set_sub_step(op, f"Dispensing {pull_uL:.4f} µL")
                _settled_pump_move(self.controller, bore, +pull_uL,
                                             rate_uL_s=cfg.push_speed_uL_s)

        self._set_sub_step(op, "Complete")

    # ── Cell Labeling / Staining ─────────────────────────────────

    def _execute_cell_labeling(self, op: PickPlaceOperation):
        """Execute one cell labeling / staining operation.

        Sequence (per region — the needle prep + the post-clean bracket the
        whole loop, run once each by ``execute_queue``; there is NO placement):

          1. Load the stain reagent: safe-travel to the reagent well + draw the
             deposit volume.
          2. Travel to the stain region, lower to the label Z (just off bottom).
          3. SLOWLY deposit the stain column.
          4. Wait the user-defined incubation time (the headline knob).
          5. SLOWLY aspirate up ``aspirate_multiplier`` × the deposited volume
             (stain + a little surrounding fluid).
          6. Travel to the waste well and dispense it (no place target).

        The pump is volume-balanced over the op (load − deposit − aspirate +
        waste-dispense = 0), so it never drifts. ``pick_z_mm`` (pushed in by the
        GUI) is the label height; ``waste_well_pos`` / ``service_z_mm`` are the
        waste-dump position + dip Z (always required for this workflow — the
        dump is core, not part of the optional prep/clean).
        """
        cfg: CellLabelingConfig = op.config
        source = op.source_target
        bore = cfg.stain_bore

        needle = (getattr(self.hw_config, "needle", None)
                  if self.hw_config is not None else None)
        deposit_uL = cfg.compute_deposit_volume_uL(needle)
        aspirate_uL = deposit_uL * float(cfg.aspirate_multiplier)

        # The label height reuses pick_z_mm (None falls back to operating_z_mm —
        # same contract as the spheroid / cell-removal handlers).
        label_z = self.pick_z_mm if self.pick_z_mm is not None else self.operating_z_mm

        logger.info(
            "Cell labeling: %s, deposit=%.5f µL (slow %.2f µL/s), aspirate=%.5f "
            "µL (slow %.2f µL/s), stain dwell=%.0fs, label_z=%.3f mm",
            source.target_id, deposit_uL, cfg.deposit_speed_uL_s, aspirate_uL,
            cfg.aspirate_speed_uL_s, cfg.stain_dwell_time_s, label_z)

        # 1. Load the stain from its reagent well.
        if self.reagent_well_pos is not None and deposit_uL > 0:
            self._set_sub_step(op, f"Loading {deposit_uL:.4f} µL of stain")
            if not self._safe_move_to_well(
                    "__reagent__", target_z_mm=self.reagent_dip_z_mm):
                raise RuntimeError(
                    "Cell labeling: reagent well not configured/resolved")
            self._check_abort()
            _settled_pump_move(self.controller, bore, -deposit_uL,
                               rate_uL_s=cfg.aspirate_speed_uL_s)
            self._check_abort()

        # 2. Travel to the stain region (lower to the label Z).
        self._set_sub_step(op, f"Moving to stain region {source.target_id}")
        self._safe_move_to(source, target_z_mm=label_z)
        self._check_abort()

        # 3. Slowly deposit the stain.
        if deposit_uL > 0:
            self._set_sub_step(op, f"Depositing {deposit_uL:.4f} µL (slow)")
            _settled_pump_move(self.controller, bore, +deposit_uL,
                               rate_uL_s=cfg.deposit_speed_uL_s)
            self._check_abort()

        # 4. Incubate — the headline knob (how long the stain develops).
        self._dwell(op, cfg.stain_dwell_time_s, "Stain incubation")
        self._check_abort()

        # 5. Slowly aspirate the stain (+ excess) back up.
        if aspirate_uL > 0:
            self._set_sub_step(op, f"Aspirating {aspirate_uL:.4f} µL (slow)")
            _settled_pump_move(self.controller, bore, -aspirate_uL,
                               rate_uL_s=cfg.aspirate_speed_uL_s)
            self._check_abort()

        # 6. Travel to the waste well and dispense the recovered stain.
        if aspirate_uL > 0 and self.waste_well_pos is not None:
            self._set_sub_step(op, "Moving to waste")
            if not self._safe_move_to_well(
                    "__waste__", target_z_mm=self.service_z_mm):
                raise RuntimeError(
                    "Cell labeling: waste well not configured/resolved")
            self._check_abort()
            self._set_sub_step(op, f"Dispensing {aspirate_uL:.4f} µL to waste")
            _settled_pump_move(self.controller, bore, +aspirate_uL,
                               rate_uL_s=cfg.deposit_speed_uL_s)

        self._set_sub_step(op, "Complete")

    # ── Prep routine (needle conditioning before the pick & place loop) ──

    def run_prep(self):
        """Condition the needle before picking, once, at the start of a run.

        Sequence (each service move is a full safe-Z travel — retract → cross →
        lower to the service dip Z, via ``_safe_move_to_well``):
          1. → waste  ; DISPENSE ``oil_needles`` × a needle of oil
          2. → oil    ; ASPIRATE ``oil_needles`` × a needle of fresh oil
          3. → wash   ; wash (dip + jiggle Z + random XY about the well centre)
          4. → buffer ; ASPIRATE ``buffer_needles`` × a needle of buffer

        Volumes are multiples of one needle's internal bore volume
        (``needle_volume_uL``). Aspirate/dispense use the configured
        ``prep_bore`` at ``prep_rate_uL_s``. Abort-aware. Raises if a required
        service well is unresolved (the GUI gates on this up front; this is a
        backstop so the run stops cleanly rather than silently skipping a step).
        """
        prep_op = SimpleNamespace(op_id="PREP", sub_step="")
        unit = float(self.needle_volume_uL or 0.0)
        bore = self.prep_bore
        rate = self.prep_rate_uL_s
        sz = self.service_z_mm

        def goto(key, label):
            self._check_abort()
            self._set_sub_step(prep_op, f"Prep: travel to {label}")
            if not self._safe_move_to_well(key, target_z_mm=sz):
                raise RuntimeError(
                    f"Prep: {label} well not configured/resolved ({key})")

        # 1. Waste — dispense oil.
        goto("__waste__", "waste")
        if unit > 0 and self.oil_needles > 0:
            self._set_sub_step(
                prep_op,
                f"Prep: dispense {self.oil_needles:g} needle(s) of oil to waste")
            _settled_pump_move(self.controller, 
                bore, +unit * self.oil_needles, rate_uL_s=rate)
            self._check_abort()

        # 2. Oil — aspirate fresh oil.
        goto("__oil__", "oil")
        if unit > 0 and self.oil_needles > 0:
            self._set_sub_step(
                prep_op, f"Prep: aspirate {self.oil_needles:g} needle(s) of oil")
            _settled_pump_move(self.controller, 
                bore, -unit * self.oil_needles, rate_uL_s=rate)
            self._check_abort()

        # 3. Wash.
        goto("__wash__", "wash")
        self._set_sub_step(prep_op, "Prep: wash needle")
        self._do_wash()

        # 4. Buffer — aspirate buffer.
        goto("__buffer__", "buffer")
        if unit > 0 and self.buffer_needles > 0:
            self._set_sub_step(
                prep_op,
                f"Prep: aspirate {self.buffer_needles:g} needle(s) of buffer")
            _settled_pump_move(self.controller, 
                bore, -unit * self.buffer_needles, rate_uL_s=rate)

        self._set_sub_step(prep_op, "Prep complete")

    def run_post_clean(self):
        """Clean + reset the needle ONCE after the operation loop.

        Sequence ("needle waste, wash, and reset"):
          1. → waste  ; DISPENSE ``post_dispense_needles`` × a needle of residual
                         (reagent + cells) so the needle is empty
          2. → wash   ; wash (dip + jiggle Z + random XY about the well centre)
          3. → buffer ; ASPIRATE ``buffer_needles`` × a needle of buffer to reset
                         the needle to a conditioned, buffer-loaded state

        Mirrors :meth:`run_prep` — same service wells, needle volume, dip Z,
        ``prep_bore`` / ``prep_rate_uL_s``. Abort-aware; raises if a required
        service well is unresolved (the GUI gates on this up front).
        """
        clean_op = SimpleNamespace(op_id="CLEAN", sub_step="")
        unit = float(self.needle_volume_uL or 0.0)
        bore = self.prep_bore
        rate = self.prep_rate_uL_s
        sz = self.service_z_mm

        def goto(key, label):
            self._check_abort()
            self._set_sub_step(clean_op, f"Clean: travel to {label}")
            if not self._safe_move_to_well(key, target_z_mm=sz):
                raise RuntimeError(
                    f"Clean: {label} well not configured/resolved ({key})")

        # 1. Waste — dispense residual.
        goto("__waste__", "waste")
        if unit > 0 and self.post_dispense_needles > 0:
            self._set_sub_step(
                clean_op,
                f"Clean: dispense {self.post_dispense_needles:g} needle(s) to "
                f"waste")
            _settled_pump_move(self.controller,
                bore, +unit * self.post_dispense_needles, rate_uL_s=rate)
            self._check_abort()

        # 2. Wash.
        goto("__wash__", "wash")
        self._set_sub_step(clean_op, "Clean: wash needle")
        self._do_wash()

        # 3. Buffer — reload buffer (reset).
        goto("__buffer__", "buffer")
        if unit > 0 and self.buffer_needles > 0:
            self._set_sub_step(
                clean_op,
                f"Clean: aspirate {self.buffer_needles:g} needle(s) of buffer")
            _settled_pump_move(self.controller, 
                bore, -unit * self.buffer_needles, rate_uL_s=rate)

        self._set_sub_step(clean_op, "Clean complete")

    def prepare_starting_oil(self, volume_uL: float, *,
                             dispense_to_waste: bool = True):
        """v7.5.x syringe-budget remedy: bring the syringe to a feasible
        STARTING fill BEFORE the prep / ink / print sequence so the whole run
        stays inside the plunger envelope.

        ``dispense_to_waste=True`` → travel (full safe-Z) to the WASTE well and
        DISPENSE ``volume_uL`` of oil (lowers the starting fill — the
        operator-confirmed "waste that amount of oil first" when the run would
        otherwise over-fill the syringe). ``dispense_to_waste=False`` → travel
        to the OIL well and ASPIRATE ``volume_uL`` of fresh oil (raises the
        starting fill when the run would otherwise run the plunger dry).
        Abort-aware; raises if the needed service well is unresolved (the GUI
        gates on this up front)."""
        v = abs(float(volume_uL or 0.0))
        if v <= 1e-6:
            return
        op = SimpleNamespace(op_id="OIL_PREP", sub_step="")
        bore = self.prep_bore
        rate = self.prep_rate_uL_s
        sz = self.service_z_mm
        self._check_abort()
        if dispense_to_waste:
            self._set_sub_step(op, f"Oil prep: waste {v:.3f} µL of oil")
            if not self._safe_move_to_well("__waste__", target_z_mm=sz):
                raise RuntimeError(
                    "Oil prep: waste well not configured/resolved (__waste__)")
            _settled_pump_move(self.controller, bore, +v, rate_uL_s=rate)
        else:
            self._set_sub_step(op, f"Oil prep: aspirate {v:.3f} µL of oil")
            if not self._safe_move_to_well("__oil__", target_z_mm=sz):
                raise RuntimeError(
                    "Oil prep: oil well not configured/resolved (__oil__)")
            _settled_pump_move(self.controller, bore, -v, rate_uL_s=rate)
        self._check_abort()

    def run_print_cleanup(self):
        """Clean + reset the needle ONCE after a Quick Print finishes.

        Sequence ("waste, wash, reset oil"):
          1. → waste ; DISPENSE ``cleanup_waste_needles`` × a needle of residual
                        (ink + buffer + oil) so the column is flushed out
          2. → wash  ; wash (dip + jiggle Z + random XY about the well centre)
          3. → oil   ; reload OIL to reset the syringe's oil level. If
                        ``cleanup_oil_baseline_uL`` is set, aspirate/dispense
                        exactly the amount that returns the plunger to that
                        pre-run position (read live at the oil well, AFTER the
                        waste dispense, so the net-zero is exact); else aspirate
                        ``cleanup_oil_needles`` ×
                        a needle. The net-zero amount is clamped to a sane
                        magnitude and falls back to the fixed draw if a position
                        read is missing or implausible (guards a bad M114 read).

        Reuses the prep service wells / needle volume / dip Z / ``prep_bore`` /
        ``prep_rate_uL_s``. Abort-aware; raises if a required service well is
        unresolved (the GUI gates on this up front).
        """
        clean_op = SimpleNamespace(op_id="PRINT_CLEAN", sub_step="")
        unit = float(self.needle_volume_uL or 0.0)
        bore = self.prep_bore
        rate = self.prep_rate_uL_s
        sz = self.service_z_mm
        baseline = self.cleanup_oil_baseline_uL

        def goto(key, label):
            self._check_abort()
            self._set_sub_step(clean_op, f"Cleanup: travel to {label}")
            if not self._safe_move_to_well(key, target_z_mm=sz):
                raise RuntimeError(
                    f"Cleanup: {label} well not configured/resolved ({key})")

        def _vol_to_baseline():
            """Polarity-correct signed ``move_pump_uL`` volume (``+`` = dispense)
            that returns the plunger from its CURRENT live position to
            ``baseline`` = ``pump_dir_sign × (baseline − current)``. Returns None
            if the live position is unreadable. (``move_pump_uL`` applies
            ``pump_dir_sign`` but ``get_pump_position_uL`` does not, so the naive
            ``baseline − current`` only works when ``pump_dir_sign == +1``;
            ``getattr`` defaults the sign to +1 on controllers/fakes that don't
            expose it.)"""
            if baseline is None:
                return None
            try:
                cur = self.controller.get_pump_position_uL(bore)
            except Exception:
                cur = None
            if cur is None:
                return None
            try:
                sign = float(self.controller.pump_dir_sign(bore))
            except Exception:
                sign = 1.0
            return sign * (float(baseline) - float(cur))

        # 1. Waste — dispense residual so the needle empties.
        #
        # v7.5.x "reset to initial condition": the waste volume is computed LIVE
        # = (current plunger − baseline) [the unprinted ink + buffer that piled
        # up since the run started] + a small oil flush margin (pushes a little
        # oil past the tip so the last of the ink/buffer is expelled). The oil
        # step below then re-aspirates the margin to return the plunger to the
        # baseline (initial) position. Falls back to the fixed needle multiple
        # when reset mode is off or the live position is unreadable.
        goto("__waste__", "waste")
        waste_uL = None
        if self.cleanup_reset_to_initial and baseline is not None and unit >= 0:
            # Leftover above baseline as a DISPENSE volume (+ = push out): the
            # signed move that returns the plunger to baseline. + = a net
            # dispense (the unprinted ink + buffer to expel); <= 0 means the
            # plunger is already at/below baseline (nothing extra to waste).
            to_baseline = _vol_to_baseline()
            if to_baseline is not None:
                leftover = max(0.0, float(to_baseline))
                margin = max(0.0, float(self.cleanup_oil_margin_uL or 0.0))
                cap = (20.0 * unit) if unit > 0 else None
                cand = leftover + margin
                if cap is None or cand <= cap:
                    waste_uL = cand
                else:
                    logger.warning(
                        "Cleanup reset waste %.3f µL exceeds cap %.3f — using "
                        "fixed %g-needle fallback", cand, cap,
                        self.cleanup_waste_needles)
        if waste_uL is None:
            waste_uL = unit * self.cleanup_waste_needles
        if waste_uL > 1e-6:
            self._set_sub_step(
                clean_op, f"Cleanup: dispense {waste_uL:.3f} µL to waste")
            _settled_pump_move(self.controller, bore, +waste_uL, rate_uL_s=rate)
            self._check_abort()

        # 2. Wash.
        goto("__wash__", "wash")
        self._set_sub_step(clean_op, "Cleanup: wash needle")
        self._do_wash()

        # 3. Oil — reset the syringe oil level (return plunger to baseline).
        goto("__oil__", "oil")
        oil_uL = None
        if baseline is not None:
            # POLARITY-SAFE: the signed move_pump_uL volume that drives
            # get_pump_position_uL back to `baseline` — correct on either
            # pump_dir_sign (the old `baseline − current` omitted the
            # pump_dir_sign factor and only worked when pump_dir_sign == +1).
            reset = _vol_to_baseline()
            if reset is not None:
                cap = (20.0 * unit) if unit > 0 else None
                if cap is None or abs(reset) <= cap:
                    oil_uL = reset
                else:
                    logger.warning(
                        "Cleanup oil reset %.3f µL exceeds cap %.3f — using "
                        "fixed %g-needle fallback", reset, cap,
                        self.cleanup_oil_needles)
        if oil_uL is None:
            # Fixed fallback: ASPIRATE oil (aspirate = negative volume).
            oil_uL = -unit * self.cleanup_oil_needles
        if abs(oil_uL) > 1e-6:
            self._set_sub_step(clean_op, f"Cleanup: reset oil ({oil_uL:+.3f} µL)")
            _settled_pump_move(self.controller, bore, oil_uL, rate_uL_s=rate)

        self._set_sub_step(clean_op, "Cleanup complete")

    def _do_wash(self):
        """Agitate the needle at the wash well (already dipped to the service
        Z): jiggle Z up/down and nudge XY to random vectors about the well
        centre, then return to centre. Intra-well agitation — no safe-Z retract
        between jiggles (the within-well exemption). Abort-aware."""
        center = self._resolve_well_position("__wash__")
        if center is None:
            return
        cx, cy = float(center[0]), float(center[1])
        amp_um = max(0.0, float(self.wash_xy_amplitude_um))
        z_amp = max(0.0, float(self.wash_z_amplitude_mm))
        dwell = max(0.0, float(self.wash_dwell_s))
        ctrl = self.controller
        # Height-frame relative Z (+ = up, away from the plate) so the lift is
        # polarity-correct and never drives toward the plate first.
        has_user_z = hasattr(ctrl, "move_z_user_relative")
        # Return to the EXACT dip height with an ABSOLUTE move (not a symmetric
        # -z_amp), so a soft-limit-clamped lift can't let the tip walk toward
        # the plate over cycles. Falls back to a symmetric relative descend when
        # the dip Z is unknown or the controller lacks an absolute Z move.
        dip_z = self.service_z_mm
        can_return_abs = dip_z is not None and hasattr(ctrl, "move_z_absolute")

        for _ in range(max(0, int(self.wash_cycles))):
            self._check_abort()
            if z_amp > 0 and has_user_z:
                ctrl.move_z_user_relative(+z_amp)   # lift away from the plate
                if dwell:
                    time.sleep(dwell)
                if can_return_abs:
                    ctrl.move_z_absolute(dip_z, from_zero_ref=True)
                else:
                    ctrl.move_z_user_relative(-z_amp)
                if dwell:
                    time.sleep(dwell)
            if amp_um > 0:
                dx = random.uniform(-amp_um, amp_um)
                dy = random.uniform(-amp_um, amp_um)
                ctrl.move_xy_absolute_um(cx + dx, cy + dy)
                if dwell:
                    time.sleep(dwell)
        # Recentre over the wash well before the next prep step.
        ctrl.move_xy_absolute_um(cx, cy)
        zero = getattr(ctrl, "zero_position", {}) or {}
        zx = float(zero.get("x", 0.0))
        zy = float(zero.get("y", 0.0))
        if hasattr(ctrl, "wait_for_xy_arrival"):
            ctrl.wait_for_xy_arrival(
                (cx - zx) / 1000.0, (cy - zy) / 1000.0, timeout_s=self.xy_timeout_s)

    # ── Ink pickup (Quick Print: "pick the ink we will need") ─────

    def aspirate_ink(self, well_pos, volume_uL, *, bore, z_mm,
                     rate_uL_s=None):
        """Safe-travel to an ink reagent well and aspirate ``volume_uL``.

        Used by Quick Print's "pick the ink we will need for the print" step:
        retract → cross → lower to ``z_mm`` at the ink well (full safe-Z
        protocol), then aspirate ``volume_uL`` µL of ink into the needle.

        Args:
            well_pos: (x_um, y_um) ABSOLUTE stage µm of the ink reagent well.
            volume_uL: Volume to aspirate (µL, > 0 = draw fluid in). <= 0 is a no-op
                pump move (the travel still happens).
            bore: Pump/bore to aspirate with (e.g. "P1").
            z_mm: Dip Z at the ink well (zero-ref mm).
            rate_uL_s: Aspirate flow rate (µL/s); defaults to ``prep_rate_uL_s``.

        Abort-aware and ZP-down-guarded — delegates the move to ``_safe_move_to``
        (which raises :class:`AbortException` if the ZP board has dropped, so we
        never drag an unretracted needle across the plate). The ink well is
        ``__ink__`` so it is never the same well as the prep's ``__buffer__`` →
        always a full safe-Z travel. Does NOT retract afterward; the caller's
        ``finally`` handles end-at-safe-Z via :meth:`_retract_to_safe_z`.
        """
        self._check_abort()
        target = PickPlaceTarget(
            target_id="ink_well",
            x_um=float(well_pos[0]), y_um=float(well_pos[1]),
            well_name="__ink__",
        )
        self._safe_move_to(target, target_z_mm=z_mm)
        self._check_abort()
        vol = float(volume_uL or 0.0)
        if vol > 0:
            rate = rate_uL_s if rate_uL_s is not None else self.prep_rate_uL_s
            _settled_pump_move(self.controller, bore, -vol, rate_uL_s=rate)

    # ── Trypsin Cell Pickup ──────────────────────────────────────

    def _execute_trypsin_pickup(self, op: PickPlaceOperation):
        """Execute a trypsin cell pickup operation.

        Single bore mode:
          1. Load trypsin from trypsin well
          2. Navigate to target
          3. Dispense trypsin
          4. Wait dwell_time_s
          5. Aspirate extraction volume (trypsin + cells)
          6. Navigate to destination
          7. Dispense

        Dual bore mode:
          1. Load trypsin on trypsin_bore
          2. Navigate to target
          3. Dispense trypsin
          4. Wait dwell_time_s
          5. Aspirate on extraction_bore
          6. Navigate to destination
          7. Dispense from extraction_bore
        """
        cfg: TrypsinPickupConfig = op.config
        source = op.source_target
        dest = op.dest_target

        logger.info(
            f"Trypsin pickup: {source.target_id}, "
            f"{'single' if cfg.single_bore else 'dual'} bore, "
            f"dwell={cfg.dwell_time_s}s")

        # 1. Load trypsin from trypsin well (if specified)
        if cfg.trypsin_well:
            self._set_sub_step(op, f"Loading trypsin from {cfg.trypsin_well}")
            trypsin_target = PickPlaceTarget(
                target_id="trypsin_well",
                x_um=0, y_um=0,  # Will be resolved from well positions
                well_name=cfg.trypsin_well,
            )
            # Note: well position resolution happens at a higher level
            # For now, we use safe_travel_to with well coords from the executor's well map
            self._safe_move_to_well(cfg.trypsin_well)
            self._check_abort()

            # Aspirate trypsin
            _settled_pump_move(self.controller, cfg.trypsin_bore, -cfg.trypsin_volume_uL,
                                         rate_uL_s=cfg.push_speed_uL_s)
            self._check_abort()

        # 2. Navigate to target
        self._set_sub_step(op, f"Moving to target {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # 3. Dispense trypsin at target
        self._set_sub_step(op, "Dispensing trypsin")
        _settled_pump_move(self.controller, cfg.trypsin_bore, cfg.trypsin_volume_uL,
                                     rate_uL_s=cfg.push_speed_uL_s)
        self._check_abort()

        # 4. Dwell
        self._dwell(op, cfg.dwell_time_s, "Trypsin incubation")
        self._check_abort()

        # 5. Aspirate cells + trypsin
        extract_bore = cfg.extraction_bore if not cfg.single_bore else cfg.trypsin_bore
        self._set_sub_step(op, f"Extracting {cfg.extraction_volume_uL} µL")
        _settled_pump_move(self.controller, extract_bore, -cfg.extraction_volume_uL,
                                     rate_uL_s=cfg.pull_speed_uL_s)
        self._check_abort()

        # 6. Navigate to destination
        if dest:
            self._set_sub_step(op, f"Moving to dest {dest.target_id}")
            self._safe_move_to(dest)
            self._check_abort()

            # 7. Dispense at destination
            self._set_sub_step(op, "Dispensing cells")
            _settled_pump_move(self.controller, extract_bore, cfg.extraction_volume_uL,
                                         rate_uL_s=cfg.pull_speed_uL_s)
        elif cfg.dest_well:
            self._set_sub_step(op, f"Moving to dest well {cfg.dest_well}")
            self._safe_move_to_well(cfg.dest_well)
            self._check_abort()
            self._set_sub_step(op, "Dispensing cells")
            _settled_pump_move(self.controller, extract_bore, cfg.extraction_volume_uL,
                                         rate_uL_s=cfg.pull_speed_uL_s)

        self._set_sub_step(op, "Complete")

    # ── Fluorescent Tagging ──────────────────────────────────────

    def _execute_fluorescent_tagging(self, op: PickPlaceOperation):
        """Execute a fluorescent tagging operation.

        Single bore (no waste bore):
          1. Pickup dye from dye well
          2. Navigate to target
          3. Deposit dye
          4. Dwell
          5. Aspirate dye from target
          6. Waste → buffer → wash cycle
          7. Next cycle

        Multi bore, all dyes:
          1. Load each bore from its dye well
          2. Navigate to target
          3. Deposit each bore's dye
          4. Dwell
          5. Aspirate each bore
          6. Waste/wash/buffer
          7. Next target

        Waste bore mode:
          Same as multi bore but aspirate spent dye using waste bore
          (skip traveling to waste well until waste bore is full)
        """
        cfg: FluorescentTaggingConfig = op.config
        source = op.source_target

        logger.info(
            f"Fluorescent tagging: {source.target_id}, "
            f"{cfg.num_bores} bore(s), dwell={cfg.dwell_time_s}s, "
            f"waste_bore={cfg.use_waste_bore_mode}")

        # Load dye(s) from respective wells
        for dye in cfg.dye_configs:
            if dye.dye_well:
                self._set_sub_step(op, f"Loading {dye.dye_name} from {dye.dye_well}")
                self._safe_move_to_well(dye.dye_well)
                self._check_abort()

                _settled_pump_move(self.controller, dye.bore, -dye.volume_uL,
                                             rate_uL_s=1.0)
                self._check_abort()

        # Navigate to target
        self._set_sub_step(op, f"Moving to target {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # Deposit dye(s)
        for dye in cfg.dye_configs:
            self._set_sub_step(op, f"Depositing {dye.dye_name}")
            _settled_pump_move(self.controller, dye.bore, dye.volume_uL,
                                         rate_uL_s=1.0)
            self._check_abort()

        # Dwell
        self._dwell(op, cfg.dwell_time_s, "Dye incubation")
        self._check_abort()

        # Aspirate dye(s)
        if cfg.use_waste_bore_mode and cfg.waste_bore:
            # Use waste bore to aspirate spent dye (skip waste well travel)
            total_vol = sum(d.volume_uL for d in cfg.dye_configs)
            self._set_sub_step(op, f"Waste bore collecting {total_vol:.2f} µL")
            _settled_pump_move(self.controller, cfg.waste_bore, -total_vol,
                                         rate_uL_s=1.0)
        else:
            # Aspirate with each dye bore
            for dye in cfg.dye_configs:
                self._set_sub_step(op, f"Aspirating {dye.dye_name}")
                _settled_pump_move(self.controller, dye.bore, -dye.volume_uL,
                                             rate_uL_s=1.0)
                self._check_abort()

        # Service cycle: waste → buffer → wash
        if not cfg.use_waste_bore_mode:
            self._service_cycle(op, cfg)

        self._set_sub_step(op, "Complete")

    def _service_cycle(self, op: PickPlaceOperation, cfg: FluorescentTaggingConfig):
        """Perform waste → buffer → wash cycle for dye bores."""
        # Waste
        if cfg.waste_well:
            self._set_sub_step(op, f"Waste at {cfg.waste_well}")
            self._safe_move_to_well(cfg.waste_well)
            self._check_abort()
            for dye in cfg.dye_configs:
                _settled_pump_move(self.controller, dye.bore, dye.volume_uL,
                                             rate_uL_s=2.0)

        # Buffer
        if cfg.buffer_well:
            self._set_sub_step(op, f"Buffer at {cfg.buffer_well}")
            self._safe_move_to_well(cfg.buffer_well)
            self._check_abort()

        # Wash
        if cfg.wash_well:
            self._set_sub_step(op, f"Wash at {cfg.wash_well}")
            self._safe_move_to_well(cfg.wash_well)
            self._check_abort()

    # ── Movement helpers (Safe Z protocol) ───────────────────────

    def _safe_move_to(self, target: PickPlaceTarget, target_z_mm: float | None = None):
        """Move to a target with appropriate Z protocol.

        INTER-WELL: full safe Z protocol (raise → wait → XY → wait → lower)
        INTRA-WELL: small retract (1mm → wait → XY → wait → lower)

        ``target_z_mm`` (zero-ref mm) is the height to lower to after the XY
        move; None falls back to ``operating_z_mm``. The intra-well shortcut is
        used only when the target is in the *same named* well as the previous
        move — an empty well name (arbitrary clicked points, e.g. the spheroid
        live picker) always uses the full safe-Z travel, since we can't assume
        two un-named points are close enough for a 1 mm retract.
        """
        # SAFETY: a cross-position XY move must be preceded by a CONFIRMED Z
        # retract. If the ZP board has dropped off USB (the documented
        # mid-session CH340 disconnect) while the ProScan XY stays connected,
        # the needle can't be retracted and would be dragged across the plate
        # (the spheroid op has no final retract, so it's typically DOWN at the
        # previous pick/place height). StageController.safe_travel_to now also
        # refuses the unretracted XY itself when a needle is present, so this is
        # defense-in-depth — and it lets us abort the WHOLE run with a clear
        # reason rather than silently failing each move. Pick & place always
        # requires the ZP board (pumps + Z descent).
        if not getattr(self.controller, "is_zp_connected", True):
            raise AbortException(
                "ZP board not connected — refusing the XY move to avoid "
                "dragging an unretracted needle across the plate. Reconnect "
                "the Z/pump board and restart the operation.")

        z = self.operating_z_mm if target_z_mm is None else target_z_mm
        same_well = bool(target.well_name) and target.well_name == self._current_well
        if not same_well:
            # Inter-well: full safe Z
            self.controller.safe_travel_to(
                target_x_um=target.x_um,
                target_y_um=target.y_um,
                safe_z_mm=self.safe_z_mm,
                target_z_mm=z,
                z_timeout_s=self.z_timeout_s,
                xy_timeout_s=self.xy_timeout_s,
            )
        else:
            # Intra-well: small retract
            self._intra_well_move(target.x_um, target.y_um, target_z_mm=z)

        self._current_well = target.well_name

    def _safe_move_to_well(self, well_name: str, target_z_mm: float | None = None):
        """Move to a well center with full safe Z protocol.

        Well position must be resolved from well_positions dict. ``target_z_mm``
        (zero-ref mm) is the height to lower to after the XY move (e.g. the
        service dip Z); None falls back to ``operating_z_mm``. Returns True if
        the move was issued, False if the well could not be resolved.
        """
        pos = self._resolve_well_position(well_name)
        if pos is None:
            logger.warning(f"Cannot resolve well position for {well_name}")
            return False

        target = PickPlaceTarget(
            target_id=f"well_{well_name}",
            x_um=pos[0], y_um=pos[1],
            well_name=well_name,
        )
        self._safe_move_to(target, target_z_mm=target_z_mm)
        return True

    def _intra_well_move(self, target_x_um: float, target_y_um: float,
                         target_z_mm: float | None = None):
        """Move within the same well — small retract + wait + XY + wait + lower.

        Still follows the wait-for-Z-before-XY rule. ``target_z_mm`` (zero-ref
        mm) is the height to lower back to; None falls back to operating_z_mm.
        """
        z = self.operating_z_mm if target_z_mm is None else target_z_mm
        # 1. Retract Z by intra_well_retract_mm
        self.controller.move_z_relative(-self.intra_well_retract_mm)
        retracted_z = z - self.intra_well_retract_mm
        self.controller.wait_for_z_arrival(retracted_z,
                                           timeout_s=self.z_timeout_s)

        # 2. Move XY
        # v7.5.x bugfix: target_{x,y}_um are ABSOLUTE stage µm (same frame the
        # inter-well safe_travel_to uses). move_xy_absolute(from_zero_ref=True)
        # would treat them as mm (×1000 + zero) → gross mis-placement; use the
        # µm entry point.
        self.controller.move_xy_absolute_um(target_x_um, target_y_um)
        self.controller.wait_for_xy_arrival(
            target_x_um / 1000.0, target_y_um / 1000.0,
            timeout_s=self.xy_timeout_s)

        # 3. Lower Z back
        self.controller.move_z_relative(self.intra_well_retract_mm)
        self.controller.wait_for_z_arrival(z, timeout_s=self.z_timeout_s)

    # ── Well position resolution ─────────────────────────────────

    # Well positions are set externally by the GUI/calibration system
    _well_positions: dict[str, tuple[float, float]] = {}

    def set_well_positions(self, positions: dict[str, tuple[float, float]]):
        """Set the stage positions for named wells.

        Args:
            positions: Dict of well_name → (x_um, y_um).
        """
        self._well_positions = dict(positions)

    def _resolve_well_position(self, well_name: str) -> Optional[tuple[float, float]]:
        """Look up the stage position for a well."""
        # Check special service wells (absolute stage µm).
        if well_name == "__waste__" and self.waste_well_pos:
            return self.waste_well_pos
        if well_name == "__oil__" and self.oil_well_pos:
            return self.oil_well_pos
        if well_name == "__wash__" and self.wash_well_pos:
            return self.wash_well_pos
        if well_name == "__buffer__" and self.buffer_well_pos:
            return self.buffer_well_pos
        if well_name == "__reagent__" and self.reagent_well_pos:
            return self.reagent_well_pos
        return self._well_positions.get(well_name)

    # ── Utility ──────────────────────────────────────────────────

    def _set_sub_step(self, op: PickPlaceOperation, msg: str):
        """Update operation sub-step and notify callback."""
        op.sub_step = msg
        logger.debug(f"[{op.op_id}] {msg}")
        if self.on_sub_step:
            self.on_sub_step(op, msg)

    def _dwell(self, op: PickPlaceOperation, duration_s: float, label: str):
        """Wait for a specified duration with progress callbacks."""
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            remaining = duration_s - elapsed
            if remaining <= 0:
                break

            self._set_sub_step(op, f"{label}: {remaining:.0f}s remaining")
            if self.on_dwell_tick:
                self.on_dwell_tick(op, elapsed, duration_s)

            # Check abort/pause every 0.5s
            self._pause_event.wait()
            if self._abort_flag.is_set():
                raise AbortException("Aborted during dwell")

            time.sleep(min(0.5, remaining))

    def _check_abort(self):
        """Raise AbortException if abort was requested."""
        if self._abort_flag.is_set():
            raise AbortException("Aborted")

    def _uL_s_to_mm_min(self, speed_uL_s: float, bore: str) -> float:
        """Convert µL/s pump speed to mm/min feedrate.

        Uses HardwareConfig pump specs for the conversion.
        Falls back to a default ratio if hw_config is not available.
        """
        if self.hw_config:
            pump_cfg = self.hw_config.pumps.get(bore)
            if pump_cfg and pump_cfg.is_configured:
                try:
                    mm_per_uL = 1.0 / pump_cfg.uL_per_mm
                    mm_s = speed_uL_s * mm_per_uL
                    return mm_s * 60.0
                except (ValueError, ZeroDivisionError, AttributeError):
                    pass
        # Fallback: assume 1 µL ≈ 0.1 mm (rough estimate)
        return speed_uL_s * 0.1 * 60.0


class AbortException(Exception):
    """Raised when execution is aborted."""
    pass
