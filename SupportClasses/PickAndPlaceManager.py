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
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

logger = logging.getLogger(__name__)


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
    config: SpheroidPickupConfig | TrypsinPickupConfig | FluorescentTaggingConfig = field(
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
        self.operating_z_mm: float = 0.0   # Z height for operations
        self.intra_well_retract_mm: float = 1.0
        self.z_timeout_s: float = 15.0
        self.xy_timeout_s: float = 30.0

        # Service well positions (stage coords in µm)
        self.waste_well_pos: Optional[tuple[float, float]] = None
        self.wash_well_pos: Optional[tuple[float, float]] = None
        self.buffer_well_pos: Optional[tuple[float, float]] = None

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

        logger.info("PickPlaceExecutor: all operations complete")
        return True

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

        logger.info(
            f"Spheroid pickup: {source.target_id} → {dest.target_id if dest else 'N/A'}, "
            f"diameter={cfg.spheroid_diameter_um}µm, volume={volume_uL:.4f}µL")

        # 1. Move to source
        self._set_sub_step(op, f"Moving to source {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # 2. Aspirate
        feedrate = self._uL_s_to_mm_min(cfg.pickup_speed_uL_s, cfg.pickup_bore)
        self._set_sub_step(op, f"Aspirating {volume_uL:.4f} µL")
        self.controller.move_pump_uL(cfg.pickup_bore, -volume_uL,
                                     feedrate_mm_min=feedrate)
        self._check_abort()

        # 3. Move to destination
        if dest:
            self._set_sub_step(op, f"Moving to dest {dest.target_id}")
            self._safe_move_to(dest)
            self._check_abort()

            # 4. Dispense
            feedrate = self._uL_s_to_mm_min(cfg.release_speed_uL_s, cfg.pickup_bore)
            self._set_sub_step(op, f"Dispensing {volume_uL:.4f} µL")
            self.controller.move_pump_uL(cfg.pickup_bore, volume_uL,
                                         feedrate_mm_min=feedrate)

        self._set_sub_step(op, "Complete")

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
            feedrate = self._uL_s_to_mm_min(cfg.push_speed_uL_s, cfg.trypsin_bore)
            self.controller.move_pump_uL(cfg.trypsin_bore, -cfg.trypsin_volume_uL,
                                         feedrate_mm_min=feedrate)
            self._check_abort()

        # 2. Navigate to target
        self._set_sub_step(op, f"Moving to target {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # 3. Dispense trypsin at target
        self._set_sub_step(op, "Dispensing trypsin")
        feedrate = self._uL_s_to_mm_min(cfg.push_speed_uL_s, cfg.trypsin_bore)
        self.controller.move_pump_uL(cfg.trypsin_bore, cfg.trypsin_volume_uL,
                                     feedrate_mm_min=feedrate)
        self._check_abort()

        # 4. Dwell
        self._dwell(op, cfg.dwell_time_s, "Trypsin incubation")
        self._check_abort()

        # 5. Aspirate cells + trypsin
        extract_bore = cfg.extraction_bore if not cfg.single_bore else cfg.trypsin_bore
        self._set_sub_step(op, f"Extracting {cfg.extraction_volume_uL} µL")
        feedrate = self._uL_s_to_mm_min(cfg.pull_speed_uL_s, extract_bore)
        self.controller.move_pump_uL(extract_bore, -cfg.extraction_volume_uL,
                                     feedrate_mm_min=feedrate)
        self._check_abort()

        # 6. Navigate to destination
        if dest:
            self._set_sub_step(op, f"Moving to dest {dest.target_id}")
            self._safe_move_to(dest)
            self._check_abort()

            # 7. Dispense at destination
            self._set_sub_step(op, "Dispensing cells")
            self.controller.move_pump_uL(extract_bore, cfg.extraction_volume_uL,
                                         feedrate_mm_min=feedrate)
        elif cfg.dest_well:
            self._set_sub_step(op, f"Moving to dest well {cfg.dest_well}")
            self._safe_move_to_well(cfg.dest_well)
            self._check_abort()
            self._set_sub_step(op, "Dispensing cells")
            self.controller.move_pump_uL(extract_bore, cfg.extraction_volume_uL,
                                         feedrate_mm_min=feedrate)

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

                feedrate = self._uL_s_to_mm_min(1.0, dye.bore)
                self.controller.move_pump_uL(dye.bore, -dye.volume_uL,
                                             feedrate_mm_min=feedrate)
                self._check_abort()

        # Navigate to target
        self._set_sub_step(op, f"Moving to target {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # Deposit dye(s)
        for dye in cfg.dye_configs:
            self._set_sub_step(op, f"Depositing {dye.dye_name}")
            feedrate = self._uL_s_to_mm_min(1.0, dye.bore)
            self.controller.move_pump_uL(dye.bore, dye.volume_uL,
                                         feedrate_mm_min=feedrate)
            self._check_abort()

        # Dwell
        self._dwell(op, cfg.dwell_time_s, "Dye incubation")
        self._check_abort()

        # Aspirate dye(s)
        if cfg.use_waste_bore_mode and cfg.waste_bore:
            # Use waste bore to aspirate spent dye (skip waste well travel)
            total_vol = sum(d.volume_uL for d in cfg.dye_configs)
            self._set_sub_step(op, f"Waste bore collecting {total_vol:.2f} µL")
            feedrate = self._uL_s_to_mm_min(1.0, cfg.waste_bore)
            self.controller.move_pump_uL(cfg.waste_bore, -total_vol,
                                         feedrate_mm_min=feedrate)
        else:
            # Aspirate with each dye bore
            for dye in cfg.dye_configs:
                self._set_sub_step(op, f"Aspirating {dye.dye_name}")
                feedrate = self._uL_s_to_mm_min(1.0, dye.bore)
                self.controller.move_pump_uL(dye.bore, -dye.volume_uL,
                                             feedrate_mm_min=feedrate)
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
                feedrate = self._uL_s_to_mm_min(2.0, dye.bore)
                self.controller.move_pump_uL(dye.bore, dye.volume_uL,
                                             feedrate_mm_min=feedrate)

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

    def _safe_move_to(self, target: PickPlaceTarget):
        """Move to a target with appropriate Z protocol.

        INTER-WELL: full safe Z protocol (raise → wait → XY → wait → lower)
        INTRA-WELL: small retract (1mm → wait → XY → wait → lower)
        """
        if target.well_name != self._current_well:
            # Inter-well: full safe Z
            self.controller.safe_travel_to(
                target_x_um=target.x_um,
                target_y_um=target.y_um,
                safe_z_mm=self.safe_z_mm,
                target_z_mm=self.operating_z_mm,
                z_timeout_s=self.z_timeout_s,
                xy_timeout_s=self.xy_timeout_s,
            )
        else:
            # Intra-well: small retract
            self._intra_well_move(target.x_um, target.y_um)

        self._current_well = target.well_name

    def _safe_move_to_well(self, well_name: str):
        """Move to a well center with full safe Z protocol.

        Well position must be resolved from well_positions dict.
        """
        pos = self._resolve_well_position(well_name)
        if pos is None:
            logger.warning(f"Cannot resolve well position for {well_name}")
            return

        target = PickPlaceTarget(
            target_id=f"well_{well_name}",
            x_um=pos[0], y_um=pos[1],
            well_name=well_name,
        )
        self._safe_move_to(target)

    def _intra_well_move(self, target_x_um: float, target_y_um: float):
        """Move within the same well — small retract + wait + XY + wait + lower.

        Still follows the wait-for-Z-before-XY rule.
        """
        # 1. Retract Z by intra_well_retract_mm
        self.controller.move_z_relative(-self.intra_well_retract_mm)
        retracted_z = self.operating_z_mm - self.intra_well_retract_mm
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
        self.controller.wait_for_z_arrival(self.operating_z_mm,
                                           timeout_s=self.z_timeout_s)

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
        # Check special wells
        if well_name == "__waste__" and self.waste_well_pos:
            return self.waste_well_pos
        if well_name == "__wash__" and self.wash_well_pos:
            return self.wash_well_pos
        if well_name == "__buffer__" and self.buffer_well_pos:
            return self.buffer_well_pos
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
