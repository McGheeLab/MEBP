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
from dataclasses import dataclass, field, fields as _dc_fields
from enum import Enum
from types import SimpleNamespace
from typing import Callable, Optional

from SupportClasses.PhysicalModels import (
    BoreProfile, needle_orifice_area_mm2, needle_bore_at, needle_bore_count,
    needle_bore_offset_um, needle_bore_z_offset_mm,
    needle_bore_internal_volume_uL, needle_max_bore_z_offset_mm,
    MAX_BORE_OFFSET_UM, MAX_BORE_Z_OFFSET_MM,
)

logger = logging.getLogger(__name__)


def _settled_pump_move(ctrl, pump, volume_uL, rate_uL_s=None, *,
                       compensate=False, abort_event=None):
    """v7.5.x: discrete, blocking pump actuation with the controller's
    configured settle dwell (Hardware Setup → Pump). Every pick/place pump
    move is discrete (a clear next step follows), so it brackets the move
    with the settle time and blocks until the pump finishes.

    ``compensate`` controls v7.5.x backlash / compliance compensation (take-up
    on reversal + unload on stop, see :meth:`StageController.move_pump_uL`):

    * ``False`` (DEFAULT) — force it OFF. This is the safe default for the
      volume-balanced pick&place captures (spheroid / cell / trypsin / dye),
      whose nL net-zero balance a µL-scale compliance move would break, and for
      dispense-to-waste moves that never needed relief.
    * ``None`` — auto: comp iff the global backlash toggle is on. Pass this at
      reagent-load aspirates (ink / oil / buffer) that DO want the residual
      needle vacuum bled before the next inter-well travel.

    v7.6 ``abort_event``: forwarded so the blocking drain/settle waits return
    at once on an abort — this is what makes an Abort during a long reagent
    aspirate (up to ~180 s of M400 drain) responsive instead of finishing the
    whole move first.

    Falls back cleanly on older controllers / fakes that lack the ``settle`` /
    ``compensate`` / ``abort_event`` kwargs."""
    if abort_event is not None:
        try:
            ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate_uL_s,
                              settle=True, compensate=compensate,
                              abort_event=abort_event)
            return
        except TypeError:
            pass
    try:
        ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate_uL_s,
                          settle=True, compensate=compensate)
        return
    except TypeError:
        pass
    try:
        ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate_uL_s, settle=True)
        return
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
    # v7.13 — per-target pick/removal Z override (zero-ref mm), resolved by
    # the GUI from the measured SAMPLE SURFACE through the verified
    # focus↔needle datum. None = use the run-level pick_z_mm. The plate-bottom
    # floor armed for the queue remains the hard backstop underneath it.
    pick_z_zref_mm: Optional[float] = None

    def to_dict(self) -> dict:
        d = {
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
        # Conditional-emit: a target without an override serializes exactly
        # as before (byte-identity for existing stored targets).
        if self.pick_z_zref_mm is not None:
            d["pick_z_zref_mm"] = float(self.pick_z_zref_mm)
        return d

    @staticmethod
    def from_dict(d: dict) -> PickPlaceTarget:
        """Deserialize, ignoring unknown keys.

        v7.9: this was a bare ``PickPlaceTarget(**d)``, which raises TypeError on
        any key the running build does not know — so a target dict written by a
        NEWER build broke the load in an older one. That is exactly the
        forward-compat hazard ``NeedleSpec.from_dict`` was hardened against in
        v7.6, and it matters more now that v7.9 stamps per-bore and target-type
        metadata alongside targets.
        """
        d = dict(d or {})
        known = {f.name for f in _dc_fields(PickPlaceTarget)}
        unknown = sorted(set(d) - known)
        if unknown:
            logger.debug("PickPlaceTarget.from_dict: ignoring unknown key(s) %s",
                         unknown)
        return PickPlaceTarget(**{k: v for k, v in d.items() if k in known})


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


class BoreRole(Enum):
    """What ONE bore of a multi-bore needle assembly does during a run (v7.9).

    The operator's requirement was *"the user defines something for each pump
    channel to go and do something"* — resolved (decision D6) as exactly ONE
    role per bore plus that role's parameters, with the run ORDER derived from
    the roles rather than authored as a step list. A fixed role set keeps the
    table auditable and makes an unsafe program hard to express.
    """
    IDLE = "idle"                      # configured but unused this run
    ASPIRATE_TARGET = "aspirate_target"  # pull the cell up into this bore
    PUSH_REAGENT = "push_reagent"      # dose the target (e.g. the trypsin bore)
    DISPENSE_PLACE = "dispense_place"   # deliver what was aspirated

    @property
    def is_active(self) -> bool:
        return self is not BoreRole.IDLE


#: Order the roles act in within one operation. Dose the cell first, let it
#: release, then aspirate, then deliver — deriving this from the roles is what
#: keeps a per-bore table from becoming a program the operator can misorder.
BORE_ROLE_ORDER = (
    BoreRole.PUSH_REAGENT,
    BoreRole.ASPIRATE_TARGET,
    BoreRole.DISPENSE_PLACE,
)


@dataclass
class BoreProgram:
    """One row of the per-bore setup table: which bore, fed by which pump, doing
    what, to which class of object.

    ``target_type_id`` is a user-defined **target type** (see
    ``SupportClasses/TargetTypeStore``) — a class of object identified by its
    fluorescent signature, e.g. "bright in FITC only" or "bright in both FITC
    and mCherry". Per operator decision D3 the signature RULES are authored and
    persisted now, but **nothing evaluates them against an image yet**; the
    selection scheme is explicitly deferred. So today this field records intent
    and drives which picked targets a bore services.

    ``target_type_name``/``target_type_color`` are STAMPED copies, following
    ``NeedleTypeStore``'s stamp-don't-reference rule: editing or deleting a
    target type later must not mutate a saved run.
    """
    bore_index: int = 0
    pump_id: str = ""
    role: BoreRole = BoreRole.IDLE
    target_type_id: str = ""
    target_type_name: str = ""
    target_type_color: str = ""
    # Role parameters. Only the ones the role uses are read, so one flat set
    # keeps the table (and its persistence) simple.
    volume_uL: float = 0.0             # 0 ⇒ derive from the bore column × depth
    depth_mm: float = 0.10
    rate_uL_s: float = 0.5
    lead_time_s: float = 0.0           # PUSH_REAGENT: wait before the aspirate
    z_offset_mm: float = 0.10          # working height above the plate bottom

    def to_dict(self) -> dict:
        return {
            "bore_index": self.bore_index,
            "pump_id": self.pump_id,
            "role": self.role.value,
            "target_type_id": self.target_type_id,
            "target_type_name": self.target_type_name,
            "target_type_color": self.target_type_color,
            "volume_uL": self.volume_uL,
            "depth_mm": self.depth_mm,
            "rate_uL_s": self.rate_uL_s,
            "lead_time_s": self.lead_time_s,
            "z_offset_mm": self.z_offset_mm,
        }

    @classmethod
    def from_dict(cls, d: dict) -> BoreProgram:
        """Deserialize, ignoring unknown keys and an unrecognised role.

        An unknown role degrades to IDLE rather than raising: a bore whose
        purpose this build does not understand must do NOTHING, never guess.
        """
        d = dict(d or {})
        known = {f.name for f in _dc_fields(cls)}
        kwargs = {k: v for k, v in d.items() if k in known}
        raw_role = kwargs.pop("role", BoreRole.IDLE)
        if isinstance(raw_role, BoreRole):
            role = raw_role
        else:
            try:
                role = BoreRole(str(raw_role))
            except ValueError:
                logger.warning(
                    "BoreProgram.from_dict: unknown role %r — treating this bore "
                    "as IDLE so it performs no motion.", raw_role)
                role = BoreRole.IDLE
        return cls(role=role, **kwargs)


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

    # ── v7.5.x: disengagement extra-aspirate ──────────────────────────
    # A spheroid stuck to the glass sometimes needs extra suction to pop it off.
    # When enabled, an EXTRA aspirate of ``disengage_volume_uL`` at
    # ``disengage_rate_uL_s`` (usually faster) is applied as the FAST LEADING
    # portion of the total pickup aspirate. Default off (0 → no-op).
    disengage_enabled: bool = False
    disengage_volume_uL: float = 0.0
    disengage_rate_uL_s: float = 2.0

    # ── v7.5.x: sink-timing model ─────────────────────────────────────
    # Once aspirated, the spheroid rises into the bore then sinks back toward
    # the tip. When enabled AND a sink curve is calibrated (SpheroidSinkCalibration
    # Store) AND the needle bore area is known, the executor sizes the pickup
    # aspirate PER MOVE so the spheroid finishes sinking right as the needle
    # arrives at the destination (lift for the estimated travel time + margin),
    # floored at the sphere-capture volume. Default off → fixed carrier volume.
    sink_timing_enabled: bool = False
    travel_margin_s: float = 1.0     # arrive slightly under-sunk; waited out at dest

    # ── v7.5.x: minimal-excess release ────────────────────────────────
    # After the spheroid has sunk to the tip, dispense only a small
    # ``release_volume_uL`` (at ``release_speed_uL_s``) so minimal excess fluid is
    # deposited; the needle keeps the rest (accumulates — clear with Post-clean).
    # Default off → dispense the full aspirated volume (volume-balanced).
    release_enabled: bool = False
    release_volume_uL: float = 0.0

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
            "disengage_enabled": self.disengage_enabled,
            "disengage_volume_uL": self.disengage_volume_uL,
            "disengage_rate_uL_s": self.disengage_rate_uL_s,
            "sink_timing_enabled": self.sink_timing_enabled,
            "travel_margin_s": self.travel_margin_s,
            "release_enabled": self.release_enabled,
            "release_volume_uL": self.release_volume_uL,
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


@dataclass(frozen=True)
class LiveTuning:
    """The settings an operator may change WHILE a batch is running.

    Deliberately a frozen snapshot: it is published from the GUI thread and read
    by the executor thread, so it must be immutable once handed over — reading a
    ``QDoubleSpinBox`` from a worker thread is not safe, and passing a mutable
    holder would reintroduce the same race one level down.

    ``removal_z_zref_mm`` is already resolved to the zero-ref Z frame BY THE GUI,
    because only the GUI can consult the calibrated plate bottom and the Z
    polarity. The executor never has to resolve a plate frame.

    Every field is Optional: None means "leave whatever the run started with".
    """
    incubation_s: Optional[float] = None
    dose_volume_uL: Optional[float] = None
    release_depth_mm: Optional[float] = None
    extract_multiplier: Optional[float] = None
    pull_speed_uL_s: Optional[float] = None
    removal_z_zref_mm: Optional[float] = None

    def summary(self) -> str:
        """One line naming what this cell actually got — the per-cell record."""
        bits = []
        if self.dose_volume_uL is not None:
            nL = float(self.dose_volume_uL) * 1000.0
            bits.append(f"{nL:.3f} nL")
        if self.incubation_s is not None:
            bits.append(f"{float(self.incubation_s):.1f} s")
        if self.extract_multiplier is not None:
            bits.append(f"{float(self.extract_multiplier):.2f}×")
        if self.pull_speed_uL_s is not None:
            bits.append(f"{float(self.pull_speed_uL_s):.2f} µL/s")
        if self.removal_z_zref_mm is not None:
            bits.append(f"Z {float(self.removal_z_zref_mm):.3f}")
        return " · ".join(bits)


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

    # ── v7.9: multi-bore assembly + a DEDICATED trypsin bore ─────────────
    # Which bore of the assembly aspirates the cell. 0 = the calibrated datum
    # bore, which is what every single-bore needle resolves to — so the default
    # is byte-identical to the pre-v7.9 single-bore sequence.
    aspirate_bore_index: int = 0
    # A SEPARATE bore that pushes the cell-release reagent onto the cell just
    # before the aspirate (operator decision D5). Because fused bores are
    # laterally offset (~100-500 µm measured), the tool cannot have both bores
    # over the cell at once: the executor parks the TRYPSIN bore on the target,
    # pushes, then shifts XY so the ASPIRATE bore is on it, then pulls.
    # ⚠ That intervening move is why aspiration CANNOT overlap the push.
    # All default off/zero → the legacy single-bore path, unchanged.
    trypsin_enabled: bool = False
    trypsin_bore: str = ""               # pump id feeding the trypsin bore
    trypsin_bore_index: int = 1          # which bore of the assembly it is
    trypsin_well_key: str = "__trypsin__"   # service-well key for its reagent
    # Push VOLUME — either an explicit µL or a bore-column depth (that bore's
    # own orifice area × depth), mirroring release_depth_mm's model.
    trypsin_depth_mm: float = 0.10
    trypsin_volume_uL: float = 0.0       # >0 overrides the depth-derived volume
    # Push RATE (⇒ push duration = volume / rate) and the LEAD TIME between the
    # push finishing and the aspirate starting. Both are operator knobs (D2).
    trypsin_push_rate_uL_s: float = 0.5
    trypsin_lead_time_s: float = 0.0     # true no-op at 0.0 (_dwell returns at once)

    def resolve_bore(self, needle, bore_index: int):
        """The ``NeedleBore``-like object for one bore of the assembly.

        Falls back to the needle itself (a single-bore assembly IS bore 0), so
        every geometry read below works on a plain legacy NeedleSpec, on a
        multi-bore assembly, and on the MagicMock stubs the tests pass.
        """
        if needle is None:
            return None
        return needle_bore_at(needle, bore_index)

    def compute_trypsin_volume_uL(self, needle=None) -> float:
        """Trypsin push volume (µL) for the dedicated trypsin bore.

        An explicit ``trypsin_volume_uL`` wins; otherwise it is that bore's OWN
        orifice column (``orifice_area_mm² × trypsin_depth_mm``). Using the
        trypsin bore's own area — not the aspirating bore's — is the whole point
        of a backpack: the two bores have different diameters, so one area cannot
        size both moves.
        """
        if not self.trypsin_enabled:
            return 0.0
        if self.trypsin_volume_uL and float(self.trypsin_volume_uL) > 0:
            return float(self.trypsin_volume_uL)
        bore = self.resolve_bore(needle, self.trypsin_bore_index)
        if bore is not None:
            try:
                area = float(needle_orifice_area_mm2(bore))
                if area > 0:
                    return area * float(self.trypsin_depth_mm)
            except (TypeError, ValueError, AttributeError):
                pass
        return 0.0

    def compute_release_volume_uL(self, needle=None) -> float:
        """Push (reagent-release) volume in µL.

        When a needle is supplied, the volume is the bore column
        ``orifice_area_mm² × release_depth_mm`` (1 mm³ == 1 µL). Otherwise
        falls back to the pre-resolved ``release_volume_uL`` the GUI stamped in.

        v7.6: the reagent column is extruded through the ORIFICE into the well,
        so a pulled capillary uses its tip area. ⚠ That makes the volume tiny —
        0.1 mm through a 30 µm tip is ~0.07 nL, below one pump microstep — so
        the GUI warns rather than letting the move silently no-op.
        """
        bore = self.resolve_bore(needle, self.aspirate_bore_index)
        if bore is not None:
            try:
                area = float(needle_orifice_area_mm2(bore))
                if area > 0:
                    return area * float(self.release_depth_mm)
            except (TypeError, ValueError, AttributeError):
                pass
        return float(self.release_volume_uL or 0.0)

    def compute_extract_volume_uL(self, needle=None) -> float:
        """Fast-pull (extraction) volume = ``extract_multiplier × push``."""
        return self.compute_release_volume_uL(needle) * float(self.extract_multiplier)

    def active_bores(self) -> list[dict]:
        """The bores the EXECUTOR actually drives, in the order it drives them.

        ``[{"pump_id": str, "bore_index": int}, …]`` — the shape
        ``PickPlaceExecutor.prep_bores`` consumes, so needle prep and post-clean
        condition exactly the bores this run will use (operator decision D8).

        Derived from THIS CONFIG, deliberately not from the GUI's per-bore
        program table: that table also carries ``DISPENSE_PLACE`` rows, which the
        executor never drives (delivery happens through the aspirating bore), and
        prepping an undriven bore wastes oil and buffer and leaves it dripping
        into the plate.

        ⚠ Why this matters: without it ``prep_bores`` had no production writer at
        all, so a dedicated dosing bore was never conditioned and arrived at its
        reagent well full of AIR — it dosed nothing, the cell never released, and
        the run reported success with the sample lost.
        """
        out: list[dict] = []
        seen: set[str] = set()

        def add(pump, idx):
            pid = str(pump or "").strip().upper()
            if not pid or pid in seen:
                return
            seen.add(pid)
            out.append({"pump_id": pid, "bore_index": int(idx or 0)})

        # Dosing bore first — it is also the first the executor loads.
        if self.trypsin_enabled and self.trypsin_bore:
            add(self.trypsin_bore, self.trypsin_bore_index)
        add(self.reagent_bore, self.aspirate_bore_index)
        return out

    def with_tuning(self, tuning) -> "CellRemovalConfig":
        """A copy with the four TUNABLE fields replaced. Structure untouched.

        Deliberately narrow. Replacing the whole config mid-run would let a
        changed reagent, bore or well reach a needle already loaded with
        something else, and prep has already conditioned the bores this config
        named. Only the numbers an operator tunes while watching cells respond
        can change: incubation, dose, extraction multiplier and pull flow.

        The removal HEIGHT is not here — it is an executor attribute
        (``pick_z_mm``), already resolved to zero-ref Z by the GUI, and is
        applied alongside this by the caller.
        """
        if tuning is None:
            return self
        import dataclasses as _dc
        changes = {}
        for field, attr in (("dwell_time_s", "incubation_s"),
                            ("extract_multiplier", "extract_multiplier"),
                            ("pull_speed_uL_s", "pull_speed_uL_s")):
            v = getattr(tuning, attr, None)
            if isinstance(v, (int, float)):
                changes[field] = float(v)
        dose = getattr(tuning, "dose_volume_uL", None)
        if isinstance(dose, (int, float)) and dose > 0:
            # The DEPTH is what `compute_release_volume_uL` prefers whenever a
            # needle resolves, so a tuned dose has to arrive as a depth or it
            # would be silently ignored. The GUI resolves it against the same
            # aspirating bore the executor does.
            depth = getattr(tuning, "release_depth_mm", None)
            if isinstance(depth, (int, float)) and depth > 0:
                changes["release_depth_mm"] = float(depth)
            changes["release_volume_uL"] = float(dose)
        if not changes:
            return self
        return _dc.replace(self, **changes)

    def to_dict(self) -> dict:
        """Serialize. The nine v7.9 fields are CONDITIONALLY emitted.

        Conditional emit (matching ``NeedleSpec.to_dict``) keeps a single-bore
        config's block byte-identical to pre-v7.9 while still round-tripping a
        two-bore run — the previous version dropped all nine silently, so any
        future consumer would have turned a trypsin-bore run into a single-bore
        one with no error.
        """
        d = {
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
        if self.aspirate_bore_index:
            d["aspirate_bore_index"] = int(self.aspirate_bore_index)
        if self.trypsin_enabled:
            d.update({
                "trypsin_enabled": True,
                "trypsin_bore": self.trypsin_bore,
                "trypsin_bore_index": int(self.trypsin_bore_index or 0),
                "trypsin_well_key": self.trypsin_well_key,
                "trypsin_depth_mm": self.trypsin_depth_mm,
                "trypsin_volume_uL": self.trypsin_volume_uL,
                "trypsin_push_rate_uL_s": self.trypsin_push_rate_uL_s,
                "trypsin_lead_time_s": self.trypsin_lead_time_s,
            })
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "CellRemovalConfig":
        """Rebuild from :meth:`to_dict`, FILTERING unknown keys.

        Same forward-compat contract as ``BoreProgram.from_dict`` and
        ``PickPlaceTarget.from_dict``: a dict written by a newer build must load
        here rather than raising ``TypeError`` and taking the whole config with it.
        """
        known = {f.name for f in _dc_fields(cls)}
        clean = {k: v for k, v in (data or {}).items() if k in known}
        dropped = set((data or {}).keys()) - known
        if dropped:
            logger.debug("CellRemovalConfig.from_dict: ignoring unknown keys %s",
                         sorted(dropped))
        return cls(**clean)


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
        ``orifice_area_mm² × deposit_depth_mm`` (1 mm³ == 1 µL). Otherwise
        falls back to the pre-resolved ``deposit_volume_uL`` the GUI stamped in.

        v7.6: uses the ORIFICE area (the pulled tip when present) — see the
        note on :meth:`CellRemovalConfig.compute_release_volume_uL`.
        """
        if needle is not None:
            try:
                area = float(needle_orifice_area_mm2(needle))
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
    #: What the operator's live tuning actually set for THIS operation, stamped
    #: by ``PickPlaceExecutor._apply_live_tuning``. With mid-run edits allowed a
    #: batch is no longer one uniform experiment, so which cell got which
    #: treatment must be recorded rather than inferred afterwards.
    applied_tuning: Optional["LiveTuning"] = None

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

        # v7.5.x sink-timing model (spheroid pickup). ``bore_area_mm2`` is the
        # needle inner cross-section (set from the needle in the GUI) used for
        # volume↔lift-height (1 µL == 1 mm³). ``sink_curve`` is a calibrated
        # SinkCurve (SpheroidSinkCalibrationStore) or None → sink timing skipped.
        self.bore_area_mm2: float = 0.0
        self.sink_curve = None
        # v7.6: full bore geometry for volume↔lift. A pulled capillary is a
        # narrow tip stacked under a wide barrel, so the relation is PIECEWISE
        # — once the spheroid clears the tip it barely rises at all. When None,
        # ``bore_area_mm2`` is used as a single-area profile, which is
        # arithmetically identical to the pre-v7.6 behaviour.
        self.bore_profile: Optional[BoreProfile] = None
        # Fallbacks for the per-move travel-time estimate when the controller
        # can't report speeds (µm/s and mm/s).
        self._travel_xy_speed_fallback_um_s: float = 20000.0
        self._travel_z_speed_fallback_mm_s: float = 5.0
        self._travel_overhead_s: float = 1.0

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

        # ── Live tuning (v7.9) ────────────────────────────────────────
        # Optional callable returning a :class:`LiveTuning` (or None). Consulted
        # once per operation, at the top of the queue loop — see
        # ``_apply_live_tuning``. None ⇒ the run uses exactly the config it was
        # queued with, which is the pre-v7.9 behaviour.
        self.tuning_provider: Optional[Callable[[], Optional[LiveTuning]]] = None

        # ── Prep routine (run once before the pick & place loop) ──────
        # The needle is conditioned before picking: DISPENSE a needle of oil to
        # waste, ASPIRATE a needle of fresh oil, wash, then load buffer. All
        # volumes are in multiples of one needle's internal bore volume.
        self.do_prep: bool = False
        self.prep_bore: str = "P1"           # pump/bore used for prep aspirate/dispense
        # v7.9 — EVERY bore this run will use, conditioned SIMULTANEOUSLY.
        # Operator decision D8: "only the bores this run uses. but the should be
        # doing the same thing simultaneously". Empty ⇒ the legacy single-bore
        # path on ``prep_bore`` alone, byte-identical to pre-v7.9.
        # Each entry is {"pump_id": str, "bore_index": int}; the per-bore volume
        # is that bore's OWN internal volume, because "1 needle's worth" differs
        # between a backpack's two bores and one scalar cannot size both.
        self.prep_bores: list[dict] = []
        # Log-once latch: the clearance giveaway on a non-coplanar assembly is a
        # per-run fact, not a per-move one, and per-target logging would bury it.
        self._logged_clearance_giveaway: bool = False
        # Set while there is REAGENT ON A LIVE CELL (between the dose and the
        # recovery pull); cleared by the pull. Read after an aborted/failed run so
        # the operator is told to quench the well — the reagent keeps digesting
        # after Abort, and the remedy (flood with serum-containing medium) is only
        # available to someone who knows. Deliberately NOT used to trigger an
        # automatic recovery aspirate: see `pending_dose_warning`.
        self._pending_dose: dict | None = None
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

        # ── Wash after reagent pickup (before deposit) ───────────────
        # Cell removal / cell labeling: after aspirating the reagent (trypsin /
        # stain) from its well, rinse the needle EXTERIOR at the wash well before
        # travelling to deposit it. The aspirated volume stays in the bore (held
        # by suction) — this only washes off the reagent film clinging to the
        # outside so only the metered push/deposit column reaches the cells.
        # Reuses the wash well + wash config above. Off by default (the two GUI
        # pages default it ON).
        self.wash_after_pickup: bool = False

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

    def _apply_live_tuning(self, op) -> None:
        """Refresh this operation's tunable numbers from ``tuning_provider``.

        The provider is called from the EXECUTOR thread, so it must not touch Qt
        — the GUI publishes an immutable :class:`LiveTuning` snapshot under a lock
        and the provider just hands it over.

        What was applied is stamped onto ``op.applied_tuning`` so the run stays
        interpretable afterwards: with mid-run edits allowed, a batch is no longer
        one uniform experiment, and which cell got which treatment has to be
        knowable rather than inferred.
        """
        provider = getattr(self, "tuning_provider", None)
        if provider is None:
            return
        try:
            tuning = provider()
        except Exception as exc:
            logger.debug("tuning_provider failed: %s", exc)
            return
        if tuning is None:
            return
        cfg = getattr(op, "config", None)
        if cfg is not None and hasattr(cfg, "with_tuning"):
            try:
                op.config = cfg.with_tuning(tuning)
            except Exception as exc:
                logger.warning("could not apply live tuning: %s", exc)
                return
        z = getattr(tuning, "removal_z_zref_mm", None)
        if isinstance(z, (int, float)):
            # Goes through the SAME `pick_z_mm` every descend already plans
            # against, so `_descend_z_mm`'s clearance guarantee and the armed
            # plate-bottom floor apply to a tuned height unchanged.
            self.pick_z_mm = float(z)
        try:
            op.applied_tuning = tuning
        except Exception:
            pass
        logger.info("Live tuning applied to %s: %s",
                    getattr(op, "op_id", "?"), tuning.summary())

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
            # SAFETY: arm the plate-bottom floor for the whole run — the systemic
            # backstop under every Z move here, and the ONLY thing standing
            # between an unmeasured/mis-measured bore offset and the glass.
            #
            # Pick & place previously never armed it (only the print managers
            # did), so `move_z_absolute` was bounded solely by the raw envelope,
            # whose lower limit is the MECHANICAL needle-down datum — far below
            # the plate. Safe to arm here because every pick&place Z is authored
            # as a HEIGHT ABOVE THE PLATE BOTTOM with a spin minimum of 0.0, so
            # no legitimate below-bottom target exists, and the floor is a no-op
            # when the plate bottom is uncalibrated.
            #
            # ⚠ Scope limit: it clamps AT the bottom (zero clearance), so it
            # prevents cracking, not touching — a backstop, not a substitute for
            # `_descend_z_mm`. ⚠ Not refcounted (a plain bool on the controller),
            # which is exactly why arming is confined to this one method rather
            # than sprinkled into run_prep/aspirate_ink/run_print_cleanup, which
            # Quick Print calls directly and which would disarm early.
            self._set_plate_floor(True)

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

                # ── live tuning ──────────────────────────────────────────
                # Applied HERE and only here: the top of the loop is the one
                # point at which no operation is in flight, so nothing can
                # change under a needle that is already inside a well. It is a
                # pure data substitution — no pump or Z command is issued.
                self._apply_live_tuning(op)

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
            # Disarm AFTER the retract: the retract is raise-only, so the floor
            # cannot obstruct it, and leaving the floor armed until the tool is
            # clear costs nothing.
            self._set_plate_floor(False)

    def pending_dose_warning(self) -> str | None:
        """Operator-facing warning when a run stopped with reagent on a cell.

        ``None`` when the last run left nothing dosed. The GUI shows this after an
        abort or a failure, because "Stopped (aborted or failed)" does not tell
        the operator that a live cell is still being digested or that a bore is
        still loaded.

        ⚠ Why this is a MESSAGE and not an automatic recovery aspirate. Pulling
        the dose back would mean un-refusing the abort guard in
        ``move_pump_uL``/``move_pumps_uL`` — the mechanism that keeps Abort
        responsive during a 180 s drain, for EVERY workflow — on a code path that
        is frequently entered *because* of the very fault (clog, ZP drop, wrong
        plunger position) that makes a pump move unsafe. And the recovery would be
        marginal anyway: the reagent keeps acting during the aspirate, while the
        operator's real remedy is to flood the well with serum-containing medium,
        which beats any nanolitre pull-back. So: tell them, and give them a
        one-click Clean. Do not add motion to the abort path.
        """
        d = getattr(self, "_pending_dose", None)
        if not d:
            return None
        return (
            f"⚠ Stopped with reagent still on target {d.get('target_id', '?')}: "
            f"~{float(d.get('volume_uL', 0.0)):.5f} µL dispensed from bore "
            f"{d.get('bore', '?')} ({d.get('pump', '?')}) and never recovered. "
            f"It keeps acting on the cells — quench the well with "
            f"serum-containing medium now. The needle is also still loaded: run "
            f"Clean needle before the next run.")

    def _set_plate_floor(self, active: bool) -> None:
        """Arm/disarm the controller's plate-bottom Z floor. Never raises.

        Guarded exactly like ``PrintManager``'s equivalent so a partial test
        controller, an older controller, or a rig with no calibrated plate bottom
        all degrade to "no floor" rather than breaking the run.
        """
        try:
            fn = getattr(self.controller, "set_print_floor_active", None)
            if callable(fn):
                fn(bool(active))
        except Exception:
            logger.debug("Could not %s the plate-bottom floor",
                         "arm" if active else "disarm", exc_info=True)

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

    def _xy_travel_speed_um_s(self) -> float:
        """XY travel speed (µm/s) for the travel-time estimate: the measured
        top speed if calibrated, else the safety-limit max, else a fallback."""
        try:
            from SupportClasses.PrintTimingCalibrationStore import (
                get_store as _pt_store)
            v = _pt_store().get_xy_max_speed_um_s()
            if v and float(v) > 0:
                return float(v)
        except Exception:
            pass
        try:
            v = float(getattr(self.controller.safety_limits,
                              "max_xy_speed", 0) or 0)
            if v > 0:
                return v
        except Exception:
            pass
        return float(self._travel_xy_speed_fallback_um_s)

    def _z_travel_speed_mm_s(self) -> float:
        """Z travel speed (mm/s) for the travel-time estimate."""
        ctrl = self.controller
        for attr in ("_zp_retract_feedrate", "max_z_feedrate"):
            try:
                fr = getattr(ctrl, attr, None)
                if callable(fr):
                    fr = fr()
                if fr and float(fr) > 0:
                    return float(fr) / 60.0   # mm/min → mm/s
            except Exception:
                pass
        try:
            d = getattr(getattr(ctrl, "device_profile", None),
                        "per_axis_max_feedrate", None)
            if isinstance(d, dict) and d.get("Z"):
                return float(d["Z"]) / 60.0
        except Exception:
            pass
        return float(self._travel_z_speed_fallback_mm_s)

    def _estimate_travel_time_s(self, source, dest) -> float:
        """Rough per-move travel time (s): XY distance ÷ XY speed + Z
        retract+lower time + a fixed overhead. Divide-by-zero-safe."""
        try:
            dx = float(dest.x_um) - float(source.x_um)
            dy = float(dest.y_um) - float(source.y_um)
            xy_dist_um = math.hypot(dx, dy)
        except Exception:
            xy_dist_um = 0.0
        xy_t = xy_dist_um / max(self._xy_travel_speed_um_s(), 1.0)
        pick = self.pick_z_mm if self.pick_z_mm is not None else self.operating_z_mm
        place = self.place_z_mm if self.place_z_mm is not None else self.operating_z_mm
        z_v = max(self._z_travel_speed_mm_s(), 0.1)
        z_t = (abs(float(self.safe_z_mm) - float(pick))
               + abs(float(self.safe_z_mm) - float(place))) / z_v
        return xy_t + z_t + float(self._travel_overhead_s)

    def _bore(self) -> Optional[BoreProfile]:
        """Resolve the active bore geometry: the injected two-stage profile,
        else the legacy single-area scalar, else None when neither is usable.

        Returning None (rather than a zero-area profile) is what preserves the
        "no bore ⇒ fixed carrier volume, no sink wait" contract.
        """
        profile = self.bore_profile
        if profile is None:
            area = float(self.bore_area_mm2 or 0.0)
            profile = BoreProfile.from_area(area) if area > 0.0 else None
        return profile if (profile is not None and profile.is_usable()) else None

    def _lift_for_uL(self, volume_uL: float) -> float:
        """Axial rise (mm) for an aspirated volume — piecewise across the tip."""
        profile = self._bore()
        return profile.lift_for_volume(volume_uL) if profile else 0.0

    def _uL_for_lift(self, lift_mm: float) -> float:
        """Volume (µL) needed for an axial rise — piecewise across the tip."""
        profile = self._bore()
        return profile.volume_for_lift(lift_mm) if profile else 0.0

    def _planned_aspirate_uL(self, cfg, source, dest):
        """Return ``(total_aspirate_uL, remaining_sink_wait_s)`` for a spheroid
        move. With the sink-timing model on (and a calibrated curve + known bore
        geometry), size the aspirate so the spheroid finishes sinking as the
        needle arrives — floored at the sphere-capture volume; the arrival wait
        is the leftover sink time. Otherwise the fixed carrier volume with no
        wait."""
        carrier = cfg.compute_volume_uL()
        curve = self.sink_curve
        bore = self._bore()
        if (not getattr(cfg, "sink_timing_enabled", False)
                or curve is None or bore is None or dest is None):
            return carrier, 0.0
        travel = self._estimate_travel_time_s(source, dest)
        margin = max(0.0, float(getattr(cfg, "travel_margin_s", 0.0)))
        lift = curve.lift_for_time(travel + margin)   # mm
        timing_vol = self._uL_for_lift(lift)           # µL
        total = max(carrier, timing_vol)
        total_sink = curve.time_for_lift(self._lift_for_uL(total))
        remaining = max(0.0, total_sink - travel)
        return total, remaining

    def _execute_spheroid_pickup(self, op: PickPlaceOperation):
        """Execute a spheroid pickup operation.

        1. Navigate to source (safe Z protocol) and lower to the pick height.
        2. Aspirate the planned volume — with the disengagement extra applied as
           the fast leading portion when enabled.
        3. Safe-travel to the destination (the spheroid sinks during travel).
        4. Wait any remaining sink time so the spheroid reaches the tip.
        5. Dispense — a small release volume (minimal excess) or the full aspirate.
        """
        cfg: SpheroidPickupConfig = op.config
        source = op.source_target
        dest = op.dest_target

        # Plan the aspirate volume + arrival sink wait for THIS move.
        total_vol, wait_s = self._planned_aspirate_uL(cfg, source, dest)
        disengage_on = (getattr(cfg, "disengage_enabled", False)
                        and float(getattr(cfg, "disengage_volume_uL", 0.0)) > 0.0)
        diseng = float(getattr(cfg, "disengage_volume_uL", 0.0)) if disengage_on else 0.0
        aspirate_total = max(total_vol, diseng)
        # If the disengage pulse alone exceeds the planned volume, the spheroid is
        # lifted higher than planned → recompute the remaining sink wait for it.
        if (aspirate_total > total_vol and self.sink_curve is not None
                and self._bore() is not None
                and getattr(cfg, "sink_timing_enabled", False) and dest is not None):
            travel = self._estimate_travel_time_s(source, dest)
            wait_s = max(0.0, self.sink_curve.time_for_lift(
                self._lift_for_uL(aspirate_total)) - travel)

        # Pick / place use independent operating heights when supplied.
        pick_z = self.pick_z_mm if self.pick_z_mm is not None else self.operating_z_mm
        place_z = self.place_z_mm if self.place_z_mm is not None else self.operating_z_mm

        logger.info(
            f"Spheroid pickup: {source.target_id} → {dest.target_id if dest else 'N/A'}, "
            f"diameter={cfg.spheroid_diameter_um}µm, aspirate={aspirate_total:.4f}µL"
            f"{f' (disengage {diseng:.4f})' if disengage_on else ''}, "
            f"sink_wait={wait_s:.2f}s, pick_z={pick_z:.3f} place_z={place_z:.3f} mm")

        # 1. Move to source (lower to the pick height)
        self._set_sub_step(op, f"Moving to source {source.target_id}")
        self._safe_move_to(source, target_z_mm=pick_z)
        self._check_abort()

        # 2. Aspirate. move_pump_uL takes µL/s and clamps the flow rate itself;
        # NEGATIVE volume = aspirate. compensate=False (volume-balanced
        # micro-capture — no backlash comp, keep the exact nL balance).
        if disengage_on:
            lead = min(diseng, aspirate_total)
            self._set_sub_step(op, f"Disengage aspirate {lead:.4f} µL")
            self._pump_move(cfg.pickup_bore, -lead,
                               rate_uL_s=cfg.disengage_rate_uL_s)
            self._check_abort()
            rest = aspirate_total - lead
            if rest > 1e-9:
                self._set_sub_step(op, f"Aspirating {rest:.4f} µL")
                self._pump_move(cfg.pickup_bore, -rest,
                                   rate_uL_s=cfg.pickup_speed_uL_s)
                self._check_abort()
        else:
            self._set_sub_step(op, f"Aspirating {aspirate_total:.4f} µL")
            self._pump_move(cfg.pickup_bore, -aspirate_total,
                               rate_uL_s=cfg.pickup_speed_uL_s)
            self._check_abort()

        # 2b. Optional pick pause (let the spheroid settle into the bore).
        # NOTE: dwelling in the well risks losing a fast-sinking spheroid — the
        # sink-timing model makes this unnecessary; default 0.
        if getattr(cfg, "pick_dwell_s", 0.0):
            self._dwell(op, float(cfg.pick_dwell_s), "Pick pause")
            self._check_abort()

        # 3. Move to destination (lower to the place height). The spheroid sinks
        #    during this travel (the retract+XY+lower is the sink window).
        if dest:
            self._set_sub_step(op, f"Moving to dest {dest.target_id}")
            self._safe_move_to(dest, target_z_mm=place_z)
            self._check_abort()

            # 3b. Wait any remaining sink time so the spheroid reaches the tip.
            if wait_s and wait_s > 0:
                self._dwell(op, float(wait_s), "Sink to tip")
                self._check_abort()

            # 4. Dispense — a small release volume (minimal excess; needle keeps
            #    the rest) or the full aspirate (volume-balanced).
            release_on = (getattr(cfg, "release_enabled", False)
                          and float(getattr(cfg, "release_volume_uL", 0.0)) > 0.0)
            dispense_vol = (float(cfg.release_volume_uL) if release_on
                            else aspirate_total)
            self._set_sub_step(op, f"Dispensing {dispense_vol:.4f} µL")
            self._pump_move(cfg.pickup_bore, dispense_vol,
                               rate_uL_s=cfg.release_speed_uL_s)

            # 4b. Optional place pause (let the spheroid release from the bore).
            if getattr(cfg, "place_dwell_s", 0.0):
                self._check_abort()
                self._dwell(op, float(cfg.place_dwell_s), "Place pause")

        self._set_sub_step(op, "Complete")

    # ── Cell Targeting & Removal ─────────────────────────────────

    def _execute_cell_removal(self, op: PickPlaceOperation):
        """Execute one cell targeting & removal operation.

        There are TWO sequences, and which one runs depends on whether a
        dedicated dosing ("trypsin") bore is armed. The needle prep and the
        post-clean bracket the whole loop, run once each by ``execute_queue``.

        SINGLE-BORE (no dosing bore) — unchanged since v7.5.x::

          1. Load the cell-release reagent: safe-travel to the reagent well,
             draw the push volume.
          2. Travel to the removal location, lower to the removal Z.
          3. SLOWLY push the reagent column in.
          4. Wait the incubation time.
          5. QUICKLY pull ``extract_multiplier`` × the pushed volume.
          6. Travel to the placement, lower to the place Z.
          7. Gently dispense.

        TWO-BORE (a dosing bore is armed) — the dosing bore REPLACES step 3's
        push rather than adding to it (operator decision; before this fix the
        cell received both, up to 2× the intended reagent)::

          0. Load the dosing bore from its OWN reagent well.
          2a. Park the DOSING bore on the target, lower to the removal Z.
          2b. Push its dose.
          2c. Shift XY so the ASPIRATING bore sits on the dosed cell. This move
              is exactly why the aspirate cannot overlap the dose.
          2d. Lead time (ADDITIVE to the incubation, not a replacement).
          4.  Incubate.
          5.  Pull the cell up with the aspirating bore — which loaded NOTHING;
              its "push volume" now only sizes this pull.
          6-7. Travel to the placement and dispense.

        The pump is volume-balanced PER BORE over the op, so neither drifts.
        Heights come from the GUI: ``pick_z_mm`` = the removal height,
        ``place_z_mm`` = the place height (both zero-ref mm; None falls back to
        ``operating_z_mm``). Every descend goes through :meth:`_descend_z_mm`, so
        the LOWEST-reaching bore clears the plate, not just the bore being placed.
        """
        cfg: CellRemovalConfig = op.config
        source = op.source_target
        dest = op.dest_target
        bore = cfg.reagent_bore

        needle = self._needle()
        push_uL = cfg.compute_release_volume_uL(needle)
        pull_uL = push_uL * float(cfg.extract_multiplier)

        # v7.9: an optional DEDICATED trypsin bore, on a different pump, doses
        # the cell just before the aspirate. Disabled ⇒ every branch below is
        # skipped and the sequence is byte-identical to the single-bore version.
        asp_bore_idx = int(cfg.aspirate_bore_index or 0)
        tryp_on = bool(cfg.trypsin_enabled) and bool(cfg.trypsin_bore)
        tryp_bore = cfg.trypsin_bore
        tryp_bore_idx = int(cfg.trypsin_bore_index or 0)
        tryp_uL = cfg.compute_trypsin_volume_uL(needle) if tryp_on else 0.0
        if tryp_on and tryp_uL <= 0:
            # A configured-but-unsized trypsin push would be a silent no-op that
            # still costs two travels and a lead-time wait. Say so.
            logger.warning(
                "Cell removal: trypsin bore %s is enabled but its resolved push "
                "volume is 0 µL (depth %.4f mm through bore %d) — skipping the "
                "trypsin dose.", tryp_bore, cfg.trypsin_depth_mm, tryp_bore_idx + 1)
            tryp_on = False

        # ── REFUSE an unmeasured two-bore assembly BEFORE dosing anything ──
        # If the dosing and aspirating bores are physically different but their
        # measured offsets are identical, `_shift_to_bore` moves NOTHING and the
        # executor then aspirates believing it is over the cell while the
        # aspirating bore is 100-500 µm away. The dose happens, the cell releases,
        # and nothing is collected — every target in the run destroyed and none
        # recovered, with no error anywhere.
        #
        # The check MUST sit here, before step 0's load: by the time
        # `_shift_to_bore` returns False the cell has already been dosed.
        if tryp_on and tryp_bore_idx != asp_bore_idx:
            src_off = self._bore_offset_um(tryp_bore_idx)
            dst_off = self._bore_offset_um(asp_bore_idx)
            if (abs(src_off[0] - dst_off[0]) < 1e-6
                    and abs(src_off[1] - dst_off[1]) < 1e-6):
                raise AbortException(
                    f"Bore {tryp_bore_idx + 1} (dosing) and bore "
                    f"{asp_bore_idx + 1} (aspirating) report the SAME mount "
                    f"offset, so their measured separation is zero — the mount "
                    f"offsets have not been measured for this assembly. Running "
                    f"would dose every target and then aspirate 100-500 µm away "
                    f"from it, destroying the cells without collecting any. "
                    f"Measure the bore offsets on Calibration → Needle Location "
                    f"first (they must be re-measured after every needle change "
                    f"or re-seat).")

        # Pick / place use independent operating heights when supplied.
        removal_z = self.pick_z_mm if self.pick_z_mm is not None else self.operating_z_mm
        place_z = self.place_z_mm if self.place_z_mm is not None else self.operating_z_mm

        # v7.13 — per-target sample-surface removal Z. Resolved by the GUI at
        # queue build (measured surface + verified focus↔needle datum + the
        # operator's offset), already gated page-side: low-confidence targets
        # arrive with None and use the run-level height. The plate-bottom
        # floor armed for this queue stays the hard backstop.
        target_removal_z = getattr(source, "pick_z_zref_mm", None)
        if target_removal_z is not None:
            try:
                removal_z = float(target_removal_z)
                logger.info(
                    "Cell removal: %s uses sample-surface removal Z "
                    "%.3f mm (zref)", source.target_id, removal_z)
            except (TypeError, ValueError):
                pass

        logger.info(
            "Cell removal: %s → %s, push=%.5f µL (slow %.2f µL/s), pull=%.5f µL "
            "(fast %.2f µL/s), dwell=%.0fs, removal_z=%.3f place_z=%.3f mm, "
            "aspirate bore %d%s",
            source.target_id, dest.target_id if dest else "N/A",
            push_uL, cfg.push_speed_uL_s, pull_uL, cfg.pull_speed_uL_s,
            cfg.dwell_time_s, removal_z, place_z, asp_bore_idx + 1,
            (f", trypsin bore {tryp_bore_idx + 1} ({tryp_bore}) "
             f"{tryp_uL:.5f} µL @ {cfg.trypsin_push_rate_uL_s:.2f} µL/s, "
             f"lead {cfg.trypsin_lead_time_s:.1f}s") if tryp_on else "")

        # ── 0. Load the TRYPSIN bore from its own reagent well ───────────
        # Done before the aspirating bore's reagent load so the assembly makes a
        # single pass over the service wells.
        if tryp_on:
            self._set_sub_step(
                op, f"Loading {tryp_uL:.4f} µL of trypsin into bore "
                    f"{tryp_bore_idx + 1}")
            if not self._safe_move_to_well(cfg.trypsin_well_key,
                                           target_z_mm=self.reagent_dip_z_mm,
                                           bore_index=tryp_bore_idx):
                raise RuntimeError(
                    f"Cell removal: trypsin well ({cfg.trypsin_well_key}) not "
                    f"configured/resolved — refusing to run a trypsin bore with "
                    f"nothing loaded.")
            self._check_abort()
            self._pump_move(tryp_bore, -tryp_uL,
                            rate_uL_s=cfg.trypsin_push_rate_uL_s)
            self._check_abort()

        # 1. Load the cell-release reagent from its well.
        # v7.9 BUGFIX: the load was gated on `reagent_well_pos is not None` while
        # the push at step 3 was gated only on `push_uL > 0`, so a run with no
        # reagent well DISPENSED a volume it had never aspirated — an unbalanced
        # pump and reagent that does not exist. `loaded` now gates both.
        #
        # v7.9 (post-audit): a DEDICATED dosing bore REPLACES this push — it does
        # not add to it. Operator decision: "Trypsin bore only". Before this the
        # cell received tryp_uL from the dosing bore AND push_uL from the
        # aspirating bore — up to 2× the intended reagent, half of it delivered
        # after the shift, out of the same orifice about to pull the cell in.
        # Cells were over-digested and the un-recovered excess kept digesting
        # neighbours.
        #
        # The gate is on the LOAD, deliberately, not on the push at step 3:
        # skipping only the push would leave the aspirating bore holding push_uL
        # it never dispenses, unbalancing the pump by that much on EVERY target.
        # Gating the load makes `loaded` False, so step 3 skips itself, the pump
        # stays balanced, and one service-well round trip per target disappears.
        loaded = False
        if self.reagent_well_pos is not None and push_uL > 0 and not tryp_on:
            self._set_sub_step(
                op, f"Loading {push_uL:.4f} µL of cell-release reagent")
            if not self._safe_move_to_well(
                    "__reagent__", target_z_mm=self.reagent_dip_z_mm,
                    bore_index=asp_bore_idx):
                raise RuntimeError(
                    "Cell removal: reagent well not configured/resolved")
            self._check_abort()
            self._pump_move(bore, -push_uL,
                                         rate_uL_s=cfg.push_speed_uL_s)
            self._check_abort()
            loaded = True
        elif tryp_on:
            # Not a problem: the dedicated bore owns the dose. The aspirating
            # bore's column now only SIZES the extraction pull.
            logger.info(
                "Cell removal: bore %d doses the reagent, so the aspirating bore "
                "loads nothing — it only pulls %.5f µL (%.2f× the %.5f µL "
                "column) up off the target.",
                tryp_bore_idx + 1, pull_uL, cfg.extract_multiplier, push_uL)
        elif push_uL > 0:
            logger.warning(
                "Cell removal: no reagent well resolved — skipping the %.5f µL "
                "reagent push so the pump stays volume-balanced.", push_uL)

        # 1b. Rinse the needle exterior (wash off the reagent film) before we
        #     go deposit it — the aspirated reagent stays in the bore.
        if self.wash_after_pickup:
            self._wash_needle(op, "Washing needle after reagent pickup")

        # ── 2. Dose the cell with the dedicated trypsin bore ─────────────
        # The TRYPSIN bore is parked on the target first; the aspirating bore is
        # laterally offset and comes over the cell in step 2c.
        if tryp_on:
            self._set_sub_step(
                op, f"Moving bore {tryp_bore_idx + 1} to target {source.target_id}")
            self._safe_move_to(source, target_z_mm=removal_z,
                               bore_index=tryp_bore_idx)
            self._check_abort()

            self._set_sub_step(op, f"Pushing {tryp_uL:.4f} µL of trypsin")
            self._pump_move(tryp_bore, +tryp_uL,
                            rate_uL_s=cfg.trypsin_push_rate_uL_s)
            # From here until the pull, there is REAGENT ON A LIVE CELL. If the
            # run stops in this window the reagent keeps digesting, so record it
            # for the abort report — the operator's remedy (flood the well with
            # serum-containing medium) is only available if they know.
            self._pending_dose = {
                "target_id": getattr(source, "target_id", "?"),
                "bore": tryp_bore_idx + 1,
                "pump": tryp_bore,
                "volume_uL": tryp_uL,
            }
            self._check_abort()

            # 2c. Shift so the ASPIRATING bore is on the dosed cell. This move is
            #     precisely why the aspirate cannot overlap the push (D5).
            self._shift_to_bore(op, source, tryp_bore_idx, asp_bore_idx,
                                removal_z)
            self._check_abort()

            # 2d. Lead time — the trypsin acts while the tool is already in
            #     position. A true no-op at 0.0.
            #
            # ADDITIVE, by operator decision: the total dose→aspirate interval is
            # this lead time PLUS the incubation at step 4. Making it *replace*
            # the incubation would silently turn the 60 s default into 0 s — a
            # change to how long cells are digested, which is a protocol change
            # and not ours to make. The true total is logged because "lead time"
            # alone is a misleading name for the interval the operator cares about.
            lead_s = float(cfg.trypsin_lead_time_s or 0.0)
            total_s = lead_s + float(cfg.dwell_time_s or 0.0)
            logger.info(
                "Cell removal: dose → aspirate interval is %.1f s "
                "(%.1f s lead + %.1f s incubation).",
                total_s, lead_s, float(cfg.dwell_time_s or 0.0))
            self._dwell(op, lead_s, "Trypsin lead time")
            self._check_abort()
        else:
            # 2. Travel to the cell-removal location (lower to the removal Z).
            self._set_sub_step(op, f"Moving to removal target {source.target_id}")
            self._safe_move_to(source, target_z_mm=removal_z,
                               bore_index=asp_bore_idx)
            self._check_abort()

        # 3. Slowly push the reagent in.
        if loaded:
            self._set_sub_step(op, f"Releasing {push_uL:.4f} µL (slow)")
            self._pump_move(bore, +push_uL,
                                         rate_uL_s=cfg.push_speed_uL_s)
            # Single-bore path: reagent is now on the cell too.
            self._pending_dose = {
                "target_id": getattr(source, "target_id", "?"),
                "bore": asp_bore_idx + 1,
                "pump": bore,
                "volume_uL": push_uL,
            }
            self._check_abort()

        # 4. Incubate.
        self._dwell(op, cfg.dwell_time_s, "Cell-release incubation")
        self._check_abort()

        # 5. Quickly pull up the extraction volume (reagent + cells).
        if pull_uL > 0:
            self._set_sub_step(op, f"Extracting {pull_uL:.4f} µL (fast)")
            self._pump_move(bore, -pull_uL,
                                         rate_uL_s=cfg.pull_speed_uL_s)
            # The dose has been recovered; nothing is left digesting.
            self._pending_dose = None
            self._check_abort()

        # 6. Travel to the placing location (lower to the place Z).
        if dest:
            self._set_sub_step(op, f"Moving to placement {dest.target_id}")
            self._safe_move_to(dest, target_z_mm=place_z,
                               bore_index=asp_bore_idx)
            self._check_abort()

            # 7. Gently dispense the extracted cells.
            if pull_uL > 0:
                self._set_sub_step(op, f"Dispensing {pull_uL:.4f} µL")
                self._pump_move(bore, +pull_uL,
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
            self._pump_move(bore, -deposit_uL,
                               rate_uL_s=cfg.aspirate_speed_uL_s)
            self._check_abort()

        # 1b. Rinse the needle exterior (wash off the stain film) before we go
        #     deposit it — the aspirated stain stays in the bore.
        if self.wash_after_pickup:
            self._wash_needle(op, "Washing needle after stain pickup")

        # 2. Travel to the stain region (lower to the label Z).
        self._set_sub_step(op, f"Moving to stain region {source.target_id}")
        self._safe_move_to(source, target_z_mm=label_z)
        self._check_abort()

        # 3. Slowly deposit the stain.
        if deposit_uL > 0:
            self._set_sub_step(op, f"Depositing {deposit_uL:.4f} µL (slow)")
            self._pump_move(bore, +deposit_uL,
                               rate_uL_s=cfg.deposit_speed_uL_s)
            self._check_abort()

        # 4. Incubate — the headline knob (how long the stain develops).
        self._dwell(op, cfg.stain_dwell_time_s, "Stain incubation")
        self._check_abort()

        # 5. Slowly aspirate the stain (+ excess) back up.
        if aspirate_uL > 0:
            self._set_sub_step(op, f"Aspirating {aspirate_uL:.4f} µL (slow)")
            self._pump_move(bore, -aspirate_uL,
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
            self._pump_move(bore, +aspirate_uL,
                               rate_uL_s=cfg.deposit_speed_uL_s)

        self._set_sub_step(op, "Complete")

    # ── Prep routine (needle conditioning before the pick & place loop) ──

    # ── Simultaneous multi-bore prep (v7.9, decision D8) ─────────────

    def _prep_bore_plan(self) -> list[tuple[str, int, float]]:
        """``[(pump_id, bore_index, one_bore_volume_uL)]`` for the bores to prep.

        Empty ⇒ the caller uses the legacy single-``prep_bore`` path unchanged.

        The volume is resolved PER BORE from that bore's own geometry: on a
        backpack "1 needle's worth" of a 22G bore is ~6.8 µL and of a 30G bore
        ~1.0 µL, so the single ``needle_volume_uL`` scalar cannot size both.
        It falls back to that scalar for a bore whose geometry cannot be read.
        """
        entries = list(getattr(self, "prep_bores", None) or ())
        if not entries:
            return []
        needle = self._needle()
        fallback = float(self.needle_volume_uL or 0.0)
        plan: list[tuple[str, int, float]] = []
        seen: set[str] = set()
        for e in entries:
            pump = str((e or {}).get("pump_id") or "").strip().upper()
            if not pump or pump in seen:
                continue          # a pump cannot be driven twice in one move
            seen.add(pump)
            idx = int((e or {}).get("bore_index") or 0)
            vol = 0.0
            if needle is not None:
                try:
                    vol = float(needle_bore_internal_volume_uL(needle, idx))
                except (TypeError, ValueError):
                    vol = 0.0
            plan.append((pump, idx, vol if vol > 0 else fallback))

        # ── BYTE-IDENTITY GUARANTEE — the load-bearing line ──────────────
        # A plan of exactly ONE bore that is already `prep_bore` describes the
        # legacy single-bore prep, so return [] and let the caller take the
        # legacy path verbatim. Without this collapse a single-bore run would be
        # silently re-routed through the COORDINATED path, which cannot carry
        # `compensate=None` (one G0 has one vector feedrate and no per-axis
        # compliance compensation) — so the oil and buffer aspirates would lose
        # their backlash take-up, changing fluidics on every existing setup for
        # no benefit. The docstring above has always promised this; now it is true.
        if len(plan) == 1:
            legacy = str(getattr(self, "prep_bore", "") or "").strip().upper()
            if plan[0][0] == legacy:
                return []
        return plan

    def _prep_pump_move(self, plan, needles: float, sign: float, rate,
                        *, compensate=False) -> None:
        """Actuate every prepping bore at once: ONE coordinated Marlin move.

        ``sign`` is ``+1`` to DISPENSE and ``-1`` to ASPIRATE (the
        :meth:`move_pump_uL` convention). All bores do the SAME thing here, which
        is what makes a coordinated move legitimate: a single ``G0`` carries one
        VECTOR feedrate, so the bores cannot have independent rates — fine when
        they are all aspirating buffer, and exactly why the slow-push/fast-pull
        asymmetry of the removal sequence stays sequential.

        Falls back to sequential per-bore moves when the controller predates
        ``move_pumps_uL`` (older stand-ins and the test fakes), so behaviour
        degrades to "correct but not simultaneous" rather than failing.

        ⚠ FAILS LOUDLY. An unconditioned bore reaches its reagent well full of
        AIR: it aspirates a few nanolitres against a multi-µL compressible
        column and then delivers approximately nothing onto the cell. The cell
        never releases, the aspirate collects nothing, and the run reports
        SUCCESS with the sample lost. So both failure modes are raised on:

        * the move was REFUSED (``False`` — no ZP board, or an abort in flight);
        * the move "succeeded" having delivered nothing (every delta clamped
          below one Marlin step), which the ``bool`` cannot express — hence the
          ``delivered`` out-param.
        """
        volumes = {pump: sign * vol * float(needles)
                   for (pump, _idx, vol) in plan if vol > 0}
        if not volumes:
            return
        mover = getattr(self.controller, "move_pumps_uL", None)
        if callable(mover):
            # Seeded with a sentinel the real implementation clears. A controller
            # that accepts `delivered` through **kwargs and ignores it would
            # otherwise leave an EMPTY dict, indistinguishable from "nothing was
            # delivered" — turning a compatibility gap into a false alarm on
            # every prep. If the sentinel survives, delivery is simply unknown.
            _UNREPORTED = "__mebp_unreported__"
            got: dict = {_UNREPORTED: 1.0}
            try:
                ok = mover(volumes, rate_uL_s=rate, settle=True,
                           abort_event=self._abort_flag, delivered=got)
                if _UNREPORTED in got:
                    got = None       # controller does not report delivery
            except TypeError:
                # Older signature (no `delivered`, or older still). Retry without
                # it before giving up, so an older controller degrades to
                # "correct but unverified" rather than falling all the way back
                # to sequential moves — which would silently lose simultaneity.
                try:
                    ok = mover(volumes, rate_uL_s=rate, settle=True,
                               abort_event=self._abort_flag)
                    got = None      # this controller cannot report delivery
                except TypeError:
                    ok = None       # much older signature — sequential fallback
            if ok is not None:
                if not ok:
                    if self._abort_flag.is_set():
                        raise AbortException(
                            "Prep aborted before the bores were conditioned.")
                    raise RuntimeError(
                        "Prep: the coordinated bore move was refused (Z/pump "
                        "board not connected?) — refusing to continue with "
                        "unconditioned bores, which would dose air onto the "
                        "targets.")
                if got is not None:
                    missing = [p for p, want in volumes.items()
                               if abs(got.get(p, 0.0)) < abs(want) * 0.5]
                    if missing:
                        raise RuntimeError(
                            "Prep: "
                            + ", ".join(
                                f"{p} requested {volumes[p]:+.5f} µL but only "
                                f"{got.get(p, 0.0):+.5f} µL was delivered"
                                for p in missing)
                            + " — the bore(s) are not conditioned (a soft-limit "
                              "clamp or a sub-microstep volume). Check the "
                              "plunger position and the syringe envelope before "
                              "running; an unconditioned bore doses air.")
                return
        for pump, dv in volumes.items():
            # ⚠ The sequential fallback has NO delivery signal at all —
            # move_pump_uL returns None. Verifying it needs a plunger-position
            # read, which is out of scope here; noted so it is not mistaken for
            # coverage.
            self._pump_move(pump, dv, rate_uL_s=rate, compensate=compensate)

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

        # v7.9 (D8): condition EVERY bore this run uses, acting simultaneously.
        # An empty plan is the legacy single-bore path, byte-identical.
        plan = self._prep_bore_plan()
        multi = len(plan) > 1
        if plan:
            logger.info(
                "Prep conditioning %d bore(s) %s: %s",
                len(plan),
                "simultaneously" if multi else "",
                ", ".join(f"bore {i + 1}/{p} @ {v:.4f} µL" for (p, i, v) in plan))

        def goto(key, label):
            self._check_abort()
            self._set_sub_step(prep_op, f"Prep: travel to {label}")
            # All bores dip in the SAME service well. They are laterally offset
            # by ~100-500 µm, which is negligible against a service well several
            # mm across, so the assembly is positioned by its datum bore.
            if not self._safe_move_to_well(key, target_z_mm=sz):
                raise RuntimeError(
                    f"Prep: {label} well not configured/resolved ({key})")

        def actuate(needles, sign, label, compensate=False):
            """One prep actuation across every prepping bore."""
            if needles <= 0:
                return
            if plan:
                self._set_sub_step(prep_op, label)
                self._prep_bore_plan_guard(plan)
                self._prep_pump_move(plan, needles, sign, rate,
                                     compensate=compensate)
            elif unit > 0:
                self._set_sub_step(prep_op, label)
                if compensate is False:
                    self._pump_move(bore, sign * unit * needles, rate_uL_s=rate)
                else:
                    _settled_pump_move(self.controller, bore,
                                       sign * unit * needles, rate_uL_s=rate,
                                       compensate=compensate)
            self._check_abort()

        # 1. Waste — dispense oil.
        goto("__waste__", "waste")
        actuate(self.oil_needles, +1.0,
                f"Prep: dispense {self.oil_needles:g} needle(s) of oil to waste")

        # 2. Oil — aspirate fresh oil.
        goto("__oil__", "oil")
        actuate(self.oil_needles, -1.0,
                f"Prep: aspirate {self.oil_needles:g} needle(s) of oil",
                compensate=None)

        # 3. Wash.
        goto("__wash__", "wash")
        self._set_sub_step(prep_op, "Prep: wash needle")
        self._do_wash()

        # 4. Buffer — aspirate buffer.
        goto("__buffer__", "buffer")
        actuate(self.buffer_needles, -1.0,
                f"Prep: aspirate {self.buffer_needles:g} needle(s) of buffer",
                compensate=None)

        self._set_sub_step(prep_op, "Prep complete")

    def _prep_bore_plan_guard(self, plan) -> None:
        """Warn once per run about a bore whose own volume could not be read.

        A bore prepped with the wrong "1 needle's worth" is under- or
        over-conditioned, which is a fluidics problem the operator can only see
        if we say so.
        """
        if getattr(self, "_prep_volume_warned", False):
            return
        bad = [f"bore {i + 1}/{p}" for (p, i, v) in plan if v <= 0]
        if bad:
            logger.warning(
                "Prep: could not resolve an internal volume for %s — those "
                "bores will not be conditioned.", ", ".join(bad))
        self._prep_volume_warned = True

    def run_post_clean(self):
        """Clean + reset the needle ONCE after the operation loop.

        Sequence ("needle waste, wash, and reset"):
          1. → waste  ; DISPENSE ``post_dispense_needles`` × a needle of residual
                         (reagent + cells) so the needle is empty
          2. → wash   ; wash (dip + jiggle Z + random XY about the well centre)
          3. → buffer ; ASPIRATE ``buffer_needles`` × a needle of buffer to reset
                         the needle to a conditioned, buffer-loaded state

        Mirrors :meth:`run_prep` — same service wells, needle volume, dip Z,
        ``prep_bore`` / ``prep_rate_uL_s``, and (v7.9) the same simultaneous
        multi-bore treatment via ``prep_bores``. This matters here specifically:
        after a run that used a DEDICATED trypsin bore, that bore also holds
        residual (whatever it did not push) that must be cleared — clearing
        only ``prep_bore`` would leave it dirty. Abort-aware; raises if a
        required service well is unresolved (the GUI gates on this up front).
        """
        clean_op = SimpleNamespace(op_id="CLEAN", sub_step="")
        unit = float(self.needle_volume_uL or 0.0)
        bore = self.prep_bore
        rate = self.prep_rate_uL_s
        sz = self.service_z_mm
        plan = self._prep_bore_plan()

        def goto(key, label):
            self._check_abort()
            self._set_sub_step(clean_op, f"Clean: travel to {label}")
            if not self._safe_move_to_well(key, target_z_mm=sz):
                raise RuntimeError(
                    f"Clean: {label} well not configured/resolved ({key})")

        def actuate(needles, sign, label, compensate=False):
            if needles <= 0:
                return
            if plan:
                self._set_sub_step(clean_op, label)
                self._prep_bore_plan_guard(plan)
                self._prep_pump_move(plan, needles, sign, rate,
                                     compensate=compensate)
            elif unit > 0:
                self._set_sub_step(clean_op, label)
                if compensate is False:
                    self._pump_move(bore, sign * unit * needles, rate_uL_s=rate)
                else:
                    _settled_pump_move(self.controller, bore,
                                       sign * unit * needles, rate_uL_s=rate,
                                       compensate=compensate)
            self._check_abort()

        # 1. Waste — dispense residual.
        goto("__waste__", "waste")
        actuate(self.post_dispense_needles, +1.0,
                f"Clean: dispense {self.post_dispense_needles:g} needle(s) to "
                f"waste")

        # 2. Wash.
        goto("__wash__", "wash")
        self._set_sub_step(clean_op, "Clean: wash needle")
        self._do_wash()

        # 3. Buffer — reload buffer (reset).
        goto("__buffer__", "buffer")
        actuate(self.buffer_needles, -1.0,
                f"Clean: aspirate {self.buffer_needles:g} needle(s) of buffer",
                compensate=None)

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
            self._pump_move(bore, +v, rate_uL_s=rate)
        else:
            self._set_sub_step(op, f"Oil prep: aspirate {v:.3f} µL of oil")
            if not self._safe_move_to_well("__oil__", target_z_mm=sz):
                raise RuntimeError(
                    "Oil prep: oil well not configured/resolved (__oil__)")
            self._pump_move(bore, -v, rate_uL_s=rate,
                               compensate=None)
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
            self._pump_move(bore, +waste_uL, rate_uL_s=rate)
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
            self._pump_move(bore, oil_uL, rate_uL_s=rate)

        self._set_sub_step(clean_op, "Cleanup complete")

    def _wash_needle(self, op, label: str = "Washing needle") -> bool:
        """Rinse the needle EXTERIOR at the wash well, then leave it there.

        Safe-travel to the ``__wash__`` well (retract → cross → lower to the
        service dip Z), then run :meth:`_do_wash` (Z + random XY jiggle). Used
        after aspirating a reagent (before travelling to deposit it) so the film
        on the needle's outer surface is washed off — the aspirated volume stays
        in the bore. The next :meth:`_safe_move_to` retracts before travelling
        on, so no explicit retract is needed here.

        Returns True if the wash ran, False (a graceful no-op) when the wash well
        isn't configured — the GUI gates on this up front. Abort-aware.
        """
        if self.wash_well_pos is None:
            logger.warning("wash_after_pickup requested but no wash well set")
            return False
        self._check_abort()
        self._set_sub_step(op, label)
        if not self._safe_move_to_well("__wash__", target_z_mm=self.service_z_mm):
            return False
        self._do_wash()
        self._check_abort()
        return True

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
        # Recentre over the wash well before the next prep step. (Advisory: a
        # service well is millimetres across, so a missed confirm here is not a
        # collision risk — unlike the in-well shift, which aborts.)
        ctrl.move_xy_absolute_um(cx, cy)
        self._wait_xy_arrival_um(cx, cy)

    # ── Ink pickup (Quick Print: "pick the ink we will need") ─────

    def aspirate_ink(self, well_pos, volume_uL, *, bore, z_mm,
                     rate_uL_s=None, prime_uL=0.0, prime_rate_uL_s=None,
                     orbit=False, orbit_diameter_mm=1.0, orbit_speed_mm_s=2.0):
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
            prime_uL: v7.5.x TIP PRIME. Aspirate this EXTRA volume beyond
                ``volume_uL`` and then immediately DISPENSE the same amount back
                into the ink well. This advances ink to the very tip and purges
                the air gap, so the ink is ready to deposit the moment printing
                starts — net retained volume stays ``volume_uL``. 0 = disabled
                (legacy). When priming, the whole pickup step runs WITHOUT
                backlash/compliance compensation (``compensate=False``): the
                drivetrain compliance is already primed by this dispense-back, so
                bracketing the pickup with take-up/unload would fight it.
            prime_rate_uL_s: Flow rate for the extra aspirate + dispense-back
                (µL/s); defaults to the aspirate ``rate``.
            orbit: v7.5.x GRANULAR ANTI-CLOG. When True, orbit the needle in a
                small circle (``orbit_diameter_mm``) around the well centre WHILE
                the pump aspirates/dispenses, so granular material behaves more
                fluid-like and does not clog the bore. In-well motion (needle
                already at dip Z) — exempt from retract-before-XY, same class as
                the wash jiggle.
            orbit_diameter_mm: Orbit circle diameter (mm). Default 1 mm.
            orbit_speed_mm_s: Tangential orbit speed along the circle (mm/s).

        Abort-aware and ZP-down-guarded — delegates the move to ``_safe_move_to``
        (which raises :class:`AbortException` if the ZP board has dropped, so we
        never drag an unretracted needle across the plate). The ink well is
        ``__ink__`` so it is never the same well as the prep's ``__buffer__`` →
        always a full safe-Z travel. Does NOT retract afterward; the caller's
        ``finally`` handles end-at-safe-Z via :meth:`_retract_to_safe_z`.
        """
        self._check_abort()
        cx, cy = float(well_pos[0]), float(well_pos[1])
        target = PickPlaceTarget(
            target_id="ink_well", x_um=cx, y_um=cy, well_name="__ink__",
        )
        self._safe_move_to(target, target_z_mm=z_mm)
        self._check_abort()
        vol = float(volume_uL or 0.0)
        prime = max(0.0, float(prime_uL or 0.0))
        if vol <= 0 and prime <= 0:
            return  # travel-only (needle already loaded, no prime)

        rate = rate_uL_s if rate_uL_s is not None else self.prep_rate_uL_s
        p_rate = prime_rate_uL_s if prime_rate_uL_s is not None else rate
        # Prime mode owns the compliance decision for the WHOLE pickup step: skip
        # comp (compensate=False) so the take-up/unload bracket does not fight the
        # dispense-back priming. Without prime, keep the legacy auto behaviour
        # (None = comp iff the global backlash toggle is on).
        compensate = False if prime > 0 else None

        stop = self._orbit_xy((cx, cy), orbit_diameter_mm, orbit_speed_mm_s) \
            if orbit else None
        try:
            aspirate_total = vol + prime  # >0 (guarded above)
            if aspirate_total > 0:
                self._pump_move(bore, -aspirate_total,
                                   rate_uL_s=rate, compensate=compensate)
                self._check_abort()
            if prime > 0:
                # Dispense the extra back INTO the ink well (+ = dispense).
                self._pump_move(bore, prime,
                                   rate_uL_s=p_rate, compensate=compensate)
        finally:
            if stop is not None:
                self._stop_orbit(stop, (cx, cy))

    # ── Anti-clog circular pickup orbit (granular inks) ───────────

    _ORBIT_POINTS_PER_REV = 24

    def _orbit_xy(self, center_um, diameter_mm, speed_mm_s):
        """Start a background daemon thread that orbits the needle in a circle
        around ``center_um`` (absolute stage µm) while a blocking pump move runs
        on the caller's thread. Returns a ``threading.Event`` to stop it (via
        :meth:`_stop_orbit`), or ``None`` when the orbit is a no-op.

        The XY (Prior) and pump (ZP/Marlin) buses are independent, so the two
        move concurrently. Points are stepped with ``move_xy_absolute_um`` (the
        same template as ``calibration._ploc_multi_edge_fit``); the loop cycles
        the circle until the stop event OR ``_abort_flag`` is set. In-well motion
        only (needle already at dip Z), so retract-before-XY does not apply."""
        r_um = max(0.0, float(diameter_mm) * 0.5) * 1000.0
        speed = max(1e-3, float(speed_mm_s))
        ctrl = self.controller
        if r_um <= 0.0 or not hasattr(ctrl, "move_xy_absolute_um"):
            return None
        cx, cy = float(center_um[0]), float(center_um[1])
        n = self._ORBIT_POINTS_PER_REV
        # Tangential speed → per-point dwell. Circumference = π·d (mm).
        step_dwell = max(0.01, (math.pi * float(diameter_mm) / speed) / n)
        stop = threading.Event()

        def _loop():
            i = 0
            while not stop.is_set() and not self._abort_flag.is_set():
                th = 2.0 * math.pi * (i % n) / n
                try:
                    ctrl.move_xy_absolute_um(cx + r_um * math.cos(th),
                                             cy + r_um * math.sin(th))
                except Exception:
                    return  # a serial hiccup must not kill the pickup
                i += 1
                stop.wait(step_dwell)

        t = threading.Thread(target=_loop, name="InkPickupOrbit", daemon=True)
        t.start()
        stop._orbit_thread = t  # keep a handle for the join in _stop_orbit
        return stop

    def _stop_orbit(self, stop, center_um):
        """Stop the orbit thread, re-centre over the well, and wait for arrival
        so the subsequent retract starts from the known well centre (mirrors
        :meth:`_do_wash`'s recentre). Best-effort — never raises."""
        try:
            stop.set()
            t = getattr(stop, "_orbit_thread", None)
            if t is not None:
                t.join(timeout=5.0)
        except Exception:
            pass
        ctrl = self.controller
        cx, cy = float(center_um[0]), float(center_um[1])
        try:
            ctrl.move_xy_absolute_um(cx, cy)
            self._wait_xy_arrival_um(cx, cy)
        except Exception:
            pass

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
            self._pump_move(cfg.trypsin_bore, -cfg.trypsin_volume_uL,
                                         rate_uL_s=cfg.push_speed_uL_s)
            self._check_abort()

        # 2. Navigate to target
        self._set_sub_step(op, f"Moving to target {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # 3. Dispense trypsin at target
        self._set_sub_step(op, "Dispensing trypsin")
        self._pump_move(cfg.trypsin_bore, cfg.trypsin_volume_uL,
                                     rate_uL_s=cfg.push_speed_uL_s)
        self._check_abort()

        # 4. Dwell
        self._dwell(op, cfg.dwell_time_s, "Trypsin incubation")
        self._check_abort()

        # 5. Aspirate cells + trypsin
        extract_bore = cfg.extraction_bore if not cfg.single_bore else cfg.trypsin_bore
        self._set_sub_step(op, f"Extracting {cfg.extraction_volume_uL} µL")
        self._pump_move(extract_bore, -cfg.extraction_volume_uL,
                                     rate_uL_s=cfg.pull_speed_uL_s)
        self._check_abort()

        # 6. Navigate to destination
        if dest:
            self._set_sub_step(op, f"Moving to dest {dest.target_id}")
            self._safe_move_to(dest)
            self._check_abort()

            # 7. Dispense at destination
            self._set_sub_step(op, "Dispensing cells")
            self._pump_move(extract_bore, cfg.extraction_volume_uL,
                                         rate_uL_s=cfg.pull_speed_uL_s)
        elif cfg.dest_well:
            self._set_sub_step(op, f"Moving to dest well {cfg.dest_well}")
            self._safe_move_to_well(cfg.dest_well)
            self._check_abort()
            self._set_sub_step(op, "Dispensing cells")
            self._pump_move(extract_bore, cfg.extraction_volume_uL,
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

                self._pump_move(dye.bore, -dye.volume_uL,
                                             rate_uL_s=1.0)
                self._check_abort()

        # Navigate to target
        self._set_sub_step(op, f"Moving to target {source.target_id}")
        self._safe_move_to(source)
        self._check_abort()

        # Deposit dye(s)
        for dye in cfg.dye_configs:
            self._set_sub_step(op, f"Depositing {dye.dye_name}")
            self._pump_move(dye.bore, dye.volume_uL,
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
            self._pump_move(cfg.waste_bore, -total_vol,
                                         rate_uL_s=1.0)
        else:
            # Aspirate with each dye bore
            for dye in cfg.dye_configs:
                self._set_sub_step(op, f"Aspirating {dye.dye_name}")
                self._pump_move(dye.bore, -dye.volume_uL,
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
                self._pump_move(dye.bore, dye.volume_uL,
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

    def _safe_move_to(self, target: PickPlaceTarget, target_z_mm: float | None = None,
                      bore_index: int = 0):
        """Move to a target with appropriate Z protocol.

        INTER-WELL: full safe Z protocol (raise → wait → XY → wait → lower)
        INTRA-WELL: small retract (1mm → wait → XY → wait → lower)

        ``target_z_mm`` (zero-ref mm) is the height to lower to after the XY
        move; None falls back to ``operating_z_mm``. The intra-well shortcut is
        used only when the target is in the *same named* well as the previous
        move — an empty well name (arbitrary clicked points, e.g. the spheroid
        live picker) always uses the full safe-Z travel, since we can't assume
        two un-named points are close enough for a 1 mm retract.

        v7.9 ``bore_index``: which bore of a multi-bore assembly to place ON the
        target. Defaults to 0 (the calibrated ``needle_origin_um`` datum), which
        makes this byte-identical to the pre-v7.9 behaviour for every single-bore
        needle and every existing caller.
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
        # v7.9: place the REQUESTED BORE on the target, not the datum bore. Both
        # helpers are no-ops at bore_index 0 / an unmeasured assembly, so every
        # pre-v7.9 call site behaves exactly as before.
        x_um, y_um = self._bore_target_xy_um(target, bore_index)
        z = self._descend_z_mm(z, bore_index)
        same_well = bool(target.well_name) and target.well_name == self._current_well
        if not same_well:
            # Inter-well: full safe Z
            ok = self._safe_travel(
                target_x_um=x_um,
                target_y_um=y_um,
                safe_z_mm=self.safe_z_mm,
                target_z_mm=z,
                z_timeout_s=self.z_timeout_s,
                xy_timeout_s=self.xy_timeout_s,
            )
            # v7.20 CRITICAL SAFETY: ACT on the verdict. `safe_travel_to`
            # returns False when the retract was not confirmed, when the XY
            # arrival was not confirmed, or on abort — and this result used to
            # be DISCARDED, so the caller went straight on to descend, aspirate
            # or dispense at a position the stage may never have reached.
            #
            # Note the asymmetry this removes: `_intra_well_move` below already
            # raises on an unconfirmed move, and it is the SHORTER, less
            # dangerous one. The inter-well travel — needle crossing the whole
            # plate — must not use a weaker policy than its intra-well sibling.
            #
            # Only an explicit False is a failure: a stub/older controller that
            # returns None cannot report a verdict, and "absent" degrades to
            # permitted exactly as `_wait_xy_arrival_um` already documents.
            if ok is False:
                raise AbortException(
                    f"Travel to ({x_um:.0f}, {y_um:.0f}) µm was not confirmed "
                    "— stopping with the needle retracted rather than "
                    "descending or dispensing at an unverified position. "
                    "Check the XY and Z/pump boards.")
        else:
            # Intra-well: small retract
            self._intra_well_move(x_um, y_um, target_z_mm=z)

        self._current_well = target.well_name

    def _safe_move_to_well(self, well_name: str, target_z_mm: float | None = None,
                           bore_index: int = 0):
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
        self._safe_move_to(target, target_z_mm=target_z_mm, bore_index=bore_index)
        return True

    def _wait_xy_arrival_um(self, x_um: float, y_um: float,
                            tolerance_mm: float | None = None) -> bool:
        """Wait for XY arrival at an ABSOLUTE stage µm position.

        ``StageController.wait_for_xy_arrival`` takes **zero-ref mm**, so the
        conversion is ``(abs_um − zero_position) / 1000``. Every caller in this
        file had written that out by hand and one of them — ``_intra_well_move``
        — omitted the ``zero_position`` term, passing absolute mm into a zero-ref
        comparison. On any machine whose zero was set away from stage home (i.e.
        every real one) the comparison could never succeed, so the wait burned its
        full timeout, returned False, and had its result discarded — adding 30 s
        per shift AND letting the following Z descent run with XY unconfirmed.
        Hence one helper rather than three hand-written conversions.

        Returns True when arrival was CONFIRMED. A missing controller method
        degrades to True (nothing to confirm against), matching the pre-existing
        ``hasattr`` guards.
        """
        ctrl = self.controller
        waiter = getattr(ctrl, "wait_for_xy_arrival", None)
        if not callable(waiter):
            return True
        zero = getattr(ctrl, "zero_position", {}) or {}
        try:
            zx = float(zero.get("x", 0.0))
            zy = float(zero.get("y", 0.0))
        except (AttributeError, TypeError, ValueError):
            zx = zy = 0.0
        kw = {"timeout_s": self.xy_timeout_s}
        if tolerance_mm is not None:
            kw["tolerance_mm"] = float(tolerance_mm)
        try:
            return bool(waiter((x_um - zx) / 1000.0, (y_um - zy) / 1000.0, **kw))
        except TypeError:
            # Older/stub controllers without tolerance_mm.
            return bool(waiter((x_um - zx) / 1000.0, (y_um - zy) / 1000.0,
                               timeout_s=self.xy_timeout_s))

    def _intra_well_move(self, target_x_um: float, target_y_um: float,
                         target_z_mm: float | None = None,
                         from_z_mm: float | None = None,
                         xy_tolerance_mm: float | None = None):
        """Move within the same well — small retract + wait + XY + wait + lower.

        Still follows the wait-for-Z-before-XY rule. ``target_z_mm`` (zero-ref
        mm) is the height to lower back to; None falls back to operating_z_mm.
        ``from_z_mm`` is the height the tool is at NOW (defaults to the target,
        which is correct whenever both are planned the same way); it only affects
        where the lift's arrival confirm expects to land.

        ⚠ v7.9 POLARITY FIX — this used to drive the needle INTO THE GLASS.
        The lift was ``move_z_relative(-intra_well_retract_mm)``, a **raw**
        Marlin delta (see StageController.move_z_relative), while "up" in the
        HEIGHT frame is ``z_up_sign × raw`` — and BOTH shipped device profiles
        (ME3B V1, ME3B V3) record ``z_up_sign = +1.0``. So a negative raw delta
        LOWERED the tip by ``intra_well_retract_mm`` (1 mm by default) starting
        from a height only ~0.1 mm off the plate bottom.

        It stayed dormant because the only caller reaches this branch when
        ``target.well_name`` is non-empty AND equals the previous move's well,
        and live-picker targets carry ``well_name=""``. v7.9's per-bore targets
        inside ONE well are exactly the case that activates it, so it is fixed
        rather than worked around. Pattern copied from :meth:`_do_wash`.
        """
        z = self.operating_z_mm if target_z_mm is None else target_z_mm
        z_from = z if from_z_mm is None else from_z_mm
        ctrl = self.controller
        amp = max(0.0, float(self.intra_well_retract_mm))

        # SAFETY GATE — belongs to the PRIMITIVE, not its callers. `_shift_to_bore`
        # (v7.9) calls this directly, bypassing `_safe_move_to`'s identical check,
        # and this function is where the needle is millimetres from glass. Without
        # it, a mid-session ZP drop (the documented CH340 disconnect) makes the
        # lift a silent no-op — `ZPStage.send_data` returns False and never raises
        # — while `move_xy_absolute_um` still executes on the still-connected
        # ProScan, DRAGGING the needle laterally at ~0.1 mm above the plate.
        # `getattr(..., True)` keeps board-less rigs and test fakes working.
        if not getattr(ctrl, "is_zp_connected", True):
            raise AbortException(
                "ZP board not connected — refusing the in-well move to avoid "
                "dragging an unretracted needle across the plate bottom. "
                "Reconnect the Z/pump board and restart the operation.")

        # 1. Retract Z — in the HEIGHT frame, so it can only ever move AWAY from
        #    the plate. A zero amplitude degrades to "no lift", never a descent.
        if amp > 0:
            if hasattr(ctrl, "move_z_user_relative"):
                ctrl.move_z_user_relative(+amp)
            else:
                ctrl.move_z_relative(self._z_up_sign() * amp)
            # Zero-ref height after a HEIGHT-frame lift of +amp from where the
            # tool actually IS. The confirm result is now ACTED ON: an
            # unconfirmed retract is the precondition for `safe_travel_to`'s own
            # refusal to start an XY move, and the intra-well twin must not use a
            # weaker policy than its inter-well sibling or the two silently drift.
            if not ctrl.wait_for_z_arrival(z_from + self._z_up_sign() * amp,
                                           timeout_s=self.z_timeout_s):
                raise AbortException(
                    "In-well retract was not confirmed — refusing the XY move "
                    "rather than dragging the needle across the plate bottom. "
                    "Check the Z/pump board and the Z soft limits.")
            # Re-check the link: the documented failure is a board drop BETWEEN
            # the dose and the shift, and both `wait_for_z_arrival` and
            # `ensure_retracted_to` return True when the board is gone, so the
            # confirm above cannot fail closed on its own.
            if not getattr(ctrl, "is_zp_connected", True):
                raise AbortException(
                    "ZP board dropped during the in-well retract — refusing the "
                    "XY move; the needle's height can no longer be verified.")

        # 2. Move XY
        # v7.5.x bugfix: target_{x,y}_um are ABSOLUTE stage µm (same frame the
        # inter-well safe_travel_to uses). move_xy_absolute(from_zero_ref=True)
        # would treat them as mm (×1000 + zero) → gross mis-placement; use the
        # µm entry point.
        ctrl.move_xy_absolute_um(target_x_um, target_y_um)
        if not self._wait_xy_arrival_um(target_x_um, target_y_um,
                                        tolerance_mm=xy_tolerance_mm):
            # Leave the tool AT THE LIFTED HEIGHT — do not lower onto an unknown
            # position. At a 0.1 mm working clearance a blind in-well move can
            # meet the well wall, and lowering anyway produces the silent
            # dose-then-miss outcome this whole stage exists to prevent.
            raise AbortException(
                "In-well XY move was not confirmed — stopping with the needle "
                "retracted rather than lowering onto an unverified position.")

        # 3. Lower Z back — ABSOLUTE, so the tip returns to exactly the working
        #    height even if the lift was soft-limit clamped (a symmetric relative
        #    descend would let the tip walk toward the plate over repeated moves,
        #    the same failure _do_wash already guards against).
        if amp > 0:
            if hasattr(ctrl, "move_z_absolute"):
                ctrl.move_z_absolute(z, from_zero_ref=True)
            elif hasattr(ctrl, "move_z_user_relative"):
                ctrl.move_z_user_relative(-amp)
            else:
                ctrl.move_z_relative(-self._z_up_sign() * amp)
            ctrl.wait_for_z_arrival(z, timeout_s=self.z_timeout_s)

    # ── Per-bore geometry (v7.9) ─────────────────────────────────

    def _needle(self):
        """The configured needle assembly, or None."""
        return (getattr(self.hw_config, "needle", None)
                if self.hw_config is not None else None)

    def _bore_offset_um(self, bore_index: int) -> tuple[float, float]:
        """Lateral offset (µm) of one bore from bore 0, the calibrated datum.

        Returns (0, 0) for a single-bore needle, for an unmeasured bore, and for
        any stub that cannot say — so the offset-aware motion below is a no-op on
        every pre-v7.9 setup and can be applied unconditionally.

        ⚠ This is the ONE read that both :meth:`_bore_target_xy_um` (inter-well
        travel that parks a bore on a target) and :meth:`_shift_to_bore`
        (intra-well shift with the needle DOWN) go through, so the plausibility
        bound is enforced here rather than in either caller. An implausible
        offset is REFUSED, never clamped and never quietly ignored: clamping
        gives a wrong move that looks right, and ignoring it produces exactly the
        unmeasured-assembly failure (dose the cell, then aspirate 100-500 µm away
        and collect nothing).
        """
        needle = self._needle()
        if needle is None or not bore_index:
            return (0.0, 0.0)
        ox, oy = needle_bore_offset_um(needle, bore_index)
        worst = max(abs(ox), abs(oy))
        if worst > MAX_BORE_OFFSET_UM:
            raise AbortException(
                f"Bore {bore_index + 1}'s measured mount offset is "
                f"({ox:.0f}, {oy:.0f}) µm, beyond the {MAX_BORE_OFFSET_UM:.0f} µm "
                f"plausibility limit for a fused needle assembly. Moving the "
                f"stage that far with the needle inside a well would hit the "
                f"well wall. Re-measure the bore offsets on Calibration → "
                f"Needle Location (a mm value typed into a µm field is the "
                f"usual cause).")
        return (ox, oy)

    def _bore_target_xy_um(self, target: PickPlaceTarget,
                           bore_index: int) -> tuple[float, float]:
        """Stage XY that puts BORE ``bore_index`` on ``target``.

        ``stage_xy = target_xy - bore_offset`` — the sign convention fixed in
        ``NeedleBore``'s docstring and pinned by a round-trip test. Bore 0 is the
        ``needle_origin_um`` datum, so its offset is (0, 0) and this reduces to
        the target itself.

        ⚠ A mis-signed offset here is a *right-distance-wrong-way* error: the
        needle lands the correct distance away on the WRONG side of the cell,
        which looks like a calibration problem rather than a sign bug. That is
        the same failure class as the plate-orientation bugs in CLAUDE.md.
        """
        ox, oy = self._bore_offset_um(bore_index)
        return (float(target.x_um) - ox, float(target.y_um) - oy)

    def _bore_z_mm(self, base_z_mm: float | None,
                   bore_index: int) -> float | None:
        """Zero-ref Z that puts BORE ``bore_index``'s tip at the working height.

        A bore whose ``z_offset_mm`` is positive reaches LOWER than the datum
        bore, so the stage must sit that much HIGHER in the height frame for its
        tip to end up at the requested clearance — otherwise a longer bore is
        driven into the glass. Converted through ``z_up_sign`` so it is correct
        on both Z polarities.

        Measured spacing on this rig is coplanar within ~50 µm (decision D7),
        which is HALF the default 0.10 mm working clearance — small, but not
        negligible, which is why it is applied rather than assumed away.
        """
        if base_z_mm is None or not bore_index:
            return base_z_mm
        dz = needle_bore_z_offset_mm(self._needle(), bore_index)
        if not dz:
            return base_z_mm
        return float(base_z_mm) + self._z_up_sign() * float(dz)

    def _descend_z_mm(self, base_z_mm: float | None,
                      bore_index: int) -> float | None:
        """Zero-ref Z to DESCEND to, guaranteeing EVERY bore clears the plate.

        :meth:`_bore_z_mm` answers a different question — "where must the stage
        be for bore *k*'s tip to sit at the working height" — and using it to
        plan a descend is a **glass-breaking** mistake on a multi-bore assembly,
        because it accounts only for the bore being placed. If bore 1 protrudes
        0.20 mm further than the datum and the datum is lowered to a 0.10 mm
        clearance, bore 1 ends up 0.10 mm BELOW the plate bottom.

        So a descend is planned against the LOWEST-REACHING bore in the
        assembly::

            stage_z = base_z + z_up_sign · max(dz_k, dz_max)

        The consequence is deliberate and worth stating plainly: on an assembly
        whose bores are not coplanar, the requested bore sits *higher* than the
        requested clearance by ``dz_max − dz_k``. Aspirating from 0.34 mm up
        instead of 0.10 mm is a worse aspirate; driving a needle into
        borosilicate is a broken needle, a scratched plate and a lost sample. The
        difference is logged so the operator knows which they are getting.

        ``max`` is taken over OFFSET DATA, never over raw Z positions, and the
        result is converted through ``z_up_sign`` — so this is correct on both Z
        polarities. Since ``NeedleSpec.__post_init__`` pins bore 0's offset to
        0.0, ``dz_max >= 0`` always: this is **raise-only**, and returns exactly
        ``base_z_mm`` for every single-bore needle and every pre-v7.9 setup.

        ⚠ NOT a substitute for the plate-bottom floor, and not a substitute for
        measuring the offsets: an UNMEASURED bore reports 0.0 and is therefore
        invisible here. The floor armed by :meth:`execute_queue` is the backstop,
        and the unmeasured case is refused before the run starts.
        """
        if base_z_mm is None:
            return base_z_mm
        needle = self._needle()
        if needle is None:
            return base_z_mm
        dz_k = needle_bore_z_offset_mm(needle, bore_index)
        dz_max = needle_max_bore_z_offset_mm(needle)
        if abs(dz_max) > MAX_BORE_Z_OFFSET_MM or abs(dz_k) > MAX_BORE_Z_OFFSET_MM:
            raise AbortException(
                f"A bore's measured axial offset "
                f"({max(abs(dz_k), abs(dz_max)):.3f} mm) is beyond the "
                f"{MAX_BORE_Z_OFFSET_MM:.2f} mm plausibility limit — the bores of "
                f"a fused assembly are coplanar to within ~0.05 mm. Descending "
                f"on this number could drive a bore into the plate. Re-measure "
                f"the bore Z offsets on Calibration → Needle Location.")
        lowest = max(dz_k, dz_max)
        if not lowest:
            return base_z_mm
        if dz_max > dz_k and not getattr(self, "_logged_clearance_giveaway", False):
            self._logged_clearance_giveaway = True
            logger.info(
                "Descend planned against the LOWEST-reaching bore: bore %d's tip "
                "will sit %.3f mm above the requested height (%.3f mm instead of "
                "the requested clearance) because another bore reaches %.3f mm "
                "lower. Raising the stage is the only alternative to driving "
                "that bore into the plate.",
                bore_index + 1, dz_max - dz_k, dz_max - dz_k, dz_max)
        return float(base_z_mm) + self._z_up_sign() * float(lowest)

    def _shift_to_bore(self, op: PickPlaceOperation,
                       target: PickPlaceTarget,
                       from_bore: int, to_bore: int,
                       target_z_mm: float | None) -> bool:
        """Shift XY so a DIFFERENT bore of the assembly sits on the same target.

        This is the move between the trypsin push and the aspirate (decision
        D5). The displacement is the inter-bore spacing — ~100-500 µm measured —
        so it stays inside the well and uses the small intra-well retract rather
        than a full safe-Z round trip, which would be both slow and more
        disturbing to the drop just deposited.

        Returns False (having moved nothing) when the two bores share a position,
        which is the single-bore case and every unmeasured assembly. Callers must
        NOT treat that False as "the shift happened anyway" — an unmeasured
        assembly is refused before the run reaches here, because dosing a cell
        and then aspirating 100-500 µm away collects nothing.

        The XY arrival tolerance is scaled to the shift itself. The controller's
        0.1 mm default is the SAME ORDER as a 100 µm inter-bore spacing, so it
        would "confirm" arrival at the pre-move position — a false confirm, which
        is strictly worse than the timeout it replaces.
        """
        src = self._bore_offset_um(from_bore)
        dst = self._bore_offset_um(to_bore)
        dx, dy = dst[0] - src[0], dst[1] - src[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return False
        x_um, y_um = self._bore_target_xy_um(target, to_bore)
        # Both heights go through the clearance guarantee, so from_z == to_z on a
        # coplanar-planned descend; the explicit from_z keeps the Z-arrival
        # confirm correct if per-bore Z planning is ever reintroduced.
        z = self._descend_z_mm(target_z_mm, to_bore)
        from_z = self._descend_z_mm(target_z_mm, from_bore)
        dist_mm = math.hypot(dx, dy) / 1000.0
        tol_mm = min(0.05, max(0.01, dist_mm / 4.0))
        self._set_sub_step(
            op, f"Shifting {abs(dx):.0f}×{abs(dy):.0f} µm "
                f"to put bore {to_bore + 1} on the target")
        self._intra_well_move(x_um, y_um, target_z_mm=z, from_z_mm=from_z,
                              xy_tolerance_mm=tol_mm)
        return True

    def _z_up_sign(self) -> float:
        """``+1``/``-1`` — which way a RAW Marlin Z delta moves the needle UP.

        Reads the live controller so the per-machine profile drives it, and
        defaults to ``+1`` (what BOTH shipped profiles record, and what a
        ``__new__``-partial test controller reports) when it cannot say.
        """
        get_sign = getattr(self.controller, "z_up_sign", None)
        if callable(get_sign):
            try:
                sign = float(get_sign())
                if sign:
                    return 1.0 if sign > 0 else -1.0
            except (TypeError, ValueError):
                pass
        return 1.0

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

    def _pump_move(self, pump, volume_uL, rate_uL_s=None, *,
                   compensate=False):
        """v7.6: every executor pump actuation goes through here so this
        operation's ``_abort_flag`` reaches the blocking drain/settle waits.

        Before this, an Abort pressed during a long reagent aspirate had to wait
        out the whole move (the M400 drain caps at 180 s per sub-move) because
        ``_abort_flag`` was only polled at ``_check_abort()`` points BETWEEN
        steps. Same signature as the module-level ``_settled_pump_move``.
        """
        return _settled_pump_move(self.controller, pump, volume_uL, rate_uL_s,
                                  compensate=compensate,
                                  abort_event=self._abort_flag)

    def _safe_travel(self, **kw):
        """v7.6: abort-aware ``safe_travel_to`` (degrades on fakes/older
        controllers that lack the kwarg)."""
        try:
            return self.controller.safe_travel_to(
                abort_event=self._abort_flag, **kw)
        except TypeError:
            return self.controller.safe_travel_to(**kw)

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
