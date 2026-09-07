"""
Stage Controller — Unified control layer for XY and ZP stages.

Contains:
    - XboxQueuePoller:  Reads Xbox events from multiprocessing queue
    - XYJogHandler:     Continuous velocity jogging for XY stage
    - ZPJogHandler:     Segmented relative moves for ZP stage
    - PositionPoller:   Background cached position reads
    - StageController:  Top-level orchestrator tying everything together

The jog handlers preserve the proven XBOXCONTROLLED workflow.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from multiprocessing import Event as MPEvent, Process, Queue
from typing import Callable, Optional

from SupportClasses.Processor import Processor
from SupportClasses.MotionEstimator import MotionEstimator
from SupportClasses.XYStage import XYStageManager
from SupportClasses.ZPStage import ZPStageManager, AXIS_MAP


def _axis_letter(zp_stage, logical: str) -> str | None:
    """v7.4.2: Resolve a logical axis (Z/P1/P2/P3) to its physical Marlin
    letter (X/Y/Z/E) using the live ZPStageManager's mapping when
    available, falling back to the module-level default."""
    if zp_stage is not None and hasattr(zp_stage, 'axis_map'):
        return zp_stage.axis_map.get(logical, AXIS_MAP.get(logical))
    return AXIS_MAP.get(logical)


# v7.4.2 hotfix: physical Marlin letter → index into the ZP position
# tuple returned by ZPStageManager.get_current_position(), which is
# always (x, y, z, e) in Marlin's physical axis order.
_PHYSICAL_TO_INDEX = {"X": 0, "Y": 1, "Z": 2, "E": 3}


# ── v7.5.x: settle-aware discrete pump moves ───────────────────────────
# A pump G0 returns on Marlin `ok` (admitted to the planner buffer), NOT
# motion-complete, so a settle-aware discrete actuation must block until the
# pump has PHYSICALLY finished before its post-settle dwell — otherwise the
# still-running move drains behind the caller's next Z/XY safe_travel_to M400
# and trips its timeout (a 4-needle buffer aspirate at 1 µL/s is ~27 s; a 15 s
# M400 cannot absorb it → "Z retract M400 timed out — ABORTING"). A small ink
# aspirate fit under the old 10 s open-loop sleep cap and "worked"; a larger
# prep volume did not. We now confirm completion via M400 (the same mechanism
# safe_travel_to uses for Z) with a timeout scaled to the estimated duration,
# so buffer and ink behave identically regardless of volume.
#
# The open-loop estimate is abs(volume)/rate + 0.1 s. It is used (a) to scale
# the M400 confirmation timeout and (b) as a fallback sleep ONLY when no board
# confirmation is available (older controller / fake without ``flush_moves``).
# Absolute backstop so a pathological near-zero rate can't hang forever.
_PUMP_MOVE_DRAIN_TIMEOUT_CAP_S = 180.0
# Extra margin (s) added to the move estimate for the M400 confirmation
# timeout, covering accel/decel ramps and busy keep-alives.
_PUMP_MOVE_DRAIN_MARGIN_S = 5.0
# Fallback flow rate (µL/s) used only for the completion estimate when a
# settle-aware caller passes no explicit rate.
_PUMP_SETTLE_FALLBACK_RATE_UL_S = 1.0

# ── v7.5.x: gentle-Z re-entry / lift confirmation timeout ──────────────
# The gentle "slow last mm" descent (and "slow first mm" lift) runs its final
# leg at a deliberately low feedrate (e.g. 1 mm @ 6 mm/min = 10 s). A FIXED
# M400/arrival confirmation timeout can therefore expire BEFORE a perfectly
# healthy slow move finishes, falsely tripping the "board stuck → abort"
# guard. Like the pump drain above, the confirmation timeout is SIZED to the
# estimated move duration (fast leg + slow leg) plus a margin, floored at the
# caller's baseline and capped so a pathological estimate can't hang forever.
_GENTLE_Z_CONFIRM_MARGIN_S = 5.0
_GENTLE_Z_CONFIRM_CAP_S = 120.0


# ── v7.5.x: Z display-numbering sign (ME3B V1) ─────────────────────────
# On this machine the needle physically DESCENDS as the raw Marlin Z
# counter INCREASES (steps_per_mm["Z"] = +5255). The app numbers Z by the
# raw value, so going up the number went negative (top read -60) and the
# Min/Max envelope recorded inverted (z_min > z_max), collapsing clamp_z
# to a single point and freezing Z.
#
# We number Z as a *height* (up = +) at the human boundary only: every Z
# value shown to the user on the Jog/Device pages is multiplied by ZDIR;
# every Z the user types/clicks there is divided by ZDIR before motion.
# The internal frames (raw Marlin, zero-ref, calibration storage, print,
# SafetyLimits.z_min/z_max) are UNCHANGED — motion is physically identical.
# Because ZDIR = -1 reverses ordering, converting between the height limit
# spinboxes and the raw-stored limits also requires a min/max SWAP.
ZDIR = -1.0

# v7.5.x: per-machine well-plate ORIENTATION. The canonical convention is
# "well A1 displays top-left; stage physical 0,0 is bottom-right". On the
# ME3B V1 Prior ProScan II the stage origin is at the bottom-right with +X/+Y
# toward the top-left, so the plate-local axes (A1 at (0,0), +col→right,
# +row→down) are ANTI-aligned with the stage axes. ``plate_flip_180`` (True
# on ME3B V1) drives BOTH the 180° display flip (stage-frame views) and the
# plate-local→stage geometry sign (``plate_axis_sign`` = (-1, -1) when True).
# Defaults to True to preserve today's ME3B rendering on a machine whose
# device profile predates this setting; flip to False for a stage mounted
# with the conventional origin/axes.
DEFAULT_PLATE_FLIP_180 = True


# ── v7.5.x: Xbox jog-speed ladder (percentage of the calibrated max) ───
# Each joystick-driven axis group (XY / Z / Pump) jogs at a fixed PERCENTAGE
# of that group's calibrated max move speed (100% = the per-axis max from the
# Hardware Setup pages). The controller cycles a group up/down one rung at a
# time via the increment_{xy,z,p}speed_up/down commands.
JOG_SPEED_LADDER_PCT: tuple[float, ...] = (0.1, 0.3, 1.0, 3.0, 10.0, 30.0, 100.0)


def _positive_number(value) -> float | None:
    """``value`` as a positive float, or None if it is not a genuine number.

    v7.21.1 — deliberately an ``isinstance`` check, NOT ``float(value)`` in a
    ``try``: ``MagicMock`` implements ``__float__`` and answers 1.0, so the
    coercing form silently accepts a stubbed attribute as a real measurement.
    That matters here because the values guarded by this helper (the declared XY
    top speed) become the DENOMINATOR of every mm/s→SMS-% conversion — a bogus
    1.0 would scale every commanded speed by ~5000×.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    v = float(value)
    return v if v > 0 else None


def _jog_speed_ladder_step(current_pct: float, direction: int) -> float:
    """Step *current_pct* to the next (direction>0) / previous rung of
    ``JOG_SPEED_LADDER_PCT``, snapping from an arbitrary value to the nearest
    rung first and clamping at both ends."""
    ladder = JOG_SPEED_LADDER_PCT
    try:
        cur = float(current_pct)
    except (TypeError, ValueError):
        cur = ladder[0]
    idx = min(range(len(ladder)), key=lambda i: abs(ladder[i] - cur))
    idx = max(0, min(len(ladder) - 1, idx + (1 if direction > 0 else -1)))
    return ladder[idx]


def z_raw_to_display(raw: float) -> float:
    """Raw Marlin Z (mm) → user-facing height (mm, up = +)."""
    return ZDIR * raw


def z_display_to_raw(disp: float) -> float:
    """User-facing height (mm, up = +) → raw Marlin Z (mm)."""
    return disp / ZDIR


# ── v7.5.x: plate-bottom Z datum (polarity-general) ────────────────────
# Print heights are expressed as *height above the plate bottom* (mm, ≥ 0 =
# above/away from the plate). These convert that to/from the internal
# zero-ref Z frame using ZDIR, so they are correct on both a conventional
# machine (ZDIR=+1, up = larger Z) and ME3B V1 (ZDIR=-1, up = smaller Z).

def plate_relative_to_zref(plate_bottom_zref: float, height_above_bottom: float,
                           zdir: float = ZDIR) -> float:
    """Height above the plate bottom (mm) → zero-ref Z (mm)."""
    return plate_bottom_zref + zdir * height_above_bottom


def zref_to_plate_relative(plate_bottom_zref: float, z_zref: float,
                           zdir: float = ZDIR) -> float:
    """Zero-ref Z (mm) → height above the plate bottom (mm).

    Negative result ⇒ the point is *below* the plate bottom (punch-through).
    """
    return zdir * (z_zref - plate_bottom_zref)


def derive_z_up_sign(plate_top_zref: float | None,
                     plate_bottom_zref: float | None,
                     fallback: float | None = None) -> float:
    """v7.5.x: which way is *up*, derived from the calibrated reference points.

    The plate top is physically above the plate bottom, so the sign of
    ``plate_top - plate_bottom`` in the zero-ref frame tells us how zero-ref Z
    maps to physical height — independent of the machine's raw-Z polarity. This
    replaces the hard-coded ``ZDIR`` for print-offset math: +1 = "up is larger
    Z" (conventional), -1 = "up is smaller Z" (ME3B V1, needle descends as raw Z
    increases). Falls back to ``fallback`` (or the live module ``ZDIR`` when
    ``fallback is None``) when the two points aren't both calibrated (or are
    coincident), so behaviour is unchanged until both references are taught.
    """
    if plate_top_zref is not None and plate_bottom_zref is not None:
        d = plate_top_zref - plate_bottom_zref
        if abs(d) > 1e-6:
            return 1.0 if d > 0 else -1.0
    return ZDIR if fallback is None else fallback


def _shorten_only_delta(cur: float, requested: float,
                        clamped_dest: float) -> float:
    """Soft-limit a RELATIVE jog so the clamp can only SHORTEN the move.

    Given the current axis position ``cur``, the operator's ``requested`` delta,
    and the already-clamped absolute destination ``clamped_dest`` (=
    ``clamp(cur + requested)``), return the delta to actually send. Invariant:
    the result **never reverses the sign** of ``requested`` and **never exceeds
    it in magnitude**.

    Why this matters (the "Xbox kept moving the axis constantly" bug): when
    ``cur`` is already OUTSIDE the envelope — a stale/incoherent Z frame, or the
    *cached* jog position lagging behind the real one — ``clamped_dest - cur`` is
    a large delta pointing back toward the limit, OPPOSITE the request. The
    velocity jog loop re-sent that every segment before the cached position
    caught up, so the axis ran continuously into the wall. Shorten-only kills
    that: a jog that would push further out of bounds becomes a no-op, and a jog
    back toward the valid range moves by at most the requested step (no surprise
    snap). For an in-bounds jog this is identical to the old ``clamped - cur``.
    """
    raw = clamped_dest - cur
    if raw == 0.0 or (raw > 0.0) != (requested > 0.0):
        # At the wall (or the clamp wants to move opposite the request, i.e.
        # cur is out of bounds and we're pushing further out) → don't move.
        return 0.0
    if abs(raw) > abs(requested):
        # cur is out of bounds and the snap-back would overshoot the request →
        # cap to the requested step so the operator re-enters at jog speed.
        return requested
    return raw


# v7.9: Marlin's own notion of a move's LENGTH, which is what its feedrate
# applies to. ``Planner::_populate_block`` sets ``block->millimeters`` to the
# Cartesian norm of the X/Y/Z deltas — the E delta does NOT contribute — unless
# no Cartesian axis moves at all, in which case the length IS the E distance.
# The block then takes ``millimeters / F`` seconds, so each axis travels its own
# delta in that time:  v_i = F · |Δ_i| / millimeters.
#
# This matters for a COORDINATED multi-pump move: on a machine that maps a pump
# onto E (ME3B V1: P3 → E), an E-mapped pump moving alongside an X/Y/Z-mapped
# one runs at |Δ_E|/|Δ_xyz| × F — which can EXCEED the commanded vector rate.
# ``move_pumps_uL`` divides by this length when sizing the vector feedrate, so
# every axis's real speed is bounded by its own ceiling regardless of mapping.
_MARLIN_CARTESIAN_LETTERS = ("X", "Y", "Z")


def _marlin_move_length_mm(deltas: dict[str, float]) -> float:
    """Length (mm) Marlin's feedrate applies to for a ``G0`` over *deltas*
    (``{physical_letter: mm}``). See ``_MARLIN_CARTESIAN_LETTERS`` above."""
    cart = [float(d) for a, d in deltas.items()
            if str(a).upper() in _MARLIN_CARTESIAN_LETTERS]
    if any(d for d in cart):
        return math.sqrt(math.fsum(d * d for d in cart))
    # No Cartesian component → Marlin uses the extruder distance as the length.
    return math.fsum(abs(float(d)) for d in deltas.values())


def compute_pump_budget(moves_uL, start_fill_uL: float, capacity_uL: float,
                        *, extra_min_fill_uL: float | None = None,
                        tol_uL: float = 1e-3) -> dict:
    """Pure pre-flight budget for a sequence of plunger moves against the
    calibrated syringe envelope ``[empty = 0, full = capacity_uL]``.

    ``moves_uL``: ordered signed volumes as passed to ``move_pump_uL``
    (``+`` = DISPENSE = lowers fill, ``−`` = ASPIRATE = raises fill).
    ``start_fill_uL``: the plunger fill (µL) before the first move.
    ``extra_min_fill_uL``: an optional FIXED extra low-water mark to fold into
    the trough (e.g. a later cleanup dip that does not scale with the start
    shift — checked but excluded from the shift remedy by the caller; here it
    only widens the reported trough).

    Returns a dict::

        ok                        bool — every fill stayed in [0, capacity]
        capacity_uL               echo
        start_fill_uL             echo
        peak_fill_uL / min_fill_uL  highest / lowest fill reached (incl. start)
        span_uL                   peak − min
        overflow_uL               max(0, peak − capacity)
        underflow_uL              max(0, −min)
        feasible_by_shift         bool — a different start fill would fit
                                  (span ≤ capacity)
        remedy                    'waste_oil' | 'add_oil' | None
        remedy_uL                 µL of oil to waste (dispense) / add (aspirate)
        recommended_start_fill_uL the feasible start fill after the remedy

    The span is invariant to the starting fill, so when ``span ≤ capacity`` a
    feasible start always exists: an overflow (``peak > capacity``) is cured by
    starting lower (waste oil), an underflow (``min < 0``) by starting higher
    (aspirate oil).
    """
    cap = float(capacity_uL)
    fill = float(start_fill_uL)
    peak = fill
    trough = fill
    for v in moves_uL:
        fill += -float(v)          # + dispense lowers fill; − aspirate raises it
        if fill > peak:
            peak = fill
        if fill < trough:
            trough = fill
    if extra_min_fill_uL is not None and float(extra_min_fill_uL) < trough:
        trough = float(extra_min_fill_uL)
    span = peak - trough
    overflow = peak - cap
    underflow = -trough
    ok = (peak <= cap + tol_uL) and (trough >= -tol_uL)
    result = {
        "ok": ok,
        "capacity_uL": cap,
        "start_fill_uL": float(start_fill_uL),
        "peak_fill_uL": peak,
        "min_fill_uL": trough,
        "span_uL": span,
        "overflow_uL": max(0.0, overflow),
        "underflow_uL": max(0.0, underflow),
        "feasible_by_shift": False,
        "remedy": None,
        "remedy_uL": 0.0,
        "recommended_start_fill_uL": float(start_fill_uL),
    }
    if ok:
        return result
    if span > cap + tol_uL:
        # No starting fill fits — the run needs more travel than the syringe has.
        return result
    result["feasible_by_shift"] = True
    if overflow > tol_uL:
        # Over-fills → start LOWER by wasting oil.
        delta = -overflow
        result["remedy"] = "waste_oil"
        result["remedy_uL"] = overflow
    else:
        # Runs dry → start HIGHER by aspirating oil.
        delta = underflow
        result["remedy"] = "add_oil"
        result["remedy_uL"] = underflow
    result["recommended_start_fill_uL"] = float(start_fill_uL) + delta
    return result


def _axis_index(zp_stage, logical: str) -> int | None:
    """v7.4.2 hotfix: index into a ZP position tuple for a logical axis.

    Equivalent of ``_axis_letter`` for *consumers* of M114 readouts —
    routes ``logical`` through the live axis_map and converts the
    resulting physical Marlin letter (X/Y/Z/E) into its tuple index.
    """
    letter = _axis_letter(zp_stage, logical)
    if letter is None:
        return None
    return _PHYSICAL_TO_INDEX.get(letter)
from SupportClasses.XboxController import xbox_polling_worker, calibrate_sticks
from SupportClasses.SerialUtils import ConnectionWatchdog, check_port_health
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.PositionLogger import PositionLogger
import SupportClasses.XYDebugLogger as _dbg

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Xbox Queue Poller
# ═══════════════════════════════════════════════════════════════════

class XboxQueuePoller:
    """
    Polls the Xbox multiprocessing queue in a thread and dispatches
    events to the Processor command bus.
    """

    def __init__(self, queue: Queue, processor: Processor,
                 debug_mode: bool = False,
                 on_lost: Callable | None = None):
        self.queue = queue
        self.processor = processor
        self.debug_mode = debug_mode
        # v7.5.x: fired when the controller is lost (worker reports
        # reconnecting/disconnected, or the heartbeat goes stale).
        # StageController wires this to zero all jog velocities so a
        # stranded velocity can't keep driving the stages — reinstates
        # the v7.3.4-documented behavior that had been lost.
        self._on_lost = on_lost
        self._running = False
        self._thread: threading.Thread | None = None
        # v7.2.6: S4-D status handler — track controller state from worker
        self._xbox_status: str = "disconnected"
        self._last_heartbeat_time: float = time.time()
        # v7.4.2: live input monitor state, v7.5.x: fed by the worker's
        # dedicated "monitor" messages (raw per-axis state) instead of the
        # dispatch stream — the old cache keyed int(group_name) which raised
        # on "0-1"/"2-3" and was silently swallowed, so stick values never
        # updated and Calibrate Sticks always snapshotted zeros.
        # Sticks (0-3): raw [-1, 1]; triggers (4, 5): pull amount [0, 1].
        self.last_axis: dict[int, float] = {}
        self.last_button: tuple[int, float] | None = None  # (button_id, ts)
        self.last_dpad: tuple[int, float] | None = None    # (direction, ts)

    def _fire_on_lost(self, reason: str) -> None:
        """v7.5.x: invoke the controller-lost callback exactly once per event."""
        if self._on_lost is None:
            return
        try:
            self._on_lost()
            logger.info(f"[Xbox] Controller lost ({reason}) — jog velocities zeroed")
        except Exception:
            logger.exception("[Xbox] on_lost callback failed")


    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="XboxPoller"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _poll_loop(self) -> None:
        while self._running:
            while not self.queue.empty():
                try:
                    msg = self.queue.get_nowait()
                except Exception:
                    break

                if "status" in msg:
                    self._last_heartbeat_time = time.time()
                    # v7.2.8: quiet heartbeat — only log on change
                    _prev = self._xbox_status
                    self._xbox_status = msg["status"]
                    if self._xbox_status != _prev:
                        logger.info(f"[Xbox] Status: {self._xbox_status}")
                        # v7.5.x: zero jog velocities the moment the worker
                        # reports the controller gone (the worker also sends
                        # explicit zero dispatches — this is belt-and-braces).
                        if (self._xbox_status in ("reconnecting", "disconnected")
                                and _prev in ("connected", "alive")):
                            self._fire_on_lost(self._xbox_status)
                elif "monitor" in msg:
                    # v7.5.x: raw per-axis state for the GUI live monitor.
                    try:
                        mon = msg["monitor"]
                        for k, v in (mon.get("sticks") or {}).items():
                            self.last_axis[int(k)] = float(v)
                        for k, v in (mon.get("triggers") or {}).items():
                            self.last_axis[int(k)] = float(v)
                    except Exception:
                        pass
                elif "debug" in msg:
                    text = msg["debug"]
                    # Always show connection events; show diagnostic messages
                    # only when debug mode is enabled.
                    if ("connect" in text.lower() or "found" in text.lower()
                            or self.debug_mode):
                        logger.info(f"[Xbox] {text}")
                elif "button" in msg:
                    # v7.4.2: cache last button for the live monitor.
                    try:
                        self.last_button = (int(msg["button"]), time.time())
                    except Exception:
                        pass
                    self.processor.add_command(msg["command"], button=msg["button"])
                elif "axis" in msg:
                    avg = msg["average"]
                    cmd = msg["command"]
                    # (v7.5.x: live-monitor caching moved to the dedicated
                    # "monitor" message — the old int(msg["axis"]) cache
                    # raised on group names like "0-1" and never worked.)
                    # Log pump axis commands only in debug mode
                    if self.debug_mode and "p" in cmd.lower() and "velocity" in cmd.lower():
                        logger.info(
                            f"[Xbox→Proc] {cmd}  axis={msg['axis']}  avg={avg}"
                        )
                    self.processor.add_command(
                        cmd, axis=msg["axis"], average=avg
                    )
                elif "dpad" in msg:
                    try:
                        self.last_dpad = (int(msg["dpad"]), time.time())
                    except Exception:
                        pass
                    self.processor.add_command(msg["command"], direction=msg["dpad"])

            # Heartbeat staleness: worker sends every 3s, stale after 10s
            if self._xbox_status in ("connected", "alive"):
                if time.time() - self._last_heartbeat_time > 10.0:
                    self._xbox_status = "disconnected"
                    logger.warning("[Xbox] Heartbeat stale — marking disconnected")
                    # v7.5.x: a dead worker can't send zeroes — do it here.
                    self._fire_on_lost("heartbeat stale")

            time.sleep(0.02)

class ZPJogHandler:
    """
    Converts velocity commands into segmented relative moves for
    the ZP stage.  Runs its own daemon thread.

    Proven working from XBOXCONTROLLED code — core logic unchanged.
    """

    def __init__(
        self,
        processor: Processor,
        zp_stage: ZPStageManager,
        safety_limits: SafetyLimits | None = None,
        get_zp_position: Callable | None = None,
        zero_position: dict | None = None,  # v7.2.6: zero ref for clamping
        flip_provider: Callable[[str], float] | None = None,  # v7.5.x
        z_up_sign_provider: Callable[[], float] | None = None,  # v7.5.x
        pump_dir_sign_provider: Callable[[str], float] | None = None,  # v7.5.x
    ):
        self.processor = processor
        self.stage = zp_stage
        self.safety_limits = safety_limits
        self._get_zp_position = get_zp_position
        self._zero_position = zero_position or {}  # v7.2.6: zero ref for jog clamping
        # v7.5.x: per-logical-axis direction flip. Callable(logical) -> ±1.0,
        # supplied by StageController so the Xbox PUMP jog honors the same
        # axis_flip settings as the GUI jog buttons (move_pump_relative applies
        # _flip_sign). None = no flip (×1.0). Z is NOT flipped here — its
        # direction comes from z_up_sign (see _z_up_sign).
        self._flip_provider = flip_provider
        # v7.5.x: the per-machine Z up-direction sign (±1.0) DERIVED by the Set
        # Bottom/Top setup. Callable() -> ±1.0, supplied by StageController so
        # the Xbox Z jog moves "up" toward the taught Top exactly like the GUI
        # buttons (move_z_user_relative). None ⇒ ZDIR (legacy, pre-setup).
        self._z_up_sign_provider = z_up_sign_provider
        # v7.5.x: per-pump dispense-direction sign (raw mm per mm of dispense
        # intent), OWNED by the plunger calibration. Callable(logical) -> ±1.0,
        # supplied by StageController so the Xbox PUMP jog moves the same
        # direction as move_pump_relative. None ⇒ fall back to _flip_sign so
        # uncalibrated pumps jog exactly as before.
        self._pump_dir_sign_provider = pump_dir_sign_provider

        # Velocity state (updated by command handlers)
        self.vel_z: float = 0.0
        self.vel_p1: float = 0.0
        self.vel_p2: float = 0.0
        self.vel_p3: float = 0.0
        self._lock = threading.Lock()

        # Tuning parameters
        self.segment_time: float = 0.12   # seconds per move segment
        # v7.5.x: jog speed = a PERCENTAGE of the per-axis calibrated max move
        # speed (see JOG_SPEED_LADDER_PCT). ``z_speed``/``p_speed`` are the
        # effective scalars (DERIVED = speed_pct/100 * speed_max) consumed by
        # the jog loop; they're recomputed by _recompute_zp_speeds() whenever a
        # % or max changes. ``*_speed_max`` (100% anchor) is pushed by
        # StageController.refresh_jog_speed_limits() from the calibrated values.
        #   Z  : mm/s     (calibrated Z feedrate mm/min ÷ 60)
        #   P  : µL/s     when a HardwareConfig is set; mm/s otherwise
        self.z_speed_pct: float = 10.0
        self.p_speed_pct: float = 10.0
        self.z_speed_max: float = 1.0     # mm/s at 100%
        self.p_speed_max: float = 10.0    # µL/s (µL mode) / mm/s (legacy) at 100%
        self.z_speed: float = 0.0         # derived (mm/s at full deflection)
        self.p_speed: float = 0.0         # derived
        self.max_speed: float = 1.0       # legacy general cap (pump mm/s fallback)
        self._hardware_config = None      # Set via set_hardware_config()
        self._recompute_zp_speeds()

        # v7.5.x: velocity staleness watchdog. The Xbox worker re-sends a
        # held non-zero velocity every avg_interval (0.1 s); if commands
        # stop arriving (worker died, queue stalled, controller lost before
        # a zero-send) the jog loop must not keep applying the last
        # velocity forever. These *_at_velocity commands have no senders
        # other than the Xbox pipeline, so the timeout cannot break any
        # GUI-driven motion.
        self.stale_timeout: float = 0.5
        self._last_vel_cmd_time: float = time.time()

        self._was_moving = False
        self._running = False
        self._thread: threading.Thread | None = None
        self._pump_disabled_warned: set = set()  # debounce warnings

        # Register processor handlers
        self.processor.register_handler("move_z_at_velocity", self._handle_z_vel)
        self.processor.register_handler("move_p1_at_velocity", self._handle_p1_vel)
        self.processor.register_handler("move_p2_at_velocity", self._handle_p2_vel)
        self.processor.register_handler("move_p3_at_velocity", self._handle_p3_vel)
        self.processor.register_handler("increment_zspeed_up", self._incr_z_up)
        self.processor.register_handler("increment_zspeed_down", self._incr_z_down)
        self.processor.register_handler("increment_pspeed_up", self._incr_p_up)
        self.processor.register_handler("increment_pspeed_down", self._incr_p_down)

        # v7.2.6: ZP registered handlers — track for unregister on stop()
        self._registered_handlers: list = [
            ("move_z_at_velocity",       self._handle_z_vel),
            ("move_p1_at_velocity",      self._handle_p1_vel),
            ("move_p2_at_velocity",      self._handle_p2_vel),
            ("move_p3_at_velocity",      self._handle_p3_vel),
            ("increment_zspeed_up",      self._incr_z_up),
            ("increment_zspeed_down",    self._incr_z_down),
            ("increment_pspeed_up",      self._incr_p_up),
            ("increment_pspeed_down",    self._incr_p_down),
        ]

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._jog_loop, daemon=True, name="ZPJog"
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: ZP unregister — prevents stale callbacks on reconnect.
        """
        self._running = False
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_relative({}, None)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)


    def set_hardware_config(self, config):
        """Enable µL-based pump jog. p_speed becomes µL/s."""
        self._hardware_config = config
        self._pump_disabled_warned.clear()  # re-evaluate on config change
        # v7.5.x: the pump 100% anchor (p_speed_max) is pushed separately by
        # StageController.refresh_jog_speed_limits() once the flow limits are
        # known; just recompute the derived scalar against the current anchor.
        self._recompute_zp_speeds()
        if config and config.configured_pump_ids:
            logger.info("ZPJog: µL mode enabled (pump jog speed = "
                        f"{self.p_speed_pct:g}% of {self.p_speed_max:g} µL/s)")

    # ── v7.5.x: percentage-of-max speed plumbing ──────────────────
    def _recompute_zp_speeds(self) -> None:
        """Recompute the effective Z/pump scalars from the current % + max."""
        self.z_speed = max(0.0, self.z_speed_pct / 100.0 * self.z_speed_max)
        self.p_speed = max(0.0, self.p_speed_pct / 100.0 * self.p_speed_max)

    def set_z_speed_max(self, mm_s: float) -> None:
        """Set the Z 100% anchor (mm/s) and recompute the effective scalar."""
        try:
            v = float(mm_s)
        except (TypeError, ValueError):
            return
        if v > 0:
            self.z_speed_max = v
            self._recompute_zp_speeds()

    def set_p_speed_max(self, val: float) -> None:
        """Set the pump 100% anchor (µL/s in µL mode, mm/s legacy)."""
        try:
            v = float(val)
        except (TypeError, ValueError):
            return
        if v > 0:
            self.p_speed_max = v
            self._recompute_zp_speeds()

    def set_z_speed_pct(self, pct: float) -> None:
        try:
            self.z_speed_pct = float(pct)
        except (TypeError, ValueError):
            return
        self._recompute_zp_speeds()

    def set_p_speed_pct(self, pct: float) -> None:
        try:
            self.p_speed_pct = float(pct)
        except (TypeError, ValueError):
            return
        self._recompute_zp_speeds()

    @property
    def p_speed_is_uL(self) -> bool:
        """True when p_speed represents µL/s (HardwareConfig available)."""
        hw = self._hardware_config
        return hw is not None and bool(hw.configured_pump_ids)

    def speeds(self) -> dict[str, float]:
        return {"z": self.z_speed, "p": self.p_speed,
                "z_pct": self.z_speed_pct, "p_pct": self.p_speed_pct}

    # ── Velocity Extraction ───────────────────────────────────────

    @staticmethod
    def _extract_velocity(*args, **kwargs) -> float:
        """Pull a scalar velocity from Xbox axis kwargs."""
        val = kwargs.get("average", 0.0)
        if isinstance(val, (list, tuple)):
            return val[1] if len(val) > 1 else val[0]
        return float(val)

    def _clamp_vel(self, raw: float, multiplier: float,
                   cap: float | None = None) -> float:
        # v7.5.x: optional explicit cap so Z clamps at its own z_speed_max
        # (100% anchor) rather than the shared legacy max_speed.
        c = self.max_speed if cap is None else cap
        return max(-c, min(c, raw * multiplier))

    def _flip_sign(self, logical: str) -> float:
        """v7.5.x: direction-flip sign for a logical axis (Z/P1/P2/P3).

        Mirrors StageController._flip_sign so the Xbox jog inverts the same
        axes the GUI jog buttons do. Returns 1.0 when no provider is set or
        the axis isn't flipped.
        """
        if self._flip_provider is None:
            return 1.0
        try:
            return float(self._flip_provider(logical))
        except Exception:
            return 1.0

    def _z_up_sign(self) -> float:
        """v7.5.x: per-machine Z up-direction sign (±1.0) for the Xbox jog.

        Mirrors ``StageController.z_up_sign`` so stick-up retracts the needle
        toward the taught Top regardless of raw-Z polarity. Falls back to the
        module ``ZDIR`` when no provider is wired (legacy / pre-setup), which
        reproduces the old ``-vz`` Z jog exactly.
        """
        if self._z_up_sign_provider is None:
            return ZDIR
        try:
            s = float(self._z_up_sign_provider())
            return s if s != 0.0 else ZDIR
        except Exception:
            return ZDIR

    def _pump_dir_sign(self, logical: str) -> float:
        """v7.5.x: dispense-direction sign for a pump (raw mm per mm of dispense
        intent), OWNED by the plunger calibration. Mirrors
        ``StageController.pump_dir_sign`` so the Xbox pump jog moves the same
        direction as ``move_pump_relative``. Falls back to ``_flip_sign`` when no
        provider is wired (uncalibrated / legacy)."""
        if self._pump_dir_sign_provider is None:
            return self._flip_sign(logical)
        try:
            return float(self._pump_dir_sign_provider(logical))
        except Exception:
            return self._flip_sign(logical)

    def _xbox_pump_ceiling_uL_s(self, pump_id: str) -> float:
        """This pump's OWN 100 %-flow ceiling (µL/s), or 0 when unknown.

        v7.9.1. Read straight from ``SafetyLimits.get_max_flow_rate`` — the same
        source ``StageController.get_max_pump_feedrate_for`` uses — so the Xbox
        and the on-screen tiles resolve a pump's rate from one authority
        instead of two. 0 means "no answer", and the caller falls back to the
        legacy shared scalars rather than guessing.
        """
        sl = self.safety_limits
        if sl is None or not hasattr(sl, "get_max_flow_rate"):
            return 0.0
        try:
            r = float(sl.get_max_flow_rate(pump_id))
        except Exception:
            return 0.0
        return r if r > 0 else 0.0

    def _xbox_pump_pct(self, pump_id: str) -> float:
        """The % of that ceiling a full stick deflection commands.

        Deliberately the SHARED 'p' ladder value: the Xbox has one physical
        speed control for the pump group, so per-pump percentages would leave
        its buttons meaningless. Independence comes from the per-pump ceiling.
        """
        pct = float(getattr(self, "p_speed_pct", 0.0) or 0.0)
        return pct if pct > 0 else 100.0

    def _pump_vel_mm_s(self, raw: float, pump_id: str) -> float:
        """Convert raw Xbox axis to pump velocity in mm/s.

        When HardwareConfig is available, p_speed is in µL/s and we
        convert per-pump using the syringe geometry.  Otherwise p_speed
        is a raw mm/s multiplier (legacy).
        """
        hw = self._hardware_config
        if hw:
            pump_cfg = hw.pumps.get(pump_id)
            if pump_cfg and pump_cfg.is_configured:
                # v7.9.1: resolve the µL/s from THIS pump's own ceiling rather
                # than the one shared `p_speed_max`. Those ceilings differ by
                # orders of magnitude between bores (v7.9 measured ~2000×
                # between a 22G and a 30 µm pulled tip), so a single anchor
                # meant a stick deflection drove a fine bore at a coarse bore's
                # rate. The Xbox's %-ladder stays SHARED — it is one physical
                # control for the whole "p" group, and splitting it would leave
                # its buttons with no defined meaning — but the rate that %
                # resolves to is now per-pump.
                pct = self._xbox_pump_pct(pump_id)
                anchor_uL_s = self._xbox_pump_ceiling_uL_s(pump_id)
                target_uL_s = raw * (pct / 100.0 * anchor_uL_s
                                     if anchor_uL_s > 0 else self.p_speed)
                vel_mm_s = pump_cfg.uL_to_mm(abs(target_uL_s))
                # Cap at this pump's own 100%-flow ceiling (mm/s) so a full
                # deflection can't exceed the anchored max; the per-pump safe
                # rate is still enforced by _clamp_pump_flow downstream.
                try:
                    cap_mm_s = pump_cfg.uL_to_mm(
                        anchor_uL_s if anchor_uL_s > 0 else self.p_speed_max)
                except Exception:
                    cap_mm_s = self.max_speed
                vel_mm_s = min(vel_mm_s, cap_mm_s)
                return vel_mm_s if target_uL_s >= 0 else -vel_mm_s
        # Fallback: mm/s mode — anchor the cap to the (mm/s) pump max.
        return self._clamp_vel(raw, self.p_speed, cap=self.p_speed_max)

    # ── Command Handlers ──────────────────────────────────────────

    def _is_pump_enabled(self, pump_id: str) -> bool:
        """Check if pump is enabled in hardware config. True if no config."""
        hw = self._hardware_config
        if hw is None:
            return True
        pump_cfg = hw.pumps.get(pump_id)
        if pump_cfg is None:
            return True
        return pump_cfg.is_configured

    def _handle_z_vel(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self._lock:
            # v7.5.x: 100% (full stick) reaches z_speed_max and never exceeds it.
            self.vel_z = self._clamp_vel(raw, self.z_speed, cap=self.z_speed_max)
            self._last_vel_cmd_time = time.time()  # v7.5.x: watchdog feed

    def _clamp_pump_flow(self, vel, pump_id):
        """Clamp pump velocity (mm/s) to max safe flow rate."""
        hw = self._hardware_config
        if hw is None:
            return vel
        sl = self.safety_limits
        if sl is None:
            return vel
        try:
            max_rate = sl.get_max_flow_rate(pump_id)
            if max_rate is not None and max_rate > 0:
                pump_cfg = hw.pumps.get(pump_id)
                if pump_cfg and pump_cfg.is_configured:
                    rate = abs(pump_cfg.mm_to_uL(abs(vel)))
                    if rate > max_rate:
                        clamped_mm = pump_cfg.uL_to_mm(max_rate)
                        vel = clamped_mm if vel > 0 else -clamped_mm
        except Exception:
            pass
        return vel

    def _handle_pump_vel(self, pump_id: str, vel_attr: str, *args, **kwargs):
        """Common handler for pump velocity commands."""
        raw = self._extract_velocity(*args, **kwargs)
        if not self._is_pump_enabled(pump_id):
            # Block movement — warn once per pump
            if pump_id not in self._pump_disabled_warned and abs(raw) > 0.05:
                self._pump_disabled_warned.add(pump_id)
                logger.warning(
                    f"[Xbox] {pump_id} is disabled — ignoring jog. "
                    f"Enable {pump_id} in Hardware Setup or change your Xbox mapping."
                )
            with self._lock:
                setattr(self, vel_attr, 0.0)
            return
        # Clear warning debounce when pump becomes enabled again
        self._pump_disabled_warned.discard(pump_id)
        vel = self._pump_vel_mm_s(raw, pump_id)
        vel = self._clamp_pump_flow(vel, pump_id)
        with self._lock:
            prev_vel = getattr(self, vel_attr, 0.0)
            setattr(self, vel_attr, vel)
            self._last_vel_cmd_time = time.time()  # v7.5.x: watchdog feed
        # v7.3.4: Pump velocity trace — debug level so it only appears when
        # the app log level is set to DEBUG (or Xbox debug mode is on).
        logger.debug(
            f"[ZPJog] {pump_id} vel: {prev_vel:.4f} → {vel:.4f}  (raw={raw:.4f})"
        )

    def _handle_p1_vel(self, *args, **kwargs):
        self._handle_pump_vel("P1", "vel_p1", *args, **kwargs)

    def _handle_p2_vel(self, *args, **kwargs):
        self._handle_pump_vel("P2", "vel_p2", *args, **kwargs)

    def _handle_p3_vel(self, *args, **kwargs):
        self._handle_pump_vel("P3", "vel_p3", *args, **kwargs)

    def _incr_z_up(self, *a, **kw):
        # v7.5.x: step the percentage-of-max ladder instead of ×10 decades.
        self.z_speed_pct = _jog_speed_ladder_step(self.z_speed_pct, +1)
        self._recompute_zp_speeds()
        logger.info(f"Z jog speed: {self.z_speed_pct:g}% of max "
                    f"({self.z_speed:.3f} mm/s)")

    def _incr_z_down(self, *a, **kw):
        self.z_speed_pct = _jog_speed_ladder_step(self.z_speed_pct, -1)
        self._recompute_zp_speeds()
        logger.info(f"Z jog speed: {self.z_speed_pct:g}% of max "
                    f"({self.z_speed:.3f} mm/s)")

    def _incr_p_up(self, *a, **kw):
        self.p_speed_pct = _jog_speed_ladder_step(self.p_speed_pct, +1)
        self._recompute_zp_speeds()
        unit = "µL/s" if self.p_speed_is_uL else "mm/s"
        logger.info(f"Pump jog speed: {self.p_speed_pct:g}% of max "
                    f"({self.p_speed:.3f} {unit})")

    def _incr_p_down(self, *a, **kw):
        self.p_speed_pct = _jog_speed_ladder_step(self.p_speed_pct, -1)
        self._recompute_zp_speeds()
        unit = "µL/s" if self.p_speed_is_uL else "mm/s"
        logger.info(f"Pump jog speed: {self.p_speed_pct:g}% of max "
                    f"({self.p_speed:.3f} {unit})")


    def _jog_loop(self) -> None:
        while self._running:
            with self._lock:
                vz, vp1, vp2, vp3 = self.vel_z, self.vel_p1, self.vel_p2, self.vel_p3
                last_cmd = self._last_vel_cmd_time

            is_moving = any(abs(v) > 0.001 for v in (vz, vp1, vp2, vp3))

            # v7.5.x: staleness watchdog — the worker re-sends held values
            # every 0.1 s, so a non-zero velocity with no command for
            # stale_timeout means the pipeline died mid-hold. Stop.
            if is_moving and (time.time() - last_cmd) > self.stale_timeout:
                logger.warning("[ZP] Velocity commands stale — zeroing jog")
                with self._lock:
                    self.vel_z = self.vel_p1 = self.vel_p2 = self.vel_p3 = 0.0
                continue

            if is_moving and not self._was_moving:
                logger.debug(f"[ZP] Jog start: z={vz:.2f} p1={vp1:.2f}")
            elif not is_moving and self._was_moving:
                logger.debug("[ZP] Jog stop")
            self._was_moving = is_moving

            if not is_moving:
                time.sleep(0.01)
                continue

            # v7.5.x: Z direction comes from the per-machine z_up_sign (the Set
            # Bottom/Top setup) so stick-up retracts the needle toward the
            # taught Top — exactly like the GUI buttons (move_z_user_relative).
            # stick-up delivers vz > 0; dz = vz·seg·z_up_sign. This is
            # algebraically identical to the old "-vz·seg" when z_up_sign = -1
            # (ZDIR), and correctly flips when the setup derived +1. Pumps keep
            # the per-axis _flip_sign.
            # v7.5.x: pump direction is OWNED by the plunger calibration
            # (_pump_dir_sign → −aspirate_sign once calibrated), mirroring how Z
            # uses z_up_sign; uncalibrated pumps fall back to _flip_sign.
            dz = vz * self.segment_time * self._z_up_sign()
            dp1 = vp1 * self.segment_time * self._pump_dir_sign("P1")
            dp2 = vp2 * self.segment_time * self._pump_dir_sign("P2")
            dp3 = vp3 * self.segment_time * self._pump_dir_sign("P3")

            # Apply safety limits
            if self.safety_limits and self.safety_limits.enabled and self._get_zp_position:
                try:
                    pos = self._get_zp_position()
                    if pos[0] is not None:
                        # v7.5.x bugfix: resolve each logical axis to its
                        # physical tuple slot via the live axis_map. The
                        # old positional unpack (cz,cp1,cp2,cp3 = pos)
                        # assumed Z,P1,P2,P3 order, so it clamped the wrong
                        # motor's position on non-default maps (ME3B V1).
                        # v7.5.x: Z/pump envelope is absolute Marlin raw mm —
                        # clamp the absolute destination (cur + delta) directly.
                        # (Was zero-ref: clamp((cur+delta)-zero)+zero-cur, which
                        # went stale after a Set Z Zero / needle-zero re-anchor
                        # and wrongly clamped the Xbox jog.)
                        # v7.5.x: shorten-only clamp — see _shorten_only_delta.
                        # An out-of-bounds CACHED position must NOT make the
                        # loop synthesize a large opposite-direction delta and
                        # re-send it every segment (the "kept moving constantly"
                        # runaway). The clamp may only shorten the jog.
                        def _clamp_delta(logical, delta, clamp_fn):
                            i = _axis_index(self.stage, logical)
                            if i is None or i >= len(pos) or pos[i] is None:
                                return delta
                            cur = pos[i]
                            return _shorten_only_delta(
                                cur, delta, clamp_fn(cur + delta))

                        dz = _clamp_delta("Z", dz, self.safety_limits.clamp_z)
                        dp1 = _clamp_delta(
                            "P1", dp1,
                            lambda v: self.safety_limits.clamp_pump(v, "P1"))
                        dp2 = _clamp_delta(
                            "P2", dp2,
                            lambda v: self.safety_limits.clamp_pump(v, "P2"))
                        dp3 = _clamp_delta(
                            "P3", dp3,
                            lambda v: self.safety_limits.clamp_pump(v, "P3"))
                except Exception:
                    pass

            combined = math.sqrt(vz**2 + vp1**2 + vp2**2 + vp3**2)
            feedrate = max(combined * 60, 1)

            if self.safety_limits and self.safety_limits.enabled:
                feedrate = min(feedrate, self.safety_limits.max_z_feedrate)

            # v7.2.6: ZP jog serial guard — protect against disconnected stage
            try:
                # v7.5.x bugfix: route logical deltas to physical letters
                # via the live axis_map (was hardcoded X/Y/Z/E = default
                # map, which drove the wrong motors on ME3B V1).
                deltas: dict[str, float] = {}
                for logical, d in (("Z", dz), ("P1", dp1),
                                   ("P2", dp2), ("P3", dp3)):
                    letter = _axis_letter(self.stage, logical)
                    if letter is not None:
                        deltas[letter] = d
                self.stage.move_relative(deltas, feedrate)
                # v7.5.x: feed the display-only motion estimator the commanded
                # per-segment logical deltas (raw mm) so the live readout tracks
                # the continuous jog between ~300 ms polls. Guarded.
                cb = getattr(self, "on_jog_estimate", None)
                if cb is not None:
                    cb({"Z": dz, "P1": dp1, "P2": dp2, "P3": dp3})
            except Exception as e:
                logger.warning(f"[ZP] Jog move failed: {e}")
                with self._lock:
                    self.vel_z = self.vel_p1 = self.vel_p2 = self.vel_p3 = 0.0
                break

            time.sleep(self.segment_time)

class XYJogHandler:
    """
    Sends continuous velocity commands to the XY stage.
    Runs its own daemon thread.

    Proven working from XBOXCONTROLLED code — core logic unchanged.
    """

    def __init__(
        self,
        processor: Processor,
        xy_stage: XYStageManager,
        safety_limits: SafetyLimits | None = None,
        get_xy_position: Callable | None = None,
    ):
        self.processor = processor
        self.stage = xy_stage
        self.safety_limits = safety_limits
        self._get_xy_position = get_xy_position

        self.vel_x: float = 0.0
        self.vel_y: float = 0.0
        self._lock = threading.Lock()

        # Tuning
        # v7.5.x: jog speed = a PERCENTAGE of the calibrated XY max (µm/s).
        # ``xy_speed`` (effective scalar) and ``max_speed`` (velocity cap) are
        # both derived from speed_pct + speed_max; the 100% anchor (speed_max)
        # is pushed by StageController.refresh_jog_speed_limits().
        self.speed_pct: float = 10.0
        self.speed_max: float = 10000.0   # µm/s at 100% (= safety max_xy_speed)
        self.xy_speed: float = 0.0        # derived
        self.max_speed: float = 0.0       # derived (= speed_max)
        self.update_interval: float = 0.1
        self._recompute_xy_speed()

        # v7.5.x: velocity staleness watchdog (see ZPJogHandler).
        self.stale_timeout: float = 0.5
        self._last_vel_cmd_time: float = time.time()

        self._was_moving = False
        self._running = False
        self._thread: threading.Thread | None = None

        self.processor.register_handler("move_stage_at_velocity", self._handle_vel)
        self.processor.register_handler("increment_xyspeed_up", self._incr_up)
        self.processor.register_handler("increment_xyspeed_down", self._incr_down)

        # v7.2.6: XY registered handlers
        self._registered_handlers: list = [
            ("move_stage_at_velocity", self._handle_vel),
            ("increment_xyspeed_up",   self._incr_up),
            ("increment_xyspeed_down", self._incr_down),
        ]


    # v7.2: Hardware config for µL conversion
    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Enable µL-based pump jog when hardware config available."""
        self._hardware_config = config

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._jog_loop, daemon=True, name="XYJog"
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: XY unregister — prevents stale callbacks on reconnect.
        """
        self._running = False
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_stage_at_velocity(0, 0)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)


    def speed(self) -> float:
        return self.xy_speed

    # ── v7.5.x: percentage-of-max speed plumbing ──────────────────
    def _recompute_xy_speed(self) -> None:
        """Recompute the effective XY scalar + cap from the current % + max.
        100% (full stick) reaches speed_max and never exceeds it."""
        self.xy_speed = max(0.0, self.speed_pct / 100.0 * self.speed_max)
        self.max_speed = self.speed_max

    def set_speed_max(self, um_s: float) -> None:
        """Set the XY 100% anchor (µm/s) and recompute the effective scalar."""
        try:
            v = float(um_s)
        except (TypeError, ValueError):
            return
        if v > 0:
            self.speed_max = v
            self._recompute_xy_speed()

    def set_speed_pct(self, pct: float) -> None:
        try:
            self.speed_pct = float(pct)
        except (TypeError, ValueError):
            return
        self._recompute_xy_speed()

    def _handle_vel(self, *args, **kwargs):
        val = kwargs.get("average", (0, 0))
        if isinstance(val, (list, tuple)) and len(val) >= 2:
            vx, vy = val[0], val[1]
        else:
            vx, vy = 0.0, 0.0
        with self._lock:
            self.vel_x = max(-self.max_speed, min(self.max_speed, vx * self.xy_speed))
            self.vel_y = max(-self.max_speed, min(self.max_speed, vy * self.xy_speed))
            self._last_vel_cmd_time = time.time()  # v7.5.x: watchdog feed

    def _incr_up(self, *a, **kw):
        # v7.5.x: step the percentage-of-max ladder instead of ×10 decades.
        self.speed_pct = _jog_speed_ladder_step(self.speed_pct, +1)
        self._recompute_xy_speed()
        logger.info(f"XY jog speed: {self.speed_pct:g}% of max "
                    f"({self.xy_speed:.0f} µm/s)")


    def _incr_down(self, *a, **kw):
        self.speed_pct = _jog_speed_ladder_step(self.speed_pct, -1)
        self._recompute_xy_speed()
        logger.info(f"XY jog speed: {self.speed_pct:g}% of max "
                    f"({self.xy_speed:.0f} µm/s)")


    def _jog_loop(self) -> None:
        while self._running:
            with self._lock:
                vx, vy = self.vel_x, self.vel_y
                last_cmd = self._last_vel_cmd_time

            is_moving = abs(vx) > 0.001 or abs(vy) > 0.001

            # v7.5.x: staleness watchdog — see ZPJogHandler._jog_loop.
            # Zeroing here makes the next iteration take the was_moving
            # transition, which sends the explicit zero-velocity stop the
            # ProScan needs in continuous velocity mode.
            if is_moving and (time.time() - last_cmd) > self.stale_timeout:
                logger.warning("[XY] Velocity commands stale — zeroing jog")
                with self._lock:
                    self.vel_x = self.vel_y = 0.0
                continue

            if is_moving and not self._was_moving:
                logger.debug(f"[XY] Jog start: x={vx:.1f} y={vy:.1f}")
            elif not is_moving and self._was_moving:
                logger.debug("[XY] Jog stop")
                # Send explicit zero-velocity so the ProScan stops immediately.
                # The stage uses continuous velocity mode and will keep moving
                # until a zero command is received.
                try:
                    self.stage.move_stage_at_velocity(0, 0)
                except Exception as e:
                    logger.warning(f"[XY] Stop command failed: {e}")
            self._was_moving = is_moving

            if not is_moving:
                time.sleep(0.01)
                continue

            # Dampen near limits
            if is_moving and self.safety_limits and self.safety_limits.enabled and self._get_xy_position:
                try:
                    pos = self._get_xy_position()
                    if pos[0] is not None:
                        # BUG-5: margin=500 microsteps ≈ 50µm at 10 steps/µm
                        # This slows jog speed when within 50µm of a limit
                        near = self.safety_limits.check_xy_near_limit(pos[0], pos[1], margin=500)
                        if near["x_near_min"] and vx < 0:
                            vx *= 0.3
                        if near["x_near_max"] and vx > 0:
                            vx *= 0.3
                        if near["y_near_min"] and vy < 0:
                            vy *= 0.3
                        if near["y_near_max"] and vy > 0:
                            vy *= 0.3
                except Exception:
                    pass

            # v7.2.6: XY jog serial guard
            try:
                self.stage.move_stage_at_velocity(vx, vy)
                # v7.5.x: feed the display-only motion estimator the commanded
                # per-segment delta (µm) so the live readout/needle track the
                # continuous jog smoothly between ~300 ms polls. Guarded.
                cb = getattr(self, "on_jog_estimate", None)
                if cb is not None:
                    cb(vx * self.update_interval, vy * self.update_interval)
            except Exception as e:
                logger.warning(f"[XY] Jog move failed: {e}")
                with self._lock:
                    self.vel_x = self.vel_y = 0.0
                break
            time.sleep(self.update_interval)


# ═══════════════════════════════════════════════════════════════════
# Position Poller
# ═══════════════════════════════════════════════════════════════════

class PositionPoller:
    """
    Background thread that caches stage positions at regular intervals.
    Prevents the GUI from blocking on serial I/O.
    """

    def __init__(self, poll_interval: float = 0.3):
        self.poll_interval = poll_interval
        self._xy_stage: XYStageManager | None = None
        self._zp_stage: ZPStageManager | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._xy_pos: tuple = (None, None, None)
        self._zp_pos: tuple = (None, None, None, None)
        # v7.21.2: REFCOUNTED, not a bool. The XY calibration nests suspends —
        # the orchestrator holds one across the whole run while each sub-probe
        # (dead time, top speed) owns its own pair. With a plain bool the FIRST
        # sub-probe's `finally` resumed polling for everything after it, so every
        # later step was measured against a contended serial read path.
        # `_suspended` stays readable as a bool via the property below, so every
        # existing `if poller._suspended:` reader is unchanged.
        self._suspend_depth = 0  # v7.3.4: pause polling during programmatic moves

        # v7.5.x: XY travel odometer. ``on_xy_travel(dist_um)`` is fired for each
        # poll sample with the straight-line XY distance from the previous
        # sample, so ALL motion (jog + programmatic) is captured at one point.
        # ``_last_odom_xy`` is the previous absolute-µm (x, y); reset to None on
        # (re)connect/disconnect so we never diff across a coordinate re-frame.
        self.on_xy_travel: Callable | None = None
        self._last_odom_xy: tuple | None = None

        # v7.2.7: init _hardware_config
        self._hardware_config = None

        # v7.5.x ZP reconnect hotfix: poller-driven liveness. A real ZP stage
        # that stops answering position queries for this many consecutive
        # polls is treated as disconnected (covers board power-off with USB
        # still attached, which the port-handle watchdog cannot detect).
        # ``on_zp_lost`` is set by StageController to fire _handle_disconnect.
        self.on_zp_lost: Callable | None = None
        self._zp_fail_count = 0
        # ~2.5 s of consecutive failed ZP position reads before declaring a
        # disconnect — long enough to ride out a brief board hiccup, short
        # enough to catch a power-off quickly. Scaled to the poll interval.
        self._zp_fail_threshold = max(5, int(2.5 / max(poll_interval, 0.05)))

    @property
    def _suspended(self) -> bool:
        """True while ANY caller holds a suspend. Kept as the historical
        attribute name so external readers (`getattr(p, "_suspended", False)`)
        need no change."""
        return self._suspend_depth > 0

    def suspend(self) -> None:
        """Pause hardware queries (e.g., during safe_travel_to) to prevent
        serial races between the poll thread and caller-side waits.

        v7.21.2: nestable. Each `suspend()` must be matched by a `resume()`.
        """
        with self._lock:
            self._suspend_depth += 1

    def resume(self) -> None:
        """Resume normal polling after a programmatic move completes."""
        # v7.5.x ZP-disconnect fix: start a FRESH liveness window on resume.
        # _zp_fail_count is only meaningful as "consecutive failures while
        # actively polling". Failures accrued during/around a suspend (e.g. a
        # print's PRINT_PATH where the poller contends with the per-segment
        # write stream, or a long safe_travel_to) must NOT carry over and trip
        # the threshold on the very first post-resume read — that manufactures
        # a false "ZP disconnected" mid-print. Mirrors set_stages()'s reset.
        self._zp_fail_count = 0
        # v7.21.2: only the OUTERMOST resume actually resumes. Clamped at zero so
        # an unbalanced resume cannot poison a later suspend (the same rule the
        # v7.9 print-floor refcount adopted).
        with self._lock:
            self._suspend_depth = max(0, self._suspend_depth - 1)

    def set_stages(
        self,
        xy_stage: XYStageManager | None = None,
        zp_stage: ZPStageManager | None = None,
    ) -> None:
        with self._lock:
            self._xy_stage = xy_stage
            self._zp_stage = zp_stage
            if xy_stage is None:
                self._xy_pos = (None, None, None)
            if zp_stage is None:
                self._zp_pos = (None, None, None, None)
            # v7.5.x: fresh liveness window for any (re)connect/disconnect.
            self._zp_fail_count = 0
            # v7.5.x: a new/changed XY stage means a new coordinate frame — drop
            # the odometer's previous sample so we don't count a phantom jump.
            self._last_odom_xy = None

    @property
    def xy_position(self) -> tuple:
        with self._lock:
            return self._xy_pos

    @property
    def zp_position(self) -> tuple:
        with self._lock:
            return self._zp_pos

    def note_xy(self, pos: tuple) -> None:
        """v7.6: back-fill the XY cache from an out-of-band DIRECT read.

        ``StageController.get_xy_position(cached=False)`` calls this on every
        successful read, which keeps the cache live even while polling is
        SUSPENDED — the print path suspends the poller for the whole
        ``PRINT_PATH`` command but reads position itself at 25–31 Hz, so before
        this the GUI's cached reads froze for the entire print (needle marker
        stuck, progress dead) despite fresher truth existing one thread away.

        Deliberately does NOT touch the travel odometer (``_last_odom_xy`` /
        ``on_xy_travel``) or the ZP liveness counters — those stay
        poll-thread-only so distance accounting cannot be double-counted.
        """
        if not pos or pos[0] is None or pos[1] is None:
            return
        with self._lock:
            self._xy_pos = tuple(pos)

    def note_zp(self, pos: tuple) -> None:
        """v7.6: ZP twin of :meth:`note_xy` (Z + pump cache back-fill)."""
        if not pos or pos[0] is None:
            return
        with self._lock:
            self._zp_pos = tuple(pos)

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="PositionPoller"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _accumulate_xy_travel(self, x_um: float, y_um: float) -> None:
        """Fire ``on_xy_travel`` with the straight-line distance from the
        previous poll sample. Best-effort — a callback error must never break
        polling. Called from the poll thread with a fresh (x, y) in µm."""
        prev = self._last_odom_xy
        self._last_odom_xy = (x_um, y_um)
        cb = self.on_xy_travel
        if prev is None or cb is None:
            return
        try:
            dist = math.hypot(x_um - prev[0], y_um - prev[1])
            if dist > 0.0:
                cb(dist)
        except Exception as e:
            logger.debug(f"XY odometer error: {e}")

    def _poll_loop(self) -> None:
        while self._running:
            if self._suspended:
                time.sleep(0.05)
                continue

            with self._lock:
                xy, zp = self._xy_stage, self._zp_stage

            if xy is not None:
                try:
                    pos = xy.get_current_position()
                    if pos[0] is not None and pos[1] is not None:
                        with self._lock:
                            self._xy_pos = pos
                        self._accumulate_xy_travel(pos[0], pos[1])
                        if _dbg.is_enabled() and getattr(xy, 'simulate', False):
                            _dbg.log("POLL", pos_x=pos[0], pos_y=pos[1], pos_z=pos[2],
                                     note="simulated")
                except Exception as e:
                    logger.debug(f"XY poll error: {e}")

            if zp is not None:
                ok = False
                try:
                    pos = zp.get_current_position()
                    # get_current_position() returns the cached (possibly
                    # stale) floats even when the board didn't answer, so
                    # gate freshness on the explicit read-ok flag rather than
                    # on pos[0] being non-None.
                    fresh = getattr(zp, "_last_position_read_ok", True)
                    if pos[0] is not None and fresh:
                        with self._lock:
                            self._zp_pos = pos
                        ok = True
                except Exception as e:
                    logger.debug(f"ZP poll error: {e}")
                # v7.5.x ZP reconnect hotfix: track consecutive query
                # failures on real hardware. A board that has been powered
                # off (USB still attached) keeps its COM port "open" so the
                # port-handle watchdog never fires — but it stops answering
                # M114, which we catch here and escalate to a disconnect.
                if ok:
                    self._zp_fail_count = 0
                elif not getattr(zp, "simulate", False):
                    self._zp_fail_count += 1
                    if (self._zp_fail_count >= self._zp_fail_threshold
                            and self.on_zp_lost is not None):
                        cb = self.on_zp_lost
                        self._zp_fail_count = 0
                        logger.warning(
                            "ZP position queries failing — treating as "
                            "disconnected (poller-driven liveness)")
                        try:
                            cb()
                        except Exception as e:
                            logger.error(f"ZP lost callback error: {e}")

            time.sleep(self.poll_interval)


# ═══════════════════════════════════════════════════════════════════
# Stage Controller (Top-Level Orchestrator)
# ═══════════════════════════════════════════════════════════════════

class StageController:
    """
    Main entry point that the GUI (or headless mode) interacts with.

    Owns the Processor, stages, jog handlers, Xbox controller,
    safety limits, and position logger.
    """

    def __init__(
        self,
        simulate_xy: bool = False,
        simulate_zp: bool = False,
        controller_json: str | None = None,
        poll_interval: float = 0.3,
        watchdog_interval: float = 2.0,
    ):
        # v7.4.2: per-device simulation is decided at connect time
        # (Connect = real hardware, Simulate = simulator). The init
        # kwargs are kept as fallback defaults for callers — notably
        # ``--simulate-xy`` / ``--simulate-zp`` in headless mode — that
        # invoke ``connect_stages()`` without an explicit ``simulate``
        # argument. ``self.simulate_xy`` / ``self.simulate_zp`` are now
        # read-only properties that reflect the live stage's mode.
        self._default_simulate_xy = bool(simulate_xy)
        self._default_simulate_zp = bool(simulate_zp)

        # v7.1 P8.27: Controller JSON path (passed through to XYStageManager)
        # "auto" = auto-detect, None = default ProScan III, or explicit path
        self.controller_json = controller_json

        # Core command processor
        self.processor = Processor()

        # Hardware stages (created on connect)
        self.xy_stage: XYStageManager | None = None
        self.zp_stage: ZPStageManager | None = None

        # Jog handlers (created after stages)
        self.xy_jog: XYJogHandler | None = None
        self.zp_jog: ZPJogHandler | None = None

        # Xbox controller
        self.xbox_queue: Queue | None = None
        self.xbox_process: Process | None = None
        self.xbox_poller: XboxQueuePoller | None = None
        # v7.5.x: live tuning channel + clean worker stop
        self.xbox_ctrl_queue: Queue | None = None
        self._xbox_stop_event = None

        # v7.4.2: Device settings cached until ZP stage connects
        self._pending_axis_map: dict | None = None
        self._pending_steps_per_mm: dict | None = None
        # v7.4.2 hotfix: per-axis M203 ceilings (logical → mm/min).
        # Pushed to Marlin once the ZP stage connects so software and
        # firmware start in sync.
        self._pending_per_axis_max_feedrate: dict | None = None
        # v7.5.x: per-group Xbox jog speed % (XY / Z / Pump). Owned here so it
        # survives (re)connect — the GUI restores the persisted value once and
        # the controller re-applies it whenever the jog handlers are recreated
        # (see set_jog_speed_pct / _apply_jog_speed_pct / refresh_jog_speed_limits).
        self._jog_speed_pct: dict[str, float] = {}
        # v7.9.x: PER-PUMP jog flow % (the jog tiles' P1/P2/P3 flow spinboxes).
        # Owned here — not on the panels — so every jog tile reads the same
        # value (nine pages each build their own StandardJogContextPanel; the
        # illumination-LED lesson is that per-panel state silently diverges).
        # Keyed "P1"/"P2"/"P3"; absent = fall back to the shared 'p' group %.
        self._pump_jog_pct: dict[str, float] = {}
        # v7.9.x: jog step-slider configuration (snap + per-axis range/value),
        # shared across every jog tile for the same reason as _pump_jog_pct.
        self._jog_step_settings: dict = {}
        # v7.4.2 hotfix: last-known-good ZP serial port. Tried first on
        # connect_stages() so we skip the rediscovery scan. Set by
        # the caller from settings (zp_stage.last_port) and re-saved
        # whenever the ZP stage reports its connected_port.
        self._preferred_zp_port: str | None = None
        # v7.18.1: last-known-good XY detection hint
        # ({protocol, port, baud}) — the XY twin of _preferred_zp_port.
        self._preferred_xy_hint: dict | None = None

        # v7.5.x: True once a REAL ZP board has connected this session. Sticky
        # (never reset on disconnect) so safe_travel_to can tell a dropped board
        # (needle present, now unretractable → refuse XY) from a rig that never
        # had one. See _needle_present() / safe_travel_to.
        self._zp_ever_connected: bool = False

        # Zero reference (set during calibration)
        self.zero_position: dict[str, float] = {
            "x": 0.0, "y": 0.0, "f": 0.0,
            "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0,
        }

        # Safety limits
        self.safety_limits = SafetyLimits()

        # Position logger
        self.position_logger = PositionLogger()

        # Watchdog for real hardware disconnects
        self._watchdog = ConnectionWatchdog(check_interval=watchdog_interval)
        self._watchdog.start()

        # v7.5.x ZP reconnect hotfix: serialize disconnect handling — it can
        # now be triggered from BOTH the watchdog thread (port-handle health)
        # and the poller thread (position-query liveness), so guard against
        # two threads tearing the same stage down concurrently.
        self._disconnect_lock = threading.Lock()

        # Background position cache — interval from settings (default 0.3s)
        self._pos_poller = PositionPoller(poll_interval=poll_interval)
        # v7.5.x: poller-driven liveness — detect a ZP stage that stops
        # answering position queries (covers board power-off with USB still
        # attached, which the port-handle watchdog cannot see).
        self._pos_poller.on_zp_lost = lambda: self._handle_disconnect("ZP")
        # v7.5.x: XY travel odometer for the calibration-usability pop-up —
        # accumulate every poll-sample chord into the per-machine store.
        self._pos_poller.on_xy_travel = self._note_xy_travel_um
        self._pos_poller.start()

        # v7.5.x: display-only motion interpolation. A jog/travel move is
        # fire-and-forget and the real position can't be polled mid-move, so the
        # GUI position readouts (via get_display_* below) ANIMATE start→target by
        # elapsed·speed instead of snapping, then snap to the real cache on
        # arrival. Pure prediction — never touches hardware/motion/safety. The
        # settle window is one poll interval + slack so a continuous-jog estimate
        # persists until the poller has caught up after the stick releases.
        self._motion = MotionEstimator(
            settle_s=max(0.4, float(poll_interval) + 0.1))

        # Disconnect callback (GUI can set this)
        self.on_disconnect: Callable | None = None
        # v7.5.x: fired with the stage name ("XY"/"ZP") after a successful
        # (re)connect. The GUI uses the ZP edge to offer last-known-position
        # restore (Marlin has no absolute encoder). May fire on a worker
        # thread (onboarding) — the GUI handler must bridge to its own thread.
        self.on_connect: Callable | None = None
        # v7.5.x: fired (no args) when a per-axis MAX speed changes from a
        # non-page source (e.g. the timing tool's "Measure top speed" worker)
        # so the GUI can fan the refresh out to every jog/speed surface through
        # its established `safety_limits_changed` channel. Page-driven edits use
        # that Qt signal directly; this callback only serves backend emitters.
        self.on_speed_limits_changed: Callable | None = None

        # v7.21.1: the operator-DECLARED true XY top speed (µm/s at 100%), from
        # Hardware Setup → Device → XY Stage Calibration → "Max speed", or from
        # the timing tool's measurement once the operator Applies it. None =
        # never declared, which is DELIBERATELY distinguishable from "declared
        # 10000": `SafetyLimits.max_xy_speed` defaults to 10000 µm/s, and
        # pushing a DEFAULT into the mm/s↔SMS-% denominator would make every
        # commanded speed run FASTER than requested on a stage whose real top
        # speed is higher (SMS% = requested ÷ denominator). Only an explicit
        # declaration may drive that denominator. See declared_xy_top_speed_um_s.
        self._declared_xy_top_speed_um_s: float | None = None

        # v7.2: Hardware configuration (set by GUI when hardware setup completes)
        self._hardware_config: HardwareConfig | None = None

        # v7.3.2: Axis direction flip (per-machine, loaded from settings)
        self._axis_flip: dict[str, bool] = {
            "Z": False, "P1": False, "P2": False, "P3": False,
        }

        # v7.5.x: per-pump plunger calibration (mirror of the Z setup). Each
        # pump's DISPENSE/ASPIRATE direction is OWNED by the calibration (the
        # "Set Dispensed / Set Aspirated" capture), exactly like z_up_sign owns
        # Z. ``_pump_aspirate_sign`` = the raw-Marlin-mm direction toward FULL
        # (aspirate); ``_pump_setup`` holds the captured extremes per pump so
        # the fill readout + capacity are independent of the soft-limit store.
        # Empty / sign defaults until a pump is calibrated → the legacy
        # _flip_sign path is used (nothing changes for uncalibrated pumps).
        self._pump_aspirate_sign: dict[str, float] = {
            "P1": 1.0, "P2": 1.0, "P3": 1.0,
        }
        self._pump_setup: dict[str, dict] = {}
        # v7.5.x: per-pump compliance / "pressure relief" value in µL — half the
        # aspirate-back volume measured by the Needle Location compliance
        # calibration (= the drivetrain/syringe flex to take up in one
        # direction). Drives backlash compensation. 0 / absent ⇒ no comp for
        # that pump. Persisted in device_profile.pump_compliance_uL.
        self._pump_relief_uL: dict[str, float] = {}
        # v7.5.x: global enable for backlash compensation (take-up on reversal +
        # unload on stop) at every discrete pump start/stop. Toggled from the
        # pump jog panel. Persisted in device_profile.backlash_comp_enabled.
        self._backlash_comp_enabled: bool = False

        # v7.3.5: Configurable ZP feedrates (mm/min), set from Settings page.
        # v7.5.x: refreshed from the configured Z max so Z moves are FAST —
        # short motor on-time (less heat) and they complete well within the
        # M400 flush timeout. Retract (UP, safe) = the Z max; insert (DOWN to
        # print) = a moderate fraction. See _refresh_zp_move_feedrates().
        self._zp_retract_feedrate: float = ZPStageManager.DEFAULT_FEEDRATE
        self._zp_insert_feedrate: float = ZPStageManager.DEFAULT_FEEDRATE / 2

        # v7.5.x: gentle "slow first mm" of every needle retract. When the
        # needle lifts OUT of a print/deposit, do the first
        # ``_retract_slow_dist_mm`` slowly (``_retract_slow_feedrate`` mm/min) so
        # back-pressure / surface tension can't peel the deposited bead up with
        # the needle at high speed; the rest of the lift is at the fast retract
        # feedrate. Applied by every retract that goes through
        # ``ensure_retracted_to`` / ``safe_travel_to`` (so all print-execution
        # lifts inherit it). Mirrors the per-sketch slow lift in
        # SketchTrajectory. Default 1 mm @ 60 mm/min (1 mm/s); 0 dist disables.
        self._retract_slow_dist_mm: float = 1.0
        self._retract_slow_feedrate: float = 60.0

        # v7.5.x: gentle "slow last mm" of every needle re-entry DESCENT — the
        # descent twin of the retract slow-lift above. When the needle touches
        # back DOWN into a print/work position, cover the FINAL
        # ``_descend_slow_dist_mm`` at ``_descend_slow_feedrate`` (mm/min) so
        # re-entry onto the plate / into a bead is controlled instead of a fast
        # crash-down; the bulk of the descent stays at the fast insert feedrate.
        # Applied by every descent that funnels through ``safe_travel_to`` step 3
        # / the discrete ``MOVE_Z`` handler (so all workflow re-entries inherit
        # it). Default 1 mm @ 60 mm/min (1 mm/s); 0 dist disables (single-speed).
        self._descend_slow_dist_mm: float = 1.0
        self._descend_slow_feedrate: float = 60.0

        # v7.5.x: auto-reconnect the ZP on an UNEXPECTED loss (USB drop /
        # re-enumeration — the CH340 typically comes back on the same COM
        # port). A background reconnect self-heals the link without an app
        # restart. Reopening DTR-resets Marlin (position lost) → on_connect
        # fires the position-restore prompt; a running print has already
        # aborted (is_zp_connected went False). Only triggered from
        # _handle_disconnect (the unexpected path) — never a manual disconnect.
        self.auto_reconnect_zp: bool = True
        self._zp_reconnect_attempts: int = 6
        self._zp_reconnect_delay_s: float = 2.0
        self._zp_reconnecting: bool = False
        self._shutting_down: bool = False
        # v7.4.8: minimum safe travel Z (zero-referenced mm) that clears the
        # tallest plate insert/tube. None = no floor. Set from the active
        # plate's max_rim_height + calibration top Z.
        self._min_travel_z_mm: float | None = None
        # v7.5.x: plate-bottom Z datum (zero-ref mm) + print-time floor.
        # The needle may never go deeper than the plate bottom while a print
        # is running. `_print_floor_active` is armed only during print
        # execution (by PrintManager) so it never blocks calibration / jog.
        self._plate_bottom_z_zref: float | None = None
        # v7.5.x: the plate bottom is a flat but TILTED plane. `_plate_bottom_z_zref`
        # above is the scalar taught at ONE point; `_plate_z_plane` optionally adds
        # the measured gradient about that point so plate-bottom Z is known at any
        # (x, y). The plane is an ANCHORED GRADIENT whose anchor IS the taught
        # scalar, so a zero/absent plane degrades to the scalar exactly (see
        # SupportClasses/PlateZPlane.py). `_plate_bottom_anchor_xy_um` records where
        # the scalar was taught; `_plate_footprint_bbox_um` bounds where the plane
        # may be trusted (extrapolating a tilt far past the taught region is how a
        # tilt fit becomes a crash).
        self._plate_z_plane = None
        self._plate_bottom_anchor_xy_um: tuple[float, float] | None = None
        self._plate_footprint_bbox_um: tuple[float, float, float, float] | None = None
        self._plate_tilt_enabled: bool = False
        self._plate_bottom_z_source: str | None = None
        # v7.5.x: plate-top Z datum (zero-ref mm). With the plate bottom it
        # forms the reference vector that derives the print-Z up-direction
        # (see `print_z_dir`), so print offsets are polarity-correct without a
        # hard-coded ZDIR. None until the plate top is taught in calibration.
        self._plate_top_z_zref: float | None = None
        # v7.10: REFCOUNT, not a bool. There are now several independent
        # arm/disarm sites (PrintManager, SimplePrintManager, PickAndPlaceManager
        # and the needle-calibration wizard's touch-off), and with a plain bool a
        # nested arm/disarm disarms EARLY — the caller still running believes it
        # is protected while the floor is off, which is worse than never arming.
        # Same non-refcounted-global hazard as `PositionPoller._suspended`.
        # The external API (`set_print_floor_active(bool)`) is unchanged.
        self._print_floor_depth: int = 0
        # v7.5.x: per-machine user-facing Z up-direction (+1 or -1). The
        # canonical user frame is  user_Z = z_up_sign · (raw − zero["Z"]),
        # with the datum (zero["Z"]) at the needle-all-the-way-DOWN raw
        # position. Both are established by the one Z setup procedure
        # (`apply_z_setup`), which DERIVES this sign from the captured
        # bottom/top extremes so user_Z always increases as the needle rises.
        # Defaults to the module ZDIR until setup runs (so behaviour is
        # unchanged on an un-set-up machine).
        self._z_up_sign: float = ZDIR
        # v7.5.x: per-machine well-plate orientation (see DEFAULT_PLATE_FLIP_180).
        # Single source of truth for both the display flip and the plate-local→
        # stage geometry sign. Restored from the device profile via
        # ``apply_z_convention``.
        self._plate_flip_180: bool = DEFAULT_PLATE_FLIP_180
        # v7.5.x: needle-tip-camera Z fiducial (USER-frame mm = height above
        # the bottom datum), captured during XY needle calibration, + the
        # standard mechanical offsets (mm BELOW the fiducial) to the plate
        # features. Together they PRE-FILL plate Z reference guesses
        # (plate top / bottom / safe-travel / max). Replace Z stays manual.
        # v7.5.x: these offsets are the GENERIC fallback — a selected plate
        # TYPE pushes its own {top,bottom,safe,max} here (see set_hardware_config).
        self._needle_cam_z_user: float | None = None
        self._plate_z_offsets: dict = {
            "top": 10.0, "bottom": 20.0, "safe": 5.0, "max": 0.0}
        # v7.3.5: Periodic position save to Marlin EEPROM (M500)
        self._zp_auto_save_position: bool = False

        # Register calibration handler
        self.processor.register_handler("zero_needle_pos", self._calibrate_zero)
        # v7.2.6: debug handler — ensures Xbox debug messages always dispatch
        self.processor.register_handler("debug", self._handle_debug)

    # ── Connection Management ─────────────────────────────────────

    def set_controller_json(self, controller_json) -> None:
        """v7.5.x: set which XY controller protocol to use on the NEXT connect.

        ``controller_json`` is a protocol-JSON path (e.g.
        ``config/controllers/mac5000.json``), ``"auto"`` (detect), or ``None``
        (default). The live XY stage is (re)built from this in ``connect_stages``
        (``XYStageManager(controller_json=...)``), so a change takes effect on
        the next XY (re)connect — no restart. Called by the device-profile load
        and the Settings-page controller dropdown so the per-machine profile
        wins, falling back to the global setting, then auto-detect.
        """
        self.controller_json = controller_json
        logger.info("XY controller protocol set to %r (applies on next XY connect)",
                    controller_json)

    def _handle_debug(self, *args, **kwargs) -> None:
        """v7.2.6: debug handler — logs Xbox worker debug messages."""
        msg = kwargs.get("message", "") or (args[0] if args else "")
        logger.info(f"[Debug] {msg}")

    def connect_stages(
        self,
        xy: bool = True,
        zp: bool = True,
        simulate_xy: bool | None = None,
        simulate_zp: bool | None = None,
    ) -> None:
        """Initialise and connect stages.

        Args:
            xy: If True, connect the XY stage (default True).
            zp: If True, connect the ZP stage (default True).
            simulate_xy: If True, the XY stage opens a simulator; if False,
                real hardware. ``None`` falls back to the ``__init__`` default
                (used by headless / CLI callers). Real hardware is now the
                default — the GUI passes an explicit flag from the Connect /
                Simulate buttons on the Hardware Setup → Device sub-page.
            simulate_zp: Same as ``simulate_xy`` but for the ZP stage.
        """
        sim_xy = self._default_simulate_xy if simulate_xy is None else bool(simulate_xy)
        sim_zp = self._default_simulate_zp if simulate_zp is None else bool(simulate_zp)
        # v7.5.x: track which stage actually came online in THIS call so we
        # fire on_connect once, only for a real new connection (the
        # `is None` guards below skip an already-connected stage).
        xy_just_connected = False
        zp_just_connected = False
        if xy and self.xy_stage is None:
            # v7.2.8: connection error handling
            # v7.5.x: never let the XY detection scan OPEN the ZP/Marlin port —
            # opening a port asserts DTR and auto-resets an Arduino/Marlin board.
            # Exclude the live ZP port (if connected) and the last-known-good ZP
            # port (persisted), so connecting a Prior/Ludl XY controller can't
            # reset the ZP board regardless of connect order.
            xy_exclude = [p for p in (
                self.zp_connected_port, self._preferred_zp_port) if p]
            # v7.18: the incubator's dedicated board (when one exists) is a
            # Marlin board too — keep the XY scan off its port as well.
            xy_exclude += self._incubator_reserved_ports()
            try:
                self.xy_stage = XYStageManager(
                    simulate=sim_xy,
                    controller_json=self.controller_json,
                    exclude_ports=xy_exclude,
                    # v7.18.1: last-known-good (protocol, port, baud) so a
                    # reconnect is ONE probe instead of re-walking every
                    # protocol × baud that was ruled out last time.
                    preferred=self._preferred_xy_hint,
                )
            except (ConnectionError, ImportError, OSError) as e:
                logger.error(f"XY stage connection failed: {e}")
                self.xy_stage = None
                if self.on_disconnect:
                    self.on_disconnect("XY")
                return
            # v7.2.6: stop old jog handlers before creating new ones
            if self.xy_jog is not None:
                self.xy_jog.stop()
                self.xy_jog = None
            self.xy_jog = XYJogHandler(
                self.processor, self.xy_stage,
                safety_limits=self.safety_limits,
                get_xy_position=lambda: self._pos_poller.xy_position,
            )
            # v7.5.x: feed the display-only motion estimator the commanded
            # per-segment deltas so the live readout tracks continuous jogging.
            self.xy_jog.on_jog_estimate = self._note_xy_jog_segment
            self.xy_jog.start()
            if not sim_xy:
                self._watchdog.watch(
                    "XY",
                    lambda: getattr(self.xy_stage, "spo", None),
                    lambda: self._handle_disconnect("XY"),
                )
            xy_just_connected = True
            # v7.5.x: on REAL hardware apply the measured XY top speed (if the
            # timing tool calibrated one) so the mm/s↔SMS-% conversion is
            # correct for prints/jog WITHOUT having to open the timing tool.
            # Guarded — never blocks the connect.
            if not sim_xy:
                # v7.21.1: ONE resolver (declared → measured), so the Hardware
                # Setup field and the timing tool cannot disagree about the
                # denominator. Undeclared ⇒ no-op, stage keeps its protocol
                # max_speed (see declared_xy_top_speed_um_s for why).
                self._push_xy_top_speed_to_stage()
                # v7.18.1: adopt what actually answered, so an in-session
                # reconnect (a USB drop, a Connect re-click) is already fast
                # without waiting for the GUI to persist the hint.
                hint = self.xy_connection_hint
                if hint:
                    self._preferred_xy_hint = hint
            logger.info(f"XY stage connected ({'SIM' if sim_xy else 'REAL'})")

        if zp and self.zp_stage is None:
            # v7.2.8: ZP connection error handling
            # v7.4.2 hotfix: pass preferred_port so the rediscovery scan
            # can short-circuit to the last-known-good port.
            # v7.18: keep the ZP scan off a dedicated incubator board's port
            # — opening it would DTR-reset that board and silently drop its
            # heater setpoints mid-hold.
            try:
                self.zp_stage = ZPStageManager(
                    simulate=sim_zp,
                    preferred_port=self._preferred_zp_port,
                    exclude_ports=self._incubator_reserved_ports(),
                )
            except (ConnectionError, ImportError, OSError) as e:
                logger.error(f"ZP stage connection failed: {e}")
                self.zp_stage = None
                if self.on_disconnect:
                    self.on_disconnect("ZP")
                return
            # v7.5.x ZP reconnect hotfix: ZPStageManager constructs
            # successfully even when no Marlin board is acquired
            # (``_initialise_serial`` returns None instead of raising,
            # unlike XYStageManager). Treat that as a failed connection so
            # the badge never shows a false "Connected", the dead object is
            # released (so a later Connect can retry — the guard above is
            # ``self.zp_stage is None``), and the GUI is notified.
            if not sim_zp and getattr(self.zp_stage, "serial", None) is None:
                logger.error(
                    "ZP connection failed: no Marlin board acquired "
                    "(check USB / power / that no other program holds the port)")
                try:
                    self.zp_stage.stop()
                except Exception:
                    pass
                self.zp_stage = None
                if self.on_disconnect:
                    self.on_disconnect("ZP")
                return
            # v7.2.6: stop old ZP jog handler
            if self.zp_jog is not None:
                self.zp_jog.stop()
                self.zp_jog = None
            self.zp_jog = ZPJogHandler(
                self.processor, self.zp_stage,
                safety_limits=self.safety_limits,
                get_zp_position=lambda: self._pos_poller.zp_position,
                zero_position=self.zero_position,
                # v7.5.x: honor the same per-axis direction flips as the GUI
                # jog buttons. Bound method reads the live _axis_flip dict, so
                # later set_axis_flip(s) calls take effect without reconnect.
                flip_provider=self._flip_sign,
                # v7.5.x: Z jog direction follows the live per-machine
                # z_up_sign (the Set Bottom/Top setup), so the Xbox stick agrees
                # with the GUI buttons + the readout. Bound method = live value.
                z_up_sign_provider=self.z_up_sign,
                # v7.5.x: pump jog direction follows the live per-pump plunger
                # calibration (pump_dir_sign), so the Xbox pump jog agrees with
                # move_pump_relative once a pump is calibrated.
                pump_dir_sign_provider=self.pump_dir_sign,
            )
            # v7.5.x: feed the display-only motion estimator the commanded
            # per-segment logical deltas so the live readout tracks the jog.
            self.zp_jog.on_jog_estimate = self._note_zp_jog_segment
            self.zp_jog.start()
            if not sim_zp:
                self._watchdog.watch(
                    "ZP",
                    lambda: getattr(self.zp_stage, "serial", None),
                    lambda: self._handle_disconnect("ZP"),
                )
                # v7.3.5: Register periodic position save (M500)
                self._watchdog.add_periodic(self._periodic_zp_position_save)
            zp_just_connected = True
            # v7.5.x: sticky session flag — once a real ZP board has been live
            # this session, a needle exists and a later is_zp_connected==False
            # means it DROPPED (not "XY-only rig"). Drives the safe_travel_to
            # refuse-XY-when-unretractable guard. Never reset on disconnect.
            if not sim_zp:
                self._zp_ever_connected = True
            logger.info(f"ZP stage connected ({'SIM' if sim_zp else 'REAL'})")

            # v7.4.2: Apply any pending device settings (axis_map, steps_per_mm)
            # that were set before the ZP stage was online.
            if self._pending_axis_map is not None:
                self.zp_stage.set_axis_map(self._pending_axis_map)
            if self._pending_steps_per_mm is not None:
                self.zp_stage.set_steps_per_mm(
                    self._pending_steps_per_mm, persist=True)
            # v7.4.2 hotfix: push per-axis M203 ceilings so Marlin agrees
            # with the software's per_axis_max_feedrate from connect-time.
            if self._pending_per_axis_max_feedrate is not None:
                self.zp_stage.set_per_axis_max_feedrate(
                    self._pending_per_axis_max_feedrate, persist=True)

        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)

        # v7.5.x: anchor the Xbox jog speed-% ladder to the calibrated per-axis
        # max and re-apply the restored % to the freshly-created jog handlers.
        try:
            self.refresh_jog_speed_limits()
        except Exception as e:
            logger.debug(f"refresh_jog_speed_limits (connect) failed: {e}")

        # v7.5.x: notify the GUI of new connections (after set_stages so the
        # poller already has the stage and the callback can read position).
        if self.on_connect:
            if xy_just_connected:
                try:
                    self.on_connect("XY")
                except Exception as e:
                    logger.debug(f"on_connect(XY) callback error: {e}")
            if zp_just_connected:
                try:
                    self.on_connect("ZP")
                except Exception as e:
                    logger.debug(f"on_connect(ZP) callback error: {e}")

    def _periodic_zp_position_save(self) -> None:
        """v7.3.5: Called by watchdog — saves ZP position to EEPROM (M500).

        Only sends M500 when auto-save is enabled and the ZP stage is
        connected on real hardware. On crash/reset, Marlin restores the
        last saved position so the user doesn't lose their reference.
        """
        if not self._zp_auto_save_position:
            return
        if self.zp_stage is None or self.simulate_zp:
            return
        if self._pos_poller._suspended:
            return  # Don't interfere with safe_travel_to
        try:
            self.zp_stage.save_settings()  # M500
            logger.debug("Periodic ZP position save (M500)")
        except Exception as e:
            logger.debug(f"ZP position save failed: {e}")

    # ── Convenience connection methods (used by Dashboard) ────────

    def set_preferred_zp_port(self, port: str | None) -> None:
        """v7.4.2 hotfix: cache the last-known-good ZP serial port so
        the next connect_stages() can try it first instead of
        scanning every tty."""
        self._preferred_zp_port = port

    def set_preferred_xy_hint(self, hint: dict | None) -> None:
        """v7.18.1: cache the last-known-good XY (protocol, port, baud) so the
        next connect_stages() probes it directly instead of re-walking every
        protocol JSON × baud × port.

        A stale hint is harmless — the probe misses and the full scan runs.
        """
        self._preferred_xy_hint = dict(hint) if isinstance(hint, dict) else None

    @property
    def xy_connection_hint(self) -> dict | None:
        """v7.18.1: {protocol, port, baud} the live XY stage actually opened.

        None when simulating or not connected — never persist a hint that
        would point the next launch at something that never answered.
        """
        if self.xy_stage and not self.simulate_xy:
            return getattr(self.xy_stage, "detection_hint", None)
        return None

    @property
    def zp_connected_port(self) -> str | None:
        """v7.4.2 hotfix: serial device the live ZP stage opened.
        None if not connected or running in simulation."""
        if self.zp_stage and not self.simulate_zp:
            return getattr(self.zp_stage, "connected_port", None)
        return None

    def _incubator_reserved_ports(self) -> list[str]:
        """v7.18: ports a DEDICATED incubator board owns, for scan exclusion.

        Two sources, both lazy and guarded so a missing/broken incubator
        module can never block a stage connect:
          * the live incubator connection, when it holds its own serial port
            (``transport == "serial"`` — the shared-ZP transport owns no
            port, and the simulator has none);
          * the saved dedicated-port hint in the incubator config store,
            honoured ONLY while the configured transport is "serial" — a
            stale hint left behind after switching back to the shared
            transport must not blind the ZP scan to a real port.

        A port equal to the ZP's own last-known-good is never excluded: if
        the operator mistypes the ZP port into the incubator config, the ZP
        board must still win its own port (the incubator connect then fails
        with its own actionable message, which is the recoverable direction).
        """
        out: list[str] = []
        try:
            from SupportClasses.incubator.service import peek_incubator
            inc = peek_incubator()
            if (inc is not None and getattr(inc, "connected", False)
                    and getattr(inc, "transport", "") == "serial"):
                p = getattr(inc, "active_port", "")
                if p:
                    out.append(p)
        except Exception as e:
            logger.debug(f"incubator live-port lookup skipped: {e}")
        try:
            from SupportClasses.incubator.config_store import get_store
            store = get_store()
            if store.get("transport", "shared") == "serial":
                p = store.get("dedicated_port", "")
                if p:
                    out.append(p)
        except Exception as e:
            logger.debug(f"incubator config-port lookup skipped: {e}")
        preferred = (self._preferred_zp_port or "").upper()
        return [p for p in dict.fromkeys(out)
                if p and p.upper() != preferred]

    def set_min_travel_z(self, z_mm: float | None) -> None:
        """v7.4.8: set the plate-wide travel-Z clearance floor (zero-ref mm).

        When set, every `safe_travel_to` retract is raised to at least this
        height so the needle clears the tallest insert/tube. Pass None to
        clear the floor (plate has no tall inserts).
        """
        self._min_travel_z_mm = z_mm
        if z_mm is not None:
            logger.info(f"StageController: min travel Z floor = {z_mm:.2f} mm "
                        f"(clears plate inserts)")

    # ── v7.5.x: unified user-facing Z frame (0 at the bottom datum, + up) ──
    #
    # ONE canonical convention for everything the operator sees or types:
    #
    #     user_Z = z_up_sign · (raw − zero_position["Z"])
    #     raw    = zero_position["Z"] + user_Z / z_up_sign
    #
    # The datum ``zero_position["Z"]`` is the needle-all-the-way-DOWN raw
    # position captured during the Z setup; ``z_up_sign`` is DERIVED from that
    # setup so user_Z increases as the needle rises regardless of motor wiring.
    # Internals (motion, the raw soft-limit envelope) stay in the raw frame —
    # these convert only at the operator boundary. Use them for EVERY Z readout,
    # spinbox, limit display, and calibration capture so the whole UI agrees.

    def z_up_sign(self) -> float:
        """Per-machine user-Z up-direction (+1 or -1). Derived by the Z setup
        (`apply_z_setup`); defaults to the module ``ZDIR`` until setup runs."""
        return getattr(self, "_z_up_sign", ZDIR)

    def set_z_up_sign(self, sign: float) -> None:
        """Set the user-Z up-direction (+1 or -1). Normally derived by the Z
        setup; exposed for persistence/restore."""
        s = 1.0 if float(sign) >= 0 else -1.0
        self._z_up_sign = s
        logger.info(f"StageController: user-Z up-direction sign = {s:+.0f}")

    # ── v7.5.x: well-plate orientation (single source of truth) ─────────
    #
    # ONE per-machine setting (``plate_flip_180``) enforces the canonical
    # convention "well A1 top-left; stage physical 0,0 bottom-right" everywhere.
    # It drives BOTH the 180° display flip (stage-frame plate views) and the
    # plate-local→stage geometry sign (``plate_axis_sign``). See
    # DEFAULT_PLATE_FLIP_180. The ``getattr`` fallbacks keep ``__new__`` test
    # stubs working (mirrors ``z_up_sign``).

    def plate_flip_180(self) -> bool:
        """Whether plate displays / geometry are rotated 180° relative to the
        stage axes (True on ME3B V1: origin bottom-right, A1 top-left)."""
        return bool(getattr(self, "_plate_flip_180", DEFAULT_PLATE_FLIP_180))

    def set_plate_flip_180(self, flip: bool) -> None:
        """Set the per-machine plate orientation. Exposed for persistence /
        restore from the device profile."""
        self._plate_flip_180 = bool(flip)
        logger.info(f"StageController: plate_flip_180 = {self._plate_flip_180}")

    def plate_axis_sign(self) -> tuple[float, float]:
        """Sign mapping plate-local axes (+col→+X, +row→+Y) onto stage axes.

        ``(-1.0, -1.0)`` when the plate is mounted 180° to the stage (ME3B V1),
        else ``(+1.0, +1.0)``. Multiply an A1-relative offset by this before
        adding it to a taught A1 / plate-centre stage coordinate so geometric
        well prediction lands on the physically-correct side of A1.
        """
        return (-1.0, -1.0) if self.plate_flip_180() else (1.0, 1.0)

    def raw_to_user_z(self, raw_mm: float) -> float:
        """Raw Marlin Z (mm) → user-facing Z (mm; 0 at the bottom datum, + up)."""
        return self.z_up_sign() * (float(raw_mm) - self.zero_position.get("Z", 0.0))

    def user_z_to_raw(self, user_z_mm: float) -> float:
        """User-facing Z (mm) → raw Marlin Z (mm)."""
        return (self.zero_position.get("Z", 0.0)
                + float(user_z_mm) / self.z_up_sign())

    def zref_to_user_z(self, zref_mm: float) -> float:
        """Zero-ref Z (mm) → user-facing Z (mm) — i.e. ``z_up_sign · zref``.

        Equals :meth:`z_height_of`; the working/calibration/print code stores
        Z in the zero-ref frame, so this is the converter for those values.
        """
        return self.z_up_sign() * float(zref_mm)

    def user_z_to_zref(self, user_z_mm: float) -> float:
        """User-facing Z (mm) → zero-ref Z (mm)."""
        return float(user_z_mm) / self.z_up_sign()

    def capture_current_z_raw(self) -> float | None:
        """Read the current RAW Marlin Z (mm) for the Z setup captures, or None
        when the ZP stage can't be read."""
        zp = self.get_zp_position(cached=False)
        return self.zp_logical_value(zp, "Z") if zp else None

    def apply_z_setup(self, raw_bottom_mm: float, raw_top_mm: float,
                      min_travel_mm: float = 1.0) -> dict:
        """Establish the whole Z convention from two captured raw extremes.

        This is the single source of truth for the Z datum, direction, and
        soft limits (decisions D1–D4):

        * ``raw_bottom_mm`` — needle all the way DOWN (mechanical hard-bottom).
          Becomes user **Z = 0** (``zero_position["Z"] = raw_bottom``).
        * ``raw_top_mm`` — needle all the way UP.

        Derived (no hard-coded polarity):

        * ``z_up_sign = sign(raw_top − raw_bottom)`` so user_Z increases toward
          the top — this *verifies/establishes the direction*.
        * soft-limit envelope (stored ABSOLUTE raw, Set-Zero-independent):
          ``z_min = min(raw_bottom, raw_top)``, ``z_max = max(...)``.

        Returns a summary dict including ``direction_ok`` (False when the two
        extremes are closer than ``min_travel_mm`` — the caller should warn and
        the sign is left unchanged).
        """
        rb = float(raw_bottom_mm)
        rt = float(raw_top_mm)
        d = rt - rb
        ok = abs(d) >= float(min_travel_mm)
        # Datum: the bottom is user Z = 0.
        self.zero_position["Z"] = rb
        # Direction: only trust it when the extremes are meaningfully apart.
        if ok:
            self.set_z_up_sign(1.0 if d > 0 else -1.0)
        # Soft-limit envelope in the absolute raw frame.
        self.safety_limits.z_min = min(rb, rt)
        self.safety_limits.z_max = max(rb, rt)
        summary = {
            "raw_bottom": rb,
            "raw_top": rt,
            "z_up_sign": self._z_up_sign,
            "travel_height_mm": abs(d),
            "z_min": self.safety_limits.z_min,
            "z_max": self.safety_limits.z_max,
            "direction_ok": ok,
        }
        logger.info(
            "Z setup applied: bottom(raw)=%.3f→userZ=0, top(raw)=%.3f, "
            "up_sign=%+.0f, travel=%.3f mm, z_envelope_raw=[%.3f, %.3f], "
            "direction_ok=%s",
            rb, rt, self._z_up_sign, abs(d),
            self.safety_limits.z_min, self.safety_limits.z_max, ok)
        return summary

    # ── v7.5.x: per-pump plunger setup (mirror of apply_z_setup) ──────────
    #
    # ONE capture flow per pump establishes the whole fill convention:
    #
    #   * "Set Dispensed" — plunger ALL THE WAY IN (syringe empty). This raw
    #     position becomes the datum (fill = 0); ``zero_position[pump] = raw``.
    #   * "Set Aspirated" — plunger ALL THE WAY OUT (syringe full).
    #
    # Derived (no hard-coded polarity): the aspirate direction sign and the
    # soft-limit envelope. ASPIRATE = draw fluid IN (toward full, raises fill);
    # DISPENSE = push fluid OUT (toward empty, lowers fill). The derived
    # direction is OWNED by this setup — once calibrated it drives motion
    # (see ``pump_dir_sign`` / ``move_pump_relative``), retiring the per-pump
    # _flip_sign hack, exactly as z_up_sign superseded the Z axis-flip.

    def pump_aspirate_sign(self, pump: str) -> float:
        """Per-pump raw-Marlin-mm direction toward FULL (+1 / -1), DERIVED by
        :meth:`apply_pump_setup`. Defaults to +1 until the pump is calibrated."""
        return getattr(self, "_pump_aspirate_sign", {}).get(pump, 1.0)

    def set_pump_aspirate_sign(self, pump: str, sign: float) -> None:
        """Set/restore a pump's aspirate-direction sign (+1 / -1). Normally
        derived by the plunger setup; exposed for persistence/restore."""
        if not hasattr(self, "_pump_aspirate_sign"):
            self._pump_aspirate_sign = {"P1": 1.0, "P2": 1.0, "P3": 1.0}
        self._pump_aspirate_sign[pump] = 1.0 if float(sign) >= 0 else -1.0
        logger.info(f"StageController: {pump} aspirate-direction sign = "
                    f"{self._pump_aspirate_sign[pump]:+.0f}")

    def is_pump_plunger_calibrated(self, pump: str) -> bool:
        """True once :meth:`apply_pump_setup` has captured both extremes for
        ``pump`` — the point at which the calibration OWNS its direction."""
        s = getattr(self, "_pump_setup", {}).get(pump)
        return bool(s and "raw_dispensed" in s and "raw_aspirated" in s)

    def pump_dir_sign(self, pump: str) -> float:
        """Raw-mm direction for one mm of *dispense intent* (the sign applied in
        :meth:`move_pump_relative`). Calibration-owned when the pump is
        calibrated: dispense moves toward empty = ``−aspirate_sign``. Falls back
        to the legacy per-axis ``_flip_sign`` for uncalibrated pumps so nothing
        changes until a pump is set up."""
        if self.is_pump_plunger_calibrated(pump):
            return -self.pump_aspirate_sign(pump)
        return self._flip_sign(pump)

    def capture_current_pump_raw(self, pump: str) -> float | None:
        """Read the current RAW Marlin position (mm) for a pump's plunger setup
        captures, routed through the live axis_map. None when unreadable."""
        zp = self.get_zp_position(cached=False)
        return self.zp_logical_value(zp, pump) if zp else None

    def _pump_widen_span_mm(self, pump: str) -> float:
        """A generous symmetric soft-limit span (mm) for the plunger-setup jog,
        derived from the configured syringe stroke (×1.2) or a 100 mm default."""
        hw = getattr(self, "_hardware_config", None)
        cfg = hw.pumps.get(pump) if hw else None
        syr = getattr(cfg, "syringe", None) if cfg else None
        stroke = getattr(syr, "stroke_length_mm", None) if syr else None
        if stroke and float(stroke) > 0:
            return float(stroke) * 1.2
        return 100.0

    def begin_pump_plunger_setup(self, pump: str) -> dict:
        """First half of plunger calibration — the "Set Dispensed" (set-zero)
        step. ZERO the pump's Marlin counter at the current (plunger all-the-way
        IN / syringe empty) position via ``G92`` so the dispensed datum is
        exactly raw **0.0**. The operator then jogs the plunger OUT and captures
        the aspirated (full) extreme with :meth:`apply_pump_setup`; the derived
        ``aspirate_sign`` then makes the displayed plunger FILL read 0 at empty
        and grow POSITIVE toward full — i.e. "fully extended is positive, empty
        is 0.0", regardless of which raw direction the motor counts.

        Also temporarily WIDENS the pump soft-limit envelope to the full syringe
        stroke (both directions) so the jog out to the full extreme is not
        clamped by the now-stale absolute-raw limits; :meth:`apply_pump_setup`
        tightens the envelope to the captured extremes at the end. No motion
        occurs (``G92`` only rebases the firmware counter — it does not move).

        Returns ``{"pump", "ok", "previous_raw"}`` (``ok`` False when the
        firmware counter could not be zeroed — e.g. ZP not connected)."""
        if pump not in ("P1", "P2", "P3"):
            raise ValueError(f"Invalid pump ID: {pump}")
        try:
            prev_raw = self.capture_current_pump_raw(pump)
        except Exception:
            prev_raw = None
        # Rebase the firmware counter to 0 at the current (empty) position so the
        # dispensed datum is genuinely raw 0.0 (the "set zero" the operator asked
        # to happen automatically as part of this step). G92 does not move.
        zp = getattr(self, "zp_stage", None)
        ok = False
        if zp is not None and hasattr(zp, "set_zero"):
            try:
                ok = bool(zp.set_zero(pump))
            except Exception as e:
                logger.warning(
                    f"begin_pump_plunger_setup({pump}): G92 zero failed: {e}")
                ok = False
        # Datum: plunger all-the-way-in (empty) is now raw 0 → fill 0.
        self.zero_position[pump] = 0.0
        # The absolute-raw soft limits no longer match the re-zeroed frame; widen
        # symmetrically so the jog to the full extreme isn't clamped. The final
        # tight envelope is set by apply_pump_setup from the captured extremes.
        span = self._pump_widen_span_mm(pump)
        try:
            setattr(self.safety_limits, f"{pump.lower()}_min", -span)
            setattr(self.safety_limits, f"{pump.lower()}_max", span)
        except Exception:
            pass
        logger.info(
            "%s plunger setup started: zeroed at empty (G92 ok=%s, prev raw=%s)"
            " → datum raw 0.0; soft limits widened to [%.1f, %.1f] mm for the "
            "jog to the full extreme.", pump, ok,
            f"{prev_raw:.3f}" if isinstance(prev_raw, (int, float)) else "n/a",
            -span, span)
        return {"pump": pump, "ok": ok, "previous_raw": prev_raw}

    def apply_pump_setup(self, pump: str, raw_dispensed_mm: float,
                         raw_aspirated_mm: float,
                         min_travel_mm: float = 1.0) -> dict:
        """Establish a pump's whole fill/direction convention from two captured
        raw extremes — the pump twin of :meth:`apply_z_setup`.

        * ``raw_dispensed_mm`` — plunger ALL THE WAY IN (syringe empty). Becomes
          the datum (fill = 0); ``zero_position[pump] = raw_dispensed``.
        * ``raw_aspirated_mm`` — plunger ALL THE WAY OUT (syringe full).

        Derived:

        * ``aspirate_sign = sign(raw_aspirated − raw_dispensed)`` (the raw
          direction toward FULL) — this *establishes the plunger direction* and
          becomes authoritative for motion (see :meth:`pump_dir_sign`).
        * soft-limit envelope (ABSOLUTE raw, exact captured extremes — no margin,
          so the plunger is never commanded past its mechanical hard stops):
          ``p_min = min(...)``, ``p_max = max(...)``.

        Returns a summary dict including ``direction_ok`` (False when the two
        extremes are closer than ``min_travel_mm`` — the sign is left unchanged,
        but datum + limits are still set).
        """
        if pump not in ("P1", "P2", "P3"):
            raise ValueError(f"Invalid pump ID: {pump}")
        rd = float(raw_dispensed_mm)
        ra = float(raw_aspirated_mm)
        d = ra - rd
        ok = abs(d) >= float(min_travel_mm)
        # Datum: plunger all-the-way-in (empty) is fill = 0.
        self.zero_position[pump] = rd
        # Direction: only trust it when the extremes are meaningfully apart.
        if ok:
            self.set_pump_aspirate_sign(pump, 1.0 if d > 0 else -1.0)
        # Soft-limit envelope in the absolute raw frame (exact extremes).
        lo, hi = min(rd, ra), max(rd, ra)
        setattr(self.safety_limits, f"{pump.lower()}_min", lo)
        setattr(self.safety_limits, f"{pump.lower()}_max", hi)
        # Capacity (µL) from the calibrated stroke, when a syringe is configured.
        capacity_uL = self._pump_mm_to_uL(pump, abs(d))
        # Remember the extremes so the fill readout + capacity are independent
        # of the soft-limit store (which a hw-config re-apply could overwrite).
        if not hasattr(self, "_pump_setup"):
            self._pump_setup = {}
        self._pump_setup[pump] = {
            "raw_dispensed": rd,
            "raw_aspirated": ra,
            "aspirate_sign": self.pump_aspirate_sign(pump),
            "capacity_uL": capacity_uL,
        }
        summary = {
            "pump": pump,
            "raw_dispensed": rd,
            "raw_aspirated": ra,
            "aspirate_sign": self.pump_aspirate_sign(pump),
            "capacity_mm": abs(d),
            "capacity_uL": capacity_uL,
            "p_min": lo,
            "p_max": hi,
            "direction_ok": ok,
        }
        logger.info(
            "%s plunger setup: dispensed(raw)=%.3f→fill0, aspirated(raw)=%.3f, "
            "aspirate_sign=%+.0f, stroke=%.3f mm (%s µL), envelope_raw=[%.3f, "
            "%.3f], direction_ok=%s",
            pump, rd, ra, self.pump_aspirate_sign(pump), abs(d),
            f"{capacity_uL:.1f}" if capacity_uL is not None else "n/a",
            lo, hi, ok)
        return summary

    def apply_pump_convention(self, pump_setup: dict | None = None) -> None:
        """Restore the persisted per-pump plunger convention at startup (mirror
        of :meth:`apply_z_convention`). ``pump_setup`` maps pump → dict with
        ``raw_dispensed`` / ``raw_aspirated`` / ``aspirate_sign``. Re-establishes
        each calibrated pump's datum, direction sign, AND soft-limit envelope —
        all three RE-DERIVED from the one authoritative source (the captured
        extremes), exactly as :meth:`apply_pump_setup` did when they were first
        set. Does NOT move.

        v7.5.x: the envelope is now re-derived here rather than relying on the
        separately-persisted ``safety_limits.p*`` mirror. That mirror could go
        stale / get clobbered (e.g. an old calibration's sign, a device-profile
        apply) while ``pump_setup`` stayed correct — the calibration owns the
        datum + direction on reboot, so it must own the envelope it captured
        too. Symptom this fixes: "location correct but min/max wrong after
        restart" — the datum restored from ``raw_dispensed`` but the envelope
        kept the stale mirror, which :meth:`set_hardware_config`'s
        ``skip_pumps`` guard then PRESERVED instead of recomputing."""
        if not pump_setup:
            return
        if not hasattr(self, "_pump_setup"):
            self._pump_setup = {}
        for pump, s in pump_setup.items():
            if pump not in ("P1", "P2", "P3") or not isinstance(s, dict):
                continue
            rd = s.get("raw_dispensed")
            ra = s.get("raw_aspirated")
            if rd is None or ra is None:
                continue
            rd, ra = float(rd), float(ra)
            self.zero_position[pump] = rd
            d = ra - rd
            # Prefer the stored sign; re-derive from the extremes if absent.
            sign = s.get("aspirate_sign")
            if sign is None and abs(d) > 0:
                sign = 1.0 if d > 0 else -1.0
            if sign is not None:
                self.set_pump_aspirate_sign(pump, sign)
            # Re-derive the soft-limit envelope from the captured extremes
            # (exact extremes, no margin — never command past the mechanical
            # hard stops), matching :meth:`apply_pump_setup`. This keeps the
            # restored min/max self-consistent with the datum + direction
            # instead of trusting the (possibly stale) safety_limits mirror.
            if getattr(self, "safety_limits", None) is not None:
                lo, hi = min(rd, ra), max(rd, ra)
                setattr(self.safety_limits, f"{pump.lower()}_min", lo)
                setattr(self.safety_limits, f"{pump.lower()}_max", hi)
            self._pump_setup[pump] = {
                "raw_dispensed": rd,
                "raw_aspirated": ra,
                "aspirate_sign": self.pump_aspirate_sign(pump),
                "capacity_uL": s.get("capacity_uL",
                                     self._pump_mm_to_uL(pump, abs(d))),
            }
        logger.info(f"Pump plunger convention restored: "
                    f"{sorted(self._pump_setup.keys())}")

    def get_pump_setup(self) -> dict:
        """Per-pump captured extremes + derived sign (for persistence)."""
        return {k: dict(v) for k, v in getattr(self, "_pump_setup", {}).items()}

    def apply_pump_relief(self, pump_compliance_uL: dict | None = None) -> None:
        """Restore the per-pump compliance / "pressure relief" values (µL) at
        startup (twin of :meth:`apply_pump_convention`). ``pump_compliance_uL``
        maps pump → µL (= ½ the aspirate-back volume from the Needle Location
        compliance calibration). Drives backlash compensation. Does NOT move."""
        if not hasattr(self, "_pump_relief_uL"):
            self._pump_relief_uL = {}
        if not pump_compliance_uL:
            return
        for pump, v in pump_compliance_uL.items():
            if pump not in ("P1", "P2", "P3"):
                continue
            try:
                self._pump_relief_uL[pump] = max(0.0, float(v))
            except (TypeError, ValueError):
                continue
        logger.info(
            "Pump compliance/relief restored: "
            f"{ {k: round(v, 4) for k, v in self._pump_relief_uL.items()} }")

    # ── v7.5.x: plunger FILL readout (0 = empty/dispensed → capacity = full) ──

    def _pump_mm_to_uL(self, pump: str, mm: float) -> float | None:
        """Convert a plunger travel (mm) to µL via the configured syringe spec,
        or None when no syringe is configured for ``pump``."""
        hw = getattr(self, "_hardware_config", None)
        cfg = hw.pumps.get(pump) if hw else None
        if not cfg or not cfg.is_configured:
            return None
        try:
            return cfg.mm_to_uL(float(mm))
        except (ValueError, AttributeError):
            return None

    def pump_capacity_uL(self, pump: str) -> float | None:
        """Calibrated usable plunger capacity in µL (full − empty), or None when
        the pump isn't calibrated or has no syringe configured."""
        s = getattr(self, "_pump_setup", {}).get(pump)
        if not s or "raw_dispensed" not in s or "raw_aspirated" not in s:
            return None
        stroke_mm = abs(float(s["raw_aspirated"]) - float(s["raw_dispensed"]))
        uL = self._pump_mm_to_uL(pump, stroke_mm)
        return abs(uL) if uL is not None else None

    def raw_to_pump_fill_uL(self, pump: str, raw_mm: float) -> float | None:
        """Raw Marlin position (mm) → plunger FILL in µL: 0 at the dispensed
        datum (empty), growing toward the aspirated extreme (full). None when
        the pump isn't calibrated or has no syringe. Uses the calibration-owned
        ``aspirate_sign`` so fill grows the right way on either polarity."""
        if not self.is_pump_plunger_calibrated(pump):
            return None
        fill_mm = self.pump_aspirate_sign(pump) * (
            float(raw_mm) - self.zero_position.get(pump, 0.0))
        return self._pump_mm_to_uL(pump, fill_mm)

    def pump_fill_uL(self, pump: str) -> float | None:
        """Current plunger fill in µL (0 = empty/dispensed → capacity =
        full/aspirated), or None when uncalibrated / unreadable."""
        idx_pos = self.get_zp_position(cached=True)
        raw = self.zp_logical_value(idx_pos, pump) if idx_pos else None
        if raw is None:
            return None
        return self.raw_to_pump_fill_uL(pump, raw)

    # ── v7.5.x: µL ↔ % of syringe volume (for the %/µL pump UI off Hardware
    # Setup). Capacity prefers the CALIBRATED plunger stroke, falling back to
    # the syringe's nominal volume so % still works pre-calibration. ──

    def pump_effective_capacity_uL(self, pump: str) -> float | None:
        """Usable pump capacity in µL: the calibrated plunger stroke
        (:meth:`pump_capacity_uL`) if available, else the configured syringe's
        nominal ``volume_uL``, else None (no basis for a %)."""
        cap = self.pump_capacity_uL(pump)
        if cap and cap > 0:
            return float(cap)
        hw = getattr(self, "_hardware_config", None)
        cfg = hw.pumps.get(pump) if hw else None
        syr = getattr(cfg, "syringe", None) if cfg else None
        vol = getattr(syr, "volume_uL", None) if syr else None
        try:
            return float(vol) if vol and float(vol) > 0 else None
        except (TypeError, ValueError):
            return None

    def pump_pct_to_uL(self, pump: str, pct: float) -> float | None:
        """Convert a % of syringe volume to µL (sign preserved). None when no
        capacity is resolvable."""
        cap = self.pump_effective_capacity_uL(pump)
        if cap is None:
            return None
        try:
            return (float(pct) / 100.0) * cap
        except (TypeError, ValueError):
            return None

    def pump_uL_to_pct(self, pump: str, uL: float) -> float | None:
        """Convert µL to a % of syringe volume (sign preserved). None when no
        capacity is resolvable."""
        cap = self.pump_effective_capacity_uL(pump)
        if cap is None or cap <= 0:
            return None
        try:
            return (float(uL) / cap) * 100.0
        except (TypeError, ValueError):
            return None

    def pump_fill_pct(self, pump: str) -> float | None:
        """Current fill as a % of capacity (0 = empty → 100 = full), or None
        when uncalibrated / unreadable / no capacity."""
        fill = self.pump_fill_uL(pump)
        if fill is None:
            return None
        return self.pump_uL_to_pct(pump, fill)

    # ── v7.5.x: needle-tip-camera Z fiducial → plate-reference guesses ──
    #
    # The needle-tip calibration cameras sit at a fixed height. When the tip is
    # centered in them (XY needle cal), the needle is at a repeatable Z. We
    # capture that Z (the "needle-cam fiducial") and apply STANDARD mechanical
    # offsets (mm below the fiducial) to PRE-FILL guesses for the plate Z
    # references (plate top / bottom / safe-travel). Max/Replace Z stay manual.

    def set_needle_cam_z_from_raw(self, raw_mm: float) -> float:
        """Record the needle-tip-camera Z fiducial from a live RAW Z reading
        (captured at the needle edge clicks). Stored in the USER frame (height
        above the bottom datum) so it stays meaningful across power cycles once
        the datum is re-established. Returns the stored user-Z."""
        self._needle_cam_z_user = self.raw_to_user_z(float(raw_mm))
        logger.info("Needle-cam Z fiducial = %.3f mm (user) from raw %.3f",
                    self._needle_cam_z_user, float(raw_mm))
        return self._needle_cam_z_user

    def set_needle_cam_z_user(self, user_z_mm: float | None) -> None:
        """Set/restore the needle-cam Z fiducial directly in the USER frame
        (mm above the bottom datum), or None to clear."""
        self._needle_cam_z_user = (None if user_z_mm is None
                                   else float(user_z_mm))

    def get_needle_cam_z_user(self) -> float | None:
        """Needle-cam Z fiducial (USER-frame mm), or None if not captured."""
        return self._needle_cam_z_user

    def set_plate_z_offsets(self, top: float | None = None,
                            bottom: float | None = None,
                            safe: float | None = None,
                            max: float | None = None) -> None:
        """Set the standard mm-BELOW-the-fiducial offsets to the plate features.
        Only the provided keys are updated. ``max`` = the soft-limit ceiling /
        Max Z guess (v7.5.x)."""
        if top is not None:
            self._plate_z_offsets["top"] = float(top)
        if bottom is not None:
            self._plate_z_offsets["bottom"] = float(bottom)
        if safe is not None:
            self._plate_z_offsets["safe"] = float(safe)
        if max is not None:
            self._plate_z_offsets["max"] = float(max)

    def get_plate_z_offsets(self) -> dict:
        """The standard plate offsets (mm below the needle-cam fiducial)."""
        return dict(self._plate_z_offsets)

    def estimate_plate_z_refs(self) -> dict | None:
        """Guess the plate Z references from the needle-cam fiducial + the
        standard offsets. Returns ``{"plate_top_z", "plate_bottom_z",
        "safe_z", "plate_max_z"}`` in ZERO-REF mm (the frame the calibration
        references and ``set_plate_*_z`` use), or None if the fiducial isn't
        captured.

        The plate features sit BELOW the fiducial (the needle descends from the
        camera height to the plate), so each guess is ``fiducial − offset`` in
        the user/height frame, then converted to zero-ref.
        """
        cam = self._needle_cam_z_user
        if cam is None:
            return None
        off = self._plate_z_offsets
        top_user = cam - float(off.get("top", 0.0))
        bottom_user = cam - float(off.get("bottom", 0.0))
        safe_user = cam - float(off.get("safe", 0.0))
        max_user = cam - float(off.get("max", 0.0))
        return {
            "plate_top_z": self.user_z_to_zref(top_user),
            "plate_bottom_z": self.user_z_to_zref(bottom_user),
            "safe_z": self.user_z_to_zref(safe_user),
            "plate_max_z": self.user_z_to_zref(max_user),
        }

    def plate_z_refs_from_top(self, top_zref: float) -> dict | None:
        """Derive the other plate Z references from a TAUGHT Plate Top Z.

        v7.17.1. ``estimate_plate_z_refs`` anchors every guess on the
        needle-cam fiducial, which is a property of the MACHINE. Once the
        operator has actually touched off the plate's top surface, the taught
        top is a far better anchor for this particular plate and seating: it
        absorbs plate-to-plate thickness variation and any drift in the
        fiducial, while the plate type still supplies the one thing it really
        knows — the SPACING between its own features.

        The stored offsets are mm BELOW the fiducial, so the fiducial cancels:

            user_z(k) = user_z(top) − (offset[k] − offset["top"])

        Returns ``{"plate_bottom_z", "safe_z", "plate_max_z"}`` in zero-ref mm
        — deliberately NOT ``plate_top_z``, which is the input. ``None`` when
        the active plate has no stored offsets, because every delta would then
        be zero and the caller would be told the well floor is exactly at the
        plate's top surface: a confident, wrong, and dangerous answer.

        NOTE — the bore wizard's step 2 uses the SAME identity for one pair
        (``NeedleBoreWizard._plate_top_bottom_distance`` = ``bottom − top``,
        with the same "the fiducial cancels" reasoning). That one yields a
        scalar spacing to seed a typed spin box; this one maps every stored
        reference into the zero-ref frame. Kept separate deliberately: folding
        the hardware-verified v7.13 path into this would change a tested
        descent-planning input for no functional gain.
        """
        off = self._plate_z_offsets or {}
        if not any(k in off for k in ("top", "bottom", "safe", "max")):
            return None
        top_off = float(off.get("top", 0.0))
        top_user = self.zref_to_user_z(float(top_zref))

        def _from(key: str) -> float:
            # + is DOWN here: a feature further below the fiducial than the
            # top is that much below the taught top.
            return top_user - (float(off.get(key, top_off)) - top_off)

        return {
            "plate_bottom_z": self.user_z_to_zref(_from("bottom")),
            "safe_z": self.user_z_to_zref(_from("safe")),
            "plate_max_z": self.user_z_to_zref(_from("max")),
        }

    def apply_z_convention(self, z_up_sign: float | None = None,
                           needle_cam_z: float | None = None,
                           plate_z_offsets: dict | None = None,
                           plate_flip_180: bool | None = None) -> None:
        """Restore the persisted Z + plate-orientation convention (called on
        load/connect from the device profile / settings). All args optional;
        None leaves the current value unchanged."""
        if z_up_sign is not None:
            self.set_z_up_sign(z_up_sign)
        if needle_cam_z is not None:
            self._needle_cam_z_user = float(needle_cam_z)
        if plate_z_offsets:
            self.set_plate_z_offsets(
                top=plate_z_offsets.get("top"),
                bottom=plate_z_offsets.get("bottom"),
                safe=plate_z_offsets.get("safe"),
                max=plate_z_offsets.get("max"))
        if plate_flip_180 is not None:
            self.set_plate_flip_180(plate_flip_180)

    # ── v7.5.x: plate-bottom Z datum + print-time "don't punch through" ──

    def set_plate_bottom_z(self, z_zero_ref_mm: float | None,
                           at_xy_um: tuple | None = None,
                           source: str | None = None) -> None:
        """Set the calibrated plate-bottom Z (zero-ref mm), or None to clear.

        This is the deepest the needle may go during a print. Pushed from the
        Calibration page's ``plate_bottom_z`` reference. Setting it does NOT by
        itself enforce anything — the floor is only applied while a print is
        running (see :meth:`set_print_floor_active`).

        ``at_xy_um`` (v7.5.x, optional) records WHERE this scalar was measured,
        in absolute stage µm. That makes the scalar the anchor of a tilt plane
        (:meth:`set_plate_z_plane`) instead of a plate-wide constant of unknown
        provenance. ``source`` is one of ``"taught"`` / ``"estimated"`` /
        ``"restored"``: a needle-cam or plate-type *estimate* anchoring a plane
        would propagate its error across the whole plate, so it is recorded and
        surfaced rather than silently trusted. Both are optional and omitting
        them leaves behaviour identical to before.

        v7.9.1 — ``at_xy_um=None`` is deliberately a NO-OP, not a clear, because
        ``app.py``'s ``_update_print_floor_datum`` re-pushes the scalar untagged
        on every ``calibration_data_changed`` and must not wipe the anchor. A
        writer that genuinely has no anchor (the needle-cam *estimate*, which is
        measured nowhere on the plate) must therefore call
        :meth:`clear_plate_bottom_anchor` — otherwise a previous touch-off's XY
        would stay paired with the new, different Z, anchoring the bed-level
        plane at the right place and the wrong height.
        """
        self._plate_bottom_z_zref = (None if z_zero_ref_mm is None
                                     else float(z_zero_ref_mm))
        if at_xy_um is not None:
            try:
                self._plate_bottom_anchor_xy_um = (float(at_xy_um[0]),
                                                   float(at_xy_um[1]))
            except (TypeError, ValueError, IndexError):
                self._plate_bottom_anchor_xy_um = None
        if source is not None:
            self._plate_bottom_z_source = str(source)
        # Stamp the needle-zero epoch this scalar was measured against. A Set Z
        # Zero re-anchors the zero-ref frame, which makes every previously taught
        # Z number mean something different. set_plate_z_plane already refuses a
        # plane whose epoch has moved — loudly — but the SCALAR had no epoch at
        # all, so the same event left it stale SILENTLY. The scalar is the more
        # dangerous of the two: it is what every non-tilt-aware consumer reads.
        try:
            self._plate_bottom_zero_z_mm = float(
                (getattr(self, "zero_position", None) or {}).get("Z", 0.0) or 0.0)
        except (TypeError, ValueError, AttributeError):
            self._plate_bottom_zero_z_mm = None
        if self._plate_bottom_z_zref is not None:
            logger.info(f"StageController: plate-bottom Z datum = "
                        f"{self._plate_bottom_z_zref:.3f} mm (zero-ref)")

    def get_plate_bottom_z(self) -> float | None:
        """Calibrated plate-bottom Z (zero-ref mm), or None if uncalibrated.

        Deliberately the ANCHOR scalar — never the plane evaluated at the plate
        centre or at the last known XY. This is the number the operator taught
        and reads back on the Calibration page, in the jog context panel, on the
        XZ side view and in the readiness report; redefining it would silently
        change every displayed value and log line. Callers that want a position-
        aware answer ask :meth:`plate_bottom_z_at_um`; callers that want the tilt
        story ask :meth:`plate_bottom_z_extremes_zref`.
        """
        return self._plate_bottom_z_zref

    def get_plate_bottom_anchor_xy_um(self) -> tuple | None:
        """Where the plate-bottom scalar was measured (absolute stage µm)."""
        return getattr(self, "_plate_bottom_anchor_xy_um", None)

    def clear_plate_bottom_anchor(self) -> None:
        """Forget where the plate bottom was measured.

        v7.9.1. Needed because ``set_plate_bottom_z(at_xy_um=None)`` preserves
        the anchor by design (app.py re-pushes the scalar untagged and must not
        wipe it). A writer that replaces the scalar with a value measured
        NOWHERE on the plate — the needle-cam estimate — must call this, or a
        previous touch-off's XY stays paired with the new, different Z and the
        bed-level plane is anchored at the right place and the wrong height.
        """
        self._plate_bottom_anchor_xy_um = None

    def get_plate_bottom_z_source(self) -> str | None:
        """Provenance of the plate-bottom scalar: taught / estimated / restored."""
        return getattr(self, "_plate_bottom_z_source", None)

    #: Provenance tags whose plate-bottom scalar must NOT clamp motion.
    NON_CLAMPING_PLATE_BOTTOM_SOURCES = ("estimated",)

    def print_floor_datum_zref(self) -> float | None:
        """The plate-bottom scalar **only when it may serve as a floor**.

        v7.17. An ESTIMATED plate bottom is a planning number (plate top minus a
        datasheet offset), not a measurement, and it must never clamp motion —
        for two independent reasons, in opposite directions:

        * Estimated too HIGH (the real glass is lower than the datasheet
          implies): the clamp stops the needle above the glass, which BLOCKS the
          very touch-off that would measure the true bottom. The operator cannot
          calibrate their way out, because the guess is what is stopping them.
        * Estimated too LOW: the clamp passes a Z that punches through the
          glass, i.e. *false* protection — worse than none, because the caller
          believes it is guarded (the same reasoning that made
          ``set_print_floor_active`` refcounted).

        So the floor is armed only by a MEASURED bottom: the contact touch-off
        or the optical measurement. ``get_plate_bottom_z`` still returns the
        estimate — print heights, survey clearances and the readouts all want a
        best guess; only the clamp insists on a measurement.

        An absent/legacy ``source`` (None) is deliberately treated as clampable:
        only an explicit "estimated" tag disarms, so every pre-v7.17 path keeps
        its floor rather than silently losing it.
        """
        z = getattr(self, "_plate_bottom_z_zref", None)
        if z is None:
            return None
        src = getattr(self, "_plate_bottom_z_source", None)
        if src is not None and str(src) in self.NON_CLAMPING_PLATE_BOTTOM_SOURCES:
            return None
        return z

    def plate_bottom_zero_z_mm(self) -> float | None:
        """The needle-zero epoch the plate-bottom scalar was taught against.

        None means it was recorded before this was tracked (or never taught) —
        which callers must treat as "unknown", not as "unchanged".
        """
        return getattr(self, "_plate_bottom_zero_z_mm", None)

    def plate_bottom_z_is_stale(self, tol_mm: float = 0.01) -> bool:
        """True when Set Z Zero has run since the plate bottom was taught.

        Every Z taught in the old frame is offset by the change, so anchoring a
        tilt plane to the scalar — or descending against it — would be wrong by
        exactly that amount.
        """
        epoch = self.plate_bottom_zero_z_mm()
        if epoch is None or self._plate_bottom_z_zref is None:
            return False
        try:
            now = float((getattr(self, "zero_position", None)
                         or {}).get("Z", 0.0) or 0.0)
        except (TypeError, ValueError, AttributeError):
            return False
        return abs(now - epoch) > float(tol_mm)

    # ── v7.5.x: plate-bottom Z PLANE (tilt) ────────────────────────────
    #
    # The plate bottom is flat but tilted; on this machine the measured tilt is
    # ~1.15 mm across a 24-well plate's row span, i.e. several times a typical
    # 0.1–0.5 mm print height. The scalar datum above cannot express that, so a
    # print at the far side of the plate was either crashing into glass or
    # printing in mid air depending on the sign.
    #
    # Everything here degrades EXACTLY to the scalar when no plane is active, so
    # an install that has not run the new calibration sees bit-identical Z.

    def set_plate_footprint_bbox_um(self, bbox_um: tuple | None) -> None:
        """Region over which a tilt plane may be trusted (absolute stage µm).

        ``(x0, y0, x1, y1)``. Queries outside it fall back to the scalar rather
        than extrapolating a tilt measured elsewhere.
        """
        if bbox_um is None:
            self._plate_footprint_bbox_um = None
            return
        try:
            self._plate_footprint_bbox_um = (float(bbox_um[0]), float(bbox_um[1]),
                                             float(bbox_um[2]), float(bbox_um[3]))
        except (TypeError, ValueError, IndexError):
            self._plate_footprint_bbox_um = None

    def get_plate_footprint_bbox_um(self) -> tuple | None:
        """The trusted tilt region, or the plane's own taught bbox, else None."""
        box = getattr(self, "_plate_footprint_bbox_um", None)
        if box is not None:
            return box
        plane = getattr(self, "_plate_z_plane", None)
        if plane is not None:
            try:
                return plane.points_bbox_um()
            except Exception:
                return None
        return None

    def set_plate_tilt_enabled(self, enabled: bool) -> None:
        """Operator switch for using the tilt plane in print-Z resolution."""
        self._plate_tilt_enabled = bool(enabled)

    def would_accept_plate_z_plane(self, plane) -> tuple[bool, str]:
        """Would :meth:`set_plate_z_plane` accept this plane? ``(ok, reason)``.

        Read-only — installs nothing, mutates nothing. It exists so a calibration
        UI can show the operator the *actual* verdict before asking them to
        accept, rather than re-implementing the rules in the dialog (where they
        would drift) or installing-then-restoring (where a crash between the two
        leaves the machine holding a plane nobody approved).

        One validator, two entry points: this and :meth:`set_plate_z_plane` share
        ``_validate_plate_z_plane``, so a Verify screen and the print path can
        never disagree about whether a plane is trustworthy.
        """
        from SupportClasses.PlateZPlane import (
            PLANE_FRAME, PlateZPlane as _PZP, DEFAULT_RESIDUAL_TOL_MM,
            tilt_is_plausible,
        )
        if plane is None:
            return (False, "no plane")
        if isinstance(plane, dict):
            plane = _PZP.from_dict(plane)
            if plane is None:
                return (False, "plane payload is not in the stage frame")
        return self._validate_plate_z_plane(
            plane, PLANE_FRAME, DEFAULT_RESIDUAL_TOL_MM, tilt_is_plausible)

    def set_plate_z_plane(self, plane) -> tuple[bool, str]:
        """Install a measured plate-bottom tilt plane. ``(accepted, reason)``.

        The plane is stored either way (so the UI can explain a refusal) but is
        only *used* when it validates. A rejected plane is exactly equivalent to
        having no plane at all: the scalar remains the sole datum.

        Validation is what makes trusting a measured plane safe. In particular
        the epoch check catches the real failure already on disk here — a saved
        plane whose intercept disagreed with the taught plate bottom by 5.5 mm
        because the two were measured in different needle-zero epochs.
        """
        from SupportClasses.PlateZPlane import (
            PLANE_FRAME, PlateZPlane as _PZP, DEFAULT_RESIDUAL_TOL_MM,
            tilt_is_plausible,
        )
        if plane is None:
            self._plate_z_plane = None
            return (True, "")
        if isinstance(plane, dict):
            plane = _PZP.from_dict(plane)
            if plane is None:
                return (False, "plane payload is not in the stage frame")

        ok, why = self._validate_plate_z_plane(
            plane, PLANE_FRAME, DEFAULT_RESIDUAL_TOL_MM, tilt_is_plausible)
        try:
            from dataclasses import replace as _replace
            plane = _replace(plane, status=("active" if ok else "rejected"),
                             reject_reason=("" if ok else why))
        except Exception:
            pass
        self._plate_z_plane = plane
        if ok:
            logger.info("StageController: plate Z plane ACTIVE — %s",
                        getattr(plane, "describe", lambda: "")())
        else:
            logger.warning(
                "StageController: plate Z plane REJECTED (%s) — using the "
                "single taught plate-bottom Z (no tilt correction)", why)
        return (ok, why)

    def _validate_plate_z_plane(self, plane, expect_frame: str,
                                resid_tol_mm: float,
                                tilt_is_plausible) -> tuple[bool, str]:
        """Decide whether a plane may drive print Z. ``(ok, reason)``."""
        if getattr(plane, "frame", None) != expect_frame:
            return (False, "plane is not in the stage frame "
                           "(legacy plate-local plane — re-teach the plate Z)")
        if getattr(plane, "degenerate", False):
            return (False, "gradient underdetermined (collinear touch points)")
        n = int(getattr(plane, "num_points", 0) or 0)
        if n < 3:
            return (False, f"only {n} touch point(s) — need ≥3")

        scalar = self._plate_bottom_z_zref
        if scalar is None:
            return (False, "no taught plate-bottom Z to anchor against")

        # Epoch: the plane's Z values live in the zero-ref frame that existed
        # when it was measured. A Set Z Zero since then invalidates them.
        zero_z = float(self.zero_position.get("Z", 0.0) or 0.0)
        fit_zero = getattr(plane, "zero_z_mm_at_fit", None)
        if fit_zero is not None and abs(float(fit_zero) - zero_z) > 0.01:
            return (False,
                    f"needle zero changed since the plate plane was measured "
                    f"({float(fit_zero):.3f} → {zero_z:.3f} mm)")

        # Orientation: a 180° remount invalidates the taught XY the points were
        # measured at, hence the plane.
        cur_flip = (self.plate_flip_180()
                    if hasattr(self, "plate_flip_180") else None)
        p_flip = getattr(plane, "plate_flip_180", None)
        if p_flip is not None and cur_flip is not None and bool(p_flip) != bool(cur_flip):
            return (False, "plate orientation changed since the plane was measured")

        # Anchor agreement. Zero by construction for a plane whose anchor is the
        # taught touch-off, so this catches a LATER manual re-teach of the scalar.
        try:
            at_anchor = plane.z_zref_mm_at_stage_um(plane.x0_um, plane.y0_um)
        except Exception:
            return (False, "plane could not be evaluated")
        if abs(at_anchor - float(scalar)) > 0.05:
            return (False,
                    f"plane anchor {at_anchor:.3f} mm disagrees with the taught "
                    f"plate bottom {float(scalar):.3f} mm")

        ok, why = tilt_is_plausible(
            getattr(plane, "sx_mm_per_mm", 0.0),
            getattr(plane, "sy_mm_per_mm", 0.0),
            bbox_um=self.get_plate_footprint_bbox_um())
        if not ok:
            return (False, why)

        # Quality. At exactly 3 points the anchored fit is exact and R² is
        # identically 1.0, so it proves nothing — such a plane is allowed but
        # the UI labels it unverified. Redundancy is where residuals mean
        # something, and the hold-out is the only real evidence.
        if n >= 4:
            rmax = float(getattr(plane, "residual_max_mm", 0.0) or 0.0)
            if rmax > resid_tol_mm:
                return (False,
                        f"max residual {rmax * 1000.0:.0f} µm exceeds "
                        f"{resid_tol_mm * 1000.0:.0f} µm")
        hold = getattr(plane, "holdout_error_mm", None)
        if hold is not None and float(hold) > resid_tol_mm:
            return (False,
                    f"hold-out error {float(hold) * 1000.0:.0f} µm exceeds "
                    f"{resid_tol_mm * 1000.0:.0f} µm — one touch point looks wrong")
        return (True, "")

    def get_plate_z_plane(self):
        """The stored plane whatever its status (so the UI can explain it)."""
        return getattr(self, "_plate_z_plane", None)

    def active_plate_z_plane(self):
        """The plane ONLY if it validated — otherwise None.

        Every consumer gates on this, so the print target and the print floor can
        never disagree about whether the tilt is real.
        """
        plane = getattr(self, "_plate_z_plane", None)
        if plane is None or getattr(plane, "status", "") != "active":
            return None
        return plane

    def plate_tilt_enabled(self) -> bool:
        """True when a validated plane exists AND the operator enabled it."""
        return bool(getattr(self, "_plate_tilt_enabled", False)
                    and self.active_plate_z_plane() is not None)

    def plate_bottom_z_at_um(self, x_stage_um: float | None,
                            y_stage_um: float | None) -> float | None:
        """Plate-bottom Z (zero-ref mm) at an absolute stage µm position.

        Falls back to the taught scalar — i.e. to exactly today's behaviour —
        when the tilt is not enabled, the position is unknown, or the position is
        outside the taught region.
        """
        scalar = self._plate_bottom_z_zref
        if x_stage_um is None or y_stage_um is None:
            return scalar
        plane = self.active_plate_z_plane()
        if plane is None or not getattr(self, "_plate_tilt_enabled", False):
            return scalar
        try:
            if not plane.is_inside(x_stage_um, y_stage_um,
                                   bbox_um=self.get_plate_footprint_bbox_um()):
                return scalar
            return plane.z_zref_mm_at_stage_um(x_stage_um, y_stage_um)
        except Exception as e:                       # never break a motion path
            logger.debug("plate_bottom_z_at_um failed (%s) — using the scalar", e)
            return scalar

    def plate_bottom_z_at_zref_mm(self, x_zref_mm: float | None,
                                  y_zref_mm: float | None) -> float | None:
        """Plate-bottom Z (zero-ref mm) at a ZERO-REF mm XY position.

        Convenience for the print path, which works in zero-ref mm. This is the
        only place the zero-ref-mm → absolute-µm conversion is written.
        """
        if x_zref_mm is None or y_zref_mm is None:
            return self._plate_bottom_z_zref
        zx = float(self.zero_position.get("x", 0) or 0)
        zy = float(self.zero_position.get("y", 0) or 0)
        return self.plate_bottom_z_at_um(float(x_zref_mm) * 1000.0 + zx,
                                         float(y_zref_mm) * 1000.0 + zy)

    def plate_bottom_z_extremes_zref(self, footprint_um: tuple | None = None
                                     ) -> tuple | None:
        """``(shallowest, deepest)`` plate-bottom Z over the plate, zero-ref mm.

        Compared in the ZDIR-scaled HEIGHT frame, so "shallowest" means highest
        physically on both polarities. Used for the operator-facing tilt readout
        and the pre-print advisory — NOT for the runtime floor.
        """
        plane = self.active_plate_z_plane()
        scalar = self._plate_bottom_z_zref
        if plane is None:
            return None if scalar is None else (scalar, scalar)
        box = footprint_um if footprint_um is not None \
            else self.get_plate_footprint_bbox_um()
        if box is None:
            return None if scalar is None else (scalar, scalar)
        try:
            return plane.extremes_zref(box, self.print_z_dir())
        except Exception:
            return None if scalar is None else (scalar, scalar)

    def plate_z_tilt_span_mm(self) -> float | None:
        """Total plate-bottom variation across the plate (mm), or None."""
        plane = self.active_plate_z_plane()
        box = self.get_plate_footprint_bbox_um()
        if plane is None or box is None:
            return None
        try:
            return plane.span_mm(box)
        except Exception:
            return None

    def plate_z_plane_for_job(self) -> dict | None:
        """The plane as a job-stampable dict in ZERO-REF mm XY, or None.

        THE single stamp source. Print jobs carry per-machine facts (``z_up_sign``,
        ``plate_axis_sign``, ``well_positions_mm``) frozen at build time so a run
        is reproducible from the job — and so a re-teach between build and run
        cannot silently change a resumed print's Z. The tilt plane follows the
        same rule. Returns None unless the tilt is enabled AND validated, which
        is what keeps this a no-op for installs that have not recalibrated.
        """
        plane = self.active_plate_z_plane()
        if plane is None or not getattr(self, "_plate_tilt_enabled", False):
            return None
        zx = float(self.zero_position.get("x", 0) or 0)
        zy = float(self.zero_position.get("y", 0) or 0)
        try:
            return {
                "x0_mm": (plane.x0_um - zx) / 1000.0,
                "y0_mm": (plane.y0_um - zy) / 1000.0,
                "z0_zref_mm": float(plane.z0_zref_mm),
                "sx_mm_per_mm": float(plane.sx_mm_per_mm),
                "sy_mm_per_mm": float(plane.sy_mm_per_mm),
                "plane_id": (plane.fitted_at or ""),
            }
        except Exception as e:
            logger.debug("plate_z_plane_for_job failed (%s)", e)
            return None

    def set_plate_top_z(self, z_zero_ref_mm: float | None) -> None:
        """Set the calibrated plate-top Z (zero-ref mm), or None to clear.

        Paired with the plate bottom, this defines the reference vector that
        tells :meth:`print_z_dir` which way is *up*. Pushed from the Calibration
        page's ``plate_top_z`` reference. No motion / enforcement on its own.
        """
        self._plate_top_z_zref = (None if z_zero_ref_mm is None
                                  else float(z_zero_ref_mm))
        if self._plate_top_z_zref is not None:
            logger.info(f"StageController: plate-top Z datum = "
                        f"{self._plate_top_z_zref:.3f} mm (zero-ref)")

    def get_plate_top_z(self) -> float | None:
        """Calibrated plate-top Z (zero-ref mm), or None if uncalibrated."""
        return getattr(self, "_plate_top_z_zref", None)

    # ── v7.5.x: needle ↔ microscope-camera-centre offset ───────────────
    #
    # Every live-view click→target path in the app implicitly assumes the needle
    # sits exactly under the camera crosshair. It does not, so each such target is
    # off by this vector (easily 0.5–3 mm for a side-mounted objective). Measured
    # once during the plate-Z touch-off, when the needle is confirmed on the glass
    # and the operator clicks its tip.
    #
    # ONE function owns the sign so no consumer can re-derive it wrongly.

    def set_needle_camera_offset_um(self, dx_um: float | None,
                                    dy_um: float | None = None) -> None:
        """Record the needle's offset from the microscope camera centre (stage µm).

        Pass ``None`` to clear. The offset means
        ``(needle stage position) − (camera crosshair stage position)``.
        """
        if dx_um is None:
            self._needle_cam_offset_um = None
            return
        try:
            self._needle_cam_offset_um = (float(dx_um), float(dy_um or 0.0))
        except (TypeError, ValueError):
            self._needle_cam_offset_um = None

    def get_needle_camera_offset_um(self) -> tuple | None:
        """The needle's offset from the microscope camera centre (stage µm)."""
        return getattr(self, "_needle_cam_offset_um", None)

    def needle_target_xy_for_feature_um(self, feature_x_um: float,
                                       feature_y_um: float) -> tuple:
        """Stage XY to command so the NEEDLE lands on a feature seen in the view.

        ``feature_*`` is the feature's absolute stage position — normally
        ``current_xy + CameraManager.pixel_to_stage_offset(click)``. Returns the
        feature position unchanged when no offset has been measured, so callers
        can adopt this unconditionally and behave exactly as today until the
        calibration has run.

        Use this for click-to-PICK (put the needle on the thing). Do NOT use it
        for click-to-CENTRE (put the thing under the crosshair, for imaging or
        mosaic work) — that wants the raw feature position.
        """
        off = self.get_needle_camera_offset_um()
        if not off:
            return (float(feature_x_um), float(feature_y_um))
        return (float(feature_x_um) - float(off[0]),
                float(feature_y_um) - float(off[1]))

    def print_z_dir(self) -> float:
        """Up-direction sign for print-Z math, derived from the calibrated
        plate-bottom→plate-top reference vector.

        Returns +1.0 when "up" is a larger zero-ref Z (conventional) and -1.0
        when "up" is a smaller zero-ref Z (ME3B V1). Falls back to the module
        ``ZDIR`` when the plate top and bottom aren't both calibrated, so print
        behaviour is unchanged until both references are taught.
        """
        return derive_z_up_sign(getattr(self, "_plate_top_z_zref", None),
                                getattr(self, "_plate_bottom_z_zref", None),
                                fallback=self.z_up_sign())

    def set_print_floor_active(self, active: bool) -> None:
        """Arm/disarm the plate-bottom floor.

        REFCOUNTED. Several independent subsystems arm this — the print
        executors, pick & place, and the needle-calibration touch-off — and they
        can overlap. With a plain flag the first disarm would drop the floor
        while another caller is still descending, i.e. *false* protection, which
        is worse than no protection because the caller believes it is guarded.

        So each ``True`` increments and each ``False`` decrements; the floor is
        active while the count is positive. The signature is deliberately
        unchanged, so every existing caller keeps working — as long as its
        arm/disarm are balanced, which they are (all three wrap the disarm in a
        ``finally``).

        The count is clamped at zero: an unbalanced extra disarm cannot drive it
        negative and thereby make a later arm a no-op.
        """
        if active:
            self._print_floor_depth = getattr(self, "_print_floor_depth", 0) + 1
        else:
            self._print_floor_depth = max(
                0, getattr(self, "_print_floor_depth", 0) - 1)

    @property
    def _print_floor_active(self) -> bool:
        """True while at least one caller has the plate-bottom floor armed.

        Kept as a property under the original name so existing readers (and
        tests that assert on it) are unaffected by the move to a refcount.
        """
        return getattr(self, "_print_floor_depth", 0) > 0

    @_print_floor_active.setter
    def _print_floor_active(self, value) -> None:
        """Direct assignment forces the count, for tests and hard resets."""
        self._print_floor_depth = 1 if value else 0

    def print_height_to_zref(self, height_above_bottom_mm: float,
                             x_zref_mm: float | None = None,
                             y_zref_mm: float | None = None) -> float | None:
        """Height above the plate bottom (mm) → zero-ref Z, or None if the
        plate bottom is not yet calibrated.

        v7.5.x: pass the target's ZERO-REF mm XY to resolve the plate bottom from
        the measured tilt plane at that position. Called with no XY — as every
        pre-existing caller does — it uses the scalar datum and is byte-identical
        to before.
        """
        pb = (self._plate_bottom_z_zref if x_zref_mm is None
              else self.plate_bottom_z_at_zref_mm(x_zref_mm, y_zref_mm))
        if pb is None:
            return None
        return plate_relative_to_zref(pb, float(height_above_bottom_mm),
                                      zdir=self.print_z_dir())

    def zref_to_print_height(self, z_zero_ref_mm: float,
                             x_zref_mm: float | None = None,
                             y_zref_mm: float | None = None) -> float | None:
        """Zero-ref Z (mm) → height above the plate bottom (mm), or None."""
        pb = (self._plate_bottom_z_zref if x_zref_mm is None
              else self.plate_bottom_z_at_zref_mm(x_zref_mm, y_zref_mm))
        if pb is None:
            return None
        return zref_to_plate_relative(pb, float(z_zero_ref_mm),
                                      zdir=self.print_z_dir())

    def print_floor_violation(self, z_zero_ref_mm: float,
                              x_zref_mm: float | None = None,
                              y_zref_mm: float | None = None) -> bool:
        """True if a zero-ref Z would put the needle *below* the plate bottom.

        Used for the early warning before a print starts. Returns False when
        the plate bottom is uncalibrated (nothing to compare against).
        """
        pb = (self._plate_bottom_z_zref if x_zref_mm is None
              else self.plate_bottom_z_at_zref_mm(x_zref_mm, y_zref_mm))
        if pb is None:
            return False
        return zref_to_plate_relative(
            pb, float(z_zero_ref_mm), zdir=self.print_z_dir()) < -1e-6

    def _apply_print_floor_raw(self, raw_z: float) -> float:
        """Clamp a *raw* Marlin Z so the needle never goes deeper than the
        plate bottom. No-op unless the floor is armed AND the datum was
        measured (:meth:`print_floor_datum_zref` — an estimate never clamps).

        Polarity-general: ``up*(raw - plate_bottom_raw) < 0`` means "deeper than
        the floor" for either Z direction, and we cap at the floor. ``up`` is
        the reference-vector direction (:meth:`print_z_dir`), falling back to
        ``ZDIR`` when the plate top isn't taught.
        """
        if not self._print_floor_active:
            return raw_z
        # v7.17: an ESTIMATED bottom is not a floor — see print_floor_datum_zref.
        datum = self.print_floor_datum_zref()
        if datum is None:
            return raw_z
        pb_raw = datum + self.zero_position.get("Z", 0.0)
        if self.print_z_dir() * (raw_z - pb_raw) < 0:
            logger.warning(
                f"Print floor: Z {raw_z:.3f} would punch through the plate "
                f"bottom ({pb_raw:.3f} mm raw) — clamped to plate bottom")
            return pb_raw
        return raw_z

    def apply_device_settings(self, axis_map: dict | None = None,
                              steps_per_mm: dict | None = None,
                              per_axis_max_feedrate: dict | None = None,
                              persist_steps: bool = False,
                              persist_feedrate: bool = False,
                              xy_max_speed_um_s: float | None = None) -> None:
        """v7.4.2: Push device-level settings into a connected ZP stage.

        Called by MainWindow after construction (or when the user clicks
        Apply on the Device sub-page). If the ZP stage is not yet
        connected, the values are stored on the controller for use when
        it eventually connects.

        Args:
            axis_map: Logical→physical axis mapping. None to skip.
            steps_per_mm: Per-logical-axis stepper calibration. None to skip.
            per_axis_max_feedrate: Per-logical-axis max feedrate ceiling
                (mm/min). None to skip.
            persist_steps: If True and zp_stage is connected, send M92
                so the new calibration takes effect on Marlin.
            persist_feedrate: If True and zp_stage is connected, send M203
                so the new per-axis feedrate ceilings take effect on Marlin.
            xy_max_speed_um_s: v7.21.1 — this machine's DECLARED true XY top
                speed (µm/s at 100%), from Hardware Setup → Device → XY Stage
                Calibration. None to skip (leaves any existing declaration
                alone), so callers that only push axis/stepper settings are
                unchanged.
        """
        # v7.21.1: declared FIRST — safe_travel_to and the jog anchors resolve
        # through it, so it must be in force before anything else re-anchors.
        if xy_max_speed_um_s is not None:
            self.set_xy_top_speed_um_s(xy_max_speed_um_s)
        # Cache for next connect
        if axis_map is not None:
            self._pending_axis_map = dict(axis_map)
        if steps_per_mm is not None:
            self._pending_steps_per_mm = dict(steps_per_mm)
        if per_axis_max_feedrate is not None:
            self._pending_per_axis_max_feedrate = dict(per_axis_max_feedrate)
        # Push live if connected
        if self.zp_stage is not None:
            if axis_map is not None:
                self.zp_stage.set_axis_map(axis_map)
            if steps_per_mm is not None:
                self.zp_stage.set_steps_per_mm(steps_per_mm, persist=persist_steps)
            if per_axis_max_feedrate is not None:
                self.zp_stage.set_per_axis_max_feedrate(
                    per_axis_max_feedrate, persist=persist_feedrate)
        # v7.5.x: the Z jog speed-% is anchored to per_axis_max_feedrate['Z'],
        # so re-anchor whenever it changes (no-op if jog handler not connected).
        if per_axis_max_feedrate is not None:
            try:
                self.refresh_jog_speed_limits()
            except Exception as e:
                logger.debug(f"refresh_jog_speed_limits (device settings) failed: {e}")
        # v7.5.x: re-derive the Z retract/insert feedrates from the (possibly
        # updated) Z max so Z moves stay fast.
        try:
            self._refresh_zp_move_feedrates()
        except Exception as e:
            logger.debug(f"_refresh_zp_move_feedrates failed: {e}")

    def connect_xy(self, simulate: bool | None = None) -> None:
        """Connect only the XY stage.

        Args:
            simulate: If True, opens a simulator instead of real hardware.
                ``None`` falls back to the ``__init__`` default (real
                hardware unless ``--simulate-xy`` was passed). The Hardware
                Setup → Device page passes ``False`` from "Connect" and
                ``True`` from "Simulate".
        """
        self.connect_stages(xy=True, zp=False, simulate_xy=simulate)

    def connect_zp(self, simulate: bool | None = None) -> None:
        """Connect only the ZP stage.

        See :meth:`connect_xy` for the meaning of ``simulate``.
        """
        self.connect_stages(xy=False, zp=True, simulate_zp=simulate)

    def _handle_disconnect(self, stage_name: str) -> None:
        # v7.5.x ZP reconnect hotfix: may be called from the watchdog thread
        # AND the poller thread — serialize and make idempotent so a stage is
        # only torn down (and the GUI only notified) once per disconnect.
        with self._disconnect_lock:
            if stage_name == "XY":
                if self.xy_stage is None:
                    return
                logger.error("XY stage disconnected!")
                self.disconnect_xy()
            elif stage_name == "ZP":
                if self.zp_stage is None:
                    return
                logger.error("ZP stage disconnected!")
                self.disconnect_zp()
            else:
                return
        if self.on_disconnect:
            self.on_disconnect(stage_name)
        # v7.5.x: an UNEXPECTED ZP loss (USB drop) → try to auto-reconnect in
        # the background. Gated to real hardware + enabled + not shutting down.
        # getattr-safe so __new__-constructed test controllers don't trigger it.
        if (stage_name == "ZP" and getattr(self, "auto_reconnect_zp", False)
                and not getattr(self, "_default_simulate_zp", True)
                and not getattr(self, "_shutting_down", False)):
            self._schedule_zp_reconnect()

    def _schedule_zp_reconnect(self) -> None:
        """Start a background ZP reconnect attempt (idempotent)."""
        if self._zp_reconnecting or self._shutting_down:
            return
        self._zp_reconnecting = True
        threading.Thread(target=self._zp_reconnect_loop, daemon=True,
                         name="ZPReconnect").start()

    def _zp_reconnect_loop(self) -> None:
        """Retry connecting the ZP a few times (the CH340 re-enumerates on the
        same COM port after a drop). On success, connect_stages fires
        on_connect("ZP") → the GUI's position-restore prompt (Marlin reset on
        the reopen). Best-effort; never raises."""
        try:
            n = max(1, int(self._zp_reconnect_attempts))
            for attempt in range(1, n + 1):
                if self._shutting_down:
                    return
                time.sleep(max(0.2, float(self._zp_reconnect_delay_s)))
                if self.is_zp_connected:
                    return  # came back (e.g. a manual reconnect) — done
                logger.info(f"ZP auto-reconnect: attempt {attempt}/{n}…")
                try:
                    self.connect_zp(simulate=False)
                except Exception as e:
                    logger.info(f"ZP auto-reconnect attempt {attempt} "
                                f"failed: {e}")
                if self.is_zp_connected:
                    logger.info("ZP auto-reconnect succeeded — re-declare the "
                                "Z position (Marlin reset on reopen).")
                    try:
                        _zp_tracer = __import__(
                            "SupportClasses.ZPSerialTrace",
                            fromlist=["tracer"]).tracer()
                        _zp_tracer.event("auto_reconnect_ok",
                                         attempt=attempt)
                    except Exception:
                        pass
                    return
            logger.warning(f"ZP auto-reconnect gave up after {n} attempts — "
                           f"reconnect manually on Hardware Setup → Device.")
        finally:
            self._zp_reconnecting = False

    def _refresh_zp_move_feedrates(self) -> None:
        """v7.5.x: set the Z retract/insert feedrates from the configured Z max
        so Z moves are fast — short on-time (less heat) and they finish within
        the M400 flush timeout. Retract (UP, always safe) uses the full Z max;
        insert (DOWN toward the plate) uses a moderate 0.6× so the descent isn't
        slammed (it's also plate-floor clamped + arrival-confirmed). No-op if no
        Z max is configured (keeps the conservative __init__ defaults)."""
        # v7.5.x: source the Z max through the single common resolver.
        z_max = None
        try:
            z_max = float(self.get_max_z_feedrate_mm_min())
        except Exception:
            z_max = None
        if z_max and z_max > 0:
            self._zp_retract_feedrate = z_max
            self._zp_insert_feedrate = max(z_max * 0.6, 100.0)

    def disconnect_xy(self) -> None:
        if self.xy_jog:
            self.xy_jog.stop()
            self.xy_jog = None
        if self.xy_stage:
            try:
                self.xy_stage.stop()
            except Exception:
                pass
            self.xy_stage = None
        self._pos_poller.set_stages(None, self.zp_stage)
        self._watchdog.unwatch("XY")
        logger.info("XY stage disconnected")

    def disconnect_zp(self) -> None:
        # v7.5.x ZP reconnect hotfix: release EVERY ZP resource in a safe
        # order so a later Connect can re-acquire the port within the same
        # process (previously a stale handle / poller reference could keep
        # the COM port busy until a full app restart).
        # 1) Stop the poller from touching the ZP serial BEFORE we close it
        #    (prevents the poll thread racing serial.close()).
        self._pos_poller.set_stages(self.xy_stage, None)
        # 2) Detach the watchdog: the port watch AND the periodic M500 save
        #    (the latter was leaked on every connect — never removed before).
        self._watchdog.unwatch("ZP")
        try:
            self._watchdog.remove_periodic(self._periodic_zp_position_save)
        except Exception:
            pass
        # 3) Stop the jog handler (unregisters its Processor handlers + thread).
        if self.zp_jog:
            try:
                self.zp_jog.stop()
            except Exception:
                pass
            self.zp_jog = None
        # 4) Close the serial handle and drop the object.
        if self.zp_stage:
            try:
                self.zp_stage.stop()
            except Exception:
                pass
            self.zp_stage = None
        # 5) Forget the last-known-good port so the next scan rediscovers the
        #    board on whatever COM it re-enumerated to after a power-cycle.
        self._preferred_zp_port = None
        logger.info("ZP stage disconnected")

    def disconnect_stages(self) -> None:
        self.disconnect_xy()
        self.disconnect_zp()

    # ── Xbox Controller ───────────────────────────────────────────

    def connect_xbox(self, mapping_file: str = "current_button_mapping.json",
                     use_thread: bool = False,
                     reconnect_timeout: float = 30.0,
                     stick_offsets: dict | None = None,
                     axis_deadzones: dict | None = None,
                     debug_mode: bool = False) -> None:
        """Connect Xbox controller. v7.2.7: thread fallback, v7.3.2: stick offsets,
        v7.3.4: axis_deadzones, debug_mode.

        Args:
            mapping_file: Path to button mapping JSON.
            use_thread: If True, run worker in a thread instead of process.
                        Use this for macOS Bluetooth controllers that are
                        invisible to spawned subprocesses.
            reconnect_timeout: Seconds to attempt reconnection after loss.
            stick_offsets: Per-axis center offsets from calibration (v7.3.2).
            axis_deadzones: Per-axis deadzone thresholds, keys are int axis
                            indices, values 0–1 (v7.3.4).
        """
        if self.xbox_process and self.xbox_process.is_alive():
            logger.warning("Xbox already connected")
            return
        # Also check thread-based worker
        if getattr(self, "_xbox_thread", None) and self._xbox_thread.is_alive():
            logger.warning("Xbox already connected (thread mode)")
            return

        from pathlib import Path as _Path
        self._mapping_file = str(_Path(mapping_file).resolve())

        if self.xy_stage is None and self.zp_stage is None:
            logger.warning(
                "Xbox started but no stages are connected -- "
                "controller input will have no effect until stages connect"
            )

        self.xbox_queue = Queue()
        # v7.5.x: live tuning channel + clean-stop signal for the worker
        self.xbox_ctrl_queue = Queue()
        self._xbox_stop_event = MPEvent()

        # v7.3.2: Convert stick_offsets keys to int for multiprocessing
        _so = None
        if stick_offsets:
            _so = {int(k): v for k, v in stick_offsets.items()}

        # v7.3.4: Convert axis_deadzones keys to int for multiprocessing
        _adz = None
        if axis_deadzones:
            _adz = {int(k): float(v) for k, v in axis_deadzones.items()}

        _worker_kwargs = {
            "mapping_file": self._mapping_file,
            "reconnect_timeout": reconnect_timeout,
            "stick_offsets": _so,
            "axis_deadzones": _adz,
            "debug_mode": debug_mode,
            "ctrl_queue": self.xbox_ctrl_queue,
            "stop_event": self._xbox_stop_event,
        }

        if use_thread:
            import threading
            self._xbox_thread = threading.Thread(
                target=xbox_polling_worker,
                args=(self.xbox_queue,),
                kwargs=_worker_kwargs,
                daemon=True,
                name="XboxWorkerThread",
            )
            self._xbox_thread.start()
            self.xbox_process = None  # Not using process mode
            logger.info(f"Xbox worker started (THREAD mode, mapping: {self._mapping_file})")
        else:
            self.xbox_process = Process(
                target=xbox_polling_worker,
                args=(self.xbox_queue,),
                kwargs=_worker_kwargs,
                daemon=True,
            )
            self.xbox_process.start()
            logger.info(f"Xbox worker started (PROCESS mode, mapping: {self._mapping_file})")

        self.xbox_poller = XboxQueuePoller(
            self.xbox_queue, self.processor, debug_mode=debug_mode,
            on_lost=self._zero_jog_velocities,  # v7.5.x: stop on loss
        )
        self.xbox_poller.start()
        self.xbox_poller._xbox_status = "waiting"  # v7.2.7: force initial status


    def disconnect_xbox(self) -> None:
        """Disconnect Xbox controller. v7.2.7: thread cleanup,
        v7.5.x: stop_event lets thread-mode workers exit cleanly too."""
        # v7.5.x: signal the worker first so it dispatches its final zeroes
        # and exits its loop (works for both process and thread mode).
        _ev = getattr(self, "_xbox_stop_event", None)
        if _ev is not None:
            try:
                _ev.set()
            except Exception:
                pass
        if self.xbox_poller:
            self.xbox_poller.stop()
            self.xbox_poller = None
        if self.xbox_process and self.xbox_process.is_alive():
            self.xbox_process.terminate()
            self.xbox_process.join(timeout=2.0)
            self.xbox_process = None
        # v7.2.7: thread mode cleanup — v7.5.x: the stop_event above makes
        # the thread-mode worker exit on its next loop iteration.
        _xt = getattr(self, "_xbox_thread", None)
        if _xt and _xt.is_alive():
            _xt.join(timeout=1.0)
        self._xbox_thread = None
        self.xbox_queue = None
        self.xbox_ctrl_queue = None
        self._xbox_stop_event = None
        # v7.3.4: Zero all jog velocities on disconnect so a stranded velocity
        # from an in-flight command can't persist and drive the stage.
        self._zero_jog_velocities()
        logger.info("Xbox controller disconnected")

    def _zero_jog_velocities(self) -> None:
        """v7.5.x: zero every jog-handler velocity (Z, pumps, XY).

        Called on Xbox disconnect, on worker-reported controller loss, and
        on heartbeat staleness, so a stranded velocity can never keep
        driving the stages. The XY jog loop sends its explicit zero-velocity
        stop on the resulting moving→stopped transition.
        """
        if self.zp_jog:
            with self.zp_jog._lock:
                self.zp_jog.vel_z = 0.0
                self.zp_jog.vel_p1 = 0.0
                self.zp_jog.vel_p2 = 0.0
                self.zp_jog.vel_p3 = 0.0
        if self.xy_jog:
            with self.xy_jog._lock:
                self.xy_jog.vel_x = 0.0
                self.xy_jog.vel_y = 0.0

    def update_xbox_tuning(self, stick_offsets: dict | None = None,
                           axis_deadzones: dict | None = None,
                           debug_mode: bool | None = None) -> bool:
        """v7.5.x: push tuning changes to the RUNNING Xbox worker.

        Previously deadzones / stick offsets / debug mode were process
        arguments, so panel changes only took effect after a full
        disconnect + reconnect. Returns True if an update was queued.
        """
        q = getattr(self, "xbox_ctrl_queue", None)
        if q is None:
            return False
        payload: dict = {}
        if stick_offsets is not None:
            payload["stick_offsets"] = {int(k): float(v)
                                        for k, v in stick_offsets.items()}
        if axis_deadzones is not None:
            payload["axis_deadzones"] = {int(k): float(v)
                                         for k, v in axis_deadzones.items()}
        if debug_mode is not None:
            payload["debug_mode"] = bool(debug_mode)
            if self.xbox_poller is not None:
                self.xbox_poller.debug_mode = bool(debug_mode)
        if not payload:
            return False
        try:
            q.put(payload)
            return True
        except Exception as e:
            logger.warning(f"Xbox tuning update failed: {e}")
            return False


    # ── v7.5.x: Xbox jog speed as a % of the calibrated max ────────

    def _pump_jog_max_native(self) -> float:
        """Pump 100% anchor for the jog speed-%.

        µL mode (HardwareConfig present) → the largest configured safe flow
        rate (µL/s); the per-pump safe rate is still enforced downstream by
        ``_clamp_pump_flow`` so anchoring to the fastest can't let a slower
        pump exceed its limit. Legacy/no-config → ``max_pump_feedrate`` ÷ 60
        (mm/s). Returns 0.0 when nothing usable is configured (no-op upstream).
        """
        hw = self._hardware_config
        sl = self.safety_limits
        if hw is not None and getattr(hw, "configured_pump_ids", None):
            rates: list[float] = []
            if sl is not None:
                for pid in hw.configured_pump_ids:
                    try:
                        r = float(sl.get_max_flow_rate(pid))
                    except Exception:
                        r = 0.0
                    if r > 0:
                        rates.append(r)
            if rates:
                return max(rates)
            return 10.0  # µL/s default when no flow limit is computed yet
        if sl is not None and getattr(sl, "max_pump_feedrate", 0):
            try:
                return float(sl.max_pump_feedrate) / 60.0
            except Exception:
                return 0.0
        return 0.0

    # ── v7.5.x: ONE common per-axis MAX-speed source ──────────────
    #
    # Every page (Control Panel, Stage Panel, Jog, Quick Print, Xbox) and every
    # internal consumer reads the per-axis max through THESE resolvers, so a max
    # set on one page is inherited everywhere. Precedence is defined here ONCE.

    # Conservative fallbacks for the no-controller / nothing-configured case.
    _XY_MAX_FALLBACK_UM_S: float = 10_000.0   # µm/s
    _Z_MAX_FALLBACK_MM_MIN: float = 500.0     # mm/min

    def _explicit_xy_top_speed_um_s(self) -> float | None:
        """Only the operator's explicit declaration (Hardware Setup → Device →
        XY Stage Calibration → Max speed, or the timing tool's Apply). None when
        never declared.

        Accepts only a genuine number. ``float()`` alone is NOT enough: a
        ``MagicMock`` implements ``__float__`` and answers 1.0, so a
        partially-stubbed controller — the pattern this repo's GUI tests use —
        would otherwise inject a 1 µm/s "top speed" that every speed command
        then scales against, and the test would pass while production was
        wrong."""
        return _positive_number(getattr(self, "_declared_xy_top_speed_um_s", None))

    def declared_xy_top_speed_um_s(self) -> float | None:
        """The EXPLICITLY declared true XY top speed (µm/s at 100%), or None.

        v7.21.1: this is the authority behind the Hardware Setup → Device → XY
        Stage Calibration "Max speed" field. It is deliberately separate from
        :meth:`get_max_xy_speed_um_s`, which can never answer None because
        ``SafetyLimits.max_xy_speed`` carries a 10000 µm/s DEFAULT.

        That distinction is load-bearing, not tidiness. This value becomes the
        denominator in ``XYStage.set_speed_mm_s`` (``SMS% = requested ÷ top``),
        so declaring a top speed BELOW the stage's real one makes every
        commanded speed run proportionally FASTER than asked. A machine that has
        never declared one must therefore keep the protocol's own ``max_speed``,
        not inherit a policy default — hence None rather than a fallback.

        Precedence: operator declaration → the timing tool's stored measurement
        (legacy: pre-v7.21.1 the tool wrote there silently; it now offers the
        value for explicit Apply instead) → None (undeclared).

        NOTE the deliberate asymmetry with :meth:`get_max_xy_speed_um_s`: there,
        ``safety_limits.max_xy_speed`` outranks the stored measurement, because
        editing the safety value must propagate over a stale measurement. Here
        the safety value is absent entirely — it carries a default and so cannot
        assert anything about the physical stage.
        """
        v = self._explicit_xy_top_speed_um_s()
        if v:
            return v
        try:
            from SupportClasses.PrintTimingCalibrationStore import (
                get_store as _get_tc_store,
            )
            ms = _get_tc_store().get_xy_max_speed_um_s()
            if ms and float(ms) > 0:
                return float(ms)
        except Exception:
            pass
        return None

    def set_xy_top_speed_um_s(self, value_um_s: float) -> None:
        """Declare this machine's true XY top speed (µm/s at 100%) and make it
        authoritative EVERYWHERE, in one call.

        Three consumers, all updated together so they cannot drift:
          1. ``XYStage._max_speed_um_s`` — the mm/s↔SMS-% denominator, so a
             commanded mm/s actually produces that mm/s.
          2. ``safety_limits.max_xy_speed`` — the 100% anchor every jog / print
             speed-% surface reads through :meth:`get_max_xy_speed_um_s`.
          3. the jog handlers' anchors, re-applied with the operator's %.

        The caller persists it (Hardware Setup writes both
        ``safety_limits.max_xy_speed`` and ``device_profile.xy_max_speed_um_s``).
        """
        try:
            v = float(value_um_s)
        except (TypeError, ValueError):
            return
        if v <= 0:
            return
        self._declared_xy_top_speed_um_s = v
        sl = getattr(self, "safety_limits", None)
        if sl is not None:
            try:
                sl.max_xy_speed = v
            except Exception as e:
                logger.debug("set_xy_top_speed_um_s: safety mirror failed: %s", e)
        self._push_xy_top_speed_to_stage()
        try:
            self.refresh_jog_speed_limits()
        except Exception as e:
            logger.debug("refresh_jog_speed_limits (top speed) failed: %s", e)

    def _push_xy_top_speed_to_stage(self) -> None:
        """Seed the connected ``XYStage``'s mm/s↔SMS denominator from the
        declared top speed. No-op when nothing is declared (the stage keeps its
        protocol ``max_speed``) or no stage is connected. Never raises."""
        top = self.declared_xy_top_speed_um_s()
        if not top:
            return
        xy = getattr(self, "xy_stage", None)
        if xy is None or not hasattr(xy, "set_max_speed_um_s"):
            return
        try:
            xy.set_max_speed_um_s(float(top))
        except Exception as e:
            logger.debug("push XY top speed to stage failed: %s", e)

    def get_max_xy_speed_um_s(self) -> float:
        """The single XY top speed (µm/s) every surface reads — the 100% anchor
        for jog %, print speed %, and (v7.21.1) safe-travel speed.

        Precedence: the operator's explicit declaration →
        ``safety_limits.max_xy_speed`` → the timing tool's stored measurement →
        a conservative constant.

        The declaration is first so the Hardware Setup field is the authority
        (:meth:`set_xy_top_speed_um_s` keeps the safety mirror in sync, so in
        practice the two agree and the order only matters for a legacy settings
        file). The stored measurement stays BELOW the safety value, unchanged
        from v7.5.x: editing the safety value must still propagate over a stale
        measurement."""
        declared = _positive_number(self._explicit_xy_top_speed_um_s())
        if declared:
            return declared
        sl = getattr(self, "safety_limits", None)
        if sl is not None and getattr(sl, "max_xy_speed", 0):
            try:
                v = float(sl.max_xy_speed)
                if v > 0:
                    return v
            except (TypeError, ValueError):
                pass
        try:
            from SupportClasses.PrintTimingCalibrationStore import (
                get_store as _get_tc_store,
            )
            ms = _get_tc_store().get_xy_max_speed_um_s()
            if ms and float(ms) > 0:
                return float(ms)
        except Exception:
            pass
        return self._XY_MAX_FALLBACK_UM_S

    def get_max_z_feedrate_mm_min(self) -> float:
        """The single Z max feedrate (mm/min) every surface reads.
        Precedence: ``per_axis_max_feedrate['Z']`` → ``safety_limits.max_z_feedrate``
        → conservative fallback."""
        pa = getattr(self, "_pending_per_axis_max_feedrate", None) or {}
        try:
            if pa.get("Z") and float(pa["Z"]) > 0:
                return float(pa["Z"])
        except (TypeError, ValueError):
            pass
        sl = self.safety_limits
        if sl is not None and getattr(sl, "max_z_feedrate", 0):
            try:
                v = float(sl.max_z_feedrate)
                if v > 0:
                    return v
            except (TypeError, ValueError):
                pass
        return self._Z_MAX_FALLBACK_MM_MIN

    def get_max_pump_feedrate(self) -> float:
        """The single pump-jog 100% anchor every surface reads — the
        needle-derived flow ceiling (µL/s in µL mode) or the legacy mm/s
        fallback. Delegates to :meth:`_pump_jog_max_native`."""
        return self._pump_jog_max_native()

    def get_max_pump_feedrate_for(self, pump: str) -> float:
        """ONE pump's own jog 100% anchor — its needle/syringe-derived safe
        flow ceiling (µL/s in µL mode), so the jog tiles can offer a per-pump
        flow rate. The shared :meth:`get_max_pump_feedrate` anchors to the
        FASTEST configured pump, which over-states a slower pump's ceiling by
        the ratio of their bores. Legacy/no-config → the shared anchor so
        per-pump surfaces degrade to the shared behaviour."""
        hw = self._hardware_config
        sl = self.safety_limits
        if hw is not None and getattr(hw, "configured_pump_ids", None):
            if sl is not None and pump in (hw.configured_pump_ids or []):
                try:
                    r = float(sl.get_max_flow_rate(pump))
                except Exception:
                    r = 0.0
                if r > 0:
                    return r
            return 10.0  # µL/s default when no flow limit is computed yet
        return self._pump_jog_max_native()

    def set_pump_jog_pct(self, pump: str, pct: float) -> None:
        """Store ONE pump's jog-flow % (the jog tiles' per-pump spinboxes).
        Held on the controller — the shared state every jog panel re-reads on
        show — so the tiles can't diverge from each other. Distinct from the
        shared 'p' group % (:meth:`set_jog_speed_pct`), which stays the Xbox
        pump-jog ladder value."""
        try:
            self._pump_jog_pct[str(pump)] = float(pct)
        except (TypeError, ValueError):
            return

    def get_pump_jog_pcts(self) -> dict[str, float]:
        """The stored per-pump jog-flow percentages ({} when none set yet)."""
        return dict(self._pump_jog_pct)

    def set_jog_step_settings(self, cfg: dict) -> None:
        """Store the jog step-slider configuration (snap / per-axis range +
        step value). Held here — like the per-pump jog % — because nine pages
        each build their own jog panel and per-panel state silently
        diverges; every panel re-reads this on show."""
        if isinstance(cfg, dict):
            self._jog_step_settings = dict(cfg)

    def get_jog_step_settings(self) -> dict:
        """The stored jog step-slider configuration ({} when none set)."""
        return dict(getattr(self, "_jog_step_settings", {}) or {})

    # ── v7.5.x: per-pump max plunger feedrate (mm/min) + µL/s readout ──

    def get_pump_max_feedrate_mm_min(self, pump: str = "P1") -> float:
        """This pump's max plunger feedrate (mm/min) — its own override when set,
        else the global ``safety_limits.max_pump_feedrate``."""
        sl = self.safety_limits
        if sl is not None and hasattr(sl, "pump_feedrate_max"):
            try:
                return float(sl.pump_feedrate_max(pump))
            except Exception:
                pass
        if sl is not None:
            try:
                return float(getattr(sl, "max_pump_feedrate", 200.0))
            except (TypeError, ValueError):
                pass
        return 200.0

    def set_pump_max_feedrate_mm_min(self, pump: str, feedrate_mm_min: float) -> None:
        """Set this pump's per-pump max plunger feedrate (mm/min)."""
        sl = self.safety_limits
        if sl is not None and hasattr(sl, "set_pump_feedrate_max"):
            try:
                sl.set_pump_feedrate_max(pump, feedrate_mm_min)
            except Exception as e:
                logger.debug(f"set_pump_max_feedrate_mm_min({pump}) failed: {e}")

    def pump_feedrate_mm_min_to_uL_s(self, pump: str, mm_min: float) -> float | None:
        """Convert a plunger feedrate (mm/min) to a volumetric rate (µL/s) via
        this pump's configured syringe. None when the pump has no syringe."""
        try:
            return self._pump_mm_to_uL(pump, float(mm_min) / 60.0)
        except (TypeError, ValueError):
            return None

    def pump_max_feedrate_uL_s(self, pump: str = "P1") -> float | None:
        """This pump's max plunger feedrate expressed as a volumetric rate (µL/s)
        via its configured syringe. None when the pump has no syringe."""
        return self.pump_feedrate_mm_min_to_uL_s(
            pump, self.get_pump_max_feedrate_mm_min(pump))

    def notify_speed_limits_changed(self) -> None:
        """Re-apply the per-axis anchors to the jog handlers and fan the change
        out to the GUI (via ``on_speed_limits_changed``) so every page re-reads
        the common source. For backend (non-page) emitters such as the timing
        tool's measurement worker — page-driven edits use the GUI's
        ``safety_limits_changed`` signal directly."""
        try:
            self.refresh_jog_speed_limits()
        except Exception as e:
            logger.debug("refresh_jog_speed_limits during notify failed: %s", e)
        cb = getattr(self, "on_speed_limits_changed", None)
        if cb:
            try:
                cb()
            except Exception as e:
                logger.debug("on_speed_limits_changed callback failed: %s", e)

    def refresh_jog_speed_limits(self) -> None:
        """Push the per-axis calibrated max move speed (100% anchor) into the
        Xbox jog handlers and re-apply the chosen %. Anchors come from the
        common resolvers (:meth:`get_max_xy_speed_um_s` /
        :meth:`get_max_z_feedrate_mm_min` / :meth:`get_max_pump_feedrate`).
        Safe to call any time (no-op for handlers that aren't connected yet)."""
        if self.xy_jog is not None:
            xy_max = self.get_max_xy_speed_um_s()
            if xy_max and xy_max > 0:
                self.xy_jog.set_speed_max(float(xy_max))
        if self.zp_jog is not None:
            z_feed_mm_min = self.get_max_z_feedrate_mm_min()
            if z_feed_mm_min and float(z_feed_mm_min) > 0:
                self.zp_jog.set_z_speed_max(float(z_feed_mm_min) / 60.0)
            p_max = self.get_max_pump_feedrate()
            if p_max and p_max > 0:
                self.zp_jog.set_p_speed_max(p_max)
        # Re-apply any restored/operator-chosen % against the new anchors.
        self._apply_jog_speed_pct()

    def _apply_jog_speed_pct(self) -> None:
        """Apply the stored per-group % to whichever jog handlers exist."""
        p = self._jog_speed_pct
        if self.xy_jog is not None and "xy" in p:
            self.xy_jog.set_speed_pct(p["xy"])
        if self.zp_jog is not None:
            if "z" in p:
                self.zp_jog.set_z_speed_pct(p["z"])
            if "p" in p:
                self.zp_jog.set_p_speed_pct(p["p"])

    def set_jog_speed_pct(self, group: str, pct: float) -> None:
        """Set a group's jog speed % (group ∈ {'xy','z','p'/'pump'}). Stored on
        the controller so it survives (re)connect, then applied to live handlers."""
        g = str(group).lower()
        if g == "pump":
            g = "p"
        if g not in ("xy", "z", "p"):
            return
        try:
            self._jog_speed_pct[g] = float(pct)
        except (TypeError, ValueError):
            return
        self._apply_jog_speed_pct()

    def get_jog_speed_state(self) -> dict:
        """Read-out for the GUI: {group: {pct, speed, unit}} for present groups.
        Falls back to the stored % (no resolved speed) when a handler isn't
        connected so the page can still show the selected value."""
        out: dict[str, dict] = {}
        if self.xy_jog is not None:
            out["xy"] = {"pct": self.xy_jog.speed_pct,
                         "speed": self.xy_jog.xy_speed, "unit": "µm/s"}
        elif "xy" in self._jog_speed_pct:
            out["xy"] = {"pct": self._jog_speed_pct["xy"],
                         "speed": None, "unit": "µm/s"}
        if self.zp_jog is not None:
            out["z"] = {"pct": self.zp_jog.z_speed_pct,
                        "speed": self.zp_jog.z_speed, "unit": "mm/s"}
            p_unit = "µL/s" if self.zp_jog.p_speed_is_uL else "mm/s"
            out["p"] = {"pct": self.zp_jog.p_speed_pct,
                        "speed": self.zp_jog.p_speed, "unit": p_unit}
        else:
            if "z" in self._jog_speed_pct:
                out["z"] = {"pct": self._jog_speed_pct["z"],
                            "speed": None, "unit": "mm/s"}
            if "p" in self._jog_speed_pct:
                out["p"] = {"pct": self._jog_speed_pct["p"],
                            "speed": None, "unit": "µL/s"}
        return out

    def calibrate_xbox_sticks(self, duration: float = 2.0) -> dict:
        """Run stick center calibration. Returns offsets dict.

        v7.3.2: Should be called with sticks untouched. Returns
        ``{0: offset, 1: offset, 2: offset, 3: offset}``.
        """
        return calibrate_sticks(duration=duration)

    @property  # v7.2.8: xbox_status property
    def xbox_status(self) -> str:
        """Return Xbox connection status string.

        Returns: 'disconnected', 'waiting', 'connected', or 'alive'.
        'waiting'   = worker running, searching for controller
        'connected' = controller just found
        'alive'     = controller confirmed active (heartbeat)
        """
        if self.xbox_poller is None:
            return "disconnected"
        return getattr(self.xbox_poller, "_xbox_status", "unknown")

    @property
    def is_xbox_connected(self) -> bool:
        """Backward-compatible bool: True when controller is active."""
        # v7.2.7: is_xbox_connected thread
        status = self.xbox_status
        if status in ("connected", "alive"):
            return True
        # Fallback: check if process or thread is alive
        if self.xbox_process and self.xbox_process.is_alive():
            return False  # Process alive but controller not found yet
        _xt = getattr(self, "_xbox_thread", None)
        if _xt and _xt.is_alive():
            return False  # Thread alive but controller not found yet
        return False


    def get_xy_position(self, cached: bool = True) -> tuple:
        """Get XY position. cached=True returns polled value (non-blocking).

        v7.6: a successful DIRECT read also back-fills the poller cache
        (:meth:`PositionPoller.note_xy`), so every cached reader — the status
        bar, the Quick Print trajectory monitor, the context panels — stays
        live even while the poller is suspended (which the print path does for
        the whole ``PRINT_PATH`` command while reading position itself at
        25–31 Hz). Odometer/liveness accounting is untouched.
        """
        if cached:
            return self._pos_poller.xy_position
        if self.xy_stage:
            try:
                pos = self.xy_stage.get_current_position()
                if pos and pos[0] is not None:
                    try:
                        self._pos_poller.note_xy(pos)
                    except Exception:
                        pass                    # never break a read on a stub
                return pos
            except Exception as e:
                logger.debug(f"XY direct query error: {e}")
        return (None, None, None)

    def get_xy_position_mm(self, cached: bool = True) -> tuple:
        """Get XY position in mm (relative to zero reference).

        v7.3: Convenience method for callers that need mm.
        Returns (x_mm, y_mm) or (None, None) if not connected.
        """
        pos = self.get_xy_position(cached)
        if pos[0] is not None:
            zx = self.zero_position.get("x", 0)
            zy = self.zero_position.get("y", 0)
            return ((pos[0] - zx) / 1000.0, (pos[1] - zy) / 1000.0)
        return (None, None)

    def default_plate_center_um(self) -> tuple[float, float]:
        """Absolute stage-µm centre of the XY safety envelope.

        The default (uncalibrated) well plate is centred here so it always
        sits in the middle of the configured travel envelope, regardless of
        whether the envelope is symmetric.

        v7.5.x: the safety-limit bounds are now **absolute** stage µm, so the
        envelope midpoint (``safety_limits.xy_center()``) is already in the
        absolute frame that ``WellPlate.get_all_positions_from_plate_center``
        expects — no ``zero_position`` offset is added (adding it would place
        the plate outside the physical box). For the symmetric default
        envelope (±130000 / ±85000) the midpoint is (0, 0).
        """
        return self.safety_limits.xy_center()

    def wait_for_xy_arrival(
        self, target_x_mm: float, target_y_mm: float,
        tolerance_mm: float = 0.1, timeout_s: float = 10.0,
    ) -> bool:
        """Block until XY stage reaches target position (zero-ref mm).

        v7.2.9: Safety method — call before Z descent to confirm XY is at
        the correct well position, preventing needle breakage.

        Returns True if position reached within tolerance, False on timeout.
        """
        import time
        import math

        if not self.is_xy_connected:
            return True  # No stage to wait for

        deadline = time.monotonic() + timeout_s
        poll_interval = 0.15  # 150ms between polls

        while time.monotonic() < deadline:
            pos = self.get_xy_position_mm(cached=False)
            if pos[0] is not None and pos[1] is not None:
                dx = pos[0] - target_x_mm
                dy = pos[1] - target_y_mm
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= tolerance_mm:
                    return True
            time.sleep(poll_interval)

        # Timeout — log warning but don't block forever
        pos = self.get_xy_position_mm(cached=False)
        logger.warning(
            f"wait_for_xy_arrival timeout ({timeout_s}s): "
            f"target=({target_x_mm:.2f}, {target_y_mm:.2f}), "
            f"actual={pos}")
        return False

    def wait_for_z_arrival(
        self, target_z_mm: float,
        tolerance_mm: float = 0.05, timeout_s: float = 10.0,
        abort_event=None,
    ) -> bool:
        """Block until Z axis reaches target position (zero-ref mm).

        v7.2.9: Companion to wait_for_xy_arrival for hybrid execution.
        Returns True if position reached within tolerance, False on timeout.

        v7.6 ``abort_event`` (optional): when set, return False within one
        poll interval instead of waiting out the timeout (callers distinguish
        "aborted" from "timed out" via ``abort_event.is_set()``). Default
        None = byte-identical.
        """
        import time
        import math

        if not self.is_zp_connected:
            return True

        deadline = time.monotonic() + timeout_s
        poll_interval = 0.15

        # v7.5.x bugfix: poll Z via axis_map slot, not pos[0] (which is a
        # pump under a non-default map). A wrong-axis read here could
        # falsely confirm Z arrival and let XY move / Z descend early.
        zi = _axis_index(self.zp_stage, "Z")

        # v7.5.x: fast-fail on a dead board. ``get_current_position`` returns
        # the last *stale* floats when the board stops answering M114, so a
        # genuine ZP drop used to make this loop wait out the full timeout
        # comparing a frozen position to the target (and freeze any GUI-thread
        # caller for that whole window). Track consecutive read failures via
        # the explicit ``_last_position_read_ok`` flag (the same signal the
        # poller-liveness watchdog uses) and bail early once the board has
        # clearly gone silent. Debounced so a single transient miss under a
        # busy write stream is ridden out, not treated as a disconnect.
        read_fail_streak = 0
        read_fail_limit = 5  # ~0.75 s of consecutive silence at poll_interval
        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                logger.info("wait_for_z_arrival: abort_event set — returning")
                return False
            pos = self.get_zp_position(cached=False)
            if getattr(self.zp_stage, "_last_position_read_ok", True):
                read_fail_streak = 0
                if zi is not None and zi < len(pos) and pos[zi] is not None:
                    z_mm = (pos[zi] - self.zero_position.get("Z", 0))
                    if abs(z_mm - target_z_mm) <= tolerance_mm:
                        return True
            else:
                read_fail_streak += 1
                if read_fail_streak >= read_fail_limit:
                    logger.warning(
                        f"wait_for_z_arrival: ZP stopped answering M114 "
                        f"({read_fail_streak} consecutive failures) — "
                        f"treating Z move as unconfirmed (target={target_z_mm:.2f})")
                    return False
            time.sleep(poll_interval)

        pos = self.get_zp_position(cached=False)
        logger.warning(
            f"wait_for_z_arrival timeout ({timeout_s}s): "
            f"target={target_z_mm:.2f}, actual={pos}")
        return False

    def get_zp_position(self, cached: bool = True) -> tuple:
        """Get ZP position. cached=True returns polled value (non-blocking).

        Returns a 4-tuple in *physical* Marlin axis order: ``(X, Y, Z, E)``.
        Consumers wanting logical (Z, P1, P2, P3) values should use
        :meth:`get_zp_position_logical` or :meth:`zp_logical_value`,
        which respect the live ``axis_map`` and stay correct under
        non-default per-machine wiring.
        """
        if cached:
            return self._pos_poller.zp_position
        if self.zp_stage:
            try:
                return self.zp_stage.get_current_position()
            except Exception as e:
                logger.debug(f"ZP direct query error: {e}")
        return (None, None, None, None)

    def zp_logical_value(self, pos: tuple, logical: str) -> float | None:
        """v7.4.2 hotfix: pluck a logical-axis value from a ZP tuple.

        ``pos`` is the 4-tuple from :meth:`get_zp_position` in physical
        Marlin order. ``logical`` is one of ``Z, P1, P2, P3``. Returns
        ``None`` if the logical axis is unmapped or the slot is empty.
        """
        idx = _axis_index(self.zp_stage, logical)
        if idx is None or pos is None or idx >= len(pos):
            return None
        return pos[idx]

    def get_zp_position_logical(self, cached: bool = True) -> dict:
        """v7.4.2 hotfix: return ZP position keyed by logical axis.

        Convenience for GUI / executors that don't want to deal with
        the physical tuple. Returns ``{"Z": val, "P1": val, "P2": val,
        "P3": val}`` with ``None`` for unmapped or unread slots.
        """
        pos = self.get_zp_position(cached=cached)
        return {logical: self.zp_logical_value(pos, logical)
                for logical in ("Z", "P1", "P2", "P3")}

    def get_zp_position_logical_tuple(self, cached: bool = True) -> tuple:
        """v7.5.x: ZP position as a logical-ordered tuple ``(Z, P1, P2, P3)``.

        Convenience for :meth:`PositionLogger.record`, whose ``zp_pos``
        contract is logical order. Resolving each slot through the live
        ``axis_map`` keeps the diagnostic position log correctly labeled
        under non-default per-machine wiring (where the raw
        :meth:`get_zp_position` tuple is physical ``(X, Y, Z, E)`` order).
        """
        pos = self.get_zp_position(cached=cached)
        return tuple(self.zp_logical_value(pos, logical)
                     for logical in ("Z", "P1", "P2", "P3"))

    def get_zp_position_zero_ref(self, cached: bool = True) -> dict:
        """v7.5.x: ZP position keyed by logical axis, **zero-referenced** (mm).

        Mirrors the readout frame everywhere in the app:
        ``displayed = raw − zero_position[axis]``. This is the frame the
        manual override (:meth:`override_zp_position`) and the saved
        last-known-position snapshot use, so a value read here can be fed
        straight back to ``override_zp_position`` to reproduce it.

        Returns ``{"Z":.., "P1":.., "P2":.., "P3":..}`` with ``None`` for
        unmapped or unread slots.
        """
        raw = self.get_zp_position_logical(cached=cached)
        out: dict[str, float | None] = {}
        for ax in ("Z", "P1", "P2", "P3"):
            v = raw.get(ax)
            out[ax] = None if v is None else (
                float(v) - float(self.zero_position.get(ax, 0.0)))
        return out

    # ── v7.5.x: display-only motion interpolation ───────────────────────
    #
    # These mirror get_xy_position/get_zp_position(cached=True) but overlay the
    # MotionEstimator's prediction while a jog/travel move is in flight, so the
    # GUI readout/needle ANIMATE toward the destination instead of snapping.
    # DISPLAY USE ONLY — every motion/clamping/safety/print consumer keeps
    # reading the raw poller cache via get_*_position. On any error these fall
    # back to the exact raw cache, so they are a safe drop-in for display sites.

    def get_display_xy_position(self) -> tuple:
        """XY position for DISPLAY (absolute stage µm): interpolated estimate
        while a move is in flight, else the raw poller cache."""
        cache = self._pos_poller.xy_position
        try:
            est = self._motion.estimate("XY", cache)
        except Exception:
            est = None
        if est is None:
            return cache
        z = cache[2] if cache is not None and len(cache) > 2 else None
        return (est[0], est[1], z)

    def get_display_zp_position(self) -> tuple:
        """ZP position for DISPLAY (physical Marlin tuple, raw mm): each logical
        axis (Z/P1/P2/P3) with a live estimate overrides its physical slot, else
        the raw poller cache."""
        cache = self._pos_poller.zp_position
        try:
            out = list(cache)
            for ch in ("Z", "P1", "P2", "P3"):
                idx = _axis_index(self.zp_stage, ch)
                if idx is None or idx >= len(out):
                    continue
                est = self._motion.estimate(ch, out[idx])
                if est is not None:
                    out[idx] = est
            return tuple(out)
        except Exception:
            return cache

    def motion_estimate_active(self) -> bool:
        """True while any axis is showing a live (display-only) motion estimate.
        Drives the GUI's fast animation tick + the 'estimated' readout cue."""
        try:
            return self._motion.is_active()
        except Exception:
            return False

    def motion_estimating(self) -> set:
        """Per-axis display tokens currently showing an estimate
        (``{"X","Y","Z","P1","P2","P3"}`` subset) — ``"XY"`` maps to X and Y."""
        try:
            return MotionEstimator.display_axes(self._motion.active_channels())
        except Exception:
            return set()

    # ── motion-estimate registration hooks (display only) ───────────────
    #
    # Called at the bottom of the move primitives (after the hardware send) and
    # from the Xbox jog loops. Gated to the NON-suspended poller so programmatic
    # sequences that suspend it (safe_travel_to, PRINT_PATH) register nothing —
    # the estimator stays scoped to user-initiated fire-and-forget jog/travel.
    # Always guarded: a prediction can NEVER perturb the move it follows.

    def _motion_estimates_live(self) -> bool:
        p = getattr(self, "_pos_poller", None)
        m = getattr(self, "_motion", None)
        return bool(m is not None and p is not None
                    and not getattr(p, "_suspended", False))

    def _note_move_estimate_xy(self, dest_x_um: float, dest_y_um: float) -> None:
        """Register an XY target estimate from the current cache to (dest µm).
        All cache access is behind the live-gate, so this is safe to call from a
        partially-constructed controller (the gate returns False)."""
        try:
            if not self._motion_estimates_live():
                return
            cache = self._pos_poller.xy_position
            if not cache or cache[0] is None:
                return
            speed = self.get_max_xy_speed_um_s()
            self._motion.note_target(
                "XY", (cache[0], cache[1]), (dest_x_um, dest_y_um), speed)
        except Exception:
            pass

    def _note_move_estimate_xy_rel(self, dx_um: float, dy_um: float) -> None:
        """Register an XY target estimate for a RELATIVE move (cache + delta)."""
        try:
            if not self._motion_estimates_live():
                return
            cache = self._pos_poller.xy_position
            if not cache or cache[0] is None:
                return
            self._note_move_estimate_xy(cache[0] + dx_um, cache[1] + dy_um)
        except Exception:
            pass

    def _note_xy_travel_um(self, dist_um: float) -> None:
        """Forward a poll-sample XY travel chord (µm) to the calibration-status
        odometer. Best-effort — never raise into the poll thread."""
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            get_store().add_xy_travel_um(dist_um)
        except Exception:
            pass

    def _note_move_estimate_axis_rel(self, channel: str, delta_raw_mm: float,
                                     feedrate_mm_min: float | None) -> None:
        """Register a Z/pump target estimate for a RELATIVE move (cache+delta)."""
        try:
            if not self._motion_estimates_live():
                return
            idx = _axis_index(self.zp_stage, channel)
            cache = self._pos_poller.zp_position
            if (idx is None or not cache or idx >= len(cache)
                    or cache[idx] is None):
                return
            self._note_move_estimate_axis(
                channel, cache[idx] + delta_raw_mm, feedrate_mm_min)
        except Exception:
            pass

    def _note_move_estimate_axis(self, channel: str, dest_raw_mm: float,
                                 feedrate_mm_min: float | None) -> None:
        """Register a Z/pump target estimate (raw mm) from the cache to dest."""
        try:
            if not self._motion_estimates_live():
                return
            idx = _axis_index(self.zp_stage, channel)
            cache = self._pos_poller.zp_position
            if (idx is None or not cache or idx >= len(cache)
                    or cache[idx] is None):
                return
            speed_mm_s = ((float(feedrate_mm_min) / 60.0)
                          if feedrate_mm_min else 0.0)
            if speed_mm_s <= 0:
                if channel == "Z":
                    speed_mm_s = self.get_max_z_feedrate_mm_min() / 60.0
                else:
                    # No feedrate on this pump move (small jog) — skip rather
                    # than guess (pump speed anchors are µL/s, not mm/min).
                    return
            self._motion.note_target(channel, cache[idx], dest_raw_mm, speed_mm_s)
        except Exception:
            pass

    def _note_xy_jog_segment(self, dx_um: float, dy_um: float) -> None:
        """Xbox XY jog: accumulate one commanded segment delta (µm)."""
        try:
            if self._motion_estimates_live():
                self._motion.advance("XY", (dx_um, dy_um))
        except Exception:
            pass

    def _note_zp_jog_segment(self, deltas: dict) -> None:
        """Xbox ZP jog: accumulate one commanded segment per logical axis
        (``{"Z": dz, "P1": dp1, ...}`` raw-mm deltas)."""
        try:
            if not self._motion_estimates_live():
                return
            for ch, d in deltas.items():
                if d and abs(float(d)) > 0.0:
                    self._motion.advance(ch, float(d))
        except Exception:
            pass

    def get_speed_info(self) -> dict:
        """Return current jog speeds as numeric values.

        v7.2.8: Reads .xy_speed/.z_speed/.p_speed directly
        to avoid broken @property decorators.
        """
        return {
            "xy": getattr(self.xy_jog, "xy_speed", 0) if self.xy_jog else 0,
            "z": getattr(self.zp_jog, "z_speed", 0) if self.zp_jog else 0,
            "p": getattr(self.zp_jog, "p_speed", 0) if self.zp_jog else 0,
        }
    @property
    def is_xy_connected(self) -> bool:
        return self.xy_stage is not None

    @property
    def is_zp_connected(self) -> bool:
        # v7.5.x ZP reconnect hotfix: a ZPStageManager can exist with a
        # dead/None serial after a failed (re)connect. Report connected
        # only when the serial backend is actually live so the badge never
        # lies. Simulators always have a live backend → short-circuit.
        if self.zp_stage is None:
            return False
        if getattr(self.zp_stage, "simulate", False):
            return True
        return getattr(self.zp_stage, "serial", None) is not None

    def set_led_brightness(self, level: int) -> bool:
        """Set the illumination LED brightness (0-255) on the ZP board.

        Guarded on the ZP connection: a no-op returning False when the board
        is not connected. Forwards to ``ZPStageManager.set_led_brightness``
        (fan-output PWM via M106). See CLAUDE.md — the LED is not a motion
        axis; this is the only hardware entry point for illumination.
        """
        if self.zp_stage and self.is_zp_connected:
            return self.zp_stage.set_led_brightness(level)
        return False

    def led_off(self) -> bool:
        """Best-effort: turn the illumination LED off. Never raises.

        Used on shutdown (see :meth:`shutdown`). The LED is a command output
        with no readback, so nothing turns it off on its own — if the app exits
        with it lit, it STAYS lit with nothing left to control it.
        """
        try:
            return self.set_led_brightness(0)
        except Exception as exc:  # a dying board must not break the exit path
            logger.debug("LED off failed: %s", exc)
            return False

    @property
    def simulate_xy(self) -> bool:
        """v7.4.2: True iff the currently-connected XY stage is a simulator.

        Replaces the previous static instance flag — simulation is now
        decided per-connection from the UI ("Simulate" button vs.
        "Connect"). When no stage is connected, falls back to the
        constructor default (controlled by ``--simulate-xy`` in headless
        mode).
        """
        if self.xy_stage is not None:
            return bool(getattr(self.xy_stage, "simulate", False))
        return self._default_simulate_xy

    @property
    def simulate_zp(self) -> bool:
        """v7.4.2: True iff the currently-connected ZP stage is a simulator.

        See :attr:`simulate_xy` for the rationale.
        """
        if self.zp_stage is not None:
            return bool(getattr(self.zp_stage, "simulate", False))
        return self._default_simulate_zp

    # ── Calibration ───────────────────────────────────────────────

    def _calibrate_zero(self, *args, **kwargs) -> None:
        """Set current position as zero reference."""
        if self.xy_stage:
            pos = self.xy_stage.get_current_position()
            if pos[0] is not None:
                self.zero_position["x"] = pos[0]
                self.zero_position["y"] = pos[1]
                self.zero_position["f"] = pos[2] if pos[2] else 0.0

        if self.zp_stage:
            pos = self.zp_stage.get_current_position()
            if pos[0] is not None:
                # v7.5.x bugfix: map each logical axis to its physical
                # tuple slot via axis_map. Hardcoding Z=pos[0]/P1=pos[1]/…
                # recorded the wrong zero on non-default maps (ME3B V1).
                for logical in ("Z", "P1", "P2", "P3"):
                    i = _axis_index(self.zp_stage, logical)
                    if i is not None and i < len(pos) and pos[i] is not None:
                        self.zero_position[logical] = pos[i]

        logger.info(f"Zero position calibrated: {self.zero_position}")

        self.position_logger.record(
            "calibrate_zero",
            # v7.5.x: cached reads — this handler runs on the Processor
            # dispatch thread (Xbox button "zero_needle_pos"); the two
            # fresh serial reads above are needed for accuracy, but
            # re-querying both stages again just for the log stalled every
            # queued velocity command behind ~2 extra serial round-trips
            # (head-of-line blocking felt as freeze-then-replay).
            xy_pos=self.get_xy_position(cached=True),
            # v7.5.x: log in logical (Z,P1,P2,P3) order — PositionLogger's
            # contract — so the CSV stays correct under non-default axis maps.
            zp_pos=self.get_zp_position_logical_tuple(cached=True),
            metadata={"zero_position": dict(self.zero_position)},
        )

    # ── v7.3.2: Axis Flip ──────────────────────────────────────────

    def set_axis_flip(self, axis: str, flipped: bool) -> None:
        """Set direction flip for an axis. Flipped axes invert move direction."""
        if axis in self._axis_flip:
            self._axis_flip[axis] = flipped
            logger.info(f"Axis {axis} flip set to {flipped}")

    def get_axis_flip(self, axis: str) -> bool:
        """Get whether an axis direction is flipped."""
        return self._axis_flip.get(axis, False)

    def set_axis_flips(self, flips: dict[str, bool]) -> None:
        """Bulk-set axis flips from a dict (e.g. loaded from settings)."""
        for axis, flipped in flips.items():
            if axis in self._axis_flip:
                self._axis_flip[axis] = flipped
        logger.info(f"Axis flips loaded: {self._axis_flip}")

    def _flip_sign(self, axis: str) -> float:
        """Return -1.0 if axis is flipped, 1.0 otherwise."""
        return -1.0 if self._axis_flip.get(axis, False) else 1.0

    # ── v7.3.2: Per-Axis Zero Calibration ────────────────────────

    def calibrate_zero_xy(self) -> None:
        """Set only XY zero from current position."""
        if not self.xy_stage:
            logger.warning("Cannot zero XY — stage not connected")
            return
        pos = self.xy_stage.get_current_position()
        if pos[0] is not None:
            self.zero_position["x"] = pos[0]
            self.zero_position["y"] = pos[1]
            self.zero_position["f"] = pos[2] if pos[2] else 0.0
            logger.info(f"XY zero set to ({pos[0]:.1f}, {pos[1]:.1f}) µm")
            self.position_logger.record(
                "zero_xy",
                xy_pos=pos,
                metadata={"zero_x": pos[0], "zero_y": pos[1]},
            )

    # ── Movement (for GUI / Print commands) ───────────────────────

    def reset_pump_zero(self, pump: str) -> None:
        """
        v7.2.6: Reset zero reference for a single pump axis.

        Sets the current absolute position as the new zero point
        for the specified pump, without affecting other axes.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
        """
        if pump not in ("P1", "P2", "P3"):
            logger.warning(f"Invalid pump ID for zero reset: {pump}")
            return

        pos = self.get_zp_position(cached=True)
        if pos[0] is None:
            logger.warning(f"Cannot reset {pump} zero — ZP stage not connected")
            return

        # v7.4.2 hotfix: route through axis_map so we read the right
        # tuple slot under non-default mappings (e.g. P1→X under
        # Conservative profile).
        idx = _axis_index(self.zp_stage, pump)
        if idx is not None and idx < len(pos) and pos[idx] is not None:
            self.zero_position[pump] = pos[idx]
            logger.info(f"{pump} zero set to {pos[idx]:.3f} mm (absolute)")

            # Log the event
            self.position_logger.record(
                f"zero_reset_{pump.lower()}",
                # v7.5.x: log in logical (Z,P1,P2,P3) order (see helper).
                zp_pos=self.get_zp_position_logical_tuple(cached=True),
                metadata={"pump": pump, "new_zero": pos[idx]},
            )

    def zero_axis(self, axis: str) -> dict:
        """v7.4.2 hotfix: zero the specified logical axis on hardware.

        - For ``"X"`` / ``"Y"``: invoke ProScan ``set_home``, which
          zeros the controller's internal position counter for that
          axis (the protocol's ``Z`` command). Updates
          ``self.zero_position["x"]`` / ``["y"]`` to 0.
        - For ``"Z"`` / ``"P1"`` / ``"P2"`` / ``"P3"``: send Marlin
          ``G92 <physical>0`` (the physical axis comes from
          ``zp_stage.axis_map``). Marlin now reports the current
          physical position as 0. Updates ``self.zero_position[axis]``
          to 0.

        Returns a dict like ``{"ok": True, "axis": "Z", "previous_raw": 3.45}``
        so the caller can show the user what was zeroed. ``previous_raw``
        is whatever the controller reported just before the zero.
        """
        previous_raw: float | None = None
        if axis in ("X", "Y"):
            if self.xy_stage is None:
                return {"ok": False, "axis": axis, "error": "XY stage not connected"}
            try:
                pos = self.xy_stage.get_current_position()
                previous_raw = float(pos[0]) if axis == "X" and pos[0] is not None \
                    else (float(pos[1]) if axis == "Y" and pos[1] is not None else None)
                # ProScan set_home is whole-stage (XY); we still call it
                # because there's no per-axis equivalent for ProScan II.
                if hasattr(self.xy_stage, "set_home"):
                    self.xy_stage.set_home()
                key = axis.lower()
                self.zero_position[key] = 0.0
                # ProScan's set_home zeros BOTH X and Y; keep the dict consistent.
                self.zero_position["x"] = 0.0
                self.zero_position["y"] = 0.0
                logger.info(f"XY hardware zeroed (axis {axis} requested); previous raw {axis} = {previous_raw}")
            except Exception as e:
                logger.warning(f"XY zero failed: {e}")
                return {"ok": False, "axis": axis, "error": str(e)}
            return {"ok": True, "axis": axis, "previous_raw": previous_raw}

        if axis in ("Z", "P1", "P2", "P3"):
            if self.zp_stage is None:
                return {"ok": False, "axis": axis, "error": "ZP stage not connected"}
            try:
                pos = self.zp_stage.get_current_position()
                # Marlin axis index lookup via axis_map → physical letter → index
                idx_lookup = {"X": 0, "Y": 1, "Z": 2, "E": 3}
                physical = self.zp_stage.axis_map.get(axis)
                idx = idx_lookup.get(physical) if physical else None
                if idx is not None and idx < len(pos) and pos[idx] is not None:
                    previous_raw = float(pos[idx])
                ok = self.zp_stage.set_zero(axis)
                if not ok:
                    return {"ok": False, "axis": axis,
                            "error": f"axis_map has no entry for {axis}"}
                self.zero_position[axis] = 0.0
                logger.info(f"ZP {axis} zeroed via G92; previous raw = {previous_raw}")
            except Exception as e:
                logger.warning(f"ZP zero failed: {e}")
                return {"ok": False, "axis": axis, "error": str(e)}
            return {"ok": True, "axis": axis, "previous_raw": previous_raw}

        return {"ok": False, "axis": axis, "error": f"unknown axis: {axis}"}

    def reset_z_zero(self) -> None:
        """
        v7.2.6: Reset zero reference for Z axis only.

        Sets the current absolute position as the new zero point
        for Z, without affecting XY or pump axes.
        """
        pos = self.get_zp_position(cached=True)
        # v7.5.x bugfix: resolve Z's slot via axis_map (was pos[0], which
        # is a pump under a non-default map). Mirrors reset_pump_zero.
        zi = _axis_index(self.zp_stage, "Z")
        if zi is None or pos is None or zi >= len(pos) or pos[zi] is None:
            logger.warning("Cannot reset Z zero — ZP stage not connected")
            return

        self.zero_position["Z"] = pos[zi]
        logger.info(f"Z zero set to {pos[zi]:.3f} mm (absolute)")

        self.position_logger.record(
            "zero_reset_z",
            # v7.5.x: log in logical (Z,P1,P2,P3) order (see helper).
            zp_pos=self.get_zp_position_logical_tuple(cached=True),
            metadata={"new_zero": pos[zi]},
        )

    def override_zp_position(self, axis: str, value: float) -> dict:
        """v7.5.x: manually re-sync a ZP axis's firmware position counter.

        The operator declares the axis is physically at ``value`` — in
        the same zero-referenced mm frame as the position readout and the
        safety limits. We send a Marlin ``G92`` setting the *physical*
        counter to ``value + zero_position[axis]`` so the zero-referenced
        readout becomes ``value`` while the established zero reference is
        preserved (unlike :meth:`zero_axis`, which forces both the
        counter and the reference to 0).

        Needed because Marlin has no absolute encoder: after a board
        power cycle it reports 0 (or a stale value) and a position
        *refresh* simply re-reads that wrong value. Only the operator
        knows where the axis actually is — this is the manual recovery
        path for that case. No motion occurs (G92 only rebases the
        counter).

        Returns ``{"ok": True, "axis": str, "raw": float,
        "previous_raw": float|None}`` on success, or
        ``{"ok": False, "axis": str, "error": str}`` on failure.
        """
        if axis not in ("Z", "P1", "P2", "P3"):
            return {"ok": False, "axis": axis,
                    "error": f"unsupported axis: {axis} (ZP axes only)"}
        if self.zp_stage is None:
            return {"ok": False, "axis": axis,
                    "error": "ZP stage not connected"}
        try:
            prev = self.get_zp_position(cached=False)
            previous_raw = self.zp_logical_value(prev, axis)
            zero = float(self.zero_position.get(axis, 0.0))
            raw = float(value) + zero
            ok = self.zp_stage.set_position(axis, raw)
            if not ok:
                return {"ok": False, "axis": axis,
                        "error": f"axis_map has no entry for {axis}"}
            logger.info(
                f"ZP {axis} position overridden to {value:.4f} "
                f"(raw G92 {raw:.4f}, zero {zero:.4f}); "
                f"previous raw = {previous_raw}")
            self.position_logger.record(
                f"override_position_{axis.lower()}",
                # v7.5.x: log in logical (Z,P1,P2,P3) order (see helper).
                zp_pos=self.get_zp_position_logical_tuple(cached=False),
                metadata={"axis": axis, "value": value, "raw": raw,
                          "previous_raw": previous_raw},
            )
            return {"ok": True, "axis": axis, "raw": raw,
                    "previous_raw": previous_raw}
        except Exception as e:
            logger.warning(f"ZP {axis} position override failed: {e}")
            return {"ok": False, "axis": axis, "error": str(e)}

    def move_xy_absolute(
        self, x: float, y: float, from_zero_ref: bool = True, fast: bool = False
    ) -> None:
        """Move XY to absolute position, optionally relative to zero ref."""
        """v7.3: Accept mm, convert to µm internally.

        When from_zero_ref=True, x/y are in mm (from WellPlate/trajectory).
        Converts to µm and adds the zero reference to get the absolute stage
        target, applies safety limits (the XY envelope is absolute stage µm),
        then sends to stage which expects µm (Prior manual page 36).

        v7.5.x: the XY envelope is now absolute, so clamp the absolute target
        (after adding zero) rather than the zero-referenced value.
        """
        if not self.xy_stage:
            return

        if from_zero_ref:
            # Convert mm → µm, then add zero reference (which is in µm) to
            # reach the absolute stage frame the envelope is defined in.
            x_um = x * 1000.0 + self.zero_position["x"]
            y_um = y * 1000.0 + self.zero_position["y"]
        else:
            # Legacy: raw values passed directly (assumed absolute µm already)
            x_um = x
            y_um = y

        if self.safety_limits.enabled:
            x_um, y_um = self.safety_limits.clamp_xy(x_um, y_um)

        self.xy_stage.move_stage_to_position(x_um, y_um, fast)
        self._note_move_estimate_xy(x_um, y_um)  # display-only animation

    def move_xy_absolute_um(self, x_um: float, y_um: float,
                            fast: bool = False) -> None:
        """Move XY to an ABSOLUTE stage position given in µm.

        Clearly-named entry point for callers that already work in absolute
        stage µm (e.g. the Calibration page's predicted/observed well centres,
        which are ``zero_position + offset``). Equivalent to
        ``move_xy_absolute(x, y, from_zero_ref=False)`` but with safety limits
        still honored.

        v7.5.x: the XY envelope is now absolute stage µm, so the absolute
        target is clamped directly (no zero-ref round-trip).
        """
        if not self.xy_stage:
            return
        if self.safety_limits.enabled:
            x_um, y_um = self.safety_limits.clamp_xy(x_um, y_um)
        self.xy_stage.move_stage_to_position(x_um, y_um, fast)
        self._note_move_estimate_xy(x_um, y_um)  # display-only animation

    def move_xy_relative(self, dx: float, dy: float) -> None:
        """
        Move XY stage by a relative offset in µm.

        BUG-1 FIX: Sends relative moves directly to hardware, eliminating
        dependency on stale cached position data.

        Safety limits are projected from cached position (minor boundary
        imprecision is acceptable vs. the gross errors from the old approach).

        v7.5.x: the XY envelope is absolute stage µm, so clamp the absolute
        destination (cached + delta) directly — no zero-ref round-trip. This
        keeps small moves valid after a Set Zero re-anchors the plate origin
        mid-travel (the old zero-ref clamp wrongly rejected negative deltas).
        """
        if not self.xy_stage:
            return

        # Safety: project cached absolute position + delta, clamp if needed
        if self.safety_limits.enabled:
            pos = self.get_xy_position(cached=True)
            if pos[0] is not None:
                new_x = pos[0] + dx
                new_y = pos[1] + dy
                clamped_x, clamped_y = self.safety_limits.clamp_xy(new_x, new_y)
                dx = clamped_x - pos[0]
                dy = clamped_y - pos[1]

        logger.debug(f"move_xy_relative: sending dx={round(dx)} dy={round(dy)} "
                     f"µm (raw: dx={dx:.2f} dy={dy:.2f})")
        self.xy_stage.move_stage_relative(dx, dy)
        self._note_move_estimate_xy_rel(dx, dy)  # display-only animation

    def move_xy_relative_um(self, dx_um: float, dy_um: float,
                            bypass_safety: bool = False) -> None:
        """
        Move XY stage by a relative offset in MICRONS.

        v7.2.5: The Prior ProScan controller accepts movement commands
        in microns directly. This method sends micron values without
        any microstep conversion.

        Safety limits are projected from cached position.

        v7.4.2: ``bypass_safety=True`` skips the soft-limit clamp.
        Used by the Hardware Setup → Device sub-page jog buttons so
        users can move freely to discover mechanical extremes when
        setting up a new machine.

        v7.5.x: the XY envelope is absolute stage µm, so clamp the absolute
        destination (cached + delta) directly — no zero-ref round-trip. Small
        moves stay valid after a Set Zero re-anchors the plate origin mid-travel.
        """
        if not self.xy_stage:
            return

        req_dx, req_dy = dx_um, dy_um  # preserve original request for debug log
        cached_pos = self.get_xy_position(cached=True)

        # Safety: project cached absolute position + delta, clamp if needed.
        # Limits and positions are all absolute stage µm.
        if self.safety_limits.enabled and not bypass_safety:
            if cached_pos[0] is not None:
                new_x = cached_pos[0] + dx_um
                new_y = cached_pos[1] + dy_um
                clamped_x, clamped_y = self.safety_limits.clamp_xy(new_x, new_y)
                dx_um = clamped_x - cached_pos[0]
                dy_um = clamped_y - cached_pos[1]

        clamped = (dx_um != req_dx or dy_um != req_dy)
        _dbg.log(
            "JOG_CMD",
            cmd_dx=f"{req_dx:.1f}", cmd_dy=f"{req_dy:.1f}",
            sent_dx=f"{dx_um:.1f}", sent_dy=f"{dy_um:.1f}",
            cached_x=f"{cached_pos[0]:.1f}" if cached_pos[0] is not None else "",
            cached_y=f"{cached_pos[1]:.1f}" if cached_pos[1] is not None else "",
            note="safety clamped" if clamped else "",
        )

        logger.debug(f"move_xy_relative_um: sending dx={dx_um:.1f} dy={dy_um:.1f} µm")
        self.xy_stage.move_stage_relative(dx_um, dy_um)
        self._note_move_estimate_xy_rel(dx_um, dy_um)  # display-only animation



    def effective_z_target_zref(self, z_zero_ref_mm: float) -> float:
        """Where a ``move_z_absolute(z, from_zero_ref=True)`` would ACTUALLY land.

        Applies the same two clamps the move applies — the absolute-raw soft
        limit and the print floor — and converts back to zero-ref mm. Pure: it
        commands no motion.

        This exists because those clamps silently MUTATE the destination. Before
        v7.9.1 the retract helper commanded ``z`` and then waited for ``z``, so
        any clamped move was structurally unconfirmable: the machine went one
        place and the wait polled for another until it timed out (16.5 s of
        unsupervised motion on the operator's bore-survey park). Callers now
        predict the landing point, refuse it if it is the wrong way, and confirm
        against what was really commanded.
        """
        raw = float(z_zero_ref_mm) + self.zero_position.get("Z", 0)
        if self.safety_limits.enabled:
            raw = self.safety_limits.clamp_z(raw)
        raw = self._apply_print_floor_raw(raw)
        return raw - self.zero_position.get("Z", 0)

    def move_z_absolute(
        self, z_value: float, from_zero_ref: bool = True, fast: bool = False,
        feedrate_mm_min: float | None = None,
    ) -> float | None:
        """Move Z needle to absolute position.

        Args:
            z_value: Target Z in mm (zero-ref or raw).
            from_zero_ref: If True, add zero reference offset.
            fast: If True, use maximum feedrate.
            feedrate_mm_min: Optional per-move feedrate (mm/min).

        Returns:
            The EFFECTIVE destination actually commanded, in the caller's own
            frame (zero-ref when ``from_zero_ref``, else raw) — i.e. after the
            soft-limit and print-floor clamps. ``None`` when there is no ZP
            stage. Returning this (it used to return ``None`` unconditionally,
            so no existing caller is affected) is what lets a retract confirm
            against the position the machine was really sent.
        """
        if not self.zp_stage:
            return None
        position = z_value
        if from_zero_ref:
            position += self.zero_position["Z"]
        # v7.5.x: the Z envelope is absolute Marlin raw mm. Clamp the
        # absolute destination (after adding the zero offset) in BOTH the
        # zero-ref and raw paths — the raw path previously skipped the soft
        # limit, so this also tightens safety.
        if self.safety_limits.enabled:
            position = self.safety_limits.clamp_z(position)
        # v7.5.x: never punch through the plate bottom during a print.
        position = self._apply_print_floor_raw(position)
        # v7.5.x ROOT-CAUSE FIX ("ZP drops on the Z retract"): a Z move with no
        # explicit feedrate emits a bare "G0 Z…" that INHERITS Marlin's last
        # modal F. Z and the pump are ONE Marlin board sharing modal feedrate,
        # so during a print the slow per-segment pump moves leave that F at the
        # flow rate (e.g. F1.8 mm/min). A bare full-travel Z retract then
        # crawled for ~12 min while Marlin sat "busy" → M400 timed out, the Z
        # motor overheated ("only Z gets hot"), and the board fell off USB.
        # Always supply a proper Z feedrate so a Z move can NEVER inherit the
        # pump's flow rate. (Jog/standalone travel always set their own F —
        # which is why only the print, with its interleaved pump moves, tripped
        # this.) Resolve: configured retract feedrate → Z max → conservative.
        if feedrate_mm_min is None:
            feedrate_mm_min = (
                getattr(self, "_zp_retract_feedrate", None)
                or getattr(self.safety_limits, "max_z_feedrate", None)
                or 200.0)
        self.zp_stage.move_absolute(
            {_axis_letter(self.zp_stage, "Z"): position}, fast,
            feedrate_mm_min=feedrate_mm_min)
        self._note_move_estimate_axis("Z", position, feedrate_mm_min)  # display-only

    def move_z_relative(self, distance: float, feedrate: float | None = None,
                        bypass_safety: bool = False) -> None:
        """Move Z by a relative RAW (zero-ref) delta — the low-level primitive.

        ``distance`` is a raw Marlin Z delta (+ = larger raw Z), NOT a
        height-frame delta. JOG inputs must NOT call this directly; they go
        through :meth:`move_z_user_relative`, which maps the operator's
        height-frame intent (up = +) to a raw delta via the per-machine
        ``z_up_sign``. Print / calibration pass raw deltas here as before.

        v7.5.x: Z direction is owned solely by ``z_up_sign`` (the Set
        Bottom/Top setup), so this no longer multiplies by ``_flip_sign("Z")``
        — the Z axis-flip UI was removed in v7.4.2 and ``axis_flip['Z']`` is
        force-zeroed (it was already a dead ×1.0); keeping it risked a
        double-invert against ``z_up_sign``. Pumps still honor ``_flip_sign``.

        v7.4.2: ``bypass_safety=True`` skips the soft-limit clamp.
        Used by the Hardware Setup → Device sub-page jog buttons.
        """
        if not self.zp_stage:
            return
        if feedrate and self.safety_limits.enabled and not bypass_safety:
            feedrate = self.safety_limits.clamp_z_feedrate(feedrate)
        if self.safety_limits.enabled and not bypass_safety:
            try:
                pos = self.get_zp_position(cached=True)
                # v7.5.x bugfix: resolve Z's tuple slot via the live
                # axis_map instead of hardcoding pos[0]. The ZP tuple is
                # physical Marlin order (X,Y,Z,E); on machines whose
                # axis_map routes Z to a physical axis other than X (e.g.
                # ME3B V1: Z→Z = index 2, P1→X = index 0), pos[0] is a
                # PUMP, not Z. Reading it bypassed the Z soft-limit and —
                # when that pump sat outside the Z envelope — drove Z a
                # fixed direction regardless of the requested up/down.
                # Mirrors the move_pump_relative pattern.
                zi = _axis_index(self.zp_stage, "Z")
                if zi is not None and zi < len(pos) and pos[zi] is not None:
                    cur_z = pos[zi]
                    # v7.5.x: the Z envelope is absolute Marlin raw mm, so
                    # clamp the absolute destination (cur + delta) directly —
                    # no zero-ref round-trip. Keeps jog valid after a Set Z
                    # Zero / needle-zero re-anchors the display zero (the old
                    # zero-ref clamp went stale and wrongly rejected moves).
                    clamped = self.safety_limits.clamp_z(cur_z + distance)
                    # v7.5.x: also hold above the plate bottom during a print.
                    clamped = self._apply_print_floor_raw(clamped)
                    # v7.5.x: the clamp may only SHORTEN the jog, never reverse
                    # or amplify it — so an out-of-bounds cached position can't
                    # produce a large opposite-direction snap (the jog-runaway).
                    distance = _shorten_only_delta(cur_z, distance, clamped)
            except Exception:
                pass
        self.zp_stage.move_relative(
            {_axis_letter(self.zp_stage, "Z"): distance}, feedrate)
        self._note_move_estimate_axis_rel("Z", distance, feedrate)  # display-only

    def move_z_user_relative(self, user_delta_mm: float,
                             feedrate: float | None = None,
                             bypass_safety: bool = False) -> None:
        """Jog Z by a delta in the USER / HEIGHT frame (up = +).

        This is the single entry point for every JOG input — on-screen Z▲/Z▼,
        PageUp/PageDown, the device-page jog, and (via its own provider) the
        Xbox stick. It maps the operator's height-frame intent to a raw Marlin
        delta with the per-machine ``z_up_sign`` — the up-direction DERIVED by
        the Set Bottom / Set Top setup (:meth:`apply_z_setup`):

            Δraw = Δuser · z_up_sign        (z_up_sign = ±1)

        so "up" always retracts the needle toward the taught Top, and the jog
        agrees with the position readout, regardless of the machine's raw-Z
        polarity (e.g. ME3B V1's negative ``steps_per_mm.Z``). Direction is
        owned here by ``z_up_sign`` alone (``move_z_relative`` no longer applies
        the retired Z ``axis_flip``), so there is no double-invert.
        """
        self.move_z_relative(float(user_delta_mm) * self.z_up_sign(),
                             feedrate=feedrate, bypass_safety=bypass_safety)

    # ── v7.5.x: polarity-safe travel-Z retract guarantee ───────────────
    #
    # CRITICAL SAFETY: before ANY XY move to a *different* location the needle
    # must be retracted to the safe / "move" / travel Z height. The two helpers
    # below make that test/guarantee POLARITY-SAFE (correct for ZDIR=+1 and the
    # ME3B V1 ZDIR=-1 where the needle descends as raw Z increases) and the
    # retract NEVER LOWERS the needle — a misconfigured/too-low target degrades
    # to a no-op instead of crashing the needle into the plate.

    def z_height_of(self, raw_zref_mm: float) -> float:
        """Zero-ref raw Z (mm) → operator HEIGHT frame (up = +).

        Multiplying by the per-machine ``z_up_sign`` yields a monotonic height
        where *larger always means the needle is physically higher* (further
        from the plate), regardless of the machine's raw-Z polarity. Use this
        for any "is the needle high enough?" comparison instead of comparing raw
        Z. Identical to :meth:`zref_to_user_z` (the user frame *is* this height).
        """
        return self.z_up_sign() * float(raw_zref_mm)

    def z_reference_reachable(self, z_zero_ref_mm) -> bool:
        """Is a stored Z reference (zero-ref mm) inside this machine's travel?

        A Z reference is captured as ``raw − zero_position["Z"]``, so it is only
        meaningful against the datum that was live when it was captured. Running
        the Set Bottom / Set Top setup (:meth:`apply_z_setup`) rewrites that
        datum, the up-sign AND the envelope — and nothing used to invalidate the
        references captured against the old one. The operator's rig carried a
        Fast-Move Z of +24.79 mm zero-ref, i.e. 24.79 mm BELOW the mechanical
        hard bottom: a "retract" to it was a full-depth plunge.

        Unreachable is provable, not a guess: the value maps to a raw position
        the soft-limit envelope itself rejects. Treat such a reference as absent
        (re-teach it) rather than clamping it into range — a clamped travel
        height is a wrong move that looks right.

        Returns True when there is nothing to check against (limits disabled or
        no envelope), so a machine without a configured envelope is unaffected.
        """
        if z_zero_ref_mm is None:
            return False
        limits = getattr(self, "safety_limits", None)
        if limits is None or not getattr(limits, "enabled", False):
            return True
        try:
            raw = float(z_zero_ref_mm) + self.zero_position.get("Z", 0)
            lo, hi = float(limits.z_min), float(limits.z_max)
        except Exception:
            return True
        if not (hi > lo):        # unset / degenerate envelope — nothing to say
            return True
        tol = 1e-6
        return (lo - tol) <= raw <= (hi + tol)

    def default_travel_z(self, reference_z_zero_ref_mm: float,
                         margin_mm: float = 10.0) -> float:
        """Fallback travel / "move" Z (zero-ref mm) when no Safe Z is calibrated.

        Returns a height ``margin_mm`` ABOVE ``reference`` in the polarity-safe
        HEIGHT frame, so the needle is genuinely retracted above the work
        position regardless of the machine's Z polarity (NOT a raw literal like
        ``5.0``, which is a *descent* on a ZDIR=-1 machine). The insert-clearance
        floor and the soft-limit envelope are still applied when this value is
        fed to a move.
        """
        return float(reference_z_zero_ref_mm) + self.z_up_sign() * abs(margin_mm)

    def needle_at_or_above(self, current_raw_zref_mm: float,
                           reference_raw_zref_mm: float,
                           tol_mm: float = 0.05) -> bool:
        """True if the needle is at least as HIGH (retracted) as a reference Z.

        Polarity-safe — compares in the height frame so it is correct for both
        ``ZDIR=+1`` and ``ZDIR=-1``. ``tol_mm`` is a small height slack.
        """
        return (self.z_height_of(current_raw_zref_mm)
                >= self.z_height_of(reference_raw_zref_mm) - abs(tol_mm))

    def set_retract_slow_lift(self, dist_mm: float,
                              feedrate_mm_min: float | None = None) -> None:
        """Configure the gentle "slow first mm" of every needle retract.

        ``dist_mm`` = how far the lift runs slowly before switching to the fast
        retract feedrate (0 disables — every retract is single-speed, the legacy
        behavior). ``feedrate_mm_min`` = the slow speed (kept if None). Applies
        to all retracts that funnel through :meth:`ensure_retracted_to` /
        :meth:`safe_travel_to`, so every print-execution lift inherits it.
        """
        self._retract_slow_dist_mm = max(0.0, float(dist_mm))
        if feedrate_mm_min is not None:
            self._retract_slow_feedrate = max(1.0, float(feedrate_mm_min))

    def set_descend_slow_final(self, dist_mm: float,
                               feedrate_mm_min: float | None = None) -> None:
        """Configure the gentle "slow last mm" of every needle re-entry descent.

        ``dist_mm`` = how far the FINAL leg of a descent runs slowly before the
        needle reaches the target (0 disables — every descent is single-speed,
        the legacy behavior). ``feedrate_mm_min`` = the slow speed (kept if
        None). Applies to descents that funnel through
        :meth:`_descend_z_moves_only` (``safe_travel_to`` step 3 / the discrete
        ``MOVE_Z`` descent), so every print/work re-entry inherits it.
        """
        self._descend_slow_dist_mm = max(0.0, float(dist_mm))
        if feedrate_mm_min is not None:
            self._descend_slow_feedrate = max(1.0, float(feedrate_mm_min))

    def _descend_z_moves_only(self, cur_zref_mm: float | None,
                              target_zref_mm: float,
                              fast_feedrate_mm_min: float | None) -> None:
        """EMIT (without confirming) a Z descent to ``target_zref_mm``
        (zero-ref), running the FINAL ``_descend_slow_dist_mm`` slowly.

        The polarity-safe descent twin of :meth:`_retract_z_slow_then_fast`,
        but **emit-only** — it does NOT M400/wait. The CALLER is responsible
        for confirming arrival (``safe_travel_to`` step 3 and the discrete
        ``MOVE_Z`` handler both already do their own M400 + ``wait_for_z_arrival``
        + abort), so this must not double-confirm.

        When the move is a net DESCENT in the HEIGHT frame (target LOWER than
        ``cur_zref_mm``), the needle descends fast to an intermediate height
        ``_descend_slow_dist_mm`` ABOVE the target, then covers the last
        ``_descend_slow_dist_mm`` at ``_descend_slow_feedrate`` — controlled
        touch-down instead of a crash-down. If the whole descent is shorter than
        the slow distance it all runs slowly (the fast leg is a no-op). An
        ASCENT (or ``cur_zref_mm`` is None, or slow dist 0) degrades to a single
        move to the target — it NEVER slows an ascent (retracts stay on the
        slow-LIFT path). Every move carries an explicit feedrate (never a bare
        ``G0 Z``, which would inherit the pump's slow modal F).
        """
        slow_dist = max(0.0, float(getattr(self, "_descend_slow_dist_mm", 0.0) or 0.0))
        if slow_dist > 0.0 and cur_zref_mm is not None:
            cur_h = self.z_height_of(float(cur_zref_mm))
            tgt_h = self.z_height_of(float(target_zref_mm))
            if tgt_h < cur_h - 1e-6:                 # a genuine descent
                # Intermediate height = slow_dist ABOVE the target, clamped so it
                # never sits above the current height (a short descent then runs
                # entirely slow — the fast leg is skipped). Mirror of the lift.
                inter_h = min(cur_h, tgt_h + slow_dist)
                if inter_h < cur_h - 1e-6:           # fast leg has distance
                    # height → zero-ref raw: zref = h / z_up_sign = h * z_up_sign.
                    inter_zref = inter_h * self.z_up_sign()
                    self.move_z_absolute(inter_zref, from_zero_ref=True,
                                         feedrate_mm_min=fast_feedrate_mm_min)
                slow_fr = (float(getattr(self, "_descend_slow_feedrate", 0.0) or 0.0)
                           or fast_feedrate_mm_min)
                self.move_z_absolute(float(target_zref_mm), from_zero_ref=True,
                                     feedrate_mm_min=slow_fr)
                return
        # Not a descent / unknown current Z / disabled → single move to target.
        self.move_z_absolute(float(target_zref_mm), from_zero_ref=True,
                             feedrate_mm_min=fast_feedrate_mm_min)

    def emit_descent_moves(self, target_zref_mm: float,
                           fast_feedrate_mm_min: float | None = None) -> None:
        """Emit (no confirm) a gentle-final-mm descent to ``target_zref_mm``.

        Reads the CACHED current Z (no serial round-trip, so it is safe to call
        OUTSIDE a poller-suspend window — e.g. the discrete ``MOVE_Z`` handler,
        where the descent is emitted before the handler suspends the poller for
        its M400/M114 confirm). The caller must confirm arrival. Falls back to a
        single move when the current Z is unknown. See
        :meth:`_descend_z_moves_only`.
        """
        cur_zref = None
        if float(getattr(self, "_descend_slow_dist_mm", 0.0) or 0.0) > 0:
            try:
                zp = self.get_zp_position(cached=True)
                cur = self.zp_logical_value(zp, "Z")
                if cur is not None:
                    cur_zref = cur - self.zero_position.get("Z", 0)
            except Exception:
                cur_zref = None
        self._descend_z_moves_only(cur_zref, float(target_zref_mm),
                                   fast_feedrate_mm_min)

    def estimate_gentle_z_time_s(self, target_zref_mm: float,
                                 fast_feedrate_mm_min: float | None = None,
                                 *, cur_zref_mm: float | None = None) -> float:
        """Estimate the wall-clock seconds a gentle two-segment Z move to
        ``target_zref_mm`` (zero-ref mm) will take.

        Mirrors the geometry of :meth:`_descend_z_moves_only` (slow LAST mm on a
        descent) and :meth:`_retract_z_slow_then_fast` (slow FIRST mm on a
        lift): the bulk runs at ``fast_feedrate_mm_min`` (defaults to the insert
        feedrate) and one ``slow_dist`` leg at the direction-appropriate slow
        feedrate. The slow leg can dominate — 1 mm @ 6 mm/min = 10 s — so a
        FIXED confirmation timeout can expire before a healthy move finishes and
        falsely flag the board as "stuck". Callers use this to SIZE the M400 /
        arrival timeout (the Z twin of :meth:`_wait_pump_move_complete`).

        Reads the CACHED current Z when ``cur_zref_mm`` is not supplied (no
        serial round-trip). Returns 0.0 when the current Z is unknown (the
        caller floors the timeout at its baseline anyway).
        """
        fast_fr = float(fast_feedrate_mm_min
                        or getattr(self, "_zp_insert_feedrate", 0.0) or 0.0)
        if fast_fr <= 0.0:
            fast_fr = 100.0

        cur_zref = cur_zref_mm
        if cur_zref is None:
            try:
                zp = self.get_zp_position(cached=True)
                cur = self.zp_logical_value(zp, "Z")
                if cur is not None:
                    cur_zref = cur - self.zero_position.get("Z", 0)
            except Exception:
                cur_zref = None
        if cur_zref is None:
            return 0.0

        cur_h = self.z_height_of(float(cur_zref))
        tgt_h = self.z_height_of(float(target_zref_mm))
        total = abs(tgt_h - cur_h)
        if tgt_h > cur_h + 1e-6:                 # a lift (slow first mm)
            slow_dist = max(0.0, float(getattr(self, "_retract_slow_dist_mm", 0.0) or 0.0))
            slow_fr = (float(getattr(self, "_retract_slow_feedrate", 0.0) or 0.0)
                       or fast_fr)
        else:                                    # a descent (slow last mm)
            slow_dist = max(0.0, float(getattr(self, "_descend_slow_dist_mm", 0.0) or 0.0))
            slow_fr = (float(getattr(self, "_descend_slow_feedrate", 0.0) or 0.0)
                       or fast_fr)
        slow = min(total, slow_dist)
        fast = max(0.0, total - slow)
        return (fast / max(fast_fr, 1.0) + slow / max(slow_fr, 1.0)) * 60.0

    def _retract_z_slow_then_fast(self, cur_zref_mm: float | None,
                                  target_zref_mm: float,
                                  fast_feedrate_mm_min: float | None,
                                  timeout_s: float,
                                  tol_mm: float = 0.1,
                                  abort_event=None) -> bool:
        """Move Z to ``target_zref_mm`` (zero-ref) and CONFIRM arrival, running
        the first ``_retract_slow_dist_mm`` of a *lift* slowly.

        Shared by :meth:`ensure_retracted_to` and :meth:`safe_travel_to`. When
        the move is a net LIFT in the polarity-safe HEIGHT frame (target higher
        than ``cur_zref_mm``), the needle first rises ``_retract_slow_dist_mm``
        at ``_retract_slow_feedrate`` — so a deposited bead can't peel off with
        the needle — then finishes at ``fast_feedrate_mm_min``. A descent (or
        ``cur_zref_mm`` is None, or slow dist 0) is a single move to the target.
        Only the FINAL position is confirmed (Marlin runs the two queued moves
        in order); every move carries an explicit feedrate (never a bare G0 Z).

        ``cur_zref_mm`` is the current needle Z in zero-ref mm (the caller has
        usually already read it). Returns True iff Z confirms at the target.

        v7.9.1 — THIS HELPER IS RAISE-ONLY, and it is the guard of record.
        Both callers are retracts, and CLAUDE.md's safety rule is absolute: a
        retract for travel must never lower the needle; a misconfigured
        travel-Z must degrade to a no-op, not a crash-down. It used to command
        the target unconditionally, so ``safe_travel_to`` — which, unlike
        ``ensure_retracted_to``, had no at-or-above pre-check — executed a
        DESCENT whenever the configured safe Z sat below the needle. On the
        operator's rig a stale ``safe_z`` (height −24.79 mm, i.e. below the
        mechanical bottom) turned the bore-survey park's "retract" into a plunge
        that only the print floor stopped, at the plate bottom.

        Note the at-or-above early-return is the WHOLE raise-only guard: if the
        needle is already high enough we do not move, and if it is not, the move
        is by construction a lift. There is deliberately no separate "refuse a
        descent" branch — every descent case is already an at-or-above case, so
        such a branch would only fire on ordinary travel that starts above the
        safe height, and would refuse it.

        Both guards below matter:
          1. Predict the EFFECTIVE landing point (the soft-limit and print-floor
             clamps silently mutate the destination) and both compare and
             confirm against that, not against the requested value. Confirming
             against the request is what made a clamped retract structurally
             unconfirmable.
          2. At-or-above ⇒ no motion.
        """
        # Read the current height if the caller did not — guard 2 needs it, and
        # without it a too-low target is executed as a descent (the crash).
        if cur_zref_mm is None:
            try:
                _zp = self.get_zp_position(cached=False)
                _cur = self.zp_logical_value(_zp, "Z")
                if _cur is not None:
                    cur_zref_mm = _cur - self.zero_position.get("Z", 0)
            except Exception:
                cur_zref_mm = None

        # Where the move would REALLY land once the clamps have had their say.
        try:
            eff_zref = self.effective_z_target_zref(float(target_zref_mm))
        except Exception:
            eff_zref = float(target_zref_mm)

        if cur_zref_mm is not None and self.needle_at_or_above(
                float(cur_zref_mm), eff_zref, tol_mm=float(tol_mm)):
            # Already retracted far enough — no motion. A misconfigured
            # (too-low) travel Z degrades to a no-op here, never a descent.
            return True

        if abs(eff_zref - float(target_zref_mm)) > 1e-6:
            logger.warning(
                "Z retract target %.2f mm clamped to %.2f mm (zero-ref) — "
                "confirming against the clamped value. A large clamp usually "
                "means the Fast-Move / Safe Z reference is stale (captured "
                "against a previous Z datum); re-teach it on Calibration → "
                "Needle Location → Advanced Z references.",
                float(target_zref_mm), eff_zref)
        target_zref_mm = eff_zref

        slow_dist = max(0.0, float(getattr(self, "_retract_slow_dist_mm", 0.0) or 0.0))
        if slow_dist > 0.0 and cur_zref_mm is not None:
            cur_h = self.z_height_of(float(cur_zref_mm))
            tgt_h = self.z_height_of(float(target_zref_mm))
            if tgt_h > cur_h + 1e-6:                 # a genuine lift
                inter_h = min(tgt_h, cur_h + slow_dist)
                if inter_h > cur_h + 1e-6:
                    slow_fr = (float(getattr(self, "_retract_slow_feedrate", 0.0) or 0.0)
                               or fast_feedrate_mm_min)
                    # height → zero-ref raw: zref = h / z_up_sign = h * z_up_sign.
                    inter_zref = inter_h * self.z_up_sign()
                    self.move_z_absolute(inter_zref, from_zero_ref=True,
                                         feedrate_mm_min=slow_fr)
        # Fast (or sole) segment to the final target, then confirm.
        self.move_z_absolute(float(target_zref_mm), from_zero_ref=True,
                             feedrate_mm_min=fast_feedrate_mm_min)
        # v7.5.x: size the confirm timeout to the (possibly slow-first-mm)
        # lift's estimated duration so a gentle low-speed lift is not falsely
        # timed out. Floored at the caller's baseline, capped as a backstop.
        eff_timeout = timeout_s
        try:
            est = self.estimate_gentle_z_time_s(
                float(target_zref_mm), fast_feedrate_mm_min,
                cur_zref_mm=cur_zref_mm)
            eff_timeout = min(max(timeout_s, est + _GENTLE_Z_CONFIRM_MARGIN_S),
                              _GENTLE_Z_CONFIRM_CAP_S)
        except Exception:
            pass
        zp = self.zp_stage
        if zp is not None and hasattr(zp, "flush_moves"):
            try:
                confirmed = zp.flush_moves(timeout_s=eff_timeout,
                                           abort_event=abort_event)
            except TypeError:       # fake/older flush without abort_event
                confirmed = zp.flush_moves(timeout_s=eff_timeout)
            if not confirmed:
                return False
        return self.wait_for_z_arrival(float(target_zref_mm),
                                       tolerance_mm=tol_mm,
                                       timeout_s=eff_timeout,
                                       abort_event=abort_event)

    def ensure_retracted_to(self, safe_z_zero_ref_mm: float,
                            tol_mm: float = 0.1,
                            timeout_s: float = 15.0,
                            feedrate_mm_min: float | None = None,
                            apply_insert_floor: bool = True,
                            abort_event=None) -> bool:
        """Guarantee the needle is retracted to >= ``safe_z`` before XY travel.

        Raises the needle (in the HEIGHT frame) to at least ``safe_z``
        (zero-ref mm) and BLOCKS until Z confirms arrival. It **never lowers**
        the needle: if the needle is already at/above the target height
        (polarity-aware), it returns immediately without motion, so a
        misconfigured/too-low target can never cause a crash-down. Honors the
        plate-insert clearance floor (:meth:`set_min_travel_z`).

        v7.5.x: ``apply_insert_floor`` (default True) — when False the
        tube-clearance floor is skipped and the retract goes to exactly the
        requested ``safe_z``. Used by the calibration MOSAIC SCAN / mapping
        travel, which stays at the operator-assigned imaging safe Z the whole
        time (never descends into a well), so the tube-clearance floor is
        unnecessary there and was surprising the operator by raising the
        retract above their assigned safe Z. Print / pick-place travel keeps
        the floor (default True).

        Returns True if the needle is confirmed at/above the height (or there is
        no ZP stage), False if the retract move timed out.

        v7.6 ``abort_event``: forwarded to the confirm waits so a *travel*
        retract can unwind quickly during an abort. ⚠ The SAFETY retract of
        record (``PrintManager._retract_to_safe_z`` and the pick&place
        executor's finally) deliberately passes **None** — that retract must
        always run to completion.
        """
        if not self.is_zp_connected:
            return True

        target = float(safe_z_zero_ref_mm)
        # Floor the retract so the needle clears the tallest insert/tube.
        # Compare in the height frame (polarity-safe).
        floor = getattr(self, "_min_travel_z_mm", None)
        if (apply_insert_floor and floor is not None
                and self.z_height_of(floor) > self.z_height_of(target)):
            target = float(floor)

        # Already at/above the target height? No motion — never descend.
        zp = self.get_zp_position(cached=False)
        cur = self.zp_logical_value(zp, "Z")
        if cur is not None:
            cur_zref = cur - self.zero_position.get("Z", 0)
            if self.needle_at_or_above(cur_zref, target, tol_mm=tol_mm):
                return True

        # Retract up to the target height and wait (M400 + position poll).
        # v7.5.x: an optional per-call feedrate (e.g. Quick Print's FAST per-line
        # hop) overrides the default retract feedrate; never a bare G0 Z (which
        # would inherit the pump's slow modal F). v7.5.x: the lift's first
        # ``_retract_slow_dist_mm`` runs slowly (gentle release of the bead) via
        # _retract_z_slow_then_fast — a near-no-op when the needle is already
        # retracted (we early-returned above) or when slow dist is 0.
        _retract_fr = (float(feedrate_mm_min) if feedrate_mm_min
                       else self._zp_retract_feedrate)
        cur_zref = (cur - self.zero_position.get("Z", 0)) if cur is not None else None
        self._pos_poller.suspend()
        try:
            ok = self._retract_z_slow_then_fast(
                cur_zref, target, _retract_fr, timeout_s, tol_mm=tol_mm,
                abort_event=abort_event)
            if not ok:
                logger.error("ensure_retracted_to: Z retract not confirmed at "
                             "travel height — needle may not be at travel Z")
            return ok
        finally:
            self._pos_poller.resume()

    def suspend_position_poller(self) -> None:
        """Pause the background ZP/XY position poller.

        v7.5.x ZP-disconnect fix: during a print's dense per-segment ZP motion
        the poller's M114 reads contend with the command write-stream, fail to
        parse, and after ~2.5 s of consecutive failures the liveness watchdog
        FALSE-POSITIVES a "ZP disconnected" mid-print. Callers that run such a
        write burst (e.g. the discrete PRINT_PATH) suspend the poller for the
        duration. v7.21.2: REFCOUNTED, so nested suspends are safe — an inner
        probe's resume no longer un-suspends an outer caller. Guarded — safe
        when no poller exists (e.g. headless/mock controllers). ALWAYS pair with
        :meth:`resume_position_poller` in a try/finally so an abort/exception
        cannot leave the poller (and the live position display) frozen.
        """
        p = getattr(self, "_pos_poller", None)
        if p is not None:
            p.suspend()

    def resume_position_poller(self) -> None:
        """Resume the background position poller (pairs with
        :meth:`suspend_position_poller`). Also resets the liveness fail window
        (see :meth:`PositionPoller.resume`). Guarded — safe when no poller."""
        p = getattr(self, "_pos_poller", None)
        if p is not None:
            p.resume()

    def is_position_poller_suspended(self) -> bool:
        """v7.19: True while some sequence has the position poller suspended.

        Read as a cheap PROXY for "another workflow is currently driving the
        stage" — every long programmatic sequence in this app (a print's
        PRINT_PATH, a mosaic scan, ``safe_travel_to``) suspends the poller for
        its duration, so the flag being set means one of them is in flight.
        Quick Print's held print queue uses it to refuse a Resume that would put
        a second driver on the serial channel: ``_stage_busy()`` is a per-PAGE
        guard, so a workflow cannot otherwise see what another page is doing.

        ⚠ A proxy, NOT a lease. It catches a running scan or print; it does not
        catch a manual jog, and it cannot say WHICH workflow is driving. A
        process-wide stage lease is the real fix. Guarded (returns False when
        there is no poller) so a mock/headless controller reads as idle.
        """
        p = getattr(self, "_pos_poller", None)
        return bool(getattr(p, "_suspended", False)) if p is not None else False

    def suspend_zp_watchdog(self) -> None:
        """v7.5.x: pause the ZP port-health watchdog for the duration of a dense
        write burst (PRINT_PATH). The watchdog reads ``serial.in_waiting``
        (ClearCommError) on the ZP COM handle every ~2 s; a concurrent
        ClearCommError + WriteFile on a flow-control-paused CH340 is a fault
        surface. Pausing it (alongside the poller) makes the print thread the
        SOLE accessor of the ZP handle during the burst. Guarded + idempotent;
        ALWAYS pair with :meth:`resume_zp_watchdog` in a try/finally."""
        wd = getattr(self, "_watchdog", None)
        if wd is not None and hasattr(wd, "pause"):
            try:
                wd.pause("ZP")
            except Exception:
                pass

    def resume_zp_watchdog(self) -> None:
        """Resume the ZP port-health watchdog (pairs with
        :meth:`suspend_zp_watchdog`); resets its debounce window. Guarded."""
        wd = getattr(self, "_watchdog", None)
        if wd is not None and hasattr(wd, "resume"):
            try:
                wd.resume("ZP")
            except Exception:
                pass

    def _needle_present(self) -> bool:
        """True if this machine has a Z/needle (ZP) axis that could be dragged.

        Used by ``safe_travel_to`` to decide whether an automated XY travel must
        be REFUSED when the needle can't be confirmed retracted (ZP not
        connected). True when the ZP board is connected now, was ever connected
        this session (a real needle that has since dropped), or the hardware
        config defines pumps (the machine is built with a needle). Only a rig
        with genuinely no ZP axis (no board, never connected, no pumps
        configured) reads False — there XY travel is safe with no retract.
        """
        if self.is_zp_connected:
            return True
        if getattr(self, "_zp_ever_connected", False):
            return True
        hw = getattr(self, "_hardware_config", None)
        return bool(hw is not None and getattr(hw, "pumps", None))

    def safe_travel_to(
        self,
        target_x_um: float,
        target_y_um: float,
        safe_z_mm: float,
        target_z_mm: float | None = None,
        fast_xy_speed_mm_s: float | None = None,
        z_timeout_s: float = 15.0,
        xy_timeout_s: float = 30.0,
        apply_insert_floor: bool = True,
        abort_event=None,
    ) -> bool:
        """Safe 3-step travel: raise Z → wait → fast XY → wait → lower Z.

        v7.3.2: Now blocks until Z reaches safe height before starting XY
        move, and waits for XY arrival before lowering Z. This prevents
        needle crashes from premature XY moves while Z is still retracting.

        v7.3.5: Two-layer Z verification (belt and suspenders):
          1. flush_moves() — M400 command-level wait (Marlin confirms done)
          2. wait_for_z_arrival() — position polling via M114 (physical verification)
        XY will NOT start unless both layers confirm Z is at safe height.

        Args:
            target_x_um: Absolute target X in µm (raw stage coords).
            target_y_um: Absolute target Y in µm (raw stage coords).
            safe_z_mm: Safe travel height in mm (zero-referenced).
            target_z_mm: Optional target Z after XY move (zero-referenced).
                         If None, stays at safe_z.
            fast_xy_speed_mm_s: XY travel speed in mm/s. v7.21.1 — ``None``
                (the default) resolves to this machine's configured XY max
                speed via :meth:`get_max_xy_speed_um_s`.

                This used to default to a hardcoded ``50.0`` that NO caller
                overrode, so every travel — the fluorescence mosaic's first
                tile, Quick Print's pre-position, pick & place, calibration
                navigation — commanded 50 mm/s regardless of what Hardware
                Setup said, and (Prior SMS being modal) every subsequent
                un-speeded move inherited it. That is the bug this default
                fixes; a caller with a genuine reason for a different speed
                still passes one explicitly.
            z_timeout_s: Max seconds to wait for Z arrival.
            xy_timeout_s: Max seconds to wait for XY arrival.
            abort_event: v7.6 — optional ``threading.Event``. When set, the
                confirm waits return at once and the sequence stops WITHOUT
                starting the next step (so an abort can't launch a fresh XY
                travel or Z descent). The safety ordering is unchanged: XY
                still never starts unless the retract was confirmed.

        Returns:
            True if all moves completed successfully, False if any timed out.
        """
        ok = True

        def _aborting() -> bool:
            return abort_event is not None and abort_event.is_set()

        if _aborting():
            logger.info("safe_travel_to: abort_event already set — no motion")
            return False

        # v7.4.8: floor the retract height so the needle clears the
        # tallest insert/tube on the plate (set via set_min_travel_z()).
        # Both values are zero-referenced mm. v7.5.x: compare in the HEIGHT
        # frame (polarity-safe) so the floor still raises — not lowers — the
        # needle on ZDIR=-1 machines where larger raw Z = lower needle.
        # v7.5.x: apply_insert_floor=False skips it for imaging-height mosaic
        # scan / mapping travel (needle never descends there), so that flow
        # honors the operator's exact assigned safe Z.
        min_travel_z = getattr(self, "_min_travel_z_mm", None)
        if apply_insert_floor and min_travel_z is not None and \
                self.z_height_of(min_travel_z) > self.z_height_of(safe_z_mm):
            logger.info(
                f"safe_travel_to: raising safe Z {safe_z_mm:.2f} → "
                f"{min_travel_z:.2f} mm to clear plate inserts")
            safe_z_mm = min_travel_z

        # Suspend the position poller for the entire sequence to prevent
        # serial races between the poller thread and our M400 / M114 waits.
        self._pos_poller.suspend()
        try:
            # Step 1: Raise Z to safe height (gentle first mm) and WAIT.
            # v7.5.x: the lift's first _retract_slow_dist_mm runs slowly so the
            # deposited bead can't peel off with the needle (see
            # _retract_z_slow_then_fast); the helper still does M400 +
            # position-poll confirmation and ABORTS the XY move if Z is not
            # confirmed at the safe height.
            if self.is_zp_connected:
                _cur_zref = None
                if float(getattr(self, "_retract_slow_dist_mm", 0.0) or 0.0) > 0:
                    try:
                        _zp = self.get_zp_position(cached=False)
                        _cur = self.zp_logical_value(_zp, "Z")
                        if _cur is not None:
                            _cur_zref = _cur - self.zero_position.get("Z", 0)
                    except Exception:
                        _cur_zref = None
                if not self._retract_z_slow_then_fast(
                        _cur_zref, safe_z_mm, self._zp_retract_feedrate,
                        z_timeout_s, tol_mm=0.1, abort_event=abort_event):
                    logger.error("safe_travel_to: Z retract not confirmed at safe "
                                 "height — ABORTING, will not start XY move")
                    return False

                logger.info(f"safe_travel_to: Z confirmed at safe height "
                            f"{safe_z_mm:.2f} mm — proceeding to XY")

            elif self._needle_present():
                # CRITICAL SAFETY: the retract block above is gated on
                # is_zp_connected. If the ZP board is NOT connected but this
                # machine has a needle (board dropped mid-session, or pumps are
                # configured), we cannot confirm — let alone perform — the
                # safe-Z retract, so we must NOT drive XY: that would drag the
                # needle (typically left DOWN) across the plate. Abort the
                # travel. A rig with genuinely no ZP axis falls through and
                # moves XY normally (nothing to drag).
                logger.error(
                    "safe_travel_to: ZP board not connected but a needle is "
                    "present — ABORTING XY move (cannot confirm/perform the "
                    "safe-Z retract; would drag the needle). Reconnect the ZP "
                    "board.")
                return False

            # Step 2: Fast XY travel and WAIT for arrival
            if _aborting():
                logger.info("safe_travel_to: abort during the retract — "
                            "not starting the XY travel")
                return False
            if self.is_xy_connected:
                _travel_mm_s = fast_xy_speed_mm_s
                if _travel_mm_s is None:
                    try:
                        _travel_mm_s = float(self.get_max_xy_speed_um_s()) / 1000.0
                    except (TypeError, ValueError):
                        _travel_mm_s = 0.0
                    if not _travel_mm_s or _travel_mm_s <= 0:
                        _travel_mm_s = self._XY_MAX_FALLBACK_UM_S / 1000.0
                    # An UNDECLARED machine is the one case where the commanded
                    # mm/s and the achieved mm/s can still disagree: the stage
                    # converts against its protocol's nominal max_speed, so a
                    # travel commanded at the (lower) safety anchor comes out
                    # proportionally slow. Safe — slow, never fast — but it
                    # looks like a fault, so name the one-click remedy. Once per
                    # session; the flag lives on the instance.
                    if (not self.declared_xy_top_speed_um_s()
                            and not getattr(self, "_warned_xy_top_undeclared",
                                            False)):
                        self._warned_xy_top_undeclared = True
                        logger.warning(
                            "XY travel is using %.2f mm/s, but this machine has "
                            "no declared top speed, so the stage is still "
                            "converting mm/s against its protocol default — "
                            "travel may run slower than commanded. Set Hardware "
                            "Setup → Device → XY Stage Calibration → Max speed "
                            "(or measure it: Workflows → XY↔ZP Timing "
                            "Calibration → Measure top speed → Apply).",
                            _travel_mm_s)
                if hasattr(self, 'xy_stage') and self.xy_stage:
                    if hasattr(self.xy_stage, 'set_speed_mm_s'):
                        self.xy_stage.set_speed_mm_s(_travel_mm_s)
                    else:
                        self.xy_stage.set_velocity(100)
                self.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)

                # Convert raw stage coords to zero-ref mm (matching get_xy_position_mm)
                zero_x = self.zero_position.get("x", 0)
                zero_y = self.zero_position.get("y", 0)
                target_x_mm = (target_x_um - zero_x) / 1000.0
                target_y_mm = (target_y_um - zero_y) / 1000.0
                if not self.wait_for_xy_arrival(target_x_mm, target_y_mm,
                                                tolerance_mm=0.5,
                                                timeout_s=xy_timeout_s):
                    # v7.20 CRITICAL SAFETY — this used to log
                    # "proceeding with Z descent anyway" and then LOWER THE
                    # NEEDLE at an unverified position. That is a broken needle
                    # against the plate or a well wall, and it is the documented
                    # bench failure this guard exists to prevent.
                    #
                    # Note the asymmetry it removes: step 1 already ABORTS when
                    # the Z RETRACT cannot be confirmed. An unconfirmed XY
                    # arrival is no less dangerous — it is the difference
                    # between descending into a well and descending into its
                    # rim — so it now fails closed the same way. The needle is
                    # left RETRACTED at the safe height, which is the recoverable
                    # outcome; the caller sees False.
                    logger.error(
                        "safe_travel_to: XY arrival NOT confirmed "
                        "(target=%.3f, %.3f mm) — ABORTING before the Z "
                        "descent; leaving the needle retracted at the safe "
                        "height rather than lowering it at an unverified "
                        "position", target_x_mm, target_y_mm)
                    return False

            # Step 3: Lower Z to target and WAIT. v7.5.x: the descent's FINAL
            # _descend_slow_dist_mm runs slowly (gentle, controlled re-entry
            # onto the plate / into a bead — the descent twin of step 1's slow
            # first mm) via _descend_z_moves_only; the bulk stays at the fast
            # insert feedrate. Emit-only — the two confirm layers below are
            # preserved (the helper does NOT M400/wait). The poller is already
            # suspended for the whole sequence, so the cur-Z read is race-free.
            if _aborting():
                logger.info("safe_travel_to: abort before the Z descent — "
                            "leaving the needle retracted")
                return False
            if self.is_zp_connected and target_z_mm is not None:
                _cur_zref3 = None
                if float(getattr(self, "_descend_slow_dist_mm", 0.0) or 0.0) > 0:
                    try:
                        _zp3 = self.get_zp_position(cached=False)
                        _cur3 = self.zp_logical_value(_zp3, "Z")
                        if _cur3 is not None:
                            _cur_zref3 = _cur3 - self.zero_position.get("Z", 0)
                    except Exception:
                        _cur_zref3 = None
                # v7.9.1: confirm against where the descent will REALLY land.
                # The soft-limit and print-floor clamps mutate the destination
                # silently, so waiting on the requested value made any clamped
                # descent structurally unconfirmable (see
                # effective_z_target_zref).
                try:
                    target_z_mm = self.effective_z_target_zref(float(target_z_mm))
                except Exception:
                    pass
                self._descend_z_moves_only(_cur_zref3, float(target_z_mm),
                                           self._zp_insert_feedrate)

                # v7.5.x: size the descent confirm timeout to the estimated
                # duration — the gentle slow-last-mm (e.g. 1 mm @ 6 mm/min =
                # 10 s) can exceed the fixed z_timeout_s on a healthy board.
                _dto = z_timeout_s
                try:
                    _est = self.estimate_gentle_z_time_s(
                        float(target_z_mm), self._zp_insert_feedrate,
                        cur_zref_mm=_cur_zref3)
                    _dto = min(max(z_timeout_s, _est + _GENTLE_Z_CONFIRM_MARGIN_S),
                               _GENTLE_Z_CONFIRM_CAP_S)
                except Exception:
                    pass

                # Layer 1: M400 command-level wait
                if hasattr(self.zp_stage, 'flush_moves'):
                    if not self.zp_stage.flush_moves(timeout_s=_dto):
                        logger.warning("safe_travel_to: Z descent M400 timed out")
                        ok = False

                # Layer 2: Poll actual Z position to verify
                if not self.wait_for_z_arrival(target_z_mm, tolerance_mm=0.1,
                                               timeout_s=_dto):
                    logger.warning("safe_travel_to: Z descent position verification failed")
                    ok = False

        finally:
            self._pos_poller.resume()

        logger.info(f"Safe travel to ({target_x_um:.0f}, {target_y_um:.0f}) µm, "
                    f"safe_z={safe_z_mm:.2f} mm, ok={ok}")
        return ok

    def is_pump_enabled(self, pump: str) -> bool:
        """Check if a pump is enabled in the hardware config."""
        hw = self._hardware_config
        if hw is None:
            return True  # No config → allow all (legacy behavior)
        pump_cfg = hw.pumps.get(pump)
        if pump_cfg is None:
            return True  # Unknown pump → allow
        return pump_cfg.is_configured

    def move_pump_relative(
        self, pump: str, distance: float, feedrate: float | None = None,
        bypass_safety: bool = False,
    ) -> None:
        """Move a pump (P1/P2/P3) by relative distance (mm of *dispense intent*:
        positive = DISPENSE, toward empty; negative = ASPIRATE, toward full).

        v7.5.x: the raw-mm direction is OWNED by the plunger calibration once a
        pump is calibrated (:meth:`pump_dir_sign` → ``−aspirate_sign``), the same
        way ``z_up_sign`` owns Z. Uncalibrated pumps fall back to the legacy
        per-axis ``_flip_sign`` so nothing changes until a pump is set up.

        v7.4.2: ``bypass_safety=True`` skips both the soft-limit clamp
        AND the ``is_pump_enabled`` check. Used by the Hardware Setup jog
        (incl. the plunger-setup capture) so users can jog pumps to their
        mechanical limits before HardwareConfig / calibration is set up.
        """
        if not self.zp_stage:
            return
        if not bypass_safety and not self.is_pump_enabled(pump):
            logger.warning(f"Pump {pump} is disabled — move blocked")
            return
        distance = distance * self.pump_dir_sign(pump)
        if feedrate and self.safety_limits.enabled and not bypass_safety:
            feedrate = self.safety_limits.clamp_pump_feedrate(feedrate, pump)
        if self.safety_limits.enabled and not bypass_safety:
            try:
                pos = self.get_zp_position(cached=True)
                if pos[0] is not None:
                    # v7.4.2 hotfix: index through live axis_map so the
                    # safety clamp reads the correct physical motor.
                    idx = _axis_index(self.zp_stage, pump)
                    if idx is not None and idx < len(pos):
                        cur = pos[idx]
                        # v7.5.x: pump envelope is absolute Marlin raw mm —
                        # clamp the absolute destination directly (no zero-ref
                        # round-trip), so it stays valid after a pump re-zero.
                        clamped = self.safety_limits.clamp_pump(
                            cur + distance, pump)
                        # v7.5.x: shorten-only — an out-of-bounds cached pump
                        # position can't produce a large opposite snap.
                        distance = _shorten_only_delta(cur, distance, clamped)
            except Exception:
                pass
        mapped = _axis_letter(self.zp_stage, pump)
        if mapped:
            self.zp_stage.move_relative({mapped: distance}, feedrate)
            self._note_move_estimate_axis_rel(pump, distance, feedrate)  # display-only

    # ── v7.1: Velocity & Timestamped Position API ──────────────────

    def send_velocity_xy(self, vx: float, vy: float) -> None:
        """
        P8.24: Send continuous velocity command to XY stage.

        Used by the MotionController for trajectory tracking. Velocity
        units match the stage's native format (µm/s for Prior (VS command default unit per manual)).

        Args:
            vx: X velocity
            vy: Y velocity
        """
        if not self.xy_stage:
            return
        self.xy_stage.move_stage_at_velocity(vx, vy)

    # ── v7.6: hard abort — kill ALL motion with bounded latency ────

    def abort_all_motion(self, reason: str = "") -> dict:
        """Immediately stop every axis: XY, Z and the pumps.

        Best-effort, callable from ANY thread (including the GUI thread — it
        never blocks for more than ~2.5 s and never raises), and idempotent, so
        several abort routes may all call it.

        Sequence:
          1. **XY** — ``XYStage.stop_stage()`` (Prior ``I``, immediate) then
             ``send_velocity_xy(0, 0)`` so a standing ``VS`` cannot resume
             motion after the stop.
          2. **ZP** — ``ZPStage.quickstop()`` (Marlin ``M410``), which kills Z
             **and** the pumps together. Bounded: it raw-writes when the serial
             lock is held by an in-flight ``M400`` (see that method).
          3. **Re-sync** — ``ZPStage.resync_position()``: an aborted move loses
             position accuracy, so the cached coordinates must be refreshed
             before anything trusts them.

        Deliberately NOT M112 (``emergency_stop``): that KILLS Marlin and needs
        a board reset, so it stays reserved for the operator's Escape-key hard
        E-stop. This is the routine-abort path — the board stays alive so the
        needle can still be retracted afterwards.

        Returns a diagnostic dict (also logged) — the caller decides what to do
        next; retracting the needle is the caller's job (the print thread's
        ``finally`` does it, raise-only).
        """
        out = {"reason": reason, "xy_stopped": False, "vs_zeroed": False,
               "zp_quickstop": False, "resync_ok": False,
               "emergency_parser": getattr(self.zp_stage, "emergency_parser",
                                           None)}
        xy = getattr(self, "xy_stage", None)
        if xy is not None:
            try:
                stop = getattr(xy, "stop_stage", None)
                if callable(stop):
                    stop()
                    out["xy_stopped"] = True
            except Exception as e:
                logger.error(f"abort_all_motion: XY stop_stage failed: {e}")
            try:
                self.send_velocity_xy(0.0, 0.0)
                out["vs_zeroed"] = True
            except Exception as e:
                logger.error(f"abort_all_motion: VS zero failed: {e}")
        zp = getattr(self, "zp_stage", None)
        if zp is not None:
            try:
                qs = getattr(zp, "quickstop", None)
                if callable(qs):
                    out["zp_quickstop"] = bool(qs())
            except Exception as e:
                logger.error(f"abort_all_motion: ZP quickstop failed: {e}")
            if out["zp_quickstop"]:
                try:
                    rs = getattr(zp, "resync_position", None)
                    if callable(rs):
                        rs()
                        out["resync_ok"] = True
                except Exception as e:
                    logger.error(f"abort_all_motion: ZP resync failed: {e}")
        logger.warning(f"ABORT ALL MOTION ({reason or 'no reason given'}): {out}")
        if not out["zp_quickstop"] and zp is not None:
            logger.error(
                "abort_all_motion: ZP motion was NOT stopped — queued Z/pump "
                "moves will run to completion")
        return out

    def get_position_with_timestamp(self) -> dict:
        """
        P8.25: Get position with monotonic timestamp for Kalman filter.

        Returns a dict with timestamped XY and ZP positions. Uses
        direct (non-cached) reads for accuracy.

        Returns:
            {
                "t": float (time.monotonic()),
                "xy": (x, y, z) or (None, None, None),
                "zp": (z, p1, p2, p3) or (None, None, None, None),
            }
        """
        t_before = time.monotonic()
        xy = self.get_xy_position(cached=False)
        zp = self.get_zp_position(cached=False)
        t_after = time.monotonic()

        return {
            "t": (t_before + t_after) / 2.0,  # Midpoint estimate
            "latency_s": t_after - t_before,
            "xy": xy,
            "zp": zp,
        }

    def test_command_rate(self, num_tests: int = 20) -> dict:
        """
        P8.26: Test the XY stage command/response rate.

        Sends a series of position queries to measure communication latency.
        Safe to call at any time — only reads position, no movement.

        ⚠ SAFETY: This method does NOT move the stage. It only queries position.

        Args:
            num_tests: Number of position queries to time (default 20)

        Returns:
            {
                "avg_round_trip_ms": float,
                "min_round_trip_ms": float,
                "max_round_trip_ms": float,
                "max_command_hz": float,
                "num_tests": int,
                "controller": str,
            }
        """
        if not self.xy_stage:
            return {
                "error": "XY stage not connected",
                "avg_round_trip_ms": 0,
                "max_command_hz": 0,
            }

        latencies = []
        for _ in range(num_tests):
            t0 = time.monotonic()
            self.xy_stage.get_current_position()
            t1 = time.monotonic()
            latencies.append((t1 - t0) * 1000)  # ms
            time.sleep(0.01)  # Small gap to avoid flooding

        avg_ms = sum(latencies) / len(latencies)
        min_ms = min(latencies)
        max_ms = max(latencies)

        controller_name = "unknown"
        if hasattr(self.xy_stage, '_protocol') and self.xy_stage._protocol:
            controller_name = self.xy_stage._protocol.controller_name

        result = {
            "avg_round_trip_ms": round(avg_ms, 2),
            "min_round_trip_ms": round(min_ms, 2),
            "max_round_trip_ms": round(max_ms, 2),
            "max_command_hz": round(1000.0 / avg_ms, 1) if avg_ms > 0 else 0,
            "num_tests": num_tests,
            "controller": controller_name,
        }

        logger.info(
            f"Command rate test: avg={avg_ms:.1f}ms, "
            f"max_hz={result['max_command_hz']}, "
            f"controller={controller_name}"
        )
        return result

    def measure_control_loop_rate(
        self,
        *,
        iterations: int = 40,
        speed_um_s: float = 2000.0,
        amplitude_um: float = 600.0,
        on_progress=None,
        stop_evt=None,
    ) -> dict:
        """Measure the achievable CLOSED-LOOP control cadence — the period of
        one interleaved *(send a motion command + read a fresh position)* cycle
        WHILE the stage is actively moving.

        This is distinct from :meth:`test_command_rate` (which only polls, no
        motion). The velocity-following print path issues exactly this cycle —
        ``send_velocity_xy`` then ``get_xy_position(cached=False)`` — every tick,
        so the period measured here is the loop_period that GOVERNS how fast the
        stage can print before the pure-pursuit controller goes unstable
        (``v_max ≈ lookahead / (loop_period × safety)``). A controller that
        streams motion commands can respond to reads more slowly while moving
        than while idle, which is why this measures under real motion.

        ⚠ SAFETY: the caller MUST have the needle retracted to a safe Z. The
        stage oscillates within ``±amplitude_um`` of its current X position
        (velocity reversed each time the bound is crossed) so net displacement
        is ~0; XY-only, no Z motion; the velocity is ALWAYS stopped (VS 0,0) and
        the stage returned to its start on exit; the position poller is
        suspended for the duration. Falls back gracefully on a controller with
        no continuous-velocity command (the command still exercises whatever
        motion primitive ``send_velocity_xy`` maps to — e.g. the Ludl pulsed
        jog).

        Returns a dict with ``avg/min/max_period_ms``, ``control_hz``,
        ``avg_read_ms``, ``avg_cmd_ms``, ``moved``, ``max_excursion_um`` — or
        ``{"error": ...}`` if the stage isn't ready.
        """
        xy = self.xy_stage
        if xy is None or not self.is_xy_connected:
            return {"error": "XY stage not connected"}

        p0 = self.get_xy_position(cached=False)
        if not p0 or p0[0] is None or p0[1] is None:
            return {"error": "could not read stage position"}
        start_x, start_y = float(p0[0]), float(p0[1])

        # Clamp the probe speed to the safety envelope so a slow loop can't run
        # the stage away between reads.
        v = abs(float(speed_um_s))
        try:
            cap = float(getattr(self.safety_limits, "max_xy_speed", 0) or 0)
            if cap > 0:
                v = min(v, cap)
        except Exception:
            pass
        v = max(v, 100.0)
        amp = max(50.0, abs(float(amplitude_um)))

        # Raise SMS enough that the commanded VS isn't capped below the probe
        # speed (so the stage actually moves → a realistic "while moving" cycle).
        try:
            if hasattr(xy, "set_acceleration"):
                xy.set_acceleration(80)
            if hasattr(xy, "set_speed_mm_s"):
                xy.set_speed_mm_s(max(v / 1000.0 * 1.5, 3.0))
        except Exception:
            pass

        read_ms: list = []
        cmd_ms: list = []
        loop_ms: list = []
        direction = 1.0
        max_excursion = 0.0
        moved = False

        self.suspend_position_poller()
        try:
            for i in range(max(1, int(iterations))):
                # v7.21.2: cooperative abort. Without this the probe runs its
                # full ~2.3 s regardless, which is the whole abort latency of
                # the calibration's first step.
                if stop_evt is not None and stop_evt.is_set():
                    break
                t_loop = time.monotonic()
                # 1) a MOTION command (the thing that makes this "while moving")
                t_c = time.monotonic()
                try:
                    self.send_velocity_xy(direction * v, 0.0)
                except Exception:
                    break
                cmd_ms.append((time.monotonic() - t_c) * 1000.0)
                # 2) a fresh position read (closed-loop feedback)
                t_r = time.monotonic()
                p = self.get_xy_position(cached=False)
                read_ms.append((time.monotonic() - t_r) * 1000.0)
                loop_ms.append((time.monotonic() - t_loop) * 1000.0)

                if p and p[0] is not None:
                    dx = float(p[0]) - start_x
                    if abs(dx) > max_excursion:
                        max_excursion = abs(dx)
                    if abs(dx) > 20.0:
                        moved = True
                    # reverse before running past the bound
                    if direction > 0 and dx >= amp:
                        direction = -1.0
                    elif direction < 0 and dx <= -amp:
                        direction = 1.0
                else:
                    # blind — don't keep driving
                    break
                if on_progress is not None:
                    try:
                        on_progress(i + 1)
                    except Exception:
                        pass
        finally:
            try:
                self.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass
            try:
                self.move_xy_absolute_um(start_x, start_y)
            except Exception:
                pass
            self.resume_position_poller()

        if not loop_ms:
            return {"error": "no samples collected"}

        def _stats(a):
            return (sum(a) / len(a), min(a), max(a)) if a else (0.0, 0.0, 0.0)

        avg_loop, min_loop, max_loop = _stats(loop_ms)
        avg_read = _stats(read_ms)[0]
        avg_cmd = _stats(cmd_ms)[0]
        result = {
            "iterations": len(loop_ms),
            "avg_period_ms": round(avg_loop, 2),
            "min_period_ms": round(min_loop, 2),
            "max_period_ms": round(max_loop, 2),
            "avg_read_ms": round(avg_read, 2),
            "avg_cmd_ms": round(avg_cmd, 2),
            "control_hz": round(1000.0 / avg_loop, 1) if avg_loop > 0 else 0.0,
            "moved": moved,
            "max_excursion_um": round(max_excursion, 1),
        }
        logger.info(
            "Control-loop rate: %.1f Hz (%.1f ms/cycle: read %.1f + cmd %.1f), "
            "moved=%s, excursion=%.0f µm",
            result["control_hz"], avg_loop, avg_read, avg_cmd,
            moved, max_excursion)
        return result

    # ── Shutdown ──────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Clean shutdown of all components."""
        logger.info("Shutting down StageController…")
        # v7.5.x: stop any in-flight ZP auto-reconnect from fighting shutdown.
        self._shutting_down = True
        self._watchdog.stop()
        self._pos_poller.stop()
        # v7.5.x: persist any throttled XY-odometer accumulation before exit.
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            get_store().flush()
        except Exception:
            pass
        # v7.5.x: turn the illumination LED off before the link goes away.
        # ORDERING IS THE WHOLE POINT: after the poller is stopped (so the write
        # doesn't contend for the ZP serial lock) but BEFORE disconnect_stages()
        # closes the port — afterwards there is no way left to reach the board.
        # Done synchronously on this thread, never on a worker: a write racing
        # serial.close() is a hard crash on Windows (see
        # MEBP_v75x_ZP_CLOSE_DURING_READ_CRASH.md). Do NOT rely on the
        # close-time DTR/RTS de-assert to do this — whether that pulses the
        # board's RESET (which would drop the fan output) is board- and
        # driver-polarity dependent, and on this rig the LED stays lit.
        self.led_off()
        self.disconnect_xbox()
        self.disconnect_stages()
        self.processor.stop()
        logger.info("Shutdown complete")


    # ══════════════════════════════════════════════════════════════
    #  v7.2: µL-Based Pump Control
    # ══════════════════════════════════════════════════════════════

    def set_hardware_config(self, config: HardwareConfig) -> None:
        """
        Set the hardware configuration. Called by MainWindow when
        hardware setup is completed or modified.

        Enables µL-based pump methods and updates safety limits.
        """
        self._hardware_config = config
        if config:
            for pid in ["P1", "P2", "P3"]:
                pump_cfg = config.pumps.get(pid)
                if pump_cfg and pump_cfg.is_configured:
                    logger.debug(f"{pid}: syringe={pump_cfg.syringe.volume_uL}µL")
        logger.info("StageController: hardware config updated")

        # Propagate to ZP jog handler for µL pump jog
        if self.zp_jog and hasattr(self.zp_jog, 'set_hardware_config'):
            self.zp_jog.set_hardware_config(config)

        # Update safety limits from hardware config. v7.5.x: skip the
        # syringe-stroke pump-limit overwrite for pumps whose envelope was set
        # by the plunger calibration (apply_pump_setup) — the calibration's
        # exact captured extremes must win over the coarse stroke estimate.
        if self.safety_limits:
            calibrated = [p for p in ("P1", "P2", "P3")
                          if self.is_pump_plunger_calibrated(p)]
            self.safety_limits.update_from_hardware_config(
                config, skip_pumps=calibrated)

        # v7.5.x: the pump jog speed-% is anchored to the configured flow
        # limits, which just changed — re-anchor (no-op if ZP not connected).
        try:
            self.refresh_jog_speed_limits()
        except Exception as e:
            logger.debug(f"refresh_jog_speed_limits (hw config) failed: {e}")

        # v7.5.x: gentle-Z near the plate — one distance + one speed (mm/s) drive
        # BOTH the slow first-mm LIFT and the slow last-mm DESCENT for every
        # workflow. dist 0 → disabled (single-speed, legacy). Configured on the
        # Common Print Settings page.
        try:
            dist = float(getattr(config, "gentle_z_slow_dist_mm", 1.0) or 0.0)
            spd = float(getattr(config, "gentle_z_slow_speed_mm_s", 1.0) or 0.0)
            fr = max(1.0, spd * 60.0)          # mm/s → mm/min
            self.set_retract_slow_lift(dist, fr)
            self.set_descend_slow_final(dist, fr)
        except Exception as e:
            logger.debug(f"gentle-Z config apply failed: {e}")

        # v7.5.x: when a selectable plate TYPE is active, adopt its standard
        # mm-below-fiducial offsets as the guess source for plate-Z estimates
        # (the calibration "Estimate plate Z" inherits them). A generic
        # selection leaves the device-profile offsets restored at boot intact.
        # Only the guess source moves here — the actual plate-bottom/top datum
        # is still pushed by the explicit Estimate click / taught values.
        self._apply_active_plate_type_offsets(config)

    def _apply_active_plate_type_offsets(self, config) -> None:
        """Push the active plate's learned ``z_offsets`` into
        ``_plate_z_offsets``.

        v7.12: resolves through ``HardwareConfig.plate_z_offsets()`` — a
        parametric DESIGN can own these too, not just a `PlateType` product.
        This used to read ``plate_type_id`` directly, so a custom plate's
        taught offsets could be neither saved nor adopted.

        No-op (keeps the device-profile / generic offsets) when the active
        plate has none. Guarded so a missing/corrupt library never breaks a
        hardware-config update.
        """
        try:
            if config is None:
                return
            getter = getattr(config, "plate_z_offsets", None)
            off = getter() if callable(getter) else {}
            if not off:
                return
            self.set_plate_z_offsets(
                top=off.get("top"), bottom=off.get("bottom"),
                safe=off.get("safe"), max=off.get("max"))
            logger.info(
                f"StageController: adopted plate Z offsets for "
                f"'{config.active_plate_key}' {self._plate_z_offsets}")
        except Exception as exc:   # pragma: no cover - defensive
            logger.debug(f"_apply_active_plate_type_offsets failed: {exc}")

    @property
    def hardware_config(self) -> HardwareConfig | None:
        """Get the current hardware configuration."""
        return self._hardware_config

    def pump_settle_time_s(self) -> float:
        """v7.5.x: configured global pump settle dwell (s), 0 if unset / no
        config. Applied BEFORE and AFTER each discrete pump actuation (see
        ``move_pump_uL(settle=True)``). Set on Hardware Setup → Pump."""
        cfg = self._hardware_config
        try:
            return max(0.0, float(getattr(cfg, "pump_settle_time_s", 0.0) or 0.0))
        except (TypeError, ValueError):
            return 0.0

    def pump_post_aspirate_dwell_s(self) -> float:
        """v7.21.7: configured global HOLD-IN-LIQUID dwell (s) after a reagent
        aspirate, 0 if unset / no config. Set on Hardware Setup → Pump (and the
        Common Print Settings page).

        This is NOT the same thing as :meth:`pump_settle_time_s`, and the
        difference is the whole point: the settle dwell brackets a pump move and
        the move itself already BLOCKS until the plunger has physically drained
        from Marlin's planner — so the *plunger* is provably finished. The fluid
        is not. A compliant column (fine bore, viscous ink, long tube) keeps
        drawing liquid in for a while after the plunger stops, so the needle has
        to STAY SUBMERGED for that tail. Retracting Z inside that window puts the
        tip in air and the tail of the aspirate becomes air.

        Consumed by :class:`PickPlaceExecutor` at every reagent aspirate (ink,
        oil, buffer) — which is where the needle is in liquid and a travel
        follows. Deliberately NOT applied to the streamed print path, to a
        dispense, or to manual jog."""
        cfg = self._hardware_config
        try:
            return max(0.0, float(
                getattr(cfg, "pump_post_aspirate_dwell_s", 0.0) or 0.0))
        except (TypeError, ValueError):
            return 0.0

    def pump_relief_uL(self, pump: str) -> float:
        """v7.5.x: per-pump compliance / "pressure relief" value in µL.

        This is HALF the aspirate-back volume measured by the Needle Location
        compliance calibration — the µL of plunger travel needed to remove the
        residual drivetrain/syringe flex in one direction. Used as the take-up /
        unload amount by the backlash-compensation engine (:meth:`move_pump_uL`).
        Returns 0 when the pump has not been calibrated (⇒ no compensation)."""
        try:
            return max(0.0, float(
                getattr(self, "_pump_relief_uL", {}).get(pump, 0.0) or 0.0))
        except (TypeError, ValueError):
            return 0.0

    def set_pump_relief_uL(self, pump: str, uL: float) -> None:
        """Set a pump's compliance / pressure-relief value (µL, clamped ≥ 0)."""
        if not hasattr(self, "_pump_relief_uL"):
            self._pump_relief_uL = {}
        try:
            self._pump_relief_uL[pump] = max(0.0, float(uL))
        except (TypeError, ValueError):
            self._pump_relief_uL[pump] = 0.0

    def get_pump_relief_all(self) -> dict:
        """Per-pump compliance / relief values (µL) — for persistence
        (``device_profile.pump_compliance_uL``)."""
        return {k: float(v)
                for k, v in getattr(self, "_pump_relief_uL", {}).items()}

    def backlash_comp_enabled(self) -> bool:
        """v7.5.x: whether backlash compensation (take-up on reversal + unload
        on stop) is applied at discrete pump start/stop. Toggled from the pump
        jog panel; persisted in ``device_profile.backlash_comp_enabled``."""
        return bool(getattr(self, "_backlash_comp_enabled", False))

    def set_backlash_comp_enabled(self, enabled: bool) -> None:
        """Enable / disable backlash compensation at pump start/stop."""
        self._backlash_comp_enabled = bool(enabled)

    def move_pump_uL(
        self, pump: str, volume_uL: float, rate_uL_s: float | None = None,
        *, settle: bool = False, compensate: bool | None = None,
        abort_event=None,
    ) -> None:
        """
        Move a pump by a specified volume in µL.

        ── Plunger conventions (single source of truth) ───────────────────
        ASPIRATE = draw fluid IN  (plunger toward MAX/full; raises fill level).
        DISPENSE = push fluid OUT (plunger toward ZERO/empty; lowers fill level).
        Volume-delta sign here: ``+`` = DISPENSE, ``−`` = ASPIRATE.
        Fill/position frame: 0 = empty (fully dispensed), capacity = full (fully
        aspirated). Per-pump dispense/aspirate DIRECTION is owned by the plunger
        calibration (:meth:`apply_pump_setup`) — the same way ``z_up_sign`` owns
        Z — and is applied in :meth:`move_pump_relative` via :meth:`pump_dir_sign`.
        Reserved terms NOT folded into this verb pair: 'prime' (start-of-print
        pre-flow lead-in) and 'retract' (Z safe-travel raise / pump pressure
        relief).
        ───────────────────────────────────────────────────────────────────

        This is the primary pump movement method for v7.2+.
        Converts µL → mm using the syringe spec, and µL/s → mm/min
        for the feedrate.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
            volume_uL: Volume to DISPENSE (+) or ASPIRATE (−) in µL
            rate_uL_s: Flow rate in µL/s. If None, uses default feedrate.
            settle: v7.5.x — when True this is a *discrete* actuation: sleep
                the configured ``pump_settle_time_s`` before the move, BLOCK
                for the (open-loop) move to complete, then sleep the settle
                time again after, so the caller's next step doesn't begin
                until the pump has finished and settled. Leave False for the
                streamed print path and manual jog (they pace themselves).
            compensate: v7.5.x — backlash / compliance compensation. When it
                resolves True, the fluid move is BRACKETED so the drivetrain
                flex is handled: TAKE-UP ``+c`` first (in the fluid direction,
                loads the flex so the commanded volume actually flows), the
                fluid move ``volume_uL``, then UNLOAD ``−c`` (releases the stored
                flex → pressure-neutral tip before the stage travels). ``c`` =
                the per-pump calibrated :meth:`pump_relief_uL` (½ the
                aspirate-back volume). Net plunger = ``volume_uL``; net fluid ≈
                ``volume_uL``. ``None`` (default) ⇒ auto: comp iff ``settle`` is
                True AND :meth:`backlash_comp_enabled`. ``True`` forces it on,
                ``False`` forces it off. Tying auto-comp to ``settle=True`` keeps
                the streamed print path (``settle=False`` per-segment) and manual
                jog untouched; volume-balanced pick&place micro-captures pass
                ``compensate=False`` to keep their exact nL net-zero balance.
            abort_event: v7.6 — optional ``threading.Event``. When set, the
                blocking drain/settle waits return at once (so an abort unwinds
                in ~one readline instead of up to 180 s per sub-move) and the
                remaining backlash sub-moves are SKIPPED — nothing new is
                commanded once an abort is in flight. Default None =
                byte-identical.

        Raises:
            ValueError: If pump has no syringe configured
        """
        def _aborting() -> bool:
            return abort_event is not None and abort_event.is_set()

        if not self._hardware_config:
            raise ValueError("No hardware config — complete Hardware Setup first")

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            raise ValueError(f"{pump}: No syringe configured")

        # Convert µL → mm
        distance_mm = pump_cfg.uL_to_mm(volume_uL)

        # Convert rate µL/s → mm/min
        feedrate_mm_min = None
        if rate_uL_s is not None:
            feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(rate_uL_s))
            # Clamp flow rate via safety limits
            if self.safety_limits.enabled:
                clamped = self.safety_limits.clamp_flow_rate(rate_uL_s, pump)
                if abs(clamped) != abs(rate_uL_s):
                    feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(clamped))

        logger.debug(
            f"move_pump_uL({pump}, {volume_uL:+.3f} µL"
            + (f", {rate_uL_s:.3f} µL/s" if rate_uL_s else "")
            + f") → {distance_mm:+.5f} mm"
            + (f", {feedrate_mm_min:.1f} mm/min" if feedrate_mm_min else "")
        )

        # v7.5.x: backlash / compliance compensation. Resolve whether to bracket
        # this fluid move with a flex take-up (before) + unload (after). Auto
        # (compensate=None) fires only for discrete actuations (settle=True) when
        # the global toggle is on — so the streamed print path (settle=False) and
        # manual jog are never auto-bracketed here. The take-up/unload recurse
        # with compensate=False (base case), so they never re-bracket.
        do_comp = (compensate if compensate is not None
                   else (settle and self.backlash_comp_enabled()))
        c_uL = self.pump_relief_uL(pump) if do_comp else 0.0
        comp = do_comp and c_uL > 0 and volume_uL != 0
        comp_dir = 1.0 if volume_uL > 0 else -1.0
        # Dwell duration + whether to block each sub-move for completion. The
        # settle dwell fires on engagement state transitions — INTO engaged
        # (after take-up) and INTO neutral (after unload) — and after a plain
        # settled non-compensated move. It applies to any discrete actuation
        # (settle=True) AND any compensated move, so a forced compensate=True
        # move dwells + blocks even when settle=False.
        settle_s = self.pump_settle_time_s() if (settle or comp) else 0.0
        wait_complete = settle or comp

        if comp:
            # TAKE-UP (neutral → engaged): load the flex in the fluid direction
            # (no net fluid) so the commanded volume actually flows. settle=False
            # so the recursion just issues the move; we block + dwell HERE so the
            # drivetrain settles INTO the engaged state before the fluid move.
            logger.debug(f"move_pump_uL({pump}): backlash take-up "
                         f"{c_uL * comp_dir:+.4f} µL")
            self.move_pump_uL(pump, c_uL * comp_dir, rate_uL_s=rate_uL_s,
                              settle=False, compensate=False,
                              abort_event=abort_event)
            self._finish_pump_submove(c_uL, rate_uL_s,
                                      block=wait_complete, dwell_s=settle_s,
                                      abort_event=abort_event)

        # MAIN fluid move (stays engaged / positive flow). Block so the caller
        # (or the unload below) does not advance while the pump is still moving —
        # e.g. a following safe_travel_to whose Z-retract M400 would otherwise
        # have to absorb a still-running pump move and time out. No dwell here:
        # the main move is not a neutral↔engaged transition.
        # v7.6: an abort raised during the take-up must not command the fluid
        # move — the whole point is to stop adding motion.
        if _aborting():
            logger.info(f"move_pump_uL({pump}): abort_event set — "
                        f"skipping the remaining sub-moves")
            return
        self.move_pump_relative(pump, distance_mm, feedrate_mm_min)
        self._finish_pump_submove(volume_uL, rate_uL_s,
                                  block=wait_complete, dwell_s=0.0,
                                  abort_event=abort_event)

        if comp and _aborting():
            logger.info(f"move_pump_uL({pump}): abort_event set — "
                        f"skipping the backlash unload")
            return
        if comp:
            # UNLOAD (engaged → neutral): release the stored flex by moving −c
            # (opposite the fluid direction) so the tip is pressure-neutral, then
            # dwell so the released flex equilibrates INTO the neutral state. Net
            # fluid ≈ the commanded volume (take-up + unload cancel in net plunger
            # travel). Subsumes the old direction-aware "pressure relief"
            # (dispense-back after an aspirate / suck-back after a dispense).
            logger.debug(f"move_pump_uL({pump}): backlash unload "
                         f"{-c_uL * comp_dir:+.4f} µL")
            self.move_pump_uL(pump, -c_uL * comp_dir, rate_uL_s=rate_uL_s,
                              settle=False, compensate=False,
                              abort_event=abort_event)
            self._finish_pump_submove(c_uL, rate_uL_s,
                                      block=wait_complete, dwell_s=settle_s,
                                      abort_event=abort_event)
        elif settle_s > 0 and not _aborting():
            # Plain settled move (no compensation): dwell after it drains so the
            # caller does not advance until the pump has settled.
            time.sleep(settle_s)

    def move_pumps_uL(
        self, volumes_uL: dict[str, float], rate_uL_s: float | None = None,
        *, settle: bool = False, abort_event=None,
        delivered: dict | None = None,
    ) -> bool:
        """Move SEVERAL pumps **simultaneously** in one coordinated Marlin move.

        Volume signs follow :meth:`move_pump_uL` exactly (``+`` = DISPENSE,
        ``−`` = ASPIRATE) and each pump keeps its own calibrated direction
        (:meth:`pump_dir_sign`), its own µL→mm scale, its own soft-limit
        envelope and its own flow ceiling.

        ── Why this is NOT "call move_pump_uL twice" ──────────────────────
        Two calls emit two ``G0`` blocks and Marlin executes blocks in ORDER, so
        the bores would silently run one after another — the opposite of the
        intent. Three further hazards make the naive version actively unsafe:
        ``_wait_pump_move_complete``'s M400 drains *every* queued move while
        sizing its timeout for ONE of them (spurious timeouts); the poller
        suspend is a plain bool, NOT refcounted, so whichever caller finishes
        first re-enables the poller under the other (→ the v7.5.x false
        "ZP disconnected"); and ``ZPStage._serial_lock`` is held across the whole
        write→``ok`` transaction anyway, so the two calls serialise on the bus.
        One ``G0`` naming several axes is a single planner block: every named
        axis starts and stops together. That is the same primitive the Xbox ZP
        jog loop has driven on real hardware since v7.5.x (one ``move_relative``
        with a combined feedrate over Z + all three pumps).

        ── ONE FEEDRATE FOR THE WHOLE VECTOR — read before reusing ────────
        A coordinated move has a single feedrate applied to the move VECTOR, so
        axis *i* runs at ``F · |Δ_i| / L`` (``L`` = the move length Marlin's
        feedrate applies to — see :func:`_marlin_move_length_mm`). **You cannot
        give one bore 0.5 µL/s and another 5.0 µL/s in the same simultaneous
        move.** All axes necessarily take the same time. That is exactly right
        for needle prep — every used bore doing the same thing at once — and
        WRONG for anything asymmetric (the cell-removal slow-push / fast-pull,
        per-bore dwell sequences): those stay sequential single-pump moves.

        Because the axes share a duration, the vector rate is set by the most
        constrained axis: ``F = min_i(f_i · L / |Δ_i|)``, where ``f_i`` is pump
        *i*'s own permitted plunger feedrate (mm/min) after the per-pump flow
        clamp. Every other axis then runs BELOW its own ceiling — the safe
        direction. A fine bore whose ceiling is low therefore drags the whole
        vector down; over-pressuring a pulled glass tip is not a trade we make
        for speed.

        Backlash / compliance compensation is deliberately **NOT** applied: the
        take-up/unload bracket is per-axis-and-direction and ill-defined when
        one bore dispenses while another aspirates. Same convention the
        volume-balanced pick&place captures already use (``compensate=False``).
        A caller that needs compensation must issue sequential single-pump
        moves.

        Args:
            volumes_uL: ``{pump_id: µL}``, e.g. ``{"P1": -5.0, "P2": -5.0}``.
                Entries that are ``None``/``0`` are ignored.
            rate_uL_s: Flow rate in µL/s applied to EVERY pump (the vector is
                sized so no pump exceeds it or its own ceiling). ``None`` ⇒ no
                explicit feedrate, exactly like :meth:`move_pump_uL`, which
                leaves Marlin on ``ZPStage.feedrate``; pass a rate for prep.
            settle: As :meth:`move_pump_uL` — block until the coordinated move
                has physically drained, then dwell ``pump_settle_time_s``.
            abort_event: Optional ``threading.Event``; forwarded to the drain
                wait so an abort unwinds promptly.
            delivered: Optional dict, **populated in place** with
                ``{pump_id: µL actually commanded}`` after soft-limit shortening
                and the sub-Marlin-step drop. Exists because the bool return
                cannot express "True, but nothing was delivered" — a caller that
                must know a prep volume really moved (an unconditioned bore
                doses AIR) passes a dict and compares. Kept as an out-param
                rather than a richer return so the documented ``bool`` contract
                and its tests are untouched.

        Returns:
            True when the coordinated move was commanded — or when there was
            genuinely nothing to move (a clean no-op: every volume zero, or
            every delta shortened below one Marlin step, both logged). False
            when the move was REFUSED (no ZP board, or ``abort_event`` already
            set): nothing moved, so the caller must not assume the volumes were
            delivered. Anything the soft-limit clamp shortens away is logged
            with requested-vs-delivered µL rather than silently assumed.

        Raises:
            ValueError: no hardware config; a named pump has no syringe (same
                contract as :meth:`move_pump_uL` — naming an unconfigured pump
                is a caller bug, not a runtime condition); a named pump has no
                axis mapping on this machine; or two named pumps map onto the
                SAME Marlin motor (their deltas would collide in the one
                ``G0``, silently dropping one bore).
        """
        if not self._hardware_config:
            raise ValueError("No hardware config — complete Hardware Setup first")

        requested = {p: float(v) for p, v in (volumes_uL or {}).items()
                     if v is not None and float(v) != 0.0}
        if not requested:
            # Nothing asked for. Never fall through to a bare "G0 F…" with no
            # axes — Marlin would treat it as a zero-length move.
            logger.debug("move_pumps_uL: no non-zero volumes — no-op")
            return True

        zp = getattr(self, "zp_stage", None)
        if zp is None or not self.is_zp_connected:
            logger.warning(
                f"move_pumps_uL: ZP not connected — refusing "
                f"{{{', '.join(f'{p}:{v:+.3f} µL' for p, v in requested.items())}}}")
            return False
        if abort_event is not None and abort_event.is_set():
            # v7.6 principle (see move_pump_uL): nothing NEW is commanded once an
            # abort is in flight. Refuse rather than emit-then-skip-the-wait, so
            # the caller cannot assume these volumes were delivered.
            logger.info("move_pumps_uL: abort_event set — commanding nothing")
            return False

        limits = getattr(self, "safety_limits", None)
        limits_on = bool(getattr(limits, "enabled", False))
        # Cached ZP position for the soft-limit clamp — read ONCE so every axis
        # is clamped against the same frame (and we don't add serial traffic).
        try:
            pos = self.get_zp_position(cached=True)
        except Exception:
            pos = None

        deltas: dict[str, float] = {}        # physical letter → raw mm
        per_axis_feed: dict[str, float] = {} # physical letter → own max mm/min
        by_letter: dict[str, str] = {}       # physical letter → pump id
        # pump id → µL actually commanded. When the caller supplied a dict we
        # populate THAT one in place, so it sees the same accounting the log does.
        if delivered is None:
            delivered = {}
        else:
            delivered.clear()
        for pump, volume_uL in requested.items():
            pump_cfg = self._hardware_config.pumps.get(pump)
            if not pump_cfg or not pump_cfg.is_configured:
                raise ValueError(f"{pump}: No syringe configured")
            letter = _axis_letter(zp, pump)
            if letter is None:
                # Skipping would silently under-deliver a prep volume. Refuse.
                raise ValueError(
                    f"move_pumps_uL: {pump} has no axis mapping on this machine")
            if letter in deltas:
                # Two pumps mapped onto the same motor: their deltas would
                # collide in the one G0. Refuse rather than silently drop one.
                raise ValueError(
                    f"move_pumps_uL: {pump} and {by_letter[letter]} both map to "
                    f"Marlin '{letter}' — cannot move them independently")

            # µL → mm of dispense intent, then the calibration-owned raw sign
            # (identical chain to move_pump_uL → move_pump_relative).
            dir_sign = self.pump_dir_sign(pump)
            raw_mm = pump_cfg.uL_to_mm(volume_uL) * dir_sign

            # Per-pump flow ceiling, then µL/s → mm/min, then the per-pump
            # plunger-feedrate ceiling. This is the fastest THIS axis may run.
            feed_mm_min = None
            if rate_uL_s is not None:
                eff_rate = abs(float(rate_uL_s))
                if limits_on:
                    eff_rate = abs(limits.clamp_flow_rate(eff_rate, pump))
                feed_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(eff_rate)
                if limits_on:
                    feed_mm_min = limits.clamp_pump_feedrate(feed_mm_min, pump)

            # Absolute-raw soft-limit clamp, shorten-only (see
            # _shorten_only_delta — an out-of-bounds cached position must never
            # synthesise a large opposite-direction move).
            if limits_on and pos and pos[0] is not None:
                try:
                    idx = _axis_index(zp, pump)
                    if idx is not None and idx < len(pos) and pos[idx] is not None:
                        cur = pos[idx]
                        raw_mm = _shorten_only_delta(
                            cur, raw_mm, limits.clamp_pump(cur + raw_mm, pump))
                except Exception:
                    pass

            deltas[letter] = raw_mm
            by_letter[letter] = pump
            if feed_mm_min is not None:
                # NO floor here. Marlin's "never emit F ≤ 0" floor belongs on the
                # VECTOR feedrate (applied once, below) — NOT on a per-axis
                # ceiling. In a coordinated move an axis legitimately runs slower
                # than 1 mm/min while the vector runs far faster, so flooring the
                # ceiling would raise it and let that bore exceed its own flow
                # limit (a 0.09 mm/min pulled-glass tip alongside a fast bore
                # would be driven at 1.0 mm/min = 11×). Over-pressuring a glass
                # tip is not a trade we make for speed.
                per_axis_feed[letter] = max(float(feed_mm_min), 0.0)
            # Honest accounting: report what the CLAMP left, not what was asked.
            delivered[pump] = (pump_cfg.mm_to_uL(raw_mm / dir_sign)
                               if dir_sign else 0.0)

        # ZPStage.move_relative drops sub-resolution deltas (< 1e-4 mm = 0.1 µm,
        # below one Marlin step). Drop them HERE too so the feedrate maths and
        # the move length see the same axes the board will, and so an all-tiny
        # request is a no-op instead of a bare G0.
        active = {a: d for a, d in deltas.items() if abs(d) >= 1e-4}
        dropped = [by_letter[a] for a in deltas if a not in active]
        if dropped:
            # Report what was asked vs what the board will actually receive —
            # a soft-limit clamp to nothing and a genuinely sub-step request
            # both land here, and neither must be silently assumed delivered.
            logger.warning(
                "move_pumps_uL: below one Marlin step after clamping, NOT "
                "delivered: "
                + ", ".join(f"{p} requested {requested[p]:+.4f} µL → "
                            f"{delivered.get(p, 0.0):+.6f} µL"
                            for p in dropped))
            for p in dropped:
                delivered.pop(p, None)
        # A PARTIAL shortening (0.9 mm → 0.6 mm against the envelope) under-
        # delivers a prep volume just as silently, and SafetyLimits' own warning
        # only names raw mm. Report requested-vs-delivered µL for those too, so
        # the promise made in the docstring holds for every clamped axis, not
        # just the ones shortened all the way to nothing.
        short = [p for p, v in delivered.items()
                 if abs(v - requested[p]) > 1e-4]
        if short:
            logger.warning(
                "move_pumps_uL: soft-limit SHORTENED, less delivered than "
                "requested: "
                + ", ".join(f"{p} requested {requested[p]:+.4f} µL → "
                            f"{delivered[p]:+.4f} µL" for p in short))
        if not active:
            return True

        # Vector feedrate: the most constrained axis sets the pace (see the
        # docstring's derivation). F = min_i(f_i · L / |Δ_i|).
        length_mm = _marlin_move_length_mm(active)
        feedrate = None
        bound_by = None
        if per_axis_feed and length_mm > 0:
            for a, d in active.items():
                f_axis = per_axis_feed.get(a)
                if f_axis is None:
                    continue
                cap = f_axis * length_mm / abs(d)
                if feedrate is None or cap < feedrate:
                    feedrate, bound_by = cap, by_letter[a]
            if feedrate is not None:
                feedrate = max(feedrate, 1.0)

        logger.debug(
            "move_pumps_uL: "
            + ", ".join(f"{by_letter[a]}({a}) {delivered.get(by_letter[a], 0.0):+.3f} µL "
                        f"→ {d:+.5f} mm" for a, d in active.items())
            + f" | vector {length_mm:.5f} mm"
            + (f" @ {feedrate:.1f} mm/min (bound by {bound_by})"
               if feedrate else " @ board default feedrate"))

        self.zp_stage.move_relative(active, feedrate)
        for a, d in active.items():
            # Display-only motion estimator, per logical axis (as the
            # single-pump path does in move_pump_relative). It wants THIS axis's
            # own speed, which in a coordinated move is F·|Δ|/L — not the vector
            # feedrate (that would over-state every axis and show the move
            # finishing early; with two equal bores it is 1.41× optimistic).
            axis_fr = (feedrate * abs(d) / length_mm
                       if feedrate and length_mm > 0 else feedrate)
            self._note_move_estimate_axis_rel(by_letter[a], d, axis_fr)

        if settle:
            # The coordinated duration is L/F — equivalently the LONGEST of the
            # individual axis durations, since F was sized by the binding axis.
            # Derive it from the vector, not from any one pump's volume/rate.
            if feedrate:
                move_s = length_mm / (feedrate / 60.0) + 0.1
            else:
                eff_rate = (abs(float(rate_uL_s)) if rate_uL_s
                            else _PUMP_SETTLE_FALLBACK_RATE_UL_S)
                move_s = (max((abs(v) for v in delivered.values()), default=0.0)
                          / max(eff_rate, 0.001) + 0.1)
            self._finish_pump_submove(
                0.0, rate_uL_s, block=True,
                dwell_s=self.pump_settle_time_s(),
                abort_event=abort_event, est_s=move_s)
        return True

    def _wait_pump_move_complete(self, move_s: float,
                                 abort_event=None) -> bool | None:
        """Block until the pump's in-flight move drains from Marlin's planner.

        Confirms motion-complete via M400 (``ZPStage.flush_moves``) — the same
        mechanism :meth:`safe_travel_to` uses for Z — with a timeout scaled to
        the estimated move duration, so a long but legitimate discrete pump
        actuation (e.g. a multi-needle buffer aspirate, ~27 s at 1 µL/s) is
        fully drained before the caller advances, instead of bleeding into the
        next safe-travel M400 and tripping its 15 s timeout. The poller is
        suspended for the wait so its M114 reads don't contend for the bus.

        Returns:
            True  — Marlin confirmed completion (or there is no real board / sim).
            False — flush_moves was available but timed out (it already waited
                    ~move_s, so the caller should NOT sleep again).
            None  — no flush_moves available (fake / older controller); the
                    caller should fall back to an open-loop sleep.
        """
        zp = getattr(self, "zp_stage", None)
        flush = getattr(zp, "flush_moves", None)
        if not callable(flush):
            return None
        timeout_s = min(max(move_s + _PUMP_MOVE_DRAIN_MARGIN_S,
                            _PUMP_MOVE_DRAIN_MARGIN_S),
                        _PUMP_MOVE_DRAIN_TIMEOUT_CAP_S)
        self.suspend_position_poller()
        try:
            try:
                return bool(flush(timeout_s=timeout_s,
                                  abort_event=abort_event))
            except TypeError:       # fake/older flush without abort_event
                return bool(flush(timeout_s=timeout_s))
        except Exception as e:                  # never let a confirm failure crash a prep
            logger.warning(f"_wait_pump_move_complete: flush_moves error: {e}")
            return None
        finally:
            self.resume_position_poller()

    def _finish_pump_submove(self, vol_uL: float, rate_uL_s: float | None,
                             *, block: bool, dwell_s: float,
                             abort_event=None, est_s: float | None = None) -> None:
        """Settle one just-issued pump sub-move (take-up / main / unload).

        When ``block`` is True, wait for the sub-move to PHYSICALLY drain from
        Marlin's planner (via :meth:`_wait_pump_move_complete`, falling back to
        an open-loop sleep of the estimated duration when no board can confirm)
        so the caller does not advance while the pump is still moving. Then, if
        ``dwell_s > 0``, sleep the configured settle dwell so the drivetrain
        equilibrates at its new neutral/engaged state before the next step.

        v7.6 ``abort_event``: forwarded to the drain wait; when set the
        open-loop fallback sleep is chunked + interruptible and the settle
        dwell is skipped.

        v7.9 ``est_s``: explicit duration estimate (s) for the drain wait. A
        COORDINATED multi-pump move (:meth:`move_pumps_uL`) has ONE vector
        feedrate, so its duration cannot be derived from any single pump's
        volume/rate pair — the caller computes it from the move vector and
        passes it here. ``None`` (default) ⇒ derive from ``vol_uL``/``rate_uL_s``
        exactly as before (byte-identical for every existing caller).
        """
        aborted = abort_event is not None and abort_event.is_set()
        if block and not aborted:
            eff_rate = (abs(rate_uL_s) if rate_uL_s
                        else _PUMP_SETTLE_FALLBACK_RATE_UL_S)
            sub_s = (float(est_s) if est_s is not None
                     else abs(vol_uL) / max(eff_rate, 0.001) + 0.1)
            if self._wait_pump_move_complete(sub_s,
                                             abort_event=abort_event) is None:
                total = min(sub_s, _PUMP_MOVE_DRAIN_TIMEOUT_CAP_S)
                if abort_event is None:
                    # No abort channel → the legacy single open-loop sleep
                    # (byte-identical; regression-locked by the pump-settle
                    # suite, which counts sleep calls).
                    time.sleep(total)
                else:
                    # Chunked so an abort interrupts the wait. Bounded by a
                    # fixed chunk COUNT, not the wall clock, so a patched /
                    # no-op sleep cannot spin.
                    chunk = 0.1
                    for _ in range(int(total / chunk) + 1):
                        if abort_event.is_set():
                            return
                        time.sleep(min(chunk, total))
                        total -= chunk
                        if total <= 0:
                            break
            aborted = abort_event is not None and abort_event.is_set()
        if dwell_s > 0 and not aborted:
            time.sleep(dwell_s)

    def get_pump_position_uL(self, pump: str) -> float | None:
        """
        Get the current pump position in µL (relative to zero reference).

        Returns None if unavailable or no syringe configured.
        """
        if not self._hardware_config:
            return None

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            return None

        pos = self.get_zp_position(cached=True)
        if pos is None or pos[0] is None:
            return None

        # v7.4.2 hotfix: respect live axis_map.
        idx = _axis_index(self.zp_stage, pump)
        if idx is None or idx >= len(pos):
            return None

        pos_mm = pos[idx]
        zero_ref = self.zero_position.get(pump, 0)
        relative_mm = pos_mm - zero_ref

        try:
            return pump_cfg.mm_to_uL(relative_mm)
        except ValueError:
            return None

    def pump_volume_to_reach_uL(self, pump: str,
                                target_position_uL: float) -> float | None:
        """Signed :meth:`move_pump_uL` volume (µL, ``+`` = dispense) that makes
        :meth:`get_pump_position_uL` equal ``target_position_uL``.

        ── POLARITY (single source of truth) ──────────────────────────────
        ``move_pump_uL`` applies :meth:`pump_dir_sign` to the volume, but
        ``get_pump_position_uL`` does NOT, so a ``move_pump_uL(V)`` changes the
        reported position by ``pump_dir_sign × V``. To drive the reported
        position from ``current`` to ``target`` therefore requires
        ``V = pump_dir_sign × (target − current)`` — NOT simply
        ``target − current`` (which is only correct when ``pump_dir_sign == +1``
        and drives the plunger the WRONG way on a pump calibrated to
        ``pump_dir_sign == −1``). Use this helper to "return the plunger to a
        captured position" instead of hand-rolling the difference.
        ───────────────────────────────────────────────────────────────────

        Returns None if the current position is unreadable.
        """
        current = self.get_pump_position_uL(pump)
        if current is None:
            return None
        try:
            sign = float(self.pump_dir_sign(pump))
        except Exception:
            sign = 1.0
        return sign * (float(target_position_uL) - float(current))

    def move_pump_to_position_uL(self, pump: str, target_position_uL: float,
                                 rate_uL_s: float | None = None,
                                 *, settle: bool = True) -> float | None:
        """Move the plunger so :meth:`get_pump_position_uL` reaches
        ``target_position_uL`` (polarity-correct via
        :meth:`pump_volume_to_reach_uL`). Returns the volume moved (µL), or None
        if the current position is unreadable (no move issued)."""
        vol = self.pump_volume_to_reach_uL(pump, target_position_uL)
        if vol is None:
            return None
        if abs(vol) > 1e-6:
            self.move_pump_uL(pump, vol, rate_uL_s, settle=settle)
        return vol

    def simulate_pump_budget(self, pump: str, moves_uL, *,
                             start_fill_uL: float | None = None,
                             extra_min_fill_uL: float | None = None) -> dict:
        """Pre-flight a planned pump-move sequence against this pump's calibrated
        envelope. Resolves the live plunger fill (``pump_fill_uL``) and capacity
        (``pump_capacity_uL``) and delegates to :func:`compute_pump_budget`.

        Returns the :func:`compute_pump_budget` dict on success, or a dict with
        ``ok=False`` and a ``reason`` of ``'uncalibrated'`` (no capacity) /
        ``'fill_unreadable'`` (no live position) when it can't simulate — the
        caller then skips the budget gate rather than blocking a valid print."""
        cap = self.pump_capacity_uL(pump)
        if cap is None or cap <= 0:
            return {"ok": False, "reason": "uncalibrated", "capacity_uL": cap}
        if start_fill_uL is None:
            start_fill_uL = self.pump_fill_uL(pump)
        if start_fill_uL is None:
            return {"ok": False, "reason": "fill_unreadable",
                    "capacity_uL": cap}
        result = compute_pump_budget(
            moves_uL, float(start_fill_uL), float(cap),
            extra_min_fill_uL=extra_min_fill_uL)
        result["reason"] = "ok" if result["ok"] else "out_of_bounds"
        return result

    def get_all_pump_positions_uL(self) -> dict[str, float | None]:
        """Get all pump positions in µL as a dict."""
        return {pid: self.get_pump_position_uL(pid) for pid in ["P1", "P2", "P3"]}

    def dispense_uL(
        self, pump: str, volume_uL: float, rate_uL_s: float | None = None
    ) -> bool:
        """
        DISPENSE (push fluid out) / ASPIRATE a specific volume with fluid-column
        tracking. v7.5.x: renamed from ``extrude_uL``.

        Positive volume_uL = DISPENSE (push out), negative = ASPIRATE (draw in).
        Updates FluidColumn in HardwareConfig if available.

        Returns True if executed successfully.
        """
        if not self._hardware_config:
            logger.error("No hardware config — cannot dispense")
            return False

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            logger.error(f"{pump}: not configured — cannot dispense")
            return False

        # Track fluid column
        if volume_uL > 0:
            if not pump_cfg.fluid_column.can_dispense(volume_uL):
                logger.warning(
                    f"{pump}: Requested {volume_uL:.2f} µL but only "
                    f"{pump_cfg.fluid_column.ink_volume_uL:.2f} µL available"
                )
            pump_cfg.fluid_column.dispense(volume_uL)
        elif volume_uL < 0 and pump_cfg.ink:
            pump_cfg.fluid_column.aspirate_ink(abs(volume_uL), pump_cfg.ink)

        self.move_pump_uL(pump, volume_uL, rate_uL_s)
        return True

    # v7.5.x: back-compat alias (renamed to dispense_uL). Kept so any external
    # script/caller still resolves; new code should use dispense_uL.
    def extrude_uL(self, pump: str, volume_uL: float,
                   rate_uL_s: float | None = None) -> bool:
        return self.dispense_uL(pump, volume_uL, rate_uL_s)