"""SimplePrintManager — a deliberately MINIMAL, fully-confirmed print executor.

This is a debug baseline for the ZP-disconnect investigation. The hypothesis:
now that the serial protocol is sound (synchronous Marlin ``ok``, atomic
``flush_moves``, explicit feedrate on every move) the elaborate workarounds in
the full :class:`PrintManager` (open-loop per-segment streaming, planner-buffer
barriers, drift pacing, poller/watchdog suspension) may no longer be needed. So
this executor runs a print the simplest possible way — a straight, blocking,
**every-move-CONFIRMED** sequence:

    retract → confirm   →   XY → confirm   →   descend → confirm   →   [prime]
    →  for each segment:  extrude + XY,  then M400 + XY-settle  →  retract → confirm

Because every segment is confirmed (M400 on the ZP board + arrival poll on XY),
the Marlin planner buffer never holds more than one queued move per board, so it
can NEVER saturate — no barriers, no pacing, no buffer bookkeeping required.

It KEEPS every safety feature (these are non-negotiable — see CLAUDE.md):
  * retract to the safe/travel Z and CONFIRM it before any cross-position XY move
  * descend with an EXPLICIT insert feedrate and CONFIRM arrival (M400 + M114)
  * EXPLICIT feedrate on every move (never inherit the pump's slow modal F)
  * ABORT the print if a Z move can't be confirmed (never extrude at the wrong Z)
  * ABORT if the ZP board drops mid-print (never dry-run / drag a dead board)
  * plate-bottom Z floor armed for the whole run (never punch through the plate)
  * ALWAYS end at the safe/travel Z, on completion / error / abort

It DELIBERATELY OMITS (add back incrementally as each version proves out on HW):
  * open-loop per-segment streaming + planner-buffer barriers + drift pacing
  * poller / port-health-watchdog suspension during the path
  * hybrid / trajectory / velocity / service-sequence executors
  * resume-from-saved, PrintRecorder, JSONL execution log, print history

Drop-in interface (``load_job`` / ``start`` / ``abort`` / ``state`` /
``on_progress``) so it can replace :class:`PrintManager` in the Stress Test and
Quick Print for A/B testing.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Callable, Optional

# Reuse the existing data model + enums so this is a true drop-in.
from SupportClasses.PrintManager import (
    PrintState, CommandType, PrintJob,
)

logger = logging.getLogger(__name__)


class SimplePrintManager:
    """Minimal, fully-confirmed executor for a ``PrintJob`` command plan.

    Consumes the SAME ``PrintJob`` produced by ``build_well_plate_job`` (so the
    geometry/ordering — including the v7.5.x retract-first plan shape — is the
    trusted one), but executes it with a tiny, blocking, confirm-everything
    interpreter instead of the full PrintManager's streaming path.
    """

    # Confirmation timeouts (s). Generous but bounded — a real Z move at the
    # configured feedrate completes well within these; a genuinely stuck/dropped
    # board fails fast via wait_for_z_arrival's _last_position_read_ok streak.
    _Z_TIMEOUT_S = 12.0
    _XY_TIMEOUT_S = 15.0
    _SEG_TOL_MM = 1e-3

    def __init__(self, controller):
        self.controller = controller
        self.job: Optional[PrintJob] = None
        self.state = PrintState.IDLE
        self._active_pump = "P1"

        # Callbacks (same signatures as PrintManager).
        self.on_progress: Optional[Callable] = None          # (step, total, msg)
        self.on_state_changed: Optional[Callable] = None      # (PrintState)

        self._thread: Optional[threading.Thread] = None
        self._abort_flag = threading.Event()
        self._current_step = 0
        self._zp_connected_at_start = False

    # ── Job management ─────────────────────────────────────────────

    def load_job(self, job: PrintJob):
        if self.state == PrintState.RUNNING:
            raise RuntimeError("Cannot load job while printing")
        self.job = job
        self._current_step = 0
        self._active_pump = "P1"
        self._set_state(PrintState.IDLE)
        logger.info("SimplePrintManager: loaded '%s' (%d commands)",
                    job.name, job.total_steps)

    # ── Execution control ──────────────────────────────────────────

    def start(self):
        if self.job is None:
            raise RuntimeError("No print job loaded")
        if self.state == PrintState.RUNNING:
            logger.warning("SimplePrintManager: already running")
            return
        self._abort_flag.clear()
        self._current_step = 0
        # Arm the plate-bottom floor for the whole run (disarmed in finally).
        self._arm_print_floor(True)
        # Set RUNNING BEFORE spawning so a fast job can't finish before the
        # caller's `while state == RUNNING` loop starts (no start race).
        self._set_state(PrintState.RUNNING)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def abort(self):
        if self.state not in (PrintState.RUNNING, PrintState.PAUSED):
            return
        self._abort_flag.set()
        self._set_state(PrintState.ABORTED)
        logger.info("SimplePrintManager: abort requested")

    # ── Main loop ──────────────────────────────────────────────────

    def _run(self):
        ctrl = self.controller
        job = self.job
        logger.info("SimplePrintManager: starting print '%s'", job.name)
        self._zp_connected_at_start = bool(
            getattr(ctrl, "is_zp_connected", False))
        try:
            for i, cmd in enumerate(job.commands):
                if self._abort_flag.is_set():
                    logger.info("SimplePrintManager: aborted at step %d", i + 1)
                    return
                # Bail if the ZP board dropped mid-print (don't dry-run it).
                if (self._zp_connected_at_start
                        and not getattr(ctrl, "is_zp_connected", True)):
                    raise RuntimeError(
                        f"ZP disconnected mid-print at step {i + 1}")
                self._current_step = i + 1
                self._report_progress(
                    f"[{i + 1}/{job.total_steps}] {cmd.label or cmd.type.value}")
                self._exec(cmd)

            self._set_state(PrintState.COMPLETED)
            self._report_progress("Print complete!")
            logger.info("SimplePrintManager: print '%s' completed", job.name)
        except Exception as e:
            logger.error("SimplePrintManager: print error: %s", e, exc_info=True)
            self._set_state(PrintState.ERROR)
            self._report_progress(f"Error: {e}")
        finally:
            # CRITICAL SAFETY: always leave the needle at the safe/travel Z,
            # however the print ended.
            self._retract_to_safe_z()
            self._arm_print_floor(False)

    # ── Command interpreter (only the types build_well_plate_job emits) ──

    def _exec(self, cmd):
        ctrl = self.controller
        settings = self.job.settings
        p = cmd.params
        t = cmd.type

        if t == CommandType.COMMENT:
            logger.debug("SimplePrintManager: %s", cmd.label)
            return

        if t == CommandType.TRAVEL_UP:
            self._safe_retract(settings.travel_z_height)
            return

        if t == CommandType.HOME_XY:
            # Return to zero reference — cross-position travel: retract first.
            self._safe_retract(settings.travel_z_height)
            self._confirmed_xy(0.0, 0.0,
                               getattr(settings, "travel_speed_mm_s", 10.0))
            return

        if t == CommandType.MOVE_XY:
            # Cross-position travel: retract (full travel Z, or a small intra-
            # well hop if requested) and CONFIRM before moving XY.
            hop_z = p.get("hop_z", None)
            self._safe_retract(float(hop_z) if hop_z is not None
                               else settings.travel_z_height)
            # v7.20 CRITICAL SAFETY: the NEXT command in a well-plate plan is
            # MOVE_Z — the descent. Continuing with XY unconfirmed lowers the
            # needle at an unverified position and breaks it against the plate.
            # Raise so _execute_loop's finally retracts to safe Z; the needle is
            # still retracted here, so this stops in the safe state.
            if not self._confirmed_xy(
                    p.get("x", 0.0), p.get("y", 0.0),
                    getattr(settings, "travel_speed_mm_s", 10.0)):
                raise RuntimeError(
                    f"XY move to ({float(p.get('x', 0.0)):.3f}, "
                    f"{float(p.get('y', 0.0)):.3f}) mm not confirmed — aborting "
                    "before the Z descent rather than lowering the needle at an "
                    "unverified position")
            return

        if t == CommandType.MOVE_Z:
            # Descent to print height — CONFIRM, abort the print if unconfirmed.
            if not self._confirmed_descent(
                    float(p.get("z", 0.0)),
                    getattr(ctrl, "_zp_insert_feedrate", None)):
                raise RuntimeError(
                    f"Z descent to {float(p.get('z', 0.0)):.3f} mm not "
                    "confirmed (board stuck/dropped) — aborting before extrusion")
            return

        if t == CommandType.DISPENSE:
            self._extrude(p, settings)
            return

        if t == CommandType.PRINT_PATH:
            self._print_path(p, settings)
            return

        # Anything else (MOVE_Z_REL / SET_PUMP_RATE / SWITCH_PUMP / TRAJECTORY /
        # SERVICE_SEQUENCE / DWELL) is intentionally NOT handled by the simple
        # baseline — log and skip so we notice if a plan needs it.
        logger.info("SimplePrintManager: skipping unsupported command %s", t.value)

    # ── Confirmed-move helpers (every move blocks until verified) ──────

    def _safe_retract(self, z_zero_ref_mm: float) -> bool:
        """Raise to >= z and CONFIRM (polarity-safe, never descends)."""
        ctrl = self.controller
        if not getattr(ctrl, "is_zp_connected", False):
            return True
        if hasattr(ctrl, "ensure_retracted_to"):
            ok = ctrl.ensure_retracted_to(float(z_zero_ref_mm),
                                          timeout_s=self._Z_TIMEOUT_S)
            if not ok:
                logger.warning("SimplePrintManager: retract to %.2f not confirmed",
                               z_zero_ref_mm)
            return ok
        # Older controller: best-effort raise with an explicit feedrate.
        ctrl.move_z_absolute(
            float(z_zero_ref_mm), from_zero_ref=True,
            feedrate_mm_min=getattr(ctrl, "_zp_retract_feedrate", None))
        time.sleep(0.5)
        return True

    def _confirmed_descent(self, z_zero_ref_mm: float,
                           feedrate_mm_min) -> bool:
        """Move Z DOWN to print height and confirm arrival (M400 + M114)."""
        ctrl = self.controller
        if not getattr(ctrl, "is_zp_connected", False):
            time.sleep(0.5)
            return True
        ctrl.move_z_absolute(float(z_zero_ref_mm), from_zero_ref=True,
                             feedrate_mm_min=feedrate_mm_min)
        zp = getattr(ctrl, "zp_stage", None)
        if zp is not None and hasattr(zp, "flush_moves"):
            if not zp.flush_moves(timeout_s=self._Z_TIMEOUT_S):
                return False
        if hasattr(ctrl, "wait_for_z_arrival"):
            return ctrl.wait_for_z_arrival(float(z_zero_ref_mm),
                                           timeout_s=self._Z_TIMEOUT_S)
        return True

    def _confirmed_xy(self, x_mm: float, y_mm: float,
                      speed_mm_s: float) -> bool:
        """Move XY (zero-ref mm) and block until arrival. Returns confirmation.

        v7.20: this method is *named* ``_confirmed_xy`` and this class's whole
        premise is "every move blocks until verified" — but the arrival result
        was DISCARDED, so MOVE_XY handed off to MOVE_Z's descent with the stage
        position unverified. Returning the verdict is what makes the name true.
        A controller without the waiter returns True (nothing to confirm
        against), matching the pre-existing ``hasattr`` guard.
        """
        ctrl = self.controller
        self._set_xy_speed(speed_mm_s)
        ctrl.move_xy_absolute(x_mm, y_mm, from_zero_ref=True)
        if hasattr(ctrl, "wait_for_xy_arrival"):
            return bool(ctrl.wait_for_xy_arrival(
                x_mm, y_mm, tolerance_mm=0.5, timeout_s=self._XY_TIMEOUT_S))
        return True

    def _set_xy_speed(self, speed_mm_s: float):
        ctrl = self.controller
        xy = getattr(ctrl, "xy_stage", None)
        if xy is None:
            return
        try:
            if hasattr(xy, "set_speed_mm_s"):
                xy.set_speed_mm_s(speed_mm_s)
        except Exception:
            pass

    def _extrude(self, p: dict, settings):
        """A prime / discrete extrude — dispense then CONFIRM (M400)."""
        ctrl = self.controller
        pump = p.get("pump", self._active_pump)
        amount_uL = p.get("amount_uL")
        if amount_uL is None or not hasattr(ctrl, "move_pump_uL"):
            return
        rate = p.get("rate_uL_s", settings.get_pump_rate(pump))
        ctrl.move_pump_uL(pump, amount_uL, rate)
        zp = getattr(ctrl, "zp_stage", None)
        if zp is not None and hasattr(zp, "flush_moves"):
            zp.flush_moves(timeout_s=self._Z_TIMEOUT_S)

    def _print_path(self, p: dict, settings):
        """The simplest robust print path: for each segment, issue the pump
        dispense (ZP board) and the XY move (Prior board), then CONFIRM both
        (M400 drains the ZP move; XY-arrival poll drains the XY move) before the
        next segment. Buffer depth stays at 1 move per board — it can never
        saturate, so no barriers/pacing are needed."""
        ctrl = self.controller
        points = p.get("points", [])
        if len(points) < 2:
            return
        pump = p.get("pump", self._active_pump)
        flow = p.get("flow_rate_uL_s", None)
        speed = getattr(settings, "print_speed_mm_s", 0) or \
            max(getattr(settings, "print_feedrate", 200), 1) / 60.0

        # Move to the start of the path (confirmed).
        self._confirmed_xy(points[0][0], points[0][1], speed)

        for i in range(1, len(points)):
            if self._abort_flag.is_set():
                return
            if (self._zp_connected_at_start
                    and not getattr(ctrl, "is_zp_connected", True)):
                raise RuntimeError(
                    f"ZP disconnected mid-path at segment {i}")

            x1, y1 = points[i - 1][0], points[i - 1][1]
            x2, y2 = points[i][0], points[i][1]
            seg = math.hypot(x2 - x1, y2 - y1)
            if seg < self._SEG_TOL_MM:
                continue

            # 1) Extrude this segment's volume (ZP board), if printing wet.
            if flow and flow > 0 and hasattr(ctrl, "move_pump_uL"):
                seg_time = seg / max(speed, 0.01)
                vol = flow * seg_time
                if vol > 0.001:
                    ctrl.move_pump_uL(pump, vol, flow)

            # 2) Move XY (Prior board).
            self._set_xy_speed(speed)
            ctrl.move_xy_absolute(x2, y2, from_zero_ref=True)

            # 3) CONFIRM both moves before the next segment.
            zp = getattr(ctrl, "zp_stage", None)
            if zp is not None and hasattr(zp, "flush_moves") \
                    and getattr(ctrl, "is_zp_connected", False):
                zp.flush_moves(timeout_s=self._Z_TIMEOUT_S)
            if hasattr(ctrl, "wait_for_xy_arrival"):
                ctrl.wait_for_xy_arrival(x2, y2, tolerance_mm=0.5,
                                         timeout_s=self._XY_TIMEOUT_S)

    # ── Safety / internal ──────────────────────────────────────────

    def _retract_to_safe_z(self):
        """Best-effort raise-only retract to the travel Z (never raises)."""
        ctrl = self.controller
        try:
            if not getattr(ctrl, "is_zp_connected", False) or self.job is None:
                return
            travel_z = getattr(self.job.settings, "travel_z_height", None)
            if travel_z is None:
                return
            if hasattr(ctrl, "ensure_retracted_to"):
                ctrl.ensure_retracted_to(float(travel_z),
                                         timeout_s=self._Z_TIMEOUT_S)
            else:
                ctrl.move_z_absolute(
                    float(travel_z), from_zero_ref=True,
                    feedrate_mm_min=getattr(ctrl, "_zp_retract_feedrate", None))
        except Exception as e:
            logger.error("SimplePrintManager: final safe-Z retract failed: %s", e)

    def _arm_print_floor(self, active: bool):
        ctrl = self.controller
        if ctrl is not None and hasattr(ctrl, "set_print_floor_active"):
            try:
                ctrl.set_print_floor_active(active)
            except Exception as e:
                logger.debug("set_print_floor_active(%s) failed: %s", active, e)

    def _set_state(self, new_state: PrintState):
        self.state = new_state
        if self.on_state_changed:
            try:
                self.on_state_changed(new_state)
            except Exception:
                pass

    def _report_progress(self, message: str):
        if self.on_progress and self.job is not None:
            try:
                self.on_progress(self._current_step, self.job.total_steps, message)
            except Exception:
                pass
