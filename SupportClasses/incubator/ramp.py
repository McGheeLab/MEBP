"""
ramp.py — staircase setpoint ramp, to stop Marlin false-tripping thermal runaway.

The problem
-----------
A large water-filled block with a low-power film heater cannot rise 2 °C in 60 s.
Marlin's heat-up watchdog concludes the heater is broken and calls ``kill()``, so
the board stops responding and needs a power cycle. Nothing is actually wrong.

Why a staircase fixes it, verified against Marlin's source
----------------------------------------------------------
``HeaterWatch::restart()`` in ``Marlin/src/module/temperature.h``::

    inline void restart(const celsius_t curr, const celsius_t tgt) {
      if (tgt) {
        const celsius_t newtarget = curr + INCREASE;
        if (newtarget < tgt - HYSTERESIS - 1) {
          target = newtarget;
          next_ms = millis() + SEC_TO_MS(PERIOD);
          return;
        }
      }
      next_ms = 0;
    }

The watchdog **only arms** when::

    target > current + INCREASE + HYSTERESIS + 1

With stock bed values (``WATCH_BED_TEMP_INCREASE`` 2, ``TEMP_BED_HYSTERESIS`` 3)
that threshold is about **current + 6 °C**. Command anything closer than that and
``next_ms = 0`` — the check is not merely passed, it is never scheduled.

So we walk the setpoint up in steps small enough that the watchdog never arms,
waiting for the block to catch up before each step. The heater still runs
continuously at full duty between steps, so this costs almost nothing in heat-up
time; it only changes what number the firmware has been told to aim at.

What this does NOT do
---------------------
It does not disable or weaken any protection:

* MINTEMP / MAXTEMP stay fully active.
* The runaway state machine that catches temperature FALLING while at target
  (``THERMAL_PROTECTION_BED_PERIOD``) stays fully active — and that is the check
  which would catch a genuinely failed heater during a hold.
* Only the "did it heat as fast as I demanded" check is avoided, by not demanding
  an impossible rate in the first place.

It is a workaround, not a substitute for widening the firmware windows: any other
host (or a direct ``M140 S37`` typed into the console) can still trip the stock
watchdog. Fixing the firmware remains the durable answer.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)

#: Default step size, in °C. Chosen to stay under the arming threshold for any
#: plausible stock configuration: the threshold is
#: ``INCREASE + HYSTERESIS + 1``, which is 6 °C for the bed defaults (2/3) and
#: still 5 °C in the least generous common variant (2/2).
DEFAULT_STEP_C = 3.0

#: Hard cap on the step size we will accept, for the same reason.
MAX_SAFE_STEP_C = 4.0

#: A step is considered reached when the temperature is within this of it.
DEFAULT_ARRIVE_BAND_C = 1.0

#: How often the ramp re-evaluates.
TICK_S = 2.0

#: If a step makes no progress for this long, the heater cannot achieve it.
#: We stop rather than sit forever, because a stalled ramp means the heater is
#: undersized (or losses are too high) and the operator needs to know.
DEFAULT_STALL_TIMEOUT_S = 900.0

#: Progress smaller than this over the stall window counts as "no progress".
STALL_PROGRESS_C = 0.3


@dataclass
class RampState:
    """Snapshot of a running ramp, for display."""

    active: bool = False
    final_target_c: float = 0.0
    #: The target currently commanded to the board.
    current_step_c: float = 0.0
    step_c: float = DEFAULT_STEP_C
    #: Temperature when the ramp began, so progress can be measured against it.
    start_temp_c: float = 0.0
    #: Latest temperature seen.
    temp_c: float = 0.0
    #: How many times the commanded target has been raised (informational).
    raises: int = 0
    started_at: float = 0.0
    last_progress_at: float = 0.0
    stalled: bool = False
    finished: bool = False
    message: str = ""

    @property
    def elapsed_s(self) -> float:
        return 0.0 if not self.started_at else time.monotonic() - self.started_at

    @property
    def progress_pct(self) -> float:
        """
        Progress measured by TEMPERATURE, not by how many commands were sent.

        Counting commands was misleading: because each rung is capped relative to
        the live temperature, the target gets nudged roughly once per degree of
        rise, which produced nonsense like "step 15 of 6".
        """
        span = self.final_target_c - self.start_temp_c
        if span <= 0.05:
            return 100.0
        return max(0.0, min(100.0, (self.temp_c - self.start_temp_c) / span * 100.0))


class SetpointRamp:
    """
    Walks one zone's setpoint up to a final target in watchdog-safe steps.

    Runs its own daemon thread and drives the zone through callbacks, so it has no
    dependency on the controller's internals beyond "read the temperature" and
    "command an integer target".
    """

    def __init__(
        self,
        zone_id: str,
        *,
        read_temp,                 # () -> float | None
        command_target,            # (celsius: float) -> None
        on_state=None,             # (RampState) -> None
        on_status=None,            # (str) -> None
        step_c: float = DEFAULT_STEP_C,
        arrive_band_c: float = DEFAULT_ARRIVE_BAND_C,
        stall_timeout_s: float = DEFAULT_STALL_TIMEOUT_S,
        tick_s: float = TICK_S,
    ):
        self.zone_id = zone_id
        self._read_temp = read_temp
        self._command_target = command_target
        self._on_state = on_state
        self._on_status = on_status
        self.step_c = max(0.5, min(MAX_SAFE_STEP_C, float(step_c)))
        self.arrive_band_c = max(0.2, float(arrive_band_c))
        self.stall_timeout_s = max(1.0, float(stall_timeout_s))
        self.tick_s = max(0.2, float(tick_s))

        self._commanded: float = 0.0
        self._last_seen_temp: float | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self.state = RampState(step_c=self.step_c)

    # ── control ─────────────────────────────────────────────────────

    @property
    def active(self) -> bool:
        return bool(self._thread is not None and self._thread.is_alive())

    def start(self, final_target_c: float) -> bool:
        """Begin ramping to ``final_target_c``. Returns False if already running."""
        if self.active:
            return False
        cur = self._read_temp()
        if cur is None:
            self._say("Cannot start the ramp — no temperature reading yet.")
            return False

        final_target_c = float(final_target_c)
        now = time.monotonic()
        with self._lock:
            self.state = RampState(
                active=True,
                final_target_c=final_target_c,
                current_step_c=0.0,
                step_c=self.step_c,
                start_temp_c=cur,
                temp_c=cur,
                started_at=now,
                last_progress_at=now,
                message="starting",
            )
        self._commanded = 0.0
        self._last_seen_temp = cur

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name=f"ramp-{self.zone_id}", daemon=True
        )
        self._thread.start()
        self._say(
            f"Ramping {self.zone_id} from {cur:.1f} °C to "
            f"{final_target_c:.1f} °C, raising the commanded target in steps of "
            f"at most {self.step_c:g} °C above the live temperature. That stays "
            f"below the threshold at which Marlin's heat-up watchdog arms, so it "
            f"cannot false-trip."
        )
        return True

    def stop(self, reason: str = "stopped") -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=self.tick_s * 3)
        self._thread = None
        with self._lock:
            self.state.active = False
            self.state.message = reason
        self._publish()

    # ── worker ──────────────────────────────────────────────────────

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                cur = self._read_temp()
                if cur is None:
                    time.sleep(self.tick_s)
                    continue

                with self._lock:
                    final = self.state.final_target_c
                    self.state.temp_c = cur

                # Highest target we may command right now without arming the
                # watchdog: at most step_c above the LIVE temperature, and never
                # past the final target. Integer, because Marlin quantises.
                allowed = min(final, float(int(cur + self.step_c)))

                if allowed > self._commanded + 0.5:
                    self._commanded = allowed
                    with self._lock:
                        self.state.current_step_c = allowed
                        self.state.raises += 1
                        self.state.message = f"holding {allowed:.0f} °C"
                    self._command_target(allowed)
                    self._publish()

                # The ramp is DONE once the final target has been commanded.
                # Waiting for the temperature to arrive would be the wrong test:
                # from this point the firmware owns the hold, and a block that
                # settles slightly short is a heater-capacity question, not a
                # ramp that never finished.
                if self._commanded >= final - 0.001:
                    with self._lock:
                        self.state.finished = True
                        self.state.active = False
                        self.state.message = "final target commanded"
                    self._publish()
                    self._say(
                        f"Ramp complete — {final:.1f} °C is now commanded directly "
                        f"and the firmware holds it from here. The watchdog was "
                        f"never armed at any point."
                    )
                    return

                self._check_stall(cur)
                if self._stop.is_set():
                    return
                time.sleep(self.tick_s)
        except Exception:
            logger.warning("ramp thread failed", exc_info=True)
            with self._lock:
                self.state.active = False
                self.state.message = "ramp error"
            self._publish()

    def _check_stall(self, cur: float) -> None:
        """Give up if the block simply is not getting any warmer."""
        now = time.monotonic()
        with self._lock:
            st = self.state
            if self._last_seen_temp is None or cur - self._last_seen_temp >= STALL_PROGRESS_C:
                self._last_seen_temp = cur
                st.last_progress_at = now
                return
            if now - st.last_progress_at < self.stall_timeout_s:
                return
            st.stalled = True
            st.active = False
        self._stop.set()
        self._publish()
        self._say(
            f"Ramp stalled at {cur:.2f} °C — the temperature has not risen "
            f"{STALL_PROGRESS_C:g} °C in "
            f"{self.stall_timeout_s / 60:.0f} min. The heater has reached the "
            f"limit of what it can hold against ambient losses. Check the "
            f"steady-state duty: near 100 % means the heater is undersized or the "
            f"block needs insulating. The setpoint has been left where it is."
        )

    # ── plumbing ────────────────────────────────────────────────────

    def snapshot(self) -> RampState:
        with self._lock:
            st = self.state
            return RampState(**{k: getattr(st, k) for k in st.__dataclass_fields__})

    def _publish(self) -> None:
        if self._on_state is None:
            return
        try:
            self._on_state(self.snapshot())
        except Exception:
            pass

    def _say(self, text: str) -> None:
        logger.info("[ramp %s] %s", self.zone_id, text)
        if self._on_status is None:
            return
        try:
            self._on_status(text)
        except Exception:
            pass


def watchdog_arm_threshold_c(increase_c: float = 2.0,
                             hysteresis_c: float = 3.0) -> float:
    """
    How far above the current temperature a target must be before Marlin's
    heat-up watchdog arms: ``INCREASE + HYSTERESIS + 1``.

    Exposed so the UI can explain the chosen step size rather than presenting it
    as a magic number.
    """
    return float(increase_c) + float(hysteresis_c) + 1.0
