"""test_v75x_xy_dead_time.py — the step-response dead-time probe.

The velocity follower's speed limit is lookahead/((loop+dead_time)·safety), so
dead_time divides the achievable print speed. The legacy source for that number
was the by_phase intercept mean — an OPTICAL SETTLE time whose measured values on
ME3B V1 include two impossible negatives and a single-point 0.577 s fit, averaging
0.287 s and capping prints at 0.31 mm/s on a 5.9 mm/s stage.

These tests drive the probe against a synthetic first-order-plus-dead-time stage
with a KNOWN delay, so we can assert the probe recovers it — and assert the safety
contract (always stops, always returns to start, never touches Z).
"""

import math
import threading
import time
import unittest
from types import SimpleNamespace

from SupportClasses import XYDeadTime as DT


class _FopdtStage:
    """Position responds to a velocity command with a transport delay + a
    first-order rise, reported through µm quantisation.

    Time is driven by wall clock (the probe times real intervals), but the
    dynamics are analytic so the recovered delay is deterministic.
    """

    def __init__(self, dead_s=0.040, tau_s=0.030, quantum_um=1.0,
                 gain=1.0, respond=True):
        self.dead_s = dead_s
        self.tau_s = tau_s
        self.quantum_um = quantum_um
        self.gain = gain
        self.respond = respond
        self.is_xy_connected = True
        # A realistic ABSOLUTE envelope (v7.5.x bounds are absolute stage µm), so
        # the probe can find the centre and check that its excursion fits.
        self.safety_limits = SimpleNamespace(
            max_xy_speed=50000.0,
            xy_min_x=0.0, xy_max_x=114332.0,
            xy_min_y=0.0, xy_max_y=76645.0,
            xy_center=lambda: (57166.0, 38322.5))
        self.xy_stage = SimpleNamespace(set_acceleration=lambda a: None,
                                       set_speed_mm_s=lambda v: None)
        self.centered_to = []
        self._x = 10000.0
        self._y = 20000.0
        self._cmd = (0.0, 0.0)
        self._cmd_t = time.monotonic()
        self._v = (0.0, 0.0)              # current actual velocity
        self._last = time.monotonic()
        self.vs_log = []                  # every commanded (vx, vy)
        self.returns = []                 # every move_xy_absolute_um target
        self.poller_suspended = 0
        self.poller_resumed = 0
        self.z_touched = False

    # ── dynamics ──
    def _advance(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        if dt <= 0:
            return
        # The command only takes effect after dead_s, then ramps with tau.
        active = self._cmd if (now - self._cmd_t) >= self.dead_s else (0.0, 0.0)
        if not self.respond:
            active = (0.0, 0.0)
        if self.tau_s > 1e-9:
            a = dt / (self.tau_s + dt)
        else:
            a = 1.0
        vx = self._v[0] + a * (active[0] * self.gain - self._v[0])
        vy = self._v[1] + a * (active[1] * self.gain - self._v[1])
        self._v = (vx, vy)
        self._x += vx * dt
        self._y += vy * dt

    # ── controller surface the probe uses ──
    def get_xy_position(self, cached=False):
        self._advance()
        q = self.quantum_um
        return (round(self._x / q) * q, round(self._y / q) * q, 0.0)

    def send_velocity_xy(self, vx, vy):
        self._advance()
        self.vs_log.append((vx, vy))
        self._cmd = (vx, vy)
        self._cmd_t = time.monotonic()
        if vx == 0.0 and vy == 0.0:
            self._v = (0.0, 0.0)          # a real stop settles quickly

    def default_plate_center_um(self):
        return (57166.0, 38322.5)

    def move_xy_absolute_um(self, x, y):
        self._advance()
        self.returns.append((x, y))
        self._x, self._y = float(x), float(y)
        self._v = (0.0, 0.0)
        self._cmd = (0.0, 0.0)

    def wait_for_xy_arrival(self, *a, **k):
        return True

    def suspend_position_poller(self):
        self.poller_suspended += 1

    def resume_position_poller(self):
        self.poller_resumed += 1

    # Any Z access is a contract violation.
    def move_z_absolute(self, *a, **k):
        self.z_touched = True

    def move_z_relative(self, *a, **k):
        self.z_touched = True


class TestDeadTimeRecovery(unittest.TestCase):
    def test_recovers_a_known_delay(self):
        st = _FopdtStage(dead_s=0.060, tau_s=0.020)
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=3,
                                          max_travel_um=600.0, timeout_s=2.0)
        self.assertNotIn("error", r, msg=r.get("error"))
        # Recovered within one poll interval of the truth.
        tol = max(0.030, 2.0 * r["poll_dt_s"])
        self.assertAlmostEqual(r["dead_time_s"], 0.060, delta=tol,
                               msg=f"got {r['dead_time_s']:.3f} (poll "
                                   f"{r['poll_dt_s']:.3f}) trials={r['trials']}")
        self.assertEqual(r["n"], 3)

    def test_longer_delay_reads_longer(self):
        fast = DT.measure_velocity_dead_time(
            _FopdtStage(dead_s=0.030, tau_s=0.010), speed_um_s=4000.0,
            repeats=3, max_travel_um=600.0, timeout_s=2.0)
        slow = DT.measure_velocity_dead_time(
            _FopdtStage(dead_s=0.220, tau_s=0.010), speed_um_s=4000.0,
            repeats=3, max_travel_um=600.0, timeout_s=2.0)
        self.assertNotIn("error", fast)
        self.assertNotIn("error", slow)
        self.assertGreater(slow["dead_time_s"], fast["dead_time_s"] + 0.08)

    def test_apparent_lag_is_at_least_the_dead_time(self):
        """The FOPDT intercept includes part of the rise, so it is the more
        conservative number to feed the stability limit."""
        st = _FopdtStage(dead_s=0.050, tau_s=0.060)
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=3,
                                          max_travel_um=800.0, timeout_s=2.0)
        self.assertNotIn("error", r)
        self.assertGreaterEqual(r["apparent_lag_s"], r["dead_time_s"] - 1e-9)
        self.assertGreater(r["tau_s"], 0.0)

    def test_reports_cruise_speed_as_a_top_speed_cross_check(self):
        st = _FopdtStage(dead_s=0.040, tau_s=0.010, gain=0.5)
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=3,
                                          max_travel_um=900.0, timeout_s=2.0)
        self.assertNotIn("error", r)
        # gain 0.5 → the stage only delivers half of what was commanded, which is
        # exactly the 8.4× declared-vs-measured discrepancy this cross-checks.
        self.assertAlmostEqual(r["cruise_um_s"], 2000.0, delta=700.0)


class TestCentringBeforeProbing(unittest.TestCase):
    """The stage must be centred before any probe.

    Every probe moves RELATIVE to where it starts. Starting near a travel extent
    makes the move clamp — and a clamped probe does not fail loudly, it silently
    measures the clamp ("no motion detected", or a truncated distance fit). So
    centring is both a safety and a correctness requirement.
    """

    def test_travels_to_the_envelope_centre_first(self):
        st = _FopdtStage()
        st._x, st._y = 500.0, 400.0          # parked in a corner
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                          max_travel_um=400.0, timeout_s=2.0)
        self.assertNotIn("error", r)
        cx, cy = st.default_plate_center_um()
        self.assertAlmostEqual(st.returns[0][0], cx, delta=1.0)
        self.assertAlmostEqual(st.returns[0][1], cy, delta=1.0)

    def test_excursion_is_shrunk_to_fit_a_small_envelope(self):
        st = _FopdtStage()
        st.safety_limits.xy_min_x = 0.0
        st.safety_limits.xy_max_x = 6000.0    # only ±3000 µm about the centre
        st.safety_limits.xy_min_y = 0.0
        st.safety_limits.xy_max_y = 6000.0
        st.safety_limits.xy_center = lambda: (3000.0, 3000.0)
        st.default_plate_center_um = lambda: (3000.0, 3000.0)
        st._x, st._y = 3000.0, 3000.0
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                          max_travel_um=50000.0, timeout_s=2.0)
        self.assertNotIn("error", r)
        # Never travelled beyond the usable radius (half-extent minus margin).
        for x, y in [(st._x, st._y)]:
            self.assertLessEqual(abs(x - 3000.0), 3000.0)
        self.assertTrue(all(0.0 <= p <= 6000.0 for p in (st._x, st._y)))

    def test_refuses_when_the_envelope_is_unknown(self):
        st = _FopdtStage()
        st.safety_limits = SimpleNamespace(max_xy_speed=50000.0)   # no bounds
        st.default_plate_center_um = lambda: None                  # unknown
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2)
        self.assertIn("error", r)
        self.assertIn("envelope", r["error"].lower())

    def test_centring_can_be_disabled_for_a_caller_that_already_centred(self):
        st = _FopdtStage()
        st._x, st._y = 20000.0, 20000.0
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                          max_travel_um=400.0, timeout_s=2.0,
                                          center_first=False)
        self.assertNotIn("error", r)
        self.assertAlmostEqual(st._x, 20000.0, delta=1.0)   # stayed put


class TestDeadTimeSafety(unittest.TestCase):
    def test_always_stops_and_returns_to_where_it_started_probing(self):
        st = _FopdtStage()
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                          max_travel_um=400.0, timeout_s=2.0)
        self.assertNotIn("error", r)
        self.assertEqual(st.vs_log[-1], (0.0, 0.0))     # stopped
        cx, cy = st.default_plate_center_um()           # the centred origin
        self.assertAlmostEqual(st._x, cx, delta=1.0)
        self.assertAlmostEqual(st._y, cy, delta=1.0)

    def test_never_touches_z(self):
        st = _FopdtStage()
        DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                      max_travel_um=400.0, timeout_s=2.0)
        self.assertFalse(st.z_touched)

    def test_poller_suspended_and_resumed(self):
        st = _FopdtStage()
        DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                      max_travel_um=400.0, timeout_s=2.0)
        self.assertEqual(st.poller_suspended, 1)
        self.assertEqual(st.poller_resumed, 1)

    def test_a_dead_stage_errors_and_still_stops(self):
        st = _FopdtStage(respond=False)
        r = DT.measure_velocity_dead_time(st, speed_um_s=4000.0, repeats=2,
                                          max_travel_um=400.0, timeout_s=0.4)
        self.assertIn("error", r)
        self.assertIn("no motion", r["error"])
        self.assertEqual(st.vs_log[-1], (0.0, 0.0))

    def test_disconnected_stage_refuses(self):
        st = _FopdtStage()
        st.is_xy_connected = False
        self.assertIn("error", DT.measure_velocity_dead_time(st))

    def test_no_velocity_command_refuses(self):
        """A stage with no continuous-velocity primitive (e.g. Ludl HLC) must be
        refused rather than silently measured through a pulsed fallback."""
        st = SimpleNamespace(
            is_xy_connected=True,
            xy_stage=SimpleNamespace(),
            safety_limits=SimpleNamespace(max_xy_speed=50000.0),
            get_xy_position=lambda cached=False: (0.0, 0.0, 0.0),
        )
        r = DT.measure_velocity_dead_time(st)
        self.assertIn("error", r)
        self.assertIn("continuous-velocity", r["error"])

    def test_stop_event_aborts_and_stops(self):
        st = _FopdtStage()
        evt = threading.Event()
        evt.set()
        r = DT.measure_velocity_dead_time(st, repeats=3, stop_evt=evt)
        self.assertIn("error", r)                # nothing measured
        self.assertEqual(st.vs_log[-1], (0.0, 0.0))

    def test_speed_clamped_to_the_envelope(self):
        st = _FopdtStage()
        st.safety_limits.max_xy_speed = 1200.0      # keep the travel bounds
        r = DT.measure_velocity_dead_time(st, speed_um_s=40000.0, repeats=2,
                                          max_travel_um=300.0, timeout_s=2.0)
        self.assertLessEqual(r.get("speed_um_s", 1e9), 1200.0)
        for vx, vy in st.vs_log:
            self.assertLessEqual(math.hypot(vx, vy), 1200.0 + 1e-6)


class TestStableSpeedArithmetic(unittest.TestCase):
    """The readout the operator sees must be the SAME arithmetic the follower
    uses, so the panel can never explain a cap that isn't the real one."""

    def test_matches_the_measured_machine_state(self):
        v = DT.stable_speed_mm_s(0.2871766666666667, 31.75, 0.2)
        self.assertAlmostEqual(v, 0.3135517046761011, places=9)

    def test_a_real_dead_time_unlocks_the_speed(self):
        legacy = DT.stable_speed_mm_s(0.2872, 31.75, 0.6)
        real = DT.stable_speed_mm_s(0.040, 31.75, 0.6)
        self.assertGreater(real, legacy * 4.0)

    def test_lead_compensation_raises_it_further(self):
        base = DT.stable_speed_mm_s(0.040, 31.75, 0.6)
        comp = DT.stable_speed_mm_s(0.040, 31.75, 0.6, lead_time_frac=0.7)
        self.assertGreater(comp, base)

    def test_unmeasured_returns_none(self):
        self.assertIsNone(DT.stable_speed_mm_s(0.0, 0.0, 0.6))


class TestRobustHelpers(unittest.TestCase):
    def test_median_and_mad(self):
        self.assertEqual(DT._median([3, 1, 2]), 2)
        self.assertEqual(DT._median([4, 1, 2, 3]), 2.5)
        self.assertEqual(DT._median([]), 0.0)
        self.assertAlmostEqual(DT._mad_spread([10, 10, 10]), 0.0)
        self.assertGreater(DT._mad_spread([1, 2, 3, 40]), 0.0)

    def test_fit_line(self):
        slope, icept = DT._fit_line([0, 1, 2, 3], [1, 3, 5, 7])
        self.assertAlmostEqual(slope, 2.0, places=9)
        self.assertAlmostEqual(icept, 1.0, places=9)
        self.assertEqual(DT._fit_line([1], [1]), (0.0, 0.0))
        self.assertEqual(DT._fit_line([2, 2], [1, 5]), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
