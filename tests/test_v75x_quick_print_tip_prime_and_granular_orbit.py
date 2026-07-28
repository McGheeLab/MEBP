"""
v7.5.x — Quick Print bioink tip-prime pickup + granular anti-clog orbit.

Covers PickPlaceExecutor.aspirate_ink's new pickup behaviors, driving the REAL
executor against a recording fake controller (no hardware):

  * Tip prime: aspirate an EXTRA prime volume, then dispense the same amount
    back into the ink well (net retained = the print volume). While priming, the
    whole pickup step skips compliance compensation (compensate=False).
  * Legacy (prime off): a single aspirate, compensate=None (auto) — unchanged.
  * Travel-only cases (vol <= 0).
  * Granular anti-clog orbit: a background XY circle around the well centre runs
    while the (blocking) aspirate runs, then re-centres; honors abort.
  * The Quick Print page's _ink_pickup_kwargs granular resolution (no Qt).
"""

import os
import threading
import time
import types
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.PickAndPlaceManager import (
    AbortException, PickPlaceExecutor,
)
from SupportClasses.PhysicalModels import InkSpec


# ── Recording fake controller ──────────────────────────────────────

class _RecCtrl:
    def __init__(self, pump_block_s=0.0):
        self.calls = []
        self.pump_calls = []      # (pump, volume_uL, rate, compensate)
        self.xy_calls = []        # (x_um, y_um)
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self._pump_block_s = pump_block_s
        self._lock = threading.Lock()

    def safe_travel_to(self, **kw):
        with self._lock:
            self.calls.append(("safe_travel_to", kw))
        return True

    def move_xy_absolute_um(self, x_um, y_um, fast=False):
        with self._lock:
            self.xy_calls.append((float(x_um), float(y_um)))
            self.calls.append(("move_xy_absolute_um", float(x_um), float(y_um)))

    def move_pump_uL(self, pump, volume_uL, rate_uL_s=None, *,
                     settle=False, compensate=None):
        with self._lock:
            self.pump_calls.append((pump, float(volume_uL), rate_uL_s,
                                    compensate))
            self.calls.append(("move_pump_uL", pump, float(volume_uL)))
        if self._pump_block_s:
            time.sleep(self._pump_block_s)   # simulate the blocking M400 drain

    def wait_for_xy_arrival(self, *a, **k):
        with self._lock:
            self.calls.append(("wait_for_xy_arrival", a))
        return True

    @property
    def pumps(self):
        return {}


def _executor(pump_block_s=0.0):
    ctrl = _RecCtrl(pump_block_s=pump_block_s)
    ex = PickPlaceExecutor(ctrl, hw_config=None)
    ex.safe_z_mm = -35.0
    ex.prep_rate_uL_s = 2.0
    return ex, ctrl


WELL = (10000.0, 20000.0)


# ── Tip prime + compliance skip ────────────────────────────────────

class TestTipPrime(unittest.TestCase):
    def test_prime_off_is_legacy_single_aspirate(self):
        ex, ctrl = _executor()
        ex.aspirate_ink(WELL, 5.0, bore="P1", z_mm=-16.0)
        # one safe travel, one aspirate (-5) at compensate=None (auto)
        self.assertEqual(len(ctrl.pump_calls), 1)
        pump, vol, rate, comp = ctrl.pump_calls[0]
        self.assertEqual(pump, "P1")
        self.assertAlmostEqual(vol, -5.0)
        self.assertEqual(rate, 2.0)          # prep_rate_uL_s default
        self.assertIsNone(comp)              # legacy auto
        self.assertTrue(any(c[0] == "safe_travel_to" for c in ctrl.calls))

    def test_prime_on_extra_aspirate_then_dispense_back(self):
        ex, ctrl = _executor()
        ex.aspirate_ink(WELL, 5.0, bore="P2", z_mm=-16.0, prime_uL=2.0)
        self.assertEqual(len(ctrl.pump_calls), 2)
        # 1) aspirate the print volume + prime = -(7)
        p0, v0, r0, c0 = ctrl.pump_calls[0]
        self.assertEqual(p0, "P2")
        self.assertAlmostEqual(v0, -7.0)
        self.assertFalse(c0)                 # compensate=False while priming
        # 2) dispense the prime back into the well = +2
        p1, v1, r1, c1 = ctrl.pump_calls[1]
        self.assertAlmostEqual(v1, 2.0)
        self.assertFalse(c1)
        # net retained volume == the requested print volume
        self.assertAlmostEqual(sum(v for _, v, _, _ in ctrl.pump_calls), -5.0)

    def test_prime_uses_prime_rate_when_given(self):
        ex, ctrl = _executor()
        ex.aspirate_ink(WELL, 5.0, bore="P1", z_mm=-16.0,
                        prime_uL=2.0, rate_uL_s=3.0, prime_rate_uL_s=8.0)
        self.assertAlmostEqual(ctrl.pump_calls[0][2], 3.0)   # aspirate rate
        self.assertAlmostEqual(ctrl.pump_calls[1][2], 8.0)   # dispense-back rate

    def test_travel_only_no_prime_no_pump(self):
        ex, ctrl = _executor()
        ex.aspirate_ink(WELL, 0.0, bore="P1", z_mm=-16.0)
        self.assertEqual(ctrl.pump_calls, [])
        self.assertTrue(any(c[0] == "safe_travel_to" for c in ctrl.calls))

    def test_prime_only_pure_cycle_when_no_print_volume(self):
        # needle already loaded (vol<=0) but prime requested → aspirate then
        # dispense back the same amount, net zero, compensate off.
        ex, ctrl = _executor()
        ex.aspirate_ink(WELL, 0.0, bore="P1", z_mm=-16.0, prime_uL=2.0)
        self.assertEqual(len(ctrl.pump_calls), 2)
        self.assertAlmostEqual(ctrl.pump_calls[0][1], -2.0)
        self.assertAlmostEqual(ctrl.pump_calls[1][1], 2.0)
        self.assertFalse(ctrl.pump_calls[0][3])


# ── Granular anti-clog orbit ───────────────────────────────────────

class TestOrbit(unittest.TestCase):
    def test_orbit_off_makes_no_xy_orbit_moves(self):
        ex, ctrl = _executor()
        ex.aspirate_ink(WELL, 5.0, bore="P1", z_mm=-16.0, orbit=False)
        self.assertEqual(ctrl.xy_calls, [])   # only safe_travel_to for the hop

    def test_orbit_traces_circle_during_aspirate_then_recenters(self):
        # Make the aspirate block so the orbit thread issues several points.
        ex, ctrl = _executor(pump_block_s=0.25)
        ex.aspirate_ink(WELL, 5.0, bore="P1", z_mm=-16.0,
                        orbit=True, orbit_diameter_mm=1.0, orbit_speed_mm_s=20.0)
        self.assertGreaterEqual(len(ctrl.xy_calls), 3)   # multiple orbit points
        r_um = 500.0   # 1 mm diameter → 0.5 mm radius
        cx, cy = WELL
        # every orbit point (all but the final recentre) sits on the circle
        for x, y in ctrl.xy_calls[:-1]:
            rad = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
            self.assertAlmostEqual(rad, r_um, delta=1.0)
        # last XY call recentres exactly on the well
        self.assertAlmostEqual(ctrl.xy_calls[-1][0], cx)
        self.assertAlmostEqual(ctrl.xy_calls[-1][1], cy)
        self.assertTrue(any(c[0] == "wait_for_xy_arrival" for c in ctrl.calls))

    def test_orbit_thread_honors_abort(self):
        ex, ctrl = _executor()
        stop = ex._orbit_xy(WELL, 1.0, 5.0)
        self.assertIsNotNone(stop)
        t = getattr(stop, "_orbit_thread")
        self.assertTrue(t.is_alive())
        ex._abort_flag.set()
        t.join(timeout=2.0)
        self.assertFalse(t.is_alive())
        ex._abort_flag.clear()

    def test_orbit_noop_when_zero_diameter(self):
        ex, ctrl = _executor()
        self.assertIsNone(ex._orbit_xy(WELL, 0.0, 5.0))


# ── Quick Print page granular / prime resolution (no Qt widgets) ────

class _Chk:
    def __init__(self, v): self._v = v
    def isChecked(self): return self._v


class _Spin:
    def __init__(self, v): self._v = v
    def value(self): return self._v


class TestInkPickupKwargs(unittest.TestCase):
    """Drives QuickPrintWorkflowPage._ink_pickup_kwargs unbound against a fake
    self, so no Qt page construction is needed."""

    def _fake(self, *, prime_on, prime_uL, orbit_on, orbit_all, inks):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        fake = types.SimpleNamespace(
            _ink_prime_check=_Chk(prime_on),
            _ink_prime_spin=_Spin(prime_uL),
            _orbit_check=_Chk(orbit_on),
            _orbit_all_check=_Chk(orbit_all),
            _orbit_dia_spin=_Spin(1.0),
            _orbit_speed_spin=_Spin(2.0),
            _hw_config=types.SimpleNamespace(ink_library=inks),
        )
        return QuickPrintWorkflowPage._ink_pickup_kwargs(fake, "Gel")

    def test_granular_auto_orbits(self):
        inks = {"Gel": InkSpec(name="Gel", ink_type="ink",
                               ink_subtype="granular material")}
        kw = self._fake(prime_on=False, prime_uL=2.0, orbit_on=True,
                        orbit_all=False, inks=inks)
        self.assertTrue(kw["orbit"])
        self.assertEqual(kw["prime_uL"], 0.0)     # prime off
        self.assertAlmostEqual(kw["orbit_diameter_mm"], 1.0)

    def test_non_granular_no_orbit_unless_forced(self):
        inks = {"Gel": InkSpec(name="Gel", ink_type="ink", ink_subtype="media")}
        kw = self._fake(prime_on=False, prime_uL=2.0, orbit_on=True,
                        orbit_all=False, inks=inks)
        self.assertFalse(kw["orbit"])
        kw2 = self._fake(prime_on=False, prime_uL=2.0, orbit_on=True,
                         orbit_all=True, inks=inks)
        self.assertTrue(kw2["orbit"])             # forced for all inks

    def test_orbit_disabled_beats_granular(self):
        inks = {"Gel": InkSpec(name="Gel", ink_type="ink",
                               ink_subtype="granular material")}
        kw = self._fake(prime_on=False, prime_uL=2.0, orbit_on=False,
                        orbit_all=False, inks=inks)
        self.assertFalse(kw["orbit"])

    def test_prime_volume_when_enabled(self):
        inks = {"Gel": InkSpec(name="Gel", ink_type="ink", ink_subtype="")}
        kw = self._fake(prime_on=True, prime_uL=2.5, orbit_on=True,
                        orbit_all=False, inks=inks)
        self.assertAlmostEqual(kw["prime_uL"], 2.5)


if __name__ == "__main__":
    unittest.main()
