"""
v7.5.x — jog direction unified on ``z_up_sign`` + shorten-only soft-limit clamp.

Two operator-reported hardware bugs:

1. **The Z jog was reversed** and the inputs disagreed with each other: the
   on-screen Z▲ button drove the needle DOWN while PageUp drove it UP, and both
   could disagree with the position readout. Root cause: jog direction was
   hardcoded per-widget (and the module ``ZDIR``), while the readout used the
   per-machine ``z_up_sign`` DERIVED by the Set Bottom/Top setup. After
   ``steps_per_mm.Z`` was flipped negative (inverting the raw frame), the
   hardcoded paths went stale and reversed.

   Fix: every jog input now carries a HEIGHT-frame delta (+ = up) and routes
   through ``StageController.move_z_user_relative``, which maps it to a raw
   delta with ``z_up_sign`` (Δraw = Δuser · z_up_sign). "Up" always retracts the
   needle toward the taught Top, machine-independently.

2. **Z went out of range and the Xbox jog "kept moving the axis constantly."**
   The velocity jog loop clamps against the CACHED position with
   ``clamp(cur+delta) − cur``; when ``cur`` is outside the envelope that is a
   large delta OPPOSITE the request, re-sent every segment.

   Fix: ``_shorten_only_delta`` — the soft-limit clamp on a relative jog may
   only SHORTEN the move (never reverse its sign or exceed its magnitude), so an
   out-of-bounds cached position can't drive a runaway.

These exercise the pure logic on a minimal controller built with ``__new__``
(no hardware), plus a real ``ZPJogHandler`` against a fake stage, and the real
``JogButtonArray`` widget (offscreen Qt).
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import sys
import time
import unittest

from SupportClasses.StageController import (
    StageController, ZPJogHandler, ZDIR, _shorten_only_delta,
)
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.Processor import Processor


# ── Minimal fakes ──────────────────────────────────────────────────────

class _FakeZP:
    """Records move_relative calls; ME3B axis_map (Z→Z, P1→X, …)."""

    def __init__(self):
        self.axis_map = {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"}
        self.moves = []

    def move_relative(self, deltas, feedrate=None):
        self.moves.append((dict(deltas or {}), feedrate))


def _ctrl(z_up_sign=1.0, cur_raw_z=-20.0, z_min=-48.2, z_max=1.8):
    """A StageController with just enough state for the Z move primitives.

    Mirrors live ME3B V1: zero_position.Z = z_min (bottom datum), envelope in
    absolute raw mm [-48.2, 1.8].
    """
    c = StageController.__new__(StageController)
    c.zero_position = {"x": 0.0, "y": 0.0, "Z": -48.2, "P1": 0, "P2": 0, "P3": 0}
    c.safety_limits = SafetyLimits()
    c.safety_limits.z_min = z_min
    c.safety_limits.z_max = z_max
    c.safety_limits.enabled = True
    c._z_up_sign = z_up_sign
    c._axis_flip = {"Z": False, "P1": False, "P2": False, "P3": False}
    c._print_floor_active = False
    c._plate_bottom_z_zref = None
    c.zp_stage = _FakeZP()
    # get_zp_position(cached=True) returns the physical Marlin tuple (X,Y,Z,E);
    # axis_map Z→Z ⇒ Z lives at index 2.
    c.get_zp_position = lambda cached=True: (0.0, 0.0, cur_raw_z, 0.0)
    return c


def _z_of(move):
    """The Z delta from a recorded (deltas, feedrate) move."""
    return move[0].get("Z")


# ════════════════════════════════════════════════════════════════════
#  _shorten_only_delta — the clamp invariant
# ════════════════════════════════════════════════════════════════════

class TestShortenOnlyDelta(unittest.TestCase):

    def test_in_bounds_unchanged(self):
        # No clamping happened: clamped_dest == cur + requested.
        self.assertEqual(_shorten_only_delta(0.0, 1.0, 1.0), 1.0)
        self.assertEqual(_shorten_only_delta(0.0, -2.5, -2.5), -2.5)

    def test_approaching_wall_partial(self):
        # cur=1.5, requested +1.0, wall at 1.8 → clamped_dest 1.8 → 0.3.
        self.assertAlmostEqual(_shorten_only_delta(1.5, 1.0, 1.8), 0.3)

    def test_at_wall_pushing_out_is_noop(self):
        # cur at the wall, request pushes past it → clamp pins at wall → 0.
        self.assertEqual(_shorten_only_delta(1.8, 1.0, 1.8), 0.0)

    def test_out_of_bounds_push_further_out_is_noop(self):
        # cur=11.8 (past max 1.8), request +1.0 (further out): clamp_dest=1.8,
        # raw delta = -10 (OPPOSITE the request) → must be 0, never the snap.
        self.assertEqual(_shorten_only_delta(11.8, 1.0, 1.8), 0.0)

    def test_out_of_bounds_push_back_in_capped_to_request(self):
        # cur=11.8, request -1.0 (back toward range): raw snap = -10, same sign
        # but bigger than the request → cap to the requested step.
        self.assertEqual(_shorten_only_delta(11.8, -1.0, 1.8), -1.0)

    def test_out_of_bounds_small_reentry_uses_raw(self):
        # If the requested step is big enough to land inside, take it as-is.
        self.assertAlmostEqual(_shorten_only_delta(2.0, -1.0, 1.0), -1.0)
        # And a request that overshoots the wall is shortened to land on it.
        self.assertAlmostEqual(_shorten_only_delta(2.0, -5.0, 1.8), -0.2)

    def test_never_amplifies_or_reverses(self):
        for cur in (-50.0, -1.0, 0.0, 1.0, 50.0):
            for req in (-3.0, -0.1, 0.1, 3.0):
                for dest in (-48.2, -10.0, 0.0, 1.8, 5.0):
                    out = _shorten_only_delta(cur, req, dest)
                    # same sign as request (or zero)
                    if out != 0.0:
                        self.assertEqual(out > 0, req > 0)
                    # never larger in magnitude than the request
                    self.assertLessEqual(abs(out), abs(req) + 1e-12)


# ════════════════════════════════════════════════════════════════════
#  move_z_user_relative — height → raw via z_up_sign
# ════════════════════════════════════════════════════════════════════

class TestMoveZUserRelative(unittest.TestCase):

    def test_up_is_positive_raw_when_z_up_sign_plus(self):
        c = _ctrl(z_up_sign=1.0)
        rec = []
        c.move_z_relative = lambda dist, feedrate=None, bypass_safety=False: \
            rec.append((dist, feedrate, bypass_safety))
        c.move_z_user_relative(2.0, feedrate=100, bypass_safety=True)
        self.assertEqual(rec, [(2.0, 100, True)])  # +2 user → +2 raw

    def test_up_is_negative_raw_when_z_up_sign_minus(self):
        c = _ctrl(z_up_sign=-1.0)
        rec = []
        c.move_z_relative = lambda dist, feedrate=None, bypass_safety=False: \
            rec.append(dist)
        c.move_z_user_relative(2.0)        # +2 user (up) → -2 raw on this frame
        c.move_z_user_relative(-3.0)       # -3 user (down) → +3 raw
        self.assertEqual(rec, [-2.0, 3.0])


# ════════════════════════════════════════════════════════════════════
#  move_z_relative — no axis_flip, shorten-only clamp
# ════════════════════════════════════════════════════════════════════

class TestMoveZRelativePrimitive(unittest.TestCase):

    def test_z_axis_flip_no_longer_inverts(self):
        # Even with axis_flip['Z'] set True (legacy, force-zeroed in the UI),
        # the raw primitive must NOT invert — Z direction is z_up_sign-owned.
        c = _ctrl(cur_raw_z=-20.0)
        c._axis_flip["Z"] = True
        c.move_z_relative(1.0, feedrate=100)
        self.assertEqual(_z_of(c.zp_stage.moves[-1]), 1.0)

    def test_in_bounds_move_passes_through(self):
        c = _ctrl(cur_raw_z=-20.0)
        c.move_z_relative(-5.0, feedrate=100)
        self.assertEqual(_z_of(c.zp_stage.moves[-1]), -5.0)

    def test_approaching_wall_is_partial(self):
        c = _ctrl(cur_raw_z=1.5)           # near z_max 1.8
        c.move_z_relative(1.0, feedrate=100)
        self.assertAlmostEqual(_z_of(c.zp_stage.moves[-1]), 0.3)

    def test_out_of_bounds_push_out_suppressed(self):
        c = _ctrl(cur_raw_z=11.8)          # past z_max 1.8
        c.move_z_relative(1.0, feedrate=100)   # push further out
        self.assertEqual(_z_of(c.zp_stage.moves[-1]), 0.0)

    def test_out_of_bounds_push_back_capped(self):
        c = _ctrl(cur_raw_z=11.8)
        c.move_z_relative(-1.0, feedrate=100)  # back toward range
        self.assertEqual(_z_of(c.zp_stage.moves[-1]), -1.0)


# ════════════════════════════════════════════════════════════════════
#  ZPJogHandler — Xbox Z direction + runaway guard
# ════════════════════════════════════════════════════════════════════

class TestZPJogDirection(unittest.TestCase):

    def _run_one(self, z_up_sign_provider, cur_raw_z, vel_z,
                 z_min=-48.2, z_max=1.8):
        proc = Processor(name="TestJogDir")
        self.addCleanup(proc.stop)
        stage = _FakeZP()
        sl = SafetyLimits()
        sl.z_min, sl.z_max, sl.enabled = z_min, z_max, True
        jog = ZPJogHandler(
            proc, stage, safety_limits=sl,
            get_zp_position=lambda: (0.0, 0.0, cur_raw_z, 0.0),
            z_up_sign_provider=z_up_sign_provider,
        )
        jog.stale_timeout = 5.0
        jog._handle_z_vel(average=vel_z)
        jog.start()
        self.addCleanup(jog.stop)
        time.sleep(0.3)
        jog.stop()
        return [_z_of(m) for m in stage.moves if _z_of(m) is not None]

    def test_default_z_up_sign_is_zdir(self):
        jog = ZPJogHandler(Processor(name="t0"), _FakeZP())
        self.addCleanup(jog.processor.stop)
        self.assertEqual(jog._z_up_sign(), ZDIR)

    def test_provider_value_used(self):
        jog = ZPJogHandler(Processor(name="t1"), _FakeZP(),
                           z_up_sign_provider=lambda: 1.0)
        self.addCleanup(jog.processor.stop)
        self.assertEqual(jog._z_up_sign(), 1.0)

    def test_stick_up_retracts_up_z_up_plus(self):
        # z_up_sign=+1: stick-up (vel>0) → POSITIVE raw delta (= up on this frame)
        zs = self._run_one(lambda: 1.0, cur_raw_z=-20.0, vel_z=0.8)
        self.assertTrue(zs, "no Z segment sent")
        self.assertTrue(all(z > 0 for z in zs), f"expected +raw, got {zs}")

    def test_stick_up_retracts_up_z_up_minus(self):
        # z_up_sign=-1: stick-up → NEGATIVE raw delta (= up on the opposite frame)
        zs = self._run_one(lambda: -1.0, cur_raw_z=-20.0, vel_z=0.8)
        self.assertTrue(zs, "no Z segment sent")
        self.assertTrue(all(z < 0 for z in zs), f"expected -raw, got {zs}")

    def test_out_of_bounds_no_runaway(self):
        # cur far past z_max, stick pushing further out: every segment must be a
        # no-op (0), NEVER the large opposite-direction snap (~ -10) the old
        # clamp produced every tick.
        zs = self._run_one(lambda: 1.0, cur_raw_z=11.8, vel_z=0.8)
        self.assertTrue(zs, "loop did not run")
        self.assertTrue(all(abs(z) < 0.5 for z in zs),
                        f"runaway: a large clamp delta was sent: {zs}")


# ════════════════════════════════════════════════════════════════════
#  JogButtonArray — height-frame Z contract (Z▲ = up = +)
# ════════════════════════════════════════════════════════════════════

class TestJogButtonArrayZContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_z_up_emits_positive_z_down_negative(self):
        from PySide6.QtWidgets import QPushButton
        from gui.widgets.jog_button_array import JogButtonArray
        arr = JogButtonArray(compact=True)
        # z_step_mm is a read-only property driven by the step selector; its
        # default is positive, so the emitted sign reflects direction intent.
        self.assertGreater(arr.z_step_mm, 0)
        got = []
        arr.jog_z_requested.connect(got.append)
        btns = {b.toolTip(): b for b in arr.findChildren(QPushButton)}
        self.assertIn("Move Z up", btns)
        self.assertIn("Move Z down", btns)
        btns["Move Z up"].click()
        btns["Move Z down"].click()
        self.assertEqual(len(got), 2)
        self.assertGreater(got[0], 0, "Z▲ must emit a positive (up) height delta")
        self.assertLess(got[1], 0, "Z▼ must emit a negative (down) height delta")


if __name__ == "__main__":
    unittest.main()
