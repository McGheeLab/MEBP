"""
v7.5.x — Plate Location "snap to well center" lands between wells.

On the Calibration page → Plate Location sub-tab, snapped wells are queued and
the Run state machine moves to each via::

    cx, cy = self._predict_well_xy(well_name)
    self.controller.move_xy_absolute_um(cx, cy)      # absolute stage µm

The OLD ``_predict_well_xy`` first consulted ``getattr(self, '_predicted_wells',
None)`` — an attribute assigned NOWHERE in the codebase — and *added*
``zero_position`` to it. That branch never fired, so every snapped move fell
through to the zero-anchored geometry fallback (well grid pinned to stage zero,
as if A1 sat at the origin). Meanwhile the canvas draws from
``_wells_in_zero_ref()`` → ``_calibrated_positions or _predicted_positions``
(plate centred on the XY safety-envelope midpoint). Display and motion used
different frames, so the needle drove to a constant offset from the drawn well —
landing in the gaps between wells while the map still looked grid-aligned.

The fix makes ``_predict_well_xy`` read the SAME dict the canvas draws and return
that absolute-µm value DIRECTLY (no ``zero_position`` offset). These tests call
the real (unbound) method with a stub ``self`` so no Qt widget is instantiated.

Frame contract under test:
  * ``_calibrated_positions`` / ``_predicted_positions`` hold ABSOLUTE stage µm.
  * the canvas draws ``value - zero_position`` (zero-ref) → so move == draw+zero.
  * ``move_xy_absolute_um`` consumes absolute stage µm.
"""

import inspect
import unittest
from types import SimpleNamespace

from gui.pages.calibration import CalibrationPage
from SupportClasses.WellPlate import WellPlate


def _stub(plate, *, calibrated=None, predicted=None, taught_a1=None,
          zero=(0.0, 0.0), plate_center=None):
    """Minimal duck-typed ``self`` for the unbound ``_predict_well_xy``.

    Only the attributes the method actually touches are provided.
    """
    def _default_plate_center_um():
        if plate_center is None:
            raise RuntimeError("no plate center configured")
        return plate_center

    controller = SimpleNamespace(
        zero_position={"x": zero[0], "y": zero[1]},
        default_plate_center_um=_default_plate_center_um,
    )
    stub = SimpleNamespace(
        controller=controller,
        _plate=plate,
        _calibrated_positions=calibrated,
        _predicted_positions=predicted,
        _taught_a1=taught_a1,
    )
    # v7.5.x: bind the real per-machine axis-sign helper; the stub controller
    # has no ``plate_axis_sign`` so it falls back to the aligned (1, 1) default.
    stub._plate_axis_sign = CalibrationPage._plate_axis_sign.__get__(stub)
    return stub


def _predict(stub, well_name):
    # Call the real method body against the stub (no Qt instance).
    return CalibrationPage._predict_well_xy(stub, well_name)


class TestPrimaryPath(unittest.TestCase):
    """Calibrated/predicted positions are returned verbatim (absolute µm)."""

    def setUp(self):
        self.plate = WellPlate.from_format(96)

    def test_calibrated_returned_directly_without_zero_offset(self):
        # The core regression. Calibrated B2 sits at an absolute stage µm
        # that is unrelated to the zero-anchored grid the old code produced.
        cal = {"B2": (50000.0, 30000.0)}
        stub = _stub(self.plate, calibrated=cal, predicted={"B2": (1.0, 2.0)},
                     zero=(12345.0, 6789.0))
        got = _predict(stub, "B2")
        self.assertEqual(got, (50000.0, 30000.0))

        # And it must NOT be the old zero-anchored grid value (zero + grid
        # offset from A1), which is what put the needle between wells.
        wells = {w.name: w for w in self.plate.get_all_wells()}
        a1, b2 = wells["A1"], wells["B2"]
        old_buggy = (12345.0 + (b2.x - a1.x) * 1000.0,
                     6789.0 + (b2.y - a1.y) * 1000.0)
        self.assertNotEqual(got, old_buggy)

    def test_calibrated_takes_precedence_over_predicted(self):
        stub = _stub(self.plate,
                     calibrated={"A1": (7.0, 8.0)},
                     predicted={"A1": (999.0, 999.0)})
        self.assertEqual(_predict(stub, "A1"), (7.0, 8.0))

    def test_predicted_used_when_no_calibrated(self):
        stub = _stub(self.plate, calibrated=None,
                     predicted={"A1": (4242.0, 1111.0)})
        self.assertEqual(_predict(stub, "A1"), (4242.0, 1111.0))

    def test_empty_calibrated_falls_through_to_predicted(self):
        # `{} or predicted` → predicted (empty dict is falsy).
        stub = _stub(self.plate, calibrated={},
                     predicted={"A1": (5.0, 6.0)})
        self.assertEqual(_predict(stub, "A1"), (5.0, 6.0))

    def test_returned_value_matches_canvas_frame(self):
        # The canvas draws `value - zero` (zero-ref). Moving to the returned
        # absolute value therefore lands exactly on the drawn well:
        #   move == draw + zero.
        cal = {"C3": (60000.0, 45000.0)}
        zero = (10000.0, 5000.0)
        stub = _stub(self.plate, calibrated=cal, zero=zero)
        got = _predict(stub, "C3")
        drawn_zero_ref = (cal["C3"][0] - zero[0], cal["C3"][1] - zero[1])
        self.assertEqual((got[0] - zero[0], got[1] - zero[1]), drawn_zero_ref)


class TestFallbacks(unittest.TestCase):
    """No position dict yet — geometry fallbacks must match the canvas seed."""

    def setUp(self):
        self.plate = WellPlate.from_format(96)

    def test_no_state_uses_envelope_center_seed_not_zero_grid(self):
        # Neither dict, A1 untaught: must mirror the canvas geometry-only
        # seed (plate centred on the XY-envelope midpoint), NOT the old
        # zero-anchored grid.
        center = (60000.0, 40000.0)
        stub = _stub(self.plate, calibrated=None, predicted=None,
                     taught_a1=None, zero=(0.0, 0.0), plate_center=center)
        got = _predict(stub, "B2")

        seed = self.plate.get_all_positions_from_plate_center(*center)["B2"]
        self.assertAlmostEqual(got[0], seed[0])
        self.assertAlmostEqual(got[1], seed[1])

        # The old zero-anchored grid would have been A1-at-origin; with a
        # non-trivial plate center the seed differs from it.
        wells = {w.name: w for w in self.plate.get_all_wells()}
        a1, b2 = wells["A1"], wells["B2"]
        old_buggy = ((b2.x - a1.x) * 1000.0, (b2.y - a1.y) * 1000.0)
        self.assertNotEqual((round(got[0], 3), round(got[1], 3)),
                            (round(old_buggy[0], 3), round(old_buggy[1], 3)))

    def test_taught_a1_anchors_grid_when_no_dicts(self):
        # taught_a1 present but no position dicts → anchor the grid at the
        # taught A1 stage position (same formula as get_all_positions_from_a1).
        taught = (33000.0, 22000.0)
        stub = _stub(self.plate, calibrated=None, predicted=None,
                     taught_a1=taught)
        got = _predict(stub, "B2")
        wells = {w.name: w for w in self.plate.get_all_wells()}
        a1, b2 = wells["A1"], wells["B2"]
        expected = (taught[0] + (b2.x - a1.x) * 1000.0,
                    taught[1] + (b2.y - a1.y) * 1000.0)
        self.assertAlmostEqual(got[0], expected[0])
        self.assertAlmostEqual(got[1], expected[1])

    def test_last_resort_zero_anchor_when_center_unavailable(self):
        # No dicts, no taught A1, and default_plate_center_um raises →
        # last-resort zero-anchored grid (unchanged legacy behavior).
        stub = _stub(self.plate, calibrated=None, predicted=None,
                     taught_a1=None, zero=(100.0, 200.0), plate_center=None)
        got = _predict(stub, "B2")
        wells = {w.name: w for w in self.plate.get_all_wells()}
        a1, b2 = wells["A1"], wells["B2"]
        expected = (100.0 + (b2.x - a1.x) * 1000.0,
                    200.0 + (b2.y - a1.y) * 1000.0)
        self.assertAlmostEqual(got[0], expected[0])
        self.assertAlmostEqual(got[1], expected[1])


class TestSourceGuard(unittest.TestCase):
    """Lock the fix at the source level."""

    def test_method_no_longer_reads_dead_predicted_wells_attr(self):
        src = inspect.getsource(CalibrationPage._predict_well_xy)
        self.assertNotIn(
            "getattr(self, '_predicted_wells'", src,
            "The dead `_predicted_wells` lookup is back — _predict_well_xy "
            "will silently fall through to the geometry fallback again.",
        )

    def test_method_reads_the_displayed_position_dicts(self):
        src = inspect.getsource(CalibrationPage._predict_well_xy)
        self.assertIn("_calibrated_positions", src)
        self.assertIn("_predicted_positions", src)


if __name__ == "__main__":
    unittest.main()
