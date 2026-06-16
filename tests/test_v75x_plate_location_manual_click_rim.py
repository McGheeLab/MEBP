"""
v7.5.x — Plate Location manual click-rim well fit + objective confirmation.

New workflow: in Manual mode the run drives the objective to N predicted rim
points around each queued well; at each the operator clicks the well EDGE in a
live microscope view. The clicks (converted to absolute stage µm) are circle-fit
to the well center. A pre-run dialog makes the operator confirm the in-use
objective matches the one selected in the config.

These tests call the real (unbound) CalibrationPage methods with duck-typed
stubs so no Qt widget / camera / event loop is needed. `QMessageBox` is patched
so dialog paths don't require a QApplication. The click→stage-µm conversion runs
the REAL `CameraManager.pixel_to_stage_offset`.
"""

import math
import unittest
from types import SimpleNamespace

from gui.pages.calibration import CalibrationPage
import gui.pages.calibration as calmod
from gui.widgets.camera_manager import CameraManager

try:
    from gui.pages.calibration import fit_circle_to_points as _FIT
except Exception:  # pragma: no cover
    _FIT = None


class _FakeMB:
    """Stand-in for QMessageBox in headless tests."""
    Yes = 1
    No = 0
    question_return = 1  # default Yes
    warned = 0

    @classmethod
    def reset(cls):
        cls.question_return = cls.Yes
        cls.warned = 0

    @classmethod
    def warning(cls, *a, **k):
        cls.warned += 1
        return cls.No

    @classmethod
    def question(cls, *a, **k):
        return cls.question_return


class _FakeMgr:
    """Minimal CameraManager exposing the REAL pixel_to_stage_offset."""
    def __init__(self, um_per_px):
        self._upp = um_per_px

    def get_um_per_px(self, idx):
        return self._upp

    def pixel_to_stage_offset(self, *args):
        return CameraManager.pixel_to_stage_offset(self, *args)


def _live_view(image_size):
    return SimpleNamespace(
        image_size=image_size,
        cam_idx=0,
        set_overlay_vector=lambda *a, **k: None,
        set_camera=lambda *a, **k: None,
    )


class TestClickToStageConversion(unittest.TestCase):
    """A live-view edge click → absolute stage µm = stage center + offset."""

    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb

    def _stub(self, **over):
        moves = []
        stub = SimpleNamespace(
            _ploc_running=True,
            _ploc_pause_kind="well_click_rim",
            _camera_manager=_FakeMgr(2.0),  # 2 µm/px
            _ploc_live_cam_idx=0,
            _ploc_live_view=_live_view((100, 80)),
            controller=SimpleNamespace(
                get_xy_position=lambda cached=False: (1000.0, 2000.0, 0.0),
                move_xy_absolute_um=lambda *a, **k: moves.append(a)),
            _ploc_click_points=[],
            _ploc_manual_well="B2",
            _PLOC_RIM_SAMPLES=CalibrationPage._PLOC_RIM_SAMPLES,
            _ploc_confirm_label=SimpleNamespace(setText=lambda t: None),
            _ploc_live_hint=SimpleNamespace(setText=lambda t: None),
        )
        stub._moves = moves
        # Bind the real prompt-refresh so the click path runs end-to-end.
        stub._ploc_update_click_rim_prompt = (
            CalibrationPage._ploc_update_click_rim_prompt.__get__(stub))
        for k, v in over.items():
            setattr(stub, k, v)
        return stub

    def test_click_offset_added_to_stage_center_in_um(self):
        stub = self._stub()
        # click at (60,50); center=(50,40); offset px=(10,10) → µm=(20,20)
        CalibrationPage._ploc_on_live_view_click(stub, 60.0, 50.0)
        self.assertEqual(len(stub._ploc_click_points), 1)
        sx, sy = stub._ploc_click_points[0]
        self.assertAlmostEqual(sx, 1000.0 + 20.0)
        self.assertAlmostEqual(sy, 2000.0 + 20.0)

    def test_click_does_not_move_the_stage(self):
        # Jog-driven: a click records a point but must NOT drive the stage
        # (no teleport to a predicted rim point).
        stub = self._stub()
        CalibrationPage._ploc_on_live_view_click(stub, 60.0, 50.0)
        self.assertEqual(stub._moves, [])

    def test_multiple_clicks_accumulate_at_live_positions(self):
        # Two clicks from two different (jogged) stage positions both record
        # relative to the position read at click time.
        positions = [(1000.0, 2000.0, 0.0), (1500.0, 2500.0, 0.0)]
        stub = self._stub()
        stub.controller.get_xy_position = lambda cached=False: positions.pop(0)
        CalibrationPage._ploc_on_live_view_click(stub, 50.0, 40.0)  # center
        CalibrationPage._ploc_on_live_view_click(stub, 50.0, 40.0)  # center
        self.assertEqual(stub._ploc_click_points,
                         [(1000.0, 2000.0), (1500.0, 2500.0)])

    def test_click_left_of_center_is_negative_offset(self):
        stub = self._stub()
        CalibrationPage._ploc_on_live_view_click(stub, 40.0, 30.0)
        sx, sy = stub._ploc_click_points[0]
        # px offset (-10,-10) → µm (-20,-20)
        self.assertAlmostEqual(sx, 980.0)
        self.assertAlmostEqual(sy, 1980.0)

    def test_no_op_when_not_in_click_rim_pause(self):
        stub = self._stub(_ploc_pause_kind="freeform")
        CalibrationPage._ploc_on_live_view_click(stub, 60.0, 50.0)
        self.assertEqual(stub._ploc_click_points, [])

    def test_no_op_when_not_running(self):
        stub = self._stub(_ploc_running=False)
        CalibrationPage._ploc_on_live_view_click(stub, 60.0, 50.0)
        self.assertEqual(stub._ploc_click_points, [])

    def test_no_op_when_no_frame_yet(self):
        stub = self._stub(_ploc_live_view=_live_view((0, 0)))
        CalibrationPage._ploc_on_live_view_click(stub, 60.0, 50.0)
        self.assertEqual(stub._ploc_click_points, [])


class TestBeginClickRim(unittest.TestCase):
    """_ploc_begin_well_click_rim anchors once at the predicted well center
    and then waits for jogged edge clicks (no automatic rim stepping)."""

    def _stub(self):
        moves = []
        stub = SimpleNamespace(
            _ploc_running=True,
            _ploc_run_idx=0,
            _predict_well_xy=lambda name: (5000.0, 6000.0),
            _plate=SimpleNamespace(well_diameter=6.4),
            _PLOC_RIM_SAMPLES=CalibrationPage._PLOC_RIM_SAMPLES,
            _ploc_ensure_live_camera=lambda: None,
            _ploc_live_view=SimpleNamespace(
                cam_idx=0, set_overlay_vector=lambda *a, **k: None),
            controller=SimpleNamespace(
                move_xy_absolute_um=lambda *a, **k: moves.append(a)),
            _ploc_click_points=[],
            _ploc_confirm_label=SimpleNamespace(
                setVisible=lambda v: None, setText=lambda t: None),
            _ploc_live_hint=SimpleNamespace(setText=lambda t: None),
            _ploc_btn_confirm=SimpleNamespace(setVisible=lambda v: None),
            _ploc_btn_skip=SimpleNamespace(setVisible=lambda v: None),
            _ploc_btn_cancel=SimpleNamespace(setVisible=lambda v: None),
        )
        stub._moves = moves
        stub._ploc_update_click_rim_prompt = (
            CalibrationPage._ploc_update_click_rim_prompt.__get__(stub))
        return stub

    def test_anchors_at_predicted_center_only(self):
        stub = self._stub()
        CalibrationPage._ploc_begin_well_click_rim(stub, "B2")
        self.assertEqual(stub._ploc_pause_kind, "well_click_rim")
        self.assertEqual(stub._ploc_manual_well, "B2")
        self.assertEqual(stub._ploc_click_points, [])
        # Exactly one automatic move — to the predicted well center.
        self.assertEqual(stub._moves, [(5000.0, 6000.0)])


@unittest.skipIf(_FIT is None, "opencv/vision helpers unavailable")
class TestFinalizeFit(unittest.TestCase):
    """_ploc_finalize_click_rim circle-fits the clicked rim points."""

    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb

    def _stub(self, points):
        advanced = []
        return SimpleNamespace(
            _ploc_manual_well="C3",
            _ploc_click_points=list(points),
            _plate=SimpleNamespace(well_diameter=6.4),  # r_exp = 3200 µm
            _PLOC_RADIUS_TOL=CalibrationPage._PLOC_RADIUS_TOL,
            _ploc_well_results={},
            _ploc_run_idx=0,
            _ploc_live_view=SimpleNamespace(
                set_overlay_vector=lambda *a, **k: None),
            _ploc_live_hint=SimpleNamespace(setText=lambda t: None),
            _ploc_exit_confirm_mode=lambda: None,
            _ploc_refresh_queue_view=lambda: None,
            _ploc_advance=lambda: advanced.append(True),
        ), advanced

    @staticmethod
    def _circle_pts(cx, cy, r, n=8):
        return [(cx + r * math.cos(2 * math.pi * i / n),
                 cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]

    def test_records_center_from_rim_clicks(self):
        cx, cy, r = 5000.0, 6000.0, 3200.0
        stub, advanced = self._stub(self._circle_pts(cx, cy, r))
        CalibrationPage._ploc_finalize_click_rim(stub)
        self.assertIn("C3", stub._ploc_well_results)
        gx, gy = stub._ploc_well_results["C3"]
        self.assertAlmostEqual(gx, cx, delta=1.0)
        self.assertAlmostEqual(gy, cy, delta=1.0)
        self.assertEqual(stub._ploc_run_idx, 1)
        self.assertTrue(advanced)
        self.assertEqual(stub._ploc_pause_kind, None)

    def test_fewer_than_three_clicks_leaves_uncalibrated(self):
        stub, advanced = self._stub([(1.0, 2.0), (3.0, 4.0)])
        CalibrationPage._ploc_finalize_click_rim(stub)
        self.assertNotIn("C3", stub._ploc_well_results)
        self.assertEqual(stub._ploc_run_idx, 1)  # still advances
        self.assertTrue(advanced)
        self.assertGreaterEqual(_FakeMB.warned, 1)

    def test_implausible_radius_rejected(self):
        # Points on a circle far from the expected 3200 µm radius.
        stub, advanced = self._stub(self._circle_pts(5000.0, 6000.0, 100.0))
        CalibrationPage._ploc_finalize_click_rim(stub)
        self.assertNotIn("C3", stub._ploc_well_results)
        self.assertGreaterEqual(_FakeMB.warned, 1)


class TestSkipWell(unittest.TestCase):
    """Skip in click-rim mode leaves the well uncalibrated and advances."""

    def test_skip_advances_without_recording(self):
        advanced = []
        stub = SimpleNamespace(
            _ploc_running=True,
            _ploc_pause_kind="well_click_rim",
            _ploc_manual_well="B2",
            _ploc_click_points=[(1.0, 2.0)],
            _ploc_run_idx=3,
            _ploc_well_results={},
            _ploc_live_view=SimpleNamespace(
                set_overlay_vector=lambda *a, **k: None),
            _ploc_live_hint=SimpleNamespace(setText=lambda t: None),
            _ploc_exit_confirm_mode=lambda: None,
            _ploc_refresh_queue_view=lambda: None,
            _ploc_advance=lambda: advanced.append(True),
        )
        CalibrationPage._ploc_skip(stub)
        self.assertEqual(stub._ploc_run_idx, 4)
        self.assertNotIn("B2", stub._ploc_well_results)
        self.assertIsNone(stub._ploc_pause_kind)
        self.assertTrue(advanced)


class TestModeToggle(unittest.TestCase):
    def test_mode_changes_when_idle(self):
        stub = SimpleNamespace(
            _ploc_running=False,
            _ploc_mode_combo=SimpleNamespace(currentData=lambda: "auto"),
            _ploc_fit_mode="manual",
        )
        CalibrationPage._ploc_on_mode_changed(stub, 1)
        self.assertEqual(stub._ploc_fit_mode, "auto")

    def test_mode_locked_during_run(self):
        stub = SimpleNamespace(
            _ploc_running=True,
            _ploc_mode_combo=SimpleNamespace(currentData=lambda: "auto"),
            _ploc_fit_mode="manual",
        )
        CalibrationPage._ploc_on_mode_changed(stub, 1)
        self.assertEqual(stub._ploc_fit_mode, "manual")  # unchanged


class TestObjectiveConfirmation(unittest.TestCase):
    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb

    def _stub(self, objective):
        return SimpleNamespace(
            _hardware_config=SimpleNamespace(
                camera_config=SimpleNamespace(
                    current_objective_name=objective)),
        )

    def test_blocks_when_no_objective_selected(self):
        stub = self._stub(None)
        ok = CalibrationPage._ploc_confirm_objective(stub, 0, 1.67)
        self.assertFalse(ok)
        self.assertGreaterEqual(_FakeMB.warned, 1)

    def test_proceeds_when_operator_confirms(self):
        _FakeMB.question_return = _FakeMB.Yes
        stub = self._stub("4x")
        self.assertTrue(CalibrationPage._ploc_confirm_objective(stub, 0, 1.67))

    def test_aborts_when_operator_declines(self):
        _FakeMB.question_return = _FakeMB.No
        stub = self._stub("4x")
        self.assertFalse(CalibrationPage._ploc_confirm_objective(stub, 0, 1.67))


if __name__ == "__main__":
    unittest.main()
