"""
v7.13 — the bore-dots overlay: colored dots on the microscope feed, one per
bore, at each bore's measured position in the camera field of view.

The projection contract is the part that can silently go wrong: a bore's FOV
offset is stage-frame µm relative to the camera centre (pto space), and it must
be projected to pixels through ``CameraManager.stage_offset_to_pixel`` — the
exact inverse of the click path — NEVER the naive identity divide. On a camera
with a calibrated rotation/mirror the identity map draws the dot away from the
bore (the v7.8 ``to_px`` lesson). The round-trip test here pins that: the drawn
pixel, fed back through ``pixel_to_stage_offset``, must reproduce the offset.
"""

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from gui.styles import COLORS
from gui.widgets.camera_feed_view import CameraFeedView
import gui.widgets.needle_bore_wizard as wizmod
from SupportClasses.PhysicalModels import NeedleBore, NeedleSpec
from gui.widgets.needle_bore_wizard import (
    STEP_BORES,
    BoreMeasurement,
    NeedleBoreWizard,
    bore_dot_color,
)


def _backpack():
    return NeedleSpec(needle_form="backpack", bores=[
        NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
        NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P2")])


class _OrientedMgr:
    """A camera manager with a rotated + mirrored calibration, implementing
    the REAL forward/inverse pair (the same math as CameraManager)."""

    def __init__(self, um_per_px=0.5, rot_deg=37.5, mirrored=True,
                 flip_y=False):
        self._upp = float(um_per_px)
        self._rot = float(rot_deg)
        self._mir = bool(mirrored)
        self._fy = bool(flip_y)
        self.cameras = [None] * 4          # CameraFeedView binds through this

    def effective_um_per_px(self, idx, w):
        return self._upp

    def pixel_to_stage_offset(self, idx, px, py, w, h):
        dx, dy = px - w / 2.0, py - h / 2.0
        if self._mir:
            dx = -dx
        if self._fy:
            dy = -dy
        dx, dy = dx * self._upp, dy * self._upp
        t = math.radians(self._rot)
        c, s = math.cos(t), math.sin(t)
        return (dx * c - dy * s, dx * s + dy * c)

    def stage_offset_to_pixel(self, idx, dx_um, dy_um, w, h):
        t = math.radians(-self._rot)
        c, s = math.cos(t), math.sin(t)
        x = dx_um * c - dy_um * s
        y = dx_um * s + dy_um * c
        x, y = x / self._upp, y / self._upp
        if self._mir:
            x = -x
        if self._fy:
            y = -y
        return (w / 2.0 + x, h / 2.0 + y)


class _Base(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


# ── projection ──────────────────────────────────────────────────────

class TestProjection(_Base):

    W, H = 1920, 1080

    def _view(self, mgr):
        v = CameraFeedView(camera_manager=mgr, cam_idx=2)
        self.addCleanup(v.deleteLater)
        return v

    def test_round_trip_through_a_rotated_mirrored_camera(self):
        """The drawn pixel, clicked, must report the bore's own offset."""
        mgr = _OrientedMgr(um_per_px=0.5, rot_deg=37.5, mirrored=True)
        v = self._view(mgr)
        for off in ((320.0, -140.0), (-880.0, 415.0), (0.0, 0.0)):
            px, py = v._bore_marker_raw_px(off[0], off[1], self.W, self.H)
            back = mgr.pixel_to_stage_offset(2, px, py, self.W, self.H)
            self.assertAlmostEqual(back[0], off[0], places=6)
            self.assertAlmostEqual(back[1], off[1], places=6)

    def test_the_naive_identity_projection_would_be_wrong_here(self):
        """The mutation this suite exists to catch: swapping the calibrated
        inverse for `centre + off/µm_per_px` survives every test on an
        unrotated camera — so pin that it FAILS on this one."""
        mgr = _OrientedMgr(um_per_px=0.5, rot_deg=37.5, mirrored=True)
        v = self._view(mgr)
        off = (320.0, -140.0)
        px, py = v._bore_marker_raw_px(off[0], off[1], self.W, self.H)
        naive = (self.W / 2.0 + off[0] / 0.5, self.H / 2.0 + off[1] / 0.5)
        self.assertGreater(math.hypot(px - naive[0], py - naive[1]), 50.0)

    def test_identity_fallback_without_a_manager_capable_of_inverse(self):
        mgr = SimpleNamespace(effective_um_per_px=lambda idx, w: 0.5,
                              cameras=[None] * 4)
        v = self._view(mgr)
        px, py = v._bore_marker_raw_px(100.0, -50.0, self.W, self.H)
        self.assertAlmostEqual(px, self.W / 2.0 + 200.0, places=6)
        self.assertAlmostEqual(py, self.H / 2.0 - 100.0, places=6)


class TestSetBoreMarkers(_Base):

    def _counting_view(self):
        v = CameraFeedView(camera_manager=None, cam_idx=0)
        self.addCleanup(v.deleteLater)
        v.renders = 0

        def _count():
            v.renders += 1
        v._rerender_last = _count
        return v

    def test_markers_are_stored_normalised(self):
        v = self._counting_view()
        v.set_bore_markers([("B1", 1, 2, "#a6e3a1")])
        self.assertEqual(v._bore_markers, [("B1", 1.0, 2.0, "#a6e3a1")])

    def test_an_identical_push_does_not_rerender(self):
        """The wizard pushes from a 3–7 Hz refresh; an unchanged push must be
        free or the overlay becomes a per-tick repaint."""
        v = self._counting_view()
        m = [("B1", 320.0, -140.0, "#a6e3a1")]
        v.set_bore_markers(m)
        v.set_bore_markers(list(m))
        self.assertEqual(v.renders, 1)

    def test_none_clears(self):
        v = self._counting_view()
        v.set_bore_markers([("B1", 1.0, 2.0, "#fff")])
        v.set_bore_markers(None)
        self.assertEqual(v._bore_markers, [])
        self.assertEqual(v.renders, 2)

    def test_garbage_entries_are_skipped_not_fatal(self):
        v = self._counting_view()
        v.set_bore_markers([("B1", 1.0, 2.0, "#fff"), ("bad",),
                            ("B2", "x", 0, "#fff")])
        self.assertEqual(len(v._bore_markers), 1)


# ── the wizard's marker sources ─────────────────────────────────────

class _Ctrl:
    def __init__(self):
        self.is_xy_connected = True
        self.cam_offset = None
        self.floor_calls = []

    def get_xy_position(self, cached=False):
        return (100_000.0, 50_000.0)

    def z_up_sign(self):
        return 1.0

    def get_needle_camera_offset_um(self):
        return self.cam_offset

    def print_z_dir(self):
        return 1.0

    def print_height_to_zref(self, h):
        return 10.0 + float(h)

    def set_print_floor_active(self, on):
        self.floor_calls.append(bool(on))

    def set_needle_camera_offset_um(self, dx, dy=None):
        self.cam_offset = (dx, dy)

    def capture_current_z_raw(self):
        return -22.5

    def raw_to_user_z(self, raw):
        return -float(raw)


class _Mgr:
    def __init__(self):
        self.cameras = [None] * 4

    def is_um_per_px_calibrated(self, idx):
        return True

    def effective_um_per_px(self, idx, w):
        return 0.5

    def start(self, idx):
        pass

    def pixel_to_stage_offset(self, idx, px, py, w, h):
        return ((px - w / 2.0) * 0.5, (py - h / 2.0) * 0.5)


class _WizBase(_Base):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_dots_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
            Path(self._tmp.name) / "bore.json")
        import SupportClasses.NeedleBoreCalibrationStore as mod
        self._prev_singleton = mod._store
        mod._store = None

        def _restore():
            mod._store = self._prev_singleton
            if self._prev is None:
                os.environ.pop("MEBP_NEEDLE_BORE_CAL_PATH", None)
            else:
                os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = self._prev
        self.addCleanup(_restore)

    def _wizard(self, needle=None):
        self.ctrl = _Ctrl()
        self.mgr = _Mgr()
        host = SimpleNamespace(
            controller=self.ctrl,
            _hardware_config=SimpleNamespace(
                needle=_backpack() if needle is None else needle),
            _camera_manager=self.mgr,
            _plate_bottom_z=10.0,
            _safe_z=44.0,
            _top_z=None,
            _max_z=None,
            _needle_origin_um=(1.0, 2.0),
            _needle_loc_xy_um=(100_000.0, 50_000.0),
            _ploc_microscope_cam_idx=lambda: 2,
        )
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        return w


class TestMarkerSources(_WizBase):

    def test_session_clicks_become_dots_immediately(self):
        w = self._wizard()
        w._meas = BoreMeasurement(2)
        w._meas.record_click(0, (960, 540), (-880.0, 415.0))
        w._meas.record_click(1, (1600, 540), (-560.0, 275.0))
        dots = w._session_bore_fov_offsets()
        self.assertEqual(len(dots), 2)
        self.assertEqual(dots[0][:3], ("B1", -880.0, 415.0))
        self.assertEqual(dots[1][:3], ("B2", -560.0, 275.0))

    def test_the_datum_dot_is_green(self):
        self.assertEqual(bore_dot_color(0), COLORS["green"])
        self.assertNotEqual(bore_dot_color(1), bore_dot_color(2))

    def test_stored_dots_from_needle_camera_offset_plus_offsets(self):
        """pto(P_k) = pto(P_0) + offset_um(k) — the exact reconstruction."""
        w = self._wizard()
        store = w._store()
        needle = w._needle()
        store.set_bore(0, (0.0, 0.0), 0.0, needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.0, needle=needle)
        self.ctrl.cam_offset = (-880.0, 415.0)
        dots = w._stored_bore_fov_offsets()
        self.assertEqual(dots[0][:3], ("B1", -880.0, 415.0))
        self.assertEqual(dots[1][:3], ("B2", -560.0, 275.0))

    def test_stored_dots_fall_back_to_pto_provenance(self):
        w = self._wizard()
        store = w._store()
        needle = w._needle()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=(-880.0, 415.0),
                       needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.0, stage_um=(-560.0, 275.0),
                       needle=needle)
        self.ctrl.cam_offset = None
        dots = w._stored_bore_fov_offsets()
        self.assertEqual(dots[1][:3], ("B2", -560.0, 275.0))

    def test_legacy_absolute_provenance_is_rejected(self):
        """The side-camera path wrote ABSOLUTE stage µm into stage_um (with
        the opposite difference sense). Projecting those would draw dots at
        plausible-looking wrong places — the invariant must reject them."""
        w = self._wizard()
        store = w._store()
        needle = w._needle()
        store.set_bore(0, (0.0, 0.0), 0.0, stage_um=(100_000.0, 50_000.0),
                       needle=needle)
        store.set_bore(1, (320.0, -140.0), 0.0,
                       stage_um=(99_680.0, 50_140.0), needle=needle)
        self.ctrl.cam_offset = None
        self.assertIsNone(w._stored_bore_fov_offsets())

    def test_no_source_hides_the_toggle(self):
        """isHidden (the widget's OWN flag), not isVisibleTo: in a standalone
        wizard the microscope pane sits hidden until a host mounts it."""
        w = self._wizard()
        w._push_bore_markers()
        self.assertTrue(w._bore_dots_chk.isHidden())

    def test_a_source_shows_the_toggle_and_pushes(self):
        w = self._wizard()
        w._meas = BoreMeasurement(2)
        w._meas.record_click(0, (960, 540), (-880.0, 415.0))
        w._push_bore_markers()
        self.assertFalse(w._bore_dots_chk.isHidden())
        self.assertEqual(len(w._mic_feed._bore_markers), 1)

    def test_unchecking_the_toggle_clears_the_dots(self):
        w = self._wizard()
        w._meas = BoreMeasurement(2)
        w._meas.record_click(0, (960, 540), (-880.0, 415.0))
        w._push_bore_markers()
        w._bore_dots_chk.setChecked(False)
        self.assertEqual(w._mic_feed._bore_markers, [])

    def test_commit_turns_the_overlay_on(self):
        w = self._wizard()
        w.go_to_step(STEP_BORES)
        w._mic_feed.image_size          # real feed; use direct session
        w._on_start_session()
        w._meas.record_click(0, (960, 540), (0.0, 0.0))
        w._meas.record_click(1, (1600, 540), (320.0, 0.0))
        w._meas.record_z(0, 22.5)
        w._meas.record_z(1, 22.548)
        w._bore_dots_chk.setChecked(False)
        w._on_commit_bores()
        self.assertTrue(w._bore_dots_chk.isChecked())
        self.assertTrue(w._store().measured_bore_indices())


if __name__ == "__main__":
    unittest.main()
