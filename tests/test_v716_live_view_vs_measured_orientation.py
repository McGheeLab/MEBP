"""v7.16 — the LIVE-VIEW orientation is separate from the MEASURED one.

Operator: *"1) I setup the camera live view so that the image is showing a
correctly rotated and mirrored view. 2) I then go into the objective calibration
for the mosaic builder, and once it is done it always flips the camera back to
the wrong orientation for the live view. If we need to, we can separate the
mosaic settings from the camera live view settings."*

They were right about the mechanism. One ``(rotation_deg, mirrored, flip_y)``
triple per camera identity served three consumers — the live display,
``MosaicBuilder._orient_tile`` and ``CameraManager.pixel_to_stage_offset`` — and
had TWO writers:

* the per-slot Flip X / Flip Y / Rotation controls on Hardware Setup → Cameras
  (a *viewing* preference), and
* ``derive_camera_stage_orientation`` inside the objective/mosaic calibration
  (the *measured* camera→stage matrix).

The measurement won, so step 1 then step 2 always ended with the view flipped
back.

They are genuinely different quantities, which is why no precedence rule could
have fixed this. The measured triple makes the display STAGE-ALIGNED (image +X →
stage +X); on a rig with ``plate_flip_180`` the plate then reads 180° from the
A1-top-left convention every other plate view uses, so the orientation the
operator wants to LOOK at is legitimately not the one the mosaic NEEDS.

What makes the split safe is that the display was already decoupled at the
plumbing level: ``CameraFeedView`` reports clicks in RAW frame coordinates (it
inverts its own display transform via ``ViewGeometry``) and
``pixel_to_stage_offset`` then applies the MEASURED orientation. So moving the
display does not move where a click lands — pinned below by
``TestTheDisplayCannotMoveTheGeometry``.
"""

import json
import shutil
import tempfile
import unittest
from pathlib import Path

_QT = True
try:
    from PySide6.QtWidgets import QApplication
except Exception:                                        # pragma: no cover
    _QT = False

_app = None


def setUpModule():
    global _app
    if _QT:
        _app = QApplication.instance() or QApplication([])


# ── The store ────────────────────────────────────────────────────────────────

class TestStoreSplit(unittest.TestCase):
    def setUp(self):
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self.dir = Path(tempfile.mkdtemp())
        self.path = self.dir / "cam_cal.json"
        self.Store = CameraCalibrationStore
        self.store = CameraCalibrationStore(self.path)

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_absent_by_default(self):
        """Absent means 'no preference', which callers read as 'follow the
        measurement' — the pre-v7.16 behaviour."""
        self.assertIsNone(self.store.get_view_orientation("id"))

    def test_round_trip_and_persist(self):
        self.store.set_view_orientation("id", 180.0, True, False, name="Cam")
        want = {"rotation_deg": 180.0, "flip_x": True, "flip_y": False}
        self.assertEqual(self.store.get_view_orientation("id"), want)
        self.assertEqual(self.Store(self.path).get_view_orientation("id"), want)

    def test_neutral_is_stored_explicitly_not_popped(self):
        """A deliberately neutral view must SURVIVE a later measurement.

        If it popped the key the measured orientation would take the view back
        over, which is the very bug — just harder to see.
        """
        self.store.set_view_orientation("id", 0.0, False, False)
        self.assertEqual(self.store.get_view_orientation("id"),
                         {"rotation_deg": 0.0, "flip_x": False, "flip_y": False})
        self.store.set_rotation("id", -90.0)
        self.assertEqual(self.store.get_view_orientation("id")["rotation_deg"],
                         0.0)

    def test_preserves_every_sibling(self):
        self.store.set_calibration("id", 3.3, um_per_px_resolution=(2600, 2048),
                                   rotation_deg=-45.0, name="Cam")
        self.store.set_crop("id", {"mode": "square", "scale": 0.8})
        self.store.set_view_orientation("id", 90.0, True, True)
        e = self.store.get_calibration("id")
        self.assertAlmostEqual(e["um_per_px"], 3.3)
        self.assertAlmostEqual(e["rotation_deg"], -45.0)
        self.assertEqual(e["crop"]["mode"], "square")

    def test_a_measurement_never_touches_the_view(self):
        """THE contract. Each of the three measured setters, separately."""
        self.store.set_view_orientation("id", 180.0, True, False)
        want = {"rotation_deg": 180.0, "flip_x": True, "flip_y": False}
        self.store.set_rotation("id", -90.0)
        self.assertEqual(self.store.get_view_orientation("id"), want)
        self.store.set_mirrored("id", False)
        self.assertEqual(self.store.get_view_orientation("id"), want)
        self.store.set_flip_y("id", True)
        self.assertEqual(self.store.get_view_orientation("id"), want)
        self.store.set_calibration("id", 1.5, rotation_deg=12.0)
        self.assertEqual(self.store.get_view_orientation("id"), want)

    def test_clear_hands_the_view_back_to_the_measurement(self):
        self.store.set_view_orientation("id", 180.0, True, False)
        self.store.clear_view_orientation("id")
        self.assertIsNone(self.store.get_view_orientation("id"))

    def test_malformed_reports_absent_not_neutral(self):
        """Absent falls back to the measured orientation (recoverable);
        pretending 'no flips' would show a mirrored feed as if it were fine."""
        self.store.set_calibration("id", 2.0, rotation_deg=180.0)
        self.store._data["cameras"]["id"]["view_orientation"] = "junk"
        self.assertIsNone(self.store.get_view_orientation("id"))


class TestMigration12to13(unittest.TestCase):
    """Every camera must look EXACTLY as it did before the upgrade."""

    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.path = self.dir / "cam_cal.json"

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def _write(self, cameras):
        self.path.write_text(json.dumps(
            {"version": "1.2", "cameras": cameras, "assignments": {}}),
            encoding="utf-8")

    def test_seeds_view_from_the_previous_geometry(self):
        from SupportClasses.CameraCalibrationStore import (
            CameraCalibrationStore, SCHEMA_VERSION)
        self._write({"tucam:0": {"um_per_px": 1.271, "rotation_deg": 180.0,
                                 "mirrored": True, "name": "Tucsen"}})
        st = CameraCalibrationStore(self.path)
        self.assertEqual(st._data["version"], SCHEMA_VERSION)
        self.assertEqual(st.get_view_orientation("tucam:0"),
                         {"rotation_deg": 180.0, "flip_x": True,
                          "flip_y": False})
        # …and the measured values are left exactly as they were.
        e = st.get_calibration("tucam:0")
        self.assertAlmostEqual(e["rotation_deg"], 180.0)
        self.assertTrue(e["mirrored"])
        self.assertAlmostEqual(e["um_per_px"], 1.271)

    def test_a_neutral_camera_is_left_byte_identical(self):
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self._write({"plain:1": {"um_per_px": 3.2, "name": "Andor"}})
        st = CameraCalibrationStore(self.path)
        self.assertIsNone(st.get_view_orientation("plain:1"))
        self.assertNotIn("view_orientation", st.get_calibration("plain:1"))

    def test_flip_y_only_camera_is_seeded(self):
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self._write({"c:2": {"flip_y": True}})
        st = CameraCalibrationStore(self.path)
        self.assertEqual(st.get_view_orientation("c:2"),
                         {"rotation_deg": 0.0, "flip_x": False,
                          "flip_y": True})

    def test_migration_is_idempotent_and_never_reseeds(self):
        """Re-running must not overwrite a view the operator has since changed —
        which is what a re-seed on every load would do."""
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        self._write({"c:0": {"rotation_deg": 180.0}})
        st = CameraCalibrationStore(self.path)
        st.set_view_orientation("c:0", 0.0, False, False)
        st._data["version"] = "1.2"          # force the chain to run again
        st.save()
        st2 = CameraCalibrationStore(self.path)
        self.assertEqual(st2.get_view_orientation("c:0")["rotation_deg"], 0.0)


# ── The manager ──────────────────────────────────────────────────────────────

@unittest.skipUnless(_QT, "PySide6 unavailable")
class TestManagerDisplayOrientation(unittest.TestCase):
    def setUp(self):
        from gui.widgets.camera_manager import CameraManager
        self.mgr = CameraManager()
        self.mgr.set_rotation_deg(0, -90.0)
        self.mgr.set_mirrored(0, False)
        self.mgr.set_flip_y(0, True)

    def test_falls_back_to_the_measurement_when_unset(self):
        self.assertFalse(self.mgr.has_display_orientation(0))
        self.assertEqual(self.mgr.display_orientation(0),
                         self.mgr.full_orientation(0))
        self.assertEqual(self.mgr.display_orientation(0), (False, True, -90.0))

    def test_an_explicit_view_wins_and_is_reported_as_explicit(self):
        self.mgr.set_display_orientation(0, True, False, 180.0)
        self.assertTrue(self.mgr.has_display_orientation(0))
        self.assertEqual(self.mgr.display_orientation(0), (True, False, 180.0))
        # The measurement is untouched.
        self.assertEqual(self.mgr.full_orientation(0), (False, True, -90.0))

    def test_field_order_matches_full_orientation(self):
        """Both return flips FIRST. A caller swapping between them must not be
        able to silently trade a mirror for an angle."""
        self.mgr.set_display_orientation(0, True, False, 33.0)
        fx, fy, rot = self.mgr.display_orientation(0)
        self.assertIs(fx, True)
        self.assertIs(fy, False)
        self.assertAlmostEqual(rot, 33.0)

    def test_clear_returns_to_the_measurement(self):
        self.mgr.set_display_orientation(0, True, True, 45.0)
        self.mgr.clear_display_orientation(0)
        self.assertFalse(self.mgr.has_display_orientation(0))
        self.assertEqual(self.mgr.display_orientation(0), (False, True, -90.0))

    def test_slots_are_independent(self):
        self.mgr.set_display_orientation(0, True, False, 180.0)
        self.assertFalse(self.mgr.has_display_orientation(1))


@unittest.skipUnless(_QT, "PySide6 unavailable")
class TestTheFirstEditSeedsFromWhatIsOnScreen(unittest.TestCase):
    """The operator's first flip must mean "what I'm looking at, PLUS this flip".

    Before any preference exists the feed shows the measured orientation, and the
    controls display it. If the first edit committed only the field it names and
    zeroed the rest, ticking Flip X would silently drop the measured flip Y and
    reset the rotation to 0 — the picture would jump for no stated reason. So a
    partial commit starts from the orientation currently in force.

    Found by measuring the real page rather than by reading the code: the first
    version of the smoke check asserted the other behaviour.
    """

    def setUp(self):
        from gui.widgets.camera_manager import CameraManager
        self.mgr = CameraManager()
        self.mgr.set_rotation_deg(0, -90.0)
        self.mgr.set_mirrored(0, False)
        self.mgr.set_flip_y(0, True)

    def _page(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        pg = HardwareSetupPage()
        pg.set_camera_manager(self.mgr)
        return pg

    def test_first_flip_keeps_the_rest_of_the_visible_orientation(self):
        pg = self._page()
        pg._refresh_slot_rotation_displays()
        # What the operator is looking at, straight off the measurement.
        self.assertEqual(self.mgr.display_orientation(0), (False, True, -90.0))
        pg._apply_slot_mirror(0, True)
        # …plus flip X, and NOTHING else changed.
        self.assertEqual(self.mgr.display_orientation(0), (True, True, -90.0))

    def test_the_controls_show_the_orientation_being_edited(self):
        """WYSIWYG: the checkboxes/spin must already reflect the fallback, or the
        seeding above would look like it invented values."""
        pg = self._page()
        pg._refresh_slot_rotation_displays()
        self.assertFalse(pg._live_cam_mirror_checks[0].isChecked())
        self.assertTrue(pg._live_cam_flip_y_checks[0].isChecked())
        self.assertAlmostEqual(pg._live_cam_rot_spins[0].value(), -90.0)

    def test_readout_distinguishes_measured_from_live_view(self):
        pg = self._page()
        pg._apply_slot_mirror(0, True)
        pg._apply_slot_rotation_value(0, 180.0)
        self.mgr.set_rotation_deg(0, 37.0)
        pg._refresh_slot_rotation_displays()
        txt = pg._live_cam_rot_labels[0].text()
        self.assertIn("37.0", txt)              # the MEASURED angle
        self.assertIn("live view", txt)         # …named separately from
        self.assertIn("+180.0", txt)            #    the view's own angle


# ── The live view ────────────────────────────────────────────────────────────

@unittest.skipUnless(_QT, "PySide6 unavailable")
class TestTheFeedFollowsTheViewNotTheMeasurement(unittest.TestCase):
    """Drives the PRODUCTION ``CameraFeedView``. A stand-in that reads the right
    accessor proves nothing about the widget the operator is looking at."""

    def _view(self, mgr):
        from gui.widgets.camera_feed_view import CameraFeedView
        v = CameraFeedView(camera_manager=mgr, cam_idx=0, auto_orient=True)
        v._sync_auto_orientation()
        return v

    def _mgr(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        mgr.set_rotation_deg(0, -90.0)
        mgr.set_mirrored(0, False)
        mgr.set_flip_y(0, True)
        return mgr

    def test_reads_the_display_orientation(self):
        mgr = self._mgr()
        mgr.set_display_orientation(0, True, False, 180.0)
        v = self._view(mgr)
        self.assertEqual(
            (v._view_mirror, v._view_flip_y, v._view_rot_deg),
            (True, False, 180.0))

    def test_a_measurement_does_not_move_the_feed(self):
        """THE reported defect, against the real widget."""
        mgr = self._mgr()
        mgr.set_display_orientation(0, True, False, 180.0)
        v = self._view(mgr)
        before = (v._view_mirror, v._view_flip_y, v._view_rot_deg)
        # The objective/mosaic calibration commits a fresh measurement.
        mgr.set_rotation_deg(0, 37.0)
        mgr.set_mirrored(0, True)
        mgr.set_flip_y(0, False)
        v._sync_auto_orientation()
        self.assertEqual(
            (v._view_mirror, v._view_flip_y, v._view_rot_deg), before)

    def test_without_a_preference_the_feed_still_follows_the_measurement(self):
        """Guard the guard: the tests above must not be passing merely because
        the feed ignores orientation altogether."""
        mgr = self._mgr()
        v = self._view(mgr)
        self.assertEqual(
            (v._view_mirror, v._view_flip_y, v._view_rot_deg),
            (False, True, -90.0))
        mgr.set_rotation_deg(0, 12.0)
        v._sync_auto_orientation()
        self.assertAlmostEqual(v._view_rot_deg, 12.0)

    def test_a_manager_without_the_new_accessor_still_works(self):
        """Lightweight doubles and older managers fall back to
        ``full_orientation`` rather than losing their orientation."""
        mgr = self._mgr()

        class _Old:
            """A manager WITHOUT display_orientation — wraps a real one so the
            widget still finds the surface it needs."""

            def __init__(self, inner):
                self._inner = inner

            def __getattr__(self, name):
                if name == "display_orientation":
                    raise AttributeError(name)
                return getattr(self._inner, name)

            def full_orientation(self, i):
                return (True, False, 90.0)

        old = _Old(mgr)
        self.assertIsNone(getattr(old, "display_orientation", None))
        v = self._view(old)
        self.assertEqual(
            (v._view_mirror, v._view_flip_y, v._view_rot_deg),
            (True, False, 90.0))


# ── The safety claim ─────────────────────────────────────────────────────────

@unittest.skipUnless(_QT, "PySide6 unavailable")
class TestTheDisplayCannotMoveTheGeometry(unittest.TestCase):
    """What makes the split safe: clicks are reported in RAW frame coordinates
    and mapped through the MEASURED orientation, so the display is free to
    differ."""

    def setUp(self):
        from gui.widgets.camera_manager import CameraManager
        self.mgr = CameraManager()
        self.mgr.set_um_per_px(0, 2.0)
        self.mgr.set_rotation_deg(0, 0.0)
        self.mgr.set_mirrored(0, False)
        self.mgr.set_flip_y(0, False)

    def test_pixel_to_stage_is_byte_identical_across_every_view(self):
        base = self.mgr.pixel_to_stage_offset(0, 130.0, 80.0, 200, 200)
        for fx in (False, True):
            for fy in (False, True):
                for rot in (0.0, 90.0, 180.0, -90.0, 37.5):
                    with self.subTest(flip_x=fx, flip_y=fy, rot=rot):
                        self.mgr.set_display_orientation(0, fx, fy, rot)
                        self.assertEqual(
                            self.mgr.pixel_to_stage_offset(
                                0, 130.0, 80.0, 200, 200), base)

    def test_the_measured_orientation_still_moves_the_click(self):
        """The companion assertion — otherwise the test above could pass because
        nothing maps clicks at all."""
        base = self.mgr.pixel_to_stage_offset(0, 130.0, 80.0, 200, 200)
        self.mgr.set_mirrored(0, True)
        self.assertNotEqual(
            self.mgr.pixel_to_stage_offset(0, 130.0, 80.0, 200, 200), base)

    def test_the_mosaic_resolver_ignores_the_view(self):
        from SupportClasses.CameraCalibrationStore import CameraCalibrationStore
        from SupportClasses.MosaicCalibration import resolve_camera_orientation
        d = Path(tempfile.mkdtemp())
        try:
            st = CameraCalibrationStore(d / "c.json")
            st.set_calibration("cam:0", 1.5, rotation_deg=-90.0)
            st.set_flip_y("cam:0", True)
            st.set_view_orientation("cam:0", 180.0, True, False)
            self.assertEqual(
                resolve_camera_orientation(identity="cam:0", cal_store=st),
                (-90.0, False, True))
        finally:
            shutil.rmtree(d, ignore_errors=True)


# ── Manager ↔ store round trip ───────────────────────────────────────────────

@unittest.skipUnless(_QT, "PySide6 unavailable")
class TestRestoreFromStore(unittest.TestCase):
    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        from gui.widgets.camera_manager import CameraManager
        self.CCS = CCS
        self.dir = Path(tempfile.mkdtemp())
        self._saved = getattr(CCS, "_store", None)
        CCS._store = CCS.CameraCalibrationStore(self.dir / "c.json")
        self.store = CCS._store
        self.mgr = CameraManager()
        self.mgr.camera_identity = lambda i: ("dev:A", "Cam A")

    def tearDown(self):
        self.CCS._store = self._saved
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_restores_the_view_alongside_the_measurement(self):
        self.store.set_calibration("dev:A", 1.5, um_per_px_resolution=(100, 100),
                                   rotation_deg=-90.0)
        self.store.set_view_orientation("dev:A", 180.0, True, False)
        self.assertTrue(self.mgr.restore_calibration_from_store(0))
        self.assertEqual(self.mgr.display_orientation(0), (True, False, 180.0))
        self.assertAlmostEqual(self.mgr.get_rotation_deg(0), -90.0)

    def test_restore_clears_a_stale_view(self):
        """⚠ CLEARS as well as sets. A slot's widget outlives the source
        assigned to it, so a camera with no preference must hand the view back to
        the measurement — otherwise reassigning a slot leaves the new camera
        showing the previous one's rotation."""
        self.mgr.set_display_orientation(0, True, True, 180.0)
        self.store.set_calibration("dev:A", 1.5,
                                   um_per_px_resolution=(100, 100))
        self.assertTrue(self.mgr.restore_calibration_from_store(0))
        self.assertFalse(self.mgr.has_display_orientation(0))


if __name__ == "__main__":
    unittest.main()
