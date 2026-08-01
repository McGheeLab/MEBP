"""test_v78_spheroid_survey_tab.py — the survey tab: detect, curate, transfer,
editable mosaic circles, and the safety gates on every motion path.

The gate tests are the important ones. A mosaic-derived coordinate can be off by
up to 20 % of a field of view when the registration shift was never recorded, and
the stage must never be driven from one — nor while a mosaic scan is already
driving it.
"""

from __future__ import annotations

import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from SupportClasses import SpheroidDetector as sd

try:
    from PySide6.QtCore import QPointF, Qt
    from PySide6.QtWidgets import QApplication
    _QT = True
except Exception:      # pragma: no cover
    _QT = False


EXTENT = (1000.0, 2000.0, 2600.0, 3200.0)
SCALE = 0.5          # mosaic px per µm → 800 x 600 px
SHIFT = (30.0, -20.0)


def _mosaic(h=600, w=800):
    img = np.full((h, w, 3), 8, dtype=np.uint8)
    return img


def _context(*, has_shift=True, scale=SCALE, well="A1", channels=("DAPI",),
             scale_warning="", um_per_px=2.0):
    return {
        "plate_key": "24", "well": well, "channel": channels[0],
        "channels": list(channels), "image": _mosaic(),
        "extent_um": EXTENT, "mosaic_scale": scale, "derived_scale": scale,
        "scale_warning": scale_warning,
        "shift_um": SHIFT if has_shift else (0.0, 0.0),
        "has_shift": has_shift, "um_per_px": um_per_px, "objective": "10x",
        "well_center_um": (1800.0, 2600.0), "well_radius_um": 700.0,
    }


def _det(det_id, diameter_um=200.0, cx=400.0, cy=300.0, source="auto"):
    r_px = sd.radius_px_for_diameter_um(diameter_um, SCALE)
    d = sd.SpheroidDetection(
        center_px=(cx, cy), radius_px=r_px, diameter_um=diameter_um,
        center_um=sd.back_project_px((cx, cy), EXTENT, SCALE, SHIFT),
        circularity=0.9, fit_fraction=0.9, area_px=100.0, confidence=0.81,
        det_id=det_id, source=source)
    return d


@unittest.skipUnless(_QT, "PySide6 not available")
class _PanelCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.app = QApplication.instance() or QApplication([])
        except Exception as e:      # pragma: no cover
            raise unittest.SkipTest(f"Qt unavailable: {e}")

    def _panel(self, context=None, dets=()):
        from gui.widgets.spheroid_survey_panel import SpheroidSurveyPanel
        p = SpheroidSurveyPanel()
        # Always push the context, including None — "no mosaic for this well" is
        # a real state the panel has to describe, not an absence of one.
        p.set_mosaic_context(context)
        if dets:
            p._dets = list(dets)
            p._enabled_ids = {d.det_id for d in dets}
            p._rebuild_table()
        return p


class TestMotionGates(_PanelCase):
    """Transfer and Go to must be refused unless the mapping is trustworthy."""

    def test_refused_without_a_recorded_shift(self):
        p = self._panel(_context(has_shift=False), [_det("S001")])
        self.assertFalse(p.can_command_motion())
        self.assertFalse(p._goto_btn.isEnabled())
        self.assertFalse(p._transfer_btn.isEnabled())
        self.assertFalse(p._crop_btn.isEnabled())

    def test_allowed_with_a_recorded_shift(self):
        p = self._panel(_context(), [_det("S001")])
        p.select_detection("S001")
        self.assertTrue(p.can_command_motion())
        self.assertTrue(p._goto_btn.isEnabled())
        self.assertTrue(p._transfer_btn.isEnabled())

    def test_legacy_mosaic_explains_itself_with_a_bound(self):
        p = self._panel(_context(has_shift=False))
        # isVisibleTo, not isVisible: the panel itself was never shown.
        self.assertTrue(p._provenance_note.isVisibleTo(p))
        text = p._provenance_note.text()
        self.assertIn("predates registration-shift recording", text)
        self.assertIn("µm", text)
        # Diameters are still fine — a length is translation-invariant.
        self.assertIn("Diameters are still", text)

    def test_no_note_for_a_registered_mosaic(self):
        p = self._panel(_context())
        self.assertFalse(p._provenance_note.isVisibleTo(p))

    def test_goto_emits_absolute_stage_um(self):
        p = self._panel(_context(), [_det("S001")])
        p.select_detection("S001")
        seen = []
        p.goto_requested.connect(lambda x, y: seen.append((x, y)))
        p._on_goto()
        self.assertEqual(len(seen), 1)
        expect = sd.back_project_px((400.0, 300.0), EXTENT, SCALE, SHIFT)
        self.assertAlmostEqual(seen[0][0], expect[0], places=6)
        self.assertAlmostEqual(seen[0][1], expect[1], places=6)

    def test_goto_silent_without_a_shift(self):
        p = self._panel(_context(has_shift=False), [_det("S001")])
        p.select_detection("S001")
        seen = []
        p.goto_requested.connect(lambda x, y: seen.append((x, y)))
        p._on_goto()
        self.assertEqual(seen, [])

    def test_transfer_silent_without_a_shift(self):
        p = self._panel(_context(has_shift=False), [_det("S001")])
        seen = []
        p.transfer_requested.connect(seen.append)
        p._on_transfer()
        self.assertEqual(seen, [])


class TestCuration(_PanelCase):
    def test_transfer_carries_position_and_diameter(self):
        p = self._panel(_context(), [_det("S001", 312.0), _det("S002", 150.0)])
        seen = []
        p.transfer_requested.connect(seen.append)
        p._on_transfer()
        self.assertEqual(len(seen), 1)
        payload = seen[0]
        self.assertEqual(len(payload), 2)
        self.assertAlmostEqual(payload[0]["diameter_um"], 312.0)
        self.assertIn("x_um", payload[0])
        self.assertIn("det_id", payload[0])

    def test_unticking_excludes_from_the_transfer(self):
        p = self._panel(_context(), [_det("S001"), _det("S002")])
        p._enabled_ids = {"S002"}
        p._rebuild_table()
        seen = []
        p.transfer_requested.connect(seen.append)
        p._on_transfer()
        self.assertEqual([e["det_id"] for e in seen[0]], ["S002"])

    def test_untick_all_disables_transfer(self):
        p = self._panel(_context(), [_det("S001")])
        p._set_all_enabled(False)
        self.assertFalse(p._transfer_btn.isEnabled())
        p._set_all_enabled(True)
        self.assertTrue(p._transfer_btn.isEnabled())

    def test_transfer_button_counts_the_ticked(self):
        p = self._panel(_context(), [_det("S001"), _det("S002")])
        self.assertIn("2", p._transfer_btn.text())

    def test_delete_removes_the_row(self):
        p = self._panel(_context(), [_det("S001"), _det("S002")])
        p.select_detection("S001")
        p._on_delete()
        self.assertEqual([d.det_id for d in p.detections()], ["S002"])
        self.assertNotIn("S001", p._enabled_ids)

    def test_checkbox_state_drives_the_enabled_set(self):
        p = self._panel(_context(), [_det("S001")])
        item = p._table.item(0, 0)
        item.setCheckState(Qt.CheckState.Unchecked)
        self.assertEqual(p._enabled_ids, set())
        item.setCheckState(Qt.CheckState.Checked)
        self.assertEqual(p._enabled_ids, {"S001"})


class TestSorting(_PanelCase):
    def test_sorts_numerically_not_lexicographically(self):
        """90 / 1000 / 200: a string sort would give 1000, 200, 90."""
        p = self._panel(_context(), [
            _det("S001", 90.0), _det("S002", 1000.0), _det("S003", 200.0)])
        p._sort_by_size(True)
        got = [p._table.item(r, 2).data(Qt.ItemDataRole.EditRole)
               for r in range(p._table.rowCount())]
        self.assertEqual(got, [90.0, 200.0, 1000.0])
        p._sort_by_size(False)
        got = [p._table.item(r, 2).data(Qt.ItemDataRole.EditRole)
               for r in range(p._table.rowCount())]
        self.assertEqual(got, [1000.0, 200.0, 90.0])

    def test_selection_survives_a_rebuild(self):
        p = self._panel(_context(), [_det("S001"), _det("S002")])
        p.select_detection("S002")
        p._rebuild_table()
        self.assertEqual(p.selected_id(), "S002")


class TestManualAdd(_PanelCase):
    def test_manual_add_is_first_class(self):
        """Detection can legitimately find nothing, so this path must work."""
        p = self._panel(_context())
        det_id = p.add_manual(400.0, 300.0, 50.0)
        self.assertIsNotNone(det_id)
        det = p.detection(det_id)
        self.assertEqual(det.source, "user")
        self.assertTrue(det.user_edited)
        # 50 px radius at 0.5 px/µm → Ø 200 µm.
        self.assertAlmostEqual(det.diameter_um, 200.0)
        # And it lands in the trusted stage frame, shift removed.
        expect = sd.back_project_px((400.0, 300.0), EXTENT, SCALE, SHIFT)
        self.assertAlmostEqual(det.center_um[0], expect[0], places=6)

    def test_manual_add_refused_without_a_mosaic(self):
        p = self._panel(None)
        self.assertIsNone(p.add_manual(1.0, 1.0, 10.0))

    def test_manual_ids_do_not_collide_with_detections(self):
        p = self._panel(_context(), [_det("S001")])
        a = p.add_manual(100.0, 100.0, 20.0)
        b = p.add_manual(200.0, 200.0, 20.0)
        self.assertNotEqual(a, b)
        self.assertNotIn("S001", (a, b))


class TestRedraw(_PanelCase):
    def test_radius_drag_updates_the_diameter_in_um(self):
        p = self._panel(_context(), [_det("S001", 200.0)])
        p.apply_radius_px("S001", 100.0)     # 100 px at 0.5 px/µm → Ø 400 µm
        self.assertAlmostEqual(p.detection("S001").diameter_um, 400.0)
        self.assertTrue(p.detection("S001").user_edited)

    def test_uncommitted_drag_still_updates_the_row(self):
        p = self._panel(_context(), [_det("S001", 200.0)])
        p.apply_radius_px("S001", 75.0, commit=False)
        self.assertAlmostEqual(p.detection("S001").diameter_um, 300.0)
        self.assertAlmostEqual(
            p._table.item(0, 2).data(Qt.ItemDataRole.EditRole), 300.0)

    def test_centre_drag_reprojects_through_the_shift(self):
        p = self._panel(_context(), [_det("S001")])
        p.apply_center_px("S001", 500.0, 350.0)
        expect = sd.back_project_px((500.0, 350.0), EXTENT, SCALE, SHIFT)
        self.assertAlmostEqual(p.detection("S001").center_um[0], expect[0],
                               places=6)
        self.assertAlmostEqual(p.detection("S001").center_um[1], expect[1],
                               places=6)

    def test_edits_ignored_without_a_mosaic_scale(self):
        p = self._panel(_context(scale=0.0), [_det("S001", 200.0)])
        p.apply_radius_px("S001", 100.0)
        self.assertAlmostEqual(p.detection("S001").diameter_um, 200.0)


class TestFitBadges(_PanelCase):
    def test_badge_column_uses_the_host_provider(self):
        p = self._panel()
        p.set_fit_badge_provider(
            lambda d: ("⚠ tight 1.25×", "barely clears") if d >= 200 else ("", ""))
        p.set_mosaic_context(_context())
        p._dets = [_det("S001", 250.0), _det("S002", 90.0)]
        p._enabled_ids = {"S001", "S002"}
        p._rebuild_table()
        badges = [p._table.item(r, 5).text() for r in range(2)]
        self.assertIn("⚠ tight 1.25×", badges)
        self.assertIn("", badges)

    def test_a_broken_provider_never_breaks_the_table(self):
        p = self._panel()
        p.set_fit_badge_provider(lambda d: 1 / 0)
        p.set_mosaic_context(_context())
        p._dets = [_det("S001")]
        p._rebuild_table()
        self.assertEqual(p._table.rowCount(), 1)


class TestContextChanges(_PanelCase):
    def test_changing_well_clears_detections(self):
        """Coordinates came from the previous well's extent — keeping them would
        be a silent mis-mapping."""
        p = self._panel(_context(well="A1"), [_det("S001")])
        self.assertEqual(len(p.detections()), 1)
        p.set_mosaic_context(_context(well="B2"))
        self.assertEqual(len(p.detections()), 0)

    def test_same_well_keeps_detections(self):
        p = self._panel(_context(well="A1"), [_det("S001")])
        p.set_mosaic_context(_context(well="A1", channels=("DAPI", "FITC")))
        self.assertEqual(len(p.detections()), 1)

    def test_scale_disagreement_is_surfaced(self):
        p = self._panel(_context(scale_warning="stored scale disagrees"))
        self.assertIn("disagrees", p._status.text())

    def test_no_mosaic_says_so(self):
        p = self._panel(None)
        self.assertIn("No mosaic", p._status.text())

    def test_channels_populate_the_combo(self):
        p = self._panel(_context(channels=("DAPI", "FITC", "Cy5")))
        items = [p._channel.itemText(i) for i in range(p._channel.count())]
        self.assertEqual(items, ["DAPI", "FITC", "Cy5"])


class TestDetectRefusal(_PanelCase):
    def test_unresolvable_band_refuses_with_a_reason(self):
        p = self._panel(_context(scale=0.01))
        p._min_d.setValue(80.0)
        p._max_d.setValue(400.0)
        p._on_detect()
        self.assertIn("px across", p._status.text())
        self.assertFalse(p.is_detecting())

    def test_detect_without_a_mosaic_is_refused(self):
        p = self._panel(None)
        p._on_detect()
        self.assertIn("No mosaic", p._status.text())

    def test_detect_report_is_applied_and_summarised(self):
        p = self._panel(_context())
        report = sd.DetectionReport(
            detections=[_det("S001", 250.0)], n_blobs=5, n_too_small=3,
            n_shape=1)
        p._on_detect_done(report)
        self.assertEqual([d.det_id for d in p.detections()], ["S001"])
        self.assertIn("3 too small", p._status.text())
        self.assertEqual(p._enabled_ids, {"S001"})

    def test_detection_keeps_manual_entries(self):
        p = self._panel(_context())
        manual = p.add_manual(100.0, 100.0, 25.0)
        p._on_detect_done(sd.DetectionReport(detections=[_det("S001")]))
        ids = [d.det_id for d in p.detections()]
        self.assertIn(manual, ids)
        self.assertIn("S001", ids)


# ── the editable mosaic circles ────────────────────────────────────

class _FakeView:
    """A scene-owning stand-in with the host view's small contract."""

    def __init__(self):
        from PySide6.QtWidgets import QGraphicsScene
        self._scene = QGraphicsScene()
        self.centered = None

    def scene_obj(self):
        return self._scene

    def scene(self):
        return self._scene

    def transform(self):
        from PySide6.QtGui import QTransform
        return QTransform()

    def centerOn(self, pt):
        self.centered = pt


@unittest.skipUnless(_QT, "PySide6 not available")
class TestMosaicOverlay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.app = QApplication.instance() or QApplication([])
        except Exception as e:      # pragma: no cover
            raise unittest.SkipTest(f"Qt unavailable: {e}")

    def _overlay(self, entries=()):
        from gui.widgets.spheroid_mosaic_items import SpheroidOverlay
        view = _FakeView()
        ov = SpheroidOverlay(view)
        if entries:
            ov.set_circles(entries)
        return ov, view

    def _entry(self, det_id="S001", cx=400.0, cy=300.0, r=50.0):
        return {"det_id": det_id, "cx": cx, "cy": cy, "r": r}

    def test_circle_centre_uses_the_rect_plus_pos_convention(self):
        """Same invariant the well-mapping dialog drives the stage with."""
        ov, _v = self._overlay([self._entry()])
        item = ov.item("S001")
        self.assertAlmostEqual(item.center().x(), 400.0)
        self.assertAlmostEqual(item.center().y(), 300.0)
        self.assertAlmostEqual(item.pos().x(), 0.0)
        self.assertAlmostEqual(item.radius(), 50.0)

    def test_dragging_the_item_moves_its_centre(self):
        ov, _v = self._overlay([self._entry()])
        item = ov.item("S001")
        item.setPos(25.0, -10.0)
        self.assertAlmostEqual(item.center().x(), 425.0)
        self.assertAlmostEqual(item.center().y(), 290.0)

    def test_release_folds_the_drag_back_into_the_rect(self):
        """So the "centre == rect().center() + pos()" invariant restarts clean."""
        ov, _v = self._overlay([self._entry()])
        item = ov.item("S001")
        item.setPos(25.0, -10.0)
        seen = []
        ov.center_committed.connect(lambda t, x, y: seen.append((t, x, y)))
        ov.on_scene_release(QPointF(0.0, 0.0))
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0][1], 425.0)
        self.assertAlmostEqual(item.pos().x(), 0.0)
        self.assertAlmostEqual(item.center().x(), 425.0)

    def test_label_and_handle_are_children_so_a_drag_carries_them(self):
        ov, _v = self._overlay([self._entry()])
        item = ov.item("S001")
        self.assertIs(item.handle.parentItem(), item)
        self.assertIs(item._label.parentItem(), item)

    def test_radius_handle_drag_resizes_about_the_centre(self):
        ov, _v = self._overlay([self._entry()])
        seen = []
        ov.radius_committed.connect(lambda t, r: seen.append((t, r)))
        # Press on the handle (at centre + r on +X), drag outward.
        ov.on_scene_press(QPointF(450.0, 300.0))
        self.assertTrue(ov.is_dragging())
        ov.on_scene_drag(QPointF(500.0, 300.0))
        ov.on_scene_release(QPointF(500.0, 300.0))
        self.assertFalse(ov.is_dragging())
        self.assertAlmostEqual(ov.item("S001").radius(), 100.0, delta=1.0)
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0][1], 100.0, delta=1.0)
        # The centre must NOT have moved.
        self.assertAlmostEqual(ov.item("S001").center().x(), 400.0, delta=0.5)

    def test_press_inside_a_circle_selects_it(self):
        ov, _v = self._overlay([self._entry()])
        seen = []
        ov.circle_clicked.connect(seen.append)
        ov.on_scene_press(QPointF(410.0, 305.0))
        self.assertEqual(seen, ["S001"])

    def test_press_on_bare_mosaic_reports_an_empty_click(self):
        ov, _v = self._overlay([self._entry()])
        seen = []
        ov.empty_clicked.connect(lambda x, y: seen.append((x, y)))
        ov.on_scene_press(QPointF(50.0, 50.0))
        self.assertEqual(seen, [(50.0, 50.0)])

    def test_rim_mode_collects_points_and_fits(self):
        import math
        ov, _v = self._overlay([])
        ov.set_rim_mode(True)
        for ang in (0.0, 120.0, 240.0):
            t = math.radians(ang)
            ov.on_scene_press(QPointF(400.0 + 60.0 * math.cos(t),
                                      300.0 + 60.0 * math.sin(t)))
        fit = ov.rim_fit()
        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit[0], 400.0, delta=1.0)
        self.assertAlmostEqual(fit[2], 60.0, delta=1.0)

    def test_rim_fit_needs_three_points(self):
        ov, _v = self._overlay([])
        ov.set_rim_mode(True)
        ov.on_scene_press(QPointF(400.0, 240.0))
        ov.on_scene_press(QPointF(460.0, 300.0))
        self.assertIsNone(ov.rim_fit())
        self.assertFalse(ov.commit_rim_fit())

    def test_commit_rim_fit_emits_and_clears(self):
        import math
        ov, _v = self._overlay([])
        ov.set_rim_mode(True)
        for ang in (0.0, 120.0, 240.0):
            t = math.radians(ang)
            ov.on_scene_press(QPointF(400.0 + 60.0 * math.cos(t),
                                      300.0 + 60.0 * math.sin(t)))
        seen = []
        ov.rim_fitted.connect(lambda x, y, r: seen.append((x, y, r)))
        self.assertTrue(ov.commit_rim_fit())
        self.assertEqual(len(seen), 1)
        self.assertIsNone(ov.rim_fit())

    def test_leaving_rim_mode_clears_points(self):
        ov, _v = self._overlay([])
        ov.set_rim_mode(True)
        ov.on_scene_press(QPointF(400.0, 240.0))
        ov.set_rim_mode(False)
        self.assertIsNone(ov.rim_fit())

    def test_rim_mode_press_never_starts_a_resize(self):
        ov, _v = self._overlay([self._entry()])
        ov.set_rim_mode(True)
        ov.on_scene_press(QPointF(450.0, 300.0))   # right on the handle
        self.assertFalse(ov.is_dragging())

    def test_clear_removes_every_item(self):
        ov, view = self._overlay([self._entry("S001"), self._entry("S002",
                                                                  cx=200.0)])
        self.assertEqual(len(view.scene().items()) > 0, True)
        ov.clear()
        self.assertIsNone(ov.item("S001"))

    def test_highlight_marks_only_one(self):
        ov, _v = self._overlay([self._entry("S001"),
                                self._entry("S002", cx=200.0)])
        ov.set_highlight("S002")
        self.assertFalse(getattr(ov.item("S001"), "_highlight", False))
        self.assertTrue(getattr(ov.item("S002"), "_highlight", False))

    def test_center_on_asks_the_view(self):
        ov, view = self._overlay([self._entry()])
        ov.center_on("S001")
        self.assertIsNotNone(view.centered)

    def test_minimum_radius_is_enforced(self):
        ov, _v = self._overlay([self._entry()])
        ov.item("S001").set_radius(0.0)
        self.assertGreater(ov.item("S001").radius(), 0.0)


if __name__ == "__main__":
    unittest.main()
