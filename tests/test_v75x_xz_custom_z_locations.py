"""
v7.5.x — Custom Z locations on the XZ side view (Jog page feature).

Drag on the Z-axis plot → floating mm readout tag → release sets a
custom Z location; a plain click ON the red current-Z line tags the
needle's current height. Custom locations render as ★ badges (click →
``go_to_z_requested``, same contract as the calibration Z-ref badges)
with an ✕ remove button; the Jog page persists them in settings.json
(``jog.custom_z_mm``, raw zero-ref mm).

Covers:
  * drag-to-tag adds a location at the dragged Z (round-trip through the
    widget's own pixel mapping), incl. while zoomed;
  * the floating readout state is live during the drag;
  * display-sign −1 (ME3B V1): tags are stored in the RAW zero-ref frame;
  * a plain click on the current-Z line tags the current Z; a click
    elsewhere in the plot adds nothing;
  * ★ badge click emits go_to_z_requested (raw frame); ✕ + right-click
    remove; custom_z_changed fires on add/remove;
  * dedup within tolerance; programmatic restore does NOT emit;
  * the feature is OFF by default (other host pages unchanged);
  * Jog page: enabled, restores from settings, persists on change.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from gui.widgets.xz_side_view import XZSideView  # noqa: E402


class _Limits:
    """Minimal safety-limits stand-in for the XZ view."""
    xy_min_x = 0.0
    xy_max_x = 100000.0
    xy_min_y = 0.0
    xy_max_y = 80000.0
    z_min = 0.0
    z_max = 60.0


def _event(etype, pos, button=Qt.MouseButton.LeftButton,
           buttons=None):
    if buttons is None:
        if etype == QEvent.Type.MouseButtonRelease:
            buttons = Qt.MouseButton.NoButton
        elif etype == QEvent.Type.MouseMove:
            buttons = Qt.MouseButton.LeftButton
        else:
            buttons = button
    return QMouseEvent(
        etype, QPointF(*pos), QPointF(*pos), button, buttons,
        Qt.KeyboardModifier.NoModifier)


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_view(self, enabled=True, sign=1.0):
        view = XZSideView()
        view.resize(420, 360)
        view.set_safety_limits(_Limits())
        view.set_z_display_sign(sign)
        if enabled:
            view.set_custom_z_enabled(True)
        view.set_position(1000.0, 25.0)
        return view

    # Gesture helpers — drive the handlers directly (deterministic
    # offscreen; QTest's synthetic move events depend on the platform).
    def _press(self, view, pos, button=Qt.MouseButton.LeftButton):
        view.mousePressEvent(_event(QEvent.Type.MouseButtonPress, pos,
                                    button))

    def _move(self, view, pos):
        view.mouseMoveEvent(_event(QEvent.Type.MouseMove, pos,
                                   Qt.MouseButton.NoButton))

    def _release(self, view, pos, button=Qt.MouseButton.LeftButton):
        view.mouseReleaseEvent(_event(QEvent.Type.MouseButtonRelease,
                                      pos, button))

    def _drag(self, view, z_from_raw, z_to_raw):
        """Full press→move→release drag between two raw zero-ref Zs,
        using the widget's own pixel mapping."""
        x = view._content_rect().center().x()
        y0 = view._z_to_px(view._disp(z_from_raw))
        y1 = view._z_to_px(view._disp(z_to_raw))
        self._press(view, (x, y0))
        # First move past the threshold, then to the target
        self._move(view, (x, (y0 + y1) / 2.0))
        self._move(view, (x, y1))
        self._release(view, (x, y1))


class TestDragToTag(_Base):
    def test_drag_adds_custom_location_at_dragged_z(self):
        view = self._make_view()
        self._drag(view, z_from_raw=40.0, z_to_raw=10.0)
        locs = view.custom_z_locations()
        self.assertEqual(len(locs), 1)
        self.assertAlmostEqual(locs[0], 10.0, delta=0.2)

    def test_readout_state_live_during_drag(self):
        view = self._make_view()
        x = view._content_rect().center().x()
        y0 = view._z_to_px(view._disp(40.0))
        y1 = view._z_to_px(view._disp(15.0))
        self._press(view, (x, y0))
        self._move(view, (x, y1))
        self.assertTrue(view._drag_active)
        self.assertIsNotNone(view._drag_z_disp)
        self.assertAlmostEqual(view._drag_z_disp, view._disp(15.0),
                               delta=0.2)
        self._release(view, (x, y1))
        self.assertFalse(view._drag_active)
        self.assertIsNone(view._drag_z_disp)

    def test_drag_stores_raw_frame_under_negative_display_sign(self):
        # ME3B V1: z_up_sign = -1 → display = -raw. A drag that reads
        # "-20 mm" on screen must store raw zero-ref +20 (the move frame).
        view = self._make_view(sign=-1.0)
        self._drag(view, z_from_raw=40.0, z_to_raw=20.0)
        locs = view.custom_z_locations()
        self.assertEqual(len(locs), 1)
        self.assertAlmostEqual(locs[0], 20.0, delta=0.2)

    def test_drag_while_zoomed_maps_through_zoomed_bounds(self):
        view = self._make_view()
        view._z_zoom = 4.0
        view._z_zoom_center_mm = 20.0
        view._sync_scrollbar()
        self._drag(view, z_from_raw=24.0, z_to_raw=18.0)
        locs = view.custom_z_locations()
        self.assertEqual(len(locs), 1)
        self.assertAlmostEqual(locs[0], 18.0, delta=0.1)

    def test_drag_emits_custom_z_changed(self):
        view = self._make_view()
        got = []
        view.custom_z_changed.connect(lambda vals: got.append(list(vals)))
        self._drag(view, z_from_raw=40.0, z_to_raw=10.0)
        self.assertEqual(len(got), 1)
        self.assertEqual(len(got[0]), 1)


class TestClickCurrentLine(_Base):
    def test_click_on_current_line_tags_current_z(self):
        view = self._make_view()  # current Z = 25.0 raw
        x = view._content_rect().center().x()
        y = view._z_to_px(view._disp(25.0))
        self._press(view, (x, y))
        self._release(view, (x, y))
        self.assertEqual(view.custom_z_locations(), [25.0])

    def test_click_elsewhere_adds_nothing(self):
        view = self._make_view()
        x = view._content_rect().center().x()
        # Far from the current-Z line (25.0) — e.g. 45.0
        y = view._z_to_px(view._disp(45.0))
        self._press(view, (x, y))
        self._release(view, (x, y))
        self.assertEqual(view.custom_z_locations(), [])

    def test_click_line_with_no_position_is_noop(self):
        view = self._make_view()
        view.set_position(None, None)
        x = view._content_rect().center().x()
        y = view._content_rect().center().y()
        self._press(view, (x, y))
        self._release(view, (x, y))
        self.assertEqual(view.custom_z_locations(), [])


class TestBadges(_Base):
    def _painted(self, view):
        view.grab()  # populate hit rects via a real paint pass
        return view

    def test_badge_click_emits_go_to_z_raw_frame(self):
        view = self._make_view(sign=-1.0)
        view.add_custom_z(12.0)
        self._painted(view)
        self.assertEqual(len(view._custom_hit_rects), 1)
        badge_rect, _close, z_raw = view._custom_hit_rects[0]
        self.assertEqual(z_raw, 12.0)
        got = []
        view.go_to_z_requested.connect(got.append)
        c = badge_rect.center()
        self._press(view, (c.x(), c.y()))
        self.assertEqual(got, [12.0])
        # The badge click must NOT have added/removed anything
        self.assertEqual(view.custom_z_locations(), [12.0])

    def test_close_button_removes_and_emits(self):
        view = self._make_view()
        view.add_custom_z(12.0)
        self._painted(view)
        _badge, close_rect, _z = view._custom_hit_rects[0]
        got = []
        view.custom_z_changed.connect(lambda vals: got.append(list(vals)))
        c = close_rect.center()
        self._press(view, (c.x(), c.y()))
        self.assertEqual(view.custom_z_locations(), [])
        self.assertEqual(got, [[]])

    def test_right_click_badge_removes(self):
        view = self._make_view()
        view.add_custom_z(12.0)
        self._painted(view)
        badge_rect, _close, _z = view._custom_hit_rects[0]
        c = badge_rect.center()
        self._press(view, (c.x(), c.y()),
                    button=Qt.MouseButton.RightButton)
        self.assertEqual(view.custom_z_locations(), [])

    def test_z_ref_badges_still_route(self):
        view = self._make_view()
        view.set_z_references({"fast_move_z": 44.0})
        self._painted(view)
        rect, value = view._z_ref_hit_rects["fast_move_z"]
        got = []
        view.go_to_z_requested.connect(got.append)
        c = rect.center()
        self._press(view, (c.x(), c.y()))
        self.assertEqual(got, [44.0])


class TestApiAndDefaults(_Base):
    def test_disabled_by_default_drag_is_noop(self):
        view = self._make_view(enabled=False)
        self.assertFalse(view._custom_z_enabled)
        self._drag(view, z_from_raw=40.0, z_to_raw=10.0)
        self.assertEqual(view.custom_z_locations(), [])

    def test_disabled_by_default_click_line_is_noop(self):
        view = self._make_view(enabled=False)
        x = view._content_rect().center().x()
        y = view._z_to_px(view._disp(25.0))
        self._press(view, (x, y))
        self._release(view, (x, y))
        self.assertEqual(view.custom_z_locations(), [])

    def test_dedup_within_tolerance(self):
        view = self._make_view()
        self.assertTrue(view.add_custom_z(10.0))
        self.assertFalse(view.add_custom_z(10.004))
        self.assertEqual(view.custom_z_locations(), [10.0])

    def test_programmatic_restore_does_not_emit(self):
        view = self._make_view()
        emits = []
        view.custom_z_changed.connect(lambda vals: emits.append(vals))
        view.set_custom_z_locations([3.0, 9.5, "garbage", 9.501])
        self.assertEqual(view.custom_z_locations(), [3.0, 9.5])
        self.assertEqual(emits, [])

    def test_remove_matches_within_tolerance(self):
        view = self._make_view()
        view.add_custom_z(10.0)
        self.assertTrue(view.remove_custom_z(10.004))
        self.assertEqual(view.custom_z_locations(), [])
        self.assertFalse(view.remove_custom_z(10.0))

    def test_locations_sorted(self):
        view = self._make_view()
        view.add_custom_z(30.0)
        view.add_custom_z(5.0)
        view.add_custom_z(12.0)
        self.assertEqual(view.custom_z_locations(), [5.0, 12.0, 30.0])


class _FakeSettings:
    def __init__(self, data=None):
        self.data = dict(data or {})
        self.saved = 0

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value

    def save(self):
        self.saved += 1


class TestJogPageIntegration(_Base):
    def _make_page(self):
        from gui.pages.jog_control import JogControlPage
        ctrl = MagicMock()
        ctrl.safety_limits = _Limits()
        ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        ctrl.z_up_sign.return_value = -1.0
        ctrl.plate_flip_180.return_value = True
        ctrl.backlash_comp_enabled.return_value = False
        ctrl.is_zp_connected = True
        self._moves = []
        ctrl.move_z_absolute.side_effect = (
            lambda z, from_zero_ref=False:
            self._moves.append((z, from_zero_ref)))
        return JogControlPage(ctrl, camera_manager=None)

    def test_feature_enabled_on_jog_view(self):
        page = self._make_page()
        self.assertTrue(page._xz_view._custom_z_enabled)

    def test_settings_restore(self):
        page = self._make_page()
        page.set_settings(_FakeSettings({"jog.custom_z_mm": [3.0, 9.5]}))
        self.assertEqual(page._xz_view.custom_z_locations(), [3.0, 9.5])

    def test_restore_does_not_immediately_resave(self):
        settings = _FakeSettings({"jog.custom_z_mm": [3.0]})
        page = self._make_page()
        page.set_settings(settings)
        self.assertEqual(settings.saved, 0)

    def test_add_persists_list(self):
        settings = _FakeSettings({"jog.custom_z_mm": [3.0]})
        page = self._make_page()
        page.set_settings(settings)
        page._xz_view.add_custom_z(12.0)
        self.assertEqual(settings.data["jog.custom_z_mm"], [3.0, 12.0])
        self.assertGreaterEqual(settings.saved, 1)

    def test_remove_persists_list(self):
        settings = _FakeSettings({"jog.custom_z_mm": [3.0, 12.0]})
        page = self._make_page()
        page.set_settings(settings)
        page._xz_view.remove_custom_z(3.0)
        self.assertEqual(settings.data["jog.custom_z_mm"], [12.0])

    def test_custom_badge_routes_to_move_z(self):
        page = self._make_page()
        page._xz_view.custom_z_changed.disconnect(
            page._on_custom_z_changed)  # no settings — avoid noise
        page._xz_view.add_custom_z(8.0)
        page._xz_view.go_to_z_requested.emit(8.0)
        self.assertEqual(self._moves, [(8.0, True)])

    def test_no_settings_add_does_not_raise(self):
        page = self._make_page()
        page._xz_view.add_custom_z(5.0)  # _settings is None → no-op persist
        self.assertEqual(page._xz_view.custom_z_locations(), [5.0])


if __name__ == "__main__":
    unittest.main()
