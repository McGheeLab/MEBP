"""
test_v715_toolbar_and_popout.py — the hover toolbar and the detachable view.

Operator, on the v7.14 buttons: *"now that there are more buttons on any live
view we may want to reconsider how all these buttons are displayed"*, plus
*"I need the ability to pop out any live view to make it larger if needed, all
mouse clicks will still land where they should to select locations etc."*

Two things are load-bearing here and are tested as behaviour, not appearance:

* the RECORDING pill must stay reachable when the pointer is away — every
  other control may hide, but not the one that stops a recording;
* a detached view must come back to the SAME slot and still emit ``clicked``.
  ``CameraFeedView.closeEvent`` disconnects the camera and ``eventFilter``
  refuses to emit while ``_last_pixmap`` is None, so "it returned" is not the
  same as "it works".
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPoint, QPointF, Qt
from PySide6.QtGui import QImage, QMouseEvent
from PySide6.QtWidgets import QApplication, QVBoxLayout, QWidget

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.camera_feed_view import CameraFeedView

RAW_W, RAW_H = 640, 480


def _frame():
    img = QImage(RAW_W, RAW_H, QImage.Format.Format_RGB888)
    img.fill(0x336699)
    return img


class _Base(unittest.TestCase):

    def _view(self, width=800, label_w=None):
        v = CameraFeedView(label="Cam 1 — live")
        v.resize(width, 400)
        v._display.resize(label_w or (width - 20), 380)
        v._last_image_size = (RAW_W, RAW_H)
        # Buttons only become available once frames are arriving.
        v._capture_available = True
        v._settings_available = True
        v._render_frame(_frame())
        return v

    @staticmethod
    def _hover(v, on=True):
        v._hovering = on
        v._position_settings_btn()

    @staticmethod
    def _shown(btn) -> bool:
        """⚠ isHidden(), not isVisible().

        These views are not added to a shown window in the tests, and
        ``isVisible()`` is False while ANY ancestor is unshown — so it would
        report every button hidden regardless of the toolbar logic, and the
        tests would pass no matter what. The production layout code has the
        same rule, for the same reason (CLAUDE.md records the trap).
        """
        return btn is not None and not btn.isHidden()


class TestHoverReveal(_Base):

    def test_buttons_are_hidden_until_hovered(self):
        v = self._view()
        self._hover(v, False)
        for b in v._overlay_buttons():
            if b is not None:
                self.assertFalse(self._shown(b),
                                 f"{b.text()} showing with no pointer over it")

    def test_hovering_reveals_them(self):
        v = self._view()
        self._hover(v, True)
        shown = [b for b in v._overlay_buttons()
                 if b is not None and self._shown(b)]
        self.assertGreaterEqual(len(shown), 4,
                                "hovering revealed almost nothing")

    def test_enter_and_leave_events_drive_it(self):
        from PySide6.QtGui import QEnterEvent
        v = self._view()
        p = QPointF(10.0, 10.0)
        v.enterEvent(QEnterEvent(p, p, p))
        self.assertTrue(v._hovering)
        any_shown = any(b is not None and self._shown(b)
                        for b in v._overlay_buttons())
        self.assertTrue(any_shown)
        v.leaveEvent(QEvent(QEvent.Type.Leave))
        self.assertFalse(v._hovering)

    def test_recording_pill_stays_visible_without_hover(self):
        """⭐ The exemption. A recording you cannot see is a recording you
        cannot stop."""
        v = self._view()
        v._on_record_state(True, 12.0, 180)
        self._hover(v, False)
        self.assertTrue(self._shown(v._record_btn),
                        "the stop pill vanished when the pointer left")
        self.assertIn("⏹", v._record_btn.text())

    def test_pill_hides_again_once_recording_stops(self):
        v = self._view()
        v._on_record_state(True, 3.0, 40)
        v._on_record_state(False, 0.0, 0)
        self._hover(v, False)
        self.assertFalse(self._shown(v._record_btn))

    def test_unavailable_buttons_never_appear(self):
        v = self._view()
        v._capture_available = False
        v._settings_available = False
        self._hover(v, True)
        self.assertFalse(self._shown(v._capture_btn))
        self.assertFalse(self._shown(v._settings_btn))


class TestOverflow(_Base):

    def test_a_narrow_feed_moves_buttons_into_the_menu(self):
        """Seven 26 px buttons do not fit a 200 px feed — the minimum size a
        CameraFeedView is allowed to be."""
        v = self._view(width=220, label_w=200)
        self._hover(v, True)
        self.assertTrue(v._overflow_actions, "nothing overflowed on a 200 px feed")
        self.assertTrue(self._shown(v._overflow_btn))

    def test_a_wide_feed_needs_no_menu(self):
        v = self._view(width=900, label_w=880)
        self._hover(v, True)
        self.assertFalse(v._overflow_actions)
        self.assertFalse(self._shown(v._overflow_btn))

    def test_buttons_stay_inside_the_label(self):
        for w in (200, 320, 640, 880):
            v = self._view(width=w + 20, label_w=w)
            self._hover(v, True)
            for b in v._overlay_buttons() + [v._overflow_btn]:
                if b is None or not self._shown(b):
                    continue
                self.assertGreaterEqual(b.x(), 0, f"{b.text()} off the left at {w}")
                self.assertLessEqual(b.x() + b.width(), w,
                                     f"{b.text()} off the right at {w}")

    def test_visible_buttons_do_not_overlap(self):
        v = self._view(width=340, label_w=320)
        self._hover(v, True)
        spans = sorted((b.x(), b.x() + b.width())
                       for b in v._overlay_buttons() + [v._overflow_btn]
                       if b is not None and self._shown(b))
        for (a0, a1), (b0, _b1) in zip(spans, spans[1:]):
            self.assertLessEqual(a1, b0, "overlay buttons overlap")

    def test_the_gear_keeps_its_position(self):
        """Pinned since v7.14: the gear is the anchor operators aim for."""
        v = self._view(width=900, label_w=880)
        self._hover(v, True)
        gear_x, gear_y = v._settings_btn.x(), v._settings_btn.y()
        v._capture_available = False          # capture buttons drop out
        v._position_settings_btn()
        self.assertEqual((v._settings_btn.x(), v._settings_btn.y()),
                         (gear_x, gear_y))

    def test_recording_pill_is_never_the_one_pushed_into_the_menu(self):
        v = self._view(width=220, label_w=200)
        v._on_record_state(True, 5.0, 60)
        self._hover(v, True)
        self.assertNotIn(v._record_btn, v._overflow_actions)
        self.assertTrue(self._shown(v._record_btn))


class TestPopout(_Base):

    def _hosted(self):
        host = QWidget()
        lay = QVBoxLayout(host)
        lay.addWidget(QWidget())              # something before it
        v = self._view()
        lay.addWidget(v, 3)
        lay.addWidget(QWidget())              # …and after
        return host, lay, v

    def test_detaches_and_leaves_a_placeholder(self):
        host, lay, v = self._hosted()
        dlg = v.pop_out()
        self.assertIsNotNone(dlg)
        self.assertIs(v.window(), dlg)
        self.assertEqual(lay.itemAt(1).widget().__class__.__name__,
                         "_Placeholder")
        dlg.close()

    def test_returns_to_the_same_slot_with_the_same_stretch(self):
        host, lay, v = self._hosted()
        dlg = v.pop_out()
        dlg.close()
        idx = lay.indexOf(v)
        self.assertEqual(idx, 1, "came back in the wrong position")
        self.assertEqual(lay.stretch(idx), 3, "lost its stretch factor")
        self.assertIs(v.parentWidget(), host)

    def test_still_emits_clicks_after_returning(self):
        """⭐ 'It came back' is not 'it works'. closeEvent disconnects the
        camera and eventFilter refuses to emit while _last_pixmap is None."""
        host, lay, v = self._hosted()
        dlg = v.pop_out()
        dlg.close()
        got = []
        v.clicked.connect(lambda x, y: got.append((x, y)))
        geo = v.geometry_map()
        self.assertTrue(geo.valid, "no geometry after returning")
        ox, oy = geo.offset
        pt = QPointF(ox + geo.pixmap_size[0] * 0.5,
                     oy + geo.pixmap_size[1] * 0.5)
        ev = QMouseEvent(QEvent.Type.MouseButtonPress, pt,
                         Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
        v.eventFilter(v._display, ev)
        self.assertTrue(got, "the returned view no longer emits clicks")

    def test_a_second_popout_raises_the_existing_window(self):
        host, lay, v = self._hosted()
        a = v.pop_out()
        b = v.pop_out()
        self.assertIs(a, b, "opened a second window for one view")
        a.close()

    def test_a_view_with_no_layout_refuses(self):
        v = self._view()          # never added to a layout
        self.assertIsNone(v.pop_out())

    def test_the_page_is_never_hidden(self):
        """Several pages stop the camera in hideEvent, and the needle-bore
        wizard disarms the print floor there — popping out must not be a
        safety change."""
        host, lay, v = self._hosted()
        host.show()
        dlg = v.pop_out()
        self.assertTrue(host.isVisible(), "the host page was hidden")
        dlg.close()

    def test_never_uses_set_parent_none(self):
        """calibration.py records that setParent(None) kills the display of a
        running camera.

        ⚠ Checked by AST, not substring: the phrase appears in this module's
        own comments explaining why it is avoided, and a substring guard would
        fail on the explanation while passing on the real thing. The repo has
        the same lesson recorded from a v7.10 guard that matched an import
        line.
        """
        import ast
        import inspect
        import gui.dialogs.camera_popout_dialog as mod
        tree = ast.parse(inspect.getsource(mod))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "setParent"
                    and len(node.args) == 1
                    and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value is None):
                self.fail("setParent(None) is called — it kills the display "
                          "of a running camera")

    def test_restore_happens_before_the_dialog_finishes(self):
        """AST: the view must be put back BEFORE super().done(), or Qt tears
        down the children — and CameraFeedView.closeEvent disconnects the
        camera on the way out."""
        import ast
        import inspect
        import gui.dialogs.camera_popout_dialog as mod
        tree = ast.parse(inspect.getsource(mod))
        fn = [n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "done"][0]
        order = []
        for stmt in fn.body:          # source order, not ast.walk (BFS)
            txt = ast.dump(stmt)
            if "_restore" in txt:
                order.append("restore")
            if "'done'" in txt or '"done"' in txt or "super" in txt:
                order.append("super")
        self.assertEqual(order[:2], ["restore", "super"],
                         "the view is restored after the dialog finishes")


if __name__ == "__main__":
    unittest.main()
