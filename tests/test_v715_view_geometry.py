"""
test_v715_view_geometry.py — one widget↔image mapping, provably invertible.

WHY THIS EXISTS
---------------
Zoom on a live view is only safe if painting and hit-testing agree. Before
v7.15 the widget→image mapping existed in FOUR independent copies:

    camera_feed_view._widget_to_image         (the only one ``clicked`` used)
    live_target_picker._image_to_widget
    live_target_picker._clamped_image_point
    measurement_camera_view._handle_drag

and none knew about zoom or pan. Adding zoom to the click path alone would
have moved the picture out from under the target rings — the v7.8 ``to_px``
identity-inverse bug returning at a different layer.

So the property under test is not "zoom works" but "``to_image`` and
``to_widget`` are exact inverses **for every combination** of orientation,
letterbox, zoom and pan" — and that every caller resolves to this one object.
"""

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QRectF
from PySide6.QtGui import QImage, QTransform
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets.camera_feed_view import CameraFeedView, view_transform_coeffs
from gui.widgets.view_geometry import (
    NULL_GEOMETRY, ZOOM_MAX, ZOOM_MIN, ViewGeometry, clamp_zoom, visible_rect)

RAW_W, RAW_H = 640, 480


def _geo(rot=0.0, mirror=False, flip_y=False, zoom=1.0, center=None,
         label=(700, 520)):
    """A geometry for the given orientation/zoom, built the way the view does."""
    m11, m12, m21, m22 = view_transform_coeffs(mirror, flip_y, rot)
    base = QTransform(m11, m12, m21, m22, 0.0, 0.0)
    true_xf = QImage.trueMatrix(base, RAW_W, RAW_H)
    swap = int(rot) % 180 == 90
    disp = (RAW_H, RAW_W) if swap else (RAW_W, RAW_H)
    src = visible_rect(disp[0], disp[1], zoom, center)
    # The label fit: KeepAspectRatio of the CROP into the label.
    sc = min(label[0] / src.width(), label[1] / src.height())
    pm = (int(src.width() * sc), int(src.height() * sc))
    return ViewGeometry.build(
        raw_size=(RAW_W, RAW_H), disp_size=disp, pixmap_size=pm,
        label_size=label, true_xform=true_xf, src_rect=src)


class TestRoundTrip(unittest.TestCase):
    """⭐ The load-bearing property."""

    POINTS = ((8.0, 8.0), (320.0, 240.0), (600.0, 440.0), (100.0, 300.0))

    def test_inverse_holds_over_orientation_zoom_and_pan(self):
        """⭐ widget → raw → widget, for points that are ON SCREEN.

        Driven from WIDGET space because that is where "visible" is trivially
        known: a point inside the pixmap is by definition clickable, whatever
        the zoom and pan are doing. Starting from raw pixels instead made most
        samples fall outside the crop and quietly thinned the sweep — which
        the self-guard below caught.
        """
        cases = 0
        for rot in (0.0, 90.0, 180.0, 270.0):
            for mirror in (False, True):
                for flip_y in (False, True):
                    for zoom in (1.0, 2.0, 5.5):
                        center = None if zoom == 1.0 else (200.0, 150.0)
                        geo = _geo(rot, mirror, flip_y, zoom, center)
                        ox, oy = geo.offset
                        pw, ph = geo.pixmap_size
                        for fx, fy in ((0.1, 0.1), (0.5, 0.5),
                                       (0.9, 0.3), (0.25, 0.8)):
                            w = (ox + pw * fx, oy + ph * fy)
                            raw = geo.to_image(*w)
                            self.assertIsNotNone(
                                raw, f"rot={rot} zoom={zoom} rejected an "
                                     f"in-pixmap point")
                            back = geo.to_widget(*raw)
                            self.assertIsNotNone(back)
                            self.assertAlmostEqual(
                                back[0], w[0], places=3,
                                msg=f"rot={rot} mir={mirror} fy={flip_y} "
                                    f"zoom={zoom}")
                            self.assertAlmostEqual(back[1], w[1], places=3)
                            cases += 1
        self.assertGreaterEqual(
            cases, 192,
            "the sweep collapsed — a geometry that rejected everything would "
            "otherwise 'pass' by never asserting")

    def test_raw_points_round_trip_when_visible(self):
        """The other direction, at fit where every raw pixel is on screen."""
        for rot in (0.0, 90.0, 180.0, 270.0):
            geo = _geo(rot)
            for raw in self.POINTS:
                w = geo.to_widget(*raw)
                back = geo.to_image(*w)
                self.assertIsNotNone(back, f"rot={rot} raw={raw}")
                self.assertAlmostEqual(back[0], raw[0], places=3)
                self.assertAlmostEqual(back[1], raw[1], places=3)

    def test_a_point_outside_the_zoom_crop_is_reported_offscreen(self):
        """The companion to the sweep: not every raw pixel is visible when
        zoomed, and saying so is the correct answer — not clamping silently."""
        geo = _geo(zoom=4.0, center=(500.0, 400.0))
        w = geo.to_widget(5.0, 5.0)          # far from the visible region
        self.assertIsNotNone(w)
        self.assertIsNone(geo.to_image(*w),
                          "an off-screen point was accepted as a click")

    def test_letterbox_offset_is_honoured_both_ways(self):
        """A label wider than the picture centres it; both directions must
        account for the bars or clicks land offset by half the gap."""
        geo = _geo(label=(900, 520))
        self.assertGreater(geo.offset[0], 0.0)
        w = geo.to_widget(0.0, 0.0)
        self.assertAlmostEqual(w[0], geo.offset[0], places=3)

    def test_a_click_in_the_letterbox_is_rejected(self):
        geo = _geo(label=(900, 520))
        self.assertIsNone(geo.to_image(2.0, 260.0))

    def test_clamp_pulls_an_outside_point_to_the_edge(self):
        """A drag must keep following the cursor off the picture, not freeze."""
        geo = _geo(label=(900, 520))
        p = geo.to_image(2.0, 260.0, clamp=True)
        self.assertIsNotNone(p)
        self.assertTrue(0.0 <= p[0] <= RAW_W)


class TestZoomModel(unittest.TestCase):

    def test_zoom_one_sees_the_whole_frame(self):
        r = visible_rect(RAW_W, RAW_H, 1.0)
        self.assertEqual((r.x(), r.y(), r.width(), r.height()),
                         (0.0, 0.0, float(RAW_W), float(RAW_H)))

    def test_zooming_in_shows_less(self):
        a = visible_rect(RAW_W, RAW_H, 1.0)
        b = visible_rect(RAW_W, RAW_H, 4.0)
        self.assertAlmostEqual(b.width() * 4.0, a.width())

    def test_pan_is_clamped_inside_the_image(self):
        """Panning must not scroll off into blank space."""
        r = visible_rect(RAW_W, RAW_H, 2.0, center=(-9999.0, 9999.0))
        self.assertGreaterEqual(r.x(), 0.0)
        self.assertGreaterEqual(r.y(), 0.0)
        self.assertLessEqual(r.right(), RAW_W + 1e-6)
        self.assertLessEqual(r.bottom(), RAW_H + 1e-6)

    def test_zoom_is_bounded(self):
        self.assertEqual(clamp_zoom(0.01), ZOOM_MIN)
        self.assertEqual(clamp_zoom(1e6), ZOOM_MAX)
        self.assertEqual(clamp_zoom("junk"), ZOOM_MIN)

    def test_scale_grows_with_zoom(self):
        """The whole point: a zoomed view shows fewer image px per widget px."""
        self.assertGreater(_geo(zoom=4.0).scale, _geo(zoom=1.0).scale)

    def test_widget_scale_is_the_grab_tolerance_conversion(self):
        geo = _geo(zoom=3.0)
        self.assertAlmostEqual(geo.widget_scale(), geo.scale, places=9)

    def test_null_geometry_maps_nothing_rather_than_guessing(self):
        self.assertFalse(NULL_GEOMETRY.valid)
        self.assertIsNone(NULL_GEOMETRY.to_image(5, 5))
        self.assertIsNone(NULL_GEOMETRY.to_widget(5, 5))


class TestIntegerCropAgreement(unittest.TestCase):
    """Qt crops on whole pixels. If the geometry recomputed a float rect
    instead of taking the one actually used, hit-testing would sit up to a
    pixel away from what was drawn."""

    def test_explicit_src_rect_wins_over_recomputation(self):
        src = QRectF(37.0, 11.0, 100.0, 80.0)      # deliberately not centred
        geo = ViewGeometry.build(
            raw_size=(RAW_W, RAW_H), disp_size=(RAW_W, RAW_H),
            pixmap_size=(200, 160), label_size=(200, 160),
            zoom=6.4, center=(320, 240), src_rect=src)
        self.assertEqual(geo.src_rect, src)
        # the crop origin must appear in the mapping
        self.assertAlmostEqual(geo.to_widget(37.0, 11.0)[0], 0.0, places=6)


class TestOneImplementation(unittest.TestCase):
    """The four copies are gone — pinned so they cannot creep back."""

    def test_picker_uses_the_base_mapping(self):
        from gui.widgets.live_target_picker import _PickerCameraView
        # No override of its own: it must resolve to CameraFeedView's.
        self.assertIs(_PickerCameraView._image_to_widget,
                      CameraFeedView._image_to_widget)
        self.assertIs(_PickerCameraView._widget_to_image,
                      CameraFeedView._widget_to_image)

    def test_clamped_point_delegates(self):
        """It used to re-derive the letterbox offset and scale itself."""
        import inspect
        from gui.widgets.live_target_picker import _PickerCameraView
        src = inspect.getsource(_PickerCameraView._clamped_image_point)
        self.assertIn("_widget_to_image", src)
        self.assertNotIn("_display.width()", src,
                         "the letterbox arithmetic came back")

    def test_no_view_recomputes_the_letterbox(self):
        """Structural sweep: only ViewGeometry may derive the centring
        offset. Anything else is a fifth copy waiting to drift."""
        import inspect
        import gui.widgets.live_target_picker as ltp
        import gui.widgets.measurement_camera_view as mcv
        import gui.widgets.target_overlay_camera_view as tocv
        for mod in (ltp, mcv, tocv):
            src = inspect.getsource(mod)
            self.assertNotIn("- pm.width()) / 2", src,
                             f"{mod.__name__} re-derives the letterbox offset")


class TestSubclassesShareThePipeline(unittest.TestCase):
    """TargetOverlayCameraView and _PickerCameraView used to build their own
    pixmap and skip ``_orient_qimage`` entirely, so ``_view_true_xform`` was
    never set — zoom added to the base would not have applied to the picker,
    which is the click-to-select surface."""

    @staticmethod
    def _frame(w=RAW_W, h=RAW_H):
        img = QImage(w, h, QImage.Format.Format_RGB888)
        img.fill(0x204060)
        return img

    def _rendered(self, view, zoom):
        """Render a frame at ``zoom`` and return the published geometry."""
        view.resize(420, 320)
        view._display.resize(400, 300)
        view._last_image_size = (RAW_W, RAW_H)
        view._zoom = zoom
        view._render_frame(self._frame())
        return view.geometry_map()

    def test_overlay_view_honours_zoom(self):
        """BEHAVIOURAL, not a substring check.

        ⚠ The first version of this test asserted ``"QPixmap.fromImage" not in
        source``. A mutation that bypassed the shared pipeline via an aliased
        import (``_P.fromImage``) SURVIVED it — a source-text guard only
        catches the spelling it happens to expect.
        """
        from gui.widgets.target_overlay_camera_view import (
            TargetOverlayCameraView)
        v = TargetOverlayCameraView(None, 0, True, "t")
        fit = self._rendered(v, 1.0)
        zoomed = self._rendered(v, 4.0)
        self.assertTrue(fit.valid and zoomed.valid)
        self.assertLess(
            zoomed.src_rect.width(), fit.src_rect.width() * 0.5,
            "the overlay view ignored the zoom — it is not going through "
            "_compose_pixmap, so the picker would not zoom at all")

    def test_base_view_honours_zoom(self):
        v = CameraFeedView(label="t", enable_capture=False)
        fit = self._rendered(v, 1.0)
        zoomed = self._rendered(v, 4.0)
        self.assertLess(zoomed.src_rect.width(), fit.src_rect.width() * 0.5)

    def test_overlay_view_clicks_track_the_zoom(self):
        """The consequence that matters: the same widget point must resolve to
        a DIFFERENT raw pixel once zoomed, and to the right one."""
        from gui.widgets.target_overlay_camera_view import (
            TargetOverlayCameraView)
        v = TargetOverlayCameraView(None, 0, True, "t")
        fit = self._rendered(v, 1.0)
        ox, oy = fit.offset
        probe = (ox + fit.pixmap_size[0] * 0.25,
                 oy + fit.pixmap_size[1] * 0.25)
        at_fit = fit.to_image(*probe)
        zoomed = self._rendered(v, 4.0)
        at_zoom = zoomed.to_image(*probe)
        self.assertIsNotNone(at_fit)
        self.assertIsNotNone(at_zoom)
        self.assertGreater(abs(at_fit[0] - at_zoom[0]), 20.0,
                           "clicks did not follow the zoom")

    def test_base_render_publishes_geometry(self):
        import inspect
        src = inspect.getsource(CameraFeedView._render_frame)
        self.assertIn("_publish_geometry", src)


class TestViewZoomApi(unittest.TestCase):

    def _view(self):
        return CameraFeedView(label="t", enable_capture=False)

    def test_starts_fitted(self):
        self.assertEqual(self._view().zoom, 1.0)

    def test_zoom_in_and_out_are_inverses_of_a_sort(self):
        v = self._view()
        v.zoom_in()
        self.assertGreater(v.zoom, 1.0)
        v.zoom_out()
        self.assertAlmostEqual(v.zoom, 1.0, places=6)

    def test_zoom_out_never_goes_below_fit(self):
        v = self._view()
        for _ in range(10):
            v.zoom_out()
        self.assertEqual(v.zoom, ZOOM_MIN)

    def test_zoom_in_is_bounded(self):
        v = self._view()
        for _ in range(40):
            v.zoom_in()
        self.assertLessEqual(v.zoom, ZOOM_MAX)

    def test_fit_resets_pan_too(self):
        v = self._view()
        v.set_zoom(4.0)
        v._view_center = (10.0, 10.0)
        v.zoom_fit()
        self.assertEqual(v.zoom, 1.0)
        self.assertIsNone(v._view_center)

    def test_disabling_zoom_returns_to_fit(self):
        v = self._view()
        v.set_zoom(3.0)
        v.set_zoom_enabled(False)
        self.assertEqual(v.zoom, 1.0)
        v.set_zoom(3.0)
        self.assertEqual(v.zoom, 1.0, "zoom applied while disabled")


if __name__ == "__main__":
    unittest.main()
