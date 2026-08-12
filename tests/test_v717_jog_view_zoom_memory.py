"""v7.17.0 tests — zooming a jog view must not segfault the process.

Operator: *"on any jog views if we zoom in too far or too aggressively, python
will crash"*.

ROOT CAUSE (reproduced, exit 139 / segfault, no Python traceback):
``JogWorkspaceView._paint_mosaic_overlay`` pre-scaled the mosaic to its FULL
on-screen size and cached that. ``rect`` is the mosaic's on-screen box, so it
grows with the zoom and the cached pixmap's AREA grows as **zoom²**. On this
machine's real full-plate mosaic (3000x2016 source, spanning the whole XY
envelope) a 1000x780 canvas asks for:

    zoom  1  ->     999 x   672  =  0.003 GB
    zoom 10  ->    9991 x  6717  =  0.27  GB
    zoom 20  ->   19981 x 13433  =  1.07  GB
    zoom 30  ->   29972 x 20150  =  2.42  GB   (past 2 GiB)
    zoom 60  ->   59943 x 40299  =  9.66  GB   <-- SEGFAULT

and ``.transformed()`` for the 180° plate flip doubles it. "Too aggressively"
is the same bug: a fast wheel flick jumps several zoom steps, each repaint
rebuilding a fresh giant pixmap.

THE FIX: never scale the cache beyond the SOURCE pixmap's own resolution.
Upscaling a 3000 px source to 60000 px invents no detail — the painter
magnifies the capped cache into the SAME destination rect under the
SmoothPixmapTransform hint ``paintEvent`` already sets. Peak memory becomes
O(source) instead of O(zoom²), and BELOW the cap nothing changes at all.

⚠ The dangerous wrong fix is clamping the DESTINATION rect: that silently
shrinks the drawn mosaic so it no longer registers with the well grid / needle,
i.e. the operator is shown a feature at the wrong place. ``TestDestinationRect``
exists to fail if anyone "fixes" it that way.
"""
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QRectF                                  # noqa: E402
from PySide6.QtGui import QColor, QPainter, QPixmap                # noqa: E402
from PySide6.QtWidgets import QApplication                         # noqa: E402

from gui.widgets.jog_workspace_view import JogWorkspaceView        # noqa: E402

# The real machine's XY envelope (ME3B V1 settings.json), so the numbers here
# are the operator's numbers and not a synthetic envelope that could let a
# wrong fix look right.
ENV_W_UM = 116327.0
ENV_H_UM = 76645.0
# A stand-in for a real stitched full-plate mosaic. MosaicBuilder targets
# ~1800-3000 px composites, and the extent spans the whole envelope.
SRC_W, SRC_H = 3000, 2016
CANVAS = (1000, 780)


class _SafetyLimits:
    xy_min_x = 0.0
    xy_min_y = 0.0
    xy_max_x = ENV_W_UM
    xy_max_y = ENV_H_UM

    def check_xy_near_limit(self, *_a, **_k):
        return {}


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


def _mosaic_pixmap() -> QPixmap:
    pm = QPixmap(SRC_W, SRC_H)
    pm.fill(QColor(40, 80, 120))
    return pm


def _view(zoom: float, *, flip: bool = True, out_rot: int = 0):
    v = JogWorkspaceView()
    v.set_safety_limits(_SafetyLimits())
    v.set_plate_flip_180(flip)
    if out_rot:
        v.set_mosaic_output_rotation(out_rot)
    v.set_mosaic_overlay(_mosaic_pixmap(), (0.0, 0.0, ENV_W_UM, ENV_H_UM))
    v.set_mosaic_visible(True)
    v.resize(*CANVAS)
    v._zoom = float(zoom)
    return v


class _Trace:
    """Record every QPixmap.scaled request and QRectF drawPixmap destination
    made while painting the PRODUCTION widget."""

    def __init__(self):
        self.scaled: list[tuple[int, int]] = []
        self.dests: list[QRectF] = []
        self.cache_at_scale: list[object] = []
        self._view = None

    def run(self, view):
        self._view = view
        orig_scaled = QPixmap.scaled
        orig_draw = QPainter.drawPixmap

        def traced_scaled(pm_self, *a, **k):
            if len(a) >= 2 and isinstance(a[0], int):
                self.scaled.append((a[0], a[1]))
                # Snapshot whether the PREVIOUS cache is still alive at the
                # moment the new (largest) allocation is made.
                self.cache_at_scale.append(
                    getattr(self._view, "_mosaic_scaled_cache", None))
            return orig_scaled(pm_self, *a, **k)

        def traced_draw(p_self, *a, **k):
            if a and isinstance(a[0], QRectF):
                self.dests.append(QRectF(a[0]))
            return orig_draw(p_self, *a, **k)

        QPixmap.scaled = traced_scaled
        QPainter.drawPixmap = traced_draw
        try:
            view.grab()
        finally:
            QPixmap.scaled = orig_scaled
            QPainter.drawPixmap = orig_draw
        return self


# Zoom levels spanning below-cap, at-cap and the ZOOM_MAX that segfaulted.
ZOOMS = (1, 2, 3, 5, 10, 20, 30, 40, 60)


class TestCacheIsBounded(_QtBase):
    """The guard: the cached pixmap must never exceed the source resolution."""

    def test_no_zoom_requests_a_pixmap_larger_than_the_source(self):
        for z in ZOOMS:
            t = _Trace().run(_view(z))
            for (w, h) in t.scaled:
                self.assertLessEqual(
                    w, SRC_W,
                    f"zoom {z}: asked for a {w}px-wide pixmap from a "
                    f"{SRC_W}px source")
                self.assertLessEqual(h, SRC_H, f"zoom {z}: height {h}")

    def test_peak_cache_bytes_stay_small_at_max_zoom(self):
        t = _Trace().run(_view(JogWorkspaceView._ZOOM_MAX))
        self.assertTrue(t.scaled, "nothing was scaled — test would be vacuous")
        worst = max(w * h * 4 for (w, h) in t.scaled)
        # Source-capped ARGB32 is ~24 MB; allow generous headroom but stay far
        # below the 2 GiB single-buffer / OOM regime.
        self.assertLess(worst, 64 * 1024 * 1024,
                        f"peak cache {worst/1e9:.2f} GB at ZOOM_MAX")

    def test_the_naive_answer_would_have_been_catastrophic(self):
        """Guard the guard: prove the cap is doing real work, so the tests
        above cannot pass merely because nothing scales at all."""
        v = _view(JogWorkspaceView._ZOOM_MAX)
        t = _Trace().run(v)
        dest = t.dests[-1]
        naive = dest.width() * dest.height() * 4
        self.assertGreater(
            naive, 4e9,
            "the un-capped request should be multi-GB; if this fails the "
            "fixture no longer reproduces the crash regime")
        worst = max(w * h * 4 for (w, h) in t.scaled)
        self.assertGreater(naive / worst, 100.0)

    def test_survives_painting_at_max_zoom(self):
        """The operator-visible contract: this used to kill the process."""
        for z in (JogWorkspaceView._ZOOM_MAX, JogWorkspaceView._ZOOM_MAX / 2):
            _view(z).grab()

    def test_survives_an_aggressive_wheel_flick(self):
        """"Too aggressively": one big wheel delta jumps many zoom steps, and
        each repaint used to rebuild a fresh giant pixmap."""
        v = _view(1.0)
        v.grab()
        for _ in range(6):
            v._zoom = min(JogWorkspaceView._ZOOM_MAX, v._zoom * 4.0)
            v.grab()
        self.assertGreater(v._zoom, 1.0)


class TestDestinationRect(_QtBase):
    """The overlay must still be drawn at its true on-screen size, or it stops
    registering with the wells / needle it is overlaid on."""

    def test_dest_rect_still_grows_with_zoom(self):
        widths = []
        for z in ZOOMS:
            t = _Trace().run(_view(z))
            self.assertTrue(t.dests, f"zoom {z}: overlay was never drawn")
            widths.append(t.dests[-1].width())
        for a, b in zip(widths, widths[1:]):
            self.assertGreater(b, a, "dest rect stopped tracking the zoom — "
                                     "the overlay would mis-register")

    def test_dest_rect_matches_the_mosaic_extent_on_screen(self):
        """Registration: the drawn box must equal the extent mapped through the
        view's own µm→px transform, at every zoom (capped or not)."""
        for z in ZOOMS:
            v = _view(z)
            t = _Trace().run(v)
            tl = v._um_to_px(0.0, 0.0)
            br = v._um_to_px(ENV_W_UM, ENV_H_UM)
            want = QRectF(tl, br).normalized()
            got = t.dests[-1]
            self.assertAlmostEqual(got.width(), want.width(), delta=1.5,
                                   msg=f"zoom {z} width")
            self.assertAlmostEqual(got.height(), want.height(), delta=1.5,
                                   msg=f"zoom {z} height")
            self.assertAlmostEqual(got.x(), want.x(), delta=1.5,
                                   msg=f"zoom {z} x")
            self.assertAlmostEqual(got.y(), want.y(), delta=1.5,
                                   msg=f"zoom {z} y")


class TestUnchangedBelowTheCap(_QtBase):
    """Below the cap this must behave exactly as it did before the fix."""

    def test_below_cap_the_scale_request_is_the_full_on_screen_size(self):
        for z in (1, 2, 3):
            v = _view(z)
            t = _Trace().run(v)
            dest = t.dests[-1]
            self.assertEqual(
                t.scaled[-1],
                (max(1, int(round(dest.width()))),
                 max(1, int(round(dest.height())))),
                f"zoom {z}: below the cap the cache should still be scaled to "
                f"the exact on-screen size")

    def test_cache_is_still_reused_across_repaints(self):
        """The cache exists to stop a full re-scale on every needle move
        (v7.5.x post-mosaic-build lag). Capping must not defeat it."""
        v = _view(10.0)
        _Trace().run(v)
        t2 = _Trace().run(v)          # same zoom, second paint
        self.assertEqual(t2.scaled, [],
                         "the mosaic was re-scaled on an unchanged repaint")

    def test_zoom_change_below_the_cap_rebuilds_the_cache(self):
        v = _view(1.0)
        _Trace().run(v)
        v._zoom = 2.0            # 999px -> 1998px: a genuinely different scale
        t2 = _Trace().run(v)
        self.assertTrue(t2.scaled, "cache was not rebuilt after a zoom change")

    def test_zooming_above_the_cap_needs_no_rebuild_at_all(self):
        """Falls out of capping at the source resolution: once the request is
        pinned to the source size, further zooming reuses the SAME cache and
        only the destination rect changes. So the aggressive-wheel case does no
        allocation whatsoever — which is precisely where it used to die."""
        v = _view(JogWorkspaceView._ZOOM_MAX / 4.0)
        first = _Trace().run(v)
        self.assertTrue(first.scaled, "expected an initial build")
        for z in (20.0, 40.0, JogWorkspaceView._ZOOM_MAX):
            v._zoom = z
            t = _Trace().run(v)
            self.assertEqual(t.scaled, [],
                             f"zoom {z} re-scaled the mosaic even though the "
                             f"capped size is unchanged")
            self.assertTrue(t.dests, f"zoom {z}: overlay was not drawn")


class TestPeakAllocation(_QtBase):
    def test_previous_cache_is_released_before_the_new_one_is_built(self):
        """At high zoom the old and new pixmaps are the two largest
        allocations in the process; holding both doubles the peak."""
        v = _view(1.0)
        _Trace().run(v)
        self.assertIsNotNone(v._mosaic_scaled_cache)
        v._zoom = 3.0            # below the cap, so this really does rebuild
        t = _Trace().run(v)
        self.assertTrue(t.cache_at_scale, "no rebuild observed")
        self.assertIsNone(t.cache_at_scale[-1],
                          "the previous cache was still alive while the new "
                          "one was being allocated")


class TestRotationAndFlipVariants(_QtBase):
    """The 180° plate flip and the operator's whole-mosaic output rotation both
    go through the same capped path."""

    def test_all_flip_and_rotation_combinations_stay_bounded(self):
        for flip in (False, True):
            for rot in (0, 90, 180, 270):
                v = _view(JogWorkspaceView._ZOOM_MAX, flip=flip, out_rot=rot)
                t = _Trace().run(v)
                for (w, h) in t.scaled:
                    self.assertLessEqual(max(w, h), max(SRC_W, SRC_H),
                                         f"flip={flip} rot={rot}")
                self.assertTrue(t.dests, f"flip={flip} rot={rot}: not drawn")

    def test_quarter_turn_dest_is_transposed(self):
        """A 90/270 turn occupies a transposed on-screen box — same rule as
        drawing the transformed pixmap at its own size, which is what this did
        before the fix."""
        v0 = _view(5.0, flip=False, out_rot=0)
        d0 = _Trace().run(v0).dests[-1]
        v90 = _view(5.0, flip=False, out_rot=90)
        d90 = _Trace().run(v90).dests[-1]
        self.assertAlmostEqual(d90.width(), d0.height(), delta=1.5)
        self.assertAlmostEqual(d90.height(), d0.width(), delta=1.5)


class TestFluorescenceOverlay(_QtBase):
    """The separate fluorescence overlay scales painter-side (no pre-scaled
    cache), so it is memory-safe by construction — pinned so a future
    'optimisation' cannot reintroduce the crash there."""

    def test_fluor_overlay_survives_max_zoom_without_prescaling(self):
        v = JogWorkspaceView()
        v.set_safety_limits(_SafetyLimits())
        v.set_fluor_overlay(_mosaic_pixmap(), (0.0, 0.0, ENV_W_UM, ENV_H_UM))
        v.set_fluor_visible(True)
        v.resize(*CANVAS)
        v._zoom = JogWorkspaceView._ZOOM_MAX
        t = _Trace().run(v)
        for (w, h) in t.scaled:
            self.assertLessEqual(max(w, h), max(SRC_W, SRC_H))


class TestXZSideViewZoom(_QtBase):
    """The other jog-page view. It paints no pixmaps and its zoom only remaps
    the Z axis, so it was never the crash — pinned so it stays that way."""

    def test_survives_its_max_z_zoom(self):
        from gui.widgets.xz_side_view import XZSideView

        class _SLZ(_SafetyLimits):
            z_min = -60.0
            z_max = 0.0

        for z in (1.0, 10.0, 30.0):
            v = XZSideView()
            v.set_safety_limits(_SLZ())
            v._z_zoom = z
            v._z_zoom_center_mm = 0.0
            v.resize(*CANVAS)
            v.grab()


if __name__ == "__main__":       # pragma: no cover
    unittest.main()
