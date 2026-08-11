"""
view_geometry.py — the ONE mapping between a camera view's widget pixels and
its raw frame pixels.

v7.15. Adding zoom to a live view is only safe if every consumer agrees on the
same transform. Before this module the mapping existed in FOUR independent
copies, none of which knew about zoom or pan:

    camera_feed_view._widget_to_image        (the only one ``clicked`` used)
    live_target_picker._image_to_widget
    live_target_picker._clamped_image_point
    measurement_camera_view._handle_drag

Zooming with those in place would have moved the picture under the clicks while
the target rings stayed where the un-zoomed maths put them — the v7.8 ``to_px``
identity-inverse bug returning at a different layer. So the four are replaced by
one immutable object, built once per rendered frame and used by the click
handler AND every overlay painter.

THE CHAIN, in order:

    raw frame px
      → true_xform            display orientation (mirror · flip_y · rotate)
      = displayed image px
      → − src_rect.topLeft()  the visible region when zoomed in
      → × scale               KeepAspectRatio fit into the label
      → + offset              letterbox centring
      = widget px

``to_image`` is that run backwards, and the two are exact inverses by
construction rather than by two people writing the same arithmetic twice.

Qt types only (QTransform/QPointF/QRectF) — no widgets, so it is testable
without building a view.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from PySide6.QtCore import QPointF, QRectF
from PySide6.QtGui import QTransform

#: Zoom bounds. 1.0 is fit-to-view; the ceiling matches JogWorkspaceView's
#: zoom control, which is the existing precedent in this repo.
ZOOM_MIN = 1.0
ZOOM_MAX = 40.0


def clamp_zoom(z: float) -> float:
    try:
        z = float(z)
    except (TypeError, ValueError):
        return ZOOM_MIN
    return max(ZOOM_MIN, min(ZOOM_MAX, z))


def visible_rect(disp_w: float, disp_h: float, zoom: float,
                 center: Optional[tuple] = None) -> QRectF:
    """The region of the displayed image that fills the view.

    At ``zoom == 1`` this is the whole image, so every downstream calculation
    reduces to the pre-v7.15 one. The rect is CLAMPED inside the image, which
    is what stops a pan from scrolling off into blank space.
    """
    w = max(1.0, float(disp_w))
    h = max(1.0, float(disp_h))
    z = clamp_zoom(zoom)
    vw, vh = w / z, h / z
    if center is None:
        cx, cy = w / 2.0, h / 2.0
    else:
        cx, cy = float(center[0]), float(center[1])
    # Keep the window inside the image.
    cx = min(max(cx, vw / 2.0), w - vw / 2.0)
    cy = min(max(cy, vh / 2.0), h - vh / 2.0)
    return QRectF(cx - vw / 2.0, cy - vh / 2.0, vw, vh)


@dataclass(frozen=True)
class ViewGeometry:
    """How the frame currently on screen relates to the raw frame.

    Built once per render and stored on the view, so the click handler and the
    overlay painters cannot be looking at different numbers.
    """

    raw_size: tuple = (0, 0)          # raw frame (w, h)
    disp_size: tuple = (0, 0)         # after the orientation transform
    src_rect: QRectF = None           # visible region, displayed-image coords
    pixmap_size: tuple = (0, 0)       # the scaled pixmap actually shown
    offset: tuple = (0.0, 0.0)        # pixmap top-left within the label
    true_xform: Optional[QTransform] = None   # raw → displayed

    # ── Construction ──────────────────────────────────────────────

    @staticmethod
    def build(raw_size, disp_size, pixmap_size, label_size,
              true_xform=None, zoom: float = 1.0, center=None,
              src_rect: Optional[QRectF] = None) -> "ViewGeometry":
        """``src_rect`` overrides the computed visible region.

        ⚠ Pass it whenever the caller has already CROPPED. Qt crops on integer
        pixels, so recomputing the float rect here would leave hit-testing up
        to a pixel out of step with what was actually drawn — small, but it is
        exactly the kind of drift this class exists to make impossible.
        """
        dw, dh = float(disp_size[0] or 0), float(disp_size[1] or 0)
        pw, ph = float(pixmap_size[0] or 0), float(pixmap_size[1] or 0)
        lw, lh = float(label_size[0] or 0), float(label_size[1] or 0)
        return ViewGeometry(
            raw_size=(int(raw_size[0] or 0), int(raw_size[1] or 0)),
            disp_size=(int(dw), int(dh)),
            src_rect=(src_rect if src_rect is not None
                      else visible_rect(dw, dh, zoom, center)),
            pixmap_size=(int(pw), int(ph)),
            offset=((lw - pw) / 2.0, (lh - ph) / 2.0),
            true_xform=true_xform)

    @property
    def valid(self) -> bool:
        return (self.pixmap_size[0] > 0 and self.pixmap_size[1] > 0
                and self.src_rect is not None
                and self.src_rect.width() > 0 and self.src_rect.height() > 0)

    @property
    def scale(self) -> float:
        """Widget px per displayed-image px (uniform — KeepAspectRatio)."""
        if not self.valid:
            return 1.0
        return self.pixmap_size[0] / self.src_rect.width()

    # ── The mapping ───────────────────────────────────────────────

    def to_image(self, wx: float, wy: float, *, clamp: bool = False
                 ) -> Optional[tuple]:
        """Widget px → RAW frame px.

        Returns None for a click outside the picture (the letterbox bars),
        unless ``clamp`` is set, in which case the point is pulled to the
        nearest edge — what a drag needs so the handle follows the cursor
        instead of freezing at the border.
        """
        if not self.valid:
            return None
        pw, ph = self.pixmap_size
        px = float(wx) - self.offset[0]
        py = float(wy) - self.offset[1]
        if clamp:
            px = min(max(px, 0.0), float(pw))
            py = min(max(py, 0.0), float(ph))
        elif px < 0 or py < 0 or px > pw or py > ph:
            return None
        sc = self.scale
        ix = self.src_rect.x() + px / sc
        iy = self.src_rect.y() + py / sc
        if self.true_xform is not None:
            inv, ok = self.true_xform.inverted()
            if ok:
                pt = inv.map(QPointF(ix, iy))
                return (pt.x(), pt.y())
        return (ix, iy)

    def to_widget(self, ix: float, iy: float) -> Optional[tuple]:
        """RAW frame px → widget px. Exact inverse of :meth:`to_image`."""
        if not self.valid:
            return None
        x, y = float(ix), float(iy)
        if self.true_xform is not None:
            pt = self.true_xform.map(QPointF(x, y))
            x, y = pt.x(), pt.y()
        sc = self.scale
        return ((x - self.src_rect.x()) * sc + self.offset[0],
                (y - self.src_rect.y()) * sc + self.offset[1])

    def to_pixmap(self, ix: float, iy: float) -> Optional[tuple]:
        """RAW frame px → coordinates within the SCALED pixmap.

        For overlays painted onto the pixmap after the downscale (the ghost,
        the saturation badge) rather than onto the full-resolution frame.
        """
        w = self.to_widget(ix, iy)
        if w is None:
            return None
        return (w[0] - self.offset[0], w[1] - self.offset[1])

    def widget_scale(self) -> float:
        """Widget px per RAW frame px — for converting a grab tolerance.

        The orientation is a rotation and/or reflection, both of which
        preserve length, so one number is exact rather than an approximation.
        """
        return self.scale

    def image_center(self) -> tuple:
        """Centre of the visible region, in displayed-image coords."""
        if not self.valid:
            return (0.0, 0.0)
        return (self.src_rect.center().x(), self.src_rect.center().y())


#: A geometry that maps nothing — before the first frame is rendered.
NULL_GEOMETRY = ViewGeometry()
