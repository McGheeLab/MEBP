"""
sketch_profile_view.py — XZ side-profile (elevation) of a compiled sketch
(v7.5.x).

The Print Builder's Sketch page authors a print top-down (XY) on
``SketchCanvas``. This companion widget shows the **side elevation**: the
compiled toolpath projected onto the XZ plane (height vs X), so the operator
can see the layer stack, the total print height, the plate floor (z = 0,
i.e. the calibrated plate bottom the sketch height is measured up from), and
the travel lifts between disconnected shapes.

It consumes exactly the data the 2D preview uses — the Nx7 trajectory
``[x, y, z, p1, p2, p3, t]`` plus the parallel ``pump_states`` list from
:func:`SupportClasses.SketchTrajectory.compile_to_trajectory` — so it stays in
lockstep with the canvas with no extra compile.

Print segments are drawn solid (coloured per pump, matching the canvas's
``PUMP_HEX``); travel/lift segments (where the leaving waypoint extrudes
nothing) are drawn dashed and faded. The Z axis is fit independently of X so
thin layers (≈0.2 mm) stay visible against mm-scale widths; both axis extents
are labelled in mm and a hint flags the vertical exaggeration.
"""

from __future__ import annotations

import numpy as np

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QPainter, QPen, QColor, QFont
from PySide6.QtWidgets import QWidget, QSizePolicy

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size as _sf
from gui.widgets.sketch_canvas import PUMP_HEX


class SketchProfileView(QWidget):
    """Side elevation (Z vs X) of a compiled sketch trajectory."""

    _MARGIN = 34          # px reserved for axis labels (scaled at paint time)
    _MIN_TRAVEL_FRAC = 0.0

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("sketchProfileView")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(s(90))
        self._traj: np.ndarray | None = None        # Nx7
        self._pump_states: list[list[float]] | None = None
        # Data extents (mm): x_min, x_max, z_min, z_max.
        self._xmin = self._xmax = self._zmin = self._zmax = 0.0

    # ── public API ────────────────────────────────────────────────

    def set_trajectory(self, traj, pump_states=None) -> None:
        """Set the compiled trajectory (Nx7) + per-waypoint pump states.

        ``traj`` columns are ``[x, y, z, p1, p2, p3, t]`` (mm). ``pump_states``
        is the parallel list of ``[f1, f2, f3]`` flow fractions for the segment
        *leaving* each waypoint (so segment i→i+1 is a PRINT move iff any of
        ``pump_states[i]`` is > 0). Pass ``None``/empty to clear."""
        arr = None
        if traj is not None:
            arr = np.asarray(traj, dtype=np.float64)
            if arr.ndim != 2 or arr.shape[0] < 2 or arr.shape[1] < 3:
                arr = None
        self._traj = arr
        self._pump_states = list(pump_states) if pump_states else None
        if arr is None:
            self._xmin = self._xmax = self._zmin = self._zmax = 0.0
        else:
            self._xmin, self._xmax = float(arr[:, 0].min()), float(arr[:, 0].max())
            zmin, zmax = float(arr[:, 2].min()), float(arr[:, 2].max())
            # Always include the plate floor (z = 0) in the view so the print
            # height reads against it.
            self._zmin, self._zmax = min(zmin, 0.0), max(zmax, 0.0)
        self.update()

    def clear(self) -> None:
        self.set_trajectory(None, None)

    # ── painting ──────────────────────────────────────────────────

    def _seg_is_print(self, i: int) -> bool:
        """Is the segment LEAVING waypoint ``i`` a printing (extruding) move?"""
        ps = self._pump_states
        if ps is not None and 0 <= i < len(ps):
            try:
                return any(float(f) > 0.0 for f in ps[i])
            except (TypeError, ValueError):
                return True
        return True   # no pump-state info → assume printing

    def _seg_pump(self, i: int) -> int:
        ps = self._pump_states
        if ps is not None and 0 <= i < len(ps):
            try:
                row = ps[i]
                for k in range(len(row)):
                    if float(row[k]) > 0.0:
                        return k
            except (TypeError, ValueError):
                pass
        return 0

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()

        bg = QColor(COLORS.get("mantle", "#181825"))
        p.fillRect(self.rect(), bg)

        margin = s(self._MARGIN)
        plot = QRectF(margin, s(8), max(1.0, w - margin - s(8)),
                      max(1.0, h - margin - s(8)))

        # Frame around the plot area.
        p.setPen(QPen(QColor(COLORS.get("surface1", "#45475a")), 1))
        p.drawRect(plot)

        if self._traj is None:
            p.setPen(QColor(COLORS.get("subtext0", "#a6adc8")))
            f = QFont(); f.setPointSizeF(_sf(9)); p.setFont(f)
            p.drawText(self.rect(), Qt.AlignCenter,
                       "Side profile (XZ) — draw a shape")
            p.end()
            return

        x_rng = max(self._xmax - self._xmin, 1e-6)
        z_rng = max(self._zmax - self._zmin, 1e-6)
        # Independent X/Z fit: thin layers stay visible. Z grows UP on screen.
        pad = 0.04
        def to_px(x: float, z: float) -> QPointF:
            fx = (x - self._xmin) / x_rng
            fz = (z - self._zmin) / z_rng
            px = plot.left() + (pad + (1 - 2 * pad) * fx) * plot.width()
            py = plot.bottom() - (pad + (1 - 2 * pad) * fz) * plot.height()
            return QPointF(px, py)

        # Plate floor (z = 0) reference line.
        if self._zmin <= 0.0 <= self._zmax:
            y0 = to_px(self._xmin, 0.0).y()
            floor_pen = QPen(QColor(COLORS.get("overlay0", "#6c7086")), 1)
            floor_pen.setStyle(Qt.DashLine)
            p.setPen(floor_pen)
            p.drawLine(QPointF(plot.left(), y0), QPointF(plot.right(), y0))
            p.setPen(QColor(COLORS.get("overlay0", "#6c7086")))
            f = QFont(); f.setPointSizeF(_sf(8)); p.setFont(f)
            p.drawText(QPointF(plot.left() + s(3), y0 - s(2)), "plate floor")

        traj = self._traj
        n = traj.shape[0]
        travel_pen = QPen(QColor(COLORS.get("overlay1", "#7f849c")), 1)
        travel_pen.setStyle(Qt.DashLine)
        for i in range(n - 1):
            a = to_px(float(traj[i, 0]), float(traj[i, 2]))
            b = to_px(float(traj[i + 1, 0]), float(traj[i + 1, 2]))
            if self._seg_is_print(i):
                hexc = PUMP_HEX[self._seg_pump(i) % len(PUMP_HEX)] \
                    if PUMP_HEX else "#89b4fa"
                pen = QPen(QColor(hexc), s(2))
                pen.setCapStyle(Qt.RoundCap)
                p.setPen(pen)
                p.drawLine(a, b)
            else:
                p.setPen(travel_pen)
                p.drawLine(a, b)

        # Axis extent labels.
        p.setPen(QColor(COLORS.get("subtext0", "#a6adc8")))
        f = QFont(); f.setPointSizeF(_sf(8)); p.setFont(f)
        p.drawText(QRectF(0, plot.bottom() + s(2), w, margin - s(2)),
                   Qt.AlignHCenter | Qt.AlignTop,
                   f"X  {self._xmin:.1f} … {self._xmax:.1f} mm")
        # Z range up the left margin (rotated).
        z_label = f"Z {self._zmin:.2f}…{self._zmax:.2f} mm"
        p.save()
        p.translate(s(11), plot.center().y())
        p.rotate(-90)
        p.drawText(QRectF(-plot.height() / 2, -s(11), plot.height(), s(12)),
                   Qt.AlignCenter, z_label)
        p.restore()

        # Vertical-exaggeration hint (Z scaled differently from X).
        try:
            x_per_px = x_rng / max(plot.width(), 1.0)
            z_per_px = z_rng / max(plot.height(), 1.0)
            ratio = x_per_px / z_per_px if z_per_px > 0 else 1.0
            if ratio > 1.5 or ratio < 0.67:
                p.setPen(QColor(COLORS.get("overlay0", "#6c7086")))
                p.drawText(QPointF(plot.right() - s(96), plot.top() + s(11)),
                           f"Z ×{ratio:.0f} exaggerated")
        except Exception:
            pass

        p.end()
