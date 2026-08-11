"""
raw_histogram_widget.py — log-y histogram of RAW sensor counts (v7.13).

Renders the ``hist`` array from ``compute_raw_frame_stats`` (see
gui/widgets/mono_display.py) so the operator can push exposure toward the
sensor's full scale WITHOUT clipping — the live view's percentile auto-scale
actively hides clipping, so this widget (not the image) is the exposure
instrument. The topmost bins are tinted red when any sampled pixel sits at or
above the clip level.

Pure Qt + numpy-agnostic (any indexable sequence works); custom-painted;
offscreen-testable.
"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from gui.styles import COLORS
from gui.scaling import s

# Fraction of the rightmost bins tinted as the "clip zone" when clipping is
# present (display cue only — the clipped fraction is computed upstream).
_CLIP_ZONE_FRAC = 0.02


class RawHistogramWidget(QWidget):
    """Log-y bar rendering of a raw-frame histogram; ``set_stats`` to update."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._stats: dict | None = None
        self.setMinimumHeight(s(90))
        self.setMaximumHeight(s(120))

    def set_stats(self, stats: "dict | None"):
        self._stats = stats
        self.update()

    def stats(self) -> "dict | None":
        return self._stats

    # ── painting ──────────────────────────────────────────────────
    def paintEvent(self, event):  # noqa: N802 (Qt override)
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing, False)
            rect = self.rect()
            painter.fillRect(rect, QColor(COLORS.get("base", "#181825")))

            st = self._stats
            hist = None if st is None else st.get("hist")
            if hist is None or len(hist) == 0:
                painter.setPen(QPen(QColor(COLORS.get("subtext0", "#a6adc8"))))
                painter.drawText(rect, Qt.AlignCenter, "no raw data")
                return

            n = len(hist)
            # Log-y: bar height ∝ log1p(count) / log1p(max). An all-zero
            # histogram draws a flat baseline (no divide-by-zero).
            try:
                peak = float(max(int(v) for v in hist))
            except (TypeError, ValueError):
                peak = 0.0
            denom = math.log1p(peak) if peak > 0 else 1.0

            w = rect.width()
            h = rect.height()
            clipped = bool(st.get("clipped_frac", 0.0) > 0.0)
            clip_start = int(n * (1.0 - _CLIP_ZONE_FRAC))
            bar_color = QColor(COLORS.get("mauve", "#cba6f7"))
            clip_color = QColor(COLORS.get("red", "#f38ba8"))

            for i in range(n):
                try:
                    v = float(int(hist[i]))
                except (TypeError, ValueError):
                    v = 0.0
                frac = (math.log1p(v) / denom) if v > 0 else 0.0
                bar_h = max(1 if v > 0 else 0, int(round(frac * (h - 4))))
                if bar_h <= 0:
                    continue
                x0 = int(i * w / n)
                x1 = max(x0 + 1, int((i + 1) * w / n))
                color = clip_color if (clipped and i >= clip_start) else bar_color
                painter.fillRect(x0, h - bar_h, x1 - x0, bar_h, color)

            # Baseline
            painter.setPen(QPen(QColor(COLORS.get("surface1", "#45475a"))))
            painter.drawLine(0, h - 1, w, h - 1)
        finally:
            painter.end()
