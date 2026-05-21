"""
scaling.py — DPI-aware scaling for MEBP GUI.

All hardcoded pixel dimensions in the UI are designed for 96 DPI (1080p).
On higher-DPI screens (1440p, 4K, Retina), this module scales them
proportionally so the UI looks correct at any resolution.

Usage::

    from gui.scaling import s, sf, scaled_font_size

    btn.setFixedSize(s(44), s(44))       # 44px at 96 DPI, 66px at 144 DPI
    f"font-size: {sf(10)}pt;"            # 10pt at 96 DPI, 15pt at 144 DPI
    label.setFont(QFont("Segoe UI", scaled_font_size(12)))

The scale factor is computed once at import time from the primary screen's
logical DPI. It can be overridden via the MEBP_UI_SCALE environment variable
for testing (e.g. MEBP_UI_SCALE=1.5).
"""

from __future__ import annotations

import os

# ── Scale factor ───────────────────────────────────────────────────
# Computed lazily on first call (QApplication must exist by then).

_scale: float | None = None
BASELINE_DPI = 96.0


def _compute_scale() -> float:
    """Detect the screen DPI scale factor relative to 96 DPI baseline."""
    # Allow manual override for testing
    env = os.environ.get("MEBP_UI_SCALE")
    if env:
        try:
            return max(0.5, float(env))
        except ValueError:
            pass

    try:
        from PySide6.QtWidgets import QApplication
        app = QApplication.instance()
        if app:
            screen = app.primaryScreen()
            if screen:
                dpi = screen.logicalDotsPerInch()
                return max(1.0, dpi / BASELINE_DPI)
    except Exception:
        pass

    return 1.0


def scale_factor() -> float:
    """Return the current DPI scale factor (>= 1.0)."""
    global _scale
    if _scale is None:
        _scale = _compute_scale()
    return _scale


def reset_scale():
    """Force re-detection (e.g. after screen change). Call sparingly."""
    global _scale
    _scale = None


# ── Public helpers ─────────────────────────────────────────────────

def s(px: int | float) -> int:
    """Scale a pixel dimension from 96-DPI design to actual screen DPI.

    Use for: setFixedSize, setMinimumWidth, setMaximumHeight, spacing, etc.
    """
    return round(px * scale_factor())


def sf(pt: float) -> float:
    """Scale a font size (pt) for QSS strings.

    Returns a float suitable for f-string interpolation:
        f"font-size: {sf(10)}pt;"
    """
    return round(pt * scale_factor(), 1)


def scaled_font_size(pt: int) -> int:
    """Scale a font size (pt) for QFont constructors.

    Returns an integer suitable for QFont("name", size):
        QFont("Segoe UI", scaled_font_size(12))
    """
    return round(pt * scale_factor())


def sp(px: int | float) -> str:
    """Scale a pixel dimension and format as a QSS string.

    Shorthand for f"{s(px)}px" — for use inside QSS templates::

        f"padding: {sp(4)} {sp(8)};"
        f"border-radius: {sp(6)};"
    """
    return f"{s(px)}px"
