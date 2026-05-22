"""Shared polish helpers for the Hardware Setup sub-pages (v7.4.2).

These are intentionally small — they exist so the various hardware
sub-pages share the same visual rhythm without each one re-inventing
"a muted info line", "a Save row at the bottom of a section", or
"a status pill that goes red on error".

Importing from gui.widgets.components is cheap; this module just
narrows it down to the patterns the hardware pages reuse most.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton, QSizePolicy, QWidget,
)

from gui.scaling import s, sp, scaled_font_size as sf
from gui.styles import COLORS


# ── Status-line helpers ───────────────────────────────────────────

def muted_label(text: str = "") -> QLabel:
    """A small secondary-information line (gray, smaller font).

    Use for inline hints, summaries, "saved at HH:MM" footers — text the
    user can ignore unless they need it.
    """
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color: {COLORS['subtext0']};"
        f"font-size: {sf(9)}pt;"
    )
    lbl.setWordWrap(True)
    return lbl


def status_label(text: str = "", variant: str = "info") -> QLabel:
    """A coloured status line. Use for transient feedback after an action.

    Variants: ``ok`` (green), ``warn`` (yellow), ``err`` (red),
    ``info`` (subtle blue), ``muted`` (default gray).
    """
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    apply_status_variant(lbl, variant)
    return lbl


def apply_status_variant(lbl: QLabel, variant: str) -> None:
    """Re-style an existing status label without rebuilding it."""
    palette = {
        "ok":    COLORS['green'],
        "warn":  COLORS['yellow'],
        "err":   COLORS['red'],
        "info":  COLORS['blue'],
        "muted": COLORS['subtext0'],
    }
    color = palette.get(variant, COLORS['subtext0'])
    lbl.setStyleSheet(
        f"color: {color};"
        f"font-size: {sf(9)}pt;"
    )


# ── Action-row helper ─────────────────────────────────────────────

def make_action_row(*buttons: QPushButton,
                    status: QLabel | None = None,
                    align_right: bool = True) -> QHBoxLayout:
    """Standard "buttons + status line" row at the bottom of a section.

    Layout: ``[status]  stretch  [btn1] [btn2] ...`` (or buttons on the
    left if ``align_right=False``).

    Caller still owns ``status`` — pass it in so you can update its
    text later via :func:`apply_status_variant`.
    """
    row = QHBoxLayout()
    row.setSpacing(s(8))
    if align_right:
        if status is not None:
            row.addWidget(status, 1)
        else:
            row.addStretch(1)
        for b in buttons:
            row.addWidget(b)
    else:
        for b in buttons:
            row.addWidget(b)
        if status is not None:
            row.addWidget(status, 1)
        else:
            row.addStretch(1)
    return row


# ── Button factory ────────────────────────────────────────────────

def primary_button(text: str, tooltip: str | None = None) -> QPushButton:
    """Primary CTA button styling (accent purple)."""
    b = QPushButton(text)
    b.setObjectName("accentBtn")
    b.setCursor(Qt.PointingHandCursor)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def success_button(text: str, tooltip: str | None = None) -> QPushButton:
    """Connect / Apply / "go ahead" button (green)."""
    b = QPushButton(text)
    b.setObjectName("successBtn")
    b.setCursor(Qt.PointingHandCursor)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def danger_button(text: str, tooltip: str | None = None) -> QPushButton:
    """Destructive button (red): Disconnect / Remove / Delete."""
    b = QPushButton(text)
    b.setObjectName("dangerBtn")
    b.setCursor(Qt.PointingHandCursor)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def secondary_button(text: str, tooltip: str | None = None) -> QPushButton:
    """Default secondary button — no special objectName."""
    b = QPushButton(text)
    b.setCursor(Qt.PointingHandCursor)
    if tooltip:
        b.setToolTip(tooltip)
    return b


# ── Sub-heading inside a card ─────────────────────────────────────

def sub_heading(text: str) -> QLabel:
    """An in-section heading for grouping related rows.

    Sits visually below the QGroupBox title — used inside long
    sections that need internal hierarchy (e.g. "Axis Mapping" vs
    "Stepper Calibration" inside the Device sub-page).
    """
    lbl = QLabel(text.upper())
    lbl.setStyleSheet(
        f"color: {COLORS.get('subtext0', '#a6adc8')};"
        f"font-size: {sf(8.5)}pt;"
        f"font-weight: 700;"
        f"letter-spacing: 0.6px;"
        f"padding-top: {sp(4)};"
        f"padding-bottom: {sp(2)};"
    )
    return lbl


# ── Horizontal divider ────────────────────────────────────────────

def divider() -> QWidget:
    """Thin horizontal rule for separating row groups inside a section."""
    w = QWidget()
    w.setFixedHeight(1)
    w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    w.setStyleSheet(f"background-color: {COLORS['surface1']};")
    return w
