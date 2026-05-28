"""
chrome.py — Visual polish helpers for the Print Setup wizard.

Reusable framing widgets that wrap each wizard step's body with a
consistent header (title + subtitle + divider) and content card.
Keeps the per-step files thin while giving every step the same
professional shell.

Catppuccin Mocha palette is sourced from ``gui.styles.COLORS`` and
all pixel sizes go through ``gui.scaling.s/sf`` so the visual reads
correctly on 1080p → 4K displays.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS


# ── Step descriptions (title + subtitle) ──────────────────────────────
# Centralized so the orchestrator and stepper agree on copy.

STEP_DESCRIPTIONS: dict[str, tuple[str, str]] = {
    "Print Objects": (
        "Print Objects",
        "Design or import the shapes you want to print, then organize "
        "them into collections.",
    ),
    "Wells & Roles": (
        "Wells & Roles",
        "Place your print collections on the plate and assign service "
        "wells (wash, waste, buffer, ink).",
    ),
    "Plan & Run": (
        "Plan & Run",
        "Tune the execution parameters, validate, generate the "
        "trajectory, and send it to the printer.",
    ),
}


# ── Step header / content frame ──────────────────────────────────────


def _step_header(title: str, subtitle: str, parent: QWidget) -> QWidget:
    """A polished header strip — title + one-line subtitle + thin rule."""
    host = QFrame(parent)
    host.setObjectName("psStepHeader")
    host.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    lay = QVBoxLayout(host)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(_s(2))

    title_lbl = QLabel(title, host)
    title_lbl.setStyleSheet(
        f"color: {COLORS['text']}; "
        f"font-size: {_sf(17)}pt; "
        f"font-weight: 700; "
        f"letter-spacing: -0.2px;"
    )
    lay.addWidget(title_lbl)

    if subtitle:
        sub_lbl = QLabel(subtitle, host)
        sub_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"font-size: {_sf(10.5)}pt;"
        )
        sub_lbl.setWordWrap(True)
        lay.addWidget(sub_lbl)

    # Hairline divider
    rule = QFrame(host)
    rule.setFrameShape(QFrame.NoFrame)
    rule.setFixedHeight(1)
    rule.setStyleSheet(
        f"background: {COLORS['surface0']}; "
        f"margin-top: {_s(8)}px;"
    )
    lay.addSpacing(_s(8))
    lay.addWidget(rule)

    return host


def build_step_frame(
    title: str,
    subtitle: str,
    body: QWidget,
    parent: QWidget | None = None,
) -> QWidget:
    """Wrap a step body with a polished header card.

    Layout::

        ┌── outer (Catppuccin mantle) ─────────────────────┐
        │ ┌── card (Catppuccin base) ──────────────────┐  │
        │ │ Title                                       │  │
        │ │ Subtitle                                    │  │
        │ │ ─────────────────────────────────────────── │  │
        │ │ <body — the embedded tab widget>            │  │
        │ └────────────────────────────────────────────┘  │
        └─────────────────────────────────────────────────┘

    The outer band is the wizard's neutral background; the inner card
    is a subtle elevation so each step reads as a focused work area.
    """
    outer = QWidget(parent)
    outer.setObjectName("psStepFrameOuter")
    outer.setAutoFillBackground(True)
    outer.setStyleSheet(
        f"#psStepFrameOuter {{"
        f"  background: {COLORS['mantle']};"
        f"}}"
    )
    outer_lay = QVBoxLayout(outer)
    outer_lay.setContentsMargins(_s(16), _s(16), _s(16), _s(16))
    outer_lay.setSpacing(0)

    card = QFrame(outer)
    card.setObjectName("psStepCard")
    card.setStyleSheet(
        f"#psStepCard {{"
        f"  background: {COLORS['base']};"
        f"  border: 1px solid {COLORS['surface0']};"
        f"  border-radius: {_s(8)}px;"
        f"}}"
    )
    card_lay = QVBoxLayout(card)
    card_lay.setContentsMargins(_s(20), _s(18), _s(20), _s(18))
    card_lay.setSpacing(_s(12))

    card_lay.addWidget(_step_header(title, subtitle, card))

    body.setParent(card)
    card_lay.addWidget(body, 1)
    body.show()

    outer_lay.addWidget(card, 1)
    return outer


# ── Empty-state placeholder ──────────────────────────────────────────


def build_empty_placeholder(message: str, parent: QWidget | None = None) -> QWidget:
    """A minimal centered placeholder used when a step body is missing."""
    host = QFrame(parent)
    host.setStyleSheet(f"background: {COLORS['base']};")
    lay = QVBoxLayout(host)
    lay.addStretch(1)
    lbl = QLabel(message, host)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setStyleSheet(
        f"color: {COLORS['subtext0']}; "
        f"font-style: italic; "
        f"font-size: {_sf(11)}pt;"
    )
    lay.addWidget(lbl)
    lay.addStretch(1)
    return host
