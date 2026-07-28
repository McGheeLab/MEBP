"""
components.py — Reusable UI primitives for MEBP (v7.4.0-a).

Small library of widgets that pages compose from instead of rolling their own
cards, status badges, section headers, etc.

    Card           Titled container, optional collapsible.
    StatusBadge    Colored pill: ok / warn / err / info / pending.
    SectionHeader  Page-level header label (uses PAGE_HEADER_STYLE).
    FormRow        Label + field + optional inline help text (progressive disclosure).
    WizardStep     Header (Step N/M, title) + body slot + prev/next buttons.
    LoadingBanner  Thin colored strip with spinner + message (mountable in MainWindow).

All visual constants are sourced from gui.styles.COLORS and dimensions from
gui.scaling — no inline hex, no hardcoded pixel literals.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal, QTimer, QPropertyAnimation, QEasingCurve
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFrame, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QSizePolicy,
)

from gui.styles import COLORS, build_page_header_style
from gui.scaling import s, sf, sp, scaled_font_size


# ── Card ───────────────────────────────────────────────────────────

class Card(QFrame):
    """Titled container with the standard surface0 / surface1 styling.

    Usage::

        card = Card("Pump Channels")
        card.add_widget(pump_widget)

        collapsible = Card("Advanced", collapsible=True)
        collapsible.add_widget(advanced_form)
        collapsible.set_collapsed(True)
    """

    def __init__(self, title: str | None = None, collapsible: bool = False,
                 parent: QWidget | None = None, *,
                 flush: bool = False, compact: bool = False):
        """Build a card.

        Args:
            title: Optional title shown at the top of the card.
            collapsible: When True, render a ▾ chevron that folds the
                          body.
            flush: When True, the body sits edge-to-edge with the card
                   frame (no horizontal / bottom padding around it).
                   The title — if shown — keeps its own internal padding
                   so it reads correctly. Use for full-bleed
                   visualisations that want to claim every pixel.
            compact: When True, the title header uses tight vertical
                   padding and a slightly smaller font so it claims as
                   little vertical space as possible. Use when the body
                   (e.g. a plate view) should own the height.
        """
        super().__init__(parent)
        self.setObjectName("componentCard")
        self.setStyleSheet(
            f"QFrame#componentCard {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(8)};"
            f"}}"
        )

        outer = QVBoxLayout(self)
        if flush:
            outer.setContentsMargins(0, 0, 0, 0)
            outer.setSpacing(0)
        else:
            outer.setContentsMargins(s(12), s(10), s(12), s(12))
            outer.setSpacing(s(8))

        self._collapsible = collapsible
        self._collapsed = False
        self._title_label: QLabel | None = None
        self._toggle_btn: QPushButton | None = None

        if title is not None:
            header = QHBoxLayout()
            if flush:
                # Re-introduce internal padding for the title only,
                # since the outer layout no longer provides it.
                if compact:
                    header.setContentsMargins(s(12), s(3), s(12), s(3))
                else:
                    header.setContentsMargins(s(12), s(8), s(12), s(6))
            else:
                header.setContentsMargins(0, 0, 0, 0)
            header.setSpacing(s(6))

            self._title_label = QLabel(title)
            # v7.5.x: don't let a long title floor the card's width. An Ignored
            # horizontal policy + tiny minimum means the title takes the space it
            # can and elides/clips on a narrow panel instead of forcing a
            # horizontal overflow — so cards in the responsive context panel
            # shrink to fit their container.
            from PySide6.QtWidgets import QSizePolicy as _QSP
            self._title_label.setSizePolicy(_QSP.Ignored, _QSP.Preferred)
            self._title_label.setMinimumWidth(s(1))
            self._title_label.setStyleSheet(
                f"color: {COLORS['blue']};"
                f"font-size: {sf(9) if compact else sf(10)}pt;"
                f"font-weight: 600;"
            )
            header.addWidget(self._title_label, 1)

            if collapsible:
                self._toggle_btn = QPushButton("▾")
                self._toggle_btn.setObjectName("flatBtn")
                self._toggle_btn.setFixedSize(s(20), s(20))
                self._toggle_btn.setCursor(Qt.PointingHandCursor)
                self._toggle_btn.clicked.connect(self._on_toggle)
                header.addWidget(self._toggle_btn)

            outer.addLayout(header)

        self._body = QWidget(self)
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 0, 0, 0)
        self._body_layout.setSpacing(s(6))
        outer.addWidget(self._body)

    def add_widget(self, w: QWidget):
        self._body_layout.addWidget(w)

    def add_layout(self, lay):
        self._body_layout.addLayout(lay)

    def body_layout(self) -> QVBoxLayout:
        return self._body_layout

    def set_collapsed(self, collapsed: bool):
        if not self._collapsible:
            return
        self._collapsed = collapsed
        self._body.setVisible(not collapsed)
        if self._toggle_btn is not None:
            self._toggle_btn.setText("▸" if collapsed else "▾")

    def _on_toggle(self):
        self.set_collapsed(not self._collapsed)


# ── StatusBadge ────────────────────────────────────────────────────

class StatusBadge(QLabel):
    """Colored pill label for status: ok / warn / err / info / pending.

    Usage::

        badge = StatusBadge("Connected", variant="ok")
        badge.set_status("err", "Disconnected")
    """

    _VARIANT_COLORS = {
        "ok":      ("green",    "base"),
        "warn":    ("yellow",   "base"),
        "err":     ("red",      "base"),
        "info":    ("blue",     "base"),
        "pending": ("subtext0", "base"),
    }

    def __init__(self, text: str = "", variant: str = "info",
                 parent: QWidget | None = None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        self._variant = variant
        self._apply_style()

    def set_status(self, variant: str, text: str | None = None):
        if variant not in self._VARIANT_COLORS:
            variant = "info"
        self._variant = variant
        if text is not None:
            self.setText(text)
        self._apply_style()

    def _apply_style(self):
        fg_key, _ = self._VARIANT_COLORS.get(self._variant, ("blue", "base"))
        fg = COLORS[fg_key]
        # Background is a tinted version of the foreground — we approximate by
        # using the foreground at low opacity via rgba; QSS supports rgba bg.
        # Foreground stays opaque for legibility.
        self.setStyleSheet(
            f"QLabel {{"
            f"  color: {fg};"
            f"  background-color: rgba({_hex_to_rgb(fg)}, 38);"
            f"  border: 1px solid {fg};"
            f"  border-radius: {sp(10)};"
            f"  padding: {sp(2)} {sp(8)};"
            f"  font-size: {sf(9)}pt;"
            f"  font-weight: 600;"
            f"}}"
        )


def _hex_to_rgb(hex_color: str) -> str:
    """'#a6e3a1' → '166, 227, 161'."""
    h = hex_color.lstrip("#")
    return f"{int(h[0:2], 16)}, {int(h[2:4], 16)}, {int(h[4:6], 16)}"


# ── SectionHeader ──────────────────────────────────────────────────

class SectionHeader(QLabel):
    """Page-level section header. Wraps build_page_header_style."""

    def __init__(self, text: str, parent: QWidget | None = None):
        super().__init__(text, parent)
        self.setStyleSheet(build_page_header_style(_current_scale()))


def _current_scale() -> float:
    from gui.scaling import scale_factor
    return scale_factor()


# ── FormRow ────────────────────────────────────────────────────────

class FormRow(QWidget):
    """Label + field + optional inline help text (revealed by help-mode).

    Usage::

        row = FormRow("Syringe volume", spin_widget,
                      help_text="Total syringe volume in µL. Used to convert "
                                "between µL and stage mm.")
        layout.addWidget(row)

        # When help-mode is enabled globally:
        row.set_help_visible(True)
    """

    def __init__(self, label: str, field: QWidget,
                 help_text: str | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(2))

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(8))

        self._label = QLabel(label)
        self._label.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(10)}pt;"
        )
        row.addWidget(self._label)
        row.addStretch(1)
        row.addWidget(field)
        lay.addLayout(row)

        self._help_label = QLabel("")
        self._help_label.setWordWrap(True)
        self._help_label.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(9)}pt;"
            f"padding: {sp(2)} 0 {sp(4)} {sp(4)};"
            f"border-left: 2px solid {COLORS['surface1']};"
        )
        self._help_label.setVisible(False)
        lay.addWidget(self._help_label)

        if help_text:
            self.set_help(help_text)

    def set_help(self, text: str):
        self._help_label.setText(text)

    def set_help_visible(self, visible: bool):
        has_text = bool(self._help_label.text().strip())
        self._help_label.setVisible(visible and has_text)


# ── WizardStep ─────────────────────────────────────────────────────

class WizardStep(QFrame):
    """Wizard step container: header + body + prev/next/finish controls.

    Signals:
        prev_clicked()
        next_clicked()
        completed()       — emitted on Finish (last step only)

    Set is_last=True to make the next button say "Finish" and emit completed
    instead of next_clicked.
    """

    prev_clicked = Signal()
    next_clicked = Signal()
    completed = Signal()

    def __init__(self, step_index: int, total_steps: int, title: str,
                 body: QWidget, is_last: bool = False, is_first: bool = False,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("wizardStep")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(16), s(16), s(16), s(16))
        outer.setSpacing(s(12))

        # Header
        header_row = QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)

        step_lbl = QLabel(f"Step {step_index} of {total_steps}")
        step_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(9)}pt;"
            f"font-weight: 600;"
            f"letter-spacing: 1px;"
        )
        header_row.addWidget(step_lbl)
        header_row.addStretch(1)
        outer.addLayout(header_row)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(16)}pt;"
            f"font-weight: 700;"
        )
        outer.addWidget(title_lbl)

        # Body
        body.setParent(self)
        outer.addWidget(body, 1)

        # Nav row
        nav = QHBoxLayout()
        nav.setContentsMargins(0, s(8), 0, 0)
        nav.setSpacing(s(8))

        self._prev_btn = QPushButton("← Back")
        self._prev_btn.setObjectName("flatBtn")
        self._prev_btn.setCursor(Qt.PointingHandCursor)
        self._prev_btn.clicked.connect(self.prev_clicked.emit)
        self._prev_btn.setEnabled(not is_first)

        self._next_btn = QPushButton("Finish ✓" if is_last else "Next →")
        self._next_btn.setObjectName("successBtn" if is_last else "accentBtn")
        self._next_btn.setCursor(Qt.PointingHandCursor)
        if is_last:
            self._next_btn.clicked.connect(self.completed.emit)
        else:
            self._next_btn.clicked.connect(self.next_clicked.emit)

        nav.addWidget(self._prev_btn)
        nav.addStretch(1)
        nav.addWidget(self._next_btn)
        outer.addLayout(nav)

    def set_next_enabled(self, enabled: bool):
        self._next_btn.setEnabled(enabled)


# ── LoadingBanner ──────────────────────────────────────────────────

class LoadingBanner(QFrame):
    """Thin horizontal banner with a spinner + message.

    Mount in MainWindow above the content stack; expose show_for(msg) /
    hide() on MainWindow. Designed to convey "something is happening" for
    long operations (config load, calibration scan, etc.).

    The spinner is a Unicode braille animation — no QMovie required.
    """

    _SPIN_FRAMES = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("loadingBanner")
        self.setStyleSheet(
            f"QFrame#loadingBanner {{"
            f"  background-color: rgba({_hex_to_rgb(COLORS['yellow'])}, 30);"
            f"  border-bottom: 1px solid {COLORS['yellow']};"
            f"}}"
        )
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(s(26))
        self.setVisible(False)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(s(12), 0, s(12), 0)
        lay.setSpacing(s(8))

        self._spin_label = QLabel(self._SPIN_FRAMES[0])
        self._spin_label.setStyleSheet(
            f"color: {COLORS['yellow']};"
            f"font-size: {sf(12)}pt;"
        )
        lay.addWidget(self._spin_label)

        self._msg_label = QLabel("")
        self._msg_label.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(10)}pt;"
        )
        lay.addWidget(self._msg_label)
        lay.addStretch(1)

        self._frame_idx = 0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def show_for(self, message: str):
        self._msg_label.setText(message)
        self._frame_idx = 0
        self._spin_label.setText(self._SPIN_FRAMES[0])
        self.setVisible(True)
        self._timer.start(90)

    def hide(self):
        self._timer.stop()
        self.setVisible(False)

    def _tick(self):
        self._frame_idx = (self._frame_idx + 1) % len(self._SPIN_FRAMES)
        self._spin_label.setText(self._SPIN_FRAMES[self._frame_idx])
