"""
stepper.py — WizardStepper top breadcrumb for the Print Setup wizard.

Visual progress indicator across the top of the page. The v7.5.2
``compact`` mode (default) targets a ~40 px total row height so the
body gets the maximum amount of vertical real estate. A legacy
``compact=False`` two-line mode is kept for the OnboardingWizard
which still has plenty of space.

Visual language matches the v7.4.x Catppuccin Mocha theme.
"""

from __future__ import annotations

from typing import Callable

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout,
    QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS


class _StepChip(QPushButton):
    """One step chip — a numbered circle next to a title label.

    State drives the palette (pending / active / done / error). In
    compact mode the chip is a single-line ~32 px tall row; in legacy
    mode it's a stacked eyebrow + title ~52 px tall.
    """

    def __init__(
        self,
        index: int,
        title: str,
        parent: QWidget | None = None,
        compact: bool = True,
    ):
        super().__init__(parent)
        self.index = index
        self.title = title
        self.compact = compact
        self.setCursor(Qt.PointingHandCursor)
        self.setFlat(True)
        self.setCheckable(False)
        self._state = "pending"

        if compact:
            self.setMinimumSize(_s(140), _s(32))
        else:
            self.setMinimumSize(_s(170), _s(52))
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

        lay = QHBoxLayout(self)
        if compact:
            lay.setContentsMargins(_s(8), _s(2), _s(12), _s(2))
            lay.setSpacing(_s(8))
            circle_px = _s(22)
        else:
            lay.setContentsMargins(_s(10), _s(8), _s(16), _s(8))
            lay.setSpacing(_s(12))
            circle_px = _s(32)

        self._circle = QLabel(str(index), self)
        self._circle.setAlignment(Qt.AlignCenter)
        self._circle.setFixedSize(circle_px, circle_px)
        lay.addWidget(self._circle, 0, Qt.AlignVCenter)

        if compact:
            self._eyebrow = None
            self._title_lbl = QLabel(title, self)
            lay.addWidget(self._title_lbl, 1, Qt.AlignVCenter)
        else:
            text_col = QVBoxLayout()
            text_col.setContentsMargins(0, 0, 0, 0)
            text_col.setSpacing(_s(1))
            self._eyebrow = QLabel(f"STEP {index}", self)
            self._title_lbl = QLabel(title, self)
            text_col.addWidget(self._eyebrow)
            text_col.addWidget(self._title_lbl)
            lay.addLayout(text_col, 1)

        self._apply_style()

    def sizeHint(self) -> QSize:  # noqa: D401
        lay = self.layout()
        if lay is not None:
            return lay.totalSizeHint()
        return super().sizeHint()

    def minimumSizeHint(self) -> QSize:  # noqa: D401
        lay = self.layout()
        if lay is not None:
            return lay.totalMinimumSize()
        return super().minimumSizeHint()

    def set_state(self, state: str) -> None:
        """state ∈ {'pending', 'active', 'done', 'error'}"""
        if state == self._state:
            return
        self._state = state
        self._apply_style()

    def _apply_style(self) -> None:
        if self._state == "active":
            circle_bg = COLORS["mauve"]
            circle_fg = COLORS["base"]
            circle_border = COLORS["mauve"]
            chip_bg = COLORS["surface0"]
            chip_border = COLORS["mauve"]
            title_color = COLORS["text"]
            eyebrow_color = COLORS["mauve"]
        elif self._state == "done":
            circle_bg = COLORS["green"]
            circle_fg = COLORS["base"]
            circle_border = COLORS["green"]
            chip_bg = "transparent"
            chip_border = "transparent"
            title_color = COLORS["text"]
            eyebrow_color = COLORS["green"]
        elif self._state == "error":
            circle_bg = COLORS["red"]
            circle_fg = COLORS["base"]
            circle_border = COLORS["red"]
            chip_bg = "transparent"
            chip_border = "transparent"
            title_color = COLORS["text"]
            eyebrow_color = COLORS["red"]
        else:  # pending
            circle_bg = COLORS["base"]
            circle_fg = COLORS["subtext0"]
            circle_border = COLORS["surface1"]
            chip_bg = "transparent"
            chip_border = "transparent"
            title_color = COLORS["subtext0"]
            eyebrow_color = COLORS["overlay0"]

        border_radius_px = _s(6 if self.compact else 8)
        self.setStyleSheet(
            f"QPushButton {{"
            f"  background: {chip_bg};"
            f"  border: 1px solid {chip_border};"
            f"  border-radius: {border_radius_px}px;"
            f"  padding: 0;"
            f"  text-align: left;"
            f"}}"
            f"QPushButton:hover {{ background: {COLORS['surface0']}; }}"
            f"QPushButton:disabled {{ background: transparent; }}"
        )

        circle_radius = (self._circle.width() // 2)
        circle_font_pt = _sf(10) if self.compact else _sf(11)
        self._circle.setStyleSheet(
            f"background: {circle_bg};"
            f"color: {circle_fg};"
            f"border: 2px solid {circle_border};"
            f"border-radius: {circle_radius}px;"
            f"font-weight: 700;"
            f"font-size: {circle_font_pt}pt;"
        )

        if self._eyebrow is not None:
            self._eyebrow.setStyleSheet(
                f"color: {eyebrow_color};"
                f"font-size: {_sf(8.5)}pt;"
                f"font-weight: 700;"
                f"letter-spacing: 1.2px;"
            )
        title_font_pt = _sf(10) if self.compact else _sf(10.5)
        self._title_lbl.setStyleSheet(
            f"color: {title_color};"
            f"font-size: {title_font_pt}pt;"
            f"font-weight: 600;"
        )


class WizardStepper(QFrame):
    """Top breadcrumb. Emits ``step_clicked(int)`` when the user picks
    a step. The orchestrator decides whether to allow the jump.

    In ``compact=True`` mode (default) the whole strip is ~40 px tall.
    """

    step_clicked = Signal(int)

    def __init__(
        self,
        titles: list[str],
        parent: QWidget | None = None,
        compact: bool = True,
    ):
        super().__init__(parent)
        self.setObjectName("wizardStepper")
        self.setAutoFillBackground(True)
        self.setStyleSheet(
            f"#wizardStepper {{"
            f"  background: {COLORS['mantle']};"
            f"  border-bottom: 1px solid {COLORS['surface0']};"
            f"}}"
        )
        self.compact = compact
        self._chips: list[_StepChip] = []
        self._current = 0

        self._outer = QHBoxLayout(self)
        if compact:
            self._outer.setContentsMargins(_s(16), _s(4), _s(16), _s(4))
            self._outer.setSpacing(_s(6))
            self.setFixedHeight(_s(40))
        else:
            self._outer.setContentsMargins(_s(20), _s(12), _s(20), _s(12))
            self._outer.setSpacing(_s(6))

        # Reserved spots for leading widget + nav bookends. None until
        # set; ``_chip_start_idx`` tracks where chips start in the
        # outer layout so re-jiggling order works.
        self._leading_widget: QWidget | None = None

        for i, title in enumerate(titles, start=1):
            chip = _StepChip(i, title, self, compact=compact)
            chip.clicked.connect(lambda _, idx=i - 1: self.step_clicked.emit(idx))
            self._chips.append(chip)
            self._outer.addWidget(chip, 0, Qt.AlignVCenter)
            if i < len(titles):
                sep = QFrame(self)
                sep.setFrameShape(QFrame.NoFrame)
                sep.setFixedHeight(1)
                sep.setMinimumWidth(_s(20 if compact else 32))
                sep.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                sep.setStyleSheet(f"background: {COLORS['surface0']};")
                self._outer.addWidget(sep, 1)

        if self._chips:
            self._chips[0].set_state("active")

    # ── Leading-widget slot (v7.5.2) ──────────────────────────────

    def set_leading_widget(self, widget: QWidget | None) -> None:
        """Insert a widget (e.g. a 'Print Setup' title label) at the
        very start of the stepper row, before the prev button + chips.

        Replaces any previously-set leading widget. Pass ``None`` to
        remove."""
        if self._leading_widget is not None:
            self._outer.removeWidget(self._leading_widget)
            self._leading_widget.setParent(None)
            self._leading_widget = None
        if widget is not None:
            widget.setParent(self)
            self._outer.insertWidget(0, widget, 0, Qt.AlignVCenter)
            self._outer.insertSpacing(1, _s(12))
            self._leading_widget = widget

    # ── Nav-button slots (sandwich the chips) ──────────────────────

    def set_nav_buttons(
        self, prev_btn: QWidget | None, next_btn: QWidget | None
    ) -> None:
        """Embed Prev / Next buttons as the row's bookends so they sit
        in the same band as the step chips. Prev goes left of the
        first chip (after any leading widget); Next goes right of the
        last chip."""
        if prev_btn is not None:
            prev_btn.setParent(self)
            # Insert before the first chip (after leading widget +
            # spacer if present).
            insert_at = 0
            if self._leading_widget is not None:
                insert_at = 2  # leading + spacer
            self._outer.insertWidget(insert_at, prev_btn, 0, Qt.AlignVCenter)
            self._outer.insertSpacing(insert_at + 1, _s(12))
        if next_btn is not None:
            next_btn.setParent(self)
            self._outer.addSpacing(_s(12))
            self._outer.addWidget(next_btn, 0, Qt.AlignVCenter)

    def set_current(self, index: int) -> None:
        if not (0 <= index < len(self._chips)):
            return
        self._current = index
        for i, chip in enumerate(self._chips):
            if i < index:
                chip.set_state("done")
            elif i == index:
                chip.set_state("active")
            else:
                chip.set_state("pending")

    def set_step_state(self, index: int, state: str) -> None:
        if 0 <= index < len(self._chips):
            self._chips[index].set_state(state)

    def set_step_enabled(self, index: int, enabled: bool) -> None:
        if 0 <= index < len(self._chips):
            self._chips[index].setEnabled(enabled)
