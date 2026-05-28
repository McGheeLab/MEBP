"""
context_panel.py — Left context pane for the Print Setup wizard (v7.5.4).

v7.5.4 split:
  * The vertical wizard step strip has been **moved out** of the
    context panel into a standalone ``WizardStepStrip`` widget that
    the orchestrator mounts inside the PrintingModePage's tabbar
    (under the mode icons). All step navigation buttons now live in
    the same far-left strip as the mode tabs.
  * The context panel itself is reduced to a single step-aware
    ``Tools`` area — a QStackedWidget keyed by wizard step index.

Hardware and Print Settings tabs have been moved to their own
sub-pages of the Printing mode (v7.5.3). Print List has been merged
into the wizard's right sidebar above Objects in This Print +
Summary.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QScrollArea, QSizePolicy,
    QStackedWidget, QVBoxLayout, QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS

from .models import PrintObjectsModel


_STEP_STRIP_WIDTH = 64   # base px (scaled) — used by WizardStepStrip


# ═══════════════════════════════════════════════════════════════════
# WizardStepStrip — vertical step nav (extracted in v7.5.4)
# ═══════════════════════════════════════════════════════════════════


class _StepIcon(QPushButton):
    """One chip in the vertical step strip — numbered circle.

    State drives palette (pending / active / done / error).
    """

    def __init__(self, index: int, parent: QWidget | None = None):
        super().__init__(parent)
        self.index = index
        self.setCursor(Qt.PointingHandCursor)
        self.setFlat(True)
        self.setText(str(index))
        self.setFixedSize(_s(40), _s(40))
        self._state = "pending"
        self._apply_style()

    def set_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        self._apply_style()

    def _apply_style(self) -> None:
        if self._state == "active":
            bg, fg, border = (
                COLORS["mauve"], COLORS["base"], COLORS["mauve"])
        elif self._state == "done":
            bg, fg, border = (
                COLORS["green"], COLORS["base"], COLORS["green"])
        elif self._state == "error":
            bg, fg, border = (
                COLORS["red"], COLORS["base"], COLORS["red"])
        else:  # pending
            bg, fg, border = (
                COLORS["base"], COLORS["subtext0"], COLORS["surface1"])
        self.setStyleSheet(
            f"QPushButton {{"
            f"  background: {bg};"
            f"  color: {fg};"
            f"  border: 2px solid {border};"
            f"  border-radius: {_s(20)}px;"
            f"  font-weight: 700;"
            f"  font-size: {_sf(12)}pt;"
            f"}}"
            f"QPushButton:hover {{ border-color: {COLORS['mauve']}; }}"
            f"QPushButton:disabled {{"
            f"  color: {COLORS['overlay0']};"
            f"  border-color: {COLORS['surface0']};"
            f"}}"
        )


class WizardStepStrip(QFrame):
    """Vertical wizard step navigation. v7.5.4: lives in the
    PrintingModePage's tabbar — under the mode icons — alongside the
    other mode-level nav buttons on the far left.

    Signals:
        step_clicked(int)  — user clicked step icon ``idx`` in the strip.
        prev_clicked()     — Back button.
        next_clicked()     — Next button.
    """

    step_clicked = Signal(int)
    prev_clicked = Signal()
    next_clicked = Signal()

    def __init__(
        self,
        step_count: int = 3,
        step_titles: list[str] | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._step_count = step_count
        self._step_titles = step_titles or [
            f"Step {i + 1}" for i in range(step_count)
        ]
        self.setObjectName("psWizardStepStrip")
        self.setFixedWidth(_s(_STEP_STRIP_WIDTH))
        self.setStyleSheet(
            f"#psWizardStepStrip {{ background: transparent; }}"
        )

        lay = QVBoxLayout(self)
        lay.setContentsMargins(_s(4), _s(8), _s(4), _s(8))
        lay.setSpacing(_s(8))
        lay.setAlignment(Qt.AlignTop)

        # Small "STEP" eyebrow at the top
        eyebrow = QLabel("STEP")
        eyebrow.setAlignment(Qt.AlignCenter)
        eyebrow.setStyleSheet(
            f"color: {COLORS['overlay0']};"
            f"font-size: {_sf(8)}pt;"
            f"font-weight: 700;"
            f"letter-spacing: 1.5px;"
        )
        lay.addWidget(eyebrow)

        self._step_icons: list[_StepIcon] = []
        for i in range(self._step_count):
            chip = _StepIcon(i + 1, self)
            chip.setToolTip(self._step_titles[i])
            chip.clicked.connect(
                lambda _, idx=i: self.step_clicked.emit(idx)
            )
            lay.addWidget(chip, 0, Qt.AlignHCenter)
            self._step_icons.append(chip)
            if i < self._step_count - 1:
                dot = QFrame(self)
                dot.setFixedSize(_s(2), _s(12))
                dot.setStyleSheet(f"background: {COLORS['surface1']};")
                lay.addWidget(dot, 0, Qt.AlignHCenter)

        lay.addStretch(1)

        # Back / Next at the bottom of the strip
        self._prev_btn = self._make_strip_button("↑", primary=False)
        self._prev_btn.setToolTip("Previous step")
        self._prev_btn.clicked.connect(self.prev_clicked.emit)
        self._next_btn = self._make_strip_button("↓", primary=True)
        self._next_btn.setToolTip("Next step")
        self._next_btn.clicked.connect(self.next_clicked.emit)
        lay.addWidget(self._prev_btn, 0, Qt.AlignHCenter)
        lay.addWidget(self._next_btn, 0, Qt.AlignHCenter)

        if self._step_icons:
            self._step_icons[0].set_state("active")

    @staticmethod
    def _make_strip_button(text: str, primary: bool) -> QPushButton:
        btn = QPushButton(text)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setFixedSize(_s(36), _s(36))
        if primary:
            btn.setStyleSheet(
                f"QPushButton {{"
                f"  background: {COLORS['mauve']};"
                f"  color: {COLORS['base']};"
                f"  border: 1px solid {COLORS['mauve']};"
                f"  border-radius: {_s(8)}px;"
                f"  font-weight: 700;"
                f"  font-size: {_sf(14)}pt;"
                f"}}"
                f"QPushButton:hover {{"
                f"  background: {COLORS['pink']};"
                f"  border-color: {COLORS['pink']};"
                f"}}"
                f"QPushButton:disabled {{"
                f"  background: {COLORS['surface0']};"
                f"  color: {COLORS['overlay0']};"
                f"  border-color: {COLORS['surface0']};"
                f"}}"
            )
        else:
            btn.setStyleSheet(
                f"QPushButton {{"
                f"  background: transparent;"
                f"  color: {COLORS['subtext0']};"
                f"  border: 1px solid {COLORS['surface1']};"
                f"  border-radius: {_s(8)}px;"
                f"  font-weight: 700;"
                f"  font-size: {_sf(14)}pt;"
                f"}}"
                f"QPushButton:hover {{"
                f"  background: {COLORS['surface0']};"
                f"  color: {COLORS['text']};"
                f"}}"
                f"QPushButton:disabled {{"
                f"  color: {COLORS['overlay0']};"
                f"  border-color: {COLORS['surface0']};"
                f"}}"
            )
        return btn

    def set_active_step(self, step_index: int) -> None:
        if not (0 <= step_index < self._step_count):
            return
        for i, chip in enumerate(self._step_icons):
            if i < step_index:
                chip.set_state("done")
            elif i == step_index:
                chip.set_state("active")
            else:
                chip.set_state("pending")

    def set_step_state(self, step_index: int, state: str) -> None:
        if 0 <= step_index < len(self._step_icons):
            self._step_icons[step_index].set_state(state)

    def set_prev_enabled(self, enabled: bool) -> None:
        self._prev_btn.setEnabled(enabled)

    def set_next_enabled(self, enabled: bool) -> None:
        self._next_btn.setEnabled(enabled)


# ═══════════════════════════════════════════════════════════════════
# PrintSetupContextPanel — Tools only (v7.5.4)
# ═══════════════════════════════════════════════════════════════════


class PrintSetupContextPanel(QWidget):
    """Left context pane = the step-aware ``Tools`` area.

    v7.5.4: the wizard step strip has moved out (now lives in the
    PrintingModePage tabbar as a ``WizardStepStrip``). This panel is
    now just the Tools content — a QStackedWidget keyed by step
    index. Each step's tools page is populated by the orchestrator
    via ``set_step_tools``.

    Signals:
        edit_hw_requested() — kept for backward compat (legacy navigate).
    """

    edit_hw_requested = Signal()

    def __init__(
        self,
        objects_model: PrintObjectsModel,
        # Kept for legacy callers — unused in v7.5.4. Print Settings
        # has moved to its own Printing Mode sub-page.
        print_settings_body: QWidget | None = None,
        parent: QWidget | None = None,
        step_count: int = 3,
        step_titles: list[str] | None = None,
    ) -> None:
        super().__init__(parent)
        self._objects_model = objects_model
        self._step_count = step_count
        self._step_titles = step_titles or [
            f"Step {i + 1}" for i in range(step_count)
        ]
        self._tools_pages: dict[int, QWidget] = {}

        self.setObjectName("printSetupContextPanel")
        self.setStyleSheet(
            f"#printSetupContextPanel {{"
            f"  background: {COLORS['base']};"
            f"}}"
        )

        # Tools area only — full width.
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._tools_stack = QStackedWidget(self)
        self._tools_stack.setStyleSheet(
            f"QStackedWidget {{ background: {COLORS['base']}; }}"
        )
        for i in range(self._step_count):
            ph = QWidget()
            ph.setStyleSheet(f"background: {COLORS['base']};")
            self._tools_stack.addWidget(ph)
            self._tools_pages[i] = ph

        outer.addWidget(self._tools_stack, 1)
        self.set_active_step(0)

    # ── Public API ────────────────────────────────────────────────

    def set_step_tools(self, step_index: int, panels: list[QWidget]) -> None:
        """Install the tool widgets for a given step. ``panels`` are
        stacked top-to-bottom inside a **scrollable** host so a
        crowded Tools column (Designer + Ink + Auto-Layout, etc.)
        stays fully reachable without resizing the window."""
        if not (0 <= step_index < self._step_count):
            return

        inner = QWidget()
        inner.setStyleSheet(f"background: {COLORS['base']};")
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(_s(8), _s(8), _s(8), _s(8))
        lay.setSpacing(_s(8))
        # A panel that asks to expand vertically (e.g. the Plan & Run
        # finalize form, which carries its own inner scroll area) fills
        # the full column height; in that case we skip the trailing
        # stretch so it isn't pinned to the top with empty space below.
        any_expand = False
        for panel in panels:
            if panel is None:
                continue
            panel.setParent(inner)
            expands = (panel.sizePolicy().verticalPolicy()
                       == QSizePolicy.Policy.Expanding)
            lay.addWidget(panel, 1 if expands else 0)
            panel.show()
            any_expand = any_expand or expands
        if not any_expand:
            lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        # v7.6.0: everything must fit the column width — vertical scroll
        # only, horizontal never. Tool panels are width-capped so nothing
        # clips; the inner widget tracks the viewport width.
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet(
            f"QScrollArea {{ background: {COLORS['base']}; border: none; }}"
        )
        scroll.setWidget(inner)

        old = self._tools_pages.get(step_index)
        if old is not None:
            idx = self._tools_stack.indexOf(old)
            if idx >= 0:
                self._tools_stack.removeWidget(old)
                old.setParent(None)
        self._tools_stack.insertWidget(step_index, scroll)
        self._tools_pages[step_index] = scroll
        if step_index == self._tools_stack.currentIndex():
            self._tools_stack.setCurrentIndex(step_index)

    def set_active_step(self, step_index: int) -> None:
        if not (0 <= step_index < self._step_count):
            return
        self._tools_stack.setCurrentIndex(step_index)

    # ── Legacy stubs (kept so existing wiring compiles) ──────────

    def set_hardware_config(self, config: Any) -> None:
        """No-op in v7.5.3+. Hardware lives in its own Printing
        Mode sub-page."""
        return None

    def set_step1_objects_panel(self, panel: QWidget | None) -> None:
        """Legacy v7.5.1 hook — superseded by ``set_step_tools``."""
        return None

    def set_step1_auto_layout_panel(self, panel: QWidget | None) -> None:
        """Legacy v7.5.2 hook — superseded by ``set_step_tools``."""
        return None
