"""cell_removal_confirm_dialog.py — the pre-run confirmation for cell removal.

v7.9 (post-audit). ``_on_start`` used to append every advisory to the status label
**on the same line as** "Running N cell removals…", immediately before starting
the executor thread — so every warning arrived AFTER the decision, including the
one whose failure mode is total (an unmeasured bore mount offset doses every cell
100-500 µm off-target and collects none). There was no summary and no
acknowledgement for an operation that lowers a needle to ~0.1 mm above glass and
doses live cells dozens of times.

Reuses Quick Print's narrative shape ("This run will: …") but as a small
``QDialog`` rather than a ``QMessageBox``, because this run needs a rendered
checklist and an acknowledgement — and ``ReadinessList`` makes that nearly free.

Default button is **Cancel**, matching every other confirm in this app, and there
is deliberately **no "don't ask again"**: a hands-free run over live cells at
0.1 mm earns a click every time.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QLabel, QScrollArea, QVBoxLayout,
    QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card, ReadinessList

logger = logging.getLogger(__name__)


class CellRemovalConfirmDialog(QDialog):
    """Confirm a cell-removal run. ``exec()`` returns Accepted only if the
    operator also ticked the acknowledgement."""

    def __init__(self, parent=None, *, n_targets: int = 0,
                 steps: list[str] | None = None, readiness=None,
                 clearance_mm: float | None = None,
                 bracket_note: str = ""):
        super().__init__(parent)
        self.setWindowTitle("Start cell removal?")
        self.setMinimumWidth(s(600))

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(10), s(10), s(10), s(10))
        outer.setSpacing(s(8))

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(8))

        # 1. What will happen, per cell, in the operator's own terms.
        card = Card(f"This run will, for each of {n_targets} "
                    f"cell{'s' if n_targets != 1 else ''}:")
        for line in (steps or []):
            lbl = QLabel(f"•  {line}")
            lbl.setWordWrap(True)
            lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-size: {sf(9)}pt;")
            card.add_widget(lbl)
        if bracket_note:
            note = QLabel(bracket_note)
            note.setWordWrap(True)
            note.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            card.add_widget(note)
        lay.addWidget(card)

        # 2. The two numbers that make this consequential, on their own line.
        head = []
        if clearance_mm is not None:
            head.append(f"The needle descends to <b>{clearance_mm:.3f} mm</b> "
                        f"above the glass")
        if n_targets:
            head.append(f"and doses live cells <b>{n_targets}</b> "
                        f"time{'s' if n_targets != 1 else ''}")
        if head:
            stake = QLabel(", ".join(head) + ".")
            stake.setWordWrap(True)
            stake.setTextFormat(Qt.TextFormat.RichText)
            stake.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(10)}pt;")
            lay.addWidget(stake)

        # 3. The rendered checklist — warnings and info only (a blocker means
        #    Start was gated and this dialog never opened).
        if readiness is not None:
            try:
                from SupportClasses.PrintReadiness import BLOCK, INFO, WARN
                checks = Card("Worth knowing before you start")
                lst = ReadinessList()
                lst.set_readiness(readiness, states=(BLOCK, WARN, INFO))
                checks.add_widget(lst)
                lay.addWidget(checks)
            except Exception:
                logger.debug("could not render the readiness list", exc_info=True)

        lay.addStretch(1)
        scroll.setWidget(body)
        outer.addWidget(scroll, stretch=1)

        # 4. One acknowledgement, gating Start. No "don't ask again".
        self._ack = QCheckBox(
            "I have checked the plate, the needle assembly and the reagent wells.")
        self._ack.setStyleSheet(f"font-size: {sf(10)}pt;")
        self._ack.toggled.connect(self._sync)
        outer.addWidget(self._ack)

        self._buttons = QDialogButtonBox()
        self._start = self._buttons.addButton(
            "Start run", QDialogButtonBox.ButtonRole.AcceptRole)
        self._cancel = self._buttons.addButton(
            QDialogButtonBox.StandardButton.Cancel)
        self._cancel.setDefault(True)          # Cancel is the default action
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)
        outer.addWidget(self._buttons)
        self._sync()

    def _sync(self, *_):
        self._start.setEnabled(self._ack.isChecked())

    def acknowledged(self) -> bool:
        return self._ack.isChecked()
