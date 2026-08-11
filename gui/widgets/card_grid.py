"""Reflowing card grid + bulk-select bar — shared library-page mechanics.

Extracted from ``print_library.py`` in v7.12 so the plate library reuses the
column arithmetic rather than copying it. Only the *mechanics* are shared: card
contents, thumbnails, action sets and the item source stay with each page. A
``LibraryPageBase`` would need six injection points for two consumers, which is
a worse abstraction than the duplication — but the reflow is the one part that
silently rots when duplicated, so it lives here.

Zero domain knowledge: a card is any ``QWidget`` of the declared fixed width.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, scaled_font_size as _sf
from gui.styles import COLORS

#: Default library card width in base (pre-DPI) pixels.
CARD_W = 232


class CardGridView(QWidget):
    """A scrollable grid that reflows its fixed-width cards on resize."""

    def __init__(self, card_width: int = CARD_W, spacing: int = 10,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._card_width = card_width
        self._cards: list[QWidget] = []
        self._cols = 0

        # Debounced: a drag-resize fires resizeEvent continuously.
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(60)
        self._timer.timeout.connect(self._relayout)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QScrollArea.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._host = QWidget()
        self._grid = QGridLayout(self._host)
        self._grid.setContentsMargins(s(2), s(2), s(2), s(2))
        self._grid.setSpacing(s(spacing))
        self._grid.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self._scroll.setWidget(self._host)
        outer.addWidget(self._scroll, 1)

        self._empty = QLabel("")
        self._empty.setAlignment(Qt.AlignCenter)
        self._empty.setWordWrap(True)
        self._empty.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(10)}pt;")
        self._empty.setVisible(False)
        outer.addWidget(self._empty, 1)

    # ── API ───────────────────────────────────────────────────────

    def set_cards(self, cards: list[QWidget]) -> None:
        """Replace the grid contents. Previous cards are destroyed."""
        while self._grid.count():
            item = self._grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        self._cards = list(cards)
        for c in self._cards:
            c.setParent(self._host)
        self._cols = 0
        self._relayout(force=True)
        self._scroll.setVisible(bool(self._cards))
        self._empty.setVisible(not self._cards)

    def cards(self) -> list[QWidget]:
        return list(self._cards)

    def set_empty_text(self, text: str) -> None:
        self._empty.setText(text)

    def empty_label(self) -> QLabel:
        return self._empty

    def columns(self) -> int:
        return self._cols

    # ── Reflow ────────────────────────────────────────────────────

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._timer.start()

    def _relayout(self, force: bool = False) -> None:
        if not self._cards:
            return
        vw = self._scroll.viewport().width()
        step = s(self._card_width) + self._grid.spacing()
        cols = max(1, vw // step) if vw > 0 else 3
        if cols == self._cols and not force:
            return
        self._cols = cols
        # Detach only — the widgets (and their painted thumbnails) survive, so
        # a resize never rebuilds a card.
        while self._grid.count():
            self._grid.takeAt(0)
        for i, card in enumerate(self._cards):
            self._grid.addWidget(card, i // cols, i % cols)


class BulkSelectBar(QFrame):
    """Hidden until something is selected: '<n> selected · Clear · Delete'."""

    clear_requested = Signal()
    delete_requested = Signal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setVisible(False)
        self.setStyleSheet(
            f"background: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {s(6)}px;")
        row = QHBoxLayout(self)
        row.setContentsMargins(s(8), s(4), s(8), s(4))
        row.setSpacing(s(8))
        self._lbl = QLabel("0 selected")
        self._lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {_sf(9)}pt;")
        row.addWidget(self._lbl)
        row.addStretch(1)
        clear = QPushButton("Clear")
        clear.setCursor(Qt.PointingHandCursor)
        clear.clicked.connect(self.clear_requested)
        row.addWidget(clear)
        dele = QPushButton("Delete selected")
        dele.setCursor(Qt.PointingHandCursor)
        dele.setStyleSheet(
            f"QPushButton {{ color: {COLORS.get('red', '#f38ba8')}; }}")
        dele.clicked.connect(self.delete_requested)
        row.addWidget(dele)

    def set_count(self, n: int) -> None:
        self._lbl.setText(f"{n} selected")
        self.setVisible(n > 0)
