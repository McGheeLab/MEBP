"""
invalidation_banner.py — Cross-page "hardware changed" banner (v7.4.0-b).

When a user changes HardwareConfig on the Hardware Setup page, downstream
pages (Print Setup, Calibration) may be showing stale derived data
(ink combos, plate geometry, etc.). MainWindow emits hw_config_invalidated
when relevant fields change; subscribing pages mount an InvalidationBanner
that surfaces the change and offers a one-click refresh.

Usage::

    banner = InvalidationBanner(
        message="Hardware changed — refresh to re-apply.",
        on_refresh=lambda: page.reload_from_hardware()
    )
    page_layout.insertWidget(0, banner)
    banner.hide()

    # In MainWindow when HW changes:
    banner.show()
"""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf, sp


class InvalidationBanner(QFrame):
    """A thin "click to refresh" strip mounted at the top of a page.

    Composes the same yellow-tinted styling as
    :class:`gui.widgets.components.LoadingBanner` but is action-oriented:
    a label on the left and a primary action button on the right.
    """

    refresh_clicked = Signal()

    def __init__(self,
                 message: str = "Hardware changed — refresh to re-apply.",
                 button_label: str = "Refresh",
                 on_refresh: Optional[Callable[[], None]] = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("invalidationBanner")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(s(28))
        self.setStyleSheet(
            f"QFrame#invalidationBanner {{"
            f"  background-color: rgba(249, 226, 175, 30);"
            f"  border-bottom: 1px solid {COLORS['yellow']};"
            f"}}"
        )

        lay = QHBoxLayout(self)
        lay.setContentsMargins(s(12), 0, s(12), 0)
        lay.setSpacing(s(8))

        self._icon = QLabel("⚠")
        self._icon.setStyleSheet(
            f"color: {COLORS['yellow']}; font-size: {sf(12)}pt; font-weight: 700;")
        lay.addWidget(self._icon)

        self._msg = QLabel(message)
        self._msg.setStyleSheet(f"color: {COLORS['text']}; font-size: {sf(10)}pt;")
        lay.addWidget(self._msg)
        lay.addStretch(1)

        self._btn = QPushButton(button_label)
        self._btn.setObjectName("accentBtn")
        self._btn.setCursor(Qt.PointingHandCursor)
        self._btn.setMaximumHeight(s(22))
        self._btn.clicked.connect(self._on_click)
        lay.addWidget(self._btn)

        if on_refresh is not None:
            self.refresh_clicked.connect(on_refresh)

    def set_message(self, message: str):
        self._msg.setText(message)

    def _on_click(self):
        self.refresh_clicked.emit()
        # Hide ourselves once the user acknowledges; subscribers can re-show
        # us when a new invalidation arrives.
        self.hide()
