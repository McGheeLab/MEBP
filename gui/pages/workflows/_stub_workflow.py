"""_stub_workflow.py — placeholder page for not-yet-built workflows.

Used by the four workflow tiles that exist in the picker but haven't
been implemented yet (Cell Targeting & Removal, Cell Labeling, Quick
Print, Immuno). Shows a "coming soon" message and a Back button.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy,
)

from gui.styles import COLORS
from gui.scaling import s, sf


class StubWorkflowPage(QWidget):
    """Placeholder for a workflow that hasn't been built yet.

    Signals:
        back_requested: User clicked the Back button.
    """

    back_requested = Signal()

    def __init__(self, title: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._title = title

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(16), s(12), s(16), s(16))
        outer.setSpacing(s(12))

        # Header row: Back + title
        header = QHBoxLayout()
        header.setSpacing(s(8))
        back_btn = QPushButton("← Back to Workflows")
        back_btn.setCursor(Qt.PointingHandCursor)
        back_btn.clicked.connect(self.back_requested.emit)
        header.addWidget(back_btn)

        title_label = QLabel(title)
        title_label.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(14)}pt;"
            f"font-weight: 600;"
        )
        header.addWidget(title_label)
        header.addStretch(1)
        outer.addLayout(header)

        # Body: centered "coming soon" message
        body = QLabel(f"{title}\n\nComing soon.")
        body.setAlignment(Qt.AlignCenter)
        body.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(13)}pt;"
        )
        body.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        outer.addWidget(body, stretch=1)
