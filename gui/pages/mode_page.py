"""
mode_page.py — Base class for mode pages with right-side sub-page navigation.

v7.3.3: Mode pages contain multiple sub-pages and display a narrow icon
column on the right side of the page for switching between them.

Layout:
    ┌──────────────────────────────────┬──┐
    │                                  │🖨️│
    │   Active sub-page content        │📈│
    │                                  │📋│
    │                                  │🧰│
    └──────────────────────────────────┴──┘
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QStackedWidget,
    QPushButton, QFrame, QSizePolicy,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QIcon, QPixmap, QPainter, QColor

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size

logger = logging.getLogger(__name__)

# Right sidebar dimensions
_SIDEBAR_WIDTH = s(48)
_BUTTON_SIZE = s(40)
_BUTTON_SPACING = s(4)


def _make_sub_icon(text: str, size: int = 24, color: str = COLORS["subtext0"]) -> QIcon:
    """Create a QIcon from a text character for the sub-page button."""
    pixmap = QPixmap(s(size), s(size))
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setPen(QColor(color))
    painter.setFont(QFont("Segoe UI Emoji", scaled_font_size(int(size * 0.55))))
    painter.drawText(pixmap.rect(), Qt.AlignCenter, text)
    painter.end()
    return QIcon(pixmap)


class ModePage(QWidget):
    """Base class for mode pages with right-side sub-page navigation.

    Subclasses should call add_sub_page() in __init__ to register their
    sub-pages. The mode page handles:
      - Right sidebar icon buttons for switching sub-pages
      - Delegating on_status_update() to the active sub-page
      - Delegating set_hardware_config() to ALL sub-pages
      - Delegating get_context_widget() to the active sub-page
    """

    sub_page_changed = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._sub_pages: list[QWidget] = []
        self._sub_buttons: list[QPushButton] = []
        self._active_index: int = 0
        self._context_widgets: list[QWidget | None] = []

        # Main layout: [content stack] + [right sidebar]
        self._main_layout = QHBoxLayout(self)
        self._main_layout.setSpacing(0)
        self._main_layout.setContentsMargins(0, 0, 0, 0)

        # Sub-page content stack
        self._sub_stack = QStackedWidget()
        self._sub_stack.setObjectName("modeSubStack")
        self._main_layout.addWidget(self._sub_stack, 1)

        # Right sidebar frame
        self._sidebar = QFrame()
        self._sidebar.setObjectName("modeRightSidebar")
        self._sidebar.setFixedWidth(_SIDEBAR_WIDTH)
        self._sidebar.setFrameShape(QFrame.NoFrame)
        self._sidebar.setStyleSheet(f"""
            #modeRightSidebar {{
                background-color: {COLORS['mantle']};
                border-left: 1px solid {COLORS['surface1']};
            }}
        """)

        self._sidebar_layout = QVBoxLayout(self._sidebar)
        self._sidebar_layout.setSpacing(_BUTTON_SPACING)
        self._sidebar_layout.setContentsMargins(4, 8, 4, 8)
        self._sidebar_layout.setAlignment(Qt.AlignTop)

        self._main_layout.addWidget(self._sidebar)

    # ── Sub-page management ──────────────────────────────────────

    def add_sub_page(self, icon_text: str, title: str, widget: QWidget):
        """Register a sub-page with an icon button in the right sidebar.

        Args:
            icon_text: Emoji or character for the button.
            title: Tooltip text for the button.
            widget: The sub-page widget to display.
        """
        index = len(self._sub_pages)
        self._sub_pages.append(widget)
        self._sub_stack.addWidget(widget)

        # Collect context widget if available
        ctx = None
        if hasattr(widget, 'get_context_widget'):
            ctx = widget.get_context_widget()
        self._context_widgets.append(ctx)

        # Create sidebar button
        btn = QPushButton(icon_text)
        btn.setFixedSize(_BUTTON_SIZE, _BUTTON_SIZE)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip(title)
        btn.setFont(QFont("Segoe UI Emoji", scaled_font_size(14)))
        btn.setObjectName(f"modeSubBtn_{index}")

        # Style
        self._apply_button_style(btn, active=index == 0)

        btn.clicked.connect(lambda checked=False, idx=index: self.switch_to(idx))
        self._sub_buttons.append(btn)
        self._sidebar_layout.addWidget(btn)

        # If first sub-page, make it active
        if index == 0:
            self._sub_stack.setCurrentIndex(0)
            self._active_index = 0

    def switch_to(self, index: int):
        """Switch to the sub-page at the given index."""
        if index < 0 or index >= len(self._sub_pages):
            return
        if index == self._active_index:
            return

        self._active_index = index
        self._sub_stack.setCurrentIndex(index)

        # Update button styles
        for i, btn in enumerate(self._sub_buttons):
            self._apply_button_style(btn, active=(i == index))

        self.sub_page_changed.emit(index)
        logger.debug(f"Mode sub-page switched to index {index}")

    def get_active_sub_page(self) -> QWidget | None:
        """Return the currently active sub-page widget."""
        if 0 <= self._active_index < len(self._sub_pages):
            return self._sub_pages[self._active_index]
        return None

    def get_active_index(self) -> int:
        """Return the index of the currently active sub-page."""
        return self._active_index

    # ── Delegation methods ───────────────────────────────────────

    def on_status_update(self):
        """Delegate status updates to the active sub-page."""
        page = self.get_active_sub_page()
        if page and hasattr(page, 'on_status_update'):
            page.on_status_update()

    def set_hardware_config(self, config):
        """Propagate hardware config to ALL sub-pages."""
        for page in self._sub_pages:
            if hasattr(page, 'set_hardware_config'):
                try:
                    page.set_hardware_config(config)
                except Exception as e:
                    logger.error(
                        f"HW config propagation failed for "
                        f"{page.__class__.__name__}: {e}")

    def get_context_widget(self) -> QWidget | None:
        """Return the context widget for the active sub-page."""
        if 0 <= self._active_index < len(self._context_widgets):
            return self._context_widgets[self._active_index]
        return None

    def get_page_title(self) -> str:
        """Return the page title — subclasses should override."""
        return "Mode"

    def get_sub_page_title(self) -> str:
        """Return the title of the active sub-page."""
        page = self.get_active_sub_page()
        if page and hasattr(page, 'get_page_title'):
            return page.get_page_title()
        return self.get_page_title()

    # ── Button styling ───────────────────────────────────────────

    def _apply_button_style(self, btn: QPushButton, active: bool):
        """Apply active/inactive style to a sub-page button."""
        if active:
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {COLORS['surface0']};
                    border: 2px solid {COLORS['mauve']};
                    border-radius: 6px;
                    color: {COLORS['text']};
                    font-size: 16px;
                }}
                QPushButton:hover {{
                    background-color: {COLORS['surface1']};
                }}
            """)
        else:
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {COLORS['mantle']};
                    border: 1px solid {COLORS['surface1']};
                    border-radius: 6px;
                    color: {COLORS['subtext0']};
                    font-size: 16px;
                }}
                QPushButton:hover {{
                    background-color: {COLORS['surface0']};
                    border: 1px solid {COLORS['mauve']};
                    color: {COLORS['text']};
                }}
            """)
