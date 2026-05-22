"""
mode_page.py — Base class for mode pages with sub-page navigation.

v7.4.2: rebuilt as a top horizontal tab bar (was a narrow right-side
icon column). Each tab is an icon + label; tabs are wider and visually
read like a tab strip so new users notice them.

Layout:
    ┌──────────────────────────────────────────────────┐
    │  [icon]   [icon]   [icon]   [icon]   [icon]      │
    │   Dev      Identity  Plate   Pumps    ...        │
    │ ──────────────────────────────────────────────── │
    │                                                  │
    │   Active sub-page content                        │
    │                                                  │
    └──────────────────────────────────────────────────┘

``add_sub_page(icon, title, widget)`` accepts either an icon factory
name (e.g. ``"settings"``) or an emoji (legacy). Icon-name path renders
the solid white SVG; emoji path falls back to text rendering for
backwards compatibility with un-migrated mode pages.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QSize, Signal
from PySide6.QtGui import QFont, QIcon
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QSizePolicy, QStackedWidget, QToolButton,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

logger = logging.getLogger(__name__)


# Tab metrics
_TAB_HEIGHT = 60          # base px (scaled)
_TAB_MIN_WIDTH = 88       # base px (scaled)
_TAB_ICON_PX = 22         # icon size inside the tab
_TABBAR_PAD = 6           # outer padding around the tab strip


class ModePage(QWidget):
    """Base class for mode pages with a top horizontal tab bar.

    Subclasses call ``add_sub_page(icon, title, widget)`` to register
    each sub-page. The mode page handles:
      - Top tab bar with icon + label per sub-page
      - Active-tab highlight (accent underline + brighter foreground)
      - Delegating ``on_status_update`` / ``set_hardware_config`` /
        ``get_context_widget`` to the active sub-page
    """

    sub_page_changed = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._sub_pages: list[QWidget] = []
        self._sub_buttons: list[QPushButton] = []
        self._active_index: int = 0
        self._context_widgets: list[QWidget | None] = []

        # Outer vertical layout: [tab bar] + [content stack]
        self._main_layout = QVBoxLayout(self)
        self._main_layout.setSpacing(0)
        self._main_layout.setContentsMargins(0, 0, 0, 0)

        self._tabbar = QFrame()
        self._tabbar.setObjectName("modeTabbar")
        self._tabbar.setFrameShape(QFrame.NoFrame)
        self._tabbar.setStyleSheet(f"""
            #modeTabbar {{
                background-color: {COLORS['mantle']};
                border-bottom: 1px solid {COLORS['surface1']};
            }}
        """)
        self._tabbar_layout = QHBoxLayout(self._tabbar)
        self._tabbar_layout.setSpacing(s(2))
        self._tabbar_layout.setContentsMargins(
            s(_TABBAR_PAD), s(_TABBAR_PAD), s(_TABBAR_PAD), 0)
        self._tabbar_layout.setAlignment(Qt.AlignLeft)
        self._main_layout.addWidget(self._tabbar)

        self._sub_stack = QStackedWidget()
        self._sub_stack.setObjectName("modeSubStack")
        self._main_layout.addWidget(self._sub_stack, 1)

    # ── Sub-page management ──────────────────────────────────────

    def add_sub_page(self, icon, title: str, widget: QWidget):
        """Register a sub-page with a tab in the top bar.

        Args:
            icon: Either an icon name from ``gui.widgets.icons`` (e.g.
                  ``"settings"``) or an emoji / glyph string for legacy
                  callers that haven't migrated yet.
            title: Tab label text (also used as tooltip).
            widget: The sub-page widget to display.
        """
        index = len(self._sub_pages)
        self._sub_pages.append(widget)
        self._sub_stack.addWidget(widget)

        ctx = None
        if hasattr(widget, 'get_context_widget'):
            ctx = widget.get_context_widget()
        self._context_widgets.append(ctx)

        btn = self._make_tab_button(icon, title, index)
        btn.clicked.connect(lambda checked=False, idx=index: self.switch_to(idx))
        self._sub_buttons.append(btn)
        self._tabbar_layout.addWidget(btn)

        if index == 0:
            self._sub_stack.setCurrentIndex(0)
            self._active_index = 0
            self._apply_tab_style(btn, active=True)

    def _make_tab_button(self, icon, title: str, index: int) -> QToolButton:
        """Build one tab — icon stacked above label via QToolButton."""
        btn = QToolButton()
        btn.setObjectName(f"modeTabBtn_{index}")
        # Escape ampersands so Qt doesn't interpret them as mnemonics.
        btn.setText(title.replace("&", "&&"))
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip(title)
        btn.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        btn.setMinimumHeight(s(_TAB_HEIGHT))
        btn.setMinimumWidth(s(_TAB_MIN_WIDTH))
        # Stack icon above label — QToolButton supports this natively.
        btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        q_icon = self._resolve_icon(icon)
        if q_icon is not None and not q_icon.isNull():
            btn.setIcon(q_icon)
            btn.setIconSize(QSize(s(_TAB_ICON_PX), s(_TAB_ICON_PX)))
        else:
            # Legacy emoji path — render emoji as the icon-line so the
            # label still sits underneath via the same layout slot.
            btn.setText(f"{icon}\n{title}")
        self._apply_tab_style(btn, active=False)
        return btn

    @staticmethod
    def _resolve_icon(icon) -> QIcon | None:
        """Accept QIcon directly, an icon-factory name, or an emoji.

        Returns a QIcon when ``icon`` is recognised as a factory name;
        otherwise None (caller falls back to text rendering).
        """
        if isinstance(icon, QIcon):
            return icon
        if not isinstance(icon, str):
            return None
        # Try the factory — only ASCII names resolve.
        try:
            from gui.widgets.icons import _ICONS, icon as make_icon
            if icon in _ICONS:
                return make_icon(icon, color="#ffffff", px=s(_TAB_ICON_PX))
        except Exception:
            return None
        return None

    def switch_to(self, index: int):
        """Switch to the sub-page at the given index."""
        if index < 0 or index >= len(self._sub_pages):
            return
        if index == self._active_index:
            return

        self._active_index = index
        self._sub_stack.setCurrentIndex(index)

        for i, btn in enumerate(self._sub_buttons):
            self._apply_tab_style(btn, active=(i == index))

        self.sub_page_changed.emit(index)
        logger.debug(f"Mode sub-page switched to index {index}")

    def get_active_sub_page(self) -> QWidget | None:
        if 0 <= self._active_index < len(self._sub_pages):
            return self._sub_pages[self._active_index]
        return None

    def get_active_index(self) -> int:
        return self._active_index

    # ── Delegation ───────────────────────────────────────────────

    def on_status_update(self):
        page = self.get_active_sub_page()
        if page and hasattr(page, 'on_status_update'):
            page.on_status_update()

    def set_hardware_config(self, config):
        for page in self._sub_pages:
            if hasattr(page, 'set_hardware_config'):
                try:
                    page.set_hardware_config(config)
                except Exception as e:
                    logger.error(
                        f"HW config propagation failed for "
                        f"{page.__class__.__name__}: {e}")

    def get_context_widget(self) -> QWidget | None:
        if 0 <= self._active_index < len(self._context_widgets):
            return self._context_widgets[self._active_index]
        return None

    def get_page_title(self) -> str:
        return "Mode"

    def get_sub_page_title(self) -> str:
        page = self.get_active_sub_page()
        if page and hasattr(page, 'get_page_title'):
            return page.get_page_title()
        return self.get_page_title()

    # ── Tab styling ──────────────────────────────────────────────

    def _apply_tab_style(self, btn: QToolButton, active: bool):
        """Apply active/inactive style to a tab. Active = mauve accent
        bottom border, brighter text. Inactive = subtle, hover lifts.
        """
        common = (
            f"padding: {s(6)}px {s(12)}px {s(6)}px {s(12)}px;"
            f"font-size: {sf(9.5)}pt;"
            f"font-weight: 500;"
        )
        if active:
            btn.setStyleSheet(
                f"QToolButton {{"
                f"  background-color: {COLORS['base']};"
                f"  border: none;"
                f"  border-bottom: {s(3)}px solid {COLORS['mauve']};"
                f"  border-top-left-radius: {s(8)}px;"
                f"  border-top-right-radius: {s(8)}px;"
                f"  color: {COLORS['text']};"
                f"  {common}"
                f"}}"
            )
        else:
            btn.setStyleSheet(
                f"QToolButton {{"
                f"  background-color: transparent;"
                f"  border: none;"
                f"  border-bottom: {s(3)}px solid transparent;"
                f"  border-top-left-radius: {s(8)}px;"
                f"  border-top-right-radius: {s(8)}px;"
                f"  color: {COLORS['subtext0']};"
                f"  {common}"
                f"}}"
                f"QToolButton:hover {{"
                f"  background-color: {COLORS['surface0']};"
                f"  color: {COLORS['text']};"
                f"}}"
            )
