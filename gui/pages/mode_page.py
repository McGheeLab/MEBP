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
    QFrame, QHBoxLayout, QSizePolicy, QSplitter, QStackedWidget,
    QToolButton, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

logger = logging.getLogger(__name__)


# Tab metrics
_TAB_HEIGHT = 60          # base px (scaled) — horizontal mode
_TAB_MIN_WIDTH = 88       # base px (scaled) — horizontal mode
_TAB_ICON_PX = 22         # icon size inside the tab
_TABBAR_PAD = 6           # outer padding around the tab strip
# v7.5.2: vertical icon-strip metrics
_VTAB_SIZE = 48           # base px square button (icon-only)
_VTABBAR_WIDTH = 56       # full strip width (button + a little padding)


class ModePage(QWidget):
    """Base class for mode pages with sub-page navigation.

    Two layouts are supported (v7.5.2):

    * ``tab_orientation="horizontal"`` (default) — top tab bar with
      icon + label per sub-page; active tab gets a mauve underline.
    * ``tab_orientation="vertical"`` — left-edge icon strip
      (~56 px wide). Buttons are icon-only with a tooltip; active tab
      gets a mauve accent strip along its right edge. Saves the top
      ~60 px for the sub-page's own nav.

    Subclasses call ``add_sub_page(icon, title, widget)`` to register
    each sub-page. The mode page handles:
      - The tab strip (horizontal or vertical)
      - Active-tab highlight + brighter foreground
      - Delegating ``on_status_update`` / ``set_hardware_config`` /
        ``get_context_widget`` to the active sub-page
    """

    sub_page_changed = Signal(int)

    def __init__(self, parent=None, tab_orientation: str = "horizontal"):
        super().__init__(parent)

        self._sub_pages: list[QWidget] = []
        self._sub_buttons: list[QToolButton] = []
        self._active_index: int = 0
        self._context_widgets: list[QWidget | None] = []
        self._tab_orientation = tab_orientation

        if tab_orientation == "vertical":
            # Outer horizontal layout: [icon strip] | [content stack]
            self._main_layout = QHBoxLayout(self)
            self._main_layout.setSpacing(0)
            self._main_layout.setContentsMargins(0, 0, 0, 0)

            self._tabbar = QFrame()
            self._tabbar.setObjectName("modeTabbar")
            self._tabbar.setFrameShape(QFrame.NoFrame)
            self._tabbar.setFixedWidth(s(_VTABBAR_WIDTH))
            self._tabbar.setStyleSheet(f"""
                #modeTabbar {{
                    background-color: {COLORS['mantle']};
                    border-right: 1px solid {COLORS['surface1']};
                }}
            """)
            self._tabbar_layout = QVBoxLayout(self._tabbar)
            self._tabbar_layout.setSpacing(s(4))
            self._tabbar_layout.setContentsMargins(
                s(4), s(_TABBAR_PAD), s(4), s(_TABBAR_PAD))
            self._tabbar_layout.setAlignment(Qt.AlignTop)
            self._main_layout.addWidget(self._tabbar)

            # v7.5.6: the active sub-page's contributed left-nav widget
            # (e.g. the wizard step strip — "sub-sub-page nav") lives in
            # its OWN column to the RIGHT of the mode-icon tabbar, not
            # stacked below the icons. ``_sub_nav_stack`` swaps per
            # active sub-page; the column hides when the active
            # sub-page contributes nothing.
            self._sub_nav_stack = QStackedWidget()
            self._sub_nav_stack.setObjectName("modeSubNavStack")
            self._sub_nav_stack.setStyleSheet(
                f"#modeSubNavStack {{"
                f"  background-color: {COLORS['mantle']};"
                f"  border-right: 1px solid {COLORS['surface0']};"
                f"}}"
            )
            self._sub_nav_stack.setVisible(False)
            self._main_layout.addWidget(self._sub_nav_stack, 0)

            # v7.5.5: embedded left-context stack BETWEEN the tabbar
            # (nav stack) and the body. Each sub-page's
            # ``get_context_widget()`` is mounted here so the
            # left-context content sits to the RIGHT of the nav
            # stack instead of being mounted by app.py outside the
            # mode page widget.
            self._embedded_ctx_stack = QStackedWidget()
            self._embedded_ctx_stack.setObjectName("modeCtxStack")
            self._embedded_ctx_stack.setStyleSheet(
                f"#modeCtxStack {{"
                f"  background: {COLORS['mantle']};"
                f"  border-right: 1px solid {COLORS['surface0']};"
                f"}}"
            )
            self._embedded_ctx_stack.setVisible(False)
            self._embedded_ctx_stack.setMinimumWidth(0)

            self._sub_stack = QStackedWidget()
            self._sub_stack.setObjectName("modeSubStack")

            # v7.6.0: the Tools context + body live in a draggable
            # QSplitter so the user can resize (or drag-collapse) the
            # Tools column. The fixed nav columns (tabbar, sub_nav)
            # stay OUTSIDE the splitter so they never resize.
            self._body_splitter = QSplitter(Qt.Orientation.Horizontal)
            self._body_splitter.setObjectName("modeBodySplitter")
            self._body_splitter.setChildrenCollapsible(True)
            self._body_splitter.setHandleWidth(s(4))
            self._body_splitter.addWidget(self._embedded_ctx_stack)
            self._body_splitter.addWidget(self._sub_stack)
            self._body_splitter.setStretchFactor(0, 0)  # ctx: fixed-ish
            self._body_splitter.setStretchFactor(1, 1)   # body: stretch
            # v7.6.0: comfortable default Tools width so the
            # single-column option panels fit without scrolling.
            self._embedded_ctx_stack.setMinimumWidth(s(300))
            self._body_splitter.setSizes([s(380), s(1200)])
            self._main_layout.addWidget(self._body_splitter, 1)
        else:
            # Horizontal (legacy default)
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

        # v7.5.6: collect the sub-page's optional left-nav widget
        # (e.g. wizard step strip) into a stacked widget that forms
        # its OWN column to the right of the mode-icon tabbar.
        # ``None`` becomes an empty placeholder.
        if (self._tab_orientation == "vertical"
                and hasattr(self, "_sub_nav_stack")):
            extra_nav = None
            if hasattr(widget, "get_left_nav_widget"):
                try:
                    extra_nav = widget.get_left_nav_widget()
                except Exception as exc:
                    logger.debug(f"get_left_nav_widget on {widget}: {exc}")
            if extra_nav is None:
                extra_nav = QWidget()
                extra_nav.setStyleSheet("background: transparent;")
            self._sub_nav_stack.addWidget(extra_nav)

        # v7.5.5: collect the sub-page's left-context widget (the
        # Tools panel) into the embedded context stack so it sits to
        # the RIGHT of the nav stack, INSIDE the mode page.
        if (self._tab_orientation == "vertical"
                and hasattr(self, "_embedded_ctx_stack")):
            if ctx is None:
                ctx_placeholder = QWidget()
                ctx_placeholder.setStyleSheet("background: transparent;")
                self._embedded_ctx_stack.addWidget(ctx_placeholder)
            else:
                self._embedded_ctx_stack.addWidget(ctx)
            self._refresh_embedded_ctx_visibility()

        if index == 0:
            self._sub_stack.setCurrentIndex(0)
            self._active_index = 0
            self._apply_tab_style(btn, active=True)
            if hasattr(self, "_sub_nav_stack"):
                self._sub_nav_stack.setCurrentIndex(0)
                self._refresh_sub_nav_visibility()
            if hasattr(self, "_embedded_ctx_stack"):
                self._embedded_ctx_stack.setCurrentIndex(0)
                self._refresh_embedded_ctx_visibility()

    def _refresh_sub_nav_visibility(self) -> None:
        """Show the sub-nav column iff the active sub-page contributed
        a non-trivial left-nav widget (e.g. the wizard step strip)."""
        if not hasattr(self, "_sub_nav_stack"):
            return
        active = self._sub_nav_stack.currentWidget()
        # An empty placeholder is a bare QWidget with no children.
        has_content = bool(active and active.children())
        self._sub_nav_stack.setVisible(has_content)

    def _refresh_embedded_ctx_visibility(self) -> None:
        """Show the embedded context stack iff the active sub-page's
        context widget has real content."""
        if not hasattr(self, "_embedded_ctx_stack"):
            return
        active = self._embedded_ctx_stack.currentWidget()
        # Placeholders are bare QWidget with no children; real
        # context panels always have at least one child (a layout
        # or a stack).
        has_content = bool(active and active.children())
        self._embedded_ctx_stack.setVisible(has_content)

    def _make_tab_button(self, icon, title: str, index: int) -> QToolButton:
        """Build one tab — icon + label (horizontal) or icon-only
        (vertical icon strip)."""
        btn = QToolButton()
        btn.setObjectName(f"modeTabBtn_{index}")
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip(title)
        q_icon = self._resolve_icon(icon)

        if self._tab_orientation == "vertical":
            # v7.5.2: icon-only square button, tooltip carries the name.
            btn.setText("")
            btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            btn.setFixedSize(s(_VTAB_SIZE), s(_VTAB_SIZE))
            btn.setToolButtonStyle(Qt.ToolButtonIconOnly)
            if q_icon is not None and not q_icon.isNull():
                btn.setIcon(q_icon)
                btn.setIconSize(QSize(s(_TAB_ICON_PX), s(_TAB_ICON_PX)))
            else:
                # Legacy emoji fallback — show the emoji as the
                # button text since we don't have an icon.
                btn.setText(icon if isinstance(icon, str) else "")
        else:
            # Horizontal (legacy)
            btn.setText(title.replace("&", "&&"))
            btn.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
            btn.setMinimumHeight(s(_TAB_HEIGHT))
            btn.setMinimumWidth(s(_TAB_MIN_WIDTH))
            btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            if q_icon is not None and not q_icon.isNull():
                btn.setIcon(q_icon)
                btn.setIconSize(QSize(s(_TAB_ICON_PX), s(_TAB_ICON_PX)))
            else:
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

        # v7.5.6: swap the active sub-page's contributed left-nav
        # widget (its own column to the right of the mode icons).
        if hasattr(self, "_sub_nav_stack"):
            self._sub_nav_stack.setCurrentIndex(index)
            self._refresh_sub_nav_visibility()
        # v7.5.5: swap the embedded left context (Tools) widget too.
        if hasattr(self, "_embedded_ctx_stack"):
            self._embedded_ctx_stack.setCurrentIndex(index)
            self._refresh_embedded_ctx_visibility()

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
        """Apply active/inactive style to a tab.

        Horizontal: active = mauve underline.
        Vertical (v7.5.2): active = mauve right-edge accent strip.
        """
        if self._tab_orientation == "vertical":
            if active:
                btn.setStyleSheet(
                    f"QToolButton {{"
                    f"  background-color: {COLORS['base']};"
                    f"  border: none;"
                    f"  border-right: {s(3)}px solid {COLORS['mauve']};"
                    f"  border-top-left-radius: {s(8)}px;"
                    f"  border-bottom-left-radius: {s(8)}px;"
                    f"  color: {COLORS['text']};"
                    f"}}"
                )
            else:
                btn.setStyleSheet(
                    f"QToolButton {{"
                    f"  background-color: transparent;"
                    f"  border: none;"
                    f"  border-right: {s(3)}px solid transparent;"
                    f"  border-top-left-radius: {s(8)}px;"
                    f"  border-bottom-left-radius: {s(8)}px;"
                    f"  color: {COLORS['subtext0']};"
                    f"}}"
                    f"QToolButton:hover {{"
                    f"  background-color: {COLORS['surface0']};"
                    f"  color: {COLORS['text']};"
                    f"}}"
                )
            return

        # Horizontal (legacy)
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
