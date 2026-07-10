"""context_panel_host.py — the left context panel's pill-based view host.

v7.5.x: The left context box used to show a single per-page widget (usually the
jog panel), with visibility decided by one code path and content by another —
which desynced and made the jog panel "sometimes disappear" in workflows. This
host replaces that: it is a SINGLE, always-mounted widget owning a pill bar
(view picker) over a stacked set of views:

    (Jog)  (Custom)                                   [＋]
    ─────────────────────────────────────────────────────
    <the active view fills the area below>

  * **Jog** — the current page's native context widget (the existing
    ``StandardJogContextPanel``), reparented into a reused scroll wrapper. The
    pill is disabled when the active page has no native widget.
  * **Custom** — a single shared ``CustomContextPanel`` the operator composes
    from sections; the same instance everywhere, so the layout is uniform and
    its camera feed is a single subscriber.

``MainWindow`` decides visibility explicitly in one place
(``_refresh_left_context``); this host only manages *which* view shows and
forwards live ticks to the Custom view (the native jog panel is still ticked by
its owning page, so it must not be double-driven here).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup, QFrame, QHBoxLayout, QPushButton, QScrollArea,
    QStackedWidget, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf, sp
from gui.styles import COLORS
from gui.widgets.context_sections import SectionContext
from gui.widgets.custom_context_panel import CustomContextPanel

logger = logging.getLogger(__name__)

_VIEW_JOG = "jog"
_VIEW_CUSTOM = "custom"


class SegmentedPillBar(QWidget):
    """A row of mutually-exclusive pill buttons. Emits ``selected(name)``."""

    selected = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons: dict[str, QPushButton] = {}
        self._lay = QHBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._lay.setSpacing(s(4))
        self._group.buttonClicked.connect(self._on_clicked)

    def add_pill(self, name: str, label: str) -> None:
        btn = QPushButton(label)
        btn.setCheckable(True)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: {COLORS['surface0']};"
            f"  color: {COLORS['subtext0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(11)};"
            f"  padding: {sp(3)} {sp(12)};"
            f"  font-size: {sf(9)}pt; font-weight: 600;"
            f"}}"
            f"QPushButton:checked {{"
            f"  background-color: {COLORS['mauve']};"
            f"  color: {COLORS['base']};"
            f"  border-color: {COLORS['mauve']};"
            f"}}"
            f"QPushButton:disabled {{ color: {COLORS.get('overlay0', '#6c7086')}; }}"
        )
        self._group.addButton(btn)
        self._buttons[name] = btn
        # Insert before any trailing stretch the owner added externally.
        self._lay.addWidget(btn)
        btn.setProperty("_pill_name", name)

    def _on_clicked(self, btn: QPushButton) -> None:
        name = btn.property("_pill_name")
        if name:
            self.selected.emit(str(name))

    def set_selected(self, name: str) -> None:
        btn = self._buttons.get(name)
        if btn is not None and not btn.isChecked():
            btn.setChecked(True)

    def set_pill_enabled(self, name: str, enabled: bool) -> None:
        btn = self._buttons.get(name)
        if btn is not None:
            btn.setEnabled(enabled)

    def set_pill_label(self, name: str, text: str) -> None:
        btn = self._buttons.get(name)
        if btn is not None:
            btn.setText(text)


class ContextPanelHost(QWidget):
    """Pill-based host for the left context: Jog (native) + Custom views."""

    view_changed = Signal(str)

    def __init__(self, ctx: SectionContext, parent: QWidget | None = None):
        super().__init__(parent)
        self._ctx = ctx
        self._native_available = False
        # ``_requested_view`` is the user's persisted pill choice; the
        # ``_active_view`` is what's actually displayed. They differ only when
        # the requested "jog" view has no native widget available — then we show
        # Custom WITHOUT clobbering the request, so the jog preference is
        # honoured again the moment a native widget returns (fixes a startup
        # clobber where the saved "jog" was lost before any page mounted).
        self._requested_view = _VIEW_JOG
        self._active_view = _VIEW_CUSTOM
        self._native_wrappers: dict[int, QScrollArea] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(s(6), s(6), s(6), s(6))
        root.setSpacing(s(6))

        # ── Pill bar row ──
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(s(6))
        self._pills = SegmentedPillBar()
        self._pills.add_pill(_VIEW_JOG, "Jog")
        self._pills.add_pill(_VIEW_CUSTOM, "Custom")
        self._pills.selected.connect(self._on_pill_selected)
        top.addWidget(self._pills)
        top.addStretch(1)
        self._add_btn = QPushButton("＋")
        self._add_btn.setObjectName("flatBtn")
        self._add_btn.setToolTip("Add a section to the Custom panel")
        self._add_btn.setCursor(Qt.PointingHandCursor)
        self._add_btn.setFixedSize(s(26), s(26))
        self._add_btn.clicked.connect(self._on_add_clicked)
        top.addWidget(self._add_btn)
        root.addLayout(top)

        # ── Views ──
        self._views = QStackedWidget()
        root.addWidget(self._views, stretch=1)

        # index 0: native slot (its own stack of reused scroll wrappers)
        self._native_stack = QStackedWidget()
        self._native_placeholder = QWidget()
        self._native_stack.addWidget(self._native_placeholder)
        self._views.addWidget(self._native_stack)

        # index 1: shared custom panel
        self._custom = CustomContextPanel(ctx)
        self._views.addWidget(self._custom)

        self._apply_effective_view()

    # ── Native (Jog) slot ─────────────────────────────────────────

    def set_native_widget(self, widget: QWidget | None) -> None:
        """Mount the current page's context widget in the Jog slot (reusing a
        scroll wrapper per instance so reparenting is stable), or clear it."""
        if widget is None:
            self._native_stack.setCurrentWidget(self._native_placeholder)
            return
        key = id(widget)
        wrapper = self._native_wrappers.get(key)
        if wrapper is None:
            wrapper = QScrollArea()
            wrapper.setObjectName("contextScrollArea")
            wrapper.setWidgetResizable(True)
            wrapper.setFrameShape(QFrame.NoFrame)
            wrapper.setWidget(widget)
            self._native_stack.addWidget(wrapper)
            self._native_wrappers[key] = wrapper
        self._native_stack.setCurrentWidget(wrapper)

    def set_native_label(self, text: str) -> None:
        """Relabel the native ("Jog") pill for the current page."""
        self._pills.set_pill_label(_VIEW_JOG, text or "Jog")

    def set_native_available(self, available: bool) -> None:
        self._native_available = bool(available)
        self._pills.set_pill_enabled(_VIEW_JOG, self._native_available)
        # Re-derive the displayed view: a requested "jog" now shows if a native
        # widget is available, else falls back to Custom — WITHOUT changing the
        # persisted request, so the preference returns when native does.
        self._apply_effective_view()

    # ── View selection ────────────────────────────────────────────

    def set_active_view(self, name: str) -> None:
        """Set the requested pill (from the user or persistence restore)."""
        self._set_requested(name)

    def active_view(self) -> str:
        """The view actually displayed (jog only when a native widget exists)."""
        return self._active_view

    def requested_view(self) -> str:
        """The user's persisted pill choice (may differ from the displayed
        view when Jog is temporarily unavailable). Persist THIS one."""
        return self._requested_view

    def custom_is_available(self) -> bool:
        return True

    def _set_requested(self, name: str) -> None:
        if name not in (_VIEW_JOG, _VIEW_CUSTOM):
            name = _VIEW_CUSTOM
        changed = name != self._requested_view
        self._requested_view = name
        self._apply_effective_view()
        if changed:
            self.view_changed.emit(name)

    def _apply_effective_view(self) -> None:
        eff = (_VIEW_JOG if self._requested_view == _VIEW_JOG
               and self._native_available else _VIEW_CUSTOM)
        self._active_view = eff
        self._pills.set_selected(eff)
        self._views.setCurrentWidget(
            self._native_stack if eff == _VIEW_JOG else self._custom)

    def _on_pill_selected(self, name: str) -> None:
        self._set_requested(name)

    def _on_add_clicked(self) -> None:
        self.set_active_view(_VIEW_CUSTOM)
        self._custom.open_add_menu()

    # ── Config + live ticks (Custom view only) ────────────────────

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg
        try:
            self._custom.set_hardware_config(cfg)
        except Exception as exc:
            logger.debug("ContextPanelHost.set_hardware_config failed: %s", exc)

    def on_status_update(self) -> None:
        # Native jog panel is ticked by its owning page; only the Custom view's
        # sections need driving from here.
        self._custom.on_status_update()

    def on_motion_tick(self) -> None:
        self._custom.on_motion_tick()
