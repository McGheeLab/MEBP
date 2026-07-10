"""custom_context_panel.py — the user-composed "Custom" context view.

v7.5.x: A scrollable vertical stack of section *cards* (live camera, syringe
overview, position read-outs, jog, hardware info) that the operator builds with
a ``+ Add section`` button. The layout is owned by ``ContextPanelLayoutStore``
(a single shared, persisted layout), so every change round-trips through the
store and the panel rebuilds from it — the same cards therefore appear
everywhere the panel is mounted, and survive restarts.

Live ``on_status_update`` / ``on_motion_tick`` ticks (forwarded from the host)
are relayed to each section that implements them. Section construction is
best-effort: a section that fails to build shows an error card instead of
crashing the panel, and a tick that raises in one section can't stall the rest.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QMenu, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, sf, sp
from gui.styles import COLORS
from gui.widgets.context_sections import (
    SectionContext, build_section, catalog,
)

logger = logging.getLogger(__name__)


class _SectionCard(QFrame):
    """A titled card wrapping one section widget, with collapse + reorder +
    remove controls in its header. Styled to match ``components.Card``."""

    def __init__(self, section_id: str, title: str, body: QWidget,
                 *, collapsed: bool, panel: "CustomContextPanel"):
        super().__init__()
        self._id = section_id
        self._panel = panel
        self._collapsed = collapsed
        self.setObjectName("componentCard")
        self.setStyleSheet(
            f"QFrame#componentCard {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(8)};"
            f"}}"
        )
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(6), s(8), s(8))
        outer.setSpacing(s(6))

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(s(4))
        self._toggle = self._mk_btn("▾" if not collapsed else "▸",
                                    "Collapse / expand", self._on_toggle)
        header.addWidget(self._toggle)
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(10)}pt; font-weight: 600;")
        header.addWidget(title_lbl)
        header.addStretch(1)
        header.addWidget(self._mk_btn("▲", "Move up",
                                      lambda: panel._move(section_id, -1)))
        header.addWidget(self._mk_btn("▼", "Move down",
                                      lambda: panel._move(section_id, +1)))
        header.addWidget(self._mk_btn("✕", "Remove section",
                                      lambda: panel._remove(section_id)))
        outer.addLayout(header)

        self._body = body
        outer.addWidget(self._body)
        self._body.setVisible(not collapsed)

    def _mk_btn(self, text: str, tip: str, cb) -> QPushButton:
        b = QPushButton(text)
        b.setObjectName("flatBtn")
        b.setToolTip(tip)
        b.setCursor(Qt.PointingHandCursor)
        b.setFixedSize(s(22), s(22))
        b.clicked.connect(cb)
        return b

    def _on_toggle(self) -> None:
        self._collapsed = not self._collapsed
        self._body.setVisible(not self._collapsed)
        self._toggle.setText("▸" if self._collapsed else "▾")
        self._panel._persist_collapsed(self._id, self._collapsed)

    def body(self) -> QWidget:
        return self._body


class CustomContextPanel(QWidget):
    """Composable panel of section cards, driven by a
    ``ContextPanelLayoutStore``."""

    def __init__(self, ctx: SectionContext, parent: QWidget | None = None):
        super().__init__(parent)
        self._ctx = ctx
        self._store = ctx.layout_store
        self._section_widgets: list[QWidget] = []
        self._rebuilding = False

        root = QVBoxLayout(self)
        root.setContentsMargins(s(6), s(6), s(6), s(6))
        root.setSpacing(s(6))

        # Add-section bar
        bar = QHBoxLayout()
        bar.setContentsMargins(0, 0, 0, 0)
        self._add_btn = QPushButton("＋ Add section")
        self._add_btn.setObjectName("accentBtn")
        self._add_btn.setCursor(Qt.PointingHandCursor)
        self._add_btn.clicked.connect(self.open_add_menu)
        bar.addWidget(self._add_btn)
        bar.addStretch(1)
        root.addLayout(bar)

        # Scrollable card stack
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._inner = QWidget()
        self._inner_lay = QVBoxLayout(self._inner)
        self._inner_lay.setContentsMargins(0, 0, 0, 0)
        self._inner_lay.setSpacing(s(8))
        self._inner_lay.addStretch(1)
        self._scroll.setWidget(self._inner)
        root.addWidget(self._scroll, stretch=1)

        if self._store is not None:
            self._store.add_listener(self._on_store_changed)
        self._rebuild()

    # ── Store-driven rebuild ──────────────────────────────────────

    def _on_store_changed(self, _store) -> None:
        self._rebuild()

    def _rebuild(self) -> None:
        if self._rebuilding:
            return
        self._rebuilding = True
        try:
            # Tear down ALL existing layout items (cards + the trailing stretch)
            # so stretches can't accumulate across rebuilds.
            while self._inner_lay.count():
                item = self._inner_lay.takeAt(0)
                w = item.widget() if item is not None else None
                if w is not None:
                    w.setParent(None)
                    w.deleteLater()
            self._section_widgets = []

            sections = self._store.sections() if self._store is not None else []
            if not sections:
                placeholder = QLabel(
                    "No sections yet.\nUse ＋ Add section to build your panel.")
                placeholder.setAlignment(Qt.AlignCenter)
                placeholder.setWordWrap(True)
                placeholder.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;"
                    f"padding: {sp(16)};")
                self._inner_lay.insertWidget(0, placeholder)
                return

            from gui.widgets.context_sections import SECTION_REGISTRY
            for idx, sec in enumerate(sections):
                stype = sec.get("type")
                sid = sec.get("id")
                spec = SECTION_REGISTRY.get(stype)
                label = spec.label if spec is not None else str(stype)
                try:
                    body = build_section(
                        stype, self._ctx, sec.get("options"))
                    self._section_widgets.append(body)
                except Exception as exc:
                    logger.warning(
                        "CustomContextPanel: section %r failed to build: %s",
                        stype, exc)
                    body = self._error_widget(stype, exc)
                card = _SectionCard(
                    sid, label, body,
                    collapsed=bool(sec.get("collapsed", False)), panel=self)
                self._inner_lay.insertWidget(idx, card)
            self._inner_lay.addStretch(1)
        finally:
            self._rebuilding = False

    def _error_widget(self, stype, exc) -> QLabel:
        lbl = QLabel(f"Section “{stype}” failed to load.\n{exc}")
        lbl.setWordWrap(True)
        lbl.setStyleSheet(
            f"color: {COLORS.get('red', '#f38ba8')}; font-size: {sf(9)}pt;")
        return lbl

    # ── Mutations (via the store) ─────────────────────────────────

    def open_add_menu(self) -> None:
        if self._store is None:
            return
        menu = QMenu(self)
        for stype, label, icon in catalog():
            act = menu.addAction(f"{icon}  {label}")
            act.triggered.connect(
                lambda _checked=False, t=stype: self._store.add_section(t))
        menu.exec(self._add_btn.mapToGlobal(
            self._add_btn.rect().bottomLeft()))

    def _remove(self, section_id: str) -> None:
        if self._store is not None:
            self._store.remove_section(section_id)

    def _move(self, section_id: str, delta: int) -> None:
        if self._store is not None:
            self._store.move_section(section_id, delta)

    def _persist_collapsed(self, section_id: str, collapsed: bool) -> None:
        if self._store is not None:
            # notify=False: a cosmetic collapse must not trigger a full rebuild.
            self._store.set_collapsed(section_id, collapsed, notify=False)

    # ── Config + live ticks ───────────────────────────────────────

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg
        for w in self._section_widgets:
            fn = getattr(w, "set_hardware_config", None)
            if callable(fn):
                try:
                    fn(cfg)
                except Exception as exc:
                    logger.debug("section set_hardware_config failed: %s", exc)

    def on_status_update(self) -> None:
        for w in self._section_widgets:
            fn = getattr(w, "on_status_update", None)
            if callable(fn):
                try:
                    fn()
                except Exception as exc:
                    logger.debug("section on_status_update failed: %s", exc)

    def on_motion_tick(self) -> None:
        for w in self._section_widgets:
            fn = getattr(w, "on_motion_tick", None)
            if callable(fn):
                try:
                    fn()
                except Exception as exc:
                    logger.debug("section on_motion_tick failed: %s", exc)
