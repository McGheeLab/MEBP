"""
reorderable_sections.py — collapsible, user-reorderable section stack (v7.5.x).

A vertical stack of titled sections where each section can be:
  * collapsed / expanded (chevron in the header, or click the title), and
  * moved up or down (▲ / ▼ in the header).

Motivation: on Hardware Setup → Device (Stage) the least-used sections sat
at the top and the operator wanted to promote the ones they reach for often
and fold the rest away. The widget is deliberately content-agnostic and
reusable — it wraps whatever QWidget you hand it. The host page owns
persistence: read :meth:`order` / :meth:`collapsed_states` on change and
restore with :meth:`set_order` / :meth:`apply_collapsed_states`.

All colors come from ``gui.styles.COLORS`` and every pixel dimension goes
through ``gui.scaling`` per the project's DPI-aware convention.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFrame, QLabel, QPushButton,
    QGroupBox, QSizePolicy,
)

from gui.styles import COLORS
from gui.scaling import s, sf, sp


class ReorderableSection(QFrame):
    """One titled section: a header row (chevron + title + ▲/▼) over a body.

    The header always stays visible so a collapsed section can still be
    expanded and moved. If the wrapped content is a ``QGroupBox`` its own
    title is cleared — the section header owns the title instead, so there
    is exactly one title and it survives collapsing.
    """

    move_requested = Signal(str, int)      # (key, delta): -1 = up, +1 = down
    collapsed_changed = Signal(str, bool)  # (key, collapsed)

    def __init__(self, key: str, title: str, content: QWidget,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._key = key
        self._collapsed = False
        self._content = content

        if isinstance(content, QGroupBox):
            # The section header carries the title now; a groupbox title
            # would double up and vanish when the body is folded.
            content.setTitle("")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(2))

        # ── Header ────────────────────────────────────────────────
        header = QFrame()
        header.setObjectName("reorderSectionHeader")
        header.setStyleSheet(
            f"#reorderSectionHeader {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(6)};"
            f"}}"
        )
        h = QHBoxLayout(header)
        h.setContentsMargins(s(6), s(2), s(6), s(2))
        h.setSpacing(s(4))

        # Title button (chevron + text). Clicking anywhere on it toggles.
        self._title_btn = QPushButton(f"▾  {title}")
        self._title_btn.setObjectName("reorderSectionTitle")
        self._title_btn.setCursor(Qt.PointingHandCursor)
        self._title_btn.setToolTip("Click to collapse / expand this section")
        self._title_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._title_btn.setStyleSheet(
            f"QPushButton#reorderSectionTitle {{"
            f"  color: {COLORS['blue']};"
            f"  font-size: {sf(10)}pt;"
            f"  font-weight: 600;"
            f"  text-align: left;"
            f"  border: none;"
            f"  background: transparent;"
            f"  padding: {sp(4)} {sp(4)};"
            f"}}"
            f"QPushButton#reorderSectionTitle:hover {{"
            f"  color: {COLORS['mauve']};"
            f"}}"
        )
        self._title_btn.clicked.connect(lambda: self.set_collapsed(
            not self._collapsed, emit=True))
        h.addWidget(self._title_btn, 1)

        self._up_btn = self._nav_button("▲", "Move section up")
        self._up_btn.clicked.connect(
            lambda: self.move_requested.emit(self._key, -1))
        h.addWidget(self._up_btn)

        self._down_btn = self._nav_button("▼", "Move section down")
        self._down_btn.clicked.connect(
            lambda: self.move_requested.emit(self._key, +1))
        h.addWidget(self._down_btn)

        lay.addWidget(header)
        lay.addWidget(content)

    def _nav_button(self, glyph: str, tip: str) -> QPushButton:
        btn = QPushButton(glyph)
        btn.setToolTip(tip)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setFixedSize(s(24), s(22))
        btn.setStyleSheet(
            f"QPushButton {{"
            f"  color: {COLORS['subtext0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {sp(4)};"
            f"  background: {COLORS['surface1']};"
            f"  font-size: {sf(8)}pt;"
            f"}}"
            f"QPushButton:hover {{"
            f"  color: {COLORS['text']};"
            f"  border-color: {COLORS['blue']};"
            f"}}"
            f"QPushButton:disabled {{"
            f"  color: {COLORS['surface2']};"
            f"  background: {COLORS['surface0']};"
            f"  border-color: {COLORS['surface0']};"
            f"}}"
        )
        return btn

    # ── Public API ────────────────────────────────────────────────

    def key(self) -> str:
        return self._key

    def is_collapsed(self) -> bool:
        return self._collapsed

    def set_collapsed(self, collapsed: bool, *, emit: bool = False):
        self._collapsed = bool(collapsed)
        self._content.setVisible(not self._collapsed)
        # Rewrite only the chevron glyph, keep the title text intact.
        txt = self._title_btn.text()
        title_only = txt[txt.find(" ") + 1:] if " " in txt else txt
        glyph = "▸" if self._collapsed else "▾"
        self._title_btn.setText(f"{glyph} {title_only}")
        if emit:
            self.collapsed_changed.emit(self._key, self._collapsed)

    def set_move_enabled(self, up: bool, down: bool):
        self._up_btn.setEnabled(up)
        self._down_btn.setEnabled(down)


class ReorderableSectionList(QWidget):
    """Vertical stack of :class:`ReorderableSection` with move + collapse.

    Emits :sig:`order_changed` (list of keys) and :sig:`collapsed_changed`
    (key, collapsed) whenever the user reorders or folds a section, so the
    host can persist the layout.
    """

    order_changed = Signal(list)
    collapsed_changed = Signal(str, bool)

    def __init__(self, parent: Optional[QWidget] = None, *, spacing: int = 18):
        super().__init__(parent)
        self._sections: dict[str, ReorderableSection] = {}
        self._order: list[str] = []
        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._lay.setSpacing(s(spacing))

    def add_section(self, key: str, title: str,
                    content: QWidget) -> ReorderableSection:
        if key in self._sections:
            raise ValueError(f"duplicate section key: {key!r}")
        sec = ReorderableSection(key, title, content)
        sec.move_requested.connect(self._on_move)
        sec.collapsed_changed.connect(self.collapsed_changed.emit)
        self._sections[key] = sec
        self._order.append(key)
        self._rebuild()
        return sec

    def order(self) -> list[str]:
        return list(self._order)

    def set_order(self, keys) -> None:
        """Reorder to match ``keys``. Unknown keys are ignored; any section
        missing from ``keys`` (e.g. a newly added one) keeps its relative
        position at the end — so a stale persisted order never drops a
        section from view."""
        new = [k for k in (keys or []) if k in self._sections]
        for k in self._order:
            if k not in new:
                new.append(k)
        self._order = new
        self._rebuild()

    def collapsed_states(self) -> dict[str, bool]:
        return {k: self._sections[k].is_collapsed() for k in self._order}

    def apply_collapsed_states(self, states: dict) -> None:
        for k, v in (states or {}).items():
            sec = self._sections.get(k)
            if sec is not None:
                sec.set_collapsed(bool(v))

    # ── internals ─────────────────────────────────────────────────

    def _on_move(self, key: str, delta: int):
        if key not in self._order:
            return
        i = self._order.index(key)
        j = i + delta
        if 0 <= j < len(self._order):
            self._order[i], self._order[j] = self._order[j], self._order[i]
            self._rebuild()
            self.order_changed.emit(list(self._order))

    def _rebuild(self):
        while self._lay.count():
            item = self._lay.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        n = len(self._order)
        for idx, key in enumerate(self._order):
            sec = self._sections[key]
            self._lay.addWidget(sec)
            sec.set_move_enabled(idx > 0, idx < n - 1)
