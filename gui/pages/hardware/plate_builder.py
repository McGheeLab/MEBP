"""The v7.12 parametric plate / rosette builder.

One builder serves both document kinds. It takes **no mode argument** — it
reads ``document.boundary.kind`` and adapts, which is the operator's "rosette
mode works the same way, except the boundary is set by the rosette" satisfied
by construction rather than by a parallel code path. It also retires the
v7.4.x arrangement of two designer instances sharing one design object.

The plate name is the document title in the header, not a form field, and
**there is no Save As** — copying is what the library's Duplicate is for.
Having both is what produced "Save behaves as Save As when the key is an int"
in the old designer.
"""
from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, QSize, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QHBoxLayout, QLabel, QLineEdit,
    QMessageBox, QPushButton, QSplitter, QToolButton, QVBoxLayout, QWidget,
)

from gui.scaling import s, scaled_font_size as _sf
from gui.styles import COLORS
from gui.widgets.icons import icon
from gui.widgets.plate_document_canvas import PlateDocumentCanvas, Tool
from gui.pages.hardware.plate_property_panel import PlatePropertyPanel
from SupportClasses.PlateDocument import PlateDocument
from SupportClasses.PlateDocumentStore import PlateDocumentStore

logger = logging.getLogger(__name__)

_TOOLS = [
    ("cursor", "Select", Tool.SELECT, "S"),
    ("circle-plus", "Single well", Tool.WELL, "W"),
    ("compass", "Ring of wells", Tool.RING, "C"),
    ("grid", "Grid of wells", Tool.GRID, "G"),
    ("line", "Construction line", Tool.LINE, "L"),
    ("ruler", "Dimension", Tool.DIMENSION, "D"),
]


class PlateBuilderPage(QWidget):
    """Header + tool palette + canvas + properties."""

    back_requested = Signal()
    document_saved = Signal(str)
    dirty_changed = Signal(bool)

    def __init__(self, store: PlateDocumentStore | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._store = store
        self._doc: Optional[PlateDocument] = None
        self._dirty = False
        self._forked_from_standard = False
        self._build_ui()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(4), s(4), s(4), s(4))
        outer.setSpacing(s(5))

        head = QHBoxLayout()
        head.setSpacing(s(6))
        back = QPushButton("‹ All plates")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self._on_back)
        head.addWidget(back)

        # The name is the document TITLE, not a form field.
        self._title = QLineEdit()
        self._title.setPlaceholderText("Untitled")
        self._title.setStyleSheet(f"""
            QLineEdit {{
                background: transparent; border: none;
                color: {COLORS['text']}; font-size: {_sf(13)}pt;
                font-weight: 700; padding: {s(2)}px;
            }}
            QLineEdit:focus {{
                background: {COLORS['surface0']};
                border-radius: {s(4)}px;
            }}
        """)
        self._title.editingFinished.connect(self._on_title_edited)
        head.addWidget(self._title, 1)

        self._dirty_lbl = QLabel("")
        self._dirty_lbl.setStyleSheet(
            f"color: #fab387; font-size: {_sf(9)}pt;")
        head.addWidget(self._dirty_lbl)

        self._save_btn = QPushButton("Save")
        self._save_btn.setCursor(Qt.PointingHandCursor)
        self._save_btn.setEnabled(False)
        self._save_btn.clicked.connect(self.save)
        head.addWidget(self._save_btn)
        outer.addLayout(head)

        self._fork_note = QLabel("")
        self._fork_note.setWordWrap(True)
        self._fork_note.setVisible(False)
        self._fork_note.setStyleSheet(
            f"color: {COLORS['crust']}; background: {COLORS['yellow']}; "
            f"border-radius: {s(4)}px; padding: {s(4)}px; "
            f"font-size: {_sf(8)}pt;")
        outer.addWidget(self._fork_note)

        body = QSplitter(Qt.Horizontal)

        palette = QWidget()
        pv = QVBoxLayout(palette)
        pv.setContentsMargins(s(2), s(2), s(2), s(2))
        pv.setSpacing(s(3))
        self._tool_group = QButtonGroup(self)
        self._tool_group.setExclusive(True)
        for ico, label, tool, key in _TOOLS:
            b = QToolButton()
            b.setCheckable(True)
            b.setChecked(tool == Tool.SELECT)
            b.setFixedSize(s(40), s(40))
            b.setIconSize(QSize(s(20), s(20)))
            b.setToolTip(f"{label}  ({key})")
            b.setCursor(Qt.PointingHandCursor)
            try:
                b.setIcon(icon(ico, color="#ffffff", px=s(20)))
            except Exception:                          # pragma: no cover
                b.setText(label[0])
            b.clicked.connect(lambda _c=False, t=tool: self._canvas.set_tool(t))
            self._tool_group.addButton(b)
            b.setProperty("tool", tool)
            pv.addWidget(b)
        pv.addSpacing(s(8))
        for glyph, tip, slot in (("⟲", "Undo (Ctrl+Z)", self._undo),
                                 ("⟳", "Redo (Ctrl+Y)", self._redo),
                                 ("⤢", "Fit view (F)", lambda: self._canvas.fit_view()),
                                 ("🗑", "Delete selection (Del)",
                                  lambda: self._canvas.delete_selection())):
            b = QToolButton()
            b.setText(glyph)
            b.setFixedSize(s(40), s(30))
            b.setToolTip(tip)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(slot)
            pv.addWidget(b)
        pv.addStretch(1)
        body.addWidget(palette)

        centre = QWidget()
        cv = QVBoxLayout(centre)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(s(3))
        self._canvas = PlateDocumentCanvas()
        cv.addWidget(self._canvas, 1)

        strip = QHBoxLayout()
        strip.setSpacing(s(8))
        self._status = QLabel("")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        strip.addWidget(self._status, 1)
        self._hover = QLabel("")
        self._hover.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: {_sf(8)}pt;")
        strip.addWidget(self._hover)
        snap = QCheckBox("Snap 1 mm")
        snap.toggled.connect(
            lambda on: self._canvas.set_snap_mm(1.0 if on else 0.0))
        strip.addWidget(snap)
        wheel = QCheckBox("Wheel zoom")
        wheel.setChecked(True)
        wheel.toggled.connect(self._canvas.set_wheel_zoom)
        strip.addWidget(wheel)
        dim_mode = QComboBox()
        dim_mode.addItem("Dim: centre", "center")
        dim_mode.addItem("Dim: edge", "edge")
        dim_mode.currentIndexChanged.connect(
            lambda _i: self._canvas.set_dim_mode(dim_mode.currentData()))
        strip.addWidget(dim_mode)
        cv.addLayout(strip)
        body.addWidget(centre)

        self._panel = PlatePropertyPanel()
        self._panel.attach(self._canvas)
        self._panel.document_edited.connect(self._mark_dirty)
        body.addWidget(self._panel)

        body.setStretchFactor(0, 0)
        body.setStretchFactor(1, 1)
        body.setStretchFactor(2, 0)
        body.setSizes([s(52), s(680), s(330)])
        outer.addWidget(body, 1)

        self._canvas.document_changed.connect(self._mark_dirty)
        self._canvas.status_message.connect(self._status.setText)
        self._canvas.hover_moved.connect(
            lambda x, y: self._hover.setText(f"{x:.2f}, {y:.2f} mm"))
        self._canvas.tool_changed.connect(self._sync_tool_buttons)

        for seq, slot in (("Ctrl+S", self.save), ("Ctrl+Z", self._undo),
                          ("Ctrl+Shift+Z", self._redo), ("Ctrl+Y", self._redo)):
            sc = QShortcut(QKeySequence(seq), self, activated=slot)
            sc.setContext(Qt.WidgetWithChildrenShortcut)

    # ── Document ──────────────────────────────────────────────────

    def set_store(self, store: PlateDocumentStore) -> None:
        self._store = store

    def load(self, doc: PlateDocument, *, forked: bool = False,
             fork_note: str = "") -> None:
        self._doc = doc
        self._forked_from_standard = forked
        self._canvas.set_document(doc)
        self._panel.set_document(doc)
        self._title.setText(doc.meta.name)
        self._fork_note.setVisible(bool(forked and fork_note))
        self._fork_note.setText(fork_note)
        self._set_dirty(forked)
        self._canvas.solve()
        self._status.setText(
            "Rosette — the bore is the boundary." if doc.boundary.is_circle()
            else "")

    def document(self) -> Optional[PlateDocument]:
        return self._doc

    def canvas(self) -> PlateDocumentCanvas:
        return self._canvas

    # ── Dirty / save ──────────────────────────────────────────────

    def _mark_dirty(self) -> None:
        self._set_dirty(True)

    def _set_dirty(self, on: bool) -> None:
        self._dirty = bool(on)
        self._dirty_lbl.setText("● unsaved" if on else "")
        self._save_btn.setEnabled(bool(on))
        self.dirty_changed.emit(self._dirty)

    def is_dirty(self) -> bool:
        return self._dirty

    def save(self) -> bool:
        if self._doc is None or self._store is None:
            return False
        problems = self._doc.validate()
        blocking = [p for p in problems if "unique" in p or "reserved" in p]
        if blocking:
            QMessageBox.warning(self, "Cannot save",
                                "\n".join(blocking[:5]))
            return False
        self._store.save(self._doc)
        self._forked_from_standard = False
        self._fork_note.setVisible(False)
        self._set_dirty(False)
        self.document_saved.emit(self._doc.meta.id)
        self._status.setText(f"Saved '{self._doc.meta.name}'.")
        return True

    def maybe_discard(self) -> bool:
        """Ask before losing unsaved work. True = it is safe to proceed.

        A plain method, deliberately NOT wired into hideEvent/closeEvent — a
        modal in a teardown path hangs an offscreen test at teardown.
        """
        if not self._dirty or self._doc is None:
            return True
        answer = QMessageBox.question(
            self, "Unsaved changes",
            f"'{self._doc.meta.name}' has unsaved changes.",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Save)
        if answer == QMessageBox.Cancel:
            return False
        if answer == QMessageBox.Save:
            return self.save()
        return True

    # ── Handlers ──────────────────────────────────────────────────

    def _on_back(self) -> None:
        if self.maybe_discard():
            self.back_requested.emit()

    def _on_title_edited(self) -> None:
        if self._doc is None:
            return
        name = self._title.text().strip()
        if name and name != self._doc.meta.name:
            self._doc.meta.name = name
            self._mark_dirty()

    def _undo(self) -> None:
        self._canvas.undo()
        self._panel.bind()

    def _redo(self) -> None:
        self._canvas.redo()
        self._panel.bind()

    def _sync_tool_buttons(self, tool) -> None:
        for b in self._tool_group.buttons():
            if b.property("tool") == tool:
                b.setChecked(True)
                break
