"""Hardware Setup → Layout: place designed rosettes into plate wells.

Replaces the v7.4.8 gesture of double-clicking a well on the *other* tab to
drill into an anonymous nested design. Rosettes are now library documents you
design once and place many times — the operator's request 8.

Both placement gestures are supported, because they suit different jobs:

* **Stamp** — pick a rosette, then click wells. Stays armed, so seating twenty
  wells is twenty clicks.
* **Select → assign** — marquee or Ctrl-click wells, then Apply. Better when
  the wells are already selected, or scattered.

Placements are LIVE references by rosette id (the operator's choice over a
frozen copy), so fixing a rosette once updates every plate using it. Renaming
one is safe — the link is by id, not name.
"""
from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QComboBox, QDoubleSpinBox, QHBoxLayout, QHeaderView,
    QLabel, QMessageBox, QPushButton, QSplitter, QTableWidget,
    QTableWidgetItem, QToolButton, QVBoxLayout, QWidget,
)

from gui.scaling import s, scaled_font_size as _sf
from gui.styles import COLORS
from gui.pages.hardware.plate_library import PlateThumbnail
from gui.widgets.plate_document_canvas import PlateDocumentCanvas, Tool
from SupportClasses.PlateDocument import PlateDocument, Ref
from SupportClasses.PlateDocumentStore import PlateDocumentStore

logger = logging.getLogger(__name__)


class _RosetteChip(QWidget):
    """A small palette entry: thumbnail + name. Clicking arms stamp mode."""

    clicked = Signal(str)

    def __init__(self, doc_id: str, name: str, doc: PlateDocument,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._id = doc_id
        self.setCursor(Qt.PointingHandCursor)
        v = QVBoxLayout(self)
        v.setContentsMargins(s(4), s(4), s(4), s(4))
        v.setSpacing(s(2))
        self._thumb = PlateThumbnail()
        self._thumb.setMinimumHeight(s(64))
        self._thumb.setMaximumHeight(s(64))
        wells = doc.evaluate()
        self._thumb.set_geometry([(w.x, w.y, w.diameter_mm) for w in wells],
                                 doc.boundary.extent_a1(), circle=True)
        v.addWidget(self._thumb)
        lbl = QLabel(name)
        lbl.setWordWrap(True)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {COLORS['text']}; font-size: {_sf(8)}pt;")
        v.addWidget(lbl)
        self.set_armed(False)

    def doc_id(self) -> str:
        return self._id

    def set_armed(self, on: bool) -> None:
        colour = COLORS["mauve"] if on else COLORS["surface1"]
        self.setStyleSheet(
            f"background: {COLORS['mantle']}; "
            f"border: {2 if on else 1}px solid {colour}; "
            f"border-radius: {s(6)}px;")

    def mousePressEvent(self, ev):
        self.clicked.emit(self._id)
        super().mousePressEvent(ev)


class RosettePlacementPage(QWidget):
    """Palette · plate schematic · placement table."""

    placements_changed = Signal()
    edit_rosette_requested = Signal(str)

    def __init__(self, plate_store: PlateDocumentStore | None = None,
                 rosette_store: PlateDocumentStore | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._plate_store = plate_store
        self._rosette_store = rosette_store
        self._doc: Optional[PlateDocument] = None
        self._armed: str = ""
        self._chips: list[_RosetteChip] = []
        self._build_ui()
        self._push_rosette_loader()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(4), s(4), s(4), s(4))
        outer.setSpacing(s(5))

        head = QHBoxLayout()
        self._title = QLabel("Layout")
        self._title.setStyleSheet(f"color: {COLORS['text']}; "
                                  f"font-size: {_sf(13)}pt; font-weight: 700;")
        head.addWidget(self._title)
        head.addStretch(1)
        self._mode_lbl = QLabel("")
        self._mode_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        head.addWidget(self._mode_lbl)
        outer.addLayout(head)

        split = QSplitter(Qt.Horizontal)

        # ── palette
        pal = QWidget()
        pv = QVBoxLayout(pal)
        pv.setContentsMargins(s(2), s(2), s(2), s(2))
        pv.setSpacing(s(4))
        cap = QLabel("Rosettes")
        cap.setStyleSheet(f"color: {COLORS['subtext0']}; "
                          f"font-size: {_sf(9)}pt; font-weight: 700;")
        pv.addWidget(cap)
        self._chip_host = QWidget()
        self._chip_layout = QVBoxLayout(self._chip_host)
        self._chip_layout.setContentsMargins(0, 0, 0, 0)
        self._chip_layout.setSpacing(s(4))
        pv.addWidget(self._chip_host)
        self._pal_empty = QLabel(
            "No rosettes yet.\nDesign one on the Plate tab → Rosettes.")
        self._pal_empty.setWordWrap(True)
        self._pal_empty.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: {_sf(8)}pt;")
        pv.addWidget(self._pal_empty)
        pv.addStretch(1)
        disarm = QPushButton("Stop stamping")
        disarm.clicked.connect(lambda: self._arm(""))
        pv.addWidget(disarm)
        split.addWidget(pal)

        # ── plate
        centre = QWidget()
        cv = QVBoxLayout(centre)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(s(3))
        self._canvas = PlateDocumentCanvas()
        # v7.9.1: on THIS tab a click means "seat a rosette in this well", so it
        # must select the pattern MEMBER. The builder's inverse rule handed every
        # click the whole GridPattern — which is not a well — and the placement
        # was written to an override key that matched nothing.
        self._canvas.set_member_pick_default(True)
        self._canvas.set_tool(Tool.SELECT)
        self._canvas.selection_changed.connect(self._on_selection)
        cv.addWidget(self._canvas, 1)
        self._hint = QLabel("")
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        cv.addWidget(self._hint)
        split.addWidget(centre)

        # ── placements
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(s(2), s(2), s(2), s(2))
        rv.setSpacing(s(4))
        cap2 = QLabel("Placements")
        cap2.setStyleSheet(f"color: {COLORS['subtext0']}; "
                           f"font-size: {_sf(9)}pt; font-weight: 700;")
        rv.addWidget(cap2)
        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(["Well", "Rosette", "Rot°", ""])
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        hh = self._table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.Stretch)
        hh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self._table.itemDoubleClicked.connect(self._on_table_double)
        rv.addWidget(self._table, 1)

        assign = QHBoxLayout()
        assign.setSpacing(s(4))
        self._assign_lbl = QLabel("Assign to selection:")
        self._assign_lbl.setStyleSheet(f"font-size: {_sf(8)}pt;")
        assign.addWidget(self._assign_lbl)
        self._assign_combo = QComboBox()
        assign.addWidget(self._assign_combo, 1)
        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self._assign_selected)
        assign.addWidget(apply_btn)
        rv.addLayout(assign)

        rot_row = QHBoxLayout()
        rot_row.addWidget(QLabel("Rotate selected:"))
        self._rot_spin = QDoubleSpinBox()
        self._rot_spin.setRange(-360.0, 360.0)
        self._rot_spin.setSuffix("°")
        self._rot_spin.setDecimals(1)
        rot_row.addWidget(self._rot_spin)
        rot_btn = QPushButton("Set")
        rot_btn.clicked.connect(self._rotate_selected)
        rot_row.addWidget(rot_btn)
        rv.addLayout(rot_row)

        clear_btn = QPushButton("Clear from selected wells")
        clear_btn.clicked.connect(self._clear_selected)
        rv.addWidget(clear_btn)
        split.addWidget(right)

        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setStretchFactor(2, 0)
        split.setSizes([s(150), s(700), s(280)])
        outer.addWidget(split, 1)

    # ── Wiring ────────────────────────────────────────────────────

    def set_stores(self, plate_store, rosette_store) -> None:
        self._plate_store = plate_store
        self._rosette_store = rosette_store
        self._push_rosette_loader()

    def _push_rosette_loader(self) -> None:
        """Let the canvas draw a seated rosette as ITSELF, not a generic mark."""
        store = self._rosette_store
        self._canvas.set_rosette_loader(
            (lambda rid: store.get(rid)) if store is not None else None)

    def set_plate(self, doc: Optional[PlateDocument]) -> None:
        self._doc = doc
        self._canvas.set_document(doc)
        self._title.setText(
            f"Layout — {doc.meta.name}" if doc else "Layout")
        self.refresh()

    def refresh(self) -> None:
        self._reload_palette()
        self._reload_table()
        self._update_hint()

    def _reload_palette(self) -> None:
        while self._chip_layout.count():
            it = self._chip_layout.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        self._chips.clear()
        self._assign_combo.clear()

        summaries = []
        if self._rosette_store is not None:
            try:
                summaries = self._rosette_store.list()
            except Exception as exc:                   # noqa: BLE001
                logger.warning("Rosette listing failed: %s", exc)
        for sm in summaries:
            doc = self._rosette_store.get(sm.id)
            if doc is None:
                continue
            chip = _RosetteChip(sm.id, sm.label, doc)
            chip.clicked.connect(self._arm)
            self._chip_layout.addWidget(chip)
            self._chips.append(chip)
            self._assign_combo.addItem(sm.label, sm.id)
        self._pal_empty.setVisible(not self._chips)

    def _reload_table(self) -> None:
        self._table.setRowCount(0)
        if self._doc is None:
            return
        by_key = {w.key: w for w in self._doc.evaluate()}
        for key, placement in self._doc.placements():
            row = self._table.rowCount()
            self._table.insertRow(row)
            well = by_key.get(key)
            self._table.setItem(row, 0, QTableWidgetItem(
                well.name if well else "?"))
            name = placement.rosette_name or placement.rosette_id
            missing = (self._rosette_store is not None
                       and self._rosette_store.get(placement.rosette_id) is None)
            item = QTableWidgetItem(name + (" (missing)" if missing else ""))
            item.setData(Qt.UserRole, key)
            if missing:
                item.setForeground(Qt.red)
            self._table.setItem(row, 1, item)
            self._table.setItem(row, 2, QTableWidgetItem(
                f"{placement.rotation_deg:g}"))
            x = QToolButton()
            x.setText("✕")
            x.setToolTip("Remove this rosette")
            x.clicked.connect(lambda _c=False, k=key: self._clear_one(k))
            self._table.setCellWidget(row, 3, x)

    # ── Stamp mode ────────────────────────────────────────────────

    def _arm(self, rosette_id: str) -> None:
        self._armed = rosette_id
        for chip in self._chips:
            chip.set_armed(chip.doc_id() == rosette_id)
        self._canvas.setCursor(
            Qt.CrossCursor if rosette_id else Qt.ArrowCursor)
        self._update_hint()

    def _update_hint(self) -> None:
        if self._doc is None:
            self._hint.setText("No plate selected — pick one on the Plate tab.")
            self._mode_lbl.setText("")
            return
        if self._armed:
            name = next((c for c in self._chips
                         if c.doc_id() == self._armed), None)
            self._mode_lbl.setText("Stamping")
            self._hint.setText(
                "Click a well to place the rosette (it stays armed, so you "
                "can click several). Shift+click removes. "
                "Or select wells and use Apply on the right.")
        else:
            self._mode_lbl.setText("")
            self._hint.setText(
                "Pick a rosette on the left to start stamping, or select "
                "wells and assign one on the right.")

    def _on_selection(self, refs: list) -> None:
        if self._armed and refs:
            self._stamp(refs)

    def _stamp(self, refs: list[Ref]) -> None:
        if self._doc is None or not self._armed:
            return
        doc = self._rosette_store.get(self._armed) if self._rosette_store \
            else None
        name = doc.meta.name if doc else ""
        placed = 0
        for ref in refs:
            try:
                self._doc.place_rosette(ref, self._armed, rosette_name=name)
                placed += 1
            except KeyError:
                continue
        if placed:
            self._after_change()
        elif refs:
            # v7.9.1: this used to be a silent no-op — every KeyError was
            # swallowed and the operator saw nothing happen and no error.
            logger.warning("Could not place a rosette at %r — not a well.", refs)
            self._hint.setText(
                "That isn't a single well — click directly on a well "
                "(Alt+click selects the whole pattern instead).")

    # ── Bulk actions ──────────────────────────────────────────────

    def _target_refs(self) -> list[Ref]:
        refs = list(self._canvas.selection())
        if refs:
            return refs
        out = []
        for item in self._table.selectedItems():
            key = self._table.item(item.row(), 1)
            if key is not None and key.data(Qt.UserRole):
                out.append(key.data(Qt.UserRole))
        return list(dict.fromkeys(out))

    def _assign_selected(self) -> None:
        if self._doc is None:
            return
        rosette_id = self._assign_combo.currentData()
        refs = self._target_refs()
        if not rosette_id or not refs:
            QMessageBox.information(
                self, "Assign",
                "Select one or more wells on the plate first.")
            return
        doc = self._rosette_store.get(rosette_id) if self._rosette_store \
            else None
        name = doc.meta.name if doc else ""
        for ref in refs:
            try:
                self._doc.place_rosette(ref, rosette_id, rosette_name=name)
            except KeyError:
                continue
        self._after_change()

    def _rotate_selected(self) -> None:
        if self._doc is None:
            return
        refs = self._target_refs()
        deg = float(self._rot_spin.value())
        touched = 0
        for ref in refs:
            existing = dict(self._doc.placements()).get(ref)
            if existing is None:
                continue
            self._doc.place_rosette(ref, existing.rosette_id,
                                    rotation_deg=deg,
                                    rosette_name=existing.rosette_name)
            touched += 1
        if touched:
            self._after_change()

    def _clear_selected(self) -> None:
        if self._doc is None:
            return
        if any(self._doc.clear_rosette(r) for r in self._target_refs()):
            self._after_change()

    def _clear_one(self, key: Ref) -> None:
        if self._doc is not None and self._doc.clear_rosette(key):
            self._after_change()

    def _on_table_double(self, item) -> None:
        """Double-click navigates to the rosette's DESIGN.

        The old gesture opened an anonymous nested design in place; rosettes
        are documents now, so editing one edits it everywhere it is used.
        """
        key_item = self._table.item(item.row(), 1)
        key = key_item.data(Qt.UserRole) if key_item else None
        if key is None or self._doc is None:
            return
        placement = dict(self._doc.placements()).get(key)
        if placement:
            self.edit_rosette_requested.emit(placement.rosette_id)

    def _after_change(self) -> None:
        self._canvas.rebuild()
        self._reload_table()
        self.placements_changed.emit()

    def get_page_title(self) -> str:
        return "Layout"
