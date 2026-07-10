"""
print_library.py — Print Library / Manager (Print Builder tab, v7.5.x).

A grid "viewer" of every saved print in ``config/prints/`` with the
file-management operations an operator asked for: **Rename**, **Duplicate**,
**Delete**, **bulk delete**, an **orphaned-file cleanup**, "Open prints
folder", and "Open in Print Setup".

Each print is shown as a card with a small auto-fit trajectory thumbnail +
metadata (modified date, object/waypoint count). Thumbnails are built from the
file's objects — ``csv_import`` objects by reading their CSV, parametric
objects via ``GeometryEngine`` — all best-effort so one malformed file never
breaks the grid.

Design note: the library manages *files* (and previews them); it does not
re-open a baked trajectory back into the vector Sketch model (not
round-trippable). "Open in Print Setup" re-uses the existing
``print_file_created`` flow so a print can still be *used*.
"""

from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, Signal, QSize, QTimer, QUrl
from PySide6.QtGui import (
    QColor, QDesktopServices, QFont, QFontMetrics, QPainter, QPainterPath, QPen,
)
from PySide6.QtWidgets import (
    QCheckBox, QFrame, QGridLayout, QHBoxLayout, QInputDialog, QLabel,
    QMessageBox, QPushButton, QScrollArea, QSizePolicy, QToolButton,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, scaled_font_size as _sf, scale_factor
from gui.styles import COLORS
from gui.widgets.icons import icon
from SupportClasses.PrintFileManager import (
    PrintFileManager, read_print_objects,
)

logger = logging.getLogger(__name__)

_CARD_W = 232          # base px (scaled) — card width
_THUMB_H = 130         # base px (scaled) — thumbnail height
_DEFAULT_PATH_COLOR = COLORS.get("green", "#a6e3a1")


# ═══════════════════════════════════════════════════════════════════
# Trajectory helpers (thumbnail source)
# ═══════════════════════════════════════════════════════════════════

def object_trajectory(obj_dict, needle, syringe_map):
    """One object's Nx7 trajectory (well-relative mm), or None.

    ``csv_import`` → read the referenced CSV; parametric → generate via the
    canonical ``GeometryEngine`` pipeline (needs a needle). Mirrors Quick
    Print's ``_obj_dict_to_trajectory`` so thumbnails match what prints.
    """
    import numpy as np
    from SupportClasses.GeometryEngine import (
        PrintObject, generate_object_trajectory,
    )

    od = dict(obj_dict)
    od.setdefault("name", "object")
    otype = od.get("object_type", "")
    traj = None

    if od.get("source") == "csv" or otype == "csv_import":
        params = od.get("params", {}) or {}
        csv_data = od.get("trajectory") or params.get("_csv_data")
        csv_src = params.get("source_file") or params.get("csv_path")
        if csv_data is None and csv_src:
            from SupportClasses.TrajectoryPlanner import import_csv_trajectory
            csv_data = import_csv_trajectory(csv_src)
        if csv_data is not None:
            traj = np.asarray(csv_data, dtype=np.float64)
    else:
        obj = PrintObject.from_dict(od)
        if obj.has_trajectory:
            traj = obj.trajectory
        elif needle is not None:
            generate_object_trajectory(obj, needle, syringe_map, pump_id="P1")
            traj = obj.trajectory

    if traj is None or len(traj) == 0:
        return None
    arr = np.asarray(traj, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2:
        return None
    return arr


def object_polylines(objects: dict, needle, syringe_map) -> list[tuple[str, list]]:
    """``{obj_name: obj_dict}`` → ``[(color_hex, [(x_mm, y_mm), ...]), ...]``.

    One polyline per object. Best-effort — an object that can't build a
    trajectory is skipped.
    """
    out: list[tuple[str, list]] = []
    for obj in (objects or {}).values():
        if not isinstance(obj, dict):
            continue
        color = obj.get("color") or _DEFAULT_PATH_COLOR
        try:
            arr = object_trajectory(obj, needle, syringe_map)
        except Exception as e:                       # noqa: BLE001
            logger.debug("thumbnail trajectory build failed: %s", e)
            arr = None
        if arr is None or len(arr) < 2:
            continue
        out.append((color, [(float(r[0]), float(r[1])) for r in arr]))
    return out


# ═══════════════════════════════════════════════════════════════════
# Thumbnail widget
# ═══════════════════════════════════════════════════════════════════

class PrintThumbnail(QWidget):
    """Custom-painted, auto-fit top-down XY toolpath preview."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._polys: list[tuple[str, list]] = []
        self._placeholder = "no preview"
        self.setMinimumHeight(s(_THUMB_H))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_polylines(self, polys: list[tuple[str, list]] | None):
        self._polys = list(polys or [])
        self.update()

    def set_placeholder(self, text: str):
        self._placeholder = text
        self._polys = []
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = self.rect()
        p.fillRect(rect, QColor(COLORS["crust"]))
        p.setPen(QPen(QColor(COLORS["surface1"]), 1))
        p.drawRect(rect.adjusted(0, 0, -1, -1))

        pts_all = [pt for _c, pts in self._polys for pt in pts]
        if not pts_all:
            p.setPen(QColor(COLORS["overlay0"]))
            f = p.font()
            f.setPointSize(_sf(8))
            p.setFont(f)
            p.drawText(rect, Qt.AlignCenter, self._placeholder)
            return

        xs = [x for x, _y in pts_all]
        ys = [y for _x, y in pts_all]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        span = max(maxx - minx, maxy - miny, 1e-6)
        pad = s(10)
        avail = min(rect.width(), rect.height()) - 2 * pad
        scale = max(avail, 1) / span
        cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0

        def to_px(x, y):
            return (rect.width() / 2.0 + (x - cx) * scale,
                    rect.height() / 2.0 - (y - cy) * scale)   # Y up

        for color, pts in self._polys:
            pen = QPen(QColor(color))
            pen.setWidthF(1.6)
            pen.setCosmetic(True)
            p.setPen(pen)
            path = QPainterPath()
            first = True
            for x, y in pts:
                px, py = to_px(x, y)
                if first:
                    path.moveTo(px, py)
                    first = False
                else:
                    path.lineTo(px, py)
            p.drawPath(path)


# ═══════════════════════════════════════════════════════════════════
# Print card
# ═══════════════════════════════════════════════════════════════════

class PrintCard(QFrame):
    """One saved print: thumbnail + name + meta + select + actions."""

    rename_requested = Signal(str)
    duplicate_requested = Signal(str)
    delete_requested = Signal(str)
    open_requested = Signal(str)
    edit_requested = Signal(str)
    selection_changed = Signal()

    def __init__(self, info: dict, parent: QWidget | None = None):
        super().__init__(parent)
        self._name = info.get("name", "?")
        self.setObjectName("printCard")
        self.setFixedWidth(s(_CARD_W))
        self.setStyleSheet(f"""
            #printCard {{
                background: {COLORS['mantle']};
                border: 1px solid {COLORS['surface1']};
                border-radius: {s(8)}px;
            }}
            #printCard QLabel {{ background: transparent; }}
        """)
        v = QVBoxLayout(self)
        v.setContentsMargins(s(8), s(8), s(8), s(8))
        v.setSpacing(s(5))

        self._thumb = PrintThumbnail(self)
        v.addWidget(self._thumb)

        # Name (elided to the card width) with a full-name tooltip.
        name_lbl = QLabel()
        name_lbl.setToolTip(self._name)
        name_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {_sf(10)}pt; "
            f"font-weight: 700;")
        fm = QFontMetrics(name_lbl.font())
        name_lbl.setText(fm.elidedText(self._name, Qt.ElideRight, s(_CARD_W - 24)))
        v.addWidget(name_lbl)

        meta_lbl = QLabel(self._meta_text(info))
        meta_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        meta_lbl.setWordWrap(True)
        v.addWidget(meta_lbl)

        row = QHBoxLayout()
        row.setSpacing(s(3))
        self._check = QCheckBox()
        self._check.setToolTip("Select for bulk delete")
        self._check.toggled.connect(lambda _c: self.selection_changed.emit())
        row.addWidget(self._check)
        row.addStretch(1)
        row.addWidget(self._icon_btn(
            "printer", "Open in Print Setup",
            lambda: self.open_requested.emit(self._name)))
        row.addWidget(self._icon_btn(
            "pencil", "Rename",
            lambda: self.rename_requested.emit(self._name)))
        row.addWidget(self._icon_btn(
            "clipboard", "Duplicate",
            lambda: self.duplicate_requested.emit(self._name)))
        row.addWidget(self._icon_btn(
            "trash", "Delete",
            lambda: self.delete_requested.emit(self._name), danger=True))
        v.addLayout(row)

        # Primary affordance: open this print back in the Sketch editor.
        edit = QPushButton("✏  Edit in Sketch")
        edit.setToolTip("Open this print in the Sketch editor to change it "
                        "(double-click the card too).")
        edit.setCursor(Qt.PointingHandCursor)
        edit.setStyleSheet(f"""
            QPushButton {{
                background: {COLORS['surface0']}; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: {s(5)}px; padding: {s(4)}px;
                font-size: {_sf(9)}pt; font-weight: 600;
            }}
            QPushButton:hover {{
                background: {COLORS['mauve']}; color: {COLORS['crust']};
            }}
        """)
        edit.clicked.connect(lambda: self.edit_requested.emit(self._name))
        v.addWidget(edit)

    def mouseDoubleClickEvent(self, event):
        self.edit_requested.emit(self._name)
        super().mouseDoubleClickEvent(event)

    # ── API ───────────────────────────────────────────────────────

    def name(self) -> str:
        return self._name

    def is_selected(self) -> bool:
        return self._check.isChecked()

    def set_polylines(self, polys, placeholder="empty print"):
        if polys:
            self._thumb.set_polylines(polys)
        else:
            self._thumb.set_placeholder(placeholder)

    # ── helpers ───────────────────────────────────────────────────

    @staticmethod
    def _meta_text(info: dict) -> str:
        n = info.get("object_count", 0)
        mod = info.get("modified", "") or ""
        when = ""
        try:
            when = datetime.fromisoformat(mod).strftime("%Y-%m-%d %H:%M")
        except (ValueError, TypeError):
            when = mod[:16]
        obj = f"{n} object" + ("" if n == 1 else "s")
        return f"{obj}\n{when}" if when else obj

    def _icon_btn(self, ico, tip, slot, danger=False) -> QToolButton:
        b = QToolButton()
        b.setToolTip(tip)
        b.setFixedSize(s(26), s(26))
        b.setCursor(Qt.PointingHandCursor)
        b.setIcon(icon(ico, color=("#f38ba8" if danger else "#ffffff"),
                       px=s(15)))
        b.setIconSize(QSize(s(15), s(15)))
        b.setStyleSheet(f"""
            QToolButton {{
                background: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: {s(4)}px;
            }}
            QToolButton:hover {{ background: {COLORS['surface1']}; }}
        """)
        b.clicked.connect(slot)
        return b


# ═══════════════════════════════════════════════════════════════════
# Library page
# ═══════════════════════════════════════════════════════════════════

class PrintLibraryPage(QWidget):
    """Grid of all saved prints with file-management operations."""

    print_file_created = Signal(str)   # "Open in Print Setup" re-uses this
    print_files_changed = Signal()     # any create/rename/delete/cleanup
    edit_print_requested = Signal(str) # "Edit in Sketch" — open in the editor

    def __init__(self, prints_dir: str | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._mgr = (PrintFileManager(prints_dir) if prints_dir
                     else PrintFileManager())
        self._needle = None
        self._syringe_map: dict = {}
        self._cards: list[PrintCard] = []
        self._cols = 0

        self._relayout_timer = QTimer(self)
        self._relayout_timer.setSingleShot(True)
        self._relayout_timer.setInterval(60)
        self._relayout_timer.timeout.connect(self._relayout)

        self._build_ui()
        self.refresh()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        self.setStyleSheet(f"background: {COLORS['base']};")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(10), s(10), s(10), s(10))
        outer.setSpacing(s(8))

        # Header
        head = QHBoxLayout()
        title = QLabel("Prints")
        title.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {_sf(14)}pt; "
            f"font-weight: 700;")
        head.addWidget(title)
        self._count_lbl = QLabel("")
        self._count_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        head.addWidget(self._count_lbl)
        head.addStretch(1)
        self._cleanup_btn = self._text_btn(
            "Clean up orphans", self._on_cleanup_orphans)
        self._cleanup_btn.setVisible(False)
        head.addWidget(self._cleanup_btn)
        head.addWidget(self._text_btn("Open folder", self._on_open_folder))
        head.addWidget(self._text_btn("↻ Refresh", self.refresh))
        outer.addLayout(head)

        # Bulk-select bar (hidden until ≥1 card is checked)
        self._bulk_bar = QFrame()
        self._bulk_bar.setStyleSheet(
            f"background: {COLORS['surface0']}; border-radius: {s(6)}px;")
        bb = QHBoxLayout(self._bulk_bar)
        bb.setContentsMargins(s(8), s(4), s(8), s(4))
        self._bulk_lbl = QLabel("")
        self._bulk_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {_sf(9)}pt;")
        bb.addWidget(self._bulk_lbl)
        bb.addStretch(1)
        bb.addWidget(self._text_btn("Clear", self._clear_selection))
        bb.addWidget(self._text_btn(
            "Delete selected", self._on_delete_selected, danger=True))
        self._bulk_bar.setVisible(False)
        outer.addWidget(self._bulk_bar)

        # Scrolling grid
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._grid_host = QWidget()
        self._grid = QGridLayout(self._grid_host)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setSpacing(s(10))
        self._grid.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self._scroll.setWidget(self._grid_host)
        outer.addWidget(self._scroll, 1)

        self._empty_lbl = QLabel(
            "No prints yet — draw one on the Sketch tab and send it here.")
        self._empty_lbl.setAlignment(Qt.AlignCenter)
        self._empty_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(10)}pt;")
        self._empty_lbl.setVisible(False)
        outer.addWidget(self._empty_lbl)

    def _text_btn(self, text, slot, danger=False) -> QPushButton:
        b = QPushButton(text)
        b.setCursor(Qt.PointingHandCursor)
        base = COLORS.get("red", "#f38ba8") if danger else COLORS["surface0"]
        fg = COLORS["crust"] if danger else COLORS["text"]
        b.setStyleSheet(f"""
            QPushButton {{
                background: {base}; color: {fg};
                border: 1px solid {COLORS['surface1']};
                border-radius: {s(5)}px;
                padding: {s(4)}px {s(10)}px; font-size: {_sf(9)}pt;
                font-weight: 600;
            }}
            QPushButton:hover {{ background: {COLORS['surface1']}; }}
        """)
        b.clicked.connect(slot)
        return b

    # ── External API ──────────────────────────────────────────────

    def set_hardware_config(self, config) -> None:
        """Needle + syringes drive the parametric-object thumbnails."""
        try:
            self._needle = getattr(config, "needle", None)
            self._syringe_map = {}
            pumps = getattr(config, "pumps", {}) or {}
            items = pumps.items() if isinstance(pumps, dict) else enumerate(pumps)
            for pid, pcfg in items:
                syr = getattr(pcfg, "syringe", None)
                if syr is not None:
                    self._syringe_map[str(pid)] = syr
        except Exception as e:                        # noqa: BLE001
            logger.debug("set_hardware_config failed: %s", e)
        if self._needle is None:
            try:
                from SupportClasses.PhysicalModels import NeedleSpec
                self._needle = NeedleSpec(
                    gauge=22, od_um=718, id_um=413, wall_um=152)
            except Exception:                         # noqa: BLE001
                self._needle = None
        self.refresh()

    def get_page_title(self) -> str:
        return "Prints"

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._relayout_timer.start()

    # ── Refresh + layout ──────────────────────────────────────────

    def refresh(self):
        """Rebuild the whole grid from disk."""
        # Clear existing cards.
        for c in self._cards:
            c.setParent(None)
            c.deleteLater()
        self._cards = []
        self._cols = 0

        try:
            files = self._mgr.list_files()
        except Exception as e:                        # noqa: BLE001
            logger.warning("list_files failed: %s", e)
            files = []

        for info in files:
            card = PrintCard(info)
            card.rename_requested.connect(self._on_rename)
            card.duplicate_requested.connect(self._on_duplicate)
            card.delete_requested.connect(self._on_delete)
            card.open_requested.connect(self._on_open)
            card.edit_requested.connect(self.edit_print_requested)
            card.selection_changed.connect(self._update_bulk_bar)
            try:
                objects = read_print_objects(info.get("path", ""))
                polys = object_polylines(
                    objects, self._needle, self._syringe_map)
            except Exception as e:                    # noqa: BLE001
                logger.debug("thumbnail build failed for %s: %s",
                             info.get("name"), e)
                polys = []
            card.set_polylines(polys)
            self._cards.append(card)

        self._count_lbl.setText(f"· {len(self._cards)} saved")
        self._empty_lbl.setVisible(not self._cards)
        self._scroll.setVisible(bool(self._cards))
        self._refresh_cleanup_btn()
        self._update_bulk_bar()
        self._relayout(force=True)

    def _relayout(self, force: bool = False):
        if not self._cards:
            return
        vw = self._scroll.viewport().width()
        card_w = s(_CARD_W) + self._grid.spacing()
        cols = max(1, vw // card_w) if vw > 0 else 3
        if cols == self._cols and not force:
            return
        self._cols = cols
        # Detach then re-add in the new column count.
        while self._grid.count():
            self._grid.takeAt(0)
        for i, card in enumerate(self._cards):
            self._grid.addWidget(card, i // cols, i % cols)

    def _refresh_cleanup_btn(self):
        try:
            n = len(self._mgr.orphan_files())
        except Exception:                             # noqa: BLE001
            n = 0
        self._cleanup_btn.setVisible(n > 0)
        if n > 0:
            self._cleanup_btn.setText(f"Clean up {n} orphan"
                                      + ("" if n == 1 else "s"))

    # ── Selection / bulk ──────────────────────────────────────────

    def _selected_names(self) -> list[str]:
        return [c.name() for c in self._cards if c.is_selected()]

    def _update_bulk_bar(self):
        names = self._selected_names()
        self._bulk_bar.setVisible(bool(names))
        self._bulk_lbl.setText(f"{len(names)} selected")

    def _clear_selection(self):
        for c in self._cards:
            c._check.setChecked(False)
        self._update_bulk_bar()

    def _on_delete_selected(self):
        names = self._selected_names()
        if not names:
            return
        preview = "\n".join(f"  • {n}" for n in names[:12])
        if len(names) > 12:
            preview += f"\n  … and {len(names) - 12} more"
        if QMessageBox.question(
                self, "Delete prints",
                f"Delete {len(names)} print(s) and their files?\n\n{preview}",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        ) != QMessageBox.Yes:
            return
        for n in names:
            try:
                self._mgr.delete(n)
            except Exception as e:                    # noqa: BLE001
                logger.warning("delete %s failed: %s", n, e)
        self._after_change()

    # ── Per-card operations ───────────────────────────────────────

    def _on_open(self, name: str):
        self.print_file_created.emit(name)

    def _on_delete(self, name: str):
        if QMessageBox.question(
                self, "Delete print",
                f"Delete '{name}' and its files (JSON + baked CSV)?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        ) != QMessageBox.Yes:
            return
        try:
            ok = self._mgr.delete(name)
        except Exception as e:                        # noqa: BLE001
            logger.error("delete failed: %s", e)
            ok = False
        if not ok:
            QMessageBox.warning(self, "Delete failed",
                                f"Could not delete '{name}'.")
        self._after_change()

    def _on_rename(self, name: str):
        new_name, ok = QInputDialog.getText(
            self, "Rename print", "New name:", text=name)
        if not ok:
            return
        new_name = (new_name or "").strip()
        if not new_name or new_name == name:
            return
        existing = {i["name"] for i in self._mgr.list_files()} - {name}
        if new_name in existing:
            QMessageBox.warning(
                self, "Name in use",
                f"A print named '{new_name}' already exists.")
            return
        try:
            ok = self._mgr.rename(name, new_name)
        except Exception as e:                        # noqa: BLE001
            logger.error("rename failed: %s", e)
            ok = False
        if not ok:
            QMessageBox.warning(self, "Rename failed",
                                f"Could not rename '{name}'.")
        self._after_change()

    def _on_duplicate(self, name: str):
        default = self._unique_name(f"{name}_copy")
        new_name, ok = QInputDialog.getText(
            self, "Duplicate print", "Name for the copy:", text=default)
        if not ok:
            return
        new_name = (new_name or "").strip()
        if not new_name:
            return
        if new_name in {i["name"] for i in self._mgr.list_files()}:
            QMessageBox.warning(
                self, "Name in use",
                f"A print named '{new_name}' already exists.")
            return
        if not self._duplicate_print(name, new_name):
            QMessageBox.warning(self, "Duplicate failed",
                                f"Could not duplicate '{name}'.")
        self._after_change()

    def _duplicate_print(self, name: str, new_name: str) -> bool:
        """Deep-copy a print under a new name. ``PrintFileManager.duplicate``
        copies the sibling CSV + rewrites pointers so the copy is standalone."""
        if self._mgr.load(name) is None:
            return False
        return self._mgr.duplicate(new_name) is not None

    def _on_open_folder(self):
        try:
            QDesktopServices.openUrl(
                QUrl.fromLocalFile(str(self._mgr.prints_dir.resolve())))
        except Exception as e:                        # noqa: BLE001
            logger.warning("open folder failed: %s", e)

    def _on_cleanup_orphans(self):
        try:
            orphans = self._mgr.orphan_files()
        except Exception:                             # noqa: BLE001
            orphans = []
        if not orphans:
            self._refresh_cleanup_btn()
            return
        names = "\n".join(f"  • {p.name}" for p in orphans[:15])
        if len(orphans) > 15:
            names += f"\n  … and {len(orphans) - 15} more"
        if QMessageBox.question(
                self, "Clean up orphaned files",
                f"Delete {len(orphans)} orphaned file(s) (unreferenced CSVs "
                f"and migration backups)?\n\n{names}",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        ) != QMessageBox.Yes:
            return
        try:
            self._mgr.cleanup_orphans()
        except Exception as e:                        # noqa: BLE001
            logger.warning("cleanup_orphans failed: %s", e)
        self._after_change()

    # ── shared ────────────────────────────────────────────────────

    def _unique_name(self, base: str) -> str:
        existing = {i["name"] for i in self._mgr.list_files()}
        if base not in existing:
            return base
        i = 2
        while f"{base}_{i}" in existing:
            i += 1
        return f"{base}_{i}"

    def _after_change(self):
        self.refresh()
        self.print_files_changed.emit()
