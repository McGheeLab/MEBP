"""Plate / rosette library — a card grid over ``PlateDocumentStore``.

Mirrors ``print_library.py`` so the two libraries read as one system, but the
thumbnail differs in two ways that matter:

* **No hardware config.** A print thumbnail needs a needle and syringes to
  build a trajectory; a plate design is already geometry. Tested as an
  invariant so nobody wires one in.
* **No Y flip.** ``PrintThumbnail`` paints Y-up because print space is Y-up.
  Plate space is Y-DOWN (A1 top-left), so copying that flip would render every
  plate upside-down with A1 in the wrong corner.

Cards carry the **stable document id**, never the display name — renaming a
plate must not change which plate an action targets.

Standard formats appear as read-only cards so a fresh install with an empty
user directory is still fully usable, and so the library can be the only plate
picker. Opening one forks it on first edit (handled by the builder).
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, Signal, QSize, QUrl, QPointF, QRectF
from PySide6.QtGui import (
    QColor, QDesktopServices, QFontMetrics, QImage, QPainter, QPen, QPixmap,
)
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QFrame, QHBoxLayout, QInputDialog,
    QLabel, QListWidget, QListWidgetItem, QMessageBox, QPushButton,
    QSizePolicy, QToolButton, QVBoxLayout, QWidget,
)

from gui.scaling import s, scaled_font_size as _sf
from gui.styles import COLORS
from gui.widgets.card_grid import CARD_W, BulkSelectBar, CardGridView
from gui.widgets.icons import icon
from SupportClasses.PlateDocument import PlateDocument
from SupportClasses.PlateDocumentStore import (
    DEFAULT_PLATES_DIR, DocSummary, PlateDocumentStore,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS

logger = logging.getLogger(__name__)

_THUMB_H = 130
#: Pseudo-ids for the bundled standard formats, which have no document.
STD_PREFIX = "std:"
#: Pseudo-ids for `PlateType` products, which are an overlay on a base format
#: rather than a document — same XY geometry, their own Z offsets, and their
#: OWN `active_plate_key`, which is what gives them a separate mosaic and
#: calibration. (Operator: *"all the plates you are building are the same,
#: they just have different mosaics attached."* Exactly so — for products.)
TYPE_PREFIX = "type:"


def standard_id(fmt: int) -> str:
    return f"{STD_PREFIX}{fmt}"


def is_standard(doc_id: str) -> bool:
    return str(doc_id).startswith(STD_PREFIX)


def standard_format(doc_id: str) -> int:
    return int(str(doc_id)[len(STD_PREFIX):])


def product_id(type_id: str) -> str:
    return f"{TYPE_PREFIX}{type_id}"


def is_product(doc_id: str) -> bool:
    return str(doc_id).startswith(TYPE_PREFIX)


def product_type_id(doc_id: str) -> str:
    return str(doc_id)[len(TYPE_PREFIX):]


def is_bundled(card_id: str) -> bool:
    """Ships with the app: a standard format or a `PlateType` product.

    Neither has a document, so neither can be renamed, edited in place or
    deleted — but both can be activated, duplicated into a real design, and
    (v7.12) hidden.
    """
    return is_standard(card_id) or is_product(card_id)


def base_format_of(card_id: str) -> int:
    """The standard well count a bundled card's geometry comes from."""
    if is_standard(card_id):
        return standard_format(card_id)
    if is_product(card_id):
        try:
            from SupportClasses.PlateTypeStore import get_store
            pt = get_store().get(product_type_id(card_id))
            return int(getattr(pt, "base_format", 0) or 0) if pt else 0
        except Exception:                              # pragma: no cover
            return 0
    return 0


def plate_key_for(card_id: str) -> str:
    """The ``active_plate_key`` a card maps to.

    This is the identity that every per-plate store keys on — mosaic, plate
    template, well training and the archived calibration — so it is also the
    right identity for the library to be a list OF.
    """
    if is_standard(card_id):
        return str(standard_format(card_id))
    if is_product(card_id):
        return product_type_id(card_id)
    return str(card_id)


@dataclass
class _Entry:
    """One offerable plate. ``card_id`` is a card identity; ``plate_key_for``
    turns it into the ``active_plate_key`` the per-plate stores use."""
    card_id: str
    title: str
    meta: str
    read_only: bool
    hideable: bool
    fmt: int
    wells: list
    extent: tuple
    circle: bool = False
    rosettes: set = field(default_factory=set)


def hidden_path(store_dir: Optional[Path] = None) -> Path:
    """Where the hidden-card list lives — **beside the plate documents**.

    Derived from the store's own directory rather than read from the
    environment, so injecting a store (as every test and any alternate
    profile does) isolates this file too. Resolving it globally let a test
    that injected a temp store still write to the operator's real config.
    """
    base = Path(store_dir) if store_dir is not None else Path(
        os.environ.get("MEBP_PLATES_DIR") or DEFAULT_PLATES_DIR)
    return base / "library_hidden.json"


def load_hidden(store_dir: Optional[Path] = None) -> set[str]:
    """Card ids the operator has hidden. Never raises — a bad file just means
    nothing is hidden, which is strictly recoverable."""
    try:
        with open(hidden_path(store_dir), "r", encoding="utf-8") as fh:
            return {str(x) for x in (json.load(fh) or {}).get("hidden", [])}
    except FileNotFoundError:
        return set()
    except Exception as exc:                           # noqa: BLE001
        logger.warning("library_hidden.json unreadable (%s) — showing all", exc)
        return set()


def save_hidden(hidden: set[str],
                store_dir: Optional[Path] = None) -> None:
    path = hidden_path(store_dir)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump({"hidden": sorted(hidden)}, fh, indent=2)
        os.replace(tmp, path)
    except Exception as exc:                           # noqa: BLE001
        logger.warning("could not save library_hidden.json: %s", exc)


# ═══════════════════════════════════════════════════════════════════

def mosaic_pixmap(store, plate_key, max_px: int = 480):
    """The stored scan for *plate_key* as a downscaled ``QPixmap``, or None.

    Downscaled because a full-plate composite is tens of megapixels and this is
    drawn into a card a couple of hundred pixels wide; scaling once here beats
    letting every repaint resample the original (the cost that made the jog
    overlay cache load-bearing rather than an optimisation).
    """
    if store is None or plate_key is None:
        return None
    try:
        img = store.load_image(plate_key)
        if img is None:
            return None
        import numpy as np
        arr = np.ascontiguousarray(img)
        h, w = arr.shape[:2]
        if h < 1 or w < 1:
            return None
        fmt = QImage.Format_BGR888 if arr.ndim == 3 else QImage.Format_Grayscale8
        stride = arr.strides[0]
        qimg = QImage(arr.data, w, h, stride, fmt).copy()
        pm = QPixmap.fromImage(qimg)
        if max(w, h) > max_px:
            pm = pm.scaled(max_px, max_px, Qt.KeepAspectRatio,
                           Qt.SmoothTransformation)
        return pm
    except Exception as exc:                               # noqa: BLE001
        logger.debug("mosaic pixmap failed for %s: %s", plate_key, exc)
        return None


def mosaic_extent_in_doc_frame(store, plate_key, doc):
    """The active scan's extent in *doc*'s storage frame (mm), or None.

    The scan records its plate frame relative to the A1 WELL, while the
    thumbnail draws in the document's storage frame (origin = the boundary's A1
    datum). Those differ whenever a pattern is authored away from the datum, so
    the offset has to be applied rather than assumed zero.

    Returns None for a legacy scan — it has no plate frame, so there is no
    honest way to place it against the outline.
    """
    if store is None or doc is None:
        return None
    try:
        frame = store.plate_frame(plate_key)
        if not frame or "extent_mm" not in frame:
            return None
        from SupportClasses.PlateDocumentStore import a1_well_offset_mm
        dx, dy = a1_well_offset_mm(doc)
        x0, y0, x1, y1 = (float(v) for v in frame["extent_mm"])
        return (x0 + dx, y0 + dy, x1 + dx, y1 + dy)
    except Exception as exc:                               # noqa: BLE001
        logger.debug("mosaic extent failed for %s: %s", plate_key, exc)
        return None


class MosaicPickerDialog(QDialog):
    """Choose, name, import and delete a plate's scanned mosaics — visually.

    v7.13. The previous chooser was a ``QInputDialog`` list of plate KEYS, so
    picking a scan meant recognising it by name; with several scans per plate
    that stops being workable. Here each scan is shown.
    """

    _PREVIEW = 320

    def __init__(self, store, plate_key, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mosaic for this plate")
        self._store = store
        self._key = plate_key
        self.changed = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(8))

        head = QLabel(f"Scans stored for <b>{plate_key}</b>")
        head.setStyleSheet(f"color: {COLORS['text']}; font-size: {_sf(10)}pt;")
        outer.addWidget(head)

        body = QHBoxLayout()
        body.setSpacing(s(10))
        self._list = QListWidget()
        self._list.setMinimumWidth(s(210))
        self._list.currentItemChanged.connect(lambda *_: self._render())
        body.addWidget(self._list, 0)

        right = QVBoxLayout()
        right.setSpacing(s(4))
        self._preview = QLabel()
        self._preview.setAlignment(Qt.AlignCenter)
        self._preview.setMinimumSize(s(self._PREVIEW), s(self._PREVIEW * 2 // 3))
        self._preview.setStyleSheet(
            f"background: {COLORS['crust']}; border: 1px solid "
            f"{COLORS['surface1']};")
        right.addWidget(self._preview, 1)
        self._detail = QLabel()
        self._detail.setWordWrap(True)
        self._detail.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        right.addWidget(self._detail)
        body.addLayout(right, 1)
        outer.addLayout(body)

        row = QHBoxLayout()
        row.setSpacing(s(4))
        for label, tip, slot in (
                ("Use this scan", "Make the selected scan the one this plate "
                                  "uses", self._use),
                ("Rename…", "Rename the selected scan", self._rename),
                ("Import from another plate…",
                 "Copy a scan stored under another plate", self._import),
                ("Delete", "Delete the selected scan", self._delete)):
            b = QPushButton(label)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            row.addWidget(b)
        row.addStretch(1)
        outer.addLayout(row)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(self.reject)
        buttons.accepted.connect(self.accept)
        outer.addWidget(buttons)

        self._reload()

    # ── state ────────────────────────────────────────────────────

    def _reload(self, select_id: str = "") -> None:
        self._list.clear()
        for scan in self._store.list_scans(self._key):
            mark = "●" if scan["active"] else "○"
            warn = "  ⚠ legacy" if scan["needs_rescan"] else ""
            item = QListWidgetItem(f"{mark} {scan['name']}{warn}")
            item.setData(Qt.UserRole, scan["id"])
            item.setToolTip(
                "This scan is referenced to the plate as well as the stage, "
                "so a re-teach carries it with the plate."
                if not scan["needs_rescan"] else
                "Scanned before plate-frame referencing existed. It still "
                "displays, but it cannot follow a re-teach — re-scan this "
                "plate to enable that.")
            self._list.addItem(item)
        if self._list.count():
            target = 0
            if select_id:
                for i in range(self._list.count()):
                    if self._list.item(i).data(Qt.UserRole) == select_id:
                        target = i
                        break
            self._list.setCurrentRow(target)
        self._render()

    def _selected(self) -> str:
        item = self._list.currentItem()
        return str(item.data(Qt.UserRole)) if item is not None else ""

    def _render(self) -> None:
        scan_id = self._selected()
        if not scan_id:
            self._preview.setText("No scan stored for this plate.")
            self._detail.setText(
                "Scan one on Calibration → Plate Location, or import a scan "
                "stored under another plate.")
            return
        active = self._store.active_scan_id(self._key)
        restore = active != scan_id
        try:
            # Preview the SELECTED scan, which means briefly making it active —
            # every reader resolves through the active slot. Restored below so
            # merely looking at a scan never changes which one the plate uses.
            if restore:
                self._store.set_active_scan(self._key, scan_id)
            pm = mosaic_pixmap(self._store, self._key,
                               max_px=s(self._PREVIEW))
            meta = self._store.get_meta(self._key) or {}
            frame = self._store.plate_frame(self._key)
        finally:
            if restore:
                self._store.set_active_scan(self._key, active)

        if pm is None:
            self._preview.setText("(image missing)")
        else:
            self._preview.setPixmap(pm.scaled(
                self._preview.width(), self._preview.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation))
        bits = [f"{meta.get('frames', 0)} tiles",
                meta.get("date", "") or "unknown date"]
        bits.append("plate-referenced" if frame else
                    "⚠ stage-frame only — re-scan to let it follow a re-teach")
        self._detail.setText(" · ".join(b for b in bits if b))

    # ── actions ──────────────────────────────────────────────────

    def _use(self) -> None:
        scan_id = self._selected()
        if scan_id and self._store.set_active_scan(self._key, scan_id):
            self.changed = True
            self._reload(scan_id)

    def _rename(self) -> None:
        scan_id = self._selected()
        if not scan_id:
            return
        current = next((x["name"] for x in self._store.list_scans(self._key)
                        if x["id"] == scan_id), "")
        name, ok = QInputDialog.getText(self, "Rename scan", "Name:",
                                        text=current)
        if ok and name.strip():
            self._store.rename_scan(self._key, scan_id, name.strip())
            self.changed = True
            self._reload(scan_id)

    def _import(self) -> None:
        sources = [k for k in (self._store.list_plate_keys() or [])
                   if k != self._key]
        if not sources:
            QMessageBox.information(self, "Import scan",
                                    "No other plate has a stored scan.")
            return
        src, ok = QInputDialog.getItem(self, "Import scan",
                                       "Copy the scan from:", sources, 0, False)
        if not ok:
            return
        if self._store.copy_plate(src, self._key) > 0:
            self.changed = True
            self._reload()
        else:
            QMessageBox.warning(self, "Import scan",
                                f"Could not copy the scan from '{src}'.")

    def _delete(self) -> None:
        scan_id = self._selected()
        if not scan_id:
            return
        name = next((x["name"] for x in self._store.list_scans(self._key)
                     if x["id"] == scan_id), scan_id)
        if QMessageBox.question(
                self, "Delete scan",
                f"Delete '{name}'? The image file is removed.") \
                != QMessageBox.Yes:
            return
        if self._store.delete_scan(self._key, scan_id):
            self.changed = True
            self._reload()


class PlateThumbnail(QWidget):
    """Auto-fit outline + well circles. Needs no HardwareConfig."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._wells: list[tuple[float, float, float]] = []
        self._rosettes: set[int] = set()
        self._extent: tuple[float, float, float, float] | None = None
        self._circle = False
        self._placeholder = "empty plate"
        # v7.13: the scan itself, drawn under the wells at its own extent, so
        # the operator picks a mosaic by looking at it rather than by reading a
        # key. Extent is in the SAME storage frame as `_extent`/`_wells`.
        self._mosaic: QPixmap | None = None
        self._mosaic_extent: tuple[float, float, float, float] | None = None
        self.setMinimumHeight(s(_THUMB_H))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_geometry(self, wells, extent, circle=False, rosettes=()):
        self._wells = list(wells or [])
        self._extent = extent
        self._circle = bool(circle)
        self._rosettes = set(rosettes or ())
        self.update()

    def set_mosaic(self, pixmap, extent_mm) -> None:
        """Show *pixmap* behind the wells, spanning *extent_mm*.

        Pass ``(None, None)`` to clear. A mosaic whose extent is unknown is NOT
        drawn: stretching it to the plate outline would show the operator a
        registration that does not exist.
        """
        self._mosaic = pixmap
        self._mosaic_extent = (tuple(float(v) for v in extent_mm)
                               if (pixmap is not None and extent_mm is not None
                                   and len(extent_mm) >= 4) else None)
        if self._mosaic_extent is None:
            self._mosaic = None
        self.update()

    def set_placeholder(self, text: str):
        self._placeholder = text
        self._wells = []
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = self.rect()
        p.fillRect(rect, QColor(COLORS["crust"]))
        p.setPen(QPen(QColor(COLORS["surface1"]), 1))
        p.drawRect(rect.adjusted(0, 0, -1, -1))

        if not self._wells or self._extent is None:
            p.setPen(QColor(COLORS["overlay0"]))
            f = p.font()
            f.setPointSize(_sf(8))
            p.setFont(f)
            p.drawText(rect, Qt.AlignCenter, self._placeholder)
            return

        # Fit the OUTLINE, not the content, so two 24-well plates render at
        # the same scale and the grid reads as a catalogue.
        x0, y0, x1, y1 = self._extent
        span_x = max(x1 - x0, 1e-6)
        span_y = max(y1 - y0, 1e-6)
        pad = s(9)
        scale = min((rect.width() - 2 * pad) / span_x,
                    (rect.height() - 2 * pad) / span_y)
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0

        def to_px(x, y):
            # NO Y flip: plate space is Y-down, so +Y must paint downward.
            return (rect.width() / 2.0 + (x - cx) * scale,
                    rect.height() / 2.0 + (y - cy) * scale)

        # v7.13: the scan, under everything else and clipped to the card, at
        # its own extent — so a mosaic that does not cover the whole plate
        # visibly does not, instead of being stretched to fit.
        if self._mosaic is not None and self._mosaic_extent is not None:
            mx0, my0, mx1, my1 = self._mosaic_extent
            tl = to_px(mx0, my0)
            br = to_px(mx1, my1)
            target = QRectF(QPointF(*tl), QPointF(*br)).normalized()
            if target.width() >= 1.0 and target.height() >= 1.0:
                p.save()
                p.setClipRect(QRectF(rect))
                p.setOpacity(0.85)
                p.drawPixmap(target, self._mosaic,
                             QRectF(self._mosaic.rect()))
                p.restore()

        ol = QPen(QColor(COLORS.get("overlay0", "#6c7086")))
        ol.setWidthF(1.2)
        ol.setCosmetic(True)
        p.setPen(ol)
        tlx, tly = to_px(x0, y0)
        brx, bry = to_px(x1, y1)
        if self._circle:
            p.drawEllipse(int(tlx), int(tly), int(brx - tlx), int(bry - tly))
        else:
            p.drawRect(int(tlx), int(tly), int(brx - tlx), int(bry - tly))

        wp = QPen(QColor(COLORS.get("blue", "#89b4fa")))
        wp.setWidthF(0.9)
        wp.setCosmetic(True)
        ros_col = QColor(COLORS.get("teal", "#94e2d5"))
        for i, (wx, wy, dia) in enumerate(self._wells):
            px, py = to_px(wx, wy)
            r = max(dia / 2.0 * scale, 0.7)
            p.setPen(wp)
            p.drawEllipse(int(px - r), int(py - r), int(2 * r), int(2 * r))
            if i in self._rosettes:
                rr = max(r * 0.32, 1.0)
                p.setPen(Qt.NoPen)
                p.setBrush(ros_col)
                p.drawEllipse(int(px - rr), int(py - rr),
                              int(2 * rr), int(2 * rr))
                p.setBrush(Qt.NoBrush)


# ═══════════════════════════════════════════════════════════════════

class PlateCard(QFrame):
    """One library entry. Every signal carries the stable document id."""

    open_requested = Signal(str)
    rename_requested = Signal(str)
    duplicate_requested = Signal(str)
    delete_requested = Signal(str)
    activate_requested = Signal(str)
    mosaic_requested = Signal(str)
    hide_requested = Signal(str)
    selection_changed = Signal()

    def __init__(self, doc_id: str, title: str, meta: str, *,
                 read_only: bool = False, is_active: bool = False,
                 mosaic: str = "", hideable: bool = False,
                 hidden: bool = False, parent: QWidget | None = None):
        super().__init__(parent)
        self._id = doc_id
        self._read_only = read_only
        self.setObjectName("plateCard")
        self.setFixedWidth(s(CARD_W))
        accent = COLORS["mauve"] if is_active else COLORS["surface1"]
        self.setStyleSheet(f"""
            #plateCard {{
                background: {COLORS['mantle']};
                border: {2 if is_active else 1}px solid {accent};
                border-radius: {s(8)}px;
            }}
            #plateCard QLabel {{ background: transparent; }}
        """)
        v = QVBoxLayout(self)
        v.setContentsMargins(s(8), s(8), s(8), s(8))
        v.setSpacing(s(5))

        self._thumb = PlateThumbnail(self)
        v.addWidget(self._thumb)

        head = QHBoxLayout()
        head.setSpacing(s(4))
        name_lbl = QLabel()
        name_lbl.setToolTip(title)
        name_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {_sf(10)}pt; "
            f"font-weight: 700;")
        fm = QFontMetrics(name_lbl.font())
        name_lbl.setText(fm.elidedText(title, Qt.ElideRight, s(CARD_W - 46)))
        head.addWidget(name_lbl, 1)
        if is_active:
            star = QLabel("★")
            star.setToolTip("Active in this hardware setup")
            star.setStyleSheet(f"color: {COLORS['mauve']}; "
                               f"font-size: {_sf(11)}pt;")
            head.addWidget(star)
        v.addLayout(head)

        meta_lbl = QLabel(meta)
        meta_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        meta_lbl.setWordWrap(True)
        v.addWidget(meta_lbl)

        # Mosaic status. The operator's own framing is that these plates are
        # geometrically the same and it is the attached mosaic that
        # distinguishes them — so it belongs on the face of the card, not
        # three tabs away.
        self._mosaic_lbl = QLabel()
        self._mosaic_lbl.setWordWrap(True)
        self._mosaic_lbl.setStyleSheet(
            f"color: {COLORS['green'] if mosaic else COLORS['overlay0']}; "
            f"font-size: {_sf(8)}pt;")
        self._mosaic_lbl.setText(f"◉ mosaic · {mosaic}" if mosaic
                                 else "○ no mosaic")
        self._mosaic_lbl.setToolTip(
            "Scanned mosaic for this plate — click ⛶ to reuse another "
            "plate's scan." if mosaic else
            "No mosaic scanned for this plate yet. Scan one on "
            "Calibration → Plate Location, or click ⛶ to reuse another "
            "plate's.")
        v.addWidget(self._mosaic_lbl)

        row = QHBoxLayout()
        row.setSpacing(s(3))
        self._check = QCheckBox()
        self._check.setToolTip("Select for bulk delete")
        self._check.setEnabled(not read_only)
        self._check.toggled.connect(lambda _c: self.selection_changed.emit())
        row.addWidget(self._check)
        row.addStretch(1)
        row.addWidget(self._icon_btn(
            "target", "Use this plate in the hardware setup",
            lambda: self.activate_requested.emit(self._id)))
        row.addWidget(self._icon_btn(
            "grid", "Choose which scanned mosaic this plate uses",
            lambda: self.mosaic_requested.emit(self._id)))
        row.addWidget(self._icon_btn(
            "pencil", "Rename" if not read_only
            else "Bundled plates cannot be renamed",
            lambda: self.rename_requested.emit(self._id),
            enabled=not read_only))
        row.addWidget(self._icon_btn(
            "clipboard",
            "Duplicate to my plates" if read_only else "Duplicate",
            lambda: self.duplicate_requested.emit(self._id)))
        if hideable:
            # Bundled entries cannot be deleted, but the operator does not
            # want a library full of products for formats they never run.
            row.addWidget(self._icon_btn(
                "plus" if hidden else "minus",
                "Show in the library again" if hidden
                else "Hide from the library (does not delete it)",
                lambda: self.hide_requested.emit(self._id)))
        else:
            row.addWidget(self._icon_btn(
                "trash", "Delete",
                lambda: self.delete_requested.emit(self._id),
                danger=True, enabled=not read_only))
        v.addLayout(row)

        openb = QPushButton("✎  Open" if not read_only else "✎  Open a copy")
        openb.setCursor(Qt.PointingHandCursor)
        openb.setStyleSheet(f"""
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
        openb.clicked.connect(lambda: self.open_requested.emit(self._id))
        v.addWidget(openb)

    def mouseDoubleClickEvent(self, event):
        self.open_requested.emit(self._id)
        super().mouseDoubleClickEvent(event)

    # ── API ───────────────────────────────────────────────────────

    def doc_id(self) -> str:
        return self._id

    def is_selected(self) -> bool:
        return self._check.isChecked()

    def set_selected(self, on: bool) -> None:
        self._check.setChecked(bool(on))

    def thumbnail(self) -> PlateThumbnail:
        return self._thumb

    def _icon_btn(self, ico, tip, slot, danger=False,
                  enabled=True) -> QToolButton:
        b = QToolButton()
        b.setToolTip(tip)
        b.setFixedSize(s(26), s(26))
        b.setEnabled(enabled)
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
            QToolButton:disabled {{ background: {COLORS['mantle']}; }}
        """)
        b.clicked.connect(slot)
        return b


# ═══════════════════════════════════════════════════════════════════

class PlateLibraryPage(QWidget):
    """Grid of every plate (or rosette) with file-management operations."""

    open_requested = Signal(str)      # doc id → host enters the builder
    library_changed = Signal()        # any create/rename/duplicate/delete
    active_changed = Signal(str)      # ★ → host writes HardwareConfig

    def __init__(self, kind: str = "plate", store: PlateDocumentStore | None = None,
                 plates_dir: str | Path | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.kind = kind
        self._store = store or PlateDocumentStore(
            kind=kind, user_dir=Path(plates_dir) if plates_dir else None)
        self._active_id = ""
        self._format_filter = 0        # 0 = all
        self._hidden: set[str] = (load_hidden(self._store.user_dir)
                                  if kind == "plate" else set())
        self._show_hidden = False
        self._cards: list[PlateCard] = []
        self._build_ui()
        self.refresh()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(4), s(4), s(4), s(4))
        outer.setSpacing(s(6))

        head = QHBoxLayout()
        head.setSpacing(s(6))
        title = QLabel("Plates" if self.kind == "plate" else "Rosettes")
        title.setStyleSheet(f"color: {COLORS['text']}; "
                            f"font-size: {_sf(13)}pt; font-weight: 700;")
        head.addWidget(title)
        self._count_lbl = QLabel("")
        self._count_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        head.addWidget(self._count_lbl)
        head.addStretch(1)

        self._new_btn = self._text_btn("＋ New", self._on_new)
        head.addWidget(self._new_btn)
        head.addWidget(self._text_btn("Open folder", self._on_open_folder))
        head.addWidget(self._text_btn("↻ Refresh", self.refresh))
        outer.addLayout(head)

        if self.kind == "plate":
            outer.addLayout(self._build_filter_row())

        self._bulk = BulkSelectBar()
        self._bulk.clear_requested.connect(self._clear_selection)
        self._bulk.delete_requested.connect(self._on_delete_selected)
        outer.addWidget(self._bulk)

        self._grid = CardGridView()
        self._grid.set_empty_text(
            "No plates yet.\n\nStandard formats are always available above — "
            "open one to start from it, or press ＋ New for a blank plate."
            if self.kind == "plate" else
            "No rosettes yet.\n\nPress ＋ New to design one, then place it "
            "into wells on the Layout tab.")
        outer.addWidget(self._grid, 1)

    def _build_filter_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(4))
        row.addWidget(QLabel("Format:"))
        self._filter_btns: dict[int, QToolButton] = {}
        for fmt in [0] + sorted(PLATE_DEFINITIONS.keys()):
            b = QToolButton()
            b.setText("All" if fmt == 0 else str(fmt))
            b.setCheckable(True)
            b.setChecked(fmt == 0)
            b.setCursor(Qt.PointingHandCursor)
            b.setStyleSheet(f"""
                QToolButton {{
                    background: {COLORS['surface0']}; color: {COLORS['text']};
                    border: 1px solid {COLORS['surface1']};
                    border-radius: {s(9)}px;
                    padding: {s(2)}px {s(9)}px; font-size: {_sf(8)}pt;
                }}
                QToolButton:checked {{
                    background: {COLORS['mauve']}; color: {COLORS['crust']};
                }}
            """)
            b.clicked.connect(lambda _c=False, f=fmt: self._set_filter(f))
            self._filter_btns[fmt] = b
            row.addWidget(b)
        self._hidden_btn = QToolButton()
        self._hidden_btn.setCheckable(True)
        self._hidden_btn.setCursor(Qt.PointingHandCursor)
        self._hidden_btn.setToolTip(
            "Show the plates you have hidden, so you can bring one back.")
        self._hidden_btn.setStyleSheet(f"""
            QToolButton {{
                background: {COLORS['surface0']}; color: {COLORS['subtext0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: {s(9)}px;
                padding: {s(2)}px {s(9)}px; font-size: {_sf(8)}pt;
            }}
            QToolButton:checked {{
                background: {COLORS['overlay0']}; color: {COLORS['text']};
            }}
        """)
        self._hidden_btn.setVisible(False)
        self._hidden_btn.clicked.connect(self._toggle_hidden)
        row.addWidget(self._hidden_btn)
        row.addStretch(1)
        self._active_lbl = QLabel("")
        self._active_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        # Ignored + a zero minimum: a plate name is arbitrarily long, and
        # without this the whole tab inherits a minimum width that grows with
        # whatever the operator called their plate.
        self._active_lbl.setSizePolicy(QSizePolicy.Ignored,
                                       QSizePolicy.Preferred)
        self._active_lbl.setMinimumWidth(0)
        row.addWidget(self._active_lbl)
        return row

    def _set_active_label(self, text: str) -> None:
        self._active_lbl.setToolTip(text)
        fm = QFontMetrics(self._active_lbl.font())
        width = max(self._active_lbl.width(), s(60))
        self._active_lbl.setText(fm.elidedText(text, Qt.ElideRight, width))

    def _toggle_hidden(self) -> None:
        self._show_hidden = not self._show_hidden
        self.refresh()

    def _text_btn(self, text, slot, danger=False) -> QPushButton:
        b = QPushButton(text)
        b.setCursor(Qt.PointingHandCursor)
        if danger:
            b.setStyleSheet(
                f"QPushButton {{ color: {COLORS.get('red', '#f38ba8')}; }}")
        b.clicked.connect(slot)
        return b

    # ── State ─────────────────────────────────────────────────────

    def store(self) -> PlateDocumentStore:
        return self._store

    def set_active_id(self, doc_id: str) -> None:
        if str(doc_id or "") != self._active_id:
            self._active_id = str(doc_id or "")
            self.refresh()

    def active_id(self) -> str:
        return self._active_id

    def _set_filter(self, fmt: int) -> None:
        self._format_filter = fmt
        for f, b in self._filter_btns.items():
            b.setChecked(f == fmt)
        self.refresh()

    # ── Refresh ───────────────────────────────────────────────────

    def _entries(self) -> list[_Entry]:
        """Every plate the library can offer, one per ``active_plate_key``.

        Three sources, because that is what ``active_plate_key`` resolves over:
        bundled **standard formats**, **`PlateType` products** (same geometry
        as their base format, own Z offsets, own mosaic) and saved
        **documents**. Products had no card at all before, which — with the
        Format→Type card retired in Phase 5 — left no way to select one.
        """
        out: list[_Entry] = []
        if self.kind != "plate":
            for summary in self._safe_list():
                doc = self._store.get(summary.id)
                out.append(self._doc_entry(summary, doc))
            return out

        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            try:
                doc = PlateDocument.from_standard_format(fmt)
            except Exception:                          # pragma: no cover
                continue
            wells = doc.evaluate()
            out.append(_Entry(
                card_id=standard_id(fmt), title=f"{fmt}-well (standard)",
                meta=f"{len(wells)} wells · ANSI/SLAS · bundled",
                read_only=True, hideable=True, fmt=fmt,
                wells=[(w.x, w.y, w.diameter_mm) for w in wells],
                extent=doc.boundary.extent_a1()))

        for ptype in self._plate_types():
            base = int(getattr(ptype, "base_format", 0) or 0)
            if base not in PLATE_DEFINITIONS:
                continue
            try:
                doc = PlateDocument.from_standard_format(base)
            except Exception:                          # pragma: no cover
                continue
            wells = doc.evaluate()
            out.append(_Entry(
                card_id=product_id(ptype.id), title=ptype.display_name,
                meta=f"{len(wells)} wells · {base}-well geometry · product",
                read_only=True, hideable=True, fmt=base,
                wells=[(w.x, w.y, w.diameter_mm) for w in wells],
                extent=doc.boundary.extent_a1()))

        for summary in self._safe_list():
            out.append(self._doc_entry(summary, self._store.get(summary.id)))
        return out

    def _doc_entry(self, summary: DocSummary, doc) -> _Entry:
        wells = doc.evaluate() if doc else []
        return _Entry(
            card_id=summary.id, title=summary.label,
            meta=self._meta_text(summary), read_only=False, hideable=False,
            fmt=int(summary.well_count or 0),
            wells=[(w.x, w.y, w.diameter_mm) for w in wells],
            extent=doc.boundary.extent_a1() if doc else (0.0, 0.0, 1.0, 1.0),
            circle=bool(doc and doc.boundary.is_circle()),
            rosettes={i for i, w in enumerate(wells) if w.rosette is not None})

    @staticmethod
    def _plate_types() -> list:
        try:
            from SupportClasses.PlateTypeStore import get_store
            return list(get_store().all())
        except Exception as exc:                       # noqa: BLE001
            logger.debug("plate types unavailable: %s", exc)
            return []

    def _mosaic_label(self, card_id: str) -> str:
        """A short description of the scan attached to this plate's key."""
        try:
            from SupportClasses.MosaicStore import get_store
            store = get_store()
            key = plate_key_for(card_id)
            if not store.has(key):
                return ""
            scans = store.list_scans(key) or []
            active = next((x for x in scans if x["active"]), None)
            label = (active or {}).get("name") or key
            # v7.13: say how many scans there are to choose between, and flag a
            # pre-v7.13 scan — it still displays, but it cannot follow a
            # re-teach, and the only way to fix that is to scan again.
            if len(scans) > 1:
                label += f"  ({len(scans)} scans)"
            n = len(store.list_well_keys(key) or [])
            if n:
                label += f" (+{n} well scans)"
            if store.needs_rescan(key):
                label += "  ⚠ re-scan to track the plate"
            return label
        except Exception as exc:                       # noqa: BLE001
            logger.debug("mosaic status unavailable for %s: %s", card_id, exc)
            return ""

    def refresh(self) -> None:
        self._store.invalidate()
        all_entries = self._entries()
        hidden = self._hidden
        shown = [
            e for e in all_entries
            if (not self._format_filter or e.fmt == self._format_filter)
            # The active plate is never hidden: an invisible ★ would leave the
            # operator unable to see what the setup is actually using.
            and (self._show_hidden or e.card_id not in hidden
                 or e.card_id == self._active_id)
        ]

        cards: list[PlateCard] = []
        for e in shown:
            card = PlateCard(
                e.card_id, e.title, e.meta, read_only=e.read_only,
                is_active=(e.card_id == self._active_id),
                mosaic=self._mosaic_label(e.card_id),
                hideable=e.hideable and e.card_id != self._active_id,
                hidden=e.card_id in hidden)
            card.open_requested.connect(self.open_requested)
            card.rename_requested.connect(self._on_rename)
            card.duplicate_requested.connect(self._on_duplicate)
            card.delete_requested.connect(self._on_delete)
            card.activate_requested.connect(self._on_activate)
            card.mosaic_requested.connect(self._on_choose_mosaic)
            card.hide_requested.connect(self._on_hide)
            card.selection_changed.connect(self._update_bulk)
            try:
                card.thumbnail().set_geometry(e.wells, e.extent, e.circle,
                                              e.rosettes)
                # v7.13: show the actual scan behind the wells.
                self._apply_card_mosaic(card, e)
            except Exception as exc:                   # noqa: BLE001
                logger.debug("thumbnail failed for %s: %s", e.card_id, exc)
            cards.append(card)

        self._cards = cards
        self._grid.set_cards(cards)
        n_user = sum(1 for e in all_entries if not e.read_only)
        self._count_lbl.setText(f"· {n_user} saved")
        n_hidden = sum(1 for e in all_entries if e.card_id in hidden)
        if hasattr(self, "_hidden_btn"):
            self._hidden_btn.setVisible(bool(n_hidden))
            self._hidden_btn.setChecked(self._show_hidden)
            self._hidden_btn.setText(
                f"Hidden ({n_hidden})" if not self._show_hidden
                else f"Hiding off ({n_hidden})")
        if self.kind == "plate" and hasattr(self, "_active_lbl"):
            name = next((e.title for e in all_entries
                         if e.card_id == self._active_id), "")
            self._set_active_label(f"Active in setup: {name}" if name
                                   else "⚠ No plate selected")
        self._update_bulk()

    def _apply_card_mosaic(self, card, entry) -> None:
        """Draw the plate's active scan into its card thumbnail."""
        store = self._mosaic_store()
        if store is None:
            return
        key = plate_key_for(entry.card_id)
        pm = mosaic_pixmap(store, key)
        if pm is None:
            return
        doc = None
        try:
            doc = self._store.get(entry.card_id)
        except Exception:                              # noqa: BLE001
            doc = None
        if doc is None:
            try:
                doc = PlateDocument.from_standard_format(int(entry.fmt))
            except Exception:                          # noqa: BLE001
                doc = None
        extent = mosaic_extent_in_doc_frame(store, key, doc)
        card.thumbnail().set_mosaic(pm, extent)

    @staticmethod
    def _mosaic_store():
        try:
            from SupportClasses.MosaicStore import get_store
            return get_store()
        except Exception as exc:                       # noqa: BLE001
            logger.debug("MosaicStore unavailable: %s", exc)
            return None

    def _safe_list(self) -> list[DocSummary]:
        try:
            return self._store.list()
        except Exception as exc:                       # noqa: BLE001
            logger.warning("Plate library listing failed: %s", exc)
            return []

    @staticmethod
    def _meta_text(summary: DocSummary) -> str:
        parts = [f"{summary.well_count} well"
                 + ("" if summary.well_count == 1 else "s")]
        if summary.rosette_count:
            parts.append(f"{summary.rosette_count} rosette"
                         + ("" if summary.rosette_count == 1 else "s"))
        when = ""
        try:
            when = datetime.fromisoformat(summary.modified).strftime(
                "%Y-%m-%d %H:%M")
        except (ValueError, TypeError):
            when = (summary.modified or "")[:16]
        line = " · ".join(parts)
        return f"{line}\n{when}" if when else line

    # ── Selection ─────────────────────────────────────────────────

    def _selected_ids(self) -> list[str]:
        return [c.doc_id() for c in self._cards if c.is_selected()]

    def _update_bulk(self) -> None:
        self._bulk.set_count(len(self._selected_ids()))

    def _clear_selection(self) -> None:
        for c in self._cards:
            c.set_selected(False)
        self._update_bulk()

    # ── Actions ───────────────────────────────────────────────────

    def _on_new(self) -> None:
        label = "plate" if self.kind == "plate" else "rosette"
        name, ok = QInputDialog.getText(self, f"New {label}",
                                        f"{label.capitalize()} name:",
                                        text=f"New {label}")
        if not ok or not name.strip():
            return
        template = 24 if self.kind == "plate" else None
        doc = self._store.create(name.strip(), template=template)
        self._after_change()
        self.open_requested.emit(doc.meta.id)

    def _on_activate(self, doc_id: str) -> None:
        self._active_id = doc_id
        self.active_changed.emit(doc_id)
        self.refresh()

    def _on_hide(self, doc_id: str) -> None:
        """Toggle a bundled plate in or out of the library, without deleting.

        Standards and products are shipped, so they cannot be deleted — but
        the operator does not want cards for formats and products they never
        run. While "Hidden (n)" is on, the same button puts one back.
        """
        if doc_id == self._active_id:
            return
        if doc_id in self._hidden:
            self._hidden.discard(doc_id)
        else:
            self._hidden.add(doc_id)
        save_hidden(self._hidden, self._store.user_dir)
        self.refresh()

    def _on_choose_mosaic(self, doc_id: str) -> None:
        """Point this plate at a scanned mosaic.

        Operator: *"all the plates you are building are the same, they just
        have different mosaics attached."* True for standards and products —
        identical geometry, separate ``active_plate_key``, separate scan — so
        being able to reuse one plate's scan on another is the difference
        between a two-minute switch and a re-scan.
        """
        store = self._mosaic_store()
        if store is None:
            QMessageBox.warning(self, "Mosaics",
                                "The mosaic store is unavailable.")
            return
        dlg = MosaicPickerDialog(store, plate_key_for(doc_id), self)
        if dlg.exec() == QDialog.Accepted or dlg.changed:
            self.refresh()
            self.library_changed.emit()

    def _on_rename(self, doc_id: str) -> None:
        if is_bundled(doc_id):
            return
        doc = self._store.get(doc_id)
        if doc is None:
            return
        name, ok = QInputDialog.getText(self, "Rename", "New name:",
                                        text=doc.meta.name)
        if not ok or not name.strip() or name.strip() == doc.meta.name:
            return
        clash = self._store.find_by_name(name.strip())
        if clash is not None and clash.id != doc_id:
            # Names need not be unique — only ids — so this informs, not blocks.
            answer = QMessageBox.question(
                self, "Rename",
                f"Another {self.kind} is also called '{name.strip()}'. "
                f"Continue?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if answer != QMessageBox.Yes:
                return
        self._store.rename(doc_id, name.strip())
        self._after_change()

    def _on_duplicate(self, doc_id: str) -> None:
        if is_bundled(doc_id):
            fmt = base_format_of(doc_id)
            label = next((e.title for e in self._entries()
                          if e.card_id == doc_id), f"{fmt}-well")
            suggested = f"{label} copy"
        else:
            src = self._store.get(doc_id)
            suggested = f"{src.meta.name} copy" if src else "copy"
        name, ok = QInputDialog.getText(self, "Duplicate", "New name:",
                                        text=suggested)
        if not ok or not name.strip():
            return
        if is_bundled(doc_id):
            new = self._store.create(name.strip(),
                                     template=base_format_of(doc_id) or None)
            # A product's identity is its type id: forking it keeps the Z
            # offsets and the product's own per-plate stores pointed at it.
            if is_product(doc_id):
                new.meta.plate_type_id = product_type_id(doc_id)
                self._store.save(new)
        else:
            new = self._store.duplicate(doc_id, name.strip())
        self._after_change()
        self.open_requested.emit(new.meta.id)

    def _on_delete(self, doc_id: str) -> None:
        if is_bundled(doc_id):
            return
        doc = self._store.get(doc_id)
        label = doc.meta.name if doc else doc_id
        answer = QMessageBox.question(
            self, "Delete",
            f"Delete '{label}'?\n\nCalibration, mosaics and well training "
            f"taught for this {self.kind} are keyed to it and will no longer "
            f"be reachable.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        self._store.delete(doc_id)
        self._after_change()

    def _on_delete_selected(self) -> None:
        ids = [i for i in self._selected_ids() if not is_bundled(i)]
        if not ids:
            return
        names = []
        for i in ids[:12]:
            d = self._store.get(i)
            names.append(d.meta.name if d else i)
        more = f"\n… and {len(ids) - 12} more" if len(ids) > 12 else ""
        answer = QMessageBox.question(
            self, "Delete selected",
            f"Delete {len(ids)} {self.kind}(s)?\n\n"
            + "\n".join(names) + more,
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        for i in ids:
            try:
                self._store.delete(i)
            except Exception as exc:                   # noqa: BLE001
                logger.warning("Could not delete %s: %s", i, exc)
        self._after_change()

    def _on_open_folder(self) -> None:
        try:
            self._store.user_dir.mkdir(parents=True, exist_ok=True)
            QDesktopServices.openUrl(
                QUrl.fromLocalFile(str(self._store.user_dir.resolve())))
        except Exception as exc:                       # noqa: BLE001
            logger.warning("Could not open plates folder: %s", exc)

    def _after_change(self) -> None:
        """Single funnel — keeps the count, the bulk bar and the grid in step."""
        self.refresh()
        self.library_changed.emit()

    def get_page_title(self) -> str:
        return "Plates" if self.kind == "plate" else "Rosettes"
