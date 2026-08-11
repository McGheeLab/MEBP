"""Hardware Setup → Plate: a library that drills into a builder.

Two collections (plates, rosettes) behind one segmented pill, each with its own
library and a shared builder. This is the operator's "the plate builder and
rosette builder can be on the same tab, but be different modes", with the Plate
tab opening as the library grid.

Switching the pill while in the builder returns to the library for the other
collection — after the unsaved-changes guard. Swapping the document under the
canvas mid-edit is how the old designer lost work.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QStackedWidget, QVBoxLayout, QWidget,
)

from gui.scaling import s
from gui.widgets.context_panel_host import SegmentedPillBar
from gui.pages.hardware.plate_builder import PlateBuilderPage
from gui.pages.hardware.plate_library import (
    PlateLibraryPage, base_format_of, is_bundled, is_product, is_standard,
    product_type_id, standard_format,
)
from SupportClasses.PlateDocument import PlateDocument
from SupportClasses.PlateDocumentStore import PlateDocumentStore

logger = logging.getLogger(__name__)

_LIBRARY, _BUILDER = 0, 1


class PlateWorkspacePage(QWidget):
    """Library ⇄ builder, for plates and rosettes."""

    active_plate_changed = Signal(str)   # doc id → host writes HardwareConfig
    library_changed = Signal()

    def __init__(self, plate_store: PlateDocumentStore | None = None,
                 rosette_store: PlateDocumentStore | None = None,
                 plates_dir: str | Path | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        root = Path(plates_dir) if plates_dir else None
        self._stores = {
            "plate": plate_store or PlateDocumentStore(
                kind="plate", user_dir=(root / "plates") if root else None),
            "rosette": rosette_store or PlateDocumentStore(
                kind="rosette", user_dir=(root / "rosettes") if root else None),
        }
        self._mode = "plate"
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(s(4))

        bar = QHBoxLayout()
        bar.setContentsMargins(s(6), s(4), s(6), 0)
        self._pills = SegmentedPillBar()
        self._pills.add_pill("plate", "Plates")
        self._pills.add_pill("rosette", "Rosettes")
        self._pills.set_selected("plate")
        self._pills.selected.connect(self._on_mode)
        bar.addWidget(self._pills)
        bar.addStretch(1)
        outer.addLayout(bar)

        self._stack = QStackedWidget()
        self._libs: dict[str, PlateLibraryPage] = {}
        lib_host = QStackedWidget()
        for kind in ("plate", "rosette"):
            lib = PlateLibraryPage(kind=kind, store=self._stores[kind])
            lib.open_requested.connect(
                lambda doc_id, k=kind: self._open(k, doc_id))
            lib.active_changed.connect(self._on_active_changed)
            lib.library_changed.connect(self.library_changed)
            self._libs[kind] = lib
            lib_host.addWidget(lib)
        self._lib_host = lib_host
        self._stack.addWidget(lib_host)

        self._builder = PlateBuilderPage(store=self._stores["plate"])
        self._builder.back_requested.connect(self._show_library)
        self._builder.document_saved.connect(self._on_saved)
        self._stack.addWidget(self._builder)
        outer.addWidget(self._stack, 1)

    # ── Mode ──────────────────────────────────────────────────────

    def _on_mode(self, name: str) -> None:
        if name == self._mode:
            return
        if self._stack.currentIndex() == _BUILDER \
                and not self._builder.maybe_discard():
            self._pills.set_selected(self._mode)     # revert the pill
            return
        self._mode = name
        self._lib_host.setCurrentIndex(0 if name == "plate" else 1)
        self._libs[name].refresh()
        self._show_library()

    def mode(self) -> str:
        return self._mode

    # ── Navigation ────────────────────────────────────────────────

    def _show_library(self) -> None:
        self._stack.setCurrentIndex(_LIBRARY)

    def _open(self, kind: str, doc_id: str) -> None:
        store = self._stores[kind]
        self._builder.set_store(store)
        if is_bundled(doc_id):
            fmt = base_format_of(doc_id)
            label = (f"{fmt}-well" if is_standard(doc_id)
                     else product_type_id(doc_id))
            doc = PlateDocument.from_standard_format(
                fmt or 24, name=f"Copy of {label}")
            if is_product(doc_id):
                # Keep the product identity on the fork, so its Z offsets and
                # its per-plate stores still resolve.
                doc.meta.plate_type_id = product_type_id(doc_id)
            self._builder.load(
                doc, forked=True,
                fork_note=f"Editing the bundled '{label}' plate — your "
                          f"changes will be saved as a new plate "
                          f"'{doc.meta.name}'.")
        else:
            doc = store.get(doc_id)
            if doc is None:
                logger.warning("Cannot open missing document %s", doc_id)
                return
            self._builder.load(doc)
        self._stack.setCurrentIndex(_BUILDER)

    def _on_saved(self, doc_id: str) -> None:
        self._libs[self._mode].refresh()
        self.library_changed.emit()

    def _on_active_changed(self, doc_id: str) -> None:
        """A card's ★ was pressed — tell the host, standards included.

        This used to drop standard formats on the floor (``and not
        is_standard(...)``), on the reasoning that a bundled format has no
        document to point ``plate_doc_id`` at. But a fresh install's library
        contains *nothing but* standards, so the effect was that pressing "Use
        this plate in the hardware setup" did nothing at all and there was no
        way to choose a plate. A standard is carried by ``plate_format``
        instead — the last leg of ``active_plate_key``'s precedence — and the
        host is what knows that, so the decision belongs there, not here.
        """
        if self._mode == "plate":
            self.active_plate_changed.emit(doc_id)

    # ── Host contract ─────────────────────────────────────────────

    def set_active_plate_id(self, doc_id: str) -> None:
        self._libs["plate"].set_active_id(doc_id or "")

    def maybe_discard(self) -> bool:
        """Called by the host before switching sub-page."""
        if self._stack.currentIndex() == _BUILDER:
            return self._builder.maybe_discard()
        return True

    def refresh(self) -> None:
        self._libs[self._mode].refresh()

    def plate_store(self) -> PlateDocumentStore:
        return self._stores["plate"]

    def rosette_store(self) -> PlateDocumentStore:
        return self._stores["rosette"]

    def builder(self) -> PlateBuilderPage:
        return self._builder

    def library(self, kind: str = "plate") -> PlateLibraryPage:
        return self._libs[kind]

    def open_rosette(self, rosette_id: str) -> None:
        """Deep link used by the Layout tab's 'edit this rosette'."""
        if not self.maybe_discard():
            return
        self._mode = "rosette"
        self._pills.set_selected("rosette")
        self._lib_host.setCurrentIndex(1)
        self._open("rosette", rosette_id)

    def get_page_title(self) -> str:
        return "Plate"
