"""
side_panel_objects.py — Persistent right-rail Print Objects browser.

Compact list view of the objects + collections that live in Step 2.
Bound to a ``PrintObjectsModel`` so any edit (Add / Edit / Delete) in
the side panel reflects instantly in the step body, and vice versa.

The browser supports starting a drag with the selected object's name
as the mime payload. Step 3's ``WellPlateView`` accepts drops so the
user can drag an object onto a well to bind it.

Collapsible: a chevron in the header animates the panel between its
expanded width (~320 px scaled) and a thin handle. Width state is
intended to be persisted by the orchestrator via QSettings (TBD).
"""

from __future__ import annotations

from PySide6.QtCore import (
    Qt, QMimeData, QPoint, QPropertyAnimation, QEasingCurve, Signal,
)
from PySide6.QtGui import QDrag
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QPushButton, QScrollArea, QSizePolicy, QStackedWidget, QVBoxLayout,
    QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS

from .models import PrintObjectsModel


MIME_OBJECT_NAME = "application/x-mebp-print-object-name"


class _ObjectListWidget(QListWidget):
    """QListWidget that emits a drag with the selected object's name."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setSelectionMode(self.SelectionMode.SingleSelection)
        self.setStyleSheet(
            f"QListWidget {{"
            f"  background: {COLORS['base']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {_s(6)}px;"
            f"  padding: {_s(4)}px;"
            f"  font-size: {_sf(10.5)}pt;"
            f"}}"
            f"QListWidget::item {{ padding: {_s(4)}px {_s(8)}px; }}"
            f"QListWidget::item:selected {{"
            f"  background: {COLORS['surface1']};"
            f"  color: {COLORS['text']};"
            f"}}"
        )

    def startDrag(self, supportedActions) -> None:  # noqa: N802
        item = self.currentItem()
        if item is None:
            return
        name = item.data(Qt.UserRole) or item.text()
        mime = QMimeData()
        mime.setData(MIME_OBJECT_NAME, name.encode("utf-8"))
        mime.setText(str(name))
        drag = QDrag(self)
        drag.setMimeData(mime)
        drag.exec(Qt.CopyAction)


class PrintObjectsSidePanel(QFrame):
    """Persistent right-rail browser bound to a ``PrintObjectsModel``.

    Signals:
        edit_requested(str)   — user double-clicked an object name
        add_requested()       — user clicked + Add Object
        delete_requested(str) — user clicked the trash button
        selected(str)         — user selected an object in the list
        collapsed_changed(bool)
    """

    EXPANDED_W = 320     # px (pre-scale)
    COLLAPSED_W = 28

    edit_requested = Signal(str)
    add_requested = Signal()
    delete_requested = Signal(str)
    selected = Signal(str)
    collapsed_changed = Signal(bool)

    def __init__(self, model: PrintObjectsModel,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("printObjectsSidePanel")
        self.setStyleSheet(
            f"#printObjectsSidePanel {{"
            f"  background: {COLORS['mantle']};"
            f"  border-left: 1px solid {COLORS['surface1']};"
            f"}}"
        )
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self._model = model
        self._collapsed = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(_s(8), _s(8), _s(8), _s(8))
        outer.setSpacing(_s(6))

        # ── Header ────────────────────────────────────────────────
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        self._title_lbl = QLabel("Print Objects")
        self._title_lbl.setStyleSheet(
            f"color: {COLORS['text']}; "
            f"font-size: {_sf(11)}pt; font-weight: 700;"
            f"letter-spacing: 0.5px;"
        )
        header.addWidget(self._title_lbl)
        header.addStretch(1)

        self._collapse_btn = QPushButton("›")
        self._collapse_btn.setCursor(Qt.PointingHandCursor)
        self._collapse_btn.setStyleSheet(
            f"QPushButton {{"
            f"  background: transparent; color: {COLORS['subtext0']};"
            f"  border: none; padding: 0; font-size: {_sf(14)}pt;"
            f"  font-weight: 700;"
            f"}}"
            f"QPushButton:hover {{ color: {COLORS['text']}; }}"
        )
        self._collapse_btn.clicked.connect(self.toggle_collapsed)
        self._collapse_btn.setFixedWidth(_s(20))
        header.addWidget(self._collapse_btn)
        outer.addLayout(header)

        # ── Body ──────────────────────────────────────────────────
        self._body = QWidget()
        body_lay = QVBoxLayout(self._body)
        body_lay.setContentsMargins(0, 0, 0, 0)
        body_lay.setSpacing(_s(6))

        self._add_btn = QPushButton("+ Add Object")
        self._add_btn.setCursor(Qt.PointingHandCursor)
        self._add_btn.setStyleSheet(
            f"QPushButton {{"
            f"  background: {COLORS['surface0']};"
            f"  color: {COLORS['mauve']};"
            f"  border: 1px dashed {COLORS['surface2']};"
            f"  border-radius: {_s(6)}px;"
            f"  padding: {_s(6)}px;"
            f"  font-weight: 600; font-size: {_sf(10.5)}pt;"
            f"}}"
            f"QPushButton:hover {{ background: {COLORS['surface1']}; }}"
        )
        self._add_btn.clicked.connect(self.add_requested.emit)
        body_lay.addWidget(self._add_btn)

        self._list = _ObjectListWidget(self._body)
        self._list.itemDoubleClicked.connect(
            lambda it: self.edit_requested.emit(it.data(Qt.UserRole) or it.text())
        )
        self._list.currentItemChanged.connect(
            lambda cur, _prev: self.selected.emit(
                (cur.data(Qt.UserRole) if cur else "") or ""
            )
        )
        body_lay.addWidget(self._list, 1)

        self._row_actions = QHBoxLayout()
        self._row_actions.setContentsMargins(0, 0, 0, 0)
        self._delete_btn = QPushButton("Delete")
        self._delete_btn.setEnabled(False)
        self._delete_btn.setCursor(Qt.PointingHandCursor)
        self._delete_btn.setStyleSheet(
            f"QPushButton {{"
            f"  background: transparent; color: {COLORS['red']};"
            f"  border: none; padding: {_s(4)}px {_s(6)}px;"
            f"  font-size: {_sf(10)}pt;"
            f"}}"
            f"QPushButton:hover {{ color: {COLORS['text']}; }}"
            f"QPushButton:disabled {{ color: {COLORS['overlay0']}; }}"
        )
        self._delete_btn.clicked.connect(self._on_delete_clicked)
        self._row_actions.addStretch(1)
        self._row_actions.addWidget(self._delete_btn)
        body_lay.addLayout(self._row_actions)

        # Collections summary (read-only mini list)
        self._coll_lbl = QLabel("Collections")
        self._coll_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"font-size: {_sf(9.5)}pt; font-weight: 700; "
            f"letter-spacing: 1px; padding-top: {_s(6)}px;"
        )
        body_lay.addWidget(self._coll_lbl)

        self._coll_list = QListWidget(self._body)
        self._coll_list.setStyleSheet(self._list.styleSheet())
        self._coll_list.setMaximumHeight(_s(120))
        body_lay.addWidget(self._coll_list)

        # ── v7.5.1: step-aware content stack ─────────────────────
        # Index 0 = generic browser (self._body)
        # Index 1 = Step 1's Objects-in-This-Print + Summary panel
        #           (injected via set_step1_objects_panel())
        self._content_stack = QStackedWidget(self)
        self._content_stack.addWidget(self._body)           # idx 0
        self._step1_placeholder = QWidget(self)             # idx 1 (default)
        self._content_stack.addWidget(self._step1_placeholder)
        outer.addWidget(self._content_stack, 1)

        # ── Animation ─────────────────────────────────────────────
        self._anim = QPropertyAnimation(self, b"maximumWidth")
        self._anim.setDuration(140)
        self._anim.setEasingCurve(QEasingCurve.OutCubic)
        self.setMaximumWidth(_s(self.EXPANDED_W))
        self.setMinimumWidth(_s(self.COLLAPSED_W))

        # ── Wire model ───────────────────────────────────────────
        self._model.changed.connect(self._refresh_from_model)
        self._refresh_from_model()
        self._list.currentItemChanged.connect(
            lambda cur, _prev: self._delete_btn.setEnabled(cur is not None)
        )

    # ── State ─────────────────────────────────────────────────────

    def is_collapsed(self) -> bool:
        return self._collapsed

    def set_collapsed(self, collapsed: bool) -> None:
        if collapsed == self._collapsed:
            return
        self._collapsed = collapsed
        self._anim.stop()
        self._anim.setStartValue(self.maximumWidth())
        self._anim.setEndValue(
            _s(self.COLLAPSED_W if collapsed else self.EXPANDED_W)
        )
        self._anim.start()
        self._content_stack.setVisible(not collapsed)
        self._title_lbl.setVisible(not collapsed)
        self._collapse_btn.setText("‹" if collapsed else "›")
        self.collapsed_changed.emit(collapsed)

    def toggle_collapsed(self) -> None:
        self.set_collapsed(not self._collapsed)

    # ── v7.5.1: step-aware content swap ──────────────────────────

    def set_step1_objects_panel(self, panel: QWidget | None) -> None:
        """Install the "This Print" widget (Print List + Objects in
        This Print + Summary).

        The wizard's PrintObjectsTab builds this as a detached
        ``tab.objects_panel`` widget; the orchestrator hands it here.
        v7.6.0: the panel is wrapped in a QScrollArea so a tall
        column (Print List + Objects + Summary + Prints to Send)
        scrolls instead of clipping.

        Pass ``None`` to revert to a placeholder.
        """
        # Replace index 1 in the stack with the new (scroll-wrapped) panel.
        self._content_stack.removeWidget(self._content_stack.widget(1))
        if panel is None:
            wrapper: QWidget = QWidget(self._content_stack)
        else:
            wrapper = QScrollArea(self._content_stack)
            wrapper.setWidgetResizable(True)
            wrapper.setFrameShape(QFrame.NoFrame)
            wrapper.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            wrapper.setStyleSheet(
                f"QScrollArea {{ background: {COLORS['mantle']}; border: none; }}"
            )
            panel.setParent(wrapper)
            wrapper.setWidget(panel)
        self._content_stack.insertWidget(1, wrapper)
        self._step1_placeholder = wrapper

    def set_step3_run_status_panel(self, panel: QWidget | None) -> None:
        """v7.6.0: install the Step 3 (Plan & Run) live status panel.
        Added as stack index 2; shown when on the last step."""
        # Ensure index 2 exists / is replaced.
        if self._content_stack.count() > 2:
            self._content_stack.removeWidget(self._content_stack.widget(2))
        if panel is None:
            panel = QWidget(self._content_stack)
        panel.setParent(self._content_stack)
        self._content_stack.insertWidget(2, panel)
        self._run_status_panel = panel

    def set_active_step(self, step_index: int) -> None:
        """Switch the panel content for the active wizard step.

        v7.6.0: "This Print" is shown on Steps 1 & 2; the Run-status
        live mirror is shown on Step 3 (if installed).
        """
        has_run_status = (
            getattr(self, "_run_status_panel", None) is not None
            and self._content_stack.count() > 2
        )
        if step_index >= 2 and has_run_status:
            self._content_stack.setCurrentIndex(2)
            self._title_lbl.setText("Running")
        else:
            self._content_stack.setCurrentIndex(1)
            self._title_lbl.setText("This Print")

    # ── Internal ──────────────────────────────────────────────────

    def _refresh_from_model(self) -> None:
        cur_name = ""
        cur = self._list.currentItem()
        if cur is not None:
            cur_name = cur.data(Qt.UserRole) or ""

        self._list.blockSignals(True)
        self._list.clear()
        for obj in self._model.objects():
            name = obj.get("name", "?")
            item = QListWidgetItem(name)
            item.setData(Qt.UserRole, name)
            ink = obj.get("ink") or (next(iter(obj.get("ink_assignments", {}).values()), "")
                                     if isinstance(obj.get("ink_assignments"), dict)
                                     else "")
            if ink:
                item.setText(f"{name}  ·  {ink}")
            self._list.addItem(item)
            if name == cur_name:
                self._list.setCurrentItem(item)
        self._list.blockSignals(False)

        self._coll_list.clear()
        for coll in self._model.collections():
            n = coll.get("name", "?")
            count = len(coll.get("objects", []))
            self._coll_list.addItem(f"{n}  ({count})")

    def _on_delete_clicked(self) -> None:
        cur = self._list.currentItem()
        if cur is None:
            return
        name = cur.data(Qt.UserRole) or cur.text()
        self.delete_requested.emit(name)
