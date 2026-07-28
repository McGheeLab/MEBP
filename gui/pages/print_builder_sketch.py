"""
print_builder_sketch.py — the Print Builder's flagship "Sketch" sub-page
(v7.5.x).

Draw a print pattern from vector primitives, fill regions, stack Z-layers,
preview the resulting toolpath live, then **Send to Print Setup** — which
bakes the drawing into a ``csv_import`` print object (via
``save_trajectory_as_print_object``) and emits ``print_file_created`` so it
lands in Print Setup's custom-prints area.

Layout: vertical tool palette | drawing canvas | right panel (properties +
live preview + send).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal, QTimer, QSize
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QSplitter, QFrame, QToolButton,
    QButtonGroup, QPushButton, QLabel, QCheckBox, QComboBox, QDoubleSpinBox,
    QSpinBox, QScrollArea, QGroupBox, QSizePolicy, QMessageBox, QLineEdit,
    QColorDialog, QGridLayout, QApplication,
)

from gui.styles import COLORS, build_section_title_style
from gui.scaling import s, scaled_font_size as _sf, scale_factor
from gui.widgets.icons import icon
from gui.widgets.sketch_canvas import SketchCanvas, Tool, PUMP_HEX
from gui.widgets.sketch_profile_view import SketchProfileView
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory, plan_print_sections,
)
from SupportClasses.PrintFileManager import save_trajectory_as_print_object

logger = logging.getLogger(__name__)


def _to_float(v):
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def _to_int(v):
    try:
        return None if v is None else int(v)
    except (TypeError, ValueError):
        return None


# (label, Tool, icon-name or glyph)
_TOOLS = [
    ("Select",      Tool.SELECT,  "cursor"),
    ("Line",        Tool.LINE,    "line"),
    ("Rectangle",   Tool.RECT,    "▭"),
    ("Circle",      Tool.CIRCLE,  "◯"),
    ("Ellipse",     Tool.ELLIPSE, "⬭"),
    ("Polygon",     Tool.POLYGON, "⬠"),
    ("Fill region", Tool.FILL,    "droplet"),
    ("Retract & move point (pen-up break in the path)",
     Tool.TRAVEL,   "arrow-up"),
]


class SketchPage(QWidget):
    """Draw-to-print Sketch tool."""

    print_file_created = Signal(str)   # baked a NEW print (→ Print Setup)
    print_file_saved = Signal(str)     # overwrote the print being edited

    TOOL_BTN = 40
    TOOL_ICON = 22

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._needle = None
        self._syringe = None
        self._needle_od_mm = 0.0
        self._needle_id_mm = 0.0          # 1× bead reference (needle inner Ø)
        self._building = False
        # v7.5.x: per-channel ink (name, colour) for the print-sequence panel,
        # keyed by pump index (0→P1,1→P2,2→P3). Populated from the hardware
        # config's pump→ink assignments.
        self._channel_info: dict[int, tuple] = {}
        # v7.5.x: per-section collapsed state (keyed by section ordinal), so a
        # sequence-panel rebuild preserves which sections the operator folded.
        self._seq_collapsed: dict[int, bool] = {}
        # v7.5.x: the well whose geometry defines the drawing boundary. The
        # sketch is authored in well-relative mm with (0,0) = well center, so
        # the boundary circles are centered at the origin.
        self._plate = None
        self._selected_well: str | None = None
        self._safe_radius_mm = 0.0        # needle-safe radius (mm); 0 = none
        self._last_oob = False            # toolpath crosses the safe boundary
        # v7.5.x: calibrated plate bottom (zero-ref mm); the Sketch's print
        # height (``z_start_mm``) is measured up from it, not absolute.
        self._plate_bottom_z: float | None = None
        # v7.5.x: user-chosen print name (persists across props rebuilds).
        self._print_name: str = "Sketch"
        # v7.5.x: back-trace parameters (persist across props rebuilds). The
        # return pass retraces a run offset in height (Z) + in-plane (XY),
        # optionally extruding a second bead.
        self._bt_z_offset: float = 0.20
        self._bt_xy_offset: float = 0.0
        self._bt_extrude: bool = True
        # v7.5.x: when editing an existing print (opened from the Library), this
        # is its display name — "Save changes" overwrites it. None = fresh
        # sketch. ``_editing_stem`` is the actual on-disk file stem (may differ
        # from a sanitized display name) so overwrite hits the right file.
        self._editing_name: str | None = None
        self._editing_stem: str | None = None
        # Directory prints are read/written from. None = the default
        # (config/prints); only overridden for tests. Threaded through so an
        # edit-save writes back to the same place the print was loaded from.
        self._prints_dir: str | None = None

        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(120)
        self._preview_timer.timeout.connect(self._recompute_preview)

        self._build_ui()
        self._canvas.set_sketch(Sketch())
        # Show the shaded print-thickness band by default (the feature's point).
        self._canvas.set_show_thickness(self._thickness_btn.isChecked())
        self._apply_bead_width()
        self._rebuild_props()
        self._schedule_preview()

    def showEvent(self, event):
        super().showEvent(event)
        # Default to a zoomed-in single-well view on entry, but only when the
        # canvas is empty so an in-progress sketch keeps the user's pan/zoom.
        # Defer to the next event-loop tick so the canvas has its laid-out
        # size before framing (avoids locking in a bad zoom on first show,
        # since fit_view marks the view initialised).
        if not self._canvas.sketch().shapes:
            QTimer.singleShot(0, self._fit_canvas_if_empty)

    def _fit_canvas_if_empty(self):
        if not self._canvas.sketch().shapes:
            self._canvas.fit_view()

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self):
        self.setObjectName("sketchPageRoot")
        self._apply_dark_theme()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(6), s(6), s(6), s(6))
        outer.setSpacing(s(6))

        header = QLabel("Sketch — draw a print, send it to Print Setup")
        header.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {_sf(13)}pt; "
            f"font-weight: 700;")
        outer.addWidget(header)

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self._build_toolbar())

        self._canvas = SketchCanvas(self)
        self._canvas.sketch_changed.connect(self._on_sketch_changed)
        self._canvas.selection_changed.connect(self._on_selection_changed)
        self._canvas.fill_result.connect(self._on_fill_result)
        self._canvas.constraints_changed.connect(self._refresh_constraints_card)

        # v7.5.x: centre pane = XY drawing canvas (top) over an XZ side-profile
        # (bottom), resizable against each other via a vertical splitter. The
        # profile shows the print's elevation — layer stack, total height, the
        # plate floor (z=0), and the travel lifts between shapes.
        self._profile_view = SketchProfileView(self)
        centre_split = QSplitter(Qt.Vertical)
        centre_split.setChildrenCollapsible(False)
        centre_split.setHandleWidth(s(4))
        centre_split.addWidget(self._canvas)
        centre_split.addWidget(self._profile_view)
        centre_split.setStretchFactor(0, 3)
        centre_split.setStretchFactor(1, 1)
        centre_split.setSizes([s(420), s(150)])
        split.addWidget(centre_split)

        split.addWidget(self._build_right_panel())
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setStretchFactor(2, 0)
        split.setSizes([s(52), s(600), s(360)])
        outer.addWidget(split, 1)

        # Status / DOF row
        row = QHBoxLayout()
        self._stats_lbl = QLabel("Empty sketch")
        self._stats_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        row.addWidget(self._stats_lbl)
        row.addStretch(1)
        self._status_lbl = QLabel("")
        self._status_lbl.setStyleSheet(f"font-size: {_sf(9)}pt;")
        row.addWidget(self._status_lbl)
        outer.addLayout(row)

    def _apply_dark_theme(self):
        red = COLORS.get("red", "#f38ba8")
        self.setStyleSheet(f"""
            #sketchPageRoot {{ background: {COLORS['base']}; }}
            #sketchRightPanel {{ background: {COLORS['base']}; }}
            #sketchPageRoot QScrollArea {{
                background: transparent; border: none;
            }}
            #sketchPageRoot QScrollArea > QWidget > QWidget {{
                background: transparent;
            }}
            #sketchToolbar {{
                background: {COLORS['mantle']};
                border-right: 1px solid {COLORS['surface1']};
            }}
            #sketchPageRoot QToolButton {{
                background: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: {s(5)}px;
                color: {COLORS['text']};
            }}
            #sketchPageRoot QToolButton:hover {{
                background: {COLORS['surface1']};
            }}
            #sketchPageRoot QToolButton:checked {{
                background: {COLORS['blue']};
                color: {COLORS['crust']};
                border: 1px solid {COLORS['blue']};
            }}
            QPushButton#primaryBtn {{
                background: {COLORS['blue']}; color: {COLORS['crust']};
                border: none; border-radius: {s(5)}px;
                padding: {s(7)}px; font-weight: 700;
            }}
            QPushButton#primaryBtn:hover {{ background: {COLORS['mauve']}; }}
            QPushButton#primaryBtn:disabled {{
                background: {COLORS['surface1']}; color: {COLORS['overlay0']};
            }}
            QPushButton#dangerBtn {{
                background: {red}; color: {COLORS['crust']};
                border: none; border-radius: {s(5)}px;
                padding: {s(5)}px; font-weight: 700;
            }}
        """)

    def _build_toolbar(self) -> QWidget:
        frame = QFrame()
        frame.setObjectName("sketchToolbar")
        frame.setFixedWidth(s(self.TOOL_BTN + 12))
        col = QVBoxLayout(frame)
        col.setContentsMargins(s(6), s(8), s(6), s(8))
        col.setSpacing(s(4))

        self._tool_group = QButtonGroup(frame)
        self._tool_group.setExclusive(True)
        self._tool_btns = {}
        for label, tool, ico in _TOOLS:
            b = self._make_tool_btn(label, tool, ico)
            self._tool_group.addButton(b)
            col.addWidget(b)
            self._tool_btns[tool] = b
        self._tool_btns[Tool.SELECT].setChecked(True)

        col.addSpacing(s(8))
        col.addWidget(self._sep())
        col.addSpacing(s(4))

        col.addWidget(self._action_btn(
            "plus", "New sketch (clear the canvas + leave edit mode)",
            self._new_sketch))
        col.addWidget(self._action_btn("trash", "Delete selected (Del)",
                                       self._canvas_delete, danger=True))
        col.addWidget(self._action_btn("undo", "Undo (Ctrl+Z)",
                                       lambda: self._canvas.undo()))
        col.addWidget(self._action_btn("redo", "Redo",
                                       lambda: self._canvas.redo()))
        col.addSpacing(s(8))
        col.addWidget(self._sep())
        col.addSpacing(s(4))
        col.addWidget(self._action_btn("search", "Fit view",
                                       lambda: self._canvas.fit_view()))
        self._snap_btn = self._action_btn("grid", "Snap to 1 mm grid",
                                          self._toggle_snap, checkable=True)
        col.addWidget(self._snap_btn)
        self._osnap_btn = self._action_btn(
            "target", "Snap to existing object borders/vertices",
            self._toggle_osnap, checkable=True)
        self._osnap_btn.setChecked(True)   # object snap on by default
        col.addWidget(self._osnap_btn)
        self._thickness_btn = self._action_btn(
            "line", "Show print thickness (shade a bead around each line at "
            "needle inner Ø × extrusion multiplier)", self._toggle_thickness,
            checkable=True)
        self._thickness_btn.setChecked(True)   # thickness preview on by default
        col.addWidget(self._thickness_btn)
        col.addStretch(1)
        return frame

    def _make_tool_btn(self, label, tool, ico) -> QToolButton:
        b = QToolButton()
        b.setCheckable(True)
        b.setAutoExclusive(False)
        b.setToolTip(label)
        b.setFixedSize(s(self.TOOL_BTN), s(self.TOOL_BTN))
        if len(ico) <= 2 and not ico.isalpha():     # glyph (colour via QSS)
            b.setText(ico)
            f = b.font()
            f.setPointSize(_sf(15))
            b.setFont(f)
        else:
            b.setIcon(icon(ico, color="#ffffff", px=s(self.TOOL_ICON)))
            b.setIconSize(QSize(s(self.TOOL_ICON), s(self.TOOL_ICON)))
        b.clicked.connect(lambda _=False, t=tool: self._canvas.set_tool(t))
        return b

    def _action_btn(self, ico, tip, slot, danger=False, checkable=False):
        b = QToolButton()
        b.setToolTip(tip)
        b.setCheckable(checkable)
        b.setFixedSize(s(self.TOOL_BTN), s(self.TOOL_BTN))
        b.setIcon(icon(ico, color=("#f38ba8" if danger else "#ffffff"),
                       px=s(self.TOOL_ICON)))
        b.setIconSize(QSize(s(self.TOOL_ICON), s(self.TOOL_ICON)))
        b.clicked.connect(slot)
        return b

    def _sep(self) -> QFrame:
        f = QFrame()
        f.setFrameShape(QFrame.HLine)
        f.setStyleSheet(f"color: {COLORS['surface1']};")
        return f

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        panel.setObjectName("sketchRightPanel")
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(s(8), s(6), s(8), s(8))
        outer.setSpacing(s(8))

        # The WHOLE panel scrolls together in one outer scroll area; every card
        # sizes to its own content (no nested fixed-height scrollers), so the
        # print sequence and its per-section operation lists grow as needed.
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        content = QWidget()
        v = QVBoxLayout(content)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(s(12))

        # Well boundary selector (persistent — not rebuilt with the props).
        v.addWidget(self._build_well_card())
        # Abstract-ink manager (persistent) — the sketch's pump-agnostic inks.
        v.addWidget(self._build_inks_card())
        # Parametric constraints (persistent) — buttons + list + DOF status.
        v.addWidget(self._build_constraints_card())
        # Print sequence panel (persistent) — color-coded sections + moves.
        v.addWidget(self._build_sequence_card())
        # Per-shape / print properties (rebuilt on selection).
        self._props_host = QWidget()
        self._props_layout = QVBoxLayout(self._props_host)
        self._props_layout.setContentsMargins(0, 0, 0, 0)
        self._props_layout.setSpacing(s(12))
        v.addWidget(self._props_host)
        v.addStretch(1)
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)

        # The toolpath raster is rendered in the main canvas (not here); the
        # shapes are the editable overlay on top of it.
        # "Save changes" overwrites the print opened from the Library (only
        # visible while editing); "Send to Print Setup" always saves as new.
        # Both are PINNED below the scroll so they're always reachable.
        self._save_btn = QPushButton("Save changes")
        self._save_btn.setObjectName("primaryBtn")
        self._save_btn.clicked.connect(self._save_changes)
        self._save_btn.setVisible(False)
        outer.addWidget(self._save_btn)

        self._send_btn = QPushButton("Send to Print Setup")
        self._send_btn.setObjectName("primaryBtn")
        self._send_btn.clicked.connect(self._send_to_print_setup)
        outer.addWidget(self._send_btn)
        return panel

    def _build_well_card(self) -> QGroupBox:
        """Persistent well-boundary selector: pick which well defines the
        drawing boundary; shows the well-wall Ø and the needle-safe Ø."""
        grp = self._group("Well boundary")
        lay = grp.layout()

        self._well_combo = QComboBox()
        self._well_combo.setToolTip(
            "Which well's geometry defines the drawing boundary. The dashed "
            "outer circle is the well wall; the inner circle is inset by the "
            "needle radius so the needle never touches the wall.")
        self._well_combo.currentIndexChanged.connect(self._on_well_changed)
        self._field_row(lay, "Well", self._well_combo)

        self._boundary_info_lbl = QLabel("No plate configured.")
        self._boundary_info_lbl.setWordWrap(True)
        self._boundary_info_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        lay.addWidget(self._boundary_info_lbl)

        self._bounds_warn_lbl = QLabel("")
        self._bounds_warn_lbl.setWordWrap(True)
        self._bounds_warn_lbl.setVisible(False)
        self._bounds_warn_lbl.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: {_sf(9)}pt;")
        lay.addWidget(self._bounds_warn_lbl)
        return grp

    # ── Print-sequence panel ──────────────────────────────────────

    def _build_sequence_card(self) -> QGroupBox:
        """Persistent card: needle-mode toggle + the ordered, colour-coded list
        of continuous print sections and the moves that separate them."""
        grp = self._group("Print sequence")
        lay = grp.layout()

        self._single_needle_chk = QCheckBox(
            "Single needle (one ink at the tip — swaps between materials)")
        self._single_needle_chk.setToolTip(
            "Single-needle: only one material is at the tip at a time, so a "
            "channel change breaks the bead (an ink replacement). Multi-needle "
            "(coaxial / multi-pump) keeps different channels welded together. "
            "Auto-detected from the configured needle until you toggle it.")
        self._single_needle_chk.toggled.connect(self._on_single_needle_toggled)
        lay.addWidget(self._single_needle_chk)

        self._seq_summary_lbl = QLabel("")
        self._seq_summary_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        lay.addWidget(self._seq_summary_lbl)

        # Sections render directly into the card (NO nested fixed-height
        # scroll); the whole right panel scrolls, so each section sizes to its
        # own content and the list grows to fit.
        self._seq_host = QWidget()
        self._seq_layout = QVBoxLayout(self._seq_host)
        self._seq_layout.setContentsMargins(0, 0, 0, 0)
        self._seq_layout.setSpacing(s(4))
        lay.addWidget(self._seq_host)
        return grp

    def _effective_single(self) -> bool:
        return self._canvas.sketch().is_single_needle(self._needle)

    def _sync_single_needle_checkbox(self):
        if not hasattr(self, "_single_needle_chk"):
            return
        self._single_needle_chk.blockSignals(True)
        self._single_needle_chk.setChecked(self._effective_single())
        self._single_needle_chk.blockSignals(False)

    def _on_single_needle_toggled(self, checked: bool):
        # Explicit override (leaves AUTO). Compiler welding changes in single
        # mode, so recompile the preview and re-plan the sequence.
        self._canvas.sketch().single_needle = bool(checked)
        self._schedule_preview()
        self._refresh_sequence()

    # ── Abstract-ink manager ──────────────────────────────────────

    def _build_inks_card(self) -> QGroupBox:
        """Persistent card: manage the sketch's ABSTRACT inks (name + colour).
        The sketch is pump-agnostic — it only declares the inks it needs; a
        physical pump is assigned later at print time (Quick Print maps each
        abstract ink → a configured ink)."""
        grp = self._group("Inks")
        lay = grp.layout()
        hint = QLabel(
            "Abstract inks this print uses — assign one to each shape. Map "
            "them to real inks at print time (Quick Print).")
        hint.setWordWrap(True)
        hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        lay.addWidget(hint)
        self._inks_host = QWidget()
        self._inks_layout = QVBoxLayout(self._inks_host)
        self._inks_layout.setContentsMargins(0, 0, 0, 0)
        self._inks_layout.setSpacing(s(4))
        lay.addWidget(self._inks_host)
        add_btn = QPushButton("+ Add ink")
        add_btn.setToolTip("Add another abstract ink to this print.")
        add_btn.clicked.connect(self._add_ink)
        lay.addWidget(add_btn)
        self._refresh_inks_card()
        return grp

    def _refresh_inks_card(self):
        if not hasattr(self, "_inks_layout"):
            return
        while self._inks_layout.count():
            it = self._inks_layout.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()  # reclaim the C++ widget now, not at GC
        for ink in self._canvas.sketch().inks:
            self._inks_layout.addWidget(self._ink_row(ink))

    def _ink_row(self, ink) -> QWidget:
        sk = self._canvas.sketch()
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(s(6))
        swatch = QPushButton()
        swatch.setFixedSize(s(18), s(18))
        swatch.setCursor(Qt.PointingHandCursor)
        swatch.setToolTip("Click to change this ink's colour")
        swatch.setStyleSheet(
            f"background: {ink.color}; border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {s(3)}px;")
        swatch.clicked.connect(lambda _=False, i=ink: self._recolor_ink(i))
        h.addWidget(swatch)
        name = QLineEdit(ink.name)
        name.setToolTip("Ink name")
        name.editingFinished.connect(
            lambda le=name, i=ink: self._rename_ink(i, le.text()))
        h.addWidget(name, 1)
        used = sum(1 for shape in sk.shapes
                   if shape.kind != "travel"
                   and int(getattr(shape, "ink_id", 1)) == ink.id)
        count = QLabel(f"{used}")
        count.setToolTip("Shapes using this ink")
        count.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        h.addWidget(count)
        delete = QPushButton("✕")
        delete.setObjectName("flatBtn")
        delete.setFixedSize(s(20), s(20))
        delete.setToolTip("Delete this ink (its shapes move to the first ink)")
        delete.setEnabled(len(sk.inks) > 1)
        delete.clicked.connect(lambda _=False, i=ink: self._delete_ink(i))
        h.addWidget(delete)
        return row

    def _add_ink(self):
        ink = self._canvas.sketch().add_ink()
        self._canvas.set_active_ink(ink.id)
        self._refresh_inks_card()
        self._rebuild_props()          # per-shape ink combos gain the new option
        self._schedule_preview()

    def _rename_ink(self, ink, text):
        text = (text or "").strip()
        if text and text != ink.name:
            ink.name = text
            self._refresh_sequence()

    def _recolor_ink(self, ink):
        col = QColorDialog.getColor(QColor(ink.color), self, "Ink colour")
        if not col.isValid():
            return
        ink.color = col.name()
        for shape in self._canvas.sketch().shapes:   # sync per-shape colour cache
            if int(getattr(shape, "ink_id", 1)) == ink.id:
                shape.color = ink.color
        self._refresh_inks_card()
        self._canvas.update()
        self._schedule_preview()

    def _delete_ink(self, ink):
        sk = self._canvas.sketch()
        if len(sk.inks) <= 1:
            return
        survivor = next((i for i in sk.inks if i.id != ink.id), None)
        if survivor is None:
            return
        sk.inks = [i for i in sk.inks if i.id != ink.id]
        for shape in sk.shapes:        # reassign orphaned shapes to a survivor
            if int(getattr(shape, "ink_id", 1)) == ink.id:
                shape.ink_id = survivor.id
                shape.color = survivor.color
        if self._canvas.active_ink_id() == ink.id:
            self._canvas.set_active_ink(survivor.id)
        self._refresh_inks_card()
        self._rebuild_props()
        self._canvas.update()
        self._schedule_preview()

    def _ink_label_color(self, ink_id: int):
        """Label + colour for an abstract ink id, read from the sketch's own
        ink list (the sketch is pump-agnostic — a physical pump is assigned
        later at print time)."""
        ink = self._canvas.sketch().ink_by_id(int(ink_id))
        if ink is not None:
            return (ink.name or f"Ink {ink_id}",
                    ink.color or PUMP_HEX[(int(ink_id) - 1) % len(PUMP_HEX)])
        return f"Ink {ink_id}", PUMP_HEX[(int(ink_id) - 1) % len(PUMP_HEX)]

    # ── Parametric constraints card ───────────────────────────────

    # (label, kind, tooltip) — enabled per-selection via can_add_constraint.
    _CONSTRAINT_BUTTONS = [
        ("Join", "coincident",
         "Weld the nearest anchor pair (endpoints / corners / centers) of the "
         "two selected shapes together"),
        ("Point on", "point_on",
         "Keep a point of one selected shape riding ON the other selected "
         "line / circle"),
        ("Tangent", "tangent",
         "Line + circle or two circles: keep them tangent"),
        ("Horizontal", "horizontal",
         "Level the selected line(s) — or align two shapes' centers "
         "horizontally"),
        ("Vertical", "vertical",
         "Plumb the selected line(s) — or align two shapes' centers "
         "vertically"),
        ("Parallel", "parallel", "Keep two selected lines parallel"),
        ("Perpendicular", "perpendicular",
         "Keep two selected lines at 90°"),
        ("Concentric", "concentric",
         "Two circles / ellipses / rects share a center"),
        ("Equal length", "equal_length",
         "Two selected lines keep equal length"),
        ("Equal radius", "equal_radius",
         "Two selected circles keep equal radius"),
        ("Distance", "distance",
         "Drive the distance between two shapes (or a line's length) to a "
         "typed value"),
        ("Radius", "radius",
         "Drive the selected circle's radius to a typed value"),
        ("Lock", "fix",
         "Pin the selected shape(s) in place (click again to unlock)"),
    ]

    def _build_constraints_card(self) -> QGroupBox:
        """Persistent card: DOF status, snap→constraint auto-capture toggle,
        constraint-creation buttons (enabled per selection), and the list of
        existing constraints (dimensions editable, each deletable)."""
        grp = self._group("Constraints")
        lay = grp.layout()

        self._dof_lbl = QLabel("")
        self._dof_lbl.setWordWrap(True)
        self._dof_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        lay.addWidget(self._dof_lbl)

        self._auto_capture_chk = QCheckBox("Capture joins while drawing")
        self._auto_capture_chk.setToolTip(
            "When drawing snaps onto an existing vertex, keep them joined "
            "with a coincident constraint (edge snaps on lines/circles become "
            "point-on). Off = snap stays a one-time positioning aid.")
        self._auto_capture_chk.setChecked(True)
        self._auto_capture_chk.toggled.connect(
            lambda on: self._canvas.set_auto_constrain(bool(on)))
        lay.addWidget(self._auto_capture_chk)

        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(s(4))
        self._constraint_btns: dict[str, QPushButton] = {}
        for n, (label, kind, tip) in enumerate(self._CONSTRAINT_BUTTONS):
            b = QPushButton(label)
            b.setToolTip(tip)
            b.setEnabled(False)
            b.clicked.connect(lambda _=False, k=kind: self._add_constraint(k))
            self._constraint_btns[kind] = b
            grid.addWidget(b, n // 3, n % 3)
        lay.addWidget(grid_host)

        self._constraints_host = QWidget()
        self._constraints_layout = QVBoxLayout(self._constraints_host)
        self._constraints_layout.setContentsMargins(0, 0, 0, 0)
        self._constraints_layout.setSpacing(s(3))
        lay.addWidget(self._constraints_host)

        self._refresh_constraints_card()
        return grp

    def _add_constraint(self, kind: str):
        ok, msg = self._canvas.add_constraint_for_selection(kind)
        if ok:
            self._status_ok(f"✓ {msg}")
        else:
            self._status_warn(f"⚠ {msg}")
        self._refresh_constraints_card()

    def _refresh_constraint_buttons(self):
        if not hasattr(self, "_constraint_btns"):
            return
        for kind, b in self._constraint_btns.items():
            b.setEnabled(self._canvas.can_add_constraint(kind))

    _DOF_TEXT = {
        "well_determined": ("Fully constrained", "green"),
        "under_determined": ("{dof} DOF free", "subtext0"),
        "inconsistent": ("⚠ Conflicting constraints — highlighted red",
                         "red"),
        "empty": ("", "subtext0"),
    }

    def _refresh_dof_label(self):
        if not hasattr(self, "_dof_lbl"):
            return
        sk = self._canvas.sketch()
        if not getattr(sk, "constraints", None):
            self._dof_lbl.setText(
                "Select shapes and add constraints — geometry re-solves live "
                "as you drag.")
            self._dof_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            return
        rep = self._canvas.last_solve_report()
        if rep is None:
            self._dof_lbl.setText(f"{len(sk.constraints)} constraint(s)")
            self._dof_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            return
        status = getattr(rep.status, "value", str(rep.status))
        text, color = self._DOF_TEXT.get(status, ("", "subtext0"))
        text = text.format(dof=rep.dof)
        self._dof_lbl.setText(
            f"{len(sk.constraints)} constraint(s) · {text}" if text
            else f"{len(sk.constraints)} constraint(s)")
        self._dof_lbl.setStyleSheet(
            f"color: {COLORS.get(color, COLORS['subtext0'])}; "
            f"font-size: {_sf(9)}pt;")

    def _refresh_constraints_card(self):
        if not hasattr(self, "_constraints_layout"):
            return
        self._refresh_dof_label()
        self._refresh_constraint_buttons()
        # Don't yank the row widgets out from under a value spinbox the user
        # is currently typing in (the debounced preview would rebuild them).
        focus = QApplication.focusWidget()
        if focus is not None and self._constraints_host.isAncestorOf(focus):
            return
        while self._constraints_layout.count():
            it = self._constraints_layout.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        for c in getattr(self._canvas.sketch(), "constraints", []):
            self._constraints_layout.addWidget(self._constraint_row(c))

    _CONSTRAINT_LABELS = {
        "coincident": "◉ Join", "point_on": "◎ Point-on",
        "tangent": "T Tangent", "horizontal": "— Horizontal",
        "vertical": "| Vertical", "parallel": "∥ Parallel",
        "perpendicular": "⊥ Perpendicular", "concentric": "◎ Concentric",
        "equal_length": "= Equal length", "equal_radius": "= Equal radius",
        "distance": "↔ Distance", "radius": "R Radius", "fix": "🔒 Lock",
    }

    def _format_constraint(self, c) -> str:
        sk = self._canvas.sketch()
        parts = []
        for sid, _anchor in c.refs:
            i = sk.shape_index_by_id(int(sid))
            sh = sk.shape_by_id(int(sid))
            parts.append(f"{getattr(sh, 'kind', '?')} #{i + 1}"
                         if sh is not None else "?")
        label = self._CONSTRAINT_LABELS.get(c.kind, c.kind)
        mode = f" ({c.mode})" if getattr(c, "mode", "") else ""
        return f"{label}{mode} — {' · '.join(parts)}"

    def _constraint_row(self, c) -> QWidget:
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(s(4))
        lbl = QPushButton(self._format_constraint(c))
        lbl.setFlat(True)
        lbl.setCursor(Qt.PointingHandCursor)
        lbl.setToolTip("Click to select the constrained shapes")
        lbl.setStyleSheet(
            f"QPushButton {{ background: transparent; border: none; "
            f"color: {COLORS['text']}; font-size: {_sf(9)}pt; "
            f"text-align: left; padding: {s(2)}px; }}"
            f"QPushButton:hover {{ color: {COLORS['blue']}; }}")
        lbl.clicked.connect(
            lambda _=False, cid=c.id:
            self._canvas.select_constraint_shapes(cid))
        h.addWidget(lbl, 1)
        if c.value is not None:                    # driven dimension → editable
            spin = self._dspin(float(c.value), 0.0, 1000.0, 0.1)
            spin.setFixedWidth(s(84))
            spin.valueChanged.connect(
                lambda v, cid=c.id:
                self._canvas.set_constraint_value(cid, float(v)))
            h.addWidget(spin)
        delete = QPushButton("✕")
        delete.setObjectName("flatBtn")
        delete.setFixedSize(s(20), s(20))
        delete.setToolTip("Delete this constraint")
        delete.clicked.connect(
            lambda _=False, cid=c.id: self._canvas.remove_constraint(cid))
        h.addWidget(delete)
        return row

    def _seq_section_row(self, number: int, item: dict) -> QWidget:
        """A COLLAPSIBLE, content-sized section block: a coloured header (click
        to select the section's shapes; chevron to fold) over a body listing
        the section's operations (one row per shape). The body grows to fit."""
        ink_id = int(item.get("ink_id", 1))
        label, color = self._ink_label_color(ink_id)
        idxs = list(item.get("shape_indices", []))
        length = float(item.get("length_mm", 0.0))
        collapsed = bool(self._seq_collapsed.get(number, False))

        box = QFrame()
        box.setObjectName("seqSection")
        box.setStyleSheet(
            f"QFrame#seqSection {{ border: 1px solid {COLORS['surface1']}; "
            f"border-left: {s(4)}px solid {color}; border-radius: {s(4)}px; "
            f"background: {COLORS['surface0']}; }}")
        outer = QVBoxLayout(box)
        outer.setContentsMargins(s(4), s(3), s(4), s(3))
        outer.setSpacing(s(2))

        header = QHBoxLayout()
        header.setSpacing(s(4))
        chev = QToolButton()
        chev.setText("▸" if collapsed else "▾")
        chev.setAutoRaise(True)
        chev.setCursor(Qt.PointingHandCursor)
        chev.setToolTip("Collapse / expand this section")
        header.addWidget(chev)
        title = QPushButton(
            f"Section {number} · {label} · {len(idxs)} shape(s) · "
            f"{length:.1f} mm")
        title.setCursor(Qt.PointingHandCursor)
        title.setToolTip("Click to select this section's shapes on the canvas.")
        title.setStyleSheet(
            f"QPushButton {{ text-align: left; border: none; "
            f"background: transparent; color: {COLORS['text']}; "
            f"font-size: {_sf(9)}pt; padding: {s(2)}px; }} "
            f"QPushButton:hover {{ color: {COLORS['blue']}; }}")
        title.clicked.connect(
            lambda _=False, ii=idxs: self._canvas.select_indices(ii))
        header.addWidget(title, 1)
        outer.addLayout(header)

        body = QWidget()
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(s(18), 0, 0, s(2))
        body_lay.setSpacing(s(1))
        shapes = self._canvas.sketch().shapes
        for si in idxs:
            if 0 <= si < len(shapes):
                sh = shapes[si]
                op = QLabel(f"▪ {sh.kind} #{si}")
                op.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
                body_lay.addWidget(op)
        body.setVisible(not collapsed)
        outer.addWidget(body)

        def _toggle():
            # Track collapse state explicitly — offscreen isVisible() is
            # unreliable (returns False until the window is shown).
            now = not bool(self._seq_collapsed.get(number, collapsed))
            self._seq_collapsed[number] = now
            body.setVisible(not now)
            chev.setText("▸" if now else "▾")
        chev.clicked.connect(_toggle)
        return box

    def _seq_break_row(self, reason: str) -> QLabel:
        txt = {
            "move": "↳ quick move (needle lifts)",
            "ink_change": "↳ ink replacement (ink change)",
            "layer": "↳ height change",
            "retrace": "↳ retrace along bead (no lift)",
        }.get(reason, "↳ move")
        lbl = QLabel(txt)
        lbl.setStyleSheet(
            f"color: {COLORS.get('overlay1', '#7f849c')}; "
            f"font-size: {_sf(8)}pt; padding-left: {s(10)}px;")
        return lbl

    def _seq_move_row(self, item: dict) -> QLabel:
        lbl = QLabel("⤴ retract & move (quick move)")
        lbl.setStyleSheet(
            f"color: {COLORS.get('overlay1', '#7f849c')}; "
            f"font-size: {_sf(8)}pt; padding-left: {s(6)}px;")
        return lbl

    def _clear_sequence(self):
        while self._seq_layout.count():
            it = self._seq_layout.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()  # reclaim the C++ widget now, not at GC

    def _refresh_sequence(self):
        if not hasattr(self, "_seq_layout"):
            return
        self._clear_sequence()
        self._sync_single_needle_checkbox()
        sk = self._canvas.sketch()
        single = self._effective_single()
        try:
            items = plan_print_sections(sk, self._needle, single_needle=single)
        except Exception as e:
            logger.debug(f"plan_print_sections failed: {e}")
            items = []
        if not items:
            empty = QLabel("Draw shapes to see the print sequence.")
            empty.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            self._seq_layout.addWidget(empty)
            self._seq_summary_lbl.setText("")
            return
        sec_n = 0
        travels = 0
        for it in items:
            if it.get("type") == "move":
                travels += 1
                self._seq_layout.addWidget(self._seq_move_row(it))
            else:
                sec_n += 1
                if it.get("break_before"):
                    self._seq_layout.addWidget(
                        self._seq_break_row(it["break_before"]))
                self._seq_layout.addWidget(self._seq_section_row(sec_n, it))
        self._seq_layout.addStretch(1)
        mode = "single-needle" if single else "multi-needle"
        nl = max(1, int(sk.num_layers))
        layers = f" · ×{nl} layers" if nl > 1 else ""
        self._seq_summary_lbl.setText(
            f"{sec_n} section(s) · {travels} move(s) · {mode}{layers}")

    # ── Properties panel ──────────────────────────────────────────

    def _group(self, title: str) -> QGroupBox:
        # Hardware-Setup look: frosted lighter box on the dark panel with a
        # blue-on-dark section-title pill (shared style from gui/styles.py).
        grp = QGroupBox(title)
        grp.setStyleSheet(build_section_title_style(scale_factor()))
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(s(6), s(6), s(6), s(8))
        lay.setSpacing(s(6))
        return grp

    def _dspin(self, val, lo, hi, step=0.5, suffix=" mm"):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setDecimals(2)
        sb.setSingleStep(step)
        sb.setValue(val)
        sb.setSuffix(suffix)
        return sb

    def _field_row(self, parent_lay, label, widget):
        row = QHBoxLayout()
        lab = QLabel(label)
        lab.setMinimumWidth(s(70))
        lab.setStyleSheet(f"color: {COLORS['subtext0']};")
        row.addWidget(lab)
        row.addWidget(widget, 1)
        parent_lay.addLayout(row)

    def _clear_props(self):
        while self._props_layout.count():
            it = self._props_layout.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()  # reclaim the C++ widget now, not at GC

    def _rebuild_props(self):
        self._building = True
        self._clear_props()
        count = self._canvas.selection_count()
        sh = self._canvas.selected_shape()
        if count >= 2:
            self._props_layout.addWidget(self._build_group_card(count))
        elif sh is not None:
            self._props_layout.addWidget(self._build_shape_card(sh))
            if sh.kind != "travel":
                self._props_layout.addWidget(self._build_backtrace_card())
        else:
            hint = QLabel(
                "Pick a tool and draw on the canvas.\n"
                "Select a shape to edit its exact size, or drag a box around "
                "several (or Ctrl+A) to move / resize them together.")
            hint.setWordWrap(True)
            hint.setStyleSheet(f"color: {COLORS['subtext0']}; "
                               f"font-size: {_sf(9)}pt;")
            self._props_layout.addWidget(hint)
            if self._canvas.sketch().shapes:
                sel_all = QPushButton("Select all")
                sel_all.clicked.connect(lambda: self._canvas.select_all())
                self._props_layout.addWidget(sel_all)
        self._props_layout.addWidget(self._build_print_card())
        self._props_layout.addStretch(1)
        self._building = False

    def _build_group_card(self, count: int) -> QGroupBox:
        grp = self._group(f"Selection — {count} shapes")
        lay = grp.layout()
        info = QLabel("Drag the corner handle to resize the whole selection, "
                      "or enter an exact scale factor and Apply. Drag inside "
                      "the box to move them together.")
        info.setWordWrap(True)
        info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        lay.addWidget(info)

        scale = self._dspin(1.0, 0.05, 20.0, 0.1, "×")
        self._field_row(lay, "Scale", scale)
        apply = QPushButton("Apply scale")
        apply.clicked.connect(lambda: self._apply_group_scale(scale))
        lay.addWidget(apply)

        delete = QPushButton("Delete selection")
        delete.setObjectName("dangerBtn")
        delete.clicked.connect(self._canvas_delete)
        lay.addWidget(delete)
        return grp

    def _apply_group_scale(self, spin):
        self._canvas.scale_selection(float(spin.value()))
        spin.setValue(1.0)

    def _build_shape_card(self, sh: SketchShape) -> QGroupBox:
        grp = self._group("Retract point" if sh.kind == "travel"
                          else f"Shape — {sh.kind}")
        lay = grp.layout()

        # Retract-&-move point: the needle lifts here and travels (pen-up) to
        # this spot; only its position is editable. It splits the print into
        # separate runs for back-tracing.
        if sh.kind == "travel":
            info = QLabel(
                "The needle lifts here and travels (pen-up) to this point — a "
                "break in the print. It also bounds the runs that “Back-trace” "
                "retraces.")
            info.setWordWrap(True)
            info.setStyleSheet(f"color: {COLORS['subtext0']}; "
                               f"font-size: {_sf(9)}pt;")
            lay.addWidget(info)
            x = self._dspin(sh.cx, -500, 500)
            y = self._dspin(sh.cy, -500, 500)
            x.valueChanged.connect(lambda v: self._set(sh, "cx", v))
            y.valueChanged.connect(lambda v: self._set(sh, "cy", v))
            self._field_row(lay, "X", x)
            self._field_row(lay, "Y", y)
            delete = QPushButton("Delete point")
            delete.setObjectName("dangerBtn")
            delete.clicked.connect(self._canvas_delete)
            lay.addWidget(delete)
            return grp

        # Region = baked paint-bucket fill: only pump + delete are editable.
        if sh.kind == "region":
            info = QLabel(f"Filled region · {len(sh.points)} points")
            info.setStyleSheet(f"color: {COLORS['subtext0']};")
            lay.addWidget(info)
            self._field_row(lay, "Ink", self._make_ink_combo(sh))
            delete = QPushButton("Delete region")
            delete.setObjectName("dangerBtn")
            delete.clicked.connect(self._canvas_delete)
            lay.addWidget(delete)
            return grp

        # Geometry fields
        if sh.kind == "circle":
            cx = self._dspin(sh.cx, -500, 500)
            cy = self._dspin(sh.cy, -500, 500)
            r = self._dspin(sh.radius, 0.1, 500)
            cx.valueChanged.connect(lambda v: self._set(sh, "cx", v))
            cy.valueChanged.connect(lambda v: self._set(sh, "cy", v))
            r.valueChanged.connect(lambda v: self._set(sh, "radius", v))
            self._field_row(lay, "Center X", cx)
            self._field_row(lay, "Center Y", cy)
            self._field_row(lay, "Radius", r)
        elif sh.kind == "ellipse":
            cx = self._dspin(sh.cx, -500, 500)
            cy = self._dspin(sh.cy, -500, 500)
            rx = self._dspin(sh.rx, 0.1, 500)
            ry = self._dspin(sh.ry, 0.1, 500)
            cx.valueChanged.connect(lambda v: self._set(sh, "cx", v))
            cy.valueChanged.connect(lambda v: self._set(sh, "cy", v))
            rx.valueChanged.connect(lambda v: self._set(sh, "rx", v))
            ry.valueChanged.connect(lambda v: self._set(sh, "ry", v))
            self._field_row(lay, "Center X", cx)
            self._field_row(lay, "Center Y", cy)
            self._field_row(lay, "Radius X", rx)
            self._field_row(lay, "Radius Y", ry)
        elif sh.kind == "rect":
            cx = self._dspin(sh.cx, -500, 500)
            cy = self._dspin(sh.cy, -500, 500)
            w = self._dspin(sh.width, 0.1, 500)
            h = self._dspin(sh.height, 0.1, 500)
            cx.valueChanged.connect(lambda v: self._set(sh, "cx", v))
            cy.valueChanged.connect(lambda v: self._set(sh, "cy", v))
            w.valueChanged.connect(lambda v: self._set(sh, "width", v))
            h.valueChanged.connect(lambda v: self._set(sh, "height", v))
            self._field_row(lay, "Center X", cx)
            self._field_row(lay, "Center Y", cy)
            self._field_row(lay, "Width", w)
            self._field_row(lay, "Height", h)
        elif sh.kind == "line" and len(sh.points) >= 2:
            x1 = self._dspin(sh.points[0][0], -500, 500)
            y1 = self._dspin(sh.points[0][1], -500, 500)
            x2 = self._dspin(sh.points[1][0], -500, 500)
            y2 = self._dspin(sh.points[1][1], -500, 500)
            x1.valueChanged.connect(lambda v: self._set_pt(sh, 0, 0, v))
            y1.valueChanged.connect(lambda v: self._set_pt(sh, 0, 1, v))
            x2.valueChanged.connect(lambda v: self._set_pt(sh, 1, 0, v))
            y2.valueChanged.connect(lambda v: self._set_pt(sh, 1, 1, v))
            self._field_row(lay, "Start X", x1)
            self._field_row(lay, "Start Y", y1)
            self._field_row(lay, "End X", x2)
            self._field_row(lay, "End Y", y2)
        elif sh.kind == "polygon":
            lab = QLabel(f"{len(sh.points)} vertices (drag to move)")
            lab.setStyleSheet(f"color: {COLORS['subtext0']};")
            lay.addWidget(lab)

        # Fill (line has no fill)
        if sh.kind != "line":
            fill = QCheckBox("Filled (raster infill)")
            fill.setChecked(sh.filled)
            fill.toggled.connect(lambda v: self._set(sh, "filled", v))
            lay.addWidget(fill)

        # Printed bead width (thicker = more passes + more volume)
        lw = self._dspin(sh.line_width_mm, 0.05, 50, 0.1)
        lw.valueChanged.connect(lambda v: self._set(sh, "line_width_mm", v))
        self._field_row(lay, "Line width", lw)

        # Ink assignment (abstract — the physical pump is chosen later, at
        # print time, in Quick Print's ink-mapping step).
        self._field_row(lay, "Ink", self._make_ink_combo(sh))

        # Closure overlap (closed-loop outlines only): continue PAST the seam so
        # the deposited ink fully closes — by one needle Ø or a typed distance.
        if self._shape_is_closed_loop(sh):
            self._build_closure_overlap_row(lay, sh)

        # Print-start control (continuity): outline shapes only (not fills).
        if sh.kind in ("line", "circle", "ellipse", "rect", "polygon") \
                and not sh.filled:
            custom = getattr(sh, "start_point", None) is not None
            start_lbl = QLabel(
                "Print start: " + ("custom" if custom else "default (auto)"))
            start_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            lay.addWidget(start_lbl)
            start_hint = QLabel(
                "Drag the green ▸ marker on the canvas to set where this shape "
                "starts printing — it snaps to existing lines, so it can begin "
                "exactly where the previous shape ended (no pen-up between them).")
            start_hint.setWordWrap(True)
            start_hint.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
            lay.addWidget(start_hint)
            reset_start = QPushButton("Reset start point")
            reset_start.setEnabled(custom)
            reset_start.clicked.connect(self._reset_start_point)
            lay.addWidget(reset_start)

        # Print-END control: OPEN shapes only — drag the red ■ marker to trim
        # where printing stops (with the start marker, prints a sub-segment).
        # Closed shapes use the closure-overlap row above for their end.
        if sh.kind in ("line", "polygon") and not sh.filled \
                and not self._shape_is_closed_loop(sh):
            has_end = getattr(sh, "end_point", None) is not None
            end_lbl = QLabel(
                "Print end: " + ("custom" if has_end else "default (far end)"))
            end_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            lay.addWidget(end_lbl)
            end_hint = QLabel(
                "Drag the red ■ marker on the canvas to trim where this shape "
                "stops printing.")
            end_hint.setWordWrap(True)
            end_hint.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
            lay.addWidget(end_hint)
            reset_end = QPushButton("Reset end point")
            reset_end.setEnabled(has_end)
            reset_end.clicked.connect(self._reset_end_point)
            lay.addWidget(reset_end)

        delete = QPushButton("Delete shape")
        delete.setObjectName("dangerBtn")
        delete.clicked.connect(self._canvas_delete)
        lay.addWidget(delete)
        return grp

    def _build_backtrace_card(self) -> QGroupBox:
        """Back-trace the run the selected shape belongs to: append a reversed
        return pass offset in height + in-plane, optionally extruding."""
        grp = self._group("Back-trace this path")
        lay = grp.layout()
        info = QLabel(
            "Retrace the continuous run this shape is part of (the lines "
            "between retract points), reversed and offset. Positive Z lays it "
            "above the print; the in-plane offset shifts it sideways (a "
            "parallel bead / return route).")
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
        lay.addWidget(info)

        zoff = self._dspin(self._bt_z_offset, -40.0, 40.0, 0.1, " mm")
        zoff.setToolTip(
            "Height offset of the return pass: + above the printed path "
            "(second bead on top), − below (drag lower).")
        zoff.valueChanged.connect(
            lambda v: setattr(self, "_bt_z_offset", float(v)))
        self._field_row(lay, "Z offset", zoff)

        xyoff = self._dspin(self._bt_xy_offset, -40.0, 40.0, 0.1, " mm")
        xyoff.setToolTip(
            "In-plane offset — shifts the return pass perpendicular to the "
            "path (a parallel bead beside the first). 0 = retrace exactly; "
            "+/− picks the side.")
        xyoff.valueChanged.connect(
            lambda v: setattr(self, "_bt_xy_offset", float(v)))
        self._field_row(lay, "In-plane offset", xyoff)

        extrude = QCheckBox("Extrude on return (lay a second bead)")
        extrude.setChecked(self._bt_extrude)
        extrude.setToolTip(
            "On: the return pass deposits material. Off: the needle just moves "
            "back through the path without extruding.")
        extrude.toggled.connect(
            lambda v: setattr(self, "_bt_extrude", bool(v)))
        lay.addWidget(extrude)

        btn = QPushButton("Back-trace this path")
        btn.clicked.connect(self._backtrace_selected)
        lay.addWidget(btn)
        return grp

    def _backtrace_selected(self):
        idx = self._canvas.selected_index()
        if idx < 0:
            self._status_warn("Select one shape in the run to back-trace.")
            return
        # A negative Z offset lays the return pass at / below the printed bead —
        # the needle can contact the deposited material. Confirm (non-blocking).
        if self._bt_z_offset < 0:
            resp = QMessageBox.warning(
                self, "Return pass below the print",
                f"A negative Z offset ({self._bt_z_offset:+.2f} mm) puts the "
                f"back-trace at or below the printed path — the needle may "
                f"contact the deposited bead.\n\nBack-trace anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if resp != QMessageBox.Yes:
                return
        n = self._canvas.backtrace_run(
            idx, z_offset=self._bt_z_offset, xy_offset=self._bt_xy_offset,
            print_on_return=self._bt_extrude)
        if n <= 0:
            self._status_warn("Nothing to back-trace.")
            return
        mode = "extruding" if self._bt_extrude else "move-only"
        self._status_ok(
            f"✓ Back-traced {n} shape(s) — Z {self._bt_z_offset:+.2f} mm, "
            f"in-plane {self._bt_xy_offset:+.2f} mm, {mode}")

    def _build_print_card(self) -> QGroupBox:
        sk = self._canvas.sketch()
        grp = self._group("Print parameters")
        lay = grp.layout()

        # Print name — used to name the baked print object.
        name_edit = QLineEdit(self._print_name)
        name_edit.setPlaceholderText("Sketch")
        name_edit.setToolTip("Name for the print saved to Print Setup.")
        name_edit.textChanged.connect(self._set_print_name)
        self._field_row(lay, "Name", name_edit)

        # One-click path optimizer — reorder shapes + set start points to
        # minimize pen-up travels (retract points stay as fixed breaks).
        opt_btn = QPushButton("✨ Optimize print path")
        opt_btn.setToolTip(
            "Reorder the shapes and choose each one's start point/direction to "
            "chain them into the fewest pen-up travels (discontinuities). Your "
            "retract points stay as fixed breaks. Undoable.")
        opt_btn.clicked.connect(self._optimize_path)
        lay.addWidget(opt_btn)

        # Overlap travel (optimizer path flexibility): retrace along an existing
        # bead for a brief length instead of lifting. Applied by Optimize.
        ot_enable = QCheckBox("Overlap travel — retrace the bead (no lift)")
        ot_enable.setChecked(bool(getattr(sk, "overlap_travel_enabled", False)))
        ot_enable.setToolTip(
            "When the next shape starts a brief distance back along an already-"
            "printed bead (same ink), keep the needle DOWN and retrace there "
            "instead of lifting. Click 'Optimize print path' to apply.")
        ot_enable.toggled.connect(self._on_overlap_travel_toggled)
        lay.addWidget(ot_enable)

        ot_max = self._dspin(
            float(getattr(sk, "overlap_travel_max_mm", 5.0)), 0.1, 100.0, 0.5)
        ot_max.setToolTip("Longest bead length the needle may retrace instead "
                          "of lifting.")
        ot_max.valueChanged.connect(
            lambda v: self._set_sketch("overlap_travel_max_mm", v))
        self._field_row(lay, "Max retrace (mm)", ot_max)

        ot_pause = QCheckBox("Pause pump on retrace (hold pressure)")
        ot_pause.setChecked(bool(getattr(sk, "overlap_travel_pause_pump", True)))
        ot_pause.setToolTip(
            "Pause (not stop) the pump while retracing so it deposits ~nothing "
            "over the existing bead yet never relieves pressure. Off = lay a "
            "second bead on the return.")
        ot_pause.toggled.connect(
            lambda v: self._set_sketch("overlap_travel_pause_pump", v))
        lay.addWidget(ot_pause)

        ot_speed = self._dspin(
            float(getattr(sk, "overlap_travel_speed_factor", 1.0)),
            0.1, 10.0, 0.5)
        ot_speed.setToolTip("Move faster during the retrace (× print speed).")
        ot_speed.valueChanged.connect(
            lambda v: self._set_sketch("overlap_travel_speed_factor", v))
        self._field_row(lay, "Retrace speed ×", ot_speed)

        if self._needle_od_mm > 0:
            info = QLabel(
                f"Needle Ø {self._needle_od_mm:.2f} mm — sets bead width "
                f"& raster step")
            info.setWordWrap(True)
            info.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            lay.addWidget(info)

        # v7.5.x: extrusion multiplier — scales the shaded thickness band on the
        # canvas AND the deposited volume of the baked print. 1× ≈ a bead the
        # needle inner Ø wide; smaller = thinner, larger = thicker.
        em = self._dspin(sk.extrusion_multiplier, 0.05, 5.0, 0.1, "×")
        em.setToolTip(
            "Extrusion multiplier — how much material is laid per mm.\n"
            "1.0× ≈ a bead the width of the needle inner Ø; 0.5× thinner, "
            "2× thicker. Scales the shaded thickness preview AND the deposited "
            "volume of the baked print.")
        em.valueChanged.connect(self._on_extrusion_changed)
        self._field_row(lay, "Extrusion", em)
        self._bead_info_lbl = QLabel(self._bead_info_text())
        self._bead_info_lbl.setWordWrap(True)
        self._bead_info_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        lay.addWidget(self._bead_info_lbl)

        # v7.5.x: print height is measured UP from the calibrated plate bottom
        # (not an absolute Z). 0 = at the plate bottom; larger = higher.
        zs = self._dspin(sk.z_start_mm, 0.0, 40.0, 0.1)
        zs.setToolTip(
            "Height of the first layer above the calibrated plate bottom.\n"
            "0 = at the plate bottom; larger = higher. The needle is clamped "
            "so it can never punch through the plate bottom.")
        lh = self._dspin(sk.layer_height_mm, 0.01, 10, 0.05)
        nl = QSpinBox()
        nl.setRange(1, 999)
        nl.setValue(sk.num_layers)
        sp = self._dspin(sk.print_speed_mm_s, 0.1, 200, 0.5, " mm/s")
        ls = self._dspin(sk.line_spacing_mm, 0.05, 20, 0.05)
        # Lift the needle this far above the print to travel between shapes
        # (and passes/layers), so it clears already-printed material.
        lift = self._dspin(sk.travel_clearance_mm, 0.0, 40.0, 0.5)
        lift.setToolTip(
            "How far the needle lifts above the print to travel between "
            "shapes. Larger = safer clearance over printed material; 0 = "
            "just above the top layer.")
        # The first part of every lift runs slowly so back-pressure / surface
        # tension can't peel the printed bead up with the needle.
        lift_slow_d = self._dspin(sk.lift_slow_dist_mm, 0.0, 20.0, 0.5, " mm")
        lift_slow_d.setToolTip(
            "Distance of the SLOW first part of every needle lift. The needle "
            "retracts this far slowly so the deposited bead doesn't lift off "
            "with it, then raises the rest of the way at travel speed.")
        lift_slow_v = self._dspin(
            sk.lift_slow_speed_mm_s, 0.05, 50.0, 0.25, " mm/s")
        lift_slow_v.setToolTip("Speed of the slow first part of the lift.")

        zs.valueChanged.connect(lambda v: self._set_sketch("z_start_mm", v))
        lh.valueChanged.connect(lambda v: self._set_sketch("layer_height_mm", v))
        nl.valueChanged.connect(lambda v: self._set_sketch("num_layers", v))
        sp.valueChanged.connect(lambda v: self._set_sketch("print_speed_mm_s", v))
        ls.valueChanged.connect(lambda v: self._set_sketch("line_spacing_mm", v))
        lift.valueChanged.connect(
            lambda v: self._set_sketch("travel_clearance_mm", v))
        lift_slow_d.valueChanged.connect(
            lambda v: self._set_sketch("lift_slow_dist_mm", v))
        lift_slow_v.valueChanged.connect(
            lambda v: self._set_sketch("lift_slow_speed_mm_s", v))

        self._field_row(lay, "Print height", zs)
        self._field_row(lay, "Layer h", lh)
        self._field_row(lay, "# Layers", nl)
        self._field_row(lay, "Lift between shapes", lift)
        self._field_row(lay, "Slow-lift dist", lift_slow_d)
        self._field_row(lay, "Slow-lift speed", lift_slow_v)
        self._field_row(lay, "Speed", sp)
        self._field_row(lay, "Raster step", ls)
        if self._plate_bottom_z is None:
            warn = QLabel("⚠ Plate bottom not calibrated — height is absolute "
                          "until you calibrate.")
            warn.setWordWrap(True)
            warn.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; "
                f"font-size: {_sf(8)}pt;")
            lay.addWidget(warn)
        return grp

    # ── Model edits ───────────────────────────────────────────────

    def _set(self, shape, attr, value):
        if self._building:
            return
        setattr(shape, attr, value)
        self._canvas.solve_after_edit()    # re-satisfy constraints (no-op if none)
        self._canvas.update()
        self._schedule_preview()

    def _set_pt(self, shape, idx, axis, value):
        if self._building or len(shape.points) <= idx:
            return
        x, y = shape.points[idx]
        shape.points[idx] = (value, y) if axis == 0 else (x, value)
        self._canvas.solve_after_edit()    # re-satisfy constraints (no-op if none)
        self._canvas.update()
        self._schedule_preview()

    def _make_ink_combo(self, shape) -> QComboBox:
        """A combo of the sketch's abstract inks (data = ink id) bound to a
        shape's ``ink_id``. Editing it re-colours the shape from the ink and
        makes new shapes inherit that ink."""
        combo = QComboBox()
        sk = self._canvas.sketch()
        for ink in sk.inks:
            combo.addItem(ink.name or f"Ink {ink.id}", int(ink.id))
        idx = combo.findData(int(shape.ink_id))
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.currentIndexChanged.connect(
            lambda _i, c=combo, sh=shape: self._set_ink(sh, c.currentData()))
        return combo

    def _set_ink(self, shape, ink_id):
        if self._building or ink_id is None:
            return
        sk = self._canvas.sketch()
        shape.ink_id = int(ink_id)
        ink = sk.ink_by_id(shape.ink_id)
        if ink is not None:
            shape.color = ink.color
        self._canvas.set_active_ink(shape.ink_id)  # new shapes inherit
        self._canvas.update()
        self._schedule_preview()

    def _set_sketch(self, attr, value):
        if self._building:
            return
        setattr(self._canvas.sketch(), attr, value)
        self._schedule_preview()

    def _set_print_name(self, text):
        if self._building:
            return
        self._print_name = text

    def _on_extrusion_changed(self, value):
        if self._building:
            return
        self._canvas.sketch().extrusion_multiplier = float(value)
        self._apply_bead_width()
        if hasattr(self, "_bead_info_lbl"):
            self._bead_info_lbl.setText(self._bead_info_text())
        self._schedule_preview()           # volume/stats change with extrusion

    # ── Bead-width (print thickness) ──────────────────────────────

    def _bead_ref_mm(self) -> float:
        """1× reference bead width = needle inner Ø (mm); falls back to the
        fill pitch (line spacing) when no needle is configured."""
        if self._needle_id_mm > 0:
            return self._needle_id_mm
        return float(self._canvas.sketch().line_spacing_mm or 0.0)

    def _bead_width_mm(self) -> float:
        """Deposited bead width to shade = 1× reference × extrusion multiplier."""
        return self._bead_ref_mm() * float(
            self._canvas.sketch().extrusion_multiplier)

    def _apply_bead_width(self):
        self._canvas.set_bead_width_mm(self._bead_width_mm())

    def _bead_info_text(self) -> str:
        mult = float(self._canvas.sketch().extrusion_multiplier)
        ref = self._bead_ref_mm()
        if ref <= 0:
            return "Configure a needle to size the bead from its inner Ø."
        src = "needle inner Ø" if self._needle_id_mm > 0 else "fill pitch"
        return (f"Shaded bead ≈ {ref * mult:.3f} mm at {mult:.2f}× "
                f"(1× = {src} {ref:.3f} mm)")

    @staticmethod
    def _shape_is_closed_loop(sh: SketchShape) -> bool:
        """Whether the shape is an unfilled closed loop (has a seam to over-close)."""
        if getattr(sh, "filled", False):
            return False
        if sh.kind in ("circle", "ellipse", "rect"):
            return True
        return sh.kind == "polygon" and len(sh.points) >= 3

    def _canvas_delete(self):
        self._canvas.delete_selected()

    def _reset_start_point(self):
        """Clear the selected shape's custom print start (back to the default)."""
        idx = self._canvas.selected_index()
        if idx < 0:
            return
        self._canvas.clear_start_point(idx)
        self._rebuild_props()          # refresh the custom/default label + button

    def _reset_end_point(self):
        """Clear an OPEN shape's custom print end (back to the far endpoint)."""
        idx = self._canvas.selected_index()
        if idx < 0:
            return
        self._canvas.clear_end_point(idx)
        self._rebuild_props()

    # ── Closure overlap ───────────────────────────────────────────

    _OVERLAP_MODES = [("None", "none"), ("Needle Ø", "needle"),
                      ("Custom distance", "distance")]

    def _build_closure_overlap_row(self, lay, sh: SketchShape):
        """Closure-overlap control for a closed loop: a mode combo (None /
        Needle Ø / Custom distance) + a distance spin shown for Custom. The end
        marker on the canvas is the same value — dragging it sets the distance."""
        combo = QComboBox()
        for label, data in self._OVERLAP_MODES:
            combo.addItem(label, data)
        mode = getattr(sh, "overlap_mode", "none") or "none"
        idx = combo.findData(mode)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.setToolTip(
            "Continue printing PAST the seam so the deposited ink fully closes "
            "(on re-entry the needle pushes ink aside). 'Needle Ø' overshoots "
            "one needle outer diameter; 'Custom distance' overshoots the typed "
            "mm. Drag the red ■ marker on the canvas to set it graphically.")
        spin = self._dspin(max(float(getattr(sh, "overlap_distance_mm", 0.0)),
                               0.0), 0.0, 100.0, 0.1)
        spin.setVisible(mode == "distance")
        spin.valueChanged.connect(
            lambda v, s=sh: self._set(s, "overlap_distance_mm", float(v)))
        combo.currentIndexChanged.connect(
            lambda _i, c=combo, s=sh, sp=spin:
            self._on_overlap_mode_changed(s, c.currentData(), sp))
        self._field_row(lay, "Closure overlap", combo)
        self._field_row(lay, "Distance", spin)

    def _on_overlap_mode_changed(self, sh, mode, spin):
        if self._building:
            return
        mode = mode or "none"
        sh.overlap_mode = mode
        # Seed a visible default when switching to Custom so the end marker
        # appears off the seam (else it sits exactly on the start flag).
        if mode == "distance" and float(getattr(sh, "overlap_distance_mm",
                                                 0.0)) <= 0.0:
            seed = self._needle_od_mm if self._needle_od_mm > 0 else 0.5
            sh.overlap_distance_mm = round(float(seed), 3)
        spin.blockSignals(True)
        spin.setValue(float(getattr(sh, "overlap_distance_mm", 0.0)))
        spin.setVisible(mode == "distance")
        spin.blockSignals(False)
        self._canvas.update()
        self._schedule_preview()

    def _on_overlap_travel_toggled(self, checked: bool):
        self._set_sketch("overlap_travel_enabled", bool(checked))
        self._refresh_sequence()

    def _optimize_path(self):
        """Reorder shapes + set start points to minimize pen-up discontinuities,
        reporting how many travels were removed."""
        from SupportClasses.SketchTrajectory import count_discontinuities
        sk = self._canvas.sketch()
        if len(sk.shapes) < 2:
            self._status_warn("Draw at least two shapes to optimize.")
            return
        before = count_discontinuities(sk, self._needle, self._syringe)
        self._canvas.optimize(self._needle)
        after = count_discontinuities(self._canvas.sketch(),
                                      self._needle, self._syringe)
        if after < before:
            self._status_ok(
                f"✓ Optimized print path — {before} → {after} travel(s)")
        else:
            self._status_ok(f"Print path already optimal — {before} travel(s)")

    def _toggle_snap(self, on):
        self._canvas.set_snap(1.0 if on else 0.0)

    def _toggle_osnap(self, on):
        self._canvas.set_object_snap(bool(on))

    def _toggle_thickness(self, on):
        self._canvas.set_show_thickness(bool(on))

    # ── Canvas callbacks ──────────────────────────────────────────

    def _on_sketch_changed(self):
        self._schedule_preview()

    def _on_selection_changed(self, _index):
        self._rebuild_props()
        self._refresh_constraint_buttons()

    def _on_fill_result(self, ok: bool):
        if ok:
            self._status_lbl.setText("✓ Region filled")
            self._status_lbl.setStyleSheet(
                f"color: {COLORS['green']}; font-size: {_sf(9)}pt;")
        else:
            self._status_lbl.setText(
                "⚠ Click inside an area fully enclosed by shapes")
            self._status_lbl.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; "
                f"font-size: {_sf(9)}pt;")

    # ── Preview + send ────────────────────────────────────────────

    def _schedule_preview(self):
        self._preview_timer.start()

    def _recompute_preview(self):
        sk = self._canvas.sketch()
        self._refresh_sequence()           # keep the sequence panel in sync
        self._refresh_constraints_card()   # constraints list + DOF status
        try:
            result = compile_to_trajectory(sk, self._needle, self._syringe)
        except Exception as e:
            logger.warning(f"sketch compile failed: {e}")
            self._canvas.set_toolpath(None, None)
            self._profile_view.clear()
            self._stats_lbl.setText("Compile error")
            return
        self._last_result = result
        if result.is_empty:
            self._canvas.set_toolpath(None, None)
            self._profile_view.clear()
            self._stats_lbl.setText("Empty sketch — draw a shape")
            self._send_btn.setEnabled(False)
            self._update_bounds_warning(None)
            return

        # Render the compiled toolpath as the main raster view + side profile.
        self._canvas.set_toolpath(result.trajectory, result.pump_states)
        self._profile_view.set_trajectory(result.trajectory, result.pump_states)
        self._stats_lbl.setText(
            f"{result.num_waypoints} wpts · {result.num_layers} layer(s) · "
            f"path {result.total_length_mm:.1f} mm · "
            f"~{result.total_time_s:.1f} s · {result.total_volume_uL:.2f} µL")
        self._send_btn.setEnabled(True)
        self._update_bounds_warning(result.trajectory)

    # ── Bake / send / save ────────────────────────────────────────

    def _compile_for_export(self):
        """Compile the sketch and run the empty / needle-safe-boundary gates.
        Returns ``(sketch, CompiledSketch)`` or ``None`` if the operator should
        not proceed."""
        sk = self._canvas.sketch()
        result = compile_to_trajectory(sk, self._needle, self._syringe)
        if result.is_empty:
            QMessageBox.warning(self, "Empty sketch",
                                "Draw at least one shape first.")
            return None
        # Non-blocking safe-boundary guard: warn (and confirm), but allow.
        if self._exceeds_safe_boundary(result.trajectory):
            resp = QMessageBox.warning(
                self, "Outside needle-safe boundary",
                f"This sketch extends past the needle-safe boundary for well "
                f"{self._selected_well or '?'} — the needle could contact the "
                f"well wall when printing.\n\nProceed anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if resp != QMessageBox.Yes:
                return None
        return sk, result

    def _do_bake(self, sk, result, *, base_name: str, overwrite: bool):
        """Bake ``result`` to a csv_import print (embedding the vector Sketch so
        it can be re-edited losslessly). Returns the saved name, or None on
        error. ``overwrite`` replaces the print of that name (Save changes)."""
        base_name = (base_name or "").strip() or "Sketch"

        # The sketch's Z column is a *height above the plate bottom*. Bake it
        # into the internal zero-ref frame when the plate bottom is known so the
        # trajectory is physically correct; always record the relative height.
        traj = result.trajectory
        if self._plate_bottom_z is not None:
            try:
                from SupportClasses.StageController import plate_relative_to_zref
                traj = traj.copy()
                traj[:, 2] = plate_relative_to_zref(
                    self._plate_bottom_z, traj[:, 2])
            except Exception as e:
                logger.debug(f"sketch Z→zero-ref bake skipped: {e}")
                traj = result.trajectory

        extra_params = {
            "z_above_plate_bottom_mm": float(sk.z_start_mm),
            "z_datum": "plate_bottom",
            "layer_height_mm": float(sk.layer_height_mm),
            "num_layers": int(sk.num_layers),
            "extrusion_multiplier": float(sk.extrusion_multiplier),
            # The vector Sketch itself — lets the Library "Edit in Sketch"
            # reload the exact shapes instead of the baked toolpath.
            "sketch": sk.to_dict(),
        }
        save_kwargs = dict(
            base_name=base_name,
            description=(f"{base_name}: {len(sk.shapes)} shape(s), "
                         f"{result.num_layers} layer(s), "
                         f"{result.total_length_mm:.1f} mm path · "
                         f"print height {sk.z_start_mm:.2f} mm above plate "
                         f"bottom"),
            color="#cba6f7",
            author="Print Builder",
            source="SketchTrajectory",
            object_name=base_name,
            extra_params=extra_params,
            overwrite=overwrite,
        )
        if self._prints_dir:
            save_kwargs["prints_dir"] = self._prints_dir
        try:
            return save_trajectory_as_print_object(traj, **save_kwargs)
        except Exception as e:
            logger.error(f"sketch bake failed: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Failed:\n{e}")
            return None

    def _send_to_print_setup(self):
        cfr = self._compile_for_export()
        if cfr is None:
            return
        sk, result = cfr
        name = self._do_bake(sk, result,
                             base_name=self._print_name, overwrite=False)
        if name is None:
            return
        # Track the just-created print as the edit target so a later "Save
        # changes" updates THIS print, not a previously-edited one (a stale
        # _editing_name would otherwise silently overwrite the wrong file).
        self._editing_name = name
        self._editing_stem = name
        self._update_edit_ui()
        self._status_ok(f"✓ Sent as '{name}'")
        self.print_file_created.emit(name)

    def _save_changes(self):
        """Overwrite the print currently being edited (opened from Library)."""
        if not self._editing_name:
            return
        cfr = self._compile_for_export()
        if cfr is None:
            return
        sk, result = cfr
        # Overwrite the exact file that was opened (its real stem), not a
        # re-sanitized display name — otherwise a stem≠sanitize(name) print
        # would be duplicated instead of replaced.
        name = self._do_bake(sk, result,
                             base_name=self._editing_stem or self._editing_name,
                             overwrite=True)
        if name is None:
            return
        self._status_ok(f"✓ Saved changes to '{self._editing_name}'")
        self.print_file_saved.emit(name)

    def _status_ok(self, text: str):
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {_sf(9)}pt;")

    def _status_warn(self, text: str):
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: {_sf(9)}pt;")

    # ── Edit an existing print ────────────────────────────────────

    def load_print_for_edit(self, name: str, prints_dir: str | None = None):
        """Open a saved print back in the editor. Prints created in Sketch
        carry their vector Sketch (lossless reload); others are imported
        best-effort from their baked toolpath as movable/scalable regions."""
        path = None
        try:
            from SupportClasses.PrintFileManager import (
                PrintFileManager, read_print_objects,
            )
            mgr = PrintFileManager(prints_dir) if prints_dir else PrintFileManager()
            path = mgr._find_path(name)
        except Exception as e:
            logger.warning(f"edit: cannot resolve print '{name}': {e}")
        if path is None:
            self._status_warn(f"⚠ Could not find print '{name}'.")
            return
        # Remember where this print lives so "Save changes" writes back there —
        # the real on-disk stem (not a re-sanitized display name) so overwrite
        # replaces this exact file instead of orphaning it.
        self._prints_dir = prints_dir
        edit_stem = path.stem
        objects = read_print_objects(path)

        sk = self._sketch_from_stored(objects)
        best_effort = False
        if sk is None:
            sk = self._sketch_from_trajectory(objects)
            best_effort = True
        if sk is None or not sk.shapes:
            self._status_warn(
                f"⚠ '{name}' has no editable vector data — open it in Print "
                f"Setup to adjust parameters instead.")
            return
        self._editing_stem = edit_stem
        self._begin_edit(sk, name, best_effort=best_effort)

    @staticmethod
    def _stored_sketch_dict(objects) -> dict | None:
        for obj in (objects or {}).values():
            if isinstance(obj, dict):
                params = obj.get("params") or {}
                sk = params.get("sketch")
                if isinstance(sk, dict) and "shapes" in sk:
                    return sk
        return None

    def _sketch_from_stored(self, objects):
        d = self._stored_sketch_dict(objects)
        if d is None:
            return None
        try:
            return Sketch.from_dict(d)
        except Exception as e:
            logger.debug(f"stored sketch parse failed: {e}")
            return None

    def _sketch_from_trajectory(self, objects):
        """Best-effort: rebuild a Sketch from the baked toolpath(s) of a print
        that carries no stored Sketch (older prints, image/CSV imports)."""
        from SupportClasses.SketchTrajectory import regions_from_trajectory
        from gui.pages.print_library import object_trajectory
        try:
            from SupportClasses.StageController import zref_to_plate_relative
        except Exception:
            zref_to_plate_relative = None

        syringe_map = {"P1": self._syringe} if self._syringe is not None else {}
        shapes: list[SketchShape] = []
        z_start = layer_h = num_layers = mult = None
        heights: list[float] = []          # plate-relative Z of every waypoint
        pb = self._plate_bottom_z
        for obj in (objects or {}).values():
            if not isinstance(obj, dict):
                continue
            params = obj.get("params") or {}
            if z_start is None:
                z_start = _to_float(params.get("z_above_plate_bottom_mm"))
            if layer_h is None:
                layer_h = _to_float(params.get("layer_height_mm"))
            if num_layers is None:
                num_layers = _to_int(params.get("num_layers"))
            if mult is None:
                mult = _to_float(params.get("extrusion_multiplier"))
            try:
                arr = object_trajectory(obj, self._needle, syringe_map)
            except Exception as e:
                logger.debug(f"edit: trajectory build failed: {e}")
                arr = None
            if arr is not None:
                shapes.extend(regions_from_trajectory(arr))
                # Best-effort recovery of the print height: region shapes are
                # XY-only, so remember each waypoint's height above the plate
                # bottom (the lowest = the first-layer print Z; travel is above).
                if (pb is not None and zref_to_plate_relative is not None
                        and arr.ndim == 2 and arr.shape[1] >= 3):
                    for zval in arr[:, 2]:
                        heights.append(zref_to_plate_relative(pb, float(zval)))
        if not shapes:
            return None
        sk = Sketch(shapes=shapes)
        # If the print didn't record its plate-relative height, recover it from
        # the baked toolpath so an edit-save doesn't silently reset the height.
        if z_start is None and heights:
            z_start = max(0.0, min(40.0, min(heights)))
        if z_start is not None:
            sk.z_start_mm = z_start
        if layer_h is not None:
            sk.layer_height_mm = layer_h
        if num_layers is not None:
            sk.num_layers = max(1, num_layers)
        if mult is not None:
            sk.extrusion_multiplier = mult
        if self._needle_od_mm > 0:
            sk.line_spacing_mm = self._needle_od_mm
        return sk

    def _begin_edit(self, sk: Sketch, name: str, best_effort: bool = False):
        # Order matters: set the name before set_sketch so the props panel
        # (rebuilt on the resulting selection_changed) shows it.
        self._print_name = name
        self._editing_name = name
        self._canvas.set_sketch(sk)
        self._canvas.set_show_thickness(self._thickness_btn.isChecked())
        self._apply_bead_width()
        self._refresh_inks_card()
        self._update_edit_ui()
        self._canvas.fit_view()
        self._schedule_preview()
        tag = " (imported from toolpath)" if best_effort else ""
        self._status_ok(f"Editing '{name}'{tag}")

    def _update_edit_ui(self):
        editing = bool(self._editing_name)
        self._save_btn.setVisible(editing)
        if editing:
            self._save_btn.setText(f"💾  Save changes to '{self._editing_name}'")

    def _new_sketch(self):
        """Start a fresh sketch and LEAVE edit mode, so a later 'Save changes'
        can't silently overwrite the print that was last opened for editing."""
        self._editing_name = None
        self._editing_stem = None
        self._prints_dir = None
        self._print_name = "Sketch"
        self._canvas.set_sketch(Sketch())
        self._apply_bead_width()
        self._refresh_inks_card()
        self._update_edit_ui()
        self._canvas.fit_view()
        self._schedule_preview()
        self._status_ok("New sketch")

    # ── External API ──────────────────────────────────────────────

    def set_z_references(self, refs: dict) -> None:
        """v7.5.x: receive the calibration Z references; the Sketch only needs
        ``plate_bottom_z`` (the datum its print height is measured up from)."""
        if not isinstance(refs, dict):
            return
        pb = refs.get("plate_bottom_z")
        new_pb = None if pb is None else float(pb)
        if new_pb != self._plate_bottom_z:
            self._plate_bottom_z = new_pb
            # Rebuild props so the "not calibrated" hint toggles correctly.
            self._rebuild_props()

    def set_hardware_config(self, config) -> None:
        """Pull needle + syringe (for volume accuracy), base the bead width /
        outline width / raster step on the needle diameter, load the active
        plate, and zoom the canvas to the selected well with a needle-safe
        boundary inset by the needle radius."""
        try:
            self._needle = getattr(config, "needle", None)
            self._syringe = None
            # ``config.pumps`` is a dict {"P1": PumpChannelConfig, ...}; iterate
            # its VALUES (iterating the dict yields the string keys).
            for pump in self._pump_values(config):
                if getattr(pump, "syringe", None) is not None:
                    self._syringe = pump.syringe
                    break
        except Exception:
            self._needle = self._syringe = None
        # Per-channel ink name + colour for the sequence panel (pump→ink).
        self._build_channel_info(config)

        # Fill, outline width and raster step all derive from the needle Ø.
        od = getattr(self._needle, "od_mm", 0.0) if self._needle else 0.0
        if od and od > 0:
            self._needle_od_mm = float(od)
            self._canvas.sketch().line_spacing_mm = float(od)
            self._canvas.set_default_line_width(float(od))
        # Feed the outer Ø so a "needle Ø" closure-overlap marker matches what
        # the compiler extrudes.
        self._canvas.set_needle_od(self._needle_od_mm)
        # The shaded print-thickness band is 1× = needle INNER Ø (the deposited
        # bead reference the operator asked for), scaled by the multiplier.
        idv = getattr(self._needle, "id_mm", 0.0) if self._needle else 0.0
        self._needle_id_mm = float(idv) if idv and idv > 0 else 0.0
        self._apply_bead_width()

        # Load the active plate and (re)populate the well selector.
        self._plate = self._load_plate(config)
        self._populate_well_combo()

        self._rebuild_props()
        self._apply_well_boundary()        # sets boundaries + fits the view
        self._sync_single_needle_checkbox()   # auto-detect single/multi
        self._refresh_sequence()
        self._schedule_preview()

    @staticmethod
    def _pump_values(config):
        """The pump configs as a list, tolerant of a dict or list ``pumps``."""
        pumps = getattr(config, "pumps", None)
        if pumps is None:
            return []
        if hasattr(pumps, "values"):
            return list(pumps.values())
        try:
            return list(pumps)
        except TypeError:
            return []

    def _build_channel_info(self, config) -> None:
        """Map each channel (0→P1,1→P2,2→P3) to its ink (name, colour) for the
        sequence panel; falls back to P1/P2/P3 + pump colours when unassigned."""
        info: dict[int, tuple] = {}
        pumps = getattr(config, "pumps", None)
        getter = pumps.get if hasattr(pumps, "get") else None
        for idx in range(3):
            name = color = None
            pump = getter(f"P{idx + 1}") if getter else None
            if pump is not None:
                inks = getattr(pump, "inks", None) or []
                if inks:
                    name = getattr(inks[0], "name", None)
                    color = getattr(inks[0], "color", None)
            info[idx] = (name, color)
        self._channel_info = info

    # ── Well boundary ─────────────────────────────────────────────

    @staticmethod
    def _load_plate(config):
        try:
            from SupportClasses.WellPlate import WellPlate
            # v7.5.x: geometry_plate_key so a rosette/custom design layered
            # under a plate TYPE is honored (falls back to active_plate_key).
            key = getattr(config, "geometry_plate_key", None) \
                or config.active_plate_key
            return WellPlate.load(key)
        except Exception as e:
            logger.debug(f"plate load failed: {e}")
            return None

    @staticmethod
    def _default_well_name(names: list[str]) -> str | None:
        if not names:
            return None
        return "A1" if "A1" in names else names[0]

    def _populate_well_combo(self) -> None:
        names = list(self._plate.well_names) if self._plate else []
        self._well_combo.blockSignals(True)
        self._well_combo.clear()
        self._well_combo.addItems(names)
        # Keep the prior selection if it still exists, else pick a default.
        sel = (self._selected_well if self._selected_well in names
               else self._default_well_name(names))
        self._selected_well = sel
        if sel is not None:
            idx = self._well_combo.findText(sel)
            if idx >= 0:
                self._well_combo.setCurrentIndex(idx)
        self._well_combo.setEnabled(bool(names))
        self._well_combo.blockSignals(False)

    def _on_well_changed(self, _idx: int) -> None:
        name = self._well_combo.currentText().strip()
        self._selected_well = name or None
        self._apply_well_boundary()
        self._schedule_preview()           # re-evaluate the safe-boundary check

    def _well_diameter_for_selected(self) -> float:
        """Diameter (mm) of the selected well; falls back to the plate's
        uniform diameter, then the largest well on custom/varied plates."""
        if self._plate is None:
            return 0.0
        try:
            if self._selected_well:
                d = float(self._plate.get_well_info(self._selected_well).diameter or 0.0)
                if d > 0:
                    return d
        except Exception:
            pass
        d = float(getattr(self._plate, "well_diameter", 0.0) or 0.0)
        if d > 0:
            return d
        try:
            wells = self._plate.get_all_wells()
            if wells:
                return float(max(w.diameter for w in wells))
        except Exception:
            pass
        return 0.0

    def _apply_well_boundary(self) -> None:
        """Push the well-wall + needle-safe boundary circles to the canvas and
        zoom to frame the selected well."""
        well_d = self._well_diameter_for_selected()
        self._canvas.set_reference_well(well_d)
        # Needle-safe boundary = well wall inset by the needle radius on each
        # side → inner Ø = well Ø − needle Ø. Needs a known needle diameter.
        nod = self._needle_od_mm
        if well_d > 0 and nod > 0:
            inner = max(0.0, well_d - nod)
            self._canvas.set_safe_boundary(inner)
            self._safe_radius_mm = inner / 2.0
        else:
            self._canvas.set_safe_boundary(0.0)
            self._safe_radius_mm = 0.0
        self._update_boundary_info(well_d)
        self._canvas.fit_view()

    def _update_boundary_info(self, well_d: float) -> None:
        if well_d <= 0:
            self._boundary_info_lbl.setText("No plate configured — draw freely.")
            return
        nod = self._needle_od_mm
        if nod > 0:
            inner = max(0.0, well_d - nod)
            self._boundary_info_lbl.setText(
                f"Well wall Ø {well_d:.2f} mm · needle-safe Ø {inner:.2f} mm "
                f"(inset {nod / 2.0:.2f} mm)")
        else:
            self._boundary_info_lbl.setText(
                f"Well wall Ø {well_d:.2f} mm · configure a needle to show the "
                f"needle-safe boundary")

    @staticmethod
    def _max_radius_mm(trajectory) -> float:
        """Largest distance of any waypoint from the well center (origin)."""
        try:
            import numpy as _np
            xy = _np.asarray(trajectory, dtype=_np.float64)[:, :2]
            if not len(xy):
                return 0.0
            return float(_np.sqrt((xy ** 2).sum(axis=1)).max())
        except Exception:
            return 0.0

    def _exceeds_safe_boundary(self, trajectory) -> bool:
        if trajectory is None or self._safe_radius_mm <= 0:
            return False
        return self._max_radius_mm(trajectory) > self._safe_radius_mm + 1e-6

    def _update_bounds_warning(self, trajectory) -> None:
        self._last_oob = False
        if trajectory is None or self._safe_radius_mm <= 0:
            self._bounds_warn_lbl.setVisible(False)
            return
        r_max = self._max_radius_mm(trajectory)
        if r_max > self._safe_radius_mm + 1e-6:
            self._last_oob = True
            self._bounds_warn_lbl.setText(
                f"⚠ Sketch extends {r_max - self._safe_radius_mm:.2f} mm past "
                f"the needle-safe boundary — the needle may contact the well "
                f"wall.")
            self._bounds_warn_lbl.setVisible(True)
        else:
            self._bounds_warn_lbl.setVisible(False)

    def get_page_title(self) -> str:
        return "Sketch"
