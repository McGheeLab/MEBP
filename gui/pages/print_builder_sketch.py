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
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QSplitter, QFrame, QToolButton,
    QButtonGroup, QPushButton, QLabel, QCheckBox, QComboBox, QDoubleSpinBox,
    QSpinBox, QScrollArea, QGroupBox, QSizePolicy, QMessageBox, QLineEdit,
)

from gui.styles import COLORS, build_section_title_style
from gui.scaling import s, scaled_font_size as _sf, scale_factor
from gui.widgets.icons import icon
from gui.widgets.sketch_canvas import SketchCanvas, Tool, PUMP_HEX
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory,
)
from SupportClasses.PrintFileManager import save_trajectory_as_print_object

logger = logging.getLogger(__name__)

# (label, Tool, icon-name or glyph)
_TOOLS = [
    ("Select",      Tool.SELECT,  "cursor"),
    ("Line",        Tool.LINE,    "line"),
    ("Rectangle",   Tool.RECT,    "▭"),
    ("Circle",      Tool.CIRCLE,  "◯"),
    ("Ellipse",     Tool.ELLIPSE, "⬭"),
    ("Polygon",     Tool.POLYGON, "⬠"),
    ("Fill region", Tool.FILL,    "droplet"),
]


class SketchPage(QWidget):
    """Draw-to-print Sketch tool."""

    print_file_created = Signal(str)

    TOOL_BTN = 40
    TOOL_ICON = 22

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._needle = None
        self._syringe = None
        self._needle_od_mm = 0.0
        self._building = False
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

        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(120)
        self._preview_timer.timeout.connect(self._recompute_preview)

        self._build_ui()
        self._canvas.set_sketch(Sketch())
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
        split.addWidget(self._canvas)

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
            "line", "Show line thickness (draw the toolpath at the needle "
            "bead width)", self._toggle_thickness, checkable=True)
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
        v = QVBoxLayout(panel)
        v.setContentsMargins(s(8), s(6), s(8), s(8))
        v.setSpacing(s(12))

        # Well boundary selector (persistent — not rebuilt with the props).
        v.addWidget(self._build_well_card())

        # Properties (scrollable)
        self._props_scroll = QScrollArea()
        self._props_scroll.setWidgetResizable(True)
        self._props_scroll.setFrameShape(QFrame.NoFrame)
        self._props_host = QWidget()
        self._props_layout = QVBoxLayout(self._props_host)
        self._props_layout.setContentsMargins(0, 0, 0, 0)
        self._props_layout.setSpacing(s(12))
        self._props_scroll.setWidget(self._props_host)
        v.addWidget(self._props_scroll, 1)

        # The toolpath raster is rendered in the main canvas (not here); the
        # shapes are the editable overlay on top of it.
        self._send_btn = QPushButton("Send to Print Setup")
        self._send_btn.setObjectName("primaryBtn")
        self._send_btn.clicked.connect(self._send_to_print_setup)
        v.addWidget(self._send_btn)
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

    def _rebuild_props(self):
        self._building = True
        self._clear_props()
        sh = self._canvas.selected_shape()
        if sh is not None:
            self._props_layout.addWidget(self._build_shape_card(sh))
        else:
            hint = QLabel("Pick a tool and draw on the canvas.\n"
                          "Select a shape to edit its exact size.")
            hint.setWordWrap(True)
            hint.setStyleSheet(f"color: {COLORS['subtext0']}; "
                               f"font-size: {_sf(9)}pt;")
            self._props_layout.addWidget(hint)
        self._props_layout.addWidget(self._build_print_card())
        self._props_layout.addStretch(1)
        self._building = False

    def _build_shape_card(self, sh: SketchShape) -> QGroupBox:
        grp = self._group(f"Shape — {sh.kind}")
        lay = grp.layout()

        # Region = baked paint-bucket fill: only pump + delete are editable.
        if sh.kind == "region":
            info = QLabel(f"Filled region · {len(sh.points)} points")
            info.setStyleSheet(f"color: {COLORS['subtext0']};")
            lay.addWidget(info)
            pump = QComboBox()
            pump.addItems(["P1", "P2", "P3"])
            pump.setCurrentIndex(max(0, min(2, sh.pump_index)))
            pump.currentIndexChanged.connect(lambda i: self._set_pump(sh, i))
            self._field_row(lay, "Pump", pump)
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

        # Pump assignment
        pump = QComboBox()
        pump.addItems(["P1", "P2", "P3"])
        pump.setCurrentIndex(max(0, min(2, sh.pump_index)))
        pump.currentIndexChanged.connect(lambda i: self._set_pump(sh, i))
        self._field_row(lay, "Pump", pump)

        delete = QPushButton("Delete shape")
        delete.setObjectName("dangerBtn")
        delete.clicked.connect(self._canvas_delete)
        lay.addWidget(delete)
        return grp

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

        if self._needle_od_mm > 0:
            info = QLabel(
                f"Needle Ø {self._needle_od_mm:.2f} mm — sets bead width "
                f"& raster step")
            info.setWordWrap(True)
            info.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            lay.addWidget(info)

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

        zs.valueChanged.connect(lambda v: self._set_sketch("z_start_mm", v))
        lh.valueChanged.connect(lambda v: self._set_sketch("layer_height_mm", v))
        nl.valueChanged.connect(lambda v: self._set_sketch("num_layers", v))
        sp.valueChanged.connect(lambda v: self._set_sketch("print_speed_mm_s", v))
        ls.valueChanged.connect(lambda v: self._set_sketch("line_spacing_mm", v))
        lift.valueChanged.connect(
            lambda v: self._set_sketch("travel_clearance_mm", v))

        self._field_row(lay, "Print height", zs)
        self._field_row(lay, "Layer h", lh)
        self._field_row(lay, "# Layers", nl)
        self._field_row(lay, "Lift between shapes", lift)
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
        self._canvas.update()
        self._schedule_preview()

    def _set_pt(self, shape, idx, axis, value):
        if self._building or len(shape.points) <= idx:
            return
        x, y = shape.points[idx]
        shape.points[idx] = (value, y) if axis == 0 else (x, value)
        self._canvas.update()
        self._schedule_preview()

    def _set_pump(self, shape, index):
        if self._building:
            return
        shape.pump_index = max(0, min(2, index))
        shape.color = PUMP_HEX[shape.pump_index]
        self._canvas.set_active_pump(shape.pump_index)  # new shapes inherit
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

    def _canvas_delete(self):
        self._canvas.delete_selected()

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
        try:
            result = compile_to_trajectory(sk, self._needle, self._syringe)
        except Exception as e:
            logger.warning(f"sketch compile failed: {e}")
            self._canvas.set_toolpath(None, None)
            self._stats_lbl.setText("Compile error")
            return
        self._last_result = result
        if result.is_empty:
            self._canvas.set_toolpath(None, None)
            self._stats_lbl.setText("Empty sketch — draw a shape")
            self._send_btn.setEnabled(False)
            self._update_bounds_warning(None)
            return

        # Render the compiled toolpath as the main raster view.
        self._canvas.set_toolpath(result.trajectory, result.pump_states)
        self._stats_lbl.setText(
            f"{result.num_waypoints} wpts · {result.num_layers} layer(s) · "
            f"path {result.total_length_mm:.1f} mm · "
            f"~{result.total_time_s:.1f} s · {result.total_volume_uL:.2f} µL")
        self._send_btn.setEnabled(True)
        self._update_bounds_warning(result.trajectory)

    def _send_to_print_setup(self):
        sk = self._canvas.sketch()
        result = compile_to_trajectory(sk, self._needle, self._syringe)
        if result.is_empty:
            QMessageBox.warning(self, "Empty sketch",
                                "Draw at least one shape first.")
            return

        # Non-blocking safe-boundary guard: warn (and confirm) if the toolpath
        # crosses the needle-safe ring, but let the operator proceed.
        if self._exceeds_safe_boundary(result.trajectory):
            resp = QMessageBox.warning(
                self, "Outside needle-safe boundary",
                f"This sketch extends past the needle-safe boundary for well "
                f"{self._selected_well or '?'} — the needle could contact the "
                f"well wall when printing.\n\nSend it anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if resp != QMessageBox.Yes:
                return

        base_name = (self._print_name or "").strip() or "Sketch"

        # v7.5.x: the sketch's Z column is a *height above the plate bottom*.
        # Bake it into the internal zero-ref frame when the plate bottom is
        # known (so the trajectory is physically correct), and always record
        # the relative height as metadata so Print Setup prints at the chosen
        # height above the plate bottom and clamps against punch-through.
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
        }
        try:
            name = save_trajectory_as_print_object(
                traj,
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
            )
        except Exception as e:
            logger.error(f"send to print setup failed: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Failed:\n{e}")
            return
        self._status_lbl.setText(f"✓ Sent as '{name}'")
        self._status_lbl.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {_sf(9)}pt;")
        self.print_file_created.emit(name)

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
            for pump in getattr(config, "pumps", []) or []:
                if getattr(pump, "syringe", None) is not None:
                    self._syringe = pump.syringe
                    break
        except Exception:
            self._needle = self._syringe = None

        # Fill, outline width and raster step all derive from the needle Ø.
        od = getattr(self._needle, "od_mm", 0.0) if self._needle else 0.0
        if od and od > 0:
            self._needle_od_mm = float(od)
            self._canvas.sketch().line_spacing_mm = float(od)
            self._canvas.set_default_line_width(float(od))

        # Load the active plate and (re)populate the well selector.
        self._plate = self._load_plate(config)
        self._populate_well_combo()

        self._rebuild_props()
        self._apply_well_boundary()        # sets boundaries + fits the view
        self._schedule_preview()

    # ── Well boundary ─────────────────────────────────────────────

    @staticmethod
    def _load_plate(config):
        try:
            from SupportClasses.WellPlate import WellPlate
            return WellPlate.load(config.active_plate_key)
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
