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
    QSpinBox, QScrollArea, QGroupBox, QSizePolicy, QMessageBox,
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

        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(120)
        self._preview_timer.timeout.connect(self._recompute_preview)

        self._build_ui()
        self._canvas.set_sketch(Sketch())
        self._rebuild_props()
        self._schedule_preview()

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

        if self._needle_od_mm > 0:
            info = QLabel(
                f"Needle Ø {self._needle_od_mm:.2f} mm — sets bead width "
                f"& raster step")
            info.setWordWrap(True)
            info.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9)}pt;")
            lay.addWidget(info)

        zs = self._dspin(sk.z_start_mm, -50, 200, 0.1)
        lh = self._dspin(sk.layer_height_mm, 0.01, 10, 0.05)
        nl = QSpinBox()
        nl.setRange(1, 999)
        nl.setValue(sk.num_layers)
        sp = self._dspin(sk.print_speed_mm_s, 0.1, 200, 0.5, " mm/s")
        ls = self._dspin(sk.line_spacing_mm, 0.05, 20, 0.05)

        zs.valueChanged.connect(lambda v: self._set_sketch("z_start_mm", v))
        lh.valueChanged.connect(lambda v: self._set_sketch("layer_height_mm", v))
        nl.valueChanged.connect(lambda v: self._set_sketch("num_layers", v))
        sp.valueChanged.connect(lambda v: self._set_sketch("print_speed_mm_s", v))
        ls.valueChanged.connect(lambda v: self._set_sketch("line_spacing_mm", v))

        self._field_row(lay, "Z start", zs)
        self._field_row(lay, "Layer h", lh)
        self._field_row(lay, "# Layers", nl)
        self._field_row(lay, "Speed", sp)
        self._field_row(lay, "Raster step", ls)
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

    def _canvas_delete(self):
        self._canvas.delete_selected()

    def _toggle_snap(self, on):
        self._canvas.set_snap(1.0 if on else 0.0)

    def _toggle_osnap(self, on):
        self._canvas.set_object_snap(bool(on))

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
            return

        # Render the compiled toolpath as the main raster view.
        self._canvas.set_toolpath(result.trajectory, result.pump_states)
        self._stats_lbl.setText(
            f"{result.num_waypoints} wpts · {result.num_layers} layer(s) · "
            f"path {result.total_length_mm:.1f} mm · "
            f"~{result.total_time_s:.1f} s · {result.total_volume_uL:.2f} µL")
        self._send_btn.setEnabled(True)

    def _send_to_print_setup(self):
        sk = self._canvas.sketch()
        result = compile_to_trajectory(sk, self._needle, self._syringe)
        if result.is_empty:
            QMessageBox.warning(self, "Empty sketch",
                                "Draw at least one shape first.")
            return
        try:
            name = save_trajectory_as_print_object(
                result.trajectory,
                base_name="Sketch",
                description=(f"Sketch: {len(sk.shapes)} shape(s), "
                             f"{result.num_layers} layer(s), "
                             f"{result.total_length_mm:.1f} mm path"),
                color="#cba6f7",
                author="Print Builder",
                source="SketchTrajectory",
                object_name="Sketch_1",
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

    def set_hardware_config(self, config) -> None:
        """Pull needle + syringe (for volume accuracy), base the bead width /
        outline width / raster step on the needle diameter, and show the
        active plate's standard-well size as a sizing reference."""
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

        # Show the standard well outline at the origin for scale.
        self._canvas.set_reference_well(self._well_diameter_from_config(config))

        self._rebuild_props()
        self._canvas.fit_view()
        self._schedule_preview()

    @staticmethod
    def _well_diameter_from_config(config) -> float:
        try:
            from SupportClasses.WellPlate import WellPlate
            plate = WellPlate.load(config.active_plate_key)
            d = float(getattr(plate, "well_diameter", 0.0) or 0.0)
            if d > 0:
                return d
            wells = getattr(plate, "wells", None) or []
            if wells:
                return float(max(w.diameter for w in wells))
        except Exception as e:
            logger.debug(f"reference well lookup failed: {e}")
        return 0.0

    def get_page_title(self) -> str:
        return "Sketch"
