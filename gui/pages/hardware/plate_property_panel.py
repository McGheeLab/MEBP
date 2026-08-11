"""Properties panel for the v7.12 plate builder.

Build once, bind, toggle visibility — the v7.4.x panel tore down and recreated
every widget on every selection change, every solve and every tool change, which
is why typing into a spin box could have it destroyed under the cursor.

Four mechanisms together replace that:

1. every card is constructed once and only shown/hidden;
2. ``refresh()`` writes values, never widgets;
3. a **focus guard** skips any refresh while the focused widget lives inside
   this panel;
4. selection → ``bind`` is immediate, document → ``refresh`` is debounced.

The pattern cards are where the operator's requests 2 and 3 land: every option
for a ring or a grid is here, live-editing, with no destructive Apply button —
because members are derived, so changing a parameter cannot lose anything.
"""
from __future__ import annotations

import logging
from typing import Callable, Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QScrollArea, QSizePolicy, QSpinBox, QToolButton, QVBoxLayout, QWidget,
)

from gui.scaling import s, scaled_font_size as _sf
from gui.styles import COLORS
from SupportClasses.PlateDocument import (
    DATUMS, GridPattern, NamingScheme, OriginRef, PatternFeature,
    PlateDocument, RingPattern, Well,
)
from SupportClasses.SolveTypes import DOFStatus

logger = logging.getLogger(__name__)

_ORIGIN_CHOICES = [
    ("Bottom-left edge", OriginRef.BOTTOM_LEFT),
    ("Bottom-right edge", OriginRef.BOTTOM_RIGHT),
    ("Top-left edge", OriginRef.TOP_LEFT),
    ("Top-right edge", OriginRef.TOP_RIGHT),
    ("Plate centre", OriginRef.CENTER),
    ("Well A1", OriginRef.A1),
]

_CONSTRAINT_BUTTONS = [
    ("Coincident", "coincident", "Put two wells at the same point"),
    ("Concentric", "concentric", "Share a centre"),
    ("Horizontal", "horizontal", "Same Y"),
    ("Vertical", "vertical", "Same X"),
    ("Distance", "distance", "Fix the distance between two wells"),
    ("Equal Ø", "equal_radius", "Give both the same diameter"),
    ("🔒 Lock", "fix", "Pin the selection in place"),
]


def _dspin(lo, hi, step=0.1, suffix=" mm", dec=3) -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setSingleStep(step)
    sp.setDecimals(dec)
    sp.setSuffix(suffix)
    sp.setKeyboardTracking(False)
    return sp


def _ispin(lo, hi) -> QSpinBox:
    sp = QSpinBox()
    sp.setRange(lo, hi)
    sp.setKeyboardTracking(False)
    return sp


def _group(title: str) -> tuple[QGroupBox, QFormLayout]:
    box = QGroupBox(title)
    box.setStyleSheet(
        f"QGroupBox {{ color: {COLORS['subtext0']}; "
        f"font-size: {_sf(9)}pt; font-weight: 700; "
        f"border: 1px solid {COLORS['surface1']}; border-radius: {s(6)}px; "
        f"margin-top: {s(9)}px; padding-top: {s(8)}px; }}"
        f"QGroupBox::title {{ subcontrol-origin: margin; left: {s(8)}px; }}")
    form = QFormLayout(box)
    form.setContentsMargins(s(8), s(6), s(8), s(8))
    form.setSpacing(s(4))
    form.setLabelAlignment(Qt.AlignRight)
    return box, form


class PlatePropertyPanel(QScrollArea):
    """Owns one instance of every card; selection rebinds, never rebuilds."""

    document_edited = Signal()
    constraint_focus_requested = Signal(int)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._doc: Optional[PlateDocument] = None
        self._canvas = None
        self._building = False
        self._sig: tuple = ()

        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(120)
        self._debounce.timeout.connect(self._do_refresh)

        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        host = QWidget()
        self._v = QVBoxLayout(host)
        self._v.setContentsMargins(s(6), s(6), s(6), s(6))
        self._v.setSpacing(s(6))
        self.setWidget(host)

        self._build_plate_card()
        self._build_ring_card()
        self._build_grid_card()
        self._build_well_card()
        self._build_member_card()
        self._build_constraints_card()
        self._build_summary_card()
        self._v.addStretch(1)

    # ── Wiring ────────────────────────────────────────────────────

    def attach(self, canvas) -> None:
        self._canvas = canvas
        canvas.selection_changed.connect(lambda _s: self.bind())
        canvas.document_changed.connect(self._debounce.start)
        canvas.solve_reported.connect(lambda _r: self._debounce.start())

    def set_document(self, doc: Optional[PlateDocument]) -> None:
        self._doc = doc
        self.bind()

    # ── Cards ─────────────────────────────────────────────────────

    def _build_plate_card(self):
        self._plate_box, f = _group("Plate")
        self._origin_combo = QComboBox()
        for label, val in _ORIGIN_CHOICES:
            self._origin_combo.addItem(label, val)
        self._origin_combo.setToolTip(
            "Which corner or centre the coordinates you see and type are "
            "measured from.\nThis changes the readout only — the plate "
            "geometry never moves.")
        self._origin_combo.currentIndexChanged.connect(self._set_origin)
        f.addRow("Origin", self._origin_combo)

        self._w_spin = _dspin(1.0, 1000.0, 0.5)
        self._w_spin.valueChanged.connect(
            lambda v: self._set_boundary("width_mm", v))
        f.addRow("Width", self._w_spin)
        self._h_spin = _dspin(1.0, 1000.0, 0.5)
        self._h_spin.valueChanged.connect(
            lambda v: self._set_boundary("height_mm", v))
        f.addRow("Height", self._h_spin)
        self._bore_spin = _dspin(0.2, 200.0, 0.1)
        self._bore_spin.valueChanged.connect(
            lambda v: self._set_boundary("radius_mm", v / 2.0))
        f.addRow("Bore Ø", self._bore_spin)

        self._a1x_spin = _dspin(-500.0, 500.0, 0.1)
        self._a1x_spin.setToolTip(
            "Where well A1 sits, measured from the plate's top-left corner.")
        self._a1x_spin.valueChanged.connect(
            lambda v: self._set_boundary("a1_x_mm", v))
        f.addRow("A1 from left", self._a1x_spin)
        self._a1y_spin = _dspin(-500.0, 500.0, 0.1)
        self._a1y_spin.valueChanged.connect(
            lambda v: self._set_boundary("a1_y_mm", v))
        f.addRow("A1 from top", self._a1y_spin)
        self._v.addWidget(self._plate_box)

    def _build_ring_card(self):
        self._ring_box, f = _group("Ring")
        self._ring_name = QLineEdit()
        self._ring_name.editingFinished.connect(
            lambda: self._set_feat("name", self._ring_name.text().strip()))
        f.addRow("Name", self._ring_name)
        self._ring_count = _ispin(1, 200)
        self._ring_count.setToolTip(
            "Number of wells on the ring.\nChanging it keeps every existing "
            "well's name, rosette and well type.")
        self._ring_count.valueChanged.connect(
            lambda v: self._set_feat("count", v))
        f.addRow("Wells", self._ring_count)
        self._ring_dia = _dspin(0.1, 500.0, 0.1)
        self._ring_dia.valueChanged.connect(
            lambda v: self._set_feat("ring_diameter_mm", v))
        f.addRow("Ring Ø", self._ring_dia)
        self._ring_angle = _dspin(-360.0, 360.0, 1.0, "°", 2)
        self._ring_angle.setToolTip("0° puts the first well at 12 o'clock.")
        self._ring_angle.valueChanged.connect(
            lambda v: self._set_feat("start_angle_deg", v))
        f.addRow("Orientation", self._ring_angle)
        self._ring_sweep = _dspin(1.0, 360.0, 5.0, "°", 1)
        self._ring_sweep.valueChanged.connect(
            lambda v: self._set_feat("sweep_deg", v))
        f.addRow("Sweep", self._ring_sweep)
        self._ring_dir = QComboBox()
        self._ring_dir.addItem("Clockwise on screen", 1)
        self._ring_dir.addItem("Counter-clockwise", -1)
        self._ring_dir.currentIndexChanged.connect(
            lambda _i: self._set_feat("direction",
                                      self._ring_dir.currentData()))
        f.addRow("Direction", self._ring_dir)
        self._ring_centre = QCheckBox("Well at the centre")
        self._ring_centre.toggled.connect(
            lambda on: self._set_feat("center_well", on))
        f.addRow("", self._ring_centre)
        self._ring_well_dia = _dspin(0.05, 200.0, 0.1)
        self._ring_well_dia.valueChanged.connect(
            lambda v: self._set_style("diameter_mm", v))
        f.addRow("Well Ø", self._ring_well_dia)
        self._ring_depth = _dspin(0.0, 200.0, 0.5)
        self._ring_depth.valueChanged.connect(
            lambda v: self._set_style("well_depth_mm", v))
        f.addRow("Well depth", self._ring_depth)
        self._ring_rim = _dspin(0.0, 100.0, 0.5)
        self._ring_rim.setToolTip(
            "How far the vessel top sits above the plate surface.")
        self._ring_rim.valueChanged.connect(
            lambda v: self._set_style("rim_height_mm", v))
        f.addRow("Rim height", self._ring_rim)
        self._build_naming_rows("ring", f)
        self._v.addWidget(self._ring_box)

    def _build_naming_rows(self, which: str, f) -> None:
        """Well-naming controls, shared by the ring and grid cards.

        Before this existed every pattern was born with the same default spec,
        so a second grid produced ``A1…`` all over again — and there was no
        control anywhere to change it. ``validate()`` reported the collision
        and the operator could do nothing about it.
        """
        scheme = QComboBox()
        scheme.addItem("Letters + numbers (A1, A2…)", NamingScheme.ANSI)
        scheme.addItem("Letters (a, b, c…)", NamingScheme.LETTERS)
        scheme.addItem("Numbers (1, 2, 3…)", NamingScheme.NUMBERS)
        scheme.addItem("Member key", NamingScheme.MANUAL)
        scheme.currentIndexChanged.connect(
            lambda _i, w=which: self._set_naming(
                w, "scheme", getattr(self, f"_{w}_scheme").currentData()))
        f.addRow("Naming", scheme)
        setattr(self, f"_{which}_scheme", scheme)

        prefix = QLineEdit()
        prefix.setPlaceholderText("none")
        prefix.setToolTip(
            "Goes in front of every well name in this pattern — the quickest "
            "way to keep two patterns apart (e.g. 'L' → LA1, LA2…).")
        prefix.editingFinished.connect(
            lambda w=which: self._set_naming(
                w, "prefix", getattr(self, f"_{w}_prefix").text().strip()))
        f.addRow("Prefix", prefix)
        setattr(self, f"_{which}_prefix", prefix)

        row = _ispin(0, 63)
        row.setToolTip("0 = row A. A second grid usually starts below the "
                       "first — that is what the automatic choice does.")
        row.valueChanged.connect(
            lambda v, w=which: self._set_naming(w, "start_row", v))
        f.addRow("First row", row)
        setattr(self, f"_{which}_start_row", row)

        col = _ispin(1, 999)
        col.setToolTip("The number (or letter position) the first well takes.")
        col.valueChanged.connect(
            lambda v, w=which: self._set_naming(w, "start_col", v))
        f.addRow("First number", col)
        setattr(self, f"_{which}_start_col", col)

        auto = QPushButton("Auto-number (avoid clashes)")
        auto.setToolTip("Shift this pattern's names until none of them collide "
                        "with another pattern's.")
        auto.clicked.connect(self._autoname)
        f.addRow("", auto)

        preview = QLabel("")
        preview.setWordWrap(True)
        preview.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        f.addRow("", preview)
        setattr(self, f"_{which}_name_preview", preview)

    def _build_grid_card(self):
        self._grid_box, f = _group("Grid")
        self._grid_name = QLineEdit()
        self._grid_name.editingFinished.connect(
            lambda: self._set_feat("name", self._grid_name.text().strip()))
        f.addRow("Name", self._grid_name)
        self._grid_rows = _ispin(1, 64)
        self._grid_rows.valueChanged.connect(
            lambda v: self._set_feat("rows", v))
        f.addRow("Rows", self._grid_rows)
        self._grid_cols = _ispin(1, 64)
        self._grid_cols.valueChanged.connect(
            lambda v: self._set_feat("cols", v))
        f.addRow("Columns", self._grid_cols)
        self._grid_px = _dspin(0.1, 200.0, 0.1)
        self._grid_px.valueChanged.connect(
            lambda v: self._set_feat("pitch_x_mm", v))
        f.addRow("Spacing X", self._grid_px)
        self._grid_py = _dspin(0.1, 200.0, 0.1)
        self._grid_py.valueChanged.connect(
            lambda v: self._set_feat("pitch_y_mm", v))
        f.addRow("Spacing Y", self._grid_py)
        self._grid_rot = _dspin(-360.0, 360.0, 1.0, "°", 2)
        self._grid_rot.valueChanged.connect(
            lambda v: self._set_feat("rotation_deg", v))
        f.addRow("Orientation", self._grid_rot)

        corner_row = QWidget()
        cg = QGridLayout(corner_row)
        cg.setContentsMargins(0, 0, 0, 0)
        cg.setSpacing(s(2))
        self._corner_btns: dict[tuple[int, int], QToolButton] = {}
        for (r, c), glyph, dirs in (
                ((0, 0), "↖", (1, 1)), ((0, 1), "↗", (-1, 1)),
                ((1, 0), "↙", (1, -1)), ((1, 1), "↘", (-1, -1))):
            b = QToolButton()
            b.setText(glyph)
            b.setCheckable(True)
            b.setToolTip("Which corner numbering starts from")
            b.clicked.connect(lambda _c=False, d=dirs: self._set_corner(d))
            cg.addWidget(b, r, c)
            self._corner_btns[dirs] = b
        f.addRow("A1 corner", corner_row)

        self._grid_well_dia = _dspin(0.05, 200.0, 0.1)
        self._grid_well_dia.valueChanged.connect(
            lambda v: self._set_style("diameter_mm", v))
        f.addRow("Well Ø", self._grid_well_dia)
        self._grid_depth = _dspin(0.0, 200.0, 0.5)
        self._grid_depth.valueChanged.connect(
            lambda v: self._set_style("well_depth_mm", v))
        f.addRow("Well depth", self._grid_depth)
        self._build_naming_rows("grid", f)
        self._v.addWidget(self._grid_box)

    def _build_well_card(self):
        self._well_box, f = _group("Well")
        self._well_name = QLineEdit()
        self._well_name.editingFinished.connect(self._set_well_name)
        f.addRow("Name", self._well_name)
        self._well_x = _dspin(-1000.0, 1000.0, 0.1)
        self._well_x.valueChanged.connect(lambda v: self._set_well_pos(0, v))
        f.addRow("X", self._well_x)
        self._well_y = _dspin(-1000.0, 1000.0, 0.1)
        self._well_y.valueChanged.connect(lambda v: self._set_well_pos(1, v))
        f.addRow("Y", self._well_y)
        self._well_dia = _dspin(0.05, 200.0, 0.1)
        self._well_dia.valueChanged.connect(
            lambda v: self._set_style("diameter_mm", v))
        f.addRow("Ø", self._well_dia)
        self._well_depth = _dspin(0.0, 200.0, 0.5)
        self._well_depth.valueChanged.connect(
            lambda v: self._set_style("well_depth_mm", v))
        f.addRow("Depth", self._well_depth)
        self._v.addWidget(self._well_box)

    def _build_member_card(self):
        self._member_box, f = _group("Pattern member")
        self._member_lbl = QLabel("")
        self._member_lbl.setWordWrap(True)
        self._member_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        f.addRow(self._member_lbl)
        self._member_name = QLineEdit()
        self._member_name.setPlaceholderText("(automatic)")
        self._member_name.editingFinished.connect(self._set_member_name)
        f.addRow("Name", self._member_name)
        self._member_rim = _dspin(0.0, 100.0, 0.5)
        self._member_rim.valueChanged.connect(
            lambda v: self._set_member("rim_height_mm", v))
        f.addRow("Rim height", self._member_rim)
        self._v.addWidget(self._member_box)

    def _build_constraints_card(self):
        self._cons_box = QGroupBox("Constraints")
        self._cons_box.setStyleSheet(
            f"QGroupBox {{ color: {COLORS['subtext0']}; "
            f"font-size: {_sf(9)}pt; font-weight: 700; "
            f"border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {s(6)}px; margin-top: {s(9)}px; "
            f"padding-top: {s(8)}px; }}"
            f"QGroupBox::title {{ subcontrol-origin: margin; "
            f"left: {s(8)}px; }}")
        outer = QVBoxLayout(self._cons_box)
        outer.setContentsMargins(s(8), s(6), s(8), s(8))
        outer.setSpacing(s(5))

        grid = QGridLayout()
        grid.setSpacing(s(3))
        self._cons_btns: dict[str, QPushButton] = {}
        for n, (label, kind, tip) in enumerate(_CONSTRAINT_BUTTONS):
            b = QPushButton(label)
            b.setToolTip(tip)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _c=False, k=kind: self._add_constraint(k))
            grid.addWidget(b, n // 3, n % 3)
            self._cons_btns[kind] = b
        outer.addLayout(grid)

        self._dof_lbl = QLabel("")
        self._dof_lbl.setWordWrap(True)
        self._dof_lbl.setStyleSheet(f"font-size: {_sf(8)}pt;")
        outer.addWidget(self._dof_lbl)

        self._cons_host = QWidget()
        self._cons_list = QVBoxLayout(self._cons_host)
        self._cons_list.setContentsMargins(0, 0, 0, 0)
        self._cons_list.setSpacing(s(2))
        outer.addWidget(self._cons_host)
        self._v.addWidget(self._cons_box)

    def _build_summary_card(self):
        self._sum_box, f = _group("Summary")
        self._sum_lbl = QLabel("")
        self._sum_lbl.setWordWrap(True)
        self._sum_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")
        f.addRow(self._sum_lbl)
        self._v.addWidget(self._sum_box)

    # ── Bind / refresh ────────────────────────────────────────────

    def bind(self) -> None:
        """Selection changed — show the right cards and load their values."""
        doc, canvas = self._doc, self._canvas
        feat = canvas.selected_features()[0] if (
            canvas and len(canvas.selected_features()) == 1) else None
        primary = canvas.primary() if canvas else None
        member = bool(primary and primary[1])
        well = None
        if doc and primary and not primary[1]:
            ent = doc.entities.get(primary[0])
            well = ent if isinstance(ent, Well) else None

        self._plate_box.setVisible(doc is not None and feat is None
                                   and well is None)
        self._ring_box.setVisible(isinstance(feat, RingPattern))
        self._grid_box.setVisible(isinstance(feat, GridPattern))
        self._well_box.setVisible(well is not None)
        self._member_box.setVisible(member)
        self._cons_box.setVisible(doc is not None)
        self._sum_box.setVisible(doc is not None)
        self._do_refresh(force=True)

    def _focus_is_mine(self) -> bool:
        w = QApplication.focusWidget()
        return w is not None and self.isAncestorOf(w)

    def _do_refresh(self, force: bool = False) -> None:
        if self._doc is None:
            return
        # Never yank a widget out from under someone who is typing in it.
        if not force and self._focus_is_mine():
            return
        self._building = True
        try:
            self._refresh_values()
        finally:
            self._building = False

    def _refresh_values(self) -> None:
        doc, canvas = self._doc, self._canvas
        b = doc.boundary
        idx = self._origin_combo.findData(b.origin_ref)
        if idx < 0:
            idx = self._origin_combo.findData(
                getattr(b.origin_ref, "value", b.origin_ref))
        if idx >= 0:
            self._origin_combo.setCurrentIndex(idx)
        circle = b.is_circle()
        for w in (self._w_spin, self._h_spin, self._a1x_spin, self._a1y_spin):
            w.setVisible(not circle)
        self._bore_spin.setVisible(circle)
        if circle:
            self._bore_spin.setValue(b.radius_mm * 2.0)
        else:
            self._w_spin.setValue(b.width_mm)
            self._h_spin.setValue(b.height_mm)
            self._a1x_spin.setValue(b.a1_x_mm)
            self._a1y_spin.setValue(b.a1_y_mm)

        feats = canvas.selected_features() if canvas else []
        feat = feats[0] if len(feats) == 1 else None
        if isinstance(feat, RingPattern):
            self._ring_name.setText(feat.name)
            self._ring_count.setValue(int(feat.count))
            self._ring_dia.setValue(feat.ring_diameter_mm)
            self._ring_angle.setValue(feat.start_angle_deg)
            self._ring_sweep.setValue(feat.sweep_deg)
            self._ring_dir.setCurrentIndex(
                0 if feat.direction >= 0 else 1)
            self._ring_centre.setChecked(feat.center_well)
            self._ring_well_dia.setValue(feat.style.diameter_mm)
            self._ring_depth.setValue(feat.style.well_depth_mm)
            self._ring_rim.setValue(feat.style.rim_height_mm)
            self._refresh_naming("ring", feat)
        elif isinstance(feat, GridPattern):
            self._grid_name.setText(feat.name)
            self._grid_rows.setValue(int(feat.rows))
            self._grid_cols.setValue(int(feat.cols))
            self._grid_px.setValue(feat.pitch_x_mm)
            self._grid_py.setValue(feat.pitch_y_mm)
            self._grid_rot.setValue(feat.rotation_deg)
            self._grid_well_dia.setValue(feat.style.diameter_mm)
            self._grid_depth.setValue(feat.style.well_depth_mm)
            want = (1 if feat.col_dir >= 0 else -1,
                    1 if feat.row_dir >= 0 else -1)
            for dirs, btn in self._corner_btns.items():
                btn.setChecked(dirs == (want[0], -want[1]))
            self._refresh_naming("grid", feat)

        primary = canvas.primary() if canvas else None
        if primary and not primary[1]:
            ent = self._doc.entities.get(primary[0])
            if isinstance(ent, Well):
                pt = self._doc.entities.get(ent.center)
                self._well_name.setText(ent.name)
                if pt is not None:
                    dx, dy = self._doc.to_display(pt.x, pt.y)
                    self._well_x.setValue(dx)
                    self._well_y.setValue(dy)
                self._well_dia.setValue(ent.style.diameter_mm)
                self._well_depth.setValue(ent.style.well_depth_mm)
        elif primary:
            ent = self._doc.entities.get(primary[0])
            name = getattr(ent, "name", "?")
            self._member_lbl.setText(
                f"Position and diameter are driven by <b>{name}</b>. "
                f"Select the pattern to change them.")
            ov = (ent.overrides.get(primary[1])
                  if isinstance(ent, PatternFeature) else None)
            self._member_name.setText(ov.name if ov and ov.name else "")
            self._member_rim.setValue(
                ov.rim_height_mm if ov and ov.rim_height_mm is not None
                else (ent.style.rim_height_mm
                      if isinstance(ent, PatternFeature) else 0.0))

        self._refresh_constraints()
        self._refresh_summary()

    def _refresh_constraints(self) -> None:
        canvas = self._canvas
        if canvas is not None:
            for kind, btn in self._cons_btns.items():
                btn.setEnabled(canvas.can_add_constraint(kind))

        rep = canvas.last_report() if canvas else None
        if rep is None:
            self._dof_lbl.setText("")
        elif rep.status == DOFStatus.INCONSISTENT:
            self._dof_lbl.setText(
                "⚠ Conflicting constraints — shown in red on the plate.")
            self._dof_lbl.setStyleSheet(
                f"color: {COLORS.get('red', '#f38ba8')}; "
                f"font-size: {_sf(8)}pt;")
        elif rep.status == DOFStatus.WELL_DETERMINED:
            self._dof_lbl.setText("✓ Fully constrained")
            self._dof_lbl.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; "
                f"font-size: {_sf(8)}pt;")
        else:
            n = rep.dof or rep.n_free_vars
            self._dof_lbl.setText(f"{n} degree(s) of freedom — drag freely")
            self._dof_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")

        sig = tuple((c.id, c.kind, c.value, c.datum)
                    for c in (self._doc.constraints if self._doc else []))
        if sig == self._sig:
            return
        self._sig = sig
        while self._cons_list.count():
            it = self._cons_list.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        for c in (self._doc.constraints if self._doc else []):
            self._cons_list.addWidget(self._constraint_row(c))

    def _constraint_row(self, c) -> QWidget:
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(s(3))
        label = c.kind.replace("_", " ")
        if c.datum:
            label += f" · {c.datum.replace('_', ' ')}"
        lb = QLabel(label)
        lb.setStyleSheet(f"color: {COLORS['text']}; font-size: {_sf(8)}pt;")
        lb.setCursor(Qt.PointingHandCursor)
        lb.mousePressEvent = lambda _e, cid=c.id: self._select_constraint(cid)
        h.addWidget(lb, 1)
        if c.value is not None:
            sp = _dspin(-1000.0, 1000.0, 0.1)
            sp.setFixedWidth(s(92))
            sp.setValue(c.value)
            sp.valueChanged.connect(
                lambda v, cid=c.id: self._set_constraint_value(cid, v))
            h.addWidget(sp)
        x = QToolButton()
        x.setText("✕")
        x.setToolTip("Remove")
        x.clicked.connect(lambda _c=False, cid=c.id: self._remove_constraint(cid))
        h.addWidget(x)
        return row

    def _refresh_summary(self) -> None:
        if self._doc is None:
            return
        wells = self._doc.evaluate()
        ros = sum(1 for w in wells if w.rosette is not None)
        pats = len(self._doc.patterns())
        problems = self._doc.validate()
        txt = (f"{len(wells)} well(s) · {pats} pattern(s)"
               + (f" · {ros} rosette(s)" if ros else ""))
        if problems:
            txt += "<br><span style='color:#f38ba8'>⚠ " \
                   + "<br>⚠ ".join(problems[:3]) + "</span>"
        self._sum_lbl.setText(txt)

    # ── Mutators ──────────────────────────────────────────────────

    def _edited(self) -> None:
        if self._canvas is not None:
            self._canvas.solve()
            self._canvas.rebuild()
        self.document_edited.emit()

    def _set_origin(self, _i: int) -> None:
        if self._building or self._doc is None:
            return
        self._doc.boundary.origin_ref = self._origin_combo.currentData()
        self._edited()

    def _set_boundary(self, attr: str, value: float) -> None:
        if self._building or self._doc is None:
            return
        setattr(self._doc.boundary, attr, float(value))
        self._edited()

    def _feature(self) -> Optional[PatternFeature]:
        feats = self._canvas.selected_features() if self._canvas else []
        return feats[0] if len(feats) == 1 else None

    def _set_feat(self, attr: str, value) -> None:
        if self._building:
            return
        feat = self._feature()
        if feat is None:
            return
        setattr(feat, attr, value)
        self._edited()

    def _set_style(self, attr: str, value) -> None:
        if self._building:
            return
        feat = self._feature()
        if feat is not None:
            setattr(feat.style, attr, float(value))
            self._edited()
            return
        primary = self._canvas.primary() if self._canvas else None
        if primary and not primary[1] and self._doc:
            ent = self._doc.entities.get(primary[0])
            if isinstance(ent, Well):
                setattr(ent.style, attr, float(value))
                self._edited()

    def _set_naming(self, _which: str, attr: str, value) -> None:
        if self._building:
            return
        feat = self._feature()
        if feat is None:
            return
        setattr(feat.naming, attr, value)
        self._edited()

    def _autoname(self) -> None:
        """Shift this pattern's names clear of every other pattern's."""
        if self._building or self._doc is None:
            return
        feat = self._feature()
        if feat is None:
            return
        if self._canvas is not None:
            self._canvas.snapshot()          # one undo step for the whole shift
        ok = self._doc.autoname_feature(feat)
        self._edited()
        self._do_refresh(force=True)
        if not ok:
            logger.warning("autoname could not fully de-conflict %s",
                           feat.name)

    def _refresh_naming(self, which: str, feat: PatternFeature) -> None:
        spec = feat.naming
        combo = getattr(self, f"_{which}_scheme")
        idx = combo.findData(spec.scheme)
        if idx < 0:                       # tolerate an unknown stored scheme
            idx = combo.findData(NamingScheme.ANSI)
        combo.setCurrentIndex(max(idx, 0))
        getattr(self, f"_{which}_prefix").setText(spec.prefix)
        getattr(self, f"_{which}_start_row").setValue(int(spec.start_row))
        getattr(self, f"_{which}_start_col").setValue(int(spec.start_col))
        # ANSI is the only scheme with a row letter; the rest ignore it.
        getattr(self, f"_{which}_start_row").setEnabled(
            spec.scheme == NamingScheme.ANSI)

        lbl = getattr(self, f"_{which}_name_preview")
        names = self._doc.feature_member_names(feat) if self._doc else []
        clash = sorted(
            (self._doc.names_in_use(exclude=feat.id)
             & {n.upper() for n in names})) if self._doc else []
        shown = ", ".join(names[:4]) + (" …" if len(names) > 4 else "")
        if clash:
            lbl.setText(f"⚠ {len(clash)} name(s) clash with another pattern "
                        f"({', '.join(clash[:3])}"
                        f"{'…' if len(clash) > 3 else ''}). "
                        f"Wells sharing a name are dropped from the plate — "
                        f"use Auto-number or a prefix.")
            lbl.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {_sf(8)}pt;")
        else:
            lbl.setText(f"Wells: {shown}")
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(8)}pt;")

    def _set_corner(self, dirs) -> None:
        if self._building:
            return
        feat = self._feature()
        if not isinstance(feat, GridPattern):
            return
        feat.col_dir, feat.row_dir = dirs[0], -dirs[1]
        self._edited()

    def _set_well_name(self) -> None:
        if self._building or self._doc is None or self._canvas is None:
            return
        primary = self._canvas.primary()
        if not primary or primary[1]:
            return
        ent = self._doc.entities.get(primary[0])
        if isinstance(ent, Well):
            ent.name = self._well_name.text().strip() or ent.name
            self._edited()

    def _set_well_pos(self, axis: int, value: float) -> None:
        if self._building or self._doc is None or self._canvas is None:
            return
        primary = self._canvas.primary()
        if not primary or primary[1]:
            return
        ent = self._doc.entities.get(primary[0])
        if not isinstance(ent, Well):
            return
        pt = self._doc.entities.get(ent.center)
        if pt is None:
            return
        # The spin boxes read in the operator's chosen origin frame.
        dx, dy = self._doc.to_display(pt.x, pt.y)
        if axis == 0:
            dx = value
        else:
            dy = value
        pt.x, pt.y = self._doc.from_display(dx, dy)
        self._edited()

    def _set_member_name(self) -> None:
        if self._building:
            return
        self._set_member("name", self._member_name.text().strip() or None)

    def _set_member(self, attr: str, value) -> None:
        if self._building or self._doc is None or self._canvas is None:
            return
        primary = self._canvas.primary()
        if not primary or not primary[1]:
            return
        ent = self._doc.entities.get(primary[0])
        if not isinstance(ent, PatternFeature):
            return
        setattr(self._doc.override(ent, primary[1]), attr, value)
        self._edited()

    def _add_constraint(self, kind: str) -> None:
        if self._canvas is None:
            return
        ok, msg = self._canvas.add_constraint_for_selection(kind)
        self._canvas.status_message.emit(("✓ " if ok else "⚠ ") + msg)

    def _set_constraint_value(self, cid: int, value: float) -> None:
        if self._building or self._doc is None:
            return
        c = self._doc.constraint_by_id(cid)
        if c is not None:
            c.value = float(value)
            self._edited()

    def _remove_constraint(self, cid: int) -> None:
        if self._doc is None:
            return
        if self._canvas:
            self._canvas.snapshot()
        self._doc.remove_constraint(cid)
        self._sig = ()
        self._edited()

    def _select_constraint(self, cid: int) -> None:
        if self._canvas:
            self._canvas.select_constraint(cid)
        self.constraint_focus_requested.emit(cid)
