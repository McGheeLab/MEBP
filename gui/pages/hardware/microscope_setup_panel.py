"""
microscope_setup_panel.py — the ONE microscope setup surface (v7.5.x).

Configures the motorized body: which driver talks to it, what is loaded in each
**filter-cube slot** and each **nosepiece position**, and the focus preferences.

Deliberately a single widget with two hosts — the Hardware Setup → Microscope
sub-page, and the ⚙ dialog on the jog panel's Microscope card. The alternative
(two independently written surfaces editing the same settings) is exactly the
failure this codebase already paid for once: *"we have multiple surfaces for
calibrating mosaics, there should only be one … Currently everything is very
messed up"* (`MEBP_v75x_UNIFIED_MOSAIC_CALIBRATION.md`).

**Read from microscope** is the headline: a motorised Nikon body knows its own
optics, so the cassette's cube names ("DAPI", "FITC") and the nosepiece's
objectives (magnification, product code, NA, working distance) are read from the
hardware instead of typed — and cannot drift out of date. Typing stays available
for bodies that do not report, and an operator name always overrides.

Nothing is written to the store until ``commit()`` (Save, or the dialog's OK).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QInputDialog,
    QLabel, QLineEdit, QMessageBox, QPushButton, QSizePolicy, QSpinBox,
    QStackedWidget, QTextEdit, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

logger = logging.getLogger(__name__)

#: Backend identifier → label shown in the driver combo.
BACKEND_OPTIONS = [
    ("simulated", "Simulated (no hardware)"),
    ("nikon_ti", "Nikon Ti — Nikon SDK (COM)"),
    ("micromanager", "Nikon Ti — Micro-Manager (pymmcore)"),
]

_STATUS_MS = 500


class _SlotTable(QWidget):
    """Per-slot rows: number · what the body reports · operator name · Go."""

    #: A user cube was added to the shared catalogue (id).
    cube_saved = Signal(str)

    def __init__(self, kind: str, on_go, parent=None):
        super().__init__(parent)
        self._kind = kind                 # "filter" | "objective"
        self._on_go = on_go
        #: v7.17 — filter cubes carry emission/excitation wavelengths for the
        #: LabLink image-job sidecar.
        self._optics = (kind == "filter")
        #: v7.18 — objectives carry NA + working distance, picked from a
        #: catalogue. They used to rely entirely on the body reporting them, but
        #: the body only knows the product code programmed into its nosepiece
        #: (routinely blank, and sometimes a different variant of the same
        #: nominal name). Both figures are load-bearing: NA sizes every focus
        #: step through the depth of field, and working distance IS the collision
        #: bound for a rotation.
        self._objectives = (kind == "objective")
        self._rows: dict[int, dict] = {}
        self._cache: dict[int, str] = {}  # typed names surviving a rebuild
        self._optics_cache: dict[int, tuple] = {}   # (em, ex) across a rebuild
        self._spec_cache: dict[int, dict] = {}      # objective specs ditto
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(s(10))
        self._grid.setVerticalSpacing(s(4))
        self._grid.setColumnStretch(1, 3)   # fitted
        self._grid.setColumnStretch(2, 4)   # name
        self._build_header()

    #: Columns after the name column. Kept as a constant because
    #: ``set_slot_count`` tears rows down by count and must keep the header.
    _HEADER = ("#", "Fitted (read from microscope)", "Name shown in the app")
    _OPTICS_HEADER = ("Excitation  ctr / width", "Emission  ctr / width",
                      "Source")
    #: v7.18 — the catalogue picker plus the two figures that size a focus sweep
    #: and bound a rotation.
    _OBJ_HEADER = ("Objective type", "NA", "WD mm", "Source")

    def _build_header(self) -> None:
        cols = list(self._HEADER)
        if self._optics:
            cols += list(self._OPTICS_HEADER)
        elif self._objectives:
            cols += list(self._OBJ_HEADER)
        cols.append("")
        self._n_cols = len(cols)
        for col, text in enumerate(cols):
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
                f"font-weight: 600;")
            if self._objectives and text in self._OBJ_HEADER:
                lbl.setToolTip(
                    "This objective's numerical aperture and working distance.\n\n"
                    "Both are load-bearing, not decoration. NA sets the DEPTH OF "
                    "FIELD, which sizes every focus step — a 20x/0.75 has about a "
                    "third the depth of field of a 20x/0.45, so a wrong NA "
                    "under-samples the focus curve. Working distance is the "
                    "COLLISION BOUND: a quarter of it is how far the focus may "
                    "travel, and whether the nosepiece may rotate at all.\n\n"
                    "Pick an objective from the list to fill these in. Catalogue "
                    "figures are NOMINAL for the series ('Source' says so) and "
                    "real parts vary between variants — the numbers engraved on "
                    "YOUR objective win, and editing one marks it as datasheet.\n\n"
                    "If these disagree with what the microscope reports, the "
                    "SHORTER working distance is used and the disagreement is "
                    "flagged.")
            if text in self._OPTICS_HEADER:
                lbl.setToolTip(
                    "This cube's excitation and emission bands, as CENTER and "
                    "WIDTH in nanometres — a filter marked '470/40' is center "
                    "470, width 40, i.e. 450–490 nm.\n\n"
                    "Picking a cube from the list fills these in. Those values "
                    "are NOMINAL for the cube type ('Source' says so); real "
                    "parts vary, so confirm against your filter's datasheet for "
                    "publication work — editing a number marks it as datasheet.\n\n"
                    "The center is what reaches LabLink, so a deconvolution "
                    "recipe can compute the right PSF. Leave blank if you do "
                    "not know it: LabLink names a missing field, and a "
                    "plausible guess would silently change the result.")
            self._grid.addWidget(lbl, 0, col)

    # ── Build / read ──────────────────────────────────────────────

    def set_slot_count(self, count: int) -> None:
        self._absorb_typed()
        while self._grid.count() > self._n_cols:    # keep the header row
            item = self._grid.takeAt(self._grid.count() - 1)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        self._rows.clear()
        for pos in range(1, max(1, int(count)) + 1):
            num = QLabel(str(pos))
            num.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            num.setStyleSheet(f"color: {COLORS['subtext0']}; font-weight: 600;")
            num.setMinimumWidth(s(18))

            fitted = QLabel("—")
            fitted.setStyleSheet(f"color: {COLORS['subtext0']};")
            fitted.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            fitted.setMinimumWidth(s(1))

            # Filter cubes are a catalogue pick; objectives stay free text.
            # Either way ``entry["edit"]`` is a QLineEdit, so every caller and
            # test keeps using .text()/.setText() unchanged.
            if self._optics:
                name_widget = self._build_cube_combo(pos)
                edit = name_widget.lineEdit()
            else:
                # Objectives keep a plain free-text name and get their catalogue
                # picker as a SEPARATE column — see _build_objective_combo for
                # why the two must not share one editable widget.
                name_widget = edit = QLineEdit()
            edit.setText(self._cache.get(pos, ""))
            edit.setPlaceholderText("(empty)")

            go = QPushButton("Go")
            go.setToolTip(
                "Rotate the turret to this slot now (to see what is in it).")
            go.setFixedWidth(s(42))
            go.setEnabled(False)
            go.clicked.connect(lambda _c=False, p=pos: self._on_go(p))

            row = self._grid.rowCount()
            self._grid.addWidget(num, row, 0)
            self._grid.addWidget(fitted, row, 1)
            self._grid.addWidget(name_widget, row, 2)
            col = 3
            entry = {"fitted": fitted, "edit": edit, "go": go}
            if self._optics:
                entry["combo"] = name_widget
                stored = dict(self._optics_cache.get(pos) or {})
                for band in ("ex", "em"):
                    holder, ctr, width = self._build_band_cell(band, stored)
                    self._grid.addWidget(holder, row, col)
                    entry[band] = ctr            # legacy key: the CENTER spin
                    entry[f"{band}_width"] = width
                    # editingFinished, not valueChanged: a value being restored
                    # or auto-filled must not read as an operator edit.
                    ctr.editingFinished.connect(
                        lambda p=pos: self.mark_edited(p))
                    width.editingFinished.connect(
                        lambda p=pos: self.mark_edited(p))
                    col += 1
                prov = QLabel("—")
                prov.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
                prov.setMinimumWidth(s(1))
                prov.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
                self._grid.addWidget(prov, row, col)
                entry["prov"] = prov
                entry["provenance"] = str(stored.get("provenance") or "")
                entry["cube_id"] = str(stored.get("cube_id") or "")
                col += 1
            elif self._objectives:
                stored = dict(self._spec_cache.get(pos) or {})
                objective_combo = self._build_objective_combo(pos)
                self._grid.addWidget(objective_combo, row, col)
                entry["combo"] = objective_combo
                col += 1
                na = QDoubleSpinBox()
                na.setRange(0.0, 1.65)
                na.setDecimals(2)
                na.setSingleStep(0.05)
                na.setSpecialValueText("—")     # 0 means "not recorded"
                na.setValue(float(stored.get("numerical_aperture") or 0.0))
                na.setMinimumWidth(s(1))
                na.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
                wd = QDoubleSpinBox()
                wd.setRange(0.0, 60.0)
                wd.setDecimals(2)
                wd.setSingleStep(0.1)
                wd.setSuffix(" mm")
                wd.setSpecialValueText("—")
                wd.setValue(float(stored.get("working_distance_mm") or 0.0))
                wd.setMinimumWidth(s(1))
                wd.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
                self._grid.addWidget(na, row, col)
                self._grid.addWidget(wd, row, col + 1)
                entry["na"] = na
                entry["wd"] = wd
                col += 2
                # editingFinished, not valueChanged: a value being restored or
                # auto-filled from the catalogue must not read as an edit.
                na.editingFinished.connect(lambda p=pos: self.mark_edited(p))
                wd.editingFinished.connect(lambda p=pos: self.mark_edited(p))
                prov = QLabel("—")
                prov.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
                prov.setMinimumWidth(s(1))
                prov.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
                self._grid.addWidget(prov, row, col)
                entry["prov"] = prov
                entry["provenance"] = str(stored.get("provenance") or "")
                entry["objective_id"] = str(stored.get("objective_id") or "")
                entry["immersion"] = str(stored.get("immersion") or "air")
                entry["coverslip_mm"] = stored.get("coverslip_mm")
                entry["field_number_mm"] = stored.get("field_number_mm")
                entry["product_code"] = str(stored.get("product_code") or "")
                col += 1
            self._grid.addWidget(go, row, col)
            self._rows[pos] = entry
            if self._optics or self._objectives:
                self._sync_provenance(pos)

    #: Sentinel userData for the combo's trailing "save this as a cube" action.
    _SAVE_CUSTOM = "\x00save-custom"

    def _build_cube_combo(self, pos: int) -> QComboBox:
        """Editable combo of catalogue cubes, plus a save-as-custom action.

        Editable on purpose: a name that is not in the catalogue must still be
        typeable (the objective column has always allowed that, and an operator
        with an unlisted cube should not be blocked). Picking a listed cube
        auto-fills the bands; typing a name leaves them alone.
        """
        combo = QComboBox()
        combo.setEditable(True)
        combo.setInsertPolicy(QComboBox.NoInsert)
        combo.setMinimumWidth(s(1))
        combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        combo.setToolTip(
            "Pick a standard cube to fill its wavelengths in, or type a name "
            "of your own.")
        self._populate_cube_combo(combo)
        # `activated` (not currentIndexChanged) so only an OPERATOR pick
        # auto-fills — a programmatic rebuild or a set_labels() must never
        # silently overwrite wavelengths the operator entered by hand.
        combo.activated.connect(lambda _i, p=pos: self._on_cube_activated(p))
        return combo

    def _build_objective_combo(self, pos: int) -> QComboBox:
        """NON-editable catalogue picker, in its OWN column — v7.18.

        ⚠ Deliberately unlike the filter-cube combo, which doubles as the name
        field. An objective's slot NAME is the key for its µm/px calibration in
        ``ObjectiveCalibration``, for its parfocal offset, and for the spec stored
        here — and an editable combo rewrites its line-edit text whenever an item
        is selected, which is Qt behaviour no amount of care in the handler can
        undo. Sharing one widget therefore renamed the objective on every pick and
        orphaned the operator's measurements. Name and part are separate facts, so
        they get separate widgets: the name column is untouched free text, exactly
        as before, and this only fills in the optics.

        Not editable because an unlisted objective needs no entry here — the
        operator types the NA and WD directly, and "＋ Save this…" adds it to the
        catalogue for next time.
        """
        combo = QComboBox()
        combo.setMinimumWidth(s(1))
        combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        combo.setToolTip(
            "Pick the objective fitted here to fill in its NA and working "
            "distance. This does NOT change the name — that stays yours, because "
            "it is what this objective's µm/px calibration is filed under.")
        self._populate_objective_combo(combo)
        # `activated`, not currentIndexChanged: only an OPERATOR pick auto-fills.
        combo.activated.connect(lambda _i, p=pos: self._on_objective_activated(p))
        return combo

    def _populate_objective_combo(self, combo: QComboBox) -> None:
        keep = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("(from the microscope)", "")
        for obj in self._objective_catalogue():
            combo.addItem(obj.label, obj.id)
            combo.setItemData(combo.count() - 1, obj.describe(), Qt.ToolTipRole)
        combo.insertSeparator(combo.count())
        combo.addItem("＋ Save this slot as a new objective…", self._SAVE_CUSTOM)
        if keep:
            idx = combo.findData(keep)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def _objective_catalogue(self) -> list:
        try:
            from SupportClasses.ObjectiveCatalogue import get_store
            return get_store().all()
        except Exception as exc:      # a bad catalogue must not kill the page
            logger.warning(f"objective catalogue unavailable: {exc}")
            return []

    def refresh_objective_catalogue(self) -> None:
        """Re-read the objective catalogue into every row's combo."""
        for row in self._rows.values():
            combo = row.get("combo")
            if combo is not None:
                self._populate_objective_combo(combo)

    def _on_objective_activated(self, pos: int) -> None:
        row = self._rows.get(pos)
        combo = (row or {}).get("combo")
        if combo is None:
            return
        oid = combo.currentData()
        if oid == self._SAVE_CUSTOM:
            self._save_slot_as_objective(pos)
            return
        if not oid:
            return
        try:
            from SupportClasses.ObjectiveCatalogue import get_store
            obj = get_store().get(oid)
        except Exception:
            obj = None
        if obj is not None:
            self._apply_objective(pos, obj)

    def _apply_objective(self, pos: int, obj) -> None:
        """Stamp a catalogue objective's OPTICS onto one row. NA + WD + source.

        ⚠ **The slot NAME is never touched.** It is the key for this objective's
        µm/px calibration in ``ObjectiveCalibration``, for its parfocal offset,
        and for the spec written here, so renaming "4X" to
        "CFI Plan Fluor 4x/0.13" on a pick would orphan a measurement the operator
        already made — the class of change this update's plan document forbids.
        The catalogue supplies the OPTICS; the operator owns the identity. A blank
        slot is filled in only as a convenience for a fresh rig.
        """
        row = self._rows.get(pos)
        if not row or "na" not in row:
            return
        if not row["edit"].text().strip():
            row["edit"].setText(obj.name or obj.label)
        row["na"].setValue(float(obj.numerical_aperture or 0.0))
        row["wd"].setValue(float(obj.working_distance_mm or 0.0))
        # A catalogue value is only ever as strong as the catalogue says.
        row["provenance"] = obj.provenance
        row["objective_id"] = obj.id
        row["immersion"] = obj.immersion
        row["coverslip_mm"] = obj.coverslip_mm
        row["field_number_mm"] = obj.field_number_mm
        row["product_code"] = obj.product_code
        self._sync_provenance(pos)

    def _save_slot_as_objective(self, pos: int) -> None:
        """Save this row's numbers into the catalogue as a user objective."""
        from SupportClasses.ObjectiveCatalogue import (
            Objective, PROV_DATASHEET, get_store, safe_id)
        row = self._rows.get(pos)
        if not row or "na" not in row:
            return
        name = row["edit"].text().strip()
        na = float(row["na"].value() or 0.0)
        wd = float(row["wd"].value() or 0.0)
        # Put the combo back to whatever it was showing: the trailing action is a
        # command, not a selection, and leaving it selected would read as "this
        # slot is a '＋ Save…' objective".
        self._populate_objective_combo(row["combo"])
        if not name or not (na and wd):
            QMessageBox.warning(
                self, "Save objective",
                "Enter a name, a numerical aperture and a working distance "
                "before saving this as an objective.\n\n"
                "Both numbers are required because both are load-bearing: NA "
                "sizes every focus step through the depth of field, and the "
                "working distance is the collision bound for a nosepiece "
                "rotation. An entry missing either would be dropped on reload.")
            return
        obj = Objective(
            id=safe_id(name), name=name, numerical_aperture=na,
            working_distance_mm=wd, immersion=row.get("immersion") or "air",
            coverslip_mm=row.get("coverslip_mm"),
            field_number_mm=row.get("field_number_mm"),
            product_code=row.get("product_code") or "",
            # Typed by hand off the part, so it is at least datasheet-grade.
            provenance=PROV_DATASHEET)
        if get_store().save_user(obj):
            row["objective_id"] = obj.id
            row["provenance"] = obj.provenance
            self.refresh_objective_catalogue()
            self._sync_provenance(pos)
            self.cube_saved.emit(obj.id)

    def _populate_cube_combo(self, combo: QComboBox) -> None:
        keep = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("", "")            # unassigned
        for cube in self._catalogue():
            combo.addItem(cube.label, cube.id)
            idx = combo.count() - 1
            combo.setItemData(idx, cube.describe(), Qt.ToolTipRole)
        combo.insertSeparator(combo.count())
        combo.addItem("＋ Save this slot as a new cube…", self._SAVE_CUSTOM)
        combo.setEditText(keep)
        combo.blockSignals(False)

    def _catalogue(self) -> list:
        try:
            from SupportClasses.FilterCubeStore import get_store
            return get_store().all()
        except Exception as exc:      # a bad catalogue must not kill the page
            logger.warning(f"filter-cube catalogue unavailable: {exc}")
            return []

    def refresh_catalogue(self) -> None:
        """Re-read the cube catalogue into every row's combo."""
        for row in self._rows.values():
            combo = row.get("combo")
            if combo is not None:
                self._populate_cube_combo(combo)

    def _build_band_cell(self, band: str, stored: dict):
        """``(container, center_spin, width_spin)`` for one band."""
        holder = QWidget()
        lay = QHBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(3))
        prefix = "excitation" if band == "ex" else "emission"
        ctr = self._wavelength_spin(stored.get(f"{prefix}_nm") or 0)
        ctr.setToolTip(f"{prefix.capitalize()} band CENTER in nm.")
        width = self._width_spin(stored.get(f"{prefix}_width_nm") or 0)
        sep = QLabel("/")
        sep.setStyleSheet(f"color: {COLORS['subtext0']};")
        lay.addWidget(ctr)
        lay.addWidget(sep)
        lay.addWidget(width)
        return holder, ctr, width

    @staticmethod
    def _width_spin(value) -> QSpinBox:
        """Bandpass FWHM spin whose 0 means "width not known"."""
        from SupportClasses.MicroscopeConfigStore import _MAX_BANDWIDTH_NM
        spin = QSpinBox()
        spin.setRange(0, int(_MAX_BANDWIDTH_NM))
        spin.setSpecialValueText("—")
        spin.setSingleStep(5)
        spin.setFixedWidth(s(56))
        spin.setValue(int(value or 0))
        spin.setToolTip(
            "Band WIDTH (FWHM) in nm — the second number on a filter marked "
            "'470/40'. Leave at — if you only know the center.")
        return spin

    def _on_cube_activated(self, pos: int) -> None:
        row = self._rows.get(pos)
        if row is None:
            return
        combo = row.get("combo")
        if combo is None:
            return
        data = combo.currentData()
        if data == self._SAVE_CUSTOM:
            self._save_slot_as_cube(pos)
            return
        if not data:
            return
        cube = None
        try:
            from SupportClasses.FilterCubeStore import get_store
            cube = get_store().get(data)
        except Exception as exc:
            logger.warning(f"filter-cube lookup failed: {exc}")
        if cube is None:
            return
        self._apply_cube(pos, cube)

    def _apply_cube(self, pos: int, cube) -> None:
        """Stamp a catalogue cube onto one row (name + both bands + source)."""
        row = self._rows.get(pos)
        if row is None:
            return
        row["edit"].setText(cube.display_name or cube.id)
        for band, prefix in (("ex", "excitation"), ("em", "emission")):
            row[band].setValue(int(getattr(cube, f"{prefix}_nm") or 0))
            row[f"{band}_width"].setValue(
                int(getattr(cube, f"{prefix}_width_nm") or 0))
        row["dichroic_nm"] = cube.dichroic_nm
        # A catalogue value is only ever as strong as the catalogue says.
        row["provenance"] = cube.provenance if cube.has_wavelengths else ""
        row["cube_id"] = cube.id
        self._sync_provenance(pos)

    def _sync_provenance(self, pos: int) -> None:
        row = self._rows.get(pos) or {}
        lbl = row.get("prov")
        if lbl is None:
            return
        # The provenance VOCABULARY is shared between the two catalogues; only
        # the explanatory text differs (an objective's says "confirm against the
        # engraving", a cube's says "against the filter's datasheet").
        if self._objectives:
            from SupportClasses.ObjectiveCatalogue import (
                PROV_NOMINAL, PROVENANCE_TEXT,
            )
        else:
            from SupportClasses.FilterCubeStore import (
                PROV_NOMINAL, PROVENANCE_TEXT,
            )
        prov = str(row.get("provenance") or "")
        if not prov:
            lbl.setText("—")
            lbl.setToolTip(
                "No optics recorded for this slot — the microscope's own report "
                "is used, and a rotation falls back to a tiny conservative "
                "focus budget when it reports nothing."
                if self._objectives else
                "No wavelengths recorded for this slot.")
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
            return
        warn = prov == PROV_NOMINAL
        lbl.setText(("⚠ " if warn else "✓ ") + prov)
        lbl.setToolTip(PROVENANCE_TEXT.get(prov, prov))
        lbl.setStyleSheet(
            f"color: {COLORS['yellow'] if warn else COLORS['green']}; "
            f"font-size: {sf(8)}pt;")

    def mark_edited(self, pos: int) -> None:
        """Promote a row off "nominal" because the operator changed a number.

        A hand-entered value is not nominal-for-the-type any more — it is what
        this operator read off their part. It is NOT promoted to "measured":
        nothing here measured anything.
        """
        row = self._rows.get(pos)
        if row is None or "prov" not in row:
            return
        from SupportClasses.FilterCubeStore import PROV_DATASHEET, PROV_NOMINAL
        if str(row.get("provenance") or "") in ("", PROV_NOMINAL):
            if self._objectives:
                # An objective needs BOTH figures to be usable, but either one
                # being present is enough to say the row is no longer nominal.
                has_any = bool(row["na"].value() or row["wd"].value())
            else:
                has_any = bool(row["ex"].value() or row["em"].value())
            row["provenance"] = PROV_DATASHEET if has_any else ""
            self._sync_provenance(pos)

    def _save_slot_as_cube(self, pos: int) -> None:
        """Save this row's numbers into the catalogue as a user cube."""
        from SupportClasses.FilterCubeStore import (
            FilterCube, PROV_DATASHEET, get_store, safe_id,
        )
        row = self._rows.get(pos)
        if row is None:
            return
        combo = row.get("combo")
        suggested = row["edit"].text().strip()
        name, ok = QInputDialog.getText(
            self, "Save filter cube",
            "Name for this cube (it joins the pick-list on every rig):",
            text=suggested)
        name = (name or "").strip()
        if not ok or not name:
            if combo is not None:
                combo.setEditText(suggested)
            return
        if not (row["ex"].value() or row["em"].value()):
            QMessageBox.warning(
                self, "Save filter cube",
                "Enter at least one wavelength before saving this as a cube — "
                "an entry with no bands would add nothing to the list.")
            if combo is not None:
                combo.setEditText(suggested)
            return
        cube = FilterCube(
            id=safe_id(name.lower()),
            display_name=name,
            excitation_nm=row["ex"].value() or None,
            excitation_width_nm=row["ex_width"].value() or None,
            emission_nm=row["em"].value() or None,
            emission_width_nm=row["em_width"].value() or None,
            dichroic_nm=row.get("dichroic_nm"),
            # The operator typed these off their own part, so they are at least
            # datasheet-grade — but never claimed as measured.
            provenance=str(row.get("provenance") or "") or PROV_DATASHEET,
            notes="Saved from Hardware Setup → Microscope.",
        )
        store = get_store()
        existing = store.get(cube.id)
        if existing is not None and existing.builtin:
            if QMessageBox.question(
                    self, "Save filter cube",
                    f"'{name}' already exists as a built-in cube. Save your "
                    f"version so it overrides the built-in on this rig?"
            ) != QMessageBox.Yes:
                if combo is not None:
                    combo.setEditText(suggested)
                return
        if not store.save_user(cube):
            QMessageBox.warning(
                self, "Save filter cube",
                "The cube could not be written. See the log for details.")
            if combo is not None:
                combo.setEditText(suggested)
            return
        row["provenance"] = cube.provenance
        row["cube_id"] = cube.id
        self.refresh_catalogue()
        if combo is not None:
            idx = combo.findData(cube.id)
            if idx >= 0:
                combo.blockSignals(True)
                combo.setCurrentIndex(idx)
                combo.blockSignals(False)
        row["edit"].setText(cube.display_name)
        self._sync_provenance(pos)
        self.cube_saved.emit(cube.id)

    @staticmethod
    def _wavelength_spin(value) -> QSpinBox:
        """A nanometre spin whose 0 means "not known".

        The range comes from the store, the one owner of the plausible band —
        a second copy here would let an operator type a value they cannot save.
        Values below the band are still reachable by typing, so ``commit()``
        refuses them rather than quietly rounding one into range: a fabricated
        wavelength looks measured and changes a deconvolution's output.
        """
        from SupportClasses.MicroscopeConfigStore import WAVELENGTH_BAND_NM
        spin = QSpinBox()
        spin.setRange(0, int(WAVELENGTH_BAND_NM[1]))
        spin.setSpecialValueText("—")         # 0 renders as "unknown"
        spin.setSingleStep(5)
        spin.setFixedWidth(s(72))
        spin.setValue(int(value or 0))
        return spin

    def _absorb_typed(self) -> None:
        for pos, row in self._rows.items():
            text = row["edit"].text().strip()
            if text:
                self._cache[pos] = text
            else:
                self._cache.pop(pos, None)
            if self._optics and "em" in row:
                entry = self._row_optics(row)
                if entry:
                    self._optics_cache[pos] = entry
                else:
                    self._optics_cache.pop(pos, None)

    @staticmethod
    def _row_optics(row) -> dict:
        """One row's optics as the store's entry shape, or ``{}``.

        A width or dichroic with no center is dropped — it describes no band,
        and the store's cleaner would drop it anyway.
        """
        entry: dict = {}
        for band, prefix in (("ex", "excitation"), ("em", "emission")):
            center = int(row[band].value())
            if not center:
                continue
            entry[f"{prefix}_nm"] = float(center)
            width = int(row[f"{band}_width"].value())
            if width:
                entry[f"{prefix}_width_nm"] = float(width)
        if not entry:
            return {}
        dichroic = row.get("dichroic_nm")
        if dichroic:
            entry["dichroic_nm"] = float(dichroic)
        prov = str(row.get("provenance") or "")
        if prov:
            entry["provenance"] = prov
        cube_id = str(row.get("cube_id") or "")
        if cube_id:
            entry["cube_id"] = cube_id
        return entry

    def labels(self) -> dict:
        return {pos: row["edit"].text() for pos, row in self._rows.items()}

    def set_labels(self, labels: dict) -> None:
        self._cache.update({int(k): str(v) for k, v in (labels or {}).items()})
        for pos, row in self._rows.items():
            row["edit"].setText(str((labels or {}).get(pos, "")))

    # ── Filter-cube optics (v7.17) ────────────────────────────────

    def optics(self) -> dict:
        """``{cube name: {"emission_nm": .., "excitation_nm": ..}}``.

        Keyed by NAME because the store is: a wavelength is a property of the
        cube, not of the slot it happens to sit in, so moving a cube between
        slots must not lose it. Two slots sharing a name therefore share one
        entry — the last row wins, which is right for two cubes of one type.
        A slot with no name contributes nothing (there is no key to file it
        under), and a row left at "—" is reported ABSENT, not zero.
        """
        out: dict = {}
        if not self._optics:
            return out
        for _pos, row in self._rows.items():
            name = row["edit"].text().strip()
            if not name or "em" not in row:
                continue
            entry = self._row_optics(row)
            if entry:
                out[name] = entry
        return out

    def set_optics(self, table: dict) -> None:
        """Fill the wavelength spins from a name-keyed store table.

        ⚠ Call AFTER :meth:`set_labels` — the rows are matched by the name they
        are currently showing.
        """
        if not self._optics:
            return
        lookup = {str(k).strip().lower(): (v or {})
                  for k, v in (table or {}).items()}
        for pos, row in self._rows.items():
            if "em" not in row:
                continue
            entry = lookup.get(row["edit"].text().strip().lower(), {})
            for band, prefix in (("ex", "excitation"), ("em", "emission")):
                row[band].setValue(int(float(entry.get(f"{prefix}_nm") or 0)))
                row[f"{band}_width"].setValue(
                    int(float(entry.get(f"{prefix}_width_nm") or 0)))
            row["dichroic_nm"] = entry.get("dichroic_nm")
            row["provenance"] = str(entry.get("provenance") or "")
            row["cube_id"] = str(entry.get("cube_id") or "")
            # Select the catalogue entry this slot came from, so the combo shows
            # what it is rather than sitting on a blank index. Signals blocked:
            # this is a restore, and `activated` must stay operator-only.
            combo = row.get("combo")
            if combo is not None and row["cube_id"]:
                idx = combo.findData(row["cube_id"])
                if idx >= 0:
                    text = row["edit"].text()
                    combo.blockSignals(True)
                    combo.setCurrentIndex(idx)
                    combo.blockSignals(False)
                    # setCurrentIndex rewrites the edit text; the operator's own
                    # label wins over the catalogue's.
                    row["edit"].setText(text)
            stored = self._row_optics(row)
            if stored:
                self._optics_cache[pos] = stored
            self._sync_provenance(pos)

    # ── Objective optics (v7.18) ──────────────────────────────────

    @staticmethod
    def _row_spec(row) -> dict:
        """One row's objective optics as the store's entry shape, or ``{}``.

        Requires NA or WD — the store's cleaner drops an entry carrying neither,
        so returning one here would look saved and vanish on the next launch.
        """
        entry: dict = {}
        na = float(row["na"].value() or 0.0)
        wd = float(row["wd"].value() or 0.0)
        if na:
            entry["numerical_aperture"] = na
        if wd:
            entry["working_distance_mm"] = wd
        if not entry:
            return {}
        for key in ("coverslip_mm", "field_number_mm"):
            v = row.get(key)
            if v:
                entry[key] = float(v)
        entry["immersion"] = str(row.get("immersion") or "air")
        prov = str(row.get("provenance") or "")
        if prov:
            entry["provenance"] = prov
        for key in ("objective_id", "product_code"):
            v = str(row.get(key) or "")
            if v:
                entry[key] = v
        return entry

    def specs(self) -> dict:
        """``{objective name: {numerical_aperture, working_distance_mm, ...}}``.

        Keyed by NAME because the store is: NA and working distance are
        properties of the lens, not of the position it sits in, so moving it
        between positions must not lose them.
        """
        out: dict = {}
        if not self._objectives:
            return out
        for _pos, row in self._rows.items():
            name = row["edit"].text().strip()
            if not name or "na" not in row:
                continue
            entry = self._row_spec(row)
            if entry:
                out[name] = entry
        return out

    def set_specs(self, table: dict) -> None:
        """Fill the NA/WD spins from a name-keyed store table.

        ⚠ Call AFTER :meth:`set_labels` — rows are matched by the name they are
        currently showing.
        """
        if not self._objectives:
            return
        lookup = {str(k).strip().lower(): (v or {})
                  for k, v in (table or {}).items()}
        for pos, row in self._rows.items():
            if "na" not in row:
                continue
            entry = lookup.get(row["edit"].text().strip().lower(), {})
            row["na"].setValue(float(entry.get("numerical_aperture") or 0.0))
            row["wd"].setValue(float(entry.get("working_distance_mm") or 0.0))
            row["provenance"] = str(entry.get("provenance") or "")
            row["objective_id"] = str(entry.get("objective_id") or "")
            row["immersion"] = str(entry.get("immersion") or "air")
            row["coverslip_mm"] = entry.get("coverslip_mm")
            row["field_number_mm"] = entry.get("field_number_mm")
            row["product_code"] = str(entry.get("product_code") or "")
            combo = row.get("combo")
            if combo is not None:
                # Non-editable and in its own column, so this cannot disturb the
                # name — unlike the cube combo, which has to restore it.
                idx = combo.findData(row["objective_id"] or "")
                combo.blockSignals(True)
                combo.setCurrentIndex(max(0, idx))
                combo.blockSignals(False)
            stored = self._row_spec(row)
            if stored:
                self._spec_cache[pos] = stored
            self._sync_provenance(pos)

    def set_mounted(self, optics) -> None:
        """Show what the body reports, and mark the current slot."""
        by_pos = {int(getattr(o, "position", 0)): o for o in (optics or ())}
        for pos, row in self._rows.items():
            optic = by_pos.get(pos)
            if optic is None:
                row["fitted"].setText("—")
                row["fitted"].setStyleSheet(f"color: {COLORS['subtext0']};")
                continue
            if getattr(optic, "present", False):
                text = optic.label or optic.code or "fitted"
                if optic.detail:
                    text += f"   {optic.detail}"
                row["fitted"].setText(text)
                row["fitted"].setStyleSheet(f"color: {COLORS['text']};")
            else:
                row["fitted"].setText("empty")
                row["fitted"].setStyleSheet(f"color: {COLORS['subtext0']};")

    def adopt_mounted_names(self, optics) -> int:
        """Copy the body's names into the editable column. Returns how many."""
        filled = 0
        for optic in optics or ():
            pos = int(getattr(optic, "position", 0))
            row = self._rows.get(pos)
            if row is None or not getattr(optic, "present", False):
                continue
            name = optic.label or optic.code
            if name:
                row["edit"].setText(name)
                filled += 1
        return filled

    def set_enabled_go(self, enabled: bool, current: int | None = None) -> None:
        for pos, row in self._rows.items():
            row["go"].setEnabled(bool(enabled))
            is_current = current is not None and pos == current
            row["edit"].setStyleSheet(
                f"border: 1px solid {COLORS['green']};" if is_current else "")


class MicroscopeSetupPanel(QWidget):
    """Driver + filter-cube / objective assignments + focus preferences."""

    def __init__(self, store=None, controller=None, parent=None, *,
                 show_save: bool = True):
        super().__init__(parent)
        if store is None:
            from SupportClasses.MicroscopeConfigStore import get_store
            store = get_store()
        self._store = store
        if controller is None:
            from SupportClasses.MicroscopeControl import get_microscope
            controller = get_microscope()
        self._scope = controller
        self._show_save = show_save

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(s(14))
        root.addWidget(self._build_connection_group())
        root.addWidget(self._build_filter_group())
        root.addWidget(self._build_objective_group())
        root.addWidget(self._build_focus_group())
        if show_save:
            root.addLayout(self._build_save_row())
        root.addStretch(1)

        self._timer = QTimer(self)
        self._timer.setInterval(_STATUS_MS)
        self._timer.timeout.connect(self._refresh_live)

        self.load()

    # ── Hosting ───────────────────────────────────────────────────

    def showEvent(self, event):  # noqa: N802 (Qt override)
        super().showEvent(event)
        self._timer.start()
        self._refresh_live()

    def hideEvent(self, event):  # noqa: N802 (Qt override)
        self._timer.stop()
        super().hideEvent(event)

    # ── Connection ────────────────────────────────────────────────

    def _build_connection_group(self) -> QGroupBox:
        box = QGroupBox("Microscope body")
        lay = QVBoxLayout(box)
        lay.setSpacing(s(8))

        top = QHBoxLayout()
        top.setSpacing(s(8))
        top.addWidget(QLabel("Driver"))
        self._backend_combo = QComboBox()
        for value, label in BACKEND_OPTIONS:
            self._backend_combo.addItem(label, value)
        self._backend_combo.currentIndexChanged.connect(
            lambda _i: self._driver_stack.setCurrentIndex(
                max(0, self._backend_combo.currentIndex())))
        top.addWidget(self._backend_combo, stretch=1)
        btn_diag = QPushButton("Diagnostics…")
        btn_diag.setToolTip(
            "Report what the microscope actually exposes — devices, "
            "properties, positions and the resolved focus scale.")
        btn_diag.clicked.connect(self._show_diagnostics)
        top.addWidget(btn_diag)
        btn_probe = QPushButton("🔎 Check write support…")
        btn_probe.setToolTip(
            "Read-only: asks the driver whether it declares the optics "
            "name/code fields as settable, i.e. whether a rename made here "
            "could ever reach the body's own display. Writes nothing.")
        btn_probe.clicked.connect(self._probe_write_support)
        top.addWidget(btn_probe)
        lay.addLayout(top)

        self._status_lbl = QLabel("not connected")
        self._status_lbl.setWordWrap(True)
        lay.addWidget(self._status_lbl)

        lay.addWidget(self._hint(
            "Connect the body from Hardware Setup → Device → Connect "
            "Hardware, alongside the XY and Z + Pumps stages (the jog "
            "panel's Microscope card also has a Connect button). A change "
            "of driver here takes effect on the next connect after Save."))

        self._driver_stack = QStackedWidget()
        self._driver_stack.addWidget(self._build_sim_note())
        self._driver_stack.addWidget(self._build_ti_options())
        self._driver_stack.addWidget(self._build_mm_options())
        lay.addWidget(self._driver_stack)
        return box

    def _build_sim_note(self) -> QWidget:
        note = QLabel(
            "No hardware is contacted. Turrets and focus are modelled in "
            "software so this page and the jog controls stay usable.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        return note

    def _build_ti_options(self) -> QWidget:
        w = QWidget()
        form = QFormLayout(w)
        form.setContentsMargins(0, 0, 0, 0)
        self._prog_id_edit = QLineEdit()
        self._prog_id_edit.setPlaceholderText(
            "auto — Nikon.TiScope.NikonTi (Ti / Ti-E), then LvMic fallbacks")
        form.addRow("COM ProgID", self._prog_id_edit)
        self._cassette_spin = QSpinBox()
        self._cassette_spin.setRange(1, 2)
        self._cassette_spin.setToolTip(
            "Which filter-block cassette to drive on a dual-cassette body.")
        form.addRow("Filter cassette", self._cassette_spin)
        self._z_units_spin = QDoubleSpinBox()
        self._z_units_spin.setRange(0.001, 100000.0)
        self._z_units_spin.setDecimals(3)
        self._z_units_spin.setToolTip(
            "Fallback only. The SDK normally DECLARES its focus unit — a Ti-E "
            "on SDK 4.4.1 reports 'um' (1 unit per µm) and that declared value "
            "is used automatically. This applies only if no unit is reported.")
        form.addRow("Z units per µm (fallback)", self._z_units_spin)
        return w

    def _build_mm_options(self) -> QWidget:
        w = QWidget()
        form = QFormLayout(w)
        form.setContentsMargins(0, 0, 0, 0)
        self._mm_cfg_edit = QLineEdit()
        form.addRow("Config (.cfg)", self._path_row(
            self._mm_cfg_edit, self._browse_mm_config))
        self._mm_dir_edit = QLineEdit()
        self._mm_dir_edit.setPlaceholderText("(default install)")
        form.addRow("Micro-Manager dir", self._path_row(
            self._mm_dir_edit, self._browse_mm_dir))
        self._mm_filter_edit = QLineEdit()
        form.addRow("Filter device", self._mm_filter_edit)
        self._mm_objective_edit = QLineEdit()
        form.addRow("Nosepiece device", self._mm_objective_edit)
        self._mm_focus_edit = QLineEdit()
        form.addRow("Focus device", self._mm_focus_edit)
        note = QLabel(
            "⚠ Micro-Manager's Nikon adapter WRAPS Nikon's own driver and SDK; "
            "it does not replace them.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        form.addRow(note)
        return w

    def _path_row(self, edit: QLineEdit, handler) -> QWidget:
        w = QWidget()
        row = QHBoxLayout(w)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(4))
        row.addWidget(edit, stretch=1)
        btn = QPushButton("Browse…")
        btn.clicked.connect(handler)
        row.addWidget(btn)
        return w

    def _browse_mm_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Micro-Manager configuration", self._mm_cfg_edit.text(),
            "Micro-Manager config (*.cfg);;All files (*)")
        if path:
            self._mm_cfg_edit.setText(path)

    def _browse_mm_dir(self) -> None:
        path = QFileDialog.getExistingDirectory(
            self, "Micro-Manager install directory", self._mm_dir_edit.text())
        if path:
            self._mm_dir_edit.setText(path)

    # ── Slot groups ───────────────────────────────────────────────

    def _build_filter_group(self) -> QGroupBox:
        box = QGroupBox("Filter cubes")
        lay = QVBoxLayout(box)
        lay.setSpacing(s(8))
        lay.addWidget(self._hint(
            "Name what is loaded in each cassette slot — these names are what "
            "the jog panel and workflows show. The turret only reports a slot "
            "NUMBER, so this mapping is the part only you know.\n"
            "Em / Ex are this cube's emission and excitation wavelengths in nm. "
            "They are recorded nowhere else on the microscope, and a "
            "deconvolution recipe needs them; leave blank if unknown.\n"
            "⚠ 'Fitted' shows only cubes REGISTERED in the body (a Ti-E does "
            "not sense them, and registering one is done with Nikon's own "
            "tooling — not from here). A cube you fitted yourself still images "
            "normally and still gets driven to; it just reads 'empty' there, "
            "and the name you type is what the app uses everywhere."))
        lay.addLayout(self._slot_header(
            "Slots", "_filter_slots_spin", self._on_filter_slots,
            self._read_filters))
        self._filter_table = _SlotTable("filter", self._go_filter)
        lay.addWidget(self._filter_table)
        return box

    def _build_objective_group(self) -> QGroupBox:
        box = QGroupBox("Objectives")
        lay = QVBoxLayout(box)
        lay.setSpacing(s(8))
        lay.addWidget(self._hint(
            "Pick the objective in each nosepiece position, or type a name. "
            "Optical calibration (µm/px) stays on the Cameras tab — this is the "
            "position ↔ objective map used for switching.\n"
            "NA and WD are filled in by the pick and are NOT cosmetic: NA sets "
            "the depth of field, which sizes every focus step, and the working "
            "distance is the collision bound that decides how far the focus may "
            "travel and whether the nosepiece may rotate at all.\n"
            "⚠ Catalogue figures are nominal for the series — variants of the "
            "same nominal objective differ (a Plan Fluor 10x has 16 mm of WD "
            "where a Plan Achromat 20x has 1.2 mm). The numbers engraved on your "
            "objective win; editing one marks it as datasheet."))
        lay.addLayout(self._slot_header(
            "Positions", "_objective_slots_spin", self._on_objective_slots,
            self._read_objectives))
        self._objective_table = _SlotTable("objective", self._go_objective)
        lay.addWidget(self._objective_table)
        return box

    def _slot_header(self, caption, spin_attr, on_change, on_read):
        row = QHBoxLayout()
        row.setSpacing(s(8))
        row.addWidget(QLabel(caption))
        spin = QSpinBox()
        spin.setRange(1, 12)
        spin.valueChanged.connect(on_change)
        setattr(self, spin_attr, spin)
        row.addWidget(spin)
        row.addStretch(1)
        btn = QPushButton("↓ Read from microscope")
        btn.setToolTip(
            "Ask the body what is physically fitted and fill the names in.")
        btn.clicked.connect(on_read)
        row.addWidget(btn)
        if spin_attr == "_filter_slots_spin":
            self._btn_read_filters = btn
        else:
            self._btn_read_objectives = btn
        return row

    def _hint(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        lbl.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        return lbl

    def _on_filter_slots(self, value: int) -> None:
        self._filter_table.set_slot_count(value)
        self._refresh_live()

    def _on_objective_slots(self, value: int) -> None:
        self._objective_table.set_slot_count(value)
        self._refresh_live()

    # ── Focus ─────────────────────────────────────────────────────

    def _build_focus_group(self) -> QGroupBox:
        box = QGroupBox("Focus")
        form = QFormLayout(box)

        self._focus_live_lbl = QLabel("—")
        self._focus_live_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-family: Consolas, Menlo, monospace;")
        form.addRow("Current position", self._focus_live_lbl)

        self._focus_step_spin = QDoubleSpinBox()
        self._focus_step_spin.setRange(0.01, 5000.0)
        self._focus_step_spin.setDecimals(2)
        self._focus_step_spin.setSuffix(" µm")
        form.addRow("Default jog step", self._focus_step_spin)

        self._focus_dir_chk = QCheckBox("\"Up\" increases the focus position")
        self._focus_dir_chk.setToolTip(
            "If the Up button moves the focal plane the wrong way on this "
            "body, clear this box — no recalibration needed.")
        form.addRow(self._focus_dir_chk)

        self._focus_limits_chk = QCheckBox("Limit focus travel")
        self._focus_limits_chk.toggled.connect(self._sync_limit_enabled)
        form.addRow(self._focus_limits_chk)

        limits_row = QHBoxLayout()
        self._focus_min_spin = QDoubleSpinBox()
        self._focus_max_spin = QDoubleSpinBox()
        for spin in (self._focus_min_spin, self._focus_max_spin):
            spin.setRange(-1e6, 1e6)
            spin.setDecimals(1)
            spin.setSuffix(" µm")
        limits_row.addWidget(QLabel("min"))
        limits_row.addWidget(self._focus_min_spin, stretch=1)
        limits_row.addWidget(QLabel("max"))
        limits_row.addWidget(self._focus_max_spin, stretch=1)
        form.addRow("", limits_row)
        return box

    def _sync_limit_enabled(self, on: bool) -> None:
        self._focus_min_spin.setEnabled(on)
        self._focus_max_spin.setEnabled(on)

    def _build_save_row(self):
        row = QHBoxLayout()
        row.addStretch(1)
        self._saved_lbl = QLabel("")
        self._saved_lbl.setStyleSheet(f"color: {COLORS['green']};")
        row.addWidget(self._saved_lbl)
        btn = QPushButton("Save microscope setup")
        btn.setObjectName("accentBtn")
        btn.clicked.connect(self._save_clicked)
        row.addWidget(btn)
        return row

    def _save_clicked(self) -> None:
        if self.commit():
            self._saved_lbl.setText("Saved ✓")
            QTimer.singleShot(2500, lambda: self._saved_lbl.setText(""))

    # ── Load / commit ─────────────────────────────────────────────

    def load(self) -> None:
        """(Re)populate every control from the store."""
        st = self._store
        backend = st.get_backend()
        idx = next((i for i, (v, _l) in enumerate(BACKEND_OPTIONS)
                    if v == backend), 0)
        self._backend_combo.setCurrentIndex(idx)
        self._driver_stack.setCurrentIndex(idx)

        self._prog_id_edit.setText(str(st.get("prog_id", "") or ""))
        self._cassette_spin.setValue(int(st.get("cassette", 1) or 1))
        self._z_units_spin.setValue(float(st.get("z_units_per_um", 100.0)))
        self._mm_cfg_edit.setText(str(st.get("mm_config_path", "") or ""))
        self._mm_dir_edit.setText(str(st.get("mm_dir", "") or ""))
        self._mm_filter_edit.setText(
            str(st.get("mm_filter_device", "TIFilterBlock1")))
        self._mm_objective_edit.setText(
            str(st.get("mm_objective_device", "TINosePiece")))
        self._mm_focus_edit.setText(str(st.get("mm_focus_device", "TIZDrive")))

        self._filter_slots_spin.setValue(st.filter_slots())
        self._objective_slots_spin.setValue(st.objective_slots())
        self._filter_table.set_slot_count(st.filter_slots())
        self._objective_table.set_slot_count(st.objective_slots())
        self._filter_table.set_labels(st.filter_labels())
        self._objective_table.set_labels(st.objective_labels())
        # AFTER set_labels: the optics table is name-keyed, so the rows have to
        # be showing their names before they can be matched.
        self._filter_table.set_optics(st.filter_optics())
        # v7.18 — after set_labels, for the same reason: rows are matched by the
        # name they are currently showing.
        self._objective_table.set_specs(st.objective_specs())

        self._focus_step_spin.setValue(st.focus_step_um())
        self._focus_dir_chk.setChecked(st.focus_up_is_positive())
        lo, hi = st.focus_soft_limits_um()
        has = lo is not None or hi is not None
        self._focus_limits_chk.setChecked(has)
        self._focus_min_spin.setValue(lo if lo is not None else 0.0)
        self._focus_max_spin.setValue(hi if hi is not None else 10000.0)
        self._sync_limit_enabled(has)
        self._refresh_live()

    def commit(self) -> bool:
        """Write every control to the store. False = refused (with a reason)."""
        st = self._store
        backend = self._backend_combo.currentData()
        if backend == "micromanager" and not self._mm_cfg_edit.text().strip():
            QMessageBox.warning(
                self, "Microscope setup",
                "The Micro-Manager driver needs a configuration (.cfg) file.")
            return False

        st.set("backend", backend, save=False)
        st.set("prog_id", self._prog_id_edit.text().strip(), save=False)
        st.set("cassette", int(self._cassette_spin.value()), save=False)
        st.set("z_units_per_um", float(self._z_units_spin.value()), save=False)
        st.set("mm_config_path", self._mm_cfg_edit.text().strip(), save=False)
        st.set("mm_dir", self._mm_dir_edit.text().strip(), save=False)
        st.set("mm_filter_device",
               self._mm_filter_edit.text().strip() or "TIFilterBlock1",
               save=False)
        st.set("mm_objective_device",
               self._mm_objective_edit.text().strip() or "TINosePiece",
               save=False)
        st.set("mm_focus_device",
               self._mm_focus_edit.text().strip() or "TIZDrive", save=False)
        st.set("filter_slots", int(self._filter_slots_spin.value()), save=False)
        st.set("objective_slots", int(self._objective_slots_spin.value()),
               save=False)
        st.set("focus_step_um", float(self._focus_step_spin.value()),
               save=False)
        st.set("focus_up_is_positive", bool(self._focus_dir_chk.isChecked()),
               save=False)
        if self._focus_limits_chk.isChecked():
            lo = float(self._focus_min_spin.value())
            hi = float(self._focus_max_spin.value())
            st.set("focus_min_um", min(lo, hi), save=False)
            st.set("focus_max_um", max(lo, hi), save=False)
        else:
            st.set("focus_min_um", None, save=False)
            st.set("focus_max_um", None, save=False)

        # v7.17 filter-cube wavelengths. REFUSED, not rounded, when out of the
        # store's band: a wavelength nudged into range still looks measured, and
        # it silently changes what a deconvolution returns.
        from SupportClasses.MicroscopeConfigStore import (
            WAVELENGTH_BAND_NM, clean_bandwidth, clean_wavelength,
        )
        # ⚠ Validate ONLY the numeric optical fields, by name. An entry also
        # carries `provenance` and `cube_id` (strings) since v7.18, and a loop
        # over entry.items() would run those through a wavelength check, fail,
        # and try to format a string with ":.0f".
        _WAVELENGTH_FIELDS = {
            "emission_nm": "emission wavelength",
            "excitation_nm": "excitation wavelength",
            "dichroic_nm": "dichroic edge",
        }
        _WIDTH_FIELDS = {
            "emission_width_nm": "emission band width",
            "excitation_width_nm": "excitation band width",
        }
        optics = self._filter_table.optics()
        for name, entry in optics.items():
            for field, what in _WAVELENGTH_FIELDS.items():
                if field not in entry:
                    continue
                if clean_wavelength(entry[field]) is None:
                    QMessageBox.warning(
                        self, "Microscope setup",
                        f"{name}: {entry[field]!r} is not a plausible {what} "
                        f"(expected "
                        f"{WAVELENGTH_BAND_NM[0]:.0f}–{WAVELENGTH_BAND_NM[1]:.0f} nm).\n\n"
                        "Leave the field blank if you do not know it — LabLink "
                        "reports a missing wavelength by name, which is "
                        "recoverable; a wrong one is not.")
                    return False
            for field, what in _WIDTH_FIELDS.items():
                if field not in entry:
                    continue
                if clean_bandwidth(entry[field]) is None:
                    QMessageBox.warning(
                        self, "Microscope setup",
                        f"{name}: {entry[field]!r} is not a plausible {what} in "
                        f"nm. A filter marked '470/40' has a width of 40.\n\n"
                        "Leave it blank if you only know the band centre.")
                    return False

        st.set_filter_labels(self._filter_table.labels())
        st.set_objective_labels(self._objective_table.labels())
        # Whole-table replace, so clearing a row's wavelengths actually clears
        # it rather than leaving a stale entry behind.
        st.set_all_filter_optics(optics)
        st.set_all_objective_specs(self._objective_table.specs())
        st.save()
        return True

    # ── Live hardware ─────────────────────────────────────────────

    def _connected(self) -> bool:
        return bool(self._scope.state().connected)

    def _read_filters(self) -> None:
        self._read_mounted(self._filter_table, "filter cube")

    def _read_objectives(self) -> None:
        self._read_mounted(self._objective_table, "objective")

    def _read_mounted(self, table: _SlotTable, what: str) -> None:
        if not self._connected():
            QMessageBox.information(
                self, "Microscope setup",
                "Connect to the microscope first — the names are read from the "
                "body itself.\n\nConnect from Hardware Setup → Device → "
                "Connect Hardware, or from the jog panel's Microscope card.")
            return
        self._scope.refresh_mounted()
        self._scope.wait_idle(timeout=15.0)
        state = self._scope.state()
        optics = (state.mounted_filters if table is self._filter_table
                  else state.mounted_objectives)
        if not optics:
            QMessageBox.information(
                self, "Microscope setup",
                f"This body does not report which {what}s are fitted. "
                "Type the names instead.")
            return
        filled = table.adopt_mounted_names(optics)
        self._refresh_live()
        QMessageBox.information(
            self, "Microscope setup",
            f"Read {filled} {what}(s) from the microscope."
            + ("" if filled else " Every slot reported empty.")
            + "\n\nReview the names, then Save.")

    def _go_filter(self, position: int) -> None:
        if self._connected():
            self._scope.set_filter(position)

    def _go_objective(self, position: int) -> None:
        if self._connected():
            self._scope.set_objective(position)

    def _refresh_live(self) -> None:
        state = self._scope.state()
        connected = bool(state.connected)
        if connected:
            bits = [f"● connected via {state.backend.replace('_', ' ')}"]
            if state.filter_position:
                bits.append(f"cube slot {state.filter_position}")
            if state.objective_position:
                bits.append(f"objective position {state.objective_position}")
            if state.busy:
                bits.append("moving…")
            self._status_lbl.setText("   ·   ".join(bits))
            self._status_lbl.setStyleSheet(f"color: {COLORS['green']};")
        else:
            self._status_lbl.setText(
                "○ not connected" + (f" — {state.error}" if state.error else ""))
            self._status_lbl.setStyleSheet(
                f"color: {COLORS['red'] if state.error else COLORS['subtext0']};")

        self._filter_table.set_mounted(state.mounted_filters)
        self._objective_table.set_mounted(state.mounted_objectives)
        self._filter_table.set_enabled_go(connected and not state.busy,
                                          state.filter_position)
        self._objective_table.set_enabled_go(connected and not state.busy,
                                             state.objective_position)
        for btn in (getattr(self, "_btn_read_filters", None),
                    getattr(self, "_btn_read_objectives", None)):
            if btn is not None:
                btn.setEnabled(connected)

        if state.focus_um is None:
            self._focus_live_lbl.setText("—")
        else:
            text = f"{state.focus_um:,.2f} µm"
            if state.focus_min_um is not None and state.focus_max_um is not None:
                text += (f"   (travel {state.focus_min_um:,.0f} – "
                         f"{state.focus_max_um:,.0f} µm)")
            self._focus_live_lbl.setText(text)

    # ── Diagnostics ───────────────────────────────────────────────

    def _show_diagnostics(self) -> None:
        try:
            text = self._scope.diagnostics()
        except Exception as exc:
            text = f"Diagnostics failed: {exc}"
        dlg = QDialog(self)
        dlg.setWindowTitle("Microscope diagnostics")
        dlg.setMinimumSize(s(620), s(430))
        lay = QVBoxLayout(dlg)
        view = QTextEdit()
        view.setReadOnly(True)
        view.setPlainText(text)
        view.setStyleSheet("font-family: Consolas, Menlo, monospace;")
        lay.addWidget(view)
        btns = QDialogButtonBox(QDialogButtonBox.Close)
        btns.rejected.connect(dlg.reject)
        btns.accepted.connect(dlg.accept)
        lay.addWidget(btns)
        from gui.widgets.components import exec_dialog
        exec_dialog(dlg)

    def _probe_write_support(self) -> None:
        if not self._connected():
            QMessageBox.information(
                self, "Microscope setup",
                "Connect to the microscope first — this probe talks to the "
                "body itself.")
            return
        op = self._scope.probe_optic_write_support()
        self._scope.wait_idle(timeout=15.0)
        if op.error:
            # Surface it rather than rendering the (stale or empty) cached
            # result, which would read as "this driver has no such notion" when
            # the truth is that the request never ran.
            QMessageBox.warning(
                self, "Microscope setup",
                f"The probe did not run: {op.error}")
            return
        text = self._format_write_support(self._scope.state().optic_write_support)
        dlg = QDialog(self)
        dlg.setWindowTitle("Optics write-support probe")
        dlg.setMinimumSize(s(600), s(420))
        lay = QVBoxLayout(dlg)
        view = QTextEdit()
        view.setReadOnly(True)
        view.setPlainText(text)
        view.setStyleSheet(f"font-family: Consolas, Menlo, monospace; "
                           f"font-size: {sf(9)}pt;")
        lay.addWidget(view)
        btns = QDialogButtonBox(QDialogButtonBox.Close)
        btns.rejected.connect(dlg.reject)
        btns.accepted.connect(dlg.accept)
        lay.addWidget(btns)
        from gui.widgets.components import exec_dialog
        exec_dialog(dlg)

    #: Verdict → the line shown for one field.
    _WRITE_VERDICT_TEXT = {
        True: "WRITABLE — the driver declares a setter",
        False: "read-only — the driver declares no setter",
        None: "undeterminable — no type information exposed",
    }

    @staticmethod
    def _format_write_support(result: dict) -> str:
        """Render probe_optic_write_support()'s result for the operator.

        This is the ONLY surface allowed to say anything about whether a rename
        could reach the body's display — nothing else in this panel should imply
        it, since it has never been confirmed on real hardware.
        """
        if not result:
            return (
                "No result to report.\n\n"
                "Either no optics could be enumerated, or this driver has no "
                "notion of an optics database at all (the Simulated and "
                "Micro-Manager drivers always report this)."
            )
        lines: list[str] = []
        verdicts: list = []
        for logical, fields in result.items():
            lines.append(f"{logical.upper()}")
            for name, verdict in sorted(fields.items()):
                verdicts.append(verdict)
                lines.append(
                    f"  {name:<6} "
                    + MicroscopeSetupPanel._WRITE_VERDICT_TEXT.get(
                        verdict, str(verdict)))
        lines.append("")
        lines.append("Read-only inspection — nothing was written to the body.")

        if True in verdicts:
            head = (
                "⚠ The driver DECLARES a setter — but on the Nikon Ti-E that "
                "is a known FALSE POSITIVE.\n\n"
                "Hardware-verified on this rig (2026-08-12, SDK 4.4.1.714): "
                "the Ti-E declares both Name and Code settable and then "
                "refuses every write at runtime with its own message, "
                "\"Database entry cannot be modified.\" The optics database is "
                "read-only through this SDK.\n\n"
                "The body's display follows a Code it SENSES from the fitted "
                "optic (resolved through Nikon's own catalogues in "
                "C:\\Program Files\\Nikon\\Shared\\Data\\Ti). A slot reporting "
                "no code is one the body sees nothing coded in — there is no "
                "entry to rename. The name you type here is an app-side "
                "label, which is the right place for it."
            )
        elif False in verdicts:
            head = (
                "❌ The identity fields are declared read-only.\n\n"
                "This body's optics names cannot be set from software. They "
                "are resolved by looking a hardware-sensed Code up in Nikon's "
                "own catalogue (an empty position's Name/NA/WD already raise "
                "\"No database code is associated with this optical "
                "element\" — the signature of a lookup, not a free field). "
                "So a CODED Nikon cube or objective already updates the "
                "physical display by itself the moment it is fitted, and an "
                "UNCODED one has no Code to attach a name to either way. The "
                "name you type here stays an app-side label."
            )
        else:
            head = (
                "❔ Undeterminable from the driver's type information.\n\n"
                "The wrapper exposes no setter/getter declaration for these "
                "fields, so whether a rename could reach the body cannot be "
                "answered without attempting one real write. Nothing was "
                "written — ask before trying, since the field that resolves an "
                "objective's working distance is one of these."
            )
        return head + "\n\n" + "\n".join(lines)
