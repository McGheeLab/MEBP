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

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QSizePolicy, QSpinBox,
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

    def __init__(self, kind: str, on_go, parent=None):
        super().__init__(parent)
        self._kind = kind                 # "filter" | "objective"
        self._on_go = on_go
        self._rows: dict[int, dict] = {}
        self._cache: dict[int, str] = {}  # typed names surviving a rebuild
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(s(10))
        self._grid.setVerticalSpacing(s(4))
        self._grid.setColumnStretch(1, 3)   # fitted
        self._grid.setColumnStretch(2, 4)   # name
        self._build_header()

    def _build_header(self) -> None:
        for col, text in enumerate(
                ("#", "Fitted (read from microscope)", "Name shown in the app",
                 "")):
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
                f"font-weight: 600;")
            self._grid.addWidget(lbl, 0, col)

    # ── Build / read ──────────────────────────────────────────────

    def set_slot_count(self, count: int) -> None:
        self._absorb_typed()
        while self._grid.count() > 4:              # keep the header row
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

            edit = QLineEdit(self._cache.get(pos, ""))
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
            self._grid.addWidget(edit, row, 2)
            self._grid.addWidget(go, row, 3)
            self._rows[pos] = {"fitted": fitted, "edit": edit, "go": go}

    def _absorb_typed(self) -> None:
        for pos, row in self._rows.items():
            text = row["edit"].text().strip()
            if text:
                self._cache[pos] = text
            else:
                self._cache.pop(pos, None)

    def labels(self) -> dict:
        return {pos: row["edit"].text() for pos, row in self._rows.items()}

    def set_labels(self, labels: dict) -> None:
        self._cache.update({int(k): str(v) for k, v in (labels or {}).items()})
        for pos, row in self._rows.items():
            row["edit"].setText(str((labels or {}).get(pos, "")))

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
            "NUMBER, so this mapping is the part only you know."))
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
            "Name the objective in each nosepiece position. Optical "
            "calibration (µm/px) stays on the Cameras tab — this is the "
            "position ↔ objective map used for switching."))
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

        st.set_filter_labels(self._filter_table.labels())
        st.set_objective_labels(self._objective_table.labels())
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
        dlg.exec()
