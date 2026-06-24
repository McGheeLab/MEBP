"""
mosaic_settings_dialog.py — Pop-out settings for the full-plate Mosaic scan.

v7.5.x: A small modal dialog to tune every parameter of the Calibration →
Plate Location "Mosaic scan" — camera timing (so a slow / long-exposure camera
gets enough time to capture a sharp post-move frame), the raster (FOV overlap,
mosaic resolution), and well detection (HoughCircles sensitivity + radius
tolerance).

Usage::

    dlg = MosaicScanSettingsDialog(current_settings, parent=page)
    if dlg.exec():
        new_settings = dlg.values()      # dict with the same keys

Settings dict keys (see ``MOSAIC_SCAN_DEFAULTS``):
    overlap_pct, settle_ms, fresh_frames, fresh_timeout_s,
    target_px, detect_param2, detect_tol_pct
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox, QLabel,
    QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QPushButton,
    QDialogButtonBox,
)

from gui.styles import COLORS
from gui.scaling import s, sp


# Single source of truth for the mosaic-scan parameters + their defaults.
MOSAIC_SCAN_DEFAULTS: dict = {
    "overlap_pct": 25,        # raster overlap (% of camera FOV)
    "settle_ms": 300,         # extra settle after each move before capture (ms)
    "fresh_frames": 3,        # new frames to wait for post-move
    "fresh_timeout_s": 2.5,   # max wait for fresh frames, per tile (s)
    "target_px": 3000,        # stitched mosaic long-edge resolution (px)
    "detect_param2": 30,      # HoughCircles accumulator threshold
    "detect_tol_pct": 35,     # well-radius tolerance (%)
    "frame_orient": "none",   # per-tile camera-mount transform (see below)
    "fov_um": 0,              # camera FOV width (µm); 0 = auto from µm/px
    "spacing_um": 0,          # explicit grid spacing (µm); 0 = auto (FOV·(1−ov))
    "register": True,         # align tiles by phase correlation while stitching
    "max_shift_um": 0,        # bound on the per-tile alignment shift (µm); 0=auto
    "cal_cols": 5,            # calibration mosaic grid columns
    "cal_rows": 5,            # calibration mosaic grid rows
}

# Camera-mount tile transforms: stored value → display label.
FRAME_ORIENT_OPTIONS = [
    ("none", "None"),
    ("rot180", "Rotate 180°"),
    ("fliph", "Flip horizontal"),
    ("flipv", "Flip vertical"),
]


def merged_settings(stored: dict | None) -> dict:
    """Return defaults overlaid with any valid stored values."""
    out = dict(MOSAIC_SCAN_DEFAULTS)
    if isinstance(stored, dict):
        for k in out:
            if k in stored and stored[k] is not None:
                try:
                    out[k] = type(out[k])(stored[k])
                except (TypeError, ValueError):
                    pass
    return out


class MosaicScanSettingsDialog(QDialog):
    """Modal editor for the mosaic-scan parameters."""

    def __init__(self, settings: dict | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mosaic Scan Settings")
        self.setModal(True)
        self.setMinimumWidth(s(420))
        self._spins: dict = {}
        self._combos: dict = {}
        self._checks: dict = {}
        self._build_ui()
        self.set_values(merged_settings(settings))

    # ── Build ──────────────────────────────────────────────────────

    def _int_spin(self, lo, hi, suffix=""):
        sb = QSpinBox()
        sb.setRange(lo, hi)
        if suffix:
            sb.setSuffix(suffix)
        return sb

    def _dbl_spin(self, lo, hi, step, decimals, suffix=""):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setSingleStep(step)
        sb.setDecimals(decimals)
        if suffix:
            sb.setSuffix(suffix)
        return sb

    def _add_row(self, form, key, label, widget, tip):
        widget.setToolTip(tip)
        self._spins[key] = widget
        lbl = QLabel(label)
        lbl.setToolTip(tip)
        form.addRow(lbl, widget)

    def _add_combo_row(self, form, key, label, options, tip):
        combo = QComboBox()
        for value, display in options:
            combo.addItem(display, value)
        combo.setToolTip(tip)
        self._combos[key] = combo
        lbl = QLabel(label)
        lbl.setToolTip(tip)
        form.addRow(lbl, combo)

    def _add_check_row(self, form, key, label, tip):
        chk = QCheckBox()
        chk.setToolTip(tip)
        self._checks[key] = chk
        lbl = QLabel(label)
        lbl.setToolTip(tip)
        form.addRow(lbl, chk)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(s(12), s(12), s(12), s(12))
        root.setSpacing(s(8))

        intro = QLabel(
            "Tune the full-plate mosaic scan. If the camera can't keep up "
            "(blurred / pre-move tiles), increase the settle time and/or the "
            "number of fresh frames to wait.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(intro)

        # ── Camera timing ───────────────────────────────────────────
        cam_box = QGroupBox("Camera timing")
        cam_form = QFormLayout(cam_box)
        self._add_row(
            cam_form, "settle_ms", "Settle after move",
            self._int_spin(0, 5000, " ms"),
            "Extra wait after each XY move before grabbing a frame. Increase "
            "for slow or long-exposure cameras so the tile is sharp and "
            "post-move.")
        self._add_row(
            cam_form, "fresh_frames", "Fresh frames to wait",
            self._int_spin(1, 30),
            "Number of NEW frames the live grabber must deliver after the move "
            "before capture — drains the buffered backlog so the captured "
            "frame is genuinely post-move.")
        self._add_row(
            cam_form, "fresh_timeout_s", "Frame wait timeout",
            self._dbl_spin(0.5, 15.0, 0.5, 1, " s"),
            "Maximum time to wait for the fresh frames per tile before "
            "proceeding with whatever the camera last delivered.")
        root.addWidget(cam_box)

        # ── Raster ──────────────────────────────────────────────────
        ras_box = QGroupBox("Raster")
        ras_form = QFormLayout(ras_box)
        self._add_row(
            ras_form, "overlap_pct", "Tile overlap",
            self._int_spin(5, 50, " %"),
            "Raster step = camera FOV × (1 − overlap). Higher overlap = more "
            "tiles but more robust stitching. (Ignored if Grid spacing is set.)")
        self._add_row(
            ras_form, "fov_um", "Camera FOV width",
            self._int_spin(0, 50000, " µm"),
            "Real width of the camera field of view in µm. 0 = auto (frame "
            "width × calibrated µm/px). Set this if the auto FOV is wrong — it "
            "sizes each tile AND the auto grid spacing, so tiles tile correctly.")
        self._add_row(
            ras_form, "spacing_um", "Grid spacing",
            self._int_spin(0, 50000, " µm"),
            "Explicit distance between tile centres (stage µm). 0 = auto "
            "(FOV × (1 − overlap)). Set this to dial coverage/overlap directly "
            "when the FOV-derived spacing doesn't match the stage.")
        self._add_row(
            ras_form, "target_px", "Mosaic resolution",
            self._int_spin(1000, 8000, " px"),
            "Long-edge pixel size of the stitched mosaic — affects display "
            "detail and detection accuracy (and memory).")
        root.addWidget(ras_box)

        # ── Stitch alignment (registration) ─────────────────────────
        reg_box = QGroupBox("Stitch alignment")
        reg_form = QFormLayout(reg_box)
        self._add_check_row(
            reg_form, "register", "Global alignment",
            "Trust the (accurate) stage for relative tile placement, then "
            "correct a single SYSTEMATIC stage↔image offset for the WHOLE "
            "mosaic: measure each textured overlap by phase correlation and "
            "apply ONE robust (median) shift to all tiles. Featureless tiles "
            "(between wells / no edge) contribute nothing and are unaffected. "
            "No tile is ever shifted on its own.")
        self._add_row(
            reg_form, "max_shift_um", "Max alignment shift",
            self._int_spin(0, 5000, " µm"),
            "Upper bound on the single global alignment shift. 0 = auto (20% "
            "of the FOV). Keeps the correction sane even if few overlaps are "
            "textured.")
        root.addWidget(reg_box)

        # ── Well detection ──────────────────────────────────────────
        det_box = QGroupBox("Well detection")
        det_form = QFormLayout(det_box)
        self._add_row(
            det_form, "detect_param2", "Detection sensitivity",
            self._int_spin(10, 120),
            "HoughCircles accumulator threshold. LOWER finds more circles "
            "(and more false positives); higher is stricter.")
        self._add_row(
            det_form, "detect_tol_pct", "Radius tolerance",
            self._int_spin(10, 60, " %"),
            "How far a detected circle's radius may differ from the plate's "
            "known well radius and still count.")
        root.addWidget(det_box)

        # ── Tile orientation (camera mount) ─────────────────────────
        orient_box = QGroupBox("Tile orientation (camera mount)")
        orient_form = QFormLayout(orient_box)
        self._add_combo_row(
            orient_form, "frame_orient", "Rotate / flip each tile",
            FRAME_ORIENT_OPTIONS,
            "Use ONLY if the stitched tiles don't line up (the camera is "
            "mounted rotated or mirrored relative to the stage). Rotates / "
            "flips each captured frame so it matches stage motion. Leave at "
            "None if tiles already align.")
        root.addWidget(orient_box)

        # ── Buttons ─────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        reset = QPushButton("Restore defaults")
        reset.clicked.connect(lambda: self.set_values(MOSAIC_SCAN_DEFAULTS))
        btn_row.addWidget(reset)
        btn_row.addStretch()
        bb = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        btn_row.addWidget(bb)
        root.addLayout(btn_row)

    # ── Values ─────────────────────────────────────────────────────

    def set_values(self, settings: dict) -> None:
        vals = merged_settings(settings)
        # Keep ALL incoming keys (incl. ones this dialog has no control for, e.g.
        # the calibration grid size) so values() round-trips them — clicking OK
        # must never silently drop a setting it didn't display.
        self._all = dict(vals)
        for key, sb in self._spins.items():
            sb.setValue(vals[key])
        for key, combo in self._combos.items():
            idx = combo.findData(vals[key])
            combo.setCurrentIndex(idx if idx >= 0 else 0)
        for key, chk in self._checks.items():
            chk.setChecked(bool(vals[key]))

    def values(self) -> dict:
        out = dict(getattr(self, "_all", {}))   # preserve undisplayed keys
        for key, sb in self._spins.items():
            v = sb.value()
            out[key] = int(v) if isinstance(sb, QSpinBox) else float(v)
        for key, combo in self._combos.items():
            out[key] = combo.currentData()
        for key, chk in self._checks.items():
            out[key] = chk.isChecked()
        return out
