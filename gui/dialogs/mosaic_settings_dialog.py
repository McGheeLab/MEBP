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
    "avg_frames": 1,          # raw frames averaged per tile (SNR ×√N); 1 = off
    "full_res_scan": False,   # capture tiles at full sensor resolution
    "target_px": 3000,        # stitched mosaic long-edge resolution (px)
    "detect_param2": 30,      # HoughCircles accumulator threshold
    "detect_tol_pct": 35,     # well-radius tolerance (%)
    "register": True,         # align tiles by phase correlation while stitching
    "max_shift_um": 0,        # bound on the per-tile alignment shift (µm); 0=auto
    "reg_method": "fourier_mellin",  # pairwise registration method (see below)
    "regularize_intensity": True,    # even out vignetting + tile level after the scan
    "cal_cols": 5,            # calibration mosaic grid columns
    "cal_rows": 5,            # calibration mosaic grid rows
}

# v7.5.x — REMOVED, and they must not come back here:
#
#   frame_orient ("Tile orientation (camera mount)") — a coarse
#       none/rot180/fliph/flipv per-tile transform predating the MEASURED
#       camera->stage orientation. It STACKED on top of the calibrated
#       ``MosaicBuilder._orient_tile``, and because only the fluorescence scan
#       still honoured it, the same camera produced two differently-oriented
#       mosaics (the plate scan applied rotation + flip_y; fluorescence applied
#       the coarse rot180 and LOST the flip). Orientation now has exactly one
#       home: the per-camera calibration store, via SupportClasses/
#       MosaicCalibration.
#
#   fov_um / spacing_um — µm/px and grid-step overrides that sat ABOVE every
#       measured value in the precedence, forever, from a hidden Advanced
#       submenu. They are why a freshly calibrated camera could keep scanning at
#       an old scale. The measured FOV now sizes the tiles and the step.
#
# Existing settings.json files may still carry these keys; they are simply
# ignored (``merged_settings`` only copies keys present in the defaults).

# Pairwise registration method: stored value → display label.
REG_METHOD_OPTIONS = [
    ("fourier_mellin", "Fourier-Mellin (auto — rotation + scale + shift)"),
    ("phase", "Phase correlation (auto — shift only)"),
    ("off", "Off — manual / stage only"),
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
        # v7.14 — keep the full-resolution memory note live: the canvas cost
        # is what makes the toggle worth (or not worth) enabling, so it must
        # not be a static sentence the operator reads once.
        try:
            self._checks["full_res_scan"].toggled.connect(
                lambda _v: self._refresh_full_res_note())
            self._spins["target_px"].valueChanged.connect(
                lambda _v: self._refresh_full_res_note())
        except (KeyError, AttributeError):      # pragma: no cover
            pass
        self._refresh_full_res_note()

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
        self._add_row(
            cam_form, "avg_frames", "Average frames per tile",
            self._int_spin(1, 32),
            "Average N frames per tile — SNR improves ×√N, the cheapest "
            "signal boost for dim fluorescence. Costs N−1 extra frame times "
            "per tile, so a long exposure makes the scan proportionally "
            "slower.\n\n"
            "Averaging is SKIPPED for any tile whose frames do not show the "
            "same thing (stage still settling, drift, a lamp flicker) — that "
            "tile falls back to one sharp frame rather than a blurred mean.\n\n"
            "v7.14: this now applies to the plate, rosette and single-well "
            "scans too; previously only the fluorescence page honoured it. "
            "1 = off.")
        # v7.14 — full-resolution capture. Binning does not change the field
        # of view, so this scans the SAME tiles; it costs per-tile transfer
        # time, not coverage. The canvas note below is the load-bearing part:
        # extra sensor pixels only reach the operator if the stitched canvas
        # can hold them, and canvas memory grows with its SQUARE.
        self._add_check_row(
            cam_form, "full_res_scan", "Scan at full camera resolution",
            "Switch the camera to its full sensor resolution for the scan "
            "and restore the preview resolution afterwards. Binning does not "
            "change the field of view, so the scan visits the same tiles — it "
            "costs per-tile transfer time, not coverage. Only worthwhile with "
            "a raised mosaic resolution (see below): the tiles are resized "
            "into the canvas, so a small canvas throws the extra pixels away.")
        self._full_res_note = QLabel("")
        self._full_res_note.setWordWrap(True)
        self._full_res_note.setStyleSheet(f"color: {COLORS['subtext0']};")
        cam_form.addRow("", self._full_res_note)
        root.addWidget(cam_box)

        # ── Raster ──────────────────────────────────────────────────
        ras_box = QGroupBox("Raster")
        ras_form = QFormLayout(ras_box)
        self._add_row(
            ras_form, "overlap_pct", "Tile overlap",
            self._int_spin(5, 50, " %"),
            "Raster step = camera FOV × (1 − overlap). Higher overlap = more "
            "tiles but more robust stitching. Below ~15% a small µm/px error "
            "opens gaps between tiles and registration has too little shared "
            "texture to lock onto; 25% is recommended.")
        self._add_row(
            ras_form, "target_px", "Mosaic resolution",
            self._int_spin(1000, 8000, " px"),
            "Long-edge pixel size of the stitched mosaic — affects display "
            "detail and detection accuracy (and memory).")
        root.addWidget(ras_box)

        # ── Stitch alignment (registration) ─────────────────────────
        reg_box = QGroupBox("Stitch alignment")
        reg_form = QFormLayout(reg_box)
        self._add_combo_row(
            reg_form, "reg_method", "Registration method",
            REG_METHOD_OPTIONS,
            "How overlapping tiles are aligned after the scan. "
            "Fourier-Mellin recovers rotation + scale + shift (most robust); "
            "Phase correlation recovers shift only; Off does no auto-alignment "
            "— the tiles are placed by the (accurate) stage and you correct by "
            "hand with the manual-align sliders. Use Off as a backup when the "
            "auto registration misbehaves.")
        self._add_check_row(
            reg_form, "register", "Global alignment",
            "Trust the (accurate) stage for relative tile placement, then "
            "correct a single SYSTEMATIC stage↔image offset for the WHOLE "
            "mosaic: measure each textured overlap by phase correlation and "
            "apply ONE robust (median) shift to all tiles. Featureless tiles "
            "(between wells / no edge) contribute nothing and are unaffected. "
            "No tile is ever shifted on its own.")
        self._add_check_row(
            reg_form, "regularize_intensity", "Even out illumination",
            "After the scan, flatten the illumination across the whole "
            "mosaic.\n\n"
            "Two effects, both of which draw the tile grid onto a large "
            "mosaic: VIGNETTING (every frame is darker at its edges, so each "
            "tile boundary becomes a soft dark seam) and TILE LEVEL (lamp "
            "drift or auto-exposure leaves neighbouring tiles at different "
            "brightness).\n\n"
            "The correction is measured from the scan itself — no extra "
            "frames and no reference slide. Specimen detail moves from tile "
            "to tile so it averages out, while the illumination pattern is "
            "fixed in the frame and survives. A scan with too few tiles for a "
            "reliable estimate is left untouched.\n\n"
            "Turn this OFF if you need the raw, uncorrected pixel values "
            "(e.g. quantitative intensity comparisons between wells).")
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

        # ── Where orientation and scale live now ────────────────────
        # The old "Tile orientation (camera mount)" combo and the
        # "Camera FOV width" / "Grid spacing" overrides lived here. They are gone
        # (see the note by MOSAIC_SCAN_DEFAULTS): orientation and µm/px are
        # MEASURED, in one place, and this dialog must not offer a second way to
        # set them.
        note = QLabel(
            "Camera orientation (mirror / axis direction / rotation) and pixel "
            "size are measured in Hardware Setup → Cameras → Mosaic & Camera "
            "Calibration, and apply to every mosaic and live view.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(note)

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

    # ── Full-resolution note (v7.14) ───────────────────────────────

    def _refresh_full_res_note(self) -> None:
        """State the canvas cost of the full-resolution toggle, live.

        Stitch memory grows with the SQUARE of the canvas long edge, so a
        toggle that silently raised it would be a quiet way to thrash the
        machine that is also driving the stage.
        """
        lbl = getattr(self, "_full_res_note", None)
        if lbl is None:
            return
        try:
            on = self._checks["full_res_scan"].isChecked()
            target = int(self._spins["target_px"].value())
        except (KeyError, AttributeError):      # pragma: no cover
            return
        if not on:
            lbl.setText("")
            return
        from SupportClasses.CaptureResolution import (
            CANVAS_WARN_MB, estimated_canvas_mb)
        mb = estimated_canvas_mb(target)
        txt = (f"Tiles are resized into the {target} px mosaic canvas "
               f"(~{mb:.0f} MB while stitching). Raise “Mosaic resolution” "
               f"to keep the extra sensor pixels — memory grows with its "
               f"square.")
        if mb >= CANVAS_WARN_MB:
            txt = "⚠ " + txt
            lbl.setStyleSheet(f"color: {COLORS.get('peach', '#fab387')};")
        else:
            lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
        lbl.setText(txt)

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
        self._refresh_full_res_note()

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
