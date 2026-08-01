"""
mosaic_calibration_confirm_dialog.py — the final check before a mosaic
calibration is committed.

v7.5.x. Operator: *"After settings are applied, there should be a final check
with the new settings for confirmation. We also need the ability to rotate the
entire mosaic to ensure the output of the mosaic is the correct way up and
down."*

Shows what the stage-motion measurement actually found — in plain terms, not raw
numbers only — lets the operator set the tile overlap and the whole-mosaic output
rotation, and offers a small **test mosaic built through the shared build path**
so what is confirmed is what every later scan will do.

**Nothing is persisted by this dialog.** It returns a candidate
``MosaicCalibration``; the caller commits only on Accept. That is deliberate: the
previous flow wrote to the stores the moment a measurement finished (and the
mirror checkbox persisted on the very first click, even on Cancel), so a bad
measurement reached every workflow before anyone could look at it.
"""

from __future__ import annotations

import dataclasses
import logging

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox, QLabel,
    QSpinBox, QComboBox, QPushButton, QDialogButtonBox, QMessageBox,
)

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size

logger = logging.getLogger(__name__)

# Output rotation choices: stored value -> display label.
OUTPUT_ROTATION_OPTIONS = [
    (0, "0° — as measured"),
    (90, "90° clockwise"),
    (180, "180° — upside down"),
    (270, "270° (90° counter-clockwise)"),
]


def _axis_phrase(rotation_deg: float, flip_x: bool, flip_y: bool) -> str:
    """Describe the measured camera→stage relationship in operator language.

    The numbers alone ("rotation 180°, flip Y") do not tell the operator whether
    the result matches what they can see on the bench; naming where stage +X ends
    up on screen does.
    """
    rot = round(float(rotation_deg)) % 360
    quad = {0: ("right", "down"), 90: ("down", "left"),
            180: ("left", "up"), 270: ("up", "right")}
    nearest = min(quad, key=lambda q: min(abs(rot - q), 360 - abs(rot - q)))
    x_dir, y_dir = quad[nearest]
    if flip_x:
        x_dir = {"right": "left", "left": "right"}.get(x_dir, x_dir)
    if flip_y:
        y_dir = {"down": "up", "up": "down"}.get(y_dir, y_dir)
    off = min(abs(rot - nearest), 360 - abs(rot - nearest))
    extra = f" (measured {rot}°, {off}° off square)" if off > 2 else ""
    return (f"Moving the stage +X moves the image {x_dir}; "
            f"+Y moves it {y_dir}{extra}.")


class MosaicCalibrationConfirmDialog(QDialog):
    """Review a candidate calibration, set overlap + output rotation, confirm."""

    def __init__(self, candidate, *, tile_count_getter=None,
                 test_mosaic_runner=None, warnings=(), parent=None):
        """``candidate`` is a ``MosaicCalibration``.

        ``tile_count_getter(cal) -> int | None`` prices the overlap choice.
        ``test_mosaic_runner(cal) -> None`` builds the confirmation mosaic (it
        must not persist anything).
        """
        super().__init__(parent)
        self.setWindowTitle("Confirm camera & mosaic calibration")
        self.setModal(True)
        self.setMinimumWidth(s(560))
        self._candidate = candidate
        self._tile_count_getter = tile_count_getter
        self._test_mosaic_runner = test_mosaic_runner
        self._warnings = list(warnings)
        self._tested = False
        self._build_ui()
        self._refresh()

    # ── Result ────────────────────────────────────────────────────

    def result_calibration(self):
        """The candidate with the operator's overlap + output rotation applied."""
        return dataclasses.replace(
            self._candidate,
            overlap_frac=self._spin_overlap.value() / 100.0,
            output_rotation_deg=float(self._cmb_rot.currentData() or 0),
        )

    # ── Build ─────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(s(14), s(14), s(14), s(14))
        root.setSpacing(s(10))

        intro = QLabel(
            "These values will be used by every mosaic (full plate, rosette, "
            "fluorescence) and by every microscope live view. Check them, then "
            "build a test mosaic to confirm before saving.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(intro)

        # ── What was measured ───────────────────────────────────
        meas = QGroupBox("Measured from stage motion")
        mlay = QFormLayout(meas)
        c = self._candidate
        self._lbl_axes = QLabel(
            _axis_phrase(c.rotation_deg, c.flip_x, c.flip_y))
        self._lbl_axes.setWordWrap(True)
        mlay.addRow(QLabel("Axis directions"), self._lbl_axes)
        mlay.addRow(QLabel("Mirrored"),
                    QLabel("yes" if (c.flip_x or c.flip_y) else "no"))
        cr = (f"{c.calib_resolution[0]}×{c.calib_resolution[1]}"
              if c.calib_resolution else "unknown")
        mlay.addRow(
            QLabel("Pixel size"),
            QLabel(f"{c.um_per_px:.4f} µm/px  (measured at {cr})"))
        fw, fh = c.fov_um
        mlay.addRow(QLabel("Field of view"),
                    QLabel(f"{fw:.0f} × {fh:.0f} µm"))
        root.addWidget(meas)

        # ── Operator settings ───────────────────────────────────
        opts = QGroupBox("Mosaic settings")
        olay = QFormLayout(opts)
        self._spin_overlap = QSpinBox()
        self._spin_overlap.setRange(5, 50)
        self._spin_overlap.setSuffix(" %")
        self._spin_overlap.setValue(int(round(c.overlap_frac * 100)))
        self._spin_overlap.setToolTip(
            "How much neighbouring tiles overlap. Raster step is "
            "FOV x (1 - overlap), so too little overlap opens gaps if the pixel "
            "size is slightly off and leaves too little shared texture for the "
            "stitcher to align on. 25% is recommended.")
        self._spin_overlap.valueChanged.connect(self._refresh)
        olay.addRow(QLabel("Tile overlap"), self._spin_overlap)

        self._lbl_tiles = QLabel("")
        self._lbl_tiles.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        olay.addRow(QLabel(""), self._lbl_tiles)

        self._cmb_rot = QComboBox()
        for value, label in OUTPUT_ROTATION_OPTIONS:
            self._cmb_rot.addItem(label, value)
        idx = self._cmb_rot.findData(int(c.output_rotation_deg or 0))
        self._cmb_rot.setCurrentIndex(idx if idx >= 0 else 0)
        self._cmb_rot.setToolTip(
            "Rotate the whole finished mosaic for VIEWING, so it appears the "
            "correct way up. This does not change how tiles are captured or "
            "placed — well positions and stage motion are unaffected.")
        olay.addRow(QLabel("Mosaic output rotation"), self._cmb_rot)
        root.addWidget(opts)

        # ── Advisories ──────────────────────────────────────────
        self._lbl_warn = QLabel("")
        self._lbl_warn.setWordWrap(True)
        self._lbl_warn.setStyleSheet(f"color: {COLORS['yellow']};")
        root.addWidget(self._lbl_warn)

        # ── Test mosaic + buttons ───────────────────────────────
        btn_row = QHBoxLayout()
        self._btn_test = QPushButton("Build test mosaic…")
        self._btn_test.setObjectName("accentBtn")
        self._btn_test.setToolTip(
            "Build a small mosaic here, with these settings, through the same "
            "code the real scan uses. Moves XY only at the current height — the "
            "needle never descends.")
        self._btn_test.clicked.connect(self._on_test)
        self._btn_test.setEnabled(self._test_mosaic_runner is not None)
        btn_row.addWidget(self._btn_test)
        btn_row.addStretch(1)
        self._bb = QDialogButtonBox()
        self._btn_ok = self._bb.addButton(
            "Save calibration", QDialogButtonBox.ButtonRole.AcceptRole)
        self._btn_ok.setObjectName("accentBtn")
        self._bb.addButton(QDialogButtonBox.StandardButton.Cancel)
        self._bb.accepted.connect(self.accept)
        self._bb.rejected.connect(self.reject)
        btn_row.addWidget(self._bb)
        root.addLayout(btn_row)

        note = QLabel(
            "Cancel discards this measurement completely — the previous "
            "calibration is left untouched.")
        note.setWordWrap(True)
        note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;")
        root.addWidget(note)

    # ── Behaviour ─────────────────────────────────────────────────

    def _refresh(self):
        cal = self.result_calibration()
        sx, sy = cal.spacing_um
        txt = f"Raster step {sx:.0f} × {sy:.0f} µm"
        n = None
        if self._tile_count_getter is not None:
            try:
                n = self._tile_count_getter(cal)
            except Exception as exc:
                logger.debug(f"tile-count preview failed: {exc}")
        if n:
            txt += f" — about {n} tiles for a full plate scan"
        self._lbl_tiles.setText(txt)

        msgs = list(self._warnings)
        try:
            from SupportClasses.MosaicCalibration import warnings_for
            msgs = warnings_for(cal) or []
        except Exception:
            pass
        self._lbl_warn.setText("\n".join(f"- {m}" for m in msgs))
        self._lbl_warn.setVisible(bool(msgs))

    def _on_test(self):
        if self._test_mosaic_runner is None:
            return
        try:
            self._test_mosaic_runner(self.result_calibration())
            self._tested = True
        except Exception as exc:
            logger.exception("test mosaic failed")
            QMessageBox.warning(
                self, "Test mosaic", f"The test mosaic could not be built:\n{exc}")

    def accept(self):
        """Confirm — but nudge once if the test mosaic was never built.

        Not a hard gate: the stage may be somewhere the operator does not want to
        raster, and refusing outright would strand them. It IS the whole point of
        this dialog though, so it is asked for explicitly.
        """
        if not self._tested and self._test_mosaic_runner is not None:
            ans = QMessageBox.question(
                self, "Save without checking?",
                "You haven't built a test mosaic with these settings yet.\n\n"
                "Building one is the only way to see that the tiles line up and "
                "the output is the right way up before this calibration is used "
                "by every mosaic and live view.\n\nSave anyway?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)
            if ans != QMessageBox.StandardButton.Yes:
                return
        super().accept()
