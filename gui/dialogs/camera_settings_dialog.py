"""
camera_settings_dialog.py — Pop-out quick camera (hardware) settings.

v7.5.x: A modeless dialog for changing a camera's *firmware* settings via its
SDK/driver — exposure (with auto-exposure toggle), analog gain, gamma,
brightness, contrast — and the *device* capture resolution. These are distinct
from the software post-processing "Image Correction" (CameraWidget brightness/
contrast/gamma applied to displayed frames); this dialog talks to the camera.

Every value shown is read back FROM the device, and a "Read from camera"
button re-reads + logs a labelled block to the terminal so the operator can
confirm the settings come from the camera and not a software default. Changes
apply live and persist per device identity (CameraCalibrationStore).

Opened from the gear button on a microscope/controllable camera feed
(``CameraFeedView``), or anywhere a CameraManager + slot index is available.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QSlider,
    QCheckBox, QComboBox, QDoubleSpinBox, QPushButton, QGroupBox,
    QPlainTextEdit, QWidget, QSizePolicy,
)

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size

logger = logging.getLogger(__name__)

try:
    from gui.widgets.camera_feed_view import CameraFeedView
    _FEED_AVAILABLE = True
except Exception:  # pragma: no cover
    CameraFeedView = None
    _FEED_AVAILABLE = False


class CameraSettingsDialog(QDialog):
    """Modeless hardware-settings panel for one camera slot."""

    # v7.5.x: (cam_idx, width, height) — emitted when the DEVICE capture
    # resolution actually changes, so the "Microscope Camera Setup" block can
    # update its `active_resolution` (the block was pulling a stale resolution).
    resolution_applied = Signal(int, int, int)

    def __init__(self, manager, cam_idx: int, identity_getter=None,
                 parent=None):
        super().__init__(parent)
        self._mgr = manager
        self._cam_idx = cam_idx
        # callable -> (identity_key, name) | None, for persistence
        self._identity_getter = identity_getter
        self._loading = False           # guard: suppress live-apply while loading
        self._rows: dict = {}           # control name -> widget(s)

        self.setWindowTitle(f"Camera Controls — Cam {cam_idx + 1}")
        self.setModal(False)
        self.setMinimumWidth(s(420))
        self._build_ui()
        self.reload()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(s(10))
        root.setContentsMargins(s(14), s(14), s(14), s(14))

        self._header = QLabel("…")
        self._header.setWordWrap(True)
        self._header.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {scaled_font_size(9)}pt;")
        root.addWidget(self._header)

        # Optional small live preview so changes are visible immediately.
        if _FEED_AVAILABLE and self._mgr is not None:
            try:
                self._preview = CameraFeedView(
                    camera_manager=self._mgr, cam_idx=self._cam_idx,
                    show_crosshair=False, label="", enable_settings=False,
                    parent=self)
                self._preview.setMinimumHeight(s(150))
                self._preview.setMaximumHeight(s(220))
                root.addWidget(self._preview)
            except Exception as exc:
                logger.debug(f"settings dialog preview unavailable: {exc}")

        controls = QGroupBox("Camera (hardware) settings")
        grid = QGridLayout(controls)
        grid.setHorizontalSpacing(s(10))
        grid.setVerticalSpacing(s(8))
        grid.setColumnStretch(1, 1)
        self._grid = grid
        self._grid_row = 0
        root.addWidget(controls)

        # Resolution row (populated in reload()).
        self._res_label = QLabel("Resolution")
        self._res_combo = QComboBox()
        self._res_combo.currentIndexChanged.connect(self._on_resolution_changed)
        self._add_grid_row(self._res_label, self._res_combo)

        # Auto-exposure
        self._auto_chk = QCheckBox("Auto-exposure")
        self._auto_chk.toggled.connect(self._on_auto_toggled)
        self._add_grid_row(QLabel(""), self._auto_chk)

        # Exposure (ms) — only meaningful with auto off.
        self._exp_label = QLabel("Exposure (ms)")
        self._exp_spin = QDoubleSpinBox()
        self._exp_spin.setDecimals(3)
        self._exp_spin.setRange(0.001, 100000.0)
        self._exp_spin.setKeyboardTracking(False)
        self._exp_spin.valueChanged.connect(self._on_exposure_changed)
        self._add_grid_row(self._exp_label, self._exp_spin)

        # Gain (%)
        self._gain_label, self._gain_sld, self._gain_val = self._make_slider(
            "Gain (%)", 0, 1000, 100, self._on_gain_changed)

        # Andor (Zyla) display scaling — the mono-16 sensor is normalized to
        # 8-bit for display; auto = per-frame percentile scaling (the image
        # "auto-adjusts" to the scene), manual = fixed black/white levels.
        self._ascale_chk = QCheckBox("Auto display scaling (per-frame)")
        self._ascale_chk.setToolTip(
            "The Zyla's 16-bit image is normalized for display. Checked: each "
            "frame is auto-scaled to its own 1–99 percentile, so brightness "
            "follows the scene. Unchecked: a fixed black/white level mapping — "
            "the current look is frozen and the sliders below take over.")
        self._ascale_chk.toggled.connect(self._on_ascale_toggled)
        self._add_grid_row(QLabel(""), self._ascale_chk)
        self._blk_label, self._blk_sld, self._blk_val = self._make_slider(
            "Black level", 0, 65535, 0, self._on_scale_lo_changed)
        self._blk_label.setToolTip(
            "Raw sensor counts shown as black (display 0). Manual mode only.")
        self._wht_label, self._wht_sld, self._wht_val = self._make_slider(
            "White level", 0, 65535, 65535, self._on_scale_hi_changed)
        self._wht_label.setToolTip(
            "Raw sensor counts shown as white (display 255). Manual mode only.")

        # Gamma / Brightness / Contrast sliders
        self._gamma_label, self._gamma_sld, self._gamma_val = self._make_slider(
            "Gamma", 20, 180, 100, self._on_gamma_changed)
        self._bri_label, self._bri_sld, self._bri_val = self._make_slider(
            "Brightness", -64, 64, 0, self._on_brightness_changed)
        self._con_label, self._con_sld, self._con_val = self._make_slider(
            "Contrast", -100, 100, 0, self._on_contrast_changed)

        # Action row
        actions = QHBoxLayout()
        self._read_btn = QPushButton("⟳ Read from camera")
        self._read_btn.setToolTip(
            "Re-read every setting from the camera and log it to the terminal.")
        self._read_btn.clicked.connect(self._on_read_clicked)
        actions.addWidget(self._read_btn)
        self._defaults_btn = QPushButton("Defaults")
        self._defaults_btn.setToolTip("Reset hardware controls to camera defaults.")
        self._defaults_btn.clicked.connect(self._on_defaults_clicked)
        actions.addWidget(self._defaults_btn)
        actions.addStretch()
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        actions.addWidget(close_btn)
        root.addLayout(actions)

        # Readout (terminal mirror)
        self._readout = QPlainTextEdit()
        self._readout.setReadOnly(True)
        self._readout.setMaximumHeight(s(150))
        self._readout.setStyleSheet(
            "font-family: Consolas, monospace; "
            f"font-size: {scaled_font_size(8)}pt;")
        root.addWidget(self._readout)

    def _add_grid_row(self, label: QWidget, field: QWidget):
        self._grid.addWidget(label, self._grid_row, 0)
        self._grid.addWidget(field, self._grid_row, 1, 1, 2)
        self._grid_row += 1

    def _make_slider(self, name, lo, hi, init, handler):
        label = QLabel(name)
        sld = QSlider(Qt.Horizontal)
        sld.setRange(lo, hi)
        sld.setValue(init)
        val = QLabel(str(init))
        val.setMinimumWidth(s(40))
        val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        sld.valueChanged.connect(lambda v, l=val: l.setText(str(v)))
        sld.valueChanged.connect(handler)
        self._grid.addWidget(label, self._grid_row, 0)
        self._grid.addWidget(sld, self._grid_row, 1)
        self._grid.addWidget(val, self._grid_row, 2)
        self._grid_row += 1
        return label, sld, val

    # ── Load state from the camera ────────────────────────────────

    def reload(self):
        """Read capabilities + current settings from the device and populate."""
        self._loading = True
        try:
            caps = self._mgr.hardware_capabilities(self._cam_idx) if self._mgr else {}
            st = self._mgr.get_hw_settings(self._cam_idx) if self._mgr else {}
            src = caps.get("source", st.get("source", "none"))
            name = caps.get("device_name") or st.get("device_id") or ""
            controllable = bool(caps.get("controllable"))
            self._header.setText(
                f"<b>Source:</b> {src}"
                + (f" &nbsp; <b>Device:</b> {name}" if name else "")
                + ("" if controllable
                   else "  —  no controllable backend (start the camera)"))

            ctrls = caps.get("controls", {})
            # Resolution
            resolutions = st.get("resolutions") or []
            cur_res = st.get("resolution")
            self._res_combo.blockSignals(True)
            self._res_combo.clear()
            for (w, h) in resolutions:
                self._res_combo.addItem(f"{w} × {h}", (w, h))
            if cur_res and tuple(cur_res) in [tuple(r) for r in resolutions]:
                self._res_combo.setCurrentIndex(
                    [tuple(r) for r in resolutions].index(tuple(cur_res)))
            self._res_combo.blockSignals(False)
            res_ok = caps.get("resolution") and bool(resolutions)
            self._res_label.setVisible(res_ok)
            self._res_combo.setVisible(res_ok)

            # Auto-exposure — only a genuine True checks the box; None
            # (unknown/unsupported read) leaves it unchecked so manual
            # exposure/gain controls stay enabled.
            has_auto = "auto_exposure" in ctrls
            auto_on = (st.get("auto_exposure") is True)
            self._auto_chk.setVisible(has_auto)
            self._auto_chk.blockSignals(True)
            self._auto_chk.setChecked(auto_on)
            self._auto_chk.blockSignals(False)

            # Exposure (ms)
            self._setup_exposure(ctrls.get("exposure_us"), st.get("exposure_us"),
                                 auto_on, has_auto)
            # Gain
            self._setup_slider(
                "exposure_gain_pct", ctrls, st.get("exposure_gain_pct"),
                self._gain_label, self._gain_sld, self._gain_val,
                enabled=not (has_auto and auto_on))
            # Andor (Zyla) display scaling — only offered when the backend
            # reports the control (i.e. an Andor camera is live).
            has_ascale = "andor_auto_scale" in ctrls
            ascale_on = (st.get("andor_auto_scale") is True)
            self._ascale_chk.setVisible(has_ascale)
            self._ascale_chk.blockSignals(True)
            self._ascale_chk.setChecked(ascale_on)
            self._ascale_chk.blockSignals(False)
            self._setup_slider("andor_scale_lo", ctrls, st.get("andor_scale_lo"),
                               self._blk_label, self._blk_sld, self._blk_val,
                               enabled=not ascale_on)
            self._setup_slider("andor_scale_hi", ctrls, st.get("andor_scale_hi"),
                               self._wht_label, self._wht_sld, self._wht_val,
                               enabled=not ascale_on)
            # Gamma / brightness / contrast
            self._setup_slider("gamma", ctrls, st.get("gamma"),
                               self._gamma_label, self._gamma_sld, self._gamma_val)
            self._setup_slider("brightness", ctrls, st.get("brightness"),
                               self._bri_label, self._bri_sld, self._bri_val)
            self._setup_slider("contrast", ctrls, st.get("contrast"),
                               self._con_label, self._con_sld, self._con_val)

            self._read_btn.setEnabled(controllable)
            self._defaults_btn.setEnabled(controllable)
            self._render_readout(st, src, name)
        finally:
            self._loading = False

    def _setup_exposure(self, spec, value, auto_on, has_auto):
        supported = spec is not None
        self._exp_label.setVisible(supported)
        self._exp_spin.setVisible(supported)
        if not supported:
            return
        rng = spec.get("range")
        self._exp_spin.blockSignals(True)
        if rng:  # (min_us, max_us, def_us)
            self._exp_spin.setRange(max(rng[0] / 1000.0, 0.001), rng[1] / 1000.0)
        if value is not None:
            self._exp_spin.setValue(float(value) / 1000.0)
        self._exp_spin.blockSignals(False)
        # No authoritative reading -> disable (don't let the operator "apply" a
        # fabricated default), and never enable while auto-exposure is on.
        self._exp_spin.setEnabled(value is not None and not (has_auto and auto_on))

    def _setup_slider(self, key, ctrls, value, label, sld, val_lbl,
                      enabled=True):
        spec = ctrls.get(key)
        supported = spec is not None
        label.setVisible(supported)
        sld.setVisible(supported)
        val_lbl.setVisible(supported)
        if not supported:
            return
        rng = spec.get("range")
        sld.blockSignals(True)
        if rng:
            sld.setRange(int(rng[0]), int(rng[1]))
        if value is not None:
            iv = int(round(float(value)))
            iv = max(sld.minimum(), min(sld.maximum(), iv))
            sld.setValue(iv)
            val_lbl.setText(str(iv))
        else:
            # Supported control but no authoritative reading from the device —
            # show a placeholder and disable so the slider's default value is
            # never mistaken for the camera's value, and can't be "applied".
            val_lbl.setText("—")
            enabled = False
        sld.blockSignals(False)
        sld.setEnabled(enabled)

    # ── Live-apply handlers (each persists) ───────────────────────

    def _on_resolution_changed(self):
        if self._loading or self._mgr is None:
            return
        data = self._res_combo.currentData()
        if not data:
            return
        actual = self._mgr.set_capture_resolution(self._cam_idx, data[0], data[1])
        logger.info(f"Cam {self._cam_idx + 1}: resolution set -> {actual}")
        # Broadcast the applied resolution so the camera-setup block updates its
        # active_resolution (and any µm/px consumer rescales) — ground truth.
        try:
            if actual and len(actual) >= 2 and actual[0] and actual[1]:
                self.resolution_applied.emit(
                    int(self._cam_idx), int(actual[0]), int(actual[1]))
            else:
                self.resolution_applied.emit(
                    int(self._cam_idx), int(data[0]), int(data[1]))
        except Exception:
            pass
        self._persist()
        # Re-read (eSize / exposure ranges can shift with resolution).
        self.reload()

    def _on_auto_toggled(self, checked):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_auto_exposure(self._cam_idx, checked)
        self._exp_spin.setEnabled(not checked)
        self._gain_sld.setEnabled(not checked)
        self._persist()

    def _on_exposure_changed(self, ms):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_exposure_us(self._cam_idx, int(round(ms * 1000.0)))
        self._persist()

    def _on_gain_changed(self, v):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_exposure_gain(self._cam_idx, int(v))
        self._persist()

    def _on_ascale_toggled(self, checked):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_andor_auto_scale(self._cam_idx, checked)
        self._persist()
        # Turning auto OFF freezes the current auto levels into the manual
        # black/white — re-read so the sliders show (and enable at) them.
        self.reload()

    def _on_scale_lo_changed(self, v):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_andor_scale_lo(self._cam_idx, int(v))
        self._persist()

    def _on_scale_hi_changed(self, v):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_andor_scale_hi(self._cam_idx, int(v))
        self._persist()

    def _on_gamma_changed(self, v):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_gamma(self._cam_idx, int(v))
        self._persist()

    def _on_brightness_changed(self, v):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_brightness(self._cam_idx, int(v))
        self._persist()

    def _on_contrast_changed(self, v):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_contrast(self._cam_idx, int(v))
        self._persist()

    def _on_defaults_clicked(self):
        """Reset controls to camera defaults (range[2]) and re-read."""
        if self._mgr is None:
            return
        caps = self._mgr.hardware_capabilities(self._cam_idx)
        ctrls = caps.get("controls", {})
        # auto-exposure on is the camera's default behavior.
        if "auto_exposure" in ctrls:
            self._mgr.set_hw_auto_exposure(self._cam_idx, True)
        # Andor: per-frame display auto-scale is the historical default.
        if "andor_auto_scale" in ctrls:
            self._mgr.set_hw_andor_auto_scale(self._cam_idx, True)
        for key, setter in (
            ("gamma", self._mgr.set_hw_gamma),
            ("brightness", self._mgr.set_hw_brightness),
            ("contrast", self._mgr.set_hw_contrast),
        ):
            rng = (ctrls.get(key) or {}).get("range")
            if rng and len(rng) >= 3:
                setter(self._cam_idx, rng[2])
        self._persist()
        self.reload()

    def _on_read_clicked(self):
        """Re-read from camera, log to terminal, refresh widgets + readout."""
        if self._mgr is None:
            return
        self._mgr.log_hw_settings(self._cam_idx, prefix="[Read button] ")
        self.reload()

    # ── Persistence ───────────────────────────────────────────────

    def _persist(self):
        if self._identity_getter is None or self._mgr is None:
            return
        try:
            identity = self._identity_getter()
        except Exception:
            identity = None
        if not identity:
            return
        st = self._mgr.get_hw_settings(self._cam_idx)
        controls = {
            "auto_exposure": st.get("auto_exposure"),
            "exposure_us": st.get("exposure_us"),
            "exposure_gain_pct": st.get("exposure_gain_pct"),
            "gamma": st.get("gamma"),
            "brightness": st.get("brightness"),
            "contrast": st.get("contrast"),
            "andor_auto_scale": st.get("andor_auto_scale"),
            "andor_scale_lo": st.get("andor_scale_lo"),
            "andor_scale_hi": st.get("andor_scale_hi"),
            "resolution": list(st["resolution"]) if st.get("resolution") else None,
        }
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            get_store().set_hw_controls(identity[0], controls, name=identity[1])
        except Exception as exc:
            logger.debug(f"persist hw controls failed: {exc}")

    # ── Readout ───────────────────────────────────────────────────

    def _render_readout(self, st, src, name):
        lines = [f"source = {src}"]
        if name:
            lines.append(f"device = {name}")
        if src in ("toupcam", "opencv", "andor"):
            lines.append(f"resolution    = {st.get('resolution')}"
                         + (f"  (eSize {st.get('eSize')})"
                            if st.get("eSize") is not None else ""))
            lines.append(f"auto-exposure = {st.get('auto_exposure')}")
            lines.append(f"exposure      = {st.get('exposure_us')} us"
                         + (f"   range {st.get('exposure_range_us')}"
                            if st.get("exposure_range_us") else ""))
            lines.append(f"gain          = {st.get('exposure_gain_pct')} %")
            if src == "andor":
                mode = ("auto (per-frame)" if st.get("andor_auto_scale")
                        else "manual")
                lines.append(f"display scale = {mode}   levels "
                             f"{st.get('andor_scale_lo')}.."
                             f"{st.get('andor_scale_hi')}")
            lines.append(f"gamma         = {st.get('gamma')}")
            lines.append(f"brightness    = {st.get('brightness')}")
            lines.append(f"contrast      = {st.get('contrast')}")
        else:
            lines.append("(no controllable camera — start the camera first)")
        self._readout.setPlainText("\n".join(str(x) for x in lines))
