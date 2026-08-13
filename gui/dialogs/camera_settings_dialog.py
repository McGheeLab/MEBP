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

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QSlider, QCheckBox, QComboBox, QDoubleSpinBox, QPushButton, QGroupBox,
    QPlainTextEdit, QWidget, QSizePolicy,
)

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size
from gui.widgets.hw_controls_snapshot import hw_controls_snapshot

logger = logging.getLogger(__name__)

try:
    from gui.widgets.camera_feed_view import CameraFeedView
    _FEED_AVAILABLE = True
except Exception:  # pragma: no cover
    CameraFeedView = None
    _FEED_AVAILABLE = False

try:
    from gui.widgets.raw_histogram_widget import RawHistogramWidget
    _HIST_AVAILABLE = True
except Exception:  # pragma: no cover
    RawHistogramWidget = None
    _HIST_AVAILABLE = False

# v7.13 — Andor sensor-quality features surfaced in this dialog. Keys match
# ANDOR_SENSOR_FEATURES in gui/widgets/andor_backend.py (and hw_controls).
_SENSOR_BOOL_ROWS = (
    ("andor_sensor_cooling", "Sensor cooling",
     "Cool the sCMOS sensor (lower dark current). Leave on for fluorescence."),
    ("andor_noise_filter", "Spurious noise filter",
     "On-camera single-pixel noise filter. CAUTION: can also suppress real "
     "sub-resolution signal (fluorescent puncta) — verify on your sample."),
    ("andor_blemish_correction", "Blemish correction",
     "On-camera static hot/dark pixel correction."),
)
_SENSOR_ENUM_ROWS = (
    ("andor_readout_rate", "Readout rate",
     "Pixel readout rate. The slower rate reads with less noise — prefer it "
     "for dim fluorescence."),
    ("andor_gain_mode", "Gain mode",
     "Pre-amp gain mode. The 16-bit low-noise / high-well-capacity mode is "
     "the best default for quantitative fluorescence."),
)


class CameraSettingsDialog(QDialog):
    """Modeless hardware-settings panel for one camera slot."""

    # v7.5.x: (cam_idx, width, height) — emitted when the DEVICE capture
    # resolution actually changes, so the "Microscope Camera Setup" block can
    # update its `active_resolution` (the block was pulling a stale resolution).
    resolution_applied = Signal(int, int, int)

    # v7.13.x — signal-optimizer worker → GUI thread (achieved exposure_us or
    # None, human-readable note).
    _optimize_done = Signal(object, str)

    def __init__(self, manager, cam_idx: int, identity_getter=None,
                 parent=None):
        super().__init__(parent)
        self._mgr = manager
        self._cam_idx = cam_idx
        # callable -> (identity_key, name) | None, for persistence
        self._identity_getter = identity_getter
        self._loading = False           # guard: suppress live-apply while loading
        self._rows: dict = {}           # control name -> widget(s)
        self._has_raw_stats = False     # v7.13: backend retains raw stats?
        self._optimizing = False        # v7.13.x: signal optimizer running?
        self._optimize_done.connect(self._on_optimize_done)
        # v7.17.1 — Debounced persistence. Every slider here is wired on
        # ``valueChanged`` (see _make_slider), so a single drag used to write
        # the whole calibration store once per tick: a JSON serialise + atomic
        # file replace on the GUI thread, at the rate the mouse moves. A gamma
        # drag from 84 to 128 was measured writing the file ~50 times in 9
        # seconds. Same failure shape as the v7.16 crop-offset spins, and the
        # same remedy: the LIVE PUSH to the camera stays immediate (aiming a
        # control is a visual task, the preview must follow the slider) while
        # only the WRITE waits for the drag to settle.
        self._persist_pending = False
        self._persist_timer = QTimer(self)
        self._persist_timer.setSingleShot(True)
        self._persist_timer.setInterval(400)
        self._persist_timer.timeout.connect(self._flush_persist)
        # hide/close cover the ordinary ways this dialog goes away; aboutToQuit
        # covers quitting the app with it still open, which neither reports.
        try:
            _qapp = QApplication.instance()
            if _qapp is not None:
                _qapp.aboutToQuit.connect(self._flush_persist)
        except Exception:
            pass

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

        # v7.13 — Andor sensor-quality features (shown only when the live
        # camera probed them; enum combos are populated from the camera's own
        # runtime-enumerated values in reload()).
        self._sensor_checks: dict = {}
        for key, label, tip in _SENSOR_BOOL_ROWS:
            chk = QCheckBox(label)
            chk.setToolTip(tip)
            chk.toggled.connect(
                lambda checked, k=key: self._on_sensor_bool(k, checked))
            self._add_grid_row(QLabel(""), chk)
            self._sensor_checks[key] = chk
        self._sensor_combos: dict = {}
        for key, label, tip in _SENSOR_ENUM_ROWS:
            lbl = QLabel(label)
            lbl.setToolTip(tip)
            combo = QComboBox()
            combo.setToolTip(tip)
            combo.currentIndexChanged.connect(
                lambda _i, k=key: self._on_sensor_enum(k))
            self._add_grid_row(lbl, combo)
            self._sensor_combos[key] = (lbl, combo)

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

        # v7.13 — Signal (raw sensor) group: live histogram + clipping readout
        # computed from RAW counts (the display auto-scale hides clipping, so
        # this — not the image — is the exposure instrument). Shown only when
        # the backend retains raw statistics; refreshed on a timer while the
        # dialog is visible.
        self._signal_group = QGroupBox("Signal (raw sensor)")
        sig_lay = QVBoxLayout(self._signal_group)
        sig_lay.setSpacing(s(6))
        if _HIST_AVAILABLE:
            self._hist_widget = RawHistogramWidget(self._signal_group)
            sig_lay.addWidget(self._hist_widget)
        else:  # pragma: no cover
            self._hist_widget = None
        self._signal_label = QLabel("—")
        self._signal_label.setStyleSheet(
            "font-family: Consolas, monospace; "
            f"font-size: {scaled_font_size(8)}pt;")
        sig_lay.addWidget(self._signal_label)
        # v7.13.x — one-shot signal optimizer: auto-expose to the target
        # histogram, then freeze the display levels (nothing per-frame after).
        opt_row = QHBoxLayout()
        self._opt_btn = QPushButton("⚡ Optimize signal")
        self._opt_btn.setToolTip(
            "One-shot: adjust the exposure until the raw histogram's P99.9 "
            "sits at ~70% of full scale with no clipping, then FREEZE the "
            "display black/white levels — nothing changes per frame "
            "afterwards.")
        self._opt_btn.clicked.connect(self._on_optimize_clicked)
        opt_row.addWidget(self._opt_btn)
        opt_row.addStretch()
        sig_lay.addLayout(opt_row)
        self._opt_note = QLabel("")
        self._opt_note.setWordWrap(True)
        self._opt_note.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {scaled_font_size(8)}pt;")
        sig_lay.addWidget(self._opt_note)
        self._signal_group.setVisible(False)
        root.addWidget(self._signal_group)
        self._stats_timer = QTimer(self)
        self._stats_timer.setInterval(500)
        self._stats_timer.timeout.connect(self._refresh_raw_stats)

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
            # bool(): setVisible rejects None, and a caps dict that simply
            # omits "resolution" would otherwise raise inside reload().
            res_ok = bool(caps.get("resolution")) and bool(resolutions)
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
            # v7.13 — Andor sensor-quality features.
            for key, chk in self._sensor_checks.items():
                self._setup_checkbox(key, ctrls, st.get(key), chk)
            for key, (lbl, combo) in self._sensor_combos.items():
                self._setup_combo(key, ctrls, st.get(key), lbl, combo)
            # Signal (raw sensor) section — offered when the backend retains
            # raw statistics; the timer only runs while the dialog is shown.
            self._has_raw_stats = "andor_raw_stats" in ctrls
            self._signal_group.setVisible(self._has_raw_stats)
            if self._has_raw_stats:
                self._refresh_raw_stats()
                # The optimizer drives exposure — needs both raw stats and an
                # exposure control (all mono scientific backends have both).
                self._opt_btn.setEnabled(
                    not self._optimizing
                    and ctrls.get("exposure_us") is not None)
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

    def _setup_checkbox(self, key, ctrls, value, chk):
        """Visible iff the key is advertised; disabled on a None readback so a
        widget default can never be mistaken for a device value."""
        supported = key in ctrls
        chk.setVisible(supported)
        if not supported:
            return
        chk.blockSignals(True)
        chk.setChecked(value is True)
        chk.blockSignals(False)
        chk.setEnabled(value is not None)

    def _setup_combo(self, key, ctrls, value, label, combo):
        """Enum combo populated from the camera's OWN runtime values."""
        spec = ctrls.get(key)
        supported = spec is not None
        label.setVisible(supported)
        combo.setVisible(supported)
        if not supported:
            return
        values = spec.get("values") or []
        combo.blockSignals(True)
        combo.clear()
        for v in values:
            combo.addItem(str(v))
        if value is not None and str(value) in values:
            combo.setCurrentIndex(values.index(str(value)))
            combo.setEnabled(True)
        else:
            # Supported but no authoritative reading — placeholder + disabled.
            combo.insertItem(0, "—")
            combo.setCurrentIndex(0)
            combo.setEnabled(False)
        combo.blockSignals(False)

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
        self._persist(now=True)
        # Re-read (eSize / exposure ranges can shift with resolution).
        self.reload()

    def _on_auto_toggled(self, checked):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_auto_exposure(self._cam_idx, checked)
        self._exp_spin.setEnabled(not checked)
        self._gain_sld.setEnabled(not checked)
        self._persist(now=True)

    def _on_exposure_changed(self, ms):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_exposure_us(self._cam_idx, int(round(ms * 1000.0)))
        # v7.13.x — re-sync the spin to the ACHIEVED exposure. The SDK may
        # clamp the request (frame-rate / readout constraints); the spin must
        # never display a value the camera isn't actually running — and the
        # persist below must record the truth, not the wish (the old path
        # silently wrote the clamped value to disk while SHOWING the request).
        try:
            got = (self._mgr.get_hw_settings(self._cam_idx) or {}).get(
                "exposure_us")
        except Exception:
            got = None
        if got is not None:
            self._exp_spin.blockSignals(True)
            self._exp_spin.setValue(float(got) / 1000.0)
            self._exp_spin.blockSignals(False)
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
        self._persist(now=True)
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

    def _on_sensor_bool(self, key, checked):
        if self._loading or self._mgr is None:
            return
        self._mgr.set_hw_andor_feature(self._cam_idx, key, bool(checked))
        self._persist(now=True)

    def _on_sensor_enum(self, key):
        if self._loading or self._mgr is None:
            return
        _lbl, combo = self._sensor_combos[key]
        value = combo.currentText()
        if not value or value == "—":
            return
        self._mgr.set_hw_andor_feature(self._cam_idx, key, value)
        self._persist(now=True)
        # Gain mode changes BitDepth (histogram clip level, exposure range);
        # readout rate can shift the exposure range — re-read either way.
        self.reload()

    def _refresh_raw_stats(self):
        """Timer tick: pull the latest raw-frame stats (lock snapshot, no SDK
        traffic on the GUI thread) into the histogram + readout line."""
        if self._mgr is None or not getattr(self, "_has_raw_stats", False):
            return
        try:
            stats = self._mgr.get_raw_frame_stats(self._cam_idx)
        except Exception:
            stats = None
        if self._hist_widget is not None:
            self._hist_widget.set_stats(stats)
        if stats is None:
            self._signal_label.setText("no raw data yet")
            return
        try:
            clipped = 100.0 * float(stats.get("clipped_frac", 0.0))
            text = (f"clipped {clipped:.2f}%   "
                    f"max {float(stats.get('max', 0)):.0f}"
                    f" / clip {stats.get('clip_level')}   "
                    f"mean {float(stats.get('mean', 0)):.0f}")
            t = stats.get("temperature_c")
            if t is not None:
                status = stats.get("temperature_status") or ""
                text += (f"   temp {float(t):.1f} °C"
                         + (f" ({status})" if status else ""))
        except (TypeError, ValueError):
            text = "raw stats unreadable"
        self._signal_label.setText(text)

    # ── One-shot signal optimizer (v7.13.x) ───────────────────────

    def _on_optimize_clicked(self):
        if self._mgr is None or self._optimizing:
            return
        self._optimizing = True
        self._opt_btn.setEnabled(False)
        self._opt_note.setText("optimizing signal…")
        import threading
        threading.Thread(target=self._optimize_worker, daemon=True,
                         name="SignalOptimize").start()

    def _optimize_worker(self):
        """Daemon worker — run_signal_optimize BLOCKS on fresh raw captures
        (worker-thread-only contract, like capture_raw_average)."""
        try:
            from gui.widgets.mono_display import run_signal_optimize
            exp_us, note = run_signal_optimize(self._mgr, self._cam_idx)
        except Exception as exc:
            exp_us, note = None, f"optimize failed: {exc}"
        self._optimize_done.emit(exp_us, note)

    def _on_optimize_done(self, exp_us, note):
        self._optimizing = False
        self._opt_btn.setEnabled(True)
        self._opt_note.setText(str(note))
        logger.info(f"Cam {self._cam_idx + 1}: signal optimize -> {note}")
        if exp_us is not None:
            # Exposure + frozen display levels changed on the device —
            # persist the achieved state and re-read every control.
            self._persist(now=True)
        self.reload()

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
        # v7.13 — Andor sensor features: re-apply the backend's low-noise
        # defaults (cooling on, slow readout, 16-bit low-noise gain, filters).
        if any(k in ctrls for k in self._sensor_checks) or \
                any(k in ctrls for k in self._sensor_combos):
            if hasattr(self._mgr, "reset_andor_sensor_defaults"):
                self._mgr.reset_andor_sensor_defaults(self._cam_idx)
        for key, setter in (
            ("gamma", self._mgr.set_hw_gamma),
            ("brightness", self._mgr.set_hw_brightness),
            ("contrast", self._mgr.set_hw_contrast),
        ):
            rng = (ctrls.get(key) or {}).get("range")
            if rng and len(rng) >= 3:
                setter(self._cam_idx, rng[2])
        self._persist(now=True)
        self.reload()

    def _on_read_clicked(self):
        """Re-read from camera, log to terminal, refresh widgets + readout."""
        if self._mgr is None:
            return
        self._mgr.log_hw_settings(self._cam_idx, prefix="[Read button] ")
        self.reload()

    # ── Persistence ───────────────────────────────────────────────

    def _persist(self, *, now: bool = False):
        """Schedule a write of the current hardware controls.

        Debounced by DEFAULT: a continuous control (slider / spin) fires this
        on every tick of a drag, and coalescing them is the whole point. Pass
        ``now=True`` from a discrete decision — a checkbox, a resolution
        change, Defaults — where there is exactly one event and the operator
        expects it committed.

        Defaulting to debounced is deliberate: a call site that forgets to ask
        for an immediate write merely lands 400 ms later (and is still flushed
        on hide / close / quit), whereas defaulting to immediate would let a
        future slider silently reintroduce the write storm.
        """
        self._persist_pending = True
        if now:
            self._flush_persist()
        else:
            self._persist_timer.start()

    def _flush_persist(self):
        """Write the pending hardware controls to the per-identity store.

        Reads the live state at FLUSH time rather than snapshotting it at
        schedule time, so the value written is the one the camera actually
        ended the drag on.
        """
        try:
            self._persist_timer.stop()
        except Exception:
            pass
        if not self._persist_pending:
            return
        self._persist_pending = False
        if self._identity_getter is None or self._mgr is None:
            return
        try:
            identity = self._identity_getter()
        except Exception:
            identity = None
        if not identity:
            return
        st = self._mgr.get_hw_settings(self._cam_idx)
        # v7.13 — ONE shared key list for both persistence sites (this dialog
        # and Hardware Setup's bulk save), so a key can no longer be persisted
        # by one and silently dropped by the other.
        controls = hw_controls_snapshot(st)
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
        if src in ("toupcam", "opencv", "andor", "tucam"):
            lines.append(f"resolution    = {st.get('resolution')}"
                         + (f"  (eSize {st.get('eSize')})"
                            if st.get("eSize") is not None else ""))
            lines.append(f"auto-exposure = {st.get('auto_exposure')}")
            lines.append(f"exposure      = {st.get('exposure_us')} us"
                         + (f"   range {st.get('exposure_range_us')}"
                            if st.get("exposure_range_us") else ""))
            lines.append(f"gain          = {st.get('exposure_gain_pct')} %")
            # Mono→8-bit display scaling: reported for every mono scientific
            # camera (Zyla, Tucsen) since both render through the same shared
            # conversion and the operator compares them directly.
            if src in ("andor", "tucam"):
                mode = ("auto (per-frame)" if st.get("andor_auto_scale")
                        else "manual")
                lines.append(f"display scale = {mode}   levels "
                             f"{st.get('andor_scale_lo')}.."
                             f"{st.get('andor_scale_hi')}")
            # Sensor temperature: reported for both mono scientific cameras
            # (v7.13 widened from tucam-only once the Zyla gained the read).
            if src in ("andor", "tucam") and st.get("temperature_c") is not None:
                temp_line = f"sensor temp   = {st.get('temperature_c')} C"
                if st.get("temperature_status"):
                    temp_line += f" ({st.get('temperature_status')})"
                lines.append(temp_line)
            if src == "andor":
                for key, lbl in (("andor_sensor_cooling", "cooling"),
                                 ("andor_readout_rate", "readout rate"),
                                 ("andor_gain_mode", "gain mode"),
                                 ("andor_noise_filter", "noise filter"),
                                 ("andor_blemish_correction", "blemish corr")):
                    if st.get(key) is not None:
                        lines.append(f"{lbl:<13} = {st.get(key)}")
                if st.get("bit_depth") is not None:
                    lines.append(f"bit depth     = {st.get('bit_depth')}"
                                 f"   clip {st.get('raw_clip_level')}")
                # v7.13.x — the frame-rate constraint made visible: exposure
                # max ≈ 1/frame rate, and running above the link max is what
                # dropped frames at full resolution.
                if st.get("frame_rate") is not None:
                    try:
                        fr_line = (f"frame rate    = "
                                   f"{float(st.get('frame_rate')):.2f} fps")
                        mitr = st.get("max_interface_transfer_rate")
                        if mitr:
                            fr_line += f" (link max {float(mitr):.2f})"
                        lines.append(fr_line)
                    except (TypeError, ValueError):
                        pass
            if src == "tucam":
                lines.append(f"frame format  = {st.get('channels')} ch, "
                             f"{st.get('elem_bytes')} byte/px")
            lines.append(f"gamma         = {st.get('gamma')}")
            lines.append(f"brightness    = {st.get('brightness')}")
            lines.append(f"contrast      = {st.get('contrast')}")
            # v7.13 — the SOFTWARE (display-only) correction always exists and
            # is always readable, even on cameras with no ISP (the Zyla's
            # gamma/brightness/contrast above are None BY DESIGN — its knobs
            # live here and in the per-slot Image Correction strip).
            try:
                corr = (self._mgr.image_correction(self._cam_idx)
                        if hasattr(self._mgr, "image_correction") else None)
            except Exception:
                corr = None
            if corr:
                lines.append(
                    f"software corr = brightness {corr.get('brightness', 0):+}"
                    f"   contrast {float(corr.get('contrast', 1.0)):.2f}"
                    f"   gamma {float(corr.get('gamma', 1.0)):.2f}"
                    f"   (display-only)")
        else:
            lines.append("(no controllable camera — start the camera first)")
        self._readout.setPlainText("\n".join(str(x) for x in lines))

    # ── Visibility-scoped stats timer (v7.13) ─────────────────────

    def showEvent(self, event):  # noqa: N802 (Qt override)
        super().showEvent(event)
        try:
            self._stats_timer.start()
        except Exception:
            pass

    def hideEvent(self, event):  # noqa: N802 (Qt override)
        try:
            self._stats_timer.stop()
        except Exception:
            pass
        # A debounced write must never be lost to closing the dialog mid-drag.
        self._flush_persist()
        super().hideEvent(event)

    def closeEvent(self, event):  # noqa: N802 (Qt override)
        try:
            self._stats_timer.stop()
        except Exception:
            pass
        self._flush_persist()
        super().closeEvent(event)
