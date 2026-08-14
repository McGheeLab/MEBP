"""
fluorescence_controls_panel.py — the Fluorescence Mosaic workflow's own left
context panel (v7.19).

Operator: *"lets make all of these settings on the left context menu for the
fluorescence mosaic workflow. this is a special left context panel just for this
workflow page."*

WHY THIS IS A PANEL AND NOT A TOP ROW
------------------------------------
The page's main area is already three competing views (well navigator, live
camera, mosaic). The controls that shape an acquisition — objective, cubes,
exposure, gain, averaging, display levels — were spread between a cramped top
row and a modal that only appeared once per channel, mid-run, which is far too
late to judge a signal by. Moving them into the left context box puts every one
of them beside a live histogram, before the run.

The left-box machinery already supports this: ``MainWindow._refresh_left_context``
mounts whatever the current page's ``get_context_widget()`` returns and shows or
hides the box accordingly, ``WorkflowsModePage`` already delegates that call to
the active workflow, and ``ContextPanelHost.set_native_label`` relabels the pill.
Nothing new was needed below this widget.

ONE CLASS, TWO MOUNT POINTS
---------------------------
Standalone, the page hands this to the context host. Embedded inside Cell
Targeting / Spheroid Pickup — which own the left box with their own jog panel —
the page mounts the SAME class inline instead. Two independently written
surfaces onto one set of settings is the failure recorded for
``UNIFIED_MOSAIC_CALIBRATION``; one class with two parents cannot drift.

WIDGET OWNERSHIP
----------------
The filter-cube pills and the objective combo are built here but published onto
the page under the attribute names it has always used
(``page._channel_checks`` / ``page._objective_combo``). That keeps
``_apply_pill`` / ``_selected_channels`` / ``_refresh_objectives`` and their
tests working unchanged — this is a relocation of ``_build_top_row``'s body, not
a rewrite of the page's model.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget)

from gui.scaling import s, sf, sp
from gui.styles import COLORS
from gui.widgets.raw_histogram_widget import RawHistogramWidget
from gui.widgets.signal_slider import LINEAR, LOG, SignalSlider

logger = logging.getLogger(__name__)

#: Histogram refresh. Matches the camera settings dialog's Signal group; the
#: raw stats are produced by the reader loop, so this only reads a snapshot.
_HIST_MS = 500

#: How often to re-read which cube/objective the body actually has in the light
#: path. The microscope panel polls at the same ~1 s cadence, and the operator
#: can turn the cassette by hand, so this is a READ of hardware truth rather
#: than a UI flag we maintain.
_OPTICS_MS = 1000

#: Below this panel width the slider name labels are dropped so the track and
#: the readout still fit. The box's own minimum is s(100).
_COMPACT_PX = 170

SCAN_ORDERS = (
    ("tile", "Every colour per tile"),
    ("channel", "Full scan per colour"),
)


class _ChannelPill(QPushButton):
    """Checkable filter-cube pill. Click toggles inclusion; double-click opens
    the colour picker for the channel's display pseudo-colour.

    Moved here from the workflow page in v7.19 (unchanged behaviour) so the
    panel owns the whole cube row.
    """

    color_requested = Signal()

    def __init__(self, text: str, parent: QWidget | None = None):
        super().__init__(text, parent)
        self.setCheckable(True)
        self.setChecked(True)
        self.setCursor(Qt.PointingHandCursor)

    def mouseDoubleClickEvent(self, event):  # noqa: N802 (Qt override)
        self.color_requested.emit()
        super().mouseDoubleClickEvent(event)


class FluorescenceControlsPanel(QWidget):
    """Objective · camera preset · scan order · per-cube signal recipe.

    ``page`` is the ``FluorescenceMosaicWorkflowPage``. Only a handful of its
    methods are called (listed in ``_PAGE_API`` below) so a stub page is enough
    to drive this offscreen in tests.
    """

    #: Documented so a rename on the page is caught by a test rather than by a
    #: silent AttributeError inside a guarded handler.
    _PAGE_API = (
        "channels", "channel_recipe", "set_channel_recipe",
        "camera_manager_for_panel", "microscope_cam_idx",
        "on_panel_objective_changed", "on_panel_activate_channel",
        "on_panel_pill_toggled", "on_panel_pick_colour",
        "on_panel_scan_order_changed", "on_panel_camera_preset_changed",
        "panel_optics_state", "is_scanning", "run_auto_exposure",
        "on_panel_objective_detected",
    )

    def __init__(self, page, parent: QWidget | None = None):
        super().__init__(parent)
        self._page = page
        self._sliders: dict[str, SignalSlider] = {}
        self._pills: dict[str, _ChannelPill] = {}
        self._activate: dict[str, QPushButton] = {}
        self._active: Optional[str] = None
        self._compact = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(4), s(4), s(4), s(4))
        outer.setSpacing(s(6))

        outer.addWidget(self._build_histogram())
        outer.addWidget(self._build_optics())
        outer.addWidget(self._build_cubes())
        outer.addWidget(self._build_signal())
        outer.addStretch(1)
        # Render once so the strip is never in an unlabelled state: _set_active
        # early-returns when the channel has not CHANGED, and "no cube yet" is
        # already the starting value.
        self._render_active_marks()
        self._apply_enablement()

        self._hist_timer = QTimer(self)
        self._hist_timer.setInterval(_HIST_MS)
        self._hist_timer.timeout.connect(self._refresh_histogram)
        self._optics_timer = QTimer(self)
        self._optics_timer.setInterval(_OPTICS_MS)
        self._optics_timer.timeout.connect(self.refresh_optics)

    # ── build ────────────────────────────────────────────────────────
    def _section(self, title: str = "") -> tuple[QFrame, QVBoxLayout]:
        frame = QFrame(self)
        box = QVBoxLayout(frame)
        box.setContentsMargins(0, 0, 0, 0)
        box.setSpacing(s(3))
        if title:
            head = QLabel(title)
            head.setWordWrap(True)
            head.setStyleSheet(
                f"color: {COLORS['blue']}; font-size: {sf(9)}pt; "
                f"font-weight: 600;")
            box.addWidget(head)
        return frame, box

    def _build_histogram(self) -> QWidget:
        frame, box = self._section("Signal")
        self._hist = RawHistogramWidget(self)
        box.addWidget(self._hist)
        self._clip = QLabel("—")
        self._clip.setWordWrap(True)
        self._clip.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        box.addWidget(self._clip)
        return frame

    def _build_optics(self) -> QWidget:
        frame, box = self._section("Optics")
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(s(4))
        grid.setVerticalSpacing(s(3))
        grid.setColumnStretch(1, 1)

        grid.addWidget(self._small("Objective"), 0, 0)
        self._objective_combo = QComboBox()
        self._objective_combo.setSizePolicy(QSizePolicy.Ignored,
                                            QSizePolicy.Preferred)
        self._objective_combo.setMinimumWidth(s(50))
        self._objective_combo.setToolTip(
            "With the microscope connected this ROTATES the nosepiece and "
            "verifies the new position by read-back. Without one it is a "
            "label: the µm/px it selects is what every tile is scaled by, so "
            "it must match the objective you actually fitted.")
        self._objective_combo.currentTextChanged.connect(
            self._on_objective_changed)
        grid.addWidget(self._objective_combo, 0, 1)

        grid.addWidget(self._small("Camera"), 1, 0)
        self._preset_btn = QPushButton("📷 Fluorescence")
        self._preset_btn.setCheckable(True)
        self._preset_btn.setChecked(True)
        self._preset_btn.setCursor(Qt.PointingHandCursor)
        self._preset_btn.setSizePolicy(QSizePolicy.Ignored,
                                       QSizePolicy.Preferred)
        self._preset_btn.setMinimumWidth(s(50))
        self._preset_btn.setToolTip(
            "Fluorescence: auto-exposure, the sensor's auto black/white "
            "levels and the per-frame display auto-scale are all OFF, and "
            "gamma/contrast/brightness are neutral — so an exposure you set "
            "stays set and tiles stay comparable.\n"
            "Camera defaults: whatever the camera was running when you opened "
            "this page. Restored automatically when you leave.")
        self._preset_btn.toggled.connect(self._on_preset_toggled)
        grid.addWidget(self._preset_btn, 1, 1)

        grid.addWidget(self._small("Scan order"), 2, 0)
        self._order_combo = QComboBox()
        self._order_combo.setSizePolicy(QSizePolicy.Ignored,
                                        QSizePolicy.Preferred)
        self._order_combo.setMinimumWidth(s(50))
        for key, label in SCAN_ORDERS:
            self._order_combo.addItem(label, key)
        self._order_combo.setToolTip(
            "Every colour per tile: the cube is switched at each tile, so the "
            "channels are captured seconds apart and share one focus and one "
            "registration — they overlay exactly. Costs one cube rotation per "
            "tile per colour.\n"
            "Full scan per colour: one rotation per colour, but a channel is "
            "captured minutes after the last one.\n"
            "Note both modes share ONE focus per tile; per-channel chromatic "
            "focus offsets are not applied (within depth of field at 4×/10×).")
        self._order_combo.currentIndexChanged.connect(self._on_order_changed)
        grid.addWidget(self._order_combo, 2, 1)

        box.addLayout(grid)
        self._optics_note = QLabel("")
        self._optics_note.setWordWrap(True)
        self._optics_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        self._optics_note.setVisible(False)
        box.addWidget(self._optics_note)
        return frame

    def _build_cubes(self) -> QWidget:
        frame, box = self._section("Filter cubes")
        for ch in self._channels():
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(s(3))

            pill = _ChannelPill(ch)
            pill.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            pill.setMinimumWidth(s(30))
            pill.setToolTip(
                f"{ch}: click to include it in the capture, double-click to "
                f"set its display colour.")
            pill.toggled.connect(lambda _c, c=ch: self._on_pill_toggled(c))
            pill.color_requested.connect(lambda c=ch: self._on_pick_colour(c))
            self._pills[ch] = pill
            row.addWidget(pill, stretch=1)

            act = QPushButton("◉")
            act.setCursor(Qt.PointingHandCursor)
            act.setFixedWidth(s(22))
            act.setToolTip(
                f"Put the {ch} cube in the light path so its signal controls "
                f"become editable.")
            act.clicked.connect(lambda _=False, c=ch: self._on_activate(c))
            self._activate[ch] = act
            row.addWidget(act)
            box.addLayout(row)
        return frame

    def _build_signal(self) -> QWidget:
        """ONE set of sliders, applied to whichever cube is in the path.

        v7.19.2, operator: *"there should be one slider for exposure gain
        averaging whie black … if we move the sliders around they get applied to
        the active filter"*.

        The first cut gave every cube its own exposure slider and hid the rest
        behind a "More signal" disclosure. That is worse on both counts: N-1 of
        the sliders were permanently disabled decoration (only the fitted cube's
        may drive the camera), and the four controls you reach for while
        judging a histogram were one click away and out of sight. One row per
        control, always visible, is the same information in a fifth of the
        height — which matters in a context box that goes down to s(100).

        These are a per-channel RECIPE, not a global: the strip re-renders from
        the newly-active channel's stored values every time the cassette moves,
        including while a mosaic is running.
        """
        frame, box = self._section()
        self._signal_hdr = QLabel("Signal")
        self._signal_hdr.setStyleSheet(
            f"color:{COLORS['blue']};font-size:{sf(9)}pt;font-weight:600;")
        box.addWidget(self._signal_hdr)

        # ⚠ lo/hi are in the CONTROL's units (µs — what the camera stack uses
        # everywhere); `scale` only changes what the READOUT says. Getting that
        # wrong caps the exposure slider at 10 µs instead of 10 s. These are a
        # sane fluorescence default until refresh_ranges() replaces them with
        # the camera's own declared range.
        specs = (
            ("exposure_us", "exposure", 50.0, 10_000_000.0, LOG, 1, " ms", 1e-3),
            ("gain_pct", "gain", 0.0, 100.0, LINEAR, 0, " %", 1.0),
            ("avg_frames", "averaging", 1.0, 32.0, LINEAR, 0, " frames", 1.0),
            ("display_lo", "black", 0.0, 65535.0, LINEAR, 0, "", 1.0),
            ("display_hi", "white", 0.0, 65535.0, LINEAR, 0, "", 1.0),
        )
        for key, label, lo, hi, mode, dec, suffix, scale in specs:
            sl = SignalSlider(label, lo=lo, hi=hi, mode=mode, decimals=dec,
                              suffix=suffix, scale=scale)
            sl.set_apply(lambda v, k=key: self._apply_active(k, v))
            sl.committed.connect(lambda v, k=key: self._commit_active(k, v))
            self._sliders[key] = sl
            box.addWidget(sl)
        self._auto_btn = QPushButton("⚡ Auto exposure")
        self._auto_btn.setCursor(Qt.PointingHandCursor)
        self._auto_btn.setToolTip(
            "Step the exposure until the brightest real signal sits just below "
            "clipping. Exposure only — gain and averaging are left as you set "
            "them.")
        self._auto_btn.clicked.connect(self._on_auto)
        box.addWidget(self._auto_btn)
        return frame

    def _small(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        return lbl

    # ── page bridge (every call guarded: a panel must never break a page) ──
    def _channels(self) -> tuple:
        try:
            return tuple(self._page.channels())
        except Exception:
            return ()

    def _call(self, name: str, *args):
        fn = getattr(self._page, name, None)
        if fn is None:
            return None
        try:
            return fn(*args)
        except Exception as exc:
            logger.debug("fluor panel: %s failed: %s", name, exc)
            return None

    # ── public API used by the page ──────────────────────────────────
    def pills(self) -> dict:
        """The cube pills, keyed by channel — published as the page's
        ``_channel_checks`` so every existing page method keeps working."""
        return self._pills

    def objective_combo(self) -> QComboBox:
        return self._objective_combo

    def order_combo(self) -> QComboBox:
        """The scan-order combo, so the page can register it for persistence."""
        return self._order_combo

    def preset_button(self) -> QPushButton:
        """The camera-preset toggle, ditto. Checkable, so it round-trips as a
        bool through the settings store's widget handling."""
        return self._preset_btn

    def scan_order(self) -> str:
        return str(self._order_combo.currentData() or "tile")

    def set_scan_order(self, key: str):
        idx = self._order_combo.findData(str(key))
        if idx >= 0:
            blocked = self._order_combo.blockSignals(True)
            try:
                self._order_combo.setCurrentIndex(idx)
            finally:
                self._order_combo.blockSignals(blocked)

    def preset_is_fluorescence(self) -> bool:
        return bool(self._preset_btn.isChecked())

    def set_preset_is_fluorescence(self, on: bool):
        blocked = self._preset_btn.blockSignals(True)
        try:
            self._preset_btn.setChecked(bool(on))
            self._render_preset()
        finally:
            self._preset_btn.blockSignals(blocked)

    def start_polling(self):
        self._hist_timer.start()
        self._optics_timer.start()
        self._refresh_histogram()
        self.refresh_optics()

    def stop_polling(self):
        self._hist_timer.stop()
        self._optics_timer.stop()
        self.flush()

    def flush(self):
        """Push any debounced slider change before the panel goes away."""
        for sl in self._sliders.values():
            try:
                sl.flush()
            except Exception:
                pass

    def refresh_ranges(self):
        """Re-read the camera's declared ranges into the sliders.

        Ranges move under us (the Andor's achievable exposure depends on the
        readout rate and gain mode), so this is called on show and after a
        resolution or preset change rather than only at construction.
        """
        caps = self._caps()
        ctrls = (caps or {}).get("controls") or {}
        rng = (ctrls.get("exposure_us") or {}).get("range")
        if rng and len(rng) >= 2 and rng[1] and rng[1] > rng[0]:
            self._sliders["exposure_us"].set_range(float(rng[0]), float(rng[1]))
        grng = (ctrls.get("exposure_gain_pct") or {}).get("range")
        if grng and len(grng) >= 2 and grng[1] and grng[1] > grng[0]:
            self._sliders["gain_pct"].set_range(float(grng[0]), float(grng[1]))
        lrng = (ctrls.get("andor_scale_lo") or {}).get("range")
        if lrng and len(lrng) >= 2 and lrng[1] and lrng[1] > lrng[0]:
            self._sliders["display_lo"].set_range(float(lrng[0]), float(lrng[1]))
            self._sliders["display_hi"].set_range(float(lrng[0]), float(lrng[1]))
        # A camera with no raw path has no histogram to show and no ⚡Auto to
        # run — hide both rather than render an empty box that looks broken.
        has_raw = "andor_raw_stats" in ctrls
        self._hist.setVisible(has_raw)
        self._clip.setVisible(has_raw)
        self._auto_btn.setVisible(has_raw)
        self._sliders["gain_pct"].setVisible("exposure_gain_pct" in ctrls)
        for key in ("display_lo", "display_hi"):
            self._sliders[key].setVisible("andor_scale_lo" in ctrls)

    def load_recipes(self):
        """Show the ACTIVE channel's stored recipe.

        There is one strip now, so there is nothing to seed per channel — the
        values follow whichever cube is in the light path.
        """
        self._render_active_recipe()

    # ── histogram ────────────────────────────────────────────────────
    def _mgr(self):
        return self._call("camera_manager_for_panel")

    def _cam_idx(self) -> int:
        idx = self._call("microscope_cam_idx")
        return int(idx) if isinstance(idx, int) else 0

    def _caps(self) -> dict:
        mgr = self._mgr()
        if mgr is None or not hasattr(mgr, "hardware_capabilities"):
            return {}
        try:
            return mgr.hardware_capabilities(self._cam_idx()) or {}
        except Exception:
            return {}

    def _refresh_histogram(self):
        mgr = self._mgr()
        stats = None
        if mgr is not None and hasattr(mgr, "get_raw_frame_stats"):
            try:
                stats = mgr.get_raw_frame_stats(self._cam_idx())
            except Exception:
                stats = None
        self._hist.set_stats(stats)
        if not stats:
            self._clip.setText("no raw data")
            return
        frac = float(stats.get("clipped_frac") or 0.0)
        clip = stats.get("clip_level")
        bits = [f"{frac * 100.0:.2f} % clipped"]
        if clip:
            bits.append(f"full scale {int(clip)}")
        # 0.5 % is the same threshold the live feed's SATURATED badge uses.
        if frac >= 0.005:
            bits.append("⚠ SATURATED")
            self._clip.setStyleSheet(
                f"color: {COLORS['red']}; font-size: {sf(8)}pt;")
        else:
            self._clip.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        self._clip.setText(" · ".join(bits))

    # ── optics truth ─────────────────────────────────────────────────
    def refresh_optics(self):
        """Re-read which cube and objective the BODY has in the light path.

        The active channel is a hardware fact, not a UI flag: the operator can
        turn the cassette by hand, and a panel that kept its own idea of
        "active" would enable the wrong channel's sliders and silently record
        an exposure against the wrong cube.
        """
        # v7.19.1 — never re-index a combo whose drop-down is open: it moves the
        # highlight under the operator's cursor mid-selection. microscope_panel
        # has guarded this since v7.5.x; this panel did not.
        try:
            popup_open = self._objective_combo.view().isVisible()
        except Exception:
            popup_open = False
        if popup_open:
            return
        # v7.19.1 — and ask the body for a fresh read first. MicroscopeController
        # does not poll itself, so when this page runs STANDALONE its context box
        # holds this panel rather than microscope_panel, and nothing else on
        # screen ever re-reads the hardware. Embedded in Cell Targeting /
        # Spheroid the jog panel happens to do it for us — a freshness that
        # depended on which page you were on.
        try:
            from gui.widgets.optics_ensure import request_state_refresh
            request_state_refresh()
        except Exception:
            pass
        st = self._call("panel_optics_state") or {}
        self._set_active(st.get("active_channel"))
        note = str(st.get("note") or "")
        self._optics_note.setText(note)
        self._optics_note.setVisible(bool(note))
        obj = st.get("objective")
        if obj and obj != self._objective_combo.currentText():
            idx = self._objective_combo.findText(str(obj))
            if idx >= 0:
                blocked = self._objective_combo.blockSignals(True)
                try:
                    self._objective_combo.setCurrentIndex(idx)
                finally:
                    self._objective_combo.blockSignals(blocked)
            # Showing the new objective is not enough — the µm/px every tile is
            # scaled by is keyed by NAME, so a detected change that is not
            # adopted leaves the mosaic measured with the previous objective.
            # The page decides what that means; the panel only reports it.
            self._call("on_panel_objective_detected", str(obj))
        for ch, btn in self._activate.items():
            reason = (st.get("cube_refusals") or {}).get(ch)
            btn.setEnabled(not reason)
            if reason:
                btn.setToolTip(str(reason))

    def _set_active(self, channel):
        channel = channel if channel in self._pills else None
        if channel == self._active:
            # Still re-assert enablement: a scan starting or ending changes it.
            self._apply_enablement()
            return
        self._active = channel
        self._apply_enablement()
        self._render_active_recipe()
        self._render_active_marks()

    def _render_active_marks(self):
        """Box the cube that is in the light path, and name it on the strip.

        v7.19.2, operator: *"it should have a highligh box around the current
        filter on the signal pannel"*. The green ``◉`` alone was too quiet for
        the one fact the whole strip depends on — every slider below writes to
        THIS channel's recipe, so mistaking which is active records an exposure
        against the wrong cube.
        """
        for ch, pill in self._pills.items():
            live = (ch == self._active)
            pill.setProperty("liveCube", live)
            pill.setStyleSheet(
                f"QPushButton{{border:{sp(2)} solid {COLORS['green']};"
                f"border-radius:{sp(4)};padding:{sp(1)};}}" if live else "")
            pill.setToolTip(
                f"{ch} is in the light path — the signal sliders below apply "
                f"to it." if live else
                f"{ch}: click to include it in the capture, double-click to "
                f"set its display colour.")
        for ch, btn in self._activate.items():
            btn.setStyleSheet(
                f"QPushButton{{color:{COLORS['green']};font-weight:700;}}"
                if ch == self._active else "")
        self._signal_hdr.setText(
            f"Signal — {self._active}" if self._active
            else "Signal — no cube in the path")

    def _apply_enablement(self):
        scanning = bool(self._call("is_scanning"))
        for sl in self._sliders.values():
            sl.setEnabled(self._active is not None and not scanning)
        self._auto_btn.setEnabled(self._active is not None and not scanning)
        self._objective_combo.setEnabled(not scanning)
        self._order_combo.setEnabled(not scanning)
        self._preset_btn.setEnabled(not scanning)

    # ── handlers ─────────────────────────────────────────────────────
    def _on_objective_changed(self, name: str):
        self._call("on_panel_objective_changed", name)

    def _on_preset_toggled(self, on: bool):
        self._render_preset()
        self._call("on_panel_camera_preset_changed", bool(on))

    def _render_preset(self):
        on = self._preset_btn.isChecked()
        self._preset_btn.setText(
            "📷 Fluorescence" if on else "📷 Camera defaults")

    def _on_order_changed(self, *_):
        self._call("on_panel_scan_order_changed", self.scan_order())

    def _on_pill_toggled(self, channel: str):
        self._call("on_panel_pill_toggled", channel)

    def _on_pick_colour(self, channel: str):
        self._call("on_panel_pick_colour", channel)

    def _on_activate(self, channel: str):
        self._call("on_panel_activate_channel", channel)

    def _on_auto(self):
        if self._active is None:
            return
        found = self._call("run_auto_exposure", self._active)
        if found:
            self._sliders["exposure_us"].set_value(float(found))
            self._commit(self._active, "exposure_us", float(found))

    # ── recipe plumbing ──────────────────────────────────────────────
    def _apply_control(self, channel: str, key: str, value: float):
        """Push one control to the camera, returning what it ACHIEVED.

        Only the channel actually in the light path may drive the camera —
        otherwise setting "Cy5's exposure" while DAPI is fitted would change
        the live image and record the value against the wrong cube.
        """
        if channel != self._active:
            return None
        return self._call("apply_panel_control", key, value)

    def _apply_active(self, key: str, value: float):
        if self._active is None:
            return None
        return self._apply_control(self._active, key, value)

    def _commit(self, channel: str, key: str, value: float):
        self._call("set_channel_recipe", channel, key, value)

    def _commit_active(self, key: str, value: float):
        if self._active is not None:
            self._commit(self._active, key, value)

    def _render_active_recipe(self):
        if self._active is None:
            return
        rec = self._call("channel_recipe", self._active) or {}
        for key, sl in self._sliders.items():
            val = rec.get(key)
            if val is not None:
                sl.set_value(float(val))

    # ── narrow-width behaviour ───────────────────────────────────────
    def minimumSizeHint(self):  # noqa: N802 (Qt override)
        """Let the context box drive this panel down to its own minimum.

        Same override HardwareControlPanel and StandardJogContextPanel use: a
        natural minimum computed from the children would stop the box shrinking
        and force a horizontal scrollbar, which the host explicitly disables.
        """
        hint = super().minimumSizeHint()
        hint.setWidth(s(90))
        return hint

    def resizeEvent(self, event):  # noqa: N802 (Qt override)
        compact = self.width() < s(_COMPACT_PX)
        if compact != self._compact:
            self._compact = compact
            for sl in self._sliders.values():
                sl.set_compact(compact)
        super().resizeEvent(event)
