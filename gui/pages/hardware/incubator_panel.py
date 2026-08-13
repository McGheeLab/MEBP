"""
incubator_panel.py — Hardware Setup → Incubator sub-page (v7.18).

The ONE settings surface for the incubator: transport (shared ZP board /
dedicated port / simulator), zone naming + enable + presets, the setpoint
ceiling and ramp/dither preferences. Follows the Microscope-tab pattern
exactly:

* both dependencies (store, controller service) default to their singletons
  but are injectable, so the panel is unit-testable against a temp-dir store;
* NOTHING reaches the store until Save — ``commit()`` writes every field with
  ``save=False`` and finishes with exactly one ``store.save()``;
* connecting does NOT live here: the Incubator page (Workflows → Incubator)
  and the Connect Hardware card open the link. A hidden extra writer of
  connection state is the "two owners of one fact" trap the microscope panel
  already retired.

This panel deliberately does not participate in ``HardwareConfig`` — heater
wiring, ceilings and zone names are properties of THIS rig, not of a
swappable print setup (see ``SupportClasses/incubator/config_store.py``).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QSpinBox, QVBoxLayout,
    QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

from SupportClasses.incubator.config_store import (
    HARD_MAX_SETPOINT_C, MAX_RAMP_STEP_C, get_store,
)
from SupportClasses.incubator.zones import ALL_ZONES

logger = logging.getLogger(__name__)

_STATUS_MS = 500

_TRANSPORT_ITEMS = (
    ("shared", "Shared ZP board (recommended)"),
    ("serial", "Own serial port"),
    ("simulate", "Simulator"),
)


class IncubatorSetupPanel(QWidget):
    """Incubator settings editor. ``load()`` / ``commit()`` are the surface."""

    def __init__(self, store=None, controller=None, parent=None, *,
                 show_save: bool = True):
        super().__init__(parent)
        self._store = store if store is not None else get_store()
        #: () -> IncubatorController | None. Lazy + injectable: the panel only
        #: OBSERVES the session (status line); it never opens or closes it.
        if controller is None:
            def controller():
                from SupportClasses.incubator.service import peek_incubator
                return peek_incubator()
        self._peek = controller
        self._show_save = show_save

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)     # host supplies margins
        root.setSpacing(s(14))
        root.addWidget(self._build_transport_group())
        root.addWidget(self._build_zones_group())
        root.addWidget(self._build_behaviour_group())
        if show_save:
            root.addLayout(self._build_save_row())
        root.addStretch(1)

        self._timer = QTimer(self)
        self._timer.setInterval(_STATUS_MS)
        self._timer.timeout.connect(self._refresh_live)

        self.load()

    # ── lifecycle ───────────────────────────────────────────────────

    def showEvent(self, event):
        super().showEvent(event)
        self._timer.start()
        self._refresh_live()

    def hideEvent(self, event):
        self._timer.stop()
        super().hideEvent(event)

    # ── construction ────────────────────────────────────────────────

    @staticmethod
    def _hint(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        lbl.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: {sf(8.5)}pt;")
        return lbl

    def _build_transport_group(self) -> QGroupBox:
        box = QGroupBox("Connection")
        g = QGridLayout(box)
        g.setContentsMargins(s(12), s(14), s(12), s(12))
        g.setHorizontalSpacing(s(10))
        g.setVerticalSpacing(s(8))
        g.setColumnStretch(2, 1)

        g.addWidget(QLabel("Transport"), 0, 0)
        self._transport = QComboBox()
        for key, label in _TRANSPORT_ITEMS:
            self._transport.addItem(label, key)
        self._transport.currentIndexChanged.connect(self._refresh_rows)
        g.addWidget(self._transport, 0, 1)

        self._status_lbl = QLabel("")
        self._status_lbl.setStyleSheet(f"color: {COLORS['overlay0']};")
        g.addWidget(self._status_lbl, 0, 2)

        self._port_lbl = QLabel("Dedicated port")
        g.addWidget(self._port_lbl, 1, 0)
        self._port = QLineEdit()
        self._port.setPlaceholderText("e.g. COM7 (blank = auto-detect)")
        self._port.setMaximumWidth(s(180))
        g.addWidget(self._port, 1, 1)
        self._port_hint = self._hint(
            "Only for a SEPARATE incubator board. The heaters on this rig are "
            "wired to the ZP board itself, which the shared transport reaches "
            "with no extra cable — and the app keeps every scan away from a "
            "port named here."
        )
        g.addWidget(self._port_hint, 1, 2)

        self._baud_lbl = QLabel("Baud")
        g.addWidget(self._baud_lbl, 2, 0)
        self._baud = QSpinBox()
        self._baud.setRange(1200, 1000000)
        self._baud.setValue(38400)
        self._baud.setMaximumWidth(s(180))
        g.addWidget(self._baud, 2, 1)

        self._sim_lbl = QLabel("Simulator time scale")
        g.addWidget(self._sim_lbl, 3, 0)
        self._sim_scale = QSpinBox()
        self._sim_scale.setRange(1, 5000)
        self._sim_scale.setValue(300)
        self._sim_scale.setPrefix("x")
        self._sim_scale.setMaximumWidth(s(180))
        g.addWidget(self._sim_scale, 3, 1)

        g.addWidget(self._hint(
            "Connecting happens on the Incubator page (Workflows → "
            "Incubator) or the Connect Hardware card — this tab only "
            "chooses HOW. A transport change takes effect on the next "
            "connect after Save."
        ), 4, 0, 1, 3)
        return box

    def _build_zones_group(self) -> QGroupBox:
        box = QGroupBox("Zones")
        g = QGridLayout(box)
        g.setContentsMargins(s(12), s(14), s(12), s(12))
        g.setHorizontalSpacing(s(10))
        g.setVerticalSpacing(s(8))
        g.setColumnStretch(2, 1)

        g.addWidget(QLabel("Zone"), 0, 0)
        g.addWidget(QLabel("Enabled"), 0, 1)
        g.addWidget(QLabel("Display name"), 0, 2)
        g.addWidget(QLabel("Preset °C"), 0, 3)

        self._zone_rows: dict[str, dict] = {}
        for i, spec in enumerate(ALL_ZONES, start=1):
            name = QLabel(f"{spec.title}")
            name.setToolTip(
                f"{spec.blurb}\nHeater {spec.heater_connector} / sensor "
                f"{spec.sensor_connector} — {spec.set_cmd} family")
            g.addWidget(name, i, 0)
            enabled = QCheckBox()
            enabled.setToolTip(
                "Untick to hide this zone's card on the Incubator page "
                "(e.g. a zone whose heater is not fitted). Display only — "
                "the firmware and the probe are untouched.")
            g.addWidget(enabled, i, 1)
            label = QLineEdit()
            label.setPlaceholderText(spec.title)
            g.addWidget(label, i, 2)
            preset = QDoubleSpinBox()
            preset.setRange(0.0, HARD_MAX_SETPOINT_C)
            preset.setDecimals(1)
            preset.setValue(37.0)
            preset.setSuffix(" °C")
            g.addWidget(preset, i, 3)
            self._zone_rows[spec.zone_id] = {
                "enabled": enabled, "label": label, "preset": preset,
            }

        g.addWidget(self._hint(
            "Display names show on the zone cards, the trend legend and the "
            "sensor table. The zone→heater wiring itself is fixed in "
            "firmware (Zone A = bed HB/THB, Zone B = hotend HE0/THO)."
        ), len(ALL_ZONES) + 1, 0, 1, 4)
        return box

    def _build_behaviour_group(self) -> QGroupBox:
        box = QGroupBox("Safety && behaviour")
        g = QGridLayout(box)
        g.setContentsMargins(s(12), s(14), s(12), s(12))
        g.setHorizontalSpacing(s(10))
        g.setVerticalSpacing(s(8))
        g.setColumnStretch(2, 1)

        g.addWidget(QLabel("Setpoint ceiling"), 0, 0)
        self._ceiling = QDoubleSpinBox()
        self._ceiling.setRange(1.0, HARD_MAX_SETPOINT_C)
        self._ceiling.setDecimals(1)
        self._ceiling.setValue(HARD_MAX_SETPOINT_C)
        self._ceiling.setSuffix(" °C")
        self._ceiling.setMaximumWidth(s(140))
        g.addWidget(self._ceiling, 0, 1)
        g.addWidget(self._hint(
            f"Hard maximum any setpoint control will accept. Can only LOWER "
            f"the built-in {HARD_MAX_SETPOINT_C:.0f} °C ceiling — the vessel "
            f"holds water and the rig can only heat."
        ), 0, 2)

        g.addWidget(QLabel("Ramp step"), 1, 0)
        self._ramp_step = QDoubleSpinBox()
        self._ramp_step.setRange(0.5, MAX_RAMP_STEP_C)
        self._ramp_step.setDecimals(1)
        self._ramp_step.setValue(3.0)
        self._ramp_step.setSuffix(" °C")
        self._ramp_step.setMaximumWidth(s(140))
        g.addWidget(self._ramp_step, 1, 1)
        g.addWidget(self._hint(
            f"Staircase rung for the watchdog-safe Ramp. Capped at "
            f"{MAX_RAMP_STEP_C:g} °C — above ≈5-6 °C Marlin's heat-up "
            f"watchdog arms, and a slow water block false-trips it (the "
            f"board then halts)."
        ), 1, 2)

        g.addWidget(QLabel("Fine-setpoint period"), 2, 0)
        self._fine_period = QSpinBox()
        self._fine_period.setRange(10, 600)
        self._fine_period.setValue(60)
        self._fine_period.setSuffix(" s")
        self._fine_period.setMaximumWidth(s(140))
        g.addWidget(self._fine_period, 2, 1)
        g.addWidget(self._hint(
            "Dither period for fractional holds (Marlin targets are whole "
            "degrees; the block's thermal mass averages the alternation)."
        ), 2, 2)

        self._log_on_connect = QCheckBox(
            "Start telemetry logging automatically on connect")
        self._log_on_connect.setToolTip(
            "JSONL to logs/incubator/ — one line per sample, flushed "
            "immediately, so a crash still leaves the record.")
        g.addWidget(self._log_on_connect, 3, 0, 1, 3)

        self._off_on_exit = QCheckBox(
            "Default the close-app prompt to \"turn heaters off\"")
        self._off_on_exit.setToolTip(
            "When the app closes while a zone is heating, it asks whether to "
            "switch the heaters off. This sets the pre-selected answer. "
            "NOTE the firmware owns the control loop — leaving heaters "
            "running for a long soak is legitimate, and closing the app "
            "never stops them by itself.")
        g.addWidget(self._off_on_exit, 4, 0, 1, 3)
        return box

    def _build_save_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        self._saved_lbl = QLabel("")
        self._saved_lbl.setStyleSheet(f"color: {COLORS['green']};")
        row.addStretch(1)
        row.addWidget(self._saved_lbl)
        btn = QPushButton("Save incubator setup")
        btn.setObjectName("primaryButton")
        btn.clicked.connect(self._save_clicked)
        row.addWidget(btn)
        return row

    # ── live status ─────────────────────────────────────────────────

    def _refresh_rows(self) -> None:
        t = str(self._transport.currentData() or "shared")
        for wdg in (self._port_lbl, self._port, self._port_hint,
                    self._baud_lbl, self._baud):
            wdg.setVisible(t == "serial")
        for wdg in (self._sim_lbl, self._sim_scale):
            wdg.setVisible(t == "simulate")

    def _refresh_live(self) -> None:
        ctrl = None
        try:
            ctrl = self._peek()
        except Exception:
            pass
        if ctrl is None or not getattr(ctrl, "connected", False):
            self._status_lbl.setText("Not connected.")
            return
        transport = {
            "shared": "shared ZP link", "serial": "dedicated port",
            "simulated": "simulator",
        }.get(getattr(ctrl, "transport", ""), "connected")
        self._status_lbl.setText(
            f"Connected via {transport} "
            f"({getattr(ctrl, 'active_port', '') or 'simulator'}).")

    # ── load / commit ───────────────────────────────────────────────

    def load(self) -> None:
        st = self._store
        idx = self._transport.findData(st.get("transport", "shared"))
        if idx >= 0:
            self._transport.setCurrentIndex(idx)
        self._port.setText(str(st.get("dedicated_port", "") or ""))
        try:
            self._baud.setValue(int(st.get("dedicated_baud", 38400)))
        except Exception:
            pass
        try:
            self._sim_scale.setValue(int(st.get("sim_time_scale", 300)))
        except Exception:
            pass
        try:
            self._ceiling.setValue(float(st.get("max_setpoint_c",
                                                HARD_MAX_SETPOINT_C)))
        except Exception:
            pass
        try:
            self._ramp_step.setValue(float(st.get("ramp_step_c", 3.0)))
        except Exception:
            pass
        try:
            self._fine_period.setValue(int(st.get("fine_period_s", 60)))
        except Exception:
            pass
        self._log_on_connect.setChecked(bool(st.get("log_on_connect", False)))
        self._off_on_exit.setChecked(
            bool(st.get("heaters_off_on_app_exit", True)))
        for zid, row in self._zone_rows.items():
            z = st.zone(zid)
            row["enabled"].setChecked(bool(z.get("enabled", True)))
            row["label"].setText(str(z.get("label") or ""))
            try:
                row["preset"].setValue(float(z.get("preset_c", 37.0)))
            except Exception:
                pass
        self._refresh_rows()

    def commit(self) -> bool:
        """Write every control to the store — ONE disk write at the end."""
        st = self._store
        st.set("transport", str(self._transport.currentData() or "shared"),
               save=False)
        st.set("dedicated_port", self._port.text().strip(), save=False)
        st.set("dedicated_baud", int(self._baud.value()), save=False)
        st.set("sim_time_scale", int(self._sim_scale.value()), save=False)
        st.set("max_setpoint_c", float(self._ceiling.value()), save=False)
        st.set("ramp_step_c", float(self._ramp_step.value()), save=False)
        st.set("fine_period_s", int(self._fine_period.value()), save=False)
        st.set("log_on_connect", bool(self._log_on_connect.isChecked()),
               save=False)
        st.set("heaters_off_on_app_exit",
               bool(self._off_on_exit.isChecked()), save=False)
        for zid, row in self._zone_rows.items():
            st.set_zone(
                zid,
                label=row["label"].text().strip(),
                enabled=row["enabled"].isChecked(),
                preset_c=float(row["preset"].value()),
                save=False,
            )
        ok = st.save()

        # A LIVE controller picks the ceiling/labels up immediately — the
        # ceiling is safety-relevant, so it must not wait for a reconnect.
        try:
            ctrl = self._peek()
            if ctrl is not None:
                from SupportClasses.incubator.service import apply_store_config
                apply_store_config(ctrl)
        except Exception:
            logger.debug("live incubator config push failed", exc_info=True)
        return ok

    def _save_clicked(self) -> None:
        if self.commit():
            self._saved_lbl.setText("Saved ✓")
            QTimer.singleShot(2500, lambda: self._saved_lbl.setText(""))
        else:
            self._saved_lbl.setText("Save failed — see log")
            QTimer.singleShot(4000, lambda: self._saved_lbl.setText(""))
