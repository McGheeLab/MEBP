"""
incubator_workflow.py — the Incubator page (Workflows tile, v7.18).

The full control surface for the two-zone incubator heater: per-zone cards
(setpoint / ramp / fine dither / heater off), temperature trend, sensor table,
and the Stability / PID / Autotune / Sensor-calibration / Firmware / Console
tabs, with ALL HEATERS OFF and EMERGENCY STOP always visible.

Ported from the standalone tool's window (``tools/incubator/gui.py``) with
three integration-driven changes:

1. **No separate connect button — the session FOLLOWS the ZP connection**
   (operator: the heaters are wired to the SAME SKR Mini E3 V3 as the ZP
   stage, *"it does not need a separate connect button"*). With the saved
   transport on its default ("shared"), opening this page while the ZP
   board is connected starts the heater session automatically over the
   live ZP link; if ZP is down the page says which button IS the connect
   (the ZP one). The simulator transport likewise auto-starts. Only the
   dedicated-serial transport — reserved for the planned ESP32 sensor
   board — shows manual Port/Detect/Connect controls, and those stay
   hidden until that transport is selected on Hardware Setup → Incubator.

2. **Nothing blocks the GUI thread.** The standalone tool called
   ``controller.connect()`` (0.35 s banner wait + a multi-second firmware
   probe) and ``detect_board()`` (0.6 s per port per baud) synchronously —
   the exact freeze class this repo has fixed repeatedly
   (JOG_TRAVEL/STAGE_JOG/PUMP_JOG_OFF_GUI_THREAD). Every session start
   runs on a daemon worker with a busy-guard, resumed via queued signals.

3. **The page does not own the session.** The controller is the
   ``get_incubator()`` singleton, so a hold started here keeps running
   while the operator works on other pages.
"""

from __future__ import annotations

import logging
import threading

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
    QPushButton, QScrollArea, QSpinBox, QSplitter, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf, scaled_font_size
from gui.styles import COLORS
from gui.widgets.incubator_widgets import (
    Banner, HeaterBridge, StatePill, TempTrendPlot, ZoneCard, _c, _hex,
    mono_font,
)

from SupportClasses.incubator import device_config
from SupportClasses.incubator.config_store import get_store
from SupportClasses.incubator.service import (
    connect_from_store, get_incubator, make_exclusion_provider,
    make_poll_gate, make_zp_getter,
)
from SupportClasses.incubator.power_staircase import (
    plan_staircase, verdict_lines,
)
from SupportClasses.incubator.stability import format_duration
from SupportClasses.incubator.zones import ALL_ZONES, zone_by_id

logger = logging.getLogger(__name__)

#: Seconds between automatic session attempts, so a down ZP board is not
#: hammered with a probe on every 300 ms status tick.
_AUTO_RETRY_S = 3.0


class _ConnBridge(QObject):
    """Marshals worker-thread connect/detect outcomes onto the GUI thread."""

    connect_done = Signal(bool)
    detect_done = Signal(object)   # DetectedBoard | None


class IncubatorWorkflowPage(QWidget):
    """Incubator control page (Workflows tile)."""

    back_requested = Signal()

    def __init__(self, controller=None, settings=None, camera_manager=None,
                 parent: QWidget | None = None, *, incubator=None, store=None):
        super().__init__(parent)
        self._stage_controller = controller
        self._settings = settings
        self._store = store if store is not None else get_store()
        self.ctrl = incubator if incubator is not None else get_incubator()
        # The scan/detect on THIS controller must never open a port the app's
        # stages own (opening DTR-resets the board behind it). ONE shared
        # definition of that rule lives in the service module.
        self._reserved_ports = make_exclusion_provider(controller)
        self._zp_getter = make_zp_getter(controller)
        self._shared_poll_gate = make_poll_gate(controller)
        self.ctrl.exclude_ports_provider = self._reserved_ports

        self.bridge = HeaterBridge()
        self._conn_bridge = _ConnBridge()
        self._conn_bridge.connect_done.connect(self._on_connect_done)
        self._conn_bridge.detect_done.connect(self._on_detect_done)
        self._connecting = False
        self._detecting = False
        #: monotonic stamp of the last AUTO attempt (throttles retries).
        self._last_auto_attempt = 0.0
        self._last_at: tuple[str, object] | None = None
        self._probe_flagged = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(16), s(12), s(16), s(12))
        outer.setSpacing(s(9))

        # ── header: back + title ──
        header = QHBoxLayout()
        header.setSpacing(s(8))
        back_btn = QPushButton("← Back to Workflows")
        back_btn.setCursor(Qt.PointingHandCursor)
        back_btn.clicked.connect(self.back_requested.emit)
        header.addWidget(back_btn)
        title = QLabel("Incubator")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)
        outer.addLayout(header)

        outer.addWidget(self._build_connection_bar())
        self._banner = Banner()
        outer.addWidget(self._banner)

        split = QSplitter(Qt.Vertical)
        split.addWidget(self._build_upper())
        split.addWidget(self._build_lower())
        split.setStretchFactor(0, 6)
        split.setStretchFactor(1, 3)
        split.setSizes([s(560), s(260)])
        outer.addWidget(split, 1)

        outer.addWidget(self._build_bottom_bar())

        self._wire()
        self._apply_store_prefs()

        # Seed from whatever the controller already knows: the probe fires
        # during connect(), so a page shown against an already-connected
        # controller (Connect card row, or a previous visit) must render the
        # session, not an empty shell.
        self._set_connected_ui(self.ctrl.connected)
        if self.ctrl.report is not None:
            self._on_probe(self.ctrl.report)
        if self.ctrl.connected:
            self._on_pid({
                z.zone_id: self.ctrl.zone_runtime(z.zone_id).pid
                for z in ALL_ZONES
            })
            self._on_channels(self.ctrl.hub.all_channels())
            self._status.setText(
                f"Connected on {self.ctrl.active_port or 'simulator'}.")

        self._tick = QTimer(self)
        self._tick.setInterval(1000)
        self._tick.timeout.connect(self._on_tick)
        self._tick.start()

    # ── workflow-page contract ──────────────────────────────────────

    def get_page_title(self) -> str:
        return "Incubator"

    def get_context_widget(self):
        return None

    def on_status_update(self) -> None:
        """MainWindow ~300 ms tick (fires only while this page is visible).

        All live data arrives through the HeaterBridge; this keeps the
        transport hint honest and — the point of the no-connect-button
        design — joins the ZP board automatically once it comes up
        (throttled inside _ensure_session).
        """
        try:
            self._refresh_transport_hint()
            self._ensure_session()
        except Exception:
            logger.debug("incubator status tick failed", exc_info=True)

    # ── construction ────────────────────────────────────────────────

    def _build_connection_bar(self) -> QWidget:
        """Status strip, not a connect surface.

        The heaters live on the ZP board, so the ZP Connect (Hardware Setup
        → Device / the Connect Hardware card) IS the incubator connect and
        this page follows it automatically — a second button for the same
        board was operator-rejected. The manual Port/Detect/Connect controls
        exist ONLY for the dedicated-serial transport (the planned ESP32
        sensor board) and stay hidden until that transport is chosen on
        Hardware Setup → Incubator.
        """
        box = QGroupBox("Board connection")
        lay = QHBoxLayout(box)
        lay.setContentsMargins(s(12), s(14), s(12), s(12))
        lay.setSpacing(s(8))

        self._conn_pill = StatePill("DISCONNECTED", "overlay0")
        lay.addWidget(self._conn_pill)

        self._transport_hint = QLabel("")
        self._transport_hint.setWordWrap(True)
        self._transport_hint.setStyleSheet(
            f"color:{_hex('overlay0', '#6c7086')};font-size:{sf(8.5)}pt;")
        lay.addWidget(self._transport_hint, 1)

        # ── dedicated-serial controls (future ESP32 board) — hidden on the
        #    shared/simulator transports ──
        self._port_lbl = QLabel("Port")
        lay.addWidget(self._port_lbl)
        self._port = QComboBox()
        self._port.setEditable(True)
        self._port.setMinimumWidth(s(170))
        lay.addWidget(self._port)

        self._rescan_btn = QPushButton("Rescan")
        self._rescan_btn.setToolTip("Re-enumerate serial ports")
        self._rescan_btn.clicked.connect(self._refresh_ports)
        lay.addWidget(self._rescan_btn)

        self._detect_btn = QPushButton("Detect board")
        self._detect_btn.setToolTip(
            "Ask each serial port for its firmware identity (M115) and select "
            "the one that answers as Marlin. The ports the app's stages own "
            "are skipped — opening one would reset the board behind it."
        )
        self._detect_btn.clicked.connect(self._on_detect)
        lay.addWidget(self._detect_btn)

        self._baud_lbl = QLabel("Baud")
        lay.addWidget(self._baud_lbl)
        self._baud = QComboBox()
        self._baud.setEditable(True)
        for b in device_config.COMMON_BAUDS:
            self._baud.addItem(str(b))
        self._baud.setCurrentText(str(device_config.DEFAULT_BAUD))
        lay.addWidget(self._baud)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setObjectName("primaryButton")
        self._connect_btn.setMinimumWidth(s(110))
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        lay.addWidget(self._connect_btn)

        self._fw = QLabel("")
        self._fw.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(self._fw)

        self._refresh_ports()
        return box

    def _build_upper(self) -> QWidget:
        """Zone cards | (trend over sensor table), every boundary draggable.

        Both dividers are splitters rather than fixed sizes because the
        right-hand column used to swallow every spare pixel — on a wide
        screen that rendered the trend as a very wide, very short strip
        (operator: *"too wide … it needs to be resizable"*). The horizontal
        divider hands that width back to the zone cards; the vertical one
        trades plot height against the sensor table, which could not grow
        at all before (it was pinned to a fixed height).
        """
        wrap = QSplitter(Qt.Horizontal)
        # Collapsing a pane to zero leaves a control surface the operator
        # cannot get back without knowing where the handle is.
        wrap.setChildrenCollapsible(False)

        cards = QWidget()
        cl = QVBoxLayout(cards)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(s(10))
        self._cards: dict[str, ZoneCard] = {}
        for spec in ALL_ZONES:
            card = ZoneCard(
                spec, self.ctrl,
                ramp_step_getter=lambda: float(
                    self._store.get("ramp_step_c", 3.0)),
            )
            self._cards[spec.zone_id] = card
            cl.addWidget(card)
        cl.addStretch(1)

        # Scroll rather than squeeze: a zone showing a fault message needs
        # more height than one that is fine.
        scroll = QScrollArea()
        scroll.setWidget(cards)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)
        # Was a FIXED width; now a floor, so the splitter can widen it.
        scroll.setMinimumWidth(s(372))
        wrap.addWidget(scroll)

        right = QSplitter(Qt.Vertical)
        right.setChildrenCollapsible(False)

        trend = QGroupBox("Temperature trend")
        tl = QVBoxLayout(trend)
        tl.setContentsMargins(s(10), s(14), s(10), s(10))
        ctlrow = QHBoxLayout()
        ctlrow.addWidget(QLabel("Window"))
        self._window = QComboBox()
        for label, secs in (("5 min", 300), ("15 min", 900), ("1 hour", 3600),
                            ("6 hours", 21600)):
            self._window.addItem(label, secs)
        self._window.setCurrentIndex(1)
        self._window.currentIndexChanged.connect(
            lambda: self._plot.set_window_s(float(self._window.currentData()))
        )
        ctlrow.addWidget(self._window)
        self._duty_chk = QCheckBox("Show heater duty")
        self._duty_chk.setChecked(True)
        self._duty_chk.toggled.connect(lambda v: self._plot.set_show_duty(v))
        ctlrow.addWidget(self._duty_chk)
        ctlrow.addStretch(1)
        self._log_btn = QPushButton("Start logging")
        self._log_btn.setCheckable(True)
        self._log_btn.setToolTip(
            "Record every sample to logs/incubator/ as JSONL")
        self._log_btn.toggled.connect(self._on_log_toggled)
        ctlrow.addWidget(self._log_btn)
        tl.addLayout(ctlrow)
        self._plot = TempTrendPlot()
        tl.addWidget(self._plot, 1)
        right.addWidget(trend)

        sensors = QGroupBox("Sensor channels")
        sl = QVBoxLayout(sensors)
        sl.setContentsMargins(s(10), s(14), s(10), s(10))

        srow = QHBoxLayout()
        srow.addStretch(1)
        self._rescan_all_btn = QPushButton("Rescan sensors")
        self._rescan_all_btn.setToolTip(
            "Re-read every sensor (one M105) and re-evaluate which zones may "
            "be heated.\nPress after plugging a thermistor in — the firmware "
            "picks it up straight away, but the verdict is cached from "
            "connect time.\nNo heater is touched and no setpoint changes."
        )
        self._rescan_all_btn.clicked.connect(self.ctrl.rescan_sensors)
        srow.addWidget(self._rescan_all_btn)
        sl.addLayout(srow)

        self._table = QTableWidget(0, 6)
        self._table.setHorizontalHeaderLabels(
            ["Channel", "Value", "Raw", "Target", "Duty", "State"]
        )
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.NoSelection)
        self._table.setAlternatingRowColors(True)
        # Was FIXED; a floor instead, so dragging the divider down actually
        # shows more rows rather than leaving the extra height empty.
        self._table.setMinimumHeight(s(124))
        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.Stretch)
        for _col in range(1, 6):
            hdr.setSectionResizeMode(_col, QHeaderView.ResizeToContents)
        sl.addWidget(self._table)
        right.addWidget(sensors)
        right.setStretchFactor(0, 3)   # extra height goes to the plot
        right.setStretchFactor(1, 0)
        right.setSizes([s(360), s(170)])

        wrap.addWidget(right)
        wrap.setStretchFactor(0, 0)
        wrap.setStretchFactor(1, 1)
        wrap.setSizes([s(372), s(720)])
        return wrap

    def _build_lower(self) -> QWidget:
        tabs = QTabWidget()
        tabs.addTab(self._build_health_tab(), "Stability && health")
        tabs.addTab(self._build_pid_tab(), "PID")
        tabs.addTab(self._build_autotune_tab(), "Autotune")
        tabs.addTab(self._build_staircase_tab(), "Power staircase")
        tabs.addTab(self._build_calibration_tab(), "Sensor calibration")
        tabs.addTab(self._build_firmware_tab(), "Firmware")
        tabs.addTab(self._build_console_tab(), "Console")
        return tabs

    def _build_bottom_bar(self) -> QWidget:
        bar = QWidget()
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(s(2), 0, s(2), 0)
        lay.setSpacing(s(8))

        self._status = QLabel("Not connected.")
        self._status.setWordWrap(True)
        lay.addWidget(self._status, 1)

        self._all_off = QPushButton("ALL HEATERS OFF")
        self._all_off.setObjectName("warnButton")
        self._all_off.setToolTip(
            "Set every zone's target to 0 (M140 S0 / M104 S0)")
        self._all_off.setMinimumWidth(s(150))
        self._all_off.clicked.connect(self.ctrl.all_heaters_off)
        lay.addWidget(self._all_off)

        self._estop = QPushButton("EMERGENCY STOP")
        self._estop.setObjectName("dangerButton")
        self._estop.setMinimumWidth(s(150))
        self._estop.setToolTip(
            "M112. HALTS the board — it ignores everything until "
            "power-cycled. For a normal stop use ALL HEATERS OFF."
        )
        self._estop.clicked.connect(self._on_estop)
        lay.addWidget(self._estop)
        return bar

    def _build_health_tab(self) -> QWidget:
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setSpacing(s(10))
        self._health: dict[str, dict[str, QLabel]] = {}
        rows = [
            ("temp", "Temperature"), ("error", "Error"),
            ("rate", "Rate of change"), ("eta", "Time to setpoint"),
            ("settle", "Settle time"), ("overshoot", "Overshoot"),
            ("ripple", "Steady ripple"), ("duty", "Steady-state duty"),
            ("tau", "Thermal time constant"),
        ]
        for spec in ALL_ZONES:
            box = QGroupBox(spec.title)
            g = QGridLayout(box)
            g.setContentsMargins(s(12), s(14), s(12), s(12))
            g.setColumnStretch(1, 1)
            fields: dict[str, QLabel] = {}
            for i, (key, label) in enumerate(rows):
                lbl = QLabel(label)
                lbl.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
                g.addWidget(lbl, i, 0)
                val = QLabel("—")
                val.setStyleSheet("font-weight:600;")
                fields[key] = val
                g.addWidget(val, i, 1)
            note = QLabel("")
            note.setWordWrap(True)
            note.setStyleSheet(
                f"color:{_hex('subtext0', '#a6adc8')};padding-top:{s(6)}px;")
            g.addWidget(note, len(rows), 0, 1, 2)
            fields["note"] = note
            self._health[spec.zone_id] = fields
            lay.addWidget(box)
        return w

    def _build_pid_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "These are the constants the BOARD is using. Marlin has no "
            "\"get PID\" command, so they are read from the M503 settings "
            "dump. If a zone's line is absent, PID is not compiled in for "
            "that heater and it runs bang-bang."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        row = QHBoxLayout()
        row.setSpacing(s(10))
        self._pid_fields: dict[str, dict] = {}
        for spec in ALL_ZONES:
            box = QGroupBox(f"{spec.title}   ({spec.pid_cmd})")
            g = QGridLayout(box)
            g.setContentsMargins(s(12), s(14), s(12), s(12))
            g.addWidget(QLabel("On board"), 0, 1)
            g.addWidget(QLabel("New value"), 0, 2)
            entry: dict = {}
            for i, key in enumerate(("kp", "ki", "kd"), start=1):
                lbl = QLabel(key.capitalize())
                lbl.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
                g.addWidget(lbl, i, 0)
                cur = QLabel("—")
                cur.setStyleSheet("font-weight:600;")
                g.addWidget(cur, i, 1)
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 10000.0)
                spin.setDecimals(2)
                g.addWidget(spin, i, 2)
                entry[key] = cur
                entry[f"{key}_spin"] = spin
            apply_btn = QPushButton("Apply to board (RAM)")
            apply_btn.clicked.connect(
                lambda _=False, z=spec.zone_id: self._on_apply_pid(z)
            )
            g.addWidget(apply_btn, 4, 0, 1, 3)
            status = QLabel("")
            status.setWordWrap(True)
            g.addWidget(status, 5, 0, 1, 3)
            entry["status"] = status
            entry["apply"] = apply_btn
            self._pid_fields[spec.zone_id] = entry
            row.addWidget(box)
        lay.addLayout(row)

        btns = QHBoxLayout()
        for text, slot, tip in (
            ("Refresh from board (M503)", self.ctrl.query_pid,
             "Re-read the board's settings"),
            ("Save to EEPROM (M500)", self._on_save_eeprom,
             "Persist current values across power cycles"),
            ("Reload EEPROM (M501)", self._on_load_eeprom,
             "Discard unsaved changes and reload stored values"),
            ("Factory reset (M502)", self._on_factory_reset,
             "Load firmware defaults into RAM (not saved until M500)"),
        ):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            btns.addWidget(b)
        btns.addStretch(1)
        lay.addLayout(btns)
        lay.addStretch(1)
        return w

    def _build_autotune_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "PID autotune (M303) drives the heater through heat/cool cycles "
            "and computes constants. On a large water mass a cycle can exceed "
            "Marlin's internal time limit and fail with \"timeout\" — a "
            "firmware limit, not a fault here. Results apply to RAM only; "
            "save to EEPROM to keep them.\n"
            "NOTE: autotune is unavailable over the shared ZP link (it would "
            "monopolise the motion board's serial channel for its whole "
            "duration) — set the PID manually, or run it over a dedicated "
            "connection with the ZP board disconnected."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        row = QHBoxLayout()
        row.addWidget(QLabel("Zone"))
        self._at_zone = QComboBox()
        for spec in ALL_ZONES:
            self._at_zone.addItem(spec.title, spec.zone_id)
        row.addWidget(self._at_zone)
        row.addWidget(QLabel("Target"))
        self._at_target = QDoubleSpinBox()
        self._at_target.setRange(0.0, float(self.ctrl.MAX_SETPOINT_C))
        self._at_target.setValue(37.0)
        self._at_target.setSuffix(" °C")
        row.addWidget(self._at_target)
        row.addWidget(QLabel("Cycles"))
        self._at_cycles = QSpinBox()
        self._at_cycles.setRange(1, 20)
        self._at_cycles.setValue(3)
        row.addWidget(self._at_cycles)
        self._at_apply = QCheckBox("Apply result (U1)")
        self._at_apply.setChecked(True)
        row.addWidget(self._at_apply)
        self._at_start = QPushButton("Start autotune")
        self._at_start.setObjectName("primaryButton")
        self._at_start.clicked.connect(self._on_autotune_start)
        row.addWidget(self._at_start)
        self._at_cancel = QPushButton("Cancel (M108)")
        self._at_cancel.clicked.connect(self.ctrl.cancel_autotune)
        row.addWidget(self._at_cancel)
        row.addStretch(1)
        lay.addLayout(row)

        self._at_status = QLabel("Idle.")
        self._at_status.setWordWrap(True)
        lay.addWidget(self._at_status)

        self._at_result = QPlainTextEdit()
        self._at_result.setReadOnly(True)
        lay.addWidget(self._at_result, 1)

        self._at_adopt = QPushButton("Copy result into the PID tab")
        self._at_adopt.setEnabled(False)
        self._at_adopt.clicked.connect(self._on_adopt_result)
        lay.addWidget(self._at_adopt)
        return w

    def _build_staircase_tab(self) -> QWidget:
        """Round 6: how much bed power can this rig actually carry?

        Bench 2026-08-17: the board leaves USB 0.45 s after ``M140 S37``,
        four attempts out of four, and never once with the heater off. The
        command path is fine and the bed heats — the question is the power
        level the link survives. Same engine as
        ``tools_incubator_heater_diagnostic.py --staircase``; this runs it
        in-app, over whatever transport is live.
        """
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "Steps the heater power up a rung at a time and reports the "
            "highest level the link survives. Use it when the board drops "
            "off USB as the heater starts, rather than failing to heat.\n"
            "Peak-limited holds Marlin's own duty down (pure-P bed PID, "
            "restored afterwards, never saved to EEPROM). Average-limited "
            "gates full-power bursts on and off — every burst is full "
            "current. If peak-limited survives where average-limited dies, "
            "the fault is inrush current."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        self._sc_warn = QLabel(
            "⚠ This test is EXPECTED to reset the board when it finds the "
            "level the supply cannot carry — that is the measurement. On the "
            "shared link that same board runs Z and the pumps, so retract "
            "the needle first and expect to re-declare the Z position "
            "afterwards. It refuses to start while a print is running."
        )
        self._sc_warn.setWordWrap(True)
        self._sc_warn.setStyleSheet(
            f"color:{_hex('yellow', '#f9e2af')};font-size:{sf(8.5)}pt;")
        lay.addWidget(self._sc_warn)

        row = QHBoxLayout()
        row.addWidget(QLabel("Zone"))
        self._sc_zone = QComboBox()
        for spec in ALL_ZONES:
            self._sc_zone.addItem(spec.title, spec.zone_id)
        row.addWidget(self._sc_zone)

        row.addWidget(QLabel("Mode"))
        self._sc_mode = QComboBox()
        self._sc_mode.addItem("Peak-limited (PID)", "pid")
        self._sc_mode.addItem("Average-limited (slow PWM)", "pwm")
        self._sc_mode.setToolTip(
            "Peak-limited reduces the current the supply ever sees.\n"
            "Average-limited keeps full-current bursts and reduces only the "
            "average — the pair separates inrush from capacity.")
        row.addWidget(self._sc_mode)

        row.addWidget(QLabel("Rungs %"))
        self._sc_rungs = QLineEdit("10,25,50,75,100")
        self._sc_rungs.setMaximumWidth(s(150))
        self._sc_rungs.setToolTip(
            "Percentages of full heater power, tried in ascending order.")
        row.addWidget(self._sc_rungs)

        row.addWidget(QLabel("s/rung"))
        self._sc_seconds = QDoubleSpinBox()
        self._sc_seconds.setRange(2.0, 120.0)
        self._sc_seconds.setValue(12.0)
        self._sc_seconds.setDecimals(0)
        row.addWidget(self._sc_seconds)
        row.addStretch(1)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Ceiling"))
        self._sc_ceiling = QDoubleSpinBox()
        self._sc_ceiling.setRange(1.0, float(self.ctrl.MAX_SETPOINT_C))
        self._sc_ceiling.setValue(min(45.0, float(self.ctrl.MAX_SETPOINT_C)))
        self._sc_ceiling.setSuffix(" °C")
        self._sc_ceiling.setToolTip(
            "The target the rungs drive against. It needs headroom above the "
            "current temperature or the firmware has no error to drive and "
            "every rung reads 0%.")
        row2.addWidget(self._sc_ceiling)

        row2.addWidget(QLabel("Abort above"))
        self._sc_abort = QDoubleSpinBox()
        self._sc_abort.setRange(1.0, 95.0)
        self._sc_abort.setValue(45.0)
        self._sc_abort.setSuffix(" °C")
        row2.addWidget(self._sc_abort)

        self._sc_start = QPushButton("Run staircase")
        self._sc_start.setObjectName("primaryButton")
        self._sc_start.clicked.connect(self._on_staircase_start)
        row2.addWidget(self._sc_start)
        self._sc_stop = QPushButton("Stop")
        self._sc_stop.setEnabled(False)
        self._sc_stop.clicked.connect(self.ctrl.cancel_power_staircase)
        row2.addWidget(self._sc_stop)
        row2.addStretch(1)
        lay.addLayout(row2)

        self._sc_log = QPlainTextEdit()
        self._sc_log.setReadOnly(True)
        self._sc_log.setMaximumBlockCount(3000)
        self._sc_log.setFont(mono_font(scaled_font_size(9)))
        self._sc_log.setPlaceholderText(
            "Results appear here — the same report the bench tool prints.")
        lay.addWidget(self._sc_log, 1)

        btns = QHBoxLayout()
        btns.addStretch(1)
        copy = QPushButton("Copy report")
        copy.clicked.connect(self._on_staircase_copy)
        btns.addWidget(copy)
        clear = QPushButton("Clear")
        clear.clicked.connect(self._sc_log.clear)
        btns.addWidget(clear)
        lay.addLayout(btns)
        return w

    def _on_staircase_start(self) -> None:
        zone_id = self._sc_zone.currentData()
        spec = zone_by_id(zone_id)
        mode = self._sc_mode.currentData()
        try:
            rungs = plan_staircase(self._sc_rungs.text())
        except ValueError as e:
            QMessageBox.warning(self, "Rungs", str(e))
            return

        # Naming the consequence, not just asking. The board this resets is
        # the motion board on the shared link.
        shared = self.ctrl.shared_transport
        msg = (
            f"Run a power staircase on {spec.title}?\n\n"
            f"Rungs: {rungs} % of full power, "
            f"{self._sc_seconds.value():.0f}s each\n"
            f"Mode: {self._sc_mode.currentText()}\n"
            f"Ceiling {self._sc_ceiling.value():.0f} °C, aborts above "
            f"{self._sc_abort.value():.0f} °C\n\n"
            f"This is EXPECTED to reset the board when it reaches a level "
            f"the supply cannot carry — that is the measurement."
        )
        if shared:
            msg += (
                "\n\nThat board also runs Z and the pumps. Retract the needle "
                "first; after a reset the Z position must be re-declared."
            )
        if mode == "pid":
            msg += ("\n\nThe bed PID is temporarily set to pure proportional "
                    "and put back afterwards. Nothing is saved to EEPROM.")
        if QMessageBox.question(
                self, "Power staircase", msg,
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No) != QMessageBox.Yes:
            return

        self._sc_log.appendPlainText(
            f"=== staircase: {spec.title}, {mode}, rungs {rungs}, "
            f"{self._sc_seconds.value():.0f}s each, ceiling "
            f"{self._sc_ceiling.value():.0f} C ===")
        self._sc_log.appendPlainText(
            "  rung   t(s)   sensor C   target   duty   %power")
        started = self.ctrl.start_power_staircase(
            zone_id, rungs=rungs, rung_seconds=self._sc_seconds.value(),
            ceiling_c=self._sc_ceiling.value(),
            abort_above_c=self._sc_abort.value(), mode=mode,
        )
        if not started:
            # start_power_staircase already published WHY on the status line.
            self._sc_log.appendPlainText("  (refused — see the status line)")
            return
        self._sc_start.setEnabled(False)
        self._sc_stop.setEnabled(True)

    def _on_staircase_row(self, row: dict) -> None:
        tgt = row.get("target_c")
        self._sc_log.appendPlainText(
            f"  {row['pct']:4d}   {row['elapsed_s']:4.1f}   "
            f"{row['temp_c']:8.2f}   "
            f"{(f'{tgt:6.1f}' if tgt is not None else '     ?')}   "
            f"{row['duty']:4d}   {row['duty_pct']:5.0f}%")

    def _on_staircase_done(self, zone_id: str, outcome) -> None:
        self._sc_start.setEnabled(True)
        self._sc_stop.setEnabled(False)
        self._sc_log.appendPlainText("")
        self._sc_log.appendPlainText("--- VERDICT ---")
        for line in verdict_lines(outcome):
            self._sc_log.appendPlainText("  " + line if line else "")
        self._sc_log.appendPlainText("")

    def _on_staircase_copy(self) -> None:
        from PySide6.QtWidgets import QApplication
        QApplication.clipboard().setText(self._sc_log.toPlainText())
        self._status.setText("Staircase report copied to the clipboard.")

    def _build_calibration_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "SENSOR calibration corrects what the board reports against a "
            "trusted external thermometer. It is applied on the host, at the "
            "display and setpoint-translation layer only — the firmware's "
            "thermistor tables are never touched. This is a different thing "
            "from PID autotune, which tunes the control loop.\n\n"
            "Put a reference thermometer next to the sensor, wait for both "
            "to settle, then enter the reference reading."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        self._cal_fields: dict[str, dict] = {}
        for spec in ALL_ZONES:
            box = QGroupBox(spec.title)
            g = QHBoxLayout(box)
            g.setContentsMargins(s(12), s(14), s(12), s(12))
            g.addWidget(QLabel("Board reads"))
            board = QLabel("—")
            board.setStyleSheet("font-weight:600;")
            g.addWidget(board)
            g.addSpacing(s(12))
            g.addWidget(QLabel("Reference reads"))
            ref = QDoubleSpinBox()
            ref.setRange(0.0, 120.0)
            ref.setDecimals(2)
            ref.setValue(37.0)
            ref.setSuffix(" °C")
            g.addWidget(ref)
            apply_btn = QPushButton("Apply offset")
            apply_btn.clicked.connect(
                lambda _=False, z=spec.zone_id: self._on_calibrate(z)
            )
            g.addWidget(apply_btn)
            clear = QPushButton("Clear")
            clear.clicked.connect(
                lambda _=False, z=spec.zone_id: self.ctrl.clear_calibration(z)
            )
            g.addWidget(clear)
            state = QLabel("uncalibrated")
            state.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
            g.addWidget(state, 1)
            self._cal_fields[spec.zone_id] = {
                "board": board, "ref": ref, "state": state,
            }
            lay.addWidget(box)
        lay.addStretch(1)
        return w

    def _build_firmware_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        self._fw_text = QPlainTextEdit()
        self._fw_text.setReadOnly(True)
        self._fw_text.setFont(mono_font(scaled_font_size(9)))
        self._fw_text.setPlainText(
            "Connect to probe the firmware.\n\n"
            "The probe asks the board what it actually supports (M115, M105, "
            "M503) instead of assuming, and names the exact Configuration.h "
            "option for anything missing."
        )
        lay.addWidget(self._fw_text)
        return w

    def _build_console_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        self._console_warn = QLabel(
            "⚠ Shared ZP link: this console talks to the MOTION board — "
            "G-code typed here can move the stage or change its settings."
        )
        self._console_warn.setWordWrap(True)
        self._console_warn.setStyleSheet(
            f"color:{_hex('yellow', '#f9e2af')};font-size:{sf(8.5)}pt;")
        self._console_warn.setVisible(False)
        lay.addWidget(self._console_warn)

        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(4000)
        self._console.setFont(mono_font(scaled_font_size(9)))
        lay.addWidget(self._console, 1)

        row = QHBoxLayout()
        self._show_temps = QCheckBox("Show temperature lines")
        self._show_temps.setToolTip(
            "Temperature polls answer once per round; hide them to see other "
            "traffic.")
        row.addWidget(self._show_temps)
        clear = QPushButton("Clear")
        clear.clicked.connect(self._console.clear)
        row.addWidget(clear)
        row.addStretch(1)
        row.addWidget(QLabel("Send G-code"))
        self._raw_in = QLineEdit()
        self._raw_in.setPlaceholderText("e.g. M105")
        self._raw_in.returnPressed.connect(self._on_send_raw)
        self._raw_in.setMinimumWidth(s(240))
        row.addWidget(self._raw_in)
        send = QPushButton("Send")
        send.clicked.connect(self._on_send_raw)
        row.addWidget(send)
        lay.addLayout(row)
        return w

    # ── wiring ──────────────────────────────────────────────────────

    def _wire(self) -> None:
        b = self.bridge
        b.attach(self.ctrl)
        b.channels.connect(self._on_channels)
        b.zone_state.connect(self._on_zone_state)
        b.raw_line.connect(self._on_raw_line)
        b.pid_updated.connect(self._on_pid)
        b.autotune_progress.connect(self._on_at_progress)
        b.autotune_done.connect(self._on_at_done)
        b.fault.connect(self._on_fault)
        b.divergence.connect(self._on_divergence)
        b.connection_changed.connect(self._on_connection)
        b.probe_done.connect(self._on_probe)
        b.status.connect(self._on_status)
        b.ramp.connect(self._on_ramp)
        b.staircase_row.connect(self._on_staircase_row)
        b.staircase_done.connect(self._on_staircase_done)

    # ── store preferences ───────────────────────────────────────────

    def _apply_store_prefs(self, *, seed_presets: bool = True) -> None:
        """Sync transport / zone naming / ceiling / dither prefs from the
        per-machine store (Hardware Setup → Incubator edits it).

        ``seed_presets=False`` on a revisit while connected: re-seeding the
        setpoint spin would silently overwrite whatever the operator typed
        mid-session every time they navigate back to this page.
        """
        store = self._store
        saved_port = store.get("dedicated_port", "")
        if saved_port and not self._port.currentText():
            self._port.setCurrentText(saved_port)
        try:
            self._baud.setCurrentText(str(int(store.get("dedicated_baud",
                                                        38400))))
        except Exception:
            pass

        # Ceiling can have been lowered on the Hardware Setup tab.
        from SupportClasses.incubator.service import apply_store_config
        apply_store_config(self.ctrl)

        for spec in ALL_ZONES:
            card = self._cards.get(spec.zone_id)
            if card is None:
                continue
            z = store.zone(spec.zone_id)
            label = str(z.get("label") or "").strip()
            card.setTitle(label or spec.title)
            card.setVisible(bool(z.get("enabled", True)))
            card.set_fine_period(store.get("fine_period_s", 60))
            if seed_presets:
                card.set_preset_value(z.get("preset_c", 37.0))
            card.refresh_ceiling()
        self._at_target.setRange(0.0, float(self.ctrl.MAX_SETPOINT_C))
        self._refresh_transport_row()

    def showEvent(self, event) -> None:
        super().showEvent(event)
        # The Hardware Setup tab may have edited the store since last shown.
        # Presets re-seed only while idle — a live session keeps whatever
        # setpoint the operator typed.
        try:
            self._apply_store_prefs(seed_presets=not self.ctrl.connected)
        except Exception:
            logger.debug("incubator store prefs re-apply failed",
                         exc_info=True)
        # Join the board straight away on entry (no throttle on a fresh
        # visit) — this page has no connect button of its own.
        try:
            self._ensure_session(force=True)
        except Exception:
            logger.debug("incubator ensure-session failed", exc_info=True)

    # ── connection ──────────────────────────────────────────────────

    def _current_transport(self) -> str:
        """The SAVED transport — Hardware Setup → Incubator owns the choice."""
        return str(self._store.get("transport", "shared") or "shared")

    def _refresh_transport_row(self) -> None:
        t = self._current_transport()
        serial_mode = (t == "serial")
        for wdg in (self._port_lbl, self._port, self._rescan_btn,
                    self._detect_btn, self._baud_lbl, self._baud,
                    self._connect_btn):
            wdg.setVisible(serial_mode)
        self._console_warn.setVisible(t == "shared")
        self._refresh_transport_hint()

    def _refresh_transport_hint(self) -> None:
        """One sentence naming how this page gets (or got) its board."""
        t = self._current_transport()
        if self.ctrl.connected:
            live = {
                "shared": "Riding the ZP board connection — the heaters are "
                          "wired to the same board that drives Z + pumps.",
                "simulated": "Running against the built-in thermal simulator.",
                "serial": "Connected to a dedicated incubator board.",
            }.get(self.ctrl.transport, "")
            self._transport_hint.setText(live)
            return
        if t == "shared":
            zp = self._zp_manager()
            if zp is None or getattr(zp, "serial", None) is None:
                self._transport_hint.setText(
                    "Follows the ZP board — connect the ZP board on "
                    "Hardware Setup → Device (or the Connect Hardware card) "
                    "and this page joins it automatically."
                )
            elif getattr(zp, "simulate", False):
                self._transport_hint.setText(
                    "The ZP board is SIMULATED (its simulator has no "
                    "heaters). Pick the Simulator transport on Hardware "
                    "Setup → Incubator for no-hardware work."
                )
            else:
                self._transport_hint.setText("Joining the ZP board link…")
        elif t == "simulate":
            self._transport_hint.setText("Starting the thermal simulator…")
        else:
            self._transport_hint.setText(
                "Dedicated incubator board (future ESP32 sensor box) — "
                "pick its port and press Connect."
            )

    def _ensure_session(self, *, force: bool = False) -> None:
        """Keep the session matching the saved transport — automatically.

        The shared and simulator transports have NO connect button (the ZP
        Connect is the connect; a simulator start is not a decision worth a
        click); this runs from showEvent and the ~300 ms status tick,
        throttled to one attempt per _AUTO_RETRY_S so a down ZP board is
        not hammered with probes. The dedicated-serial transport is left
        strictly manual.
        """
        import time as _time
        if self._connecting:
            return
        t = self._current_transport()

        # A stale SIMULATOR session may be auto-retired when the operator
        # switches the transport away — it is the one transport with no real
        # heaters behind it. A shared/serial session is never torn down
        # automatically (disconnecting would not stop the firmware's hold,
        # only blind us to it).
        if (self.ctrl.connected and self.ctrl.transport == "simulated"
                and t != "simulate"):
            self._start_conn_worker(lambda: self.ctrl.disconnect() or True,
                                    label="retiring the simulator session")
            return

        if self.ctrl.connected or t == "serial":
            return
        now = _time.monotonic()
        if not force and (now - self._last_auto_attempt) < _AUTO_RETRY_S:
            return

        if t == "shared":
            zp = self._zp_manager()
            if (zp is None or getattr(zp, "simulate", False)
                    or getattr(zp, "serial", None) is None):
                return          # nothing to join yet; the hint says so
        self._last_auto_attempt = now
        sc = self._stage_controller
        self._start_conn_worker(
            lambda: connect_from_store(stage_controller=sc),
            label="joining the board")

    def _start_conn_worker(self, fn, *, label: str) -> None:
        """Run one blocking session operation on a daemon worker."""
        if self._connecting:
            return
        self._connecting = True
        self._conn_pill.set_state("CONNECTING", "yellow")
        self._status.setText(f"Incubator: {label}…")

        def work():
            ok = False
            try:
                ok = bool(fn())
            except Exception:
                logger.warning("incubator session op failed", exc_info=True)
            self._conn_bridge.connect_done.emit(ok)

        threading.Thread(target=work, daemon=True,
                         name="incubator-session").start()

    def _zp_manager(self):
        return self._zp_getter()

    def _refresh_ports(self) -> None:
        current = self._port.currentText()
        self._port.clear()
        try:
            ranked = device_config.ranked_ports(
                exclude_ports=self._reserved_ports())
        except Exception:
            ranked = []
        for dev, score, _desc in ranked:
            hint = "  (likely the board)" if score >= 100 else ""
            self._port.addItem(f"{dev}{hint}", dev)
        if current:
            self._port.setCurrentText(current)

    def _selected_port(self) -> str:
        data = self._port.currentData()
        if data:
            return str(data)
        return (self._port.currentText().split()[0]
                if self._port.currentText() else "")

    def _on_detect(self) -> None:
        """Scan for a Marlin board — on a worker thread (0.6 s per port per
        baud adds up; blocking the GUI thread here is the freeze class the
        GuiWatchdog exists to catch)."""
        if self._detecting or self._connecting:
            return
        self._detecting = True
        self._detect_btn.setEnabled(False)
        self._status.setText("Scanning for a Marlin board…")

        def work():
            found = None
            try:
                found = self.ctrl.detect_board()
            except Exception:
                logger.warning("incubator detect failed", exc_info=True)
            self._conn_bridge.detect_done.emit(found)

        threading.Thread(target=work, daemon=True,
                         name="incubator-detect").start()

    def _on_detect_done(self, found) -> None:
        self._detecting = False
        self._detect_btn.setEnabled(True)
        self._refresh_ports()
        if found is None:
            self._banner.show_message(
                "No Marlin board found on any free serial port. Check the "
                "USB cable — and note the ports the app's stages hold are "
                "deliberately skipped: if the heaters are on the ZP board, "
                "use the Shared ZP board transport instead.",
                "error",
            )
            return
        idx = self._port.findData(found.port)
        if idx >= 0:
            self._port.setCurrentIndex(idx)
        else:
            self._port.setCurrentText(found.port)
        self._baud.setCurrentText(str(found.baud))
        self._banner.show_message(
            f"Found {found.firmware} on {found.port}. Press Connect.", "info")

    def _on_connect_clicked(self) -> None:
        """Manual connect/disconnect — DEDICATED-SERIAL transport only (the
        future ESP32 sensor board). Shared/simulator sessions are automatic
        and show no button."""
        if self._connecting:
            return
        if self.ctrl.connected:
            # Disconnect commands both heaters off first (courtesy) — quick,
            # but still off the GUI thread for the M400-class waits.
            self._connect_btn.setEnabled(False)
            self._start_conn_worker(
                lambda: self.ctrl.disconnect() or True,
                label="disconnecting")
            return

        port = self._selected_port()
        try:
            baud = int(self._baud.currentText())
        except ValueError:
            baud = device_config.DEFAULT_BAUD
        if port:
            self._store.set("dedicated_port", port, save=False)
            self._store.set("dedicated_baud", baud)

        self._connect_btn.setEnabled(False)
        self._start_conn_worker(lambda: self.ctrl.connect(port, baud),
                                label="connecting and probing firmware")

    def _on_connect_done(self, ok: bool) -> None:
        self._connecting = False
        self._connect_btn.setEnabled(True)
        self._set_connected_ui(self.ctrl.connected)
        if not ok and not self.ctrl.connected:
            self._conn_pill.set_state("DISCONNECTED", "overlay0")
            # Only a MANUAL (serial) attempt earns a banner — a quiet auto
            # attempt while the ZP board is down would spam one every retry,
            # and the hint label already names the remedy.
            if self._current_transport() == "serial":
                self._banner.show_message(
                    "Could not connect. Use \"Detect board\" to scan for it "
                    "and check the USB cable.",
                    "error",
                )
        self._refresh_transport_hint()
        # v7.18: log-on-connect preference (long unattended holds).
        if (self.ctrl.connected and self._store.get("log_on_connect", False)
                and not self.ctrl.telemetry.active):
            self._log_btn.setChecked(True)

    def _on_connection(self, connected: bool) -> None:
        self._set_connected_ui(connected)
        if not connected:
            self._banner.show_message(
                "Link lost. IMPORTANT: the board runs its own control loop "
                "and will keep holding its last setpoint. Power it down if "
                "that is not what you want.",
                "error",
            )
        else:
            self._banner.clear()
            self._plot.clear()
            port = self.ctrl.active_port or "simulator"
            self._status.setText(f"Connected on {port}.")

    def _set_connected_ui(self, connected: bool) -> None:
        self._connect_btn.setText("Disconnect" if connected else "Connect")
        if connected and self.ctrl.transport == "shared":
            self._conn_pill.set_state("ON ZP BOARD", "green")
        elif connected and self.ctrl.simulated:
            self._conn_pill.set_state("SIMULATED", "mauve")
        elif connected:
            self._conn_pill.set_state("CONNECTED", "green")
        else:
            self._conn_pill.set_state("DISCONNECTED", "overlay0")
        for wdg in (self._port, self._baud):
            wdg.setEnabled(not connected)
        self._at_start.setEnabled(not (connected
                                       and self.ctrl.transport == "shared"))
        self._refresh_transport_hint()

    # ── probe ───────────────────────────────────────────────────────

    def _on_probe(self, report) -> None:
        fw = report.firmware_name or "unknown firmware"
        port = self.ctrl.active_port
        self._fw.setText(f"{fw}" + (f"  on {port}" if port else ""))

        for spec in ALL_ZONES:
            self._cards[spec.zone_id].update_capability(
                report.zone(spec.zone_id))

        lines = [
            f"Firmware       : {report.firmware_name or '(not reported)'}",
            f"Port / baud    : {self.ctrl.active_port or '(simulated)'}"
            + (f" @ {self.ctrl.active_baud}" if self.ctrl.active_baud else ""),
            f"Transport      : {self.ctrl.transport or '(none)'}",
            f"Sensor fields  : {', '.join(report.sensor_fields) or '(none)'}",
            f"Setpoint step  : {report.setpoint_resolution_c:g} C"
            + ("   (whole degrees only)" if report.integer_setpoints else ""),
            "",
            "Capabilities:",
        ]
        if report.capabilities:
            for k, v in sorted(report.capabilities.items()):
                lines.append(f"    {'yes' if v else 'NO '}  {k}")
        else:
            lines.append("    (not reported — built without "
                         "EXTENDED_CAPABILITIES_REPORT)")
        lines.append("")
        for spec in ALL_ZONES:
            cap = report.zone(spec.zone_id)
            lines.append(f"{spec.title}")
            lines.append(
                f"    sensor      : "
                f"{'present' if cap.sensor_present else 'MISSING'}"
                f"  ({spec.sensor_connector}, M105 '{spec.temp_key}:')"
                + (f"  reading {cap.reading_c:.2f} C"
                   if cap.reading_c is not None else "")
            )
            lines.append(f"    control     : {cap.control_mode}")
            if cap.pid:
                lines.append(
                    f"    PID         : Kp={cap.pid.kp:.2f}  "
                    f"Ki={cap.pid.ki:.2f}  Kd={cap.pid.kd:.2f}"
                )
            if cap.sensor_fault:
                lines.append(f"    SENSOR FAULT: {cap.sensor_fault}")
            if cap.note:
                lines.append(f"    note        : {cap.note}")
            lines.append("")

        warnings = report.warnings()
        if warnings:
            lines.append("=" * 66)
            lines.append("THINGS THAT WILL LIMIT THIS RIG")
            lines.append("=" * 66)
            for i, wtext in enumerate(warnings, 1):
                lines.append(f"{i}. {wtext}")
                lines.append("")
        if report.errors:
            lines.append("Probe errors:")
            lines.extend(f"  * {e}" for e in report.errors)
        self._fw_text.setPlainText("\n".join(lines))

        # Lead with the most serious thing found.
        faults = [
            spec for spec in ALL_ZONES
            if report.zone(spec.zone_id).sensor_fault
            or not report.zone(spec.zone_id).sensor_present
        ]
        if faults:
            names = ", ".join(z.title for z in faults)
            self._banner.show_message(
                f"{names}: sensor problem — heating is blocked for that "
                f"zone. See the Firmware tab for the detail.",
                "error",
            )
            self._probe_flagged = True
        elif warnings:
            self._banner.show_message(
                (f"{len(warnings)} firmware limitation detected — see the "
                 f"Firmware tab." if len(warnings) == 1 else
                 f"{len(warnings)} firmware limitations detected — see the "
                 f"Firmware tab."),
                "warn",
            )
            self._probe_flagged = True
        elif self._probe_flagged:
            self._banner.clear()
            self._probe_flagged = False

    # ── data ────────────────────────────────────────────────────────

    def _on_channels(self, channels) -> None:
        self._table.setRowCount(len(channels))
        for r, ch in enumerate(channels):
            state, tone = "live", "green"
            if ch.stale:
                state, tone = "stale", "yellow"
            elif ch.raw_c < 0:
                state, tone = "sensor fault", "red"
            elif ch.calibrated:
                state, tone = "calibrated", "blue"
            vals = [
                ch.label,
                f"{ch.value_c:.2f} °C",
                f"{ch.raw_c:.2f} °C",
                "—" if ch.target_c is None else f"{ch.target_c:.1f} °C",
                "—" if ch.power_pct is None else f"{ch.power_pct:.0f} %",
                state,
            ]
            for col, text in enumerate(vals):
                item = QTableWidgetItem(text)
                if col == 5:
                    item.setForeground(_c(tone))
                if col in (1, 2, 3, 4):
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._table.setItem(r, col, item)

        for spec in ALL_ZONES:
            cal = self._cal_fields.get(spec.zone_id)
            chan = self.ctrl.hub.marlin_channel(spec.temp_key)
            if cal and chan is not None:
                cal["board"].setText(f"{chan.raw_c:.2f} °C")
                cal["state"].setText(
                    self.ctrl.calibration.get(chan.uid).describe()
                )

    def _on_zone_state(self, zone_id: str, report, runtime) -> None:
        card = self._cards.get(zone_id)
        if card is not None:
            card.update_state(report, runtime)

        f = self._health.get(zone_id)
        if f is not None:
            f["temp"].setText(
                "—" if report.temp_c is None else f"{report.temp_c:.2f} °C")
            f["error"].setText(
                "—" if report.error_c is None else f"{report.error_c:+.2f} °C")
            f["rate"].setText(
                "—" if report.rate_c_per_min is None
                else f"{report.rate_c_per_min:+.3f} °C/min")
            f["eta"].setText(
                "reached" if (report.eta_s is not None and report.eta_s <= 0.5)
                else format_duration(report.eta_s))
            f["settle"].setText(format_duration(report.settle_time_s))
            f["overshoot"].setText(
                "—" if report.overshoot_c is None
                else f"{report.overshoot_c:.2f} °C")
            f["ripple"].setText(
                "—" if report.ripple_half_c is None
                else f"± {report.ripple_half_c:.3f} °C")
            f["duty"].setText(
                "—" if report.steady_duty_pct is None
                else f"{report.steady_duty_pct:.1f} %")
            f["tau"].setText(format_duration(report.time_constant_s))
            note = report.headroom_note
            if self.ctrl.simulated:
                note = (note + "  ").strip() + "  (simulated time accelerated)"
            f["note"].setText(note)

        if report.temp_c is not None and runtime.sensor_ok:
            spec = next(z for z in ALL_ZONES if z.zone_id == zone_id)
            card = self._cards.get(zone_id)
            label = card.title() if card is not None else spec.title
            self._plot.add_sample(
                zone_id, label, report.temp_c, report.target_c,
                report.duty_pct
            )

    def _on_tick(self) -> None:
        if not self.ctrl.connected:
            return
        self._plot.update()
        for zid, card in self._cards.items():
            st = self.ctrl.ramp_state(zid)
            if st is not None and (st.active or st.stalled):
                card.show_ramp(st)

    def _on_raw_line(self, direction: str, text: str) -> None:
        if not self._show_temps.isChecked():
            low = text.strip().lower()
            if (low.startswith("ok t:") or low.startswith("t:")
                    or low.startswith("ok b:") or low.startswith("b:")):
                return
        self._console.appendPlainText(
            ("TX  " if direction == "tx" else "RX  ") + text
        )

    # ── PID ─────────────────────────────────────────────────────────

    def _on_pid(self, snapshot) -> None:
        for zone_id, pid in (snapshot or {}).items():
            e = self._pid_fields.get(zone_id)
            if e is None:
                continue
            spec = next(z for z in ALL_ZONES if z.zone_id == zone_id)
            if pid is None:
                for k in ("kp", "ki", "kd"):
                    e[k].setText("unavailable")
                    e[k].setStyleSheet(
                        f"color:{_hex('yellow', '#f9e2af')};font-weight:600;")
                    e[f"{k}_spin"].setEnabled(False)
                e["apply"].setEnabled(False)
                e["status"].setText(
                    f"This heater has no PID in firmware — enable "
                    f"{spec.pid_config_symbol} in Configuration.h. It is "
                    f"currently bang-bang controlled."
                )
                e["status"].setStyleSheet(
                    f"color:{_hex('yellow', '#f9e2af')};")
                continue
            for k, v in (("kp", pid.kp), ("ki", pid.ki), ("kd", pid.kd)):
                e[k].setText(f"{v:.2f}")
                e[k].setStyleSheet(
                    f"color:{_hex('text', '#cdd6f4')};font-weight:600;")
                e[f"{k}_spin"].setEnabled(True)
                if e[f"{k}_spin"].value() == 0.0:
                    e[f"{k}_spin"].setValue(v)
            e["apply"].setEnabled(True)
            e["status"].setText("")

    def _on_apply_pid(self, zone_id: str) -> None:
        e = self._pid_fields[zone_id]
        self.ctrl.set_pid(
            zone_id, e["kp_spin"].value(), e["ki_spin"].value(),
            e["kd_spin"].value(),
        )

    def _confirm(self, title: str, text: str) -> bool:
        return QMessageBox.question(
            self, title, text, QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        ) == QMessageBox.Yes

    def _shared_board_note(self) -> str:
        if self.ctrl.transport == "shared":
            return ("\n\nNOTE: this is the SHARED motion board — its EEPROM "
                    "also holds the Z/pump settings (steps/mm, feedrates).")
        return ""

    def _on_save_eeprom(self) -> None:
        if self._confirm(
            "Save to EEPROM",
            "Write the board's current settings to EEPROM (M500)?\n\n"
            "This overwrites the stored values permanently."
            + self._shared_board_note(),
        ):
            self.ctrl.save_eeprom()

    def _on_load_eeprom(self) -> None:
        if self._confirm(
            "Reload EEPROM",
            "Reload settings from EEPROM (M501)?\n\n"
            "Any unsaved changes you have applied will be discarded."
            + self._shared_board_note(),
        ):
            self.ctrl.load_eeprom()

    def _on_factory_reset(self) -> None:
        if self._confirm(
            "Factory reset",
            "Load the firmware's compiled-in default settings (M502)?\n\n"
            "This discards your tuning. It affects RAM only until you also "
            "save to EEPROM." + self._shared_board_note(),
        ):
            self.ctrl.factory_reset()

    # ── autotune ────────────────────────────────────────────────────

    def _on_autotune_start(self) -> None:
        if self.ctrl.transport == "shared":
            QMessageBox.information(
                self, "Autotune unavailable",
                "PID autotune is unavailable over the shared ZP link — the "
                "tune would monopolise the motion board's serial channel for "
                "its whole duration.\n\nEither set the PID manually on the "
                "PID tab, or disconnect the ZP board in the app and run the "
                "autotune over a dedicated connection.",
            )
            return
        zone_id = self._at_zone.currentData()
        target = self._at_target.value()
        cycles = self._at_cycles.value()
        spec = next(z for z in ALL_ZONES if z.zone_id == zone_id)
        if not self._confirm(
            "Start PID autotune",
            f"Run PID autotune on {spec.title} at {target:g} °C for {cycles} "
            f"cycles?\n\nThe heater will be driven through repeated "
            f"heat/cool cycles. On a large water mass this can take a long "
            f"time and may hit Marlin's per-cycle time limit.",
        ):
            return
        self._at_result.clear()
        self._at_adopt.setEnabled(False)
        self.ctrl.start_autotune(
            zone_id, target, cycles, apply_result=self._at_apply.isChecked()
        )

    def _on_at_progress(self, zone_id, progress, elapsed) -> None:
        bits = []
        if progress.t_min is not None and progress.t_max is not None:
            bits.append(f"swing {progress.t_min:.2f}-{progress.t_max:.2f} °C")
        if progress.ku is not None:
            bits.append(f"Ku={progress.ku:.2f} Tu={progress.tu:.2f}")
        if progress.kp is not None:
            bits.append(
                f"Kp={progress.kp:.2f} Ki={progress.ki:.2f} "
                f"Kd={progress.kd:.2f}")
        self._at_status.setText(
            f"Autotune running on {zone_id} — {format_duration(elapsed)} "
            f"elapsed."
        )
        if bits:
            self._at_result.appendPlainText("   ".join(bits))

    def _on_at_done(self, zone_id, pid, err, hint) -> None:
        if pid is None:
            self._at_status.setText(f"Autotune failed: {err}")
            self._at_result.appendPlainText(f"\nFAILED: {err}\n{hint}")
            self._at_adopt.setEnabled(False)
            if hint:
                self._banner.show_message(
                    f"Autotune failed: {err}. {hint}", "warn")
            return
        self._at_status.setText(
            f"Autotune finished: Kp={pid.kp:.2f} Ki={pid.ki:.2f} "
            f"Kd={pid.kd:.2f}. Save to EEPROM to keep it."
        )
        self._at_result.appendPlainText(
            f"\nRESULT   Kp={pid.kp:.2f}   Ki={pid.ki:.2f}   Kd={pid.kd:.2f}"
        )
        self._last_at = (zone_id, pid)
        self._at_adopt.setEnabled(True)

    def _on_adopt_result(self) -> None:
        if not self._last_at:
            return
        zone_id, pid = self._last_at
        e = self._pid_fields.get(zone_id)
        if e is None:
            return
        e["kp_spin"].setValue(pid.kp)
        e["ki_spin"].setValue(pid.ki)
        e["kd_spin"].setValue(pid.kd)
        self._status.setText(
            "Copied the autotune result into the PID tab. Press "
            "\"Apply to board\" to send it."
        )

    # ── misc ────────────────────────────────────────────────────────

    def _on_calibrate(self, zone_id: str) -> None:
        self.ctrl.calibrate_single_point(
            zone_id, self._cal_fields[zone_id]["ref"].value()
        )

    def _on_fault(self, latched) -> None:
        f = latched.fault
        extra = ""
        if self.ctrl.transport == "shared":
            extra = (" This is the SHARED motion board — Z and the pumps are "
                     "halted with it.")
        self._banner.show_message(
            f"HEATER FAULT — {latched.summary}. {f.message}  {f.hint}  "
            f"The board has halted and must be POWER-CYCLED.{extra}",
            "error", dismissable=False,
        )
        QMessageBox.critical(
            self, "Heater fault",
            f"{f.message}\n\n{f.hint}\n\nThe board will ignore further "
            f"commands until it is power-cycled.{extra}",
        )

    def _on_divergence(self, ev) -> None:
        self._banner.show_message(f"SENSOR DIVERGENCE — {ev.message}", "warn")

    def _on_estop(self) -> None:
        extra = ""
        if self.ctrl.transport == "shared":
            extra = ("\n\nThis is the SHARED motion board: M112 also halts "
                     "Z and the pumps mid-move.")
        if self._confirm(
            "Emergency stop",
            "Send M112?\n\nThis HALTS the board immediately. It will ignore "
            "all commands until it is power-cycled.\n\nFor a normal stop use "
            "\"ALL HEATERS OFF\" instead." + extra,
        ):
            self.ctrl.emergency_stop()

    def _on_send_raw(self) -> None:
        cmd = self._raw_in.text().strip()
        if not cmd:
            return
        self.ctrl.send_raw(cmd)
        self._raw_in.clear()

    def _on_log_toggled(self, on: bool) -> None:
        if on:
            path = self.ctrl.start_logging("hold")
            self._log_btn.setText("Stop logging" if path else "Logging disabled")
        else:
            self.ctrl.stop_logging()
            self._log_btn.setText("Start logging")

    def _on_ramp(self, zone_id: str, st) -> None:
        card = self._cards.get(zone_id)
        if card is not None:
            card.show_ramp(st)

    def _on_status(self, text: str) -> None:
        self._status.setText(text)
