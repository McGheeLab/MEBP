"""stress_test_workflow.py — ZP serial flow-control bench-validation harness.

v7.5.x: A repeatable, hands-off stress test that hammers the ZP (Marlin Z+pump)
board with exactly the command patterns that historically made it "drop" mid-
operation, so the operator can validate the synchronous ``ok`` flow-control fix
(``MEBP_v75x_ZP_SERIAL_FLOW_CONTROL.md``) on the real ME3B V1 board.

What it exercises (each selectable, looped over N cycles):
    • Travel stress  — repeated ``StageController.safe_travel_to`` hops around
      the plate (retract Z → fast XY → wait; needle NEVER lowers). This is the
      exact path that failed in the reported log (Quick Print pre-position) and
      drives a dense Z-move + M400 + XY + M114 stream.
    • Jog stress     — long bursts of small Z (and optional pump) relative jogs,
      the dense un-acked write stream that used to overflow Marlin's serial
      buffer and freeze/desync the board.
    • XY stress      — a CONCURRENT XY (Prior) oscillation during the jog phase
      (needle retracted, so the plate can move safely), driving the XY serial
      channel at the same time as the ZP channel. The two links are independent
      (separate serial locks), so this surfaces any cross-communication conflict
      — contention on the shared position poller, threading, or host buffers.
    • Print stress   — runs a small synthesized print at the plate-centre well
      through the real discrete ``PrintManager`` path several times (opt-in;
      default flow 0 = dry run so no material is dispensed — the *command
      stream* is what we are validating, not deposition).

It watches for — and counts — every "ZP disconnected" transition and every
mid-session board RESET, and reports the flow-control telemetry
(``ZPStageManager.get_comm_counters``: commands issued vs cleanly acked, ack
failures, resets). A run PASSES only if there were zero disconnects, zero
resets, and zero un-acknowledged commands.

SAFETY: the harness refuses to start unless XY+ZP are connected and a Safe Z is
calibrated (without it a retract target of 0 is the plate-bottom datum on ME3B
V1 = a crash). All XY travel goes through ``safe_travel_to`` (retract-first),
Z jogs are small oscillations around the retracted Safe Z (never a descent
toward the plate), and Stop / completion always retracts to Safe Z. The whole
run executes on a worker thread so the GUI + live camera stay responsive.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QCheckBox, QSpinBox, QDoubleSpinBox, QFrame, QSizePolicy, QPlainTextEdit,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.section_stack import (
    PromotedSectionsPanel, wire_section_promotion)
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.dialogs.workflow_settings_dialog import (
    WorkflowSettingsDialog, build_locations_widget,
)

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, build_well_plate_job,
)

logger = logging.getLogger(__name__)


@dataclass
class _StressConfig:
    cycles: int
    do_travel: bool
    do_jog: bool
    do_print: bool
    travel_hops: int
    jog_moves: int
    jog_amp_mm: float
    prints_per_cycle: int
    print_flow_uL_s: float
    print_descend: bool        # full print descends to real print height
    print_height_mm: float     # print height above the calibrated plate bottom
    jog_pumps: bool
    do_xy: bool          # oscillate XY concurrently during the jog phase
    xy_amp_um: float
    # ── individual print-component isolation ──────────────────────
    do_descend: bool     # Z descent + M400/arrival confirm (the print MOVE_Z)
    descend_depth_mm: float
    do_dispense: bool     # pump-only command stream (the print DISPENSE)
    component_reps: int
    use_simple_pm: bool = False  # run prints via SimplePrintManager (debug)


class _StressBridge(QObject):
    """Worker-thread → GUI-thread signal relay (queued)."""

    log_line = Signal(str)
    stats = Signal(dict)
    finished = Signal(bool, str)   # passed, summary


class StressTestWorkflowPage(QWidget):
    """ZP serial flow-control stress / bench-validation page."""

    back_requested = Signal()

    # Travel hop offsets from plate centre (µm). A spread that forces real
    # cross-position XY moves (each preceded by a Z retract).
    _HOP_OFFSETS_UM = [
        (0, 0), (6000, 6000), (-6000, 6000), (-6000, -6000),
        (6000, -6000), (10000, 0), (-10000, 0), (0, 8000), (0, -8000),
    ]
    _ENV_MARGIN_UM = 2000.0

    # SAFETY: stop the run once the ZP has missed this many acks (or any
    # disconnect). A live-but-failing link (M400/'ok' lost) does NOT trip the
    # connection watchdog, so without this the loop would keep oscillating the
    # Z motor on a degraded board — which drives it continuously and overheats
    # it (observed: only the Z stepper got very hot after a comms fault).
    _ABORT_ON_OK_FAILS = 3

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None

        self._plate = None
        self._well_positions = None
        self._safe_z: float | None = None
        self._z_references: dict = {}

        self._context_widget: StandardJogContextPanel | None = None

        # Execution state.
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._bridge = _StressBridge()
        self._bridge.log_line.connect(self._append_log)
        self._bridge.stats.connect(self._update_stats)
        self._bridge.finished.connect(self._on_finished)

        # Comprehensive settings popout (scrollable, saveable stress presets).
        self._settings_dialog = WorkflowSettingsDialog(
            "stress_test", "ZP Stress Test",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))
        outer.addLayout(self._build_header())
        outer.addWidget(self._build_monitor(), stretch=1)
        outer.addWidget(self._build_run_row())
        # v7.21: a section moved out of ⚙ Settings lands in the drawer, which is
        # hidden (zero footprint) until something is in it. AFTER the run row on
        # purpose, so Start / Abort never move.
        self._promoted_panel = PromotedSectionsPanel()
        outer.addWidget(self._promoted_panel)
        self._layout_store = wire_section_promotion(
            self, self._settings_dialog, self._promoted_panel.stack,
            settings=self._settings, workflow_id="stress_test")
        self._update_button_state()
        self._settings_dialog.load_last()
        self._update_settings_summary()

    # ── UI ────────────────────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)
        title = QLabel("ZP Stress Test")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        row.addWidget(title)
        self._settings_summary = QLabel("")
        self._settings_summary.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(self._settings_summary)
        row.addStretch(1)
        settings_btn = QPushButton("⚙ Settings")
        settings_btn.setCursor(Qt.PointingHandCursor)
        settings_btn.setToolTip(
            "Open the full, saveable stress-test configuration (phases, "
            "counts, amplitudes, print/component isolation).")
        settings_btn.clicked.connect(self._open_settings)
        row.addWidget(settings_btn)
        return row

    # ── Settings popout ───────────────────────────────────────────

    def _open_settings(self):
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        self._update_button_state()
        self._update_settings_summary()

    def _update_settings_summary(self):
        if not hasattr(self, "_settings_summary"):
            return
        try:
            phases = []
            if self._chk_travel.isChecked():
                phases.append("travel")
            if self._chk_jog.isChecked():
                phases.append("jog")
            if self._chk_print.isChecked():
                phases.append("print")
            self._settings_summary.setText(
                f"{self._spin_cycles.value()} cycles · "
                f"{'+'.join(phases) or 'no phases'}")
        except Exception:
            pass

    @staticmethod
    def _dspin(lo, hi, val, suffix="", decimals=2, step=None, tip=""):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setDecimals(decimals)
        if suffix:
            sb.setSuffix(suffix)
        sb.setValue(val)
        if step is not None:
            sb.setSingleStep(step)
        if tip:
            sb.setToolTip(tip)
        return sb

    @staticmethod
    def _ispin(lo, hi, val, step=1, suffix="", tip=""):
        sb = QSpinBox()
        sb.setRange(lo, hi)
        sb.setValue(val)
        sb.setSingleStep(step)
        if suffix:
            sb.setSuffix(suffix)
        if tip:
            sb.setToolTip(tip)
        return sb

    def _build_settings_dialog(self, dlg: WorkflowSettingsDialog):
        # ── Phases & cycles ──
        self._chk_travel = QCheckBox("Travel stress (retract → XY hops)")
        self._chk_travel.setChecked(True)
        self._chk_travel.toggled.connect(self._on_settings_changed)
        self._chk_jog = QCheckBox("Jog stress (dense Z/pump moves)")
        self._chk_jog.setChecked(True)
        self._chk_jog.toggled.connect(self._on_settings_changed)
        self._chk_print = QCheckBox("Print stress (real print path)")
        self._chk_print.setChecked(False)
        self._chk_print.toggled.connect(self._on_settings_changed)
        self._spin_cycles = self._ispin(1, 100000, 20)
        self._spin_cycles.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        sec = dlg.add_section("Phases & cycles")
        sec.add_check("travel", self._chk_travel, True)
        sec.add_check("jog", self._chk_jog, True)
        sec.add_check("print", self._chk_print, False)
        sec.add("cycles", "Cycles", self._spin_cycles, 20)

        # ── Travel ──
        self._spin_hops = self._ispin(1, 200, 8)
        sec = dlg.add_section("Travel stress")
        sec.add("hops", "Travel hops / cycle", self._spin_hops, 8)

        # ── Jog ──
        self._spin_jog = self._ispin(1, 5000, 200)
        self._spin_amp = self._dspin(0.1, 5.0, 1.0, " mm", 2, 0.1)
        self._chk_jog_pumps = QCheckBox("Also oscillate pump (net-zero)")
        self._chk_jog_pumps.setChecked(True)
        sec = dlg.add_section("Jog stress")
        sec.add("jog_moves", "Jog moves / cycle", self._spin_jog, 200)
        sec.add("jog_amp", "Jog amplitude", self._spin_amp, 1.0)
        sec.add_check("jog_pumps", self._chk_jog_pumps, True)

        # ── Concurrent XY ──
        self._chk_xy = QCheckBox("Move XY concurrently (during jog phase)")
        self._chk_xy.setChecked(True)
        self._chk_xy.setToolTip(
            "Oscillates the XY (Prior) stage on a separate thread while the ZP "
            "board is being jogged, so both serial channels are driven at once.")
        self._spin_xy_amp = self._ispin(50, 20000, 500, 50, " µm")
        sec = dlg.add_section("Concurrent XY")
        sec.add_check("xy", self._chk_xy, True)
        sec.add("xy_amp", "XY amplitude", self._spin_xy_amp, 500)

        # ── Print stress ──
        self._spin_prints = self._ispin(1, 20, 1)
        self._spin_flow = self._dspin(
            0.0, 10.0, 0.0, " µL/s", 3, 0.05,
            "0 = dry run (no material dispensed). The command stream is "
            "identical regardless — that is what the test validates.")
        self._chk_print_descend = QCheckBox("Descend to print Z")
        self._chk_print_descend.setChecked(True)
        self._chk_print_descend.setToolTip(
            "Print stress drives Z DOWN to the real print height (plate bottom "
            "+ the height below) — a faithful real print. Plate-floor clamped.")
        self._spin_print_height = self._dspin(
            0.0, 20.0, 0.5, " mm", 2, 0.1,
            "Print height above the calibrated plate bottom (0.5 mm default).")
        self._chk_simple_pm = QCheckBox("Simple PrintManager (debug)")
        self._chk_simple_pm.setToolTip(
            "Run the print via SimplePrintManager — a minimal, every-move-"
            "CONFIRMED executor (no open-loop streaming / barriers / pacing; "
            "keeps all safety). If this prints reliably but the full "
            "PrintManager doesn't, the streaming path is implicated.")
        sec = dlg.add_section("Print stress")
        sec.add("prints", "Prints / cycle", self._spin_prints, 1)
        sec.add("flow", "Print flow", self._spin_flow, 0.0)
        sec.add_check("print_descend", self._chk_print_descend, True)
        sec.add("print_height", "Print height", self._spin_print_height, 0.5)
        sec.add_check("simple_pm", self._chk_simple_pm, False)

        # ── Component isolation ──
        self._chk_descend = QCheckBox("Z descent + confirm")
        self._chk_descend.setToolTip(
            "Reproduces the print's MOVE_Z in isolation: drive Z DOWN by the "
            "depth below, M400 + wait-for-arrival confirm, then retract.")
        self._spin_descend = self._dspin(
            0.5, 40.0, 5.0, " mm", 1, 1.0,
            "How far below Safe Z to descend. Plate-floor clamped.")
        self._chk_dispense = QCheckBox("Pump dispense")
        self._chk_dispense.setToolTip(
            "Reproduces the print's DISPENSE in isolation: net-zero pump stream.")
        self._spin_comp_reps = self._ispin(1, 2000, 50)
        sec = dlg.add_section("Component isolation")
        sec.add_check("descend", self._chk_descend, False)
        sec.add("descend_depth", "Descent depth", self._spin_descend, 5.0)
        sec.add_check("dispense", self._chk_dispense, False)
        sec.add("comp_reps", "Component reps", self._spin_comp_reps, 50)

        # ── Locations & Hardware (read-only) ──
        dlg.add_info_section()
        dlg.set_info_refresher(lambda: build_locations_widget(
            self._controller, self._hw_config, self._well_positions,
            z_references=self._z_references, safe_z=self._safe_z))
        dlg.finalize()

    def _build_monitor(self) -> QWidget:
        card = Card("Monitor — live counters · event log", flush=True,
                    compact=True)
        body = QWidget()
        v = QVBoxLayout(body)
        v.setContentsMargins(s(8), s(6), s(8), s(8))
        v.setSpacing(s(8))

        # Result banner.
        self._banner = QLabel("Idle — configure and press Start.")
        self._banner.setAlignment(Qt.AlignCenter)
        self._banner.setStyleSheet(
            f"color: {COLORS['subtext0']}; background-color: {COLORS['surface0']};"
            f" border-radius: 6px; padding: {s(6)}px; font-size: {sf(11)}pt;"
            f" font-weight: 600;")
        v.addWidget(self._banner)

        # Counter grid.
        stat = QGridLayout()
        stat.setHorizontalSpacing(s(16))
        stat.setVerticalSpacing(s(4))
        self._stat_labels: dict[str, QLabel] = {}
        cells = [
            ("phase", "Phase"), ("elapsed", "Elapsed"), ("cycle", "Cycle"),
            ("cmd", "Commands"), ("ok", "Acked"), ("ok_fail", "Unacked"),
            ("reset", "Board resets"), ("disconnect", "Comms drops (XY+ZP)"),
        ]
        for i, (key, title) in enumerate(cells):
            r, c = divmod(i, 4)
            box = QVBoxLayout()
            t = QLabel(title)
            t.setStyleSheet(
                f"color: {COLORS['overlay0']}; font-size: {sf(8)}pt;")
            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-size: {sf(12)}pt;"
                f" font-weight: 600;")
            box.addWidget(t)
            box.addWidget(val)
            stat.addLayout(box, r, c)
            self._stat_labels[key] = val
        v.addLayout(stat)

        # Event log.
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(2000)
        self._log.setStyleSheet(
            f"QPlainTextEdit {{ background-color: {COLORS['mantle']};"
            f" color: {COLORS['subtext0']}; border: 1px solid "
            f"{COLORS['surface1']}; border-radius: 6px; font-family: "
            f"'Cascadia Code','Consolas',monospace; font-size: {sf(9)}pt; }}")
        v.addWidget(self._log, stretch=1)

        card.add_widget(body)
        return card

    def _build_run_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))
        self._start_btn = QPushButton("Start stress test")
        self._start_btn.clicked.connect(self._on_start)
        row.addWidget(self._start_btn)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        row.addWidget(self._stop_btn)
        row.addStretch(1)
        self._status = QLabel("")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._status, stretch=1)
        return frame

    # ── MainWindow hooks ──────────────────────────────────────────

    def get_page_title(self) -> str:
        return "ZP Stress Test"

    def get_sub_page_title(self) -> str:
        return "ZP Stress Test"

    def get_context_widget(self) -> QWidget:
        if self._context_widget is None:
            self._context_widget = StandardJogContextPanel(
                controller=self._controller, settings=self._settings,
                show_connect=False, bypass_safety=False)
            if self._hw_config is not None:
                self._context_widget.set_hardware_config(self._hw_config)
            if any(v is not None for v in
                   (self._plate, self._well_positions, self._safe_z)):
                self._context_widget.set_calibration_data(
                    self._plate, self._well_positions, self._safe_z)
        return self._context_widget

    def on_status_update(self) -> None:
        if self._context_widget is not None and hasattr(
                self._context_widget, "on_status_update"):
            self._context_widget.on_status_update()

    def set_settings(self, settings) -> None:
        self._settings = settings
        if self._context_widget is not None:
            self._context_widget.set_settings(settings)

    def set_hardware_config(self, hw_config) -> None:
        self._hw_config = hw_config
        if self._context_widget is not None:
            self._context_widget.set_hardware_config(hw_config)

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if self._context_widget is not None:
            self._context_widget.set_calibration_data(
                plate, well_positions, safe_z)
        self._update_button_state()

    def set_z_references(self, refs) -> None:
        if isinstance(refs, dict):
            self._z_references = dict(refs)
        if self._context_widget is not None and hasattr(
                self._context_widget, "set_z_references"):
            try:
                self._context_widget.set_z_references(refs)
            except Exception:
                pass

    def hideEvent(self, event):
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass
        super().hideEvent(event)

    # ── GUI-thread slots ──────────────────────────────────────────

    def _append_log(self, line: str) -> None:
        self._log.appendPlainText(line)

    def _update_stats(self, d: dict) -> None:
        for key, lbl in self._stat_labels.items():
            if key in d:
                lbl.setText(str(d[key]))
        # Recolor the failure counters when non-zero.
        for key, good in (("ok_fail", COLORS["green"]),
                          ("reset", COLORS["green"]),
                          ("disconnect", COLORS["green"])):
            try:
                bad = int(d.get(key, 0)) > 0
            except (TypeError, ValueError):
                bad = False
            self._stat_labels[key].setStyleSheet(
                f"color: {COLORS['red'] if bad else COLORS['text']};"
                f" font-size: {sf(12)}pt; font-weight: 600;")

    def _on_finished(self, passed: bool, summary: str) -> None:
        self._thread = None
        if passed:
            self._banner.setText("✅ PASS — " + summary)
            self._banner.setStyleSheet(
                f"color: {COLORS['base']}; background-color: {COLORS['green']};"
                f" border-radius: 6px; padding: {s(6)}px;"
                f" font-size: {sf(11)}pt; font-weight: 700;")
        else:
            self._banner.setText("❌ FAIL — " + summary)
            self._banner.setStyleSheet(
                f"color: {COLORS['base']}; background-color: {COLORS['red']};"
                f" border-radius: 6px; padding: {s(6)}px;"
                f" font-size: {sf(11)}pt; font-weight: 700;")
        self._status.setText("Done.")
        self._update_button_state()

    # ── Run / stop ────────────────────────────────────────────────

    def _running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _update_button_state(self, *_):
        running = self._running()
        connected = (getattr(self._controller, "is_xy_connected", False)
                     and getattr(self._controller, "is_zp_connected", False))
        self._start_btn.setEnabled(connected and not running)
        self._stop_btn.setEnabled(running)

    def _gather_config(self) -> _StressConfig:
        return _StressConfig(
            cycles=int(self._spin_cycles.value()),
            do_travel=self._chk_travel.isChecked(),
            do_jog=self._chk_jog.isChecked(),
            do_print=self._chk_print.isChecked(),
            travel_hops=int(self._spin_hops.value()),
            jog_moves=int(self._spin_jog.value()),
            jog_amp_mm=float(self._spin_amp.value()),
            prints_per_cycle=int(self._spin_prints.value()),
            print_flow_uL_s=float(self._spin_flow.value()),
            print_descend=self._chk_print_descend.isChecked(),
            print_height_mm=float(self._spin_print_height.value()),
            jog_pumps=self._chk_jog_pumps.isChecked(),
            do_xy=self._chk_xy.isChecked(),
            xy_amp_um=float(self._spin_xy_amp.value()),
            do_descend=self._chk_descend.isChecked(),
            descend_depth_mm=float(self._spin_descend.value()),
            do_dispense=self._chk_dispense.isChecked(),
            component_reps=int(self._spin_comp_reps.value()),
            use_simple_pm=self._chk_simple_pm.isChecked(),
        )

    def _on_start(self):
        if self._running():
            return
        ctrl = self._controller
        if not getattr(ctrl, "is_xy_connected", False) or \
                not getattr(ctrl, "is_zp_connected", False):
            self._status.setText("Connect the XY and ZP stages first.")
            return
        if self._safe_z is None:
            self._status.setText(
                "No Safe Z calibrated — set it on Calibration → Needle Offset "
                "before stress testing (a retract to 0 would crash the needle).")
            return
        cfg = self._gather_config()
        if not (cfg.do_travel or cfg.do_jog or cfg.do_print or cfg.do_xy
                or cfg.do_descend or cfg.do_dispense):
            self._status.setText("Select at least one stress mode.")
            return
        if cfg.do_print and self._print_well_center_mm() is None:
            self._status.setText(
                "Print stress needs a plate — run Calibration, or uncheck it.")
            return

        self._log.clear()
        self._banner.setText("Running…")
        self._banner.setStyleSheet(
            f"color: {COLORS['base']}; background-color: {COLORS['blue']};"
            f" border-radius: 6px; padding: {s(6)}px; font-size: {sf(11)}pt;"
            f" font-weight: 700;")
        self._status.setText("Stress test running…")
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, args=(cfg,), name="ZPStressTest", daemon=True)
        self._thread.start()
        self._update_button_state()

    def _on_stop(self):
        self._stop.set()
        self._status.setText("Stopping… (finishing the current move)")

    # ── Worker thread ─────────────────────────────────────────────

    def _log_t(self, msg: str) -> None:
        self._bridge.log_line.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _run(self, cfg: _StressConfig) -> None:
        ctrl = self._controller
        zp = getattr(ctrl, "zp_stage", None)
        if zp is not None and hasattr(zp, "reset_comm_counters"):
            try:
                zp.reset_comm_counters()
            except Exception:
                pass

        t0 = time.monotonic()
        disconnects = 0
        prev_zp = bool(getattr(ctrl, "is_zp_connected", False))
        prev_xy = bool(getattr(ctrl, "is_xy_connected", False))
        last_reset_seen = 0
        degraded_logged = False

        def counters() -> dict:
            if zp is not None and hasattr(zp, "get_comm_counters"):
                try:
                    return zp.get_comm_counters()
                except Exception:
                    pass
            return {"cmd": 0, "ok": 0, "ok_fail": 0, "reset": 0}

        def emit_stats(phase: str, cycle: int) -> None:
            c = counters()
            self._bridge.stats.emit({
                "phase": phase,
                "elapsed": f"{time.monotonic() - t0:.0f}s",
                "cycle": f"{cycle}/{cfg.cycles}",
                "cmd": c["cmd"], "ok": c["ok"], "ok_fail": c["ok_fail"],
                "reset": c["reset"], "disconnect": disconnects,
            })

        def check_link() -> bool:
            """Watch BOTH serial channels and the ack health. Returns True only
            while the link is healthy enough to keep driving the motors.

            A cross-comms conflict can drop either channel; and a *live-but-
            failing* board (M400/'ok' lost) does NOT trip the connection
            watchdog, so we also stop on sustained ack failures — otherwise the
            loop keeps oscillating (and overheating) the Z motor on a degraded
            link. Logs + counts drops, resets, and the degraded-stop reason."""
            nonlocal disconnects, prev_zp, prev_xy, last_reset_seen, \
                degraded_logged
            zp_now = bool(getattr(ctrl, "is_zp_connected", False))
            xy_now = bool(getattr(ctrl, "is_xy_connected", False))
            if prev_zp and not zp_now:
                disconnects += 1
                self._log_t("❌ ZP DISCONNECTED detected!")
            if prev_xy and not xy_now:
                disconnects += 1
                self._log_t("❌ XY DISCONNECTED detected!")
            prev_zp, prev_xy = zp_now, xy_now
            c = counters()
            if c["reset"] > last_reset_seen:
                self._log_t(f"⚠ ZP board RESET detected (total {c['reset']}) "
                            f"— position lost.")
                last_reset_seen = c["reset"]
            degraded = (c["ok_fail"] >= self._ABORT_ON_OK_FAILS
                        or disconnects > 0)
            if degraded and not degraded_logged:
                degraded_logged = True
                self._log_t(
                    f"🛑 SAFETY STOP — comms degraded ({c['ok_fail']} unacked, "
                    f"{disconnects} disconnects). Halting so the ZP is not "
                    f"driven on a failing link (a live-but-unacked board keeps "
                    f"the Z motor energized/moving and it overheats).")
            return zp_now and not degraded

        self._log_t(
            f"Start: cycles={cfg.cycles} travel={cfg.do_travel} "
            f"jog={cfg.do_jog} xy={cfg.do_xy} print={cfg.do_print} "
            f"safe_z={self._safe_z:.2f}")
        try:
            for cycle in range(1, cfg.cycles + 1):
                if self._stop.is_set():
                    self._log_t("Stopped by operator.")
                    break
                if not check_link():
                    self._log_t("Aborting run — link not healthy (see above).")
                    break

                if cfg.do_travel:
                    emit_stats("travel", cycle)
                    self._travel_burst(cfg, check_link)
                if self._stop.is_set() or not check_link():
                    break
                if cfg.do_jog or cfg.do_xy:
                    emit_stats("jog+xy" if cfg.do_xy else "jog", cycle)
                    self._jog_burst(cfg, check_link)
                if self._stop.is_set() or not check_link():
                    break
                if cfg.do_descend:
                    emit_stats("descend+confirm", cycle)
                    self._descend_burst(cfg, check_link)
                if self._stop.is_set() or not check_link():
                    break
                if cfg.do_dispense:
                    emit_stats("dispense", cycle)
                    self._dispense_burst(cfg, check_link)
                if self._stop.is_set() or not check_link():
                    break
                if cfg.do_print:
                    emit_stats("print", cycle)
                    if not self._print_burst(cfg, check_link):
                        self._log_t("Aborting run — print stopped the test "
                                    "(error / drop / operator stop).")
                        break

                emit_stats("cycle done", cycle)
                self._log_t(f"Cycle {cycle}/{cfg.cycles} complete.")
        except Exception as e:
            logger.exception("Stress test worker error: %s", e)
            self._log_t(f"⚠ Worker error: {e}")
        finally:
            # Always leave the needle retracted at Safe Z.
            try:
                if getattr(ctrl, "is_zp_connected", False) and \
                        self._safe_z is not None:
                    ctrl.ensure_retracted_to(self._safe_z)
            except Exception as e:
                logger.warning("Stress test final retract failed: %s", e)
            c = counters()
            passed = (disconnects == 0 and c["reset"] == 0
                      and c["ok_fail"] == 0)
            emit_stats("done", cfg.cycles)
            summary = (f"{c['cmd']} cmds, {c['ok_fail']} unacked, "
                       f"{c['reset']} resets, {disconnects} disconnects in "
                       f"{time.monotonic() - t0:.0f}s")
            self._log_t(("PASS — " if passed else "FAIL — ") + summary)
            self._bridge.finished.emit(passed, summary)

    # ── Stress primitives (worker thread) ─────────────────────────

    def _envelope(self):
        lim = getattr(self._controller, "safety_limits", None)
        return (
            getattr(lim, "xy_min_x", 0.0), getattr(lim, "xy_min_y", 0.0),
            getattr(lim, "xy_max_x", 100000.0),
            getattr(lim, "xy_max_y", 100000.0),
        )

    def _travel_burst(self, cfg: _StressConfig, check_link) -> None:
        ctrl = self._controller
        try:
            cx, cy = ctrl.default_plate_center_um()
        except Exception:
            return
        xmin, ymin, xmax, ymax = self._envelope()
        m = self._ENV_MARGIN_UM

        def clamp(v, lo, hi):
            return max(lo + m, min(hi - m, v))

        for i in range(cfg.travel_hops):
            if self._stop.is_set():
                return
            ox, oy = self._HOP_OFFSETS_UM[i % len(self._HOP_OFFSETS_UM)]
            x = clamp(cx + ox, xmin, xmax)
            y = clamp(cy + oy, ymin, ymax)
            try:
                # target_z_mm=None → retract + travel, NEVER lower the needle.
                ok = ctrl.safe_travel_to(
                    x, y, safe_z_mm=float(self._safe_z), target_z_mm=None)
                if not ok:
                    self._log_t(f"  travel hop {i + 1} reported not-confirmed")
            except Exception as e:
                self._log_t(f"  travel hop {i + 1} error: {e}")
            if not check_link():
                return

    def _jog_burst(self, cfg: _StressConfig, check_link) -> None:
        """Dense ZP jog stream, optionally with a CONCURRENT XY oscillation.

        The needle is retracted to Safe Z first, so it is safe to also move the
        plate (XY) here. When ``do_xy`` is set, an XY oscillator runs on its own
        thread for the duration of the burst — so the Prior (XY) and ZP serial
        channels are driven *simultaneously*, which is the cross-communication
        conflict case. XY is the ONLY thing the XY channel is doing here (the
        main worker drives ZP only), so the two never fight over one channel.
        """
        ctrl = self._controller
        # Start from a known safe height so neither the Z oscillation nor the
        # concurrent XY plate motion can approach the plate.
        try:
            ctrl.ensure_retracted_to(float(self._safe_z))
        except Exception:
            pass

        # Spin up the concurrent XY oscillator (separate serial channel).
        xy_stop = threading.Event()
        xy_thread = None
        if cfg.do_xy and getattr(ctrl, "is_xy_connected", False):
            xy_thread = threading.Thread(
                target=self._xy_oscillator, args=(cfg, xy_stop, check_link),
                name="ZPStressXY", daemon=True)
            xy_thread.start()

        try:
            if not cfg.do_jog:
                # XY-only: just let the concurrent oscillator run for a span
                # comparable to a jog burst, watching the link.
                for i in range(cfg.jog_moves):
                    if self._stop.is_set():
                        return
                    time.sleep(0.01)
                    if (i + 1) % 50 == 0 and not check_link():
                        return
                return

            amp = float(cfg.jog_amp_mm)
            pump = self._first_pump() if cfg.jog_pumps else None
            for i in range(cfg.jog_moves):
                if self._stop.is_set():
                    return
                try:
                    # Down a little from Safe Z, then back up — net zero, stays
                    # well above the plate (amp small; soft limits also clamp).
                    ctrl.move_z_user_relative(-amp)
                    ctrl.move_z_user_relative(+amp)
                    if pump is not None:
                        ctrl.move_pump_relative(pump, +0.05)
                        ctrl.move_pump_relative(pump, -0.05)
                except Exception as e:
                    self._log_t(f"  jog move {i + 1} error: {e}")
                    if not check_link():
                        return
                if (i + 1) % 50 == 0 and not check_link():
                    return
        finally:
            xy_stop.set()
            if xy_thread is not None:
                xy_thread.join(timeout=5.0)

    def _xy_oscillator(self, cfg: _StressConfig, stop: threading.Event,
                       check_link) -> None:
        """Drive the XY (Prior) channel with small net-zero relative moves until
        ``stop`` is set. Runs on its own thread, concurrently with the ZP jog —
        so both serial links carry traffic at the same time. Relative + net-zero
        + envelope-clamped keeps the plate near its start position."""
        ctrl = self._controller
        a = float(cfg.xy_amp_um)
        steps = ((a, 0.0), (-a, 0.0), (0.0, a), (0.0, -a))  # net zero per loop
        i = 0
        while not stop.is_set() and not self._stop.is_set():
            dx, dy = steps[i % len(steps)]
            i += 1
            try:
                ctrl.move_xy_relative_um(dx, dy)
            except Exception as e:
                self._log_t(f"  XY move error: {e}")
                if not getattr(ctrl, "is_xy_connected", False):
                    return
            # Brief pace so the XY controller isn't flooded faster than it acks
            # while still keeping a steady concurrent stream.
            time.sleep(0.02)

    # ── Individual print-component bursts (isolation) ─────────────

    def _z_move_confirm(self, target_zref: float, label: str) -> None:
        """Move Z to ``target_zref`` (zero-ref mm) and confirm arrival exactly
        like the discrete print's MOVE_Z handler: move → suspend poller →
        M400 flush → wait-for-arrival → resume. So a descent burst reproduces
        the real print Z step (and its serial pattern) in isolation."""
        ctrl = self._controller
        ctrl.move_z_absolute(target_zref, from_zero_ref=True)
        suspended = False
        if hasattr(ctrl, "suspend_position_poller"):
            ctrl.suspend_position_poller()
            suspended = True
        try:
            zp = getattr(ctrl, "zp_stage", None)
            if zp is not None and hasattr(zp, "flush_moves"):
                if not zp.flush_moves(timeout_s=10.0):
                    self._log_t(f"  {label}: M400 not confirmed")
            if hasattr(ctrl, "wait_for_z_arrival"):
                if not ctrl.wait_for_z_arrival(float(target_zref),
                                               timeout_s=10.0):
                    self._log_t(f"  {label}: Z arrival not confirmed")
        finally:
            if suspended:
                ctrl.resume_position_poller()

    def _descend_burst(self, cfg: _StressConfig, check_link) -> None:
        """Isolate the print's MOVE_Z: descend Z toward the plate by
        ``descend_depth_mm`` (polarity-safe, clamped by soft limits + plate
        floor), confirm, then retract to Safe Z and confirm — looped. This is
        the one motion travel/jog never do; if it breaks the link while
        up-moves don't, the descent is the culprit."""
        ctrl = self._controller
        safe = float(self._safe_z)
        # Polarity-safe descent target: lower the *height* by the depth.
        try:
            target = ctrl.user_z_to_zref(
                ctrl.zref_to_user_z(safe) - float(cfg.descend_depth_mm))
        except Exception:
            target = safe - float(cfg.descend_depth_mm)
        try:
            ctrl.ensure_retracted_to(safe)
        except Exception:
            pass
        self._log_t(f"  descend burst: Safe {safe:.2f} ↔ "
                    f"{target:.2f} (depth {cfg.descend_depth_mm} mm) "
                    f"× {cfg.component_reps}")
        for i in range(cfg.component_reps):
            if self._stop.is_set():
                return
            try:
                self._z_move_confirm(target, "descend")      # DOWN
                self._z_move_confirm(safe, "retract")        # back UP
            except Exception as e:
                self._log_t(f"  descend rep {i + 1} error: {e}")
            if not check_link():
                return

    def _dispense_burst(self, cfg: _StressConfig, check_link) -> None:
        """Isolate the print's DISPENSE: a net-zero pump command stream."""
        ctrl = self._controller
        pump = self._first_pump() or "P1"
        rate = max(float(cfg.print_flow_uL_s), 0.25)
        amt = 0.05  # µL, tiny + net-zero
        self._log_t(f"  dispense burst: pump {pump} ±{amt} µL "
                    f"× {cfg.component_reps}")
        for i in range(cfg.component_reps):
            if self._stop.is_set():
                return
            try:
                if hasattr(ctrl, "move_pump_uL"):
                    ctrl.move_pump_uL(pump, +amt, rate)
                    ctrl.move_pump_uL(pump, -amt, rate)
                else:
                    ctrl.move_pump_relative(pump, +amt)
                    ctrl.move_pump_relative(pump, -amt)
            except Exception as e:
                self._log_t(f"  dispense rep {i + 1} error: {e}")
            if (i + 1) % 25 == 0 and not check_link():
                return

    def _first_pump(self) -> str | None:
        pumps = getattr(self._hw_config, "pumps", None) if self._hw_config else None
        if not pumps:
            return None
        for pid, pcfg in pumps.items():
            if getattr(pcfg, "enabled", True) and getattr(
                    pcfg, "is_configured", True):
                return pid
        return None

    def _print_well_center_mm(self):
        """Plate-centre (or first calibrated well) in zero-ref mm, or None."""
        ctrl = self._controller
        if self._well_positions:
            try:
                name = next(iter(self._well_positions))
                wx, wy = self._well_positions[name]
                zero = ctrl.zero_position
                return ((wx - zero["x"]) / 1000.0, (wy - zero["y"]) / 1000.0)
            except Exception:
                pass
        if self._plate is not None:
            try:
                cx, cy = ctrl.default_plate_center_um()
                zero = ctrl.zero_position
                return ((cx - zero["x"]) / 1000.0, (cy - zero["y"]) / 1000.0)
            except Exception:
                pass
        return None

    def _print_burst(self, cfg: _StressConfig, check_link) -> bool:
        """Run ``prints_per_cycle`` real discrete prints. Returns False if the
        stress test should STOP (operator stop, a print start error, a print
        that ended in ERROR, or the link going unhealthy) — so the caller never
        launches another print onto a degraded/dropped board."""
        ctrl = self._controller
        center = self._print_well_center_mm()
        if center is None:
            return True
        # Small circle (relative mm) — exercises MOVE_XY/MOVE_Z/DISPENSE/PRINT_PATH.
        r, n = 2.0, 48
        path = [(r * math.cos(2 * math.pi * k / n),
                 r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]

        # Print Z. With ``print_descend`` (default) the full print descends to
        # the REAL print height (plate bottom + print_height_mm) — moving Z into
        # position exactly like a real print, so the stress combines all
        # components: TRAVEL_UP → MOVE_XY → descend MOVE_Z → (prime) → PRINT_PATH
        # → retract. The descent is clamped by the soft limits + plate floor (it
        # cannot punch through the plate bottom), same as a real print. Uncheck
        # "Descend to print Z" to keep the path at safe Z (comms-only, no plunge).
        print_z = float(self._safe_z)
        if cfg.print_descend:
            try:
                z = ctrl.print_height_to_zref(float(cfg.print_height_mm))
                if z is not None:
                    print_z = float(z)
                else:
                    self._log_t("  print: no plate-bottom datum — staying at "
                                "Safe Z (no descent)")
            except Exception:
                pass
        self._log_t(
            f"  print: travel_z={self._safe_z:.2f} print_z={print_z:.2f} "
            f"({'DESCEND' if abs(print_z - self._safe_z) > 0.05 else 'no descent'})"
            f" flow={cfg.print_flow_uL_s}")
        pump = self._first_pump() or "P1"
        settings = PrintSettings(
            num_layers=1, travel_z_height=float(self._safe_z),
            print_z_height=print_z, pump_rate_uL_s=float(cfg.print_flow_uL_s),
            print_speed_mm_s=5.0, travel_speed_mm_s=10.0)
        try:
            settings.z_up_sign = float(ctrl.print_z_dir())
        except (TypeError, ValueError, AttributeError):
            pass

        for k in range(cfg.prints_per_cycle):
            if self._stop.is_set():
                return False
            job = build_well_plate_job(
                well_positions=[("stress", center[0], center[1])],
                path_points=path, settings=settings, pump=pump,
                flow_rate=float(cfg.print_flow_uL_s),
                job_name=f"Stress print {k + 1}", return_home=False)
            if cfg.use_simple_pm:
                from SupportClasses.SimplePrintManager import SimplePrintManager
                pm = SimplePrintManager(ctrl)
                _pm_label = "SimplePrintManager"
            else:
                pm = PrintManager(ctrl)
                _pm_label = "PrintManager"
            self._log_t(f"  print {k + 1}/{cfg.prints_per_cycle} starting… "
                        f"[{_pm_label}]")
            try:
                pm.load_job(job)
                pm.start()
            except Exception as e:
                self._log_t(f"  print {k + 1} start error: {e} — stopping test")
                return False
            # Wait for completion (or stop), watching the link.
            while getattr(pm, "state", None) == PrintState.RUNNING:
                if self._stop.is_set():
                    try:
                        pm.abort()
                    except Exception:
                        pass
                    return False
                time.sleep(0.1)
                check_link()
            st = getattr(pm, "state", None)
            if st == PrintState.ERROR:
                # A print that errored (Z not confirmed / ZP dropped mid-print)
                # is a HARD STOP — do not keep driving a degraded board.
                self._log_t(f"  print {k + 1} ended in ERROR — stopping test")
                return False
            self._log_t(f"  print {k + 1} complete.")
            if not check_link():
                return False
        return True
