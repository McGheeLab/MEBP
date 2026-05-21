"""
wizard.py — First-run onboarding wizard (v7.4.0-c).

A 5-step modal QDialog that orients new users to MEBP and ensures the
minimum hardware configuration needed to unlock the rest of the app.

Steps
    1. Welcome           — what MEBP is, what we'll set up
    2. Stages            — simulation/real toggle + non-blocking Test Connection
    3. Plate & Needle    — well plate format + needle gauge in one panel
    4. Pumps & Inks      — explanation + deep-link to Hardware Setup
    5. Ready             — confirms what was configured, loads prefab inks

The wizard owns a fresh :class:`HardwareConfig`. On Finish it returns the
config to MainWindow, which calls :meth:`HardwareSetupPage.set_config()`
to push it through the normal config_changed pipeline.

Trigger condition (in MainWindow): the wizard appears when
``settings.get("hardware_config.needle.gauge")`` is falsy AND
``not settings.get("workspace.needle_gauge")`` — i.e. no needle has
ever been configured. After a successful run, those keys are set
and the wizard does not re-fire.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Callable

from PySide6.QtCore import Qt, QThread, Signal, QObject
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QStackedWidget, QWidget, QLabel,
    QPushButton, QCheckBox, QComboBox, QFrame, QSizePolicy, QApplication,
)

from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.PhysicalModels import (
    NeedleSpec, InkSpec, load_needle_catalog,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS
from gui.styles import COLORS
from gui.scaling import s, sf, sp, scaled_font_size
from gui.widgets.components import (
    Card, StatusBadge, SectionHeader, WizardStep,
)

logger = logging.getLogger(__name__)


PREFAB_INKS_PATH = Path(__file__).parent / "prefab_inks.json"


# ════════════════════════════════════════════════════════════════════
#  Test-connection worker (non-blocking)
# ════════════════════════════════════════════════════════════════════

class _ConnectProbeWorker(QObject):
    """v7.4.0-c: Run a stage probe on a QThread so the UI stays responsive.

    Emits :attr:`finished` with (axis_label, ok, message). Uses the
    blocking ``StageController.connect_xy`` / ``connect_zp`` paths
    because the controller handles its own error cases gracefully —
    we just need to keep the UI responsive while the probe runs.
    """

    finished = Signal(str, bool, str)

    def __init__(self, controller, axis: str):
        super().__init__()
        self._controller = controller
        self._axis = axis  # "XY" or "ZP"

    def run(self):
        axis = self._axis
        try:
            if axis == "XY":
                self._controller.connect_xy()
                ok = self._controller.is_xy_connected
            else:
                self._controller.connect_zp()
                ok = self._controller.is_zp_connected
            msg = f"{axis} connected" if ok else f"{axis} not detected"
            self.finished.emit(axis, ok, msg)
        except Exception as e:
            logger.warning(f"Test-connection probe ({axis}) failed: {e}")
            self.finished.emit(axis, False, f"{axis} error: {e}")


# ════════════════════════════════════════════════════════════════════
#  OnboardingWizard
# ════════════════════════════════════════════════════════════════════

class OnboardingWizard(QDialog):
    """Modal first-run wizard. See module docstring for flow."""

    # Emitted on Finish with the configured HardwareConfig. MainWindow
    # subscribes and applies it.
    completed = Signal(object)

    # Emitted if the user opts to deep-link to a specific page after the
    # wizard. Payload is the integer page index (0 = Hardware Setup).
    deep_link_requested = Signal(int)

    _TOTAL_STEPS = 5

    def __init__(self, controller, settings, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._config = HardwareConfig()
        self._config.config_name = "First-Run Setup"
        self._deep_link_target: int | None = None
        self._probe_threads: list[QThread] = []

        self.setWindowTitle("Welcome to MEBP")
        self.setModal(True)
        self.setMinimumSize(s(640), s(540))

        self._needle_catalog = load_needle_catalog()
        self._setup_ui()

    # ── UI scaffold ──────────────────────────────────────────────

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Header banner with branding
        header = QFrame()
        header.setObjectName("wizardHeader")
        header.setStyleSheet(
            f"#wizardHeader {{"
            f"  background-color: {COLORS['mantle']};"
            f"  border-bottom: 1px solid {COLORS['surface1']};"
            f"}}"
        )
        header.setFixedHeight(s(56))
        h_lay = QHBoxLayout(header)
        h_lay.setContentsMargins(s(20), 0, s(20), 0)
        logo = QLabel("🧬 MEBP")
        logo.setStyleSheet(
            f"color: {COLORS['mauve']}; font-size: {sf(18)}pt; font-weight: 700;")
        h_lay.addWidget(logo)
        h_lay.addStretch(1)
        subtitle = QLabel("Onboarding")
        subtitle.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(11)}pt;")
        h_lay.addWidget(subtitle)
        outer.addWidget(header)

        # Stack of wizard steps
        self._stack = QStackedWidget()
        outer.addWidget(self._stack, 1)

        # Build each step
        self._build_step_welcome()
        self._build_step_stages()
        self._build_step_plate_needle()
        self._build_step_pumps_inks()
        self._build_step_ready()

        self._stack.setCurrentIndex(0)

    # ── Step 1: Welcome ──────────────────────────────────────────

    def _build_step_welcome(self):
        body = QWidget()
        b_lay = QVBoxLayout(body)
        b_lay.setSpacing(s(12))

        intro = QLabel(
            "<p>MEBP orchestrates Prior ProScan XY stages, Marlin Z/pump "
            "controllers, and Hamilton syringes to deposit biological "
            "materials into standard well plates.</p>"
            "<p>This wizard walks you through the few settings you need "
            "to unlock the rest of the app:</p>"
            "<ul>"
            "<li><b>Stages</b> — connect real hardware or use simulation</li>"
            "<li><b>Plate & Needle</b> — what you're printing into and with</li>"
            "<li><b>Pumps & Inks</b> — what materials you'll print</li>"
            "</ul>"
            "<p>You can revisit any of this at any time from Hardware "
            "Setup.</p>"
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['text']}; font-size: {sf(11)}pt;")
        b_lay.addWidget(intro)
        b_lay.addStretch(1)

        step = WizardStep(1, self._TOTAL_STEPS, "Welcome", body, is_first=True)
        step.next_clicked.connect(lambda: self._stack.setCurrentIndex(1))
        self._stack.addWidget(step)

    # ── Step 2: Stages ───────────────────────────────────────────

    def _build_step_stages(self):
        body = QWidget()
        b_lay = QVBoxLayout(body)
        b_lay.setSpacing(s(12))

        intro = QLabel(
            "Choose whether to connect real hardware or run in simulation. "
            "Simulation is fully featured and recommended for first-time setup."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        b_lay.addWidget(intro)

        # Simulation toggles
        sim_card = Card("Simulation mode")
        self._chk_sim_xy = QCheckBox("Simulate XY stage")
        self._chk_sim_xy.setChecked(self._controller.simulate_xy)
        sim_card.add_widget(self._chk_sim_xy)

        self._chk_sim_zp = QCheckBox("Simulate Z and pump axes")
        self._chk_sim_zp.setChecked(self._controller.simulate_zp)
        sim_card.add_widget(self._chk_sim_zp)

        sim_note = QLabel(
            "Simulation flags take effect on next launch. The Test "
            "buttons below use the current mode."
        )
        sim_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sim_note.setWordWrap(True)
        sim_card.add_widget(sim_note)
        b_lay.addWidget(sim_card)

        # Test connection card
        probe_card = Card("Test connection")

        xy_row = QHBoxLayout()
        self._btn_test_xy = QPushButton("Test XY")
        self._btn_test_xy.setObjectName("accentBtn")
        self._btn_test_xy.clicked.connect(lambda: self._start_probe("XY"))
        xy_row.addWidget(self._btn_test_xy)
        self._badge_xy = StatusBadge("Not tested", "pending")
        xy_row.addWidget(self._badge_xy)
        xy_row.addStretch(1)
        probe_card.add_layout(xy_row)

        zp_row = QHBoxLayout()
        self._btn_test_zp = QPushButton("Test ZP")
        self._btn_test_zp.setObjectName("accentBtn")
        self._btn_test_zp.clicked.connect(lambda: self._start_probe("ZP"))
        zp_row.addWidget(self._btn_test_zp)
        self._badge_zp = StatusBadge("Not tested", "pending")
        zp_row.addWidget(self._badge_zp)
        zp_row.addStretch(1)
        probe_card.add_layout(zp_row)

        b_lay.addWidget(probe_card)
        b_lay.addStretch(1)

        step = WizardStep(2, self._TOTAL_STEPS, "Stages", body)
        step.prev_clicked.connect(lambda: self._stack.setCurrentIndex(0))
        step.next_clicked.connect(self._stages_next)
        self._stack.addWidget(step)

    def _stages_next(self):
        # Persist simulation flags now (require restart to take effect)
        self._settings.set("simulation.simulate_xy", self._chk_sim_xy.isChecked())
        self._settings.set("simulation.simulate_zp", self._chk_sim_zp.isChecked())
        self._stack.setCurrentIndex(2)

    def _start_probe(self, axis: str):
        badge = self._badge_xy if axis == "XY" else self._badge_zp
        btn = self._btn_test_xy if axis == "XY" else self._btn_test_zp
        badge.set_status("info", f"Probing {axis}…")
        btn.setEnabled(False)

        thread = QThread(self)
        worker = _ConnectProbeWorker(self._controller, axis)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(self._on_probe_finished)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        self._probe_threads.append(thread)
        thread.start()

    def _on_probe_finished(self, axis: str, ok: bool, message: str):
        badge = self._badge_xy if axis == "XY" else self._badge_zp
        btn = self._btn_test_xy if axis == "XY" else self._btn_test_zp
        badge.set_status("ok" if ok else "err", message)
        btn.setEnabled(True)

    # ── Step 3: Plate & Needle ───────────────────────────────────

    def _build_step_plate_needle(self):
        body = QWidget()
        b_lay = QVBoxLayout(body)
        b_lay.setSpacing(s(12))

        # Plate format
        plate_card = Card("Well plate format")
        plate_row = QHBoxLayout()
        plate_row.addWidget(QLabel("Format:"))
        self._plate_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            pdef = PLATE_DEFINITIONS[fmt]
            rows = pdef.get("rows", "?")
            cols = pdef.get("cols", "?")
            self._plate_combo.addItem(f"{fmt}-well ({rows}×{cols})", fmt)
        # Default to current config plate format
        idx = self._plate_combo.findData(self._config.plate_format)
        if idx >= 0:
            self._plate_combo.setCurrentIndex(idx)
        plate_row.addWidget(self._plate_combo, 1)
        plate_card.add_layout(plate_row)
        b_lay.addWidget(plate_card)

        # Needle
        needle_card = Card("Needle")
        nrow = QHBoxLayout()
        nrow.addWidget(QLabel("Gauge:"))
        self._gauge_combo = QComboBox()
        self._gauge_combo.addItem("— Select —", None)
        for g in sorted(self._needle_catalog.keys()):
            self._gauge_combo.addItem(f"{g}G", g)
        nrow.addWidget(self._gauge_combo, 1)
        needle_card.add_layout(nrow)

        needle_note = QLabel(
            "16G is the thickest, 32G the thinnest. You can change "
            "this later on Hardware Setup → Needle."
        )
        needle_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        needle_note.setWordWrap(True)
        needle_card.add_widget(needle_note)

        b_lay.addWidget(needle_card)
        b_lay.addStretch(1)

        step = WizardStep(3, self._TOTAL_STEPS, "Plate & Needle", body)
        step.prev_clicked.connect(lambda: self._stack.setCurrentIndex(1))
        step.next_clicked.connect(self._plate_needle_next)
        # Disable Next until a gauge is selected
        step.set_next_enabled(False)
        self._step3 = step
        self._gauge_combo.currentIndexChanged.connect(self._on_gauge_picked)
        self._stack.addWidget(step)

    def _on_gauge_picked(self, _idx):
        gauge = self._gauge_combo.currentData()
        self._step3.set_next_enabled(gauge is not None)

    def _plate_needle_next(self):
        gauge = self._gauge_combo.currentData()
        if gauge is None:
            return
        # Commit to config
        self._config.plate_format = int(self._plate_combo.currentData())
        needle_spec = self._needle_catalog.get(gauge)
        if needle_spec is not None:
            self._config.needle = needle_spec
        self._stack.setCurrentIndex(3)

    # ── Step 4: Pumps & Inks (deep-link) ─────────────────────────

    def _build_step_pumps_inks(self):
        body = QWidget()
        b_lay = QVBoxLayout(body)
        b_lay.setSpacing(s(12))

        intro = QLabel(
            "<p>Configuring pumps and inks is where you describe what "
            "materials you'll print and which syringe-pump each one "
            "lives in.</p>"
            "<p>It's more detailed than what fits in this wizard — we'll "
            "open <b>Hardware Setup → Pumps & Inks</b> for you when "
            "the wizard finishes.</p>"
            "<p>To get you started, MEBP will load a small library of "
            "common ink presets (PBS, Alginate 2%, GelMA, Cell Media, "
            "Water). You can edit, delete, or add to these any time.</p>"
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {COLORS['text']}; font-size: {sf(11)}pt;")
        b_lay.addWidget(intro)

        # Toggle for the prefab inks
        self._chk_load_prefabs = QCheckBox(
            "Load starter ink library (recommended)")
        self._chk_load_prefabs.setChecked(True)
        b_lay.addWidget(self._chk_load_prefabs)

        b_lay.addStretch(1)

        step = WizardStep(4, self._TOTAL_STEPS, "Pumps & Inks", body)
        step.prev_clicked.connect(lambda: self._stack.setCurrentIndex(2))
        step.next_clicked.connect(lambda: self._stack.setCurrentIndex(4))
        self._stack.addWidget(step)

    # ── Step 5: Ready ────────────────────────────────────────────

    def _build_step_ready(self):
        body = QWidget()
        b_lay = QVBoxLayout(body)
        b_lay.setSpacing(s(12))

        self._summary_label = QLabel("")
        self._summary_label.setWordWrap(True)
        self._summary_label.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {sf(11)}pt;")
        b_lay.addWidget(self._summary_label)

        deep_link_card = Card("Open Hardware Setup → Pumps & Inks after this")
        self._chk_deep_link = QCheckBox(
            "Take me to Hardware Setup → Pumps & Inks when I click Finish")
        self._chk_deep_link.setChecked(True)
        deep_link_card.add_widget(self._chk_deep_link)
        b_lay.addWidget(deep_link_card)

        b_lay.addStretch(1)

        step = WizardStep(5, self._TOTAL_STEPS, "Ready", body, is_last=True)
        step.prev_clicked.connect(lambda: self._stack.setCurrentIndex(3))
        step.completed.connect(self._on_finish)
        self._stack.addWidget(step)

    def showEvent(self, e):
        super().showEvent(e)
        # Rebuild summary every time the step is shown (in case user
        # back-navigates and changes earlier steps).
        self._refresh_summary()

    def _refresh_summary(self):
        if not hasattr(self, '_summary_label'):
            return
        plate = self._config.plate_format
        needle = self._config.needle
        needle_str = f"{needle.gauge}G" if needle else "(not set)"
        prefab_count = self._count_prefab_inks() if getattr(
            self, '_chk_load_prefabs', None) and \
            self._chk_load_prefabs.isChecked() else 0
        self._summary_label.setText(
            f"<p>You're all set! Here's what will be saved:</p>"
            f"<ul>"
            f"<li><b>Simulation:</b> XY={'on' if self._chk_sim_xy.isChecked() else 'off'}, "
            f"ZP={'on' if self._chk_sim_zp.isChecked() else 'off'} "
            f"(takes effect on next launch)</li>"
            f"<li><b>Plate:</b> {plate}-well</li>"
            f"<li><b>Needle gauge:</b> {needle_str}</li>"
            f"<li><b>Starter inks:</b> {prefab_count}</li>"
            f"</ul>"
        )

    @staticmethod
    def _count_prefab_inks() -> int:
        try:
            with open(PREFAB_INKS_PATH) as f:
                data = json.load(f)
            return len(data.get("inks", {}))
        except Exception:
            return 0

    # ── Finish ───────────────────────────────────────────────────

    def _on_finish(self):
        # Load prefab inks if requested
        if self._chk_load_prefabs.isChecked():
            self._load_prefab_inks()
        # Determine deep-link target
        if self._chk_deep_link.isChecked():
            # Hardware Setup is page 0; the wizard caller will switch
            # to it and we'll have set the active sub-page to Pumps & Inks
            self._deep_link_target = 0
        # Persist a marker so we don't re-trigger
        self._settings.set("workspace.needle_gauge",
                           self._config.needle.gauge if self._config.needle else None)
        self._settings.set("workspace.plate_format", self._config.plate_format)
        self._settings.save()
        # Emit completion
        self.completed.emit(self._config)
        self.accept()

    def _load_prefab_inks(self):
        try:
            with open(PREFAB_INKS_PATH) as f:
                data = json.load(f)
            inks = data.get("inks", {})
            for ink_name, ink_dict in inks.items():
                # Strip the wrapper "name" field — InkSpec accepts a dict
                # mirroring its fields.
                spec = InkSpec(
                    name=ink_dict.get("name", ink_name),
                    ink_type=ink_dict.get("ink_type", "custom"),
                    viscosity_cP=float(ink_dict.get("viscosity_cP", 1.0)),
                    granule_diameter_um=float(ink_dict.get("granule_diameter_um", 0.0)),
                    cell_diameter_um=float(ink_dict.get("cell_diameter_um", 0.0)),
                    density_g_mL=float(ink_dict.get("density_g_mL", 1.0)),
                    color=ink_dict.get("color", "#a6e3a1"),
                )
                self._config.ink_library[ink_name] = spec
            logger.info(f"Loaded {len(inks)} prefab inks")
        except Exception as e:
            logger.warning(f"Failed to load prefab inks: {e}")

    def get_deep_link_target(self) -> int | None:
        """Return the page index the user wants to land on, or None."""
        return self._deep_link_target

    @property
    def hardware_config(self) -> HardwareConfig:
        return self._config


# ════════════════════════════════════════════════════════════════════
#  Trigger helper for MainWindow
# ════════════════════════════════════════════════════════════════════

def should_show_onboarding(settings) -> bool:
    """Return True if the user has never completed first-run setup.

    Trigger condition: no needle gauge has been saved (the most reliable
    signal that Hardware Setup was never completed).
    """
    needle_gauge = settings.get("workspace.needle_gauge")
    if needle_gauge:
        return False
    # Also check the hardware_config last_config_file — if a saved
    # config exists, the user probably already completed setup once.
    if settings.get("hardware_config.last_config_file"):
        return False
    return True
