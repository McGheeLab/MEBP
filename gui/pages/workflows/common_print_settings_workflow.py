"""common_print_settings_workflow.py — Workflows-mode "Common Print Settings".

v7.5.x: A central page collecting the settings common to every workflow:

  * **Pump (global)** — settle/dwell time, post-aspirate pressure relief, and
    prime time. These live on :class:`HardwareConfig` (the single source of
    truth the controller reads); editing them here updates that one value, which
    persists + propagates to Hardware Setup and every workflow.

  * **Needle prep (shared defaults)** — the prep knobs every prep workflow
    duplicates (service dip Z, prep flow, oil/buffer needle counts, wash cycles
    + amplitudes). Edited here as shared DEFAULTS; each workflow INHERITS them
    but can OVERRIDE locally in its own ⚙ Settings.

  * **Calibration heights** — read-only (Safe Z, plate bottom/top …) via the
    shared locations panel.

All edits go through the single :class:`CommonPrintSettings` model
(``set``-then-notify); ``gui/app.py`` owns it, persists, and fans changes out to
the workflow popouts so their inheriting fields re-sync live.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QDoubleSpinBox, QSpinBox, QCheckBox,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card, FormRow
from gui.dialogs.workflow_settings_dialog import build_locations_widget
from SupportClasses.CommonPrintSettings import CommonPrintSettings

logger = logging.getLogger(__name__)


class CommonPrintSettingsWorkflowPage(QWidget):
    """Central editor of the settings common to every workflow."""

    back_requested = Signal()

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None
        self._common: CommonPrintSettings | None = None
        # Calibration context for the read-only heights panel.
        self._plate = None
        self._well_positions: dict = {}
        self._safe_z = None
        self._z_references: dict = {}

        # key -> widget, for refresh on common fan-out.
        self._widgets: dict[str, QWidget] = {}
        self._refreshing = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(16), s(12), s(16), s(16))
        outer.setSpacing(s(12))

        # ── Header ──
        header = QHBoxLayout()
        header.setSpacing(s(8))
        back_btn = QPushButton("← Back to Workflows")
        back_btn.setCursor(Qt.PointingHandCursor)
        back_btn.clicked.connect(self.back_requested.emit)
        header.addWidget(back_btn)
        title = QLabel("Common Print Settings")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)
        outer.addLayout(header)

        intro = QLabel(
            "Settings shared by every workflow. Pump values are global (one "
            "value, used everywhere). Needle-prep values are shared defaults — "
            "each workflow inherits them but can override locally in its own "
            "⚙ Settings.")
        intro.setWordWrap(True)
        intro.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(intro)

        # ── Scroll body ──
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        body = QWidget()
        self._body = QVBoxLayout(body)
        self._body.setContentsMargins(0, 0, s(6), 0)
        self._body.setSpacing(s(10))
        scroll.setWidget(body)
        outer.addWidget(scroll, stretch=1)

        self._build_sections()

    # ── Spin builders ──────────────────────────────────────────────

    def _dspin(self, lo, hi, step, decimals, suffix):
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setSingleStep(step)
        w.setDecimals(decimals)
        if suffix:
            w.setSuffix(suffix)
        return w

    def _ispin(self, lo, hi):
        w = QSpinBox()
        w.setRange(lo, hi)
        return w

    def _add_row(self, card: Card, key: str, label: str, widget: QWidget,
                 help: str | None = None):
        self._widgets[key] = widget
        sig = getattr(widget, "valueChanged", None)
        if sig is not None:
            sig.connect(lambda _v, k=key: self._on_edit(k))
        row = FormRow(label, widget, help_text=help)
        if help:
            row.set_help_visible(True)
        card.add_widget(row)
        return widget

    def _build_sections(self):
        # ── Pump (global) ──
        pump = Card("Pump (global — applies to every workflow)")
        self._body.addWidget(pump)
        self._add_row(
            pump, "pump_settle_time_s", "Dwell after syringe moves",
            self._dspin(0.0, 30.0, 0.05, 2, " s"),
            "Settle dwell held before AND after every discrete pump move "
            "(aspirate / dispense / prep) so the fluid settles before the "
            "workflow advances.")
        self._add_row(
            pump, "pump_prime_time_s", "Prime time",
            self._dspin(0.0, 30.0, 0.05, 2, " s"),
            "Pre-flow lead-in printing workflows use to prime the needle "
            "(prime volume = flow × time).")
        # v7.21.7 — NOT a duplicate of the settle dwell above, and the help text
        # has to say why: the settle dwell brackets a pump move that already
        # blocks until the PLUNGER has finished, whereas this covers the FLUID
        # still being drawn afterwards through a compliant column.
        self._add_row(
            pump, "pump_post_aspirate_dwell_s", "Hold in liquid after aspirating",
            self._dspin(0.0, 120.0, 0.5, 2, " s"),
            "Extra time the needle stays IN THE LIQUID after a reagent aspirate "
            "(ink / oil / buffer) finishes, before Z retracts and the stage "
            "travels on. The pump move already waits for the plunger, but the "
            "fluid column is compressible — with a fine bore or a viscous ink "
            "liquid keeps being drawn in after the plunger stops, so lifting the "
            "needle too early finishes the aspirate in AIR. Raise it if a pickup "
            "ends with air in the needle. 0 = no extra hold.")

        # ── Pressure relief / compliance (per pump) ──
        self._relief_spins: dict = {}
        relief = Card("Pressure relief / compliance (per pump)")
        self._body.addWidget(relief)
        relief_note = QLabel(
            "Per-pump compliance value (µL) — the plunger travel that removes "
            "the drivetrain/syringe flex, measured on Calibration → Needle "
            "Location (dispense in small steps to a droplet, aspirate it back; "
            "the relief is HALF that aspirate). When backlash compensation is "
            "on, every discrete pump start/stop takes up the flex then unloads "
            "it — so small volumes flow accurately and no residual pressure "
            "remains during moves.")
        relief_note.setWordWrap(True)
        relief_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        relief.add_widget(relief_note)
        self._backlash_chk = QCheckBox("Enable backlash compensation (all pumps)")
        self._backlash_chk.setToolTip(
            "Bracket every discrete pump actuation and jog with a flex take-up "
            "(before) + unload (after), sized per-pump from the values below. "
            "Also toggled from the pump jog panel.")
        self._backlash_chk.toggled.connect(self._on_backlash_toggled)
        relief.add_widget(self._backlash_chk)
        for pid in ("P1", "P2", "P3"):
            spin = self._dspin(0.0, 50.0, 0.01, 3, " µL")
            spin.valueChanged.connect(
                lambda _v, p=pid: self._on_relief_edit(p))
            self._relief_spins[pid] = spin
            relief.add_widget(FormRow(f"{pid} relief", spin))

        # ── Gentle Z near the plate (global) ──
        gz = Card("Gentle Z near the plate (global — applies to every workflow)")
        self._body.addWidget(gz)
        gz_note = QLabel(
            "Eases the needle in and out of every print / work position: the "
            "first part of a LIFT out of a print and the last part of a DESCENT "
            "back down both run at the slow speed below (the rest is fast). The "
            "slow lift keeps a laid bead from peeling up with the needle; the "
            "slow descent is a controlled touch-down. Set distance to 0 to "
            "disable (single-speed).")
        gz_note.setWordWrap(True)
        gz_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        gz.add_widget(gz_note)
        self._add_row(
            gz, "gentle_z_slow_dist_mm", "Slow zone distance",
            self._dspin(0.0, 20.0, 0.5, 2, " mm"),
            "How far the slow segment covers — the first N mm of a lift and the "
            "last N mm of a descent. 0 disables the gentle easing entirely.")
        self._add_row(
            gz, "gentle_z_slow_speed_mm_s", "Slow speed",
            self._dspin(0.1, 50.0, 0.5, 2, " mm/s"),
            "Speed of the slow lift-off and slow touch-down segments.")

        # ── Needle prep (shared defaults) ──
        prep = Card("Needle prep (shared defaults — workflows inherit these)")
        self._body.addWidget(prep)
        self._add_row(
            prep, "service_z", "Service dip Z (↑ bottom)",
            self._dspin(0.0, 30.0, 0.1, 2, " mm"),
            "Needle dip height above the plate bottom at the service wells.")
        self._add_row(
            prep, "prep_rate", "Prep / clean flow",
            self._dspin(0.01, 50.0, 0.1, 2, " µL/s"),
            "Aspirate / dispense flow during prep + clean.")
        self._add_row(
            prep, "oil_needles", "Oil (needles)",
            self._dspin(0.0, 20.0, 0.5, 1, ""),
            "Needles of oil dispensed to waste AND aspirated from the oil well.")
        self._add_row(
            prep, "buffer_needles", "Buffer (needles)",
            self._dspin(0.0, 20.0, 0.5, 1, ""),
            "Needles of buffer aspirated after the wash.")
        self._add_row(
            prep, "wash_cycles", "Wash cycles", self._ispin(0, 50),
            "Dip-jiggle (Z + random XY) cycles at the wash well.")
        self._add_row(
            prep, "wash_z_amp", "Wash Z jiggle",
            self._dspin(0.0, 10.0, 0.1, 2, " mm"),
            "How far up/down each wash jiggle moves.")
        self._add_row(
            prep, "wash_xy_amp", "Wash XY jiggle",
            self._dspin(0.0, 5000.0, 10.0, 0, " µm"),
            "Random XY radius about the well centre during the wash.")
        self._add_row(
            prep, "wash_dwell", "Wash settle",
            self._dspin(0.0, 30.0, 0.1, 2, " s"),
            "Settle time between wash jiggles.")

        # ── Calibration heights (read-only) ──
        self._heights_card = Card("Calibration heights (read-only)")
        self._body.addWidget(self._heights_card)
        self._refresh_heights_panel()

        self._body.addStretch(1)

    # ── Edits → model ──────────────────────────────────────────────

    def _on_edit(self, key: str):
        if self._refreshing or self._common is None:
            return
        w = self._widgets.get(key)
        if w is None:
            return
        val = w.value()
        self._common.set(key, val)

    # ── Per-pump relief / backlash (controller + device profile) ───

    def _on_relief_edit(self, pump: str):
        if self._refreshing:
            return
        ctrl = self._controller
        spin = self._relief_spins.get(pump)
        if ctrl is None or spin is None or not hasattr(ctrl, "set_pump_relief_uL"):
            return
        try:
            ctrl.set_pump_relief_uL(pump, float(spin.value()))
        except Exception as exc:
            logger.debug("set_pump_relief_uL(%s) failed: %s", pump, exc)
            return
        self._persist_relief()

    def _on_backlash_toggled(self, on: bool):
        if self._refreshing:
            return
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_backlash_comp_enabled"):
            try:
                ctrl.set_backlash_comp_enabled(bool(on))
            except Exception as exc:
                logger.debug("set_backlash_comp_enabled failed: %s", exc)
        s = self._settings
        if s is not None:
            try:
                s.set("device_profile.backlash_comp_enabled", bool(on))
                s.save()
            except Exception as exc:
                logger.debug("persist backlash toggle failed: %s", exc)

    def _persist_relief(self):
        ctrl = self._controller
        s = self._settings
        if ctrl is None or s is None or not hasattr(ctrl, "get_pump_relief_all"):
            return
        try:
            s.set("device_profile.pump_compliance_uL", ctrl.get_pump_relief_all())
            s.save()
        except Exception as exc:
            logger.debug("persist pump compliance failed: %s", exc)

    def _refresh_relief_widgets(self):
        ctrl = self._controller
        if ctrl is None or not hasattr(self, "_relief_spins"):
            return
        self._refreshing = True
        try:
            if (hasattr(self, "_backlash_chk")
                    and hasattr(ctrl, "backlash_comp_enabled")):
                self._backlash_chk.blockSignals(True)
                try:
                    self._backlash_chk.setChecked(bool(ctrl.backlash_comp_enabled()))
                finally:
                    self._backlash_chk.blockSignals(False)
            get = getattr(ctrl, "pump_relief_uL", None)
            for pid, spin in self._relief_spins.items():
                try:
                    v = float(get(pid)) if callable(get) else 0.0
                except Exception:
                    v = 0.0
                spin.blockSignals(True)
                spin.setValue(v)
                spin.blockSignals(False)
        finally:
            self._refreshing = False

    # ── Refresh widgets from the model ─────────────────────────────

    def _refresh_widgets(self):
        if self._common is None:
            return
        self._refreshing = True
        try:
            for key, w in self._widgets.items():
                cv = self._common.get(key)
                if cv is None:
                    continue
                w.blockSignals(True)
                try:
                    if isinstance(w, QSpinBox):
                        w.setValue(int(round(float(cv))))
                    else:
                        w.setValue(float(cv))
                finally:
                    w.blockSignals(False)
        finally:
            self._refreshing = False

    def _refresh_heights_panel(self):
        lay = self._heights_card.body_layout()
        while lay.count():
            item = lay.takeAt(0)
            wdg = item.widget()
            if wdg is not None:
                wdg.setParent(None)
                wdg.deleteLater()
        try:
            panel = build_locations_widget(
                self._controller, self._hw_config, self._well_positions,
                z_references=self._z_references, safe_z=self._safe_z)
        except Exception as exc:
            logger.debug("heights panel build failed: %s", exc)
            panel = QLabel("(hardware info unavailable)")
        self._heights_card.add_widget(panel)

    # ── External wiring (called by WorkflowsModePage / app.py) ─────

    def set_common_print_settings(self, common: CommonPrintSettings):
        self._common = common
        self._refresh_widgets()

    def set_hardware_config(self, hw_config):
        self._hw_config = hw_config
        # The model proxies the globals from this config.
        if self._common is not None:
            try:
                self._common.set_hardware_config(hw_config)
            except Exception:
                pass
        self._refresh_widgets()
        self._refresh_relief_widgets()
        self._refresh_heights_panel()

    def set_calibration_data(self, plate, well_positions, safe_z):
        self._plate = plate
        self._well_positions = well_positions or {}
        self._safe_z = safe_z
        self._refresh_heights_panel()

    def set_z_references(self, refs):
        self._z_references = refs or {}
        self._refresh_heights_panel()

    def set_settings(self, settings):
        self._settings = settings

    def get_page_title(self) -> str:
        return "Common Print Settings"

    def get_context_widget(self):
        return None

    def showEvent(self, event):
        self._refresh_widgets()
        self._refresh_relief_widgets()
        self._refresh_heights_panel()
        super().showEvent(event)
