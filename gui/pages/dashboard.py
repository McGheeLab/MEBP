"""
Dashboard Page — Device status overview.

Main content: Position readouts, speeds, zero reference, safety status, print history
Context panel: Connection management, Xbox mapping, position log controls

v7.1.1: XY positions displayed in microns (µm) instead of raw microsteps.
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QFrame, QSizePolicy, QScrollArea,
)
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from SupportClasses.PrintHistory import PrintHistory
from gui.styles import COLORS, SECTION_TITLE_STYLE

try:
    from SupportClasses.HardwareConfig import HardwareConfig
except ImportError:
    HardwareConfig = None

logger = logging.getLogger(__name__)


class _ConnectWorker(QThread):
    """Run a blocking stage-connect call off the main thread."""
    finished = Signal(bool, str)  # (success, error_message)

    def __init__(self, fn):
        super().__init__()
        self._fn = fn

    def run(self):
        try:
            self._fn()
            self.finished.emit(True, "")
        except Exception as e:
            self.finished.emit(False, str(e))


class DashboardPage(QWidget):
    """Dashboard: read-only overview of device state."""

    def __init__(self, controller: StageController,
                 print_history: PrintHistory | None = None,
                 settings: dict | None = None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.print_history = print_history
        self.settings = settings or {}
        self._context_widget = None

        # Microsteps per micron — set by MainWindow
        self._microsteps_per_micron: float = 10.0
        self._hardware_config = None  # v7.2: HardwareConfig for µL display

        self._setup_ui()

    def get_page_title(self) -> str:
        return "Dashboard"

    def set_microsteps_per_micron(self, value: float):
        """Called by MainWindow when the conversion factor changes."""
        self._microsteps_per_micron = max(0.001, value)

    def set_hardware_config(self, config):
        """v7.2: Set hardware config for µL pump display."""
        self._hardware_config = config


    def get_context_widget(self) -> QWidget:
        """Context panel: connection controls + log management."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        # ── Connections ──────────────────────────────────────────
        conn_label = QLabel("Connections")
        conn_label.setObjectName("contextSectionLabel")
        layout.addWidget(conn_label)

        for name, connect_fn, disconnect_fn in [
            ("XY Stage", self._connect_xy, self._disconnect_xy),
            ("ZP Stage", self._connect_zp, self._disconnect_zp),
            ("Xbox Controller", self._connect_xbox, self._disconnect_xbox),
        ]:
            row = QHBoxLayout()
            self._ctx_status = QLabel("●")
            self._ctx_status.setObjectName("connDotOff")
            self._ctx_status.setFixedWidth(14)
            row.addWidget(self._ctx_status)
            row.addWidget(QLabel(name), stretch=1)

            btn_conn = QPushButton("Connect")
            btn_conn.setObjectName("connectBtn")
            btn_conn.clicked.connect(connect_fn)
            row.addWidget(btn_conn)

            btn_disc = QPushButton("✕")
            btn_disc.setObjectName("dangerBtn")
            btn_disc.setFixedWidth(30)
            btn_disc.clicked.connect(disconnect_fn)
            row.addWidget(btn_disc)

            layout.addLayout(row)

            # Store status and button references
            attr_prefix = name.split()[0].lower()
            setattr(self, f'ctx_dot_{attr_prefix}', self._ctx_status)
            setattr(self, f'_btn_connect_{attr_prefix}', btn_conn)

        # ── Xbox Mapping Editor ──────────────────────────────────
        btn_xbox_edit = QPushButton("Xbox Mapping Editor...")
        btn_xbox_edit.setObjectName("accentBtn")
        btn_xbox_edit.clicked.connect(self._open_xbox_editor)
        layout.addWidget(btn_xbox_edit)

        # ── Position Log ─────────────────────────────────────────
        log_label = QLabel("Position Log")
        log_label.setObjectName("contextSectionLabel")
        layout.addWidget(log_label)

        self.ctx_lbl_log_count = QLabel("Entries: 0")
        self.ctx_lbl_log_count.setObjectName("contextLabel")
        layout.addWidget(self.ctx_lbl_log_count)

        log_btns = QHBoxLayout()
        btn_export_csv = QPushButton("Export CSV")
        btn_export_csv.clicked.connect(self._export_log_csv)
        log_btns.addWidget(btn_export_csv)

        btn_export_json = QPushButton("Export JSON")
        btn_export_json.clicked.connect(self._export_log_json)
        log_btns.addWidget(btn_export_json)
        layout.addLayout(log_btns)

        # ── Print History ────────────────────────────────────────
        hist_label = QLabel("Print History")
        hist_label.setObjectName("contextSectionLabel")
        layout.addWidget(hist_label)

        hist_btns = QHBoxLayout()
        btn_export_hist = QPushButton("Export")
        btn_export_hist.clicked.connect(self._export_history)
        hist_btns.addWidget(btn_export_hist)

        btn_clear_hist = QPushButton("Clear")
        btn_clear_hist.setObjectName("warningBtn")
        btn_clear_hist.clicked.connect(self._clear_history)
        hist_btns.addWidget(btn_clear_hist)
        layout.addLayout(hist_btns)

        layout.addStretch()
        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        _bg = COLORS['base']
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(f"QScrollArea {{ background-color: {_bg}; border: none; }}")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        container = QWidget()
        container.setStyleSheet(f"background-color: {_bg};")
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        layout.setContentsMargins(16, 12, 16, 12)
        scroll.setWidget(container)

        mono = QFont("Consolas", 12)
        small_mono = QFont("Consolas", 10)

        # ── Positions row (XY + ZP side by side) ─────────────────
        pos_row = QHBoxLayout()
        pos_row.setSpacing(8)

        # XY card (displayed in µm)
        xy_card = QFrame()
        xy_card.setObjectName("cardFrame")
        xy_layout = QVBoxLayout(xy_card)
        xy_layout.setSpacing(4)

        xy_title = QLabel("XY Stage Position (µm)")
        xy_title.setObjectName("sectionLabel")
        xy_layout.addWidget(xy_title)

        xy_grid = QGridLayout()
        xy_grid.setSpacing(4)
        xy_grid.addWidget(QLabel("X:"), 0, 0)
        self.lbl_x = QLabel("—")
        self.lbl_x.setFont(mono)
        self.lbl_x.setObjectName("valueLabel")
        xy_grid.addWidget(self.lbl_x, 0, 1)
        lbl_x_unit = QLabel("µm")
        lbl_x_unit.setObjectName("unitLabel")
        xy_grid.addWidget(lbl_x_unit, 0, 2)

        xy_grid.addWidget(QLabel("Y:"), 0, 3)
        self.lbl_y = QLabel("—")
        self.lbl_y.setFont(mono)
        self.lbl_y.setObjectName("valueLabel")
        xy_grid.addWidget(self.lbl_y, 0, 4)
        lbl_y_unit = QLabel("µm")
        lbl_y_unit.setObjectName("unitLabel")
        xy_grid.addWidget(lbl_y_unit, 0, 5)

        self.lbl_xy_status = QLabel("Disconnected")
        self.lbl_xy_status.setStyleSheet(f"color: {COLORS['red']};")
        xy_grid.addWidget(self.lbl_xy_status, 1, 0, 1, 6)

        xy_layout.addLayout(xy_grid)
        pos_row.addWidget(xy_card)

        # ZP card (mm)
        zp_card = QFrame()
        zp_card.setObjectName("cardFrame")
        zp_layout = QVBoxLayout(zp_card)
        zp_layout.setSpacing(4)

        zp_title = QLabel("ZP Stage Position (mm)")
        zp_title.setObjectName("sectionLabel")
        zp_layout.addWidget(zp_title)

        zp_grid = QGridLayout()
        zp_grid.setSpacing(4)
        for col, (name, attr) in enumerate([
            ("Z", "lbl_z"), ("P1", "lbl_p1"), ("P2", "lbl_p2"), ("P3", "lbl_p3")
        ]):
            zp_grid.addWidget(QLabel(f"{name}:"), 0, col * 2)
            lbl = QLabel("—")
            lbl.setFont(small_mono)
            lbl.setObjectName("valueLabel")
            zp_grid.addWidget(lbl, 0, col * 2 + 1)
            setattr(self, attr, lbl)

        self.lbl_zp_status = QLabel("Disconnected")
        self.lbl_zp_status.setStyleSheet(f"color: {COLORS['red']};")
        zp_grid.addWidget(self.lbl_zp_status, 1, 0, 1, 8)

        zp_layout.addLayout(zp_grid)
        pos_row.addWidget(zp_card)

        layout.addLayout(pos_row)

        # ── Speed Card ───────────────────────────────────────────
        speed_card = QFrame()
        speed_card.setObjectName("cardFrame")
        speed_layout = QVBoxLayout(speed_card)
        speed_layout.setSpacing(4)

        speed_title = QLabel("Speeds")
        speed_title.setObjectName("sectionLabel")
        speed_layout.addWidget(speed_title)

        speed_grid = QGridLayout()
        speed_grid.setSpacing(4)
        for col, (name, attr, unit) in enumerate([
            ("XY", "lbl_speed_xy", ""), ("Z", "lbl_speed_z", "mm/s"),
            ("Pump", "lbl_speed_p", "mm/s"),
        ]):
            speed_grid.addWidget(QLabel(f"{name}:"), 0, col * 2)
            lbl = QLabel("—")
            lbl.setFont(small_mono)
            lbl.setObjectName("valueLabel")
            speed_grid.addWidget(lbl, 0, col * 2 + 1)
            setattr(self, attr, lbl)

        speed_layout.addLayout(speed_grid)
        layout.addWidget(speed_card)

        # ── Zero Reference Card ──────────────────────────────────
        zero_card = QFrame()
        zero_card.setObjectName("cardFrame")
        zero_layout = QVBoxLayout(zero_card)
        zero_layout.setSpacing(4)

        zero_title = QLabel("Zero Reference")
        zero_title.setObjectName("sectionLabel")
        zero_layout.addWidget(zero_title)

        zero_grid = QGridLayout()
        zero_grid.setSpacing(4)
        self.zero_labels = {}
        for col, key in enumerate(["x", "y", "Z", "P1", "P2", "P3"]):
            display_name = key.upper() if key in ("x", "y") else key
            zero_grid.addWidget(QLabel(f"{display_name}:"), 0, col * 2)
            lbl = QLabel("0.0")
            lbl.setFont(small_mono)
            lbl.setObjectName("valueLabel")
            zero_grid.addWidget(lbl, 0, col * 2 + 1)
            self.zero_labels[key] = lbl

        zero_layout.addLayout(zero_grid)
        layout.addWidget(zero_card)

        # ── Safety Card ──────────────────────────────────────────
        safety_card = QFrame()
        safety_card.setObjectName("cardFrame")
        safety_layout = QVBoxLayout(safety_card)
        safety_layout.setSpacing(4)

        safety_title = QLabel("Safety Limits")
        safety_title.setObjectName("sectionLabel")
        safety_layout.addWidget(safety_title)

        self.lbl_safety_status = QLabel("Loading...")
        self.lbl_safety_status.setFont(QFont("Segoe UI", 10, QFont.Bold))
        safety_layout.addWidget(self.lbl_safety_status)

        self.lbl_safety_info = QLabel("")
        self.lbl_safety_info.setObjectName("dimLabel")
        self.lbl_safety_info.setWordWrap(True)
        safety_layout.addWidget(self.lbl_safety_info)

        layout.addWidget(safety_card)

        # ── Print History Card ───────────────────────────────────
        history_card = QFrame()
        history_card.setObjectName("cardFrame")
        history_layout = QVBoxLayout(history_card)
        history_layout.setSpacing(4)

        history_title = QLabel("Print History")
        history_title.setObjectName("sectionLabel")
        history_layout.addWidget(history_title)

        self.lbl_history_stats = QLabel("No prints recorded")
        self.lbl_history_stats.setObjectName("dimLabel")
        self.lbl_history_stats.setWordWrap(True)
        history_layout.addWidget(self.lbl_history_stats)

        layout.addWidget(history_card)

        layout.addStretch()

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATES
    # ════════════════════════════════════════════════════════════════


    def _update_xbox_btn_state(self) -> None:
        """v7.2.6: xbox button awareness — grey out Xbox connect when no stages connected."""
        if not hasattr(self, "_btn_connect_xbox"):
            return
        xy_ok = getattr(self.controller, "xy_stage", None) is not None
        zp_ok = getattr(self.controller, "zp_stage", None) is not None
        enabled = xy_ok or zp_ok
        self._btn_connect_xbox.setEnabled(enabled)
        tip = "" if enabled else "Connect XY or ZP stage first"
        self._btn_connect_xbox.setToolTip(tip)

    def on_status_update(self):
        """Called by MainWindow timer — refresh all readouts."""
        self.update_data()

        # Update context panel connection statuses
        self._update_conn_status("xy", "on" if self.controller.is_xy_connected else "off")
        self._update_conn_status("zp", "on" if self.controller.is_zp_connected else "off")

        # Xbox: tri-state — green/yellow/red + tooltip
        _xbox_st = self.controller.xbox_status if hasattr(self.controller, "xbox_status") else "disconnected"
        if callable(_xbox_st):
            _xbox_st = _xbox_st()
        if _xbox_st in ("connected", "alive"):
            self._update_conn_status("xbox", "on")
        elif _xbox_st == "reconnecting":
            self._update_conn_status("xbox", "warn")
        else:
            self._update_conn_status("xbox", "off")

        # Xbox tooltip detail
        _xbox_dot = getattr(self, "ctx_dot_xbox", None)
        if _xbox_dot:
            _tooltips = {
                "waiting": "Searching for Xbox controller...",
                "connected": "Xbox controller active",
                "alive": "Xbox controller active",
                "reconnecting": "Attempting to reconnect...",
                "unknown": "Xbox worker running, status unknown",
            }
            _xbox_dot.setToolTip(_tooltips.get(_xbox_st, "Xbox controller disconnected"))

        # Update log count in context panel
        if hasattr(self, 'ctx_lbl_log_count'):
            self.ctx_lbl_log_count.setText(
                f"Entries: {self.controller.position_logger.count}")
        self._update_xbox_btn_state()


    def update_data(self):
        """Update all data readouts."""
        ctrl = self.controller

        # XY position (converted to µm)
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            # v7.2.7: controller reports µm directly — no conversion needed

            ux = xy[0] - ctrl.zero_position["x"]

            uy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{ux:,.1f}")
            self.lbl_y.setText(f"{uy:,.1f}")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")

        # XY status label — only re-style on connection state change
        _xy_conn = ctrl.is_xy_connected
        if getattr(self, '_last_xy_conn', None) != _xy_conn:
            self._last_xy_conn = _xy_conn
            conn = "Connected" if _xy_conn else "Disconnected"
            color = COLORS['green'] if _xy_conn else COLORS['red']
            self.lbl_xy_status.setText(conn)
            self.lbl_xy_status.setStyleSheet(f"color: {color};")

        # ZP position — Z in mm, pumps in µL (v7.2) or mm (fallback)
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            for pid, lbl in [("P1", self.lbl_p1), ("P2", self.lbl_p2), ("P3", self.lbl_p3)]:
                idx = {"P1": 1, "P2": 2, "P3": 3}[pid]
                pos_mm = zp[idx] if idx < len(zp) else None
                zero_ref = ctrl.zero_position.get(pid, 0)
                if pos_mm is not None:
                    rel_mm = pos_mm - zero_ref
                    if self._hardware_config:
                        pump_cfg = self._hardware_config.pumps.get(pid)
                        if pump_cfg and pump_cfg.is_configured:
                            try:
                                pos_uL = pump_cfg.mm_to_uL(rel_mm)
                                lbl.setText(f"{pos_uL:.2f} µL")
                                continue
                            except (ValueError, AttributeError):
                                pass
                    lbl.setText(f"{rel_mm:.2f}")
                else:
                    lbl.setText("—")
        else:
            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:
                lbl.setText("—")

        # ZP status label — only re-style on connection state change
        _zp_conn = ctrl.is_zp_connected
        if getattr(self, '_last_zp_conn', None) != _zp_conn:
            self._last_zp_conn = _zp_conn
            conn = "Connected" if _zp_conn else "Disconnected"
            color = COLORS['green'] if _zp_conn else COLORS['red']
            self.lbl_zp_status.setText(conn)
            self.lbl_zp_status.setStyleSheet(f"color: {color};")

        # Speeds
        try:
            speeds = ctrl.get_speed_info()
            self.lbl_speed_xy.setText(f"{float(speeds.get('xy', 0)):.0f}")
            self.lbl_speed_z.setText(f"{float(speeds.get('z', 0)):.1f}")
            self.lbl_speed_p.setText(f"{float(speeds.get('p', 0)):.1f}")
        except (TypeError, ValueError, AttributeError):
            pass  # Speed info temporarily unavailable
        # Zero reference
        for key, lbl in self.zero_labels.items():
            lbl.setText(f"{ctrl.zero_position.get(key, 0):.1f}")

        # Safety limits — only re-style and rebuild text on state change
        sl = ctrl.safety_limits
        _safety_on = sl.enabled
        if getattr(self, '_last_safety_on', None) != _safety_on:
            self._last_safety_on = _safety_on
            if _safety_on:
                self.lbl_safety_status.setText("🛡️ Enabled")
                self.lbl_safety_status.setStyleSheet(
                    f"color: {COLORS['green']}; font-weight: bold;")
                # v7.2.6: Show per-pump limits
                pump_parts = []
                for pid, pmin, pmax in [
                    ('P1', sl.p1_min, sl.p1_max),
                    ('P2', sl.p2_min, sl.p2_max),
                    ('P3', sl.p3_min, sl.p3_max),
                ]:
                    pump_parts.append(f'{pid}:[{pmin:.1f}..{pmax:.1f}]')
                all_same = (sl.p1_min == sl.p2_min == sl.p3_min
                            and sl.p1_max == sl.p2_max == sl.p3_max)
                pump_str = (f'P:[{sl.p1_min:.1f}..{sl.p1_max:.1f}]'
                            if all_same else '  '.join(pump_parts))
                self.lbl_safety_info.setText(
                    f"XY: [{sl.xy_min_x:.0f}..{sl.xy_max_x:.0f}] × "
                    f"[{sl.xy_min_y:.0f}..{sl.xy_max_y:.0f}]  "
                    f"Z: [{sl.z_min:.1f}..{sl.z_max:.1f}]  "
                    f"{pump_str}"
                )
            else:
                self.lbl_safety_status.setText("⚠ Disabled")
                self.lbl_safety_status.setStyleSheet(
                    f"color: {COLORS['red']}; font-weight: bold;")
                self.lbl_safety_info.setText("Software endstops are OFF — be careful!")

        # Print history
        if self.print_history:
            stats = self.print_history.get_stats()
            self.lbl_history_stats.setText(
                f"Prints: {stats.get('count', 0)}  |  "
                f"Success: {stats.get('success_rate', 0):.0%}  |  "
                f"Total time: {stats.get('total_time_str', '—')}"
            )

    def _update_conn_status(self, name: str, state: str):
        """Update context panel connection dot — only re-styles on state change."""
        if getattr(self, f'_ctx_conn_state_{name}', None) == state:
            return
        setattr(self, f'_ctx_conn_state_{name}', state)
        dot = getattr(self, f'ctx_dot_{name}', None)
        if dot:
            _names = {"on": "connDotOn", "warn": "connDotWarn", "off": "connDotOff"}
            dot.setObjectName(_names.get(state, "connDotOff"))
            dot.style().unpolish(dot)
            dot.style().polish(dot)
            dot.update()


    def _connect_xy(self):
        btn = getattr(self, '_btn_connect_xy', None)
        self._run_connect(self.controller.connect_xy, btn, "XY")

    def _disconnect_xy(self):
        try:
            self.controller.disconnect_xy()
        except Exception as e:
            logger.error(f"XY disconnect failed: {e}")
        self.on_status_update()

    def _connect_zp(self):
        btn = getattr(self, '_btn_connect_zp', None)
        self._run_connect(self.controller.connect_zp, btn, "ZP")

    def _disconnect_zp(self):
        try:
            self.controller.disconnect_zp()
        except Exception as e:
            logger.error(f"ZP disconnect failed: {e}")
        self.on_status_update()

    def _run_connect(self, fn, btn, label):
        """Spawn a worker thread for a blocking connect call."""
        if btn is not None:
            btn.setEnabled(False)
            btn.setText("Connecting…")
        worker = _ConnectWorker(fn)
        worker.finished.connect(
            lambda ok, err, b=btn, lbl=label: self._on_connect_done(b, lbl, ok, err))
        worker.finished.connect(worker.deleteLater)
        # Keep a reference so the thread isn't garbage-collected
        setattr(self, f'_worker_{label.lower()}', worker)
        worker.start()

    def _on_connect_done(self, btn, label, ok, err):
        if btn is not None:
            btn.setText("Connect")
            btn.setEnabled(True)
        if not ok:
            logger.error(f"{label} connect failed: {err}")
        self.on_status_update()

    def _connect_xbox(self):
        """Connect Xbox controller. v7.2.7: thread mode option"""
        try:
            import platform
            use_thread = platform.system() == "Darwin"
            if use_thread:
                logger.info("macOS detected — using thread mode for Bluetooth compatibility")
            mapping = getattr(self.controller, "_mapping_file",
                              "current_button_mapping.json")
            timeout = self.settings.get("xbox", {}).get("reconnect_timeout_s", 30)
            # v7.3.2: Load stick calibration offsets
            stick_offsets = self.settings.get_section("xbox_stick_offsets")
            if stick_offsets:
                stick_offsets = {int(k): v for k, v in stick_offsets.items()}
            # v7.3.4: Build per-axis deadzone dict from saved stick/trigger values
            stick_dz = self.settings.get("xbox.deadzones.sticks", 0.20)
            trigger_dz = self.settings.get("xbox.deadzones.triggers", 0.05)
            axis_deadzones = {
                0: stick_dz, 1: stick_dz, 2: stick_dz, 3: stick_dz,
                4: trigger_dz, 5: trigger_dz,
            }
            debug_mode = bool(self.settings.get("xbox.debug_mode", False))
            self.controller.connect_xbox(
                mapping_file=mapping, use_thread=use_thread,
                reconnect_timeout=timeout,
                stick_offsets=stick_offsets or None,
                axis_deadzones=axis_deadzones,
                debug_mode=debug_mode,
            )
        except Exception as e:
            logger.error(f"Xbox connect failed: {e}")


    def _disconnect_xbox(self):
        try:
            self.controller.disconnect_xbox()
        except Exception as e:
            logger.error(f"Xbox disconnect failed: {e}")
        self.on_status_update()  # v7.2.7: disconnect refresh

    def _open_xbox_editor(self):
        # v7.2.6: S5-B mapping path — use the resolved path from StageController
        from gui.widgets.xbox_mapping_editor import XboxMappingEditor
        mapping_path = getattr(
            self.controller, "_mapping_file", "current_button_mapping.json"
        )
        editor = XboxMappingEditor(mapping_file=mapping_path, parent=self)
        editor.exec()


    def _export_log_csv(self):
        if hasattr(self.controller, 'position_logger'):
            path = self.controller.position_logger.export_csv()
            if path:
                logger.info(f"Position log exported to {path}")

    def _export_log_json(self):
        if hasattr(self.controller, 'position_logger'):
            path = self.controller.position_logger.export_json()
            if path:
                logger.info(f"Position log exported to {path}")

    def _export_history(self):
        if self.print_history:
            path = self.print_history.export()
            if path:
                logger.info(f"Print history exported to {path}")

    def _clear_history(self):
        if self.print_history:
            self.print_history.clear()
            logger.info("Print history cleared")
