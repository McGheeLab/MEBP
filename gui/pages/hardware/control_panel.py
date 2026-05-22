"""HardwareControlPanel — persistent left panel for the Hardware Setup page
(v7.4.2).

Sits flush against the left edge of the Hardware Setup page and stays
visible regardless of which sub-page is active. Contains the controls
the user reaches for most often during initial setup:

  * Connect Hardware: XY / ZP / Xbox connect+disconnect buttons + live
    status badges.
  * Full Jog Pad: XY direction pad, Z buttons, pump buttons, step-size
    selectors. Setup-mode (safety bypassed) — the user expects to drive
    the stages to limits.
  * Live Position: X / Y / Z / P1 / P2 / P3 readouts, refreshed on every
    ``on_status_update`` tick from MainWindow.

The panel doesn't own the controller — it talks to it through
``self._controller`` after ``set_controller()`` is called by the parent
page. Connect/disconnect logic mirrors what was on the Device sub-page
before this move (v7.4.2: a952e33 et al.).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QVBoxLayout, QWidget,
)

from gui.scaling import s, sp, scaled_font_size as sf
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.widgets.components import StatusBadge
from gui.widgets.icons import icon, icon_button
from gui.widgets.jog_button_array import JogButtonArray
from SupportClasses.ZPStage import AXIS_MAP as _DEFAULT_AXIS_MAP

logger = logging.getLogger(__name__)


class HardwareControlPanel(QWidget):
    """Always-on Connect + Jog + Live Position panel for Hardware Setup."""

    _PHYSICAL_TO_INDEX = {"X": 0, "Y": 1, "Z": 2, "E": 3}

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("hardwareControlPanel")
        self.setStyleSheet(
            f"#hardwareControlPanel {{"
            f"  background-color: {COLORS['mantle']};"
            f"  border-right: 1px solid {COLORS['surface1']};"
            f"}}"
        )
        self._controller = None
        self._settings = None
        self._build_ui()

    # ── Public API ──────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._controller = controller
        self._sync_badges()
        self.on_status_update()

    def set_settings(self, settings) -> None:
        self._settings = settings

    def on_status_update(self) -> None:
        """Refresh position labels + status badges. Called by MainWindow tick."""
        if self._controller is None:
            return
        try:
            xy = self._controller.get_xy_position(cached=True)
            zp = self._controller.get_zp_position(cached=True)
            self._update_position_displays(xy, zp)
        except Exception:
            pass
        self._sync_badges()

    # ── UI ──────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(
            f"QScrollArea {{ background-color: transparent; border: none; }}")

        wrap = QVBoxLayout(self)
        wrap.setContentsMargins(0, 0, 0, 0)
        wrap.addWidget(scroll)

        content = QWidget()
        content.setStyleSheet("background-color: transparent;")
        layout = QVBoxLayout(content)
        layout.setSpacing(s(14))
        layout.setContentsMargins(s(14), s(14), s(14), s(14))
        scroll.setWidget(content)

        layout.addWidget(self._build_connect_group())
        layout.addWidget(self._build_jog_group())
        layout.addWidget(self._build_position_group())
        layout.addStretch(1)

        self.lbl_status = QLabel("")
        self.lbl_status.setStyleSheet(f"color: {COLORS['subtext0']};")
        self.lbl_status.setWordWrap(True)
        layout.addWidget(self.lbl_status)

    # ── Connect group ───────────────────────────────────────────

    def _build_connect_group(self) -> QGroupBox:
        grp = QGroupBox("Connect Hardware")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        grid = QGridLayout(grp)
        grid.setHorizontalSpacing(s(8))
        grid.setVerticalSpacing(s(10))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)

        def _name(text: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(f"font-weight: 600; color: {COLORS['text']};")
            return lbl

        def _connect_btn(text: str, icon_name: str) -> QPushButton:
            return icon_button(text, icon_name, object_name="successBtn")

        def _disconnect_btn(tooltip: str) -> QPushButton:
            b = icon_button("", "x", object_name="dangerBtn", tooltip=tooltip)
            b.setFixedWidth(s(36))
            return b

        # XY row
        row = 0
        grid.addWidget(_name("XY stage"), row, 0)
        self.badge_xy = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_xy, row, 1)
        btns = QHBoxLayout()
        btns.setSpacing(s(6))
        btns.setContentsMargins(0, 0, 0, 0)
        self.btn_connect_xy = _connect_btn("Connect", "plug")
        self.btn_connect_xy.clicked.connect(self._connect_xy)
        btns.addWidget(self.btn_connect_xy)
        self.btn_disconnect_xy = _disconnect_btn("Disconnect XY")
        self.btn_disconnect_xy.clicked.connect(self._disconnect_xy)
        btns.addWidget(self.btn_disconnect_xy)
        grid.addLayout(btns, row, 2)

        # ZP row
        row += 1
        grid.addWidget(_name("Z + Pumps"), row, 0)
        self.badge_zp = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_zp, row, 1)
        btns = QHBoxLayout()
        btns.setSpacing(s(6))
        btns.setContentsMargins(0, 0, 0, 0)
        self.btn_connect_zp = _connect_btn("Connect", "plug")
        self.btn_connect_zp.clicked.connect(self._connect_zp)
        btns.addWidget(self.btn_connect_zp)
        self.btn_disconnect_zp = _disconnect_btn("Disconnect ZP")
        self.btn_disconnect_zp.clicked.connect(self._disconnect_zp)
        btns.addWidget(self.btn_disconnect_zp)
        grid.addLayout(btns, row, 2)

        # Xbox row
        row += 1
        grid.addWidget(_name("Xbox"), row, 0)
        self.badge_xbox = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_xbox, row, 1)
        btns = QHBoxLayout()
        btns.setSpacing(s(6))
        btns.setContentsMargins(0, 0, 0, 0)
        self.btn_connect_xbox = _connect_btn("Connect", "gamepad")
        self.btn_connect_xbox.clicked.connect(self._connect_xbox)
        btns.addWidget(self.btn_connect_xbox)
        self.btn_disconnect_xbox = _disconnect_btn("Disconnect Xbox")
        self.btn_disconnect_xbox.clicked.connect(self._disconnect_xbox)
        btns.addWidget(self.btn_disconnect_xbox)
        grid.addLayout(btns, row, 2)

        return grp

    # ── Jog group ───────────────────────────────────────────────

    def _build_jog_group(self) -> QGroupBox:
        grp = QGroupBox("Jog Stages (setup-mode, safety bypassed)")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(8))
        lay.setContentsMargins(s(8), s(8), s(8), s(8))

        self._jog_array = JogButtonArray(compact=True, show_pumps=True)
        self._jog_array.jog_xy_requested.connect(self._on_jog_xy)
        self._jog_array.jog_z_requested.connect(self._on_jog_z)
        self._jog_array.jog_pump_requested.connect(self._on_jog_pump)
        self._jog_array.home_requested.connect(self._force_refresh_positions)
        lay.addWidget(self._jog_array)

        self.btn_refresh = icon_button(
            "Refresh Positions", "refresh",
            tooltip="Force a fresh position read from each connected stage.")
        self.btn_refresh.clicked.connect(self._force_refresh_positions)
        lay.addWidget(self.btn_refresh)

        return grp

    # ── Live position group ─────────────────────────────────────

    def _build_position_group(self) -> QGroupBox:
        grp = QGroupBox("Live Position")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(s(8), s(8), s(8), s(8))

        grid = QGridLayout()
        grid.setHorizontalSpacing(s(8))
        grid.setVerticalSpacing(s(4))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)

        self.lbl_pos: dict[str, QLabel] = {}
        for r, (axis, unit) in enumerate([
            ("X", "µm"), ("Y", "µm"), ("Z", "mm"),
            ("P1", "mm"), ("P2", "mm"), ("P3", "mm"),
        ]):
            ax_lbl = QLabel(f"<b>{axis}</b>")
            ax_lbl.setStyleSheet(f"color: {COLORS['text']};")
            grid.addWidget(ax_lbl, r, 0)
            val = QLabel("—")
            val.setStyleSheet(f"color: {COLORS['text']}; font-family: monospace;")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(val, r, 1)
            unit_lbl = QLabel(f"({unit})")
            unit_lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
            grid.addWidget(unit_lbl, r, 2)
            self.lbl_pos[axis] = val
        lay.addLayout(grid)

        return grp

    # ── Connect handlers ────────────────────────────────────────

    def _connect_xy(self) -> None:
        if self._controller is None:
            return
        self.badge_xy.set_status("info", "Connecting…")
        try:
            self._controller.connect_xy()
            ok = bool(self._controller.is_xy_connected)
            self.badge_xy.set_status("ok" if ok else "err",
                                     "Connected" if ok else "Failed")
        except Exception as e:
            self.badge_xy.set_status("err", f"Error: {e}")

    def _disconnect_xy(self) -> None:
        if self._controller is None:
            return
        try:
            self._controller.disconnect_xy()
        except Exception as e:
            logger.warning(f"disconnect_xy failed: {e}")
        self.badge_xy.set_status("pending", "Not connected")

    def _connect_zp(self) -> None:
        if self._controller is None:
            return
        self.badge_zp.set_status("info", "Connecting…")
        try:
            self._controller.connect_zp()
            ok = bool(self._controller.is_zp_connected)
            self.badge_zp.set_status("ok" if ok else "err",
                                     "Connected" if ok else "Failed")
            if ok and self._settings is not None:
                port = self._controller.zp_connected_port
                if port:
                    self._settings.set("zp_stage.last_port", port)
                    self._settings.save()
        except Exception as e:
            self.badge_zp.set_status("err", f"Error: {e}")

    def _disconnect_zp(self) -> None:
        if self._controller is None:
            return
        try:
            self._controller.disconnect_zp()
        except Exception as e:
            logger.warning(f"disconnect_zp failed: {e}")
        self.badge_zp.set_status("pending", "Not connected")

    def _connect_xbox(self) -> None:
        ctrl = self._controller
        if ctrl is None:
            return
        self.badge_xbox.set_status("info", "Connecting…")
        try:
            import platform
            use_thread = platform.system() == "Darwin"
            mapping = getattr(
                ctrl, "_mapping_file", "current_button_mapping.json")
            s_obj = self._settings
            if s_obj is not None:
                timeout = s_obj.get("xbox.reconnect_timeout_s", 30)
                stick_offsets = s_obj.get_section("xbox_stick_offsets") or {}
                if stick_offsets:
                    stick_offsets = {int(k): v for k, v in stick_offsets.items()}
                stick_dz = s_obj.get("xbox.deadzones.sticks", 0.20)
                trigger_dz = s_obj.get("xbox.deadzones.triggers", 0.05)
                debug_mode = bool(s_obj.get("xbox.debug_mode", False))
            else:
                timeout = 30
                stick_offsets = {}
                stick_dz = 0.20
                trigger_dz = 0.05
                debug_mode = False
            axis_deadzones = {
                0: stick_dz, 1: stick_dz, 2: stick_dz, 3: stick_dz,
                4: trigger_dz, 5: trigger_dz,
            }
            ctrl.connect_xbox(
                mapping_file=mapping,
                use_thread=use_thread,
                reconnect_timeout=timeout,
                stick_offsets=stick_offsets or None,
                axis_deadzones=axis_deadzones,
                debug_mode=debug_mode,
            )
            self.badge_xbox.set_status("ok", "Connected")
        except Exception as e:
            logger.warning(f"Xbox connect failed: {e}")
            self.badge_xbox.set_status("err", f"Error: {e}")

    def _disconnect_xbox(self) -> None:
        if self._controller is None:
            return
        try:
            self._controller.disconnect_xbox()
        except Exception as e:
            logger.warning(f"disconnect_xbox failed: {e}")
        self.badge_xbox.set_status("pending", "Not connected")

    def _sync_badges(self) -> None:
        if self._controller is None:
            return
        xy_ok = bool(getattr(self._controller, 'is_xy_connected', False))
        zp_ok = bool(getattr(self._controller, 'is_zp_connected', False))
        self.badge_xy.set_status("ok" if xy_ok else "pending",
                                 "Connected" if xy_ok else "Not connected")
        self.badge_zp.set_status("ok" if zp_ok else "pending",
                                 "Connected" if zp_ok else "Not connected")
        xbox_st = getattr(self._controller, "xbox_status", None)
        if callable(xbox_st):
            try:
                xbox_st = xbox_st()
            except Exception:
                xbox_st = "disconnected"
        if xbox_st in ("connected", "alive"):
            self.badge_xbox.set_status("ok", "Connected")
        elif xbox_st == "reconnecting":
            self.badge_xbox.set_status("warn", "Reconnecting…")
        elif xbox_st == "waiting":
            self.badge_xbox.set_status("info", "Searching…")
        else:
            self.badge_xbox.set_status("pending", "Not connected")

    # ── Jog handlers ────────────────────────────────────────────

    def _on_jog_xy(self, dx_um: float, dy_um: float) -> None:
        if self._controller is None:
            return
        try:
            self._controller.move_xy_relative_um(
                dx_um, dy_um, bypass_safety=True)
        except Exception as e:
            logger.warning(f"jog XY ({dx_um}, {dy_um}) failed: {e}")
        self._force_refresh_positions()

    def _on_jog_z(self, dz_mm: float) -> None:
        if self._controller is None:
            return
        try:
            self._controller.move_z_relative(dz_mm, bypass_safety=True)
        except Exception as e:
            logger.warning(f"jog Z {dz_mm} failed: {e}")
        self._force_refresh_positions()

    def _on_jog_pump(self, pump: str, distance: float) -> None:
        if self._controller is None:
            return
        try:
            self._controller.move_pump_relative(
                pump, distance, bypass_safety=True)
        except Exception as e:
            logger.warning(f"jog {pump} {distance} failed: {e}")
        self._force_refresh_positions()

    # ── Position refresh ────────────────────────────────────────

    def _force_refresh_positions(self) -> None:
        ctrl = self._controller
        if ctrl is None:
            self.lbl_status.setText("No controller available.")
            return
        try:
            xy = ctrl.get_xy_position(cached=False)
            zp = ctrl.get_zp_position(cached=False)
        except Exception as e:
            self.lbl_status.setText(f"Read failed: {e}")
            return
        self._update_position_displays(xy, zp)

    def _logical_zp_value(self, zp, logical: str) -> float | None:
        """v7.4.2: read ZP tuple at index for ``logical`` axis via live axis_map."""
        if self._controller and self._controller.zp_stage:
            axis_map = self._controller.zp_stage.axis_map
        else:
            axis_map = _DEFAULT_AXIS_MAP
        physical = axis_map.get(logical)
        idx = self._PHYSICAL_TO_INDEX.get(physical)
        if idx is None or zp is None or idx >= len(zp):
            return None
        v = zp[idx]
        return float(v) if v is not None else None

    def _update_position_displays(self, xy, zp) -> None:
        if xy and len(xy) >= 2:
            if xy[0] is not None:
                self.lbl_pos["X"].setText(f"{xy[0]:,.1f}")
            else:
                self.lbl_pos["X"].setText("—")
            if xy[1] is not None:
                self.lbl_pos["Y"].setText(f"{xy[1]:,.1f}")
            else:
                self.lbl_pos["Y"].setText("—")
        if zp:
            for logical in ("Z", "P1", "P2", "P3"):
                v = self._logical_zp_value(zp, logical)
                self.lbl_pos[logical].setText(
                    f"{v:.3f}" if v is not None else "—")
