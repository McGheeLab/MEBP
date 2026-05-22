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

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

from gui.scaling import s, sp, scaled_font_size as sf
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.widgets.components import StatusBadge
from gui.widgets.icons import icon, icon_button
from gui.widgets.jog_button_array import JogButtonArray
from SupportClasses.ZPStage import AXIS_MAP as _DEFAULT_AXIS_MAP

logger = logging.getLogger(__name__)


class PositionBar(QWidget):
    """v7.4.2: Slim horizontal slider showing where an axis sits between
    its safety-limit min and max.

    Renders a track (rounded rect) with a marker line at the current
    value. ``set_range(min, max)`` defines the extents; ``set_value(v)``
    moves the marker. If no value is set yet, only the track is drawn.

    Range comes from ``safety_limits.*`` on the active Settings; the
    parent ``HardwareControlPanel`` calls ``set_range`` after settings
    arrive and re-calls it when the user edits limits and saves.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._min = 0.0
        self._max = 100.0
        self._value: float | None = None
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(s(10))
        self.setMinimumWidth(s(60))

    def set_range(self, lo: float, hi: float) -> None:
        if hi <= lo:
            hi = lo + 1.0
        self._min, self._max = float(lo), float(hi)
        self.update()

    def set_value(self, value: float | None) -> None:
        self._value = float(value) if value is not None else None
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        radius = h / 2.0
        # Track
        track_color = QColor(COLORS.get("surface1", "#45475a"))
        p.setPen(Qt.NoPen)
        p.setBrush(track_color)
        p.drawRoundedRect(QRectF(0, 0, w, h), radius, radius)
        # Marker (current position)
        if self._value is None:
            return
        span = self._max - self._min
        if span <= 0:
            return
        clamped = max(self._min, min(self._max, self._value))
        frac = (clamped - self._min) / span
        x = frac * (w - h) + radius  # leave room for the dot at edges
        # Fill from left edge to marker — visual cue for "how far in".
        fill_color = QColor(COLORS.get("mauve", "#cba6f7"))
        fill_color.setAlpha(80)
        p.setBrush(fill_color)
        p.drawRoundedRect(QRectF(0, 0, x + radius, h), radius, radius)
        # Marker dot
        dot_color = QColor(COLORS.get("mauve", "#cba6f7"))
        p.setBrush(dot_color)
        dot_r = h * 0.7
        p.drawEllipse(QRectF(x - dot_r / 2, (h - dot_r) / 2, dot_r, dot_r))
        # If the value is outside the recorded envelope, paint a red
        # ring around the dot — alerts the user to a clamp / overrun.
        if self._value < self._min or self._value > self._max:
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(COLORS.get("red", "#f38ba8")), 1.5))
            p.drawEllipse(QRectF(x - dot_r / 2 - 1, (h - dot_r) / 2 - 1,
                                  dot_r + 2, dot_r + 2))


class HardwareControlPanel(QWidget):
    """Always-on Connect + Jog + Live Position panel.

    v7.4.2: configurable via kwargs so the same widget can serve both
    Hardware Setup (default: connect visible, soft limits bypassed for
    setup-mode jogs) and Calibration (connect hidden, soft limits
    enforced so jogs respect the recorded envelope).
    """

    _PHYSICAL_TO_INDEX = {"X": 0, "Y": 1, "Z": 2, "E": 3}

    def __init__(self, parent: QWidget | None = None, *,
                 show_connect: bool = True,
                 bypass_safety: bool = True):
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
        self._show_connect = show_connect
        self._bypass_safety = bypass_safety
        self._build_ui()

    # ── Public API ──────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._controller = controller
        self._sync_badges()
        self.on_status_update()

    def set_settings(self, settings) -> None:
        self._settings = settings
        # v7.4.2: pull safety-limit ranges into the position bars so the
        # slider extents reflect the user's recorded envelope.
        self._refresh_bar_ranges()

    def refresh_safety_limits(self) -> None:
        """v7.4.2: External hook — call after the user saves new safety
        limits in the Device sub-page so the bar extents update live.
        """
        self._refresh_bar_ranges()

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

        if self._show_connect:
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
        grp = QGroupBox("Jog Stages")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(8))
        lay.setContentsMargins(s(8), s(8), s(8), s(8))

        # v7.4.2: variant-aware banner. Setup-mode (bypass_safety=True)
        # gets the yellow "limits OFF" warning. Safety-on mode gets a
        # quieter green banner confirming jogs are clamped.
        warn = QFrame()
        warn.setObjectName("jogSafetyWarn")
        accent = (COLORS.get("yellow", "#f9e2af") if self._bypass_safety
                  else COLORS.get("green", "#a6e3a1"))
        bg_rgba = ("rgba(249, 226, 175, 28)" if self._bypass_safety
                   else "rgba(166, 227, 161, 24)")
        warn.setStyleSheet(
            f"#jogSafetyWarn {{"
            f"  background-color: {bg_rgba};"
            f"  border: 1px solid {accent};"
            f"  border-left: 3px solid {accent};"
            f"  border-radius: {sp(6)};"
            f"  padding: {sp(8)} {sp(10)};"
            f"}}"
        )
        warn_lay = QHBoxLayout(warn)
        warn_lay.setSpacing(s(8))
        warn_lay.setContentsMargins(0, 0, 0, 0)
        warn_icon = QLabel()
        icon_name = "alert" if self._bypass_safety else "check-circle"
        warn_icon.setPixmap(
            icon(icon_name, color=accent, px=s(20)).pixmap(s(20), s(20)))
        warn_icon.setFixedSize(s(20), s(20))
        warn_lay.addWidget(warn_icon, 0, Qt.AlignTop)
        if self._bypass_safety:
            text = (
                "<b>Safety limits are OFF in this section.</b><br>"
                "Soft-limit clamping and the pump-enabled check are "
                "bypassed so you can drive each stage to its mechanical "
                "extremes. Watch the Live Position bars below — markers "
                "turn red when they cross the recorded envelope.")
        else:
            text = (
                "<b>Safety limits are enforced.</b> Jogs respect the "
                "recorded soft-limit envelope and the pump-enabled "
                "check. Position bars highlight in red if the stage "
                "approaches a limit.")
        warn_text = QLabel(text)
        warn_text.setWordWrap(True)
        warn_text.setStyleSheet(f"color: {accent};")
        warn_lay.addWidget(warn_text, 1)
        lay.addWidget(warn)

        self._jog_array = JogButtonArray(compact=True, show_pumps=True)
        self._jog_array.jog_xy_requested.connect(self._on_jog_xy)
        self._jog_array.jog_z_requested.connect(self._on_jog_z)
        self._jog_array.jog_pump_requested.connect(self._on_jog_pump)
        self._jog_array.home_requested.connect(self._force_refresh_positions)
        lay.addWidget(self._jog_array)

        # v7.4.2: per-axis jog speeds. Values flow through the
        # ``_on_jog_*`` handlers — XY uses ProScan SMS via
        # xy_stage.set_velocity (µm/s); Z and pumps pass feedrate
        # (mm/min) to move_z_relative / move_pump_relative.
        lay.addWidget(self._build_speed_controls())

        self.btn_refresh = icon_button(
            "Refresh Positions", "refresh",
            tooltip="Force a fresh position read from each connected stage.")
        self.btn_refresh.clicked.connect(self._force_refresh_positions)
        lay.addWidget(self.btn_refresh)

        return grp

    # ── Speed controls ─────────────────────────────────────────

    def _build_speed_controls(self) -> QWidget:
        wrap = QFrame()
        wrap.setStyleSheet(
            f"background-color: rgba(255,255,255,8);"
            f"border-radius: {sp(6)}; padding: {sp(2)};"
        )
        outer = QVBoxLayout(wrap)
        outer.setContentsMargins(s(8), s(6), s(8), s(6))
        outer.setSpacing(s(4))

        heading = QLabel("Speeds")
        heading.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-weight: 600; "
            f"letter-spacing: 0.4px;")
        outer.addWidget(heading)

        # XY speed — µm/s (ProScan native)
        self.spin_xy_speed = self._speed_spin(10000, suffix=" µm/s",
                                              default=2000, step=100)
        outer.addLayout(self._labeled(
            "XY", self.spin_xy_speed,
            tooltip="ProScan max velocity in µm/s. Applied via SMS on each jog."))
        # Z speed — mm/min feedrate
        self.spin_z_speed = self._speed_spin(3000, suffix=" mm/min",
                                             default=600, step=50)
        outer.addLayout(self._labeled(
            "Z", self.spin_z_speed,
            tooltip="Z move feedrate in mm/min."))
        # Pump speed — mm/min feedrate
        self.spin_p_speed = self._speed_spin(3000, suffix=" mm/min",
                                             default=200, step=50)
        outer.addLayout(self._labeled(
            "P", self.spin_p_speed,
            tooltip="Pump move feedrate in mm/min."))

        # Push the XY speed to the controller whenever it changes so
        # the next ProScan move uses the new SMS value.
        self.spin_xy_speed.valueChanged.connect(self._apply_xy_speed)
        return wrap

    def _labeled(self, name: str, widget, tooltip: str = "") -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(6))
        row.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(name)
        lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 500;")
        lbl.setMinimumWidth(s(22))
        row.addWidget(lbl)
        row.addWidget(widget, 1)
        if tooltip:
            widget.setToolTip(tooltip)
            lbl.setToolTip(tooltip)
        return row

    def _speed_spin(self, maximum: float, suffix: str, default: float,
                    step: float) -> "QDoubleSpinBox":
        from PySide6.QtWidgets import QDoubleSpinBox
        sp_w = QDoubleSpinBox()
        sp_w.setRange(1, maximum)
        sp_w.setDecimals(0)
        sp_w.setSingleStep(step)
        sp_w.setValue(default)
        sp_w.setSuffix(suffix)
        return sp_w

    def _apply_xy_speed(self, value: float) -> None:
        """Push XY speed to ProScan via set_velocity (µm/s)."""
        ctrl = self._controller
        if ctrl is None or ctrl.xy_stage is None:
            return
        try:
            ctrl.xy_stage.set_velocity(int(value))
        except Exception as e:
            logger.debug(f"set_velocity({value}) failed: {e}")

    # ── Live position group ─────────────────────────────────────

    def _build_position_group(self) -> QGroupBox:
        grp = QGroupBox("Live Position")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(s(8), s(8), s(8), s(8))

        # v7.4.2: per-axis row = label + slider bar (extents from safety
        # limits) + numeric value + unit. The bar visualises how close
        # the stage is to its recorded min/max envelope.
        grid = QGridLayout()
        grid.setHorizontalSpacing(s(8))
        grid.setVerticalSpacing(s(6))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)

        self.lbl_pos: dict[str, QLabel] = {}
        self.bar_pos: dict[str, PositionBar] = {}
        for r, (axis, unit) in enumerate([
            ("X", "µm"), ("Y", "µm"), ("Z", "mm"),
            ("P1", "mm"), ("P2", "mm"), ("P3", "mm"),
        ]):
            ax_lbl = QLabel(f"<b>{axis}</b>")
            ax_lbl.setStyleSheet(f"color: {COLORS['text']};")
            ax_lbl.setMinimumWidth(s(22))
            grid.addWidget(ax_lbl, r, 0)
            bar = PositionBar()
            grid.addWidget(bar, r, 1)
            self.bar_pos[axis] = bar
            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-family: monospace;")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setMinimumWidth(s(70))
            grid.addWidget(val, r, 2)
            unit_lbl = QLabel(unit)
            unit_lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
            grid.addWidget(unit_lbl, r, 3)
            self.lbl_pos[axis] = val
        lay.addLayout(grid)

        return grp

    # ── Safety-limit range loader ───────────────────────────────

    def _refresh_bar_ranges(self) -> None:
        """v7.4.2: pull min/max from settings.safety_limits and apply to
        each PositionBar so the slider extents reflect the user's
        recorded envelope. Called on set_settings + on the first
        on_status_update tick after a controller arrives.
        """
        if not hasattr(self, 'bar_pos'):
            return
        s_obj = self._settings
        if s_obj is None:
            return
        try:
            ranges = {
                "X":  (s_obj.get("safety_limits.xy_min_x", -130000.0),
                       s_obj.get("safety_limits.xy_max_x", 130000.0)),
                "Y":  (s_obj.get("safety_limits.xy_min_y",  -85000.0),
                       s_obj.get("safety_limits.xy_max_y",   85000.0)),
                "Z":  (s_obj.get("safety_limits.z_min",      -10.0),
                       s_obj.get("safety_limits.z_max",       50.0)),
                "P1": (s_obj.get("safety_limits.p1_min",     -50.0),
                       s_obj.get("safety_limits.p1_max",      50.0)),
                "P2": (s_obj.get("safety_limits.p2_min",     -50.0),
                       s_obj.get("safety_limits.p2_max",      50.0)),
                "P3": (s_obj.get("safety_limits.p3_min",     -50.0),
                       s_obj.get("safety_limits.p3_max",      50.0)),
            }
        except Exception as e:
            logger.debug(f"_refresh_bar_ranges failed: {e}")
            return
        for axis, (lo, hi) in ranges.items():
            bar = self.bar_pos.get(axis)
            if bar is not None:
                try:
                    bar.set_range(float(lo), float(hi))
                except Exception:
                    pass

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
        # v7.4.2: panel may be configured without the Connect group
        # (calibration variant) — bail if the badge widgets don't exist.
        if not hasattr(self, 'badge_xy'):
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
        # v7.4.2: ensure the latest speed is on ProScan before the move
        # (the user may have edited the spinbox without pressing Tab).
        self._apply_xy_speed(self.spin_xy_speed.value())
        bypass = self._bypass_safety
        try:
            self._controller.move_xy_relative_um(
                dx_um, dy_um, bypass_safety=bypass)
        except Exception as e:
            logger.warning(f"jog XY ({dx_um}, {dy_um}) failed: {e}")
        self._force_refresh_positions()

    def _on_jog_z(self, dz_mm: float) -> None:
        if self._controller is None:
            return
        feed = float(self.spin_z_speed.value())
        bypass = self._bypass_safety
        try:
            self._controller.move_z_relative(
                dz_mm, feedrate=feed, bypass_safety=bypass)
        except TypeError:
            self._controller.move_z_relative(dz_mm, bypass_safety=bypass)
        except Exception as e:
            logger.warning(f"jog Z {dz_mm} failed: {e}")
        self._force_refresh_positions()

    def _on_jog_pump(self, pump: str, distance: float) -> None:
        if self._controller is None:
            return
        feed = float(self.spin_p_speed.value())
        bypass = self._bypass_safety
        try:
            self._controller.move_pump_relative(
                pump, distance, feedrate=feed, bypass_safety=bypass)
        except TypeError:
            self._controller.move_pump_relative(
                pump, distance, bypass_safety=bypass)
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
        def _set(axis: str, value: float | None, fmt: str) -> None:
            self.lbl_pos[axis].setText(fmt.format(value) if value is not None else "—")
            bar = self.bar_pos.get(axis) if hasattr(self, 'bar_pos') else None
            if bar is not None:
                bar.set_value(value)
        if xy and len(xy) >= 2:
            _set("X", xy[0] if xy[0] is not None else None, "{:,.1f}")
            _set("Y", xy[1] if xy[1] is not None else None, "{:,.1f}")
        if zp:
            for logical in ("Z", "P1", "P2", "P3"):
                v = self._logical_zp_value(zp, logical)
                _set(logical, v, "{:.3f}")
