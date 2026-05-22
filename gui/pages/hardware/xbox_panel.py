"""XboxHardwarePanel — Hardware Setup → Xbox sub-page (v7.4.2).

A dedicated controller setup workspace. Three cards stacked top-to-
bottom:

  Live Input Monitor
    Visual feedback of the controller state — left + right stick
    positions drawn as crosshairs in two pucks, LT/RT triggers as
    vertical bars, plus a "Last button" stamp. Updates at 20 Hz
    from the live ``StageController.xbox_poller`` state cached by
    the worker. Helps the user confirm the controller is alive +
    pick the right axes to calibrate.

  Calibration
    Stick deadzone (% of stick range), trigger deadzone (%),
    Calibrate Sticks (zeroes the resting-position offset), and a
    read-only display of the current stick offsets.

  Behavior
    Auto-reconnect timeout, Debug mode toggle. Both persist to
    settings.json.

  Button Mapping
    Opens the existing XboxMappingEditor dialog.

Connect / Disconnect buttons live in the persistent left context
panel so the user can hit Connect from any Hardware Setup sub-page.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QSlider,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, sp, scaled_font_size as sf
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.widgets.components import StatusBadge
from gui.widgets.icons import icon, icon_button

logger = logging.getLogger(__name__)


# ── Stick + trigger visualisers ────────────────────────────────

class StickPuck(QWidget):
    """Round dial showing a single stick's X/Y position.

    A crosshair sits at the current (x, y) where both are in [-1, 1].
    Includes a faint deadzone ring so users can see at a glance
    whether their current input is within the deadzone.
    """

    def __init__(self, label: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._label = label
        self._x = 0.0
        self._y = 0.0
        self._deadzone = 0.0
        self.setMinimumSize(s(110), s(110))
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

    def set_value(self, x: float, y: float) -> None:
        self._x = max(-1.0, min(1.0, float(x)))
        self._y = max(-1.0, min(1.0, float(y)))
        self.update()

    def set_deadzone(self, dz: float) -> None:
        self._deadzone = max(0.0, min(1.0, float(dz)))
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        side = min(w, h) - s(8)
        ox, oy = (w - side) / 2.0, (h - side) / 2.0
        bg = QColor(COLORS.get("surface0", "#313244"))
        border = QColor(COLORS.get("surface1", "#45475a"))
        p.setBrush(bg)
        p.setPen(QPen(border, 1.5))
        p.drawEllipse(QRectF(ox, oy, side, side))
        # Centerlines
        center = QColor(COLORS.get("subtext0", "#a6adc8"))
        center.setAlpha(70)
        p.setPen(QPen(center, 1))
        p.drawLine(int(ox + side / 2), int(oy + side * 0.18),
                   int(ox + side / 2), int(oy + side * 0.82))
        p.drawLine(int(ox + side * 0.18), int(oy + side / 2),
                   int(ox + side * 0.82), int(oy + side / 2))
        # Deadzone ring
        if self._deadzone > 0.0:
            dz_color = QColor(COLORS.get("red", "#f38ba8"))
            dz_color.setAlpha(60)
            p.setPen(QPen(dz_color, 1))
            p.setBrush(Qt.NoBrush)
            dz_r = side * 0.5 * self._deadzone
            cx, cy = ox + side / 2, oy + side / 2
            p.drawEllipse(QRectF(cx - dz_r, cy - dz_r, dz_r * 2, dz_r * 2))
        # Crosshair at current position
        cx = ox + side / 2 + self._x * (side * 0.45)
        cy = oy + side / 2 + self._y * (side * 0.45)
        dot_r = s(6)
        mauve = QColor(COLORS.get("mauve", "#cba6f7"))
        p.setBrush(mauve)
        p.setPen(Qt.NoPen)
        p.drawEllipse(QRectF(cx - dot_r, cy - dot_r, dot_r * 2, dot_r * 2))
        # Label
        p.setPen(QPen(QColor(COLORS.get("subtext0", "#a6adc8")), 1))
        p.setFont(QFont(self.font().family(), int(sf(8.5))))
        p.drawText(int(ox), int(oy + side + s(14)),
                   int(side), s(18),
                   Qt.AlignHCenter, self._label)


class TriggerBar(QWidget):
    """Vertical bar showing a trigger's pull amount in [0, 1]."""

    def __init__(self, label: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._label = label
        self._value = 0.0
        self._deadzone = 0.0
        self.setMinimumSize(s(36), s(110))
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

    def set_value(self, v: float) -> None:
        # The Xbox worker reports axis 4 / 5 in [-1, 1]; the resting
        # state is -1 and a fully-pulled trigger is +1. Map back to
        # [0, 1] for display.
        try:
            normalized = (float(v) + 1.0) / 2.0
        except Exception:
            normalized = 0.0
        self._value = max(0.0, min(1.0, normalized))
        self.update()

    def set_deadzone(self, dz: float) -> None:
        self._deadzone = max(0.0, min(1.0, float(dz)))
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        bar_w = s(16)
        bar_h = h - s(26)
        ox = (w - bar_w) / 2.0
        oy = s(4)
        bg = QColor(COLORS.get("surface0", "#313244"))
        border = QColor(COLORS.get("surface1", "#45475a"))
        p.setBrush(bg)
        p.setPen(QPen(border, 1.0))
        p.drawRoundedRect(QRectF(ox, oy, bar_w, bar_h), bar_w / 2, bar_w / 2)
        # Filled portion
        fill_h = bar_h * self._value
        if fill_h > 0:
            fill = QColor(COLORS.get("mauve", "#cba6f7"))
            p.setBrush(fill)
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(
                QRectF(ox, oy + (bar_h - fill_h), bar_w, fill_h),
                bar_w / 2, bar_w / 2)
        # Deadzone tick
        if self._deadzone > 0:
            dz_y = oy + bar_h - bar_h * self._deadzone
            p.setPen(QPen(QColor(COLORS.get("red", "#f38ba8")), 1))
            p.drawLine(int(ox - 2), int(dz_y), int(ox + bar_w + 2), int(dz_y))
        # Label
        p.setPen(QPen(QColor(COLORS.get("subtext0", "#a6adc8")), 1))
        p.setFont(QFont(self.font().family(), int(sf(8.5))))
        p.drawText(0, h - s(18), w, s(18), Qt.AlignHCenter, self._label)


# ── Panel ───────────────────────────────────────────────────────

class XboxHardwarePanel(QWidget):
    """Hardware Setup → Xbox sub-page."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = None
        self._settings = None
        # v7.4.2: panel paints the page background colour explicitly so
        # it doesn't inherit Qt's native white from the parent scroll
        # area.
        self.setAutoFillBackground(True)
        self.setObjectName("xboxHardwarePanel")
        self.setStyleSheet(
            f"#xboxHardwarePanel {{ background-color: {COLORS['base']}; }}"
        )
        self._mapping_widget = None
        self._build_ui()

        # 20 Hz live-monitor refresh
        self._monitor_timer = QTimer(self)
        self._monitor_timer.setInterval(50)
        self._monitor_timer.timeout.connect(self._tick_monitor)
        self._monitor_timer.start()

    def get_page_title(self) -> str:
        return "Xbox Controller"

    # ── External wiring ─────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._controller = controller
        self._sync_status()

    def set_settings(self, settings) -> None:
        self._settings = settings
        self._load_from_settings()

    def on_status_update(self) -> None:
        self._sync_status()

    # ── UI build ────────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(s(18))
        outer.setContentsMargins(s(20), s(20), s(20), s(20))

        # Banner — quick description of what the user can do here.
        banner = QLabel(
            "<b>Xbox controller setup.</b> Connect from the left "
            "panel, then use this page to calibrate sticks, adjust "
            "deadzones, and edit the button-action mapping. The "
            "monitor below mirrors the live controller state so you "
            "can verify each input as you tune it."
        )
        banner.setWordWrap(True)
        banner.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"padding: {sp(12)} {sp(16)};"
            f"background-color: qlineargradient("
            f"  x1: 0, y1: 0, x2: 0, y2: 1,"
            f"  stop: 0 rgba(205, 214, 244, 14),"
            f"  stop: 1 rgba(166, 227, 161, 14)"
            f");"
            f"border: 1px solid rgba(166, 173, 200, 32);"
            f"border-left: 3px solid {COLORS['green']};"
            f"border-radius: {sp(10)};"
        )
        outer.addWidget(banner)

        outer.addWidget(self._build_status_group())
        outer.addWidget(self._build_monitor_group())
        outer.addWidget(self._build_calibration_group())
        outer.addWidget(self._build_behavior_group())
        outer.addWidget(self._build_mapping_group())
        outer.addStretch(1)

    # ── Status group ────────────────────────────────────────────

    def _build_status_group(self) -> QGroupBox:
        grp = QGroupBox("Status")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        row = QHBoxLayout(grp)
        row.setSpacing(s(12))

        icon_lbl = QLabel()
        icon_lbl.setPixmap(
            icon("gamepad", color=COLORS["mauve"], px=s(40)).pixmap(s(40), s(40)))
        icon_lbl.setFixedSize(s(40), s(40))
        row.addWidget(icon_lbl)

        self.lbl_state = QLabel("Disconnected")
        self.lbl_state.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 600; "
            f"font-size: {sf(13)}pt;")
        row.addWidget(self.lbl_state)
        row.addStretch(1)

        self.badge_state = StatusBadge("Not connected", "pending")
        row.addWidget(self.badge_state)

        return grp

    # ── Monitor group ───────────────────────────────────────────

    def _build_monitor_group(self) -> QGroupBox:
        grp = QGroupBox("Live Input Monitor")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(10))

        sticks = QHBoxLayout()
        sticks.setSpacing(s(16))
        self.stick_left = StickPuck("Left stick (LX, LY)")
        self.stick_right = StickPuck("Right stick (RX, RY)")
        self.trig_left = TriggerBar("LT")
        self.trig_right = TriggerBar("RT")
        sticks.addStretch(1)
        sticks.addWidget(self.stick_left)
        sticks.addSpacing(s(8))
        sticks.addWidget(self.trig_left)
        sticks.addWidget(self.trig_right)
        sticks.addSpacing(s(8))
        sticks.addWidget(self.stick_right)
        sticks.addStretch(1)
        lay.addLayout(sticks)

        # Footer — last button + axis values
        footer = QHBoxLayout()
        footer.setSpacing(s(10))
        self.lbl_last_button = QLabel("Last button: —")
        self.lbl_last_button.setStyleSheet(
            f"color: {COLORS['subtext0']};")
        footer.addWidget(self.lbl_last_button)
        footer.addStretch(1)
        self.lbl_axis_dump = QLabel("L: (0.0, 0.0)   R: (0.0, 0.0)")
        self.lbl_axis_dump.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-family: monospace;")
        footer.addWidget(self.lbl_axis_dump)
        lay.addLayout(footer)

        return grp

    # ── Calibration group ──────────────────────────────────────

    def _build_calibration_group(self) -> QGroupBox:
        grp = QGroupBox("Calibration")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(10))

        info = QLabel(
            "Set the deadzone so light stick drift doesn't trigger "
            "jogs. Click <i>Calibrate Sticks</i> while resting your "
            "hands off the controller to capture the resting offsets."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']};")
        lay.addWidget(info)

        form = QFormLayout()
        form.setHorizontalSpacing(s(12))
        form.setVerticalSpacing(s(10))

        # Stick deadzone — slider + spinbox
        self.spin_stick_dz = QDoubleSpinBox()
        self.spin_stick_dz.setRange(0.0, 0.9)
        self.spin_stick_dz.setSingleStep(0.01)
        self.spin_stick_dz.setDecimals(2)
        self.spin_stick_dz.setValue(0.20)
        self.spin_stick_dz.valueChanged.connect(self._on_stick_dz_changed)
        form.addRow("Stick deadzone:", self._slider_row(
            self.spin_stick_dz, 0.0, 0.9))

        # Trigger deadzone
        self.spin_trigger_dz = QDoubleSpinBox()
        self.spin_trigger_dz.setRange(0.0, 0.9)
        self.spin_trigger_dz.setSingleStep(0.01)
        self.spin_trigger_dz.setDecimals(2)
        self.spin_trigger_dz.setValue(0.05)
        self.spin_trigger_dz.valueChanged.connect(self._on_trigger_dz_changed)
        form.addRow("Trigger deadzone:", self._slider_row(
            self.spin_trigger_dz, 0.0, 0.9))

        lay.addLayout(form)

        # Calibrate + offsets row
        cal_row = QHBoxLayout()
        cal_row.setSpacing(s(10))
        self.btn_calibrate = icon_button(
            "Calibrate Sticks", "ruler", object_name="accentBtn",
            tooltip=("Snapshot the current resting stick values as "
                     "the new center offset. Take your hands off the "
                     "controller before clicking."))
        self.btn_calibrate.clicked.connect(self._on_calibrate_clicked)
        cal_row.addWidget(self.btn_calibrate)

        self.btn_clear_offsets = icon_button(
            "Clear Offsets", "x", object_name="dangerBtn",
            tooltip="Reset stick offsets to zero.")
        self.btn_clear_offsets.clicked.connect(self._on_clear_offsets_clicked)
        cal_row.addWidget(self.btn_clear_offsets)
        cal_row.addStretch(1)
        lay.addLayout(cal_row)

        self.lbl_offsets = QLabel("Offsets: LX=0.00  LY=0.00  RX=0.00  RY=0.00")
        self.lbl_offsets.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-family: monospace;")
        lay.addWidget(self.lbl_offsets)

        return grp

    # ── Behavior group ─────────────────────────────────────────

    def _build_behavior_group(self) -> QGroupBox:
        grp = QGroupBox("Behavior")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        form = QFormLayout(grp)
        form.setHorizontalSpacing(s(12))
        form.setVerticalSpacing(s(10))

        self.spin_reconnect_to = QDoubleSpinBox()
        self.spin_reconnect_to.setRange(0, 600)
        self.spin_reconnect_to.setDecimals(0)
        self.spin_reconnect_to.setSuffix(" s")
        self.spin_reconnect_to.setValue(30)
        self.spin_reconnect_to.setToolTip(
            "How long to wait for the controller to reconnect after a "
            "disconnect before the worker gives up.")
        self.spin_reconnect_to.valueChanged.connect(
            self._on_reconnect_timeout_changed)
        form.addRow("Reconnect timeout:", self.spin_reconnect_to)

        self.chk_debug = QCheckBox("Log every event from the Xbox worker")
        self.chk_debug.setToolTip(
            "Useful when troubleshooting button mappings or strange "
            "axis behavior. Spammy — turn off when you're done.")
        self.chk_debug.toggled.connect(self._on_debug_toggled)
        form.addRow("Debug mode:", self.chk_debug)

        return grp

    # ── Mapping group (inline editor, no popup) ────────────────

    def _build_mapping_group(self) -> QGroupBox:
        from gui.widgets.xbox_mapping_editor import XboxMappingEditorWidget
        grp = QGroupBox("Button Mapping")
        grp.setStyleSheet(SECTION_TITLE_STYLE)
        lay = QVBoxLayout(grp)
        lay.setSpacing(s(10))

        info = QLabel(
            "Map physical Xbox inputs to stage commands. Changes save "
            "to ``current_button_mapping.json`` and hot-reload — no "
            "need to reconnect the controller."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS['subtext0']};")
        lay.addWidget(info)

        mapping_path = (
            getattr(self._controller, "_mapping_file",
                    "current_button_mapping.json")
            if self._controller is not None
            else "current_button_mapping.json")
        self._mapping_widget = XboxMappingEditorWidget(
            mapping_file=mapping_path, parent=self)
        lay.addWidget(self._mapping_widget)
        return grp

    # ── Helpers ────────────────────────────────────────────────

    def _slider_row(self, spin: QDoubleSpinBox,
                    lo: float, hi: float) -> QHBoxLayout:
        """Pair a slider (0..100) with the spinbox so users can drag
        OR type. Slider expresses the value as a percentage of [lo, hi].
        """
        sld = QSlider(Qt.Horizontal)
        sld.setRange(0, 100)
        sld.setValue(int((spin.value() - lo) / (hi - lo) * 100))
        sld.setSingleStep(1)
        sld.setPageStep(5)

        def _slider_to_spin(v: int):
            spin.blockSignals(True)
            spin.setValue(lo + (v / 100.0) * (hi - lo))
            spin.blockSignals(False)
            spin.valueChanged.emit(spin.value())

        def _spin_to_slider(v: float):
            sld.blockSignals(True)
            sld.setValue(int((v - lo) / (hi - lo) * 100))
            sld.blockSignals(False)

        sld.valueChanged.connect(_slider_to_spin)
        spin.valueChanged.connect(_spin_to_slider)

        row = QHBoxLayout()
        row.setSpacing(s(8))
        row.addWidget(sld, 1)
        spin.setMaximumWidth(s(90))
        row.addWidget(spin)
        return row

    # ── State sync ─────────────────────────────────────────────

    def _sync_status(self) -> None:
        if self._controller is None:
            return
        st = getattr(self._controller, "xbox_status", "disconnected")
        if callable(st):
            try:
                st = st()
            except Exception:
                st = "disconnected"
        if st in ("connected", "alive"):
            self.badge_state.set_status("ok", "Connected")
            self.lbl_state.setText("Controller connected")
        elif st == "reconnecting":
            self.badge_state.set_status("warn", "Reconnecting…")
            self.lbl_state.setText("Reconnecting…")
        elif st == "waiting":
            self.badge_state.set_status("info", "Searching…")
            self.lbl_state.setText("Searching for controller…")
        else:
            self.badge_state.set_status("pending", "Not connected")
            self.lbl_state.setText("Not connected")

    def _load_from_settings(self) -> None:
        if self._settings is None:
            return
        s_obj = self._settings
        stick_dz = float(s_obj.get("xbox.deadzones.sticks", 0.20))
        trig_dz = float(s_obj.get("xbox.deadzones.triggers", 0.05))
        timeout = float(s_obj.get("xbox.reconnect_timeout_s", 30))
        debug = bool(s_obj.get("xbox.debug_mode", False))
        for w in (self.spin_stick_dz, self.spin_trigger_dz,
                  self.spin_reconnect_to, self.chk_debug):
            w.blockSignals(True)
        try:
            self.spin_stick_dz.setValue(stick_dz)
            self.spin_trigger_dz.setValue(trig_dz)
            self.spin_reconnect_to.setValue(timeout)
            self.chk_debug.setChecked(debug)
        finally:
            for w in (self.spin_stick_dz, self.spin_trigger_dz,
                      self.spin_reconnect_to, self.chk_debug):
                w.blockSignals(False)
        # Feed deadzone into visualisers
        self.stick_left.set_deadzone(stick_dz)
        self.stick_right.set_deadzone(stick_dz)
        self.trig_left.set_deadzone(trig_dz)
        self.trig_right.set_deadzone(trig_dz)
        self._refresh_offsets_label()

    def _refresh_offsets_label(self) -> None:
        if self._settings is None:
            return
        offsets = self._settings.get_section("xbox_stick_offsets") or {}
        # Normalize possibly-string keys to int
        clean = {}
        for k, v in offsets.items():
            try:
                clean[int(k)] = float(v)
            except (TypeError, ValueError):
                pass
        lx = clean.get(0, 0.0)
        ly = clean.get(1, 0.0)
        rx = clean.get(2, 0.0)
        ry = clean.get(3, 0.0)
        self.lbl_offsets.setText(
            f"Offsets: LX={lx:+.2f}  LY={ly:+.2f}  "
            f"RX={rx:+.2f}  RY={ry:+.2f}")

    # ── Live-monitor tick ──────────────────────────────────────

    _BUTTON_NAMES = {
        0: "A", 1: "B", 2: "X", 3: "Y",
        4: "LB", 5: "RB",
        6: "Back", 7: "Start", 8: "Guide",
        9: "Left stick click", 10: "Right stick click",
    }

    def _tick_monitor(self) -> None:
        poller = getattr(self._controller, "xbox_poller", None) \
            if self._controller is not None else None
        if poller is None:
            return
        last_axis = getattr(poller, "last_axis", {}) or {}
        lx = last_axis.get(0, 0.0)
        ly = last_axis.get(1, 0.0)
        rx = last_axis.get(2, 0.0)
        ry = last_axis.get(3, 0.0)
        lt = last_axis.get(4, -1.0)
        rt = last_axis.get(5, -1.0)
        self.stick_left.set_value(lx, ly)
        self.stick_right.set_value(rx, ry)
        self.trig_left.set_value(lt)
        self.trig_right.set_value(rt)
        self.lbl_axis_dump.setText(
            f"L: ({lx:+.2f}, {ly:+.2f})   R: ({rx:+.2f}, {ry:+.2f})")
        # Last button
        last_button = getattr(poller, "last_button", None)
        if last_button is not None:
            btn_id, ts = last_button
            name = self._BUTTON_NAMES.get(btn_id, f"#{btn_id}")
            import time as _t
            age = _t.time() - ts
            if age < 1.5:
                self.lbl_last_button.setText(f"Last button: <b>{name}</b>")
                self.lbl_last_button.setStyleSheet(
                    f"color: {COLORS['mauve']};")
            else:
                self.lbl_last_button.setText(f"Last button: {name}")
                self.lbl_last_button.setStyleSheet(
                    f"color: {COLORS['subtext0']};")

    # ── Handlers ───────────────────────────────────────────────

    def _persist_xbox_setting(self, key: str, value) -> None:
        if self._settings is None:
            return
        try:
            self._settings.set(key, value)
            self._settings.save()
        except Exception as e:
            logger.debug(f"Xbox setting save failed for {key}: {e}")

    def _on_stick_dz_changed(self, v: float) -> None:
        self.stick_left.set_deadzone(v)
        self.stick_right.set_deadzone(v)
        self._persist_xbox_setting("xbox.deadzones.sticks", float(v))

    def _on_trigger_dz_changed(self, v: float) -> None:
        self.trig_left.set_deadzone(v)
        self.trig_right.set_deadzone(v)
        self._persist_xbox_setting("xbox.deadzones.triggers", float(v))

    def _on_reconnect_timeout_changed(self, v: float) -> None:
        self._persist_xbox_setting("xbox.reconnect_timeout_s", float(v))

    def _on_debug_toggled(self, checked: bool) -> None:
        self._persist_xbox_setting("xbox.debug_mode", bool(checked))

    def _on_calibrate_clicked(self) -> None:
        poller = getattr(self._controller, "xbox_poller", None) \
            if self._controller is not None else None
        if poller is None:
            return
        last_axis = getattr(poller, "last_axis", {}) or {}
        # Snapshot the four stick axes as the new center offsets.
        offsets = {
            0: float(last_axis.get(0, 0.0)),
            1: float(last_axis.get(1, 0.0)),
            2: float(last_axis.get(2, 0.0)),
            3: float(last_axis.get(3, 0.0)),
        }
        if self._settings is not None:
            try:
                # set_section is required because individual keys
                # under xbox_stick_offsets are integers.
                self._settings.set_section(
                    "xbox_stick_offsets",
                    {str(k): v for k, v in offsets.items()})
                self._settings.save()
            except Exception as e:
                logger.warning(f"Failed to save stick offsets: {e}")
        self._refresh_offsets_label()
        logger.info(f"Stick offsets captured: {offsets}")

    def _on_clear_offsets_clicked(self) -> None:
        if self._settings is None:
            return
        try:
            self._settings.set_section(
                "xbox_stick_offsets",
                {str(k): 0.0 for k in (0, 1, 2, 3)})
            self._settings.save()
        except Exception as e:
            logger.warning(f"Failed to clear stick offsets: {e}")
        self._refresh_offsets_label()

    # v7.4.2: _open_mapping_editor removed — editor lives inline now.
