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
import threading

from PySide6.QtCore import QEvent, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

from gui.scaling import s, sp, scaled_font_size as sf
from gui.styles import COLORS
from gui.widgets.components import Card, StatusBadge
from gui.widgets.icons import icon, icon_button
from gui.widgets.jog_button_array import JogButtonArray
from SupportClasses.ZPStage import AXIS_MAP as _DEFAULT_AXIS_MAP
from SupportClasses.StageController import z_raw_to_display

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


class PositionValueLabel(QLabel):
    """v7.9.x: numeric position readout whose minimum width TRACKS its text.

    Each Live-Position row is ``axis | PositionBar (stretch) | value | unit``.
    The value label used to sit behind a tiny fixed floor (s(20)) with an
    Ignored size policy so the row could scrunch — but the bar owns the row's
    only stretch column, so at normal widths the bar absorbed everything and
    the number was clipped to its low-order digits, visually running into the
    unit label. Priority is now inverted: this label claims exactly the width
    its CURRENT text needs (recomputed on every setText and on font changes,
    so the responsive font scaler keeps it honest at narrow widths) and the
    BAR is the element that yields. The width is capped so a pathological
    string can't blow up the layout.

    ⚠ v7.9.1 — the horizontal policy must NOT be ``Ignored``. It was, and it
    fought the tracking minimum width: ``Ignored`` tells the layout to size the
    COLUMN without regard to this widget, while ``setMinimumWidth`` forces the
    WIDGET to stay that wide. The grid handed column 2 about 5 px and the label
    then drew itself ~100 px wide straight over the unit label — measured at
    every panel width, which is the operator's "the position is overtop of the
    units". ``Preferred`` makes the layout actually reserve the width, and the
    BAR (``Ignored``, 10 px minimum) stays the element that yields, so a narrow
    panel still scrunches without clipping the number."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__("—", parent)
        self.setStyleSheet(
            f"color: {COLORS['text']}; font-family: monospace;")
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self._sync_min_width()

    def setText(self, text: str) -> None:  # noqa: N802 (Qt override)
        super().setText(text)
        self._sync_min_width()

    def changeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        super().changeEvent(event)
        if event.type() == QEvent.FontChange:
            self._sync_min_width()

    def _sync_min_width(self) -> None:
        w = self.fontMetrics().horizontalAdvance(self.text()) + s(4)
        self.setMinimumWidth(max(s(20), min(w, s(160))))


class HardwareControlPanel(QWidget):
    """Always-on Connect + Jog + Live Position panel.

    v7.4.2: configurable via kwargs so the same widget can serve both
    Hardware Setup (default: connect visible, soft limits bypassed for
    setup-mode jogs) and Calibration (connect hidden, soft limits
    enforced so jogs respect the recorded envelope).
    """

    _PHYSICAL_TO_INDEX = {"X": 0, "Y": 1, "Z": 2, "E": 3}

    # v7.5.x: emitted (queued, from the pump-jog worker thread) when a pump
    # jog move finishes, so the GUI-thread slot can safely refresh widgets.
    _pump_jog_done = Signal()

    # v7.5.x: emitted (queued, from an XY/Z jog worker thread) with the freshly
    # read (xy, zp) positions so the GUI-thread slot updates the readout without
    # doing any blocking serial I/O on the GUI thread (which froze the camera
    # feed while jogging). Either value may be None if that read failed.
    _stage_jog_done = Signal(object, object)

    def __init__(self, parent: QWidget | None = None, *,
                 show_connect: bool = True,
                 bypass_safety: bool = True,
                 embedded: bool = False,
                 pump_action_labels: bool = False,
                 speed_as_max: bool = False):
        """Build the control panel.

        Args:
            show_connect: Show the Connect Hardware section (default True).
                          Hide on pages where connection happens elsewhere.
            bypass_safety: Set True for Hardware Setup's mechanical-envelope
                           workflow; False on Calibration / Jog where soft
                           limits should clamp every move.
            embedded: When False (default) the panel owns a QScrollArea
                      around its content. Set True when embedding inside
                      a parent that already scrolls — the parent then sees
                      the panel's natural size, and the inner sections
                      don't have to share that scroll with neighbours.
                      v7.4.3: required by StandardJogContextPanel so the
                      outer context-pane scroll handles everything.
        """
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
        self._embedded = embedded
        # v7.5.x: ASPIRATE/DISPENSE pump jog labels (jog side panels) vs the
        # raw ▲/▼ extend/retract arrows (Hardware Setup page, the default).
        self._pump_action_labels = pump_action_labels
        # v7.5.x: on Hardware Setup ("hardware calibration") the speed section
        # edits the ABSOLUTE per-axis max (the single common ceiling) instead of
        # a % of it. Every other page keeps "% of max". Passed True only from
        # hardware_setup.py so existing panels/tests default to percent mode.
        self._speed_as_max = speed_as_max
        # v7.5.x: jog speeds default to 1/2 of the calibrated max once
        # settings arrive. Seed once; preserve a manual edit thereafter.
        self._speeds_seeded = False
        self._speed_user_edited = {"xy": False, "z": False, "p": False}
        # v7.5.x: a pump jog move (esp. with backlash compensation, which
        # brackets take-up/main/unload with blocking M400 drains + settle
        # dwells) runs on a daemon thread — see ``_on_jog_pump`` — so it can't
        # freeze the Qt event loop (and with it, every QTimer-driven camera
        # feed) for the seconds it can take on real hardware.
        self._pump_jog_busy = False
        self._pump_jog_done.connect(self._on_pump_jog_done)
        # v7.5.x: XY / Z incremental jog also runs off the GUI thread (the move
        # + the fresh position read are blocking serial round-trips that froze
        # the camera feed). Per-axis busy guards drop overlapping clicks rather
        # than queuing blocking moves behind the shared serial channel.
        self._xy_jog_busy = False
        self._z_jog_busy = False
        self._stage_jog_done.connect(self._on_stage_jog_done)
        self._build_ui()

    # ── Public API ──────────────────────────────────────────────

    def set_controller(self, controller) -> None:
        self._controller = controller
        self._sync_badges()
        self.on_status_update()
        # v7.5.x: adopt the controller's single per-axis max + shared jog-%.
        self.refresh_speed_limits()

    def set_settings(self, settings) -> None:
        self._settings = settings
        # v7.4.2: pull safety-limit ranges into the position bars so the
        # slider extents reflect the user's recorded envelope.
        self._refresh_bar_ranges()
        # v7.5.x: jog speed is now a % of the single common per-axis max — sync
        # the percent spinboxes + resolved labels from the controller.
        self.refresh_speed_limits()

    def minimumSizeHint(self):  # noqa: N802 (Qt override)
        """v7.5.x: report a small minimum WIDTH so the context-panel scroll area
        can drive this panel down to a narrow width (the operator wants a 100 px
        minimum). Every row is proportional (stretch-driven with tiny minimums),
        so the content fits/scrunches at any width; this just removes the
        text-based floor that would otherwise keep the panel wide. Height is left
        to the real layout so nothing is vertically clipped."""
        from PySide6.QtCore import QSize
        h = super().minimumSizeHint().height()
        return QSize(s(96), h)

    def _apply_responsive_fonts(self, *, force: bool = False) -> None:
        """v7.5.x: scale every control's font by the width %% so the panel is
        genuinely *sized to fit* — text shrinks with the width instead of
        clipping. The nested JogButtonArray scales itself, so it's skipped."""
        w = self.width()
        if w <= 0:
            return
        from gui.widgets.responsive import (
            container_scale, quantize, scale_descendant_fonts)
        factor = quantize(container_scale(w, s(440), 0.45, 1.12))
        if not force and abs(factor - getattr(self, "_font_factor", -1.0)) < 1e-6:
            return
        self._font_factor = factor
        try:
            scale_descendant_fonts(
                self, factor, skip_subtrees=[getattr(self, "_jog_array", None)])
        except Exception as exc:  # never let a resize raise
            logger.debug(f"panel font scale failed: {exc}")

    def resizeEvent(self, event):  # noqa: N802 (Qt override)
        super().resizeEvent(event)
        self._apply_responsive_fonts()

    def showEvent(self, event):  # noqa: N802 (Qt override)
        """Re-read the common max + shared jog-% whenever the panel is shown,
        so a max changed while this page was hidden is reflected on entry."""
        try:
            self.refresh_speed_limits()
        except Exception:
            pass
        super().showEvent(event)
        # v7.5.x: also (re)apply the width-based font scale on show, so the
        # panel is sized-to-fit even if it's mounted at its final width without
        # a distinct resize event.
        self._apply_responsive_fonts(force=True)

    def refresh_safety_limits(self) -> None:
        """v7.4.2: External hook — call after the user saves new safety
        limits in the Device sub-page so the bar extents update live.
        v7.5.x: also re-read the per-axis speed maxes."""
        self._refresh_bar_ranges()
        self.refresh_speed_limits()

    def on_status_update(self) -> None:
        """Refresh position labels + status badges. Called by MainWindow tick."""
        # The microscope is a separate singleton, so its badge must refresh
        # even before a StageController is injected (and its connect is
        # asynchronous, so this tick is how the result becomes visible).
        self._sync_microscope_badge()
        if self._controller is None:
            return
        try:
            self._update_position_displays(
                self._display_xy(), self._display_zp())
        except Exception:
            pass
        self._sync_badges()

    def on_motion_tick(self) -> None:
        """v7.5.x: fast (~30 fps) display-only refresh of just the position
        readouts while a jog/travel motion estimate is live (no badge work)."""
        if self._controller is None:
            return
        try:
            self._update_position_displays(
                self._display_xy(), self._display_zp())
        except Exception:
            pass

    def _display_xy(self):
        """XY for display — the interpolated estimate while a move is in flight,
        else the raw poller cache (fallback for controllers without the getter)."""
        c = self._controller
        fn = getattr(c, "get_display_xy_position", None)
        return fn() if fn is not None else c.get_xy_position(cached=True)

    def _display_zp(self):
        c = self._controller
        fn = getattr(c, "get_display_zp_position", None)
        return fn() if fn is not None else c.get_zp_position(cached=True)

    # ── UI ──────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        # v7.4.3: when embedded, skip our own QScrollArea so the parent
        # context pane provides one outer scroll. Nested scrolls were
        # squeezing the jog buttons in the calibration / jog left panel.
        if self._embedded:
            layout = QVBoxLayout(self)
            layout.setSpacing(s(14))
            layout.setContentsMargins(s(14), s(14), s(14), s(14))
        else:
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
        if not self._embedded:
            layout.addStretch(1)

        self.lbl_status = QLabel("")
        self.lbl_status.setStyleSheet(f"color: {COLORS['subtext0']};")
        self.lbl_status.setWordWrap(True)
        layout.addWidget(self.lbl_status)

    # ── Connect group ───────────────────────────────────────────

    def _build_connect_group(self) -> QWidget:
        # v7.4.3: Card-with-collapsible replaces QGroupBox so the whole
        # context pane can be one scrolling list of foldable sections.
        card = Card("Connect Hardware", collapsible=True)
        grid = QGridLayout()
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

        def _simulate_btn(tooltip: str) -> QPushButton:
            # v7.4.2: per-stage Simulate button — opens the built-in
            # simulator instead of real hardware. Mirrors the Connect
            # button's visual weight but uses the accent color so the
            # two paths are easy to tell apart at a glance.
            b = icon_button("Simulate", "flask", object_name="accentBtn",
                            tooltip=tooltip)
            return b

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
        self.btn_connect_xy.setToolTip(
            "Open the real XY stage over serial.")
        self.btn_connect_xy.clicked.connect(self._connect_xy)
        btns.addWidget(self.btn_connect_xy)
        self.btn_simulate_xy = _simulate_btn(
            "Use the XY stage simulator instead of real hardware.")
        self.btn_simulate_xy.clicked.connect(self._simulate_xy)
        btns.addWidget(self.btn_simulate_xy)
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
        self.btn_connect_zp.setToolTip(
            "Open the real Marlin Z + pumps controller over serial.")
        self.btn_connect_zp.clicked.connect(self._connect_zp)
        btns.addWidget(self.btn_connect_zp)
        self.btn_simulate_zp = _simulate_btn(
            "Use the Z + pumps simulator instead of real hardware.")
        self.btn_simulate_zp.clicked.connect(self._simulate_zp)
        btns.addWidget(self.btn_simulate_zp)
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

        # Microscope row — v7.5.x. The motorised body (turrets + focus) is
        # hardware, so it is opened here with the stages rather than on the
        # page that only assigns cube/objective names. Which DRIVER is used
        # stays on Hardware Setup → Microscope; this row only opens the link.
        row += 1
        grid.addWidget(_name("Microscope"), row, 0)
        self.badge_scope = StatusBadge("Not connected", "pending")
        grid.addWidget(self.badge_scope, row, 1)
        btns = QHBoxLayout()
        btns.setSpacing(s(6))
        btns.setContentsMargins(0, 0, 0, 0)
        self.btn_connect_scope = _connect_btn("Connect", "plug")
        self.btn_connect_scope.setToolTip(
            "Open the microscope body using the driver saved on Hardware "
            "Setup → Microscope.")
        self.btn_connect_scope.clicked.connect(self._connect_microscope)
        btns.addWidget(self.btn_connect_scope)
        self.btn_simulate_scope = _simulate_btn(
            "Use the microscope simulator instead of the real body. Does not "
            "change the saved driver.")
        self.btn_simulate_scope.clicked.connect(self._simulate_microscope)
        btns.addWidget(self.btn_simulate_scope)
        self.btn_disconnect_scope = _disconnect_btn("Disconnect microscope")
        self.btn_disconnect_scope.clicked.connect(self._disconnect_microscope)
        btns.addWidget(self.btn_disconnect_scope)
        grid.addLayout(btns, row, 2)

        # v7.18 NOTE — deliberately NO Incubator row: the heaters are wired
        # to the ZP board itself, so the ZP Connect above IS the incubator
        # connect (operator: "it does not need a separate connect button").
        # The Incubator page opens its session over the live ZP link
        # automatically. When the planned ESP32 sensor board arrives it gets
        # its own dedicated-serial transport — revisit a row here then.

        card.add_layout(grid)
        return card

    # ── Jog group ───────────────────────────────────────────────

    def _build_jog_group(self) -> QWidget:
        card = Card("Jog Stages", collapsible=True)
        lay = card.body_layout()
        lay.setSpacing(s(8))

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
        # v7.5.x: tiny min so the (wrapping) banner can shrink with the panel
        # instead of flooring its width.
        warn_text.setMinimumWidth(s(1))
        warn_text.setStyleSheet(f"color: {accent};")
        warn_lay.addWidget(warn_text, 1)
        lay.addWidget(warn)

        self._jog_array = JogButtonArray(
            compact=True, show_pumps=True,
            pump_action_labels=self._pump_action_labels)
        self._jog_array.jog_xy_requested.connect(self._on_jog_xy)
        self._jog_array.jog_z_requested.connect(self._on_jog_z)
        self._jog_array.jog_pump_requested.connect(self._on_jog_pump)
        self._jog_array.home_requested.connect(self._force_refresh_positions)
        self._jog_array.step_settings_changed.connect(
            self._persist_step_settings)
        lay.addWidget(self._jog_array)

        # v7.4.2: per-axis jog speeds. Values flow through the
        # ``_on_jog_*`` handlers — XY uses ProScan SMS via
        # xy_stage.set_velocity (µm/s); Z and pumps pass feedrate
        # (mm/min) to move_z_relative / move_pump_relative.
        lay.addWidget(self._build_speed_controls())

        self.btn_refresh = icon_button(
            "Refresh Positions", "refresh",
            tooltip="Force a fresh position read from each connected stage.")
        # v7.5.x: let the button shrink (Ignored h-policy + small min) so it
        # doesn't force the panel wide; its label clips gracefully when narrow.
        self.btn_refresh.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.btn_refresh.setMinimumWidth(s(40))
        self.btn_refresh.clicked.connect(self._force_refresh_positions)
        lay.addWidget(self.btn_refresh)

        return card

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

        # v7.5.x: Hardware Setup edits the ABSOLUTE per-axis max (the single
        # common ceiling); every other page shows "% of max".
        if self._speed_as_max:
            return self._build_max_speed_controls(wrap, outer)

        heading = QLabel("Jog speed (% of max)")
        heading.setWordWrap(True)   # wrap so the section fits a narrow panel
        heading.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-weight: 600; "
            f"letter-spacing: 0.4px;")
        heading.setToolTip(
            "v7.5.x: jog speed is a percentage of the single calibrated "
            "per-axis max (set once on Hardware Setup → Stage / Timing "
            "calibration; inherited by every page). The resolved absolute "
            "speed is shown beside each axis.")
        outer.addWidget(heading)

        # v7.5.x: percent-of-max spinboxes + a read-only resolved-speed label.
        # The per-axis MAX comes from the controller's single resolvers
        # (get_max_xy_speed_um_s / get_max_z_feedrate_mm_min /
        # get_max_pump_feedrate), so every page inherits the same source.
        self.spin_xy_pct = self._pct_spin()
        self.lbl_xy_resolved = self._resolved_label()
        outer.addLayout(self._labeled_pct(
            "XY", self.spin_xy_pct, self.lbl_xy_resolved,
            tooltip="XY jog speed as a % of the calibrated XY top speed."))

        self.spin_z_pct = self._pct_spin()
        self.lbl_z_resolved = self._resolved_label()
        outer.addLayout(self._labeled_pct(
            "Z", self.spin_z_pct, self.lbl_z_resolved,
            tooltip="Z jog speed as a % of the max Z feedrate."))

        # v7.9.x: PER-PUMP jog flow rows (the operator asked for each pump to
        # get its own flow rate on every jog tile, not just Hardware Setup).
        # One row per pump: a % spin (can go below 1 % for fine plunger jogs)
        # anchored to THIS pump's own safe flow ceiling, + a resolved µL/s
        # readout. Unconfigured pumps are hidden in _resolve_speeds.
        from PySide6.QtWidgets import QWidget as _QWidget
        self.spin_p_pct_pumps: dict[str, "QDoubleSpinBox"] = {}
        self.lbl_p_resolved_pumps: dict[str, QLabel] = {}
        self.row_p_pct_pumps: dict[str, QWidget] = {}
        for pid in ("P1", "P2", "P3"):
            spin = self._pct_spin(min_pct=0.01, decimals=2, step=0.1)
            lbl = self._resolved_label()
            row_lay = self._labeled_pct(
                pid, spin, lbl,
                tooltip=(f"{pid} jog flow as a % of {pid}'s own safe flow "
                         f"ceiling (needle/syringe-derived; can go below "
                         f"1 %). The resolved rate is shown beside it."))
            row_w = _QWidget()
            row_w.setLayout(row_lay)
            outer.addWidget(row_w)
            self.spin_p_pct_pumps[pid] = spin
            self.lbl_p_resolved_pumps[pid] = lbl
            self.row_p_pct_pumps[pid] = row_w
            spin.valueChanged.connect(
                lambda _v=None, p=pid: self._on_pump_pct_changed(p))

        self.spin_xy_pct.valueChanged.connect(
            lambda: self._on_speed_pct_changed("xy"))
        self.spin_z_pct.valueChanged.connect(
            lambda: self._on_speed_pct_changed("z"))
        return wrap

    def _pct_spin(self, *, min_pct: float = 1.0, decimals: int = 0,
                  step: float = 5.0) -> "QDoubleSpinBox":
        from PySide6.QtWidgets import QDoubleSpinBox
        sp_w = QDoubleSpinBox()
        sp_w.setRange(min_pct, 100.0)
        sp_w.setDecimals(decimals)
        sp_w.setSingleStep(step)
        sp_w.setValue(50.0)
        sp_w.setSuffix(" %")
        return sp_w

    def _max_spin(self, *, maximum: float, decimals: int, step: float,
                  unit: str) -> "QDoubleSpinBox":
        """v7.5.x: absolute per-axis MAX-speed spinbox (Hardware Setup)."""
        from PySide6.QtWidgets import QDoubleSpinBox
        sp_w = QDoubleSpinBox()
        sp_w.setRange(1.0, maximum)
        sp_w.setDecimals(decimals)
        sp_w.setSingleStep(step)
        sp_w.setSuffix(f" {unit}")
        return sp_w

    def _build_max_speed_controls(self, wrap: QWidget,
                                  outer: "QVBoxLayout") -> QWidget:
        """v7.5.x: Hardware Setup speed section — edit the ABSOLUTE per-axis max
        speed (the single common ceiling every % page reads). Reuses the
        ``spin_*_pct`` attribute names (they hold absolute values here) so the
        shared handlers/refresh paths work with a ``_speed_as_max`` branch."""
        heading = QLabel("Max speed (per axis)")
        heading.setWordWrap(True)   # wrap so the section fits a narrow panel
        heading.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-weight: 600; "
            f"letter-spacing: 0.4px;")
        heading.setToolTip(
            "v7.5.x: the maximum speed for each axis. This is the single "
            "ceiling every other page reads — jog and print speeds elsewhere "
            "are a % of these. Jogging here runs at the value you set.")
        outer.addWidget(heading)

        self.spin_xy_pct = self._max_spin(
            maximum=1_000_000.0, decimals=0, step=100.0, unit="µm/s")
        self.lbl_xy_resolved = self._resolved_label()
        self.lbl_xy_resolved.setVisible(False)
        outer.addLayout(self._labeled_pct(
            "XY", self.spin_xy_pct, self.lbl_xy_resolved,
            tooltip="Maximum XY speed (µm/s) — the 100% anchor everywhere."))

        self.spin_z_pct = self._max_spin(
            maximum=100_000.0, decimals=0, step=50.0, unit="mm/min")
        self.lbl_z_resolved = self._resolved_label()
        self.lbl_z_resolved.setVisible(False)
        outer.addLayout(self._labeled_pct(
            "Z", self.spin_z_pct, self.lbl_z_resolved,
            tooltip="Maximum Z feedrate (mm/min) — the 100% anchor everywhere."))

        # v7.5.x: pump max rate is PER-PUMP (each pump may hold a different
        # syringe, so the same mm/min plunger feedrate maps to a different
        # µL/s). One row per pump: a mm/min spin (the firmware plunger unit,
        # edited primary) + a live µL/s secondary readout derived from that
        # pump's syringe. Unconfigured pumps are hidden in
        # _seed_pump_max_from_controller.
        from PySide6.QtWidgets import QWidget as _QWidget
        self.spin_p_max_pumps: dict[str, "QDoubleSpinBox"] = {}
        self.lbl_p_max_uL: dict[str, QLabel] = {}
        self.row_p_max_pumps: dict[str, QWidget] = {}
        for pid in ("P1", "P2", "P3"):
            spin = self._max_spin(
                maximum=100_000.0, decimals=0, step=10.0, unit="mm/min")
            uL_lbl = self._resolved_label()
            row_lay = self._labeled_pct(
                pid, spin, uL_lbl,
                tooltip=(f"Maximum {pid} plunger feedrate (mm/min). The µL/s "
                         f"equivalent — via this pump's syringe — is shown "
                         f"beside it."))
            row_w = _QWidget()
            row_w.setLayout(row_lay)
            outer.addWidget(row_w)
            self.spin_p_max_pumps[pid] = spin
            self.lbl_p_max_uL[pid] = uL_lbl
            self.row_p_max_pumps[pid] = row_w
            spin.valueChanged.connect(
                lambda _v=None, p=pid: self._on_pump_max_changed(p))

        self.spin_xy_pct.valueChanged.connect(
            lambda: self._on_speed_pct_changed("xy"))
        self.spin_z_pct.valueChanged.connect(
            lambda: self._on_speed_pct_changed("z"))
        return wrap

    def _resolved_label(self) -> QLabel:
        lbl = QLabel("—")
        # v7.5.x: no font-size in the QSS so the width-based font scaler
        # (scale_descendant_fonts) can rescale it; family/colour stay in QSS.
        lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-family: monospace;")
        # v7.5.x: tiny minimum + Ignored h-policy so the row shrinks to fit a
        # narrow panel; it takes a %% of the width via its stretch factor.
        lbl.setMinimumWidth(s(20))
        lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        return lbl

    def _labeled_pct(self, name: str, spin, resolved: QLabel,
                     tooltip: str = "") -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(4))
        row.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(name)
        lbl.setStyleSheet(f"color: {COLORS['text']}; font-weight: 500;")
        lbl.setMinimumWidth(s(16))
        # v7.5.x: proportional row — the spin + resolved value each take a %% of
        # the panel width so the row fits down to the 100 px minimum.
        try:
            spin.setMinimumWidth(s(24))
            spin.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        except Exception:
            pass
        row.addWidget(lbl, 0)
        row.addWidget(spin, 3)
        row.addWidget(resolved, 2)
        if tooltip:
            spin.setToolTip(tooltip)
            lbl.setToolTip(tooltip)
        return row

    # ── v7.5.x: % of the single calibrated max (shared across pages) ──

    def _pump_speed_anchor(self) -> tuple[float, str]:
        """The pump-jog 100% anchor + its unit, by mode.

        %/µL pages → the needle-derived flow ceiling (µL/s) from the controller.
        Hardware Setup (mm jog) → the legacy raw-plunger feedrate
        (``safety_limits.max_pump_feedrate``, mm/min)."""
        ctrl = self._controller
        if self._pump_action_labels:
            if ctrl is not None and hasattr(ctrl, "get_max_pump_feedrate"):
                try:
                    m = float(ctrl.get_max_pump_feedrate())
                    if m > 0:
                        return m, "µL/s"
                except Exception:
                    pass
            return 0.0, "µL/s"
        mpf = 0.0
        sl = getattr(ctrl, "safety_limits", None) if ctrl else None
        if sl is not None:
            mpf = getattr(sl, "max_pump_feedrate", 0.0) or 0.0
        if not mpf and self._settings is not None:
            try:
                mpf = float(self._settings.get(
                    "safety_limits.max_pump_feedrate", 200.0))
            except (TypeError, ValueError):
                mpf = 200.0
        return float(mpf or 200.0), "mm/min"

    def _pump_speed_anchor_for(self, pump: str) -> tuple[float, str]:
        """ONE pump's own jog 100% anchor + unit, by mode.

        %/µL pages → THIS pump's needle/syringe-derived flow ceiling (µL/s)
        via the controller's per-pump resolver. Hardware Setup (mm jog) →
        this pump's max plunger feedrate (per-pump override aware, mm/min).
        Falls back to the shared :meth:`_pump_speed_anchor` when the
        controller lacks the per-pump resolvers (older stub / test double)."""
        ctrl = self._controller
        if self._pump_action_labels:
            if ctrl is not None and hasattr(ctrl, "get_max_pump_feedrate_for"):
                try:
                    m = float(ctrl.get_max_pump_feedrate_for(pump))
                    if m > 0:
                        return m, "µL/s"
                except Exception:
                    pass
            return self._pump_speed_anchor()
        if ctrl is not None and hasattr(ctrl, "get_pump_max_feedrate_mm_min"):
            try:
                m = float(ctrl.get_pump_max_feedrate_mm_min(pump))
                if m > 0:
                    return m, "mm/min"
            except Exception:
                pass
        return self._pump_speed_anchor()

    def _pump_jog_pct_value(self, pump: str) -> float:
        """The jog-flow % chosen for ``pump`` (its per-pump spin; 50% default
        when the row doesn't exist, e.g. a partially built test panel)."""
        spin = getattr(self, "spin_p_pct_pumps", {}).get(pump)
        if spin is None:
            return 50.0
        try:
            return float(spin.value())
        except (TypeError, ValueError):
            return 50.0

    def _max_xy_speed_um_s(self) -> float:
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "get_max_xy_speed_um_s"):
            try:
                v = float(ctrl.get_max_xy_speed_um_s())
                if v > 0:
                    return v
            except Exception:
                pass
        if self._settings is not None:
            try:
                v = float(self._settings.get("safety_limits.max_xy_speed", 10000.0))
                if v > 0:
                    return v
            except (TypeError, ValueError):
                pass
        return 10000.0

    def _max_z_feedrate_mm_min(self) -> float:
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "get_max_z_feedrate_mm_min"):
            try:
                v = float(ctrl.get_max_z_feedrate_mm_min())
                if v > 0:
                    return v
            except Exception:
                pass
        if self._settings is not None:
            per_axis = self._settings.get(
                "device_profile.per_axis_max_feedrate") or {}
            try:
                v = float(per_axis.get("Z")
                          or self._settings.get("safety_limits.max_z_feedrate", 500.0))
                if v > 0:
                    return v
            except (TypeError, ValueError):
                pass
        return 500.0

    def _resolve_speeds(self) -> None:
        """Refresh the read-only resolved-speed labels from the current % and
        the controller's single per-axis max resolvers."""
        if not hasattr(self, "spin_xy_pct"):
            return
        # Max mode (Hardware Setup): the spins hold absolute values, the resolved
        # labels are hidden — nothing to resolve.
        if getattr(self, "_speed_as_max", False):
            return
        xy_max = self._max_xy_speed_um_s()
        z_max = self._max_z_feedrate_mm_min()
        xy = self.spin_xy_pct.value() / 100.0 * xy_max
        z = self.spin_z_pct.value() / 100.0 * z_max
        self.lbl_xy_resolved.setText(f"= {xy:,.0f} µm/s")
        self.lbl_z_resolved.setText(f"= {z:,.0f} mm/min")
        # v7.9.x: per-pump rows — each resolved against its OWN pump's anchor;
        # pumps that aren't configured are hidden (mirrors the Hardware Setup
        # per-pump max rows).
        configured = set(self._configured_pump_ids())
        for pid, spin in getattr(self, "spin_p_pct_pumps", {}).items():
            row = self.row_p_pct_pumps.get(pid)
            show = pid in configured
            if row is not None:
                row.setVisible(show)
            if not show:
                continue
            lbl = self.lbl_p_resolved_pumps.get(pid)
            if lbl is None:
                continue
            p_max, p_unit = self._pump_speed_anchor_for(pid)
            p = spin.value() / 100.0 * p_max
            if p_unit == "µL/s":
                lbl.setText(f"= {p:.2f} µL/s" if p_max > 0 else "= — µL/s")
            else:
                lbl.setText(f"= {p:,.0f} mm/min")

    def _on_speed_pct_changed(self, group: str) -> None:
        """A percent spinbox changed: persist the shared % on the controller,
        refresh the resolved labels, and (for XY) push the velocity now."""
        spin = {"xy": getattr(self, "spin_xy_pct", None),
                "z": getattr(self, "spin_z_pct", None)}.get(group)
        if spin is None:
            return
        self._speed_user_edited[group] = True
        # Max mode (Hardware Setup): the spin value IS the axis's max speed —
        # write it to the one common source + fan out instead of a jog %.
        if getattr(self, "_speed_as_max", False):
            self._write_axis_max(group, float(spin.value()))
            return
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_jog_speed_pct"):
            try:
                ctrl.set_jog_speed_pct(group, float(spin.value()))
            except Exception as e:
                logger.debug(f"set_jog_speed_pct({group}) failed: {e}")
        self._resolve_speeds()
        if group == "xy":
            self._apply_xy_speed(spin.value() / 100.0 * self._max_xy_speed_um_s())

    def _on_pump_pct_changed(self, pump: str) -> None:
        """A per-pump jog-flow % spin changed: store it on the controller (the
        shared state every jog tile re-reads on show, so the nine panels can't
        diverge), persist it, and refresh the resolved rate."""
        spin = getattr(self, "spin_p_pct_pumps", {}).get(pump)
        if spin is None:
            return
        self._speed_user_edited["p"] = True
        val = float(spin.value())
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_pump_jog_pct"):
            try:
                ctrl.set_pump_jog_pct(pump, val)
            except Exception as e:
                logger.debug(f"set_pump_jog_pct({pump}) failed: {e}")
        if self._settings is not None:
            try:
                stored = dict(self._settings.get("jog.pump_jog_pct") or {})
                stored[pump] = val
                self._settings.set("jog.pump_jog_pct", stored)
                self._settings.save()
            except Exception:
                pass
        self._resolve_speeds()

    def _seed_jog_pct_from_controller(self) -> None:
        """Adopt the shared jog-% the controller holds (so this panel agrees
        with the Xbox page and any other surface). Falls back to the spinbox
        default when the controller has no stored value yet."""
        if not hasattr(self, "spin_xy_pct"):
            return
        ctrl = self._controller
        state = {}
        if ctrl is not None and hasattr(ctrl, "get_jog_speed_state"):
            try:
                state = ctrl.get_jog_speed_state() or {}
            except Exception:
                state = {}
        for group, spin in (("xy", self.spin_xy_pct),
                            ("z", self.spin_z_pct)):
            st = state.get(group)
            if st and st.get("pct") is not None:
                spin.blockSignals(True)
                try:
                    spin.setValue(float(st["pct"]))
                except (TypeError, ValueError):
                    pass
                spin.blockSignals(False)
        self._seed_pump_jog_pcts(state)

    # ── v7.9.x: jog step-slider settings, shared across every jog tile ──

    def _persist_step_settings(self) -> None:
        """A step slider's config changed: store it on the controller (shared
        by every jog panel) and persist it (survives a restart)."""
        arr = getattr(self, "_jog_array", None)
        if arr is None:
            return
        try:
            cfg = arr.step_settings()
        except Exception as e:
            logger.debug(f"step_settings read failed: {e}")
            return
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "set_jog_step_settings"):
            try:
                ctrl.set_jog_step_settings(cfg)
            except Exception as e:
                logger.debug(f"set_jog_step_settings failed: {e}")
        if self._settings is not None:
            try:
                self._settings.set("jog.step_settings", cfg)
                self._settings.save()
            except Exception:
                pass

    def _seed_step_settings(self) -> None:
        """Seed the jog array's step sliders. Precedence mirrors the per-pump
        jog %: the controller's store (freshest — another panel may have
        edited it this session) → the persisted settings (restart; pushed
        back onto the controller so later panels agree without re-reading
        settings) → the array's built-in defaults."""
        arr = getattr(self, "_jog_array", None)
        if arr is None:
            return
        ctrl = self._controller
        cfg = {}
        if ctrl is not None and hasattr(ctrl, "get_jog_step_settings"):
            try:
                cfg = ctrl.get_jog_step_settings() or {}
            except Exception:
                cfg = {}
        if not cfg and self._settings is not None:
            try:
                cfg = self._settings.get("jog.step_settings") or {}
            except Exception:
                cfg = {}
            if cfg and ctrl is not None and hasattr(
                    ctrl, "set_jog_step_settings"):
                try:
                    ctrl.set_jog_step_settings(cfg)
                except Exception:
                    pass
        if cfg:
            try:
                arr.apply_step_settings(cfg)
            except Exception as e:
                logger.debug(f"apply_step_settings failed: {e}")

    def _seed_pump_jog_pcts(self, state: dict | None = None) -> None:
        """Seed each per-pump jog-flow % spin. Precedence: the controller's
        per-pump store (freshest — another panel may have edited it this
        session) → the persisted ``jog.pump_jog_pct`` settings (restart; the
        value is pushed back onto the controller so every other panel agrees
        without re-reading settings) → the shared 'p' group % (pre-per-pump
        installs) → leave the spinbox default."""
        pumps = getattr(self, "spin_p_pct_pumps", None)
        if not pumps:
            return
        ctrl = self._controller
        stored_ctrl: dict = {}
        if ctrl is not None and hasattr(ctrl, "get_pump_jog_pcts"):
            try:
                stored_ctrl = dict(ctrl.get_pump_jog_pcts() or {})
            except Exception:
                stored_ctrl = {}
        stored_settings: dict = {}
        if self._settings is not None:
            try:
                stored_settings = dict(
                    self._settings.get("jog.pump_jog_pct") or {})
            except Exception:
                stored_settings = {}
        shared = None
        if state is not None:
            st = state.get("p") or {}
            shared = st.get("pct")
        for pid, spin in pumps.items():
            val = stored_ctrl.get(pid)
            if val is None:
                val = stored_settings.get(pid)
                if (val is not None and ctrl is not None
                        and hasattr(ctrl, "set_pump_jog_pct")):
                    try:
                        ctrl.set_pump_jog_pct(pid, float(val))
                    except Exception:
                        pass
            if val is None:
                val = shared
            if val is None:
                continue
            spin.blockSignals(True)
            try:
                spin.setValue(float(val))
            except (TypeError, ValueError):
                pass
            spin.blockSignals(False)

    def _seed_axis_max_from_controller(self) -> None:
        """Max mode: seed the absolute-max spinboxes from the single common
        resolvers so this panel reflects the stored ceilings."""
        if not hasattr(self, "spin_xy_pct"):
            return
        for spin, val in ((self.spin_xy_pct, self._max_xy_speed_um_s()),
                          (self.spin_z_pct, self._max_z_feedrate_mm_min())):
            try:
                v = float(val)
            except (TypeError, ValueError):
                continue
            if v <= 0:
                continue
            spin.blockSignals(True)
            try:
                spin.setValue(v)
            except (TypeError, ValueError):
                pass
            spin.blockSignals(False)
        self._seed_pump_max_from_controller()

    # ── v7.5.x: per-pump max feedrate (mm/min) + µL/s readout ────

    def _configured_pump_ids(self) -> list[str]:
        """Configured pump ids from the controller's hardware config; falls back
        to all three so the section is never empty when no config is present."""
        ctrl = self._controller
        hw = getattr(ctrl, "_hardware_config", None) if ctrl is not None else None
        if hw is not None:
            try:
                ids = list(getattr(hw, "configured_pump_ids", []) or [])
                if ids:
                    return ids
            except Exception:
                pass
        return ["P1", "P2", "P3"]

    def _seed_pump_max_from_controller(self) -> None:
        """Seed each per-pump max-feedrate spin (mm/min) from the controller and
        hide pumps that aren't configured."""
        pumps = getattr(self, "spin_p_max_pumps", None)
        if not pumps:
            return
        ctrl = self._controller
        configured = set(self._configured_pump_ids())
        for pid, spin in pumps.items():
            row = self.row_p_max_pumps.get(pid)
            show = pid in configured
            if row is not None:
                row.setVisible(show)
            if not show:
                continue
            mm_min = 200.0
            if ctrl is not None and hasattr(ctrl, "get_pump_max_feedrate_mm_min"):
                try:
                    mm_min = float(ctrl.get_pump_max_feedrate_mm_min(pid))
                except Exception:
                    mm_min = 200.0
            elif self._settings is not None:
                try:
                    mm_min = float(self._settings.get(
                        f"safety_limits.max_pump_feedrate_{pid.lower()}", 0.0)) \
                        or float(self._settings.get(
                            "safety_limits.max_pump_feedrate", 200.0))
                except (TypeError, ValueError):
                    mm_min = 200.0
            if mm_min <= 0:
                mm_min = 200.0
            spin.blockSignals(True)
            try:
                spin.setValue(mm_min)
            except (TypeError, ValueError):
                pass
            spin.blockSignals(False)
            self._update_pump_max_uL(pid)

    def _update_pump_max_uL(self, pump: str) -> None:
        """Refresh a pump's µL/s secondary readout from its current mm/min spin
        value and its configured syringe. Shows '—' when no syringe."""
        lbl = getattr(self, "lbl_p_max_uL", {}).get(pump)
        spin = getattr(self, "spin_p_max_pumps", {}).get(pump)
        if lbl is None or spin is None:
            return
        mm_min = float(spin.value())
        uL_s = None
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "pump_feedrate_mm_min_to_uL_s"):
            try:
                uL_s = ctrl.pump_feedrate_mm_min_to_uL_s(pump, mm_min)
            except Exception:
                uL_s = None
        try:
            if uL_s is not None:
                lbl.setText(f"= {float(uL_s):.3f} µL/s")
            else:
                lbl.setText("= — µL/s")
        except (TypeError, ValueError):
            lbl.setText("= — µL/s")

    def _on_pump_max_changed(self, pump: str) -> None:
        """A per-pump max-feedrate spin changed: persist it, refresh the µL/s
        readout, and fan the change out so every %-of-max surface re-anchors."""
        spin = getattr(self, "spin_p_max_pumps", {}).get(pump)
        if spin is None:
            return
        self._speed_user_edited["p"] = True
        self._write_pump_max(pump, float(spin.value()))
        self._update_pump_max_uL(pump)

    def _write_pump_max(self, pump: str, value: float) -> None:
        """Write ``value`` (mm/min) as ``pump``'s per-pump max plunger feedrate to
        the controller + settings, then broadcast the speed-limit change."""
        ctrl = self._controller
        s = self._settings
        if ctrl is not None and hasattr(ctrl, "set_pump_max_feedrate_mm_min"):
            try:
                ctrl.set_pump_max_feedrate_mm_min(pump, value)
            except Exception as e:
                logger.debug(f"set_pump_max_feedrate_mm_min({pump}) failed: {e}")
        if s is not None:
            try:
                s.set(f"safety_limits.max_pump_feedrate_{pump.lower()}", value)
                s.save()
            except Exception:
                pass
        if ctrl is not None and hasattr(ctrl, "notify_speed_limits_changed"):
            try:
                ctrl.notify_speed_limits_changed()
            except Exception as e:
                logger.debug(f"notify_speed_limits_changed failed: {e}")

    def _write_axis_max(self, group: str, value: float) -> None:
        """Max mode (Hardware Setup): write ``value`` as the axis's max speed to
        the single common source, persist it, and fan the change out so every
        %-of-max page re-anchors. XY → ``safety_limits.max_xy_speed`` (µm/s);
        Z → ``device_profile.per_axis_max_feedrate['Z']`` (mm/min, resolver
        primary) + mirror ``max_z_feedrate``; Pump → ``max_pump_feedrate``."""
        ctrl = self._controller
        s = self._settings
        sl = getattr(ctrl, "safety_limits", None) if ctrl is not None else None
        if group == "xy":
            # v7.21.2: delegate to the ONE writer. This branch used to set
            # `safety_limits.max_xy_speed` and nothing else — moving the 100 %
            # anchor every jog/print percentage reads while leaving the XYStage
            # still converting mm/s against a stale denominator, so every
            # commanded speed came out wrong by the ratio of the two. It also
            # never wrote `device_profile.xy_max_speed_um_s`, so the value did
            # not survive a restart, nor the timing store the simulator reads.
            try:
                from SupportClasses.PrintTimingCalibrationStore import (
                    get_store as _get_tc_store,
                )
                from SupportClasses.XYCalibrationRun import commit_top_speed
                commit_top_speed(ctrl, s, _get_tc_store(), float(value))
                if ctrl is not None and hasattr(
                        ctrl, "notify_speed_limits_changed"):
                    ctrl.notify_speed_limits_changed()
            except Exception as e:
                logger.debug(f"_write_axis_max(xy) delegate failed: {e}")
                if sl is not None:
                    try:
                        sl.max_xy_speed = value
                    except Exception:
                        pass
                if s is not None:
                    try:
                        s.set("safety_limits.max_xy_speed", value)
                        s.save()
                    except Exception:
                        pass
        elif group == "z":
            per_axis = {}
            if s is not None:
                per_axis = dict(
                    (s.get("device_profile.per_axis_max_feedrate") or {}))
            per_axis["Z"] = value
            if s is not None:
                try:
                    s.set("device_profile.per_axis_max_feedrate", per_axis)
                    s.set("safety_limits.max_z_feedrate", value)
                    s.save()
                except Exception:
                    pass
            if ctrl is not None and hasattr(ctrl, "apply_device_settings"):
                try:
                    ctrl.apply_device_settings(
                        per_axis_max_feedrate=per_axis, persist_feedrate=True)
                except Exception as e:
                    logger.debug(f"apply Z max failed: {e}")
            if sl is not None:
                try:
                    sl.max_z_feedrate = value
                except Exception:
                    pass
        else:  # pump
            if sl is not None:
                try:
                    sl.max_pump_feedrate = value
                except Exception:
                    pass
            if s is not None:
                try:
                    s.set("safety_limits.max_pump_feedrate", value)
                    s.save()
                except Exception:
                    pass
        # Re-anchor the jog handlers + fan out to every %-of-max surface.
        if ctrl is not None and hasattr(ctrl, "notify_speed_limits_changed"):
            try:
                ctrl.notify_speed_limits_changed()
            except Exception as e:
                logger.debug(f"notify_speed_limits_changed failed: {e}")

    def refresh_speed_limits(self) -> None:
        """v7.5.x: external hook — re-read the single common per-axis max and
        the shared jog-%. Call after any page changes a max (the GUI fans this
        out via ``HardwareSetupPage.safety_limits_changed``) or on show."""
        # v7.9.x: the step-slider config is shared by every jog tile, so it is
        # re-read on the same hooks as the speeds (set_controller /
        # set_settings / showEvent) in BOTH modes.
        self._seed_step_settings()
        if getattr(self, "_speed_as_max", False):
            self._seed_axis_max_from_controller()
            return
        self._seed_jog_pct_from_controller()
        self._resolve_speeds()

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

    def _build_position_group(self) -> QWidget:
        card = Card("Live Position", collapsible=True)
        lay = card.body_layout()

        # v7.4.2: per-axis row = label + slider bar (extents from safety
        # limits) + numeric value + unit. The bar visualises how close
        # the stage is to its recorded min/max envelope.
        grid = QGridLayout()
        grid.setHorizontalSpacing(s(5))
        grid.setVerticalSpacing(s(6))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)   # bar absorbs the slack
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)

        self.lbl_pos: dict[str, QLabel] = {}
        self.bar_pos: dict[str, PositionBar] = {}
        # v7.5.x: per-axis unit labels are kept so the pump readout can flip
        # "mm" → "µL" once the plunger is calibrated (fill 0=empty → full).
        self.unit_lbl_pos: dict[str, QLabel] = {}
        for r, (axis, unit) in enumerate([
            ("X", "µm"), ("Y", "µm"), ("Z", "mm"),
            ("P1", "mm"), ("P2", "mm"), ("P3", "mm"),
        ]):
            ax_lbl = QLabel(f"<b>{axis}</b>")
            ax_lbl.setStyleSheet(f"color: {COLORS['text']};")
            ax_lbl.setMinimumWidth(s(14))
            grid.addWidget(ax_lbl, r, 0)
            bar = PositionBar()
            # Let the bar shrink freely so the row fits a narrow panel — the
            # bar is the element that yields; the value text keeps priority.
            bar.setMinimumWidth(s(10))
            bar.setSizePolicy(QSizePolicy.Ignored, bar.sizePolicy().verticalPolicy())
            grid.addWidget(bar, r, 1)
            self.bar_pos[axis] = bar
            # v7.9.x: the numeric readout's min width tracks its text (see
            # PositionValueLabel) so the stretch-driven bar can never squeeze
            # the number into clipping over the unit label.
            val = PositionValueLabel()
            grid.addWidget(val, r, 2)
            unit_lbl = QLabel(unit)
            unit_lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
            grid.addWidget(unit_lbl, r, 3)
            self.lbl_pos[axis] = val
            self.unit_lbl_pos[axis] = unit_lbl
        lay.addLayout(grid)

        return card

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
                # v7.5.x: the Z readout is the unified user frame (0 at the
                # bottom datum, up = +), so the Z bar extents are too. Convert
                # both raw bounds and order by value (polarity-general).
                "Z":  (min(self._z_raw_to_user(s_obj.get("safety_limits.z_min", -10.0)),
                           self._z_raw_to_user(s_obj.get("safety_limits.z_max", 50.0))),
                       max(self._z_raw_to_user(s_obj.get("safety_limits.z_min", -10.0)),
                           self._z_raw_to_user(s_obj.get("safety_limits.z_max", 50.0)))),
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
        # v7.5.x: a calibrated pump's bar shows the FILL range (0 = empty →
        # capacity = full), matching the µL readout; uncalibrated pumps keep the
        # raw-mm envelope above.
        c = self._controller
        if c is not None and hasattr(c, "pump_capacity_uL"):
            for pump in ("P1", "P2", "P3"):
                try:
                    cap = c.pump_capacity_uL(pump)
                except Exception:
                    cap = None
                if cap is not None and cap > 0:
                    ranges[pump] = (0.0, float(cap))
        for axis, (lo, hi) in ranges.items():
            bar = self.bar_pos.get(axis)
            if bar is not None:
                try:
                    bar.set_range(float(lo), float(hi))
                except Exception:
                    pass

    # ── Connect handlers ────────────────────────────────────────

    def _connect_xy(self) -> None:
        """v7.4.2: open the real XY hardware."""
        self._open_xy(simulate=False)

    def _simulate_xy(self) -> None:
        """v7.4.2: open the XY simulator (no hardware required)."""
        self._open_xy(simulate=True)

    def _open_xy(self, *, simulate: bool) -> None:
        if self._controller is None:
            return
        # If a stage is already attached, tear it down first so we can
        # switch between real / simulator without leaving a stale handle.
        if self._controller.is_xy_connected:
            try:
                self._controller.disconnect_xy()
            except Exception as e:
                logger.warning(f"disconnect_xy before reopen failed: {e}")
        self.badge_xy.set_status(
            "info", "Starting simulator…" if simulate else "Connecting…")
        try:
            self._controller.connect_xy(simulate=simulate)
            ok = bool(self._controller.is_xy_connected)
            if ok:
                self.badge_xy.set_status(
                    "ok", "Simulated" if simulate else "Connected")
            else:
                self.badge_xy.set_status("err", "Failed")
            # v7.18.1: cache what actually answered (protocol + port + baud)
            # so the next connect is one probe, not a full protocol × baud
            # sweep. Real HW only — a simulator hint would be meaningless.
            if ok and not simulate and self._settings is not None:
                hint = self._controller.xy_connection_hint
                if hint:
                    self._settings.set("xy_stage.last_good", hint)
                    self._settings.save()
                    logger.info("XY last_good cached: %s", hint)
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
        """v7.4.2: open the real ZP hardware."""
        self._open_zp(simulate=False)

    def _simulate_zp(self) -> None:
        """v7.4.2: open the ZP simulator (no hardware required)."""
        self._open_zp(simulate=True)

    def _open_zp(self, *, simulate: bool) -> None:
        if self._controller is None:
            return
        if self._controller.is_zp_connected:
            try:
                self._controller.disconnect_zp()
            except Exception as e:
                logger.warning(f"disconnect_zp before reopen failed: {e}")
        self.badge_zp.set_status(
            "info", "Starting simulator…" if simulate else "Connecting…")
        try:
            self._controller.connect_zp(simulate=simulate)
            ok = bool(self._controller.is_zp_connected)
            if ok:
                self.badge_zp.set_status(
                    "ok", "Simulated" if simulate else "Connected")
            else:
                self.badge_zp.set_status("err", "Failed")
            # Cache the connected serial port — only meaningful for real HW.
            if ok and not simulate and self._settings is not None:
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

    # ── Microscope body ─────────────────────────────────────────

    @staticmethod
    def _microscope():
        """The shared microscope controller, or None if unavailable.

        Imported lazily so a build missing the microscope module (or with a
        broken optional dependency) still shows the rest of this panel.
        """
        try:
            from SupportClasses.MicroscopeControl import get_microscope
            return get_microscope()
        except Exception as e:            # pragma: no cover - import guard
            logger.warning(f"microscope controller unavailable: {e}")
            return None

    def _connect_microscope(self) -> None:
        """Open the body with the driver saved on Hardware Setup → Microscope."""
        self._open_microscope(None)

    def _simulate_microscope(self) -> None:
        """Open the software microscope — leaves the saved driver alone."""
        self._open_microscope("simulated")

    def _open_microscope(self, backend_name) -> None:
        scope = self._microscope()
        if scope is None:
            self.badge_scope.set_status("err", "Unavailable")
            return
        self.badge_scope.set_status(
            "info", "Starting simulator…" if backend_name == "simulated"
            else "Connecting…")
        try:
            # Asynchronous: the worker thread owns the COM apartment, so the
            # outcome arrives on a later tick via _sync_badges().
            scope.connect(backend_name)
        except Exception as e:
            logger.warning(f"microscope connect failed: {e}")
            self.badge_scope.set_status("err", f"Error: {e}")

    def _disconnect_microscope(self) -> None:
        scope = self._microscope()
        if scope is None:
            return
        try:
            scope.disconnect()
        except Exception as e:
            logger.warning(f"microscope disconnect failed: {e}")
        self.badge_scope.set_status("pending", "Not connected")

    def _sync_microscope_badge(self) -> None:
        """v7.5.x: reflect the body's live state.

        ``connect()`` is asynchronous, so the outcome lands here on a later
        tick rather than at the click.
        """
        if not hasattr(self, 'badge_scope'):
            return
        scope = self._microscope()
        if scope is None:
            self.badge_scope.set_status("err", "Unavailable")
            return
        try:
            state = scope.state()
        except Exception:                 # pragma: no cover - defensive
            return
        if not state.connected:
            # busy while disconnected == a connect is queued or running, so
            # the transient label survives until the worker resolves it.
            if state.busy:
                self.badge_scope.set_status("info", "Connecting…")
            elif state.error:
                self.badge_scope.set_status("err", str(state.error)[:60])
            else:
                self.badge_scope.set_status("pending", "Not connected")
        elif state.busy:
            self.badge_scope.set_status("info", "Moving…")
        elif state.backend == "simulated":
            self.badge_scope.set_status("ok", "Simulated")
        else:
            self.badge_scope.set_status("ok", "Connected")

    def _sync_badges(self) -> None:
        # The microscope is its own singleton, not part of StageController, so
        # its badge syncs whether or not a stage controller has been injected.
        self._sync_microscope_badge()
        if self._controller is None:
            return
        # v7.4.2: panel may be configured without the Connect group
        # (calibration variant) — bail if the badge widgets don't exist.
        if not hasattr(self, 'badge_xy'):
            return
        xy_ok = bool(getattr(self._controller, 'is_xy_connected', False))
        zp_ok = bool(getattr(self._controller, 'is_zp_connected', False))
        # v7.4.2: distinguish a real-hardware connection from a simulator
        # so the operator sees at a glance which one is driving the stage.
        xy_sim = bool(getattr(self._controller, 'simulate_xy', False)) and xy_ok
        zp_sim = bool(getattr(self._controller, 'simulate_zp', False)) and zp_ok
        if not xy_ok:
            self.badge_xy.set_status("pending", "Not connected")
        else:
            self.badge_xy.set_status(
                "ok", "Simulated" if xy_sim else "Connected")
        if not zp_ok:
            self.badge_zp.set_status("pending", "Not connected")
        else:
            self.badge_zp.set_status(
                "ok", "Simulated" if zp_sim else "Connected")
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
        # v7.5.x: run the XY move + speed-set + fresh read OFF the GUI thread —
        # each is a blocking serial round-trip that otherwise freezes the camera
        # feed (a QTimer on the GUI thread) while jogging. Overlapping clicks are
        # dropped (busy guard) instead of queuing behind the serial channel.
        if self._xy_jog_busy:
            return
        # v7.5.x: jog speed = % of the single calibrated XY max. Resolve the
        # absolute µm/s now (reads the GUI spin) and push it inside the worker.
        # Floored at ≥1 so a zero/unset max can never command F0.
        if getattr(self, "_speed_as_max", False):
            xy_um_s = max(1.0, self.spin_xy_pct.value())
        else:
            xy_um_s = max(1.0, self.spin_xy_pct.value() / 100.0
                          * self._max_xy_speed_um_s())
        bypass = self._bypass_safety
        ctrl = self._controller

        def _move():
            self._apply_xy_speed(xy_um_s)
            ctrl.move_xy_relative_um(dx_um, dy_um, bypass_safety=bypass)

        self._run_stage_jog("_xy_jog_busy", _move,
                            f"jog XY ({dx_um}, {dy_um})")

    def _on_jog_z(self, dz_mm: float) -> None:
        if self._controller is None:
            return
        if self._z_jog_busy:
            return
        # v7.5.x: jog speed = % of the single calibrated Z max feedrate.
        # Floored at ≥1 mm/min so an unset max can never command F0.
        if getattr(self, "_speed_as_max", False):
            feed = max(1.0, self.spin_z_pct.value())
        else:
            feed = max(1.0, self.spin_z_pct.value() / 100.0
                       * self._max_z_feedrate_mm_min())
        bypass = self._bypass_safety
        ctrl = self._controller

        def _move():
            # dz_mm is a HEIGHT-frame delta (+ = up); route through
            # move_z_user_relative so "up" follows the taught z_up_sign.
            try:
                ctrl.move_z_user_relative(
                    dz_mm, feedrate=feed, bypass_safety=bypass)
            except TypeError:
                ctrl.move_z_user_relative(dz_mm, bypass_safety=bypass)

        self._run_stage_jog("_z_jog_busy", _move, f"jog Z {dz_mm}")

    def _run_stage_jog(self, busy_attr: str, move_fn, label: str) -> None:
        """v7.5.x: run an XY/Z jog move on a daemon thread, then read the fresh
        positions there too, and hand them back to the GUI thread via
        ``_stage_jog_done`` — so no blocking serial I/O happens on the GUI thread
        (which was freezing the camera feed during a jog)."""
        setattr(self, busy_attr, True)
        ctrl = self._controller

        def _worker():
            xy = zp = None
            try:
                move_fn()
            except Exception as e:
                logger.warning(f"{label} failed: {e}")
            # Fresh reads on THIS thread (serial access is lock-protected), so
            # the GUI thread never blocks on the round-trip.
            try:
                xy = ctrl.get_xy_position(cached=False)
            except Exception:
                xy = None
            try:
                zp = ctrl.get_zp_position(cached=False)
            except Exception:
                zp = None
            setattr(self, busy_attr, False)
            self._stage_jog_done.emit(xy, zp)

        threading.Thread(target=_worker, name="StageJog", daemon=True).start()

    def _on_stage_jog_done(self, xy, zp) -> None:
        """GUI-thread continuation of an XY/Z jog: update the readout from the
        worker-read positions (no serial I/O here). Falls back to the cached
        display values if a read failed."""
        try:
            if xy is None or zp is None:
                xy, zp = self._display_xy(), self._display_zp()
            self._update_position_displays(xy, zp)
        except Exception as e:
            logger.debug(f"stage jog display refresh failed: {e}")

    def _on_jog_pump(self, pump: str, distance: float) -> None:
        if self._controller is None:
            return
        ctrl = self._controller
        # v7.5.x: a pump move (esp. backlash-compensated: take-up + main +
        # unload, each drained via a blocking M400 + a settle dwell) can take
        # seconds on real hardware. Refuse to overlap a second jog on top of
        # one already in flight rather than queuing it behind the shared
        # serial channel.
        if self._pump_jog_busy:
            logger.debug(f"pump jog already in flight — ignoring {pump} "
                         f"{distance:g}")
            return
        # v7.5.x: %/µL pages drive the plunger volumetrically; Hardware Setup
        # (raw arrows) keeps the mm path for plunger calibration.
        if self._pump_action_labels:
            # v7.9.x: ``distance`` is a SIGNED µL VOLUME straight from the
            # step slider (− = aspirate, + = dispense — matches move_pump_uL).
            # It used to be a % of the syringe volume, which could not express
            # the sub-µL steps the operator asked for (0.001 µL is 0.0004 % of
            # a 250 µL syringe). A legacy array still emitting % is converted
            # so an un-upgraded embedder can't silently dose 100× wrong.
            volume_uL = distance
            arr = getattr(self, "_jog_array", None)
            if arr is not None and not getattr(arr, "pump_step_is_uL", True):
                volume_uL = None
            if volume_uL is None:
                if hasattr(ctrl, "pump_pct_to_uL"):
                    try:
                        volume_uL = ctrl.pump_pct_to_uL(pump, distance)
                    except Exception:
                        volume_uL = None
                if volume_uL is None:
                    self.lbl_status.setText(
                        f"{pump}: calibrate the plunger or assign a syringe "
                        f"to jog this pump.")
                    return
            # v7.9.x: THIS pump's own spin % × its own flow ceiling — each
            # pump gets its own jog flow rate on the tiles.
            p_max, _ = self._pump_speed_anchor_for(pump)   # µL/s ceiling
            rate = self._pump_jog_pct_value(pump) / 100.0 * p_max \
                if p_max > 0 else None
            # v7.5.x: backlash compensation — when enabled (pump jog toggle), the
            # click is bracketed with a flex take-up + unload so it leaves the
            # tip pressure-neutral. A single click is one discrete start→stop, so
            # per-click bracketing is correct here. Off ⇒ compensate=False.
            comp_on = False
            _bce = getattr(ctrl, "backlash_comp_enabled", None)
            if callable(_bce):
                try:
                    comp_on = bool(_bce())
                except Exception:
                    comp_on = False

            def _move():
                try:
                    ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate,
                                      compensate=comp_on)
                except TypeError:
                    try:
                        ctrl.move_pump_uL(pump, volume_uL, rate_uL_s=rate)
                    except TypeError:
                        ctrl.move_pump_uL(pump, volume_uL)
        else:
            # Hardware Setup mm jog — speed = the raw-plunger feedrate.
            # Max mode: the PER-PUMP mm/min spin holds the absolute ceiling for
            # this pump directly; percent mode (legacy / tests): % of THIS
            # pump's own mm/min anchor.
            if getattr(self, "_speed_as_max", False):
                feed, _ = self._pump_speed_anchor()       # mm/min
                spin = getattr(self, "spin_p_max_pumps", {}).get(pump)
                feed = max(1.0, float(spin.value())) if spin is not None \
                    else max(1.0, feed)
            else:
                feed, _ = self._pump_speed_anchor_for(pump)   # mm/min
                feed = self._pump_jog_pct_value(pump) / 100.0 * feed
            bypass = self._bypass_safety

            def _move():
                try:
                    ctrl.move_pump_relative(
                        pump, distance, feedrate=feed, bypass_safety=bypass)
                except TypeError:
                    ctrl.move_pump_relative(
                        pump, distance, bypass_safety=bypass)

        self._pump_jog_busy = True

        def _worker() -> None:
            try:
                _move()
            except Exception as e:
                logger.warning(f"jog {pump} {distance:g} failed: {e}")
            finally:
                # Cross-thread: delivered as a queued call to the GUI thread.
                self._pump_jog_done.emit()

        threading.Thread(target=_worker, name="PumpJog", daemon=True).start()

    def _on_pump_jog_done(self) -> None:
        """GUI-thread continuation once a background pump jog finishes."""
        self._pump_jog_busy = False
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

    def _z_raw_to_user(self, raw_mm: float) -> float:
        """v7.5.x: raw Marlin Z → unified user frame (0 at bottom datum, + up)
        via the live controller; falls back to the module height helper."""
        c = self._controller
        if c is not None and hasattr(c, "raw_to_user_z"):
            return c.raw_to_user_z(raw_mm)
        return z_raw_to_display(raw_mm)

    def _pump_raw_to_fill_uL(self, pump: str, raw_mm: float) -> float | None:
        """v7.5.x: raw Marlin pump position → plunger FILL in µL (0 = empty /
        fully dispensed → capacity = full / fully aspirated) via the live
        controller's plunger calibration. None when the pump isn't calibrated
        (the caller then keeps the raw-mm readout)."""
        c = self._controller
        if c is not None and hasattr(c, "raw_to_pump_fill_uL"):
            try:
                return c.raw_to_pump_fill_uL(pump, raw_mm)
            except Exception:
                return None
        return None

    def _update_position_displays(self, xy, zp) -> None:
        # v7.5.x: axes whose readout is currently a (display-only) motion
        # estimate — marked with a leading '~' + tooltip so the number can't be
        # mistaken for a confirmed sensor reading. Empty when nothing is moving.
        est: set = set()
        c = self._controller
        if c is not None and hasattr(c, "motion_estimating"):
            try:
                est = c.motion_estimating()
            except Exception:
                est = set()

        def _set(axis: str, value: float | None, fmt: str,
                 unit: str | None = None, tooltip: str | None = None) -> None:
            lbl = self.lbl_pos[axis]
            if value is not None:
                text = fmt.format(value)
                if axis in est:
                    text = "~" + text
                    if tooltip is None:
                        tooltip = ("Estimated position (in motion) — snaps to "
                                   "the measured value on arrival")
                lbl.setText(text)
            else:
                lbl.setText("—")
            if tooltip is not None:
                lbl.setToolTip(tooltip)
            if unit is not None and hasattr(self, "unit_lbl_pos"):
                u = self.unit_lbl_pos.get(axis)
                if u is not None:
                    u.setText(unit)
            bar = self.bar_pos.get(axis) if hasattr(self, 'bar_pos') else None
            if bar is not None:
                bar.set_value(value)
        if xy and len(xy) >= 2:
            _set("X", xy[0] if xy[0] is not None else None, "{:,.1f}")
            _set("Y", xy[1] if xy[1] is not None else None, "{:,.1f}")
        if zp:
            for logical in ("Z", "P1", "P2", "P3"):
                v = self._logical_zp_value(zp, logical)
                # v7.5.x: show Z in the unified user frame (0 at bottom datum,
                # up = +).
                if logical == "Z" and v is not None:
                    v = self._z_raw_to_user(v)
                    _set(logical, v, "{:.3f}")
                    continue
                # v7.5.x: show pumps as a FILL LEVEL in µL (0 = empty/dispensed
                # → capacity = full/aspirated) once the plunger is calibrated;
                # keep the raw-mm readout (with the raw value in the tooltip)
                # for uncalibrated pumps. On the %/µL pages (action labels) also
                # show the fill as a % of the syringe capacity.
                if v is not None:
                    fill = self._pump_raw_to_fill_uL(logical, v)
                    if fill is not None:
                        unit = "µL"
                        c = self._controller
                        if (self._pump_action_labels and c is not None
                                and hasattr(c, "pump_uL_to_pct")):
                            try:
                                pct = c.pump_uL_to_pct(logical, fill)
                            except Exception:
                                pct = None
                            if pct is not None:
                                unit = f"µL · {pct:.0f}%"
                        _set(logical, fill, "{:.1f}", unit=unit,
                             tooltip=f"raw {v:.3f} mm")
                        continue
                _set(logical, v, "{:.3f}", unit="mm")
