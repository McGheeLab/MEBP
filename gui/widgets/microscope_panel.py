"""
microscope_panel.py — manual control of the motorized microscope body (v7.5.x).

A compact card for the Nikon Ti Eclipse's three motorized devices:

  1. **Filter cubes** — pick a cassette slot by the operator's own name
     ("DAPI", "mCherry"), see which one is currently in the light path, and
     switch. Slot ↔ name assignment is edited in the Microscope Setup dialog
     and persisted per machine.
  2. **Focus (Z)** — raise / lower the focal plane by a settable step, or drive
     to an absolute position.
  3. **Objectives** — switch the nosepiece and see which objective is in.
  4. **Illumination + light path** (v7.17) — the epi (excitation) shutter, the
     transmitted-light lamp (on/off + level) and the eyepiece ↔ camera-port
     selector. All three are accessories, so each row **hides entirely** when
     the body does not report that device: a control that is visible but
     permanently dead reads as broken software rather than as hardware this
     microscope was never fitted with.

Deliberately standalone: nothing here is wired into prints, workflows or
calibrations yet — that integration is a later, separate step. This is the
manual surface.

Threading: every hardware call goes through :class:`MicroscopeController`, which
serializes them onto its own worker thread, so a multi-hundred-millisecond
turret rotation can never stall the Qt event loop (the failure mode this
codebase has had to fix for jogs, pumps and travel moves). The widget only ever
*reads a cached snapshot* — it polls ``controller.state()`` on a timer rather
than registering a cross-thread callback, so there is no listener that can
outlive the widget.

Single source of truth used in two places, mirroring the illumination LED:
embedded as a ``Card("Microscope")`` in :class:`StandardJogContextPanel`, and
wrapped by ``MicroscopeSection`` so it can be dropped into a Custom panel.
"""

from __future__ import annotations

import logging
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS

logger = logging.getLogger(__name__)

#: How often the widget re-renders from the cached controller state.
_RENDER_MS = 400
#: Minimum spacing between hardware re-reads (turret + focus polls).
_REFRESH_INTERVAL_S = 1.0


class MicroscopePanel(QWidget):
    """Filter cube / focus / objective controls for the microscope body."""

    def __init__(self, controller=None, parent: QWidget | None = None, *,
                 store=None):
        super().__init__(parent)
        if store is None:
            from SupportClasses.MicroscopeConfigStore import get_store
            store = get_store()
        self._store = store
        if controller is None:
            from SupportClasses.MicroscopeControl import get_microscope
            controller = get_microscope()
        self._scope = controller

        self._suppress = False          # block feedback during programmatic sets
        self._last_refresh = 0.0
        self._rendered_key = None       # skip no-op re-renders

        self._build_ui()

        self._timer = QTimer(self)
        self._timer.setInterval(_RENDER_MS)
        self._timer.timeout.connect(self._tick)

        self._rebuild_slot_combos()
        self._render()

    # ── Public API ─────────────────────────────────────────────────

    def microscope(self):
        """The shared :class:`MicroscopeController` this panel drives."""
        return self._scope

    def on_status_update(self) -> None:
        """MainWindow / custom-panel tick — render the cached state."""
        self._render()

    def showEvent(self, event):  # noqa: N802 (Qt override)
        super().showEvent(event)
        self._timer.start()
        self._render()

    def hideEvent(self, event):  # noqa: N802 (Qt override)
        # Stop polling the body while the panel is off-screen; the connection
        # itself is left alone (other pages share it).
        self._timer.stop()
        super().hideEvent(event)

    # ── UI ─────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(s(6))

        # Row 1: status + connect + setup.
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(s(6))
        self._status_lbl = QLabel("not connected")
        self._status_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']};")
        self._status_lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self._status_lbl.setMinimumWidth(s(1))
        top.addWidget(self._status_lbl, stretch=1)
        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setToolTip("Connect to / disconnect from the microscope body.")
        self._btn_connect.clicked.connect(self._toggle_connect)
        top.addWidget(self._btn_connect)
        self._btn_setup = QPushButton("⚙")
        self._btn_setup.setToolTip(
            "Microscope setup — driver, filter-cube and objective assignments, "
            "focus preferences.")
        self._btn_setup.setFixedWidth(s(28))
        self._btn_setup.clicked.connect(self.open_settings)
        top.addWidget(self._btn_setup)
        root.addLayout(top)

        self._error_lbl = QLabel("")
        self._error_lbl.setWordWrap(True)
        self._error_lbl.setStyleSheet(f"color: {COLORS['red']};")
        self._error_lbl.setVisible(False)
        root.addWidget(self._error_lbl)

        # Row 2/3: turret selectors.
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(s(6))
        grid.setVerticalSpacing(s(4))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)

        grid.addWidget(self._caption("Cube"), 0, 0)
        self._filter_combo = self._fluid_combo(
            "Filter cube currently in the light path. Selecting another "
            "rotates the cassette.")
        self._filter_combo.currentIndexChanged.connect(self._on_filter_selected)
        grid.addWidget(self._filter_combo, 0, 1)

        grid.addWidget(self._caption("Obj"), 1, 0)
        self._objective_combo = self._fluid_combo(
            "Objective currently in the light path. Selecting another rotates "
            "the nosepiece.")
        self._objective_combo.currentIndexChanged.connect(
            self._on_objective_selected)
        grid.addWidget(self._objective_combo, 1, 1)

        grid.addWidget(self._caption("Focus"), 2, 0)
        self._focus_lbl = QLabel("—")
        self._focus_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-family: Consolas, Menlo, monospace;")
        self._focus_lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self._focus_lbl.setMinimumWidth(s(1))
        grid.addWidget(self._focus_lbl, 2, 1)

        # v7.17 — light path (eyepiece ↔ camera port).
        self._path_caption = self._caption("Path")
        grid.addWidget(self._path_caption, 3, 0)
        self._light_combo = self._fluid_combo(
            "Which port the light is sent to. Selecting the eyepiece takes the "
            "light off the camera, so every captured frame goes black — that "
            "reads downstream as an exposure fault, not as a setting.")
        self._light_combo.currentIndexChanged.connect(self._on_light_path_selected)
        grid.addWidget(self._light_combo, 3, 1)
        root.addLayout(grid)

        # Row 4: focus jog — down / step / up.
        jog = QHBoxLayout()
        jog.setContentsMargins(0, 0, 0, 0)
        jog.setSpacing(s(4))
        self._btn_down = QPushButton("▼")
        self._btn_down.setToolTip("Lower the focal plane by one step.")
        self._btn_down.clicked.connect(lambda: self._jog_focus(-1))
        jog.addWidget(self._btn_down, stretch=1)
        self._step_spin = QDoubleSpinBox()
        self._step_spin.setRange(0.01, 5000.0)
        self._step_spin.setDecimals(2)
        self._step_spin.setSuffix(" µm")
        self._step_spin.setValue(self._store.focus_step_um())
        self._step_spin.setToolTip("Focus step per button press.")
        self._step_spin.setMinimumWidth(s(30))
        self._step_spin.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self._step_spin.valueChanged.connect(self._on_step_changed)
        jog.addWidget(self._step_spin, stretch=2)
        self._btn_up = QPushButton("▲")
        self._btn_up.setToolTip("Raise the focal plane by one step.")
        self._btn_up.clicked.connect(lambda: self._jog_focus(+1))
        jog.addWidget(self._btn_up, stretch=1)
        root.addLayout(jog)

        # Row 5: absolute go-to.
        goto = QHBoxLayout()
        goto.setContentsMargins(0, 0, 0, 0)
        goto.setSpacing(s(4))
        self._goto_spin = QDoubleSpinBox()
        self._goto_spin.setRange(-1e6, 1e6)
        self._goto_spin.setDecimals(2)
        self._goto_spin.setSuffix(" µm")
        self._goto_spin.setToolTip("Absolute focus position.")
        self._goto_spin.setMinimumWidth(s(30))
        self._goto_spin.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        goto.addWidget(self._goto_spin, stretch=1)
        self._btn_goto = QPushButton("Go")
        self._btn_goto.clicked.connect(self._goto_focus)
        goto.addWidget(self._btn_goto)
        root.addLayout(goto)

        # Row 6: illumination — epi (excitation) shutter + transmitted lamp.
        # Both rows HIDE entirely when the body has no such device: a control that
        # is present but permanently dead reads as broken software rather than as
        # an accessory this microscope was not fitted with.
        illum = QHBoxLayout()
        illum.setContentsMargins(0, 0, 0, 0)
        illum.setSpacing(s(4))
        self._shutter_btn = QPushButton("Excitation")
        self._shutter_btn.setCheckable(True)
        self._shutter_btn.setToolTip(
            "Epi (excitation) shutter. Closed keeps excitation off the sample "
            "between acquisitions.")
        self._shutter_btn.clicked.connect(self._on_shutter_clicked)
        illum.addWidget(self._shutter_btn, stretch=3)
        root.addLayout(illum)
        self._illum_row = illum

        lamp = QHBoxLayout()
        lamp.setContentsMargins(0, 0, 0, 0)
        lamp.setSpacing(s(4))
        self._lamp_btn = QPushButton("Dia lamp")
        self._lamp_btn.setCheckable(True)
        self._lamp_btn.setToolTip("Transmitted-light (brightfield) lamp on/off.")
        self._lamp_btn.clicked.connect(self._on_lamp_clicked)
        lamp.addWidget(self._lamp_btn, stretch=2)
        self._lamp_spin = QDoubleSpinBox()
        self._lamp_spin.setDecimals(0)
        self._lamp_spin.setRange(0.0, 100.0)
        self._lamp_spin.setToolTip(
            "Lamp level, in the units the microscope itself declares — not a "
            "percentage, which would be a made-up number.")
        self._lamp_spin.setMinimumWidth(s(30))
        self._lamp_spin.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self._lamp_spin.editingFinished.connect(self._on_lamp_level)
        lamp.addWidget(self._lamp_spin, stretch=2)
        # Software-vs-front-panel control. A separate, explicit button because
        # switching to Remote takes the lamp away from the knob on the
        # microscope — the operator's call, not a side effect of a slider.
        self._lamp_remote_btn = QPushButton("Man")
        self._lamp_remote_btn.setCheckable(True)
        self._lamp_remote_btn.setToolTip(
            "Software (Remote) vs the microscope's own front-panel (Main) "
            "control of the dia lamp.\n\nThe SDK refuses every software change "
            "while the body's panel owns the lamp, so this must be on before "
            "the controls beside it do anything.")
        self._lamp_remote_btn.clicked.connect(self._on_lamp_remote_clicked)
        lamp.addWidget(self._lamp_remote_btn, stretch=1)
        root.addLayout(lamp)
        self._lamp_row = lamp

    def _caption(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color: {COLORS['subtext0']}; font-weight: 600;")
        lbl.setMinimumWidth(s(24))
        return lbl

    def _fluid_combo(self, tooltip: str) -> QComboBox:
        combo = QComboBox()
        combo.setToolTip(tooltip)
        combo.setMinimumWidth(s(40))
        combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        return combo

    # ── Slot combos ────────────────────────────────────────────────

    def _slot_text(self, position: int, label: str, native: str = "") -> str:
        name = label or native
        return f"{position} · {name}" if name else f"{position} · (empty)"

    def _rebuild_slot_combos(self) -> None:
        """Repopulate both combos from the store + whatever the hardware
        reports. Called at build time and whenever the assignments change."""
        state = self._scope.state()
        self._suppress = True
        try:
            self._fill_combo(
                self._filter_combo,
                count=max(state.filter_count, self._store.filter_slots()),
                labels=self._store.filter_labels(),
                native=state.native_filter_names,
                current=state.filter_position)
            self._fill_combo(
                self._objective_combo,
                count=max(state.objective_count, self._store.objective_slots()),
                labels=self._store.objective_labels(),
                native=state.native_objective_names,
                current=state.objective_position)
            self._fill_light_combo(state)
        finally:
            self._suppress = False

    def _fill_light_combo(self, state) -> None:
        """Light-path positions. No store labels — see the plan's D11.

        The SDK exposes no per-position name table, so whatever the driver
        reports is used and the fallback is the position number. ``(empty)`` would
        be wrong here: every position on a light-path drive exists.
        """
        combo = self._light_combo
        names = state.native_light_path_names or ()
        combo.clear()
        for pos in range(1, max(0, int(state.light_path_count)) + 1):
            name = names[pos - 1] if len(names) >= pos else ""
            combo.addItem(f"{pos} · {name}" if name else f"port {pos}", pos)
        if state.light_path_position is not None:
            idx = combo.findData(int(state.light_path_position))
            if idx >= 0:
                combo.setCurrentIndex(idx)

    def _fill_combo(self, combo: QComboBox, *, count: int, labels: dict,
                    native, current) -> None:
        combo.clear()
        for pos in range(1, max(1, int(count)) + 1):
            native_name = (native[pos - 1]
                           if native and len(native) >= pos else "")
            combo.addItem(self._slot_text(pos, labels.get(pos, ""),
                                          native_name), pos)
        if current is not None:
            idx = combo.findData(int(current))
            if idx >= 0:
                combo.setCurrentIndex(idx)

    # ── Commands ───────────────────────────────────────────────────

    def _connected(self) -> bool:
        return bool(self._scope.state().connected)

    def _toggle_connect(self) -> None:
        if self._connected():
            self._scope.disconnect()
        else:
            self._scope.connect()
        self._last_refresh = time.monotonic()
        self._render(force=True)

    def open_settings(self) -> None:
        """Open the Microscope Setup dialog and re-apply what it changed."""
        from gui.dialogs.microscope_settings_dialog import (
            MicroscopeSettingsDialog)
        backend_before = self._store.get_backend()
        dlg = MicroscopeSettingsDialog(
            self._store, parent=self, controller=self._scope)
        if not dlg.exec():
            return  # Cancel — the store was not touched
        self._suppress = True
        try:
            self._step_spin.setValue(self._store.focus_step_um())
        finally:
            self._suppress = False
        self._rebuild_slot_combos()
        # A driver change only takes effect on the next connect; reconnect now
        # so what the panel shows matches what it is talking to.
        if self._store.get_backend() != backend_before and self._connected():
            self._scope.disconnect()
            self._scope.connect()
        self._render(force=True)

    def _on_step_changed(self, value: float) -> None:
        if self._suppress:
            return
        try:
            self._store.set_focus_step_um(float(value))
        except Exception as exc:
            logger.debug(f"focus step persist failed: {exc}")

    def _on_filter_selected(self, _index: int) -> None:
        if self._suppress or not self._connected():
            return
        pos = self._filter_combo.currentData()
        if pos is None or pos == self._scope.state().filter_position:
            return
        self._scope.set_filter(int(pos))
        self._render(force=True)

    def _on_objective_selected(self, _index: int) -> None:
        if self._suppress or not self._connected():
            return
        pos = self._objective_combo.currentData()
        if pos is None or pos == self._scope.state().objective_position:
            return
        self._scope.set_objective(int(pos))
        self._render(force=True)

    def _on_light_path_selected(self, _index: int) -> None:
        if self._suppress or not self._connected():
            return
        pos = self._light_combo.currentData()
        if pos is None or pos == self._scope.state().light_path_position:
            return
        self._scope.set_light_path(int(pos))
        self._render(force=True)

    def _on_shutter_clicked(self) -> None:
        if self._suppress or not self._connected():
            return
        self._scope.set_epi_shutter(bool(self._shutter_btn.isChecked()))
        self._render(force=True)

    def _on_lamp_clicked(self) -> None:
        if self._suppress or not self._connected():
            return
        self._scope.set_dia_lamp_on(bool(self._lamp_btn.isChecked()))
        self._render(force=True)

    def _on_lamp_remote_clicked(self) -> None:
        if self._suppress or not self._connected():
            return
        self._scope.set_dia_lamp_remote(bool(self._lamp_remote_btn.isChecked()))
        self._render(force=True)

    def _on_lamp_level(self) -> None:
        if self._suppress or not self._connected():
            return
        self._scope.set_dia_lamp_intensity(float(self._lamp_spin.value()))
        self._render(force=True)

    def _jog_focus(self, direction: int) -> None:
        if not self._connected():
            return
        step = float(self._step_spin.value())
        # The store owns the per-body direction convention, so a scope that
        # counts the other way is a checkbox, not a code change.
        if not self._store.focus_up_is_positive():
            direction = -direction
        self._scope.move_focus_um(direction * step)
        self._render(force=True)

    def _goto_focus(self) -> None:
        if not self._connected():
            return
        self._scope.set_focus_um(float(self._goto_spin.value()))
        self._render(force=True)

    # ── Rendering ──────────────────────────────────────────────────

    @staticmethod
    def _popup_open(combo: QComboBox) -> bool:
        """Is this combo's drop-down list currently showing?"""
        try:
            view = combo.view()
            return bool(view is not None and view.isVisible())
        except Exception:
            return False

    def _any_popup_open(self) -> bool:
        return (self._popup_open(self._filter_combo)
                or self._popup_open(self._objective_combo)
                or self._popup_open(self._light_combo))

    @staticmethod
    def _set_row_visible(layout, visible: bool) -> None:
        """Show/hide every widget in a row layout."""
        for i in range(layout.count()):
            item = layout.itemAt(i)
            widget = item.widget() if item is not None else None
            if widget is not None:
                widget.setVisible(bool(visible))

    def _tick(self) -> None:
        # Don't poll the body while the operator has a drop-down open: the
        # refresh sets busy=True, which used to disable the combo mid-selection
        # and snap the list shut under their cursor.
        if self._any_popup_open():
            return
        state = self._scope.state()
        now = time.monotonic()
        if (state.connected and not state.busy
                and now - self._last_refresh >= _REFRESH_INTERVAL_S):
            self._last_refresh = now
            self._scope.refresh()
        self._render()

    def _render(self, *, force: bool = False) -> None:
        state = self._scope.state()
        key = (state.connected, state.busy, state.backend,
               state.filter_position, state.objective_position,
               state.focus_um, state.error,
               state.filter_count, state.objective_count,
               state.epi_shutter_present, state.epi_shutter_open,
               state.dia_lamp_present, state.dia_lamp_on,
               state.dia_lamp_remote,
               state.dia_lamp_intensity, state.dia_lamp_min, state.dia_lamp_max,
               state.light_path_position, state.light_path_count)
        if not force and key == self._rendered_key:
            return
        self._rendered_key = key

        connected = bool(state.connected)
        # Status line.
        if connected:
            dot, colour = "●", COLORS["green"]
            text = state.backend.replace("_", " ")
            if state.busy:
                text += " · moving…"
        else:
            dot, colour = "○", COLORS["subtext0"]
            text = "not connected"
        self._status_lbl.setText(f"{dot} {text}")
        self._status_lbl.setStyleSheet(f"color: {colour};")
        self._btn_connect.setText("Disconnect" if connected else "Connect")

        self._error_lbl.setVisible(bool(state.error))
        if state.error:
            self._error_lbl.setText(state.error)

        # Enablement — everything but Connect/Setup needs a live body.
        has_filter = connected and state.filter_count > 0
        has_objective = connected and state.objective_count > 0
        has_focus = connected and state.focus_um is not None
        # Never disable or re-index a combo whose list is open — doing so
        # closes the drop-down out from under the operator's click.
        if not self._popup_open(self._filter_combo):
            self._filter_combo.setEnabled(has_filter and not state.busy)
        if not self._popup_open(self._objective_combo):
            self._objective_combo.setEnabled(has_objective and not state.busy)
        # Focus jog stays live DURING a move. Repeated small steps are the most
        # common microscope interaction, and queued deltas are additive and each
        # bounded by the step size — so dropping the operator's click (which is
        # what busy-gating these two buttons did) is worse than queueing it.
        # Absolute go-to and the turrets stay gated: those are not additive, and
        # re-entering a selection mid-rotation is confusing.
        self._btn_up.setEnabled(has_focus)
        self._btn_down.setEnabled(has_focus)
        self._btn_goto.setEnabled(has_focus and not state.busy)
        self._goto_spin.setEnabled(has_focus and not state.busy)

        # Grow the combos if the hardware reports more positions than the
        # configured slot count (the body is the authority on how many exist).
        # Rebuilding clears the list, so never do it with a popup open.
        if (not self._any_popup_open()
                and (state.filter_count > self._filter_combo.count()
                     or state.objective_count > self._objective_combo.count())):
            self._rebuild_slot_combos()

        self._suppress = True
        try:
            if (state.filter_position is not None
                    and not self._popup_open(self._filter_combo)):
                idx = self._filter_combo.findData(int(state.filter_position))
                if idx >= 0:
                    self._filter_combo.setCurrentIndex(idx)
            if (state.objective_position is not None
                    and not self._popup_open(self._objective_combo)):
                idx = self._objective_combo.findData(
                    int(state.objective_position))
                if idx >= 0:
                    self._objective_combo.setCurrentIndex(idx)
        finally:
            self._suppress = False

        if state.focus_um is None:
            self._focus_lbl.setText("—")
        else:
            self._focus_lbl.setText(f"{state.focus_um:,.2f} µm")

        self._render_illumination(state, connected)

    def _render_illumination(self, state, connected: bool) -> None:
        """Shutter / lamp / light-path rows (v7.17).

        Each row is hidden outright when the body does not have that device —
        a permanently-dead control reads as broken software, not as an accessory
        that was never fitted.
        """
        # -- epi shutter --
        self._set_row_visible(self._illum_row, bool(state.epi_shutter_present))
        if state.epi_shutter_present:
            is_open = state.epi_shutter_open
            self._suppress = True
            try:
                self._shutter_btn.setChecked(bool(is_open))
            finally:
                self._suppress = False
            # "unknown" is shown as unknown. Rendering it as "closed" would tell
            # the operator the sample is dark when we do not actually know.
            word = "—" if is_open is None else ("open" if is_open else "closed")
            self._shutter_btn.setText(f"Excitation {word}")
            self._shutter_btn.setEnabled(connected and not state.busy)

        # -- dia lamp --
        self._set_row_visible(self._lamp_row, bool(state.dia_lamp_present))
        if state.dia_lamp_present:
            on = state.dia_lamp_on
            self._suppress = True
            try:
                self._lamp_btn.setChecked(bool(on))
                lo, hi = state.dia_lamp_min, state.dia_lamp_max
                if lo is not None and hi is not None and hi > lo:
                    self._lamp_spin.setRange(float(lo), float(hi))
                # Don't overwrite a level the operator is part-way through
                # typing; the ~1 s poll would otherwise fight the keyboard.
                if (state.dia_lamp_intensity is not None
                        and not self._lamp_spin.hasFocus()):
                    self._lamp_spin.setValue(float(state.dia_lamp_intensity))
            finally:
                self._suppress = False
            self._lamp_btn.setText(
                "Dia lamp —" if on is None else
                ("Dia lamp on" if on else "Dia lamp off"))
            live = connected and not state.busy
            # In MainMode the SDK refuses every write, so an enabled control
            # would be a button that reliably produces an error. `None` means the
            # driver cannot tell, and is NOT treated as a refusal.
            remote = state.dia_lamp_remote
            writable = live and remote is not False
            self._lamp_remote_btn.setEnabled(live and remote is not None)
            self._lamp_remote_btn.setText("Remote" if remote else "Man")
            self._lamp_btn.setEnabled(writable)
            self._lamp_spin.setEnabled(writable and state.dia_lamp_min is not None)
            hint = ("" if remote is not False else
                    "  —  the microscope's front panel owns the lamp; "
                    "press Remote to take software control")
            self._lamp_btn.setToolTip(
                "Transmitted-light (brightfield) lamp on/off." + hint)

        # -- light path --
        has_path = bool(state.light_path_count)
        self._path_caption.setVisible(has_path)
        self._light_combo.setVisible(has_path)
        if has_path and not self._popup_open(self._light_combo):
            self._light_combo.setEnabled(connected and not state.busy)
            if state.light_path_count != self._light_combo.count():
                self._suppress = True
                try:
                    self._fill_light_combo(state)
                finally:
                    self._suppress = False
            if state.light_path_position is not None:
                idx = self._light_combo.findData(int(state.light_path_position))
                if idx >= 0:
                    self._suppress = True
                    try:
                        self._light_combo.setCurrentIndex(idx)
                    finally:
                        self._suppress = False
