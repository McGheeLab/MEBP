"""context_sections.py — catalog of cards for the Custom context panel.

v7.5.x: The left context panel gains a **Custom** view the operator composes
from *sections* (cards). This module owns:

  * ``SectionContext`` — the shared handles a section needs (controller,
    hardware config, camera manager, settings, layout store).
  * ``SECTION_REGISTRY`` + ``register_section`` / ``catalog`` / ``build_section``
    — a tiny factory mapping a section *type* string to a builder. Adding a new
    section type is a single ``register_section(...)`` call; it then appears in
    the ``+ Add section`` menu and is persisted by ``ContextPanelLayoutStore``.
  * The built-in section widgets: live camera, syringe overview, X/Y/Z/P1/P2/P3
    location read-outs, jog controls, hardware info.

Each built section is a plain ``QWidget`` that MAY expose ``on_status_update()``
and/or ``on_motion_tick()`` (duck-typed) so the panel can forward the
MainWindow's live ticks, and MAY expose ``set_hardware_config(cfg)`` so config
changes reach it. None are required — a static section simply omits them.

Data-feed logic (pump fills, position read-outs) mirrors the proven Jog-page /
Hardware-Control-Panel code paths (``jog_control._refresh_pump_panel`` and
``control_panel._update_position_displays``) so behaviour matches exactly.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf
from gui.styles import COLORS
from gui.widgets.components import Card
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.pump_rack import PumpRack
from gui.pages.hardware.control_panel import PositionBar

try:  # module-level height helper fallback for Z display
    from SupportClasses.StageController import z_raw_to_display
except Exception:  # pragma: no cover - defensive
    def z_raw_to_display(v):  # type: ignore
        return v

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:  # pragma: no cover - defensive
    CameraRole = None  # type: ignore

logger = logging.getLogger(__name__)


# ── Shared context handed to every section builder ─────────────────

@dataclass
class SectionContext:
    controller: object = None
    hardware_config: object = None
    camera_manager: object = None
    settings: object = None
    layout_store: object = None


@dataclass
class SectionSpec:
    label: str
    icon: str
    builder: Callable[["SectionContext", dict], QWidget]


SECTION_REGISTRY: dict[str, SectionSpec] = {}


def register_section(section_type: str, spec: SectionSpec) -> None:
    SECTION_REGISTRY[section_type] = spec


def catalog() -> list[tuple[str, str, str]]:
    """(type, label, icon) for every registered section, insertion order."""
    return [(t, sp.label, sp.icon) for t, sp in SECTION_REGISTRY.items()]


def known_types() -> set[str]:
    return set(SECTION_REGISTRY.keys())


def build_section(section_type: str, ctx: SectionContext,
                  options: dict | None = None) -> QWidget:
    """Build one section widget. Raises KeyError for an unknown type (the
    caller renders an error card)."""
    spec = SECTION_REGISTRY[section_type]
    return spec.builder(ctx, dict(options or {}))


# ── Live camera viewer ─────────────────────────────────────────────

class CameraSection(QWidget):
    """A live microscope feed. Ensures the shared camera is running while the
    card is visible; NEVER stops it on hide (the feed is shared — other pages
    may still need it, and a page that started it owns the stop)."""

    def __init__(self, ctx: SectionContext, options: dict):
        super().__init__()
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        cam_idx = self._resolve_cam_idx()
        self._view = CameraFeedView(
            ctx.camera_manager, cam_idx=cam_idx,
            show_crosshair=True, enable_settings=True)
        self._view.setMinimumHeight(s(200))
        lay.addWidget(self._view)

    def _resolve_cam_idx(self) -> int:
        hw = self._ctx.hardware_config
        if hw is not None and CameraRole is not None:
            try:
                idx = hw.camera_for_role(CameraRole.MICROSCOPE)
                if idx is not None:
                    return int(idx)
            except Exception:
                pass
        return 0

    def _ensure_running(self) -> None:
        mgr = self._ctx.camera_manager
        if mgr is None:
            return
        idx = self._resolve_cam_idx()
        try:
            if self._view.cam_idx != idx:
                self._view.set_camera(idx)
            if not mgr.is_running(idx):
                mgr.start(idx)
        except Exception as exc:
            logger.debug("CameraSection: start failed: %s", exc)

    def showEvent(self, event):
        self._ensure_running()
        super().showEvent(event)

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg
        # A config change may re-assign the microscope camera slot.
        if self.isVisible():
            self._ensure_running()


# ── Syringe overview ───────────────────────────────────────────────

class SyringeSection(QWidget):
    """P1/P2/P3 live plunger-fill read-out (µL when calibrated, raw mm else).

    Reproduces ``jog_control._refresh_pump_panel`` exactly."""

    def __init__(self, ctx: SectionContext, options: dict):
        super().__init__()
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self._rack = PumpRack()
        lay.addWidget(self._rack)

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg

    def on_status_update(self) -> None:
        ctrl = self._ctx.controller
        hw = self._ctx.hardware_config
        if ctrl is None or hw is None or not hasattr(hw, "pumps"):
            return
        try:
            zp = ctrl.get_zp_position(cached=True)
        except Exception:
            return
        have_zp = bool(zp) and zp[0] is not None
        fills: dict[str, dict | None] = {}
        for pid in ("P1", "P2", "P3"):
            pump_cfg = hw.pumps.get(pid)
            if pump_cfg is None or not getattr(pump_cfg, "is_configured", False):
                fills[pid] = None
                continue
            p_raw = ctrl.zp_logical_value(zp, pid) if have_zp else None
            calibrated = bool(
                hasattr(ctrl, "is_pump_plunger_calibrated")
                and ctrl.is_pump_plunger_calibrated(pid))
            fill_uL = capacity_uL = None
            if calibrated and p_raw is not None:
                try:
                    fill_uL = ctrl.raw_to_pump_fill_uL(pid, p_raw)
                    capacity_uL = ctrl.pump_capacity_uL(pid)
                except Exception:
                    fill_uL = capacity_uL = None
            ink_spec = getattr(
                getattr(pump_cfg, "fluid_column", None), "ink_spec", None)
            fills[pid] = {
                "fill_uL": fill_uL,
                "capacity_uL": capacity_uL,
                "raw_mm": p_raw,
                "syringe": pump_cfg.syringe,
                "ink_spec": ink_spec,
                "calibrated": calibrated and fill_uL is not None,
            }
        self._rack.update_live_fills(fills)

    # Pump fills glide with the plunger too — refresh on the fast tick.
    on_motion_tick = on_status_update


# ── Position read-outs (X/Y/Z/P1/P2/P3) ────────────────────────────

class PositionReadoutCard(QWidget):
    """Compact live position grid, reusing ``PositionBar``.

    Mirrors ``control_panel._update_position_displays`` / ``_refresh_bar_ranges``
    (Z shown in the unified user frame, pumps as µL fill once calibrated) but
    without the Connect / jog / speed machinery of the full control panel."""

    _AXES = [("X", "µm"), ("Y", "µm"), ("Z", "mm"),
             ("P1", "mm"), ("P2", "mm"), ("P3", "mm")]

    def __init__(self, ctx: SectionContext, options: dict):
        super().__init__()
        self._ctx = ctx
        self._ranges_applied = False
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        grid = QGridLayout()
        grid.setHorizontalSpacing(s(8))
        grid.setVerticalSpacing(s(6))
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)
        self._lbl: dict[str, QLabel] = {}
        self._bar: dict[str, PositionBar] = {}
        self._unit: dict[str, QLabel] = {}
        for r, (axis, unit) in enumerate(self._AXES):
            ax_lbl = QLabel(f"<b>{axis}</b>")
            ax_lbl.setStyleSheet(f"color: {COLORS['text']};")
            ax_lbl.setMinimumWidth(s(22))
            grid.addWidget(ax_lbl, r, 0)
            bar = PositionBar()
            grid.addWidget(bar, r, 1)
            self._bar[axis] = bar
            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-family: monospace;")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setMinimumWidth(s(70))
            grid.addWidget(val, r, 2)
            unit_lbl = QLabel(unit)
            unit_lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
            grid.addWidget(unit_lbl, r, 3)
            self._lbl[axis] = val
            self._unit[axis] = unit_lbl
        lay.addLayout(grid)

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg
        self._refresh_ranges()

    # ── Ranges ────────────────────────────────────────────────────

    def _refresh_ranges(self) -> None:
        s_obj = self._ctx.settings
        ctrl = self._ctx.controller
        if s_obj is None:
            return
        z_to_user = (ctrl.raw_to_user_z if ctrl is not None
                     and hasattr(ctrl, "raw_to_user_z") else z_raw_to_display)
        try:
            z_lo = z_to_user(s_obj.get("safety_limits.z_min", -10.0))
            z_hi = z_to_user(s_obj.get("safety_limits.z_max", 50.0))
            ranges = {
                "X": (s_obj.get("safety_limits.xy_min_x", -130000.0),
                      s_obj.get("safety_limits.xy_max_x", 130000.0)),
                "Y": (s_obj.get("safety_limits.xy_min_y", -85000.0),
                      s_obj.get("safety_limits.xy_max_y", 85000.0)),
                "Z": (min(z_lo, z_hi), max(z_lo, z_hi)),
                "P1": (s_obj.get("safety_limits.p1_min", -50.0),
                       s_obj.get("safety_limits.p1_max", 50.0)),
                "P2": (s_obj.get("safety_limits.p2_min", -50.0),
                       s_obj.get("safety_limits.p2_max", 50.0)),
                "P3": (s_obj.get("safety_limits.p3_min", -50.0),
                       s_obj.get("safety_limits.p3_max", 50.0)),
            }
        except Exception as exc:
            logger.debug("PositionReadoutCard: range read failed: %s", exc)
            return
        if ctrl is not None and hasattr(ctrl, "pump_capacity_uL"):
            for pump in ("P1", "P2", "P3"):
                try:
                    cap = ctrl.pump_capacity_uL(pump)
                except Exception:
                    cap = None
                if cap is not None and cap > 0:
                    ranges[pump] = (0.0, float(cap))
        for axis, (lo, hi) in ranges.items():
            try:
                self._bar[axis].set_range(float(lo), float(hi))
            except Exception:
                pass
        self._ranges_applied = True

    # ── Live values ───────────────────────────────────────────────

    def _display_xy(self):
        c = self._ctx.controller
        get = getattr(c, "get_display_xy_position", None) or (
            lambda: c.get_xy_position(cached=True))
        return get()

    def _display_zp(self):
        c = self._ctx.controller
        get = getattr(c, "get_display_zp_position", None) or (
            lambda: c.get_zp_position(cached=True))
        return get()

    def _set(self, axis: str, value, fmt: str, unit: str | None = None,
             tooltip: str | None = None) -> None:
        lbl = self._lbl[axis]
        lbl.setText(fmt.format(value) if value is not None else "—")
        if tooltip is not None:
            lbl.setToolTip(tooltip)
        if unit is not None:
            self._unit[axis].setText(unit)
        self._bar[axis].set_value(value)

    def on_status_update(self) -> None:
        c = self._ctx.controller
        if c is None:
            return
        if not self._ranges_applied:
            self._refresh_ranges()
        try:
            xy = self._display_xy()
            zp = self._display_zp()
        except Exception:
            return
        if xy and len(xy) >= 2:
            self._set("X", xy[0], "{:,.1f}")
            self._set("Y", xy[1], "{:,.1f}")
        if not zp:
            return
        for logical in ("Z", "P1", "P2", "P3"):
            try:
                v = c.zp_logical_value(zp, logical)
            except Exception:
                v = None
            if logical == "Z":
                if v is not None and hasattr(c, "raw_to_user_z"):
                    v = c.raw_to_user_z(v)
                self._set("Z", v, "{:.3f}", unit="mm")
                continue
            if v is not None and hasattr(c, "raw_to_pump_fill_uL"):
                try:
                    fill = c.raw_to_pump_fill_uL(logical, v)
                except Exception:
                    fill = None
                if fill is not None:
                    self._set(logical, fill, "{:.1f}", unit="µL",
                              tooltip=f"raw {v:.3f} mm")
                    continue
            self._set(logical, v, "{:.3f}", unit="mm")

    on_motion_tick = on_status_update


# ── Jog controls (reuse the full jog panel) ────────────────────────

class JogSection(QWidget):
    """A full jog panel as a card, reusing the proven
    ``StandardJogContextPanel``."""

    def __init__(self, ctx: SectionContext, options: dict):
        super().__init__()
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        # Imported lazily to avoid a heavy import when this section is unused.
        from gui.widgets.standard_jog_context import StandardJogContextPanel
        self._panel = StandardJogContextPanel(
            ctx.controller, settings=ctx.settings)
        if ctx.hardware_config is not None:
            try:
                self._panel.set_hardware_config(ctx.hardware_config)
            except Exception:
                pass
        lay.addWidget(self._panel)

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg
        try:
            self._panel.set_hardware_config(cfg)
        except Exception:
            pass

    def on_status_update(self) -> None:
        try:
            self._panel.on_status_update()
        except Exception:
            pass


# ── Hardware info (read-only quick reference) ──────────────────────

class HardwareInfoSection(QWidget):
    """Compact needle + syringe + plate summary from the hardware config."""

    def __init__(self, ctx: SectionContext, options: dict):
        super().__init__()
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(4))
        self._rows: dict[str, QLabel] = {}
        for key, label in (("needle", "Needle"), ("P1", "P1 syringe"),
                           ("P2", "P2 syringe"), ("P3", "P3 syringe"),
                           ("plate", "Plate")):
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(s(6))
            name = QLabel(label)
            name.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;"
                f"font-weight: 600;")
            name.setMinimumWidth(s(80))
            row.addWidget(name)
            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-size: {sf(9)}pt;"
                f"font-family: Consolas, Menlo, monospace;")
            val.setWordWrap(True)
            row.addWidget(val, stretch=1)
            self._rows[key] = val
            lay.addLayout(row)
        self._refresh()

    def set_hardware_config(self, cfg) -> None:
        self._ctx.hardware_config = cfg
        self._refresh()

    def _refresh(self) -> None:
        hw = self._ctx.hardware_config
        needle_text = "Not configured"
        if hw is not None and getattr(hw, "needle", None) is not None:
            n = hw.needle
            parts = [f"{n.gauge}G"] if getattr(n, "gauge", None) else []
            if getattr(n, "outer_diameter_um", None):
                parts.append(f"OD {n.outer_diameter_um:.0f} µm")
            if getattr(n, "inner_diameter_um", None):
                parts.append(f"ID {n.inner_diameter_um:.0f} µm")
            if getattr(n, "length_mm", None):
                parts.append(f"L {n.length_mm:.1f} mm")
            needle_text = " · ".join(parts) if parts else "Configured"
        self._rows["needle"].setText(needle_text)

        for pid in ("P1", "P2", "P3"):
            text = "—"
            pumps = getattr(hw, "pumps", None) if hw is not None else None
            if pumps and pid in pumps:
                pump_cfg = pumps[pid]
                if pump_cfg is not None and getattr(
                        pump_cfg, "is_configured", False):
                    parts = []
                    if pump_cfg.syringe is not None:
                        parts.append(f"{pump_cfg.syringe.volume_uL:g} µL")
                    inks = [getattr(i, "name", None)
                            for i in (getattr(pump_cfg, "inks", []) or [])]
                    inks = [i for i in inks if i]
                    if inks:
                        parts.append(", ".join(inks))
                    text = " · ".join(parts) if parts else "Configured"
                else:
                    text = "Not configured"
            self._rows[pid].setText(text)

        plate_text = "—"
        if hw is not None and getattr(hw, "plate_name", None):
            plate_text = str(hw.plate_name)
        elif hw is not None and getattr(hw, "plate_format", None):
            plate_text = f"{hw.plate_format}-well"
        self._rows["plate"].setText(plate_text)


# ── Register the built-in catalog ──────────────────────────────────

register_section("camera", SectionSpec("Live camera", "📷", CameraSection))
register_section("syringe", SectionSpec("Syringe overview", "💉", SyringeSection))
register_section("positions", SectionSpec(
    "Positions (X/Y/Z/P1-3)", "📍", PositionReadoutCard))
register_section("jog", SectionSpec("Jog controls", "🕹️", JogSection))
register_section("hardware_info", SectionSpec(
    "Hardware info", "ℹ️", HardwareInfoSection))
