"""
service.py — the shared IncubatorController singleton.

The same pattern as ``MicroscopeControl.get_microscope()`` /
``LabLinkService.get_service()``: several GUI surfaces (the Incubator page,
the Connect Hardware card's Incubator row, the Hardware Setup tab's status
line) must all drive — and observe — the same physical heaters over one
session, so the controller is a module singleton with a ``peek_*`` accessor
that never constructs (publishers and the stage-connect port-exclusion logic
no-op when the incubator has never been touched).
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

from .config_store import get_store
from .controller import IncubatorController

logger = logging.getLogger(__name__)

_controller: Optional[IncubatorController] = None
_lock = threading.Lock()


def apply_store_config(ctrl: IncubatorController) -> None:
    """Push the per-machine store's preferences onto a controller.

    Idempotent and safe on a live controller — everything here is a host-side
    preference (ceiling, display labels), never a command to the board.
    """
    store = get_store()
    try:
        ctrl.set_max_setpoint_c(store.get("max_setpoint_c"))
    except Exception:
        logger.debug("could not apply incubator ceiling", exc_info=True)
    try:
        ctrl.set_zone_labels(store.zone_labels())
    except Exception:
        logger.debug("could not apply incubator zone labels", exc_info=True)


def get_incubator() -> IncubatorController:
    """The shared controller, created on first use."""
    global _controller
    with _lock:
        if _controller is None:
            _controller = IncubatorController()
            apply_store_config(_controller)
        return _controller


# ── StageController plumbing (ONE home for these rules) ──────────
#
# Both the Incubator page and the Connect Hardware card need the same three
# callables; two independent copies would be the "two homes for one fact"
# trap, with the symptom being a scan that resets a board from one surface
# but not the other.

def make_zp_getter(stage_controller):
    """() -> the CURRENT ZPStageManager (or None). Resolved per call —
    the controller replaces the manager object on every reconnect."""
    def _get():
        return getattr(stage_controller, "zp_stage", None) \
            if stage_controller is not None else None
    return _get


def make_poll_gate(stage_controller):
    """() -> False while the position poller is suspended (a PRINT_PATH
    burst owns the ZP channel) so the shared link skips its M105 round."""
    def _gate():
        poller = getattr(stage_controller, "_pos_poller", None) \
            if stage_controller is not None else None
        return not bool(getattr(poller, "_suspended", False))
    return _gate


def make_exclusion_provider(stage_controller):
    """() -> ports the app's stages own (never opened, never offered)."""
    def _ports():
        out = []
        if stage_controller is not None:
            for p in (getattr(stage_controller, "zp_connected_port", None),
                      getattr(stage_controller, "_preferred_zp_port", None)):
                if p:
                    out.append(p)
            xy = getattr(stage_controller, "xy_stage", None)
            p = getattr(getattr(xy, "spo", None), "port", None)
            if p:
                out.append(p)
        return list(dict.fromkeys(out))
    return _ports


def connect_from_store(*, stage_controller=None,
                       transport_override: str | None = None) -> bool:
    """Open the incubator with the SAVED transport + settings.

    BLOCKING for the duration of the firmware probe — call from a worker
    thread, never the GUI thread. Used by the Connect Hardware card's
    Incubator row; the Incubator page performs the same dispatch with its
    own (possibly just-edited) values.
    """
    ctrl = get_incubator()
    if ctrl.connected:
        return True
    store = get_store()
    ctrl.exclude_ports_provider = make_exclusion_provider(stage_controller)
    apply_store_config(ctrl)
    transport = transport_override or store.get("transport", "shared")
    if transport == "simulate":
        return ctrl.connect(
            simulate=True,
            sim_time_scale=float(store.get("sim_time_scale", 300)))
    if transport == "serial":
        return ctrl.connect(
            str(store.get("dedicated_port", "") or ""),
            int(store.get("dedicated_baud", 38400)))
    return ctrl.connect_shared(
        make_zp_getter(stage_controller),
        poll_gate=make_poll_gate(stage_controller))


def peek_incubator() -> Optional[IncubatorController]:
    """The shared controller IF it exists — never constructs one."""
    return _controller


def shutdown_incubator(*, heaters_off: bool = True) -> None:
    """Best-effort teardown at app close.

    ``heaters_off`` mirrors the operator's answer to the close prompt (or the
    store default when there was nothing to ask). NOTE this is a courtesy,
    not a safety mechanism: the firmware owns the control loop and keeps
    holding its last setpoint if the link is already gone.
    """
    global _controller
    with _lock:
        ctrl = _controller
        _controller = None
    if ctrl is None:
        return
    try:
        ctrl.disconnect(heaters_off=heaters_off)
    except Exception:
        logger.debug("incubator shutdown raised", exc_info=True)


def reset_service() -> None:
    """Drop the singleton WITHOUT touching hardware (test isolation)."""
    global _controller
    with _lock:
        _controller = None
