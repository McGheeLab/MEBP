"""
optics_ensure.py — run one ``OpticsService.ensure_*`` from the GUI thread.

v7.19. ``SupportClasses/OpticsService.py`` has existed since v7.18, fully
specified and tested, with **zero production callers**. Its own module docstring
says why: it blocks on ``op.done.wait``, so it is worker-thread-only, and *"a GUI
caller must hand it to one (gui/widgets/optics_ensure.py)"* — the file this is.
Without it every GUI surface that wants to move a turret has to rediscover the
five things the service already gets right (check ``op.error`` and not just
``op.done``, abort rather than retry a stale drop, hold the lease, verify by
read-back, restore on every exit).

WHAT THIS ADDS ON TOP OF THE SERVICE
------------------------------------
Only the thread hop and the reporting. A daemon thread runs the blocking call;
the result comes back through a queued signal so the callback runs on the GUI
thread and may safely touch widgets. Nothing here decides optics policy — that
all lives in ``OpticsService`` / ``OpticsRegistry``.

``simulated`` is passed through deliberately: a simulated switch reported as a
real one is a fabricated fact.
"""

from __future__ import annotations

import logging
import threading
from typing import Callable, Optional

from PySide6.QtCore import QObject, Signal

logger = logging.getLogger(__name__)

#: Lease identity for a switch the operator asked for by hand. Distinct from a
#: workflow's own owner string so a manual switch cannot silently re-enter (and
#: then release) a running survey's lease.
MANUAL_OWNER = "manual_optics"

#: Process-wide floor between hardware re-reads. Matches
#: ``microscope_panel._REFRESH_INTERVAL_S``, the surface that has been doing
#: this correctly since v7.5.x.
_REFRESH_MIN_S = 1.0
_last_refresh = [0.0]
_refresh_lock = threading.Lock()


def request_state_refresh(min_interval_s: float = _REFRESH_MIN_S) -> bool:
    """Ask the body for a fresh read. Returns True if one was queued.

    🐞 v7.19.1 — **``MicroscopeController`` does not poll itself.**
    ``state()`` returns a cached snapshot updated only by ``_read_all()`` after
    an op, or by an explicit ``refresh()``; its worker loop waits on a command
    queue and reads nothing on its own. Repo-wide, ``refresh()`` had exactly TWO
    callers: ``microscope_panel._tick`` (only while that panel is on screen) and
    ``OpticsService._ensure`` (only around a switch we command).

    The consequence is not obvious and is why this exists: a surface can poll
    ``state()`` every second, forever, and never notice a turret the operator
    turned BY HAND — because nothing on its page ever re-reads the hardware. An
    app-driven change propagates (``set_objective`` calls ``_read_all``), so the
    staleness is invisible in exactly the case most likely to be tested.

    THREE THINGS THIS REFUSES TO DO, each a real cost:

    * **Never while the body is busy or leased.** A refresh is a queued op; the
      lease exists so a scan or the plate-level wizard owns the body outright,
      and ``OpticsService`` already refreshes around its own switches. Polling
      underneath it would put ops on the queue mid-survey — and
      ``STALE_OP_S`` drops a queued op silently, so the one it displaces could
      be the one that mattered.
    * **Never faster than ``min_interval_s``, process-wide.** Several panels can
      be alive at once (jog card + a workflow panel), and each one having its own
      throttle multiplies the hardware reads by the number of visible surfaces.
    * **Never blocks.** ``refresh()`` returns an op handle; this deliberately
      does not wait on it. The caller reads the cache on its NEXT tick, which is
      the whole reason the callers are timers.
    """
    try:
        from SupportClasses.MicroscopeControl import get_microscope
        scope = get_microscope()
        state = scope.state()
        if not getattr(state, "connected", False):
            return False
        if getattr(state, "busy", False):
            return False
        if getattr(scope, "lease_owner", lambda: None)():
            return False
    except Exception:
        return False
    import time
    now = time.monotonic()
    with _refresh_lock:
        if now - _last_refresh[0] < max(0.0, float(min_interval_s)):
            return False
        _last_refresh[0] = now
    try:
        scope.refresh()
    except Exception as exc:
        logger.debug("optics refresh failed: %s", exc)
        return False
    return True


def build_service(owner: str, *, controller=None, config_store=None,
                  objective_store=None):
    """An ``OpticsService`` wired to the process-wide stores, or None.

    Returns None — rather than raising — when the microscope module cannot be
    imported at all, which is the ordinary state of a rig with no SDK
    installed. Callers treat that as "not drivable".
    """
    try:
        from SupportClasses.OpticsService import OpticsService
        if controller is None:
            from SupportClasses.MicroscopeControl import get_microscope
            controller = get_microscope()
        if config_store is None:
            from SupportClasses.MicroscopeConfigStore import get_store as _cfg
            config_store = _cfg()
        if objective_store is None:
            try:
                from SupportClasses.ObjectiveCalibration import (
                    get_store as _obj)
                objective_store = _obj()
            except Exception:
                objective_store = None
        return OpticsService(controller, config_store, objective_store,
                             owner=str(owner))
    except Exception as exc:
        logger.debug("optics service unavailable: %s", exc)
        return None


class _Bridge(QObject):
    """Marshals the worker's result onto the GUI thread."""

    done = Signal(object)


def ensure_optics_async(*, kind: str, name: str, owner: str = MANUAL_OWNER,
                        on_done: Optional[Callable] = None,
                        service=None, **kwargs) -> bool:
    """Put optic ``name`` of ``kind`` in the light path, off the GUI thread.

    ``on_done`` is called on the GUI thread with the ``EnsureResult`` — or with
    None when no microscope could be reached at all, which callers must treat
    as "not drivable" rather than as a failure to switch. Returns False if it
    could not even be started.

    ``kwargs`` are forwarded verbatim to ``ensure_filter`` / ``ensure_objective``
    (``camera_identity``, ``glass_focus_um``, ``needle_retracted``,
    ``apply_parfocal``, ``cancelled``), so this adds no policy of its own.
    """
    if service is None:
        service = build_service(owner)
    if service is None:
        if on_done is not None:
            on_done(None)
        return False

    bridge = _Bridge()
    if on_done is not None:
        # Queued by default across threads: the callback lands on the GUI
        # thread and may touch widgets.
        bridge.done.connect(lambda res: on_done(res))

    def _run():
        try:
            if kind == "filter":
                res = service.ensure_filter(name, **kwargs)
            else:
                res = service.ensure_objective(name, **kwargs)
        except Exception as exc:                       # never cross a thread
            logger.debug("ensure %s %r failed: %s", kind, name, exc)
            res = None
        try:
            bridge.done.emit(res)
        except Exception:
            pass

    t = threading.Thread(target=_run, name="OpticsEnsure", daemon=True)
    # Keep the bridge alive for the thread's lifetime — a QObject that goes out
    # of scope before the emit drops the result silently.
    t._bridge = bridge                                  # type: ignore[attr-defined]
    t.start()
    return True


def describe_refusal(result) -> str:
    """One operator-facing sentence for a refused or degraded switch."""
    if result is None:
        return ("no microscope is connected, so the optics could not be "
                "changed automatically")
    why = getattr(result, "why_not", "") or ""
    if getattr(result, "degraded", False) and not why:
        return "this microscope cannot drive that turret"
    return why or "the switch could not be verified"
