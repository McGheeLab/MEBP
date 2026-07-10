"""SafeTravelWorker — run ``StageController.safe_travel_to`` off the GUI thread.

The retract-before-XY safe-travel sequence (raise Z → M400 + ``wait_for_z_arrival``
→ fast XY → ``wait_for_xy_arrival`` → optional descent) is a BLOCKING, multi-second
serial operation. Calling it directly from a Qt signal handler (a click on the
workspace, an "absolute Go To") runs it on the GUI thread and freezes the Qt
event loop for the whole duration.

When the needle is already retracted, step 1 is a near-no-op, so the block is
sub-second and imperceptible. But when the needle is DOWN in a well, step 1 must
perform a real Z retract + confirmation — 11-14 s on ME3B V1, and up to ~60-120 s
(or, on a stuck/crawling axis, effectively forever) — during which the whole
application appears frozen ("Not Responding"). That was the "program freezes when
I jog to another well with the needle down, but not if I hit Move-to-Safe-Z
first" bug: the Move-to-Safe-Z button is a fire-and-forget ``move_z_absolute``, so
after it the click's step-1 retract is the fast no-op.

The workflow-execution paths (Quick Print pre-position, the pick-&-place
executors) already run their blocking moves on a daemon thread with a Qt Signal
"bridge"; this helper packages that pattern for the click-to-travel handlers so
they stop freezing the GUI. A busy-guard means rapid re-clicks are IGNORED (not
queued behind each other on the shared serial channel).

Usage::

    self._travel = SafeTravelWorker(self)
    self._travel.finished.connect(self._on_travel_finished)   # ok: bool
    ...
    if self._travel.start(controller, stage_x, stage_y,
                          safe_z_mm=safe_z, target_z_mm=None):
        self._set_travelling(True)   # visual "busy" cue; else the click was ignored

``finished`` is emitted from the worker thread; because the worker QObject lives
on the GUI thread the connection is delivered as a queued call, so slots run on
the GUI thread — safe to touch widgets there. NEVER touch Qt widgets from inside
``safe_travel_to`` itself; it only drives the controller/serial.
"""

from __future__ import annotations

import logging
import threading

from PySide6.QtCore import QObject, Signal

logger = logging.getLogger(__name__)


class SafeTravelWorker(QObject):
    """Runs ``controller.safe_travel_to(...)`` on a daemon thread, marshalling the
    boolean result back to the GUI thread via :attr:`finished`.

    A single travel may be in flight at a time; :meth:`start` returns ``False``
    (and does nothing) while one is running so re-clicks don't queue.
    """

    #: Emitted (on the GUI thread) when a travel finishes; ``True`` if
    #: ``safe_travel_to`` returned truthy, ``False`` on timeout / abort / error.
    finished = Signal(bool)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._thread: threading.Thread | None = None
        self._busy = False
        # Clear the busy flag on the GUI thread (queued) so a re-click during the
        # emit→slot window is still refused. Connected FIRST so it runs before any
        # page-level ``finished`` slot that inspects ``busy``.
        self.finished.connect(self._clear_busy)

    @property
    def busy(self) -> bool:
        """True while a travel is in flight."""
        return self._busy

    def start(self, controller, *args, **kwargs) -> bool:
        """Launch ``controller.safe_travel_to(*args, **kwargs)`` on a daemon thread.

        Returns ``True`` if the travel was started, ``False`` if one is already in
        flight (the request is ignored — nothing is queued) or the controller is
        missing.
        """
        if self._busy:
            logger.debug("SafeTravelWorker: a travel is already in flight — "
                         "ignoring the new request")
            return False
        if controller is None or not hasattr(controller, "safe_travel_to"):
            return False

        self._busy = True

        def _worker() -> None:
            ok = False
            try:
                ok = bool(controller.safe_travel_to(*args, **kwargs))
            except Exception:
                logger.exception("SafeTravelWorker: safe_travel_to raised")
                ok = False
            finally:
                # Cross-thread: delivered as a queued call to GUI-thread slots.
                self.finished.emit(ok)

        self._thread = threading.Thread(
            target=_worker, name="SafeTravel", daemon=True)
        self._thread.start()
        return True

    def _clear_busy(self, _ok: bool) -> None:
        self._busy = False
