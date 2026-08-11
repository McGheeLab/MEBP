"""
lablink_bridge.py — marshal LabLink service notifications onto the GUI thread.

v7.17. :class:`SupportClasses.LabLinkService.LabLinkService` is deliberately
Qt-free and notifies its listeners **from its worker thread**. Touching a widget
from there is undefined behaviour, so this object sits between them: it
subscribes as a plain callable and re-emits a Qt Signal, which Qt delivers on
the GUI thread because the connection crosses a thread boundary.

Two details are the point of the class rather than incidental:

* **The signal carries no payload.** The listener fires from the worker while
  the worker still holds the job it is mutating; handing that object across
  would be a live reference to shared state. The page re-reads
  ``service.snapshot()`` instead, which takes the lock and returns a deep copy.

* **It unsubscribes on destruction.** The service outlives every page (it is a
  process-wide singleton), so a bridge that stayed subscribed would leave the
  worker calling into a deleted Qt object — the exact hazard
  ``MicroscopeControl``'s polling design was chosen to avoid.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import QObject, Signal

logger = logging.getLogger(__name__)


class LabLinkBridge(QObject):
    """Re-emits a service notification as a GUI-thread Signal."""

    #: Something changed — re-read ``snapshot()``. No payload, on purpose.
    changed = Signal()

    def __init__(self, service, parent: QObject | None = None):
        super().__init__(parent)
        self._service = service
        self._subscribed = False
        self.subscribe()

    def subscribe(self) -> None:
        if self._subscribed or self._service is None:
            return
        try:
            self._service.add_listener(self._on_service_event)
            self._subscribed = True
        except Exception:
            logger.debug("LabLink bridge could not subscribe", exc_info=True)

    def unsubscribe(self) -> None:
        if not self._subscribed or self._service is None:
            return
        try:
            self._service.remove_listener(self._on_service_event)
        except Exception:
            logger.debug("LabLink bridge could not unsubscribe", exc_info=True)
        self._subscribed = False

    # Worker thread → GUI thread.
    def _on_service_event(self, _service) -> None:
        try:
            self.changed.emit()
        except RuntimeError:
            # The C++ side is already gone (page torn down mid-notify). Drop
            # the subscription rather than raising into the worker's loop.
            self._subscribed = False

    def __del__(self):  # pragma: no cover - interpreter teardown order
        try:
            self.unsubscribe()
        except Exception:
            pass
