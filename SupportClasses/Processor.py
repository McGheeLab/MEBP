"""
Processor — Thread-safe command bus with pub/sub dispatch.

All device commands flow through a single Processor instance, ensuring
serialised execution and preventing race conditions between the GUI,
Xbox controller, and print engine.

Usage:
    proc = Processor()
    proc.register_handler("move_xy", my_handler)
    proc.add_command("move_xy", x=100, y=200)
    ...
    proc.stop()
"""

from __future__ import annotations

import logging
import queue
import threading
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class Processor:
    """
    Single-threaded command queue with subscriber dispatch.

    Commands are enqueued via :meth:`add_command` (thread-safe) and
    dispatched in FIFO order on a dedicated daemon thread.  Multiple
    handlers can be registered for the same command name.
    """

    def __init__(self, name: str = "Processor"):
        self._name = name
        self._queue: queue.Queue = queue.Queue()
        self._subscribers: dict[str, list[Callable]] = {}
        self._lock = threading.Lock()  # protects _subscribers
        self._running = True
        self._thread = threading.Thread(
            target=self._process_loop,
            name=f"{name}Thread",
            daemon=True,
        )
        self._thread.start()
        logger.debug(f"[{self._name}] Started")

    # ── Handler Registration ──────────────────────────────────────

    def register_handler(self, command_name: str, handler: Callable) -> None:
        """Register *handler* to be called when *command_name* is dispatched."""
        with self._lock:
            self._subscribers.setdefault(command_name, []).append(handler)
        logger.debug(f"[{self._name}] Registered handler for '{command_name}'")

    def unregister_handler(self, command_name: str, handler: Callable) -> None:
        """Remove a previously registered handler."""
        with self._lock:
            handlers = self._subscribers.get(command_name, [])
            try:
                handlers.remove(handler)
                if not handlers:
                    del self._subscribers[command_name]
            except ValueError:
                logger.warning(
                    f"[{self._name}] Handler not found for '{command_name}'"
                )

    def has_handler(self, command_name: str) -> bool:
        """Return True if at least one handler is registered for *command_name*."""
        with self._lock:
            return bool(self._subscribers.get(command_name))

    @property
    def registered_commands(self) -> list[str]:
        """List all command names with registered handlers."""
        with self._lock:
            return list(self._subscribers.keys())

    # ── Command Submission ────────────────────────────────────────

    def add_command(self, command_name: str, *args, **kwargs) -> None:
        """
        Enqueue a command for asynchronous dispatch.

        Thread-safe — may be called from any thread or process bridge.
        """
        self._queue.put((command_name, args, kwargs))

    @property
    def pending_count(self) -> int:
        """Approximate number of commands waiting to be processed."""
        return self._queue.qsize()

    # ── Lifecycle ─────────────────────────────────────────────────

    def stop(self, timeout: float = 2.0) -> None:
        """Signal the processor to stop and wait for the thread to finish."""
        if not self._running:
            return
        self._running = False
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            logger.warning(f"[{self._name}] Thread did not exit cleanly")
        else:
            logger.debug(f"[{self._name}] Stopped")

    @property
    def is_running(self) -> bool:
        return self._running and self._thread.is_alive()

    # ── Internal ──────────────────────────────────────────────────

    def _process_loop(self) -> None:
        """Drain the queue and dispatch commands to subscribers."""
        while self._running:
            try:
                command_name, args, kwargs = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue

            with self._lock:
                handlers = list(self._subscribers.get(command_name, []))

            if not handlers:
                logger.debug(
                    f"[{self._name}] No handler for '{command_name}'"
                )
                self._queue.task_done()
                continue

            for handler in handlers:
                try:
                    handler(*args, **kwargs)
                except Exception:
                    logger.exception(
                        f"[{self._name}] Error in handler {handler.__qualname__} "
                        f"for '{command_name}'"
                    )

            self._queue.task_done()

    def __repr__(self) -> str:
        with self._lock:
            n = sum(len(v) for v in self._subscribers.values())
        return f"Processor(name={self._name!r}, handlers={n}, pending={self.pending_count})"
