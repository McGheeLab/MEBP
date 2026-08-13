"""
marlin_link.py — serial transport + Marlin ``ok`` handshake.

Owns a pyserial-shaped port object (real or :class:`~.fake_marlin.FakeMarlinLink`)
and two things on top of it:

  1. A READER thread that does nothing but ``readline()`` and hand every decoded
     line to a callback. Because it never blocks on anything else, temperature
     autoreports and PID-autotune progress keep flowing even while a command
     transaction is in flight.

  2. :meth:`MarlinLink.send_and_wait` — write a line, then block the CALLING
     thread until a terminal line arrives. The ok/busy/error/reset state machine
     mirrors the proven pattern in ``SupportClasses/ZPStage.py::_read_until_ok``
     (a ``busy:`` keep-alive pushes the deadline out; ``ok`` finishes; ``error``
     rejects; a reset banner means the board rebooted) but is generalised rather
     than copied, since that method is welded to the stage/pump class.

  3. :meth:`MarlinLink.send_priority` — an OUT-OF-BAND write that bypasses the
     queue and any in-flight transaction. This is what makes M112 / M108 / M410
     usable: Marlin's ``EMERGENCY_PARSER`` scans the input buffer for exactly
     those commands and acts on them without waiting for the queue to drain, so
     writing them immediately is both safe and the only thing that works.

IMPORTANT: ``send_and_wait`` must never be called from the GUI thread. The
controller owns a single worker thread for that.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field

from .marlin_gcode import LineKind, classify_line, is_terminal

logger = logging.getLogger(__name__)

#: Default seconds of silence before a command is considered unanswered.
DEFAULT_OK_TIMEOUT_S = 8.0

#: Absolute ceiling for long-running operations (PID autotune, M190 waits).
#: A pathological stream of keep-alive lines must not wedge the app forever.
DEFAULT_HARD_CEILING_S = 3600.0

#: Commands Marlin's EMERGENCY_PARSER handles out-of-band.
PRIORITY_COMMANDS = ("M112", "M108", "M410", "M876")


@dataclass
class Transaction:
    """Outcome of one :meth:`MarlinLink.send_and_wait` call."""

    command: str
    ok: bool = False
    timed_out: bool = False
    rejected: bool = False
    board_reset: bool = False
    lines: list[str] = field(default_factory=list)
    error_text: str = ""
    elapsed_s: float = 0.0

    @property
    def failed(self) -> bool:
        return not self.ok


class MarlinLink:
    """Serial transport with a synchronous Marlin ``ok`` handshake."""

    def __init__(
        self,
        port,
        *,
        on_line=None,
        on_write=None,
        on_disconnect=None,
        name: str = "marlin",
    ):
        """
        Args:
            port: an open pyserial ``Serial`` (or a fake exposing the same
                  ``readline``/``write``/``flush``/``is_open``/``close`` surface).
            on_line: called from the READER thread for every inbound line.
            on_write: called for every outbound line (console echo).
            on_disconnect: called once if the port dies under the reader.
        """
        self._port = port
        self._on_line = on_line
        self._on_write = on_write
        self._on_disconnect = on_disconnect
        self._name = name

        self._write_lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()

        # Lines relevant to the transaction currently in flight.
        self._txn_q: queue.Queue[tuple[LineKind, str]] = queue.Queue()
        self._txn_active = threading.Event()

        self._disconnected = False
        self._last_line_at = 0.0
        self._consumer_errors = 0

    # ── lifecycle ───────────────────────────────────────────────────

    @property
    def port(self):
        return self._port

    def is_open(self) -> bool:
        try:
            return bool(self._port is not None and getattr(self._port, "is_open", True))
        except Exception:
            return False

    def start(self) -> None:
        """Spawn the reader thread."""
        if self._reader is not None:
            return
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name=f"{self._name}-reader"
        )
        self._reader.start()

    def stop(self) -> None:
        """Stop the reader thread. Does NOT close the port (caller owns it)."""
        self._stop.set()
        r = self._reader
        self._reader = None
        if r is not None and r.is_alive():
            r.join(timeout=2.5)

    def close(self) -> None:
        """
        Stop reading and close the port.

        The write lock is acquired first so we never close the handle while a
        write is in flight — closing a USB-serial handle mid-operation is a hard
        crash on Windows (the lesson behind
        ``coding plans/Update plans/MEBP_v75x_ZP_CLOSE_DURING_READ_CRASH.md``).
        """
        self.stop()
        got = self._write_lock.acquire(timeout=DEFAULT_OK_TIMEOUT_S + 2.0)
        try:
            try:
                if self._port is not None:
                    self._port.close()
            except Exception:
                pass
        finally:
            if got:
                self._write_lock.release()

    # ── reader thread ───────────────────────────────────────────────

    def _read_loop(self) -> None:
        consecutive_errors = 0
        while not self._stop.is_set():
            try:
                raw = self._port.readline()
                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors >= 3:
                    logger.warning("[%s] read failed %dx: %s",
                                   self._name, consecutive_errors, e)
                    self._flag_disconnect()
                    return
                time.sleep(0.05)
                continue

            if not raw:
                continue

            try:
                text = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue
            if not text:
                continue

            self._last_line_at = time.monotonic()
            kind = classify_line(text)

            # Feed the in-flight transaction first so it can finish promptly.
            if self._txn_active.is_set():
                self._txn_q.put((kind, text))

            if self._on_line is not None:
                try:
                    self._on_line(text, kind)
                except Exception:
                    # The reader must survive a broken consumer, but a silent
                    # swallow hides real bugs (it once masked a TypeError in the
                    # fault path). Warn loudly for the first few, then fall back
                    # to debug so a persistently broken consumer cannot spam.
                    self._consumer_errors += 1
                    if self._consumer_errors <= 3:
                        logger.warning(
                            "[%s] on_line consumer raised (%d)", self._name,
                            self._consumer_errors, exc_info=True,
                        )
                    else:
                        logger.debug("[%s] on_line consumer raised", self._name,
                                     exc_info=True)

    def _flag_disconnect(self) -> None:
        if self._disconnected:
            return
        self._disconnected = True
        if self._on_disconnect is not None:
            try:
                self._on_disconnect()
            except Exception:
                pass

    # ── writing ─────────────────────────────────────────────────────

    def _write_line(self, command: str) -> bool:
        data = (command.strip() + "\n").encode("ascii", errors="replace")
        with self._write_lock:
            try:
                self._port.write(data)
                self._port.flush()
            except Exception as e:
                logger.warning("[%s] write failed: %s", self._name, e)
                self._flag_disconnect()
                return False
        if self._on_write is not None:
            try:
                self._on_write(command.strip())
            except Exception:
                pass
        return True

    def send_priority(self, command: str) -> bool:
        """
        Write immediately, bypassing the queue and any in-flight transaction,
        and do not wait for an ``ok``.

        Intended only for Marlin's emergency commands (M112 / M108 / M410).
        Those are parsed straight out of the serial input buffer when
        ``EMERGENCY_PARSER`` is enabled, so they take effect even while the
        board is busy in an M190 wait or a PID autotune. Without
        ``EMERGENCY_PARSER`` compiled in they queue normally and will be late.
        """
        head = command.strip().split()[0].upper() if command.strip() else ""
        if head not in PRIORITY_COMMANDS:
            logger.debug("[%s] send_priority used for non-emergency %r",
                         self._name, command)
        return self._write_line(command)

    def send_and_wait(
        self,
        command: str,
        *,
        timeout_s: float = DEFAULT_OK_TIMEOUT_S,
        hard_ceiling_s: float | None = None,
        collect: bool = True,
    ) -> Transaction:
        """
        Write ``command`` and block until Marlin answers with a terminal line.

        Deadline behaviour: ``timeout_s`` is a ROLLING deadline reset by any
        inbound line — a ``busy:`` keep-alive or a streamed temperature report
        proves the board is alive, so a slow-but-working operation is not killed.
        ``hard_ceiling_s`` is an absolute cap so that a board which streams
        forever without ever finishing cannot hang the caller indefinitely.

        Must be called from the controller's worker thread, never the GUI thread.
        """
        txn = Transaction(command=command.strip())
        started = time.monotonic()
        ceiling = (
            DEFAULT_HARD_CEILING_S if hard_ceiling_s is None else float(hard_ceiling_s)
        )

        # Drain anything stale, then arm collection before writing so we cannot
        # miss a fast reply.
        while True:
            try:
                self._txn_q.get_nowait()
            except queue.Empty:
                break
        self._txn_active.set()

        try:
            if not self._write_line(command):
                txn.error_text = "write failed (port closed?)"
                txn.elapsed_s = time.monotonic() - started
                return txn

            deadline = time.monotonic() + timeout_s
            while True:
                now = time.monotonic()
                if now - started > ceiling:
                    txn.timed_out = True
                    txn.error_text = (
                        f"exceeded hard ceiling of {ceiling:.0f}s "
                        f"(board kept talking but never finished)"
                    )
                    break

                remaining = min(deadline - now, ceiling - (now - started))
                if remaining <= 0:
                    txn.timed_out = True
                    txn.error_text = f"no reply within {timeout_s:.1f}s"
                    break

                try:
                    kind, text = self._txn_q.get(timeout=min(remaining, 0.25))
                except queue.Empty:
                    continue

                if collect:
                    txn.lines.append(text)

                # Any line at all proves liveness -> push the rolling deadline.
                deadline = time.monotonic() + timeout_s

                if kind is LineKind.OK:
                    txn.ok = True
                    break
                if kind is LineKind.ERROR:
                    txn.rejected = True
                    txn.error_text = text
                    break
                if kind is LineKind.RESET:
                    txn.board_reset = True
                    txn.error_text = f"board reset mid-command: {text}"
                    break
                if is_terminal(kind):  # future-proofing
                    break
        finally:
            self._txn_active.clear()
            txn.elapsed_s = time.monotonic() - started

        return txn
