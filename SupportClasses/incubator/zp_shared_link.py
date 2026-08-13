"""
zp_shared_link.py — incubator transport that RIDES the app's live ZP link.

Why this exists
---------------
On this rig the incubator heaters are wired to the SAME SKR Mini E3 V3 that
drives Z and the pumps (Zone A = the Marlin *bed* outputs HB/THB, Zone B = the
*hotend* HE0/THO). Only one process — and within it, only one owner — can hold
that COM port, and in the app that owner is ``ZPStageManager``. So instead of
opening a second connection (impossible) or asking the operator to disconnect
the stage (the standalone tool's workaround), this class implements the same
surface as :class:`~.marlin_link.MarlinLink` but routes every command through
``ZPStageManager.transact()``, which serializes the whole write→read-until-ok
round trip under the ZP ``_serial_lock``. Heater traffic and motion traffic
interleave safely because neither can ever see the other's bytes.

The rules that keep the MOTION side healthy (each learned the hard way
elsewhere in this repo):

* **No autoreport.** M155 pushes unsolicited temperature lines that would land
  in the middle of M114/M400 parsing — the "stolen ok" class of bug. Shared
  mode polls M105 through atomic transactions instead, and the controller
  sends ``M155 S0`` at session start in case a previous dedicated session left
  autoreport enabled.
* **No long-running transactions.** A PID autotune or an M190 board-side wait
  answers its ``ok`` minutes-to-hours later; holding ``_serial_lock`` that
  long starves the position poller into a false "ZP disconnected". Requested
  timeouts are clamped to :data:`MAX_SHARED_TIMEOUT_S`, and the controller
  refuses autotune / board-side waits in shared mode outright.
* **Polling yields to prints.** ``poll_gate`` (when provided) returns False
  while the position poller is suspended — i.e. during a PRINT_PATH burst —
  and the controller then skips the M105 rather than adding lock-contention
  jitter to print pacing. Marlin holds the setpoint on its own the whole time.

Emergency commands (M112/M108/M410) go through
``ZPStageManager.priority_write()`` — the bounded-lock raw-write path proven
by ``quickstop()`` — because Marlin's EMERGENCY_PARSER reads them straight out
of the serial buffer. NOTE in shared mode M112 halts the WHOLE board: heaters
AND motion. The UI says so before sending it.

``close()`` never touches the port: it belongs to the ZP stage, not to us.
"""

from __future__ import annotations

import logging
import threading
from typing import Callable, Optional

from .marlin_gcode import classify_line
from .marlin_link import Transaction

logger = logging.getLogger(__name__)

#: Hard clamp on any one shared transaction's rolling timeout. Generous for a
#: heater command (M104/M140/M105/M503 answer in milliseconds) yet far below
#: anything that could starve the ZP position poller into a false disconnect.
MAX_SHARED_TIMEOUT_S = 20.0

#: Commands routed out-of-band (Marlin EMERGENCY_PARSER set).
PRIORITY_COMMANDS = ("M112", "M108", "M410", "M876")


class ZPSharedLink:
    """MarlinLink-compatible transport over the live ``ZPStageManager``.

    Args:
        zp_getter: zero-arg callable returning the CURRENT ``ZPStageManager``
            (or ``None``). Resolved per call, never cached — the controller
            replaces the manager object on every reconnect.
        on_line: called for every inbound line, ``(text, LineKind)`` — same
            contract as ``MarlinLink``; fires on the calling (worker) thread.
        on_write: called for every outbound line (console echo).
        on_disconnect: called ONCE when the ZP link stops being usable.
        poll_gate: optional zero-arg callable; False = "not now" (a print owns
            the channel). Read by the controller's sampler via
            :meth:`poll_allowed`.
    """

    def __init__(
        self,
        zp_getter: Callable[[], object],
        *,
        on_line=None,
        on_write=None,
        on_disconnect=None,
        poll_gate: Optional[Callable[[], bool]] = None,
        name: str = "incubator-shared",
    ):
        self._zp_getter = zp_getter
        self._on_line = on_line
        self._on_write = on_write
        self._on_disconnect = on_disconnect
        self._poll_gate = poll_gate
        self._name = name
        self._down_flagged = False
        self._closed = False
        # send_and_wait is only ever called from the controller's single
        # worker thread, but a lock keeps that an implementation detail of the
        # caller rather than a correctness requirement here.
        self._lock = threading.Lock()

    # ── lifecycle (MarlinLink surface) ──────────────────────────────

    def start(self) -> None:
        """No reader thread — lines arrive from transactions."""

    def stop(self) -> None:
        """Nothing to stop."""

    def close(self) -> None:
        """Detach WITHOUT closing the port — the ZP stage owns it."""
        self._closed = True

    @property
    def port(self):  # parity with MarlinLink; there is no private port object
        return None

    def is_open(self) -> bool:
        return self._usable_zp() is not None

    # ── polling gate ────────────────────────────────────────────────

    def poll_allowed(self) -> bool:
        """False while the motion side owns the channel (print in flight)."""
        gate = self._poll_gate
        if gate is None:
            return True
        try:
            return bool(gate())
        except Exception:
            return True

    # ── helpers ─────────────────────────────────────────────────────

    def _usable_zp(self):
        """The live ZP manager if it can carry heater traffic, else None.

        A SIMULATED ZP is deliberately not usable: its simulator models motion,
        not heaters, so every probe would report "no sensors" — the incubator
        has its own thermal simulator for no-hardware work, and pointing the
        operator there beats a confusing empty probe.
        """
        if self._closed:
            return None
        try:
            zp = self._zp_getter()
        except Exception:
            return None
        if zp is None:
            return None
        if getattr(zp, "simulate", False):
            return None
        if getattr(zp, "serial", None) is None:
            return None
        if not hasattr(zp, "transact"):
            return None
        return zp

    def _flag_disconnect(self) -> None:
        if self._down_flagged:
            return
        self._down_flagged = True
        if self._on_disconnect is not None:
            try:
                self._on_disconnect()
            except Exception:
                pass

    def _emit_line(self, text: str) -> None:
        if self._on_line is None or not text:
            return
        try:
            self._on_line(text, classify_line(text))
        except Exception:
            logger.debug("[%s] on_line consumer raised", self._name,
                         exc_info=True)

    # ── writing (MarlinLink surface) ────────────────────────────────

    def send_priority(self, command: str) -> bool:
        """Out-of-band write via the ZP stage's bounded-lock raw path."""
        zp = self._usable_zp()
        if zp is None:
            self._flag_disconnect()
            return False
        head = command.strip().split()[0].upper() if command.strip() else ""
        if head not in PRIORITY_COMMANDS:
            logger.debug("[%s] send_priority used for non-emergency %r",
                         self._name, command)
        ok = False
        try:
            ok = bool(zp.priority_write(command))
        except Exception as e:
            logger.warning("[%s] priority write failed: %s", self._name, e)
        if ok and self._on_write is not None:
            try:
                self._on_write(command.strip())
            except Exception:
                pass
        return ok

    def send_and_wait(
        self,
        command: str,
        *,
        timeout_s: float = 8.0,
        hard_ceiling_s: float | None = None,
        collect: bool = True,
    ) -> Transaction:
        """One command through the shared channel, as a ``Transaction``.

        ``hard_ceiling_s`` is accepted for interface parity but the effective
        wait is clamped to :data:`MAX_SHARED_TIMEOUT_S` — a transaction that
        needs longer has no business on the motion board's lock, and the
        controller refuses those operations upstream. The clamp is
        defence-in-depth for any future caller that forgets.
        """
        txn = Transaction(command=command.strip())
        zp = self._usable_zp()
        if zp is None:
            txn.error_text = (
                "the ZP board link is not available (not connected, or "
                "simulated) — connect the ZP board on Hardware Setup first, "
                "or use the incubator's own simulator"
            )
            self._flag_disconnect()
            return txn

        wait_s = min(float(timeout_s), MAX_SHARED_TIMEOUT_S)
        if hard_ceiling_s is not None:
            wait_s = min(wait_s, max(1.0, float(hard_ceiling_s)))

        import time as _time
        started = _time.monotonic()
        if self._on_write is not None:
            try:
                self._on_write(txn.command)
            except Exception:
                pass
        # An M115's legitimate reply contains FIRMWARE_NAME/Marlin — the ZP
        # reset classifier must not misread the probe as a boot banner (a
        # false _board_reset_detected would poison the position-restore
        # flow). A genuine reset still announces itself with "start".
        head = txn.command.split()[0].upper() if txn.command else ""
        with self._lock:
            try:
                ok, text, outcome = zp.transact(
                    txn.command,
                    ok_timeout=wait_s,
                    collect=collect,
                    include_terminal_line=True,
                    allow_identity_lines=(head == "M115"),
                )
            except Exception as e:
                txn.error_text = f"shared-link transaction failed: {e}"
                txn.elapsed_s = _time.monotonic() - started
                self._flag_disconnect()
                return txn
        txn.elapsed_s = _time.monotonic() - started

        lines = [ln for ln in (text or "").splitlines() if ln.strip()]
        if collect:
            txn.lines = list(lines)
        # Feed every line to the consumer so temperature frames, PID dumps and
        # fault lines are parsed exactly as they would be off a reader thread.
        for ln in lines:
            self._emit_line(ln)

        txn.ok = bool(ok)
        if outcome == "error":
            txn.rejected = True
            err = next((ln for ln in reversed(lines)
                        if ln.lower().startswith("error")), "")
            txn.error_text = err or "command rejected"
        elif outcome == "reset":
            txn.board_reset = True
            txn.error_text = "board reset mid-command"
        elif outcome in ("timeout", "hard_timeout"):
            txn.timed_out = True
            txn.error_text = f"no reply within {wait_s:.1f}s"
        elif outcome in ("write_error", "read_error"):
            txn.error_text = f"serial {outcome.replace('_', ' ')}"
            self._flag_disconnect()
        return txn
