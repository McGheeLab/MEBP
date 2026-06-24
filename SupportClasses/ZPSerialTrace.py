"""ZPSerialTrace — robust, low-overhead trace of the ZP (Marlin) serial link.

The recurring ZP "disconnect" debugging has been blind: the main log only shows
the *outcome* ("M400 timed out", "ZP disconnected"), never what the serial link
was actually doing — was the board acking? how fast? when did it stop? did a
reset banner come back? was an 'ok' stolen? This module captures every command
transaction (and, in full mode, every byte) to a dedicated rotating file so a
future failure can be diagnosed exactly, without drowning the main app log.

It is separate from :class:`PrintExecutionLogger` (which is per-print) because
ZP comms problems also happen during jog, calibration, travel, and idle polling.

Verbosity — env ``MEBP_ZP_TRACE``:
    "0"        — off (a couple of cheap branches, no file, no ring)
    "" / "1"   — DEFAULT: keep an in-memory ring of recent transactions; write
                 to the file only on an ANOMALY (timeout / error / reset / a
                 slow ack), and when an anomaly fires, dump the whole recent
                 ring so the lead-up is captured. Plus a periodic stats line.
    "full"     — also write every transaction summary and every TX/RX line
                 (byte-level trace) — heaviest, for deep debugging.

Everything here swallows its own exceptions: tracing must NEVER break motion.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque
from datetime import datetime
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional

_LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
_LOG_PATH = _LOG_DIR / "zp_serial.log"

# An 'ok' normally returns in a few ms; flag anything slower than this as an
# anomaly worth writing (and dumping the ring around).
_SLOW_ACK_MS = 750.0
_RING_SIZE = 300
_STATS_EVERY = 250  # emit a rolling stats line every N transactions


def _mode() -> str:
    v = (os.environ.get("MEBP_ZP_TRACE", "") or "").strip().lower()
    if v == "0":
        return "off"
    if v == "full":
        return "full"
    return "summary"


class ZPSerialTracer:
    """Singleton-ish tracer for the ZP serial link. Use :func:`tracer`."""

    def __init__(self):
        self.mode = _mode()
        self._lock = threading.Lock()
        self._ring: deque = deque(maxlen=_RING_SIZE)
        self._logger: Optional[logging.Logger] = None
        self._handler_ok = False
        # rolling stats
        self.n_txn = 0
        self.n_ok = 0
        self.n_fail = 0
        self.n_reset = 0
        self.max_latency_ms = 0.0
        self._sum_latency_ms = 0.0
        self._since_stats = 0
        self._anomaly_streak = 0
        self._t0 = time.monotonic()

    # ── file logger (lazy) ────────────────────────────────────────

    def _log(self) -> Optional[logging.Logger]:
        if self.mode == "off":
            return None
        if self._logger is not None:
            return self._logger if self._handler_ok else None
        try:
            _LOG_DIR.mkdir(parents=True, exist_ok=True)
            lg = logging.getLogger("zp.serial.trace")
            lg.setLevel(logging.DEBUG)
            lg.propagate = False  # keep it OUT of the main console/app log
            if not lg.handlers:
                h = RotatingFileHandler(
                    _LOG_PATH, maxBytes=4_000_000, backupCount=4,
                    encoding="utf-8")
                h.setFormatter(logging.Formatter(
                    "%(asctime)s.%(msecs)03d %(message)s",
                    datefmt="%H:%M:%S"))
                lg.addHandler(h)
            self._logger = lg
            self._handler_ok = True
            lg.info("=== ZP serial trace opened (mode=%s, pid=%s) at %s ===",
                    self.mode, os.getpid(),
                    datetime.now().isoformat(timespec="seconds"))
            return lg
        except Exception:
            self._handler_ok = False
            return None

    def _emit(self, text: str) -> None:
        lg = self._log()
        if lg is not None:
            try:
                lg.info(text)
            except Exception:
                pass

    # ── byte-level trace (full mode only) ─────────────────────────

    def tx(self, line: str) -> None:
        if self.mode == "full":
            self._emit(f"TX  {line!r}")

    def rx(self, line: str) -> None:
        if self.mode == "full":
            self._emit(f"RX  {line!r}")

    # ── per-transaction record ────────────────────────────────────

    def txn(self, command: str, *, outcome: str, latency_ms: float,
            rx_count: int = 0, busy_count: int = 0,
            note: str = "") -> None:
        """Record one command→ack transaction. ``outcome`` ∈ {ok, timeout,
        error, reset, read_error}. Cheap on the hot path; only writes to the
        file on an anomaly (or in full mode) but always updates the ring."""
        if self.mode == "off":
            return
        try:
            latency_ms = float(latency_ms)
            anomaly = (outcome != "ok") or (latency_ms >= _SLOW_ACK_MS)
            rec = (f"{command[:40]:<40} {outcome:<10} "
                   f"{latency_ms:7.1f}ms rx={rx_count} busy={busy_count}"
                   + (f" {note}" if note else ""))
            recovered = 0
            with self._lock:
                self._ring.append(
                    f"+{time.monotonic() - self._t0:8.3f}s  {rec}")
                self.n_txn += 1
                self._since_stats += 1
                if outcome == "ok":
                    self.n_ok += 1
                else:
                    self.n_fail += 1
                if outcome == "reset":
                    self.n_reset += 1
                self.max_latency_ms = max(self.max_latency_ms, latency_ms)
                self._sum_latency_ms += latency_ms
                want_stats = self._since_stats >= _STATS_EVERY
                if want_stats:
                    self._since_stats = 0
                # Anomaly-burst throttle: a USB drop makes EVERY subsequent
                # command fail (3 Hz poller → hundreds/s). Without throttling,
                # dumping the 300-line ring on each one floods the log (a 2.5 MB
                # file in 1 s, observed). Track the consecutive-anomaly streak
                # so we dump the full ring ONCE on the leading edge, log a few
                # more, then only a periodic count — and a single RECOVERED line.
                if anomaly:
                    self._anomaly_streak += 1
                else:
                    recovered = self._anomaly_streak
                    self._anomaly_streak = 0
                streak = self._anomaly_streak
            if self.mode == "full":
                self._emit(f"TXN {rec}")
            elif anomaly:
                if streak == 1:
                    # Leading edge: full context (the lead-up to the failure).
                    self._emit(f"ANOMALY {rec}")
                    self.dump_recent(
                        reason=f"anomaly: {command[:40]} -> {outcome}")
                elif streak <= 5:
                    self._emit(f"ANOMALY {rec}")
                elif streak % 100 == 0:
                    self._emit(f"ANOMALY (continuing, {streak} consecutive) "
                               f"{rec}")
            elif recovered > 5:
                self._emit(f"RECOVERED after {recovered} consecutive anomalies")
            if want_stats:
                self._emit_stats()
        except Exception:
            pass

    def event(self, kind: str, **fields) -> None:
        """Record a notable link event (connect / disconnect / reset / abort)
        and dump the recent transaction ring for context."""
        if self.mode == "off":
            return
        try:
            extra = " ".join(f"{k}={v}" for k, v in fields.items())
            self._emit(f"EVENT {kind} {extra}")
            self.dump_recent(reason=f"event: {kind}")
        except Exception:
            pass

    def _emit_stats(self) -> None:
        try:
            with self._lock:
                n, ok, fail = self.n_txn, self.n_ok, self.n_fail
                rst = self.n_reset
                mx = self.max_latency_ms
                avg = (self._sum_latency_ms / n) if n else 0.0
            self._emit(
                f"STATS txns={n} ok={ok} fail={fail} reset={rst} "
                f"avg={avg:.1f}ms max={mx:.1f}ms")
        except Exception:
            pass

    def dump_recent(self, reason: str = "") -> None:
        """Write the in-memory ring (recent transactions) to the file — the
        lead-up to a failure even when running in summary mode."""
        lg = self._log()
        if lg is None:
            return
        try:
            with self._lock:
                lines = list(self._ring)
            lg.info("---- recent ZP transactions (%s) [%d] ----",
                    reason, len(lines))
            for ln in lines:
                lg.info("    %s", ln)
            lg.info("---- end recent ----")
        except Exception:
            pass

    def snapshot(self) -> dict:
        """Counters for callers that want them (e.g. the print exec log)."""
        with self._lock:
            return {
                "txns": self.n_txn, "ok": self.n_ok, "fail": self.n_fail,
                "reset": self.n_reset, "max_ms": round(self.max_latency_ms, 1),
            }


_TRACER: Optional[ZPSerialTracer] = None
_TRACER_LOCK = threading.Lock()


def tracer() -> ZPSerialTracer:
    """Module-wide ZP serial tracer (lazy singleton)."""
    global _TRACER
    if _TRACER is None:
        with _TRACER_LOCK:
            if _TRACER is None:
                _TRACER = ZPSerialTracer()
    return _TRACER
