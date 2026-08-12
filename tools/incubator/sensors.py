"""
sensors.py — multi-channel sensor model.

Phase 1 has exactly two sensors, both read by Marlin. Phase 2 adds a separate
box carrying several sensors distributed through the incubator, on its own
connection (the SKR Mini E3 V3 exposes only two analog temperature inputs and
both are consumed by the two control zones).

Rather than hard-wire "two readings from Marlin" through the GUI, telemetry,
calibration and stability code — and then have to unpick it later — everything
downstream consumes a :class:`SensorHub` of N named channels. Adding the box
means writing ONE new :class:`SensorSource` implementation; nothing else
changes.

No Qt, no serial. A source is handed readings by whoever owns its transport.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Iterable, Protocol, runtime_checkable

from .marlin_gcode import TempFrame

#: Source id used by the Marlin-backed source.
MARLIN_SOURCE_ID = "marlin"


@dataclass(frozen=True)
class SensorChannel:
    """One temperature channel at one instant."""

    source_id: str
    key: str                      # unique within the source (e.g. "B", "T")
    label: str                    # operator-facing name
    raw_c: float                  # exactly what the device reported
    value_c: float                # raw_c with calibration applied
    timestamp: float              # time.monotonic()
    target_c: float | None = None
    power_pct: float | None = None
    #: True when a calibration correction was actually applied.
    calibrated: bool = False
    #: Set when the reading is suspect (stale, fault, out of range).
    stale: bool = False

    @property
    def uid(self) -> str:
        """Globally unique channel id across all sources."""
        return f"{self.source_id}:{self.key}"

    @property
    def offset_c(self) -> float:
        return self.value_c - self.raw_c


@runtime_checkable
class SensorSource(Protocol):
    """
    Minimal contract a sensor provider must satisfy.

    Implementations are responsible for their own transport and threading;
    they push readings by calling the callback given to :meth:`set_callback`.
    """

    source_id: str

    def channels(self) -> list[SensorChannel]:
        """Most recent reading for every channel this source knows about."""
        ...

    def set_callback(self, cb: Callable[[list[SensorChannel]], None] | None) -> None:
        """Register a callback invoked with each fresh batch of readings."""
        ...

    def start(self) -> None: ...

    def stop(self) -> None: ...


class _BaseSource:
    """Shared bookkeeping for sensor sources (thread-safe channel cache)."""

    def __init__(self, source_id: str):
        self.source_id = source_id
        self._lock = threading.RLock()
        self._channels: dict[str, SensorChannel] = {}
        self._cb: Callable[[list[SensorChannel]], None] | None = None

    # ── SensorSource ────────────────────────────────────────────────

    def channels(self) -> list[SensorChannel]:
        with self._lock:
            return list(self._channels.values())

    def set_callback(self, cb: Callable[[list[SensorChannel]], None] | None) -> None:
        with self._lock:
            self._cb = cb

    def start(self) -> None:  # pragma: no cover - trivial
        pass

    def stop(self) -> None:
        with self._lock:
            self._channels.clear()

    # ── helpers for subclasses ──────────────────────────────────────

    def _publish(self, batch: list[SensorChannel]) -> None:
        if not batch:
            return
        with self._lock:
            for ch in batch:
                self._channels[ch.key] = ch
            cb = self._cb
        if cb is not None:
            try:
                cb(batch)
            except Exception:
                # A misbehaving consumer must never break the read path.
                pass


class MarlinSensorSource(_BaseSource):
    """
    Derives sensor channels from Marlin temperature frames.

    Feed it every parsed :class:`TempFrame` (from both M105 replies and M155
    autoreport pushes) via :meth:`ingest_frame`. It figures out the channel set
    from whatever fields the firmware actually emits, so a board with no hotend
    configured, or one with extra chamber/redundant sensors, needs no change
    here.

    ``power_for`` maps a temperature key onto its PWM key (``B`` -> ``B@``,
    ``T`` -> ``@``); pass the mapping derived from :mod:`.zones` so duty shows
    up on the right channel.
    """

    def __init__(
        self,
        *,
        power_for: dict[str, str] | None = None,
        labels: dict[str, str] | None = None,
        calibrator: Callable[[str, float], tuple[float, bool]] | None = None,
        source_id: str = MARLIN_SOURCE_ID,
    ):
        super().__init__(source_id)
        self._power_for = dict(power_for or {})
        self._labels = dict(labels or {})
        #: (key, raw_c) -> (corrected_c, was_corrected)
        self._calibrator = calibrator

    def set_calibrator(
        self, calibrator: Callable[[str, float], tuple[float, bool]] | None
    ) -> None:
        self._calibrator = calibrator

    def set_labels(self, labels: dict[str, str]) -> None:
        self._labels.update(labels or {})

    def ingest_frame(self, frame: TempFrame) -> list[SensorChannel]:
        """Convert one frame into channels, cache them, and notify."""
        if not frame:
            return []

        now = time.monotonic()
        batch: list[SensorChannel] = []

        for key in frame.temperature_keys():
            fld = frame.fields[key]
            raw = fld.value

            value, was_cal = raw, False
            if self._calibrator is not None:
                try:
                    value, was_cal = self._calibrator(key, raw)
                except Exception:
                    value, was_cal = raw, False

            pwm_key = self._power_for.get(key)
            power = frame.power_pct(pwm_key) if pwm_key else None

            batch.append(
                SensorChannel(
                    source_id=self.source_id,
                    key=key,
                    label=self._labels.get(key, fld.label),
                    raw_c=raw,
                    value_c=value,
                    timestamp=now,
                    target_c=fld.target,
                    power_pct=power,
                    calibrated=was_cal,
                )
            )

        self._publish(batch)
        return batch


class StaticSensorSource(_BaseSource):
    """
    Trivial source used by the self-test to prove the phase-2 seam: pushing
    channels through this shows up in the hub and the GUI table with no other
    code change. Also handy as a template for the real sensor box.
    """

    def push(self, readings: Iterable[tuple[str, str, float]]) -> list[SensorChannel]:
        """Push ``(key, label, celsius)`` triples as a fresh batch."""
        now = time.monotonic()
        batch = [
            SensorChannel(
                source_id=self.source_id,
                key=key,
                label=label,
                raw_c=float(c),
                value_c=float(c),
                timestamp=now,
            )
            for key, label, c in readings
        ]
        self._publish(batch)
        return batch


class SensorHub:
    """
    Aggregates every :class:`SensorSource` and is what the GUI, telemetry,
    calibration and stability code consume.

    Channels are keyed by ``"<source_id>:<key>"`` so two sources may safely use
    the same local key.
    """

    #: A channel older than this is reported stale.
    STALE_AFTER_S = 10.0

    def __init__(self):
        self._lock = threading.RLock()
        self._sources: dict[str, SensorSource] = {}
        self._latest: dict[str, SensorChannel] = {}
        self._listeners: list[Callable[[list[SensorChannel]], None]] = []

    # ── source registry ─────────────────────────────────────────────

    def add_source(self, source: SensorSource) -> None:
        with self._lock:
            self._sources[source.source_id] = source
        source.set_callback(self._on_batch)

    def remove_source(self, source_id: str) -> None:
        with self._lock:
            src = self._sources.pop(source_id, None)
            for uid in [u for u in self._latest if u.startswith(f"{source_id}:")]:
                self._latest.pop(uid, None)
        if src is not None:
            try:
                src.set_callback(None)
            except Exception:
                pass

    def sources(self) -> list[str]:
        with self._lock:
            return sorted(self._sources)

    # ── consumers ───────────────────────────────────────────────────

    def add_listener(self, cb: Callable[[list[SensorChannel]], None]) -> None:
        with self._lock:
            self._listeners.append(cb)

    def _on_batch(self, batch: list[SensorChannel]) -> None:
        with self._lock:
            for ch in batch:
                self._latest[ch.uid] = ch
            listeners = list(self._listeners)
        for cb in listeners:
            try:
                cb(batch)
            except Exception:
                pass

    # ── queries ─────────────────────────────────────────────────────

    def all_channels(self) -> list[SensorChannel]:
        """Every known channel, staleness applied, in stable order."""
        now = time.monotonic()
        with self._lock:
            chans = list(self._latest.values())
        out = []
        for ch in chans:
            if not ch.stale and (now - ch.timestamp) > self.STALE_AFTER_S:
                ch = replace(ch, stale=True)
            out.append(ch)
        out.sort(key=lambda c: (c.source_id, c.key))
        return out

    def channel(self, uid: str) -> SensorChannel | None:
        with self._lock:
            return self._latest.get(uid)

    def marlin_channel(self, key: str) -> SensorChannel | None:
        """Convenience lookup for a Marlin-sourced channel by field key."""
        return self.channel(f"{MARLIN_SOURCE_ID}:{key}")

    def clear(self) -> None:
        with self._lock:
            self._latest.clear()
