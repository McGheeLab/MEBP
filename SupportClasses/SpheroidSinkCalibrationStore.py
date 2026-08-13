"""
SpheroidSinkCalibrationStore.py — the spheroid sink-timing curve.

v7.5.x: When a spheroid is aspirated into the needle it rises into the bore then
slowly **sinks** back toward the tip under gravity. The Spheroid Pick & Place
guided calibration measures a **timing function** — sink time as a function of how
far the spheroid was lifted — by aspirating a *cumulative staircase* of increasing
volumes over one test spheroid and having the operator click each time it reappears
at the tip.

Each calibration step contributes a sample ``(lift_mm, sink_s)`` where
``lift_mm = ΔV_uL / bore_area_mm2`` (1 µL == 1 mm³) is the height that step's
aspirate lifted the spheroid, and ``sink_s`` is how long it took to sink back to
the tip. Increasing lift → increasing sink time → the curve ``t_sink(lift)``.

At run time the spheroid workflow **inverts** this curve: given the estimated
travel time to the placement, it picks the aspirate volume so the spheroid finishes
sinking right as the needle arrives (``lift* = lift_for_time(target_sink)``), so the
spheroid never sinks out inside the well and arrives at the tip for a minimal-excess
release.

**One curve for all** spheroids (assumed roughly uniform). For a plain cannula the
curve ``t(lift)`` is a property of the spheroid + fluid rather than the needle —
volume↔lift uses the *current* bore area, so a gauge change only warns.

⚠ **v7.6 — that no longer holds for a pulled glass capillary.** Sink velocity is
governed by wall drag, which differs by orders of magnitude between a 250 µm tip
and a 1 mm barrel, and the volume↔lift relation itself becomes piecewise at the
tip length. A curve is therefore valid ONLY for the tip geometry it was measured
in: a change in ``tip_id_um`` or ``tip_length_mm`` INVALIDATES it and sink timing
must be re-calibrated, which is why those dimensions are stored alongside.

Data file: ``config/hardware/spheroid_sink_calibration.json``::

    {
      "version": "1.0",
      "curve": {
        "samples": [[0.50, 2.1], [1.00, 4.3], [1.50, 6.8]],
        "bore_area_mm2": 0.0731,
        "needle_gauge": 24,
        "needle_id_um": 305.0,
        "spheroid_diameter_um": 200.0,
        "updated": "2026-06-30T10:15:00"
      }
    }
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

_DEFAULT_PATH = resolve_machine_path("spheroid_sink_calibration.json")


def _interp(x: float, xs: Sequence[float], ys: Sequence[float]) -> float:
    """Piecewise-linear interpolation of y at x. ``xs`` MUST be ascending.
    Clamps to the end values outside ``[xs[0], xs[-1]]`` (no extrapolation)."""
    n = len(xs)
    if n == 0:
        return 0.0
    if n == 1 or x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    # binary-ish linear scan (curves are tiny — a handful of points)
    for i in range(1, n):
        if x <= xs[i]:
            x0, x1 = xs[i - 1], xs[i]
            y0, y1 = ys[i - 1], ys[i]
            if x1 <= x0:
                return y1
            frac = (x - x0) / (x1 - x0)
            return y0 + frac * (y1 - y0)
    return ys[-1]


class SinkCurve:
    """A standalone, monotone sink-timing curve: ``t_sink(lift)`` and its inverse.

    Built from ``(lift_mm, sink_s)`` samples. An implicit ``(0, 0)`` point is
    prepended (zero lift = zero sink time) and the time axis is made monotone
    non-decreasing (running max) so the inverse ``lift_for_time`` is well-defined
    even with slightly noisy samples. Values outside the calibrated range clamp
    to the endpoints — the runtime never over-lifts beyond what was measured.
    """

    def __init__(self, samples: Sequence[Sequence[float]],
                 meta: Optional[dict] = None):
        pts: List[Tuple[float, float]] = []
        for pair in samples or []:
            try:
                lift = float(pair[0])
                t = float(pair[1])
            except (TypeError, ValueError, IndexError):
                continue
            if lift > 0.0 and t > 0.0:
                pts.append((lift, t))
        # sort by lift ascending, dedupe by lift (keep the last), prepend origin
        pts.sort(key=lambda p: p[0])
        dedup: dict = {}
        for lift, t in pts:
            dedup[round(lift, 6)] = t
        lifts = [0.0] + sorted(dedup.keys())
        times = [0.0] + [dedup[k] for k in sorted(dedup.keys())]
        # enforce a monotone non-decreasing time axis for a well-defined inverse
        mono_times = list(times)
        for i in range(1, len(mono_times)):
            if mono_times[i] < mono_times[i - 1]:
                mono_times[i] = mono_times[i - 1]
        self._lifts = lifts
        self._times = times          # for time_for_lift (as measured)
        self._mono_times = mono_times  # for lift_for_time (monotone)
        self.meta = dict(meta or {})

    # ── Query ─────────────────────────────────────────────────────

    @property
    def n_points(self) -> int:
        """Number of real (non-origin) samples."""
        return max(0, len(self._lifts) - 1)

    @property
    def is_valid(self) -> bool:
        return self.n_points >= 1

    def time_for_lift(self, lift_mm: float) -> float:
        """Sink time (s) for a lift height (mm). Clamped to the sampled range."""
        return _interp(float(lift_mm), self._lifts, self._times)

    def lift_for_time(self, t_s: float) -> float:
        """Lift height (mm) whose sink time is ``t_s``. Clamped: for a target
        longer than the largest measured sink time, returns the largest measured
        lift (never extrapolates to an unbounded over-aspirate)."""
        return _interp(float(t_s), self._mono_times, self._lifts)

    def effective_rate_mm_s(self) -> float:
        """A single representative sink velocity (mm/s), least-squares through
        origin over the samples: ``rate = Σ lift·t / Σ t²``. For a readout only."""
        num = 0.0
        den = 0.0
        for lift, t in zip(self._lifts[1:], self._times[1:]):
            num += lift * t
            den += t * t
        return (num / den) if den > 0 else 0.0

    def to_samples(self) -> List[List[float]]:
        return [[lift, t] for lift, t in zip(self._lifts[1:], self._times[1:])]


class SpheroidSinkCalibrationStore:
    """Load/save the single spheroid sink-timing curve (atomic JSON)."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        env = os.environ.get("MEBP_SPHEROID_SINK_CAL_PATH")
        self._path = Path(env) if env else Path(path)
        self._data: dict = {"version": "1.0", "curve": None}
        self._load()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
        except Exception as exc:
            logger.warning(
                "SpheroidSinkCalibrationStore: failed to load %s: %s",
                self._path, exc)
            self._data = {"version": "1.0", "curve": None}

    def save(self) -> None:
        """Atomic write (tmp + os.replace) so a crash can't truncate the file."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=str(self._path.parent), suffix=".tmp")
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(self._data, f, indent=2)
                os.replace(tmp, self._path)
            finally:
                if os.path.exists(tmp):
                    try:
                        os.remove(tmp)
                    except OSError:
                        pass
        except Exception as exc:
            logger.error("SpheroidSinkCalibrationStore: failed to save: %s", exc)

    # ── Read ──────────────────────────────────────────────────────

    def is_calibrated(self) -> bool:
        c = self._data.get("curve")
        return bool(c and isinstance(c.get("samples"), list)
                    and self.get_curve() is not None
                    and self.get_curve().is_valid)

    def get_curve(self) -> Optional[SinkCurve]:
        """Return a standalone :class:`SinkCurve`, or None if uncalibrated."""
        c = self._data.get("curve")
        if not c or not isinstance(c.get("samples"), list):
            return None
        curve = SinkCurve(c.get("samples", []), meta=c)
        return curve if curve.is_valid else None

    def get_meta(self) -> Optional[dict]:
        c = self._data.get("curve")
        return dict(c) if isinstance(c, dict) else None

    # ── Write ─────────────────────────────────────────────────────

    def set_curve(self, samples: Sequence[Sequence[float]],
                  *, bore_area_mm2: Optional[float] = None,
                  needle_gauge: Optional[int] = None,
                  needle_id_um: Optional[float] = None,
                  spheroid_diameter_um: Optional[float] = None,
                  needle_type: Optional[str] = None,
                  tip_id_um: Optional[float] = None,
                  tip_length_mm: Optional[float] = None,
                  tip_area_mm2: Optional[float] = None,
                  tip_profile: Optional[str] = None) -> None:
        """Replace the stored curve with these ``(lift_mm, sink_s)`` samples.

        The ``tip_*`` provenance (v7.6) is written only when supplied, so a
        straight-needle calibration produces exactly the JSON entry it always
        did. It exists so a pulled-tip change can invalidate the curve — see
        the module docstring.
        """
        clean = SinkCurve(samples).to_samples()
        entry: dict = {
            "samples": clean,
            "updated": datetime.now().isoformat(timespec="seconds"),
        }
        if bore_area_mm2 is not None:
            entry["bore_area_mm2"] = round(float(bore_area_mm2), 6)
        if needle_gauge is not None:
            entry["needle_gauge"] = int(needle_gauge)
        if needle_id_um is not None:
            entry["needle_id_um"] = round(float(needle_id_um), 3)
        if spheroid_diameter_um is not None:
            entry["spheroid_diameter_um"] = round(float(spheroid_diameter_um), 3)
        # v7.6 pulled-capillary provenance — omitted entirely for a cannula.
        if needle_type is not None:
            entry["needle_type"] = str(needle_type)
        if tip_id_um is not None:
            entry["tip_id_um"] = round(float(tip_id_um), 3)
        if tip_length_mm is not None:
            entry["tip_length_mm"] = round(float(tip_length_mm), 4)
        if tip_area_mm2 is not None:
            entry["tip_area_mm2"] = round(float(tip_area_mm2), 8)
        if tip_profile is not None:
            entry["tip_profile"] = str(tip_profile)
        self._data["curve"] = entry
        self.save()
        logger.info(
            "Spheroid sink curve stored: %d point(s), ~%.3f mm/s.",
            len(clean), SinkCurve(clean).effective_rate_mm_s())

    def clear(self) -> None:
        self._data["curve"] = None
        self.save()


_store: Optional[SpheroidSinkCalibrationStore] = None


def get_store() -> SpheroidSinkCalibrationStore:
    """Process-wide singleton (mirrors the other calibration stores)."""
    global _store
    if _store is None:
        _store = SpheroidSinkCalibrationStore()
    return _store
