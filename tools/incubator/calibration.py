"""
calibration.py — SENSOR calibration (distinct from control-loop tuning).

Two completely different things get called "calibration" on a heater rig, and
conflating them is a good way to waste an afternoon:

  * **Sensor calibration** (this file) — the board says 36.4 C but a trusted
    reference thermometer says 37.0 C. We store a per-channel correction and
    apply it at the DISPLAY and SETPOINT-TRANSLATION layer only.
  * **Control-loop calibration** — PID autotune (``M303``), which changes how
    the firmware drives the heater. See :mod:`.controller`.

This module never touches the firmware's thermistor tables. Those live in
``Configuration.h`` and changing them means a reflash; a host-side offset is
reversible, per-machine, and auditable.

Correction model, deliberately kept to two forms:
  * ``offset``   — one reference point:  ``real = raw + offset``
  * ``linear2pt``— two reference points: ``real = slope * raw + intercept``

Anything fancier (polynomial, Steinhart-Hart refit) is a false promise at the
±0.2 C level this rig operates at, and would hide rather than reveal error.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Literal

logger = logging.getLogger(__name__)

DEFAULT_PATH = Path(__file__).resolve().parent / "_data" / "calibration.json"

Mode = Literal["none", "offset", "linear2pt"]


@dataclass
class ChannelCalibration:
    """Correction for a single sensor channel."""

    channel_uid: str
    mode: Mode = "none"
    offset_c: float = 0.0
    slope: float = 1.0
    intercept: float = 0.0
    #: Reference points as ``{"raw_c": .., "ref_c": .., "at": "ISO ts"}``.
    points: list[dict] = field(default_factory=list)
    note: str = ""

    # ── application ─────────────────────────────────────────────────

    @property
    def active(self) -> bool:
        return self.mode != "none"

    def apply(self, raw_c: float) -> float:
        """Raw device reading -> corrected (real-world) temperature."""
        if self.mode == "offset":
            return raw_c + self.offset_c
        if self.mode == "linear2pt":
            return self.slope * raw_c + self.intercept
        return raw_c

    def invert(self, real_c: float) -> float:
        """
        Desired real-world temperature -> the raw value to command, so that the
        board's own (uncorrected) control loop settles at ``real_c``.

        This is what makes the setpoint honest: if the sensor reads 0.6 C low,
        asking the board for 37 gives you 37.6 in reality. Inverting first, THEN
        letting the caller round to Marlin's integer resolution, keeps the error
        visible instead of silently baked in.
        """
        if self.mode == "offset":
            return real_c - self.offset_c
        if self.mode == "linear2pt":
            if abs(self.slope) < 1e-9:
                return real_c
            return (real_c - self.intercept) / self.slope
        return real_c

    # ── construction from measurements ──────────────────────────────

    def set_single_point(self, raw_c: float, ref_c: float) -> None:
        """One-point calibration: pure offset."""
        self.mode = "offset"
        self.offset_c = float(ref_c) - float(raw_c)
        self.slope, self.intercept = 1.0, 0.0
        self.points = [_point(raw_c, ref_c)]

    def set_two_point(
        self, raw1: float, ref1: float, raw2: float, ref2: float
    ) -> None:
        """
        Two-point calibration: slope + intercept.

        Falls back to a single-point offset when the two raw readings are too
        close together to define a slope — extrapolating a slope from a tiny
        baseline amplifies noise enormously, which would be worse than no
        slope correction at all.
        """
        if abs(float(raw2) - float(raw1)) < 1.0:
            logger.warning(
                "two-point calibration span is only %.2f C; falling back to a "
                "single-point offset", abs(float(raw2) - float(raw1))
            )
            self.set_single_point(raw2, ref2)
            self.note = "2-point span too small; used offset only"
            return
        self.slope = (float(ref2) - float(ref1)) / (float(raw2) - float(raw1))
        self.intercept = float(ref1) - self.slope * float(raw1)
        self.mode = "linear2pt"
        self.offset_c = 0.0
        self.points = [_point(raw1, ref1), _point(raw2, ref2)]

    def clear(self) -> None:
        self.mode = "none"
        self.offset_c = 0.0
        self.slope, self.intercept = 1.0, 0.0
        self.points = []
        self.note = ""

    def describe(self) -> str:
        if self.mode == "offset":
            return f"offset {self.offset_c:+.2f} °C"
        if self.mode == "linear2pt":
            return f"linear ×{self.slope:.4f} {self.intercept:+.2f} °C"
        return "uncalibrated"


def _point(raw_c: float, ref_c: float) -> dict:
    return {
        "raw_c": round(float(raw_c), 3),
        "ref_c": round(float(ref_c), 3),
        "at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }


class CalibrationStore:
    """
    Per-channel calibrations, persisted as JSON next to the tool.

    Written atomically (temp file + replace) so an interrupted save cannot
    leave a corrupt file — the same pattern the app's other stores use.
    """

    def __init__(self, path: Path | None = None):
        self.path = Path(path) if path else DEFAULT_PATH
        self._cals: dict[str, ChannelCalibration] = {}
        self.load()

    # ── access ──────────────────────────────────────────────────────

    def get(self, channel_uid: str) -> ChannelCalibration:
        cal = self._cals.get(channel_uid)
        if cal is None:
            cal = ChannelCalibration(channel_uid=channel_uid)
            self._cals[channel_uid] = cal
        return cal

    def all(self) -> dict[str, ChannelCalibration]:
        return dict(self._cals)

    def apply(self, channel_uid: str, raw_c: float) -> tuple[float, bool]:
        """Returns ``(corrected_c, was_corrected)``."""
        cal = self._cals.get(channel_uid)
        if cal is None or not cal.active:
            return raw_c, False
        return cal.apply(raw_c), True

    def invert(self, channel_uid: str, real_c: float) -> tuple[float, bool]:
        """Returns ``(raw_setpoint_c, was_corrected)``."""
        cal = self._cals.get(channel_uid)
        if cal is None or not cal.active:
            return real_c, False
        return cal.invert(real_c), True

    def clear(self, channel_uid: str) -> None:
        cal = self._cals.get(channel_uid)
        if cal is not None:
            cal.clear()

    # ── persistence ─────────────────────────────────────────────────

    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            logger.warning("calibration load failed (%s): %s", self.path, e)
            return
        for uid, raw in (data.get("channels") or {}).items():
            try:
                self._cals[uid] = ChannelCalibration(
                    channel_uid=uid,
                    mode=raw.get("mode", "none"),
                    offset_c=float(raw.get("offset_c", 0.0)),
                    slope=float(raw.get("slope", 1.0)),
                    intercept=float(raw.get("intercept", 0.0)),
                    points=list(raw.get("points") or []),
                    note=str(raw.get("note", "")),
                )
            except Exception:
                logger.debug("skipping malformed calibration for %s", uid)

    def save(self) -> bool:
        payload = {
            "_description": (
                "Host-side SENSOR calibration for the incubator tool. Applied at "
                "display/setpoint level only; firmware thermistor tables are "
                "never modified."
            ),
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "channels": {
                uid: {k: v for k, v in asdict(c).items() if k != "channel_uid"}
                for uid, c in self._cals.items()
                if c.active or c.points
            },
        }
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            tmp.replace(self.path)
            return True
        except Exception as e:
            logger.warning("calibration save failed (%s): %s", self.path, e)
            return False
