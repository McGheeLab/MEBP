"""
ObjectiveCalibration.py — Per-objective µm/px calibration persistence.

v7.3.4: Stores empirically-measured µm/px values per camera model and
objective, allowing the system to track the real (as opposed to nominal)
objective magnification by comparing the manufacturer's sensor pixel size
to the measured µm/px.

Data file: config/hardware/objectives.json

Structure::

    {
      "objectives": [
        {"name": "4x", "nominal_magnification": 4.0},
        ...
      ],
      "camera_objective_calibrations": {
        "BUC3D-1000C": {
          "4x": {
            "measured_um_per_px": 0.823,
            "resolution": [916, 686],
            "date": "2026-03-13"
          }
        }
      }
    }

Usage::

    store = get_store()
    cal = store.get_calibration("BUC3D-1000C", "4x")
    if cal:
        um_per_px = cal["measured_um_per_px"]

    store.set_calibration("BUC3D-1000C", "4x", 0.823, (916, 686))
"""

from __future__ import annotations

import json
import logging
from datetime import date
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# Default path relative to working directory (project root)
_DEFAULT_PATH = Path("config/hardware/objectives.json")

# Standard microscope objectives included on first-time creation
STANDARD_OBJECTIVES: list[dict] = [
    {"name": "1x",   "nominal_magnification": 1.0},
    {"name": "2x",   "nominal_magnification": 2.0},
    {"name": "4x",   "nominal_magnification": 4.0},
    {"name": "10x",  "nominal_magnification": 10.0},
    {"name": "20x",  "nominal_magnification": 20.0},
    {"name": "40x",  "nominal_magnification": 40.0},
    {"name": "100x", "nominal_magnification": 100.0},
]


class ObjectiveCalibrationStore:
    """
    Load/save per-camera, per-objective µm/px calibration data.

    Wraps objectives.json and provides typed read/write access.
    Instantiate once and reuse (or use the module-level singleton via
    ``get_store()``).
    """

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = path
        self._data: dict = {
            "version": "1.0",
            "objectives": list(STANDARD_OBJECTIVES),
            "camera_objective_calibrations": {},
        }
        self._load()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            # Merge — prefer loaded data but keep defaults for missing keys
            self._data.update(loaded)
        except Exception as exc:
            logger.warning(f"ObjectiveCalibrationStore: failed to load "
                           f"{self._path}: {exc}")

    def save(self) -> None:
        """Persist current state to objectives.json."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            logger.debug(f"ObjectiveCalibrationStore: saved to {self._path}")
        except Exception as exc:
            logger.error(f"ObjectiveCalibrationStore: failed to save: {exc}")

    # ── Objective list ─────────────────────────────────────────────

    @property
    def objectives(self) -> list[dict]:
        """List of standard objective dicts with 'name' and 'nominal_magnification'."""
        return list(self._data.get("objectives", STANDARD_OBJECTIVES))

    def objective_names(self) -> list[str]:
        return [o["name"] for o in self.objectives]

    def nominal_magnification(self, objective_name: str) -> Optional[float]:
        """Return nominal magnification for a named objective, or None."""
        for obj in self.objectives:
            if obj["name"] == objective_name:
                return float(obj["nominal_magnification"])
        return None

    # ── Calibration read / write ───────────────────────────────────

    def get_calibration(self, camera_name: str,
                        objective_name: str) -> Optional[dict]:
        """
        Return the stored calibration dict for camera_name + objective_name,
        or None if not yet calibrated.

        The dict contains at minimum:
            ``measured_um_per_px``, ``resolution``, ``date``
        """
        return (
            self._data
            .get("camera_objective_calibrations", {})
            .get(camera_name, {})
            .get(objective_name)
        )

    def set_calibration(
        self,
        camera_name: str,
        objective_name: str,
        measured_um_per_px: float,
        resolution: tuple[int, int],
    ) -> None:
        """
        Store an empirically measured µm/px calibration.

        Args:
            camera_name: Camera model string (e.g. "BUC3D-1000C").
            objective_name: Objective label (e.g. "4x").
            measured_um_per_px: Measured microns per pixel.
            resolution: Active frame size (width, height) at calibration time.
        """
        cals = self._data.setdefault("camera_objective_calibrations", {})
        cam_cals = cals.setdefault(camera_name, {})
        cam_cals[objective_name] = {
            "measured_um_per_px": round(measured_um_per_px, 6),
            "resolution": list(resolution),
            "date": str(date.today()),
        }
        self.save()
        logger.info(
            f"Objective calibration saved: {camera_name}/{objective_name} "
            f"= {measured_um_per_px:.4f} µm/px @ {resolution}"
        )

    def clear_calibration(self, camera_name: str,
                          objective_name: str) -> None:
        """Remove a stored calibration entry."""
        try:
            del self._data["camera_objective_calibrations"][camera_name][objective_name]
            self.save()
        except KeyError:
            pass

    def all_calibrations_for_camera(self, camera_name: str) -> dict:
        """Return all stored calibrations for a camera as {objective_name: dict}."""
        return dict(
            self._data
            .get("camera_objective_calibrations", {})
            .get(camera_name, {})
        )


# ── Module-level singleton ─────────────────────────────────────────────────

_store: ObjectiveCalibrationStore | None = None


def get_store(path: Path = _DEFAULT_PATH) -> ObjectiveCalibrationStore:
    """Return the module-level singleton ObjectiveCalibrationStore."""
    global _store
    if _store is None:
        _store = ObjectiveCalibrationStore(path)
    return _store
