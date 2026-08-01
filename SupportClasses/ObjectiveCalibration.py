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

# v7.4.x: Objectives are user-defined — no prepopulated list. Users
# create entries via the Objective Calibration Setup card with their
# own names and nominal magnifications. Existing objectives.json
# files (e.g. carried over from a previous version that auto-populated
# 1x/2x/.../100x) keep their data on load; only first-time installs
# start with an empty list.
STANDARD_OBJECTIVES: list[dict] = []


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

    # ── Objective library CRUD (v7.4.x) ────────────────────────────

    def add_objective(self, name: str, nominal_magnification: float) -> bool:
        """Add a user-defined objective. Returns True on success.

        Duplicate names are rejected (case-sensitive). Persists immediately.
        """
        name = (name or "").strip()
        if not name:
            return False
        objectives = list(self._data.get("objectives", []))
        for obj in objectives:
            if obj.get("name") == name:
                return False  # duplicate
        objectives.append({
            "name": name,
            "nominal_magnification": float(nominal_magnification),
        })
        self._data["objectives"] = objectives
        self.save()
        logger.info(
            f"Objective added: {name} @ nominal {nominal_magnification:g}×"
        )
        return True

    def remove_objective(self, name: str) -> bool:
        """Remove a user-defined objective (and its calibration entries).

        Returns True if the objective existed and was removed. Persists.
        """
        objectives = list(self._data.get("objectives", []))
        remaining = [o for o in objectives if o.get("name") != name]
        if len(remaining) == len(objectives):
            return False  # not found
        self._data["objectives"] = remaining
        # Also strip any calibration entries keyed by this objective name
        # so the store stays internally consistent.
        for cam, cals in self._data.get(
            "camera_objective_calibrations", {}
        ).items():
            cals.pop(name, None)
        self.save()
        logger.info(f"Objective removed: {name}")
        return True

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
        rotation_deg: Optional[float] = None,
    ) -> None:
        """
        Store an empirically measured µm/px calibration.

        Args:
            camera_name: Camera model string (e.g. "BUC3D-1000C").
            objective_name: Objective label (e.g. "4x").
            measured_um_per_px: Measured microns per pixel.
            resolution: Active frame size (width, height) at calibration time.
            rotation_deg: v7.5.x — in-plane stage direction (deg from +X) that
                produced the calibration move, as measured by the stage-motion
                dialog. Lets the live-target picker map clicks → stage offset.
                ``None`` omits the field (backwards compatible).
        """
        cals = self._data.setdefault("camera_objective_calibrations", {})
        cam_cals = cals.setdefault(camera_name, {})
        entry = {
            "measured_um_per_px": round(measured_um_per_px, 6),
            "resolution": list(resolution),
            "date": str(date.today()),
        }
        if rotation_deg is not None:
            entry["rotation_deg"] = round(float(rotation_deg), 3)
        else:
            # v7.5.x: a µm/px-only update must not silently drop a previously
            # measured rotation (mirrors CameraCalibrationStore.set_calibration).
            prev = cam_cals.get(objective_name)
            if isinstance(prev, dict) and prev.get("rotation_deg") is not None:
                entry["rotation_deg"] = prev["rotation_deg"]
        cam_cals[objective_name] = entry
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

    def all_calibrations(self) -> dict:
        """Every stored calibration as ``{camera_name: {objective: dict}}``.

        v7.5.x: used by the ``CameraCalibrationStore`` v1.2 migration to recover
        the resolution a legacy µm/px was measured at (this store has always
        recorded ``resolution``; the camera store had not).
        """
        return {
            cam: dict(cals)
            for cam, cals in self._data
            .get("camera_objective_calibrations", {}).items()
            if isinstance(cals, dict)
        }


# ── Module-level singleton ─────────────────────────────────────────────────

_store: ObjectiveCalibrationStore | None = None


def get_store(path: Path = _DEFAULT_PATH) -> ObjectiveCalibrationStore:
    """Return the module-level singleton ObjectiveCalibrationStore."""
    global _store
    if _store is None:
        _store = ObjectiveCalibrationStore(path)
    return _store
