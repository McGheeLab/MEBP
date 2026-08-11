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

    # ── Plausibility ───────────────────────────────────────────────

    @staticmethod
    def _implied_pixel_um(um_per_px: float, magnification: float,
                          calib_w: float) -> Optional[float]:
        """Effective sensor pixel pitch implied by a calibration, at 1x width.

        ``um_per_px`` is specimen µm per image pixel, so ``× magnification``
        gives the µm of SENSOR each pixel spans. Multiplying by the capture
        width normalises across binning/ROI modes, giving a quantity that is a
        property of the camera alone — the same for every objective.
        """
        try:
            if um_per_px <= 0 or magnification <= 0 or calib_w <= 0:
                return None
            return float(um_per_px) * float(magnification) * float(calib_w)
        except (TypeError, ValueError):
            return None

    def sensor_width_um(self, camera_name: str,
                        tol_frac: float = 0.25) -> Optional[float]:
        """The camera's imaged sensor WIDTH in µm, measured from its objectives.

        v7.16. ``µm/px × magnification × capture_width`` is the width of sensor
        the image spans — a property of the camera alone, identical for every
        objective. Measured on this rig's ToupTek it is 5707 / 5669 / 5724 for
        2x / 4x / 10x (1.0 % spread), and on the Andor 13218 against a datasheet
        13312 (6.5 µm × 2048), so it is real and it is accurate.

        Returns None when nothing on this camera is calibrated, **and also when
        the calibrated objectives disagree by more than ``tol_frac``**. That
        refusal is the point: with two entries a plain median just picks one, so
        a single bad stamp would silently become the camera's "native" scale and
        every derived bound would inherit it. This machine has exactly that case
        — the Andor's 10x is stamped 2048 but was measured at 1024, giving
        13218 vs 27036 — and an unusable answer must read as unusable.
        """
        vals: list[float] = []
        for obj, cal in (self.all_calibrations_for_camera(camera_name)
                         or {}).items():
            if not isinstance(cal, dict):
                continue
            mag = self.nominal_magnification(obj)
            res = cal.get("resolution") or ()
            try:
                w = float(res[0]) if res else 0.0
            except (TypeError, ValueError, IndexError):
                w = 0.0
            v = self._implied_pixel_um(
                float(cal.get("measured_um_per_px") or 0.0),
                float(mag or 0.0), w)
            if v is not None:
                vals.append(v)
        if not vals:
            return None
        vals.sort()
        if vals[0] <= 0 or (vals[-1] / vals[0]) > (1.0 + float(tol_frac)):
            logger.warning(
                "Camera %r: its objectives disagree on sensor width (%s um) — "
                "at least one calibration's resolution stamp or measurement is "
                "wrong, so the camera's native um/px cannot be derived. "
                "Re-measure the odd one out.",
                camera_name, ", ".join(f"{v:.0f}" for v in vals))
            return None
        n = len(vals)
        if n % 2:
            return vals[n // 2]
        return 0.5 * (vals[n // 2 - 1] + vals[n // 2])

    def native_um_per_px(self, camera_name: str,
                         frame_width: float) -> Optional[float]:
        """µm/px this camera would have at **1x magnification**, at
        ``frame_width`` pixels — i.e. the operator's "native µm/px for the
        camera at a 1x frame". None when the camera has no calibration."""
        width_um = self.sensor_width_um(camera_name)
        try:
            fw = float(frame_width)
        except (TypeError, ValueError):
            return None
        if width_um is None or fw <= 0:
            return None
        return width_um / fw

    def predicted_um_per_px(self, camera_name: str, objective_name: str,
                            frame_width: float) -> Optional[float]:
        """µm/px this objective SHOULD have on this camera, at ``frame_width``.

        v7.16, and the operator's own reasoning: *"for it to know how far it can
        move it needs to know the objective its on and the measured
        magnification of the objective — only this way can we get the native
        microns per pixel for the camera at a 1x frame."*

        Lets a NEVER-CALIBRATED objective still get a correct scale estimate
        from its siblings, which is what any bound on the calibration move
        itself has to be sized from — the move happens BEFORE the measurement
        exists, so it cannot use it.
        """
        native = self.native_um_per_px(camera_name, frame_width)
        mag = self.nominal_magnification(objective_name)
        if native is None or not mag or mag <= 0:
            return None
        return native / float(mag)

    def implausible_reason(self, camera_name: str, objective_name: str,
                           um_per_px: float, resolution,
                           tol_frac: float = 0.25) -> Optional[str]:
        """Why a fresh µm/px looks wrong for this camera — or None.

        v7.16. A stage-motion measurement can silently return a number for the
        WRONG camera or the wrong scale (a stale value echoed back, a feature
        that left the frame, an objective that is not the one on the turret).
        Nothing downstream can tell: a mosaic simply scans at that scale, and
        the only symptom is a tile count that is wrong by the square of the
        error — an 8-hour scan instead of 45 minutes.

        The check needs no camera spec and no datasheet. For ONE camera,
        ``µm/px × magnification × capture_width`` is a property of the sensor,
        so it must agree across every objective. Measured on this rig's
        ToupTek: 2x → 0.778843×2×3664 = 5707, 4x → 0.389135×4×3664 = 5703 —
        0.07 % apart.

        Compares against the camera's OTHER calibrated objectives only, so it
        cannot be fooled by another camera's numbers. Returns None when there
        is nothing to compare against — an unverifiable value is reported as
        unverified by the caller, never as passed.
        """
        mag = self.nominal_magnification(objective_name)
        try:
            calib_w = float(resolution[0]) if resolution else 0.0
        except (TypeError, ValueError, IndexError):
            calib_w = 0.0
        mine = self._implied_pixel_um(float(um_per_px or 0.0),
                                      float(mag or 0.0), calib_w)
        if mine is None:
            return None

        others: list[tuple[str, float]] = []
        for obj, cal in (self.all_calibrations_for_camera(camera_name)
                         or {}).items():
            if obj == objective_name or not isinstance(cal, dict):
                continue
            m2 = self.nominal_magnification(obj)
            res2 = cal.get("resolution") or ()
            try:
                w2 = float(res2[0]) if res2 else 0.0
            except (TypeError, ValueError, IndexError):
                w2 = 0.0
            v = self._implied_pixel_um(
                float(cal.get("measured_um_per_px") or 0.0),
                float(m2 or 0.0), w2)
            if v is not None:
                others.append((obj, v))
        if not others:
            return None

        ref = sorted(v for _o, v in others)[len(others) // 2]     # median
        if ref <= 0:
            return None
        ratio = mine / ref
        if (1.0 - float(tol_frac)) <= ratio <= (1.0 + float(tol_frac)):
            return None
        names = ", ".join(o for o, _v in others)
        if ratio < 1.0:
            # µm/px too SMALL ⇒ the FOV is under-estimated ⇒ the raster steps
            # too finely. Tile count grows with the square of the error.
            consequence = (
                f"This reads {1.0 / ratio:.2f}x too SMALL, which under-states "
                f"the field of view — a mosaic would scan about "
                f"{1.0 / (ratio * ratio):.0f}x more tiles than it needs.")
        else:
            # µm/px too LARGE ⇒ FOV over-estimated ⇒ the raster steps too far.
            # Fewer tiles, but they no longer overlap: the mosaic has holes.
            consequence = (
                f"This reads {ratio:.2f}x too LARGE, which over-states the "
                f"field of view — a mosaic would step further than each frame "
                f"actually covers and leave GAPS between tiles.")
        return (
            f"This measurement implies a sensor {ratio:.2f}x the size that "
            f"{objective_name!r} should have on this camera, judged against "
            f"its other calibrated objectives ({names}).\n\n"
            f"For one camera, µm/px × magnification must be the same for every "
            f"objective — so either this reading is wrong (the tracked feature "
            f"left the frame, or a stale value was picked up), or the "
            f"objective actually on the scope is not {objective_name!r}.\n\n"
            + consequence)

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
