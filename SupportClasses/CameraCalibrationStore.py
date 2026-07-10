"""
CameraCalibrationStore.py — Per-machine camera µm/px + rotation calibration.

v7.5.x: A needle/plate camera's µm/px scale and in-plane rotation are a
property of the *physical camera on its USB port* — not of the swappable
bioprinting "hardware setup" (inks, pumps, plate). Storing them inside
`HardwareConfig` meant that loading a saved hardware-setup file (which has no
such calibration) replaced the in-memory config and the auto-save then wiped
the calibration. So this lives in a dedicated machine-level store, keyed by
the stable device identity from ``gui/widgets/camera_identity.py`` (DirectShow
device path = model + USB port). This mirrors how the microscope's per-objective
µm/px lives in ``ObjectiveCalibration`` / objectives.json.

Data file: config/hardware/camera_calibrations.json

Structure::

    {
      "version": "1.0",
      "cameras": {
        "dshow:\\\\?\\usb#vid_f007&pid_a999&mi_00#6&29d1719c&2&0000#{...}\\global": {
          "um_per_px": 5.54,
          "rotation_deg": 45.0,
          "name": "Teslong Camera",
          "date": "2026-05-29"
        }
      }
    }

Usage::

    store = get_store()
    store.set_calibration(identity_key, 5.54, rotation_deg=45.0, name="Teslong")
    cal = store.get_calibration(identity_key)   # {"um_per_px": ..., ...} or None
"""

from __future__ import annotations

import json
import logging
from datetime import date
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_PATH = Path("config/hardware/camera_calibrations.json")


class CameraCalibrationStore:
    """Load/save per-camera (identity-keyed) µm/px + rotation calibration."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = path
        self._data: dict = {"version": "1.0", "cameras": {}, "assignments": {}}
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
            if not isinstance(self._data.get("cameras"), dict):
                self._data["cameras"] = {}
            if not isinstance(self._data.get("assignments"), dict):
                self._data["assignments"] = {}
        except Exception as exc:
            logger.warning(
                f"CameraCalibrationStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "cameras": {}, "assignments": {}}

    def save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            logger.debug(f"CameraCalibrationStore: saved to {self._path}")
        except Exception as exc:
            logger.error(f"CameraCalibrationStore: failed to save: {exc}")

    # ── Read / write ──────────────────────────────────────────────

    def get_calibration(self, identity: str) -> Optional[dict]:
        """Return the stored calibration dict for an identity key, or None."""
        if not identity:
            return None
        return self._data.get("cameras", {}).get(identity)

    def set_calibration(
        self,
        identity: str,
        um_per_px: float,
        rotation_deg: Optional[float] = None,
        name: str = "",
    ) -> None:
        """Store/replace a camera's µm/px (+ optional rotation). Persists.

        On a µm/px-only update (rotation_deg None) any previously-measured
        rotation for this identity is preserved.
        """
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        entry["um_per_px"] = round(float(um_per_px), 6)
        if name:
            entry["name"] = str(name)
        if rotation_deg is not None:
            entry["rotation_deg"] = round(float(rotation_deg), 4)
        # else: keep any existing rotation_deg already in `entry`.
        entry["date"] = str(date.today())
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera calibration saved: {entry.get('name', '?')} "
            f"= {um_per_px:.4f} µm/px"
            f"{f' @ {rotation_deg:.1f}°' if rotation_deg is not None else ''} "
            f"[{identity[:48]}…]"
        )

    def get_rotation(self, identity: str) -> Optional[float]:
        """Return the stored camera→stage rotation (deg) for an identity, or
        None. Independent of µm/px — orientation may be calibrated on its own."""
        if not identity:
            return None
        entry = self._data.get("cameras", {}).get(identity)
        if not isinstance(entry, dict):
            return None
        rot = entry.get("rotation_deg")
        try:
            return float(rot) if rot is not None else None
        except (TypeError, ValueError):
            return None

    def set_rotation(self, identity: str, rotation_deg: Optional[float],
                     name: str = "") -> None:
        """Store/replace ONLY a camera's rotation vs the stage axes (deg).

        The camera's mount rotation is measured on its own (stage-motion
        "Calibrate orientation") and does not require a µm/px value. Kept in the
        same identity entry as µm/px so a physical camera carries both; does not
        disturb any existing ``um_per_px`` / ``image_correction`` siblings.
        Passing ``None`` clears the stored rotation.
        """
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        if rotation_deg is None:
            entry.pop("rotation_deg", None)
        else:
            entry["rotation_deg"] = round(float(rotation_deg), 4)
        if name:
            entry["name"] = str(name)
        entry["date"] = str(date.today())
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera orientation saved: {entry.get('name', '?')} "
            f"= {rotation_deg if rotation_deg is None else round(rotation_deg, 2)}"
            f"° vs stage [{identity[:48]}…]")

    def clear_calibration(self, identity: str) -> None:
        try:
            del self._data["cameras"][identity]
            self.save()
        except KeyError:
            pass

    def all_calibrations(self) -> dict:
        return dict(self._data.get("cameras", {}))

    # ── Per-camera image correction (display-only) ────────────────
    # Brightness / contrast / gamma are a property of the *physical camera +
    # lighting* (like µm/px), so they live in the same identity-keyed entry as
    # a nested ``image_correction`` dict — siblings of um_per_px / rotation_deg.

    def get_image_correction(self, identity: str) -> Optional[dict]:
        """Return ``{brightness, contrast, gamma}`` for an identity, or None."""
        if not identity:
            return None
        entry = self._data.get("cameras", {}).get(identity)
        if not entry:
            return None
        corr = entry.get("image_correction")
        return dict(corr) if isinstance(corr, dict) else None

    def set_image_correction(
        self,
        identity: str,
        brightness: int = 0,
        contrast: float = 1.0,
        gamma: float = 1.0,
        name: str = "",
    ) -> None:
        """Store/replace a camera's display correction. Persists.

        Kept in the same identity entry as µm/px so a single physical camera
        carries both; does not disturb any existing ``um_per_px`` /
        ``rotation_deg`` siblings.
        """
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        entry["image_correction"] = {
            "brightness": int(brightness),
            "contrast": round(float(contrast), 4),
            "gamma": round(float(gamma), 4),
        }
        if name:
            entry["name"] = str(name)
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera image correction saved: {entry.get('name', '?')} "
            f"b={brightness} c={contrast:.2f} g={gamma:.2f} [{identity[:48]}…]"
        )

    # ── Per-camera hardware (camera-side) controls ────────────────
    # Exposure / gain / gamma / brightness / contrast / auto-exposure /
    # capture resolution set ON the camera firmware. Stored in the same
    # identity entry as a nested ``hw_controls`` dict so a physical camera
    # carries its full setup; siblings (um_per_px, image_correction) untouched.

    def get_hw_controls(self, identity: str):
        if not identity:
            return None
        entry = self._data.get("cameras", {}).get(identity)
        if not entry:
            return None
        hw = entry.get("hw_controls")
        return dict(hw) if isinstance(hw, dict) else None

    def set_hw_controls(self, identity: str, controls: dict,
                        name: str = "") -> None:
        """Store/replace a camera's hardware control set. Persists.

        ``controls`` is stored verbatim (only JSON-serialisable values should be
        passed); typically brightness/contrast/gamma/exposure_us/
        exposure_gain_pct/auto_exposure/resolution.
        """
        if not identity or not isinstance(controls, dict):
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        # Drop None values so we never persist "unknown".
        entry["hw_controls"] = {k: v for k, v in controls.items()
                                if v is not None}
        if name:
            entry["name"] = str(name)
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera hardware controls saved: {entry.get('name', '?')} "
            f"{entry['hw_controls']} [{identity[:48]}…]")

    # ── Role → device-identity assignments ────────────────────────
    # Remembers which physical camera (identity) plays each workflow role,
    # so the source assignment auto-restores on the next detect — the user
    # doesn't have to re-pick which camera is needle_x / needle_y each session.

    def get_assignment(self, role: str) -> Optional[str]:
        """Return the stored device identity for a role, or None."""
        if not role:
            return None
        return self._data.get("assignments", {}).get(role)

    def set_assignment(self, role: str, identity: Optional[str]) -> None:
        """Remember (or clear, with None) which camera identity plays a role."""
        if not role:
            return
        assigns = self._data.setdefault("assignments", {})
        if identity is None:
            assigns.pop(role, None)
        else:
            assigns[role] = str(identity)
        self.save()

    def all_assignments(self) -> dict:
        """Return the full role→device-identity map (copy)."""
        return dict(self._data.get("assignments", {}))

    # ── Autostart (which cameras to start on Load / at startup) ───
    # A per-camera flag (sibling of um_per_px / hw_controls) recording whether
    # this physical camera was running when the operator last saved the camera
    # setup. "Load Cameras" and the startup auto-load restore the setup and
    # start exactly the cameras flagged here — "load it exactly as we had it".

    def get_autostart(self, identity: str) -> bool:
        if not identity:
            return False
        entry = self._data.get("cameras", {}).get(identity)
        return bool(entry.get("autostart")) if isinstance(entry, dict) else False

    def set_autostart(self, identity: str, autostart: bool) -> None:
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        entry["autostart"] = bool(autostart)
        cams[identity] = entry
        self.save()

    def any_autostart(self) -> bool:
        """True if any stored camera is flagged to auto-start."""
        return any(
            isinstance(e, dict) and e.get("autostart")
            for e in self._data.get("cameras", {}).values())


# ── Module-level singleton ──────────────────────────────────────────

_store: CameraCalibrationStore | None = None


def get_store(path: Path = _DEFAULT_PATH) -> CameraCalibrationStore:
    """Return the module-level singleton CameraCalibrationStore."""
    global _store
    if _store is None:
        _store = CameraCalibrationStore(path)
    return _store
