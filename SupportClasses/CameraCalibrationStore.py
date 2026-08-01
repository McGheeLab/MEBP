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
      "version": "1.2",
      "cameras": {
        "dshow:\\\\?\\usb#vid_f007&pid_a999&mi_00#6&29d1719c&2&0000#{...}\\global": {
          "um_per_px": 5.54,
          "um_per_px_resolution": [1024, 1024],
          "rotation_deg": 1.2,
          "column_dir_deg": -44.6,
          "name": "Teslong Camera",
          "date": "2026-05-29"
        }
      }
    }

v1.1 (rotated needle rig): ``rotation_deg`` is strictly the DISPLAY roll
(deviation from parallel); the needle cameras' column→stage mount direction
(±45° about +X) lives in the separate ``column_dir_deg`` and feeds only the
two-camera needle aligner. Legacy needle entries migrate on load
(rotation_deg → column_dir_deg; see ``_migrate``).

v1.2 (unified mosaic calibration): ``um_per_px`` gains its measurement
resolution ``um_per_px_resolution``. µm/px ∝ 1/frame_width, so a value measured
at 1024 px is WRONG by 2× when the camera later runs at 2048 px — and without
the stamp ``CameraManager.effective_um_per_px`` silently degrades to a
passthrough, which is exactly how a restored camera ended up scaling its mosaic
wrong. **Absent means UNKNOWN, never "valid at whatever the live width is"** —
callers must report it as uncalibrated rather than guess. Also adds
``mosaic_output_rotation_deg``: a DISPLAY-ONLY whole-mosaic rotation (see
``set_mosaic_output_rotation``).

Usage::

    store = get_store()
    store.set_calibration(identity_key, 5.54, rotation_deg=45.0, name="Teslong",
                          um_per_px_resolution=(1024, 1024))
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

# Current on-disk schema version. A FRESH store is born here; only files that
# load with an older version run the migration chain in ``_migrate``.
SCHEMA_VERSION = "1.2"

# Relative tolerance for matching a stored µm/px against an objective
# calibration's ``measured_um_per_px`` during the v1.2 backfill. Both are
# rounded to 6 decimals by their writers, so a genuine match is essentially
# exact; this only absorbs that rounding.
_UM_PER_PX_MATCH_RTOL = 1e-4


class CameraCalibrationStore:
    """Load/save per-camera (identity-keyed) µm/px + rotation calibration."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = path
        # A FRESH store is born at the current schema version — only files
        # that load with an older version run the migration (a version-less
        # legacy file is treated as 1.0 in _load).
        self._data: dict = {
            "version": SCHEMA_VERSION, "cameras": {}, "assignments": {}}
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
                # A stored file that predates the version key is legacy 1.0
                # (must not inherit the fresh-store default above).
                self._data["version"] = str(loaded.get("version", "1.0"))
            if not isinstance(self._data.get("cameras"), dict):
                self._data["cameras"] = {}
            if not isinstance(self._data.get("assignments"), dict):
                self._data["assignments"] = {}
            self._migrate()
        except Exception as exc:
            logger.warning(
                f"CameraCalibrationStore: failed to load {self._path}: {exc}")
            self._data = {
                "version": SCHEMA_VERSION, "cameras": {}, "assignments": {}}

    def _migrate(self) -> None:
        """Run the migration chain up to ``SCHEMA_VERSION``. Idempotent.

        Each step is guarded on the stored version so a current file does no
        work, and a very old file walks 1.0 → 1.1 → 1.2 in order.
        """
        ver = str(self._data.get("version", "1.0"))
        dirty = False
        if ver == "1.0":
            self._migrate_10_to_11()
            ver = "1.1"
            dirty = True
        if ver == "1.1":
            self._migrate_11_to_12()
            ver = "1.2"
            dirty = True
        if dirty or self._data.get("version") != SCHEMA_VERSION:
            self._data["version"] = SCHEMA_VERSION
            self.save()

    def _migrate_11_to_12(self) -> None:
        """v1.1 → v1.2: backfill ``um_per_px_resolution``.

        µm/px ∝ 1/frame_width, so a value with no recorded resolution cannot be
        rescaled and ``effective_um_per_px`` degrades to a passthrough — the
        camera then scales its mosaic wrong at any resolution other than the one
        it was calibrated at. Recover the stamp where it is *knowable*:

        1. A matching ``measured_um_per_px`` in ``ObjectiveCalibration``
           (which HAS always recorded its resolution). The values are written
           rounded to 6 decimals and are highly distinctive, so a **unique**
           value match across all camera/objective entries is strong evidence
           they are the same measurement. An ambiguous match is rejected.
        2. Else ``hw_controls.resolution`` — the resolution the camera is
           configured to, which is what a calibration would have run at.
        3. Else leave it ABSENT. Absent means *unknown*, and callers must treat
           the camera as needing re-calibration; guessing the live width here
           would silently reintroduce the bug this stamp exists to prevent.
        """
        cams = self._data.get("cameras", {})
        by_value = self._objective_resolutions_by_um_per_px()
        for ident, entry in cams.items():
            if not isinstance(entry, dict):
                continue
            if entry.get("um_per_px") is None:
                continue
            if entry.get("um_per_px_resolution") is not None:
                continue
            try:
                val = float(entry["um_per_px"])
            except (TypeError, ValueError):
                continue
            res, source = self._match_resolution(val, by_value, entry)
            if res is None:
                logger.info(
                    f"CameraCalibrationStore: '{entry.get('name', '?')}' "
                    f"µm/px {val:.6f} has no recoverable measurement "
                    f"resolution — left unstamped (needs re-calibration)")
                continue
            entry["um_per_px_resolution"] = [int(res[0]), int(res[1])]
            logger.info(
                f"CameraCalibrationStore: backfilled µm/px resolution for "
                f"'{entry.get('name', '?')}' — {val:.6f} @ {res[0]}×{res[1]} "
                f"(from {source})")

    @staticmethod
    def _objective_resolutions_by_um_per_px() -> dict:
        """``{measured_um_per_px: [(w, h), …]}`` over every objective
        calibration, for the v1.2 backfill. Lazy-imported and fully guarded so
        the store stays usable (and unit-testable) without the objective store.
        """
        out: dict = {}
        try:
            from SupportClasses.ObjectiveCalibration import get_store as _objs
            data = _objs().all_calibrations()
        except Exception as exc:
            logger.debug(f"objective resolutions unavailable: {exc}")
            return out
        if not isinstance(data, dict):
            return out
        for per_obj in data.values():
            if not isinstance(per_obj, dict):
                continue
            for cal in per_obj.values():
                if not isinstance(cal, dict):
                    continue
                res = cal.get("resolution")
                try:
                    meas = float(cal.get("measured_um_per_px"))
                except (TypeError, ValueError):
                    continue
                if not (res and len(res) >= 2 and res[0] and res[1]):
                    continue
                out.setdefault(meas, []).append((int(res[0]), int(res[1])))
        return out

    @staticmethod
    def _match_resolution(value: float, by_value: dict, entry: dict):
        """Resolve ``(w, h), source`` for a µm/px value, or ``(None, "")``."""
        hits: set = set()
        for meas, resolutions in by_value.items():
            if meas == 0:
                continue
            if abs(value - meas) <= _UM_PER_PX_MATCH_RTOL * abs(meas):
                hits.update(resolutions)
        if len(hits) == 1:
            return next(iter(hits)), "matching objective calibration"
        hw = entry.get("hw_controls")
        if isinstance(hw, dict):
            res = hw.get("resolution")
            if res and len(res) >= 2 and res[0] and res[1]:
                return (int(res[0]), int(res[1])), "hw_controls.resolution"
        return None, ""

    def _migrate_10_to_11(self) -> None:
        """v1.0 → v1.1: split the needle cameras' conflated ``rotation_deg``.

        Before v1.1 a needle side-camera's ``rotation_deg`` stored the
        column→stage MOUNT direction (±45°-ish) — the aligner input — and
        the display wrongly rotated the live view by it. v1.1 keeps
        ``rotation_deg`` strictly as the display roll and adds
        ``column_dir_deg`` for the mount direction. A legacy needle entry's
        rotation therefore MOVES to ``column_dir_deg`` (that was its
        semantic) and the display roll becomes unknown (absent ⇒ level
        view). Non-needle identities are untouched. Idempotent.
        """
        assigns = self._data.get("assignments", {})
        cams = self._data.get("cameras", {})
        for role in ("needle_x", "needle_y"):
            ident = assigns.get(role)
            entry = cams.get(ident) if ident else None
            if (isinstance(entry, dict)
                    and entry.get("rotation_deg") is not None
                    and entry.get("column_dir_deg") is None):
                entry["column_dir_deg"] = entry.pop("rotation_deg")
                logger.info(
                    f"CameraCalibrationStore: migrated needle cam "
                    f"'{entry.get('name', '?')}' rotation_deg → "
                    f"column_dir_deg ({entry['column_dir_deg']}°)")

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
        column_dir_deg: Optional[float] = None,
        um_per_px_resolution: Optional[tuple[int, int]] = None,
    ) -> None:
        """Store/replace a camera's µm/px (+ optional rotation / column dir).
        Persists.

        On a µm/px-only update (rotation_deg / column_dir_deg None) any
        previously-measured rotation / column direction for this identity is
        preserved.

        ``um_per_px_resolution`` is the (w, h) frame size the µm/px was measured
        at. It travels WITH the value — a new µm/px measured at an unknown
        resolution CLEARS any previous stamp rather than inheriting it, because
        pairing a fresh value with a stale resolution would produce a
        confidently wrong rescale (worse than a known-unknown).
        """
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        entry["um_per_px"] = round(float(um_per_px), 6)
        if (um_per_px_resolution and len(um_per_px_resolution) >= 2
                and um_per_px_resolution[0] and um_per_px_resolution[1]):
            entry["um_per_px_resolution"] = [
                int(um_per_px_resolution[0]), int(um_per_px_resolution[1])]
        else:
            entry.pop("um_per_px_resolution", None)
        if name:
            entry["name"] = str(name)
        if rotation_deg is not None:
            entry["rotation_deg"] = round(float(rotation_deg), 4)
        # else: keep any existing rotation_deg already in `entry`.
        if column_dir_deg is not None:
            entry["column_dir_deg"] = round(float(column_dir_deg), 4)
        # else: keep any existing column_dir_deg already in `entry`.
        entry["date"] = str(date.today())
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera calibration saved: {entry.get('name', '?')} "
            f"= {um_per_px:.4f} µm/px"
            f"{f' @ {rotation_deg:.1f}°' if rotation_deg is not None else ''} "
            f"[{identity[:48]}…]"
        )

    def get_um_per_px_resolution(
            self, identity: str) -> Optional[tuple[int, int]]:
        """Resolution (w, h) the stored ``um_per_px`` was measured at, or None.

        None means UNKNOWN — the value cannot be rescaled to another capture
        resolution, so callers must treat the camera as needing re-calibration
        rather than assuming the value is valid at the live width.
        """
        if not identity:
            return None
        entry = self._data.get("cameras", {}).get(identity)
        if not isinstance(entry, dict):
            return None
        res = entry.get("um_per_px_resolution")
        try:
            if res and len(res) >= 2 and res[0] and res[1]:
                return (int(res[0]), int(res[1]))
        except (TypeError, ValueError):
            pass
        return None

    def get_mosaic_output_rotation(self, identity: str) -> float:
        """DISPLAY-ONLY whole-mosaic output rotation (deg CCW), default 0.

        v7.5.x: lets the operator put a built mosaic the correct way up without
        touching the camera→stage calibration. Quantised to 0/90/180/270.

        ⚠ This is applied by the VIEWERS at paint time and is deliberately NOT
        baked into the stored composite or its ``extent_um`` — every consumer
        back-projects ``stage = (extent − shift) + px/scale`` to derive well
        centres for MOTION, and rotating the stored extent would reintroduce the
        class of bug that once drove the stage millimetres off target.
        """
        if not identity:
            return 0.0
        entry = self._data.get("cameras", {}).get(identity)
        if not isinstance(entry, dict):
            return 0.0
        try:
            return float(entry.get("mosaic_output_rotation_deg") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def set_mosaic_output_rotation(self, identity: str, rotation_deg: float,
                                   name: str = "") -> None:
        """Store the display-only whole-mosaic output rotation (0/90/180/270).

        Snapped to the nearest quadrant; 0 pops the key so an unrotated camera's
        entry stays byte-identical to a legacy one.
        """
        if not identity:
            return
        try:
            quad = int(round(float(rotation_deg) / 90.0)) % 4 * 90
        except (TypeError, ValueError):
            quad = 0
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        if quad:
            entry["mosaic_output_rotation_deg"] = quad
        else:
            entry.pop("mosaic_output_rotation_deg", None)
        if name:
            entry["name"] = str(name)
        cams[identity] = entry
        self.save()
        logger.info(
            f"Mosaic output rotation saved: {entry.get('name', '?')} "
            f"= {quad}° (display only) [{identity[:48]}…]")

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

    def get_column_dir(self, identity: str) -> Optional[float]:
        """Return the stored column→stage MOUNT direction (deg CCW from
        stage +X) for an identity, or None.

        v7.5.x (rotated rig): a needle side-camera's mount direction (±45°
        about +X) feeds ONLY the two-camera needle aligner — never the
        display. The display roll lives in ``rotation_deg``.
        """
        if not identity:
            return None
        entry = self._data.get("cameras", {}).get(identity)
        if not isinstance(entry, dict):
            return None
        cd = entry.get("column_dir_deg")
        try:
            return float(cd) if cd is not None else None
        except (TypeError, ValueError):
            return None

    def set_column_dir(self, identity: str, column_dir_deg: Optional[float],
                       name: str = "") -> None:
        """Store/replace ONLY a camera's column→stage mount direction (deg).

        Sibling of ``rotation_deg`` (display roll) in the same identity
        entry; preserves ``um_per_px`` / ``image_correction`` / etc.
        Passing ``None`` clears the stored direction.
        """
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        if column_dir_deg is None:
            entry.pop("column_dir_deg", None)
        else:
            entry["column_dir_deg"] = round(float(column_dir_deg), 4)
        if name:
            entry["name"] = str(name)
        entry["date"] = str(date.today())
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera column direction saved: {entry.get('name', '?')} "
            f"= {column_dir_deg if column_dir_deg is None else round(column_dir_deg, 2)}"
            f"° vs stage +X [{identity[:48]}…]")

    def get_mirrored(self, identity: str) -> bool:
        """Whether the camera's view is mirrored (horizontal flip), or False.

        A mirror flips image handedness — which the rotation alone cannot
        express — so it is stored as its own sibling of ``rotation_deg``.
        Absent key ⇒ not mirrored (legacy entries stay byte-identical)."""
        if not identity:
            return False
        entry = self._data.get("cameras", {}).get(identity)
        if not isinstance(entry, dict):
            return False
        return bool(entry.get("mirrored", False))

    def set_mirrored(self, identity: str, mirrored: bool,
                     name: str = "") -> None:
        """Store/replace ONLY a camera's mirrored-view flag.

        Independent of µm/px + rotation (the mount/optics handedness is a
        property of the physical camera). Kept in the same identity entry,
        preserving any ``um_per_px`` / ``rotation_deg`` / ``image_correction``
        siblings. Stored only when True so an un-mirrored camera's entry is
        unchanged (``False`` pops the key)."""
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        if mirrored:
            entry["mirrored"] = True
        else:
            entry.pop("mirrored", None)
        if name:
            entry["name"] = str(name)
        entry["date"] = str(date.today())
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera mirror saved: {entry.get('name', '?')} "
            f"mirrored={bool(mirrored)} [{identity[:48]}…]")

    def get_flip_y(self, identity: str) -> bool:
        """Whether the camera's view is flipped vertically (flip Y), or False.
        Sibling of ``mirrored`` (flip X); absent key ⇒ not flipped."""
        if not identity:
            return False
        entry = self._data.get("cameras", {}).get(identity)
        if not isinstance(entry, dict):
            return False
        return bool(entry.get("flip_y", False))

    def set_flip_y(self, identity: str, flip_y: bool, name: str = "") -> None:
        """Store/replace ONLY a camera's vertical-flip (flip Y) flag — sibling of
        ``mirrored``/``rotation_deg``. Stored only when True (``False`` pops the
        key so legacy entries stay byte-identical)."""
        if not identity:
            return
        cams = self._data.setdefault("cameras", {})
        entry = dict(cams.get(identity, {}))
        if flip_y:
            entry["flip_y"] = True
        else:
            entry.pop("flip_y", None)
        if name:
            entry["name"] = str(name)
        entry["date"] = str(date.today())
        cams[identity] = entry
        self.save()
        logger.info(
            f"Camera flip-Y saved: {entry.get('name', '?')} "
            f"flip_y={bool(flip_y)} [{identity[:48]}…]")

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
