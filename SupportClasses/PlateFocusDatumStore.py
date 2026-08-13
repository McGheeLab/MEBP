"""
PlateFocusDatumStore — the focus-axis ↔ needle-Z datum.

The microscope measures the plate's SHAPE beautifully and its absolute HEIGHT not
at all: a focus reading says "the glass is in focus at 1190 µm on the focus
axis", which says nothing about where the needle is. This store holds the bridge:

    needle_z_zref_mm  =  focal_sign * (focus_um / 1000)  +  offset_mm

Once it exists, the plate bottom can be found **optically** — focus on the glass,
convert, done — and the needle never has to be driven down onto the glass to
teach it again. That is the whole safety argument for this feature.

WHY THE SCALE IS NOT STORED
---------------------------
The relationship's scale must be exactly ±1.000 mm/mm: both axes measure the same
physical displacement of the same rigid object along the same direction. There is
no mechanism for a genuine non-unity scale — only for bugs (a mm/µm slip, a wrong
steps-per-mm, a wrong ``focus_units_per_um`` — this rig has already hit that one
as a 40× error). So the calibration MEASURES the scale, refuses unless it lands
within a couple of percent of 1, and then applies exactly ±1. A measurement that
says "not 1.0" is a bug report, not a calibration constant, and storing it would
bake the error in while masking the fault it was measured to find.

This supersedes ``NeedleFocusTemplateStore.focus_to_needle_z_mm()``, which fits
the offset only and assumes scale = +1 and sign = +1 without checking either.

KEYING
------
``camera identity | objective name | plate key``:

* **camera + objective** — the focus value is where THAT optical train forms an
  image. Change either and the number means something else.
* **plate type** — this is a HEIGHT. A 24-well plate's glass does not sit where a
  96-well plate's does.

Lives under ``config/hardware/`` (never inside a swappable hardware-setup file),
so it cannot travel to another machine.

Zero GUI dependencies.
"""

from __future__ import annotations

import json
import logging
import os
import re
from datetime import datetime
from pathlib import Path

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

_DEFAULT_PATH = resolve_machine_path("plate_focus_datum.json")

#: Measured |scale| must land this close to 1.000 mm/mm or the calibration is
#: refused as a bug rather than stored as a constant.
#:
#: Sized from what a scale error actually COSTS. The applied scale is exactly
#: ±1.000, so a true scale of (1 + e) leaks an error of ``e × Δfocus`` wherever
#: the focus differs from where the datum was captured. Across a plate the focus
#: varies by the tilt — ~1.15 mm on this rig — so e = 1 % is ~12 µm, comfortably
#: inside the 25 µm plate-wide target, while e = 2 % would spend most of it.
#: The physics says the scale is exactly 1, so a real measurement lands far
#: inside this; the band exists to catch units slips, not to permit drift.
SCALE_TOLERANCE = 0.01

#: Needle-zero drift beyond this (mm) makes a stored datum stale.
EPOCH_TOL_MM = 0.01


def _safe(part) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(part)) or "x"


def datum_key(camera_identity: str, objective_name: str,
              plate_key) -> str:
    return f"{_safe(camera_identity)}|{_safe(objective_name)}|{_safe(plate_key)}"


class PlateFocusDatumStore:
    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._data: dict = {"version": "1.0", "datums": {}}
        self._load()

    # ── persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
            if not isinstance(self._data.get("datums"), dict):
                self._data["datums"] = {}
        except Exception as exc:
            logger.warning(f"PlateFocusDatumStore: failed to load "
                           f"{self._path}: {exc}")
            self._data = {"version": "1.0", "datums": {}}

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"PlateFocusDatumStore: failed to save: {exc}")

    def reload(self) -> None:
        self._data = {"version": "1.0", "datums": {}}
        self._load()

    # ── write ─────────────────────────────────────────────────────

    def save(self, camera_identity: str, objective_name: str, plate_key, *,
             focal_sign: int, offset_mm: float,
             focus_um_at_bottom: float | None = None,
             plate_bottom_z_zref_mm: float | None = None,
             anchor_xy_um=None, zero_z_mm: float | None = None,
             turret_position: int | None = None, product_code: str = "",
             measured_scale: float | None = None,
             scale_residual_um: float | None = None,
             closure_um: float | None = None,
             n_points: int = 0, span_um: float | None = None) -> bool:
        """Store a measured datum. ``measured_scale`` is recorded for diagnosis
        only — the APPLIED scale is always exactly ``focal_sign``."""
        key = datum_key(camera_identity, objective_name, plate_key)
        rec = {
            "focal_sign": 1 if int(focal_sign or 1) >= 0 else -1,
            "offset_mm": float(offset_mm),
            "camera_identity": str(camera_identity),
            "objective_name": str(objective_name),
            "plate_key": str(plate_key),
            "date": datetime.now().isoformat(timespec="seconds"),
        }
        for k, v in (("focus_um_at_bottom", focus_um_at_bottom),
                     ("plate_bottom_z_zref_mm", plate_bottom_z_zref_mm),
                     ("zero_z_mm", zero_z_mm),
                     ("turret_position", turret_position),
                     ("measured_scale", measured_scale),
                     ("scale_residual_um", scale_residual_um),
                     ("closure_um", closure_um),
                     ("span_um", span_um)):
            if v is not None:
                rec[k] = v
        if product_code:
            rec["product_code"] = str(product_code)
        if n_points:
            rec["n_points"] = int(n_points)
        if anchor_xy_um is not None:
            try:
                rec["anchor_xy_um"] = [float(anchor_xy_um[0]),
                                       float(anchor_xy_um[1])]
            except (TypeError, ValueError, IndexError):
                pass
        self._data["datums"][key] = rec
        self._save()
        return True

    def clear(self, camera_identity: str, objective_name: str,
              plate_key) -> None:
        if self._data["datums"].pop(
                datum_key(camera_identity, objective_name, plate_key), None):
            self._save()

    # ── read ──────────────────────────────────────────────────────

    def get(self, camera_identity: str, objective_name: str,
            plate_key) -> dict | None:
        return self._data["datums"].get(
            datum_key(camera_identity, objective_name, plate_key))

    def has(self, camera_identity: str, objective_name: str,
            plate_key) -> bool:
        return self.get(camera_identity, objective_name, plate_key) is not None

    def needle_z_zref_mm(self, camera_identity: str, objective_name: str,
                         plate_key, focus_um: float) -> float | None:
        """Convert a focus-axis reading to needle zero-ref mm."""
        rec = self.get(camera_identity, objective_name, plate_key)
        if not rec:
            return None
        try:
            return (float(rec["focal_sign"]) * float(focus_um) / 1000.0
                    + float(rec["offset_mm"]))
        except (TypeError, ValueError, KeyError):
            return None

    def is_stale(self, camera_identity: str, objective_name: str, plate_key,
                 current_zero_z_mm: float | None,
                 tol_mm: float = EPOCH_TOL_MM) -> bool:
        """Has Set Z Zero run since this datum was captured?

        A stale datum is reported, never deleted: the operator needs to see the
        old number to judge how far things moved.
        """
        rec = self.get(camera_identity, objective_name, plate_key)
        if not rec or current_zero_z_mm is None:
            return False
        old = rec.get("zero_z_mm")
        if old is None:
            return False
        try:
            return abs(float(current_zero_z_mm) - float(old)) > float(tol_mm)
        except (TypeError, ValueError):
            return False


def check_scale(measured_scale: float) -> tuple[bool, str]:
    """Is a measured focus↔needle scale acceptable? ``(ok, why)``.

    Refuses rather than applies. See the module docstring: the scale is physics,
    not a free parameter, so a deviation is a fault to find — most likely a units
    slip, a wrong steps-per-mm, or a wrong ``focus_units_per_um``.
    """
    s = abs(float(measured_scale))
    if s < 0.5:
        return (False,
                f"the measured focus↔needle scale is {measured_scale:+.4f} "
                f"mm/mm — far below 1. The focus axis barely moved relative to "
                f"the needle, so the sign cannot be trusted either. Check "
                f"focus_units_per_um and the needle's steps-per-mm.")
    if abs(s - 1.0) > SCALE_TOLERANCE:
        return (False,
                f"the measured focus↔needle scale is {measured_scale:+.4f} "
                f"mm/mm, but it must be exactly ±1.000 — both axes measure the "
                f"same physical displacement. A deviation this size is a bug "
                f"(units, steps-per-mm, or focus_units_per_um), not a "
                f"calibration constant. It is NOT being applied.")
    return (True, "")


_store: PlateFocusDatumStore | None = None


def get_store(path=None) -> PlateFocusDatumStore:
    global _store
    if path is not None:
        return PlateFocusDatumStore(Path(path))
    if _store is None:
        env = os.environ.get("MEBP_PLATE_FOCUS_DATUM_DIR")
        p = (Path(env) / "plate_focus_datum.json") if env else _DEFAULT_PATH
        _store = PlateFocusDatumStore(p)
    return _store


def reset_store() -> None:
    """Test isolation."""
    global _store
    _store = None
