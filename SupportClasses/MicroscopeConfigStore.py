"""
MicroscopeConfigStore.py — per-machine motorized-microscope settings (v7.5.x).

Owns everything the operator configures about the microscope BODY (the Nikon Ti
Eclipse turrets + focus drive), kept deliberately separate from
``HardwareConfig`` because — like the camera calibrations and the calibration
status — it is a property of *this physical rig*, not of a swappable print
setup. A hardware-setup file loaded from another machine must never carry
another rig's filter-cube assignments.

What lives here:

* **backend selection** — which driver talks to the body (``simulated`` /
  ``nikon_ti`` / ``micromanager``) plus the few driver-specific knobs
  (COM ProgID, Micro-Manager config path, Z device-units-per-µm).
* **slot assignments** — the operator's name for each filter-cube slot and each
  nosepiece position. The turret reports a POSITION NUMBER; only the operator
  knows that slot 3 holds the mCherry cube. That mapping is exactly what this
  store exists to remember.
* **focus preferences** — jog step, direction convention, optional soft limits.

Data file: ``config/hardware/microscope.json`` (override the directory with
``$MEBP_MICROSCOPE_CONFIG_DIR`` for test isolation). Atomic writes
(temp + ``os.replace``), same as ``MosaicStore`` / ``CalibrationStatusStore``.

Slot numbering is **1-based everywhere** — it matches the numbers engraved on
the turret, so what the operator reads on the microscope is what the UI shows.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_FILENAME = "microscope.json"

#: Backend identifiers understood by :mod:`SupportClasses.MicroscopeControl`.
BACKENDS = ("simulated", "nikon_ti", "micromanager")

#: A Ti filter cassette holds 6 cubes; the nosepiece is commonly 6 (some are 5).
_DEFAULT_FILTER_SLOTS = 6
_DEFAULT_OBJECTIVE_SLOTS = 6

#: Nikon Ti ZDrive counts in 10 nm units → 100 units per µm. Bench-verifiable
#: (command a 100 µm move, measure it) and overridable per machine, because
#: getting this wrong scales every focus move.
_DEFAULT_Z_UNITS_PER_UM = 100.0

_DEFAULT_FOCUS_STEP_UM = 10.0

_MAX_SLOTS = 12  # sanity ceiling; no microscope turret is anywhere near this


def _default_path() -> Path:
    d = os.environ.get("MEBP_MICROSCOPE_CONFIG_DIR")
    if d:
        return Path(d) / _DEFAULT_FILENAME
    return Path("config/hardware") / _DEFAULT_FILENAME


#: Plausibility band for a filter wavelength (nm). Deliberately generous —
#: it exists to catch a unit slip (µm typed as 0.519, or an Ångström value),
#: not to police exotic optics.
_MIN_WAVELENGTH_NM = 200.0
_MAX_WAVELENGTH_NM = 1600.0

#: Public so the Microscope setup UI can bound its spin boxes to exactly what
#: the store will accept. There is ONE owner of this rule and it is here —
#: a second copy in the UI is the "two homes for one fact" trap, and the
#: symptom would be a value the operator can type but not save.
WAVELENGTH_BAND_NM = (_MIN_WAVELENGTH_NM, _MAX_WAVELENGTH_NM)


def clean_wavelength(value):
    """A plausible wavelength in nm, or None. Never raises.

    Public for the same reason as :data:`WAVELENGTH_BAND_NM`.
    """
    try:
        nm = float(value)
    except (TypeError, ValueError):
        return None
    if not (_MIN_WAVELENGTH_NM <= nm <= _MAX_WAVELENGTH_NM):
        return None
    return nm


#: Retained name for this module's own callers.
_clean_wavelength = clean_wavelength


def _clean_optics(raw) -> dict:
    """``{cube name: {emission_nm, excitation_nm}}``, dropping anything unusable.

    Entries with neither wavelength are removed entirely, so "present but
    empty" can never be mistaken for "measured".
    """
    out: dict = {}
    if not isinstance(raw, dict):
        return out
    for name, entry in raw.items():
        key = str(name).strip()
        if not key or not isinstance(entry, dict):
            continue
        clean = {}
        for field in ("emission_nm", "excitation_nm"):
            nm = _clean_wavelength(entry.get(field))
            if nm is not None:
                clean[field] = nm
        if clean:
            out[key] = clean
    return out


def _clean_slots(value, default: int) -> int:
    try:
        n = int(value)
    except (TypeError, ValueError):
        return default
    return max(1, min(_MAX_SLOTS, n))


class MicroscopeConfigStore:
    """Load/save the per-machine microscope configuration."""

    def __init__(self, path: Optional[Path] = None):
        self._path = Path(path) if path is not None else _default_path()
        self._lock = threading.RLock()
        self._data: dict = self._blank()
        self._load()

    # ── Defaults / load / save ────────────────────────────────────

    @staticmethod
    def _blank() -> dict:
        return {
            "version": "1.0",
            "backend": "simulated",
            # Driver-specific knobs (only the selected backend reads its own).
            "prog_id": "",              # Ti SDK COM ProgID override
            "cassette": 1,              # which filter cassette on a dual-cube body
            "z_units_per_um": _DEFAULT_Z_UNITS_PER_UM,
            "mm_config_path": "",       # Micro-Manager .cfg
            "mm_dir": "",               # Micro-Manager install (device adapters)
            # Device names as published by Micro-Manager's NikonTI adapter
            # (https://micro-manager.org/NikonTI). They hang off the TIScope hub,
            # which the .cfg must load first. Editable — a config may relabel them.
            "mm_filter_device": "TIFilterBlock1",
            "mm_objective_device": "TINosePiece",
            "mm_focus_device": "TIZDrive",
            # v7.17 — illumination + light path. Same adapter, same hub; these
            # three are accessories, so a configuration that does not load them
            # is normal and every backend degrades to "not fitted".
            "mm_epi_shutter_device": "TIEpiShutter",
            "mm_dia_lamp_device": "TIDiaLamp",
            "mm_light_path_device": "TILightPath",
            # Slot assignments (operator-owned names).
            "filter_slots": _DEFAULT_FILTER_SLOTS,
            "objective_slots": _DEFAULT_OBJECTIVE_SLOTS,
            "filter_cubes": {},         # {"1": "DAPI", ...} — 1-based string keys
            "objectives": {},           # {"1": "4x Plan Fluor", ...}
            # v7.17 — per-cube emission/excitation, for the LabLink image-job
            # sidecar. Keyed by the cube's NAME, not its turret position: the
            # wavelengths are a property of the cube and travel with it if it
            # is moved to another slot, and a fluorescence scan labels its
            # channels by name, which is the join a sidecar needs.
            #   {"FITC": {"emission_nm": 519.0, "excitation_nm": 495.0}}
            # ⚠ Absent means UNKNOWN and is left absent all the way to the
            # wire. LabLink answers `missing_metadata` naming the fields,
            # which is recoverable; a fabricated nominal value is not — it
            # silently changes the answer (measured on real data: supplying
            # NA + emission moved a segmented object count 2855 -> 2660 with
            # no warning from any layer).
            "filter_optics": {},
            # Focus preferences.
            "focus_step_um": _DEFAULT_FOCUS_STEP_UM,
            "focus_up_is_positive": True,
            "focus_min_um": None,       # optional operator soft limits
            "focus_max_um": None,
            # Per-objective focus offsets measured by the plate-leveling survey.
            # Parfocality is a property of the objectives AS MOUNTED IN THIS
            # NOSEPIECE and of the tube/camera path — not of the plate — so it
            # belongs to the body, is measured once, and must be readable by any
            # surface that switches objectives without importing the wizard.
            #   {camera_identity: {"reference": "4x",
            #                      "offsets_um": {...},
            #                      "centration_um": {name: [dx, dy]},
            #                      "product_codes": {name: "MRH20040"},
            #                      "measured_at": iso}}
            "parfocal_offsets_um": {},
            # Apply the parfocal shift automatically when the objective changes.
            # Off by default: silently moving focus on a turret change is a
            # surprise until the operator has seen the measurement.
            "parfocal_auto_apply": False,
            # v7.17 — close the epi (excitation) shutter for the duration of a
            # filter-cassette rotation, then restore whatever it was.
            #
            # OFF by default, for a reason that is about correctness rather than
            # timidity: the shutter's open/closed ENCODING is not yet verified on
            # hardware (see MicroscopeControl.NikonTiSdkBackend._shutter_codes).
            # If it is inverted on this body, an interlock would OPEN the shutter
            # for the rotation — exactly the exposure it exists to prevent. Verify
            # the direction on the body, then enable. Same shape as
            # parfocal_auto_apply above.
            "filter_shutter_interlock": False,
            # Operator override for that encoding, so a body which disagrees with
            # the SDK's declared range is a checkbox and not a code change — the
            # treatment focus_up_is_positive and plate_flip_180 already get.
            "epi_shutter_invert": False,
        }

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
        except Exception as exc:
            logger.warning(
                f"MicroscopeConfigStore: failed to load {self._path}: {exc}")
            return
        if not isinstance(loaded, dict):
            return
        # Merge over the blank template so a partial / older file still yields a
        # complete, well-typed structure.
        data = self._blank()
        for key, value in loaded.items():
            if key not in data:
                continue  # ignore unknown keys rather than choking on them
            data[key] = value
        data["backend"] = (data.get("backend")
                           if data.get("backend") in BACKENDS else "simulated")
        data["filter_slots"] = _clean_slots(
            data.get("filter_slots"), _DEFAULT_FILTER_SLOTS)
        data["objective_slots"] = _clean_slots(
            data.get("objective_slots"), _DEFAULT_OBJECTIVE_SLOTS)
        for key in ("filter_cubes", "objectives"):
            raw = data.get(key)
            data[key] = ({str(k): str(v) for k, v in raw.items()}
                         if isinstance(raw, dict) else {})
        # v7.17 — nested {name: {emission_nm, excitation_nm}}, so it cannot go
        # through the {str: str} coercion above. A malformed entry is DROPPED
        # rather than coerced: a wavelength that is not a positive number is
        # not recoverable into one, and absent is the honest answer.
        data["filter_optics"] = _clean_optics(data.get("filter_optics"))
        self._data = data

    def _write(self) -> None:
        """Atomic write. Caller holds ``_lock``."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_name(self._path.name + ".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"MicroscopeConfigStore: failed to save: {exc}")

    def save(self) -> None:
        with self._lock:
            self._write()

    # ── Generic access ────────────────────────────────────────────

    def get(self, key: str, default=None):
        with self._lock:
            return self._data.get(key, default)

    def set(self, key: str, value, *, save: bool = True) -> None:
        with self._lock:
            self._data[key] = value
            if save:
                self._write()

    def as_dict(self) -> dict:
        with self._lock:
            return json.loads(json.dumps(self._data))  # deep copy

    # ── Backend ───────────────────────────────────────────────────

    def get_backend(self) -> str:
        with self._lock:
            b = self._data.get("backend", "simulated")
            return b if b in BACKENDS else "simulated"

    def set_backend(self, backend: str) -> None:
        if backend not in BACKENDS:
            raise ValueError(f"unknown microscope backend: {backend!r}")
        self.set("backend", backend)

    def backend_kwargs(self) -> dict:
        """Constructor kwargs for the selected backend."""
        with self._lock:
            d = self._data
            backend = self.get_backend()
            if backend == "nikon_ti":
                return {
                    "prog_id": (d.get("prog_id") or None),
                    "z_units_per_um": float(
                        d.get("z_units_per_um") or _DEFAULT_Z_UNITS_PER_UM),
                    "cassette": int(d.get("cassette") or 1),
                }
            if backend == "micromanager":
                return {
                    "config_path": d.get("mm_config_path") or "",
                    "mm_dir": (d.get("mm_dir") or None),
                    "filter_device": d.get("mm_filter_device")
                    or "TIFilterBlock1",
                    "objective_device": d.get("mm_objective_device")
                    or "TINosePiece",
                    "focus_device": d.get("mm_focus_device") or "TIZDrive",
                    "epi_shutter_device": d.get("mm_epi_shutter_device")
                    or "TIEpiShutter",
                    "dia_lamp_device": d.get("mm_dia_lamp_device")
                    or "TIDiaLamp",
                    "light_path_device": d.get("mm_light_path_device")
                    or "TILightPath",
                }
            return {
                "filter_slots": int(d.get("filter_slots")
                                    or _DEFAULT_FILTER_SLOTS),
                "objective_slots": int(d.get("objective_slots")
                                       or _DEFAULT_OBJECTIVE_SLOTS),
            }

    # ── Slot counts ───────────────────────────────────────────────

    def filter_slots(self) -> int:
        with self._lock:
            return _clean_slots(self._data.get("filter_slots"),
                                _DEFAULT_FILTER_SLOTS)

    def objective_slots(self) -> int:
        with self._lock:
            return _clean_slots(self._data.get("objective_slots"),
                                _DEFAULT_OBJECTIVE_SLOTS)

    def set_filter_slots(self, n: int) -> None:
        self.set("filter_slots", _clean_slots(n, _DEFAULT_FILTER_SLOTS))

    def set_objective_slots(self, n: int) -> None:
        self.set("objective_slots", _clean_slots(n, _DEFAULT_OBJECTIVE_SLOTS))

    # ── Slot assignments ──────────────────────────────────────────

    def _labels(self, key: str, count: int) -> dict[int, str]:
        with self._lock:
            raw = self._data.get(key) or {}
            out: dict[int, str] = {}
            for pos in range(1, count + 1):
                name = str(raw.get(str(pos), "") or "").strip()
                if name:
                    out[pos] = name
            return out

    def filter_labels(self) -> dict[int, str]:
        """``{position: name}`` for every *named* filter slot (1-based)."""
        return self._labels("filter_cubes", self.filter_slots())

    def objective_labels(self) -> dict[int, str]:
        """``{position: name}`` for every *named* nosepiece position (1-based)."""
        return self._labels("objectives", self.objective_slots())

    def filter_label(self, position: int) -> str:
        return self.filter_labels().get(int(position), "")

    def objective_label(self, position: int) -> str:
        return self.objective_labels().get(int(position), "")

    def set_filter_label(self, position: int, name: str, *,
                         save: bool = True) -> None:
        self._set_label("filter_cubes", position, name, save=save)

    def set_objective_label(self, position: int, name: str, *,
                            save: bool = True) -> None:
        self._set_label("objectives", position, name, save=save)

    def _set_label(self, key: str, position: int, name: str, *,
                   save: bool) -> None:
        pos = int(position)
        if pos < 1:
            raise ValueError(f"slot positions are 1-based, got {position!r}")
        clean = str(name or "").strip()
        with self._lock:
            table = self._data.setdefault(key, {})
            if clean:
                table[str(pos)] = clean
            else:
                table.pop(str(pos), None)  # blank clears the assignment
            if save:
                self._write()

    # ── Filter-cube optics (v7.17, for the LabLink image-job sidecar) ──

    def filter_optics(self) -> dict:
        """``{cube name: {"emission_nm": .., "excitation_nm": ..}}``.

        Only cubes the operator has actually filled in appear. A cube with no
        entry is UNKNOWN, and stays unknown all the way to the sidecar.
        """
        with self._lock:
            return json.loads(json.dumps(self._data.get("filter_optics") or {}))

    def filter_optics_for(self, cube_name: str) -> dict:
        """One cube's optics, or ``{}`` if never recorded.

        Matched case-insensitively, because a scan's channel name comes from
        `FluorescenceMosaicStore.CHANNELS` while the cube name is typed by
        hand on the Microscope tab, and "FITC" vs "FITC " vs "fitc" should not
        silently cost a deconvolution its emission wavelength.
        """
        want = str(cube_name or "").strip().lower()
        if not want:
            return {}
        for name, entry in self.filter_optics().items():
            if name.strip().lower() == want:
                return dict(entry)
        return {}

    def set_filter_optics(self, cube_name: str, *,
                          emission_nm=None, excitation_nm=None,
                          save: bool = True) -> None:
        """Record one cube's wavelengths. Passing None for both clears it.

        Values outside a plausible band are REFUSED rather than clamped: a
        clamped wavelength is a fabricated number that looks measured, and
        this value silently changes an analysis result.
        """
        key = str(cube_name or "").strip()
        if not key:
            raise ValueError("a filter cube name is required")
        entry = {}
        for field, value in (("emission_nm", emission_nm),
                             ("excitation_nm", excitation_nm)):
            if value in (None, ""):
                continue
            nm = _clean_wavelength(value)
            if nm is None:
                raise ValueError(
                    f"{field}={value!r} is not a wavelength in nm "
                    f"(expected {_MIN_WAVELENGTH_NM:.0f}-{_MAX_WAVELENGTH_NM:.0f}). "
                    f"A green emission is about 519, not 0.519.")
            entry[field] = nm
        with self._lock:
            table = self._data.setdefault("filter_optics", {})
            if entry:
                table[key] = entry
            else:
                table.pop(key, None)
            if save:
                self._write()

    def set_all_filter_optics(self, optics: dict) -> None:
        """Replace the whole table in one write (unusable entries dropped)."""
        with self._lock:
            self._data["filter_optics"] = _clean_optics(optics)
            self._write()

    def set_filter_labels(self, labels: dict) -> None:
        """Replace every filter-cube assignment in one write."""
        self._set_labels("filter_cubes", labels)

    def set_objective_labels(self, labels: dict) -> None:
        """Replace every nosepiece assignment in one write."""
        self._set_labels("objectives", labels)

    def _set_labels(self, key: str, labels: dict) -> None:
        table = {}
        for pos, name in (labels or {}).items():
            try:
                p = int(pos)
            except (TypeError, ValueError):
                continue
            clean = str(name or "").strip()
            if p >= 1 and clean:
                table[str(p)] = clean
        with self._lock:
            self._data[key] = table
            self._write()

    # ── Focus preferences ─────────────────────────────────────────

    def focus_step_um(self) -> float:
        with self._lock:
            try:
                return max(0.001, float(self._data.get("focus_step_um")
                                        or _DEFAULT_FOCUS_STEP_UM))
            except (TypeError, ValueError):
                return _DEFAULT_FOCUS_STEP_UM

    def set_focus_step_um(self, step: float) -> None:
        self.set("focus_step_um", max(0.001, float(step)))

    def focus_up_is_positive(self) -> bool:
        with self._lock:
            return bool(self._data.get("focus_up_is_positive", True))

    def set_focus_up_is_positive(self, value: bool) -> None:
        self.set("focus_up_is_positive", bool(value))

    def focus_soft_limits_um(self) -> tuple[Optional[float], Optional[float]]:
        """Operator soft limits ``(min, max)``; either may be ``None``."""
        with self._lock:
            def _f(key):
                v = self._data.get(key)
                if v is None or v == "":
                    return None
                try:
                    return float(v)
                except (TypeError, ValueError):
                    return None
            lo, hi = _f("focus_min_um"), _f("focus_max_um")
        if lo is not None and hi is not None and lo > hi:
            lo, hi = hi, lo
        return lo, hi

    def set_focus_soft_limits_um(self, lo: Optional[float],
                                 hi: Optional[float]) -> None:
        with self._lock:
            self._data["focus_min_um"] = None if lo is None else float(lo)
            self._data["focus_max_um"] = None if hi is None else float(hi)
            self._write()

    # ── Illumination policy (v7.17) ────────────────────────────────

    def filter_shutter_interlock(self) -> bool:
        """Close the epi shutter while the filter cassette rotates?

        See the field's note in :meth:`_blank` for why this defaults off.
        """
        with self._lock:
            return bool(self._data.get("filter_shutter_interlock", False))

    def set_filter_shutter_interlock(self, value: bool) -> None:
        self.set("filter_shutter_interlock", bool(value))

    def epi_shutter_invert(self) -> bool:
        """Does this body's shutter report/accept open and closed the other way?"""
        with self._lock:
            return bool(self._data.get("epi_shutter_invert", False))

    def set_epi_shutter_invert(self, value: bool) -> None:
        self.set("epi_shutter_invert", bool(value))

    # ── Parfocality (per camera identity) ───────────────────────────

    def parfocal_block(self, camera_identity: str) -> dict:
        with self._lock:
            blk = self._data.get("parfocal_offsets_um") or {}
            return dict(blk.get(str(camera_identity)) or {})

    def set_parfocal(self, camera_identity: str, *, reference: str,
                     offsets_um: dict, centration_um: Optional[dict] = None,
                     product_codes: Optional[dict] = None) -> None:
        from datetime import datetime
        rec = {
            "reference": str(reference),
            "offsets_um": {str(k): float(v) for k, v in (offsets_um or {}).items()},
            "measured_at": datetime.now().isoformat(timespec="seconds"),
        }
        if centration_um:
            rec["centration_um"] = {
                str(k): [float(v[0]), float(v[1])]
                for k, v in centration_um.items() if v is not None}
        if product_codes:
            rec["product_codes"] = {str(k): str(v)
                                    for k, v in product_codes.items()}
        with self._lock:
            blk = self._data.setdefault("parfocal_offsets_um", {})
            if not isinstance(blk, dict):
                blk = self._data["parfocal_offsets_um"] = {}
            blk[str(camera_identity)] = rec
            self._write()

    def parfocal_offset_um(self, camera_identity: str, objective_name: str,
                           product_code: Optional[str] = None
                           ) -> tuple[Optional[float], str]:
        """Stored focus offset for one objective. ``(offset_um, why_not)``.

        Returns ``(None, why)`` when the objective at that name has been
        physically SWAPPED since the measurement — detected from the body's own
        product code, which is the strongest invalidation signal available: the
        hardware tells us, so no operator discipline is required.
        """
        blk = self.parfocal_block(camera_identity)
        offsets = blk.get("offsets_um") or {}
        if str(objective_name) not in offsets:
            return (None, f"no parfocal offset measured for "
                          f"'{objective_name}' on this camera")
        if product_code:
            known = (blk.get("product_codes") or {}).get(str(objective_name))
            if known and str(known) != str(product_code):
                return (None,
                        f"the objective at '{objective_name}' is now "
                        f"{product_code} but the parfocal offset was measured "
                        f"with {known} — it was physically swapped, so the "
                        f"offset no longer applies. Re-measure it.")
        try:
            return (float(offsets[str(objective_name)]), "")
        except (TypeError, ValueError):
            return (None, "stored parfocal offset is malformed")

    def parfocal_auto_apply(self) -> bool:
        with self._lock:
            return bool(self._data.get("parfocal_auto_apply", False))

    def set_parfocal_auto_apply(self, value: bool) -> None:
        self.set("parfocal_auto_apply", bool(value))


# ── Module-level singleton ──────────────────────────────────────────

_store: Optional[MicroscopeConfigStore] = None


def get_store(path: Optional[Path] = None) -> MicroscopeConfigStore:
    """Return the module-level singleton :class:`MicroscopeConfigStore`."""
    global _store
    if _store is None:
        _store = MicroscopeConfigStore(path)
    return _store


def reset_store() -> None:
    """Drop the singleton (test isolation after changing the env override)."""
    global _store
    _store = None
