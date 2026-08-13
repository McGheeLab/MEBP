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

from SupportClasses.MachineConfig import resolve_machine_path
from SupportClasses.OpticsRegistry import normalize_optic_name

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
    return resolve_machine_path(_DEFAULT_FILENAME)


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


#: A bandpass FWHM in nm. 0/absent means "width not recorded" — a center with
#: no width is still useful (it is what the LabLink sidecar consumes), so an
#: unknown width must not invent one.
_MAX_BANDWIDTH_NM = 600.0

#: Where a wavelength came from, weakest first. Mirrors
#: ``FilterCubeStore.PROVENANCES`` — the catalogue is the writer, this is the
#: store, and neither may silently promote a value it merely read.
_PROVENANCES = ("nominal", "datasheet", "measured")


#: Narrowest plausible bandpass FWHM. Exists to catch the SAME unit slip the
#: wavelength band guards against: a 40 nm band typed in µm is 0.04, which is
#: otherwise a perfectly well-formed positive number. Real narrowband filters
#: bottom out around 1-2 nm, so 1 nm is generous.
_MIN_BANDWIDTH_NM = 1.0


def clean_bandwidth(value):
    """A plausible bandpass width in nm, or None. Never raises."""
    try:
        nm = float(value)
    except (TypeError, ValueError):
        return None
    if not (_MIN_BANDWIDTH_NM <= nm <= _MAX_BANDWIDTH_NM):
        return None
    return nm


def _clean_optics(raw) -> dict:
    """``{cube name: {emission_nm, excitation_nm, …}}``, dropping the unusable.

    Entries with no usable wavelength are removed entirely, so "present but
    empty" can never be mistaken for "measured".

    v7.18 additions are all OPTIONAL and purely additive — the band CENTERS keep
    their original ``emission_nm`` / ``excitation_nm`` keys and meaning, because
    those are what every existing consumer reads (``OpticsRegistry.resolve_slots``
    and the ``lablink.imagejob/1`` sidecar, which carries one number per
    channel). A pre-v7.18 entry therefore round-trips byte-identically:

    * ``*_width_nm`` — bandpass FWHM, so the real RANGE is recoverable;
    * ``dichroic_nm`` — the beamsplitter EDGE (not a band);
    * ``provenance``  — nominal / datasheet / measured, so an auto-filled
      catalogue value can never be mistaken for a verified one;
    * ``cube_id``     — which ``FilterCubeStore`` entry filled this in.

    ⚠ A width or dichroic is dropped when there is no matching center: a width
    alone describes no band, and keeping it would imply knowledge we lack.
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
                width = clean_bandwidth(entry.get(f"{field[:-3]}_width_nm"))
                if width is not None:
                    clean[f"{field[:-3]}_width_nm"] = width
        if not clean:
            # No usable band at all — a cube_id/provenance on its own says
            # nothing measurable, so the whole entry goes.
            continue
        dichroic = _clean_wavelength(entry.get("dichroic_nm"))
        if dichroic is not None:
            clean["dichroic_nm"] = dichroic
        # ⚠ ABSENT is a third state, distinct from "nominal": it means the
        # provenance was never recorded (every pre-v7.18 entry, hand-typed by an
        # operator from their own knowledge). Stamping those "nominal" would
        # both break the byte-identical round-trip promised above and understate
        # a value the operator may well have taken off a datasheet. Only a
        # provenance actually present is kept, and an unrecognised one degrades
        # to the weakest claim rather than being promoted.
        raw_prov = entry.get("provenance")
        if raw_prov not in (None, ""):
            prov = str(raw_prov).strip().lower()
            clean["provenance"] = (prov if prov in _PROVENANCES
                                   else _PROVENANCES[0])
        cube_id = str(entry.get("cube_id") or "").strip()
        if cube_id:
            clean["cube_id"] = cube_id
        out[key] = clean
    return out


def _clean_objective_specs(raw) -> dict:
    """``{objective name: {optics}}``, dropping anything unusable.

    v7.18. An entry with neither NA nor working distance is removed entirely, so
    "present but empty" can never be mistaken for "known" — the two figures are
    what size the focus sweep and bound a nosepiece rotation, and an entry that
    carries only a provenance says nothing measurable.
    """
    from SupportClasses.ObjectiveCatalogue import (
        MAX_COVERSLIP_MM, MAX_FIELD_NUMBER_MM, clean_immersion,
        clean_magnification, clean_na, clean_optional_mm, clean_provenance,
        clean_wd_mm)

    out: dict = {}
    if not isinstance(raw, dict):
        return out
    for name, entry in raw.items():
        key = str(name).strip()
        if not key or not isinstance(entry, dict):
            continue
        clean: dict = {}
        na = clean_na(entry.get("numerical_aperture"))
        if na is not None:
            clean["numerical_aperture"] = na
        wd = clean_wd_mm(entry.get("working_distance_mm"))
        if wd is not None:
            clean["working_distance_mm"] = wd
        if not clean:
            continue
        mag = clean_magnification(entry.get("magnification"))
        if mag is not None:
            clean["magnification"] = mag
        for field, ceiling in (("coverslip_mm", MAX_COVERSLIP_MM),
                               ("field_number_mm", MAX_FIELD_NUMBER_MM)):
            v = clean_optional_mm(entry.get(field), ceiling)
            if v is not None:
                clean[field] = v
        clean["immersion"] = clean_immersion(entry.get("immersion"))
        clean["provenance"] = clean_provenance(entry.get("provenance"))
        for field in ("objective_id", "product_code"):
            v = str(entry.get(field) or "").strip()
            if v:
                clean[field] = v
        out[key] = clean
    return out


def _clean_aliases(raw) -> dict:
    """``{kind: {alias: slot name}}``, dropping anything unusable.

    v7.18. Only the two real turrets are kept, and an alias is dropped unless
    both sides are non-empty strings — a half-written alias would otherwise read
    as configured while resolving to nothing.
    """
    out: dict = {}
    if not isinstance(raw, dict):
        return out
    for kind, table in raw.items():
        k = str(kind).strip().lower()
        if k not in ("filter", "objective") or not isinstance(table, dict):
            continue
        clean = {}
        for alias, target in table.items():
            a = str(alias).strip()
            t = str(target).strip()
            if a and t:
                clean[a] = t
        if clean:
            out[k] = clean
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
            # v7.18 — operator-declared equivalences, {requested name: slot name},
            # per turret. The app's imaging-channel vocabulary and the label on
            # the cube physically in the cassette need not match: this rig holds a
            # cube labelled "TxRed" while the channel vocabulary (and the built-in
            # target-type rules) say "mCherry". Case and spacing are handled by
            # normalization and need NO alias; this is only for genuinely
            # different names, which the software must never equate on its own —
            # a TxRed image filed as an mCherry channel is a result nothing
            # downstream can detect. Per-machine because "the red cube in slot 3
            # is the one we use for mCherry" is a fact about THIS cassette.
            #   {"filter": {"mCherry": "TxRed"}, "objective": {}}
            "optic_aliases": {},
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
            # v7.18 — per-objective optics, keyed by the objective's NAME for the
            # same reason filter_optics is keyed by the cube's: they belong to the
            # lens and travel with it between nosepiece positions.
            #   {"4X": {"numerical_aperture": 0.13, "working_distance_mm": 17.1,
            #           "immersion": "air", "provenance": "datasheet", ...}}
            # ⚠ These OVERRIDE what the body reports, because the body only knows
            # the product code programmed into it — which can be blank, or a
            # different variant of the same nominal name. NA sizes every focus
            # step and working distance IS the collision bound, so an operator who
            # has read the engraving knows better than the nosepiece EEPROM. When
            # the two disagree, OpticsRegistry keeps the SHORTER working distance
            # (the fail-safe direction) and reports the disagreement.
            "objective_specs": {},
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
        # v7.18 — nested {kind: {alias: slot name}}; same reasoning as above, a
        # malformed entry is dropped rather than coerced. An alias that does not
        # name a real slot is left in place here (the slot may simply not be
        # named yet) and refused at resolution time, where the turret is known.
        data["optic_aliases"] = _clean_aliases(data.get("optic_aliases"))
        data["objective_specs"] = _clean_objective_specs(
            data.get("objective_specs"))
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

    # ── Optic name aliases (v7.18) ─────────────────────────────────

    def optic_aliases(self, kind: str) -> dict:
        """``{requested name: slot name}`` for one turret ('filter'/'objective').

        Consumed by ``OpticsRegistry.find_slot``'s alias tier. Case and spacing
        are already handled by normalization, so an alias is only ever needed for
        two genuinely different names (mCherry ⇄ TxRed).
        """
        k = str(kind).strip().lower()
        with self._lock:
            table = (self._data.get("optic_aliases") or {}).get(k) or {}
            return dict(table)

    def set_optic_alias(self, kind: str, alias: str, slot_name: str, *,
                        save: bool = True) -> None:
        """Declare that ``alias`` means the cube/objective labelled ``slot_name``.

        ⚠ **Refuses a target that is not currently a named slot on that turret.**
        An alias pointing at nothing reads as configured and then fails at the
        moment a run needs it — the operator would see "mCherry is aliased" and
        still get a refusal. Failing here, while they are looking at the setting,
        is the recoverable half.

        Passing an empty ``slot_name`` clears the alias (mirrors ``_set_label``).
        """
        k = str(kind).strip().lower()
        if k not in ("filter", "objective"):
            raise ValueError(f"kind must be 'filter' or 'objective', got {kind!r}")
        a = str(alias or "").strip()
        if not a:
            raise ValueError("alias must not be empty")
        t = str(slot_name or "").strip()
        if not t:
            self.clear_optic_alias(k, a, save=save)
            return

        labels = (self.filter_labels() if k == "filter"
                  else self.objective_labels())
        names = list(labels.values())
        norm = normalize_optic_name(t)
        match = [n for n in names if normalize_optic_name(n) == norm]
        if not match:
            have = ", ".join(f"{n!r} ({p})" for p, n in sorted(labels.items()))
            raise ValueError(
                f"Cannot alias {a!r} to {t!r}: no {k} slot is named that. "
                f"This turret holds {have or 'nothing named'}. Name the slot "
                f"first on Hardware Setup → Microscope.")
        if normalize_optic_name(a) == norm:
            raise ValueError(
                f"{a!r} and {t!r} are already the same name once case and "
                f"spacing are ignored — no alias is needed.")

        with self._lock:
            table = self._data.setdefault("optic_aliases", {})
            if not isinstance(table, dict):
                table = self._data["optic_aliases"] = {}
            table.setdefault(k, {})[a] = match[0]
            if save:
                self._write()
        logger.info("Optic alias set: %s %r -> %r", k, a, match[0])

    def clear_optic_alias(self, kind: str, alias: str, *,
                          save: bool = True) -> None:
        """Forget one alias. Silent when it was not set."""
        k = str(kind).strip().lower()
        a = str(alias or "").strip()
        with self._lock:
            table = (self._data.get("optic_aliases") or {}).get(k)
            if isinstance(table, dict) and table.pop(a, None) is not None:
                if save:
                    self._write()
                logger.info("Optic alias cleared: %s %r", k, a)

    # ── Objective optics (v7.18) ──────────────────────────────────

    def objective_specs(self) -> dict:
        """``{objective name: {magnification, numerical_aperture, ...}}``.

        Only objectives the operator has actually assigned appear. An objective
        with no entry is UNKNOWN and falls back to whatever the body reports.

        Keyed by the objective's NAME rather than its nosepiece position — same
        reasoning as ``filter_optics``: the optics are a property of the lens and
        travel with it if it is moved to another position.
        """
        with self._lock:
            return json.loads(
                json.dumps(self._data.get("objective_specs") or {}))

    def objective_spec_for(self, objective_name: str) -> dict:
        """One objective's optics, or ``{}`` if never recorded.

        Matched case-insensitively: the nosepiece label is typed by hand (this
        rig has "4X" against a calibration keyed "4x"), and a spelling difference
        must not silently cost the objective its working distance — which is the
        collision bound.
        """
        want = normalize_optic_name(objective_name)
        if not want:
            return {}
        for name, entry in self.objective_specs().items():
            if normalize_optic_name(name) == want:
                return dict(entry)
        return {}

    def set_objective_spec(self, objective_name: str, *,
                           magnification=None, numerical_aperture=None,
                           working_distance_mm=None, immersion=None,
                           coverslip_mm=None, field_number_mm=None,
                           objective_id=None, product_code=None,
                           provenance=None, save: bool = True) -> None:
        """Record one objective's optics. Passing nothing usable clears it.

        ⚠ Out-of-band values are REFUSED, not clamped, and the working-distance
        refusal is the one that matters: ``WD_SWEEP_FRACTION`` of this number is
        how far the focus may travel and whether a nosepiece rotation is allowed
        at all, so a value that survived a unit slip (17100 for 17.1 mm) would
        authorise a 17-metre excursion. Clearing it instead makes
        ``plan_turret_change`` fall back to its tiny conservative budget and say
        so, which is recoverable.
        """
        from SupportClasses.ObjectiveCatalogue import (
            MAX_COVERSLIP_MM, MAX_FIELD_NUMBER_MM, MAX_NA, MAX_WD_MM, MIN_NA,
            MIN_WD_MM, clean_immersion, clean_magnification, clean_na,
            clean_optional_mm, clean_provenance, clean_wd_mm)

        key = str(objective_name or "").strip()
        if not key:
            raise ValueError("an objective name is required")

        entry: dict = {}
        na = clean_na(numerical_aperture) if numerical_aperture not in (None, "") else None
        if numerical_aperture not in (None, "") and na is None:
            raise ValueError(
                f"numerical_aperture={numerical_aperture!r} is not an NA "
                f"(expected {MIN_NA}-{MAX_NA}). A dry 20x is about 0.75, not 75.")
        if na is not None:
            entry["numerical_aperture"] = na

        wd = clean_wd_mm(working_distance_mm) if working_distance_mm not in (None, "") else None
        if working_distance_mm not in (None, "") and wd is None:
            raise ValueError(
                f"working_distance_mm={working_distance_mm!r} is not a working "
                f"distance in MILLIMETRES (expected {MIN_WD_MM}-{MAX_WD_MM}). "
                f"A Plan Fluor 4x is about 17.1, not 17100.")
        if wd is not None:
            entry["working_distance_mm"] = wd

        mag = clean_magnification(magnification) if magnification not in (None, "") else None
        if magnification not in (None, "") and mag is None:
            raise ValueError(f"magnification={magnification!r} is implausible")
        if mag is not None:
            entry["magnification"] = mag

        # ⚠ The NA/WD check comes BEFORE the trimmings. An entry carrying only a
        # coverslip thickness (or only a provenance) says nothing measurable, and
        # `_clean_objective_specs` drops exactly that on reload — so accepting one
        # here would look saved and silently vanish on the next launch.
        if not entry:
            with self._lock:
                table = self._data.setdefault("objective_specs", {})
                if isinstance(table, dict) and table.pop(key, None) is not None:
                    if save:
                        self._write()
            return

        for field, value, ceiling in (
                ("coverslip_mm", coverslip_mm, MAX_COVERSLIP_MM),
                ("field_number_mm", field_number_mm, MAX_FIELD_NUMBER_MM)):
            if value in (None, ""):
                continue
            v = clean_optional_mm(value, ceiling)
            if v is None:
                raise ValueError(f"{field}={value!r} is implausible")
            entry[field] = v

        if not entry:
            with self._lock:
                table = self._data.setdefault("objective_specs", {})
                if isinstance(table, dict) and table.pop(key, None) is not None:
                    if save:
                        self._write()
            return

        entry["immersion"] = clean_immersion(immersion)
        entry["provenance"] = clean_provenance(provenance)
        for field, value in (("objective_id", objective_id),
                             ("product_code", product_code)):
            v = str(value or "").strip()
            if v:
                entry[field] = v

        with self._lock:
            table = self._data.setdefault("objective_specs", {})
            if not isinstance(table, dict):
                table = self._data["objective_specs"] = {}
            table[key] = entry
            if save:
                self._write()

    def set_all_objective_specs(self, specs: dict) -> None:
        """Replace the whole table (the setup panel's Save)."""
        with self._lock:
            self._data["objective_specs"] = _clean_objective_specs(specs)
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
                          emission_width_nm=None, excitation_width_nm=None,
                          dichroic_nm=None, provenance=None, cube_id=None,
                          save: bool = True) -> None:
        """Record one cube's optics. Passing no usable band clears the entry.

        Values outside a plausible band are REFUSED rather than clamped: a
        clamped wavelength is a fabricated number that looks measured, and
        this value silently changes an analysis result.

        v7.18: the ``*_width_nm`` FWHMs make the real RANGE recoverable,
        ``dichroic_nm`` records the beamsplitter edge, and ``provenance`` says
        whether the numbers are nominal-for-the-type or verified. All optional —
        omitting every one of them writes exactly the pre-v7.18 entry.
        """
        key = str(cube_name or "").strip()
        if not key:
            raise ValueError("a filter cube name is required")
        entry = {}
        for field, value in (("emission_nm", emission_nm),
                             ("excitation_nm", excitation_nm),
                             ("dichroic_nm", dichroic_nm)):
            if value in (None, ""):
                continue
            nm = _clean_wavelength(value)
            if nm is None:
                raise ValueError(
                    f"{field}={value!r} is not a wavelength in nm "
                    f"(expected {_MIN_WAVELENGTH_NM:.0f}-{_MAX_WAVELENGTH_NM:.0f}). "
                    f"A green emission is about 519, not 0.519.")
            entry[field] = nm
        for field, value in (("emission_width_nm", emission_width_nm),
                             ("excitation_width_nm", excitation_width_nm)):
            if value in (None, ""):
                continue
            nm = clean_bandwidth(value)
            if nm is None:
                raise ValueError(
                    f"{field}={value!r} is not a bandpass width in nm "
                    f"(expected {_MIN_BANDWIDTH_NM:.0f}-{_MAX_BANDWIDTH_NM:.0f}). "
                    f"A 40 nm-wide band is 40, not 0.04.")
            entry[field] = nm
        if provenance not in (None, ""):
            entry["provenance"] = provenance
        if cube_id not in (None, ""):
            entry["cube_id"] = str(cube_id).strip()
        # One cleaner owns the shape, so a direct set and a whole-table replace
        # cannot disagree about what a valid entry looks like (this is also what
        # drops a width with no center, and any junk provenance).
        cleaned = _clean_optics({key: entry}) if entry else {}
        entry = cleaned.get(key, {})
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
