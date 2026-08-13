"""
NeedleBoreCalibrationStore.py — per-MOUNT lateral/axial offsets of each needle bore.

v7.9: A multi-bore assembly (backpack / triple) has several lumens fused side by
side, ~100-500 µm apart laterally and coplanar to within ~50 µm axially (decision
D7). The software knows exactly ONE needle position — ``needle_origin_um``, taught
by centring *a* tip in the two side cameras — so without these offsets, putting
"bore 1" on a target leaves bore 2 a few hundred µm away, which is **wider than a
cell**, with no error reported. That is what this store measures.

WHY A CALIBRATION AND NOT A CONFIG FIELD
----------------------------------------
A fused assembly's bores form a line (backpack) or a triangle (triple), but the
assembly's **rotation about Z in the holder is arbitrary** — so the offsets are not
a manufacturing constant of the needle *type*. They must be re-measured on every
needle change or re-seat, and they belong beside the needle-location calibration
(per-machine), **never** in ``NeedleTypeStore``: a preset library is copied between
rigs and a setup file is swapped between runs, so either would carry one mount's
geometry onto another mount's needle. That is the ``CAMERA_CAL_PERSIST_STORE``
lesson recorded in CLAUDE.md, and it is the same reason ``NeedleType`` is
STAMP-DON'T-REFERENCE.

``NeedleBore`` carries ``offset_um`` / ``z_offset_mm`` so the motion path can read
them off the live needle, but **this store is the authority**;
:meth:`NeedleBoreCalibrationStore.apply_to_needle` pushes them onto the live
``NeedleSpec`` after a measurement and at load.

THE SIGN CONVENTION — fixed once, pinned by a round-trip test
------------------------------------------------------------
::

    to place bore k on target T:   stage_xy = T - offset_um(k)
    bore k currently sits at:      stage_xy + offset_um(k)
    bore 0 is the datum:           offset_um(0) == (0.0, 0.0)

which makes the MEASUREMENT a plain subtraction of the two centred stage
positions. Centring bore k on the crosshair C puts the stage at ``S_k``, and by
the convention above ``C = S_k + offset_k``; centring the datum bore gives
``C = S_0``. Therefore::

    offset_um(k) = S_0 - S_k          (:func:`offset_from_centred_positions`)

Worked example, because a mis-signed offset is a *right-distance-wrong-way* error —
the same failure class as the plate-orientation bugs in CLAUDE.md, and it looks like
a calibration problem rather than a bug. If bore 1 physically sticks out 320 µm
toward +X, the stage must sit 320 µm toward −X for bore 1 to reach the crosshair, so
``S_1 = S_0 - 320`` and ``offset_1 = S_0 - S_1 = +320``. Placing it on a target then
commands ``stage = T - 320`` — the stage backs off in −X and the bore, which
protrudes in +X, lands on T. ✔

Z uses the HEIGHT frame (up = +), and ``z_offset_mm`` is positive when a bore
reaches **LOWER** than the datum. A longer bore must be met by a HIGHER stage for
its tip to reach the same crosshair height, so::

    z_offset_mm(k) = Z_k - Z_0        (both in the user/height frame)

Consumption (``PickPlaceExecutor._bore_z_mm``) adds ``z_up_sign · z_offset_mm`` to
the zero-ref Z, i.e. raises the stage by that height — correct on both polarities.

Data file: ``config/hardware/needle_bore_calibration.json``::

    {
      "version": "1.0",
      "measured_at": "2026-08-01T10:15:00",
      "fingerprint": {
        "needle_form": "backpack",
        "bore_count": 2,
        "bores": [{"pump_id": "P1", "orifice_id_um": 413.0, "total_length_mm": 50.8},
                  {"pump_id": "P2", "orifice_id_um": 159.0, "total_length_mm": 50.8}]
      },
      "bores": [
        {"bore_index": 0, "offset_um": [0.0, 0.0], "z_offset_mm": 0.0,
         "stage_um": [105617.6, 65890.4], "z_user_mm": 22.564,
         "measured_at": "2026-08-01T10:14:12"},
        {"bore_index": 1, "offset_um": [320.0, -140.0], "z_offset_mm": 0.04, ...}
      ]
    }

``stage_um`` / ``z_user_mm`` are provenance: they are what the operator actually
centred, so a suspect offset can be re-derived and checked by hand rather than
being an unexplainable number.

Zero GUI dependencies (json + dataclasses only). Set
``MEBP_NEEDLE_BORE_CAL_PATH`` to redirect the file (test isolation).
"""

from __future__ import annotations

import json
import logging
import math
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

_DEFAULT_PATH = resolve_machine_path("needle_bore_calibration.json")

# Fingerprint dimensions → human label, in the order they are reported.
_FINGERPRINT_LABELS = {
    "needle_form": "needle form",
    "bore_count": "bore count",
    "bores": "bore geometry",
}


# ── the sign convention, as one pure function ───────────────────────

def offset_from_centred_positions(
    datum_stage_um: Sequence[float],
    bore_stage_um: Sequence[float],
) -> tuple[float, float]:
    """``offset_um = datum_stage - bore_stage`` — see the module docstring.

    Both arguments are the stage XY (µm, absolute) at which that bore's tip sat on
    the side-camera crosshairs. The subtraction order is the whole calibration, so
    it lives in ONE named function that the GUI and the tests both call rather
    than being written out at a call site where it could be flipped.
    """
    return (float(datum_stage_um[0]) - float(bore_stage_um[0]),
            float(datum_stage_um[1]) - float(bore_stage_um[1]))


def z_offset_from_centred_heights(datum_z_user_mm: float,
                                  bore_z_user_mm: float) -> float:
    """``z_offset_mm = bore_height - datum_height`` (+ ⇒ reaches LOWER).

    Both in the USER/height frame (up = +). A bore that protrudes further needs a
    HIGHER stage to bring its tip to the same crosshair, so its centred height is
    the larger number and the offset comes out positive — which is what
    ``_bore_z_mm`` then adds, raising the stage so the long bore is not driven
    into the glass.
    """
    return float(bore_z_user_mm) - float(datum_z_user_mm)


# ── v7.10: the MICROSCOPE measurement ───────────────────────────────────────
#
# The side-camera method above jogs each bore onto a crosshair and subtracts the
# two stage positions. The microscope method instead parks ONCE and clicks every
# bore's tip in a single frame: faster, needs no needle-cam calibration, and —
# because the stage never moves between clicks — the unknown microscope-to-
# needle registration cancels out exactly.
#
# ⚠⚠ THE ARGUMENT ORDER IS THE OPPOSITE OF `offset_from_centred_positions`, AND
# THAT IS CORRECT. This is the single most dangerous line in the file, so the
# derivation is written out rather than asserted.
#
# The ONE fact both this and the pick workflow rest on is the code-verified
# contract of `CameraManager.pixel_to_stage_offset` (call it `pto`), used
# identically at every live-view click site in the app:
#
#     a PLATE feature seen at pixel P has stage-coordinate label
#         c(P) = current_xy + pto(P)
#
# Bore k's tip appears at pixel P_k. Whatever plate point lies directly under
# that tip is the plate feature at P_k, so at stage S bore k is over the plate
# point labelled  S + pto(P_k).
#
# To put bore k on a target whose label is T:   S + pto(P_k) = T
#                                          ⟹   S = T − pto(P_k)
# and the store's convention is                S = T − offset_um(k)
#                                          ⟹   offset_um(k) = pto(P_k)
#
# Bore 0 is the datum and its offset is defined to be zero, so the recorded,
# datum-relative value is
#
#     offset_um(k) = pto(P_k) − pto(P_0)          ← bore MINUS datum
#
# whereas the side-camera form is `datum − bore`. The asymmetry is real: one
# subtracts stage positions the stage was DRIVEN to, the other subtracts
# pixel-derived labels of where the bores ALREADY are. Do not "tidy" them into
# the same order.
#
# WHY THIS IS FRAME-INDEPENDENT — and why an earlier attempt to settle it by
# measurement was abandoned. Whether the XY stage carries the plate or carries
# the needle+microscope head changes where things sit in the LAB frame, but both
# the measurement above and the target the pick workflow builds
# (`T = current_xy + pto(click)`) go through the same `pto` label space, so the
# lab frame never appears and cannot flip the answer. Reasoning via "the bore
# protrudes +320 µm in world +X" — as this module's own worked example does —
# introduces exactly that unobservable intermediate and is where the confusion
# comes from; the arithmetic contract (`offset = S_0 − S_k` composed with
# `stage = T − offset`) is what actually runs and is self-consistent either way.
# A probe move cannot settle it either: the needle and the microscope are on the
# same body in every configuration, so the tip never moves in that view.
#
# A mis-signed offset is a RIGHT-DISTANCE-WRONG-WAY error — the bore lands the
# correct distance from the cell, on the wrong side, missing by TWICE the
# spacing — and it reads as a calibration problem rather than a bug. So: one
# function, pinned by a composed end-to-end test against
# `PickAndPlaceManager._bore_target_xy_um`, plus an in-app cross-check against
# the side-camera method (the two must agree, sign included) and a bench check
# that drives every bore to one target and confirms DIRECTION, not just distance.


def offset_from_frame_clicks(datum_pto_um: Sequence[float],
                             bore_pto_um: Sequence[float],
                             ) -> tuple[float, float]:
    """``offset_um = bore_pto − datum_pto`` — see the block comment above.

    Both arguments are ``CameraManager.pixel_to_stage_offset(...)`` for the pixel
    at which that bore's tip appears, in ONE frame with the stage stationary.

    Feeds the same store field and obeys the same consumption convention as
    :func:`offset_from_centred_positions` (``stage = target − offset``), so a
    bore measured by both methods must give the same answer — that is the
    cross-check the wizard offers. The argument order differs from that function
    ON PURPOSE; the block comment derives why.

    The datum's own position cancels, so the datum bore need not be centred in
    the frame — only visible.
    """
    return (_finite(bore_pto_um[0]) - _finite(datum_pto_um[0]),
            _finite(bore_pto_um[1]) - _finite(datum_pto_um[1]))


def needle_camera_offset_from_click(datum_pto_um: Sequence[float],
                                    ) -> tuple[float, float]:
    """The datum bore's offset from the microscope camera centre (stage µm).

    Same click as the bore-0 measurement, answering a different question, so it
    is worth its own name. By the derivation above, at stage S the datum bore is
    over the plate point labelled ``S + pto(P_0)``; to put it on a feature
    labelled ``f`` the stage must go to ``f − pto(P_0)``. That matches
    ``StageController.needle_target_xy_for_feature_um``'s contract
    (``target = feature − offset``) with

        offset = pto(P_0)

    i.e. the raw click, with NO negation — unlike the bore offsets, which are a
    difference. This is the value that makes every live-view click-to-PICK land
    on the needle rather than under the crosshair, and it is why a SINGLE-bore
    needle also has something to measure here.

    Invariant worth testing: ``offset_um(k) == centre_offset(k) − centre_offset(0)``.
    """
    return (_finite(datum_pto_um[0]), _finite(datum_pto_um[1]))


def _finite(value, default: float = 0.0) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


@dataclass(frozen=True)
class BoreOffset:
    """One bore's measured mount offset (immutable — a stored measurement)."""

    bore_index: int
    offset_um: tuple[float, float] = (0.0, 0.0)
    z_offset_mm: float = 0.0
    # Provenance: the stage position / height that was actually centred.
    stage_um: Optional[tuple[float, float]] = None
    z_user_mm: Optional[float] = None
    measured_at: str = ""

    @property
    def is_datum(self) -> bool:
        return self.bore_index == 0

    @property
    def magnitude_um(self) -> float:
        return math.hypot(self.offset_um[0], self.offset_um[1])

    def to_dict(self) -> dict:
        d: dict = {
            "bore_index": int(self.bore_index),
            "offset_um": [round(self.offset_um[0], 3),
                          round(self.offset_um[1], 3)],
            "z_offset_mm": round(float(self.z_offset_mm), 5),
        }
        if self.stage_um is not None:
            d["stage_um"] = [round(self.stage_um[0], 3),
                             round(self.stage_um[1], 3)]
        if self.z_user_mm is not None:
            d["z_user_mm"] = round(float(self.z_user_mm), 4)
        if self.measured_at:
            d["measured_at"] = self.measured_at
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "BoreOffset":
        """Build from JSON, tolerating missing/garbage fields.

        A corrupt record degrades to "measured at the datum" — i.e. a zero offset,
        which is the pre-v7.9 behaviour — rather than raising and taking the whole
        calibration with it.
        """
        if not isinstance(data, dict):
            data = {}
        try:
            idx = int(data.get("bore_index", 0))
        except (TypeError, ValueError):
            idx = 0
        off = data.get("offset_um") or (0.0, 0.0)
        try:
            offset = (_finite(off[0]), _finite(off[1]))
        except (TypeError, IndexError, KeyError):
            offset = (0.0, 0.0)
        stage = data.get("stage_um")
        try:
            stage_um = (_finite(stage[0]), _finite(stage[1])) if stage else None
        except (TypeError, IndexError, KeyError):
            stage_um = None
        z_user = data.get("z_user_mm")
        return cls(
            bore_index=max(0, idx),
            offset_um=offset,
            z_offset_mm=_finite(data.get("z_offset_mm"), 0.0),
            stage_um=stage_um,
            z_user_mm=(None if z_user is None else _finite(z_user)),
            measured_at=str(data.get("measured_at") or ""),
        )

    def describe(self) -> str:
        """``Bore 2 · offset (+320, -140) µm · Z +0.040 mm`` for a readout/log."""
        if self.is_datum:
            return f"Bore {self.bore_index + 1} · datum (0, 0)"
        bits = [f"Bore {self.bore_index + 1}",
                f"offset ({self.offset_um[0]:+.0f}, {self.offset_um[1]:+.0f}) µm"]
        if self.z_offset_mm:
            bits.append(f"Z {self.z_offset_mm:+.3f} mm")
        return " · ".join(bits)


# ── assembly fingerprint ────────────────────────────────────────────

def build_fingerprint(needle) -> dict:
    """Identify the assembly these offsets were measured on.

    Deliberately NOT an attempt to detect a re-seat — rotating the SAME assembly
    in the holder is invisible to software and changes every offset, which is why
    the UI says so and offers a one-click clear. What this catches is the case
    software *can* catch: the stored offsets belonging to a **different assembly**
    than the one currently configured (a different form, a different bore count,
    or a bore re-plumbed to another pump / swapped for a different size).

    Duck-typed: a stub or MagicMock that cannot describe its bores yields an empty
    fingerprint, which :func:`fingerprint_diff` reports as "unchanged" rather than
    manufacturing a warning out of absent data.
    """
    if needle is None:
        return {}
    try:
        bores = list(needle.bores_resolved())
    except Exception:
        return {}
    rows = []
    for b in bores:
        try:
            rows.append({
                "pump_id": (str(b.pump_id) if getattr(b, "pump_id", None)
                            else None),
                "orifice_id_um": round(_finite(b.orifice_id_um), 3),
                "total_length_mm": round(_finite(b.total_length_mm), 4),
            })
        except Exception:
            # One unreadable bore makes the whole fingerprint untrustworthy —
            # a partial list would compare unequal for the wrong reason.
            return {}
    form = getattr(needle, "needle_form", None)
    return {
        "needle_form": (str(form) if form else None),
        "bore_count": len(rows),
        "bores": rows,
    }


def fingerprint_diff(saved: Optional[dict],
                     current: Optional[dict]) -> list[str]:
    """Human-readable list of changed dimensions (empty ⇒ unchanged).

    A missing saved fingerprint (older file) and a dimension ADDED after the file
    was written both report no diff — the same absent-key rule as
    ``CalibrationSnapshotStore.fingerprint_diff``, and for the same reason: a
    None-vs-value comparison would fire a bogus warning for every existing user
    the first time they launch a build that adds a dimension.
    """
    if not saved:
        return []
    current = current or {}
    if not current:
        return []
    diffs = []
    for key, label in _FINGERPRINT_LABELS.items():
        if key not in saved:
            continue                 # dimension added after this file was written
        s = saved.get(key)
        c = current.get(key)
        if s == c:
            continue
        if key == "bores":
            diffs.append(f"{label}: {_describe_bore_rows(s)} → "
                         f"{_describe_bore_rows(c)}")
        else:
            diffs.append(f"{label}: {s} → {c}")
    return diffs


def _describe_bore_rows(rows) -> str:
    if not isinstance(rows, list) or not rows:
        return "—"
    out = []
    for r in rows:
        if not isinstance(r, dict):
            out.append("?")
            continue
        pump = r.get("pump_id") or "—"
        out.append(f"{pump}/{_finite(r.get('orifice_id_um')):.0f}µm")
    return ", ".join(out)


# ── the store ───────────────────────────────────────────────────────

class NeedleBoreCalibrationStore:
    """Load/save the per-mount bore offsets (single record, atomic JSON)."""

    def __init__(self, path: Path | str | None = None):
        env = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")
        if path is not None:
            self._path = Path(path)
        elif env:
            self._path = Path(env)
        else:
            self._path = _DEFAULT_PATH
        self._data: dict = self._empty()
        self._load()

    @staticmethod
    def _empty() -> dict:
        return {"version": "1.0", "measured_at": None,
                "fingerprint": {}, "bores": []}

    @property
    def path(self) -> Path:
        return self._path

    # ── persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data = self._empty()
                self._data.update(loaded)
            if not isinstance(self._data.get("bores"), list):
                self._data["bores"] = []
            if not isinstance(self._data.get("fingerprint"), dict):
                self._data["fingerprint"] = {}
        except Exception as exc:
            logger.warning(
                "NeedleBoreCalibrationStore: failed to load %s: %s",
                self._path, exc)
            self._data = self._empty()

    def save(self) -> None:
        """Atomic write (tmp + ``os.replace``) so a crash cannot truncate it."""
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
            logger.error("NeedleBoreCalibrationStore: failed to save: %s", exc)

    # ── read ──────────────────────────────────────────────────────

    def get_fingerprint(self) -> dict:
        fp = self._data.get("fingerprint")
        return dict(fp) if isinstance(fp, dict) else {}

    def measured_at(self) -> Optional[str]:
        return self._data.get("measured_at") or None

    def all_bores(self) -> list[BoreOffset]:
        """Every stored record, bore order, datum first."""
        out = [BoreOffset.from_dict(r) for r in self._data.get("bores", [])
               if isinstance(r, dict)]
        out.sort(key=lambda b: b.bore_index)
        return out

    def get_bore(self, bore_index: int) -> Optional[BoreOffset]:
        try:
            want = int(bore_index)
        except (TypeError, ValueError):
            return None
        for rec in self.all_bores():
            if rec.bore_index == want:
                return rec
        return None

    def measured_bore_indices(self) -> list[int]:
        """Indices with a stored record (the datum counts — it is measured too)."""
        return [b.bore_index for b in self.all_bores()]

    def offset_um(self, bore_index: int) -> tuple[float, float]:
        """Stored offset, or ``(0, 0)`` when unmeasured — the fail-safe value.

        ``(0, 0)`` is what a single-bore needle has, so an unmeasured bore behaves
        exactly like the pre-v7.9 code (targets the datum) instead of moving to a
        made-up position.
        """
        rec = self.get_bore(bore_index)
        return rec.offset_um if rec is not None else (0.0, 0.0)

    def z_offset_mm(self, bore_index: int) -> float:
        rec = self.get_bore(bore_index)
        return rec.z_offset_mm if rec is not None else 0.0

    def fingerprint_matches(self, needle) -> bool:
        """True when the stored offsets belong to ``needle``'s assembly.

        An empty stored fingerprint (older file, or a needle that could not be
        described when it was written) counts as a match — see
        :func:`fingerprint_diff`.
        """
        return not fingerprint_diff(self.get_fingerprint(),
                                    build_fingerprint(needle))

    def is_calibrated(self, needle=None) -> bool:
        """True when at least one NON-datum bore has been measured.

        The datum alone says nothing — bore 0's offset is ``(0, 0)`` by
        definition — so a store holding only the datum is "not calibrated" and
        the UI keeps asking for the remaining bores. When ``needle`` is given the
        fingerprint must also match, so a swapped assembly reads as uncalibrated
        rather than silently reusing another needle's geometry.
        """
        if needle is not None and not self.fingerprint_matches(needle):
            return False
        return any(not b.is_datum for b in self.all_bores())

    def describe(self) -> str:
        recs = self.all_bores()
        if not recs:
            return "no bore offsets measured"
        return " | ".join(r.describe() for r in recs)

    # ── write ─────────────────────────────────────────────────────

    def set_bore(self, bore_index: int,
                 offset_um: Sequence[float] = (0.0, 0.0),
                 z_offset_mm: float = 0.0,
                 *,
                 stage_um: Optional[Sequence[float]] = None,
                 z_user_mm: Optional[float] = None,
                 needle=None,
                 save: bool = True) -> BoreOffset:
        """Record ONE bore's measured offset, replacing any previous record.

        Bore 0 is the datum **by definition**, so its offset is forced to
        ``(0, 0)`` / ``0.0`` regardless of what is passed: it is the point
        ``needle_origin_um`` names, and letting a caller store a non-zero datum
        offset would double-count it at every consumer (``bore_offset_um(0)`` is
        assumed zero throughout the motion path).

        Passing ``needle`` re-stamps the assembly fingerprint, which is how a
        measurement session claims the store for the currently-configured
        assembly.
        """
        try:
            idx = max(0, int(bore_index))
        except (TypeError, ValueError):
            idx = 0
        if idx == 0:
            off = (0.0, 0.0)
            dz = 0.0
        else:
            try:
                off = (_finite(offset_um[0]), _finite(offset_um[1]))
            except (TypeError, IndexError, KeyError):
                off = (0.0, 0.0)
            dz = _finite(z_offset_mm, 0.0)
        stamp = datetime.now().isoformat(timespec="seconds")
        rec = BoreOffset(
            bore_index=idx,
            offset_um=off,
            z_offset_mm=dz,
            stage_um=(None if stage_um is None
                      else (_finite(stage_um[0]), _finite(stage_um[1]))),
            z_user_mm=(None if z_user_mm is None else _finite(z_user_mm)),
            measured_at=stamp,
        )
        rows = [r for r in self._data.get("bores", [])
                if isinstance(r, dict) and r.get("bore_index") != idx]
        rows.append(rec.to_dict())
        rows.sort(key=lambda r: r.get("bore_index", 0))
        self._data["bores"] = rows
        self._data["measured_at"] = stamp
        if needle is not None:
            fp = build_fingerprint(needle)
            if fp:
                self._data["fingerprint"] = fp
        if save:
            self.save()
        logger.info("Needle bore offset stored: %s", rec.describe())
        return rec

    def set_offsets(self, offsets: dict[int, BoreOffset], *, needle=None) -> None:
        """Replace the WHOLE record set in one write (bore 0 forced to datum)."""
        self._data["bores"] = []
        for idx in sorted(offsets):
            rec = offsets[idx]
            self.set_bore(idx, rec.offset_um, rec.z_offset_mm,
                          stage_um=rec.stage_um, z_user_mm=rec.z_user_mm,
                          needle=None, save=False)
        if needle is not None:
            fp = build_fingerprint(needle)
            if fp:
                self._data["fingerprint"] = fp
        self.save()

    def clear(self) -> None:
        """Forget every measurement (what a re-seat requires — see the docstring)."""
        self._data = self._empty()
        self.save()
        logger.info("Needle bore offsets cleared — re-measure after a re-seat.")

    def clear_bore(self, bore_index: int) -> bool:
        try:
            idx = int(bore_index)
        except (TypeError, ValueError):
            return False
        rows = [r for r in self._data.get("bores", [])
                if isinstance(r, dict) and r.get("bore_index") != idx]
        if len(rows) == len(self._data.get("bores", [])):
            return False
        self._data["bores"] = rows
        self.save()
        return True

    # ── apply to the live needle ──────────────────────────────────

    def apply_to_needle(self, needle, *, force: bool = False) -> int:
        """RECONCILE the live ``NeedleSpec.bores`` with this store, in place.

        ``NeedleBore`` carries the offsets so the executor can read them off the
        needle it already has; this is the one place they get there. Returns how
        many bores were written from a stored MEASUREMENT (0 when there is
        nothing measured to apply).

        ⚠ AUTHORITATIVE IN BOTH DIRECTIONS — clearing matters as much as
        applying. ``NeedleBore.offset_um`` / ``z_offset_mm`` round-trip through
        ``NeedleSpec.to_dict``, so they end up inside the **swappable** hardware
        setup file (and the ``settings.json`` mirror of it). A setup saved on
        another rig, or before a re-seat, therefore ARRIVES carrying that mount's
        offsets, and nothing in it can be vouched for by THIS machine's store.
        Merely declining to apply would leave those foreign numbers live:
        the executor would command a real 100-500 µm error (wider than a cell,
        in a confidently-wrong direction) while :meth:`is_calibrated`, the
        Needle-Location rows and the status line all correctly reported the bore
        as unmeasured. That is the ``CAMERA_CAL_PERSIST_STORE`` failure mode
        this store exists to prevent, so every non-datum bore is written on
        every call — from its stored record when there is one, and from the
        fail-safe ``(0, 0)`` / ``0.0`` when there is not.

        On a fingerprint mismatch (unless ``force``) NOTHING is applied and
        every non-datum bore is CLEARED: pushing one assembly's geometry onto a
        different one would drive the stage a real, confidently-wrong distance,
        and an unmeasured bore targeting the datum is the pre-v7.9 behaviour.

        Bore 0 is never written, and ``NeedleSpec.__post_init__`` re-forces it
        to the datum anyway — so a stray non-zero datum in the file cannot leak
        into motion.
        """
        if needle is None:
            return 0
        bores = getattr(needle, "bores", None)
        if not bores:
            # A synthesized single-bore needle has nothing to write onto (and
            # nothing to correct — __post_init__ pins bore 0 at the datum).
            return 0
        stored: dict[int, BoreOffset] = {}
        if force or self.fingerprint_matches(needle):
            stored = {r.bore_index: r for r in self.all_bores()}
        else:
            logger.warning(
                "NeedleBoreCalibrationStore: stored offsets were measured on a "
                "different assembly (%s) — NOT applied; every bore offset "
                "cleared to the datum. Re-measure the bore offsets.",
                "; ".join(fingerprint_diff(self.get_fingerprint(),
                                           build_fingerprint(needle))) or "?")
        applied = 0
        cleared = 0
        for k in range(1, len(bores)):
            rec = stored.get(k)
            off = rec.offset_um if rec is not None else (0.0, 0.0)
            dz = rec.z_offset_mm if rec is not None else 0.0
            # Read the previous value first, purely so a discarded foreign offset
            # can be REPORTED — a silent overwrite of a real number is exactly
            # what made this hazard invisible. Best-effort: a duck-typed bore
            # that cannot be read must still be written.
            try:
                prev = (tuple(bores[k].offset_um), float(bores[k].z_offset_mm))
            except Exception:
                prev = None
            try:
                bores[k].offset_um = (float(off[0]), float(off[1]))
                bores[k].z_offset_mm = float(dz)
            except (AttributeError, TypeError, ValueError):
                continue
            if rec is not None:
                applied += 1
            elif prev is not None and prev != ((0.0, 0.0), 0.0):
                cleared += 1
        if applied:
            logger.info("Applied %d measured bore offset(s) to the live needle: %s",
                        applied, self.describe())
        if cleared:
            logger.warning(
                "Discarded %d unvouched bore offset(s) that arrived with the "
                "hardware setup — mount offsets are per-machine and this store "
                "has no measurement for them. Re-measure on Calibration → "
                "Needle Location.", cleared)
        return applied


# ── module-level singleton ──────────────────────────────────────────

_store: Optional[NeedleBoreCalibrationStore] = None


def get_store(path: Path | str | None = None) -> NeedleBoreCalibrationStore:
    """Process-wide singleton (mirrors the other calibration stores)."""
    global _store
    if _store is None:
        _store = NeedleBoreCalibrationStore(path)
    return _store
