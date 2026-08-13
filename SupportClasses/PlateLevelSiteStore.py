"""
PlateLevelSiteStore — the trained feature bank and the plate-to-plate prior.

Two things get learned during a leveling run and reused on the next plate of the
same type:

**1. A multi-defocus template bank per site, per objective.** The through-focus
sweep already produces images of the feature at known defocus above and below
best focus, so banking them costs nothing. It matters because
``TM_CCOEFF_NORMED`` is invariant to affine intensity change but **not to blur** —
a sharp template matches a badly defocused feature poorly, which is exactly the
state the feature is in when a search starts. Matching against the whole bank and
taking the best confidence is what lets the software find the feature "out of
focus below and above".

**2. A prior on how much plates of this type vary.** Glass thickness and how the
plate seats in the holder shift the whole focal surface by some amount that
differs plate to plate. Recording the mean and spread of that shift lets site 1 of
the next run start with a search window sized from evidence rather than from the
full working-distance range.

THE INVARIANT
-------------
**The prior only ever narrows a SEARCH. It never contributes to a fitted plane.**
Every run re-measures the tilt from scratch. A plate that seats differently is
then measured correctly instead of being assumed to match the last one — which is
what makes reusing this data safe at all.

Keyed by ``plate_key | camobj``. Deliberately NOT an extension of
``ReanchorFeatureStore``: that store holds one feature per plate key and is owned
by the map re-anchor, and overloading it would couple two unrelated invalidation
stories.

Zero GUI dependencies (numpy + OpenCV + json only).
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

try:
    import cv2
    _CV2 = True
except ImportError:                                   # pragma: no cover
    cv2 = None
    _CV2 = False

_DEFAULT_PATH = resolve_machine_path("plate_level_sites.json")

#: Defocus levels banked either side of best focus, as multiples of the focus
#: curve's own FWHM. Beyond ~2 FWHM the feature is a smear that matches anything.
DEFOCUS_LEVELS_FWHM = (-2.0, -1.0, 0.0, 1.0, 2.0)

#: A stored patch is unusable if the live µm/px differs by more than this — the
#: template would be at the wrong scale. Same threshold the auto-re-anchor uses.
UM_PER_PX_TOLERANCE = 0.02

#: Cap on banked patches per (plate, camobj) before the oldest are pruned.
MAX_PATCHES = 400


def _safe(part) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(part)) or "x"


def bank_key(plate_key, camobj: str) -> str:
    return f"{_safe(plate_key)}|{_safe(camobj)}"


class PlateLevelSiteStore:
    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._img_dir = self._path.parent / "plate_level_sites"
        self._data: dict = {"version": "1.0", "banks": {}, "priors": {}}
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
            for k in ("banks", "priors"):
                if not isinstance(self._data.get(k), dict):
                    self._data[k] = {}
        except Exception as exc:
            logger.warning(f"PlateLevelSiteStore: failed to load "
                           f"{self._path}: {exc}")
            self._data = {"version": "1.0", "banks": {}, "priors": {}}

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"PlateLevelSiteStore: failed to save: {exc}")

    def reload(self) -> None:
        self._data = {"version": "1.0", "banks": {}, "priors": {}}
        self._load()

    # ── feature bank ──────────────────────────────────────────────

    def save_patch(self, plate_key, camobj: str, site_id: str,
                   objective_name: str, patch_bgr, *,
                   defocus_um: float, um_per_px: float,
                   stage_um=None, frame_wh=None) -> bool:
        """Bank one defocus level of one site's feature."""
        if not _CV2 or patch_bgr is None or getattr(patch_bgr, "size", 0) == 0:
            return False
        key = bank_key(plate_key, camobj)
        # site_id reaches the filesystem, so it is sanitised rather than trusted.
        name = (f"{_safe(key)}__{_safe(site_id)}__{_safe(objective_name)}"
                f"__{int(round(defocus_um)):+d}.png")
        path = self._img_dir / name
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            resolved = path.resolve()
            if self._img_dir.resolve() not in resolved.parents:
                logger.error("PlateLevelSiteStore: refusing a patch path that "
                             "escapes the store directory")
                return False
            if not cv2.imwrite(str(path), patch_bgr):
                return False
        except Exception as exc:
            logger.error(f"PlateLevelSiteStore: failed to write patch: {exc}")
            return False

        rec = {
            "site_id": str(site_id), "objective": str(objective_name),
            "defocus_um": float(defocus_um), "um_per_px": float(um_per_px),
            "image": str(path).replace("\\", "/"),
            "date": datetime.now().isoformat(timespec="seconds"),
        }
        if stage_um is not None:
            try:
                rec["stage_um"] = [float(stage_um[0]), float(stage_um[1])]
            except (TypeError, ValueError, IndexError):
                pass
        if frame_wh is not None:
            try:
                rec["frame_wh"] = [int(frame_wh[0]), int(frame_wh[1])]
            except (TypeError, ValueError, IndexError):
                pass

        bank = self._data["banks"].setdefault(key, [])
        bank[:] = [r for r in bank
                   if not (r.get("site_id") == rec["site_id"]
                           and r.get("objective") == rec["objective"]
                           and abs(float(r.get("defocus_um", 1e9))
                                   - rec["defocus_um"]) < 1e-6)]
        bank.append(rec)
        self._prune(bank)
        self._save()
        return True

    def _prune(self, bank: list) -> None:
        """Keep the bank bounded, deleting the PNGs of what is dropped."""
        while len(bank) > MAX_PATCHES:
            old = bank.pop(0)
            try:
                p = Path(old.get("image", ""))
                if p.exists():
                    p.unlink()
            except Exception:
                pass

    def patches_for(self, plate_key, camobj: str, site_id: str,
                    objective_name: str | None = None) -> list[dict]:
        bank = self._data["banks"].get(bank_key(plate_key, camobj), [])
        return [r for r in bank
                if r.get("site_id") == str(site_id)
                and (objective_name is None
                     or r.get("objective") == str(objective_name))]

    def load_patch(self, rec: dict):
        if not _CV2 or not rec:
            return None
        try:
            return cv2.imread(str(rec.get("image", "")))
        except Exception:
            return None

    def has_site(self, plate_key, camobj: str, site_id: str) -> bool:
        return bool(self.patches_for(plate_key, camobj, site_id))

    def site_ids(self, plate_key, camobj: str) -> list[str]:
        bank = self._data["banks"].get(bank_key(plate_key, camobj), [])
        seen, out = set(), []
        for r in bank:
            s = r.get("site_id")
            if s and s not in seen:
                seen.add(s)
                out.append(s)
        return out

    def patch_scale_ok(self, rec: dict, live_um_per_px: float) -> bool:
        """Is a stored patch still at a usable scale for the live camera?"""
        try:
            stored = float(rec.get("um_per_px") or 0.0)
            live = float(live_um_per_px or 0.0)
        except (TypeError, ValueError):
            return False
        if stored <= 0 or live <= 0:
            return False
        return abs(stored / live - 1.0) <= UM_PER_PX_TOLERANCE

    def clear_bank(self, plate_key, camobj: str) -> None:
        key = bank_key(plate_key, camobj)
        for rec in self._data["banks"].pop(key, []):
            try:
                p = Path(rec.get("image", ""))
                if p.exists():
                    p.unlink()
            except Exception:
                pass
        self._save()

    # ── plate-to-plate prior ──────────────────────────────────────

    def get_prior(self, plate_key, camobj: str):
        from SupportClasses.PlateLeveling import PlatePrior
        rec = self._data["priors"].get(bank_key(plate_key, camobj))
        if not rec:
            return PlatePrior()
        try:
            ref = rec.get("ref_xy_um")
            return PlatePrior(
                n_runs=int(rec.get("n_runs", 0) or 0),
                offset_mean_um=float(rec.get("offset_mean_um", 0.0) or 0.0),
                offset_sigma_um=float(rec.get("offset_sigma_um", 0.0) or 0.0),
                sx_mm_per_mm=float(rec.get("sx_mm_per_mm", 0.0) or 0.0),
                sy_mm_per_mm=float(rec.get("sy_mm_per_mm", 0.0) or 0.0),
                ref_xy_um=((float(ref[0]), float(ref[1])) if ref else None),
                ref_focus_um=(None if rec.get("ref_focus_um") is None
                              else float(rec["ref_focus_um"])))
        except (TypeError, ValueError, IndexError):
            return PlatePrior()

    def set_prior(self, plate_key, camobj: str, prior) -> None:
        rec = {
            "n_runs": int(getattr(prior, "n_runs", 0) or 0),
            "offset_mean_um": float(getattr(prior, "offset_mean_um", 0.0)),
            "offset_sigma_um": float(getattr(prior, "offset_sigma_um", 0.0)),
            "sx_mm_per_mm": float(getattr(prior, "sx_mm_per_mm", 0.0)),
            "sy_mm_per_mm": float(getattr(prior, "sy_mm_per_mm", 0.0)),
            "date": datetime.now().isoformat(timespec="seconds"),
        }
        ref = getattr(prior, "ref_xy_um", None)
        if ref:
            rec["ref_xy_um"] = [float(ref[0]), float(ref[1])]
        rf = getattr(prior, "ref_focus_um", None)
        if rf is not None:
            rec["ref_focus_um"] = float(rf)
        self._data["priors"][bank_key(plate_key, camobj)] = rec
        self._save()


_store: PlateLevelSiteStore | None = None


def get_store(path=None) -> PlateLevelSiteStore:
    global _store
    if path is not None:
        return PlateLevelSiteStore(Path(path))
    if _store is None:
        env = os.environ.get("MEBP_PLATE_LEVEL_SITE_DIR")
        p = (Path(env) / "plate_level_sites.json") if env else _DEFAULT_PATH
        _store = PlateLevelSiteStore(p)
    return _store


def reset_store() -> None:
    """Test isolation."""
    global _store
    _store = None
