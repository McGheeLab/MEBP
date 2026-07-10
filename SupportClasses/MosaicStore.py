"""
MosaicStore.py — Per-machine persistence of stitched full-plate mosaics.

v7.5.x: The Plate Location tab can raster the whole plate, stitch the camera
snapshots into one composite image (see ``MosaicBuilder``), and detect every
well at once on that field. The stitched mosaic is also useful as a background
**overlay** on other pages (Jog, Plate Location) so the operator can see the
real plate under the rendered wells/needle.

Because that mosaic is a property of the *physical plate on the stage* (its
absolute stage-µm extent matters), it is persisted at the machine level —
not inside the swappable ``HardwareConfig`` — so it survives restarts and any
page can show it. This mirrors ``CameraCalibrationStore`` / objectives.json.

Files:
    config/hardware/plate_mosaics.json          — metadata, keyed by plate key
    config/hardware/mosaics/<plate_key>.png      — the composite image (BGR)

Metadata per plate key::

    {
      "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
      "um_per_px": 3.34,                            # camera scale used
      "mosaic_scale": 0.0185,                       # mosaic px per µm
      "image": "mosaics/96.png",                    # path relative to config dir
      "frames": 42,
      "date": "2026-06-16"
    }

This module has ZERO GUI dependencies (numpy + OpenCV + json only).
"""

from __future__ import annotations

import json
import logging
import os
import re
from datetime import date
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import cv2
    _CV2 = True
except ImportError:   # pragma: no cover - cv2 always present in this project
    cv2 = None
    _CV2 = False

_DEFAULT_PATH = Path("config/hardware/plate_mosaics.json")


def _safe_key(plate_key) -> str:
    """Filesystem-safe token for a plate key (used in the PNG filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(plate_key)) or "plate"


class MosaicStore:
    """Load/save per-plate stitched mosaics (image + absolute-µm extent)."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._img_dir = self._path.parent / "mosaics"
        self._data: dict = {"version": "1.0", "mosaics": {}}
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
            if not isinstance(self._data.get("mosaics"), dict):
                self._data["mosaics"] = {}
        except Exception as exc:
            logger.warning(f"MosaicStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "mosaics": {}}

    def _save_meta(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"MosaicStore: failed to save metadata: {exc}")

    # ── Write ─────────────────────────────────────────────────────

    def save(
        self,
        plate_key,
        image_bgr,
        extent_um: tuple[float, float, float, float],
        um_per_px: float = 0.0,
        mosaic_scale: float = 0.0,
        frames: int = 0,
        shift_um: tuple[float, float] = (0.0, 0.0),
    ) -> bool:
        """Persist a stitched mosaic for ``plate_key``.

        ``image_bgr`` is the composite (numpy BGR uint8). ``extent_um`` is the
        composite's world extent ``(min_x, min_y, max_x, max_y)`` in **absolute
        stage µm** (i.e. ``MosaicBuilder.canvas_extent_um``). Returns True on a
        successful image+metadata write.

        ``shift_um`` (v7.5.x) records the global registration shift BAKED INTO
        ``extent_um`` (``MosaicBuilder._global_shift_um``): the extent is
        display-registered (shifted), while tile PIXELS sit in the trusted raw
        stage frame — so any px → stage-µm back-projection must use
        ``extent[:2] − shift_um``. Persisting the shift lets store-loaded
        mosaics (Map wells…, sub-well mapping re-open) recover the trusted
        frame exactly. Legacy entries lack it → (0, 0).
        """
        if not _CV2 or image_bgr is None:
            logger.warning("MosaicStore.save: no cv2 or empty image")
            return False
        key = str(plate_key)
        fname = f"{_safe_key(key)}.png"
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            ok = cv2.imwrite(str(self._img_dir / fname), image_bgr)
            if not ok:
                logger.error("MosaicStore.save: cv2.imwrite failed")
                return False
        except Exception as exc:
            logger.error(f"MosaicStore.save: image write failed: {exc}")
            return False

        ex = [float(v) for v in extent_um]
        try:
            sh = [float(shift_um[0]), float(shift_um[1])]
        except (TypeError, ValueError, IndexError):
            sh = [0.0, 0.0]
        self._data.setdefault("mosaics", {})[key] = {
            "extent_um": ex,
            "um_per_px": float(um_per_px),
            "mosaic_scale": float(mosaic_scale),
            "image": f"mosaics/{fname}",
            "frames": int(frames),
            "shift_um": sh,
            "date": date.today().isoformat(),
        }
        self._save_meta()
        logger.info(
            f"MosaicStore: saved mosaic for '{key}' "
            f"({fname}, extent={ex}, shift={sh})")
        return True

    # ── Read ──────────────────────────────────────────────────────

    def get_meta(self, plate_key) -> Optional[dict]:
        """Return the metadata dict for ``plate_key`` (or None)."""
        if plate_key is None:
            return None
        return self._data.get("mosaics", {}).get(str(plate_key))

    def image_path(self, plate_key) -> Optional[str]:
        """Absolute path to the stored PNG for ``plate_key`` (or None)."""
        meta = self.get_meta(plate_key)
        if not meta:
            return None
        rel = meta.get("image")
        if not rel:
            return None
        p = self._path.parent / rel.replace("mosaics/", "mosaics" + os.sep)
        return str(p) if p.exists() else None

    def load_image(self, plate_key):
        """Return the stored composite as a numpy BGR array (or None)."""
        if not _CV2:
            return None
        p = self.image_path(plate_key)
        if p is None:
            return None
        try:
            return cv2.imread(p)
        except Exception as exc:
            logger.warning(f"MosaicStore.load_image failed: {exc}")
            return None

    def get_extent_um(self, plate_key) -> Optional[tuple]:
        """Return ``(min_x, min_y, max_x, max_y)`` absolute µm (or None)."""
        meta = self.get_meta(plate_key)
        if not meta or "extent_um" not in meta:
            return None
        ex = meta["extent_um"]
        if not (isinstance(ex, (list, tuple)) and len(ex) == 4):
            return None
        return tuple(float(v) for v in ex)

    def update_extent(self, plate_key, extent_um) -> bool:
        """Rewrite ONLY an entry's extent (metadata; the PNG is untouched).
        Used to translate a stored mosaic without re-encoding the image."""
        meta = self.get_meta(plate_key)
        if not meta or not (isinstance(extent_um, (list, tuple))
                            and len(extent_um) >= 4):
            return False
        meta["extent_um"] = [float(v) for v in extent_um[:4]]
        self._save_meta()
        return True

    def get_shift_um(self, plate_key) -> tuple:
        """Global registration shift baked into the stored extent, ``(0, 0)``
        for legacy entries. Back-projection frame = ``extent[:2] − shift``."""
        meta = self.get_meta(plate_key)
        if not meta:
            return (0.0, 0.0)
        sh = meta.get("shift_um")
        if not (isinstance(sh, (list, tuple)) and len(sh) >= 2):
            return (0.0, 0.0)
        try:
            return (float(sh[0]), float(sh[1]))
        except (TypeError, ValueError):
            return (0.0, 0.0)

    def has(self, plate_key) -> bool:
        return self.image_path(plate_key) is not None

    def list_plate_keys(self) -> list:
        """PLATE-level keys with a stored mosaic (single-well ``key#well``
        entries excluded). Used by "Load mosaic from another plate…"."""
        return [k for k in self._data.get("mosaics", {}) if "#" not in k]

    # ── Last good well mapping (v7.5.x) ───────────────────────────
    # The operator-confirmed well mapping is stored WITH the mosaic (meta
    # field ``wells_um``: name → [x, y] absolute stage µm, same frame as the
    # calibrated positions at commit time) so loading the mosaic anywhere —
    # e.g. "Load mosaic from another plate…" — restores the mapping without
    # re-mapping. NOTE: ``save()`` intentionally rebuilds the meta and DROPS
    # any stored mapping (a fresh scan needs a fresh mapping); callers that
    # re-save the same image (translate/copy) must carry the wells across
    # explicitly via ``get_wells``/``set_wells``.

    def set_wells(self, plate_key, wells: dict) -> bool:
        """Store the last GOOD well mapping for ``plate_key`` (metadata-only
        write; the PNG is untouched). Empty/invalid input clears nothing and
        returns False."""
        meta = self.get_meta(plate_key)
        if meta is None or not isinstance(wells, dict) or not wells:
            return False
        try:
            meta["wells_um"] = {
                str(n): [float(p[0]), float(p[1])]
                for n, p in wells.items()}
        except (TypeError, ValueError, IndexError):
            return False
        self._save_meta()
        return True

    def get_wells(self, plate_key) -> dict:
        """The stored well mapping (name → (x, y) absolute stage µm), ``{}``
        when none was saved with this mosaic."""
        meta = self.get_meta(plate_key) or {}
        raw = meta.get("wells_um")
        out: dict = {}
        if isinstance(raw, dict):
            for n, p in raw.items():
                try:
                    out[str(n)] = (float(p[0]), float(p[1]))
                except (TypeError, ValueError, IndexError):
                    continue
        return out

    # ── Single-well mosaics (v7.5.x) ──────────────────────────────
    # A single-well mosaic is stored in the SAME store under the composite key
    # ``f"{plate_key}#{well}"`` — plate-mosaic consumers look keys up verbatim
    # (nothing enumerates), so well entries never leak into the plate overlay.

    def list_well_keys(self, plate_key) -> list:
        """Well names with a stored single-well mosaic for ``plate_key``."""
        prefix = f"{str(plate_key)}#"
        return [k[len(prefix):]
                for k in self._data.get("mosaics", {})
                if k.startswith(prefix)]

    def has_well(self, plate_key, well) -> bool:
        """True if a single-well mosaic is stored for (plate_key, well)."""
        return self.has(f"{str(plate_key)}#{well}")

    def clear(self, plate_key) -> None:
        """Remove the stored mosaic (image + metadata) for ``plate_key``."""
        key = str(plate_key)
        meta = self._data.get("mosaics", {}).pop(key, None)
        if meta and meta.get("image"):
            try:
                p = self._path.parent / meta["image"].replace(
                    "mosaics/", "mosaics" + os.sep)
                if p.exists():
                    p.unlink()
            except Exception as exc:
                logger.debug(f"MosaicStore.clear: image unlink failed: {exc}")
        self._save_meta()


def composite_with_wells(store: "MosaicStore", plate_key, plate_img,
                         plate_extent):
    """Paste every stored single-well mosaic (``"{plate_key}#{well}"``) into
    the PLATE mosaic canvas at its own extent — the display composite for the
    "include well scans" overlay toggle.

    ``plate_img`` (numpy BGR) is copied; the input is never mutated. Extents
    are the display-registered (shift-including) ones on both sides, so the
    paste is frame-consistent for display. Well images falling (partly)
    outside the plate extent are clipped. Best-effort per well — one bad entry
    never breaks the composite. Returns ``(composite_img, plate_extent)``; the
    plate image/extent pass through unchanged when there are no well mosaics.
    """
    if (not _CV2 or store is None or plate_img is None
            or not (isinstance(plate_extent, (list, tuple))
                    and len(plate_extent) >= 4)):
        return plate_img, plate_extent
    try:
        wells = store.list_well_keys(plate_key)
    except Exception:
        wells = []
    if not wells:
        return plate_img, plate_extent
    p_minx, p_miny, p_maxx, p_maxy = (float(v) for v in plate_extent[:4])
    p_h, p_w = plate_img.shape[:2]
    if p_w < 2 or p_h < 2 or p_maxx <= p_minx or p_maxy <= p_miny:
        return plate_img, plate_extent
    sx = p_w / (p_maxx - p_minx)          # plate px per µm
    sy = p_h / (p_maxy - p_miny)
    composite = plate_img.copy()
    for well in wells:
        try:
            wkey = f"{str(plate_key)}#{well}"
            w_img = store.load_image(wkey)
            w_ext = store.get_extent_um(wkey)
            if w_img is None or w_ext is None:
                continue
            w_minx, w_miny, w_maxx, w_maxy = (float(v) for v in w_ext[:4])
            # Target px block in the plate canvas (clipped to the canvas).
            x0 = max(0, int(round((w_minx - p_minx) * sx)))
            y0 = max(0, int(round((w_miny - p_miny) * sy)))
            x1 = min(p_w, int(round((w_maxx - p_minx) * sx)))
            y1 = min(p_h, int(round((w_maxy - p_miny) * sy)))
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            # Crop the SOURCE proportionally when the target was clipped, so
            # the pasted content stays position-true.
            wh, ww = w_img.shape[:2]
            fx0 = ((p_minx + x0 / sx) - w_minx) / (w_maxx - w_minx)
            fy0 = ((p_miny + y0 / sy) - w_miny) / (w_maxy - w_miny)
            fx1 = ((p_minx + x1 / sx) - w_minx) / (w_maxx - w_minx)
            fy1 = ((p_miny + y1 / sy) - w_miny) / (w_maxy - w_miny)
            sx0 = max(0, min(ww - 1, int(round(fx0 * ww))))
            sy0 = max(0, min(wh - 1, int(round(fy0 * wh))))
            sx1 = max(sx0 + 1, min(ww, int(round(fx1 * ww))))
            sy1 = max(sy0 + 1, min(wh, int(round(fy1 * wh))))
            src = w_img[sy0:sy1, sx0:sx1]
            resized = cv2.resize(src, (x1 - x0, y1 - y0),
                                 interpolation=cv2.INTER_AREA)
            composite[y0:y1, x0:x1] = resized
        except Exception as exc:
            logger.debug(f"composite_with_wells: skip '{well}': {exc}")
    return composite, plate_extent


_store_singleton: Optional[MosaicStore] = None


def get_store() -> MosaicStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = MosaicStore()
    return _store_singleton
