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

Metadata per SCAN::

    {
      "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
      "um_per_px": 3.34,                            # camera scale used
      "mosaic_scale": 0.0185,                       # mosaic px per µm
      "image": "mosaics/96.png",                    # path relative to config dir
      "frames": 42,
      "date": "2026-06-16",
      "plate_frame": {                              # v7.13, absent = legacy
        "extent_mm":  [x0, y0, x1, y1],             # the SAME rectangle, in the
                                                    #   plate's A1-relative frame
        "anchor_um":  [a1_x, a1_y],                 # taught A1 it was tied to
        "axis_sign":  [sx, sy]
      }
    }

v7.13 — TWO changes, both driven by the same operator requirement:

**A plate may hold SEVERAL scans.** A plate key maps to a container
``{"active": <scan_id>, "scans": {<scan_id>: <scan meta>}}`` instead of one
scan. Single-well entries (``"<plate_key>#<well>"``) stay single and are
untouched.

*The public API is unchanged.* ``get_meta`` / ``image_path`` / ``load_image`` /
``get_extent_um`` / ``get_wells`` / ``has`` all resolve to the **active** scan
and return exactly the shape they always did, so the nine consumer modules and
eleven test files that use this store keep working without edits. That
invariant is what made the v7.12 plate work shippable and it is kept here.

**A scan is referenced to BOTH coordinate systems.** ``extent_um`` is where the
mosaic sits on the STAGE; ``plate_frame.extent_mm`` is where it sits on the
PLATE. Storing only the former is why re-seating a plate needed the mosaic to
be dragged back by hand: nothing recorded what the image was of, only where the
stage happened to be. With both, ``get_extent_um(key, anchor_um=...)`` returns
the mosaic re-placed against the CURRENT taught anchor, so a re-teach carries
the scan with it.

Scans captured before v7.13 have no ``plate_frame``. They keep working exactly
as before (stage frame, manual re-anchor) and report ``needs_rescan`` — a
plate-frame extent cannot be invented for them, because the calibration they
were taken under is not recoverable from the file.

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

#: v7.13 — plate keys hold a container of named scans.
SCHEMA_VERSION = "2.0"
#: Slot the pre-v7.13 single scan is promoted into (keeps its PNG filename).
FIRST_SCAN_ID = "s1"
LEGACY_SCAN_NAME = "Scan 1"


def _safe_key(plate_key) -> str:
    """Filesystem-safe token for a plate key (used in the PNG filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(plate_key)) or "plate"


def stage_to_plate_mm(extent_um, anchor_um, axis_sign=(1.0, 1.0)):
    """Absolute stage µm → plate-local mm, relative to the taught A1.

    The inverse of :func:`plate_mm_to_stage_um`. ``axis_sign`` is
    ``StageController.plate_axis_sign()`` — on a 180°-mounted stage the plate
    axes run against the stage axes, and applying it here is what lets the
    stored plate frame be compared across machines with different mountings.

    Returned as ``(x0, y0, x1, y1)`` re-normalised so ``x0 <= x1``: a negative
    sign flips the rectangle's corners, and an un-normalised extent silently
    breaks every ``min_x``-style consumer downstream.
    """
    sx, sy = (float(axis_sign[0]), float(axis_sign[1]))
    ax, ay = (float(anchor_um[0]), float(anchor_um[1]))
    xs = [sx * (float(extent_um[i]) - ax) / 1000.0 for i in (0, 2)]
    ys = [sy * (float(extent_um[i]) - ay) / 1000.0 for i in (1, 3)]
    return (min(xs), min(ys), max(xs), max(ys))


def plate_mm_to_stage_um(extent_mm, anchor_um, axis_sign=(1.0, 1.0)):
    """Plate-local mm → absolute stage µm. Inverse of :func:`stage_to_plate_mm`."""
    sx, sy = (float(axis_sign[0]), float(axis_sign[1]))
    ax, ay = (float(anchor_um[0]), float(anchor_um[1]))
    xs = [ax + sx * float(extent_mm[i]) * 1000.0 for i in (0, 2)]
    ys = [ay + sy * float(extent_mm[i]) * 1000.0 for i in (1, 3)]
    return (min(xs), min(ys), max(xs), max(ys))


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
            self._data = {"version": SCHEMA_VERSION, "mosaics": {}}
        self._normalise()

    # ── v7.13: container ⇄ leaf ───────────────────────────────────
    #
    # A plate-level key holds a CONTAINER of named scans; a single-well key
    # (`plate#well`) stays a bare leaf. Normalising on load means the rest of
    # the class only ever deals with containers for plate keys, instead of
    # every method branching on which shape it got — the drift that two
    # coexisting shapes invite.

    @staticmethod
    def _is_container(entry) -> bool:
        return isinstance(entry, dict) and isinstance(entry.get("scans"), dict)

    @staticmethod
    def _is_well_key(key) -> bool:
        return "#" in str(key)

    def _normalise(self) -> None:
        """Promote v1 plate-level leaves to single-scan containers, in memory.

        The file itself is only rewritten when something else causes a save, so
        merely opening the app does not churn the operator's data.
        """
        mosaics = self._data.get("mosaics", {})
        for key, entry in list(mosaics.items()):
            if self._is_well_key(key) or self._is_container(entry):
                continue
            if not isinstance(entry, dict):
                mosaics.pop(key, None)
                continue
            entry.setdefault("name", LEGACY_SCAN_NAME)
            mosaics[key] = {"active": FIRST_SCAN_ID,
                            "scans": {FIRST_SCAN_ID: entry}}
        self._data["version"] = SCHEMA_VERSION

    def _container(self, plate_key, create: bool = False) -> Optional[dict]:
        """The scan container for a plate key, or None."""
        if plate_key is None:
            return None
        key = str(plate_key)
        mosaics = self._data.setdefault("mosaics", {})
        entry = mosaics.get(key)
        if self._is_container(entry):
            return entry
        if entry is not None and not self._is_well_key(key):
            entry.setdefault("name", LEGACY_SCAN_NAME)
            entry = {"active": FIRST_SCAN_ID, "scans": {FIRST_SCAN_ID: entry}}
            mosaics[key] = entry
            return entry
        if create and not self._is_well_key(key):
            entry = {"active": "", "scans": {}}
            mosaics[key] = entry
            return entry
        return None

    def _next_scan_id(self, container: dict) -> str:
        n = 1
        while f"s{n}" in container.get("scans", {}):
            n += 1
        return f"s{n}"

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
        anchor_um: Optional[tuple[float, float]] = None,
        axis_sign: tuple[float, float] = (1.0, 1.0),
        scan_id: Optional[str] = None,
        name: str = "",
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

        v7.13: pass ``anchor_um`` (the taught A1 in absolute stage µm) and
        ``axis_sign`` to ALSO record where the mosaic sits on the PLATE. Omit
        them and the scan is stored stage-frame-only, exactly as before — which
        is what an uncalibrated plate has to do, and what makes the new field
        additive rather than a precondition for scanning.

        ``scan_id`` targets one slot of a multi-scan plate; the default writes
        the active slot (creating the first). ``name`` labels a new slot.
        """
        if not _CV2 or image_bgr is None:
            logger.warning("MosaicStore.save: no cv2 or empty image")
            return False
        key = str(plate_key)
        # A well key stays a bare leaf; a plate key resolves to a scan slot so
        # a second scan does not overwrite the first one's PNG.
        if self._is_well_key(key):
            fname = f"{_safe_key(key)}.png"
        else:
            container = self._container(key, create=True)
            scan_id = scan_id or container.get("active") or FIRST_SCAN_ID
            if scan_id not in container["scans"] and container["scans"]:
                scan_id = self._next_scan_id(container)
            fname = (f"{_safe_key(key)}.png" if scan_id == FIRST_SCAN_ID
                     else f"{_safe_key(key)}__{scan_id}.png")
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
        meta = {
            "extent_um": ex,
            "um_per_px": float(um_per_px),
            "mosaic_scale": float(mosaic_scale),
            "image": f"mosaics/{fname}",
            "frames": int(frames),
            "shift_um": sh,
            "date": date.today().isoformat(),
        }
        if anchor_um is not None:
            try:
                meta["plate_frame"] = {
                    "extent_mm": list(stage_to_plate_mm(ex, anchor_um,
                                                        axis_sign)),
                    "anchor_um": [float(anchor_um[0]), float(anchor_um[1])],
                    "axis_sign": [float(axis_sign[0]), float(axis_sign[1])],
                }
            except (TypeError, ValueError, IndexError) as exc:
                logger.warning("MosaicStore.save: bad plate anchor (%s) — "
                               "storing stage frame only", exc)

        if self._is_well_key(key):
            self._data.setdefault("mosaics", {})[key] = meta
        else:
            container = self._container(key, create=True)
            meta["name"] = (name or container["scans"].get(scan_id, {})
                            .get("name") or f"Scan {len(container['scans']) + 1}")
            container["scans"][scan_id] = meta
            container["active"] = scan_id
        self._save_meta()
        logger.info(
            f"MosaicStore: saved mosaic for '{key}' "
            f"({fname}, extent={ex}, shift={sh}, "
            f"plate_frame={'yes' if 'plate_frame' in meta else 'no'})")
        return True

    # ── Read ──────────────────────────────────────────────────────

    def get_meta(self, plate_key) -> Optional[dict]:
        """Metadata for ``plate_key``'s ACTIVE scan (or None).

        v7.13: a plate key now holds several scans, but this still returns one
        scan's dict in exactly the pre-v7.13 shape. Every reader below — and
        every consumer outside this module — goes through here, which is what
        makes multi-scan invisible to them.
        """
        if plate_key is None:
            return None
        entry = self._data.get("mosaics", {}).get(str(plate_key))
        if self._is_container(entry):
            return entry["scans"].get(entry.get("active"))
        return entry

    # ── v7.13: several scans per plate ────────────────────────────

    def list_scans(self, plate_key) -> list[dict]:
        """``[{id, name, date, active, needs_rescan, frames}, …]``, newest last.

        Empty for a plate with no scans, so a caller can render "no mosaic"
        without special-casing.
        """
        container = self._container(plate_key)
        if container is None:
            return []
        active = container.get("active")
        out = []
        for scan_id, meta in container.get("scans", {}).items():
            if not isinstance(meta, dict):
                continue
            out.append({
                "id": scan_id,
                "name": meta.get("name") or scan_id,
                "date": meta.get("date", ""),
                "frames": int(meta.get("frames", 0) or 0),
                "active": scan_id == active,
                "needs_rescan": "plate_frame" not in meta,
            })
        out.sort(key=lambda s: (s["date"], s["id"]))
        return out

    def active_scan_id(self, plate_key) -> str:
        container = self._container(plate_key)
        return str(container.get("active", "")) if container else ""

    def set_active_scan(self, plate_key, scan_id) -> bool:
        """Point the plate at one of its scans. False if it has no such scan."""
        container = self._container(plate_key)
        if container is None or str(scan_id) not in container.get("scans", {}):
            return False
        container["active"] = str(scan_id)
        self._save_meta()
        logger.info("MosaicStore: '%s' now uses scan '%s'", plate_key, scan_id)
        return True

    def rename_scan(self, plate_key, scan_id, name: str) -> bool:
        container = self._container(plate_key)
        meta = (container or {}).get("scans", {}).get(str(scan_id))
        if meta is None or not str(name).strip():
            return False
        meta["name"] = str(name).strip()
        self._save_meta()
        return True

    def delete_scan(self, plate_key, scan_id) -> bool:
        """Drop one scan (and its PNG). The active slot moves to a survivor."""
        container = self._container(plate_key)
        if container is None:
            return False
        meta = container.get("scans", {}).pop(str(scan_id), None)
        if meta is None:
            return False
        self._unlink_image(meta)
        if container.get("active") == str(scan_id):
            remaining = list(container.get("scans", {}))
            container["active"] = remaining[0] if remaining else ""
        if not container.get("scans"):
            self._data.get("mosaics", {}).pop(str(plate_key), None)
        self._save_meta()
        return True

    # ── v7.13: the plate-frame reference ──────────────────────────

    def plate_frame(self, plate_key) -> Optional[dict]:
        """``{extent_mm, anchor_um, axis_sign}`` for the active scan, or None.

        None means the scan predates v7.13 (or was taken with no calibration to
        reference against) and is stage-frame only.
        """
        meta = self.get_meta(plate_key) or {}
        frame = meta.get("plate_frame")
        return frame if isinstance(frame, dict) else None

    #: The largest re-seat a re-anchor may follow, in µm.
    #:
    #: ⚠ Deliberately ABSOLUTE, not a fraction of the mosaic. A re-seat is a
    #: physical quantity — how far the plate can shift in its nest — and that
    #: is bounded by the holder, not by how much of the plate was imaged. A
    #: first cut scaled it to the mosaic's own size and the v7.13 suite caught
    #: it immediately: a SINGLE-WELL scan is ~2 mm across, so any fraction of
    #: it is smaller than a real remount, and a legitimate 3.5 mm re-teach was
    #: refused. 25 mm is larger than a well pitch (19.3 mm here) — past that
    #: the plate is not re-seated, it is in the wrong place — and far below the
    #: ~95 mm corner-to-corner jump this guard exists to stop.
    MAX_REANCHOR_SHIFT_UM = 25000.0

    def _reanchor_shift_is_plausible(self, old_ext, new_ext, plate_key) -> bool:
        """False when a re-anchor would move the mosaic further than a re-seat.

        See :meth:`reanchor`. Hardware-free: judged purely on the magnitude of
        the move, so the store needs no machine envelope.
        """
        try:
            ox0, oy0 = float(old_ext[0]), float(old_ext[1])
            nx0, ny0 = float(new_ext[0]), float(new_ext[1])
        except (TypeError, ValueError, IndexError):
            return True                     # nothing to judge — behave as before
        dx, dy = abs(nx0 - ox0), abs(ny0 - oy0)
        if max(dx, dy) <= self.MAX_REANCHOR_SHIFT_UM:
            return True
        logger.error(
            "MosaicStore: REFUSING to re-anchor '%s' — it would move the "
            "mosaic by (%.0f, %.0f) µm, past the %.0f µm re-seat limit. A "
            "plate re-seat is millimetres; a jump this large means the stored "
            "plate frame was recorded against a different A1 corner. The "
            "stored extent is left untouched — re-scan this plate to rebuild "
            "its plate frame.",
            plate_key, dx, dy, self.MAX_REANCHOR_SHIFT_UM)
        return False

    def reanchor(self, plate_key, anchor_um, axis_sign=(1.0, 1.0)) -> bool:
        """Re-place every referenced scan of a plate against a NEW taught A1.

        Call this once when the plate's calibration is re-taught. It rewrites
        each scan's stored STAGE extent from its unchanged PLATE extent, so the
        mosaic moves with the plate — the thing that previously had to be done
        by hand with "Re-anchor mosaic".

        Doing it here, at the one moment the anchor changes, rather than in
        ``get_extent_um`` at every read, is deliberate: the eight reader sites
        across the app keep calling the store exactly as they always have and
        still see a corrected extent.

        Legacy (stage-frame-only) scans are left alone — there is nothing to
        re-place them from, and moving them on a guess would put the mosaic,
        and every well centre mapped off it, silently in the wrong place.
        Returns True if anything was rewritten.

        ⚠ v7.9.1 — a re-anchor is a PLATE RE-SEAT, which is millimetres. This
        method used to translate by whatever the anchor delta happened to be,
        on the assumption that a changed ``taught_a1`` always means the plate
        moved. It does not: when the v7.9.1 well-labelling fix corrected which
        corner is A1, the anchor jumped by the plate DIAGONAL while
        ``extent_mm`` was still recorded against the old corner, and this
        method rigid-translated the operator's mosaic ~95 × 57 mm — clean off
        the machine, and unrecoverable from the ±10 mm manual nudge sliders.
        Two incompatible definitions of "A1" straddled a saved artifact.

        So a shift larger than :data:`MAX_REANCHOR_SHIFT_FRAC` of the mosaic's
        own size is REFUSED, not applied: at that magnitude it is not a re-seat
        but a frame mismatch, and the honest response is to leave the stored
        extent alone and say so. The operator re-scans (or nudges), which is
        recoverable; a silently relocated mosaic is not — every well centre
        mapped off it inherits the error.
        """
        if anchor_um is None:
            return False
        entry = self._data.get("mosaics", {}).get(str(plate_key))
        if self._is_container(entry):
            metas = list(entry.get("scans", {}).values())
        elif isinstance(entry, dict):
            metas = [entry]            # a single-well scan is a bare leaf
        else:
            return False
        changed = False
        for meta in metas:
            frame = meta.get("plate_frame") if isinstance(meta, dict) else None
            if not isinstance(frame, dict) or "extent_mm" not in frame:
                continue
            try:
                new_ext = plate_mm_to_stage_um(
                    frame["extent_mm"], anchor_um,
                    frame.get("axis_sign", axis_sign))
            except (TypeError, ValueError, IndexError):    # pragma: no cover
                continue
            if [round(v, 6) for v in new_ext] == [round(float(v), 6)
                                                  for v in meta["extent_um"]]:
                continue
            if not self._reanchor_shift_is_plausible(
                    meta.get("extent_um"), new_ext, plate_key):
                continue
            meta["extent_um"] = [float(v) for v in new_ext]
            frame["anchor_um"] = [float(anchor_um[0]), float(anchor_um[1])]
            changed = True
        if changed:
            self._save_meta()
            logger.info("MosaicStore: re-anchored '%s' to A1 %s",
                        plate_key, tuple(round(float(v), 1) for v in anchor_um))
        return changed

    def needs_rescan(self, plate_key) -> bool:
        """True when a stored scan cannot follow the plate.

        A plate-frame extent cannot be back-filled for a pre-v7.13 scan: it
        would have to assume the calibration that was live when the scan was
        taken, and a wrong assumption there puts the mosaic — and every well
        centre mapped off it — silently in the wrong place. Reporting "re-scan
        to enable this" is the honest answer.
        """
        return self.has(plate_key) and self.plate_frame(plate_key) is None

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

    def get_extent_um(
        self, plate_key,
        anchor_um: Optional[tuple[float, float]] = None,
        axis_sign: tuple[float, float] = (1.0, 1.0),
    ) -> Optional[tuple]:
        """``(min_x, min_y, max_x, max_y)`` absolute stage µm (or None).

        v7.13: pass the CURRENT taught anchor (A1 in absolute stage µm) and the
        mosaic is re-placed against it from its stored plate frame — so a plate
        that has been re-seated and re-taught carries its scan along instead of
        needing the operator to drag it back.

        Both arguments default to None, in which case this returns the stored
        stage extent verbatim: byte-identical to every pre-v7.13 caller, and
        the only thing a legacy (stage-frame-only) scan can do.
        """
        meta = self.get_meta(plate_key)
        if not meta or "extent_um" not in meta:
            return None
        ex = meta["extent_um"]
        if not (isinstance(ex, (list, tuple)) and len(ex) == 4):
            return None
        stored = tuple(float(v) for v in ex)
        if anchor_um is None:
            return stored
        frame = meta.get("plate_frame")
        if not isinstance(frame, dict) or "extent_mm" not in frame:
            return stored          # legacy scan — nothing to re-place from
        try:
            return plate_mm_to_stage_um(
                frame["extent_mm"], anchor_um,
                frame.get("axis_sign", axis_sign))
        except (TypeError, ValueError, IndexError) as exc:
            logger.warning("MosaicStore: bad plate frame for '%s' (%s) — "
                           "using the stored stage extent", plate_key, exc)
            return stored

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

    def copy_plate(self, src, dst) -> int:
        """Copy the mosaic under ``src`` to ``dst`` — image, extent, scale,
        shift, the last good well mapping, and every single-well scan.

        The source is untouched. Returns the number of entries written
        (plate + wells), or ``-1`` when the plate copy itself failed.

        Two surfaces need this: the calibration page's "Load mosaic from
        another plate…" and the plate library's per-card mosaic picker. Plates
        that share a geometry but not an ``active_plate_key`` — a bare 24-well
        and a NEST-plastic-24, say — differ only by their attached scan, so
        reusing one is the common case, not an edge case.
        """
        def _copy_one(a, b) -> bool:
            img = self.load_image(a)
            ext = self.get_extent_um(a)
            meta = self.get_meta(a) or {}
            if img is None or ext is None:
                return False
            # v7.13: carry the plate-frame reference across, or the copy would
            # arrive looking like a legacy scan and report needs_rescan even
            # though the source was fully referenced.
            frame = meta.get("plate_frame") if isinstance(meta, dict) else None
            anchor = axis = None
            if isinstance(frame, dict):
                anchor = frame.get("anchor_um")
                axis = frame.get("axis_sign") or (1.0, 1.0)
            ok = bool(self.save(
                b, img, ext,
                um_per_px=float(meta.get("um_per_px", 0.0) or 0.0),
                mosaic_scale=float(meta.get("mosaic_scale", 0.0) or 0.0),
                frames=int(meta.get("frames", 0) or 0),
                shift_um=self.get_shift_um(a),
                anchor_um=anchor,
                axis_sign=axis or (1.0, 1.0),
                name=str(meta.get("name", "") or "")))
            # save() rebuilds the metadata, so the well mapping has to be
            # carried across explicitly or it is silently lost.
            if ok:
                try:
                    wells = self.get_wells(a)
                    if wells:
                        self.set_wells(b, wells)
                except Exception as exc:
                    logger.debug(f"copy_plate: wells copy skipped: {exc}")
            return ok

        if not _copy_one(src, dst):
            return -1
        n = 1
        for well in self.list_well_keys(src):
            try:
                if _copy_one(f"{src}#{well}", f"{dst}#{well}"):
                    n += 1
            except Exception as exc:
                logger.debug(f"copy_plate: well '{well}' skipped: {exc}")
        return n

    def _unlink_image(self, meta) -> None:
        if not (isinstance(meta, dict) and meta.get("image")):
            return
        try:
            p = self._path.parent / meta["image"].replace(
                "mosaics/", "mosaics" + os.sep)
            if p.exists():
                p.unlink()
        except Exception as exc:
            logger.debug(f"MosaicStore: image unlink failed: {exc}")

    def clear(self, plate_key) -> None:
        """Remove the stored mosaic(s) for ``plate_key`` — images + metadata.

        v7.13: a plate key may hold several scans; clearing removes all of
        them, which is what "this plate has no mosaic" has to mean.
        """
        key = str(plate_key)
        entry = self._data.get("mosaics", {}).pop(key, None)
        if self._is_container(entry):
            for meta in entry.get("scans", {}).values():
                self._unlink_image(meta)
        else:
            self._unlink_image(entry)
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
