"""
FluorescenceMosaicStore.py — Per-(plate, well) multi-channel fluorescence mosaics.

v7.5.x: The **Fluorescence Mosaic** workflow rasters a single well at high
resolution for each selected fluorescence channel (DAPI / FITC / mCherry / Cy5 /
…). Every channel of a well shares the SAME raster grid + camera scale, so their
composites register pixel-for-pixel and can be overlaid in any pseudo-colour.

⚠ This docstring used to add "the operator switches the physical filter between
channels (there is no filter-wheel hardware)". **That is wrong and has been since
v7.5.x** — the Nikon Ti's cassette is motorized and hardware-verified switching
all six slots with read-back. Since v7.19 the workflow drives it (see
``channel_slot`` below) and only prompts when the cube cannot be resolved or the
body cannot be reached.

⚠ The channel names here are OUR vocabulary, not the cassette's labels. Use
``channel_slot`` — never ``channel_ordinal`` / the deprecated
``channel_number`` — to turn a channel into a turret position.

The captured mosaics are a property of the *physical plate on the stage* (their
absolute stage-µm extent matters) — so, like ``MosaicStore`` / objectives.json,
they are persisted at the machine level (not inside the swappable
``HardwareConfig``). Any other workflow (Spheroid Pick & Place, Cell Targeting,
Cell Labeling, Quick Print, the Jog plate view, …) can then look up a well and
draw the blended fluorescence image as a registered background overlay.

Files:
    config/hardware/fluorescence_mosaics.json          — metadata
    config/hardware/fluor_mosaics/<plate>_<well>_<channel>_<YYYYMMDD-HHMMSS>.png

⚠ **v7.21.6 — A CAPTURE IS NEVER OVERWRITTEN.** The image name used to be
``<plate>_<well>_<channel>.png``, which is one slot per (plate, well, channel):
re-scanning A1/FITC truncated the previous PNG *and* replaced its whole metadata
record, so yesterday's image was unrecoverable. The name now carries the capture
instant, and the superseded record is moved onto the channel's ``history`` list
(newest first) instead of being dropped — nothing is lost and nothing is
orphaned. The ACTIVE capture is still plain ``channels[<name>]``, so every
reader (:meth:`load_channel_image`, :meth:`get_extent_um`,
:meth:`composite_overlay`, the workflow overlays, the ND3 export) is unchanged.
Use :meth:`list_history` / :meth:`restore_history` to reach the older ones.

Metadata layout::

    {
      "version": "1.0",
      "wells": {
        "<plate_key>|<well_name>": {
          "plate_key": "24",
          "well_name": "A1",
          "objective": "10x",
          "date": "2026-06-22",
          "channels": {
            "DAPI": {
              "image": "fluor_mosaics/24_A1_DAPI_20260622-141503.png",
              "color": [0, 0, 255],          # display pseudo-colour, RGB 0-255
              "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
              "shift_um": [dx, dy],          # registration shift baked into extent
              "um_per_px": 0.92, "mosaic_scale": 0.31,
              "frames": 36, "exposure_us": 0, "date": "2026-06-22",
              "captured_at": "2026-06-22T14:15:03",
              "history": [ {<the same shape, newest first>}, ... ]
            },
            ...
          }
        }
      }
    }

⚠ ``extent_um`` is DISPLAY-REGISTERED: ``MosaicBuilder.canvas_extent_um`` adds
the global registration shift to it while the tile PIXELS stay in the trusted
raw stage frame. So any px → stage-µm back-projection must use
``extent[:2] − shift_um`` (see :meth:`get_shift_um`, and ``MosaicStore.save``
which documents the same contract). Entries written before v7.8 have no
``shift_um`` key — :meth:`has_shift` reports False for those, and their pixel →
stage mapping cannot be trusted for MOTION (a diameter measured on them is
still fine; a translation does not change a length).

Colours are stored RGB (matching the channel swatches in the UI / QColor) and
converted to BGR only when blending with OpenCV. This module has ZERO GUI
dependencies (numpy + OpenCV + json only).
"""

from __future__ import annotations

import json
import logging
import os
import re
from datetime import date, datetime
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

try:
    import cv2
    import numpy as np
    _CV2 = True
except ImportError:   # pragma: no cover - cv2/numpy always present in this project
    cv2 = None
    np = None
    _CV2 = False

_DEFAULT_PATH = resolve_machine_path("fluorescence_mosaics.json")
# Migrated at import time (see MosaicStore.py's _DEFAULT_IMG_DIR comment) —
# NOT lazily inside __init__, so a plain `import` is enough to relocate it.
_DEFAULT_IMG_DIR = resolve_machine_path("fluor_mosaics")

# The app's IMAGING-CHANNEL vocabulary: the channels surfaced in the workflow UI,
# with their default display pseudo-colours (RGB 0-255). The operator can override
# any colour per capture; this is just the seed.
#
# ⚠ These are channel names, NOT the labels on the cubes in any particular
# cassette, and v7.18 keeps them deliberately independent of the hardware. They
# key stored captures (the PNG filename is
# ``{plate}_{well}_{channel}.png``) and the built-in target-type rules
# (``config/hardware/target_types/builtin/*.json`` author clauses against
# ``"imaging_channel": "mCherry"``), so renaming one to match a physical cube
# label would orphan data and invalidate those rules. The channel → cube slot
# mapping is a separate, per-machine fact — see ``CHANNEL_ORDINALS`` below and
# ``MicroscopeConfigStore.optic_aliases("filter")``.
CHANNELS: tuple[str, ...] = ("DAPI", "FITC", "mCherry", "Cy5", "Bright Field")

DEFAULT_CHANNEL_COLORS: dict[str, tuple[int, int, int]] = {
    "DAPI": (60, 120, 255),     # blue
    "FITC": (0, 230, 0),        # green
    "mCherry": (255, 40, 40),   # red
    "Cy5": (255, 0, 230),       # magenta / far-red
    "Bright Field": (255, 255, 255),  # grayscale / white (non-fluorescent)
}

# ⚠ v7.18 — THIS IS AN ORDINAL, **NOT** A TURRET SLOT NUMBER. Do not use it to
# drive the filter cassette.
#
# It was introduced as "the microscope hardware channel number … so the operator
# knows which turret position to select", and that claim was never true of any
# particular rig: nothing reconciled it against the operator's actual slot
# assignments (``MicroscopeConfigStore.filter_labels()``) or the body's own
# reported names. On THIS machine the cassette holds DAPI(1) FITC(2) **TxRed**(3)
# Cy5(4) with slots 5 and 6 empty — so "mCherry → 3" names a cube that is not
# mCherry, and "Bright Field → 5" names an EMPTY slot. Driving either would put
# the wrong thing (or nothing) in the light path and store the result as a
# legitimate channel, which no downstream consumer can detect.
#
# What it legitimately IS: a stable, per-channel-name ordinal giving a
# deterministic ACQUISITION ORDER. ``LabLinkJob`` relies on exactly that — the
# sidecar's ``channels`` array is order-load-bearing while ``ND3Reader.image_ids``
# returns alphabetical ids, so without an ordinal a DAPI/FITC/Cy5 well would be
# declared Cy5, DAPI, FITC and a recipe indexing by position would analyse the
# wrong channel. It is also fine as a display hint.
#
# To find the slot a channel actually lives in, use
# ``OpticsRegistry.find_slot(resolve_filters(...), channel, aliases=...)``, which
# refuses rather than guessing. See ``channel_slot_hint`` below.
CHANNEL_ORDINALS: dict[str, int] = {
    "DAPI": 1, "FITC": 2, "mCherry": 3, "Cy5": 4, "Bright Field": 5,
}

#: Deprecated alias. Same object, so existing readers are unaffected; the name
#: is what was misleading. New code should use ``CHANNEL_ORDINALS`` (for order)
#: or ``OpticsRegistry.find_slot`` (for a real slot).
CHANNEL_NUMBERS = CHANNEL_ORDINALS


def default_color(channel: str) -> tuple[int, int, int]:
    """Default display pseudo-colour (RGB) for a channel name."""
    return DEFAULT_CHANNEL_COLORS.get(channel, (220, 220, 220))


def channel_ordinal(channel: str):
    """Stable acquisition-order ordinal for a channel name, or ``None``.

    ⚠ **Not a turret slot.** See :data:`CHANNEL_ORDINALS`. Use
    :func:`channel_slot` when you need the cube's actual position.
    """
    return CHANNEL_ORDINALS.get(channel)


def channel_number(channel: str):
    """Deprecated: use :func:`channel_ordinal` (order) or :func:`channel_slot`.

    Retained because ND3 sidecars already on disk carry the value under the name
    ``channel_number`` and ``LabLinkJob`` reads it for acquisition ordering.
    """
    return channel_ordinal(channel)


def channel_slot(channel: str, *, scope_state, config_store):
    """Which cassette POSITION holds this channel's cube? ``SlotMatch``.

    v7.18. The real resolution, and the only thing a caller may drive the filter
    turret from: it joins the live turret against the operator's slot labels and
    aliases via ``OpticsRegistry``, and **refuses** — with a sentence naming what
    IS configured — rather than guessing. ``result.ok`` is False for a channel
    whose name matches no cube ("mCherry" against a cassette holding "TxRed",
    until the operator declares that alias) and for a slot the body reports empty
    ("Bright Field", which the legacy ordinal table pointed at slot 5).
    """
    from SupportClasses.OpticsRegistry import FILTER, find_slot, resolve_filters
    slots = resolve_filters(scope_state=scope_state, config_store=config_store)
    aliases = {}
    try:
        aliases = config_store.optic_aliases("filter")
    except Exception:
        aliases = {}
    # kind is explicit: with no cassette the slot list is empty, and a refusal
    # that cannot tell which turret was asked about names the wrong one.
    return find_slot(slots, channel, aliases=aliases, kind=FILTER)


def _safe_token(value) -> str:
    """Filesystem-safe token (used in the PNG filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "x"


#: Capture-stamp format embedded in every image filename (local time).
CAPTURE_STAMP_FMT = "%Y%m%d-%H%M%S"


def capture_stamp(when: Optional[datetime] = None) -> str:
    """``YYYYMMDD-HHMMSS`` stamp for an image filename."""
    return (when or datetime.now()).strftime(CAPTURE_STAMP_FMT)


def channel_image_name(plate_key, well_name, channel, stamp: str) -> str:
    """The ONE place a channel image's filename is formed.

    v7.21.6 — the stamp is what makes a re-scan of the same
    (plate, well, channel) a NEW file instead of an overwrite. Before this the
    name was ``{plate}_{well}_{channel}.png``, so a second capture of A1/FITC
    silently truncated the first one's pixels and replaced its metadata; the
    only reason any older capture survived on this rig is that a machine-id
    change (ME3B_2 → ME3B_01) happened to fork the whole folder.

    Keep the stamp LAST: every existing tool, sort and glob that groups by
    ``{plate}_{well}_{channel}`` keeps working, and ``ls`` sorts a channel's
    captures chronologically.
    """
    return (f"{_safe_token(plate_key)}_{_safe_token(well_name)}_"
            f"{_safe_token(channel)}_{_safe_token(stamp)}.png")


def well_key(plate_key, well_name) -> str:
    """Composite metadata key for one (plate, well)."""
    return f"{plate_key}|{well_name}"


def _archived(ch_entry: dict) -> dict:
    """A channel record ready to sit in another record's ``history``.

    Drops the nested ``history`` key — the list is kept FLAT (newest first) on
    the active record, so archiving N times costs N entries and not 2^N.
    """
    return {k: v for k, v in dict(ch_entry).items() if k != "history"}


class FluorescenceMosaicStore:
    """Load/save per-(plate, well) multi-channel fluorescence mosaics."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        # An env override lets tests isolate the store so automated runs never
        # read or write the repo's config/hardware (mirrors the other stores).
        env = os.environ.get("MEBP_FLUOR_MOSAIC_PATH")
        if env:
            self._path = Path(env)
            self._img_dir = self._path.parent / "fluor_mosaics"
        else:
            self._path = Path(path)
            # A test-supplied path keeps its own sibling dir untouched; the
            # default path uses the already-migrated _DEFAULT_IMG_DIR.
            self._img_dir = (_DEFAULT_IMG_DIR if self._path == _DEFAULT_PATH
                              else self._path.parent / "fluor_mosaics")
        self._data: dict = {"version": "1.0", "wells": {}}
        self._mtime = None      # set by _load/_save_meta; see _reload_if_changed
        self._load()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            self._mtime = self._path.stat().st_mtime
        except OSError:
            self._mtime = None
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                self._data.update(loaded)
            if not isinstance(self._data.get("wells"), dict):
                self._data["wells"] = {}
        except Exception as exc:
            logger.warning(
                f"FluorescenceMosaicStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "wells": {}}

    def _save_meta(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
            self._mtime = self._path.stat().st_mtime
        except Exception as exc:
            logger.error(
                f"FluorescenceMosaicStore: failed to save metadata: {exc}")

    def _reload_if_changed(self) -> None:
        """Re-read the file before mutating it (v7.21.6).

        ⚠ THIS PREVENTS A LOST UPDATE, and it is not theoretical: the app holds
        this store for its whole session and ``_save_meta`` dumps the ENTIRE
        ``_data`` dict, so any edit made to the file from outside the app (an
        import tool, a hand-repair, a second process) is silently reverted by
        the app's next capture. Observed on ME3B_01: an import completed at
        17:16 and the running app's 17:28 capture wrote the file back from its
        own startup-era memory, dropping all 33 imported records.

        Safe because every mutator calls ``_save_meta`` immediately, so
        in-memory and on-disk never diverge in the other direction — there is
        never unflushed state for a reload to discard.

        ⚠ The read is UNCONDITIONAL, not gated on mtime. A first cut compared
        ``st_mtime`` against the value stamped by the last write and skipped the
        reload when they matched — but an external edit landing inside the same
        filesystem timestamp tick then reads as "unchanged" and is lost anyway,
        which a test caught. The file is ~20 kB of JSON against a multi-megabyte
        PNG write in the same call, so there is nothing to optimise here and a
        cheap-but-wrong guard is the worse trade.
        """
        self._load()

    # ── Write ─────────────────────────────────────────────────────

    def save_channel(
        self,
        plate_key,
        well_name,
        channel: str,
        image_bgr,
        extent_um: tuple[float, float, float, float],
        color_rgb: tuple[int, int, int] | None = None,
        objective: str = "",
        um_per_px: float = 0.0,
        mosaic_scale: float = 0.0,
        frames: int = 0,
        exposure_us: float = 0.0,
        shift_um: tuple[float, float] = (0.0, 0.0),
        display_levels: tuple[float, float] | None = None,
        avg_frames: int = 1,
        gain_pct: float | None = None,
        cube_slot: int | None = None,
        cube_label: str = "",
    ) -> bool:
        """Persist one channel's stitched single-well mosaic.

        ``image_bgr`` is the composite (numpy BGR uint8). ``extent_um`` is its
        world extent ``(min_x, min_y, max_x, max_y)`` in **absolute stage µm**
        (``MosaicBuilder.canvas_extent_um``). ``color_rgb`` is the display
        pseudo-colour (defaults to the channel's standard colour). Returns True
        on a successful image + metadata write.

        ``shift_um`` (v7.8) records the global registration shift BAKED INTO
        ``extent_um`` (``MosaicBuilder._global_shift_um``): the extent is
        display-registered (shifted) while tile PIXELS sit in the trusted raw
        stage frame, so any px → stage-µm back-projection must use
        ``extent[:2] − shift_um``. Persisting it is what makes a store-loaded
        mosaic safe to drive the stage from. Legacy entries lack the key →
        :meth:`get_shift_um` returns (0, 0) and :meth:`has_shift` False.
        """
        if not _CV2 or image_bgr is None:
            logger.warning("FluorescenceMosaicStore.save_channel: no cv2 / empty image")
            return False
        self._reload_if_changed()
        wkey = well_key(plate_key, well_name)
        now = datetime.now()
        fname = self._unique_image_name(plate_key, well_name, channel, now)
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            ok = cv2.imwrite(str(self._img_dir / fname), image_bgr)
            if not ok:
                logger.error("FluorescenceMosaicStore.save_channel: imwrite failed")
                return False
        except Exception as exc:
            logger.error(
                f"FluorescenceMosaicStore.save_channel: image write failed: {exc}")
            return False

        if color_rgb is None:
            color_rgb = default_color(channel)
        ex = [float(v) for v in extent_um]
        try:
            sh = [float(shift_um[0]), float(shift_um[1])]
        except (TypeError, ValueError, IndexError):
            sh = [0.0, 0.0]
        wells = self._data.setdefault("wells", {})
        entry = wells.setdefault(wkey, {
            "plate_key": str(plate_key),
            "well_name": str(well_name),
            "channels": {},
        })
        entry["plate_key"] = str(plate_key)
        entry["well_name"] = str(well_name)
        if objective:
            entry["objective"] = str(objective)
        entry["date"] = date.today().isoformat()
        ch_entry = {
            "image": f"fluor_mosaics/{fname}",
            "color": [int(c) for c in color_rgb],
            "extent_um": ex,
            "shift_um": sh,
            "um_per_px": float(um_per_px),
            "mosaic_scale": float(mosaic_scale),
            "frames": int(frames),
            "exposure_us": float(exposure_us),
            "date": now.date().isoformat(),
            # v7.21.6 — full capture instant, so two scans of the same well on
            # the same day are distinguishable (``date`` alone was not enough,
            # and it is kept for every pre-v7.21.6 reader).
            "captured_at": now.isoformat(timespec="seconds"),
        }
        # v7.13 — quantitative-capture metadata. display_lo/hi are the FROZEN
        # per-channel mono16→8-bit levels every averaged tile was converted
        # with (absent for legacy / single autoscaled captures — absent means
        # UNKNOWN, never assumed); avg_frames is the per-tile raw average
        # count. A fresh save also drops any stale processed sibling — the
        # raw stitch just changed, so a previous run's *_proc.png must not
        # keep shadowing it (attach_processed() re-adds it afterwards).
        if display_levels is not None:
            try:
                ch_entry["display_lo"] = float(display_levels[0])
                ch_entry["display_hi"] = float(display_levels[1])
            except (TypeError, ValueError, IndexError):
                pass
        if int(avg_frames) > 1:
            ch_entry["avg_frames"] = int(avg_frames)
        # v7.19 — the rest of the capture recipe, and WHICH CUBE was really in
        # the light path. The cube fields are read back from the body, so they
        # are ABSENT when it could not be read rather than echoing what was
        # asked for: a channel labelled with a cube that was not fitted is a
        # wrong fact nothing downstream can detect. All conditional, so a
        # capture that knows none of them round-trips byte-identically to a
        # pre-v7.19 one.
        if gain_pct is not None:
            try:
                ch_entry["gain_pct"] = float(gain_pct)
            except (TypeError, ValueError):
                pass
        if cube_slot:
            try:
                ch_entry["cube_slot"] = int(cube_slot)
            except (TypeError, ValueError):
                pass
        if cube_label:
            ch_entry["cube_label"] = str(cube_label)
        # v7.21.6 — the previous capture is ARCHIVED, not discarded. Its image
        # file is a different name now (see channel_image_name), so it is still
        # on disk either way; recording it keeps it FINDABLE (and deletable)
        # instead of leaving an orphan PNG nothing in the store points at.
        channels = entry.setdefault("channels", {})
        prev = channels.get(str(channel))
        channels[str(channel)] = ch_entry
        if isinstance(prev, dict) and prev.get("image"):
            ch_entry["history"] = ([_archived(prev)]
                                   + list(prev.get("history") or []))
        self._save_meta()
        logger.info(
            f"FluorescenceMosaicStore: saved {plate_key}/{well_name}/{channel} "
            f"({fname}, extent={ex}, shift={sh})"
            + (f"; archived {len(ch_entry.get('history') or [])} earlier "
               f"capture(s)" if ch_entry.get("history") else ""))
        return True

    def _unique_image_name(self, plate_key, well_name, channel,
                           when: Optional[datetime] = None) -> str:
        """A filename that does not exist yet — never overwrite pixels.

        The stamp is second-resolution, so two captures inside one second (or a
        clock stepped backwards) would still collide; a ``-2``, ``-3``, …
        disambiguator makes the no-overwrite guarantee absolute rather than
        merely likely.
        """
        when = when or datetime.now()
        base = channel_image_name(plate_key, well_name, channel,
                                  capture_stamp(when))
        if not (self._img_dir / base).exists():
            return base
        stem = base[:-4]
        for n in range(2, 1000):
            cand = f"{stem}-{n}.png"
            if not (self._img_dir / cand).exists():
                return cand
        return f"{stem}-{when.strftime('%f')}.png"

    def set_channel_color(self, plate_key, well_name, channel: str,
                          color_rgb: tuple[int, int, int]) -> bool:
        """Update the stored display pseudo-colour for one channel."""
        self._reload_if_changed()
        ch = self._channel_meta(plate_key, well_name, channel)
        if ch is None:
            return False
        ch["color"] = [int(c) for c in color_rgb]
        self._save_meta()
        return True

    def attach_processed(self, plate_key, well_name, channel: str,
                         image_bgr, processing: dict) -> bool:
        """v7.13 — store a POST-PROCESSED copy beside the raw channel stitch.

        Non-destructive by design: the raw PNG stays the canonical ``image``
        entry (detection and re-processing always have the untouched stitch);
        the processed copy is written as ``*_proc.png`` and preferred by the
        display overlays. ``processing`` records exactly what was applied
        (see FluorescencePostProcess.process) so the image can say what was
        done to it.
        """
        if not _CV2 or image_bgr is None:
            return False
        self._reload_if_changed()
        ch = self._channel_meta(plate_key, well_name, channel)
        if ch is None:
            return False
        # v7.21.6 — derive the name from THIS capture's raw stem so the pair
        # cannot come apart. With a fixed name a fresh raw stitch would inherit
        # the previous run's processed file, and the overlays prefer the
        # processed one — i.e. the display would show a stale image while the
        # store said it had a new one.
        raw = str(ch.get("image") or "").rsplit("/", 1)[-1]
        stem = raw[:-4] if raw.lower().endswith(".png") else raw
        if not stem:
            stem = (f"{_safe_token(plate_key)}_{_safe_token(well_name)}_"
                    f"{_safe_token(channel)}")
        fname = f"{stem}_proc.png"
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(self._img_dir / fname), image_bgr):
                return False
        except Exception as exc:
            logger.error(f"FluorescenceMosaicStore.attach_processed: {exc}")
            return False
        ch["processed_image"] = f"fluor_mosaics/{fname}"
        ch["processing"] = dict(processing or {})
        self._save_meta()
        return True

    # ── v7.13 quantitative-capture metadata getters ───────────────

    def get_display_levels(self, plate_key, well_name, channel=None):
        """Frozen (lo, hi) display levels a channel was converted with, or
        None for legacy / autoscaled captures (absent = unknown)."""
        for ch in self._channels_for(plate_key, well_name, channel):
            lo, hi = ch.get("display_lo"), ch.get("display_hi")
            if lo is not None and hi is not None:
                try:
                    return (float(lo), float(hi))
                except (TypeError, ValueError):
                    return None
        return None

    def get_avg_frames(self, plate_key, well_name, channel=None) -> int:
        for ch in self._channels_for(plate_key, well_name, channel):
            try:
                return max(1, int(ch.get("avg_frames", 1)))
            except (TypeError, ValueError):
                return 1
        return 1

    def get_exposure_us(self, plate_key, well_name, channel=None) -> float:
        for ch in self._channels_for(plate_key, well_name, channel):
            try:
                return float(ch.get("exposure_us", 0.0))
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def get_processing(self, plate_key, well_name, channel=None):
        """The applied post-processing record for a channel, or None."""
        for ch in self._channels_for(plate_key, well_name, channel):
            p = ch.get("processing")
            if isinstance(p, dict):
                return dict(p)
        return None

    # ── v7.13 per-well focus survey (the CRITICAL SAMPLE SURFACE) ─

    def set_focus_survey(self, plate_key, well_name, samples,
                         summary: dict | None = None,
                         model: str = "plane") -> bool:
        """Persist a mosaic focus survey for one well.

        ``samples`` are the accepted (x_um, y_um, focus_um[, …]) dicts from
        the scan's MosaicFocusTracker. This is the operator's CRITICAL SAMPLE
        SURFACE — where the cells are, commonly ABOVE the well bottom (e.g.
        on hydrogel) — and is deliberately a separate artifact from the
        plate-bottom Z plane (v7.11 wizard). The raw samples are kept so any
        surface model can be re-fit later; ``model`` records the operator's
        chosen evaluation model (plane / linear / spline).
        """
        self._reload_if_changed()
        wkey = well_key(plate_key, well_name)
        wells = self._data.setdefault("wells", {})
        entry = wells.setdefault(wkey, {
            "plate_key": str(plate_key),
            "well_name": str(well_name),
            "channels": {},
        })
        try:
            clean = []
            for s_ in (samples or ()):
                d = dict(s_) if isinstance(s_, dict) else dict(
                    getattr(s_, "to_dict", lambda: {})())
                if not d:
                    continue
                clean.append({k: (float(v) if isinstance(v, (int, float))
                                  else v) for k, v in d.items()})
        except Exception:
            return False
        entry["focus_survey"] = {
            "samples": clean,
            "summary": dict(summary or {}),
            "model": str(model or "plane"),
            "date": date.today().isoformat(),
        }
        self._save_meta()
        logger.info(
            f"FluorescenceMosaicStore: focus survey saved for "
            f"{plate_key}/{well_name} ({len(clean)} samples, model={model})")
        return True

    def get_focus_survey(self, plate_key, well_name) -> Optional[dict]:
        well = self.get_well(plate_key, well_name)
        if not well:
            return None
        fs = well.get("focus_survey")
        return dict(fs) if isinstance(fs, dict) else None

    def set_surface_model(self, plate_key, well_name, model: str) -> bool:
        self._reload_if_changed()
        well = self.get_well(plate_key, well_name)
        if not well or not isinstance(well.get("focus_survey"), dict):
            return False
        well["focus_survey"]["model"] = str(model or "plane")
        self._save_meta()
        return True

    # ── Read (metadata) ───────────────────────────────────────────

    def get_well(self, plate_key, well_name) -> Optional[dict]:
        """Return the metadata dict for one (plate, well) (or None)."""
        return self._data.get("wells", {}).get(well_key(plate_key, well_name))

    def _channel_meta(self, plate_key, well_name, channel) -> Optional[dict]:
        well = self.get_well(plate_key, well_name)
        if not well:
            return None
        return well.get("channels", {}).get(str(channel))

    def list_channels(self, plate_key, well_name) -> list[str]:
        well = self.get_well(plate_key, well_name)
        if not well:
            return []
        return list(well.get("channels", {}).keys())

    def list_wells(self, plate_key) -> list[str]:
        """Well names that have at least one stored channel for ``plate_key``."""
        out: list[str] = []
        for entry in self._data.get("wells", {}).values():
            if str(entry.get("plate_key")) == str(plate_key) and entry.get("channels"):
                out.append(str(entry.get("well_name")))
        return out

    def channel_color(self, plate_key, well_name, channel) -> Optional[tuple[int, int, int]]:
        ch = self._channel_meta(plate_key, well_name, channel)
        if not ch or "color" not in ch:
            return None
        c = ch["color"]
        if not (isinstance(c, (list, tuple)) and len(c) == 3):
            return None
        return tuple(int(v) for v in c)

    def get_extent_um(self, plate_key, well_name, channel=None) -> Optional[tuple]:
        """Extent of one channel, or (channel=None) of the first stored channel
        — all channels of a well share the same raster grid + extent."""
        well = self.get_well(plate_key, well_name)
        if not well:
            return None
        channels = well.get("channels", {})
        if channel is not None:
            ch = channels.get(str(channel))
            target = [ch] if ch else []
        else:
            target = list(channels.values())
        for ch in target:
            ex = ch.get("extent_um")
            if isinstance(ex, (list, tuple)) and len(ex) == 4:
                return tuple(float(v) for v in ex)
        return None

    def get_shift_um(self, plate_key, well_name, channel=None) -> tuple:
        """Global registration shift baked into the stored extent, ``(0, 0)``
        for legacy entries. Back-projection frame = ``extent[:2] − shift``.

        Mirrors ``MosaicStore.get_shift_um`` deliberately — the two stores hold
        the same contract and must not drift. ``channel=None`` reads the first
        stored channel (all channels of a well share one raster grid).
        """
        for ch in self._channels_for(plate_key, well_name, channel):
            sh = ch.get("shift_um")
            if isinstance(sh, (list, tuple)) and len(sh) >= 2:
                try:
                    return (float(sh[0]), float(sh[1]))
                except (TypeError, ValueError):
                    return (0.0, 0.0)
        return (0.0, 0.0)

    def has_shift(self, plate_key, well_name, channel=None) -> bool:
        """True when the registration shift was RECORDED for this mosaic.

        Distinct from ``get_shift_um() == (0, 0)``, which is also what a legacy
        entry returns: only a recorded shift makes the pixel → stage mapping
        trustworthy enough to command motion from. Callers that drive the stage
        must gate on this, not on the shift's value.
        """
        for ch in self._channels_for(plate_key, well_name, channel):
            sh = ch.get("shift_um")
            if isinstance(sh, (list, tuple)) and len(sh) >= 2:
                return True
        return False

    def _channels_for(self, plate_key, well_name, channel=None) -> list[dict]:
        """Channel dicts to consult: the named one, else every stored channel."""
        well = self.get_well(plate_key, well_name)
        if not well:
            return []
        channels = well.get("channels", {})
        if channel is not None:
            ch = channels.get(str(channel))
            return [ch] if isinstance(ch, dict) else []
        return [c for c in channels.values() if isinstance(c, dict)]

    def get_mosaic_scale(self, plate_key, well_name, channel=None):
        """Stored mosaic px-per-µm for a channel (or None).

        Pair it with :meth:`get_extent_um` from the SAME channel — the two are
        only mutually consistent per channel.
        """
        for ch in self._channels_for(plate_key, well_name, channel):
            val = ch.get("mosaic_scale")
            try:
                scale = float(val)
            except (TypeError, ValueError):
                continue
            if scale > 0:
                return scale
        return None

    def get_um_per_px(self, plate_key, well_name, channel=None):
        """Stored camera µm/px for a channel (or None)."""
        for ch in self._channels_for(plate_key, well_name, channel):
            val = ch.get("um_per_px")
            try:
                eff = float(val)
            except (TypeError, ValueError):
                continue
            if eff > 0:
                return eff
        return None

    def get_objective(self, plate_key, well_name) -> str:
        """Objective the well was scanned with ("" when unknown)."""
        well = self.get_well(plate_key, well_name)
        return str(well.get("objective", "")) if well else ""

    def has(self, plate_key, well_name) -> bool:
        return bool(self.list_channels(plate_key, well_name))

    # ── Read (images) ─────────────────────────────────────────────

    def _abs_image_path(self, rel: str) -> Optional[str]:
        if not rel:
            return None
        p = self._path.parent / rel.replace("fluor_mosaics/", "fluor_mosaics" + os.sep)
        return str(p) if p.exists() else None

    def load_channel_image(self, plate_key, well_name, channel,
                           prefer_processed: bool = False):
        """Return one channel's stored composite as a numpy BGR array (or None).

        ``prefer_processed`` (v7.13): load the post-processed sibling when one
        exists, falling back to the raw stitch when its file is missing. The
        default stays False DELIBERATELY — detection code reads raw channels
        and must never have its input silently swapped for a denoised /
        background-subtracted copy; only the display overlays opt in.
        """
        if not _CV2:
            return None
        ch = self._channel_meta(plate_key, well_name, channel)
        if not ch:
            return None
        paths = []
        if prefer_processed and ch.get("processed_image"):
            paths.append(ch.get("processed_image"))
        paths.append(ch.get("image", ""))
        for rel in paths:
            path = self._abs_image_path(rel)
            if path is None:
                continue
            try:
                img = cv2.imread(path)
            except Exception as exc:
                logger.warning(
                    f"FluorescenceMosaicStore.load_channel_image failed: {exc}")
                img = None
            if img is not None:
                return img
        return None

    # ── Blending ──────────────────────────────────────────────────

    def composite_overlay(self, plate_key, well_name, channels=None):
        """Blend the stored channels of one well into a single BGR overlay.

        Each channel image is converted to a grayscale intensity, tinted by its
        display colour, and additively combined (saturating) — the standard
        fluorescence false-colour merge. ``channels`` optionally restricts to a
        subset (defaults to all stored). Returns ``(image_bgr, extent_um)`` or
        ``(None, None)`` if nothing is available.
        """
        if not _CV2:
            return None, None
        well = self.get_well(plate_key, well_name)
        if not well:
            return None, None
        names = list(channels) if channels else list(well.get("channels", {}).keys())
        acc = None
        extent = None
        shape = None
        for name in names:
            # v7.13: overlays prefer the post-processed copy when one exists
            # (denoise / background subtraction); detection paths keep loading
            # the raw stitch via load_channel_image's default.
            img = self.load_channel_image(plate_key, well_name, name,
                                          prefer_processed=True)
            if img is None:
                continue
            color = self.channel_color(plate_key, well_name, name) or default_color(name)
            if shape is None:
                shape = img.shape[:2]
                extent = self.get_extent_um(plate_key, well_name, name) or extent
            elif img.shape[:2] != shape:
                img = cv2.resize(img, (shape[1], shape[0]),
                                 interpolation=cv2.INTER_AREA)
            tinted = _tint_gray(img, color)
            acc = tinted if acc is None else cv2.add(acc, tinted)
        if acc is None:
            return None, None
        return acc, extent

    def composite_plate_overlay(self, plate_key, channels=None, target_px: int = 3000):
        """Blend EVERY stored well of a plate onto one canvas spanning their
        union extent (absolute stage µm), so a single registered overlay shows
        all captured wells. Returns ``(image_bgr, extent_um)`` or ``(None, None)``.
        """
        if not _CV2:
            return None, None
        items = []  # (well_bgr, extent)
        union = None
        for well_name in self.list_wells(plate_key):
            img, ex = self.composite_overlay(plate_key, well_name, channels)
            if img is None or ex is None:
                continue
            items.append((img, ex))
            if union is None:
                union = list(ex)
            else:
                union[0] = min(union[0], ex[0])
                union[1] = min(union[1], ex[1])
                union[2] = max(union[2], ex[2])
                union[3] = max(union[3], ex[3])
        if not items or union is None:
            return None, None
        w_um = max(1.0, union[2] - union[0])
        h_um = max(1.0, union[3] - union[1])
        scale = float(target_px) / max(w_um, h_um)   # px per µm
        cw = max(1, int(round(w_um * scale)))
        ch_ = max(1, int(round(h_um * scale)))
        canvas = np.zeros((ch_, cw, 3), dtype=np.uint8)
        for img, ex in items:
            tw = max(1, int(round((ex[2] - ex[0]) * scale)))
            th = max(1, int(round((ex[3] - ex[1]) * scale)))
            tile = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA)
            left = int(round((ex[0] - union[0]) * scale))
            top = int(round((ex[1] - union[1]) * scale))
            x0, y0 = max(0, left), max(0, top)
            x1, y1 = min(cw, left + tw), min(ch_, top + th)
            if x1 <= x0 or y1 <= y0:
                continue
            sub = tile[(y0 - top):(y1 - top), (x0 - left):(x1 - left)]
            canvas[y0:y1, x0:x1] = cv2.add(canvas[y0:y1, x0:x1], sub)
        return canvas, tuple(float(v) for v in union)

    # ── Delete ────────────────────────────────────────────────────

    # ── Capture history (v7.21.6) ─────────────────────────────────

    def list_history(self, plate_key, well_name, channel) -> list[dict]:
        """Earlier captures of this channel, NEWEST FIRST (never ``None``).

        Each entry is the channel record as it stood when it was superseded —
        its own ``image`` / ``extent_um`` / ``captured_at`` / exposure / cube,
        so an archived capture stays fully interpretable (and georeferenced)
        rather than being a bare filename.
        """
        ch = self._channel_meta(plate_key, well_name, channel)
        if not ch:
            return []
        return [dict(h) for h in (ch.get("history") or []) if isinstance(h, dict)]

    def history_count(self, plate_key, well_name, channel) -> int:
        return len(self.list_history(plate_key, well_name, channel))

    def add_history_entry(self, plate_key, well_name, channel,
                          record: dict) -> bool:
        """Register an already-on-disk capture as an ARCHIVED one.

        Used by the importer that folds another machine folder's mosaics in.
        Its ``image`` must already be a ``fluor_mosaics/<name>.png`` relative
        path that exists; the list is re-sorted newest-first afterwards so a
        back-filled older capture lands in the right place.
        """
        self._reload_if_changed()
        ch = self._channel_meta(plate_key, well_name, channel)
        if ch is None or not isinstance(record, dict) or not record.get("image"):
            return False
        hist = [h for h in (ch.get("history") or []) if isinstance(h, dict)]
        hist.append(_archived(record))
        hist.sort(key=lambda h: str(h.get("captured_at")
                                    or h.get("date") or ""), reverse=True)
        ch["history"] = hist
        self._save_meta()
        return True

    def restore_history(self, plate_key, well_name, channel, which) -> bool:
        """Promote an archived capture back to ACTIVE (a swap, never a delete).

        ``which`` is an index into :meth:`list_history` or an ``image`` path.
        The capture being displaced goes into the history in its place, so this
        is reversible and no image is ever dropped.
        """
        self._reload_if_changed()
        ch = self._channel_meta(plate_key, well_name, channel)
        if ch is None:
            return False
        hist = [h for h in (ch.get("history") or []) if isinstance(h, dict)]
        idx = which if isinstance(which, int) else next(
            (i for i, h in enumerate(hist)
             if str(h.get("image", "")).endswith(str(which))), -1)
        if not (0 <= idx < len(hist)):
            return False
        promoted = _archived(hist.pop(idx))
        hist.insert(0, _archived(ch))
        hist.sort(key=lambda h: str(h.get("captured_at")
                                    or h.get("date") or ""), reverse=True)
        promoted["history"] = hist
        self.get_well(plate_key, well_name)["channels"][str(channel)] = promoted
        self._save_meta()
        logger.info("FluorescenceMosaicStore: restored %s/%s/%s from %s",
                    plate_key, well_name, channel, promoted.get("captured_at")
                    or promoted.get("date"))
        return True

    def clear_channel(self, plate_key, well_name, channel,
                      include_history: bool = True) -> None:
        """Forget a channel. ``include_history`` also deletes its archive.

        Default True so a deliberate delete does not leave the archive behind
        as invisible disk usage; pass False to drop only the active capture and
        keep the earlier ones (they stay findable — the newest is promoted).
        """
        self._reload_if_changed()
        well = self.get_well(plate_key, well_name)
        if not well:
            return
        ch = well.get("channels", {}).pop(str(channel), None)
        if not ch:
            self._save_meta()
            return
        hist = [h for h in (ch.get("history") or []) if isinstance(h, dict)]
        if ch.get("image"):
            self._unlink(ch["image"])
        if ch.get("processed_image"):
            self._unlink(ch["processed_image"])
        if include_history:
            for h in hist:
                if h.get("image"):
                    self._unlink(h["image"])
                if h.get("processed_image"):
                    self._unlink(h["processed_image"])
        elif hist:
            promoted = _archived(hist[0])
            promoted["history"] = hist[1:]
            well.setdefault("channels", {})[str(channel)] = promoted
        if not well.get("channels"):
            self._data.get("wells", {}).pop(well_key(plate_key, well_name), None)
        self._save_meta()

    def clear_well(self, plate_key, well_name) -> None:
        self._reload_if_changed()
        well = self._data.get("wells", {}).pop(well_key(plate_key, well_name), None)
        if not well:
            return
        for ch in well.get("channels", {}).values():
            for rec in [ch] + [h for h in (ch.get("history") or [])
                               if isinstance(h, dict)]:
                if rec.get("image"):
                    self._unlink(rec["image"])
                if rec.get("processed_image"):
                    self._unlink(rec["processed_image"])
        self._save_meta()

    def _unlink(self, rel: str) -> None:
        try:
            p = self._path.parent / rel.replace(
                "fluor_mosaics/", "fluor_mosaics" + os.sep)
            if p.exists():
                p.unlink()
        except Exception as exc:
            logger.debug(f"FluorescenceMosaicStore: image unlink failed: {exc}")


def _tint_gray(image_bgr, color_rgb):
    """Grayscale-intensity → BGR tinted by ``color_rgb`` (RGB 0-255)."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    r, g, b = (float(c) for c in color_rgb)
    # OpenCV is BGR.
    out = np.empty((*gray.shape, 3), dtype=np.uint8)
    out[..., 0] = np.clip(gray * b, 0, 255).astype(np.uint8)
    out[..., 1] = np.clip(gray * g, 0, 255).astype(np.uint8)
    out[..., 2] = np.clip(gray * r, 0, 255).astype(np.uint8)
    return out


_store_singleton: Optional[FluorescenceMosaicStore] = None


def get_store(path=None) -> FluorescenceMosaicStore:
    """Process-wide singleton (lazy).

    An explicit ``path`` re-creates the singleton against that file — the hook
    tests use for isolation (``MEBP_FLUOR_MOSAIC_PATH`` does the same for the
    default construction).
    """
    global _store_singleton
    if path is not None:
        _store_singleton = FluorescenceMosaicStore(Path(path))
    elif _store_singleton is None:
        _store_singleton = FluorescenceMosaicStore()
    return _store_singleton
