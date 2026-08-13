"""
FluorescenceMosaicStore.py — Per-(plate, well) multi-channel fluorescence mosaics.

v7.5.x: The new **Fluorescence Mosaic** workflow rasters a single well at high
resolution once per fluorescence channel (DAPI / FITC / mCherry / Cy5 / …). The
operator switches the physical filter/illumination between channels (there is no
filter-wheel hardware), so each channel is captured as its own full single-well
mosaic. Every channel of a well shares the SAME raster grid + camera scale, so
their composites register pixel-for-pixel and can be overlaid in any pseudo-colour.

The captured mosaics are a property of the *physical plate on the stage* (their
absolute stage-µm extent matters) — so, like ``MosaicStore`` / objectives.json,
they are persisted at the machine level (not inside the swappable
``HardwareConfig``). Any other workflow (Spheroid Pick & Place, Cell Targeting,
Cell Labeling, Quick Print, the Jog plate view, …) can then look up a well and
draw the blended fluorescence image as a registered background overlay.

Files:
    config/hardware/fluorescence_mosaics.json          — metadata
    config/hardware/fluor_mosaics/<plate>_<well>_<channel>.png  — per-channel BGR

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
              "image": "fluor_mosaics/24_A1_DAPI.png",  # relative to config dir
              "color": [0, 0, 255],          # display pseudo-colour, RGB 0-255
              "extent_um": [min_x, min_y, max_x, max_y],   # absolute stage µm
              "shift_um": [dx, dy],          # registration shift baked into extent
              "um_per_px": 0.92, "mosaic_scale": 0.31,
              "frames": 36, "exposure_us": 0, "date": "2026-06-22"
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
from datetime import date
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


def well_key(plate_key, well_name) -> str:
    """Composite metadata key for one (plate, well)."""
    return f"{plate_key}|{well_name}"


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
        except Exception as exc:
            logger.error(
                f"FluorescenceMosaicStore: failed to save metadata: {exc}")

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
        wkey = well_key(plate_key, well_name)
        fname = f"{_safe_token(plate_key)}_{_safe_token(well_name)}_{_safe_token(channel)}.png"
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
            "date": date.today().isoformat(),
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
        entry.setdefault("channels", {})[str(channel)] = ch_entry
        self._save_meta()
        logger.info(
            f"FluorescenceMosaicStore: saved {plate_key}/{well_name}/{channel} "
            f"({fname}, extent={ex}, shift={sh})")
        return True

    def set_channel_color(self, plate_key, well_name, channel: str,
                          color_rgb: tuple[int, int, int]) -> bool:
        """Update the stored display pseudo-colour for one channel."""
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
        ch = self._channel_meta(plate_key, well_name, channel)
        if ch is None:
            return False
        fname = (f"{_safe_token(plate_key)}_{_safe_token(well_name)}_"
                 f"{_safe_token(channel)}_proc.png")
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

    def clear_channel(self, plate_key, well_name, channel) -> None:
        well = self.get_well(plate_key, well_name)
        if not well:
            return
        ch = well.get("channels", {}).pop(str(channel), None)
        if ch and ch.get("image"):
            self._unlink(ch["image"])
        if not well.get("channels"):
            self._data.get("wells", {}).pop(well_key(plate_key, well_name), None)
        self._save_meta()

    def clear_well(self, plate_key, well_name) -> None:
        well = self._data.get("wells", {}).pop(well_key(plate_key, well_name), None)
        if not well:
            return
        for ch in well.get("channels", {}).values():
            if ch.get("image"):
                self._unlink(ch["image"])
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
