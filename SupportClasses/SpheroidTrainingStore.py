"""
SpheroidTrainingStore.py — banked spheroid crops for future detector training.

v7.8: when the operator selects a spheroid to pick, a fresh microscope frame is
cropped to that spheroid's measured circle and saved here with the geometry
needed to interpret it. This module ONLY stores images + metadata — there is
deliberately no training system, no model, no inference.

Files::

    config/hardware/spheroid_training.json                      — index
    config/hardware/spheroid_training/<plate>_<well>_<stamp>_<n>.png

Conventions copied from ``NeedleFocusTemplateStore`` / ``WellTrainingStore``:
images are raw BGR uint8, the JSON records the path RELATIVE to the config dir
(so the whole config tree relocates), filenames go through a sanitising regex,
the index is written atomically, and ``MEBP_SPHEROID_TRAINING_DIR`` redirects
BOTH the JSON and the image dir so tests never touch the repo.

Unlike the needle store there is no per-key cap — this corpus is meant to
accumulate — but a total ``max_samples`` with oldest-first trim (and image
unlink) keeps a stuck loop from filling the disk.

What must be true of a sample, or it is worse than no sample
-----------------------------------------------------------
* **The crop comes from the RAW camera frame**, never from a display pixmap.
  ``TargetOverlayCameraView`` paints the crosshair and the target rings onto its
  pixmap, so cropping the display would bake our own annotation ring into every
  training image. The raw frame is also the frame ``um_per_px`` and
  ``pixel_to_stage_offset`` are defined against.
* **The stage must actually be on the spheroid.** Nothing else stops a crop of a
  different location being filed under this spheroid's diameter, and a
  mislabelled sample is worse than a missing one — hence
  :func:`stage_is_on_target`.
* **A clipped circle is refused by default.** Half a spheroid mislabels the
  diameter. Padding is clamped to the frame rather than filled, because a
  synthetic black border is an artefact a detector would learn.
* ``um_per_px`` is recorded because it is the only thing that makes ``radius_px``
  and ``diameter_um`` mutually consistent across objectives.
* ``detection_source`` (auto / redrawn / manual) is recorded so weak labels can
  be found offline.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import cv2
    _CV2 = True
except ImportError:   # pragma: no cover - cv2 always present in this project
    cv2 = None
    _CV2 = False

_DEFAULT_PATH = Path("config/hardware/spheroid_training.json")
_IMG_SUBDIR = "spheroid_training"

# A backstop, not a policy: the corpus is meant to grow, but a stuck loop must
# not fill the disk. Oldest samples are trimmed (and their PNGs unlinked).
MAX_SAMPLES = 5000

# Fraction of the radius added around the circle. A zero-context tight bounding
# box is a poor training crop — a detector has to see the edge transition — while
# 25 % is not enough to pull in a neighbouring spheroid.
DEFAULT_PAD_FRAC = 0.25

# Provenance of the diameter on a saved crop.
SOURCE_AUTO = "auto"
SOURCE_REDRAWN = "redrawn"
SOURCE_MANUAL = "manual"


def _safe_token(value) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "x"


# ── pure crop geometry (unit-testable without a camera) ────────────

@dataclass(frozen=True)
class CropRect:
    """A crop window in raw frame pixels, plus how to read it back.

    ``center_px_in_crop`` is the spheroid centre relative to the CLAMPED origin,
    so it stays correct when the window was pushed back inside the frame.
    ``clipped`` means the padded window hit a frame edge; ``circle_clipped``
    means the spheroid's own circle did — a much stronger objection, since the
    measured diameter then describes something not fully in view.
    """
    x0: int
    y0: int
    x1: int
    y1: int
    center_px_in_crop: tuple[float, float]
    radius_px: float
    clipped: bool
    circle_clipped: bool

    @property
    def width(self) -> int:
        return max(0, self.x1 - self.x0)

    @property
    def height(self) -> int:
        return max(0, self.y1 - self.y0)

    def is_empty(self) -> bool:
        return self.width <= 0 or self.height <= 0


def target_center_px(target_um, stage_um, um_per_px: float,
                     frame_wh) -> tuple[float, float]:
    """Absolute stage µm → raw frame pixel of that point.

    Identical projection to the live overlay's (``_draw_marker_set``): the camera
    centre is the stage position, so an offset in µm divided by µm/px is an
    offset in px from the frame centre.
    """
    eff = float(um_per_px)
    if eff <= 0:
        raise ValueError("um_per_px must be > 0 to project a target")
    w, h = float(frame_wh[0]), float(frame_wh[1])
    cx = (float(target_um[0]) - float(stage_um[0])) / eff + w / 2.0
    cy = (float(target_um[1]) - float(stage_um[1])) / eff + h / 2.0
    return (cx, cy)


def crop_rect_for_circle(center_px, radius_px: float, frame_wh,
                         pad_frac: float = DEFAULT_PAD_FRAC) -> CropRect:
    """The clamped crop window around a circle, with clipping flagged.

    Clamps to the frame rather than padding, and reports the two clipping cases
    separately so a caller can accept a trimmed margin while refusing a trimmed
    spheroid.
    """
    w = int(frame_wh[0])
    h = int(frame_wh[1])
    r = max(0.0, float(radius_px))
    half = r * (1.0 + max(0.0, float(pad_frac)))
    cx, cy = float(center_px[0]), float(center_px[1])

    wx0 = int(math.floor(cx - half))
    wy0 = int(math.floor(cy - half))
    wx1 = int(math.ceil(cx + half))
    wy1 = int(math.ceil(cy + half))

    x0, y0 = max(0, wx0), max(0, wy0)
    x1, y1 = min(w, wx1), min(h, wy1)
    clipped = (x0 != wx0) or (y0 != wy0) or (x1 != wx1) or (y1 != wy1)
    circle_clipped = (cx - r < 0) or (cy - r < 0) or (cx + r > w) or (cy + r > h)
    return CropRect(
        x0=x0, y0=y0, x1=x1, y1=y1,
        center_px_in_crop=(cx - x0, cy - y0),
        radius_px=r, clipped=clipped, circle_clipped=circle_clipped)


def stage_is_on_target(target_um, stage_um, um_per_px: float, frame_wh,
                       tolerance_frac: float = 0.5) -> bool:
    """Is the target actually inside (a fraction of) the current field of view?

    The guard against filing a crop of somewhere else under this spheroid's
    diameter. Default half a FOV — comfortably inside the frame, so a small
    settle error still passes.
    """
    eff = float(um_per_px)
    if eff <= 0:
        return False
    half_w_um = (float(frame_wh[0]) / 2.0) * eff * float(tolerance_frac)
    half_h_um = (float(frame_wh[1]) / 2.0) * eff * float(tolerance_frac)
    dx = abs(float(target_um[0]) - float(stage_um[0]))
    dy = abs(float(target_um[1]) - float(stage_um[1]))
    return dx <= half_w_um and dy <= half_h_um


def refuse_crop_reason(target_um, stage_um, um_per_px: float, frame_wh,
                       radius_px: float, *, pad_frac=DEFAULT_PAD_FRAC,
                       skip_if_circle_clipped: bool = True,
                       tolerance_frac: float = 0.5) -> Optional[str]:
    """Why this crop must not be saved, in operator language — or None.

    Refusing loudly beats banking a mislabelled sample nobody notices.
    """
    if not _CV2:
        return "OpenCV is not available, so a crop cannot be written."
    try:
        eff = float(um_per_px)
    except (TypeError, ValueError):
        eff = 0.0
    if eff <= 0:
        return ("The camera has no µm/px calibration, so the crop could not be "
                "scaled. Calibrate the objective first.")
    if float(radius_px) <= 0:
        return "This spheroid has no measured diameter yet."
    if not stage_is_on_target(target_um, stage_um, eff, frame_wh,
                              tolerance_frac):
        return ("The stage is not on this spheroid — go to it first, so the "
                "saved image is actually of the spheroid it is labelled with.")
    rect = crop_rect_for_circle(
        target_center_px(target_um, stage_um, eff, frame_wh),
        radius_px, frame_wh, pad_frac)
    if rect.is_empty():
        return "The spheroid is outside the camera frame."
    if skip_if_circle_clipped and rect.circle_clipped:
        return ("The spheroid touches the edge of the frame — re-centre the "
                "stage on it and retry, so the measured diameter matches what "
                "is in the image.")
    return None


def crop_from_frame(frame_bgr, rect: CropRect):
    """Slice ``rect`` out of a raw BGR frame (a copy), or None if degenerate."""
    if frame_bgr is None or rect.is_empty():
        return None
    sub = frame_bgr[rect.y0:rect.y1, rect.x0:rect.x1]
    if getattr(sub, "size", 0) == 0:
        return None
    return sub.copy()


# ── the store ──────────────────────────────────────────────────────

class SpheroidTrainingStore:
    """Append-only corpus of spheroid crops + their measurement metadata."""

    def __init__(self, path: Path | None = None, max_samples: int = MAX_SAMPLES):
        if path is None:
            env = os.environ.get("MEBP_SPHEROID_TRAINING_DIR")
            path = (Path(env) / "spheroid_training.json" if env
                    else _DEFAULT_PATH)
        self._path = Path(path)
        self._img_dir = self._path.parent / _IMG_SUBDIR
        self._max_samples = max(1, int(max_samples))
        self._data: dict = {"version": "1.0", "samples": []}
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
            if not isinstance(self._data.get("samples"), list):
                self._data["samples"] = []
        except Exception as exc:
            logger.warning(
                f"SpheroidTrainingStore: failed to load {self._path}: {exc}")
            self._data = {"version": "1.0", "samples": []}

    def _save_meta(self) -> None:
        """Atomic write — a half-written index would orphan every crop."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = f"{self._path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"SpheroidTrainingStore: failed to save: {exc}")

    # ── write ─────────────────────────────────────────────────────

    def add_crop(self, crop_bgr, *,
                 diameter_um: float,
                 radius_px: float,
                 um_per_px: float,
                 center_px_in_crop: tuple,
                 crop_origin_px: tuple,
                 frame_wh: tuple,
                 stage_um: tuple,
                 well: str = "",
                 plate_key: str = "",
                 objective: str = "",
                 channel: str = "",
                 detection_source: str = SOURCE_AUTO,
                 user_edited: bool = False,
                 clipped: bool = False,
                 pad_frac: float = 0.0,
                 target_id: str = "",
                 z_zref_mm: float | None = None,
                 label: str = "") -> Optional[str]:
        """Bank one crop. Returns its relative image path, or None on failure.

        ``crop_bgr`` must come from the RAW camera frame (see the module
        docstring) — this method cannot tell an annotated pixmap from a real
        frame, so the caller owns that invariant.
        """
        if not _CV2 or crop_bgr is None or getattr(crop_bgr, "size", 0) == 0:
            logger.warning("SpheroidTrainingStore.add_crop: no cv2 / empty crop")
            return None
        samples = self._data.setdefault("samples", [])
        n = 1 + max((int(s.get("n", 0)) for s in samples), default=0)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = (f"{_safe_token(plate_key)}_{_safe_token(well)}_"
                 f"{stamp}_{n}.png")
        try:
            self._img_dir.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(self._img_dir / fname), crop_bgr):
                logger.error("SpheroidTrainingStore.add_crop: imwrite failed")
                return None
        except Exception as exc:
            logger.error(f"SpheroidTrainingStore.add_crop: write failed: {exc}")
            return None

        try:
            ch_, cw_ = crop_bgr.shape[:2]
        except Exception:
            ch_ = cw_ = 0
        rel = f"{_IMG_SUBDIR}/{fname}"
        samples.append({
            "n": n,
            "image": rel,
            "crop_wh": [int(cw_), int(ch_)],
            "center_px_in_crop": [float(center_px_in_crop[0]),
                                  float(center_px_in_crop[1])],
            "radius_px": float(radius_px),
            "diameter_um": float(diameter_um),
            "um_per_px": float(um_per_px),
            "crop_origin_px": [int(crop_origin_px[0]), int(crop_origin_px[1])],
            "frame_wh": [int(frame_wh[0]), int(frame_wh[1])],
            "pad_frac": float(pad_frac),
            "clipped": bool(clipped),
            "stage_um": [float(stage_um[0]), float(stage_um[1])],
            "z_zref_mm": (None if z_zref_mm is None else float(z_zref_mm)),
            "well": str(well or ""),
            "plate_key": str(plate_key or ""),
            "objective": str(objective or ""),
            "channel": str(channel or ""),
            "detection_source": str(detection_source or SOURCE_AUTO),
            "user_edited": bool(user_edited),
            "target_id": str(target_id or ""),
            "label": str(label or ""),
            "date": datetime.now().isoformat(timespec="seconds"),
        })
        self._trim()
        self._save_meta()
        logger.info(
            f"SpheroidTrainingStore: saved {rel} "
            f"(Ø{float(diameter_um):.0f} µm, {detection_source})")
        return rel

    def _trim(self) -> None:
        """Drop oldest samples (and their images) past ``max_samples``."""
        samples = self._data.get("samples", [])
        while len(samples) > self._max_samples:
            old = samples.pop(0)
            self._unlink(old.get("image", ""))

    def _unlink(self, rel: str) -> None:
        if not rel:
            return
        try:
            p = self._path.parent / rel.replace(
                f"{_IMG_SUBDIR}/", _IMG_SUBDIR + os.sep)
            if p.exists():
                p.unlink()
        except Exception as exc:
            logger.debug(f"SpheroidTrainingStore: unlink failed: {exc}")

    # ── read ──────────────────────────────────────────────────────

    def samples(self) -> list[dict]:
        """Stored samples whose image is still on disk."""
        return [s for s in self._data.get("samples", [])
                if self._abs_path(s.get("image", "")) is not None]

    def count(self) -> int:
        return len(self.samples())

    def total_bytes(self) -> int:
        """On-disk size of the surviving crops (shown in the UI so a growing
        corpus is visible rather than a surprise)."""
        total = 0
        for s in self._data.get("samples", []):
            p = self._abs_path(s.get("image", ""))
            if p is None:
                continue
            try:
                total += os.path.getsize(p)
            except OSError:
                pass
        return total

    def _abs_path(self, rel: str) -> Optional[str]:
        if not rel:
            return None
        p = self._path.parent / rel.replace(
            f"{_IMG_SUBDIR}/", _IMG_SUBDIR + os.sep)
        return str(p) if p.exists() else None

    def load_crop(self, entry: dict):
        """Read one sample's image back as a numpy BGR array (or None)."""
        if not _CV2:
            return None
        p = self._abs_path((entry or {}).get("image", ""))
        if p is None:
            return None
        try:
            return cv2.imread(p)
        except Exception as exc:
            logger.warning(f"SpheroidTrainingStore.load_crop failed: {exc}")
            return None

    def clear(self) -> None:
        """Delete every sample and its image."""
        for s in list(self._data.get("samples", [])):
            self._unlink(s.get("image", ""))
        self._data["samples"] = []
        self._save_meta()

    @property
    def image_dir(self) -> Path:
        return self._img_dir

    @property
    def path(self) -> Path:
        return self._path


_store_singleton: Optional[SpheroidTrainingStore] = None


def get_store(path=None) -> SpheroidTrainingStore:
    """Process-wide singleton (lazy). An explicit ``path`` rebuilds it."""
    global _store_singleton
    if path is not None:
        _store_singleton = SpheroidTrainingStore(Path(path))
    elif _store_singleton is None:
        _store_singleton = SpheroidTrainingStore()
    return _store_singleton
