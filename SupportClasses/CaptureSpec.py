"""
CaptureSpec.py — capture settings, filename templating, safe file creation.

v7.14. The operator asked for a destination picker and "a name picker with
optional meta data for the date, channel, objective (10x NA 0.3), etc." — so a
capture's filename is a TEMPLATE over the same metadata that gets embedded in
the file.

Pure: no Qt, no cv2, no hardware. Everything here is decided before a single
byte is written, which is what makes it testable without a camera.

Two rules worth stating up front because both have bitten this repo before:

* The sanitiser is deliberately MANY-TO-ONE — "10x NA 0.3" and "10x_NA_0.3"
  collapse to the same stem. That is fine for a filename and fatal for a key,
  so a capture NEVER trusts its rendered stem: :func:`open_unique` creates the
  file with ``O_CREAT | O_EXCL`` and hands back the path it actually got. Two
  distinct captures that sanitise identically become ``…_002``; they never
  overwrite each other, and there is no exists()-then-write window to lose.
* Booleans do not round-trip through ``type(default)(stored)`` — ``bool("false")``
  is ``True``. They get an explicit branch here.
"""

from __future__ import annotations

import os
import re
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, BinaryIO, Mapping, Optional

# Environment override for the capture destination (tests, and a machine that
# wants captures on a data drive without touching settings.json).
ENV_CAPTURE_DIR = "MEBP_CAPTURE_DIR"

# Single source of truth for capture configuration. The settings dialog builds
# itself from this rather than re-listing keys, so a new option cannot be half
# wired (the MOSAIC_SCAN_DEFAULTS idiom).
CAPTURE_DEFAULTS: dict = {
    # ── destination ───────────────────────────────────────────────
    "output_dir": "",              # "" → <repo>/captures ; env var wins
    "subfolder_by_date": True,     # captures/2026-08-07/
    # ── stills ────────────────────────────────────────────────────
    "still_source": "display",     # "display" (as seen) | "raw" (16-bit)
    "still_format": "png",         # "png" | "tiff"  (raw REQUIRES tiff)
    "still_template": "{date}_{time}_{camera}_{objective}",
    "still_raw_avg_frames": 1,     # 1..32, raw only (SNR ×√N)
    "still_full_res": False,       # momentary full-sensor switch, raw only
    # ── video ─────────────────────────────────────────────────────
    "video_source": "display",     # "display" | "raw_timelapse"
    "video_fps": 15.0,             # PLAYBACK rate — see plan_frame_repeats
    "video_quality": 80,           # 1..100 (honoured by MJPG, not by mp4v)
    "video_container": "mp4",      # "mp4" | "avi"
    "video_template": "{date}_{time}_{camera}",
    "video_max_seconds": 600,      # 0 = unlimited
    "video_max_gb": 8.0,           # 0 = unlimited
    "raw_timelapse_interval_s": 1.0,
    # ── metadata ──────────────────────────────────────────────────
    "embed_metadata": True,        # inside the PNG/TIFF itself
    "write_sidecar": True,         # …and always a readable .json beside it
    "operator": "",
    "notes": "",
}

# Filename tokens: (name, example, help) — drives the dialog's token palette so
# the list an operator sees cannot drift from the list that renders.
TOKENS: tuple[tuple[str, str, str], ...] = (
    ("date", "2026-08-07", "Capture date (YYYY-MM-DD)"),
    ("time", "142305", "Capture time (HHMMSS)"),
    ("datetime", "20260807_142305", "Date and time together"),
    ("camera", "Zyla", "Camera name"),
    ("cam_idx", "1", "Camera slot number"),
    ("objective", "10x NA 0.3", "Objective magnification and numerical aperture"),
    ("mag", "10x", "Objective magnification only"),
    ("channel", "FITC", "Fluorescence channel / filter cube"),
    ("well", "A1", "Well being imaged"),
    ("plate", "plate-24", "Plate type"),
    ("exposure_ms", "340", "Exposure in milliseconds"),
    ("um_per_px", "0.65", "Micrometres per pixel"),
    ("x_um", "105617", "Stage X (µm)"),
    ("y_um", "65890", "Stage Y (µm)"),
    ("operator", "alex", "Operator name from settings"),
    ("kind", "still", "still or video"),
    ("n", "003", "Sequence number within the run"),
)

TOKEN_NAMES: tuple[str, ...] = tuple(name for name, _e, _h in TOKENS)

# Windows reserved device names — a file called CON.png cannot be created.
_RESERVED = {"CON", "PRN", "AUX", "NUL"} | {
    f"{p}{i}" for p in ("COM", "LPT") for i in range(1, 10)}

_ILLEGAL = re.compile(r'[<>:"/\\|?*\x00-\x1f]')
_SEPS = re.compile(r"[_\-\s]{2,}")

MAX_COMPONENT = 64
MAX_STEM = 120


def sanitize_component(text: Any, *, max_len: int = MAX_COMPONENT) -> str:
    """One token value → a filesystem-safe fragment (possibly empty)."""
    s = "" if text is None else str(text)
    s = _ILLEGAL.sub("_", s)
    s = re.sub(r"\s+", "_", s).strip("._ -")
    if s.upper() in _RESERVED:
        s += "_"
    return s[:max_len]


def render_template(template: str, tokens: Mapping[str, Any]
                    ) -> tuple[str, list[str]]:
    """Render a filename stem. Returns ``(stem, unknown_token_names)``.

    An absent or empty token renders empty and its adjacent separator is
    collapsed, so a capture with no channel gives ``2026-08-07_142305_Zyla``
    rather than ``2026-08-07_142305__Zyla``. An unknown token renders empty and
    is REPORTED, so the dialog can tell the operator instead of silently
    dropping what they typed.
    """
    unknown: list[str] = []
    clean = {k: sanitize_component(v) for k, v in (tokens or {}).items()}

    def sub(m: re.Match) -> str:
        name = m.group(1)
        if name not in clean:
            if name not in unknown:
                unknown.append(name)
            return ""
        return clean[name]

    stem = re.sub(r"\{(\w+)\}", sub, str(template or ""))
    stem = _SEPS.sub("_", stem).strip("._ -")
    stem = stem[:MAX_STEM].strip("._ -")
    if not stem:
        stem = "capture_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    return stem, unknown


def open_unique(directory: Path, stem: str, ext: str) -> tuple[Path, BinaryIO]:
    """Create and open a NEW file, never overwriting an existing one.

    Uses ``O_CREAT | O_EXCL`` so there is no exists()-then-write window: two
    captures racing on the same stem cannot both win. The caller must use the
    RETURNED path for everything downstream (sidecar, the ``filename`` inside
    the embedded metadata, the log line) — re-rendering the template would
    reintroduce exactly the mismatch this guards against.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    ext = ext if ext.startswith(".") else "." + ext
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    candidates = [stem] + [f"{stem}_{i:03d}" for i in range(2, 1000)]
    candidates.append(f"{stem}_{uuid.uuid4().hex[:8]}")
    for name in candidates:
        path = directory / f"{name}{ext}"
        try:
            fd = os.open(str(path), flags, 0o644)
        except FileExistsError:
            continue
        return path, os.fdopen(fd, "wb")
    raise OSError(f"could not create a unique file for {stem!r} in {directory}")


def unique_dir(directory: Path, stem: str) -> Path:
    """Collision-safe directory (raw time-lapse writes a folder of frames)."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name in [stem] + [f"{stem}_{i:03d}" for i in range(2, 1000)]:
        path = directory / name
        try:
            path.mkdir()
            return path
        except FileExistsError:
            continue
    path = directory / f"{stem}_{uuid.uuid4().hex[:8]}"
    path.mkdir()
    return path


def merged_settings(stored: Optional[Mapping]) -> dict:
    """``CAPTURE_DEFAULTS`` overlaid with valid stored values.

    ⚠ Booleans get an explicit branch: ``bool("false")`` is ``True``, so the
    ``type(default)(stored)`` shortcut used elsewhere in the repo would turn a
    persisted "off" into "on" the first time a value round-tripped as a string.
    """
    out = dict(CAPTURE_DEFAULTS)
    if not isinstance(stored, Mapping):
        return out
    for key, default in CAPTURE_DEFAULTS.items():
        if key not in stored or stored[key] is None:
            continue
        raw = stored[key]
        try:
            if isinstance(default, bool):
                if isinstance(raw, str):
                    out[key] = raw.strip().lower() in ("1", "true", "yes", "on")
                else:
                    out[key] = bool(raw)
            elif isinstance(default, int):
                out[key] = int(raw)
            elif isinstance(default, float):
                out[key] = float(raw)
            else:
                out[key] = str(raw)
        except (TypeError, ValueError):
            pass
    return out


def repo_root() -> Path:
    """The MEBP checkout root (this file lives in SupportClasses/)."""
    return Path(__file__).resolve().parent.parent


def resolve_output_dir(cfg: Mapping, *, when: Optional[datetime] = None) -> Path:
    """Where this capture is written. Precedence: env → setting → <repo>/captures.

    The directory is NOT created here — stores in this repo create at write
    time, so merely opening the settings dialog never litters the disk.
    """
    raw = os.environ.get(ENV_CAPTURE_DIR) or (cfg or {}).get("output_dir") or ""
    base = Path(raw).expanduser() if str(raw).strip() else repo_root() / "captures"
    if (cfg or {}).get("subfolder_by_date", True):
        base = base / (when or datetime.now()).strftime("%Y-%m-%d")
    return base


def still_extension(cfg: Mapping) -> str:
    return ".tif" if str((cfg or {}).get("still_format", "png")) == "tiff" else ".png"


def video_extension(cfg: Mapping) -> str:
    """v7.15: was inlined in the dialog and the controller, which is how a
    container change had to be remembered in two places."""
    return "." + str((cfg or {}).get("video_container", "mp4") or "mp4")


def validate_still(cfg: Mapping) -> list[str]:
    """Problems that would block or spoil an IMAGE capture.

    Raw 16-bit is refused as PNG rather than silently rewritten: Pillow's
    16-bit PNG byte order is surprising and ImageJ reads 16-bit TIFF natively,
    so a "saved" raw PNG would be a file that looks right and reads wrong.
    """
    out: list[str] = []
    cfg = cfg or {}
    if cfg.get("still_source") == "raw" and cfg.get("still_format") != "tiff":
        out.append("Raw 16-bit stills must be saved as TIFF — 16-bit PNG is "
                   "not reliably readable. Switch the still format to TIFF.")
    return out


def validate_video(cfg: Mapping) -> list[str]:
    """Problems that would block or spoil a RECORDING."""
    out: list[str] = []
    cfg = cfg or {}
    try:
        if float(cfg.get("video_fps", 15)) <= 0:
            out.append("Video frame rate must be greater than zero.")
    except (TypeError, ValueError):
        out.append("Video frame rate is not a number.")
    return out


def validate(cfg: Mapping) -> list[str]:
    """Both sets together — for anything that genuinely spans the two.

    ⚠ v7.15: do NOT use this to gate one kind of capture. It used to gate the
    STILL path, so a video-only problem ("frame rate must be greater than
    zero") refused to take a photograph. Use ``validate_still`` /
    ``validate_video``.
    """
    return validate_still(cfg) + validate_video(cfg)


def estimated_video_mb_per_min(width: int, height: int, fps: float,
                               source: str = "display",
                               quality: int = 80) -> float:
    """Rough size of a minute of recording, for the dialog's live estimate.

    Raw time-lapse is exact arithmetic (uncompressed 16-bit frames); encoded
    video is a bitrate approximation — good enough to tell 40 MB from 7 GB,
    which is the decision the operator is actually making.
    """
    w, h, f = max(1, int(width)), max(1, int(height)), max(0.01, float(fps))
    if source == "raw_timelapse":
        return (w * h * 2 * f * 60) / (1024.0 * 1024.0)
    # ~0.07 bits/pixel/frame at quality 80, scaled linearly with quality.
    bits = w * h * f * 60 * 0.07 * (max(1, int(quality)) / 80.0)
    return bits / 8.0 / (1024.0 * 1024.0)
