"""
CaptureMetadata.py — everything worth knowing about one captured frame.

v7.14. The operator asked for the objective ("10x NA 0.3"), the channel, the
date and more in the filename, and — where possible — inside the image itself.
Both come from here: :func:`to_tokens` feeds the filename template and
:func:`to_text_pairs` / :func:`to_json` feed the file's metadata block, so the
name and the embedded record can never disagree.

Pure: no Qt, no SDK imports at module scope. Every source is duck-typed and
every field is individually guarded — a disconnected stage or an absent
microscope yields a record with holes, never an exception in the middle of a
capture. **Absent keys are omitted, never fabricated**: "unknown" is
recoverable, a confident wrong number is not.
"""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any, Mapping, Optional

logger = logging.getLogger(__name__)

SOFTWARE = "MEBP"


@dataclass(frozen=True)
class CaptureMeta:
    """A flat, JSON-safe record of one capture. Every field optional."""

    kind: str = "still"                     # "still" | "video"
    timestamp_iso: str = ""
    software: str = SOFTWARE
    source_mode: str = "display"            # "display" | "raw" | "raw(avg N)"

    # camera
    camera_slot: Optional[int] = None
    camera_name: Optional[str] = None
    camera_identity: Optional[str] = None
    captured_w: Optional[int] = None
    captured_h: Optional[int] = None
    exposure_us: Optional[float] = None
    gain_pct: Optional[float] = None
    bit_depth: Optional[str] = None
    clip_level: Optional[int] = None
    gain_mode: Optional[str] = None
    readout_rate: Optional[str] = None

    # optics
    um_per_px: Optional[float] = None
    objective: Optional[str] = None         # "10x NA 0.3"
    objective_label: Optional[str] = None   # operator's own name for the slot
    magnification: Optional[str] = None
    numerical_aperture: Optional[float] = None
    working_distance_mm: Optional[float] = None
    objective_position: Optional[int] = None
    channel: Optional[str] = None           # filter cube / imaging channel
    filter_position: Optional[int] = None

    # geometry
    stage_x_um: Optional[float] = None
    stage_y_um: Optional[float] = None
    focus_um: Optional[float] = None
    needle_z_mm: Optional[float] = None
    well: Optional[str] = None
    plate: Optional[str] = None

    # display transform actually applied to the saved pixels
    view_rotation_deg: Optional[float] = None
    view_flip_x: Optional[bool] = None
    view_flip_y: Optional[bool] = None

    operator: Optional[str] = None
    notes: Optional[str] = None
    filename: Optional[str] = None
    warnings: tuple = field(default_factory=tuple)

    def as_dict(self, *, drop_empty: bool = True) -> dict:
        d = asdict(self)
        d["warnings"] = list(self.warnings)
        if drop_empty:
            d = {k: v for k, v in d.items()
                 if v is not None and v != "" and v != []}
        return d


def _num(value) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if f == f else None      # drop NaN


def describe_objective(optic) -> tuple[Optional[str], Optional[str],
                                       Optional[float], Optional[float]]:
    """``(display, magnification, NA, working_distance_mm)`` from a MountedOptic.

    Renders the operator's own phrasing — "10x NA 0.3" — and degrades to just
    "10x" when the body reports no NA, rather than the nonsense "10x NA None".
    """
    if optic is None:
        return (None, None, None, None)
    mag = None
    for attr in ("magnification_text", "magnification"):
        v = getattr(optic, attr, None)
        if v:
            mag = str(v)
            break
    if mag and not mag.lower().endswith("x"):
        mag = f"{mag}x"
    na = _num(getattr(optic, "numerical_aperture", None))
    wd = _num(getattr(optic, "working_distance_mm", None))
    parts = [p for p in (mag, (f"NA {na:g}" if na else None)) if p]
    display = " ".join(parts) if parts else (
        str(getattr(optic, "name", "") or "") or None)
    return (display, mag, na, wd)


def collect(*, camera_manager=None, cam_idx=None, captured_wh=None,
            kind: str = "still", source_mode: str = "display",
            orientation: Optional[tuple] = None, controller=None,
            microscope=None, config_store=None, cfg: Optional[Mapping] = None,
            extra: Optional[Mapping] = None) -> CaptureMeta:
    """Assemble a :class:`CaptureMeta` from whatever is reachable right now.

    ``captured_wh`` is the size of the image ACTUALLY saved — a full-resolution
    still has a different µm/px than the live feed, and a stamped-wrong scale
    is worse than none, so the scale is resolved against this width and not
    against whatever the preview happens to be running.
    """
    warn: list[str] = []
    vals: dict[str, Any] = {
        "kind": kind,
        "source_mode": source_mode,
        "timestamp_iso": datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    cw = ch = None
    if captured_wh:
        try:
            cw, ch = int(captured_wh[0]), int(captured_wh[1])
            vals["captured_w"], vals["captured_h"] = cw, ch
        except (TypeError, ValueError, IndexError):
            pass

    if cam_idx is not None:
        vals["camera_slot"] = int(cam_idx) + 1
    if camera_manager is not None and cam_idx is not None:
        try:
            ident = camera_manager.camera_identity(cam_idx)
            if ident:
                vals["camera_identity"] = str(ident[0])
                if len(ident) > 1 and ident[1]:
                    vals["camera_name"] = str(ident[1])
        except Exception:
            pass
        try:
            st = camera_manager.get_hw_settings(cam_idx) or {}
            vals["exposure_us"] = _num(st.get("exposure_us"))
            vals["gain_pct"] = _num(st.get("exposure_gain_pct"))
            if st.get("bit_depth"):
                vals["bit_depth"] = str(st["bit_depth"])
            if st.get("raw_clip_level"):
                vals["clip_level"] = int(st["raw_clip_level"])
            if st.get("andor_gain_mode"):
                vals["gain_mode"] = str(st["andor_gain_mode"])
            if st.get("andor_readout_rate"):
                vals["readout_rate"] = str(st["andor_readout_rate"])
        except Exception:
            pass
        # µm/px AT THE CAPTURED WIDTH — see the docstring.
        try:
            if cw:
                vals["um_per_px"] = _num(
                    camera_manager.effective_um_per_px(cam_idx, cw))
            if vals.get("um_per_px") is None:
                vals["um_per_px"] = _num(camera_manager.get_um_per_px(cam_idx))
        except Exception:
            pass

    if orientation:
        try:
            rot, fx, fy = orientation
            vals["view_rotation_deg"] = _num(rot)
            vals["view_flip_x"] = bool(fx)
            vals["view_flip_y"] = bool(fy)
        except (TypeError, ValueError):
            pass

    # Microscope body: objective + filter cube, live position and the
    # operator's own slot labels.
    if microscope is not None:
        try:
            state = microscope.state()
        except Exception:
            state = None
        if state is not None:
            pos = getattr(state, "objective_position", None)
            if pos:
                vals["objective_position"] = int(pos)
            fpos = getattr(state, "filter_position", None)
            if fpos:
                vals["filter_position"] = int(fpos)
            vals["focus_um"] = _num(getattr(state, "focus_um", None))
            try:
                optics = list(getattr(state, "mounted_objectives", ()) or ())
                if pos and 1 <= int(pos) <= len(optics):
                    disp, mag, na, wd = describe_objective(optics[int(pos) - 1])
                    vals["objective"] = disp
                    vals["magnification"] = mag
                    vals["numerical_aperture"] = na
                    vals["working_distance_mm"] = wd
            except Exception:
                pass
    if config_store is not None:
        try:
            pos = vals.get("objective_position")
            labels = config_store.objective_labels() or {}
            if pos and labels.get(pos):
                vals["objective_label"] = str(labels[pos])
                vals.setdefault("objective", str(labels[pos]))
        except Exception:
            pass
        try:
            fpos = vals.get("filter_position")
            flabels = config_store.filter_labels() or {}
            if fpos and flabels.get(fpos):
                vals["channel"] = str(flabels[fpos])
        except Exception:
            pass

    if controller is not None:
        try:
            xy = controller.get_xy_position(cached=True)
            if xy and xy[0] is not None:
                vals["stage_x_um"] = _num(xy[0])
                vals["stage_y_um"] = _num(xy[1])
        except Exception:
            pass
        try:
            zp = controller.get_zp_position_zero_ref()
            if zp:
                vals["needle_z_mm"] = _num(zp[0])
        except Exception:
            pass

    if cfg:
        for key in ("operator", "notes"):
            v = str(cfg.get(key) or "").strip()
            if v:
                vals[key] = v

    # Caller-supplied context (channel/well/plate from a workflow page) wins:
    # the page knows what it is imaging, the hardware only knows a slot number.
    for key, value in (extra or {}).items():
        if value not in (None, ""):
            vals[key] = value

    vals = {k: v for k, v in vals.items() if v is not None}
    vals["warnings"] = tuple(warn)
    known = set(CaptureMeta.__dataclass_fields__)
    return CaptureMeta(**{k: v for k, v in vals.items() if k in known})


def to_tokens(meta: CaptureMeta, *, n: Optional[int] = None) -> dict:
    """Filename-template tokens for this capture (all strings)."""
    now = datetime.now()
    d = meta.as_dict()
    tok = {
        "date": now.strftime("%Y-%m-%d"),
        "time": now.strftime("%H%M%S"),
        "datetime": now.strftime("%Y%m%d_%H%M%S"),
        "kind": meta.kind or "still",
        "camera": d.get("camera_name") or d.get("camera_identity") or "cam",
        "cam_idx": str(d.get("camera_slot") or ""),
        "objective": d.get("objective") or "",
        "mag": d.get("magnification") or "",
        "channel": d.get("channel") or "",
        "well": d.get("well") or "",
        "plate": d.get("plate") or "",
        "operator": d.get("operator") or "",
        "n": f"{int(n):03d}" if n is not None else "",
    }
    for key, src, fmt in (("exposure_ms", "exposure_us", lambda v: f"{v / 1000.0:.0f}"),
                          ("um_per_px", "um_per_px", lambda v: f"{v:.3f}"),
                          ("x_um", "stage_x_um", lambda v: f"{v:.0f}"),
                          ("y_um", "stage_y_um", lambda v: f"{v:.0f}")):
        v = d.get(src)
        tok[key] = fmt(v) if isinstance(v, (int, float)) else ""
    return tok


def to_text_pairs(meta: CaptureMeta) -> dict:
    """``{"MEBP:field": "value"}`` for PNG text chunks."""
    return {f"MEBP:{k}": ("; ".join(str(x) for x in v)
                          if isinstance(v, (list, tuple)) else str(v))
            for k, v in meta.as_dict().items()}


def to_json(meta: CaptureMeta) -> str:
    return json.dumps(meta.as_dict(), indent=2, sort_keys=True,
                      ensure_ascii=False)


def to_imagej_description(meta: CaptureMeta) -> str:
    """TIFF ImageDescription: an ImageJ header block, then our JSON.

    ImageJ parses ``key=value`` lines from the top and ignores the remainder,
    so the operator gets a correctly-scaled image on open AND the full record
    survives in the same tag.
    """
    lines = ["ImageJ=1.54"]
    if meta.um_per_px:
        lines += ["unit=um"]
    lines += ["", to_json(meta)]
    return "\n".join(lines)
