"""
MosaicCalibration.py — ONE resolved calibration for every mosaic and live view.

v7.5.x. Operator: *"We have multiple surfaces for calibrating mosaics; there
should only be ONE … All mosaic building should follow exactly the same pattern
as the calibrated settings. Rosette, full plate overview, and fluorescence
workflow should all have the exact same behavior."*

Before this module there were **six** mosaic-build pipelines, each resolving its
own inputs with its own precedence, and they disagreed:

* the full-plate scan read the camera store first for orientation, the
  calibration dialog read only the live ``CameraManager`` (which comes up
  un-synced), and the **fluorescence scan applied no calibrated orientation at
  all** — it used a retired coarse ``mosaic_scan.frame_orient`` string, so on a
  camera stored as ``rotation 180° + flip_y`` it rotated but **lost the flip**;
* µm/px had four different precedences, two of which let a *learned* value or a
  hidden ``fov_um`` override shadow the calibration the operator had just
  measured;
* tile overlap resolved to 5 %, 25 % or 50 % depending on the code path.

This module is the single answer. It is **pure**: no Qt, no hardware, no I/O of
its own beyond reading the two calibration stores. Everything a mosaic needs is
resolved once, in one place, with recorded provenance.

The five deviation types the operator enumerated map onto it as:

===  ==================================  =========================================
 #   Deviation                           Field
===  ==================================  =========================================
 1   camera is mirrored                  ``flip_x``
 2   stage ±X/±Y vs ±pixel axes           ``rotation_deg`` + ``flip_x``/``flip_y``
 3   camera rotated about +Z             ``rotation_deg``
 4   pixel size wrong for the resolution ``um_per_px`` (rescaled) + ``calib_resolution``
 5   tile overlap                        ``overlap_frac``
===  ==================================  =========================================

(1)(2)(3) are not three independent knobs — they are one 2×2 camera→stage matrix,
measured together by ``derive_camera_stage_orientation`` and stored as the
canonical ``(rotation_deg, flip_x, flip_y)`` triple that
``MosaicBuilder._orient_tile``, ``CameraFeedView`` and
``CameraManager.pixel_to_stage_offset`` all consume.

Usage::

    cal = resolve(camera_manager=mgr, cam_idx=idx, camera_name=spec_name,
                  objective="4x", live_resolution=(fw, fh),
                  scan_settings=settings.get_section("mosaic_scan"))
    why = refuse_reason(cal)
    if why:
        ...  # tell the operator to calibrate; do not scan
    builder = build_mosaic_builder(cal)

Ownership (deliberate, and the invariant this module enforces):

* ``um_per_px`` + the resolution it was measured at → per camera **and
  objective**, in ``ObjectiveCalibration`` (objectives.json).
* ``rotation_deg`` / ``flip_x`` / ``flip_y`` → per camera **identity** (a mount
  property), in ``CameraCalibrationStore``.
* scan parameters (overlap, target_px, registration, timing) → the **scan**, in
  ``settings.json → mosaic_scan``.
* ``output_rotation_deg`` → per camera identity, **display only** (see below).

``ObjectiveCalibration``'s per-objective ``rotation_deg`` is deliberately NOT in
the orientation precedence. Rotation is a property of how the camera is mounted,
not of which objective is fitted; keeping a second home for it is what let a
stale per-objective value overwrite a freshly measured one. It is still written
by the existing calibration paths so older files stay readable — it is simply
never read here.

⚠ ``output_rotation_deg`` is **display only**. Mosaic tiles are placed at trusted
raw stage positions and every consumer back-projects
``stage = (extent − shift) + px/scale`` to derive well centres for **motion**.
Baking a rotation into the stored composite or its extent would reintroduce the
class of bug that once drove the stage millimetres off target. Viewers apply it
at paint time; the builder never sees it.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

logger = logging.getLogger(__name__)

# ── Scan-parameter defaults ──────────────────────────────────────────────────
# Mirrors gui/dialogs/mosaic_settings_dialog.MOSAIC_SCAN_DEFAULTS for the keys
# this module resolves. Duplicated deliberately: this module must stay GUI-free
# and importable by the backend / tests without pulling in PySide6.
#
# NOTE the retired keys are absent on purpose and must never come back here:
#   frame_orient  — a coarse none/rot180/fliph/flipv per-tile transform that
#                   predates the measured orientation. It double-oriented the
#                   plate scan and (because only the fluorescence worker still
#                   honoured it) was the direct cause of the same camera
#                   producing two different mosaic orientations.
#   fov_um        — a µm/px override that beat every measured value, forever,
#   spacing_um      from a hidden Advanced submenu.
SCAN_DEFAULTS: dict = {
    "overlap_pct": 25,
    "settle_ms": 300,
    "fresh_frames": 3,
    "fresh_timeout_s": 2.5,
    "avg_frames": 1,
    "target_px": 3000,
    "register": True,
    "max_shift_um": 0,
    "reg_method": "fourier_mellin",
    # v7.14 — capture each tile at the camera's FULL sensor resolution,
    # restoring the preview resolution when the scan ends. Binning does not
    # change the field of view, so this visits the SAME tiles; it costs
    # per-tile transfer time, not coverage.
    "full_res_scan": False,
}

# Overlap is clamped to this band. The floor is not cosmetic: raster spacing is
# ``FOV × (1 − overlap)``, so with too little overlap a small µm/px error opens
# visible GAPS between tiles, and registration has too little shared texture to
# lock onto. The operator's machine was running 5 %.
MIN_OVERLAP_FRAC = 0.05
MAX_OVERLAP_FRAC = 0.60
RECOMMENDED_OVERLAP_FRAC = 0.25


@dataclass(frozen=True)
class MosaicCalibration:
    """Everything one mosaic build (or live view) needs, resolved once.

    ``um_per_px`` is already rescaled to ``live_resolution``; use it directly.
    ``base_um_per_px`` / ``calib_resolution`` are kept for display and for
    telling the operator what was measured where.
    """

    # (4) scale
    um_per_px: float = 0.0
    base_um_per_px: float = 0.0
    calib_resolution: Optional[tuple[int, int]] = None
    live_resolution: Optional[tuple[int, int]] = None
    # v7.16: the PRE-crop frame size. ``live_resolution`` is what the camera
    # DELIVERS (and so what the FOV is computed from — a crop really does
    # shrink the field); this is the sensor mode the µm/px rescale is keyed on.
    # None => no crop, i.e. identical to ``live_resolution``.
    capture_resolution: Optional[tuple[int, int]] = None
    # v7.16: raw-pixel displacement of the delivered frame's centre from the
    # sensor's centre, non-zero only for an OFF-CENTRE crop. See
    # ``crop_offset_um`` — the tile placement has to carry it or the mosaic and
    # the click path disagree about where the stage was pointing.
    crop_offset_px: tuple[float, float] = (0.0, 0.0)

    # (1)(2)(3) orientation — the camera→stage 2×2, canonically decomposed
    rotation_deg: float = 0.0
    flip_x: bool = False
    flip_y: bool = False

    # (5) raster + stitch
    overlap_frac: float = RECOMMENDED_OVERLAP_FRAC
    target_px: int = 3000
    register: bool = True
    reg_method: str = "fourier_mellin"
    max_shift_um: float = 0.0

    # capture timing
    settle_ms: int = 300
    fresh_frames: int = 3
    fresh_timeout_s: float = 2.5
    # v7.13 — raw frames averaged per tile (SNR ×√N); 1 = single frame.
    avg_frames: int = 1
    # v7.14 — scan at the camera's full sensor resolution (a SCAN parameter:
    # the switch happens before the calibration is resolved, so by the time
    # this object exists ``live_resolution`` already reflects it; the flag is
    # carried only so the worker and the log can say what was asked for).
    full_res_scan: bool = False

    # display-only whole-mosaic rotation (never baked into the composite)
    output_rotation_deg: float = 0.0

    # per-field provenance, for logging and for telling the operator why
    provenance: Mapping[str, str] = field(default_factory=dict)

    # ── Derived ──────────────────────────────────────────────────────

    @property
    def fov_um(self) -> tuple[float, float]:
        """Camera field of view (w, h) in µm at the live resolution."""
        if not self.live_resolution or self.um_per_px <= 0:
            return (0.0, 0.0)
        w, h = self.live_resolution
        return (float(w) * self.um_per_px, float(h) * self.um_per_px)

    @property
    def spacing_um(self) -> tuple[float, float]:
        """Raster step (x, y) between tile centres — ``FOV × (1 − overlap)``."""
        fw, fh = self.fov_um
        k = 1.0 - self.overlap_frac
        return (fw * k, fh * k)

    @property
    def scale_is_calibrated(self) -> bool:
        """True when µm/px is real AND we know what resolution it came from.

        An unstamped value is NOT calibrated for this purpose: it cannot be
        rescaled, so it is only correct by luck at the live resolution.
        """
        return self.um_per_px > 0 and self.calib_resolution is not None

    @property
    def crop_offset_um(self) -> tuple[float, float]:
        """``crop_offset_px`` mapped into the STAGE frame.

        v7.16. An off-centre crop means the delivered frame's middle shows a
        point that is NOT where the stage is pointing. The displacement is a
        raw-pixel vector, so it goes through the SAME camera→stage chain a click
        does — ``µm/px · R(θ) · diag(mx, my)`` — which is what keeps the mosaic's
        tile placement and ``CameraManager.pixel_to_stage_offset`` in agreement.
        Two conventions consistently applied would each be fine; one of each is
        the bug.

        ``(0, 0)`` for a centred or absent crop.
        """
        dx_px, dy_px = self.crop_offset_px
        if (abs(dx_px) < 1e-9 and abs(dy_px) < 1e-9) or self.um_per_px <= 0:
            return (0.0, 0.0)
        if self.flip_x:
            dx_px = -dx_px
        if self.flip_y:
            dy_px = -dy_px
        dx = dx_px * self.um_per_px
        dy = dy_px * self.um_per_px
        theta = float(self.rotation_deg or 0.0)
        if not theta:
            return (dx, dy)
        t = math.radians(theta)
        c, s = math.cos(t), math.sin(t)
        return (dx * c - dy * s, dx * s + dy * c)

    @property
    def is_cropped(self) -> bool:
        """True when the delivered frame is a crop of the captured one."""
        if not (self.capture_resolution and self.live_resolution):
            return False
        return (abs(self.capture_resolution[0] - self.live_resolution[0]) > 1
                or abs(self.capture_resolution[1] - self.live_resolution[1]) > 1)

    @property
    def was_rescaled(self) -> bool:
        # v7.16: compared against the CAPTURE width — a crop changes the
        # delivered width without any rescale having happened, and reporting
        # that as "rescaled" would point the operator at the wrong thing.
        ref = self.capture_resolution or self.live_resolution
        if not (self.calib_resolution and ref):
            return False
        return abs(self.calib_resolution[0] - ref[0]) > 1

    def describe(self) -> str:
        """One-line human summary — used in logs and in the calibration UI."""
        cr = (f"{self.calib_resolution[0]}×{self.calib_resolution[1]}"
              if self.calib_resolution else "unknown resolution")
        lr = (f"{self.live_resolution[0]}×{self.live_resolution[1]}"
              if self.live_resolution else "?")
        flips = ", ".join(
            [n for n, on in (("flip X", self.flip_x), ("flip Y", self.flip_y))
             if on]) or "none"
        fw, fh = self.fov_um
        crop = ""
        if self.is_cropped and self.capture_resolution:
            crop = (f" | cropped from "
                    f"{self.capture_resolution[0]}x{self.capture_resolution[1]}")
        return (f"{self.um_per_px:.4f} um/px at {lr} "
                f"(measured {self.base_um_per_px:.4f} @ {cr}) | "
                f"FOV {fw:.0f}x{fh:.0f} um | rotation {self.rotation_deg:.1f} deg "
                f"| flips {flips} | overlap {self.overlap_frac * 100:.0f}%{crop}")


# ── Resolution ───────────────────────────────────────────────────────────────

def _as_res(value: Any) -> Optional[tuple[int, int]]:
    """Coerce a (w, h)-ish value to a positive int pair, else None."""
    try:
        if value is None:
            return None
        w, h = int(value[0]), int(value[1])
        return (w, h) if w > 0 and h > 0 else None
    except (TypeError, ValueError, IndexError):
        return None


def _scan_params(scan_settings: Optional[Mapping]) -> dict:
    """``SCAN_DEFAULTS`` overlaid with any valid stored values."""
    out = dict(SCAN_DEFAULTS)
    if isinstance(scan_settings, Mapping):
        for key, default in SCAN_DEFAULTS.items():
            if key in scan_settings and scan_settings[key] is not None:
                try:
                    out[key] = type(default)(scan_settings[key])
                except (TypeError, ValueError):
                    pass
    return out


def objective_camera_key(camera_manager=None, cam_idx=None,
                         camera_spec_name=None,
                         identity=None) -> Optional[str]:
    """THE key ``ObjectiveCalibrationStore`` is addressed by, for every caller.

    v7.16 — this used to be ``camera_config.camera_spec.name`` at each call
    site: a CONFIGURED catalogue label, not a property of the camera that is
    actually plugged in. Two physical cameras sharing one spec name therefore
    shared one per-objective calibration block, and the damage ran both ways:

    * calibrating a newly fitted camera **overwrote** the previous camera's
      values for that objective, and
    * the mosaic then **read them back** and resolution-rescaled them, which is
      only ever valid WITHIN one sensor — across two cameras it produces a
      physically meaningless µm/px.

    Measured on this rig: a Tucsen was calibrated while the spec still read
    ``"Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)"``, so the ToupTek's 4x
    calibration was destroyed, and a mosaic resolved
    ``1.2710 um/px at 2600×2048 (measured 3.2271 @ 1024×1024)`` — the *Andor's*
    number stretched onto the Tucsen's sensor.

    The device identity is used whenever one can be read. There is deliberately
    **no fallback to the spec name once an identity exists**: falling back is
    precisely how one camera reads another's calibration, and an absent
    calibration is recoverable (re-run a 30-second measurement) where a wrong
    one is not. The spec name is used only when there is no identity at all
    (simulated cameras, tests, no manager), which preserves legacy behaviour
    exactly for those.
    """
    if identity is None and camera_manager is not None and cam_idx is not None:
        try:
            got = camera_manager.camera_identity(cam_idx)
            if got:
                identity = got[0] if isinstance(got, (tuple, list)) else got
        except Exception:
            identity = None
    if identity:
        return str(identity)
    name = str(camera_spec_name or "").strip()
    return name or None


def capture_width_px(camera_manager=None, cam_idx=None,
                     fallback: float = 0.0) -> float:
    """Frame width in CAPTURE pixels, falling back to a delivered width.

    v7.16 — THE one place the crop is divided out of a µm/px rescale.

    ``µm/px ∝ 1/width`` describes the sensor being sampled differently (binning,
    a resolution switch): the same optical field over more pixels. A centred
    CROP changes the pixel count without changing what a pixel spans, so a
    cropped width in that ratio rescales µm/px by the crop fraction — 27 % on
    this rig's Tucsen at a square crop, and in the direction that makes a mosaic
    believe its field is wider than it is and step past its own edges.

    Every rescale site resolves its denominator through here rather than reading
    ``frame.shape``, so there is one answer to "how wide is this sensor mode"
    instead of one per call site. With no crop configured, the capture width IS
    the delivered width and every caller is unchanged.
    """
    if camera_manager is not None and cam_idx is not None:
        try:
            getter = getattr(camera_manager, "capture_width", None)
            if callable(getter):
                got = float(getter(cam_idx, fallback) or 0.0)
                if got > 0:
                    return got
        except Exception as exc:
            logger.debug(f"capture_width_px: manager lookup failed: {exc}")
    try:
        return float(fallback or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _resolve_scale(camera_manager, cam_idx, camera_name, objective,
                   live_res, obj_store, prov, identity=None
                   ) -> tuple[float, float, Optional[tuple[int, int]]]:
    """``(effective, base, calib_resolution)`` µm/px. See ``resolve``.

    Precedence — measurement first, and NOTHING may shadow it:
      1. ``ObjectiveCalibration`` for (camera key, objective). This is the one
         source that has always recorded the resolution it was measured at.
      2. The live ``CameraManager`` value (with its resolution, when it has one).
      3. Nothing — 0.0, which ``refuse_reason`` turns into a refusal.

    v7.16: the camera key is :func:`objective_camera_key` — the DEVICE
    IDENTITY, not the configured spec name. See that function for why.
    """
    base = 0.0
    cal_res = None

    cam_key = objective_camera_key(
        camera_manager, cam_idx, camera_name, identity)
    if cam_key and objective:
        try:
            if obj_store is None:
                from SupportClasses.ObjectiveCalibration import get_store
                obj_store = get_store()
            cal = obj_store.get_calibration(str(cam_key), str(objective))
            if cal:
                base = float(cal.get("measured_um_per_px") or 0.0)
                cal_res = _as_res(cal.get("resolution"))
                if base > 0:
                    prov["um_per_px"] = (
                        f"objective calibration ({cam_key} / {objective})")
        except Exception as exc:
            logger.debug(f"MosaicCalibration: objective lookup failed: {exc}")

    if base <= 0 and camera_manager is not None and cam_idx is not None:
        try:
            base = float(camera_manager.get_um_per_px(cam_idx) or 0.0)
            getter = getattr(camera_manager, "get_um_per_px_resolution", None)
            cal_res = _as_res(getter(cam_idx)) if callable(getter) else None
            calibrated = True
            checker = getattr(camera_manager, "is_um_per_px_calibrated", None)
            if callable(checker):
                calibrated = bool(checker(cam_idx))
            if base > 0 and calibrated:
                prov["um_per_px"] = "live camera manager"
            else:
                # An uncalibrated slot still carries a seed default; treating it
                # as a measurement is how a mosaic silently scans at the wrong
                # scale, so drop it and let refuse_reason speak.
                base = 0.0
                cal_res = None
        except Exception as exc:
            logger.debug(f"MosaicCalibration: manager lookup failed: {exc}")

    if base <= 0:
        prov["um_per_px"] = "UNCALIBRATED"
        return 0.0, 0.0, None

    if cal_res is None:
        # Known value, unknown measurement resolution: use it as-is but SAY so.
        # Silently substituting the live width is what made a restored camera
        # scale its mosaic wrong with no visible symptom.
        prov["um_per_px_resolution"] = "UNSTAMPED (cannot rescale)"
        return base, base, None

    prov["um_per_px_resolution"] = f"{cal_res[0]}×{cal_res[1]}"
    eff = base
    # v7.16: rescale against the CAPTURE width, not the delivered one. A centred
    # crop removes pixels without changing what a pixel spans, so dividing by a
    # cropped width would rescale µm/px by the crop fraction — in the direction
    # that makes the mosaic believe its field is wider than it is, which opens
    # seams. With no crop configured this IS the live width.
    denom = capture_width_px(camera_manager, cam_idx,
                             live_res[0] if live_res else 0.0)
    if denom > 0:
        # µm/px ∝ 1/frame_width: the same optical FOV over more pixels means
        # each pixel spans fewer µm.
        eff = base * float(cal_res[0]) / denom
        if abs(cal_res[0] - denom) > 1:
            logger.info(
                f"MosaicCalibration: um/px rescaled {base:.4f}@{cal_res[0]}px "
                f"-> {eff:.4f}@{denom:.0f}px")
    return eff, base, cal_res


def resolve_camera_orientation(camera_manager=None, cam_idx=None,
                               identity: Optional[str] = None,
                               cal_store=None,
                               provenance: Optional[dict] = None
                               ) -> tuple[float, bool, bool]:
    """THE single answer to "how is this camera oriented vs the stage?".

    Returns the canonical ``(rotation_deg, flip_x, flip_y)`` that
    ``MosaicBuilder._orient_tile``, ``CameraFeedView.set_view_orientation`` and
    ``CameraManager.pixel_to_stage_offset`` all consume as ``R(θ)·diag(mx, my)``.

    Precedence per field: the persisted ``CameraCalibrationStore`` entry (ground
    truth — it survives navigation and restarts) then the live
    ``CameraManager``, then neutral. Per-field rather than all-or-nothing so a
    camera with a stored rotation but a live-only flip still gets both.

    v7.10: this used to exist THREE times — here, hand-rolled again in
    ``CalibrationPage._ploc_microscope_frame_orientation``, and implicitly a
    third time in ``CameraFeedView`` (manager-only, no store). Three copies of a
    precedence rule is three chances for the mosaic, the plate-location feed and
    the live view to disagree about the same camera, which is exactly what the
    operator saw. There is now one implementation and the callers delegate; a
    test pins that they are the same function object.

    ``provenance`` (optional dict) is filled in with per-field sources.
    """
    prov = provenance if provenance is not None else {}
    rot = flip_x = flip_y = None

    if identity:
        try:
            if cal_store is None:
                from SupportClasses.CameraCalibrationStore import get_store
                cal_store = get_store()
            entry = cal_store.get_calibration(identity)
            if entry:
                if entry.get("rotation_deg") is not None:
                    rot = float(entry["rotation_deg"])
                    prov["rotation_deg"] = "camera store"
                if "mirrored" in entry:
                    flip_x = bool(entry["mirrored"])
                    prov["flip_x"] = "camera store"
                if "flip_y" in entry:
                    flip_y = bool(entry["flip_y"])
                    prov["flip_y"] = "camera store"
        except Exception as exc:
            logger.debug(f"MosaicCalibration: camera store read failed: {exc}")

    if camera_manager is not None and cam_idx is not None:
        try:
            if rot is None:
                live = camera_manager.get_rotation_deg(cam_idx)
                if live is not None:
                    rot = float(live)
                    prov["rotation_deg"] = "live camera manager"
            if flip_x is None:
                flip_x = bool(camera_manager.get_mirrored(cam_idx))
                prov["flip_x"] = "live camera manager"
            if flip_y is None:
                gfy = getattr(camera_manager, "get_flip_y", None)
                if callable(gfy):
                    flip_y = bool(gfy(cam_idx))
                    prov["flip_y"] = "live camera manager"
        except Exception as exc:
            logger.debug(f"MosaicCalibration: manager orientation read: {exc}")

    prov.setdefault("rotation_deg", "unmeasured (0°)")
    prov.setdefault("flip_x", "unmeasured (no flip)")
    prov.setdefault("flip_y", "unmeasured (no flip)")
    return (float(rot or 0.0), bool(flip_x), bool(flip_y))


def _resolve_orientation(camera_manager, cam_idx, identity, cal_store,
                         prov) -> tuple[float, bool, bool, float]:
    """``(rotation_deg, flip_x, flip_y, output_rotation_deg)``.

    Thin wrapper over :func:`resolve_camera_orientation` that additionally
    reads the display-only whole-mosaic output rotation (which is a mosaic
    concern, not a camera-orientation one, so it does not belong in the
    shared resolver).
    """
    rot, flip_x, flip_y = resolve_camera_orientation(
        camera_manager, cam_idx, identity, cal_store, prov)

    out_rot = 0.0
    if identity:
        try:
            if cal_store is None:
                from SupportClasses.CameraCalibrationStore import get_store
                cal_store = get_store()
            getter = getattr(cal_store, "get_mosaic_output_rotation", None)
            if callable(getter):
                out_rot = float(getter(identity) or 0.0)
        except Exception as exc:
            logger.debug(f"MosaicCalibration: output rotation read: {exc}")

    return (rot, flip_x, flip_y, out_rot)


def resolve(*, camera_manager=None, cam_idx: Optional[int] = None,
            camera_name: Optional[str] = None,
            objective: Optional[str] = None,
            identity: Optional[str] = None,
            live_resolution: Any = None,
            scan_settings: Optional[Mapping] = None,
            cal_store=None, obj_store=None) -> MosaicCalibration:
    """Resolve the ONE calibration every mosaic pipeline and live view uses.

    ``identity`` is the camera's stable device-identity key; when omitted it is
    read from ``camera_manager.camera_identity(cam_idx)``. ``live_resolution``
    is the ACTUAL captured frame size — pass ``frame.shape[1], frame.shape[0]``,
    not a configured value, since that is what the rescale must target.

    Every store read is individually guarded: a missing or broken store degrades
    to "unmeasured", which ``refuse_reason`` reports, rather than raising in the
    middle of a scan.
    """
    prov: dict[str, str] = {}

    if identity is None and camera_manager is not None and cam_idx is not None:
        try:
            got = camera_manager.camera_identity(cam_idx)
            identity = got[0] if got else None
        except Exception:
            identity = None

    live_res = _as_res(live_resolution)
    if live_res is None:
        prov["live_resolution"] = "unknown"

    # v7.16: the PRE-crop sensor mode. Kept beside the delivered size because
    # they answer different questions — the FOV comes from what is delivered,
    # the µm/px rescale from what was captured.
    cap_res = None
    crop_off = (0.0, 0.0)
    if camera_manager is not None and cam_idx is not None:
        try:
            getter = getattr(camera_manager, "capture_resolution", None)
            cap_res = _as_res(getter(cam_idx)) if callable(getter) else None
        except Exception:
            cap_res = None
        try:
            coff = getattr(camera_manager, "crop_center_offset_px", None)
            if callable(coff):
                got = coff(cam_idx)
                crop_off = (float(got[0]), float(got[1]))
        except Exception:
            crop_off = (0.0, 0.0)
    if cap_res and live_res and (abs(cap_res[0] - live_res[0]) > 1
                                 or abs(cap_res[1] - live_res[1]) > 1):
        prov["crop"] = (f"cropped to {live_res[0]}×{live_res[1]} of "
                        f"{cap_res[0]}×{cap_res[1]}")

    eff, base, cal_res = _resolve_scale(
        camera_manager, cam_idx, camera_name, objective, live_res, obj_store,
        prov, identity)
    rot, flip_x, flip_y, out_rot = _resolve_orientation(
        camera_manager, cam_idx, identity, cal_store, prov)

    p = _scan_params(scan_settings)
    prov["scan_params"] = (
        "mosaic_scan settings" if scan_settings else "defaults")
    overlap = max(MIN_OVERLAP_FRAC,
                  min(MAX_OVERLAP_FRAC, float(p["overlap_pct"]) / 100.0))

    cal = MosaicCalibration(
        um_per_px=eff,
        base_um_per_px=base,
        calib_resolution=cal_res,
        live_resolution=live_res,
        capture_resolution=cap_res,
        crop_offset_px=crop_off,
        rotation_deg=rot,
        flip_x=flip_x,
        flip_y=flip_y,
        overlap_frac=overlap,
        target_px=int(p["target_px"]),
        register=bool(p["register"]),
        reg_method=str(p["reg_method"]),
        max_shift_um=float(p["max_shift_um"]),
        settle_ms=max(0, int(p["settle_ms"])),
        fresh_frames=max(1, int(p["fresh_frames"])),
        fresh_timeout_s=float(p["fresh_timeout_s"]),
        avg_frames=max(1, int(p.get("avg_frames", 1))),
        full_res_scan=bool(p.get("full_res_scan", False)),
        output_rotation_deg=out_rot,
        provenance=prov,
    )
    logger.info(f"Mosaic calibration resolved: {cal.describe()}")
    return cal


# ── Consumption ──────────────────────────────────────────────────────────────

def refuse_reason(cal: MosaicCalibration) -> Optional[str]:
    """Why this camera must not build a mosaic yet, or None if it may.

    One place decides, so the full-plate scan, the rosette scan and the
    fluorescence scan cannot disagree about whether a camera is ready — and the
    message names the missing piece instead of failing with a wrong-looking
    mosaic.
    """
    if cal.um_per_px <= 0:
        return ("The microscope camera has no um/px calibration. Run Hardware "
                "Setup > Cameras > Mosaic & Camera Calibration first.")
    if cal.live_resolution is None:
        return ("The camera is not delivering frames, so its capture resolution "
                "is unknown. Start the camera and try again.")
    if cal.calib_resolution is None:
        return ("The um/px calibration has no recorded measurement resolution, "
                "so it cannot be corrected for the current capture resolution. "
                "Re-run Hardware Setup > Cameras > Mosaic & Camera Calibration.")
    return None


def warnings_for(cal: MosaicCalibration) -> list[str]:
    """Non-blocking advisories about a resolved calibration.

    Kept separate from ``refuse_reason`` so a usable-but-suspect setup still
    scans while the operator is told what looks wrong.
    """
    out: list[str] = []
    # NOTE: these strings are shown to the operator AND written to the log, which
    # on Windows can be a cp1252 console — keep them free of characters that
    # codec cannot encode (e.g. U+2212 MINUS SIGN), or emitting a warning raises.
    if cal.overlap_frac < 0.15:
        out.append(
            f"Tile overlap is only {cal.overlap_frac * 100:.0f}%. Raster "
            f"spacing is FOV x (1 - overlap), so a small um/px error will open "
            f"gaps between tiles and registration has little shared texture to "
            f"lock onto. {RECOMMENDED_OVERLAP_FRAC * 100:.0f}% is recommended.")
    if cal.settle_ms < 50:
        out.append(
            f"Settle after move is {cal.settle_ms} ms — tiles may be captured "
            f"before the stage has stopped, which blurs them and offsets the "
            f"stitch. {SCAN_DEFAULTS['settle_ms']} ms is the default.")
    if cal.was_rescaled and cal.calib_resolution:
        out.append(
            f"um/px was measured at {cal.calib_resolution[0]} px wide and the "
            f"camera is running at {cal.live_resolution[0]} px, so it has been "
            f"rescaled to {cal.um_per_px:.4f}. Re-calibrate at the working "
            f"resolution if the mosaic still does not line up.")
    if cal.rotation_deg == 0.0 and not cal.flip_x and not cal.flip_y:
        prov = cal.provenance.get("rotation_deg", "")
        if "unmeasured" in prov:
            out.append(
                "The camera-to-stage orientation has never been measured, so tiles "
                "are placed unrotated. If the mosaic comes out mirrored or on "
                "the wrong side, run the camera calibration.")
    return out


def build_mosaic_builder(cal: MosaicCalibration, *, retain_frames: bool = False,
                         retain_for_reorient: bool = False,
                         initial_shift_um: tuple[float, float] = (0.0, 0.0),
                         **overrides):
    """Construct a ``MosaicBuilder`` from a resolved calibration.

    THE single place builder arguments are chosen. Every pipeline goes through
    here, which is what makes the full-plate, rosette, calibration-preview and
    fluorescence mosaics identical by construction — previously they disagreed on
    orientation, ``target_mosaic_px`` (3000 / 1800 / 2500 / 2000),
    ``registration_method`` and whether ``retain_for_reorient`` was set at all
    (and without it the caller's ``optimize_registration`` silently did nothing).

    ``retain_for_reorient=True`` is required if the caller intends to run
    ``optimize_registration`` or re-orient without re-scanning. ``overrides``
    are passed through for the rare genuinely per-pipeline argument.

    Note ``output_rotation_deg`` is deliberately NOT passed: it is display-only
    and must never reach the stored composite or its extent.
    """
    from SupportClasses.MosaicBuilder import MosaicBuilder

    frame_wh = cal.live_resolution or (0, 0)
    kwargs = dict(
        frame_size_px=(int(frame_wh[0]), int(frame_wh[1])),
        micron_per_pixel=float(cal.um_per_px),
        overlap=float(cal.overlap_frac),
        target_mosaic_px=int(cal.target_px),
        register=bool(cal.register),
        max_shift_um=float(cal.max_shift_um),
        initial_shift_um=initial_shift_um,
        retain_frames=bool(retain_frames),
        frame_rotation_deg=float(cal.rotation_deg),
        frame_mirrored=bool(cal.flip_x),
        frame_flip_y=bool(cal.flip_y),
        retain_for_reorient=bool(retain_for_reorient),
        registration_method=str(cal.reg_method),
        tile_center_offset_um=cal.crop_offset_um,
    )
    kwargs.update(overrides)
    return MosaicBuilder(**kwargs)
