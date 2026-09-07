"""
hw_controls_snapshot.py — THE persisted hw_controls key set AND the order they
are applied in, in one place.

v7.13: Two independent sites persist a camera's hardware controls to
``CameraCalibrationStore.set_hw_controls`` — the camera settings dialog's
``_persist()`` (fires on every change) and Hardware Setup's bulk
"Save Camera Settings". They each hand-listed the keys, and the lists had
already drifted: the bulk save silently dropped the three ``andor_*``
display-scaling keys (documented gap since v7.9). Any key added to only one
site gets silently un-persisted by the other.

This helper is the single source of truth for WHICH keys are persisted; both
write sites build their payload through it, so the gap class cannot recur.
Values are taken from a ``get_hw_settings()`` readback dict — persisted values
are always device readbacks, never widget state. ``None`` values are kept
here; the store drops them on write (its documented "unknown is never
persisted" rule).

v7.19 adds the other half of the same argument. ``apply_hw_controls`` used to
live only on ``HardwareSetupPage``, but the order it encodes is load-bearing
(see that function's comments) and the fluorescence workflow now needs the
identical order to apply its capture preset and to restore the camera on exit.
A second hand-written applier would drift from this one exactly as the two key
lists did, so the applier lives here beside the key list and Hardware Setup
delegates to it.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

# Ordered for readable JSON diffs. The andor_* names are historical (the mono
# display scaling shipped first on the Zyla) and are pinned by the existing
# test suites — do not rename.
PERSISTED_HW_CONTROL_KEYS = (
    "auto_exposure",
    # v7.19 — the SENSOR's own auto black/white points (Tucsen TUIDC_ATLEVELS).
    # ⚠ Not the same channel as the andor_auto_scale trio below: that is the
    # software mono16→8-bit DISPLAY mapping and changes no captured pixel, this
    # moves the sensor's levels and changes the raw data.
    "auto_levels",
    "exposure_us",
    "exposure_gain_pct",
    "gamma",
    "brightness",
    "contrast",
    # Mono→8-bit display scaling (Zyla + Tucsen).
    "andor_auto_scale",
    "andor_scale_lo",
    "andor_scale_hi",
    # v7.13 sensor-quality features (Zyla).
    "andor_sensor_cooling",
    "andor_readout_rate",
    "andor_gain_mode",
    "andor_noise_filter",
    "andor_blemish_correction",
)


def hw_controls_snapshot(st: dict) -> dict:
    """Build the persistable hw_controls payload from a settings readback.

    ``st`` is a ``get_hw_settings()`` dict. Returns exactly the persisted
    keys (plus ``resolution``, normalised to a list). Keys absent from the
    readback come through as None and are dropped by the store on write.
    """
    st = st or {}
    controls = {key: st.get(key) for key in PERSISTED_HW_CONTROL_KEYS}
    res = st.get("resolution")
    controls["resolution"] = list(res) if res else None
    return controls


def apply_hw_controls(mgr, cam_idx: int, hw: dict, *,
                      skip_resolution: bool = False) -> None:
    """Push a hardware-control set onto a running camera, IN ORDER.

    The order is load-bearing and each step says why. Lifted verbatim from
    ``HardwareSetupPage._apply_hw_controls`` (which now delegates here) so the
    restore-on-camera-start path and the fluorescence workflow's preset /
    restore cannot apply the same keys in two different orders.

    ``skip_resolution`` is the caller's decision, not this module's: the
    MICROSCOPE slot has ONE source of truth for resolution
    (``camera_config.active_resolution``, applied on start), so a per-identity
    ``hw_controls`` resolution must not compete for it. Every other camera keeps
    its own.
    """
    if mgr is None or not isinstance(hw, dict):
        return
    auto = hw.get("auto_exposure")
    # Only drive auto-exposure when we have a CLEAN boolean. OpenCV/DShow
    # cameras often report a raw CAP_PROP value (e.g. -1.0) or nothing for
    # auto-exposure; coercing that via bool() would wrongly force auto ON,
    # so ignore non-bool values rather than guess.
    if isinstance(auto, bool):
        mgr.set_hw_auto_exposure(cam_idx, auto)
    # v7.19 — the sensor's own auto levels, same clean-boolean rule and applied
    # beside auto-exposure because it belongs to the same "stop the camera
    # deciding for itself" group. Advertised only where the camera implements
    # it, so this is a silent no-op elsewhere.
    alevels = hw.get("auto_levels")
    if isinstance(alevels, bool) and hasattr(mgr, "set_hw_auto_levels"):
        mgr.set_hw_auto_levels(cam_idx, alevels)
    res = hw.get("resolution")
    if res and not skip_resolution:
        try:
            mgr.set_capture_resolution(cam_idx, int(res[0]), int(res[1]))
        except Exception as exc:
            logger.debug(f"restore resolution failed: {exc}")
    # v7.13 — Andor sensor-quality features, BEFORE the exposure restore
    # (v7.13.x: the achievable exposure range depends on the readout rate
    # and gain mode, so the saved exposure must be applied against the
    # constraint set it was saved UNDER, not the open-time defaults) and
    # BEFORE the display-scaling block (the manual black/white levels are
    # raw counts whose meaning depends on the bit depth the gain mode
    # selects, so the stored levels must be the LAST thing applied).
    # Within this block: gain mode first (it constrains bit depth and the
    # legal readout rates), then readout rate, then the booleans.
    if hasattr(mgr, "set_hw_andor_feature"):
        if isinstance(hw.get("andor_gain_mode"), str):
            mgr.set_hw_andor_feature(cam_idx, "andor_gain_mode",
                                     hw["andor_gain_mode"])
        if isinstance(hw.get("andor_readout_rate"), str):
            mgr.set_hw_andor_feature(cam_idx, "andor_readout_rate",
                                     hw["andor_readout_rate"])
        for key in ("andor_sensor_cooling", "andor_noise_filter",
                    "andor_blemish_correction"):
            if isinstance(hw.get(key), bool):
                mgr.set_hw_andor_feature(cam_idx, key, hw[key])
    # Restore manual exposure/gain UNLESS auto-exposure is explicitly on.
    # (When auto is unknown/None — e.g. an OpenCV cam — we still restore the
    # saved exposure so "reload exactly" holds; there's no auto state to
    # clobber.)
    if auto is not True:
        if hw.get("exposure_us") is not None:
            mgr.set_hw_exposure_us(cam_idx, hw["exposure_us"])
        if hw.get("exposure_gain_pct") is not None:
            mgr.set_hw_exposure_gain(cam_idx, hw["exposure_gain_pct"])
    if hw.get("gamma") is not None:
        mgr.set_hw_gamma(cam_idx, hw["gamma"])
    if hw.get("brightness") is not None:
        mgr.set_hw_brightness(cam_idx, hw["brightness"])
    if hw.get("contrast") is not None:
        mgr.set_hw_contrast(cam_idx, hw["contrast"])
    # Andor (Zyla) display scaling. Auto flag FIRST — turning auto off
    # seeds the levels from the last auto frame, so the stored manual
    # levels must be applied after it to win.
    ascale = hw.get("andor_auto_scale")
    if isinstance(ascale, bool) and hasattr(mgr, "set_hw_andor_auto_scale"):
        mgr.set_hw_andor_auto_scale(cam_idx, ascale)
    if (hw.get("andor_scale_lo") is not None
            and hasattr(mgr, "set_hw_andor_scale_lo")):
        mgr.set_hw_andor_scale_lo(cam_idx, hw["andor_scale_lo"])
    if (hw.get("andor_scale_hi") is not None
            and hasattr(mgr, "set_hw_andor_scale_hi")):
        mgr.set_hw_andor_scale_hi(cam_idx, hw["andor_scale_hi"])


#: What "neutral" means for the three software display corrections. Gamma's
#: neutral is 1.0 conceptually, but each backend states it in its own units
#: (the Tucsen's is an integer 1..255 centred on 100), so the real neutral is
#: taken from the camera's own declared DEFAULT and these are only the fallback
#: when a range carries no default.
_NEUTRAL_FALLBACK = {"gamma": 1.0, "contrast": 1.0, "brightness": 0.0}


def fluorescence_preset(caps: dict) -> dict:
    """The camera state a quantitative fluorescence mosaic needs.

    v7.19, operator: *"the camera needs to be set out of autoexposure mode and
    auto white black balance mode when doing these mosaics"*.

    Returns only keys THIS camera advertises, so a webcam slot gets an empty
    (no-op) preset instead of a set of fabricated writes. Feed the result to
    ``apply_hw_controls``.

    What each entry is actually for — the distinction matters and is easy to
    get wrong:

    * ``auto_exposure`` / ``auto_levels`` are the camera deciding for itself.
      A per-channel exposure is meaningless while either is on, and
      ``auto_levels`` moves the SENSOR's black/white points, so it changes the
      raw data a mosaic captures.
    * ``andor_auto_scale`` and gamma/contrast/brightness are DISPLAY-path
      corrections. On a raw-capable camera the mosaic's tiles come from
      ``capture_raw_average`` with levels frozen at the probe, so these three
      change no captured pixel. They are here because a per-frame-autoscaled,
      gamma-warped preview makes judging an exposure impossible — and because
      the single-frame fallback path (a camera with no raw support) DOES go
      through them.

    Turning ``andor_auto_scale`` off is not a bare disable: the backend seeds
    fixed black/white points from the last auto-scaled frame, so the preview
    freezes at its current appearance rather than jumping.
    """
    ctrls = (caps or {}).get("controls") or {}
    preset: dict = {}
    if "auto_exposure" in ctrls:
        preset["auto_exposure"] = False
    if "auto_levels" in ctrls:
        preset["auto_levels"] = False
    if "andor_auto_scale" in ctrls:
        preset["andor_auto_scale"] = False
    for key, fallback in _NEUTRAL_FALLBACK.items():
        spec = ctrls.get(key)
        if spec is None:
            continue
        rng = spec.get("range") if isinstance(spec, dict) else None
        # A range is (min, max, default) — prefer the camera's OWN declared
        # default over our guess at what "neutral" means in its units.
        if isinstance(rng, (tuple, list)) and len(rng) >= 3 and rng[2] is not None:
            preset[key] = rng[2]
        else:
            preset[key] = fallback
    return preset
