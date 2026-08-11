"""
hw_controls_snapshot.py — THE persisted hw_controls key set, in one place.

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
"""

from __future__ import annotations

# Ordered for readable JSON diffs. The andor_* names are historical (the mono
# display scaling shipped first on the Zyla) and are pinned by the existing
# test suites — do not rename.
PERSISTED_HW_CONTROL_KEYS = (
    "auto_exposure",
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
