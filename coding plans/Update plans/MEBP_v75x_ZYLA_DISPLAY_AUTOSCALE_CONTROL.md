# MEBP v7.5.x — Zyla Display Auto-Scale Option + Manual Black/White Levels

## Objective

Operator (2026-07-29): *"the microscope camera is adjusting its gain, image,
etc according to the current frame. I want an option in the camera settings
for auto exposure gain or whatever adjustment for the zyla camera. I should
also be able to control these options."*

**Root cause:** the Andor Zyla (mono-16 sCMOS) has **no ISP auto-gain /
auto-exposure** — what looks like the camera adjusting itself is the
`AndorBackend` display conversion: `_mono_to_bgr8` normalized **every frame to
its own 1–99 percentile** (`gui/widgets/andor_backend.py`), so the displayed
brightness chases the scene content frame by frame. The camera hardware never
changed anything.

**Change:** make that display scaling a controllable option, surfaced in the
existing per-camera settings dialog (gear button on the feed):

- **"Auto display scaling (per-frame)"** checkbox — ON = the historical
  per-frame percentile normalize (default, good for finding dim fluorescence);
  OFF = a **fixed black/white level mapping** so the image stops "auto-adjusting".
- **"Black level" / "White level"** sliders (raw sensor counts 0..65535),
  active in manual mode. Turning auto OFF seeds them from the levels the last
  auto frame used, so the image **freezes at its current appearance** instead
  of jumping.
- True hardware **exposure (ms)** remains independently controllable (it
  already was; it is the only real hardware control the Zyla exposes here).
- Persisted per device identity (`CameraCalibrationStore.hw_controls`) and
  restored when the camera starts, like every other hw control.

Deliberately NOT labeled "auto-exposure" in the UI — that would claim a camera
behaviour the Zyla doesn't have; the readout/log name it
`display scale = auto (per-frame) | manual  levels lo..hi`.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/andor_backend.py` | Display-scale state (`_auto_scale`, `_scale_lo/_hi`, `_last_auto_levels` under `_lock`); reader loop picks auto vs fixed levels and records the auto levels; `_auto_levels()` extracted; `_mono_to_bgr8(frame, levels=None)`; new `get/set_display_auto_scale`, `get_display_levels`, `put_display_black/white` (clamped, `hi > lo` invariant), `get_display_level_range`; `get_settings()` carries `andor_auto_scale/_scale_lo/_scale_hi` |
| `gui/widgets/camera_widget.py` | andor capabilities advertise `andor_auto_scale`/`andor_scale_lo`/`andor_scale_hi`; new `set_hw_andor_auto_scale/_scale_lo/_scale_hi` setters (False on other backends); `log_hw_settings` prints the display-scale line for andor |
| `gui/widgets/camera_manager.py` | Three per-slot delegates (hasattr-guarded) |
| `gui/dialogs/camera_settings_dialog.py` | Checkbox + two level sliders (visible only when the backend advertises them; sliders disabled while auto is on); live-apply handlers (+ reload after the auto toggle so the frozen levels appear); `_persist` includes the three keys; Defaults restores auto ON; readout line |
| `gui/pages/hardware_setup.py` | `_apply_hw_controls` restores the auto flag FIRST, then the stored manual levels (so stored levels win over the freeze-seed) |
| `tests/test_v75x_andor_zyla_camera.py` | `FakeAndorCam` mirrors the new backend surface |
| `tests/test_v75x_andor_display_scaling.py` | NEW — 21 tests |

## Implementation Steps

- [x] Backend state + reader-loop switch (auto records `_last_auto_levels`)
- [x] `_mono_to_bgr8(levels=)` + `_auto_levels` extraction (auto path byte-identical)
- [x] Control methods with clamping + freeze-on-toggle-off semantics
- [x] Widget capabilities + setters + log line
- [x] Manager delegates
- [x] Settings dialog rows, handlers, persistence, Defaults, readout
- [x] Startup restore in `_apply_hw_controls` (auto-then-levels order)
- [x] Tests + regression

## Testing Notes

- NEW `tests/test_v75x_andor_display_scaling.py` (21): backend defaults /
  freeze-on-toggle / no-reseed-on-repeat-toggle / clamp+ordering / settings
  keys; fixed-level conversion is deterministic and does NOT chase the scene
  (the regression this exists for), clips ends, degenerate-levels safe;
  widget+manager plumbing incl. non-andor returns False; dialog rows visible
  only for andor, slider-enable follows the auto flag, live-apply, persistence
  per identity, Defaults restores auto; `_apply_hw_controls` restore order +
  legacy blobs without the keys are ignored.
- Updated `FakeAndorCam` (mirror surface). Full andor suite 45 green;
  camera hardware-controls / calibration-store / image-correction (70) +
  cal-liveview / rotation-monitor / async-open (61) green.
- **Needs real-HW verification on the Zyla rig:** open the gear dialog on the
  microscope feed → uncheck "Auto display scaling" → image freezes at its
  current look and stops re-normalizing as objects move through; black/white
  sliders adjust it; exposure (ms) still drives the real sensor; restart
  restores the choice.

## Issues & Decisions

- **Honest naming:** the operator asked for "auto exposure gain" control, but
  the Zyla has no such hardware feature — the auto behaviour lives in OUR
  mono16→8-bit display conversion. The control is therefore named "display
  scaling" (tooltip explains it), not auto-exposure; hardware exposure keeps
  its own independent row. Reusing the dialog's `auto_exposure` checkbox was
  rejected: it disables the exposure spin while on, which is wrong here.
- **Freeze-on-toggle-off:** turning auto off seeds the manual levels from the
  last auto frame's levels — the image keeps its current appearance, giving
  the operator a sensible starting point. Stored levels are restored AFTER the
  auto flag at startup so a saved manual configuration wins over the seed.
- **Display-only:** the raw cached frame used by detection/calibration is the
  BGR8 conversion output (as before); manual levels affect it the same way the
  auto scale always did. µm/px and click mapping are unaffected (geometry
  unchanged).
- uint8 frames (non-Zyla path) still pass through untouched; levels apply only
  to >8-bit mono frames, matching the previous behaviour envelope.
