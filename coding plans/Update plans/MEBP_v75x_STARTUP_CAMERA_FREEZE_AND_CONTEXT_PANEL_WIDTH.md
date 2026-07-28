# MEBP v7.5.x — Startup camera freeze + left context-panel width

## Objective

Two startup-usability issues reported by the operator:

1. **The camera boot-up freezes the UI at startup.** The saved cameras
   auto-start when the app launches, and the blocking device open
   (`cv2.VideoCapture` / ToupCam / Andor — each takes up to several seconds) ran
   **synchronously on the Qt GUI thread**, so the whole UI (including the live
   camera feeds it's trying to bring up) locked up until every saved camera was
   opened.

2. **The left context panel can't be widened enough to show its buttons.** On a
   modest-width window the panel is stuck at its minimum and many jog/control
   buttons are clipped; it only becomes resizable "after a certain width" (i.e.
   after the window is stretched wide enough).

Both are software-only.

---

## Root causes

### (1) Camera freeze

`hardware_setup._auto_load_cameras` already moved the slow **OpenCV device
probe** off the GUI thread (`CameraManager.detect_cameras_async`), but the
subsequent **device open** — `_start_saved_cameras` → `CameraManager.start(i)` →
`CameraWidget.start()` → `cv2.VideoCapture(idx, CAP_DSHOW)` / `ToupCamBackend.open()`
/ `AndorBackend.open()` — still ran on the GUI thread. That open is the multi-second
blocking call, so the event loop (and every `QTimer`-driven camera feed) was
frozen for the duration of opening each saved camera.

### (2) Context panel width

The left context box (`ui_extraLeftBox`) had `minimumWidth = s(340)` and a
**dynamic maximum width** = `total − right − reserve`, with
`_content_reserve_px = s(460)`. On any window where the context-splitter width is
below `min + reserve` (≈ `340 + 460 = 800` base px, and **×1.5 at 150 % DPI** →
~1200 physical px), `total − reserve` drops below the minimum, so:

* `_update_context_panel_bounds` capped the panel's *maximum* at `340` → the
  handle couldn't move at all, and
* `_apply_saved_context_width` opened the panel at `min(saved=540, max=340)` =
  340 → **narrower than its designed width, clipping the buttons.**

The panel's designed "everything fits" width is `LEFT_BOX_WIDTH = 540`
(comment in `ui_functions.py`: 400 was too narrow). So the fix is to never let
the panel be forced below that.

---

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/camera_widget.py` | New `CameraWidget.start_async(on_done=None)` — opens the device on a daemon thread, marshals the opened handle back to the GUI thread via a new private `_open_result` signal, and finalizes there (`_on_open_result`: assign handle → set `_running`/`_backend_type` → start the display `QTimer`). New `_release_handle`. Async-open state (`_open_token`/`_opening`/`_pending_on_done`) init in `__init__`; `stop()` bumps `_open_token` so an in-flight open is superseded (its result is discarded + handle released). Simulated backend + already-running/unavailable cases finish synchronously. |
| `gui/widgets/camera_manager.py` | `start()` kept **synchronous** (mosaic scan etc. grab a frame immediately after) but now only emits `camera_started` when the widget actually ends up running. New `start_async(cam_idx)` routes to the widget's `start_async`, emitting `camera_started` from the completion callback (same signal contract as the sync path); falls back to sync `start` on an older widget. |
| `gui/pages/hardware_setup.py` | `_start_saved_cameras` (the startup auto-start) now calls `mgr.start_async(i)` when available (fallback to `mgr.start(i)`), so the blocking opens run off the GUI thread. |
| `gui/app.py` | Context splitter: `ui_extraLeftBox.setMinimumWidth(s(AppSettings.LEFT_BOX_WIDTH))` (was `s(340)`) so the panel is never narrower than its designed width → all buttons always visible. Content pane (slot 1) is now **non-collapsible** so it yields by shrinking (children scroll/clip) instead of snapping shut — removing the reason the dynamic-max had to be so conservative. `_content_reserve_px` reduced `s(460)` → `s(240)`. `_update_context_panel_bounds` / `_apply_saved_context_width` unchanged in shape but now read the larger min, so the max is never capped below it. |
| `tests/test_v75x_camera_calibration_store.py` | The three `_start_saved_cameras` tests stub `mgr.start`; they now also stub `mgr.start_async` (the new startup code path) with the same synchronous fake. |
| `tests/test_v75x_camera_async_open_and_context_width.py` | **New** — 8 tests (async open success/fail/supersede-on-stop, manager `start_async` emits `camera_started` when running, sync `start` intact for simulated; context-panel bounds never cap below min / grow with window / designed width reachable on a modest window). |

---

## Implementation Steps

- [x] `CameraWidget.start_async` + `_on_open_result` + `_release_handle` + async-open state
- [x] `stop()` supersedes an in-flight async open
- [x] `CameraManager.start_async` (sync `start` preserved for immediate-frame callers)
- [x] `hardware_setup._start_saved_cameras` uses `start_async`
- [x] `app.py` panel min = designed width, content non-collapsible, reserve reduced
- [x] Update the 3 affected calibration-store tests
- [x] New test module (8 tests)
- [x] Byte-compile + full camera/context regression green (141 tests)

---

## Testing Notes

* Automated (offscreen, `MEBP_UI_SCALE=1.0`):
  * `tests/test_v75x_camera_async_open_and_context_width.py` — 8/8.
  * Regression: `test_v75x_context_panel`, `test_v75x_camera_hardware_controls`,
    `test_v75x_camera_cal_liveview`, `test_v75x_camera_image_correction`,
    `test_v730_simulated_camera`, `test_v75x_camera_calibration_store`,
    `test_v75x_camera_rotation` — 141/141 green.
* **Needs real-HW verification on ME3B V1 / V3:**
  * At launch the UI stays responsive while the saved cameras come up (the feeds
    populate progressively rather than freezing the window). Confirm the
    microscope/needle cameras all end up streaming and `camera_started`-driven
    behaviour (hw-control restore, preview state) still fires.
  * Stop/close during startup doesn't leave a camera streaming or crash (the
    supersede-on-stop path).
  * The left context panel opens showing **all** its buttons on a normal
    (non-maximized) window, and can be dragged wider; content pane shrinks/scrolls
    rather than snapping.

---

## Issues & Decisions

* **Kept `CameraManager.start()` synchronous.** The mosaic-scan path
  (`calibration.py::_ploc_*`) starts the camera and **immediately** calls
  `get_current_frame` / `capture_fresh_frame`, which needs the device already
  open. Making `start` globally async would regress it. Only the **startup
  auto-start** — where nothing needs a frame right away and the freeze is real —
  uses the new `start_async`.
* **Concurrency:** an `_open_token` is bumped by `stop()` (and each new open); a
  finalize whose token is stale (or that finds the widget already running)
  releases the just-opened handle and bails, so a stop/restart during an in-flight
  open can never leave a camera streaming behind the app's back.
* **Enumeration left on the GUI thread.** The DirectShow/ToupCam/Andor *enumeration*
  in `CameraManager.detect_cameras` still runs on the GUI thread (COM prefers the
  main thread and it's comparatively fast). The dominant freeze was the device
  *open*, now async. A follow-up could background the enumeration if needed.
* **Panel width:** pinning the minimum to the designed width (540) guarantees the
  buttons are always visible whenever the box is shown, at the cost of the content
  pane yielding on very narrow windows (it shrinks/scrolls — content is now
  non-collapsible, so no "snap to full open" jump). Chosen over reducing the
  reserve alone, which is bounded by the content pane's real minimum and couldn't
  reliably let the panel reach 540 on modest windows without risking that snap.
  **Superseded by the responsive addendum below** — the panel content now *fits
  itself* to the width, so the hard 540 minimum was relaxed to 260.

---

## Addendum (2026-07-24) — responsive "sized to fit" panel content

**Operator follow-up #1:** "there are still issues with resizing. I want the
content of the left context menu and the main menu to be sized to fit. We may need
to do adaptive text sizing and button sizing. Almost like a css file for html."

**Operator follow-up #2 (after a first FlowLayout-wrap attempt):** "now it looks
terrible. we want the left context menu to look well designed and easy to use.
make the button and edit text fields and labels properly sized. things like the
jog axis sections should be aligned. The contents should be sized to fit the width
of the context menu. so no scrolling is allowed in the context menu. its ok to
scrunch the buttons, we just need to make sure the text gets smaller as well."

**Final design — aligned grid, fit-to-width, no horizontal scroll.** The first
attempt used a wrapping `FlowLayout`, which looked ragged/misaligned → **rejected
and removed** (`gui/widgets/flow_layout.py` deleted). Instead the content now
fills the width via **aligned `QGridLayout`s with expanding controls**, and the
fonts shrink with the width so text stays inside its (scrunched) control.

### The "CSS-like" layer

| File | What |
|------|------|
| `gui/widgets/responsive.py` | **`container_scale(width, design, lo, hi)`** — a CSS `clamp()` analogue: 1.0 at the design width, shrinking toward `lo` below it and growing toward `hi` above; 1.0 for non-positive inputs. Applied *on top of* the global DPI `scale_factor()`. `quantize(scale, step)` coarsens the factor so resize handlers skip identical re-layouts (no resize→relayout churn). |

### Applied to the jog panel — `gui/widgets/jog_button_array.py` (rewritten)

* **Aligned step selectors:** the three step rows (XY / Z / P) now share ONE
  `QGridLayout` — col 0 = axis label, cols 1-5 = magnitude buttons, col 6 =
  custom field — so the labels and button columns line up across all three rows
  (a tidy segmented-control look). Magnitude buttons + custom fields are
  `Expanding`, and the columns carry equal stretch, so the row always *fills and
  fits* the panel width (no horizontal scroll); on a narrow panel they scrunch.
* **Direction pad** = a centred cluster (XY pad + Z column) in its own grid with
  outer stretch columns; **pump columns** = a 3-column grid with equal stretch so
  they fill the width and align.
* **Fonts scale with width:** font-size + min-width were pulled out of the
  buttons' inline QSS so they're set via `_set_pt` / `setMinimumWidth`. On
  `resizeEvent`, `container_scale(width, _DESIGN_BASE=470, 0.62, 1.15)` (quantized)
  → `_apply_scale()` recomputes every label/button/field point size, height, the
  label-column width and the direction-pad button size. So text gets smaller as
  the panel narrows (the operator's explicit ask), buttons scrunch, and nothing
  clips.
* All sizing lives in centralised base constants (`_BASE_*`, `_FONT_*`).
* Public API (signals, `xy_step_um` / `z_step_mm` / `pump_step` /
  `pump_step_is_percent`, `_pump_buttons`, constructor) unchanged — verified by
  the jog-direction / speed suites and by rendered snapshots at 260 / 380 / 540 px.

### Panel plumbing — `gui/app.py` + `gui/widgets/context_panel_host.py`

* Left context box minimum dropped `s(540)` → **`s(260)`** (still *opens* at
  `LEFT_BOX_WIDTH`; the small min lets the user drag it narrow now that the
  content fits itself). Content pane stays non-collapsible; reserve stays `s(240)`.
* The context host's native-widget `QScrollArea` now sets
  `setHorizontalScrollBarPolicy(ScrollBarAlwaysOff)` — the content fits the width,
  so a horizontal scrollbar must never appear. (Vertical scrolling stays available
  for a tall panel — see note below.)

**Main menu (nav sidebar):** left as-is — its labels already fit within the
expanded `MENU_WIDTH` (200 base px) at every DPI, and it collapses to a 60 px icon
rail, so there was no width problem to solve there.

**Note on "no scrolling":** horizontal scrolling is now impossible (content fits
the width). *Vertical* scrolling remains available because the full
`StandardJogContextPanel` (jog + speeds + live position + Absolute Go-To +
Hardware Info + Illumination) is taller than most windows; eliminating vertical
scroll entirely would need either restructuring into collapsible sections/tabs or
shrinking everything to unreadable sizes. Flagged for the operator to decide.

### Tests

* `tests/test_v75x_responsive_context_panel.py` (10): `container_scale`/`quantize`
  math + clamps; the jog array registers the right widget counts for scaling,
  `_apply_scale` shrinks/grows buttons+fonts, `resizeEvent` drives the scale, and
  the jog signal contract (Z↑=+, ◀=−dx, aspirate=−/dispense=+) is unchanged.
* Regression at DPI 1.0 **and** 1.5: jog-direction / axis-speed / common-axis /
  per-pump-rate / jog-motion / jog-pump-fill / context-panel / camera suites green.
* Rendered offscreen snapshots (260 / 380 / 540 px) confirmed the layout is
  aligned and fits the width at every size.

**Needs real-HW/GUI verification on ME3B V1** (drag the context panel from ~260 px
to wide: the jog rows stay aligned + fill the width, buttons scrunch + text
shrinks with them, no horizontal scrollbar; jogging still moves the right axis by
the selected step).

### Rev 2 (2026-07-24) — 100 px minimum + fully-proportional whole panel

**Operator:** "the min width for the left context menu should be 100px and the
width of each item in the menu should take up a percentage of the context menu
width."

Every item in the panel is now proportional, and the panel fits down to **100 px**
(rendered + verified offscreen through a real `QScrollArea` host at 100 / 130 /
200 / 320 px):

* **`gui/app.py`** — context box minimum `s(260)` → **`s(100)`**.
* **`jog_button_array.py`** — magnitude buttons / custom fields / pump buttons
  keep equal column stretch but their *minimums* dropped to a few px
  (`_BASE_PRESET_MIN_W=5`, `_BASE_PUMP_MIN_W=6`, …) so stretch alone drives the
  width (each = a %% of the row). Font floor lowered (`_MIN_SCALE=0.42`,
  `_MIN_FONT_PT=4.5`) so text keeps shrinking to 100 px. Added a
  `minimumSizeHint()` override (small width) so the parent can drive it narrow —
  otherwise the direction pad's per-scale fixed buttons floored the width.
* **`control_panel.py`** — speed rows made proportional (label / spin / resolved
  each a stretch share; spin + resolved get `Ignored` h-policy + tiny min); the
  "Jog speed…" / "Max speed…" headings `setWordWrap`; the Refresh button + the
  safety-warning text can shrink; position rows made proportional (bar + value
  `Ignored` + tiny min). Added a `minimumSizeHint()` override (small width).
* **`standard_jog_context.py`** — Absolute-Go-To spins + Hardware-Info rows made
  proportional (`Ignored` + tiny min, stretch columns). Added a
  `minimumSizeHint()` override (small width) — this is the actual native widget
  in the context scroll area, so its small min is what lets the box reach 100 px.
* **`components.py::Card`** — the card title uses `Ignored` h-policy + tiny min so
  a long title elides instead of flooring the card's width (helps every card that
  lands in the narrow panel).

Net: the panel's natural min dropped 338 → ~96 px; combined with the box min of
100 px and the always-off horizontal scrollbar, the content is genuinely
*sized to fit* the width at any width ≥ 100 px — buttons scrunch, text shrinks,
nothing scrolls sideways. Tests: `test_v75x_responsive_context_panel.py`
(+`TestPanelMinimumsAllowNarrow`, DPI-relative) green at 1.0× and 1.5×.

### Rev 3 (2026-07-24) — uniform font scaling across the WHOLE panel

**Operator:** "why can't the items in the context menu just get smaller by a
percentage of the width of the context menu?" (AskUserQuestion → **scale every
control's text**, vs. a QGraphicsView zoom).

Rev 2 scaled the jog buttons' fonts but let the speed / position / Go-To /
Hardware-Info sections *clip* their text (via `Ignored` policy) rather than
scale it — so only some items got smaller with width. Rev 3 makes it uniform:

* New **`responsive.scale_descendant_fonts(root, factor, skip_subtrees=…)`** —
  records each descendant widget's base point size once, then sets
  `pointSizeF = base × factor`. A single "shrink the text by a %% of the width"
  pass. `skip_subtrees` excludes children that scale themselves.
* **`HardwareControlPanel.resizeEvent`** and
  **`StandardJogContextPanel.resizeEvent`** now compute
  `factor = container_scale(width, s(440), 0.45, 1.12)` and call it over their
  own widgets — the control panel skips the self-scaling `JogButtonArray`; the
  context panel skips the self-scaling `HardwareControlPanel`. So every level
  scales its own text, and the whole panel shrinks uniformly with the width.
* Stripped the hard-coded `font-size` from the QSS of the resolved-speed label
  and the Hardware-Info rows (QSS `font-size` overrides `setFont`), so those now
  scale too. Card titles keep their QSS size and elide (short labels).
* A visible bonus: the verbose safety-warning banner now scales down with the
  rest instead of dominating the narrow panel.

Verified offscreen through a real `QScrollArea` host at 140 / 240 / 340 px —
`_font_factor` tracks the width (0.44 → 0.54 → 0.78) and all text scales together,
no clipping. Tests: +`TestDescendantFontScaling` (scale + skip; control-panel
font factor shrinks with width) — green at 1.0× and 1.5×.
