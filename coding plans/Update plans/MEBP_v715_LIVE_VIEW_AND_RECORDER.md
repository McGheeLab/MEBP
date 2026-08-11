# MEBP v7.15 — the broken recorder, split capture settings, and live-view zoom / pop-out

## Objective

Four operator reports against the v7.14 capture work:

1. **Recording is broken.** *"It says it's recording but (a) doesn't stop after the amount of time requested, (b) if I stop it manually it says failed to save."* Stills work.
2. *"The settings for the snapshot should be separate from the video settings — it's confusing when they are the same."*
3. **Zoom in/out + full screen** on the live view, and *"now that there are more buttons on any live view we may want to reconsider how all these buttons are displayed."*
4. **Pop out any live view to make it larger**, with *"all mouse clicks still landing where they should to select locations etc."*

**Operator decisions (AskUserQuestion, 3):** separate dialogs per button · hover-revealed toolbar · **detach the view itself** (a placeholder holds its slot).

---

## 🐞🐞 Stage 1 — why recording failed, diagnosed and confirmed on the operator's machine

`CameraWidget.frame_captured` is `Signal(object)` carrying a **QImage** (`camera_widget.py:232`, emitted `:1549`), but `_RecordingSession._encode_loop` treated it as a numpy array.

`orient_array` was contractually *"never raises"*, so it **swallowed the type error and returned a non-image** — measured: a 1-element ndarray at identity, or the QImage untouched when a flip is active. That reached `_open_writer`'s `h, w = img.shape[:2]`, which **raised inside a daemon thread with no `try`**. The thread died before `self.active = False`, so:

- **(a)** `active` stuck `True` → the 1 Hz tick never finalized the session and the `video_max_seconds` check sat inside the dead loop → **never stopped**;
- **(b)** manual stop → `_writer is None` → `"no frames were recorded"` → **"failed to save"**.

Both symptoms, one cause. Confirmed in `logs/app.log`: `Capture failed: recording failed: no frames were recorded` ×2, with four successful stills either side. **No traceback anywhere** — an unhandled exception in a `threading.Thread` goes to `threading.excepthook` → stderr, which a windowed app discards.

**Why the v7.14 tests passed:** `_FakeCam` used `SimpleNamespace(connect=lambda f: None)` — it never delivered a frame, so the encode path was never executed. A stub that agrees with its caller proves nothing.

### Fixes

- **NEW `capture_controller.to_bgr()`** — one conversion at the ingress boundary, accepting a QImage **or** an ndarray. ⚠ The QImage is **RGB888** and `cv2.VideoWriter` wants **BGR**: a type-only fix would have produced silently colour-swapped video. Row **stride** is honoured (`bytesPerLine`), because Qt pads rows and a naive `reshape(h, w, 3)` *shears* the picture — pinned at widths 5/7/13/65/99.
- **`_encode_loop` is guarded**, with `active` cleared in a **`finally`**. A raise can never again strand a session as permanently "recording".
- **`video_max_gb` is enforced.** It was defined in `CAPTURE_DEFAULTS`, shown in the dialog, and read by **nothing**.
- **Prompt self-termination** — the encoder emits `_session_ended` (queued onto the GUI thread) instead of waiting up to a tick; the timer stays as a backstop at 500 ms.
- **`orient_array` refuses a non-image up front** (not an ndarray, or `ndim < 2`) and returns `None`. Its never-raises contract still holds for genuine *orientation* failures, but defensive catching that moves a fault away from its cause is worse than no catching — that is what carried this failure three frames past the type error.
- The recording path now calls `validate_video` and reports unknown filename tokens (the still path already did; the video path discarded them), and writes the **JSON sidecar** — `write_sidecar` was unread on the video path despite its own tooltip calling the sidecar *"the only metadata a video can carry"*.

---

## Stage 2 — image settings and video settings, separated

`CaptureSettingsDialog` → **`ImageCaptureSettingsDialog`** + **`VideoRecordingSettingsDialog`**, over the same `CAPTURE_DEFAULTS`. Right-click 📷 opens one, right-click ⏺ the other.

The keys were always separate; the *presentation* was shared in ways that actively misled. Fixed with it:

- one preview rendered **both** names and **merged** their unknown-token warnings, so it never said which template was wrong;
- a video-only problem was displayed as a warning under **Images**;
- 🐞 **a still capture was blocked by a video-only problem** — `_do_still` called the combined `validate`, so a zero frame rate refused to take a photograph. `validate` split into `validate_still` / `validate_video`, with an AST test pinning that `_do_still` uses the still-only one;
- 🐞 `values()` **hard-coded `video_max_gb`**, so a stored value was reset to 8.0 on every OK;
- the "Metadata" box visually governed everything while `embed_metadata` / `write_sidecar` were stills-only.

⚠ **`Settings.set_section` REPLACES a whole section**, so each dialog writes back the **full merged dict**. A partial write would delete the other dialog's settings on OK — the load-bearing test here.

Also NEW `CaptureSpec.video_extension()` (the extension was inlined in two places).

---

## Stage 3 — ONE view geometry (the enabler for 3 and 4)

**The load-bearing change.** The widget→image mapping existed in **four independent copies**, and none knew about zoom or pan:

| copy | used by |
|---|---|
| `camera_feed_view._widget_to_image` | the only one `clicked` used |
| `live_target_picker._image_to_widget` | target rings, hit-testing |
| `live_target_picker._clamped_image_point` | drags |
| `measurement_camera_view._handle_drag` | endpoint drags |

Adding zoom to the click path alone would have moved the picture out from under the target rings — the v7.8 `to_px` identity-inverse bug returning at another layer.

**NEW `gui/widgets/view_geometry.py`** — an immutable `ViewGeometry` holding raw size, orientation `true_xform`, the visible `src_rect`, pixmap size and letterbox offset, with `to_image` / `to_widget` as exact inverses **by construction**. All four sites delegate; the four copies are deleted and a structural test forbids a fifth.

Zoom is a **crop** of the oriented image, then the usual fit — so only the visible region is resampled and the cost does not grow with the zoom factor on a 2048² frame. ⚠ The geometry is built from the **actual integer crop** Qt applied, not from recomputing the float rect, or hit-testing would sit up to a pixel out of step with what was drawn.

⚠ **`TargetOverlayCameraView` and `MeasurementCameraView` overrode `_render_frame` without calling `_orient_qimage`**, so `_view_true_xform` was never set and `_displayed_image_size` stayed `(0,0)` — their clicks only stayed correct because `_widget_to_image` had fallbacks for exactly that. Zoom added to the base alone would **silently not have applied to the picker**, which is the click-to-select surface (spheroid pickup, cell targeting, cell labeling). Extracted `_compose_pixmap` + `_publish_geometry`; every `_render_frame` now goes through them.

`geometry_map()` derives a mapping on demand when none was published, so a caller that sets `_last_pixmap` directly (several tests) and a future subclass that forgets to publish both still get correct clicks rather than silently dead ones.

**Interaction:** wheel zooms about the cursor, buttons about the centre, pan on **middle**-drag. ⚠ Left-click stays *selection* — a left-drag pan (`ScrollHandDrag`, or `RubberBandDrag`, which the repo records as claiming the left button view-wide) would take the button the whole feature exists for.

---

## Stage 4 — hover-revealed toolbar

Buttons hide until the pointer is over the feed. Seven at 26 px ≈ 206 px does not fit the 200 px minimum a `CameraFeedView` may be, so **anything that does not fit moves into a `⋯` overflow menu** — decided by measurement, not per-call-site configuration, so the same code serves the 4-up camera grid and the height-capped settings preview.

**The recording pill is exempt** and outranks everything while active: a recording you cannot see is one you cannot stop.

⚠ Layout gates on **`isHidden()`, not `isVisible()`** — `isVisible()` is False while any ancestor is unshown, so a page not yet displayed would lay out nothing and every button would pile up at (0,0) the first time it appeared. CLAUDE.md records this exact trap; the tests use the same rule, or they would pass vacuously.

---

## Stage 5 — detach into a window, and full screen

**NEW `gui/dialogs/camera_popout_dialog.py`.** The view MOVES into the window and a placeholder holds its slot; closing returns it to the **same index with the same stretch**. F11 toggles full screen, Esc leaves it.

Hazards handled explicitly:

- **`setParent(None)` is never used** — `calibration.py:11064` records that it kills the display of a running camera. Pinned by an **AST** test, because the phrase appears in this module's own comments explaining why it is avoided and a substring guard would fail on the explanation while passing on the real thing.
- ⚠ **The origin is captured BEFORE the view is reparented** — my first cut let the dialog's constructor adopt the view, so `indexOf` reported a slot in the *dialog's* layout and the placeholder landed in the wrong window entirely.
- **`_restore()` runs before `super().done()`** (AST-pinned): once the dialog finishes, its children are torn down and `CameraFeedView.closeEvent` disconnects the camera.
- `eventFilter` refuses to emit `clicked` while `_last_pixmap` is None and `_on_frame` skips rendering while hidden, so the view is **re-rendered on return** — a view that merely waited for the next frame would be click-dead in between, which on a long exposure is seconds.
- The cached camera-settings dialog is parented to `self.window()` and is dropped across the move.
- ⚠ **The page is never hidden.** Several pages stop their camera in `hideEvent`, and the needle-bore wizard **disarms the print floor** there — a pop-out that worked by hiding its host would be a safety change, not a layout one.

---

## Files modified

| File | Change |
|---|---|
| `gui/widgets/capture_controller.py` | `to_bgr` at ingress; guarded `_encode_loop` + `finally`; `video_max_gb`; `_session_ended`; per-kind validate; video sidecar |
| `SupportClasses/CaptureOrientation.py` | refuse a non-image input loudly |
| `SupportClasses/CaptureSpec.py` | `video_extension`; `validate_still` / `validate_video` |
| `gui/dialogs/capture_settings_dialog.py` | split into two dialogs over one shared base |
| **NEW** `gui/widgets/view_geometry.py` | the single widget↔image transform |
| `gui/widgets/camera_feed_view.py` | `_compose_pixmap` / `_publish_geometry`, zoom + pan, hover toolbar + overflow, `pop_out` |
| `gui/widgets/target_overlay_camera_view.py`, `live_target_picker.py`, `measurement_camera_view.py` | delegate to `ViewGeometry`; three duplicate mappings deleted |
| **NEW** `gui/dialogs/camera_popout_dialog.py` | detach / restore / full screen |

---

## Testing

**NEW `test_v715_recording.py` (24)** — ⭐ the fake camera is a real `QObject` with a real `Signal(object)` emitting a real **QImage built exactly as `camera_widget._grab_frame` builds it**, so the stub-agrees-with-caller gap that hid this cannot recur. Covers: a playable non-empty file end to end; colour surviving RGB→BGR; padded stride; self-stop at `video_max_seconds` and at `video_max_gb`; an exception in the encoder **ending** the session rather than stranding it; per-kind validation; and `orient_array` refusing non-images.
**NEW `test_v715_settings_split.py` (17)** — per-kind previews/validation, and the load-bearing "editing one kind preserves the other".
**NEW `test_v715_view_geometry.py` (27)** — ⭐ widget→raw→widget over orientation × mirror × flip_y × zoom × pan (192 assertions, self-guarded against a collapsed sweep); integer-crop agreement; the four copies pinned to one implementation; behavioural proof that the overlay view honours zoom **and that its clicks follow it**.
**NEW `test_v715_toolbar_and_popout.py` (20)** — hover reveal, the recording-pill exemption, overflow on a 200 px feed, no overlap, the gear's pinned position, detach/restore to the same slot **and still emitting clicks**.

**8/8 mutations CAUGHT**, each a real source edit reverted afterwards: no QImage conversion (**7 tests fail — both operator symptoms reproduced**) · no exception guard · `video_max_gb` unenforced · partial section write · hit-test ignoring the crop origin · subclass bypassing `_compose_pixmap` · recording pill losing its hover exemption · restore after `done()`.

⚠ **Two of my own tests were too weak and mutations caught them first:**
- the subclass guard asserted `"QPixmap.fromImage" not in source`; a mutation using an **aliased import** (`_P.fromImage`) sailed through it. Replaced by two behavioural tests (does the crop shrink; do clicks follow).
- the round-trip sweep started from **raw** pixels, so most samples fell outside the zoom crop and it silently thinned to 85 assertions. Rewritten to start from **widget** space, where "visible" is trivially known, with a floor that fails if the sweep collapses.

**Legitimate test updates:** the v7.14 `TestCaptureSettingsDialog` class is retired with a pointer (its widgets moved with the split; `test_v715_settings_split` is stricter — a full round-trip of every key each dialog owns, vs four); `TestOverlayButtons` and `test_gear_visible_when_controllable` adapted to the hover model (availability and on-screen are now two things), each strengthened so it cannot pass vacuously; and the v7.10 `test_picker_maps_raw_pixels_through_the_view_transform` **substring** guard replaced by the round trip it was standing in for.

**🐞 A pre-existing test-isolation bug found and fixed:** `test_v714_capture_ui` patched `EncodedVideoWriter._make_writer` and restored it with a **bare assignment**. Reading the attribute unwraps the `staticmethod`, so restoring it that way made it an **instance** method — `self._make_writer` bound, every later call got an extra `self`, the `TypeError` was swallowed by `open()`'s `except`, and **every recording for the rest of the process reported "no available video encoder"**. Invisible when the suite runs alone. Fixed, and pinned by a guard test.

**Regression, per-suite green:** v714 + v715 capture (248) · picker / orientation / square-up (273) · camera backends + v713 (187) · mosaic worker + fluorescence + unreachable (49) · bore wizard (60) · click-rim + spheroid detection + per-bore targeting (121) · camera hardware controls + cal-liveview (32) · suite hygiene (10) · `gui.app` import smoke.

⚠ `test_v710_camera_mount_square_up::test_a_live_turn_drives_the_residual_toward_zero` failed once in a batch that also contained a bad module name, and passed on two clean re-runs — it drives a worker thread for ~3.7 s and is timing-sensitive under load, not affected by this change.

---

## Bench verification (ME3B V1) — IN ORDER

1. **Record on the Zyla.** It must stop itself at the **10 s** already in `settings.json` (that is the limit that was being ignored), the file must play outside the app, and the colours must be right — not blue-for-red.
2. **Stop manually** before the limit: it saves and reports frames/duration instead of "failed to save".
3. Set *Stop at* a small size and confirm the recording ends on the **size** limit too.
4. **Right-click 📷 and ⏺** — two different dialogs, each with its own filename preview. Change the video name, reopen the image dialog, and confirm the image settings are intact (and vice versa).
5. **Zoom into a well on the Plate Location feed and click a rim point — while zoomed AND panned, not just fitted.** The stage must go where you clicked. This is the critical check; a zoom that moves the picture but not the mapping looks fine until the needle travels.
6. Middle-drag to pan while zoomed; confirm **left-click still selects** rather than panning.
7. **Pop out the picker feed** (spheroid or cell targeting), place a target in the big window, confirm it lands correctly, then close and confirm the view returns to its slot **and still responds to clicks**.
8. F11 in the popped-out window for full screen, Esc to leave.
9. Check a **needle-cam feed and the 4-up camera grid**: hovering reveals the toolbar, and the controls that do not fit are reachable from `⋯`.
10. Start a recording, move the pointer off the feed, and confirm the **red pill stays visible** and still stops the recording.

---

## Deferred / not done

- The overflow menu lists the buttons that did not fit; it does not yet group them (no submenu). Fine at seven controls, worth revisiting if more are added.
- Zoom state is per-view and resets on `zoom_fit`; it is **not persisted** across a page switch or restart. Nobody asked for that, and a view that reopens mid-zoom on a different plate would be confusing.
- `TargetOverlayCameraView`'s base `_draw_targets` now projects through the geometry, but plain `TargetOverlayCameraView` has **no production instantiation** (one test); the production surface is `_PickerCameraView`, which overrides it.
