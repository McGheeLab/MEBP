# MEBP v7.21.8 — Video recordings are no longer capped at 10 minutes

## Objective

Operator: *"For the video captures, I would for them to be longer than 10
minutes."*

---

## Root cause

`CAPTURE_DEFAULTS["video_max_seconds"] = 600`, and the encoder loop
(`capture_controller._encode_loop`) stops the recording the moment
`elapsed_s` crosses it:

```python
if max_s and self.elapsed_s >= max_s:
    self._reason = f"reached the {max_s:.0f} s limit"
    break
```

The value was **already** operator-editable — the ⏺ button's right-click *Video
recording settings* has an "After [ N s ]" spin ranging 0–86400 with `0` shown
as *no limit*, and it persists to `settings.json` under `capture`. But this rig
has **no saved `capture` section**, so every recording ran on the 600 s default
and stopped at exactly ten minutes.

So this is a bad default rather than a missing feature — but a bad default that
silently truncates an experiment is a defect, not a preference.

---

## Design

### 1. The default becomes `0` (no limit)

The time cap never protected anything the **size** cap does not protect better.
A runaway recording is a *disk* problem, and **bytes fill a disk; minutes do
not**. `video_max_gb` (default 8 GB) measures the thing that actually matters,
so it becomes the guard and the clock is left to the operator.

Measured against this rig's cameras at 15 fps, 8 GB is worth:

| Camera | MB/min | 8 GB lasts |
|---|---|---|
| Tucsen Libra 25 (2600×2048) | ~40 | ~3.4 h |
| Andor Zyla (2048×2048) | ~32 | ~4.3 h |
| Teslong needle cam (1280×720) | ~7 | ~20 h |

Raw time-lapse is the exception — ~609 MB/min on the Tucsen, so 8 GB is ~13 min.
That is inherent to writing uncompressed 16-bit frames, and it is now **stated
on screen** rather than discovered.

### 2. A container-aware byte ceiling — the wall that replaces the clock

Removing the time cap is precisely what makes a **multi-GB file reachable**, and
that turns a latent container problem into a live one. AVI is RIFF, whose chunk
offsets are 32-bit: 4 GB is the absolute ceiling and the practical one is lower
(the OpenCV AVI muxer is widely reported to stop producing seekable files around
2 GB). At a 10-minute cap nothing got near it; without one, an 8 GB AVI is
ordinary.

New `CaptureVideoWriter.container_byte_limit(container, configured_bytes)`:

* **mp4** → exactly the operator's value (no comparable limit);
* **avi** → their value, or **2 GB** if theirs is larger or unlimited.

It is a **floor on trust, not a preference**: it only ever *lowers* the
configured cap, and only for AVI. A raw time-lapse is a *directory of TIFFs*,
not one container, so it is deliberately exempt — even when `video_container`
still says `avi` from a previous session.

### 3. The dialog answers the question the operator now has

With no time cap, "how long can I record?" is a division the operator would
otherwise do in their head. `_runtime_note` reports **whichever limit binds
first**, converted to time:

* `Stops after about 3.4 h — the 8.0 GB limit.`
* `Stops after about 4.3 h — the AVI 2 GB container limit.`
* `Stops after 10 min — the time limit.` (if they set one)
* `No automatic stop — it records until you press ⏺ again, or the disk fills.`

Quoting a limit that is *not* the one that will stop the recording would be
worse than quoting none, hence "binds first" rather than "the one you typed".
The clamp is folded in, so the number shown is the number that will happen.

---

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/CaptureSpec.py` | `video_max_seconds` default `600 → 0` |
| `SupportClasses/CaptureVideoWriter.py` | new `CONTAINER_MAX_BYTES` + `container_byte_limit()` |
| `gui/widgets/capture_controller.py` | `_encode_loop` resolves its byte limit through the clamp (skipped for raw time-lapse) |
| `gui/dialogs/capture_settings_dialog.py` | new `_runtime_note()` folded into `_info()`; tooltips on both limit spins |
| `tests/test_v7218_long_video_recording.py` | **NEW** — 24 tests |

No migration: the value is read through `merged_settings`, so an install with no
stored `capture` section picks the new default up, and one that stored a limit
keeps it.

---

## Implementation Steps

- [x] Default → 0 (no limit)
- [x] `container_byte_limit` + `CONTAINER_MAX_BYTES`
- [x] Encoder loop applies it (and not to raw time-lapse)
- [x] Dialog reports the binding limit as a duration
- [x] Tooltips on the time + size spins
- [x] Tests + mutation matrix

---

## Testing Notes

`tests/test_v7218_long_video_recording.py` — **24 tests, green.**
**Mutation matrix: 12/12 CAUGHT**, sources restored byte-identical, baseline
asserted green before scoring.

⚠ **Two mutations SURVIVED the first run, and the reason is the lesson this
project keeps re-recording.** The clamp tests originally *re-implemented* the
encoder loop's `max_b` resolution and asserted on the copy, plus an AST check
that `container_byte_limit` is called *somewhere* in the module. Both passed
with the production branch turned into `if False:` — i.e. with the clamp removed
from the shipping code entirely — because the copy still worked and the call
node still existed inside the dead branch. Rewritten to **start a real recording
and assert whether the session actually stops**, reusing the v7.15 harness whose
camera is a real `QObject` emitting a real `QImage`. Both mutations are now
caught, and the class carries a companion mp4 test so it cannot pass because
*everything* stops.

**Regression, all green:** v7.15 recording · v7.14 capture core + UI (117) ·
fluorescence capture · ND3 export · v7.21.6 capture history · suite hygiene (97).

**Offscreen smoke through the REAL `VideoRecordingSettingsDialog`:** a fresh
install shows the spin reading **"no limit"**, the note reads *"Stops after about
17.3 h — the 8.0 GB limit"*, switching the container to AVI re-reads *"about
4.3 h — the AVI 2 GB container limit"*, and typing 600 s back in re-reads
*"Stops after 10 min — the time limit"*. Plus a `gui.app` import smoke.

---

## Issues & Decisions

* **Why not just raise 600 to, say, 3600?** It moves the surprise rather than
  removing it — an operator recording a 90-minute experiment hits the same wall
  one hour later. The size cap already bounds the disk, which is the only thing
  the time cap was really doing.
* **Why keep a size cap at all?** "No time limit" must not mean "nothing stops
  it". A forgotten recording still ends, at a bounded number of gigabytes; a
  test pins that the size default stays non-zero for exactly that reason.
* **The 2 GB AVI figure is conservative and deliberately so.** 4 GB is the RIFF
  structural maximum; the practical OpenCV limit is reported lower and is not
  verified on this bench. A clean stop slightly early beats a file that looks
  finished and will not play — and since AVI recordings could never previously
  exceed 10 minutes, no existing recording can be affected by the clamp.

---

## Disclosed, NOT done

* **No free-disk pre-flight.** `CaptureVideoWriter.enough_disk_space()` exists,
  is written and documented, and has **zero production callers** — it was never
  wired. Wiring it is genuinely more attractive now that the size cap is the
  sole guard, but its `headroom=2.0` would refuse an 8 GB-capped recording
  unless 16 GB is free, which would block recordings that would have completed
  fine. That is a behaviour change worth making deliberately, with the operator
  choosing between "refuse" and "warn", so it is left out rather than smuggled
  in here. Recorded so it is not mistaken for coverage.
* A very long single file is unwieldy regardless of format; **segment-rollover**
  (auto-starting `..._002.mp4` at the size cap instead of stopping) is the
  natural follow-up if hours-long runs become routine.

---

## Needs verification on ME3B V1, IN ORDER

1. **Go/no-go:** right-click ⏺ → *Video recording settings*. "After" reads
   **"no limit"**, and the grey note under it names a duration in hours, not
   minutes.
2. Record for **more than 10 minutes** and confirm it is still going — this is
   the whole request.
3. Stop it manually and **play the file back**; check the duration matches the
   wall clock (the frame-rate machinery repeats/drops frames to keep real time,
   and a long recording is the first thing that would expose drift there).
4. Set "After" to 60 s, confirm it stops at a minute and the note said it would
   — the setting must still work, not just be ignored.
5. Switch the container to **AVI** and confirm the note changes to the 2 GB
   figure. If you record a long AVI, confirm the resulting file actually plays;
   if it does not, the clamp needs to come down further and that is worth
   knowing.
6. **Raw time-lapse** is the one mode where the size cap binds quickly (~13 min
   at 8 GB on the Tucsen). Confirm the note says so before starting a long one,
   and raise `video_max_gb` if you need more.
7. Leave a recording running while you work on other pages, then close the app —
   `finalize_all_recordings` should still produce a playable file.
