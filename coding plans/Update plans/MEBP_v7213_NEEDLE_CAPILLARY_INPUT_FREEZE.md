# MEBP v7.21.3 — Needle tool: the pulled-capillary fields freeze the app, and clamp what you type

## Objective

Two operator reports on Hardware Setup → Needle → **Pulled glass capillary**:

1. *"under pulled glass capillary it freezes every time i input a number"*
2. *"also it wont allow me to input the pulled tip diameter it puts a range on
   the value, i should be able to input anything i want."*

Both are fixed. They are independent defects that happen to meet in the same
card.

---

## 1. The freeze — ROOT-CAUSED FROM THE OPERATOR'S OWN `logs/freeze.log`

Not inferred. The v7.16 `GuiWatchdog` had already captured it: **six GUI-thread
stalls on 2026-08-19 between 16:21:11 and 16:22:31, every one of them rooted at
`hardware_setup._on_needle_changed`**, each stuck several frames deep inside a
*different* page:

```
_on_needle_changed → _on_config_changed → config_changed.emit
  → MainWindow._on_hardware_config_changed          (gui/app.py:1844)
    → _save_hardware_config      → settings.json write
    → _propagate_hardware_config (gui/app.py:2002)  → EVERY page
      → print_builder_sketch.set_hardware_config
          → _refresh_sequence → _seq_section_row        ← stall frame (16:21:32, 16:22:31)
      → calibration.set_hardware_config
          → _emit_calibration_data_changed → _push_cal_to_jog
            → workflows_mode → print_calibrator._push_line_object
              → quick_print.set_external_objects → _refresh_objects
                → _refresh_readiness → _render_readiness ← stall frame (16:21:11, :24, :59)
```

**`config_changed` is not a local notification.** The receiver writes
`settings.json` *and* calls `set_hardware_config` on every page, which cascades
into three separate widget-tree rebuilds (the sketch sequence, the calibration
page's own fan-out, Quick Print's readiness list). A `QDoubleSpinBox` emits
`valueChanged` on **every keystroke**, so typing a three-digit tip diameter ran
that entire cascade three times — hence "freezes every time I input a number".

Measured, so the shape of the fix is not a guess:

| | cost |
|---|---|
| the page's OWN work per edit (`_rebuild_config` + validity + readouts + max-flow) | **~1 ms** (profiled, 595 calls) |
| `settings.json` write | ~3 ms |
| each receiver in isolation, offscreen, no live feeds | 6–74 ms |
| the real cascade on the rig, all pages built + cameras streaming | **> 5 s** (watchdog threshold, ×6) |

### Fixed at the SOURCE, in one place

`_on_config_changed` keeps everything that is this page's own state
**synchronous** — it is ~1 ms, and debouncing it too would replace a freeze with
a stale panel. Only the app-wide emit is coalesced, through a 250 ms
`QTimer` (`CONFIG_EMIT_DEBOUNCE_MS`):

- `_schedule_config_changed()` — restarts the timer on every edit, so a burst
  collapses to ONE propagation.
- `_flush_config_changed()` — emits any pending config immediately.

**One enforcement point on purpose.** The page has ~25 call sites of
`_on_config_changed`; a per-call-site fix would leave the next one to be written
unprotected. Because the choke point is shared, this also fixes the same
per-keystroke fan-out on the **setup-name** and **notes** fields, the pump
settle/prime spins and the camera µm/px override — all of which had it too.

**A pending emit is never DROPPED**, because the receiver persists to
`settings.json`: an edit swallowed by the debounce would be a silently lost
setting, which is worse than the freeze. Every exit flushes —
`hideEvent` (navigating away), `QApplication.aboutToQuit` (quitting while still
on the page, the one exit no page-level event reports; the same pair the v7.16
crop-persist debounce already uses), and the timer itself.

**A whole-config LOAD flushes at once** rather than waiting out the debounce: it
replaces the plate, needle and pumps together and is one deliberate decision,
not a burst of typing (the same reasoning as the crop store's `persist_now`).

### Typed text now commits once, not per digit

`setKeyboardTracking(False)` on the six capillary spins. Typing `300` otherwise
walks the **live** needle through 3 µm and 30 µm — each a real config change
that was fanned out to every page and written to `settings.json`. Arrow steps
still apply immediately, so only the half that was never a value the operator
meant is suppressed.

---

## 2. The clamped tip diameter

`_CAP_SPIN_SPECS["tip_id"]` was `0.5 – 500 µm`, and the other five fields were
bracketed just as tightly (`barrel_id` 50–3000, `tip_len` 0.1–50 …).

**A spin box does not merely refuse an out-of-range number — it REWRITES it as
you type**, so the value that reached the config was one the operator never
entered, with nothing on screen saying so. The bundled preset library already
sat on the edge of it: `capillary-500-_m-tip` has a 500.0 µm tip, exactly the
old maximum, so no wider tip was expressible at all.

The ranges are now open (0 → 100 mm for diameters, 0 → 10 m for lengths) with
finer decimals (2 for µm, 3 for mm) so a sub-micron pull is typeable.
**Nothing is lost by removing the clamp, because none of it was being enforced
there:** every constraint the range encoded is already checked by
`HardwareConfig._needle_bore_issues` and reported BY NAME on the page —

- `Capillary tip inner Ø must be greater than 0 µm`
- `Capillary tip inner Ø (900.0 µm) cannot exceed the barrel inner Ø (100 µm)`
- `Capillary tip outer Ø must exceed its inner Ø` …

and `NeedleSpec` itself already logs *"tip ID 5000.0 µm exceeds barrel ID
1000.0 µm — a pulled tip should be narrower. **Keeping the values as
entered.**"* The model was always designed to accept and warn; only the widget
clamped. `0` is admitted throughout and reads as "not set", which `validate()`
reports.

The spec table is shared by the single-needle card AND every per-bore row, so a
backpack's second bore inherits the same open range by construction.

---

## Files Modified

| File | Why |
|---|---|
| `gui/pages/hardware_setup.py` | debounce timer + `_schedule_config_changed` / `_flush_config_changed`; flush on `hideEvent`, `aboutToQuit` and whole-config load; `_CAP_SPIN_SPECS` ranges/decimals; `setKeyboardTracking(False)` |
| `tests/test_v7213_needle_capillary_input.py` | NEW — 14 tests |

Nothing else changed. No model, no persistence format, no motion path.

---

## Implementation Steps

- [x] Reproduce + root-cause from `logs/freeze.log` (6 stalls, one chain)
- [x] Profile the page's own per-edit work (~1 ms) vs the fan-out (>5 s)
- [x] Debounce the emit at the single choke point
- [x] Flush on hide / quit / whole-config load
- [x] `setKeyboardTracking(False)` on the capillary spins
- [x] Open the six capillary ranges; confirm `validate()` covers what the range did
- [x] Verify all 6 bundled pull recipes still round-trip (0 flipped to "(custom)")
- [x] Tests + mutation matrix
- [ ] GUI verification on ME3B_01 (below)

---

## Testing Notes

`tests/test_v7213_needle_capillary_input.py` — **14 tests, green**, driving the
REAL `HardwareSetupPage` offscreen (a stand-in that debounces proves nothing
about the widget the operator types into).

**7/7 mutations CAUGHT**, source restored byte-identical:

| Mutation | Caught by |
|---|---|
| M1 debounce removed (**the original freeze**) | burst-fans-out-once |
| M2 `hideEvent` no longer flushes | leaving-the-page-flushes |
| M3 `tip_id` range back to 0.5–500 | sub-micron / wide-tip / floors |
| M4 keyboard tracking back on | commits-on-enter |
| M5 **the page readout debounced too** (stale panel) | readout-tracks-every-edit |
| M6 `aboutToQuit` no longer wired | quit-is-wired |
| M7 a setup load waits out the debounce | load-propagates-at-once |

M5 is the guard-the-guard: without it, a "fix" that also froze the page's own
readouts would have passed the burst test.

⚠ **The mutation harness itself needed a fix, and it cost real time.** Its first
run died on a `CreateProcess` failure *after* writing a mutation and *before*
restoring it, leaving **M1 (the debounce removed) on disk**. The next run then
captured the mutated file as its baseline and reported six "CAUGHT" verdicts
that were all contaminated by the un-debounced source (visible in hindsight:
every run showed extra failures). The restore now happens in a `finally`, and
the re-run gives clean single-failure attributions for M2/M4/M6/M7.

⚠ **Shell heredocs collapse `\n` here** (already recorded in CLAUDE.md) — it
broke both the test file and the harness; escape-heavy source was written with
the file tools instead.

Regression, run per-batch:

- needle/capillary/multibore + new suite — **184**, 1 pre-existing failure
- bore-offset / plate-builder-ui / camera-cal-store / suite-hygiene — **301**,
  3 pre-existing failures
- `gui.app` import smoke — OK

**3 pre-existing failures, all already documented in CLAUDE.md and none of them
ours:** `test_v79_needle_form_ui::test_every_on_disk_setup_round_trips_through_the_page`
(wants ≥6 setup files carrying a `needle` block; this tree has 3 — the
per-file round-trip subTests all PASS, which is the half that would have caught
a range/decimals regression) and
`plate_builder_ui::TestLearnLoopSavesToADesign` ×2 (literally `{'max': None}`).

---

## Issues & Decisions

- **Why not make the fan-out itself cheaper?** It is the real cost, but it means
  touching `set_hardware_config` on the sketch page, the calibration page's
  synchronous re-emit and Quick Print's readiness rebuild — three separate
  changes to pages this report does not implicate. Debouncing at the source is
  one change at one choke point and removes the reported symptom. Making a
  single pass cheap is a good follow-up, not this fix.
- **250 ms** — long enough to swallow a burst of keystrokes and a held spin
  arrow, short enough that a single deliberate edit lands before the operator
  can reach another control. Every exit flushes, so the number is not
  load-bearing for correctness.
- **The page stays synchronous.** Measured at ~1 ms, so there is no reason to
  defer it and a real cost if it were deferred (the operator would be reading a
  stale needle summary while typing).
- **Deliberately NOT changed:** the hypodermic gauge/length path (combos, no
  clamp to remove), and the other spin boxes on the page — the debounce already
  covers them, and switching *their* keyboard-tracking off is a feel change
  nobody asked for.

---

## Needs GUI verification on ME3B_01, IN ORDER

1. Hardware Setup → Needle → **Pulled glass capillary**, type a tip diameter —
   **no freeze**, and `logs/freeze.log` gains no new stall rooted at
   `_on_needle_changed`.
2. Type `0.4` and `750` into **Pulled tip ID** and confirm both **stay as
   typed** (the old build rewrote them to 0.5 and 500).
3. Confirm the barrel/tip readout line under the card tracks each committed
   edit **live** (if it lags, the debounce leaked into the page's own work).
4. Set a tip WIDER than the barrel on purpose → the value is kept and the page
   reports *"Capillary tip inner Ø … cannot exceed the barrel inner Ø"*.
5. Pick each bundled pull recipe and confirm the picker does **not** flip to
   "(custom)" the instant it is selected.
6. Edit a value, immediately navigate to another page, come back and restart —
   the value is still there (the flush-on-hide path).
7. Edit a value and close the app straight away → it survives the restart (the
   `aboutToQuit` path).
8. Load a saved setup file → the other pages pick it up at once, not a beat
   later.
9. ⚠ **Re-check this rig's saved `tip_od_um`.** The live config carries
   `tip_od_um: 1.0` with a 48 µm tip, which is impossible (an OD inside its own
   ID) and looks like a value the old clamp mangled. Set it to the real drawn
   OD, or 0 for "unknown".
