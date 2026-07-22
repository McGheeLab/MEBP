# MEBP v7.5.x — Calibration Usability Pop-up

## Objective

Add a **"usability" pop-up** that communicates when the **last calibration for XY,
Z, and P (pump)** was performed, and warns the operator to recalibrate when either
of two thresholds is crossed (operator request, `coding plans/changes.txt` line 22 —
Cason, 7.21):

- **M** — if the XY stage has *traveled more than M distance* since the last XY
  calibration, warn **"recalibrate X/Y"**.
- **H** — if it has been *more than H hours* since a calibration, warn **"update"**.

M and H are settings on the **Hardware Setup** page.

### Operator-confirmed decisions (AskUserQuestion)

1. **Pop-up trigger:** on startup, show the pop-up **only when attention is needed**
   (a threshold tripped, or a type never calibrated). Otherwise stay silent. An
   always-available **"Calibration status…"** button (Hardware Setup → Device)
   opens the same pop-up on demand.
2. **Travel counting:** **all** XY stage motion counts (manual jog + click-travel +
   prints + calibration hops), measured from position-poller deltas (one chokepoint
   that catches everything).

---

## Design

### New per-machine store — `SupportClasses/CalibrationStatusStore.py`

Mirrors `CalibrationSnapshotStore` (atomic temp+`os.replace`, module singleton
`get_store()`, env override `MEBP_CALIBRATION_STATUS_DIR` for test isolation). File:
`config/hardware/calibration_status.json`.

```json
{
  "version": "1.0",
  "thresholds": { "xy_recal_travel_mm": 1000.0, "recal_interval_hours": 168.0 },
  "xy_travel_um": 0.0,
  "xy": { "at": "2026-07-21T12:00:00", "travel_um": 0.0 },
  "z":  { "at": "2026-07-21T12:00:00" },
  "p":  { "at": "2026-07-21T12:00:00" }
}
```

- `add_xy_travel_um(delta_um)` — thread-safe accumulate (called from the poller
  thread); **throttled** persistence (in-memory accumulate, flush at most every
  `_SAVE_INTERVAL_S`≈15 s via `time.monotonic()`), `flush()` forces a write.
- `mark_calibrated(kind, when=None)` — `kind ∈ {"xy","z","p"}`; stamps ISO time
  (`timespec="seconds"`, matching `zp_last_position.timestamp`); **xy also
  snapshots the current odometer** into `xy.travel_um`. Immediate save.
- `get_calibrated_at(kind)`, `hours_since(kind, now=None)` (None if never / bad),
  `get_xy_travel_um()`, `xy_travel_since_cal_um()` (None if never XY-calibrated,
  else `max(0, odometer − xy.travel_um)`).
- `get_thresholds()` / `set_thresholds(m_mm=None, h_hours=None)` (immediate save).

**Why the store owns M/H (not `HardwareConfig`):** only the pop-up consumes them;
no controller/workflow needs them, so the config fan-out buys nothing and would add
coupling. The store becomes the single owner of all calibration-status state + its
thresholds (a per-machine property, like the odometer).

### XY odometer — `SupportClasses/StageController.py`

`PositionPoller` gains `on_xy_travel: Callable | None` and a `_last_odom_xy`
previous-sample cache. In `_poll_loop`, right after `self._xy_pos = pos`, it diffs
the new absolute-µm XY against the previous sample and fires `on_xy_travel(dist_um)`.
Guards:

- prev/current must be valid floats (skip after connect / on `(None,…)`);
- `_last_odom_xy` reset to `None` in `set_stages()` (a new stage / disconnect must
  not diff against a stale frame);
- a generous per-step sanity cap (`_ODOM_MAX_STEP_UM` = 1e6 µm = 1 m) drops a
  single pathological jump (corrupt read / re-zero) — a real move, even the full
  envelope traversed while the poller was suspended, is well under this.

`StageController.__init__` wires `self._pos_poller.on_xy_travel =
self._note_xy_travel_um`, which forwards to `get_calibration_status_store()
.add_xy_travel_um(...)` (best-effort, never raises into the poller). `shutdown()`
calls `get_calibration_status_store().flush()`.

> The poller is suspended during `safe_travel_to` / prints, so a suspended move is
> counted as one straight-line chord on resume (net displacement, not path length).
> Acceptable for an odometer; documented.

### Timestamp stamping (three subsystems)

- **P** → `gui/pages/hardware/stage_panel.py::_pump_setup_persist` — after the
  plunger setup persists, `mark_calibrated("p")`.
- **Z** → `stage_panel::_z_setup_capture_top` — after `apply_z_setup` persists (the
  canonical Z-axis datum), `mark_calibrated("z")`. *(Also stamped at the calibration
  page's Plate-Z autocal finish if a single clean site exists; else documented.)*
- **XY** → `gui/pages/calibration.py` at the plate-teach finalize handlers
  (`_manual_fit_xy`, `_accept_plate_scan`, `_ploc_finish_run`,
  `_ploc_finalize_click_rim` — the exact set confirmed while implementing) via a
  small `_mark_xy_calibrated()` helper.

### Hardware Setup UI — `gui/pages/hardware/stage_panel.py`

New **"Recalibration Reminders"** group (Device sub-page, near Z Axis / Pump Plunger
Setup): `M` spinbox (mm, 0 = disabled), `H` spinbox (hours, 0 = disabled), **Save**,
and a **"Calibration status…"** button → `show_calibration_status(self, force=True)`.
Reads/writes the store singleton directly (no `HardwareConfig`/`settings` coupling).

### Pop-up — `gui/dialogs/calibration_status_dialog.py` (new) + `gui/app.py`

Standalone `show_calibration_status(parent, *, store=None, force=True) -> bool`:
resolves the store singleton, computes info lines (XY/Z/P last-cal time + "N h ago")
and warning lines (XY travel > M → recalibrate XY; hours > H or never → update),
and:

- `force=False` (startup): returns `False` without showing if nothing needs
  attention; otherwise shows a `QMessageBox` (Warning icon).
- `force=True` (button): always shows (Information icon when clean).

`app.py` mirrors `_maybe_prompt_calibration_restore`: a `self._usability_prompted`
once-per-session flag + `QTimer.singleShot(600, self._maybe_show_calibration_status)`
in the page-wiring tail (after the existing restore prompts so it doesn't collide).

---

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/CalibrationStatusStore.py` | **NEW** — odometer + per-type timestamps + M/H thresholds store |
| `SupportClasses/StageController.py` | `PositionPoller.on_xy_travel` + `_last_odom_xy`; `_note_xy_travel_um`; wire + flush on shutdown |
| `gui/pages/hardware/stage_panel.py` | "Recalibration Reminders" group (M/H + Save + status button); stamp Z + P |
| `gui/pages/calibration.py` | stamp XY at plate-teach finalize handlers |
| `gui/dialogs/calibration_status_dialog.py` | **NEW** — `show_calibration_status()` |
| `gui/app.py` | `_usability_prompted` flag + `_maybe_show_calibration_status` startup trigger |
| `tests/test_v75x_calibration_usability_popup.py` | **NEW** — store round-trip, odometer poller, threshold eval, dialog build |

---

## Implementation Steps

- [x] `CalibrationStatusStore` + singleton + `$MEBP_CALIBRATION_STATUS_DIR` override
- [x] `PositionPoller` odometer hook (`on_xy_travel` / `_accumulate_xy_travel` +
      jitter floor / sanity cap / frame-reset) + `StageController` wiring
      (`_note_xy_travel_um`) + shutdown flush
- [x] Stamp Z (`_z_setup_capture_top`) + P (`_pump_setup_persist`) in `stage_panel`
- [x] Stamp XY in `calibration.py::_save_calibration` (change-signature gated so a
      Z-only save or a calibration *load* doesn't bump the XY time)
- [x] "Recalibration Reminders" group (M/H + Save + "Calibration status…" button)
- [x] `calibration_status_dialog.evaluate_calibration_status()` +
      `show_calibration_status()`
- [x] `app.py` startup trigger (once/session, only-when-attention) + `save_settings`
      odometer flush
- [x] Tests (`tests/test_v75x_calibration_usability_popup.py`, 14) + offscreen
      panel/dialog smoke + adjacent suites green

## Status Tracking

`[x]` implemented + unit-tested. **Needs real-HW / GUI verification on ME3B V1.**

### Concurrency note

StageController.py was being edited by a concurrent session while this landed;
the odometer edits had to be re-applied against the churned file (only the
`PositionPoller` init attrs survived the first pass). Re-verified all four
insertion points are present + compile.

## Testing Notes

- Unit: store round-trip (thresholds/timestamps/odometer), throttled save + `flush`,
  `hours_since`/`xy_travel_since_cal_um` (inject `now`), threshold evaluation
  (never-calibrated / travel-over-M / hours-over-H / all-clean).
- Poller: drive `PositionPoller` with a fake XY stage returning a sequence of
  positions → assert the odometer accumulates the summed chord and the sanity cap /
  None-guards hold.
- GUI (offscreen): stage-panel group builds + Save round-trips the store; dialog
  builds and `force=False` returns `False` when clean.
- **Needs real-HW / GUI verification on ME3B V1** (pop-up fires on the right
  conditions; travel accrues during jog; the three stamps update on each calibration).

## Issues & Decisions

- Thresholds owned by the store, not `HardwareConfig` (see Design rationale).
- Odometer counts poller-sample chords; suspended moves counted as net chord.
- "P" is a single most-recent-pump timestamp (any pump's plunger setup updates it),
  matching the request's XY/Z/P granularity.
