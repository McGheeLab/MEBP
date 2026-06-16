# MEBP v7.5.x — ZP Axis Position Override (power-cycle recovery)

## Objective

Add a Hardware Setup control that lets the operator **manually override the
current position of any ZP axis** (Z, P1, P2, P3).

### Why this is needed

The ZP board (Marlin) has **no absolute encoder**. The app displays
`displayed = firmware_raw − zero_position[axis]`. The `zero_position`
software offset *is* persisted and restored across restarts
(`main.py:207` reads the `zero_position` settings section). The problem the
user hit:

- When the Marlin board loses power on shutdown (USB power-cycles), its
  internal position counter resets to **0** (or a stale value).
- On restart `zero_position` is restored correctly, but `firmware_raw` is
  now wrong, so the displayed position is wrong.
- **"Refresh Positions" just re-reads the wrong firmware value** — nothing
  in software can recover the true physical position. Only the operator
  knows where the axis actually is.

The fix is a manual override: the operator tells the firmware where the
axis physically is, which sends a Marlin **`G92`** to rebase the counter
(no motion). This is the generalization of the existing **Set Zero**
(`G92 <phys>0`).

## Design / semantics

- The operator enters the value in the **zero-referenced mm** frame — the
  same frame as the position readout, the Jog page, and the safety limits.
- `override_zp_position(axis, value)` sends `G92` setting the **physical**
  counter to `value + zero_position[axis]`, so the zero-referenced readout
  becomes `value` while the **established zero reference is preserved**
  (unlike Set Zero, which forces both the counter and the reference to 0).
- No motion occurs — `G92` only rebases the counter.
- Units are **mm** for all four axes, matching this hardware panel's
  safety-limit frame and readout (pumps are µL-native elsewhere, but the
  Stage panel is mm-native throughout).
- Persistence: the override changes only the firmware counter, not
  `zero_position`, so no settings change is required. It is a per-session
  re-sync; after the next power cycle it must be re-applied (this is
  inherent to hardware without an absolute encoder — auto-applying a stored
  position on connect would be unsafe if the axis moved while powered off).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | New `set_position(logical_axis, value_mm)` — generalization of `set_zero`; sends `G92 <phys><value>` via the live `axis_map`. |
| `SupportClasses/StageController.py` | New `override_zp_position(axis, value)` — computes `raw = value + zero_position[axis]`, calls `zp_stage.set_position`, logs to `PositionLogger`, returns a result dict. |
| `SupportClasses/ZPStageSimulator.py` | Added a `G92` handler (`_cmd_set_position`). Previously `G92` silently fell through to a bare `ok`, so even `Set Zero` never updated the simulated M114 readout. Now both `Set Zero` and the override are visible in sim (and testable). |
| `gui/pages/hardware/stage_panel.py` | New **"Override / Sync Axis Position"** group in the Stage sub-page: per-axis row (zero-ref readout + value spinbox + "⟳ Set Position") + explanation + status line. Handler `_override_axis_position`; readouts populated (zero-referenced) in `_update_position_displays`. |
| `tests/test_v75x_zp_position_override.py` | New test suite (14 tests). |

## Implementation Steps

- [x] `ZPStage.set_position` (G92 to arbitrary value, axis_map-aware)
- [x] `StageController.override_zp_position` (raw = value + zero, preserves zero ref, logs)
- [x] `ZPStageSimulator` G92 handler (`_cmd_set_position` + dispatch)
- [x] Stage sub-page "Override / Sync Axis Position" group + row builder + handler
- [x] `_update_position_displays` populates the override card's zero-ref readouts
- [x] Tests
- [ ] Architecture doc / README (defer to version-completion checklist)

## Testing Notes

`python -m unittest tests.test_v75x_zp_position_override`

Covers:
1. **Simulator G92** — rebases the named axis without moving; the `…0`
   (Set Zero) form; only named axes touched.
2. **`ZPStage.set_position`** — correct `G92` string; respects a
   non-default axis_map (ME3B V1); returns `False` for an unmapped axis.
3. **`StageController.override_zp_position`** — `raw = value + zero`; zero
   reference preserved; `previous_raw` read from the correct (axis_map)
   slot; rejects unsupported axes / disconnected stage; propagates an
   unmapped-axis failure.
4. **End-to-end** through a real `ZPStageManager(simulate=True)` —
   `set_position` then M114 reports the value; `set_zero` now actually
   zeroes in sim.

Manual (hardware): Hardware Setup → Stage → "Override / Sync Axis Position".
After a power cycle, enter where each axis physically is and click
**Set Position**; verify the readout (and the Jog page / status bar) match.

## Issues & Decisions

- **Root cause is hardware, not a persistence bug.** `zero_position` is
  already saved and restored. The lost value is the *firmware counter*,
  which only the operator can re-declare — hence a manual override rather
  than an auto-restore.
- **Why preserve the zero reference** (vs. Set Zero which zeroes it): the
  user is declaring the physical position in the coordinate frame the rest
  of the app already uses; resetting the reference would silently move the
  origin of the safety envelope and any calibration tied to it.
- **Simulator G92 was a latent no-op.** Adding the handler is a fidelity
  improvement and makes the existing Set Zero behave correctly in sim too.

---

## Addendum — last-known-position save + startup restore prompt

Builds on the override: instead of only a manual per-axis entry, the app now
**saves the last-known ZP position on clean shutdown** and, on the next ZP
connect, **asks the operator whether to assign it** to all ZP axes.

### Flow

- **Save (clean shutdown):** `MainWindow.save_settings()` writes a
  `zp_last_position` settings section — the zero-referenced mm value per axis
  (`Z, P1, P2, P3`) plus an ISO `timestamp`. Only written when ZP is connected
  and a real reading is available, so a disconnected-stage shutdown never
  clobbers a good snapshot.
- **Restore (startup, first ZP connect):** `StageController` fires a new
  `on_connect("ZP")` callback (mirrors `on_disconnect`) at the end of
  `connect_stages`. `app.py` bridges it via a `stage_connected` Signal to
  `_maybe_prompt_zp_position_restore` (GUI thread, deferred via
  `QTimer.singleShot(0)`), which shows the saved values + timestamp and asks
  *"assign these as the current position for all ZP axes?"*. On accept it calls
  `override_zp_position(axis, value)` per axis (G92, no motion). Once per
  session; skipped in simulation and when no snapshot exists.

### Why zero-ref (not raw)

The snapshot is stored in the same zero-referenced frame as the readout and the
override. On restore, `override_zp_position` adds `zero_position[axis]` back, so
the firmware counter lands exactly on the original raw value — provided
`zero_position` persisted (it now does, see
`MEBP_v75x_XY_ZERO_PERSISTENCE_HOTFIX.md`). The prompt shows the values the
operator recognizes (zero-ref mm), not raw counts.

### Files Modified (addendum)

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | New `on_connect` callback + fired in `connect_stages` (per-stage `*_just_connected` flags, after `set_stages`); new `get_zp_position_zero_ref()` helper. |
| `gui/app.py` | `stage_connected` Signal + `_emit_stage_connected`/`_on_stage_connected`; `_maybe_prompt_zp_position_restore` dialog; `save_settings()` writes `zp_last_position` (zero-ref + timestamp). |
| `tests/test_v75x_zp_position_restore.py` | New tests (7): zero-ref snapshot (axis_map-aware), save→restore reproduces raw, `on_connect` fires once on ZP connect. |

### Decisions (addendum)

- **Operator-confirmed, not automatic.** The position is only a *guess* (valid
  only if nothing moved by hand while powered off); auto-applying it could
  drive the needle from a wrong assumed height. The prompt keeps a human in the
  loop, defaulting to Assign but one click from Skip.
- **Clean-shutdown save.** A force-kill won't capture the latest snapshot;
  periodic throttled saves are a possible follow-up. The manual override card
  remains the always-available fallback.
