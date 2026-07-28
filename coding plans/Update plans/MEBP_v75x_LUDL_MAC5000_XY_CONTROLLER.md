# MEBP v7.5.x — LEP MAC 5000 XY controller support

## Objective

Add support for a **Ludl Electronic Products (LEP) MAC 5000** XY-stage
controller alongside the existing Prior ProScan II / III, so the app sends the
correct wire commands to whichever controller a given machine has, and its
**speed, acceleration, and travel-range** settings stay consistent with that
stage. Selection is **per-machine** (device profile) with connect-time
auto-detect as a fallback; the Ludl protocol/units are fully **tunable** on the
bench; a **Ludl simulator** exercises the path in sim mode.

The XY layer is a "JSON-protocol abstraction" (`config/controllers/*.json` +
`ControllerProtocol` + `XYStageManager`), but five behaviors were hardcoded
Prior-isms in `XYStage.py`. This update generalizes them to be protocol-driven
(defaulting to Prior behavior, so the Prior JSONs are byte-for-byte unchanged),
adds `mac5000.json`, a `LudlStageSimulator`, a per-machine controller field, and
the UI to pick it.

### Decisions (AskUserQuestion)
- **Per-machine + auto-detect** — controller type is a `DeviceProfile` field
  (`xy_controller_json`) written into the global `controller.controller_json`
  key on apply; auto-detect remains the fallback.
- **Research + make tunable** — the Ludl protocol was researched (Micro-Manager
  `Ludl.cpp` + shared ASI/LEP command-set docs); units/speed/accel/travel are
  tunable defaults dialed in on the bench.
- **Build a MAC 5000 simulator** — standalone `LudlStageSimulator` (the working
  Prior sim is untouched → zero regression risk).

## The 5 hardcoded Prior-isms generalized (all default to Prior)

| # | Spot (`XYStage.py`) | Prior | LEP MAC 5000 |
|---|---------------------|-------|--------------|
| 1 | move send | raw µm (`xy_position_scale` unused) | counts (µm × `position_scale`) |
| 2 | ack read | bare `R` | `:A` / `:N -<code>` |
| 3 | `_parse_position_response` | `x,y,z` CSV, strip `R` | `WHERE X Y` → `:A <x> <y>` (whitespace, 2-axis) |
| 4 | `set_speed_mm_s`/`set_velocity` | % of max (SMS) | absolute per-axis `SPEED` (counts/s) |
| 5 | `set_acceleration` | 1-100 (SAS) | 1-255 ramp index (ACCEL) — range from protocol, template does the rest |
| +6 | serial open | `STOPBITS_ONE` hardcoded | `stop_bits` from protocol (2) |
| +7 | command-mode init | n/a | raw `0xFF 0x41` bytes at connect (force ASCII) |

## Files Modified / Created

**Modified**
- `SupportClasses/ControllerProtocol.py` — new optional schema getters:
  `family`, `command_mode_init_bytes` (from `command_mode_init_hex`),
  `ack_success_token` + `match_ack_error`, `get_position_parse`, `speed_model` /
  `speed_units` / `speed_is_per_axis`, `accel_model` / `accel_units`,
  `position_scale`. All default to Prior behavior.
- `SupportClasses/XYStage.py` — `_position_scale()` + `_drain_ack()` helpers;
  scale applied in `move_stage_to_position` / `move_stage_relative`;
  `_parse_position_response` is now a protocol-dispatched instance method;
  `set_speed_mm_s` / `set_velocity` branch on `speed_model` (+`_set_speed_absolute`,
  clamps to the µm/s ceiling and `speed_range` for safety); `_find_with_protocol`
  applies `stop_bits` and writes `command_mode_init_bytes`; `__init__` sim branch
  picks `LudlStageSimulator` when `family=="ludl"`.
- `SupportClasses/StageController.py` — `set_controller_json()` setter (applies on
  the next XY connect).
- `gui/pages/hardware/device_profile.py` — `xy_controller_json` +
  `xy_max_speed_um_s` fields with full round-trip; `apply_to_settings` writes
  `xy_controller_json` into the global `controller.controller_json` key (None ⇒
  inherit, doesn't clobber); `from_settings` reads it back.
- `gui/pages/hardware/stage_panel.py` — "XY controller" combo in
  `_build_xy_cal_group` + `_populate_xy_controller_combo` /
  `_select_xy_controller` / `_relabel_xy_cal_for_json` / `_on_xy_controller_changed`;
  `_apply_xy_calibration` + `_load_from_settings` + `_load_selected_profile`
  persist/restore/push the choice; accel spinbox range/label relabels per model
  (Prior 1-100 / Ludl 1-255 ramp).
- `gui/pages/settings_page.py` — `_apply_settings` pushes `set_controller_json`
  to the live controller.

**Created**
- `config/controllers/mac5000.json` — the LEP MAC 5000 protocol (researched
  values; bench-TBD fields flagged in its `notes`).
- `SupportClasses/LudlStageSimulator.py` — Ludl HLC simulator (µm internally,
  counts on the wire, `:A`/`:N` acks; own state file `sim_xy_state_ludl.json`).
- `tests/test_v75x_ludl_xy_prior_regression.py`,
  `tests/test_v75x_ludl_protocol_semantics.py`,
  `tests/test_v75x_ludl_xystage.py`,
  `tests/test_v75x_ludl_simulator.py`.

## Implementation Steps

- [x] Extend `ControllerProtocol` schema + getters (Prior-safe defaults).
- [x] Add Prior-behavior regression test; green before touching `XYStage`.
- [x] Generalize `XYStage.py` (7 spots) + family-based sim selection.
- [x] `config/controllers/mac5000.json`.
- [x] `LudlStageSimulator.py` (standalone) + wire selection.
- [x] `DeviceProfile.xy_controller_json` + `xy_max_speed_um_s` round-trip;
  `StageController.set_controller_json`.
- [x] Stage-panel combo + model relabel + persistence; settings-page push.
- [x] Ludl tests (semantics / xystage / simulator) + Prior regression.
- [ ] **Real-HW verification on the MAC 5000 machine** (bench checklist below).
- [ ] Register in the CLAUDE.md "Existing Update Plans" table (done at completion).

## Testing Notes

Run:
```
python -m unittest tests.test_v75x_ludl_protocol_semantics \
  tests.test_v75x_ludl_xystage tests.test_v75x_ludl_simulator \
  tests.test_v75x_ludl_xy_prior_regression
```
54 tests green (incl. the Prior-frozen regression). Regression suites
re-verified: jog-navigation / axis-map-jog / axis-max-speed / common-axis-speed /
jog-direction-z-up-sign / xbox-axis-speed / z-retract / z-axis-setup (134) and
last-known-calibration / calibration-revision / plate-orientation (58) all green.
Prior + Ludl simulator smoke both move→read correctly.

**Bench checklist (MAC 5000 machine):**
1. Device profile with `xy_controller_json` = `mac5000.json`; XY connect forces
   ASCII (`FF41`), `VER` → `:A …`, auto-detect (fallback) picks Ludl by the
   leading `:`.
2. Jog → correct direction/magnitude; read-back tracks → tune `xy_position_scale`
   (µm/count) until commanded µm == measured µm.
3. "Measure top speed" → confirm the absolute conversion; if speed runs wildly
   off, switch `speed_units` `counts_per_s` ↔ `um_per_s` (LEP vs ASI firmware).
4. Set/record travel envelope; `safe_travel_to` retract-before-XY ends at safe Z,
   travels to the right place.
5. Small multi-well print → correct wells, ends at safe Z. Confirm the Prior
   machine (ME3B V1) is byte-for-byte unaffected.

## Real-hardware verification (2026-07-22, COM5)

Probed a live MAC 5000 on COM5 read-only (no motion):
- `VER` → `Version no. : 5.100  MAC5000 Interface with USB`; `WHERE X Y` → `:A 399 321`.
- Confirmed **9600-8-N-2 + `0xFF 0x41`** ASCII-mode init (mac5000.json defaults correct).
- Through MEBP's own code: `ControllerProtocol` loads Ludl, detection ACCEPTs, and
  `XYStageManager._parse_position_response(":A 399 321")` → (39.9, 32.1) µm (scale 10).
- Full `XYStageManager` connect with `exclude_ports=["COM4"]` opened **only COM5**,
  detected "Ludl MAC 5000", read live position — **COM4 never opened** (proven by a
  `serial.Serial` recording wrapper).

**Detection fix (found on HW):** `VER` replies `Version no. …` (NOT `:A`-prefixed), which
would fail the `^:[AN]` match — and `_find_with_protocol` gates *explicit* selection on
that match too, so the MAC 5000 wouldn't connect. Changed `detection.firmware_query` to
`WHERE X Y` (replies `:A <x> <y>`) + added a `MAC5000` identify token. `/` and `STATUS X`
returned empty on this firmware (motion_status poll unavailable) — non-blocking, since
completion is detected by position polling.

**DTR-reset-on-scan fix:** opening ANY serial port asserts DTR, which auto-resets an
Arduino/Marlin board — so the XY detection scan opening the ZP port (COM4) would reset
the ZP stage. Added `XYStageManager(exclude_ports=…)`; `_find_with_protocol` skips
excluded ports **without opening them**. `StageController.connect_stages` passes the live
`zp_connected_port` + the persisted `_preferred_zp_port` (loaded at startup from
`settings.zp_stage.last_port`), so an XY connect never resets the ZP board regardless of
connect order. Tests: `TestExcludePorts` in `test_v75x_ludl_xystage.py`.

## Follow-up fix: Xbox stick jog was a no-op on the MAC 5000 (2026-07-22)

**Report:** "the xbox controller is not moving the xy stage" (after live-connecting
the real MAC 5000 on COM5). **Root cause (as designed, not a regression):** the
Xbox stick drives XY via `XYJogHandler` → `XYStageManager.move_stage_at_velocity`
at a steady ~10 Hz, which maps to Prior's continuous-velocity `VS` command. The
Ludl HLC command set has **no continuous-velocity primitive** (`set_velocity` is
correctly `null` in `mac5000.json` — MOVE/MOVREL are the only motion commands), so
`_send_protocol_command("set_velocity", ...)` silently returned `None` every tick —
exactly the limitation flagged in the original plan, but worth fixing rather than
leaving as a dead end.

**Fix:** `XYStageManager.move_stage_at_velocity` now checks
`protocol.has_command("set_velocity")`; when absent it delegates to a new
`_jog_pulse(vx, vy)` — integrates the commanded velocity (µm/s) over the actual
wall-clock time elapsed since the last call into a running accumulator, and fires
a real `move_stage_relative(...)` once the accumulated delta clears one wire count
(avoids flooding the serial link with sub-resolution MOVRELs that would round to 0
counts anyway). `vx==vy==0` (stick released / the jog-handler's staleness watchdog)
resets the accumulator/timer so the next jog start doesn't inherit a stale gap.
Prior (which HAS `set_velocity`) takes the exact same continuous-VS path as
before — verified zero `move_stage_relative` calls during a Prior velocity jog.

Verified in the **Ludl simulator** (no real-HW risk to test the mechanism): holding
`move_stage_at_velocity(2000, 0)` at 10 Hz for 0.6 s moved the simulated stage
~800 µm in +X, confirming the pulse path actually drives motion end-to-end through
the same `move_stage_relative` → `MOVREL` → count-scale path already proven on the
real MAC 5000. Tests: `TestJogPulseFallback` in `test_v75x_ludl_xystage.py` (5
cases: Ludl delegates to `_jog_pulse` / Prior does not / accumulation fires a move
once past threshold / sub-threshold doesn't fire yet / stop resets state).

**Still needs a live-Xbox bench check:** the stick should now visibly (if coarsely
— pulsed, not smooth) jog the MAC 5000; confirm direction/feel and that holding the
stick for a while doesn't desync the accumulator from reality (a dropped/garbled
MOVREL ack under `_drain_ack` is logged but non-fatal, matching the existing
drain-and-continue discipline).

## Definitive answer: can the MAC 5000 move "at velocity" like Prior? (2026-07-22)

The user explicitly asked for TRUE continuous-velocity motion (Prior's `VS` semantics —
commanded velocity vector, moves indefinitely until countermanded) for like-for-like
downstream behavior, not the pulsed-relative-move approximation.

**Research** (Micro-Manager `Ludl.cpp`/`Ludl.h` grepped directly for `VE`/`VECTOR`/
`SCAN`/`JOYSTICK` — zero matches; cross-referenced against ASI's *separate*
`ASIXYStage.cpp` adapter, which DOES implement a capability-probed `VE`/`VECTOR`
command): ASI's `VECTOR`/`VE X=<mm/s>` is a real continuous-velocity primitive on
genuine ASI MS-2000/Tiger firmware (ramps to the commanded velocity, runs indefinitely,
`VE X=0` or `HALT` stops it) — but there is no evidence it exists on Ludl-manufactured
MAC 5000/6000 firmware specifically; the production, hardware-tested `Ludl.cpp` adapter
implements only `MOVE`/`MOVREL`/`SPEED`/`STSPEED`/`ACCEL`/`HALT`/`WHERE`/`STATUS`/`HOME`.

**Empirically confirmed against the real MAC 5000 (COM5), zero-risk probe:** `VE X=0`,
`VE X=0 Y=0`, `VE X?`, `VECTOR X=0` — every variant returned **`:N -1` (unrecognized
command)**. **This firmware genuinely has no continuous-velocity command.** The
pulsed relative-move fallback (`XYStage._jog_pulse`) is not a workaround for something
we failed to find — it's the correct, necessary approach for this hardware. The
*software interface* (`move_stage_at_velocity`) is identical to Prior either way, so
downstream callers (Xbox jog, any future continuous-scan feature) don't need to know
or care which family is connected — that is the achievable form of "like-for-like."

## Incident: uncommanded ~15mm drift during live testing (2026-07-22) — root-caused, benign

While probing further (testing `SPEED X=0.3`, a fractional value), the controller
returned a garbled reply (`:A N-2  N-2`) and then went **silent on every subsequent
query — including plain `WHERE X Y`** — for dozens of polls. A fresh port reconnect
restored communication, but the reported position had moved **~-8.6mm X / +15.8mm Y**
from the last known-good reading, with **zero visibility** into what happened during
the silent window. All further live testing paused immediately; user confirmed the
stage was not at a hard limit.

**Root cause, confirmed via careful single-step-verified follow-up testing:** the
malformed `SPEED X=0.3` corrupted the **X-axis SPEED register** to a tiny integer
value (**84 counts/sec ≈ 8.4 µm/s** — read back cleanly via `SPEED X?` once
communication was restored; Y's `SPEED` was unaffected at `230400`, confirming only X
was hit). **The "drift" was NOT runaway/erratic motion** — it was the earlier
`MOVREL X=3000`×2 test moves *genuinely, correctly, exactly* executing in the
background at that crawl rate (Ludl `MOVREL` is documented async — it keeps running
after the host disconnects), just far slower than any reasonable poll timeout expected,
so every position-settle loop gave up early looking like "no response," and the total
displacement accumulated across several test scripts before anyone re-checked. Once
`SPEED X` was restored to an integer value (`SPEED X=230400`, matching Y), motion
speed returned to normal immediately, confirmed via a clean, fast, exact-delta test
move. **Lesson hardened into every subsequent test script:** integer-only arguments
(never a float like `0.3` again — likely not a valid format for this firmware's numeric
parser, which corrupted its own state on receiving one), verify every single response
before sending the next command, and abort (HALT + stop, no further commands) on any
malformed/empty reply rather than pushing through it.

## ACCEL is a ramp-TIME index — bench-CONFIRMED backwards from typical intuition

User asked to "try maxing out acceleration" for a snappier jog — typical motion-
controller intuition (higher ACCEL = faster ramp) suggested this should help. Tested
cleanly (SPEED held constant at a restored-normal value; same 200 µm test move,
strict single-step verification): **ACCEL=1 settled in 0.61s; ACCEL=255 settled in
0.88s — LOWER is FASTER.** This matches the ramp-TIME-index reading of the ASI/Ludl
docs (not a ramp-RATE): a longer/slower ramp is a LARGER number. **Do not "max out"
ACCEL on this stage expecting speed — default it LOW (near 1) for responsive
jog/print motion.** `config/controllers/mac5000.json` `parameters.acceleration`
updated 100→1 and its `notes` field documents the measured numbers; the Device page
now shows "Acceleration (ramp 1-255, LOWER = faster):" for Ludl with a hint
explaining the inversion, and a freshly-selected Ludl profile defaults the
acceleration spinbox to 1 instead of inheriting Prior's mid-range 50
(`StageHardwarePanel._xy_accel_family_default`, `_relabel_xy_cal_for_json`).

## Stage-specific Hardware Setup: live velocity/acceleration read-back (2026-07-22)

Follow-on request after the ACCEL finding: a dedicated way to see/set velocity and
acceleration PER STAGE TYPE on the Hardware Setup page, motivated directly by the
SPEED-corruption incident (there was no way to SEE that X had silently diverged from
Y — only ad-hoc bench scripts caught it).

**Protocol layer:** new optional `get_max_speed`/`get_acceleration` command entries —
Prior (`proscan_ii.json`/`proscan_iii.json`): bare `SMS`/`SAS` (no value = query,
matching the existing simulator's modeled behavior). Ludl (`mac5000.json`): per-axis
`SPEED {axis}?` / `ACCEL {axis}?` (confirmed-working format from the earlier live
probing). `ControllerProtocol.format_command(..., axis=...)` is safe for Prior too —
`str.format` silently ignores an unused `axis` kwarg on templates with no `{axis}`
placeholder.

**`XYStage.py`:** new `_query_protocol_value(command_name, axis)` — a write-AND-read
helper (unlike `_send_protocol_command`, which is fire-and-forget on real hardware);
`_extract_first_int` tolerates the real Ludl reply quirk where a single-axis `?` query
appends trailing junk for the OTHER axis (`ACCEL X?` → `:A 20 N-2`, confirmed on real
HW) and treats an explicit `:N` as an error, not garbage-with-a-number. Public
`get_speed_readback(axis)` / `get_acceleration_readback(axis)` return
`{raw, axis, model, display}`, best-effort (`raw=None`/`display='unavailable'` on any
failure — unsupported, error reply, not connected — never raises).

**`LudlStageSimulator.py`:** now tracks `_speed_raw`/`_accel_raw` PER AXIS (previously
only a shared physics ceiling) and answers the `?` query form by echoing back what was
actually last SET per axis — lets a test reproduce the exact real-world divergence
scenario (`SPEED X=84 Y=230400` → reading back shows 84 vs 230400) without needing
real hardware.

**UI (`stage_panel.py`):** new "Live Stage State (read from hardware)" card inside XY
Stage Calibration — X/Y velocity + X/Y acceleration display labels, a "Read from
stage" button (`_read_xy_stage_state`), family-aware: Ludl shows both axes (genuinely
independent registers) and flags a MISMATCH in yellow if X≠Y; Prior collapses to a
single shared row (querying "axis" is meaningless there — hides the redundant Y row,
relabels "Velocity (X):"→"Velocity:"). Read-only — no motion, no settings written.

**Real-HW verified (COM5, read-only, through the actual production code path — not a
raw script):** `get_speed_readback`/`get_acceleration_readback` correctly returned the
stage's current values (Speed X=Y=230400, Accel X=Y=255 — matching what the earlier
live testing had left the registers at). **Note: the hardware's ACCEL is still at 255
(slow) from that testing — it does NOT automatically pick up the new low-default
recommendation until the operator opens Hardware Setup, reviews/adjusts the
Acceleration spinbox, and clicks "Save XY Calibration" to push+persist a low value.**

Tests: `TestReadback` in `test_v75x_ludl_xystage.py` (7 cases, mocked query) +
`test_readback_round_trip_through_real_sim` in `test_v75x_ludl_simulator.py` (real sim,
no mocking, reproduces the exact divergence scenario). 146 tests green across the full
Ludl + jog/axis/speed suite.

## Still to confirm on the bench (needs motion)
- **µm-per-count** (`xy_position_scale`, default 10): jog a known distance and tune
  until commanded µm == measured (not yet checked against a physical ruler/caliper).
- **SPEED units**: `SPEED X?` on this unit currently reads in a scale consistent with
  "counts/sec" (230400 for a fast-feeling default) — `speed_units: counts_per_s`
  appears right for this firmware; no evidence yet of the ASI mm/s variant.
- **Xbox-stick feel** with the corrected low-ACCEL default — not yet tested live with
  a physical controller (only the pulsed-jog *mechanism* was verified, in the
  simulator, before this session's live-hardware testing).

## Issues & Decisions

- **Continuous vector-jog (`set_velocity` / VS) is unsupported on Ludl HLC** — the
  `set_velocity` command is `null` in `mac5000.json`, so `move_stage_at_velocity`
  is a graceful no-op (Xbox stick jog does nothing on Ludl); click/arrow
  point-to-point jog works. Documented in the JSON `notes`.
- **Auto-detect collision**: a Ludl `:A <ver>` could in principle match Prior
  III's permissive `response_pattern`; `discover_controller_files` is
  alphabetical so `mac5000.json` is tried first, and per-machine selection is the
  primary mechanism. The Prior JSONs were left byte-identical (no pattern
  tightening) to preserve the regression guarantee.
- **Two bench-critical unknowns** (flagged, not blocking): µm-per-count and
  SPEED units are stage/firmware-specific — shipped as tunable defaults.
- **`set_acceleration` needed no code branch** — its clamp range already comes
  from the protocol's `acceleration_range` (Ludl `[1,255]`) via
  `_apply_protocol_parameters`; only the UI label/range relabels per model.
- **Simulator: standalone, not a shared base** — chosen over the plan's optional
  refactor of `XYStageSimulator` because a concurrent session was actively
  editing the repo; zero regression risk to the working Prior sim.
- **Travel envelope stays µm-native** — `SafetyLimits` clamps in µm *above*
  `XYStage`; the count scale is applied strictly inside `XYStage`, so
  `plate_axis_sign`/`plate_flip_180` geometry and clamping are family-agnostic
  and unchanged.
