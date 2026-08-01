# MEBP v7.7 — Quick Print: usability, live information, and post-print reporting

## Objective

The v7.6 hardware run proved the print *engine* works (10 sections / 9 corner stops,
`path_end status=arrived`, p95 27 µm inside the 30 µm element, time estimate within 2 %) while
exposing that **the interface hid almost everything the software knew**. Operator brief: *"make it the
best most straightforward as well as keep the user as informed as possible with all of the data we can
give them. we may want to include post print reports to help the user understand the print output."*

Evidence that motivated each thrust:

- The **default motion mode was `open_loop`** and `_kick_prediction` returned early unless the mode was
  `velocity`, so by default there was no predicted path, no time estimate, no overlays and no
  simulation — all of the v7.6 feed-plan work was dark out of the box.
- The two knobs the code itself calls *"THE TWO PARAMETERS everything else derives from"* were buried
  in a modeless popout; the page had four inline controls.
- The simulation predicted **7 µm**, the machine delivered **27 µm** (**3.75×**), and nothing in the
  product closed that loop.
- **No post-print report of any kind.** The only artefact was the JSONL execution log, surfaced as a
  bare filename in a grey label — and **no reader for it existed anywhere in the repo**.
- Bioprinting checks that already existed were never run for a print: `InkSpec.can_flow_through` /
  `flow_compatibility_detail` (clog risk), `FlowPhysics.wall_shear_stress` vs the 5 Pa cell-viability
  limit, and the needle flow ceiling was computed against a fixed `REFERENCE_VISCOSITY_CP` rather than
  the selected ink without telling the operator.
- ~18 concrete defects, including a **doubled progress counter**, `Abort` **dead during a plain-print
  preposition while the stage was moving**, dead code, nine popout fields with no change handler, and a
  refusal message naming a control label that does not exist.

## Decisions (operator, via AskUserQuestion)

1. **Post-print report** = in-app Report view **plus** self-contained HTML + CSV export, reusing the
   four existing QPainter widgets in `gui/pages/print_results.py`. **matplotlib must not become a GUI
   dependency.**
2. **Motion mode auto-selects per machine** (velocity + feed plan when characterised, else open-loop).
3. **Bioprinting checks are advisory badges with a one-line reason; hard-block only the physically
   impossible** (ink particles larger than the needle bore).
4. **Three zones: Setup / Run / Report** — promotion and regrouping, not rewrites.

## Files Modified

**New**
| File | Why |
|---|---|
| `SupportClasses/PrintReadiness.py` | Pure, Qt-free readiness/advisory model (`Check` / `Readiness` / `ReadinessContext` / `evaluate`). Unit-testable without hardware. |
| `SupportClasses/PrintLogReader.py` | **The first reader for `logs/prints/*.jsonl`**, plus the adapter to the report widgets' sample shape and the report-side analyses. |
| `gui/pages/workflows/quick_print_report.py` | The Report panel + `compare_prediction` + self-contained HTML / CSV export. |
| `tests/test_v77_print_readiness.py` (39) · `tests/test_v77_print_log_reader.py` (23) · `tests/test_v77_quick_print_zones.py` (54) | |

**Modified**
| File | Change |
|---|---|
| `gui/pages/workflows/quick_print_workflow.py` | Three zones, readiness checklist, live-run panel, report hand-off, the Stage-0 defect fixes. |
| `gui/dialogs/workflow_settings_dialog.py` | `notify_on_field_change`, `register_external`, `set_extra_state`, tooltip→help fallback + Help-toggle registration. |
| `SupportClasses/PrintManager.py` | `vel_sample` gains `v_meas_mm_s` + `deposited_uL`; new `on_vel_sample` callback; `deposit()` returns the cumulative volume. |
| `SupportClasses/PrintExecutionLogger.py` | `path_points()` — stores the (decimated) PRINT_PATH points so a saved log can be scored. |
| `gui/widgets/print_trajectory_monitor.py` | `set_predicted_path(..., predicted_p95_um=)` so the prediction stays visible beside the live deviation. |
| `tests/test_v75x_quick_print_predicted_overlay.py` · `tests/test_v75x_print_setup_routine.py` | Legitimate updates (new kwarg; the retired `speed_pct` knob). |

**Reused, not modified**: the four widgets in `print_results.py` · `XYChallenge.path_report` /
`verdict_for` · `XYFeedPlan.build_plan` / `min_attainable_resolution_um` · `components.Card` /
`StatusBadge` / `FormRow` · `FlowPhysics.wall_shear_stress` / `max_safe_flow_rate_uL_s` ·
`InkSpec.flow_compatibility_detail` · `CalibrationStatusStore` ·
`PrintTimingCalibrationStore.effective_dead_time_s`.

## Implementation Steps

### Stage 0 — defect fixes — `[x]`
- `[x]` Doubled progress counter (`[14/57] [14/57]`) — `_PROGRESS_PREFIX_RE` guard.
- `[x]` Terminal states report **why**: the specific error / ZP-disconnect text is remembered instead of
  being overwritten by "Error — see log".
- `[x]` An abort/error now states the fluidic consequences that previously existed **only in the log
  file**: cleanup did not run so the needle still holds ink, and the dispensed volume is indeterminate.
- `[x]` **`Abort` live during any positioning phase** (was gated on an active executor, so dead through
  a plain-print preposition while the stage was travelling).
- `[x]` `Pause`/`Resume` exposed (`PrintManager.pause()`/`resume()` existed, unreachable).
- `[x]` `Open log` button; dead `_update_top_speed_cap` now called; `_update_settings_summary`'s silent
  `except: pass` now logs and shows a fallback; post-clean refusal names the real checkbox; navigator
  tooltip corrected.
- `[x]` The nine silently-editing fields fixed **structurally**: `notify_on_field_change` connects every
  registered field to a debounced `on_change`, so no future field can go stale either.
- `[x]` Inline help falls back to each widget's tooltip and the rows register with the top-bar Help
  toggle (previously `help=` was passed on **zero** of ~25 fields, so the toggle had nothing to reveal).
- `[x]` Multi-ink mapping persists via the new generic `set_extra_state` hook.
- `[x]` The prediction stays visible during the print (`dev 38 µm (pred 7)`).

### Stage 1 — readiness model + bioprinting advisories — `[x]`
- `[x]` `PrintReadiness.evaluate(ReadinessContext) -> Readiness`, 19 checks in 8 groups.
- `[x]` **Unknown data never blocks**; `can_print()` blocks on exactly the pre-existing gates
  (XY+ZP, object, plate, well) plus the physically-impossible ink/bore case.
- `[x]` Single source of truth for the checklist, the derived facts and `_update_button_state` — a
  disabled `Print` now names every missing precondition in its tooltip.
- `[x]` New information surfaced: which of the three limits binds **and the headroom when none does**;
  calibration staleness; `StageCharacteristics.missing()`; dead-time provenance
  (`measured` / `phase_lag` / `unmeasured`); resolved absolute print/travel Z **including when a travel
  height was synthesised**; taught-vs-geometric well centre; object-fits-the-well (a check that did not
  exist); bead width; path length and stroke count; pickup breakdown; syringe fill.
- `[x]` Advisories: clog risk, wall shear vs 5 Pa, and the reference-fluid flow ceiling disclosed.

### Stage 2 — three zones — `[x]`
- `[x]` `QStackedWidget` + pill strip: **Setup → Run → Report**; `show_zone` / `current_zone`.
- `[x]` The two driving parameters **promoted onto the Setup zone** while still persisted with the
  profile via the new `register_external` (a widget has one parent, so it cannot also live in a
  section).
- `[x]` The well selector moved to Setup (choosing a well is a setup action); the Run zone is the watch
  surface (plan | camera + live panel). Auto-advance: Print → Run, terminal → Report.
- `[x]` **Attribute-name contract pinned by a test** — the partial-page suites never build the UI, so a
  rename would break them silently.

### Stage 3 — live in-print information — `[x]`
- `[x]` `vel_sample` gains `v_meas_mm_s` (already computed and thrown away) and `deposited_uL`, and a new
  `on_vel_sample` callback — all on the **existing every-5th-tick branch**, so the ~25 Hz control loop
  gains no per-tick work.
- `[x]` `_PrintBridge.vel_sample` → the slot only **stores**; rendering happens on the existing 10 Hz
  timer, so a slow paint cannot back-pressure the control loop.
- `[x]` Panel shows progress + ETA, live deviation vs the operator's own element with a verdict, running
  max/p95, dispensed-vs-planned µL, section n/N, and measured-vs-commanded speed.

### Stage 4 — post-print report — `[x]`
- `[x]` `PrintLogReader.read_log` — streaming, tolerant of a truncated final line and a log still being
  written (counts `n_bad_lines`, never raises).
- `[x]` `to_recorder_samples` → the exact `list[dict]` the four widgets consume, in **mm**.
- `[x]` `section_stats` / `restart_decomposition` / `volume_reconciliation` / `problems` / `comm_health`.
- `[x]` Report panel: verdict badge, **predicted vs actual with the ratio**, the four reused widgets
  (path coloured by error, playback scrubber, error-vs-time, statistics), per-section table annotated
  with the plan's own `reason`, the restart-vs-mid-section decomposition, fluidics reconciliation,
  machine health, problems with the traceback.
- `[x]` Self-contained HTML (figures as base64 PNGs via `QWidget.grab()`) + CSV.
- `[x]` The logger now records the path points so a **saved** log can be scored later; older logs open
  with no deviation scoring and say why.

### Stage 5 — closed-loop default + NEVER pause on corners — `[x]`

Operator decision, superseding the "auto-select" design: *"the closed loop velocity mode should now be
default. We should never pause on corners, just slow down. We always want the pump moving unless there
is a break in the section that is planned."*

- `[x]` **Closed-loop velocity is the default motion mode** (first in the combo, the persisted default,
  and the fallback in `_motion_mode()`). Open-loop reads **no** stage position during the path, so it
  has no prediction, no live deviation and no deviation in the report — every information surface v7.7
  added was inert in the old default.
- `[x]` **NEW `XYFeedPlan.build_continuous_plan`** — one section with **arc-length profiles** for
  lookahead and speed instead of per-section scalars, so a corner is slowed through rather than stopped
  on. Wired through new optional `lookahead_at=` on `VelocityControl.pursuit_step` and
  `lookahead_at=` / `speed_limit_at=` on `XYPathSimulator.follow_path` — **both default `None`, so the
  shared follower is byte-identical when unused** (126 follower/plan tests unchanged).
- `[x]` `PrintSettings.feed_plan_corner_policy` — **`"slow"` is the default**; `"stop"` keeps the
  hardware-validated v7.6 sectioned plan. Logged in `path_start` as `corner_policy` + `continuous`.
- `[x]` The pump therefore advances continuously: with no section split there is no dwell, and
  deposition tracks arc length, so it only pauses at a genuinely planned break (a travel/pen-up, which
  is already its own PRINT_PATH command) — pinned by a monotone-deposition test.

**Two modelling bugs found and fixed by simulating it — the first attempt was much WORSE, not better:**

1. **The arc formula does not describe a corner.** Sizing the carrot with `la = 0.8·√(2·R·δ)` at a
   *sharp vertex* asked for 0.092 mm at a 90° corner and simulated **40 µm** against an 18.75 µm
   budget, because on a polyline vertex the cut is `≈ (la/2)·sin(θ/2)` — **linear** in lookahead, not
   quadratic. New `corner_budget_lookahead_mm(θ, δ) = 2δ/sin(θ/2)` asks for 0.053 mm instead; the plan
   now takes the **min of both models** per vertex. Square went 58.6 → 7.6 µm, Star 54.9 → 7.5,
   Zigzag 62.6 → 7.7, Comb 58.0 → 7.6.
2. **Gating slowdown knots on turn angle ignored smooth curvature.** A 5 mm circle sampled at 0.5 mm
   turns only ~11°/vertex — below any sane threshold — so circles ran at full lookahead and simulated
   **48.7 µm**. Knots are now created wherever *either* model binds, with no angle gate. Circle 5 mm
   48.7 → 5.3 µm, Circle 10 mm 25.4 → 3.1 µm.

Also added **lead compensation** (`LEAD_LAG_MARGIN`): the slowdown is commanded `v·(dead+τ+loop/2)·1.25`
**before** the corner, on the approach side only, because the stage's response trails the command — the
same reason the v7.6 end taper had to be lag-aware.

**One thing "never pause" cannot do: a near-180° reversal still stops** (`REVERSAL_STOP_DEG = 150°`).
A cusp cannot be traversed by pure pursuit at any speed — a carrot placed forward in *arc length* sits
*behind* the stage in *space*. Simulated without a stop, Line-Reversal ran to the wall cap (855 s) with
no progress. This is geometry, not policy, and the plan says so in its notes.

**Measured trade (simulation, ME3B V1 characteristics, 3 mm/s, 30 µm element):**

| shape | stop p95 / time | slow p95 / time |
|---|---|---|
| Square 5 | 6.9 µm / 9.6 s | 7.6 µm / 14.4 s |
| Star 10 | 7.0 / 18.3 | 7.7 / 34.0 |
| Star 5 | 8.1 / 12.3 | 7.5 / 28.2 |
| Circle 10 | 3.1 / 19.0 | 3.1 / 18.7 |
| Circle 5 | 5.3 / 14.0 | 5.3 / 13.3 |
| Zigzag 5 | 8.0 / 19.9 | 7.7 / 36.4 |
| Comb 5 | 8.0 / 27.6 | 7.6 / 42.0 |
| **pass** | **10/10** | **10/10** |

⇒ **Never-stopping holds the element just as well (within ~1 µm) but costs ~1.5–1.9× the time on
corner-heavy geometry**, because crawling through a corner at ~0.3 mm/s over a ~1 mm ramp costs more
wall time than a 1.3 s stop. On smooth shapes it is slightly *faster*. The reason to prefer it is print
quality — deposition is never interrupted — not speed.

## Testing Notes

**496 tests green**, comprising the 116 new v7.7 tests plus the v7.6 / v7.5.x quick-print, print-path,
feed-plan, abort, logging, settings-popout, seam and context suites.

Validated against the **real** hardware log `logs/prints/print_20260729_121416_*.jsonl`:
- the reader parses it with 0 bad lines and reconstructs the section offsets (0 → 25.78 mm);
- its `restart_decomposition` returns **near-boundary p95 31.5 µm vs mid-section 19.8 µm**, independently
  reproducing the 31.7 / 17.8 computed by hand during the v7.6 hardware session;
- the report scores it **p95 27.5 / rms 11.7 / max 35.9 µm, verdict PASS**, and its predicted-vs-actual
  ratio is **3.76** against the 3.75 computed by hand;
- volume reconciliation returns the planned 3.8373 µL (the harness's own figure), ZP link 477/477 acked,
  and time estimate 21.2 s predicted vs 21.7 s actual (+2 %).

Offscreen smokes cover: the three-zone page builds and switches; every contract attribute survives;
the promoted parameters remain in `collect()`; the readiness checklist renders and the Print tooltip
names all four missing preconditions; the live panel renders from synthetic telemetry and tolerates junk;
HTML export embeds figures (83 kB vs 5.8 kB) with no external URLs.

## Issues & Decisions

**Three real bugs found and fixed while verifying, none of them in new code:**

1. **`_reserve_warning` read `self._hw_config` without a getattr guard**, breaking the `__new__`-partial
   test pattern the whole suite relies on (from the uncommitted v7.6 work).
2. **`_needle_cross_section_mm2` was a class-level *alias* of `_needle_orifice_area_mm2`** — the same
   function object, so overriding one name (which every partial-page test does) silently left the other
   pointing at the real implementation, and **the flow-ceiling warning, which read the other name, had
   quietly stopped firing**. Converted to a delegating method, and `_append_limit_warnings` now uses the
   same accessor as its three siblings, so the warning threshold and the commanded flow can never
   disagree about which area the bead is based on.
3. **My own reader indexed the full-path arc length with a section-local `s_mm`.** A feed-plan
   `vel_sample` records section-local `s_mm` (with `tot_mm` = that section's length), so every section
   after the first would have been projected onto the wrong part of the ideal path. Fixed by
   reconstructing global arc length from each `plan_section`'s `base_s_mm` — which also works on logs
   already on disk. The same mistake was then caught twice more in the live panel (a progress
   percentage and a planned volume computed from `tot_mm`), both corrected.

**Judgement calls**
- `can_print()` blocks on **only** the pre-existing gates plus the impossible-particle case, so the
  readiness model cannot start refusing prints that previously ran. Everything else is a visible
  warning.
- The **full run's syringe budget** is still verified at Print (it needs the settings and the cleanup
  context); the readiness panel shows fill / capacity / pickup breakdown rather than approximating a
  budget it cannot compute cheaply.
- The **live panel only covers the feed-plan executor.** The legacy velocity path still logs
  `vel_sample` without the two new fields and does not call `on_vel_sample`; open-loop reads no position
  at all, and the report says so explicitly rather than drawing a flat zero-error trace.
- `PREDICTION_HEADROOM` (1.6 in `XYFeedPlan`, 2.0 in `sketch_printability_dialog`) is **still not
  reconciled** — the report now measures the ratio every run so the decision can be evidence-based.
  Outstanding from v7.6.

## Needs hardware verification (ME3B V1)

Stages 0–4 are display/evaluation only and were checked offscreen and against a real log. **Stage 5
changes how the machine moves and has only been verified in simulation**, so it is the priority:

1. **A supervised continuous print.** Confirm the corner slowdown is smooth (no dither entering or
   leaving a corner — the lead compensation is the term most likely to need tuning against the real
   stage), that p95 still holds the element, and the wall time against the ~1.5–1.9× simulated cost.
2. **With real ink** — the whole reason to prefer continuous motion is uninterrupted deposition, so
   compare corner appearance against a `"stop"`-policy print of the same object. This is the one claim
   simulation cannot make.
3. A reversal geometry (the sketch back-trace / retrace features produce them): confirm the plan reports
   its unavoidable stop and the print completes.
4. The live panel through the print: ETA, deviation-vs-element colouring, dispensed µL, section n/N;
   the camera still running; abort mid-print responsive.
5. The Report zone opening on the terminal state, its ratio matching a hand check, and HTML/CSV opening
   outside the app.

If the corner slowdown misbehaves on hardware, `feed_plan_corner_policy = "stop"` restores the
hardware-validated v7.6 behaviour without a code change.
