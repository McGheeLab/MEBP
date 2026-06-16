# MEBP v7.5.x — Print Execution Logging (JSONL, Claude-debuggable)

## Objective

Two real-hardware bugs surfaced in Quick Print (circle stopped mid-print and
drove to XY 0,0; spiral moves grew "longer and longer" near each spiral's
end). Both are timing/desync failures that cannot be conclusively diagnosed
without knowing what the software commanded vs. what the stage actually did,
when. This update adds an always-on, machine-readable **execution log** for
every print (Quick Print discrete mode, standard Printing hybrid mode, and
trajectory playback) so that a future debugging session — human or Claude —
can reconstruct the full commanded-vs-actual timeline from a single file.

Companion analysis: `coding plans/PRINT_PIPELINE_AUDIT_V75x.md` — the full
adversarially-verified print-pipeline audit (35 findings, both root causes,
prioritized P0/P1/P2 fix plan). HW logs from this feature confirm/quantify
the remaining hardware-only magnitudes.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PrintExecutionLogger.py` | **NEW** — JSONL logger: per-run file in `logs/prints/`, manifest builder (`manifest_for_job`), clamp-checking `xy_cmd_fields`, background actual-position sampler (5 Hz, cached poller reads only — zero serial traffic), `lag_um` vs last commanded target |
| `SupportClasses/PrintManager.py` | Hooks: `PrintManager` auto-creates/closes the log (`_begin_exec_log`/`_end_exec_log`, `start()`/`resume_from_saved()`/`_execute_loop` finally); `_execute_command` logs xy/z/extrude events; `_execute_print_path` logs `path_start`/`path_segment` (target, seg len, sleep, pump vol incl. `vol_dropped`, `drift_s`)/`path_end`; `_wait_for_xy_settle` logs ok/timeout/abort with `final_err_um`; `abort()` logs `abort_requested`; `TrajectoryExecutor` (+`exec_logger` param) logs `traj_start`/`traj_wp` (sparse: every 25th or >0.25 s late)/`traj_end` + sampler targets; `DirectCommandExecutor` (+`exec_logger`) logs blocking `xy_cmd`/`xy_arrival`/`z_move`/`extrude`; `HybridPlanExecutor` (+`exec_logger`) logs `plan_step` and passes the logger to child executors |
| `gui/app.py` | `_on_monitor_start`: opens the log for hybrid + trajectory modes (`pm.exec_logger`), closes it in both threads' `finally`, records executor exceptions |
| `gui/pages/workflows/quick_print_workflow.py` | Status line shows the log filename while printing and at terminal states |
| `logs/prints/README.md` | **NEW** — full event-schema doc + how to triage the two known bug patterns from a log |
| `.gitignore` | Ignore `logs/prints/*.jsonl` (README stays tracked) |
| `tests/test_v75x_print_execution_logging.py` | **NEW** — 13 tests |

## Implementation Steps

- [x] `PrintExecutionLogger` module (lifecycle, JSONL, manifest, sampler, clamp fields, kill switch `MEBP_PRINT_LOG=0`)
- [x] Discrete-path hooks (`PrintManager`: lifecycle, commands, print-path segments, settle waits, abort, speed sets)
- [x] Trajectory-path hooks (`TrajectoryExecutor`: start/wp lateness/end, sampler targets)
- [x] Blocking-move hooks (`DirectCommandExecutor`: arrival results) + hybrid plan steps (`HybridPlanExecutor`)
- [x] Wire standard Printing mode launch paths in `gui/app.py` (hybrid + trajectory threads, error + finally close)
- [x] Quick Print page surfaces log filename
- [x] Schema README for offline analysis
- [x] Tests (13) green; compile-checked all touched files
- [ ] Real-hardware run: reproduce both bugs once with logging on; analyze the JSONL (`sample.lag_um`, `path_segment.drift_s`, `settle_wait` timeouts)

## Testing Notes

- `python -m unittest tests.test_v75x_print_execution_logging` — 13/13 pass.
- Regression: `tests.test_v726_print_execution`, `tests.test_hybrid_execution`,
  `tests.test_v75x_quick_print_workflow` — 29/33 pass; the 4 errors are
  **pre-existing** bugs in `test_v726_print_execution.TestPlanToCommands`'s
  own `_make_mock_plate` helper (`dict()` over 3-tuples always raises —
  broken as checked in, unrelated to this change).
- Hardware verification still required (last step above): logs land in
  `logs/prints/print_*.jsonl`; see `logs/prints/README.md` for the analysis
  recipe.

## Issues & Decisions

- **New logger vs. extending PrintRecorder**: the audit recommended
  considering PrintRecorder, but it (a) records nothing at all in discrete
  mode — the very mode both bugs occurred in, (b) has a confirmed µm-vs-mm
  unit bug in its TrajectoryExecutor sampling (actual XY recorded 1000×
  too large), and (c) buffers in memory (a freeze loses the evidence). The
  new logger is event-oriented, flushed per line, and frame-complete
  (manifest carries `zero_position` + safety limits). PrintRecorder remains
  untouched for the Results page.
- **Sampler uses cached positions only** — deliberately no extra serial
  traffic during prints (the XY serial path is exactly where the suspected
  contention lives). Sampling fidelity is bounded by the PositionPoller
  rate; that is sufficient for lag-vs-command diagnosis.
- **Per-segment events are unconditional** (no sampling) in
  `_execute_print_path`: the discrete path is the bug site; complete
  per-segment evidence outweighs file size (64-segment circle ≈ 70 lines).
  Trajectory waypoints ARE sampled (every 25th / late) since dt ≤ 50 ms.
- **Settle timeouts were previously invisible** (debug-level log, silent
  continue) — now always captured as `settle_wait{ok:false}` events; the
  audit flags this silent-continue as a prime desync suspect.
- Audit root causes (for the companion fix plan, not this update):
  BUG-1 = trailing `HOME_XY` preempting an unfinished open-loop PRINT_PATH;
  BUG-2 = `generate_spiral`'s constant-angle sampling (segment spacing grows
  ×17 toward the rim → commanded speed ramp exhausts SMS headroom). Also
  confirmed: `set_speed_mm_s` int-floor quantization (0.5 mm/s steps),
  `_protocol_max_speed_um_s` never set on real hardware, per-waypoint pump
  G-code flooding in trajectory mode (broken `!=0.0` gate, dead
  `_prev_pumps`), inter-object jumps printed as extruding moves.
