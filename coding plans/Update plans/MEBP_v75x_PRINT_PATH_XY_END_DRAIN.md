# MEBP v7.5.x — drain the XY queue at the end of a print path (XY/ZP de-sync)

## Objective

After the ZP feedrate root-cause fix (`MEBP_v75x_ZP_Z_FEEDRATE_INHERITANCE_FIX.md`)
stopped the board drops, a new symptom surfaced: **the Z retracts (and the print
reports done) while the XY stage is still moving** — "there is still a backlog of
moves in the XY stage causing it to keep moving after the print is done." The Z
is now confirmed/synchronous; the XY is not, so they de-sync at the print end.

## Cause

`PrintManager._execute_print_path` streams the path **open-loop**: each segment
issues one `move_xy_absolute` → Prior `G x,y`, which returns its `R` (received)
ack **immediately** and moves **asynchronously**. The loop is paced only by
`time.sleep(_sleep_s)` (≈ the segment's transit time), NOT by arrival — and
short segments actually take longer than `seg_length / max_speed` (accel/decel
ramps), so the Prior accumulates a **backlog of queued moves**. The loop settles
XY only at the **start** (move-to-first-point); it never waits at the **end**. So
when the segment loop finishes, `_execute_command` returns, the plan advances to
`TRAVEL_UP` (Z retract) / the next object, and the Prior is **still draining its
queued moves** — the needle retracts out of sync while XY finishes the path.

## Fix

`SupportClasses/PrintManager.py` — at the end of `_execute_print_path`, after the
segment loop, **drain the XY queue**: `_wait_for_xy_settle(final_x, final_y,
timeout=30.0)` blocks until the stage physically reaches the final path point
before returning. So the print does not report done — and Z does not retract /
the next object's hop does not start — until XY is settled at the path end.

- A **mid-path** XY barrier was deliberately NOT added: stopping the stage every
  N segments would dwell the needle → over-extrusion blobs. The drain is only at
  the path boundary, where a brief settle is harmless.
- `_wait_for_xy_settle` uses a `cached=False` direct Prior query (works with the
  ZP poller suspended), honors the abort flag, and logs a `settle_wait` event
  (so a rare can't-reach-target shows as a timeout instead of a silent stall).
- This also tightens **multi-object** prints: each object's path now drains
  before the inter-object hop's Z-up + `MOVE_XY`, so the hop starts in sync.

## Testing Notes

- `tests/test_v75x_print_path_planner_barrier.py` — new
  `TestPrintPathDrainsXYAtEnd` (2): the path settles at the FINAL point after
  the loop (in addition to the move-to-start settle). Full file 13 OK.
- Print/quick-print/logging/trajectory/stress suites → 120 OK.
- **Real-HW (ME3B V1):** at the end of a print the stage should come to rest at
  the last path point and only THEN should Z retract — no "XY keeps creeping
  after the print is done." For a multi-object print, each object finishes
  (stage stopped) before the hop to the next.

## Issues & Decisions

- The remaining open-loop *lag during* the path (XY trailing the commands while
  printing) is cosmetic for shape fidelity as long as the pump and XY lag
  together (both are streamed with the same sleep pacing). If a future print
  needs tighter in-path tracking, the principled fix is accel-aware per-segment
  pacing — not a stop-and-go barrier. Out of scope here.
