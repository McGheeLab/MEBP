# MEBP v7.5.x — Quick Print: pre-print position confirmation + no return-to-origin

## Objective

Two operator-requested changes to the **Quick Print** workflow:

1. **Confirm the needle / XY position before the print starts.** Before any
   extrusion, the stage travels (needle **retracted** to the safe travel Z) to
   the print-start point, then the operator must confirm — watching the live
   microscope feed — that the needle is in the correct location. Only then does
   the needle descend and printing begin.

2. **CRITICAL — do not drive to XY 0,0 at the end.** The print job must finish by
   retracting the needle out of the well to the travel Z and **stopping where it
   is**, instead of returning the stage to the zero reference (origin).

## Root cause / current behavior

- **(#2)** `build_well_plate_job` (`SupportClasses/PrintManager.py`) always
  appended a trailing `TRAVEL_UP` **then `HOME_XY`** ("Return home"). The
  `HOME_XY` discrete handler does `move_xy_absolute(0, 0, from_zero_ref=True)` —
  i.e. it drives XY back to the zero reference (0,0) after every print, including
  Quick Print.
- **(#1)** Quick Print's only pre-print gate was a yes/no text dialog ("Move the
  needle to well X and print …?") — it did not physically position the needle or
  let the operator verify the location before extrusion.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PrintManager.py` | `build_well_plate_job` gains `return_home: bool = True`. When `False`, the trailing `HOME_XY` is omitted (the final `TRAVEL_UP` retract is kept). Default `True` = byte-identical legacy plan, so Print Setup / `PrintPlanOfAction` callers are unchanged. |
| `gui/pages/workflows/quick_print_workflow.py` | New `_preposition_for_print(start_zref_mm, travel_z)` → `StageController.safe_travel_to(..., target_z_mm=None)` (raise→XY, **never lowers**). `_on_print` restructured to: build settings → pre-position to the print-start point (retracted) → confirm-position dialog (verify on the microscope) → run job with `return_home=False`. |
| `tests/test_v75x_quick_print_position_confirm.py` | **New** — 12 tests covering the `return_home` flag and the pre-position/confirm flow. |

## Implementation Steps

- [x] Add `return_home` param to `build_well_plate_job`; gate the trailing
  `HOME_XY` on it; document in the docstring.
- [x] Add `QuickPrintWorkflowPage._preposition_for_print` (zero-ref mm → absolute
  µm; `safe_travel_to` with `target_z_mm=None`; returns `True`/`False`; swallows
  errors → `False`).
- [x] Restructure `_on_print`: build settings up-front, compute the print-start
  point (`well center + first path point`), pre-position (buttons disabled +
  status during the blocking move), confirm-position dialog (annotates a warning
  if the move timed out), then build/run the job with `return_home=False`.
- [x] New test suite; run it + the related Quick Print / seam / Z-retract /
  print-execution suites.
- [x] Update plan document (this file).
- [ ] Add a row to the **Existing Update Plans** table in `CLAUDE.md`.

## Testing Notes

- `tests/test_v75x_quick_print_position_confirm.py` (9 tests, all green):
  - `return_home=True` (default) ends `… TRAVEL_UP, HOME_XY`; `return_home=False`
    has **no** `HOME_XY` and ends on the final `TRAVEL_UP`; the two plans differ
    by exactly that one trailing command.
  - `_preposition_for_print` calls `safe_travel_to` with the correct absolute µm
    (`zref_mm·1000 + zero`), the given `safe_z_mm`, and `target_z_mm=None`
    (never lowers); returns `False` on timeout and swallows exceptions → `False`.
  - `_on_print` (PrintManager + `QMessageBox.question` patched): confirm **Yes**
    positions first (`safe_travel_to`, `target_z_mm=None`) then loads/starts a job
    whose commands contain **no `HOME_XY`** and end on `TRAVEL_UP`; confirm **No**
    still positions (so the operator can look) but does **not** run; an empty
    geometry guard moves the stage **not at all**.
- Regression: `tests.test_v75x_quick_print_workflow` (15),
  `tests.test_v75x_multi_object_print_seam` (11),
  `tests.test_v75x_quick_print_trajectory_view` (13),
  `tests.test_v75x_z_retract_before_xy_travel`,
  `tests.test_v75x_print_execution_logging`,
  `tests.test_v75x_print_z_reference_vector`, `tests.test_hybrid_execution` —
  all green.
- Pre-existing (NOT caused by this change): `tests.test_v726_print_execution::
  TestPlanToCommands` has 4 errors in its own `_make_mock_plate` helper
  (`dict()` on a list of 3-tuples). Verified identical failures against
  `HEAD:SupportClasses/PrintManager.py`.
- **Needs real-HW verification on ME3B V1**: (1) Quick Print positions the needle
  over the well print-start retracted, the confirm dialog appears, and printing
  begins only after confirming; (2) at the end the needle lifts to the travel Z
  and the stage does **not** travel back toward (0,0).

## Issues & Decisions

- **Pre-position target = print-start point** (well center + first path point of
  the first object), not the well center — that is exactly where the needle
  descends, so it is the most meaningful thing to verify on the microscope.
- **Blocking move on the GUI thread** matches the established pattern
  (`CalibrationPage._safe_navigate_to` calls `safe_travel_to` directly from a
  button handler). Buttons are disabled and the status label is force-repainted
  (`self._status.repaint()`, not `processEvents()`, to avoid reentrant clicks)
  before the move.
- **Never lowers**: pre-position uses `safe_travel_to(..., target_z_mm=None)`, so
  it only raises Z + travels XY; the needle stays retracted until the operator
  confirms (then the print job's `MOVE_Z` lowers it). Honors the polarity-safe
  retract + insert-clearance floor already in `safe_travel_to`.
- **Timeout handling**: a `safe_travel_to` that returns `False` (XY/Z arrival
  timeout) does not auto-abort; the single confirm dialog gains an inline ⚠
  warning so the operator decides with the camera in view.
- **`return_home` defaults `True`** so Print Setup, `PrintPlanOfAction`, and all
  other `build_well_plate_job` callers keep the existing return-home behavior;
  only Quick Print opts out. Keeps the final `TRAVEL_UP` so the needle is always
  retracted out of the well even when not homing.
- Complements `MEBP_v75x_Z_RETRACT_BEFORE_XY_TRAVEL.md` (which made the
  end-of-print `HOME_XY` itself retract first) — this change additionally lets
  Quick Print skip that origin move entirely.
