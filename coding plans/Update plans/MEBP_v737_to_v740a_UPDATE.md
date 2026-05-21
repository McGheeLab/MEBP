# MEBP v7.3.7 → v7.4.0-a Update Plan

## Objective

Foundation work for the v7.4.0 UX-focused release. Fix the calibration auto-save bug, introduce a small reusable component library, add a page-transition primitive and global loading banner, and standardize hardcoded paddings through the existing DPI scaling system. Establishes the building blocks that v7.4.0-b (Hardware/Settings restructure) and v7.4.0-c (onboarding wizard) depend on.

Branch: `Version-7.4.0-a` (branched from `Version-7.3.8`).

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/scaling.py` | Add `sp(px)` helper returning `"{N}px"` string for use inside QSS templates |
| `gui/widgets/components.py` (new) | Reusable widgets: `Card`, `StatusBadge`, `SectionHeader`, `FormRow`, `WizardStep`, `LoadingBanner` |
| `gui/widgets/page_transition.py` (new) | `fade_swap(stack, new_index, ms)` helper using `QPropertyAnimation` + `QGraphicsOpacityEffect`; skips fade for camera-bearing pages |
| `gui/pages/calibration.py` | Auto-save calibration on every mutation via debounced `_emit_calibration_data_changed` |
| `gui/app.py` | Mount `LoadingBanner` above content stack; route `_navigate_to` through `fade_swap`; expose `show_loading()` / `hide_loading()` |
| `gui/pages/*.py`, `gui/widgets/*.py` | Replace hardcoded `padding:`/`border-radius:` values with `{sp(N)}` (one pass) |

## Implementation Steps

- [x] Add `sp(px)` helper to `gui/scaling.py`
- [x] Create `gui/widgets/components.py`:
  - [x] `Card(QFrame)` — titled container, optional collapsible
  - [x] `StatusBadge(QLabel)` — ok/warn/err/info/pending variants
  - [x] `SectionHeader(QLabel)` — wraps `build_page_header_style(scale)`
  - [x] `FormRow(QWidget)` — label + field + help-text slot
  - [x] `WizardStep(QFrame)` — header + body + prev/next/finish + `completed` signal
  - [x] `LoadingBanner(QFrame)` — braille spinner + message + `show_for()`/`hide()`
- [x] Create `gui/widgets/page_transition.py` with `fade_swap()`
- [x] Fix calibration auto-save:
  - [x] Add `_autosave_timer` `QTimer` (500 ms, single-shot)
  - [x] Call `self._autosave_timer.start()` inside `_emit_calibration_data_changed`
  - [x] Connect timer timeout to `_save_calibration`
- [x] Wire into shell:
  - [x] Import `LoadingBanner` + `fade_swap` in `gui/app.py`
  - [x] Mount `LoadingBanner` above the content stack
  - [x] Expose `show_loading()` / `hide_loading()` on `MainWindow`
  - [x] Route `_navigate_to` through `fade_swap` (skip if destination contains a camera widget)
- [x] Hardcoded-padding sweep on primary-workflow pages:
  - [x] `gui/pages/jog_control.py`
  - [x] `gui/pages/calibration.py`
  - [x] `gui/pages/hardware_setup.py`
  - [x] `gui/pages/print_setup.py`
  - [x] `gui/pages/print_workspace.py`
  - [x] `gui/pages/print_well_setup.py`
  - [x] `gui/pages/helper_functions.py`
  - [x] `gui/pages/settings_page.py`
  - Deferred to v7.4.0-b (secondary / monitor pages, ~94 remaining hits):
    `print_objects`, `print_results`, `pp_execution`, `print_monitor`,
    `pp_operation_queue`, `pp_target_selection`, `pp_operation_setup`,
    `well_preview`
- [x] Bump version string in `gui/app.py` (`v7.3.3` → `v7.4.0-a`)
- [x] Run test suite: 334 tests, 4 failures + 9 errors. **All pre-existing on
  Version-7.3.8 baseline** — confirmed by re-running `test_simple_print_plan`
  after `git stash`: identical `ValueError: dictionary update sequence element
  #0 has length 3; 2 is required` in `tests/test_v726_print_execution.py:150`.
  No v7.4.0-a regression.
- [x] Architecture doc + README archive per CLAUDE.md checklist

## Testing Notes

- **Calibration persistence:** Set Safe Z → kill app → restart → label restored green without clicking Save.
- **Debouncing:** Run mosaic auto-cal, check `settings.json` mtime — single write at end, not per-signal.
- **Transitions:** Page-to-page navigation shows ~120 ms fade. No flicker on Calibration page (camera widget present → fade skipped).
- **Component reuse:** Instantiate each component in a throwaway test page; verify renders correctly at 1.0x, 1.5x, 2.0x scale.
- **Padding sweep:** Open all 7 pages at 4K resolution; no padding outliers vs. 1080p baseline.
- **Loading banner:** Programmatically call `main_window.show_loading("Testing...")` then `hide_loading()`; banner appears/disappears smoothly.

## Issues & Decisions

- **Calibration "wasn't persisted" framing was wrong.** The Plan agent correctly noted that `_save_calibration` exists and works, but auto-save only triggers in one spot (line 3708, post-mosaic). The 5 other `_emit_calibration_data_changed` call sites (lines 2284, 3025, 3630, 3731, 3782, 4453) only fire signals, not save. Fix is a 500 ms debounced timer hook inside the emit method — much smaller than a persistence rewrite.
- **QSS opacity effect can break camera widgets.** `QGraphicsOpacityEffect` on a page with raw-QPainter camera streams can cause black frames during fade. Mitigation: `fade_swap` checks `findChildren(CameraWidget)` and skips animation if any are present.
- **`sp(px)` instead of inline `f"{s(4)}px"`.** Reduces visual noise in QSS template strings and standardizes the pattern. Adds 5 lines to `scaling.py`; everything else stays.
