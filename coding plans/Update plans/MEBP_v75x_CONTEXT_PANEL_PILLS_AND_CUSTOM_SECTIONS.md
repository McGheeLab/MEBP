# MEBP v7.5.x — Left context panel: pill view-picker + composable Custom sections (+ fix vanishing jog)

## Objective

Two operator requests plus a latent bug they surfaced ("when in a workflow the
jog panel sometimes just disappears from the left context"):

1. **Pill buttons at the top of the left context** to pick which *view* shows —
   a built-in **Jog** view (the existing jog panel) and a **Custom** view.
2. **A Custom panel the operator composes from *sections* (cards)** via a `＋`
   button — live camera viewer, syringe overview, X/Y/Z/P1/P2/P3 location
   read-outs, jog controls, hardware info. The layout **persists** and is
   **shared everywhere** the left context appears.
3. **Fix the vanishing jog panel** (root cause below).

**Confirmed with the operator (AskUserQuestion):**
- Pill model = **view picker** (`(Jog) (Custom) [＋]`).
- Scope = **one shared global layout**, everywhere the left box already appears
  (Jog Control, Calibration, all jog-capable Workflow tiles). Pages that
  intentionally have no left box (Print Builder, Full Print inner, Fluorescence,
  Common Print Settings, the workflow picker, Settings-with-only-quick-safety
  keep their existing behaviour).

## Root cause of the vanishing jog panel

The left box's **visibility** and its **content** were decided by two code paths
that didn't both run on every transition:
- Visibility: `_navigate_to` via `UIFunctions.toggleLeftBox` — an animation
  *toggle* keyed on `box.isVisible()`, gated on `page.get_context_widget() is not None`.
- Content for mode/workflow pages: `_update_mode_context` (from
  `_on_mode_sub_page_changed` on `sub_page_changed`) — set the stack index,
  never touched visibility.
- `WorkflowsModePage` is not a `ModePage`, so `_navigate_to` pointed the context
  stack at a *placeholder*; the jog panel was only mounted later by a
  `sub_page_changed` fire. Order-dependent → box shown-but-empty or
  content-mounted-but-hidden.

Fix: a **single global `ContextPanelHost`** replaces the per-page left-context
`QStackedWidget`, and **one method `_refresh_left_context()`** (called from BOTH
`_navigate_to` and `_on_mode_sub_page_changed`) mounts the page's context widget
AND sets visibility **explicitly** (no toggle). Content and visibility can no
longer desync.

## Files Modified

**New**
- `SupportClasses/ContextPanelLayoutStore.py` — single shared, persisted layout
  (`config/context_panel_layout.json`); atomic tmp+`os.replace`, listeners, env
  override `MEBP_CONTEXT_PANEL_DIR`, `known_types` load-time pruning; CRUD +
  `move_section` + `set_collapsed(notify=False)`. Qt-free (headless-testable).
- `gui/widgets/context_sections.py` — `SectionContext`, `SECTION_REGISTRY` +
  `register_section`/`catalog`/`build_section`/`known_types`, and the five
  section widgets: `CameraSection` (reuses `CameraFeedView`; safe start on show,
  never stops the shared feed on hide), `SyringeSection` (reuses `PumpRack`,
  fill-dict replicated from `jog_control._refresh_pump_panel`), a new light
  `PositionReadoutCard` (reuses `PositionBar` + `_update_position_displays`
  read logic; Z→user frame, pump→µL fill), `JogSection` (reuses
  `StandardJogContextPanel`), `HardwareInfoSection`.
- `gui/widgets/custom_context_panel.py` — `CustomContextPanel` + `_SectionCard`
  (per-card `▾/▸ collapse ▲ ▼ ✕`). Store-listener → re-entrancy-guarded
  `_rebuild`; `＋ Add section` `QMenu` from `catalog()`; per-section try/except
  → error card; empty-state placeholder; forwards `on_status_update`/
  `on_motion_tick` to sections that implement them.
- `gui/widgets/context_panel_host.py` — `SegmentedPillBar` (exclusive
  `QButtonGroup`) + `ContextPanelHost`: native (Jog) slot = reused
  `QScrollArea` wrappers keyed by widget id; shared single `CustomContextPanel`;
  `set_native_widget/set_native_available/set_native_label`, requested-vs-
  effective view (`set_active_view`/`active_view`/`requested_view`,
  `view_changed`), `[＋]` → switch to Custom + open add-menu; forwards ticks to
  the Custom view ONLY (native jog panel is ticked by its owning page).
- `tests/test_v75x_context_panel.py` — 20 tests.

**Modified**
- `gui/app.py` — replaced the left `_context_stack` with a `_context_content`
  holder + the single `_context_host` (built in `_create_pages`); new
  `_refresh_left_context()` / `_context_title_for()` / `_toggle_left_context()`
  / `_on_context_view_changed()`; `_navigate_to` and `_on_mode_sub_page_changed`
  now both call `_refresh_left_context()`; **deleted `_update_mode_context`**;
  dropped the per-page left-context registration in the pages loop (right
  context unchanged); host added to `_propagate_hardware_config` fan-out and to
  the `_update_status`/`_motion_anim_tick` dispatch; close-x + top-bar toggle
  repointed to `_toggle_left_context`; `save_settings` persists
  `context_panel.active_view` (the *requested* pill) + `context_panel.collapsed`;
  restore in `_create_pages`. Imports: `ContextPanelLayoutStore`,
  `ContextPanelHost`, `SectionContext`, `known_types`.

## Implementation Steps

1. [x] `ContextPanelLayoutStore` (schema, atomic write, listeners, prune, CRUD).
2. [x] `context_sections` registry + five section widgets (+ `PositionReadoutCard`).
3. [x] `CustomContextPanel` (store-driven rebuild, `＋` menu, per-card controls,
   tick forwarding, error card, empty state).
4. [x] `ContextPanelHost` (pill bar, native slot reuse, requested/effective view,
   `view_changed`, tick forwarding to Custom).
5. [x] `gui/app.py` wiring (holder + host, `_refresh_left_context`,
   `_toggle_left_context`, delete `_update_mode_context`, config fan-out, ticks,
   persistence, repoint toggle callers).
6. [x] Tests + baseline regression run.
7. [x] Update plan doc + CLAUDE.md table row.

**Scope note:** `in_scope = (native context is not None)` — this matches the
approved "everywhere the left box already appears" exactly and made the
`full_print` carve-out / `WorkflowsModePage.current_workflow_id()` helper (from
the original plan) unnecessary: Full Print / Fluorescence / Common Print
Settings / the picker all return `None` native context and so keep no left box,
unchanged.

## Testing Notes

- `tests/test_v75x_context_panel.py` (20, offscreen): store round-trip
  (add/move/remove/collapse, unknown-type prune, listener-not-on-collapse);
  every registered section builds + ticks headless (fake controller + fake
  camera manager); camera section starts the shared cam on show and does NOT
  stop it on hide; panel add/remove/reorder tracks the store, tick forwarding,
  a builder that raises → error card (no crash); host native mount/reuse/clear,
  pill availability + fall-back, `view_changed`, native relabel; **and the
  bug-fix logic** — `_refresh_left_context` invoked unbound against a fake
  `self` shows the box with a native widget, hides it without, treats a
  `get_context_widget` exception as no-native (box hidden, no crash), and the
  manual collapse/expand toggle. Also `requested_view` survives a
  restore-before-native-mount (startup clobber guard).
- Baseline green (121): `test_v75x_full_print_workflow`, `test_v731_jog_navigation`,
  `test_v75x_needle_location_quick_move`, `test_v75x_plate_z_autocal_per_well_focus`,
  `test_v75x_plate_location_manual_click_rim`.
- **Needs GUI verification on ME3B V1:** enter a workflow → jog panel shows via
  the Jog pill and no longer vanishes when switching tiles / to the picker and
  back; Custom pill → `＋` → add Live camera / Syringe / Positions cards → live
  updates; reorder/remove/collapse; restart → same cards, same active pill, same
  collapse, on Jog / Calibration / all jog workflows; Print Builder / Full Print
  inner / Settings unchanged.

## Issues & Decisions

- **Single global host** (not per-page instances): one managed widget + one
  explicit-visibility method is the definitive fix for the desync, and the
  shared `CustomContextPanel` keeps the camera feed a single subscriber.
- **Requested vs effective view:** persisting the raw displayed view would let
  an availability-driven auto-fallback (native "jog" temporarily absent → show
  Custom) clobber the operator's saved "jog" pill at startup (before the first
  page mounts a native widget). The host keeps `_requested_view` (persisted,
  changed only by the user/restore) distinct from `_active_view` (displayed);
  `view_changed` fires only on a requested change, never on auto-fallback.
- **Camera lifecycle:** the Custom camera section only *ensures* the shared cam
  is running while visible and never stops it on hide — a page that started it
  still owns the stop, so the two can't fight over the shared feed.
- **Full `MainWindow` boot is intractable headless** (camera/serial
  enumeration) — the app-level `_refresh_left_context` logic is tested unbound
  against a fake `self`, mirroring the repo's existing
  `TestApplyAndSaveAfterRestore` approach.
