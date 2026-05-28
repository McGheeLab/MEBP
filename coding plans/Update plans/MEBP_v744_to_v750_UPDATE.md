# MEBP v7.4.4 → v7.5.0 Update Plan

## Objective

Major revision of the Print Setup page from a 4-tab structure to a
linear 4-step wizard with a persistent Print Objects side panel,
on-demand polished validation, and a single Print Session JSON for
save/load. Backend data model is rationalized in lockstep
(deduplicated service toggles, single home for `InkSwapStrategy`,
parametric trajectories regenerated on load instead of persisted,
clearer hardware vs. per-print config boundaries).

Reference: `/Users/alexmcghee/.claude/plans/the-print-setup-page-merry-kernighan.md`

## Files Modified

### Backend (Phase A — additive, low-risk)
- `SupportClasses/WellSetup.py` — role-tightened behavior serialization
- `SupportClasses/PrintPlanOfAction.py` — rename `max_ink_volume_uL` →
  `pump_volume_overrides_uL` with alias property; add `@property` shims
  for `use_waste/wash/buffer` reading from `ink_swap`
- `SupportClasses/GeometryEngine.py` — add `PrintObject.source`
  discriminator (`"parametric"` | `"csv"`); only persist trajectory
  for csv-sourced objects

### Backend (Phase B — Print Session backend)
- `SupportClasses/PrintSessionManager.py` (new) — `PrintSession`
  dataclass + save/load/list/duplicate/delete
- `SupportClasses/PrintFileManager.py` — bump `SCHEMA_VERSION` 7.2.3 →
  7.5.0; extend `migrate_print_file()` with `< 7.5.0` branch; one-time
  `.bak-v7.2.3` snapshot

### UI (Phase C — wizard package)
- `gui/pages/print_setup/__init__.py` (new) — exports
  `PrintSetupPage`
- `gui/pages/print_setup/page.py` (new) — orchestrator, replaces
  `gui/pages/print_setup.py`
- `gui/pages/print_setup/stepper.py` (new) — `WizardStepper` top
  breadcrumb
- `gui/pages/print_setup/step_workspace.py` (new)
- `gui/pages/print_setup/step_objects.py` (new)
- `gui/pages/print_setup/step_wells.py` (new)
- `gui/pages/print_setup/step_plan.py` (new)
- `gui/pages/print_setup/side_panel_objects.py` (new)
- `gui/pages/print_setup/validation_panel.py` (new) +
  `ValidationIssue` dataclass
- `gui/pages/print_setup/session.py` (new)
- `gui/pages/print_setup/object_designer_dialog.py` (new) — reuses
  `PrintObjectsEditor`
- `gui/pages/print_setup/models.py` (new) — `PrintObjectsModel`,
  `WellAssignmentModel`
- `gui/pages/print_setup.py` — **deleted** (orchestrator + Finalize
  body migrate into the package above)
- `gui/pages/print_workspace.py` — **mostly deleted** (read-only HW
  summary helper migrates to `step_workspace.py`)
- `gui/pages/print_objects.py` — refactored: extract
  `PrintObjectsEditor` (full surface) + `PrintObjectsBrowser`
  (compact); both bind to one `PrintObjectsModel`
- `gui/pages/print_well_setup.py` — UI rewrite of 4-click flow →
  single-click + inspector; `_WellSetupModel`/`_generate_plan`/
  `validate()` preserved verbatim
- `gui/pages/printing_mode.py` — light update to import the new
  `PrintSetupPage`

### Cleanup (Phase D)
- Retire `PrintFileManager` user-facing CRUD; keep read-only legacy
  import for one release

## Implementation Steps

### Phase A — Backend additive (this commit)
- [x] A.1 — Role-tightened `WellAssignment.to_dict()` (`WellSetup.py`)
- [x] A.2 — Add `pump_volume_overrides_uL` alias for
  `max_ink_volume_uL` (`PrintPlanOfAction.py`)
- [x] A.3 — Add `@property` shims `use_waste/use_wash/use_buffer` →
  `ink_swap.*` (`PrintPlanOfAction.py`)
- [x] A.4 — Add `PrintObject.source` discriminator; skip trajectory
  persistence for parametric (`GeometryEngine.py`)
- [x] A.5 — Run existing test suite; confirm no regressions
  (4 pre-existing failures in `test_v726_print_execution.py` confirmed
  unrelated via stash diff — test helper `_make_mock_plate` has a
  pre-existing tuple-shape bug)

### Phase B — Print Session backend (next commit)
- [x] B.1 — New `SupportClasses/PrintSessionManager.py` +
  `PrintSession` dataclass
- [x] B.2 — Bump `PrintFileManager.SCHEMA_VERSION` 7.2.3 → 7.5.0;
  extend `migrate_print_file()` + `write_migration_backup` helper
- [x] B.3 — Trajectory regeneration on load via
  `regenerate_parametric_trajectories()` inside `PrintSessionManager`
- [x] B.4 — `tests/test_v75_print_session.py` — 9 tests passing
  (round-trip, list, load, dup, delete, byte-equal save→load→save)
- [x] B.5 — `tests/test_v75_migration.py` — 9 tests passing
  (schema bump, parametric strip, csv preserve, flag remap, alias
  rename, idempotent, v7.1 list→dict, `.bak-v7.2.3` once)
- [x] B.6 — `tests/test_v75_trajectory_regen.py` — 5 tests passing
  (regen parametric, leave csv alone, no-hw safe, no-objects safe,
  bad-object tolerance)

### Phase C — UI wizard (multi-commit)

**Strategy:** the legacy `gui/pages/print_setup.py` has been renamed
to `print_setup_legacy.py` and continues to own all of the tab
widgets, signal wiring, Finalize tab body, PrintManager, PrintQueue,
and helper methods. The new `WizardPrintSetupPage` orchestrator
**composes** the legacy class internally and **reparents** its four
tab widgets into wizard steps. This guarantees behavior parity in
v7.5.0 while adding the wizard shell, side panel, and validation
panel as pure UI overlays. Future commits can incrementally pull the
Finalize body into `step_plan.py` and bind `PrintObjectsModel` to the
Objects tab.

- [x] C.1 — Package skeleton: `__init__.py`, `validation.py`
  (`ValidationIssue` + `aggregate`), `wizard_step_base.py`
  (`WizardStepBase` contract), `models.py` (`PrintObjectsModel` +
  `WellAssignmentModel`)
- [x] C.2 — `stepper.py` (`WizardStepper` top breadcrumb with chip
  states pending / active / done / error)
- [x] C.3 — `step_workspace.py` (thin wrapper, exposes wizard
  contract over `WorkspaceTab`)
- [x] C.4 — `step_objects.py` (thin wrapper over `PrintObjectsTab`;
  exposes prints_changed signal + per-object validation)
- [x] C.5 — `step_wells.py` (thin wrapper over `WellSetupTab`;
  validation surfaces missing print collections + soft waste-well
  warning; single-click + inspector UX deferred to a follow-up)
- [x] C.6 — `step_plan.py` (hosts the legacy Finalize body via the
  composed `print_setup_legacy.PrintSetupPage`)
- [x] C.7 — `side_panel_objects.py` (compact `PrintObjectsBrowser`
  bound to `PrintObjectsModel`; drag mime payload `MIME_OBJECT_NAME`
  ready for Step 3 drop wiring; collapsible)
- [x] C.8 — `validation_panel.py` (slide-up `Card` with severity-
  coded issue rows + Fix → buttons; emits `jump_requested(step,
  target_id)` so the orchestrator switches step and focuses target)
- [x] C.9 — `page.py` (`WizardPrintSetupPage` orchestrator + Prev/Next
  nav + stepper jump + run_validation + side panel + validation panel)
- [x] C.10 — `__init__.py` exports both `LegacyPrintSetupPage` (the
  current default `PrintSetupPage`) and the new `WizardPrintSetupPage`
  side by side. Flip the default by rebinding `PrintSetupPage =
  WizardPrintSetupPage` once parity is verified end-to-end in the app.
- [x] C.10.55 — Visual polish pass. New
  `gui/pages/print_setup/chrome.py` with `STEP_DESCRIPTIONS` (title +
  one-line subtitle per step) and `build_step_frame()` — wraps each
  step body in a mantle outer band + base-colored content card with a
  proper step header (large title + subtitle + hairline divider).
  Stepper refined: numbered circles (28→32 px), "STEP N" eyebrow +
  title in a stacked label, state-driven palette (mauve active, green
  done, red error, subdued pending) with connecting hairlines.
  `_StepChip.sizeHint()`/`minimumSizeHint()` overridden to defer to
  the internal layout so the QPushButton doesn't collapse to its
  text-driven default (which had shrunk chips to ~36×17 px). Wizard
  outer background is now `mantle` so the inner card visually
  elevates. Nav bar gets a top border, transparent "← Back" secondary,
  mauve primary "Next →" / "Finish", and a centered "STEP N OF M"
  indicator.

- [x] C.10.4 — Drop the Workspace wizard step entirely. The wizard
  is now three steps (Print Objects → Wells & Roles → Plan & Run).
  `WorkspaceTab` stays alive as a hidden child of the wizard so its
  `workspace_changed` signal continues to drive the downstream
  legacy wiring (Print Objects + Well Setup listen for plate-format
  updates), but it no longer occupies a step in the breadcrumb.
  `STEP_TITLES` and step `step_index` values updated accordingly.

- [x] C.10.45 — Fix "floating embedded window" bug. `QTabWidget.removeTab`
  leaves the removed widget parentless while keeping its prior
  visibility state, so a previously-current tab could briefly become
  a top-level visible window. Fix: hide every tab widget *before*
  calling `removeTab`. Additionally, hide the legacy `PrintSetupPage`
  itself and set `Qt.WA_DontShowOnScreen` so the composed-but-never-
  laid-out legacy widget can never blip on screen.

- [x] C.10.5 — Build `gui/pages/print_setup/context_panel.py`
  (`PrintSetupContextPanel`) — three-tab left context:
    1. **Hardware** — reuses `HardwareSummaryWidget` from
       `print_workspace.py`; relays `edit_hw_requested` →
       `navigate_to_page(0)`.
    2. **Print List** — bound to `PrintObjectsModel`; shows current
       objects + collections with counts; refreshes on
       `model.changed`.
    3. **Print Settings** — reparents the legacy `get_context_widget`
       output (Display Options) into a tab so users keep the same
       checkboxes.
  `WizardPrintSetupPage.get_context_widget()` now returns this panel;
  `set_hardware_config` forwards into the Hardware tab.
  Step 1's `WorkspaceTab._summary` is hidden (the widget remains
  alive for the bridge logic that emits `workspace_changed`) and the
  step body becomes a small "Workspace" welcome card pointing the
  user at the left panel.

- [x] C.11 — Flip default to `WizardPrintSetupPage` in
  `gui/pages/print_setup/__init__.py`. `LegacyPrintSetupPage` remains
  importable for debugging/fallback. Verified end-to-end through
  `PrintingModePage(controller, settings).setup_page` — type is
  `WizardPrintSetupPage`, full external contract preserved
  (`set_hardware_config`, `get_context_widget`, `on_status_update`,
  `set_xy_position_scale`, signals `workspace_updated`,
  `navigate_to_page`, `job_ready`).
- [ ] C.12 — Single-click + inspector UX in `step_wells.py`
  (currently still the 4-click role flow inherited from the legacy
  `WellSetupTab`).
- [ ] C.13 — Bind `PrintObjectsModel` bidirectionally to
  `PrintObjectsTab` so side-panel edits propagate live.
- [ ] C.14 — Drop wiring on `WellPlateView` (accept the
  `MIME_OBJECT_NAME` drag from the side panel onto a Print well).
- [ ] C.15 — Pull the Finalize body out of `print_setup_legacy.py`
  into `step_plan.py` proper.
- [ ] C.16 — Delete `print_setup_legacy.py` + `print_workspace.py`.

### Phase D — Cleanup
- [ ] D.1 — Retire `PrintFileManager` user-facing CRUD (read-only
  legacy import only)
- [ ] D.2 — Update existing tests for moved/renamed call sites

## Testing Notes

### Phase A
- `python -m pytest tests/` — all existing tests must pass.
- Manual: open the current 4-tab print setup, edit a well's role,
  save setup, reload — confirm only role-relevant behavior fields
  appear in the saved JSON.

### Phase B
- New tests: round-trip a session, migrate a v7.2.3 fixture, confirm
  `.bak-v7.2.3` written.
- Manual: open an existing `config/prints/*.json`, save as session,
  reopen — confirm trajectories are regenerated for parametric
  objects and preserved for csv-sourced.

### Phase C
- Launch app, navigate Printing → Print Setup, walk through all 4
  steps. Confirm:
  - Side panel browser updates live as objects are added in Step 2
  - Single-click well + role palette in Step 3 applies in one
    interaction
  - Drag-from-side-panel onto Print well binds the object
  - Validate produces a slide-up issue list; Fix → jumps to the
    target widget with a flash
  - Save → close → Load reconstructs every step's state
  - Send to Monitor delivers the same `PrintJob` shape as today
- Regression: ensure existing `config/prints/*.json` open without
  error (legacy import path).

## Issues & Decisions

(empty — update during implementation)
