# MEBP v7.4.x — Workflows Mode + Live Target Pickers (Spheroid Pick & Place v1)

## Objective

Replace the v7.3.3 Pick & Place mode with a new **Workflows** mode whose landing page is a picker of named workflows (Spheroid Pick & Place, Cell Targeting & Removal, Cell Labeling, Quick Print, Immuno). Build the **Spheroid Pick & Place** workflow as the first functional workflow on top of one new "tool" — `LiveTargetPicker`, a single shared live-microscope view with a Mode toggle (Pick / Place) that routes each click into one of two **paired** target lists (picks `P001…` green, places `D001…` mauve). Pick *i* is paired with place *i* by index — spheroid picked at `P001` is deposited at `D001`, `P002` → `D002`, … — and a connector line is drawn between each pair on both the camera overlay and the XY workspace. The targets are mapped to absolute stage µm with sub-pixel precision and then handed to the existing `PickPlaceExecutor` for execution as one operation per pair.

This is a topical update (not numerically sequential) because the v7.4.3 / v7.4.4 / v7.4.5 / v7.5.0 slots are already taken by parallel in-progress plans (jog cockpit, calibration revision, plate designer, print-setup wizard). Versioning is left to the user when this work is cut.

Three principles drive the redesign:

1. **Workflows are modal**, not parallel tabs. Commit to one workflow, work it, Back to the picker.
2. **Each workflow composes a curated set of tools.** Tools are reusable across workflows.
3. **No new transform math.** The pickers compose existing helpers (`CameraManager.pixel_to_stage_offset`, `StageController.get_xy_position`, `ObjectiveCalibrationStore.get_calibration`, `CameraFeedView._widget_to_image`) and add only widget composition + signal plumbing.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/workflows_mode.py` (NEW) | `WorkflowsModePage` — top-level mode container; QStackedWidget with picker at index 0, one page per workflow at 1..n; `sub_page_changed` signal; delegates `get_page_title()` / `get_sub_page_title()` / `get_context_widget()` / `on_status_update()` to the active workflow page; fans out `set_hardware_config` / `set_calibration_data` / `set_z_references` / `set_settings`. Not a `ModePage` subclass — workflows are modal. |
| `gui/pages/workflows/__init__.py` (NEW) | Package marker. |
| `gui/pages/workflows/workflow_picker.py` (NEW) | `WorkflowPickerPage` — 5 clickable tiles in a 3-column grid; `workflow_selected(workflow_id)` signal. `WORKFLOWS` tuple is the single source of truth for tile order + enablement. |
| `gui/pages/workflows/_stub_workflow.py` (NEW) | `StubWorkflowPage` — reusable "coming soon" page for the four not-yet-built workflows; emits `back_requested`. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` (NEW) | `SpheroidPickupWorkflowPage` — composes the workflow as a horizontal split: LEFT = `LiveTargetPicker` (camera + pick/place lists); RIGHT = vertical split of `WorkspaceTargetView` (top, XY workspace with target overlays + click-to-travel) and `XZSideView` (bottom, click Z-ref badges to move). Header / config row / Start-Abort row. Implements `get_context_widget()` (lazy `StandardJogContextPanel`, same lifecycle as Jog page), `get_page_title()` / `get_sub_page_title()`, `set_hardware_config`, `set_calibration_data(plate, well_positions, safe_z)`, `set_z_references(refs)`, `set_settings`, `on_status_update`. Click-to-travel handlers `_on_workspace_position_clicked` / `_on_workspace_fast_travel_requested` / `_on_go_to_z_requested` mirror the Jog page (safe-Z + zero-ref + `safe_travel_to` vs `move_xy_absolute` routing). `_refresh_target_overlays` converts the picker's stage-frame µm → zero-ref µm and pushes onto the workspace overlay. `_refresh_position_indicators` (5 Hz QTimer) pushes current XY/Z into the visualizations. Builds `OperationQueue` of `SPHEROID_PICKUP` ops + runs `PickPlaceExecutor.execute_queue()` on a daemon thread; `_ExecutorBridge` (`QObject` Qt signals) marshals executor callbacks back to the GUI thread. |
| `gui/widgets/live_target_picker.py` (NEW) | `LiveTargetPicker` — single composite tool: one camera feed + Mode toggle (Pick / Place) routing clicks into two **paired** multi-lists (picks `P001…`, places `D001…`). `_PickerCameraView` (a `TargetOverlayCameraView` subclass) adds a `right_clicked` signal and **overrides `_draw_targets`** to render picks green, places mauve, and a dashed connector between each paired pick/place. Public API: `picks()`, `places()`, `pairs()` (index-aligned), `is_balanced()` (≥1 pair + equal counts), `clear_picks()`, `clear_places()`. Signals `picks_changed` / `places_changed`. A pairing-status line warns when pick/place counts differ. Click pipeline: widget → `_widget_to_image` → `CameraManager.pixel_to_stage_offset` → + stage µm → emit. µm/px sourced from `ObjectiveCalibrationStore.get_calibration(camera_model, current_objective_name)` with `CameraManager.get_um_per_px` fallback. |
| `gui/widgets/workspace_target_view.py` (NEW) | `WorkspaceTargetView(JogWorkspaceView)` — subclass that paints pick (green) + place (mauve) target markers + dashed pick→place connector lines on top of the base view's plate / well / breadcrumb / needle layer. Targets pushed in as zero-ref µm tuples via `set_pick_targets` / `set_place_targets`. Reuses the base `_um_to_px(x, y)` so overlays share the wells' coordinate frame. |
| `gui/app.py` | Import `WorkflowsModePage` instead of `PickPlaceModePage`. Sidebar tile renamed "Pick & Place" → "Workflows" (icon `🧫`, obj_name `btn_workflows`). `pages[]` index 4 swapped. Title + context-title lists updated. `btn_map` key swapped. **New v7.4.x**: `WorkflowsModePage` added to the dynamic-context dispatch (placeholder at `_create_pages` + `sub_page_changed` → `_on_mode_sub_page_changed` wiring) so the left context panel swaps to the active workflow's `StandardJogContextPanel` when the user picks a workflow. `_push_cal_to_jog` extended to also push plate / well_positions / safe_z / Z-references / settings to the Workflows mode so its embedded XY workspace + XZ side view + jog context render identically to the Jog page. |
| `gui/pages/pick_place_mode.py` (DELETED) | Replaced by `workflows_mode.py`. |
| `gui/pages/pp_target_selection.py` (DELETED) | Replaced by the two pickers. Stitched-image / auto-scan flow is out of scope (see "Out of scope"). |
| `gui/pages/pp_operation_setup.py` (DELETED) | Replaced by the inline config row on `SpheroidPickupWorkflowPage`. |
| `gui/pages/pp_operation_queue.py` (DELETED) | Was redundant with operation_setup; no replacement. |
| `gui/pages/pp_execution.py` (DELETED) | Replaced by Start/Abort row + status label on `SpheroidPickupWorkflowPage`; threading + bridge pattern preserved as `_ExecutorBridge`. |
| `SupportClasses/PickAndPlaceManager.py` | UNCHANGED. Executor + dataclasses fully reused. |

## Implementation Steps

### Phase A — Scaffolding & nav swap

- [x] Create `gui/pages/workflows/__init__.py` and `_stub_workflow.py`.
- [x] Create `gui/pages/workflows_mode.py` with `WorkflowsModePage` (QStackedWidget; picker @ index 0; workflow pages @ 1..n; `get_page_title()` delegates to the active stack widget's tile title; `set_hardware_config` / `set_well_list` / `set_well_positions` fan out to all workflow sub-pages).
- [x] Update `gui/app.py:57` import (`PickPlaceModePage` → `WorkflowsModePage`).
- [x] Update `gui/app.py:310` menu item (`btn_pickplace`/"🔬"/"Pick & Place" → `btn_workflows`/"🧫"/"Workflows").
- [x] Update `gui/app.py:628` instantiation (`self._pick_place_mode` → `self._workflows_mode`).
- [x] Update `gui/app.py:641` pages list entry.
- [x] Update `gui/app.py:1458` `btn_map` key.
- [x] Update `gui/app.py:1486` titles list ("Pick & Place" → "Workflows").
- [x] Update `gui/app.py:1497` context_titles list ("Pick & Place" → "Workflows").
- [x] Delete `gui/pages/pick_place_mode.py`, `pp_target_selection.py`, `pp_operation_setup.py`, `pp_operation_queue.py`, `pp_execution.py`.
- [x] Verify no stale imports remain (`grep -r "from gui.pages.pp_\|PickPlaceModePage"` returns nothing).

### Phase B — Workflow picker

- [x] `WorkflowTile` dataclass + `WORKFLOWS` tuple (5 tiles; only `spheroid_pickup` enabled).
- [x] `_TileButton(QFrame)` clickable card with icon + title + description + hover style + "Coming soon" badge for disabled tiles.
- [x] `WorkflowPickerPage` lays tiles in a 3-column `QGridLayout`; emits `workflow_selected(workflow_id)`.
- [x] `WorkflowsModePage._on_workflow_selected` maps `workflow_id` → stack index via `_workflow_index` dict; `back_requested` from any workflow page returns to picker (index 0).

### Phase C — Live target picker tool

- [x] `_PickerCameraView(TargetOverlayCameraView)` extends the base's `eventFilter` to also emit `right_clicked(px_x, px_y)` for RightButton.
- [x] `LiveTargetPicker(QWidget)` — single composite tool:
  - one shared `_PickerCameraView` on the left of a `QSplitter`,
  - side panel on the right with: Mode toggle (Pick / Place radios), Pick targets section (list + Clear all / Remove selected), Place target section (coord label + Clear),
  - resolves microscope camera index via `hw_config.camera_for_role(CameraRole.MICROSCOPE)` (falls back to cam_idx=0),
  - reads `current_objective_name` + camera model (`camera_spec.model`) from `hw_config.cameras[cam_idx]` for the `ObjectiveCalibrationStore.get_calibration` lookup,
  - `QTimer(200ms)` polls `StageController.get_xy_position(cached=True)` → `view.set_stage_position(x_um, y_um)`,
  - click router: `view.clicked(px_x, px_y)` → `CameraManager.pixel_to_stage_offset` → + stage center µm → if mode == Pick, append; if mode == Place, replace,
  - right-click router: same pipeline → if mode == Pick, remove nearest pick within 250 µm; if mode == Place, clear,
  - µm/px label rendered in the header for visibility / debugging,
  - place target gets `target_id = "PLACE"`; setting `_view.set_selected_target("PLACE")` causes the overlay renderer to draw it mauve while picks (selected=True) render green — both visible at once on the single camera view,
  - public API: `picks()`, `place()`, `clear_picks()`, `clear_place()`, `set_hardware_config(hw_config)`,
  - signals: `picks_changed(list[PickPlaceTarget])`, `place_changed(PickPlaceTarget | None)`.

### Phase D — Spheroid Pick & Place workflow

- [x] `_ExecutorBridge(QObject)` with signals `op_started / op_completed / op_failed / progress / sub_step / finished` (cross-thread Qt signals = automatic main-thread dispatch).
- [x] `SpheroidPickupWorkflowPage`:
  - Back button row (emits `back_requested`).
  - Config row: `QDoubleSpinBox` diameter (default 200 µm, step 10), `QComboBox` bore (populated from `hw_config.pumps`), `QDoubleSpinBox` safety factor (default 1.5). Live volume label updates from `SpheroidPickupConfig.compute_volume_uL()`.
  - One shared `LiveTargetPicker` instance.
  - Start/Abort row + status `QLabel`.
- [x] `_on_start`:
  - validates `is_balanced()` (≥1 pair + equal pick/place counts),
  - builds one `PickPlaceOperation(SPHEROID_PICKUP, source=pick_i, dest=place_i, config=SpheroidPickupConfig(…))` per pair,
  - constructs `PickPlaceExecutor(controller, hw_config)`, wires callbacks → bridge signals,
  - spawns `threading.Thread(target=executor.execute_queue, daemon=True)`,
  - disables Start, enables Abort.
- [x] `_on_abort` sets `executor._abort_flag`; executor's per-iteration check drops out cleanly.
- [x] `_on_finished` resets buttons; status reflects `ok`.

### Phase E — Verification + project conventions

- [x] AST + import smoke tests pass for all new files + the modified `gui/app.py`.
- [x] Widget construction smoke test (without controller/camera) succeeds.
- [ ] Manual GUI smoke test under `python main.py --simulate` (operator-driven; see "Testing Notes").
- [x] This update plan exists at `coding plans/Update plans/MEBP_v74x_WORKFLOWS_MODE.md`.
- [ ] Add a row to the **Existing Update Plans** table in `CLAUDE.md` (deferred; user decides exact wording during version sign-off).
- [ ] Architecture doc delta (deferred until version number is agreed and the other in-flight v7.4.x plans land).
- [ ] README archive + bump (deferred to version-completion checklist).
- [ ] Commit + push (deferred; user manages branch cut).

## Testing Notes

**AST + import (run during implementation):**

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from PySide6.QtWidgets import QApplication; QApplication([])
import gui.app  # should not raise
from gui.pages.workflows_mode import WorkflowsModePage
from gui.widgets.live_target_picker import PickTargetPicker, PlaceTargetPicker
print('imports OK')
"
```

**Manual smoke test (operator):**

1. `python main.py --simulate` — should boot without errors. Sidebar shows "Workflows" at index 4 with the 🧫 icon.
2. Click **Workflows** — landing page shows 5 tiles in a 3-column grid; only Spheroid Pick & Place is fully enabled, the other four show "Coming soon" badges.
3. Click any **stub** tile → "{title} — coming soon" page with Back button; Back returns to the picker.
4. Click **Spheroid Pick & Place** → workflow page opens with the config row, two pickers, and Start/Abort row.
5. The single live picker view shows the simulated microscope feed with a crosshair.
6. With Mode = **Pick**, click in the view — a green-outlined target appears at the click; the side-panel pick list adds a row like `P001   (   123.4,    456.7) µm`. Right-click near the target removes it.
7. Switch Mode to **Place**, click — a mauve target appears on the same view alongside the pick markers; the Place coord label updates. Click again to replace; Clear or right-click to empty.
8. Add equal numbers of picks and places (the pairing-status line turns green: "✓ N pick→place pair(s) ready"); with a bore selected, **Start spheroid pickup** becomes enabled. Click it. The status line walks through `OP-…: Moving to source`, `Aspirating … µL`, `Moving to dest`, `Dispensing … µL`, `complete.` for each pair (P001→D001, P002→D002, …), then `Done.` If pick/place counts differ, the status line stays orange and Start is disabled.
9. **Abort** during a run sets the abort flag; executor exits the next loop iteration; status shows `Stopped (aborted or failed).` The GUI remains responsive.

**Sub-pixel precision check (real microscope, operator-driven):**

- Move the stage to a known XY; click a feature exactly at the field center — clicked stage µm should match the stage's reported XY within ±1 µm.
- Drag the stage by a known offset (e.g. 500 µm in X); re-click the same feature; the difference between the two clicked stage µm values should equal the commanded offset within ±1 µm.

## Issues & Decisions

- **Version number left open.** The v7.4.3 / v7.4.4 / v7.4.5 / v7.5.0 update-plan slots are already occupied by parallel in-progress work. This plan is filed under a topical name (`MEBP_v74x_WORKFLOWS_MODE.md`) — matching the existing precedent of `MEBP_v74x_OBJECTIVE_CALIBRATION.md` — so the user can decide its release ordering when the other v7.4.x work merges.
- **WorkflowsModePage is not a `ModePage` subclass.** `ModePage` is built around parallel sub-pages with persistent tab access. Workflows are modal — you commit to one, do work, then Back. A plain `QStackedWidget` is the right primitive. To stay compatible with the main window's existing dispatch, `WorkflowsModePage` implements `get_page_title()` (queried at line 1481 of `app.py`) and the fan-out hooks (`set_hardware_config`, `set_well_list`, `set_well_positions`).
- **One camera view, two separate target lists.** Pick and place are not separate camera feeds — that would duplicate the same live view twice on the same page. Instead `LiveTargetPicker` is a single composite widget with one feed and a Mode toggle that routes each new click into either the pick list or the place list. Both overlays draw on the same view so the user always sees the full picture.
- **Picks are paired with places by index.** The user requires that spheroid *i* picked from one location is deposited at a specific second location — so place is a multi-list, not a single shared destination. Pick *i* ↔ place *i*; `pairs()` returns the index-aligned pairs and `_on_start` builds one `SPHEROID_PICKUP` op per pair (`source=pick`, `dest=place`). A dashed connector line on both the camera overlay and the XY workspace makes each pairing visible, and the pairing-status line + the `is_balanced()` Start-gate enforce equal counts. **Index pairing shifts when a middle entry is deleted** (removing `P002` re-pairs the later picks); for v1 this is acceptable and the connector lines + status make the current pairing legible. Explicit linked-pair objects (each pick carrying its own place) are deferred.
- **Right-click handling required subclassing `TargetOverlayCameraView`.** The base `CameraFeedView.eventFilter` only consumes `LeftButton`. Rather than monkey-patching, `_PickerCameraView` adds a `right_clicked` signal and extends the filter.
- **µm/px lookup priority: ObjectiveCalibrationStore → CameraManager fallback.** The store has the empirically measured per-camera, per-objective value; the per-camera cached value in `CameraManager` is the fallback when no calibration exists yet. The picker also writes back to `CameraManager.set_um_per_px` so other consumers (e.g. the executor's downstream calls into `pixel_to_stage_offset`) see the same value.
- **Stage position polled at 5 Hz on the GUI thread.** The `StageController.get_xy_position(cached=True)` call is cheap (reads `PositionPoller.xy_position`), and 5 Hz is fast enough for the overlay to track XY motion smoothly without measurable load.
- **All executor callbacks go through `_ExecutorBridge` Qt signals** rather than calling GUI methods directly. Executor runs on a daemon thread; direct GUI mutation would race against the Qt event loop. The cross-thread signal connection in Qt is automatically `QueuedConnection`, which lands handlers on the main thread.
- **Spheroid config: only diameter + bore + safety factor exposed in v1.** `pickup_speed_uL_s` / `release_speed_uL_s` use the `SpheroidPickupConfig` defaults. We can re-expose them when there's a user need.

## Out of scope (deferred)

- **Stitched-image / mosaic picker** — the old `StitchedImageView` flow is gone. If we need to pick targets across a multi-tile composite, that becomes its own tool (`MosaicTargetPicker`) and a separate workflow.
- **Auto-scan target detection** — the auto-scan well loop in the old `pp_target_selection.py` is gone. A future "Detect spheroids" tool can plug in alongside the manual picker.
- **Explicit linked pick/place pairs** — v1 pairs by list index (pick *i* ↔ place *i*), which re-pairs later entries if a middle one is deleted. A future variant can bind each pick to its own place object so deletes don't shift pairings.
- **The four other workflows** — Cell Targeting & Removal, Cell Labeling, Quick Print, Immuno are stubs. Each will become its own workflow page reusing as many of the same tools as possible.
- **Operation queue editing UI** — no reorder / delete / insert. Executor runs the queue as built.
- **Per-pick config overrides** — one config applies to all picks in a run.
