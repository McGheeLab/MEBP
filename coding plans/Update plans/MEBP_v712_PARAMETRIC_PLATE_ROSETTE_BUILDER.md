# MEBP v7.12 — Parametric Plate & Rosette Builder Redesign

## Objective

Redesign Hardware Setup → Plate / Rosette into a fully parametric builder with the Sketch tool's
feature set, fronted by a Print-Library-style plate library, plus a new Layout tab for placing
designed rosettes into wells.

Operator request (verbatim intent):

1. Reference plate (0,0) from the **bottom-left edge, bottom-right edge, or the center**.
2. **Ring pattern** — pick one point as the ring centre and dimension from it; a **handle to
   rotate one of the circles** to orient the ring; settings expose well diameter, ring diameter,
   number of wells, etc.
3. **Grid pattern** — pick a **seed point for the first well** and dimension it; grid orientation,
   direction and spacing in that object's settings, plus well diameter/height.
4. Set dimensions **from the plate edge and from other features in the part**.
5. A **better way to save the plate name**.
6. **Copy existing plates** — a grid overview page resembling the print-files page, with the same
   delete / duplicate features.
7. **Rosette mode works the same way**, except the design boundary is set by the rosette.
8. A **new tab for rosette placement into plate wells** — design a rosette in the parametric
   builder, then place them into wells (replacing double-click-a-well → add rosette).

Plate builder and rosette builder share one tab as two modes.

## Operator decisions (AskUserQuestion)

| Decision | Choice |
|---|---|
| Migration | **Fresh start** — existing custom designs need not load. Built-in formats + `PlateType` products must keep working. |
| Library home | **Plate tab opens as the library grid**; opening a card enters the builder. |
| Rosette placement gesture | **Both** — stamp-click *and* multi-select → assign. |
| Rosette linkage | **Live link** (edit once, every plate updates) — not a frozen copy. |
| Pattern member drag | **Anchor only** — generated wells are not individually draggable. |
| Legacy Rosette Library table | **Remove it.** |

Full design rationale, the two resolved design conflicts (origin = display-only; live-link guards),
the risk register and the bench checklist live in the approved plan at
`C:\Users\mcghe\.claude\plans\on-hardware-setup-plate-wiggly-journal.md`.

---

## Phase 0 — De-risk the current rig  `[x]` COMPLETE

Ships alone. No new features. These are the defects a large rewrite would otherwise re-introduce,
and fixing them first means the hygiene tests are written against the **old** code so they *prove*
the fix rather than describe it.

### Files Modified

| File | Why |
|---|---|
| `gui/pages/calibration.py` | 🔴 Guard the unhandled `get_well_position("A1")` `KeyError` |
| `SupportClasses/PlateDesign.py` | Filter `drag_ghost` / `id <= 0` out of `to_dict` **and** `from_dict` |
| `config/hardware/plates/user/*.json` | Remove the 7 already-persisted ghost pins |
| `gui/pages/hardware/plate_designer.py` | Save no longer clobbers the typed name; shortcut scoping |
| `gui/widgets/plate_designer_canvas.py` | Shortcut scoping, zoom clamp, grid LOD, drag teardown, dead-code deletion |
| `gui/pages/hardware_setup.py` | Connect `design_edited`; fix `get_sub_page_title` |
| `gui/app.py` | Widen `_emit_invalidation`; fix the `switch_to` index comment/target |
| `SupportClasses/PlateTypeStore.py` | Add `delete_user` (mirroring `WellTypeStore`) |
| `SupportClasses/WellTypeStore.py` | Fix the docstring describing a disarmed safety chain |
| `tests/test_v712_phase0_hygiene.py` | NEW — mutation-style guards for each fix |

### Implementation Steps

- [x] **D1** 🔴 Guard `get_well_position("A1")` at `calibration.py:14262` and `:14331`.
      `get_well_position` **raises `KeyError`, never returns `None`** (`WellPlate.py:352`), so the
      `if a1_expected is None` check at `:14265` is dead code. `compile()` drops the parent well
      when it carries a rosette, so **a rosette in A1 means there is no `"A1"` well** and
      `_calculate_alignment` raises. The operator's orphan calibration key
      `plate-24_Rossette A1` suggests this was already hit.
- [x] **D2** 🔴 Stop `drag_ghost` reaching disk. `PlateSketchSolver.begin_drag:205` appends the
      ghost into `design.constraints`; `to_dict:850` serializes unfiltered and `CONSTRAINT_KINDS`
      includes `"drag_ghost"`, so `from_dict` reloads it as an invisible weight-1000 pin.
      **Verified on disk: 7 ghosts across 3 of 7 user plate files.** Filter on both read and write,
      then clean the files.
- [x] **D3** `_on_save:804` overwrites `design.name` with `_current_key`, making the Plate card's
      Name field decorative (the literal complaint behind requirement 5).
- [x] **D4** Single-letter `QShortcut`s at `Qt.WindowShortcut` scope
      (`plate_designer.py:384-401`, `plate_designer_canvas.py:367-370`) steal keystrokes from every
      text field on the Hardware Setup window. Three-part fix: tool letters move to the canvas's
      `keyPressEvent`, remaining shortcuts get `WidgetWithChildrenShortcut`, plus a focus veto.
- [x] **D5** `_plate_designer.design_edited` is unconnected (`hardware_setup.py:894` wires only
      `plate_changed`), so geometry edits never dirty the config; and `_emit_invalidation`
      (`gui/app.py:1790`) diffs only `plate_format` + pumps, so custom-plate changes never raise
      `hw_config_invalidated`.
- [x] **D6** `PlateTypeStore` has `save_user` but **no `delete_user`** — user overrides are
      permanent. Mirror `WellTypeStore.delete_user:202`.
- [x] **D7** Canvas hygiene: clamp `wheelEvent:1217` (no clamp today → degenerate transform);
      skip the 1 mm grid in `drawBackground:375` when it would draw sub-4-px spacing (~215 lines
      per paint, including during drag); end an in-flight `begin_drag` on tool change and Esc.
- [x] **D8** Delete the dead `Tool.ADD_CONSTRAINT` path — `_enough_picks_for_kind:1559` returns
      `False` on every branch, so `_commit_pending_constraint` can never fire and no UI creates it.
- [x] **D9** Fix the stale index tables this work makes more wrong: `get_sub_page_title():2013`
      (9 labels for 10 tabs, wrong order) and `gui/app.py:229-232` (`switch_to(3)` commented
      "3=Pump", actually lands on Rosette → should be 6).
- [x] **D10** Fix the `WellTypeStore.py:15-21` docstring, which describes a `rim_height_mm` →
      travel-Z safety chain that `gui/app.py:1671` disarmed (`set_min_travel_z(None)`,
      2026-07-27 decision — Fast Move Z is authoritative). `max_rim_height_mm` currently has
      **zero production consumers**.

### Testing Notes

- NEW `tests/test_v712_phase0_hygiene.py` — one mutation-style guard per defect. In particular:
  a ghost must not survive `to_dict()` taken *mid-drag*, plus a sweep asserting no file in
  `USER_PLATES_DIR` contains one; and the shortcut test focuses a `QLineEdit`, sends `"s"`, and
  asserts the field received it **and** the tool is unchanged (reverting any of D4's three parts
  fails it).
- Regression, run **per-suite** (this repo has a documented cross-suite camera hang):
  `test_v745_plate_design` · `test_v745_plate_sketch_solver` · `test_v745_canvas_history` ·
  `test_v746_phase2` · `test_v747_plate_designer` · `test_v748_rosette_flatten` ·
  `test_v75x_plate_types` · `test_v75x_well_type_presets` · `test_v75x_rosette_tab_auto_reanchor` ·
  `test_v75x_rosette_ink_pickup_well` · `test_v75x_plate_template_reregister` ·
  `test_v75x_ink_location_assignments` · `test_v75x_plate_well_detection` · plus a `gui.app`
  import smoke and an offscreen build of the real `HardwareSetupPage`.

### Results

**`tests/test_v712_phase0_hygiene.py` — 31 tests, all green.**

**All 10 mutations confirmed CAUGHT** (each fix reverted in source, its test run, the file restored
in a `finally`): ghost filter write-side · ghost filter read-side · A1 `KeyError` guard · tool
letters returned to window scope · focus veto · `PlateTypeStore.delete_user` · sub-page titles from
registration · plate-identity invalidation · drag teardown on tool change · zoom clamp.

⚠ **One mutation initially SURVIVED and the test was strengthened, not the claim.** Disabling the
focus veto (`_tool_key_allowed`) changed nothing, because it is the *third* leg of the D4 fix and is
redundant while the first (tool letters living on the canvas) holds — the canvas never receives the
key event when a `QLineEdit` has focus. Rather than record a fix no test could see, two tests were
added that exercise the veto's own contract directly (`_tool_key_allowed` against
`QLineEdit`/`QDoubleSpinBox`/editable `QComboBox`, and a key delivered straight to the canvas while
a field holds focus). The mutation is now caught.

**Ghost cleanup:** 7 removed across 3 files — `plate-24_Rosette in A2.json` (3),
`plate-24_Rosette in A2&A3.json` (2), `plate-glass-24_Rosette in A2&A3.json` (2). Backups written as
`*.json.bak-v712ghost`. All 7 user plates re-verified to load and compile with the expected
sub-well counts. `grep drag_ghost config/hardware/plates/user` → **0**.

**Verified by consequence, not by readback:** onboarding's `switch_to(3)` was landing on **Rosette**,
not Pump — the name lookup now resolves Pump to index **6**. And all 10 tabs now report their own
title (the old list had 9 labels, wrong order from index 3 on, Microscope missing entirely).

**Regression, per-suite, 269 green:** phase0-hygiene (31) · plate-design (15) · plate-sketch-solver
(6) · canvas-history (8) · v746-phase2 (13) · v747-plate-designer (16) · rosette-flatten (32) ·
plate-types (38) · well-type-presets (18) · rosette-ink-pickup (18) · plate-template-reregister (15)
· ink-location-assignments (23) · plate-well-detection (36). Plus a second batch, 93 green:
mosaic-orientation-remap (14) · last-known-calibration (21) · plate-centering (6) · freeform-warp
(12) · calibration-revision (20) · plate-location-manual-click-rim (27) ·
startup-well-map-persistence (6) · plate-location-predict-well (10). Plus a `gui.app` import smoke
and an offscreen build of the real `HardwareSetupPage` (10 tabs).

**Three pre-existing problems, each PROVED not caused by this change:**

1. `test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices` — expects the calibration tab
   to be named "Plate Z Auto-Cal"; the working tree calls it **"Plate Bed Level"**. That rename is
   the operator's **uncommitted v7.11** work: the string appears **0×** in `HEAD` and **1×** in the
   working tree, and this change's diff to `calibration.py` is only the five `_well_position_mm`
   lines. Left alone — the test belongs with that in-flight work, not this one.
2. `test_v75x_plate_template_reregister::test_scan_well_guarded_without_rosette` — calls
   `CalibrationPage._ploc_scan_rosette_well`, which exists **0×** in `HEAD`, **0×** in the working
   tree, and **0×** in this diff. It was already failing before this work started.
3. `test_v75x_plate_mosaic` **never terminates** — the `TestManualAlignPage` hang already documented
   in CLAUDE.md ("that hang makes a whole-suite run never terminate and hides real failures").
   Excluded from the batch rather than waited on.

### Issues & Decisions

- **D3 is a rename, not just "stop clobbering".** The operator's ask is that a typed name is the
  plate's name. But in Phase 0 the name IS the per-plate store key (mosaics, templates, well
  training, taught calibration), so honouring a rename genuinely starts a fresh set. Rather than
  either discard the name (the bug) or move data silently, Save now performs a real rename **behind
  a confirmation that states the consequence in those words**. Phase 1's stable id removes the
  trade-off entirely, at which point the confirmation can go.
- **`_activate_tool` deleted** from the designer — it existed solely for the removed window-scoped
  shortcuts; toolbar buttons wire straight to `canvas.set_tool`.
- **`ModePage` gained `_sub_titles` + `sub_page_title()` / `sub_page_index()`** rather than fixing
  the drifted label list in place. A parallel list re-drifts on the next tab insertion; deriving
  from the registration call cannot. Every mode page gets the fix for free, and the redesign is
  about to insert and rename tabs.
- **`HardwareConfig.rosette_library` retained** (deprecated, unread) even though the operator asked
  to remove the legacy Rosette Library. The *UI* removal lands in Phase 4; dropping the field now
  would break deserialization of existing setup JSON.

---

## Phase 1 — The new document, headless  `[~]` model + store COMPLETE

Zero UI, zero user-visible change. Nothing in `gui/` imports any of it yet.

### Files added

| File | Contents |
|---|---|
| `SupportClasses/SolveTypes.py` | `DOFStatus` / `SolveReport`, moved out of `PlateSketchSolver` so three solvers can share them. `PlateSketchSolver` re-exports them, verified `assertIs`-identical, so every existing import still resolves. |
| `SupportClasses/PlateDocument.py` | The model: `PlateBoundary` + `OriginRef`, `WellStyle`, `NamingSpec`, `MemberOverride`, `RosettePlacement`, entities (`Point`/`Line`/`Well`/`RingPattern`/`GridPattern`), 17 constraint kinds, `evaluate()`, `compile()`, validation, serialization, `DocMeta` identity. |
| `SupportClasses/PlateDocumentStore.py` | Library CRUD with builtin/user shadowing, atomic writes, injectable dirs + `MEBP_PLATES_DIR`/`MEBP_ROSETTES_DIR`, `thumbnail_wells()`, `adopt_legacy_key()`, `plate_store_keys()`. |
| `tests/test_v712_plate_doc_geometry.py` | 57 tests — the geometry gate. |
| `tests/test_v712_plate_identity_and_stores.py` | 27 tests — identity, CRUD, key resolution, `WellPlate.load` integration. |

### Files changed

`SupportClasses/WellPlate.py` — `load()` gains `_load_plate_document()`, probed **before** the v1 path (rollback = delete the block); resolves by stable id, then falls back to a display-name scan. `SupportClasses/HardwareConfig.py` — new `plate_doc_id`, inserted into `active_plate_key` between the type and the legacy name, honoured by `geometry_plate_key`, round-tripped in `to_dict`/`from_dict`.

### Key design points, and what they buy

- **Pattern wells are derived, not stored.** A pattern owns one constrainable anchor plus a sparse `overrides` dict keyed by a stable member key (`r0`, `g3_5`). Growing a ring 6→8 preserves `r0`–`r5`'s name, rosette, well type and rim height; shrinking keeps them dormant so growing back restores them. **`rebuild_group`'s silent data loss is now structurally impossible** — there is nothing to rebuild. A 384-well plate is one `GridPattern`: **2,676 bytes** on disk against the old 15–22 KB, and **2** solver variables instead of 768.
- **Origin is presentation.** Pinned by a test asserting `to_dict()` is byte-identical across all six origin modes, and by the golden matrix below.
- **Identity is an opaque hex id; the filename IS the id.** Renaming touches only `meta.name` — no file moves, no key changes. `meta.legacy_keys` + `plate_store_keys()` let a plate adopt its old name as a read-fallback, so switching to stable ids costs no taught calibration.
- **`a1_offset` no longer does two jobs.** The footprint placement is authored (`PlateBoundary.a1_x_mm/a1_y_mm`); the stage-facing `a1_offset_x/y` that `WellPlate` needs is **derived** at compile.
- **`drag_ghost` is not a document kind at all**, so the v7.4.x leak cannot recur even via a hand-edited file.

### Results

**84 new tests green.** The gate is `TestGoldenPositions`: for **all 6 formats × all 6 origin modes**, every well's compiled position is identical to `WellPlate.from_format` to 9 decimal places — asserting the *full* well set, because a pure row inversion preserves both endpoints on some formats. Plus A1-at-origin, the two-prediction-path agreement (they diverge only if A1 leaves the origin, and if they ever did the plate view would show one grid while Go-To drove to another), rotation round-trip, and save→reload→recompile bit-identity.

**Phase 1 gate met — the 120 duck-typed consumer tests are green without edits:** rosette-tab-auto-reanchor (46, 1 pre-existing failure), plate-well-detection (36), ink-location-assignments (23), plate-template-reregister (15, 1 pre-existing error). Full batch **358 green**.

**Three bugs found by writing the tests first**, all in new code:
1. `str()` on a `(str, Enum)` member returns `'OriginRef.BOTTOM_LEFT'`, not `'bottom_left'`, since Python 3.11 — every enum field would have serialized as its repr and failed to round-trip. Fixed with an `_enum_str` helper.
2. Row/column binning scaled its tolerance by the **median** inter-value gap. With two rows of two the gaps are `[jitter, pitch, jitter]`, so the median IS the jitter and every well became its own row. Now scaled by half the **largest** gap, with a 0.5 mm absolute floor for the opposite case (a single row whose only gaps are jitter).
3. One assertion of mine was wrong, not the code: translating a design's anchor legitimately introduces float noise, so bit-equality there was over-strict. Replaced with almost-equal, and bit-equality asserted where it is genuinely required — save → reload → recompile.

### `SupportClasses/PlateSolver.py`

Written at the head of Phase 3, where the builder first needs it. Four deliberate differences from
`PlateSketchSolver`:

- **Pay-for-play variables** — only points referenced by an active constraint enter the vector. An
  unconstrained 384-well plate reports `EMPTY` and the solver never runs (measured: 0 free vars).
- **Pattern members are not variables** — a member resolves to its anchor plus a constant offset, so
  dimensioning one member translates the whole pattern. Verified: an `edge_left` dimension on
  `g0_0` moved all 24 wells by an identical 7.950 mm and landed A1 exactly 25.000 mm from the edge.
- **The ghost lives in the solver** — verified `to_dict()` mid-drag contains no ghost.
- **Structural validation before residuals** — fixes a real latent bug in the old solver, where
  `_residual_count` counted rows `_residuals` then skipped, so the row cursor drifted and the
  **wrong constraint ids were reported as conflicting**.

---

## Phase 2 — Plate library  `[x]`

`gui/widgets/card_grid.py` (`CardGridView` + `BulkSelectBar`) extracts only the reflow arithmetic
from `print_library.py` — the one part that rots when duplicated. A shared `LibraryPageBase` would
need six injection points for two consumers, which is a worse abstraction than the duplication.

`gui/pages/hardware/plate_library.py` — `PlateThumbnail`, `PlateCard`, `PlateLibraryPage(kind=…)`.
One class serves both collections. Standard formats render as read-only cards with a `● standard`
badge, so a fresh install with an empty user directory is fully usable and the library can be the
only picker.

Two thumbnail differences from `PrintThumbnail`, both tested: it needs **no `HardwareConfig`** (a
plate design is already geometry), and it must **not** copy the Y-up flip — plate space is Y-down,
so copying it paints every plate upside-down with A1 in the wrong corner.

## Phase 3 — Builder  `[x]`

`plate_canvas_gizmos.py` · `plate_document_canvas.py` · `plate_property_panel.py` ·
`plate_builder.py` · `plate_workspace.py`.

- **Request 2 — the ring gets TWO handles.** `RadiusHandleItem` sets the diameter with the angle
  locked; `RotationHandleItem`, on a stalk outside the ring so it can never land on a well, sets the
  angle with the size locked. The v7.4.x canvas had one handle whose drag set both, so a ring could
  not be oriented without also resizing it. **Both directions mutation-verified.**
- **Request 3 — the grid is seed + settings.** Click places the seed (the first well); rows,
  columns, spacing, orientation and the **A1-corner 2×2** live in its card. A rotation gizmo and a
  seed handle are on the canvas; the seed drag is a rigid translation (tested: one distinct delta
  across all wells).
- **Request 1 — origin is a display transform.** Six modes; changing one leaves `to_dict()`
  byte-identical (mutation-verified) and moves only the readout and the spin boxes.
- **Request 4 — dimensions** are a two-pick gesture over wells, all four plate edges and both
  centre-lines, with the value seeded from current geometry so nothing jumps.
- Pattern members render dashed and **click-select their pattern** (Alt+click reaches the member),
  because the old behaviour opened a card whose X/Y spin boxes wrote a point the next rebuild
  reverted. Both paths mutation-verified.
- Ported from Sketch: single-mutator selection, marquee via `scene.items(rect, IntersectsItemShape)`
  (not `RubberBandDrag`, which would claim the left button view-wide), throttled ghost drag,
  conflicts painted red, constraint card with a **focus guard** and a 3-column button grid.
- **No in-scene editors** — tested; clicking a dimension pill selects its constraint instead.
- **No Save As** — Duplicate lives in the library. Standards **auto-fork on first edit** with a
  banner naming the copy.

## Phase 4 — Layout (rosette placement)  `[x]`

`rosette_placement.py` — palette · plate schematic · placement table. **Both gestures**, as decided:
stamp stays armed so twenty wells are twenty clicks, and multi-select → Apply covers the bulk case;
tested to reach identical document state. Per-instance rotation from the table, the spin box, or the
shared `RotationHandleItem`. Double-clicking a placement opens that rosette's **design**, replacing
the old drill-into-an-anonymous-nested-design gesture.

Live linking is verified end-to-end: editing a rosette in the library moves the sub-wells of every
plate using it (mutation-verified — freezing the link fails the test), and
`rosette_revision_fingerprint()` changes so the plate's calibration can force a re-teach. A missing
rosette compiles as an ordinary well **and** is reported by name and flagged in the table.

## Phase 5 — Wiring, and what was retired  `[x]`

Hardware Setup tabs are now `Device · Identity · **Plate** · **Layout** · Ink · Needle · Pump ·
Cameras · Microscope · Xbox Controller` — same count; Plate absorbed the rosette designer and the
freed slot became Layout.

**Retired (229 lines + the dialog):**

- The **Format→Type card** and its six methods. It gave plate identity two owners, and
  `_on_designer_plate_changed` / `_on_plate_type_format_changed` each silently cleared the other's
  choice. The active design is the single owner now: the library's ★ sets `plate_doc_id` and the
  product comes from the document.
- Both `PlateDesignerWidget` instances and the `adopt_design` sharing dance.
- The **legacy Rosette Library** table, its four CRUD handlers and `RosetteEditorDialog` — write-only
  (its only consumer, `WellSetup.attach_rosette`, has no callers repo-wide).
  `HardwareConfig.rosette_library` is **retained as a deprecated field** so older setup JSON still
  round-trips.

**Four tests in `test_v75x_plate_types.py` were consciously replaced, not deleted** — they pinned the
retired card's widgets; the contract they protected (selecting a plate resolves `active_plate_key`,
`set_hardware_config` round-trips it) is re-pinned against the new surface.

### Results

**`tests/test_v712_plate_builder_ui.py` — 47 tests.** Totals: **590 green across 26 suites**, run
per-suite. **8 further mutations confirmed CAUGHT**: conflated ring handles (both directions) ·
member click selecting the member · origin leaking into serialized geometry · thumbnail Y-flip ·
rosette link frozen instead of live · pattern overrides dropped on a parameter change · store rename
minting a new id. Plus a `gui.app` import smoke and a real `HardwareSetupPage` build cycling all ten
tabs.

**The same two pre-existing failures as at Phase 0, re-confirmed identical and still not ours**:
`test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices` (the operator's uncommitted v7.11
"Plate Bed Level" rename) and `test_v75x_plate_template_reregister::test_scan_well_guarded_without_rosette`
(calls a method that exists 0× in HEAD).

---

## Phase 6 — Operator feedback: pattern anchors and the pick tools  `[x]`

Operator, on the shipped builder: *"the grids and ring of wells objects have no way to dimension the
reference point to the edge of the plate"* and *"in general the dimension and line drawing tools are
not easy to use."*

### 1. The pattern anchor was never clickable

The **solver could always do this** — `PlateSolver._base_point((feature_id, ""))` resolves a pattern
anchor, `_valid` accepts `distance_to_datum` on it, and satisfying that dimension translates the
whole pattern rigidly, exactly as the model was designed to. The entire gap was in the canvas:

- `_dim_target` hit-tested **only wells** (`_hit_well`). A ring with `center_well=False` has *no well
  at its centre*, so its reference point was not merely hard to click — there was nothing there to
  click. Requirement 2's "dimension everything from that point" was unreachable.
- The existing `SeedHandleItem` (whose tooltip already reads *"Dimension this point to place the
  pattern"*) is created by `_refresh_gizmos` **only while exactly one feature is selected**, and was
  never hit-tested by the dimension tool anyway.
- A grid seed *is* a well, so it could be picked — but it returned the **member** ref `(feat, "g0_0")`.
  That key changes when the A1-corner direction flips, so a dimension hung on it would silently
  retarget to a different corner well.

Fixed with a new `AnchorMarkerItem` (`plate_canvas_gizmos.py`): a quiet, **always-present,
always-hittable**, screen-constant ✛ at every pattern anchor, drawn by `_draw_anchors()`. It is
distinct from `SeedHandleItem`, which stays the bold selected-state drag handle. `_dim_target` and
`_press_select` now check it **first**, so it wins over a coincident seed well and the stable
`(feature_id, "")` ref is what gets dimensioned.

Two rendering gaps fell out of the same audit:

- **`_draw_dimensions` indexed `evaluate()`**, which has no entry for an anchor — so a datum
  dimension on a ring centre would have been created, solved, and drawn **nowhere**. Now falls back
  through `_base_point`.
- **Well-to-well `distance` dimensions were never drawn at all.** Only `distance_to_datum` had a
  renderer, so adding one left the canvas visually unchanged. New `_draw_distance_pill`.

### 2. "Not easy to use" reduces to *you cannot see what a click will do*

- **The plate edges were an invisible target.** `_nearest_datum` used `max(tol, 1.5)` mm, so zoomed
  out on a 128 mm plate the four edges were a couple of device pixels wide, with **no hover feedback
  of any kind** — the approved plan called for hoverable edge bands and they were not built. Now a
  screen-constant `_DATUM_PX` band (`_datum_tol_mm`) plus a full live preview layer.
- **New preview layer** (`_update_preview`, driven from `mouseMoveEvent`): hovering highlights
  exactly what will be picked — the edge/centre-line as a bright full-length line, a well as a ring,
  an anchor as a marker. The **first pick stays highlighted** while choosing the second. The line
  tool draws a rubber band to the cursor; the ring tool previews its circle during the drag. The
  layer is pure decoration — a test asserts `to_dict()` is unchanged across a dozen moves, and
  another that the items cannot accumulate.
- **Escape now backs out one step.** It used to jump straight to `Tool.SELECT` *and* clear the
  selection, so mis-clicking the first of a dimension's two references cost you the tool as well.
  First Escape abandons the gesture and leaves you armed; second leaves the tool. Right-click
  cancels an in-flight gesture too (CAD idiom).
- Re-picking the same reference is refused rather than silently accepted; two edges report why they
  cannot be dimensioned to each other; `_describe` names things the operator recognises ("ring
  centre", "grid seed") instead of a raw member key.

### Results

**`tests/test_v712_plate_builder_ui.py` 47 → 69** (+22: `TestPatternAnchorIsDimensionable` 8,
`TestDimensionAndLineUsability` 14). **8 mutations confirmed CAUGHT**: anchor hit-test removed ·
`_draw_anchors` no-op · datum band back to the 1.5 mm floor · Escape jumping to SELECT ·
hover preview unhooked from the mouse · `_draw_dimensions` losing its anchor fallback ·
well-to-well pills not drawn.

⚠ **One mutation initially SURVIVED and exposed a weak test of mine:** every preview test called
`_update_preview` directly, so simply **unhooking it from `mouseMoveEvent`** — which would restore
precisely the invisible-target complaint — passed all of them. Added
`test_moving_the_real_mouse_drives_the_preview`, which drives a real `QMouseEvent`; the mutation is
now caught. (Same trap already recorded twice in this repo: a v7.10 guard that passed on an import
line alone, and a v7.11 guard that matched its own explanatory comment.)

Regression re-run per-suite and green: the four v7.12 suites (69/57/27/31), `test_v75x_plate_types`
(38), `test_v748_rosette_flatten` (32), `test_v75x_rosette_ink_pickup_well` (18),
`test_test_suite_hygiene` (8), plus the **free consumer gate** — `test_v75x_plate_well_detection`
(36) and `test_v75x_ink_location_assignments` (23) — and the frozen v1 suites
`test_v745_plate_design` (15) / `test_v745_plate_sketch_solver` (6) /
`test_v745_canvas_history` (8) / `test_v746_phase2` (13) / `test_v747_plate_designer` (16), all
**without edits**. Plus a `gui.app` import smoke and a real `HardwareSetupPage` cycling all ten tabs.

The same pre-existing `test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices` failure,
re-confirmed not ours: `git diff` shows the `"Plate Bed Level"` rename lives in the operator's
uncommitted v7.11 `gui/pages/calibration.py`, and today's changes are confined to
`plate_document_canvas.py`, `plate_canvas_gizmos.py` and the v7.12 test file.

### 3. Every pattern was born with the same names

Operator: *"when i put multiple grids or rings, the names of the wells are the same and there is no
way to change that."*

`add_grid` defaulted to `NamingSpec(scheme=ANSI, start_row=0, start_col=1)` and `add_ring` to
`LETTERS` — **unconditionally**. So the second grid re-emitted `A1…` over the first, and the second
ring re-emitted `a…`.

**This was not cosmetic.** `WellPlate.from_wells` keys its dict by `name.upper()`, so the duplicates
are **silently dropped** — most of the second pattern simply does not exist in the compiled plate,
and therefore not in the calibration map, the ink assignments or a print. `validate()` did report it
(*"N wells are named 'A1'… a duplicate is silently dropped at runtime"*) — but **the property panel
had no naming control of any kind**, so the operator was handed a diagnosis with no treatment. That
is the literal complaint.

Three changes:

- **`autoname_feature()`** — shifts a pattern's naming until none of its wells collide, advancing the
  natural axis for the scheme (the row letter for an ANSI grid, the letter/number offset otherwise).
  Called from `add_ring`/`add_grid` **only when the caller passed no explicit spec**, so a deliberate
  choice is never silently overridden. Two grids now read `A1…D6` then `E1…F3`; two rings `a…f` then
  `g…j` — the sequence continues across the plate, which is what an operator would have typed.
- **`_member_name` generalised** so LETTERS and NUMBERS honour `prefix` and the start offset the way
  ANSI already did. **At the defaults every scheme is byte-identical to before**, pinned by tests on
  a lone grid, a lone ring, and all four standard formats.
- **A Naming block on both pattern cards** — scheme · prefix · first row · first number · an
  **Auto-number** button · and a live preview that either lists the names (`Wells: E1, E2, E3, F1 …`)
  or warns in peach, *naming the consequence*: *"⚠ 6 name(s) clash with another pattern (A1, A2, A3…).
  Wells sharing a name are dropped from the plate — use Auto-number or a prefix."*

Also added: `feature_member_names()` and `names_in_use(exclude=)` as the shared, testable primitives
(the latter counts **override** names, or a later pattern could be auto-numbered straight onto a
hand-renamed well), and a guaranteed-unique prefix fallback for a document too crowded to shift into.

### Results (naming)

**+8 geometry tests, +8 panel tests** → `test_v712_plate_doc_geometry` 57 → **73**,
`test_v712_plate_builder_ui` 69 → **77**. **13 further mutations confirmed CAUGHT**: autoname
unhooked from `add_grid` / from `add_ring` · autoname never advancing · `names_in_use` not excluding
itself · each of the four scheme branches losing its prefix/offset · the panel's clash warning
removed · the naming rows not built · the unique-prefix fallback removed.

⚠ **Two of my own mutations were mis-aimed and one of them found a real coverage hole.** The first
"LETTERS ignores the offset" mutation SURVIVED — the 8-space anchor was a *substring* of the
16-space grid branch, so it patched the grid path instead of the ring path, **and no test covered
grid-with-LETTERS at all**. Added `test_every_scheme_honours_the_prefix_and_offset`, which walks all
three schemes × both feature kinds. The unique-prefix fallback was likewise unreached until
`test_the_unique_prefix_fallback_actually_works` built a genuine deadlock out of two MANUAL-scheme
grids (whose names come from the member key, so shifting can never break the tie).

### 4. 🔴 There was no way to select a plate for the setup

Operator: *"there is no way to select a plate to use for the actual setup"* — and, separately, *"I do
not want to see all those extra well plates"*, *"we want standard 6,12,24,48,96,384 · corning 24 glass
bottom · nest plastic 24 · and the custom one i just made"*, *"on the selection tool it should also
have the option to select the mosaic we have made to scan it"*, *"i think all the plates you are
building are the same they just have different mosaics attached."*

Phase 5 retired the Format→Type card and made the library's ★ the plate selector. Two defects made
that selector inert:

- **`PlateWorkspacePage._on_active_changed` dropped standards** (`and not is_standard(doc_id)`), on
  the reasoning that a bundled format has no document for `plate_doc_id`. But a bundled format is
  carried by `plate_format` — the last leg of `active_plate_key`'s precedence — and that decision
  belongs to the host, which knows the field. On a fresh install the library is *nothing but*
  bundled entries, so "Use this plate in the hardware setup" did nothing at all.
- **`PlateType` products had no card whatsoever.** With the Format→Type card gone, `corning-glass-24`
  and `nest-plastic-24` — which carry the Z offsets and their own mosaic — became unreachable.

**The operator's own diagnosis is the design.** A product is not different geometry; it is the same
base format with different Z offsets and, decisively, **its own `active_plate_key`** — which is what
gives it a separate mosaic, calibration, template and well-training set. So the library is now a list
of exactly that: **one card per `active_plate_key`**, over three sources (standards · products ·
documents), with `plate_key_for()` as the single mapping. The activation handler branches to match
`active_plate_key`'s precedence, and syncs the shim combo in the bundled case — without that,
`_rebuild_config`'s `elif not plate_doc_id:` fallback silently reverted the choice on the next
rebuild.

**Mosaic on the face of the card**, because that is the real difference between two cards that draw
identically: `◉ mosaic · nest-plastic-24 (+3 well scans)` or `○ no mosaic`, plus a picker to reuse
another plate's scan or clear this one. That copy is `MosaicStore.copy_plate` — **extracted from
`calibration._ploc_copy_mosaic_key`, which now delegates to it**, because a multi-step copy (image ·
extent · scale · shift · well mapping · every single-well scan) kept in two places drifts the first
time either gains a field.

**Curation**: standards and products ship and cannot be deleted, so they gain **Hide** — persisted
beside the documents in `library_hidden.json`, reversible from a `Hidden (n)` toggle, and refused for
the active plate (an invisible ★ would leave the operator unable to see what the setup uses). The
four products for formats this lab does not run are hidden by default on this machine, matching the
list above.

**🐞 A layout regression found by an existing test, not invented by it:** the "Active in setup: …"
label pushed the whole tab's minimum width to **832 px**, because a label's minimum grows with its
text and a plate name is arbitrarily long. The card-reflow test caught it (3 columns at both 900 px
and 360 px). Now `Ignored` + elide → **604 px**.

**🐞🐞 AND MY OWN TESTS HAD BEEN WRITING INTO THE OPERATOR'S LIVE PLATE LIBRARY.** `HardwareSetupPage`
builds its own `PlateDocumentStore` from `MEBP_PLATES_DIR`, so the Phase 5 tests — which inject a
store into the *page's helpers* but construct the *page* itself — created their fixtures in
`config/hardware/plates/v2`. **Nine junk plates** (`Wired plate` ×5, `Bench plate`, `Plain plate`,
`Restored plate`) had accumulated there and were showing as cards on the operator's Plate tab, which
is part of what "all those extra well plates" was. Removed (their one real plate, *"Custom 6 insert
with rosette"*, untouched); both fixtures now redirect the env var; `load_hidden`/`save_hidden`
resolve their path **from the store's own directory** rather than globally, so injecting a store
isolates this file too; and a new `TestPlateStoreIsolation` in `test_test_suite_hygiene.py` fails any
test file that builds a `HardwareSetupPage` and creates a document without setting
`MEBP_PLATES_DIR` — with a companion test proving the check can actually fire.

### Results (selection)

`test_v712_plate_builder_ui` 77 → **99** (+22: `TestPlateSelection` 15, `TestActivePlateReachesTheConfig`
6, one layout guard), `test_test_suite_hygiene` 8 → **10**. **8 mutations confirmed CAUGHT**: the
workspace dropping standards again (the bug itself) · products removed from the library · product
activation losing its type id · the combo left unsynced so a rebuild reverts · the ★ not following a
product · the hidden set not persisted · the active plate becoming hideable · `plate_key_for` not
unwrapping a product.

⚠ **One mutation survived and exposed a test that checked the wrong thing.** "The active plate
becomes hideable" passed because a *second* guard (`refresh` always shows the active card) produced
the same visible outcome. The distinction matters: if the hide were merely masked rather than
refused, switching the active plate away would make the old one vanish. `test_hiding_the_active_
plate_is_not_merely_deferred` now asserts the hide is never recorded, and the mutation is caught.

### 5. 🔴 A valid custom plate locked the operator out of Calibration

Operator: *"now the software doesnt let me continue to calibration because the hardware says its not
fully defined. It should be defined."*

`HardwareConfig.is_valid` gates page unlocking (`app.py:898`), and `validate()`'s plate branch is
supposed to check that the key `active_plate_key` returns actually resolves. **Phase 1 inserted
`plate_doc_id` into that property and not into this check**, so a v2 design fell through to the
legacy branch:

```python
if self.plate_type_id:      ...          # product
elif self.plate_name:       ...          # ← a v2 doc landed HERE
    if not (USER_PLATES_DIR / f"{self.plate_name}.json").exists(): ...
```

— and was looked up as a **v1 file named after its DISPLAY NAME**. That file can never exist: the
entire point of the v7.12 identity change is that the filename is the stable id and the name is only
a label. So `Custom 6 insert with rosette` reported *"not found in …/plates/user"*, `is_valid` went
False, and **every page stayed locked**. Standards and products were unaffected, which is why it
surfaced only once the operator selected their own plate.

Fixed by walking the same precedence the property does, with the doc resolved **by id** through
`get_plate_store()`. A genuinely dangling `plate_doc_id` is still reported, now with an actionable
message (*"…is no longer in the plate library — pick one on Hardware Setup → Plate"*) instead of a
path the operator cannot act on.

`geometry_plate_key` had already been updated in Phase 1; **`app.py::_emit_invalidation` had not**.
It watched `("plate_name", "plate_type_id")`, and since two plates may legitimately share a display
name (the library warns but allows it), switching between them changed only `plate_doc_id` and
invalidated nothing — leaving the calibration page on the previous plate's taught wells. Now watched.

**The lesson, and the guard:** `active_plate_key`'s precedence is duplicated in three places
(`validate`, `geometry_plate_key`, `_emit_invalidation`). The new
`test_validate_accepts_whatever_active_plate_key_returns` closes the loop instead of restating the
rule — for each identity shape it asserts `WellPlate.load(active_plate_key)` succeeds **and**
`validate()` raises no plate issue, so the two cannot disagree again without a test failing.

### Results (validation)

`test_v712_plate_identity_and_stores` 27 → **32**, `test_v712_phase0_hygiene` 31 → **32**.
**3 mutations confirmed CAUGHT**: the `plate_doc_id` branch removed (the bug itself) · a dangling
document no longer reported · the invalidation diff reverted to two fields.

⚠ **One of my own tests was wrong in a way worth recording.** `test_the_hidden_list_lives_beside_
its_own_store` asserted the *default* `library_hidden.json` does not exist — which passed only while
the operator had never hidden anything, and broke the moment their library was curated. A test must
not assert the absence of a real file it does not own; rewritten to assert the two paths differ and
that the hidden id is absent from the default list.

### 6. A custom plate had nowhere to store its learned Z offsets

Operator: *"now the needle offsets wont save to the plate because it has no type: Select a specific
plate TYPE on Hardware Setup → Plate first (the generic format has nowhere to store offsets)."*

The v7.5.x learn loop persists the taught needle-Z references back as mm-below-fiducial offsets so
the plate auto-fills next time — and it could write them **only to a `PlateType`**. A v7.12 design is
a plate identity in its own right (it is what `active_plate_key` returns and what every per-plate
store keys on), but it had no field for these, so the operator was told to pick a "specific plate
TYPE" for a plate that already was specific.

Same root shape as defect 5: **three sites reached for `plate_type_id` independently** — the writer
(`_zoff_save_offsets_to_plate_type`), the adopter (`StageController._apply_active_plate_type_offsets`)
and the calibration page's auto-fill (via the controller). Fixed by giving `DocMeta` a `z_offsets`
dict with the same shape and meaning as `PlateType.z_offsets` (conditional-emit, so every existing
plate file round-trips byte-identically) and adding **one resolver** each side:

- `HardwareConfig.plate_z_offsets()` — what the active plate's offsets ARE.
- `HardwareConfig.plate_z_offset_home()` — where a newly learned one would be SAVED
  (`("document", id)` / `("type", id)` / `("", "")`).

**Merged per key, product then design**, so a fork of a Corning-24 that has learned only its plate
bottom keeps inheriting the product's guesses for top / safe / max. A measured value always beats an
inherited one, and both are only ever the *starting guess* — the taught-wins gate in
`_apply_plate_type_z_estimates` still means a taught reference is never overwritten.

A bare standard format genuinely has nowhere to put these, so that one case still refuses — but the
message now says what to do (pick a product, or Duplicate the standard into your own plate) instead
of naming a control that no longer exists.

### Results (Z offsets)

`test_v712_plate_identity_and_stores` 32 → **42**, `test_v712_plate_builder_ui` 99 → **103**.
**6 mutations confirmed CAUGHT**: `plate_z_offset_home` ignoring a design (the bug) · the resolver
ignoring a design's offsets · the resolver replacing instead of merging · the controller reverting to
`plate_type_id` · the writer unable to target a document · `z_offsets` dropped from serialization.

### Bench note added to the checklist

The paper test (step 1) is unchanged, but add: **dimension a ring centre and a grid seed to a plate
edge, type a new value, and confirm the whole pattern translates rigidly and no member moves
relative to its neighbours.** That is the one behaviour this phase makes reachable, and it is
verifiable with zero motion.

Also add, still with zero motion: **place two grids and two rings, then check the compiled well
count equals the sum of their members and that Go-To lists every name exactly once.** A dropped
duplicate is invisible on the canvas — the drawing shows both patterns while the plate has lost one
of them — so this has to be read off the compiled plate, not the screen.

---

## Status

- [x] Phase 0 — de-risk the current rig
- [x] Phase 1 — `PlateDocument` model, `PlateSolver`, `PlateDocumentStore`, `WellPlate`/`HardwareConfig` integration
- [x] Phase 2 — plate library
- [x] Phase 3 — parametric builder
- [x] Phase 4 — rosette placement
- [x] Phase 5 — host wiring, v1 retired
- [x] Phase 6 — operator feedback: pattern anchors dimensionable, dimension/line tools made legible,
      per-pattern well naming (auto-de-conflicted, and controllable), and the plate SELECTOR fixed —
      standards and `PlateType` products are activatable, one card per `active_plate_key`, mosaic
      shown and re-attachable per card, bundled entries hideable, and
      `validate()` taught about `plate_doc_id` so a custom plate no longer
      locks every page, and a design can own its learned Z offsets

**⚠ A pattern worth naming, since it caused defects 5 AND 6:** Phase 1 added `plate_doc_id` to
`active_plate_key` but that precedence is *restated* in five places — `validate`,
`geometry_plate_key`, `_emit_invalidation`, the Z-offset writer and the Z-offset adopter. Each one
missed is a plate identity that half-works. Both are now single resolvers on `HardwareConfig`
(`plate_z_offsets` / `plate_z_offset_home`) with a loop-closing test, but **any future field added to
plate identity must sweep all of these** — grep `plate_type_id` outside the stores.

**Still outstanding:** the old `PlateDesign` / `PlateSketchSolver` / `plate_designer.py` /
`plate_designer_canvas.py` files remain in the tree (unreferenced by `gui/` except through their own
tests, which still pass). Deleting them is a separate, separately-verified change, and the plan's own
precondition applies: **not until every plate the lab actually runs has been re-authored in v2 and
bench-verified.** The bench checklist in the approved plan has not been run — no hardware was
available in this session, and step 1 (the paper test) is the go/no-go for everything after it.
