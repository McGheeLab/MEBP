# MEBP v7.21 — customisable workflow pages: move settings sections onto the page and reorder them

## Objective

Operator: *"on the quickprint workflow setup page I want to be able to take any
section in the settings and move it into the setup workflow section area where the
queue, readiness etc is, all of these sections should be able to be moved up and
down to make a customizable workflow. in the settings page make a checkbox on all
sections to move to main workflow page."*

Two things:

1. **Promotion** — every section in the ⚙ Settings popout gets a checkbox that
   moves that whole section onto the workflow page.
2. **Reordering** — every card in that page's column (the page's own Object /
   Queue / Print parameters / Readiness / status cards *and* anything promoted)
   moves up and down, so the column reads in whatever order matches how the
   operator actually works.

### Decisions (with the operator, via AskUserQuestion)

| Question | Answer |
|---|---|
| Where the arrangement is remembered | **Machine preference** (`settings.json`) — loading a colleague's saved print profile changes the VALUES but never rearranges the page |
| Which cards reorder | **Everything in the column**, built-ins included |
| Who gets the checkbox | **Every workflow with a popout** (all seven) |

---

## The fact the whole design rests on

`WorkflowSettingsDialog` registers fields **by widget identity**:

```python
self._fields[key] = (widget, default)
```

`collect()` reads `widget.value()`; `apply()` / `reset_defaults()` set it; the
Common-Print-Settings links hold widget references. **None of them care where the
widget is laid out.** So promoting a section is a *reparent of one `Card`*, and
every field in it keeps saving, loading, resetting and inheriting with **no
re-registration at all**.

That is why this is not the `register_external` problem again (v7.7 added that
because a widget has one parent, so a field promoted to the page could not also
be laid out inside a section). Moving the whole **card** sidesteps it: the section
leaves the popout and arrives intact.

It is also why the promotion host must **place** widgets and never **build** them
— the opposite of `CustomContextPanel`, which rebuilds its bodies from a registry
on every change. A settings section cannot be rebuilt; rebuilding it would mint
new widgets and orphan every registered field. `SectionStack` therefore treats
entries as **externally owned** and never deletes one (pinned by an AST test that
no `deleteLater` targets a caller's widget).

---

## Files

### New

| File | Why |
|---|---|
| `SupportClasses/WorkflowLayoutStore.py` | The pure model: which sections are promoted, and the card order, per workflow. Qt-free, duck-typed on `Settings`. |
| `gui/widgets/section_stack.py` | `SectionStack` (the reorderable column), `PromotedSectionsPanel` (a bounded self-hiding drawer), and `wire_section_promotion` (the ONE wiring implementation all seven pages use). |
| `tests/test_v721_workflow_layout_store.py` | 21 tests, no Qt, 1 ms. |
| `tests/test_v721_section_promotion.py` | 49 tests: stack, dialog promotion, wiring order, all seven pages. |
| `coding plans/Update plans/MEBP_v721_CUSTOMISABLE_WORKFLOW_SECTIONS.md` | This document. |

### Modified

| File | Change |
|---|---|
| `gui/widgets/components.py` | `Card.has_header()` + `Card.add_header_widget()` — the header slot a titled card lacked, so a control can be added after construction without landing in the body where it would read as content. |
| `gui/dialogs/workflow_settings_dialog.py` | `section_id_for`, section registry, the per-section promote checkbox, `set_promotion_host` / `set_section_promoted` / `restore_promotions` / `_restore_index`. |
| `gui/pages/workflows/quick_print_workflow.py` | The Setup column becomes a `SectionStack` with the five built-in cards as entries; five `SEC_*` ids; `_wire_section_promotion`. |
| `gui/pages/workflows/{spheroid_pickup,cell_targeting,cell_labeling,stress_test,timing_calibration,fluorescence_mosaic}_workflow.py` | A `PromotedSectionsPanel` + one `wire_section_promotion` call each. |
| `CLAUDE.md` | Row in the *Existing Update Plans* table. |

---

## Implementation Steps

- [x] 1. `WorkflowLayoutStore` — `promoted` / `order` / `collapsed`, `merge_order`, `move_in_order`.
- [x] 2. `Card.has_header()` / `add_header_widget()`.
- [x] 3. `SectionStack` — add / take / set_order / move, two affordance paths, edge-greyed buttons.
- [x] 4. `PromotedSectionsPanel` — bounded, self-hiding drawer.
- [x] 5. Dialog: section ids, checkbox, promote/restore, original-neighbour restore.
- [x] 6. `wire_section_promotion` — the single shared wiring, owning the restore ORDER.
- [x] 7. Quick Print: the Setup column becomes the stack; built-ins are entries.
- [x] 8. The other six pages: drawer + wiring.
- [x] 9. Tests (70) + 30-mutation matrix + regression.
- [x] 10. Plan doc + CLAUDE.md row.

---

## Design notes worth keeping

### The arrangement is a machine preference, not part of a profile

`settings.json` section `workflow_layout`, beside `jog.custom_z_mm` and
`context_panel.*`. Deliberately **not** `set_extra_state` (where the v7.19 plate
queue lives): the queue *is* the work to be done and should travel with a profile,
whereas how the operator likes their page laid out should not change because they
loaded someone else's print settings.

⚠ Always load-modify-save through the injected settings object. The store never
constructs a `Settings()` — doing that and saving wipes the file (a documented
incident in CLAUDE.md).

### A stored order is a HINT, never an authority

`merge_order(stored, present)` always returns a **permutation of `present`**:

* a stored id whose card no longer exists is dropped;
* a card that `stored` has never heard of (added by a later version, or just
  promoted) is appended rather than omitted.

That single property is what makes it impossible for a stale, hand-edited or
older-version order to **hide a card** from a workflow page — the failure mode
that would be hardest to diagnose from a screenshot. Pinned by a property test
over six stored shapes, plus a page-level test that a stored order naming a ghost
card still shows all five real ones.

### The section id is the TITLE slug, not an ordinal

An ordinal would shift every stored promotion the moment a section is inserted
above it. A slug means a *renamed* section quietly loses its stored promotion and
reverts to the popout — self-healing, because both `promoted()` and `merge_order`
ignore an unknown id. ⚠ Slugging is many-to-one (punctuation is dropped), so
`add_section` **dedupes** rather than letting two sections share one id and fight
over one checkbox.

### An un-promoted card returns to its original NEIGHBOURS, not its original index

Sections above it may themselves be promoted right now, so a stored absolute
index drifts. `_restore_index` inserts before the first still-present section with
a larger ordinal — and always before `finalize()`'s trailing stretch, which is why
the index comes from a widget scan rather than arithmetic. Both properties are
mutation-pinned, including the three-promoted-then-return case where a stored
index would visibly land the card in the wrong place.

### Promotions restore BEFORE the order is applied

An order applied first cannot place a card that does not exist yet, so a promoted
section would silently end up at the bottom regardless of where the operator put
it. `wire_section_promotion` owns that ordering, which is the main reason it is
one shared function rather than seven hand-written copies (an AST test pins that
all seven pages call it).

### 🐞 The bug that would have read as "reordering doesn't persist"

`restore_promotions` re-applies the stored promotions on every launch, and each
`set_section_promoted` fired the host's `on_change` — which persists the card
order. So the restore wrote the **current (default)** order over the stored one
*before* it could be read: the promotions survived a restart and the arrangement
silently reset to default every time. Found by the end-to-end smoke, not by a unit
test. Fixed with `notify=False` on the restore path (mirroring the `persist=False`
already there for the same reason), and mutation-pinned.

### Quick Print hosts directly; the other six get a bounded drawer

Quick Print's Setup column is already a `QScrollArea`, so the stack goes straight
in and the five built-in cards are ordinary entries. The alternative — a fixed
block plus a reorderable tail — would mean two placement rules for one column, and
"move Readiness above Queue", the operator's own example, would be impossible.

The other six pages' main columns do **not** scroll. Promoting three tall sections
there would grow the page past the window and push the run row — and therefore
**Abort** — out of reach, with no scrollbar to get back. So they get
`PromotedSectionsPanel`: **hidden while empty** (a page with nothing promoted has
exactly its pre-v7.21 geometry) and **height-bounded with internal scrolling**
when not. It is added **after** the run row on purpose, so Start / Abort never
move. Both properties are mutation-pinned, and the hide/show is verified on all
six pages.

### An embedded Quick Print gets no host

`print_calibrator_workflow` hosts a live `QuickPrintWorkflowPage`. Letting the
operator move a section onto an embedded surface would put a card inside someone
else's page with nothing saying it had happened, so `_wire_section_promotion`
returns early when `embedded` — the same reasoning that hides the v7.19 plate
queue there. Pinned, and `set_section_promoted` then refuses rather than crashing.

### Three redundancies removed because a mutation proved them dead

Every one of these was a *second* enforcement point for one fact, and a mutation
of one alone was therefore a **no-op that proved nothing**:

1. `chk.setVisible(False)` at construction **and** `_sync_promote_checks`.
   Checkbox visibility now has one owner, called at build time and whenever a host
   is registered.
2. An explicit `_content_layout.removeWidget(card)` before handing the card to the
   host. Qt drops a reparented widget from its previous layout itself — proved by
   the mutation changing nothing — so the call is gone.
3. A `_promo_applying` re-entry guard around `_sync_check`. `set_section_promoted`
   is idempotent (it returns early unless membership must actually change), so the
   `toggled → set → sync → toggled` loop already terminated after one pass; the
   guard never fired. Deleted, with the idempotence documented as the mechanism,
   and the mutation re-aimed at the idempotence check itself.

---

## Testing Notes

**70 new tests**, all green: `test_v721_workflow_layout_store` (21, no Qt, 1 ms)
+ `test_v721_section_promotion` (49, offscreen).

### Mutation matrix — 30/30 CAUGHT

Anchors are verified unique against the live source before the run starts (a stale
anchor SKIPS and looks like a pass) and sources are restored and re-read in a
`finally`.

| # | Defect re-introduced |
|---|---|
| M1–M5 | `merge_order` drops a new card · keeps a dead id · `move_in_order` wraps instead of clamping · a redundant `set_promoted` writes anyway · stored garbage is trusted |
| M6–M12 | `set_order` drops a card · `move` wraps · a reorder moves the list but not the LAYOUT · `take()` deletes the caller's widget · `take()` leaves the injected ▲▼ behind · edge buttons stay enabled · a duplicate id stacks a second copy |
| M13–M14 | the drawer is always visible (empty pages grow) · its height is unbounded |
| M15–M17 | **the order is applied before promotions are restored** · a move no longer persists · ↩ unwired |
| M18 | **restore re-persists the order = the launch-reset bug** |
| M19/M19b | the checkbox is visible with no host · the one visibility sync is not called at build time |
| M20–M23 | the card never reaches the host · an un-promoted card is appended instead of restored to its neighbours · `_restore_index` ignores which siblings are present · promotion is not persisted |
| M24–M26 | duplicate titles share one id · `promotable=False` still gets a checkbox · `set_section_promoted` is no longer idempotent |
| M27–M28 | **an EMBEDDED Quick Print gets a host** · the Setup column's default order changes |
| M29 | `add_header_widget` silently drops a control on a headerless card |

⚠ **One of my own tests was passing on a corpse, and a mutation caught it.**
`deleteLater` is **deferred to the event loop**, so a widget already scheduled for
destruction still answers `objectName()` perfectly well — the "taking a wrapped
widget does not delete it" test therefore SURVIVED the mutation that deletes the
caller's widget along with the stack's wrapper. Both `take()` tests now flush the
deferred-delete queue first (`_flush_deletions`), which is what gives them teeth.

⚠ **Three further mutations survived as no-ops** and are what exposed the
redundant enforcement points listed above; they were re-aimed only after the
production code had one owner per fact.

⚠ **The harness itself needed a fix**: a mutated run produced non-cp1252 bytes on
stderr and the subprocess decode raised, killing the run mid-matrix. Now
`encoding="utf-8", errors="replace"` with None-safe concatenation.

### Regression

**411 green** — the two new suites + workflow-settings-popout + the v7.19 Quick
Print queue + zones + the v7.20 calibrator + suite hygiene.
**372** across all six integrated pages (spheroid / cell-targeting / cell-labeling
/ stress-test / timing-calibration / fluorescence-mosaic + the cell-targeting
setup panel + the spheroid survey tab).
**178** across the `Card`/components consumers (common-print-settings,
context-panel, plate-builder-ui, incubator-gui), plus a `gui.app` import smoke.

**Pre-existing failures, PROVED not ours:**

| Test | Why |
|---|---|
| `test_v79_cell_targeting_setup_page::test_the_real_saved_profile_reproduces_its_exact_volume` | It reads `config/workflows/cell_targeting/__last__.json`, whose `push_depth` is **0.1 in the working tree vs 0.400052 committed** — an operator edit present in this session's first `git status`, before any of this work. The failing assertion is exactly a push_depth-derived volume. Already named as pre-existing in CLAUDE.md's v7.19 and v7.20 entries. |
| `test_v712_plate_builder_ui::TestLearnLoopSavesToADesign` ×2 | The plate-type `max` z-offset from other working-tree WIP — the failure text is literally `{'max': None}`. Already named as pre-existing in CLAUDE.md. |

⚠ **Disclosed:** this tree carries substantial concurrent work from other sessions
(fluorescence optics, the v7.20 calibrator, `PrintManager`). `workflow_settings_dialog.py`
and `camera_feed_view.py` both changed under me mid-edit; every edit here was
re-verified present afterwards.

### End-to-end smokes (not mocks)

* The real `WorkflowSettingsDialog` + real `SectionStack`: promote → the card's
  parent IS the stack, the popout loses a card, `collect()` still returns the
  promoted section's value, un-promote → it lands between its original
  neighbours, and the header is left with exactly its own controls again.
* The real `QuickPrintWorkflowPage`: default order unchanged, every built-in card
  still reachable, Readiness moved to the top and **restored on a fresh page**, a
  promoted section interleaved with the built-ins and restored, ↩ sending it home,
  and an embedded instance getting no host.
* **All seven pages** built for real: each registers a host, each can promote its
  first section and persist it, and each of the six drawer pages hides the drawer
  while empty and re-hides it when emptied.

---

## Needs GUI verification on ME3B V1, IN ORDER

The first two are go/no-go.

1. **Nothing looks different on launch.** Quick Print's Setup column reads Object
   → Queue → Print parameters → Readiness → status, exactly as before, and the six
   other workflow pages are unchanged (no drawer visible anywhere).
2. Open ⚙ Settings on Quick Print: **every section header now has an "On page"
   checkbox**. Tick one — the section leaves the popout and appears in the Setup
   column. Its fields still work, and **Save / load a profile still round-trips
   its values** (that is the identity-registration claim).
3. Press **▲ / ▼** on that card and on **Queue** and **Readiness** — the column
   reorders. ▲ is greyed on the top card and ▼ on the bottom one.
4. **Restart.** Both the promotion and the order came back.
5. Press **↩** on the promoted card (or untick the checkbox) — it returns to the
   popout **between the sections it was originally between**, not at the bottom.
6. **Load a different settings profile** and confirm the page does **not**
   rearrange (the arrangement is a machine preference, not part of the profile).
7. On a page whose column does not scroll — **Stress Test** is the clearest —
   promote three sections and confirm the drawer appears **below the run row**,
   scrolls internally, and that **Start / Abort never move**. Un-promote them all
   and the drawer disappears completely.
8. Open the **Print Calibrator** (which embeds Quick Print) and confirm the
   embedded surface has **no** "On page" checkboxes and no drawer.
9. Repeat step 2 briefly on spheroid / cell targeting / cell labeling /
   fluorescence mosaic / timing calibration to confirm each one's checkbox
   actually moves a section.
