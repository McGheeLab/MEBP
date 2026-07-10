# MEBP v7.5.x — Edit any saved print from the Library (Sketch round-trip)

## Objective

From the Print Builder **Prints** (Library) tab, open **any** saved print back
into the **Sketch** editor, change it, and save — either overwriting the print
in place or saving a new one. (Operator: "go into all of the print sketches
overview and edit any of the prints.")

The core problem: baked prints are raw `csv_import` trajectories, not the vector
`Sketch` model — a bake isn't losslessly reversible. Solution (both confirmed
with the operator):
- **Embed the vector Sketch** in every sketch-created print (`params["sketch"]`)
  → prints made after this land back in the editor **losslessly**.
- **Best-effort import** for prints without one (older prints, image/CSV
  imports): split the baked toolpath into printing sub-paths → editable
  `region` shapes, so they can still be repositioned / rescaled / re-saved.
- **"Save changes"** overwrites the opened print; **"Send to Print Setup"**
  still saves a new incremented print.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SketchTrajectory.py` | New `regions_from_trajectory(traj)` → `list[SketchShape]`: split an Nx7 trajectory into contiguous **printing** runs (pump-delta > 0; flat pumps → one run; travel dropped so a re-bake won't extrude across it) → one `region` shape each; pump inferred from the dominant advancing pump. |
| `SupportClasses/PrintFileManager.py` | `save_trajectory_as_print_object(..., overwrite=False)`: `overwrite=True` uses the sanitized `base_name` as the file stem verbatim and **replaces** that print (the "save changes" path); default still auto-increments. |
| `gui/pages/print_library.py` | `_object_trajectory` → public `object_trajectory`. `PrintCard`: `edit_requested` signal + full-width **"✏ Edit in Sketch"** button + `mouseDoubleClickEvent`. `PrintLibraryPage.edit_print_requested` signal, wired from every card. |
| `gui/pages/print_builder_sketch.py` | `_editing_name` / `_prints_dir` state; `print_file_saved` signal; **"Save changes"** button (shown only while editing, labelled with the target). `_compile_for_export` + `_do_bake` (embeds `sketch` in `extra_params`, threads `_prints_dir`, `overwrite` param) shared by `_send_to_print_setup` (new) and `_save_changes` (overwrite). `load_print_for_edit(name, prints_dir=None)` → `_sketch_from_stored` (lossless) or `_sketch_from_trajectory` (best-effort) → `_begin_edit` / `_update_edit_ui`. `_to_float`/`_to_int` helpers. |
| `gui/pages/print_builder.py` | `library_page.edit_print_requested` → `_on_edit_print` (`sketch_page.load_print_for_edit` + `switch_to_sketch`); `sketch_page.print_file_saved` → `library.refresh`. |
| `gui/app.py` | `sketch_page.print_file_saved` → `_on_print_files_changed` (arg-dropping lambda) so the Quick/Full-Print combos refresh after an in-place save. |

## Design decisions

- **Embed the whole `Sketch.to_dict()`** in the baked print's object params
  (survives `PrintFileManager` load/save/duplicate/rename + migration, which
  all preserve `objects`). Both save paths embed it, so a best-effort import,
  once saved, becomes losslessly re-editable.
- **`_prints_dir` threading:** `load_print_for_edit` records the dir the print
  was read from; `_do_bake` writes back there. Default (None → `config/prints`)
  is unchanged for production; only tests pass a temp dir.
- **No Z drift:** the stored sketch keeps `z_start_mm` plate-relative; only the
  CSV is baked to zero-ref. Reload → same plate-relative z_start → re-bake
  re-derives. Best-effort re-seeds z_start from `z_above_plate_bottom_mm`.
- **Region shapes** (not polygons) for best-effort — they preserve an OPEN path
  (polygon would close it) and are group-move/scale-friendly.

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_edit_print` (15)
- Regression: sketch suites + library + quick-print = **117 green**.
- Adversarial review workflow (3 dims → verify).
- **Needs GUI verification on ME3B V1**: Prints tab → Edit in Sketch (or
  double-click) → edit → "Save changes" updates the same print; older/imported
  prints load as region shapes; "Send to Print Setup" still makes a new one.

## Status

- [x] `regions_from_trajectory` + `overwrite` backend
- [x] Embed vector Sketch on both save paths
- [x] `load_print_for_edit` (lossless + best-effort) + editing state + Save-changes
- [x] Library Edit button + signal
- [x] PrintBuilder + app.py wiring
- [x] Tests (19) + regression (130)
- [x] Adversarial review (3 dims → verify) → 3 real issues fixed
- [x] CLAUDE.md table + memory

## Review findings (3-dim workflow → verify) — all fixed

1. **Best-effort edit silently reset the print height** (medium): region shapes
   are XY-only, so a print without a `z_above_plate_bottom_mm` param re-baked at
   the 0.2 mm default. Fix: `_sketch_from_trajectory` now **recovers `z_start`
   from the baked toolpath** (lowest waypoint height via `zref_to_plate_relative`
   against the calibrated plate bottom).
2. **Overwrite orphaned the file when stem ≠ sanitize(display name)** (low):
   `_save_changes` re-sanitized the display name. Fix: track `_editing_stem`
   (the real on-disk stem from the resolved path) and overwrite that.
3. **Stuck in edit mode / stale Save target** (medium): `_editing_name` was
   never cleared and "Send to Print Setup" left it set → Save-changes could
   overwrite the wrong print. Fix: **"New sketch"** toolbar action clears edit
   mode; Send-as-new **retargets** the edit to the just-created print.

## Issues & Decisions

- An edit session persists (`_editing_name` stays) until another print is
  loaded — standard editor behaviour; the Save button always names its target
  ("Save changes to 'X'") so it can't silently overwrite the wrong print.
- Prints with no vector data AND no reconstructable toolpath (e.g. a parametric
  object with no needle configured) can't be edited → a clear status message
  directs the operator to Print Setup for parameter tweaks.
