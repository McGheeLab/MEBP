# MEBP v7.5.x — Print Library / Manager (Print Builder tab)

## Objective

Give the operator a place to **see all saved prints and manage them on disk**:
a new **"Prints"** tab in the Print Builder mode (sibling of Sketch / Image
Import / Hardware / Print Settings) that shows a **grid of every saved print**
with a small trajectory thumbnail + metadata, and per-print file operations:
**Rename, Duplicate, Delete**, plus **bulk delete**, an **orphaned-file
cleanup**, "Open prints folder", and "Open in Print Setup".

Motivated by an operator request ("I want to be able to delete any prints in
the system … a print viewer in the sketch page where a tab will show a grid of
all the prints, and we can delete them or keep them and rename them … that has
to do with filestructure stuff") and the fact that `config/prints/` has
accumulated junk: empty prints (`objects: {}`), orphaned `.csv` files with no
matching `.json`, and `.bak-v7.2.3` migration backups.

## Files Modified / Added

| File | Change |
|------|--------|
| `SupportClasses/PrintFileManager.py` | **delete()** now also removes the sibling `<stem>.csv` and any `<stem>.json.bak-*` backups (was JSON-only → orphaned CSVs). **New `rename(old, new)`** renames the `.json` + sibling `.csv`, rewrites the object `source_file`/`csv_path` pointers, and updates `metadata.name`. **New `orphan_files()` / `cleanup_orphans()`** detect+remove unreferenced `.csv` and `.bak-*` files. **New module helper `read_print_objects(path)`**. |
| `gui/pages/print_library.py` | **NEW.** `PrintThumbnail` (custom-painted XY toolpath preview, auto-fit), `PrintCard` (thumbnail + name + meta + select checkbox + Rename/Duplicate/Delete/Open buttons), `PrintLibraryPage` (`QScrollArea` reflowing grid of cards + header + bulk-delete bar + orphan-cleanup). Signals `print_file_created(str)` (Open in Print Setup) and `print_files_changed()`. |
| `gui/pages/print_builder.py` | Register the new `PrintLibraryPage` as a 5th sub-page ("Prints", `grid` icon); expose `library_page`; forward its two signals up. |
| `gui/app.py` | Wire `library_page.print_file_created → _on_print_created`; wire `library_page.print_files_changed → _on_print_files_changed` (best-effort refresh of the Quick Print objects combo + Full Print objects tab). |
| `tests/test_v75x_print_library.py` | **NEW.** Backend (delete-removes-csv, rename, orphan cleanup) + offscreen page build/refresh/delete smoke. |

## Design decisions

- **Placement = new Print Builder sub-page tab** (not a tab literally inside the
  Sketch canvas). Print Builder already presents its sub-pages as a vertical
  icon "tab" strip; a sibling "Prints" tab is the idiomatic reading of "a tab
  … that shows a grid of all the prints" and keeps the Sketch canvas untouched.
- **File operations, not editing.** The library manages files (rename /
  duplicate / delete / cleanup) + previews them; it does **not** re-open a baked
  trajectory back into the vector Sketch model (the Sketch model is vector
  shapes, a baked `csv_import` trajectory is raw waypoints — not round-trippable).
  A per-card **"Open in Print Setup"** re-uses the existing `print_file_created`
  flow so a print can still be *used*.
- **Delete cleans up the whole print.** A Sketch/Image print is a `.json` + a
  sibling `.csv`; deleting only the JSON (the old behaviour) left the CSV
  orphaned. Delete now removes both + any `.bak-*`.
- **Thumbnails** are built from each file's objects: `csv_import` → read the CSV
  (`import_csv_trajectory`); parametric → `GeometryEngine.generate_object_trajectory`
  with a default-needle fallback. Each object drawn as an auto-fit XY polyline.
  Empty prints show a "no preview" placeholder. All wrapped in try/except so one
  bad file never breaks the grid.

## Implementation Steps

- [x] Plan doc (this file)
- [x] `PrintFileManager.delete` removes sibling `.csv` + `.bak-*`
- [x] `PrintFileManager.rename(old, new)` (+ pointer rewrite)
- [x] `PrintFileManager.duplicate` copies the sibling `.csv` + rewrites pointers (standalone copy)
- [x] `PrintFileManager.orphan_files()` / `cleanup_orphans()`
- [x] `read_print_objects(path)` module helper
- [x] `gui/pages/print_library.py` (thumbnail + card + page)
- [x] Register the tab in `PrintBuilderPage` + expose/forward signals
- [x] Wire signals in `gui/app.py` (+ `quick_print_page` accessor)
- [x] **Regression fix:** `print_objects.py::_on_name_changed` rename switched from `save_as+delete` → `PrintFileManager.rename` (the old path stranded a csv_import print's CSV once delete started removing siblings)
- [x] Tests (`tests/test_v75x_print_library.py`, 18)
- [x] Update CLAUDE.md Existing Update Plans table + memory

## Testing Notes

- `python -m pytest tests/test_v75x_print_library.py -q`
- Regression: `tests/test_v75x_sketch_print_visibility.py` (writer/reader
  contract unchanged), Quick Print + print-setup suites.
- **Needs real-HW-independent GUI verification on ME3B V1**: Print Builder →
  Prints → grid renders thumbnails; Rename/Duplicate/Delete update disk +
  the grid; bulk delete; orphan cleanup; "Open in Print Setup" lands the print.

## Issues & Decisions

- Rename of a `csv_import` print must keep the JSON→CSV pointer valid: the CSV
  is renamed to the new stem and every `source_file`/`csv_path` that pointed at
  the old CSV basename is rewritten to the new path. Rename **fails** (UI warns)
  if the target name already exists — no silent overwrite.
- Duplicate/rename both keep the print **standalone**: `duplicate` copies the
  sibling CSV + rewrites the copy's pointers (so deleting the original — which
  now removes the original's sibling CSV — can't strand the copy). The library
  page enforces a unique duplicate name (`<name>_copy`, rejects an existing
  name) since the raw `PrintFileManager.duplicate` overwrites a same-named file.
- **Regression caught & fixed:** the Print Setup objects tab renamed via
  `save_as(new) + delete(old)`. Once `delete` also removed the sibling CSV, that
  path deleted the CSV the freshly-saved `new` file still pointed at. Switched
  it to `save() + rename(old, new)` (moves JSON+CSV, rewrites pointers).
- Verified against the live (messy) `config/prints/`: 39 prints render (32 with
  thumbnails; 7 empty/`dot` prints show a placeholder), 9 orphans detected
  (`.bak-v7.2.3` backups + an unreferenced CSV).
- Orphan definition: a `.csv` whose stem has no matching `.json` **and** which
  no `.json` references, plus `*.json.bak-*` backups. Conservative — a CSV still
  referenced by any print is never an orphan.
