# MEBP v7.5.x — Sketched prints visible in Quick Print + render in preview

## Objective

Fix two regressions where a print baked from the Print Builder → Sketch page (or
Image Import) via `PrintFileManager.save_trajectory_as_print_object` was not usable
by the operator until an app restart:

1. The new print did **not** appear in the Quick Print object dropdown until restart.
2. Loading the new print into Print Builder / Print Setup showed a **blank preview**.

## Root Causes

### A — Quick Print object list never refreshed during a session
`QuickPrintWorkflowPage._refresh_objects()` re-scans `config/prints/` (via
`PrintFileManager.list_files()`) on every call, but was only called at construction
and from the manual refresh button. The `print_file_created` signal is wired in
`gui/app.py::_on_print_created` only to the Print Setup tab — never to the Quick
Print page — so its combo stayed stale until a restart rebuilt the page.

### B — CSV pointer key mismatch (writer vs. readers)
`save_trajectory_as_print_object` wrote the trajectory CSV pointer under `csv_path`,
but every `csv_import` reader looked for `source_file`. The key never matched, so
`import_csv_trajectory(...)` was never called, `obj.trajectory` stayed empty, and the
preview / path rendered nothing. (Manual CSV import worked because it stores the
pointer under `source_file` — `print_objects.py:1983`.)

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PrintFileManager.py` | Writer now emits `source_file` (canonical key) plus a `csv_path` alias for back-compat. |
| `gui/pages/print_objects.py` | Reader (`_build_print_object`) accepts `source_file` **or** `csv_path`. |
| `gui/pages/print_setup_legacy.py` | Same reader fallback for the legacy path-extraction. |
| `gui/pages/workflows/quick_print_workflow.py` | Same reader fallback in `_obj_dict_to_path_points`; **new `showEvent`** re-scans prints each time the page is shown. |
| `tests/test_v75x_sketch_print_visibility.py` | New — writer-key, reader-fallback (both keys), end-to-end round-trip, showEvent-refresh. |

## Implementation Steps

- [x] Writer: key CSV pointer as `source_file` (+ `csv_path` alias).
- [x] Readers (3 sites): `params.get("source_file") or params.get("csv_path")`.
- [x] Quick Print: `showEvent` → `_refresh_objects()` (preserves selection via `findData`).
- [x] Tests: `tests/test_v75x_sketch_print_visibility.py` (9).

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_print_visibility tests.test_v75x_quick_print_workflow` → 25 passed.
- `tests.test_v75_migration` (9) + import-check of edited modules → green.
- Manual bench (pending): sketch → Send to Print Setup → preview renders without
  restart; open Quick Print (no restart) → new print listed and path previews;
  pre-existing sketch files (only `csv_path`) also preview (reader fallback).

## Issues & Decisions

- Chose a `showEvent` refresh over threading `print_file_created` into the workflows
  page: lower coupling, and it catches prints created from any source on every entry.
- Kept the `csv_path` alias in the writer so the JSON stays human-readable and any
  external tooling keyed on it still works; the functional key is `source_file`.
- Made readers accept **both** keys so sketch files already written to disk with only
  `csv_path` start previewing without a re-save / migration.
