# MEBP v7.21.6 — a fluorescence capture is never overwritten

## Objective

Operator: *"when taking a mosaic fluorescent image. Does the new image (e.g.
A1-FITC taken today) over ride / replace any old image of the same well and
color (e.g. A1-FITC taken yesterday)?"* → **it did** → *"Change the fluorescence
mosaic name to include the date and time so no images get over written, even
with the same well, color and plate"* + *"transfer all of them in the …
ME3B_01 > fluor_mosaics folder"*.

## The defect

`FluorescenceMosaicStore.save_channel` built a **deterministic** filename
`f"{plate}_{well}_{channel}.png"` and assigned
`entry["channels"][channel] = <fresh dict>`. So a re-scan of the same
(plate, well, channel):

* `cv2.imwrite` **truncated** the previous PNG — pixels unrecoverable, and
* the whole metadata record (extent, µm/px, exposure, display levels, cube slot,
  date) was **replaced**, not versioned.

No prompt, no warning, no versioning layer. Note the asymmetry with the
brightfield `MosaicStore`, which v7.13 gave a `{active, scans:{id: …}}` container
so one plate can hold several scans — the fluorescence store never got that.

**The only reason any older capture survived on this rig is an accident:** the
machine id changed (`ME3B_2` → `ME3B_01`) on 2026-08-13, which forked the whole
per-machine folder and left a 33-image snapshot untouched in
`config/hardware/ME3B_2/`. Within `ME3B_01` every re-scan since then is gone.

## Changes

### 1. `SupportClasses/FluorescenceMosaicStore.py`

* **NEW `capture_stamp()` / `channel_image_name()`** — the ONE place a channel
  image's filename is formed, now
  `<plate>_<well>_<channel>_<YYYYMMDD-HHMMSS>.png`. The stamp is **last** so
  every existing glob/sort that groups by `{plate}_{well}_{channel}` keeps
  working and `ls` sorts a channel's captures chronologically.
* **NEW `_unique_image_name()`** — appends `-2`, `-3`, … when the stamped name
  already exists, making the no-overwrite guarantee **absolute** rather than
  merely likely (the stamp is second-resolution; a clock stepped backwards or two
  captures inside one second would otherwise still collide).
  ⚠ These are **two independent legs** and each is pinned by its own test: with
  the stamp removed, leg 2 still prevents the overwrite, so the two-file test
  cannot pin the stamp — `test_the_stamp_alone_separates_two_captures` does.
* **The superseded record is ARCHIVED, not discarded** — pushed onto the
  channel's `history` list (newest first) with `_archived()` stripping the nested
  key so the list stays **FLAT** (N archives cost N entries, not 2^N). Without
  this the old PNG would survive but be an orphan nothing points at.
* **`captured_at`** (ISO datetime) added; `date` still written for every
  pre-v7.21.6 reader.
* **NEW `list_history` / `history_count` / `add_history_entry` /
  `restore_history`** — `restore_history` is a **swap**, not a delete: the
  displaced capture goes into the history in its place, so it is reversible.
* **`attach_processed` derives its name from THIS capture's raw stem.** With a
  fixed `_proc.png` name a fresh raw stitch inherits the previous run's processed
  file — and the display overlays *prefer* the processed one, so the screen would
  show a stale image while the store said it had a new one.
* **`clear_channel(include_history=True)` / `clear_well`** unlink archived images
  too (else a delete leaves invisible disk usage); `include_history=False`
  promotes the next-newest instead.
* 🔴 **NEW `_reload_if_changed()` on every mutator — THE LOST-UPDATE FIX, and it
  is not theoretical.** The app holds one store for the whole session and
  `_save_meta` dumps the ENTIRE `_data` dict, so any edit made to the file from
  outside is silently reverted by the app's next capture. **Observed live on
  ME3B_01 during this very change:** the import completed at 17:16, the running
  app's genuine `plt_7863c425dc27 B2 DAPI` capture at 17:28 wrote the file back
  from its own startup-era memory, and all 33 imported records vanished. Safe
  because every mutator calls `_save_meta` immediately, so there is never
  unflushed in-memory state for a reload to discard. Pinned by an **AST test**
  that every method calling `_save_meta` also calls `_reload_if_changed`.
  ⚠ The read is **UNCONDITIONAL, not mtime-gated** — a first cut compared
  `st_mtime` against the last write's value, but an external edit landing inside
  the same filesystem tick then reads as "unchanged" and is lost anyway (a test
  caught it; mutation M13b pins the wrong version). ~20 kB of JSON against a
  multi-MB PNG write in the same call — nothing to optimise.

**The ACTIVE capture is still plain `channels[<name>]`**, which is what kept this
small: `load_channel_image` / `get_extent_um` / `get_shift_um` /
`composite_overlay` / the five workflow overlays / the ND3 export are all
**unchanged** (325 regression tests green with no edits).

### 2. NEW `tools_import_fluor_mosaics.py`

Folds another machine folder's mosaics into the live store:

* absent (plate, well, channel) → imported **ACTIVE** (visible in the app again)
* present → imported into that channel's **history** (the current capture stays
  active — importing an older image over a newer one is exactly the overwrite
  this release removes)
* `--dry-run`, `--history-only`, `--active-anyway`, `--into <machine>`
* Every file **COPIED** (never moved) under a stamped name derived from the
  source record's own date; the source folder is left untouched.
* **Idempotent** — records carry `imported_from` and a repeat run reports
  `SKIP-DONE`. This matters *because* the app can revert an import (above), so a
  re-run is likely.
* Backs up the destination metadata as `.bak-preimport-<stamp>` first.

## Migration performed on ME3B_01

| | before | after |
|---|---|---|
| active captures | 28 (+1 live at 17:28) | **56** |
| archived captures | 0 | **6** |
| PNGs on disk | 29 | **62** (370 MB) |
| broken pointers | — | **0** |

27 captures that were invisible to the app (A3/A4/A6 DAPI+FITC+BF, B3/B5/B6/C4/C5
Cy5+FITC, D4/D5, …) are visible again; the 6 collisions kept the newer capture
active with the ME3B_2 version archived. `config/hardware/ME3B_2/` untouched.

## Testing

* NEW `tests/test_v7216_fluor_capture_history.py` — **33 tests** driving the
  production store + the real importer via subprocess.
* **16/16 mutations CAUGHT**, sources restored byte-identically. The harness
  asserts the baseline is GREEN before starting and restores in a `finally`
  (this repo's recorded lesson — a stranded mutation once scored every later one
  against an already-red suite).
* Regression **325 green** (fluorescence ×5, nd3 export+container, fluor optics/
  panel/preset, mosaic orientation remap+adjust, suite hygiene) + `gui.app`
  import smoke + an offscreen overlay smoke through the **real**
  `JogWorkspaceView` on the restored wells.
* `test_v75x_plate_mosaic` run class-by-class excluding the documented
  `TestManualAlignPage` hang: **110 green**.
* One pre-existing failure PROVED not ours in a `git worktree` at committed HEAD:
  `test_v713_tucam_raw_parity::test_avg_request_serviced_by_plane_path`.
* ⚠ The heredoc `\n`-collapsing trap already recorded in CLAUDE.md bit again
  while editing the mutation harness — escape-heavy source was written with the
  file tools.

## Issues & decisions

* **Collisions keep the NEWER capture active.** Nothing is lost either way, and
  silently demoting today's scan would be its own surprise.
* **Restored captures overlay at their capture-time absolute stage µm.** If the
  plate has been re-seated or re-taught since, an old mosaic sits where it was
  taken, not where the plate is now — the same caveat that already applies to any
  older capture in this store, which has no re-anchor concept (unlike
  `MosaicStore`'s v7.13 `plate_frame`). Pixels correct; georeference as good as
  the calibration it was taken under.
* **No cap on history depth** — the operator's ask was explicitly "no images get
  overwritten". At ~6 MB/capture this is the operator's disk-space call;
  `clear_channel` is the pruning tool.
* **Deliberately NOT done:** a UI for browsing/restoring the archive. The store
  API is there (`list_history`/`restore_history`) but the Fluorescence Mosaic
  page has no affordance yet — a channel pill showing "3 earlier captures" with a
  restore menu is the natural follow-up and its own change.

## Needs GUI/HW verification on ME3B_01, IN ORDER

1. **Restart the app** — the running instance is on the old code, so its captures
   still overwrite *and* its whole-file save can revert the import again.
2. Open Fluorescence Mosaic and confirm `nest-plastic-24` A3/A4/A6 now show their
   restored DAPI/FITC/Bright-Field channels (go/no-go for the migration).
3. Re-scan a well that already has that channel — confirm a **second** PNG
   appears in `fluor_mosaics` with a new `YYYYMMDD-HHMMSS`, the old file is still
   there, and the app shows the NEW one.
4. Confirm the overlays on the restored wells land on the wells (if a restored
   mosaic sits off-plate, the plate has moved since — re-scan it).
5. `plt_7863c425dc27 B2 DAPI` (the live 17:28 capture) must still be present.
6. Run a post-processing pass and confirm the `_proc.png` name carries the raw
   stem, then re-scan and confirm the display shows the NEW raw, not the old
   processed copy.
