# MEBP v7.9.1 — Save Setup lands in the folder the Saved-setups list reads

## Objective

Operator: *"on the identity tab there is a saved setups selection to load a saved
setup. this saved setup block auto loads all the setups in a folder. when i hit
save setup, it opens a folder for me to save into and that is not the right
folder. I want to save into the same folder that the setups selection block
automatically reads from."*

Make Save write to `config/hardware/<Name>.json` — the exact files
`_refresh_setup_list()` globs — so a saved setup is immediately loadable from the
list on the same tab.

## Root cause

v7.16 already routed both dialogs through `_setup_dir()` (= `CONFIG_HARDWARE_DIR`),
but that only supplied the file dialog's **default** path. That is not enough:

- The Windows native save dialog re-opens at the folder it was last used in, so
  once the operator has saved to the repo root the dialog keeps returning there.
- A default is a suggestion. The Saved-setups list scans exactly one folder, so a
  setup saved anywhere else is invisible to the page meant to load it — a silent
  failure (the save reports success; the setup just never appears).

Confirmed on this machine: **`config/hardware` contains zero setup files** and the
Identity list builds with `0` rows, while **nine** real setups sit in the repo
root (`Alexs Setup.json`, `Standard Bioprinting Setup*.json`, `24_nest_rosette.json`,
… — two of them saved 2026-08-10). Exactly the litter the v7.16 note predicted.

## Fix

The destination is **derived, not chosen**.

| File | Change |
|---|---|
| `gui/pages/hardware_setup.py` | New `_setup_path_for_name(name)` → `<setups folder>/<name>.json`, sanitising path characters so a typed name can never escape the folder (`../..`, `a/b`, `C:\…`) and falling back to `Untitled Setup` on a blank/dot-only name. New `_write_config_to(path)` shared writer (save + both browser refreshes). `_save_config` no longer opens a chooser: it validates the Name, confirms an overwrite, and writes. New `_save_config_as` keeps the old dialog as an escape hatch (still defaulting to the setups folder) and its tooltip states the consequence — a file saved elsewhere will not appear in the list. Button relabelled **Save Setup** (matches the list it feeds) + a **Save As…** beside it. `import re`. |
| `tests/test_v716_tucsen_mosaic_fov_and_intensity.py` | `TestSetupFolderDefault` extended to the stronger contract. |

Save is blocked on an empty Name with an explanation, because the name *is* the
filename and the list entry — an "Untitled Setup.json" nobody meant to create is
the same invisibility problem by another route.

## Implementation steps

- [x] `_setup_path_for_name` + sanitisation
- [x] `_write_config_to` shared writer
- [x] `_save_config` derives its path, confirms overwrite, refuses a blank name
- [x] `_save_config_as` escape hatch
- [x] Buttons relabelled/wired, tooltips state where the file lands
- [x] Tests updated + extended
- [ ] Operator decision: relocate the nine root setups into `config/hardware`
      (not done unilaterally — it is their data; the deleted `config/hardware`
      copies are older and still in git, the root copies are the live ones)

## Testing notes

`TestSetupFolderDefault` (5): the save path resolves through
`_setup_path_for_name` → `_setup_dir`; **`_save_config` contains no
`QFileDialog`** (the mutation that reintroduces the bug); the saved file's parent
is `CONFIG_HARDWARE_DIR` and its name is `<Name>.json`; and a name carrying path
characters still lands inside the folder. Load still defaults its dialog there.

Regression green: `test_v716_tucsen_mosaic_fov_and_intensity` ·
`test_v79_hardware_config_bores` · `test_v75x_camera_hardware_controls` ·
`test_test_suite_hygiene` (70) + an offscreen build of the real
`HardwareSetupPage` (page builds, `_setup_dir` = `C:\dev\MEBP\config\hardware`).

⚠ Pre-existing failure, NOT from this change:
`test_v79_needle_form_ui::test_every_on_disk_setup_round_trips_through_the_page`
("only 0 on-disk setup(s) exercised") — it globs `config/hardware/*.json` for
setups and the working tree has had all six deleted (`git status` shows `D`).
This diff deletes no files; restoring the setups to that folder fixes it.

## Issues & decisions

- **Why not just fix the dialog default again** — it was already correct. A
  default cannot guarantee the destination, and the failure is silent. Deriving
  the path removes the whole class.
- **Save As… kept** — saving a copy elsewhere is legitimate; it just must not be
  the default, and the tooltip says what it costs.
- **The root setups were left in place.** Moving nine files the operator has been
  actively saving to is their call, and the older same-named copies still in git
  make a blind move ambiguous.

## Needs GUI verification on ME3B V1

Type a Name → **Save Setup** → no folder chooser, and the setup appears in the
Saved setups list on that same tab immediately → **Load Setup** applies it →
restart and it is still listed → **Save As…** still opens a chooser, defaulting
to `config/hardware`.
