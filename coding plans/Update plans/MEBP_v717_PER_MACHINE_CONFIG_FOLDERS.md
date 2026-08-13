# MEBP v7.17.x — Per-machine config folders (device profile = machine identity)

## Objective

Several physical rigs share one git repo. Per-machine state (camera/objective
calibration, taught plate positions, mosaics, needle-bore + focus calibration,
device profiles, print timing, LabLink node config) must never collide between
them; shared catalogues (needle/plate/well/target types, plate + rosette
designs, needle/syringe/camera catalogues, inserts, swappable `*Setup*.json`
hardware configs) must keep syncing.

Previously the split was a hand-maintained list of exact filenames in
`.gitignore`. That list had fallen behind — `config/hardware/well_training/`
(hundreds of per-scan mosaic PNGs + labels, growing every session),
`needle_bore_calibration.json`, `needle_focus_templates.json`,
`plate_level_sites.json`, `plate_templates.json`, `plate_focus_datum.json`,
`spheroid_*.json` were all **per-machine data tracked in git**. That is the
operator-reported cause of the device-specific pull conflicts.

The fix is structural rather than a longer list: every per-machine store lives
under `config/hardware/<machine-id>/`, so ONE `.gitignore` rule covers all of
them and a future store needs no `.gitignore` edit at all.

## Layout

```
config/hardware/
  devices/            *.json  — device profiles. TRACKED. Names the machine.
  ME3B_general/               — shared/portable catalogues. TRACKED.
  ME3B_01/                    — this rig's per-machine state. IGNORED.
  ME3B_02/                    — the other rig's. IGNORED.
```

## Status

### Done (landed + verified)

- [x] NEW `SupportClasses/MachineConfig.py` — `machine_id()`,
      `machine_config_dir()`, `shared_config_dir()`, `resolve_machine_path()`,
      `resolve_shared_path()`, `sweep_remaining_flat_files()`. Stdlib only, no
      Qt, no repo imports.
- [x] ~30 call sites rewired to `resolve_machine_path` / `resolve_shared_path`;
      each store's existing `MEBP_*_DIR` / `_PATH` env override preserved
      (they are read inside functions, so they still win).
- [x] Paired image dirs (`mosaics/`, `fluor_mosaics/`, `reanchor_features/`,
      `needle_focus_templates/`, `spheroid_training/`) migrate at **import**
      time, not lazily in `__init__` — a plain `import` must be enough for the
      migration tool. Test-supplied paths still keep their own sibling dir.
- [x] `.gitignore` rewritten to the structural rule. **Verified with real git**
      (`git ls-files`): 0 addable under `ME3B_01/`, 38 addable under
      `ME3B_general/`.
- [x] `tools_set_machine_id.py`, `tools_migrate_machine_config.py`.
- [x] Migration run on this rig; `ME3B_01/` + `ME3B_general/` populated, no
      loose `*.json` left at the `config/hardware/` root.
- [x] NEW `tests/test_v717_machine_config.py` — **15 tests, 3/3 mutations
      caught**.

### ⚠ Four invariants (do not relax — **6/6 mutations CAUGHT**)

1. **Nothing is MOVED while the machine id is unconfigured.** Paths resolve at
   *module-import* time and a store can be imported before the operator has
   said which rig this is — importing `gui.app` alone pulls in five of them
   (measured: objectives, needle-bore, fluorescence mosaics + its image dir,
   print timing). Migrating then files real calibration under a fallback
   bucket; once the operator answers, the app looks in `<their-id>/` and finds
   an empty folder — data on disk but invisible. **This already happened once
   on the bench.** So while unconfigured `resolve_machine_path` returns the
   LEGACY location and moves nothing.
2. **`main.py` imports `gui.app` only AFTER the first-run prompt.** Pinned by
   an AST test. ⚠ The first version of that test took the LAST `gui.app`
   import, so an added early one hid behind the correct later one — it
   SURVIVED its mutation. It now takes the earliest.
3. **The leftover sweep never files a per-machine store as shared.** "Shared"
   means committed and pushed to every rig, so the sweep is denied by name via
   `PER_MACHINE_FILENAMES` rather than trusting every store imported. Add new
   per-machine filenames to that set.

4. **Reserved machine names are refused.** A rig named `ME3B_general` or
   `devices` would write its private calibration into a shared, TRACKED folder
   and publish it to every other rig. Refused — and a profile with such a name
   reports as *unconfigured*, so the read-in-place fallback applies rather than
   the shared folder being used as a per-machine bucket.

🐞 Found by the new tests: `set_machine_id("..")` was accepted (`.` is in the
id charset), which would resolve the per-machine folder to `config/` itself.
Refused now.

## Device profile becomes the machine identity — DONE

Operator: *"the device profile should just be the pointer that allows us to
name the system and make a folder for all the calibration etc. We load in the
device profile on the hardware setup page."*

This removes a genuine two-homes-for-one-fact smell: `config/machine_id.txt`
and `settings.json → device_profile.active` both answered "which machine is
this". The device profile already names the rig and is already loaded on
Hardware Setup → Device (the mandatory Page 0), so it should be the only
answer.

**Decisions (operator):** device profiles are **TRACKED** in git — each rig's
file has a distinct name so they cannot conflict on pull, it backs up every
rig's envelope/steps-per-mm/axis map, and a rebuilt rig can be provisioned by
picking its profile; loading stays an explicit operator action. The first-run
popup is **kept, but only when there are NO device profiles on the machine
yet**, and should show the user around rather than just ask for a string.

- [x] `DEVICES_DIR` → `config/hardware/devices/` via new
      `MachineConfig.devices_dir()` (machine-INDEPENDENT). It cannot stay
      under `resolve_machine_path("devices")`: the profile now names that
      folder, so keeping it inside is a chicken-and-egg.
- [x] `MachineConfig.machine_id()` resolves from `settings.json →
      device_profile.active` (read from the FILE with `json` — MachineConfig
      stays stdlib-only and import-free of `Settings`), keeping
      `MEBP_MACHINE_ID` as the test/CI override. `config/machine_id.txt`
      retired and deleted; a test asserts the attribute is gone so a second
      home cannot creep back.
- [x] `main.py --settings other.json` now calls `set_settings_path()` BEFORE
      anything resolves a config path — otherwise one rig's settings would run
      against another rig's config folder.
- [x] **Reserved names refused** (`ME3B_general`, `devices`, case-insensitive)
      plus `.`/`..` and path separators. Never many-to-one sanitized.
- [x] Charset widened to allow spaces / `()+` — real profiles are named like
      `ME3B V1`.
- [x] This rig's profile renamed `081126` → `ME3B_01`, moved to
      `config/hardware/devices/ME3B_01.json`, and set active.
- [x] Save-As **asks** whether this is a rename (calibration folder follows,
      `rename_machine_folder`) or a different machine (fresh folder); refuses
      to merge onto an existing machine's folder. Both Save-As and Load show a
      **restart notice**, because stores resolve their paths once at import.
- [x] Onboarding wizard triggers when `list_profiles()` is empty (a machine
      with no profile has no identity yet) and lands the operator on the
      **Device** sub-page rather than Pump. The redundant `main.py` popup is
      gone; `_ensure_machine_id` now only logs which bucket is in use — it
      stays a named function because the AST ordering guard depends on it.
- [x] `.gitignore`: `!config/hardware/devices/` + `/**`; `machine_id.txt` line
      dropped. Verified with `git ls-files`.
- [x] `tools_set_machine_id.py` writes `device_profile.active` (load-modify-
      save, atomic) and warns when no profile of that name exists yet.
- [x] `tests/test_v717_machine_config.py` → **25 tests**, and CLAUDE.md's
      section rewritten.

## Issues & decisions

- **Two corrupt files found and repaired**, both pre-existing git conflict
  markers: `plt_9d77efd71695.json` (would have been committed into the SHARED
  folder and distributed to every rig) and `plate_templates.json` — the latter
  unparseable, so **all 6 plate templates were being silently lost on every
  load**. Each resolved by its own embedded timestamp; note the newer side was
  `Stashed changes` in one file and `Updated upstream` in the other, so there
  is no consistent side to pick. Backup: `*.bak-conflict`.
- **Five stale hardcoded paths fixed** (`Settings.py:298` device profile,
  `tools_regen_plate_orientation.py`, and 4 test sites). The `Settings.py` one
  was missed by the original sweep because the path is built from separate
  string literals (`/ "config" / "hardware" /`), not a `"config/hardware"`
  substring.
- **Real-data tests were silently vacuous.** Several globbed the old flat root
  and iterated zero files while passing — including the byte-identity
  guarantee over on-disk setups. Fixed; the mosaic ones additionally still read
  the pre-v7.13 flat store shape (`m["image"]`) instead of the nested
  `scans` container, so they had been skipping since v7.13.

## Pre-existing findings surfaced (NOT from this work)

- **This rig's `nest-plastic-24` scan detects only 7/24 wells** (tuned scan:
  24/24 at 1410 tiles / 3000×1944; on-disk now: 1040 tiles / 3000×1937,
  2026-08-11). It also carries **no `plate_frame`**, so per v7.13 it cannot
  follow a plate re-teach. The 24 mapped well centres in the store are intact.
  Recommend a re-scan. Left failing rather than weakened — the test had been
  skipping since v7.13 and only now runs.
- `config/CMH Bioprinting Setup.json` sits at the `config/` root, which the
  Hardware Setup page has never scanned (it lists the setups dir only). Moving
  it into `ME3B_general/` would make it visible again.

## Testing notes

- `python -m unittest tests.test_v717_machine_config` — 15 tests.
- Regression run per-suite (the documented cross-suite camera-probe hang makes
  a single combined run unreliable): ~640 green across capillary-needle /
  multibore / target-types / plate-types / well-type-presets / camera-cal-store
  / lablink-service / nikon-ti / last-known-calibration / plate-design /
  rosette-flatten / reanchor-guard.
- Verify `.gitignore` with `git ls-files -o --exclude-standard` on both folders
  (NOT `git check-ignore -v`, whose `!`-prefixed output means *not* ignored and
  is easy to misread).

## Bench verification (ME3B_01, after the device-profile change)

1. Launch with no device profiles → the onboarding wizard appears.
2. Create/load a profile named `ME3B_01`; confirm calibration still loads
   (needle location, plate map, camera µm/px) — nothing should look reset.
3. Restart; confirm the profile is remembered and no migration re-runs.
4. On rig 2: pull, load/create `ME3B_02`, confirm it gets its own empty folder
   and does NOT see rig 1's calibration.
5. `git status` on both rigs stays clean of per-machine files.
