# MEBP v7.18 — Filter cubes become a pick-list with real bands

## Objective

Operator: *"lets make the filter cubes standard as drop downs that auto fill in
everything. the excitation and emission are actually ranges, but we are makeing a
single number. Lets look up the most common filter cubes and add them as a list
of selectable ones, the add a custom option that saves into our list if we need
it."*

Three things: a **pick-list** instead of free text, band **ranges** instead of a
single number, and a **save-as-custom** path that joins the list.

## Decisions (AskUserQuestion ×3, all recommended options taken)

1. **Center + bandwidth**, not min–max. A part engraved `470/40` is entered as
   470 and 40; the edges (450–490) are derived. Storing edges would make the
   operator do that arithmetic off the part every time — a transcription step
   that produces a wrong number nobody can spot later. FWHM is symmetric by
   definition, so nothing is lost.
2. **Both catalogues** — Nikon Ti block designations (taken from this body's own
   `FilterBlockNames.txt`, so the names match what the scope reports) *and* the
   common dye-named sets.
3. **Auto-fill, stamped `nominal`.** Catalogue wavelengths are nominal for the
   cube TYPE; real parts vary, and these numbers reach the LabLink sidecar where
   they select a deconvolution PSF. `MicroscopeConfigStore`'s own rule is *"leave
   blank if you do not know it — LabLink names a missing field, which is
   recoverable; a wrong one is not"*, so silent auto-fill would quietly break it.
   Every value now carries its provenance and the UI shows it.

## Files Modified

- **NEW `SupportClasses/FilterCubeStore.py`** — `FilterCube` + `FilterCubeStore`,
  modelled directly on `WellTypeStore` (builtin + user dirs, user shadows builtin
  by id, atomic writes, `get_store()` singleton, zero GUI deps). Adds
  `band_edges` / `format_band` / `clean_nm` / `clean_width_nm` /
  `clean_provenance`, and `FilterCube.optics_entry()` — the one place that
  translates a cube into the store's entry shape.
- **NEW `config/hardware/ME3B_general/filter_cubes/builtin/standard_cubes.json`**
  — 17 cubes. A builtin file may hold one object or a list, so the shipped
  catalogue is one readable file while a saved custom cube is its own
  `user/<id>.json`.
- `SupportClasses/MicroscopeConfigStore.py` — `_clean_optics` and
  `set_filter_optics` carry the additive `*_width_nm` / `dichroic_nm` /
  `provenance` / `cube_id` fields; new `clean_bandwidth` + `_MIN_BANDWIDTH_NM`.
- `gui/pages/hardware/microscope_setup_panel.py` — the filter table's name cell
  becomes an editable combo over the catalogue with a trailing **"＋ Save this
  slot as a new cube…"**; each band becomes center + width spins in one grid
  cell; a **Source** column shows provenance. New `_build_cube_combo`,
  `_populate_cube_combo`, `_catalogue`, `refresh_catalogue`, `_build_band_cell`,
  `_width_spin`, `_on_cube_activated`, `_apply_cube`, `_sync_provenance`,
  `mark_edited`, `_save_slot_as_cube`, `_row_optics`, and a `cube_saved` signal.
- **NEW `tests/test_v718_filter_cube_catalogue.py`** — 36 tests.

## Invariants that made this safe

- **The band CENTERS keep their original `excitation_nm` / `emission_nm` keys and
  meaning.** That is what `OpticsRegistry.resolve_slots` and the
  `lablink.imagejob/1` sidecar read (the sidecar contract carries ONE number per
  channel), so everything downstream is untouched. Widths, dichroic, provenance
  and cube id are purely additive.
- **A pre-v7.18 entry round-trips BYTE-IDENTICALLY** — pinned by
  `test_pre_v718_entry_round_trips_byte_identically`, and mutation-confirmed.
- **`provenance` ABSENT is a third state**, distinct from `nominal`: it means
  never recorded, which is every existing entry (hand-typed by the operator from
  their own knowledge). Stamping those `nominal` would both break the round-trip
  above and *understate* values that may well have come off a datasheet.
- **`entry["edit"]` stays a `QLineEdit`** — it is the combo's `lineEdit()` — so
  every existing caller and test keeps using `.text()` / `.setText()`. That is
  what let a whole-column rework land with the 129-test microscope suite
  unchanged.
- **Auto-fill is bound to `activated`, not `currentIndexChanged`**, so only an
  operator's pick fills a row. A programmatic rebuild or a `set_labels()` restore
  must never silently overwrite wavelengths entered by hand.
- **A width or dichroic with no center is dropped** — a width alone describes no
  band, and keeping it implies knowledge we lack. Likewise an entry with a
  `cube_id` but no band is dropped entirely.
- **Bright Field and Analyzer deliberately carry NO wavelengths.** There is no
  band to report and a fabricated one would reach the sidecar.
- **The name matcher is exact, never fuzzy** — the reason `OpticsRegistry`
  documents at length: TxRed and mCherry are separate cubes and a wrong match
  silently mislabels a channel.
- **`mark_edited` promotes `nominal` → `datasheet`, never → `measured`.** Nothing
  here measured anything.
- Objectives are untouched: the catalogue is a filter-cube concept and the
  nosepiece column stays free text (pinned by test).

## Testing Notes

`python -m unittest tests.test_v718_filter_cube_catalogue
tests.test_v75x_nikon_ti_microscope tests.test_v717_lablink_job
tests.test_v717_lablink_page tests.test_v716_nd3_export
tests.test_v711_objective_ladder tests.test_v711_microscope_focus_state_bug
tests.test_v75x_fluorescence_mosaic tests.test_test_suite_hygiene -q`
→ **376 green**, plus a clean `gui.app` import and an offscreen render of the
real panel against the operator's own config.

**10/10 mutations CAUGHT:** provenance stamped on legacy entries (the
round-trip) · width kept with no center · junk provenance promoted · zero-width
band invented · fuzzy substring matching · hand-edit claimed as measured ·
`commit` validating provenance as a wavelength · brightfield given fake bands ·
pick fills the name but not the bands · nominal shown without a warning.

⚠ **My first run of the junk-provenance mutation SURVIVED, and the mutation was
at fault, not the test** — the test imports `clean_provenance` by name, so
patching only the module attribute never reached the code under test. Re-run with
both bindings patched: CAUGHT. Exactly the partial-mutation trap this repo
already records ("a mutation that does not change behaviour proves nothing").
Also confirmed en route that `MicroscopeConfigStore` has its OWN independent
provenance guard, so that rule is defended in two places.

🐞 **A bug I introduced and the tests caught — as a HANG, not a failure.**
`commit()` validated *every* value in an optics entry as a wavelength. Once
entries carry `provenance` / `cube_id` strings, that check fails on a string,
fires a modal `QMessageBox` and blocks forever under offscreen Qt. Fixed to
validate only the named numeric fields, and — because a wedged suite is a
terrible regression signal — converted into two fast explicit assertions
(`test_commit_does_not_mistake_provenance_for_a_wavelength` plus a companion
proving the guard was not loosened into uselessness).

🐞 Also caught by test: `clean_bandwidth` initially accepted `0.04`, which is
precisely the µm-for-nm slip it exists to catch (40 nm written as 0.04 µm). Now
floored at `_MIN_BANDWIDTH_NM = 1.0`.

## Issues & Decisions

- ⚠ **The catalogue's numbers are NOMINAL and the code says so everywhere.** They
  are typical values for each cube type, not measurements of any specific part.
  That is why provenance exists, why the UI shows `⚠ nominal`, and why editing a
  value re-stamps it `datasheet`. **For publication work, confirm against your
  own filters' datasheets.**
- **Existing entries are left completely alone.** Verified against the operator's
  real config: their four cubes load with their original numbers, width `—` and
  `Source: —`. Their values look like *fluorophore peaks* (e.g. Cy5 640/685)
  rather than *filter bands* (catalogue: 620/60 and 700/75) — a real distinction
  worth their attention, but not one this change is entitled to overwrite.
  Picking a cube from the list is the explicit opt-in.
- **Objectives deliberately out of scope.** A nosepiece catalogue is a separate
  feature (the body already reports magnification/NA/WD for coded objectives, so
  it would duplicate a working source).
- **No migration.** Nothing on disk changes shape until the operator picks a cube.

## Needs GUI verification on ME3B V1

1. Hardware Setup → Microscope → Filter cubes shows a **drop-down** per slot with
   the catalogue, and your four existing names/numbers are unchanged with
   `Source: —`.
2. Pick **Cy5** on slot 4 → name, Ex 620/60, Em 700/75 fill in and Source reads
   **⚠ nominal** with an explanatory tooltip.
3. Edit one number → Source flips to **✓ datasheet**.
4. Save → reopen the page → the bands, widths and Source all come back, and the
   combo shows the cube it came from.
5. Type a name that is not in the list → it stays, and the numbers are not
   disturbed.
6. Set your own numbers, choose **＋ Save this slot as a new cube…**, name it →
   it appears in the pick-list on **every** slot, and after a restart.
7. Confirm a fluorescence scan and a LabLink export still carry the wavelengths
   (the centers) exactly as before.
