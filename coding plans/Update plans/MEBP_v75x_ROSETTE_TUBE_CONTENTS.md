# MEBP v7.5.x — Rosette Sub-Well Contents (Eppendorf tubes vs. silicon-mold wells)

Status: **PLANNING** · Needs real-HW verification on ME3B V1.

---

## Objective

When an operator builds a **rosette**, let them declare *what is physically in
each sub-well* — either an **open silicon-mold well** or an **Eppendorf-style
tube** (the motivating case: 0.1 mL tube, **Ø ≈ 6.5 mm, height ≈ 16 mm**). The
tube case is **safety-critical**: a 16 mm-tall tube changes the **safe-travel Z**
(the needle must clear the tube tops when moving across the plate) and the tube's
narrow, deep bore changes **needle reach / fit**.

Operator-chosen scope (decided 2026-06-23):

1. **Capture point — in the rosette designer.** The "contents kind" is treated as
   *geometry* (it sets opening Ø, rim height, dispense Z, bore). Reagent/ink
   assignment stays where it is today (Hardware Setup → Ink → Reagent Locations,
   `well_reagent_map`).
2. **Granularity — whole-rosette default + per-sub-well override.** Pick one kind
   for the whole rosette ("all sub-wells are Eppendorf 0.1 mL"), with optional
   per-sub-well override.
3. **Geometry — a reusable tube-type library** (`config/hardware/tube_types/*.json`,
   mirroring the existing standard-insert library), seeded with an **Eppendorf
   0.1 mL** preset; editable for other sizes.
4. **Validation — clearance + reach/bore checks.** Drive safe-travel Z from tube
   height AND validate the needle can (a) reach the tube bottom and (b) fit the
   inner bore; warn before a run if not.

### What already exists (do NOT rebuild)

The safe-travel-clearance pipeline is **already wired** and polarity-safe — the
new work mostly *populates* it:

- `WellInfo.rim_height_mm` (height the well/tube top sits **above** the plate top)
  and `WellInfo.ink_z_mm` (dispense Z relative to plate top) already exist and are
  carried through `PlateDesign.compile()` → flattened sub-wells
  (`SupportClasses/PlateDesign.py:645`, `SupportClasses/WellPlate.py:149`).
- `WellPlate.max_rim_height_mm` → `gui/app.py::_update_insert_clearance`
  (`plate_top_z + max_rim + INSERT_CLEARANCE_MARGIN_MM` = 3.0 mm) →
  `StageController.set_min_travel_z(...)` floors **every** `safe_travel_to` /
  `ensure_retracted_to` retract in the **height frame** (correct on ME3B V1
  `ZDIR=-1`). See `SupportClasses/StageController.py:3677` / `:3540`.
- The rosette designer already edits `rim_height_mm` / `ink_z_mm` per sub-well
  and supports a standard-insert library
  (`gui/pages/hardware/plate_designer.py:1223`, `:1754`;
  `SupportClasses/PlateDesign.py:55-86`).

So once a tube preset sets `rim_height_mm` ≈ tube protrusion above plate, the
needle automatically clears the tubes during travel — **provided plate-top Z is
calibrated** (else `min_travel_z` is `None` = no extra floor). Surfacing that gap
is part of this work.

---

## Design

### 1. New: tube/insert-type library

New module `SupportClasses/TubeTypeLibrary.py`, modelled on the standard-insert
library functions in `PlateDesign.py` (`INSERTS_DIR` / `list_/load_/save_`):

```python
@dataclass
class WellContentSpec:
    name: str                      # "Eppendorf 0.1 mL", "Silicon mold (open)"
    kind: str = "open"             # "tube" | "mold" | "open"
    opening_diameter_mm: float     # footprint / opening Ø  → WellInfo.diameter (≈6.5)
    inner_bore_mm: float = 0.0     # usable inner Ø for needle fit (≈5.0; 0 = same as opening)
    total_height_mm: float = 0.0   # full tube height (≈16; informational + reach)
    rim_height_mm: float = 0.0     # protrusion ABOVE plate top → travel clearance
    well_depth_mm: float = 0.0     # reachable depth from opening to bottom (dispense)
    dispense_z_mm: float | None = None  # → WellInfo.ink_z_mm (rel. plate top; neg = into tube)
    cone_bottom: bool = False      # Eppendorf conical bottom (informational)
    description: str = ""
```

- Directory `config/hardware/tube_types/` (auto-created).
- `list_tube_types()` / `load_tube_type(name)` / `save_tube_type(spec, name)` +
  `seed_default_tube_types()` (writes the **Eppendorf 0.1 mL** + **Silicon mold
  (open)** presets on first use if the dir is empty).
- Eppendorf 0.1 mL defaults (operator's numbers; editable):
  `kind="tube", opening_diameter_mm=6.5, inner_bore_mm=5.0, total_height_mm=16.0,
  rim_height_mm=12.0` *(protrusion above plate — depends on the insert; default
  conservative)*, `well_depth_mm=15.0, dispense_z_mm=-14.0, cone_bottom=True`.
- Silicon mold (open): `kind="mold", opening_diameter_mm=2.0, rim_height_mm=0.0,
  well_depth_mm=2.0, dispense_z_mm=None` (flush, shallow → no travel impact).

### 2. Sub-well "contents kind" on the model

Add to `PlateDesign.Well` (and mirror on `WellPlate.WellInfo`):

```python
content_type: str = ""    # tube-type preset name applied (display + re-apply)
content_kind: str = ""    # "tube" | "mold" | "open" | "" (geometry discriminator)
inner_bore_mm: float = 0.0  # usable inner Ø for needle-fit validation (0 = use diameter)
```

- Extend `_entity_to_dict` / `_entity_from_dict` (`PlateDesign.py:930` / `:957`)
  to round-trip the three fields (back-compat: missing keys → defaults).
- `compile()` (`PlateDesign.py:690`) carries them onto the flattened `WellInfo`
  alongside the existing `rim_height_mm` / `ink_z_mm`.
- **Applying a preset = copy spec → sub-well fields**: `diameter ←
  opening_diameter_mm`, `rim_height_mm ← rim_height_mm`, `ink_z_mm ←
  dispense_z_mm`, `well_depth_mm ← well_depth_mm`, `inner_bore_mm ←
  inner_bore_mm`, `content_type ← name`, `content_kind ← kind`. The existing
  `rim_height_mm`/`ink_z_mm` manual spinboxes remain editable as overrides.

### 3. Designer UI (`gui/pages/hardware/plate_designer.py`)

- **Whole-rosette default** — in the rosette breadcrumb bar (near the preset combo
  at `:317`), add an **"All sub-well contents:"** combo populated from
  `list_tube_types()` + "(custom)". Choosing a type applies that preset to **all**
  sub-wells of the current rosette, then refreshes the canvas + properties.
- **Per-sub-well override** — in the sub-well properties panel (`:1223`, where Rim
  height / Ink Z already live), add a **"Contents:"** combo (same list, default =
  the rosette default). Selecting a preset auto-fills the geometry spinboxes for
  that sub-well; "(custom)" leaves them as hand-entered.
- **Manage tube types** — a "Manage tube types…" / "Save current as tube type…"
  button → small dialog backed by the library (add/edit/delete; seeded with
  Eppendorf 0.1 mL). Reuse the `QInputDialog`-style flow used by the standard
  insert drop (`:1754`) for the minimal version.
- **Advisory validation** — show a non-blocking ⚠ label in the rosette editor when
  the current needle can't fit/reach any tube sub-well (see §4), and a hint when a
  plate has tubes but plate-top Z is uncalibrated (clearance won't apply).

### 4. Needle reach / bore-fit validation

New helper (in `TubeTypeLibrary.py` or `PhysicalModels.py`):

```python
def tube_fit_warnings(well_info, needle: NeedleSpec) -> list[str]:
    """Advisory warnings for a tube/insert sub-well vs the active needle."""
```

Checks (use `NeedleSpec.od_mm`, `.id_mm`, `.length_mm` —
`PhysicalModels.py:224-272`):

- **Bore fit:** `needle.od_mm + margin > (well_info.inner_bore_mm or diameter)` →
  "needle Ø {od} mm too wide for tube bore {bore} mm".
- **Reach (needle length):** the exposed needle must span (tube protrusion above
  plate) + (descent below plate top to the dispense Z): roughly
  `needle.length_mm < rim_height_mm + max(0, -ink_z_mm)` → "needle may be too short
  to reach the tube bottom (chuck/holder could hit the tube rim)".
- **Z-range reach:** the dispense Z (`controller.print_height_to_zref(...)` of the
  target) must lie within the Z soft-limit envelope.

Wiring:
- **Designer:** advisory only (warning label; never blocks editing).
- **Pre-run gate:** in the Quick Print / workflow start paths that already gate on
  bore + Safe Z + calibrated plate bottom, add a tube-fit check for any rosette
  sub-well the run will touch — warn-and-confirm (consistent with existing
  start-gate patterns), not a hard refuse, since the operator may know better.

### 5. Safe travel — confirm + guard (mostly already done)

- Eppendorf preset sets `rim_height_mm` → `max_rim_height_mm` → `min_travel_z`
  already floors travel retracts. **No new motion math.**
- **Guard:** in `gui/app.py::_update_insert_clearance` (`:1485`), when
  `max_rim_height_mm > 0` but `plate_top_z` is `None` (uncalibrated), log + surface
  a warning that tube clearance is NOT being applied (currently it silently sets
  `min_travel_z(None)`). This is the one real gap for the tube case.

---

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/TubeTypeLibrary.py` (**new**) | `WellContentSpec` + library load/save/list + `seed_default_tube_types()` + `tube_fit_warnings()`. |
| `SupportClasses/PlateDesign.py` | Add `content_type`/`content_kind`/`inner_bore_mm` to `Well`; extend `_entity_to_dict`/`_entity_from_dict`; carry through `compile()`. |
| `SupportClasses/WellPlate.py` | Mirror the three fields on `WellInfo` (carried by `from_wells`). |
| `gui/pages/hardware/plate_designer.py` | Whole-rosette contents combo + per-sub-well override combo + apply-preset + Manage-tube-types + advisory ⚠. |
| `gui/app.py` | `_update_insert_clearance` guard: warn when tubes present but plate-top Z uncalibrated. |
| Quick Print / workflow start gates (TBD which) | Tube reach/bore warn-and-confirm for touched rosette sub-wells. |
| `config/hardware/tube_types/Eppendorf 0.1 mL.json`, `Silicon mold (open).json` (**new seed**) | Shipped presets. |
| `tests/test_v75x_rosette_tube_contents.py` (**new**) | Library round-trip, preset→sub-well apply, compile() carries fields, `max_rim_height_mm`→clearance for a tube rosette, `tube_fit_warnings` bore/reach, serialization back-compat. |

---

## Implementation Steps

- [ ] 1. `TubeTypeLibrary.py`: `WellContentSpec` dataclass + `to_dict`/`from_dict`.
- [ ] 2. Library fns (`TUBE_TYPES_DIR`, `list_/load_/save_`, `seed_default_tube_types`) + Eppendorf 0.1 mL & mold presets.
- [ ] 3. `tube_fit_warnings(well_info, needle)` helper (bore + reach + Z-range).
- [ ] 4. `Well` (+ serialization) gains `content_type`/`content_kind`/`inner_bore_mm`; back-compat defaults.
- [ ] 5. `WellInfo` mirrors the three fields; `compile()` carries them onto flattened sub-wells.
- [ ] 6. Designer: whole-rosette "All sub-well contents" combo + apply-to-all.
- [ ] 7. Designer: per-sub-well "Contents" override combo + auto-fill geometry.
- [ ] 8. Designer: "Manage tube types…" dialog + advisory ⚠ (fit + uncalibrated-top hint).
- [ ] 9. `app.py::_update_insert_clearance` guard (tubes present + plate-top Z `None` → warn).
- [ ] 10. Pre-run tube-fit warn-and-confirm in the relevant start gate(s).
- [ ] 11. Tests (`test_v75x_rosette_tube_contents.py`) + keep `test_v748_rosette_flatten.py` / `test_v747_plate_designer.py` / `test_v745_plate_design.py` / ink-location suites green.
- [ ] 12. Real-HW verification on ME3B V1 (checklist below).

---

## Testing Notes

Automated:
- Tube-type JSON round-trip; `seed_default_tube_types()` is idempotent.
- Apply Eppendorf preset to a rosette → every sub-well has `diameter≈6.5`,
  `rim_height_mm≈12`, `ink_z_mm≈-14`, `content_kind="tube"`.
- `PlateDesign.compile()` carries `content_kind`/`content_type`/`inner_bore_mm`
  onto flattened `WellInfo` (e.g. `A1.a`).
- A rosette of tubes → `WellPlate.max_rim_height_mm ≈ 12` → with a known
  `plate_top_z`, `_update_insert_clearance` computes `min_travel_z = top + 12 + 3`.
- `tube_fit_warnings`: a wide needle (od ≥ bore) warns; a short needle (length <
  rim+depth) warns; a fitting needle is clean.
- Back-compat: an OLD insert/design JSON (no new keys) loads with defaults.

Real-HW (ME3B V1) — **must verify**:
- Build a rosette of Eppendorf 0.1 mL tubes; confirm Safe-Travel retract visibly
  rises so the needle clears the 16 mm tubes when hopping between sub-wells.
- Confirm dispense descends into a tube to the intended depth without the
  holder striking the tube rim; confirm bore fit.
- Confirm the uncalibrated-plate-top warning appears when expected.

---

## Issues & Decisions

- **2026-06-23 — Scope split.** Contents *geometry* (tube/mold) lives in the
  designer; contents *reagent* stays in Hardware Setup. Avoids duplicating the
  `well_reagent_map` flow.
- **2026-06-23 — Reuse, don't rebuild.** The `rim_height_mm → max_rim_height_mm →
  set_min_travel_z` travel-clearance chain already exists and is polarity-safe; a
  tube preset simply populates `rim_height_mm`. The only real travel gap is the
  silent no-op when plate-top Z is uncalibrated → add a guard/warning.
- **Open — `rim_height_mm` for a tube is insert-dependent.** It is the tube's
  protrusion *above the plate top*, which depends on how deep the insert seats the
  tube. The preset ships a conservative default (12 mm); the operator confirms per
  physical insert. Consider deriving it from `total_height_mm − seated_depth` if a
  `seated_depth_mm` is added to the insert model later.
- **Open — which start gate(s) get the reach/bore confirm.** Confirm whether the
  rosette tube case runs through Quick Print, Print Setup, and/or the workflows
  (spheroid / cell targeting / labeling); add the gate to whichever the operator
  uses for tube rosettes.

---

## On completion (per CLAUDE.md checklist)

- Add this plan to the **Existing Update Plans** table in `CLAUDE.md`.
- Refresh the architecture doc if the data model changes land.
- Mark all steps `[x]`; record HW-verification outcome.
