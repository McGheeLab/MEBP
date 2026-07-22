# MEBP v7.5.x — Dwell/Prime max 30 s + Fluorescence Bright-Field channel

## Objective

Four operator-requested changes (2026-07-12):

1. **Dwell time max 10 s → 30 s** — raise the ceiling on the global pump
   **dwell** (`pump_settle_time_s`) spinbox.
2. **Prime time max 10 s → 30 s** — raise the ceiling on the global pump
   **prime** (`pump_prime_time_s`) spinbox.
3. **Fluorescence Mosaic — add a "Bright Field" filter cube** corresponding to
   the microscope's non-excitation channels. Operator chose: **one** "Bright
   Field" option mapped to **channel 5** only (channel 6 is also brightfield but
   is not surfaced as a separate option); default display pseudo-colour
   **grayscale / white**.
4. **Fluorescence Mosaic — show the channel number in the pre-scan prompt**,
   e.g. `Set the microscope filter / illumination for the DAPI channel (1),
   focus if needed, then click OK to scan.` (DAPI 1, FITC 2, mCherry 3, Cy5 4,
   Bright Field 5).

## Key decisions (operator, via AskUserQuestion)

- **Bright Field = one pill, channel (5) only** (not two pills, not (5, 6)).
- **Bright Field colour = grayscale / white** — rendered as a true grayscale of
  the brightfield intensity in the additive composite (`(255, 255, 255)`); the
  operator can still recolour it by double-clicking the pill.
- **Dwell/prime max change = both locations** — the value is a single
  `HardwareConfig` field edited from several UI surfaces; every surface must use
  the same ceiling or one clamps a >10 s value back to 10 s. This required
  updating **six** spinbox definitions, not just the two the operator named
  (the four prep-workflow popouts also expose the same globals).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/FluorescenceMosaicStore.py` | `CHANNELS` gains `"Bright Field"`; `DEFAULT_CHANNEL_COLORS["Bright Field"] = (255,255,255)`; new `CHANNEL_NUMBERS` dict + `channel_number()` helper (1-based microscope channel numbers). |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | `_prompt_next_channel` appends the channel number to the "Set filter" prompt via `fms.channel_number()` (omits `(n)` if unknown). Pills auto-include Bright Field (loop over `fms.CHANNELS`). |
| `gui/pages/hardware/hardware_setup.py` | `_pump_settle_spin` / `_pump_prime_spin` ranges `0.0–10.0` → `0.0–30.0`. |
| `gui/pages/workflows/common_print_settings_workflow.py` | Dwell + Prime rows `_dspin(0.0, 10.0, …)` → `0.0, 30.0`. |
| `gui/pages/workflows/quick_print_workflow.py` | `_g_settle` (Common — Pump global) max → 30. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | `_g_settle` / `_g_prime` max → 30. |
| `gui/pages/workflows/cell_targeting_workflow.py` | `_g_settle` / `_g_prime` max → 30. |
| `gui/pages/workflows/cell_labeling_workflow.py` | `_g_settle` / `_g_prime` max → 30. |
| `tests/test_v75x_fluorescence_mosaic.py` | Updated `CHANNELS`/`_selected_channels` assertions; new `test_bright_field_channel` + `test_channel_numbers`. |

## Implementation Steps

- [x] Add `"Bright Field"` to `CHANNELS` + default grayscale/white colour.
- [x] Add `CHANNEL_NUMBERS` + `channel_number()` helper.
- [x] Prompt shows the channel number (`… channel (n), …`).
- [x] Bump dwell/prime spinbox max to 30 s at all six edit surfaces.
- [x] Update/extend tests.

## Testing Notes

- `python -m unittest tests.test_v75x_fluorescence_mosaic` → 35 pass.
- `python -m unittest tests.test_v75x_common_print_settings
  tests.test_v75x_workflow_settings_popout
  tests.test_v75x_pump_settle_and_prime_time` → 92 pass.
- Store sanity check: `CHANNELS` includes Bright Field; `channel_number` returns
  1–5 for the named channels and `None` for unknown; Bright Field colour
  `(255,255,255)`.

**Needs GUI verification on ME3B V1:**
- Common Print Settings + Hardware Setup → Pump: Dwell and Prime spinboxes now
  accept up to 30 s; a 30 s value set on one page shows 30 s on the other and in
  every prep-workflow popout (no clamp back to 10).
- Fluorescence Mosaic: a "Bright Field" filter-cube pill appears (grayscale/white
  swatch); selecting it and running shows the prompt "… for the Bright Field
  channel (5), …"; the fluorescence prompts show (1)/(2)/(3)/(4) respectively.

## Issues & Decisions

- The dwell/prime value has **six** UI edit points (Common Print Settings, the
  Hardware Setup Pump page, and the "Common — Pump (global)" section of the
  Spheroid / Cell Targeting / Cell Labeling / Quick Print popouts). All six were
  bumped so no surface clamps the value back to 10 s. Backend coercion on
  `HardwareConfig` is non-negative-only (no upper cap), so 30 s persists cleanly.
- Bright Field's channel 6 is intentionally not offered (operator choice). If a
  second brightfield position is wanted later, add a `"Bright Field 6"` entry to
  `CHANNELS` / `CHANNEL_NUMBERS` — the pill/prompt/store paths are name-keyed and
  need no other change.
