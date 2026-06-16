# MEBP v7.5.x — Hardware Setup: split "Pumps & Inks" into Pump / Needle / Ink tabs

## Objective

During hardware setup the user wanted three distinct, logically-ordered sub-tabs:
**Pump → Needle → Ink**. Previously pump and ink configuration shared a single
**"Pumps & Inks"** sub-tab, with **Needle** after it. This update separates the Ink
Library onto its own tab and reorders the three tabs to Pump, Needle, Ink.

Purely a UI re-organization of existing widgets — no behavior, config schema, or data
flow changes. Pump↔ink and needle-channel↔pump linkages keep working because they
operate through the shared `HardwareConfig` object and cross-widget refresh methods,
independent of which tab a widget lives on.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/hardware_setup.py` | Add an `"inks"` sub-page scaffold; route the Ink Library group to it; re-register the three sub-tabs in order Pump, Needle, Ink. |

## Implementation Steps

- [x] Add `"inks"` key to the sub-page scaffold-creation loop (`_setup_ui`).
- [x] Add `"inks"` key to the trailing `addStretch()` loop.
- [x] Route the Ink Library `QGroupBox` to `self._sub_layouts["inks"]` (Pump Channels group stays on `"pumps_inks"`).
- [x] Re-register sub-pages: `Pump` (`droplet`, `pumps_inks` scroll) → `Needle` (`needle`) → `Ink` (`flask`, `inks` scroll).
- [x] Verify `_plate_sub_index` / `_rosette_sub_index` remain correct (computed via `len(self._sub_pages)` at registration time — self-adjust).
- [x] `python -m py_compile` passes.

Resulting tab order: Device, Identity, Plate, **Pump, Needle, Ink**, Rosette, Cameras, Xbox Controller.

## Testing Notes

- Launch `python main.py`, open **Hardware Setup**, confirm the sub-tab bar reads
  Device · Identity · Plate · **Pump · Needle · Ink** · Rosette · Cameras · Xbox Controller.
- **Pump** tab: 3 Pump Channel cards + pump-ink summary (no ink table).
- **Ink** tab: Ink Library table + Add/Edit/Remove (no pump cards).
- Cross-link: add an ink on the Ink tab → it appears in each pump's ink checklist on the
  Pump tab; checking an ink for a pump survives removing a *different* ink on the Ink tab.
- **Needle** tab: channel→pump combos still list only enabled pumps; validation works.
- Save Config → reload → pumps, inks, needle channel map round-trip correctly.

## Issues & Decisions

- Kept the internal dict key `"pumps_inks"` for the **Pump** tab container (now holds only
  the Pump Channels group) to minimize churn; added a new `"inks"` key for the Ink tab.
  The key is internal only — the user-facing label is "Pump".
- No linkage code changed: `_refresh_pump_ink_combos()` / `_refresh_pump_ink_exclusions()`,
  `set_ink_names()`, `_refresh_channel_map_pump_options()`, and `_rebuild_config()` all
  iterate the shared widgets/config and are tab-placement-agnostic.
