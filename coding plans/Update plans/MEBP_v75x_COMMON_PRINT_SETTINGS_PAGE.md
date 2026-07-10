# MEBP v7.5.x — Common Print Settings page (Workflows mode)

## Objective

Operator request: a central **Common Print Settings** page in Workflows mode
that collects every setting common to all workflows — the hardware-setup-level
globals (pump settle/dwell time, pressure-relief volume, prime time) **and** the
needle-prep knobs every workflow currently duplicates (service/dip Z, prep flow
rate, oil/buffer needle counts, wash cycles + amplitudes). These same values
stay visible/editable in each workflow's ⚙ popout.

**Chosen model (operator):** promote the per-workflow knobs into the common page
as shared **DEFAULTS**; each workflow **inherits** the common value but can
**override** it locally.

## Design

Two classes of common setting, one shared mutation bus:

1. **Global params** — `pump_settle_time_s`, `pump_relief_volume_uL`,
   `pump_prime_time_s`. Canonically live on `HardwareConfig` (single source of
   truth; already read by the controller + Quick Print). No per-workflow
   override — there is one value. Editable on the Common page AND in each
   workflow popout (a "Common — Pump (global)" section); every edit mutates the
   one HardwareConfig value and persists/propagates.

2. **Promoted prep defaults** — `service_z`, `prep_rate`, `oil_needles`,
   `buffer_needles`, `wash_cycles`, `wash_z_amp`, `wash_xy_amp`, `wash_dwell`.
   Shared defaults live in a new `CommonPrintSettings` model (persisted in
   `settings.json` → `common_print_settings`). Each workflow popout shows these
   with an **"Override common"** checkbox: unchecked = inherit (widget disabled,
   mirrors the common value live); checked = local override (workflow's own
   value, saved per-workflow). The Common page edits the shared defaults.

### `SupportClasses/CommonPrintSettings.py` (new, pure-Python, no Qt)
- `GLOBAL_KEYS` + `GLOBAL_DEFAULTS` + `PROMOTED_DEFAULTS`.
- Holds a ref to the live `HardwareConfig` (for the globals) + a `_promoted`
  dict. `get(key)` / `set(key, value)` route globals → HardwareConfig attr,
  promoted → `_promoted`. `set` notifies registered listeners `(model, key)`.
- `set_hardware_config`, `values()`, `promoted_dict()` / `load_promoted()`,
  `is_global()`, `add_listener()`. Value coercion (int wash_cycles, non-neg
  floats).

### `gui/dialogs/workflow_settings_dialog.py` (extend)
- `SettingsSection.add_common(key, label, widget, common_key=None, *,
  overridable=True, help=None)` — registers the value widget AND (if
  overridable) an "Override common" checkbox under `key + "__ovr"`. Lays them
  out as one row.
- `WorkflowSettingsDialog.set_common(common)` — store the `CommonPrintSettings`
  ref; re-sync every linked widget (inheriting/global widgets ← common value).
- Linked-widget behaviour: non-overridable (global) widget edit → `common.set`;
  overridable widget gated by its override checkbox (off = disabled + mirrors
  common). `apply()` re-syncs links + a **migration** pass (old saved value with
  no `__ovr` key → override=True iff it differs from the common default, so
  existing customisations are preserved as overrides).
- Workflow runtime reads are UNCHANGED — an inheriting widget is force-set to the
  common value, so `self._service_z.value()` already returns the effective value.

### `gui/pages/workflows/common_print_settings_workflow.py` (new page)
- A full workflow page (tile `common_print_settings`, ⚙ icon). Sections: Pump
  (global) [settle, relief, prime], Needle prep (shared defaults) [the 8
  promoted keys], Calibration heights (read-only via `build_locations_widget`).
- Edits the `CommonPrintSettings` model directly (globals + promoted defaults).
- `back_requested`, `get_page_title`, `set_hardware_config`,
  `set_calibration_data`, `set_common_print_settings`.

### `gui/pages/workflows/workflow_picker.py` + `workflows_mode.py`
- New tile + `elif "common_print_settings"` dispatch. WorkflowsModePage gains
  `set_common_print_settings(common)` fan-out (like `set_hardware_config`).

### `gui/app.py`
- Own a `CommonPrintSettings`; restore promoted dict from settings.json; set its
  HardwareConfig ref (refreshed on every HW-config change). Register a listener:
  global key → persist+propagate HardwareConfig; promoted key → save
  `common_print_settings` + re-fan-out. Pass it to WorkflowsModePage; fan out at
  startup + on change. Wire the Common page.

### Workflows (4 prep workflows): spheroid, cell_targeting, cell_labeling, quick_print
- Promoted prep fields `sec.add(...)` → `sec.add_common(...)`.
- Add a "Common — Pump (global)" section (settle/relief/prime, non-overridable).
- `set_common_print_settings(common)` → `dlg.set_common(common)`.
- Fluorescence Mosaic: no prep params → untouched (still gets the global
  section? — no; it has no pump prep. Left as-is).

## Implementation Steps

- [x] `CommonPrintSettings` model + test.
- [x] Dialog `add_common` / `set_common` + migration + test.
- [x] Common page + tile + dispatch + WorkflowsModePage fan-out.
- [x] app.py: own/restore/persist/propagate/wire.
- [x] Update spheroid (end-to-end proof) then cell_targeting / cell_labeling /
  quick_print.
- [x] Tests (model, dialog inherit/override + migration, page build smoke,
  fan-out) + run existing workflow-settings/spheroid/cell/quick-print suites.
- [ ] Real-HW verification on ME3B V1.

## Issues & Decisions

- **Single source of truth:** globals stay on HardwareConfig (no duplication);
  the Common page + workflow popouts are just editors of the one value.
- **Inherit/override via per-field checkbox** (auto-save means a saved value
  can't itself signal "override" — an explicit flag is required). Inheriting
  widget mirrors common so runtime read sites need no change.
- **Migration:** existing per-workflow saved values are preserved as overrides
  iff they differ from the common default; matching-default fields inherit.
- **Promoted set** = the Needle-prep group shared by the 4 prep workflows
  (Quick Print only has service_z/buffer_needles/wash_cycles — promotes its
  subset). Easily extended later.

## Testing Notes

- `tests/test_v75x_common_print_settings.py` (22) — model (proxy globals,
  promoted defaults, coercion, listeners, load round-trip), dialog
  inherit/override + migration + global write-through, Common page edits → model
  / HW config, and WorkflowsModePage tile + fan-out reaching the Common page AND
  an inheriting workflow field.
- Regression green (230 across): `test_v75x_workflow_settings_popout` (35),
  `test_v75x_pump_settle_and_prime_time`, `test_v75x_spheroid_pick_place_z`,
  `test_v75x_cell_targeting_removal`, `test_v75x_cell_labeling`,
  `test_v75x_quick_print_pick_and_place`, `test_v75x_quick_print_workflow`,
  `test_v75x_pump_plunger_setup`.
- Offscreen smokes: Common page build + global/promoted edit propagation;
  full WorkflowsModePage fan-out (Common page + spheroid inheriting field both
  reflect the common value).

### Real-HW checklist (ME3B V1)

- Workflows → Common Print Settings: edit dwell / relief / prime → reflected on
  Hardware Setup → Pump and used by a run; edit a prep default → an
  un-overridden workflow inherits it.
- A workflow ⚙ popout: prep field shows "Override" — off mirrors the common
  value (disabled); on holds a local value; the global pump fields edit the one
  shared value. Restart preserves overrides + common defaults.
