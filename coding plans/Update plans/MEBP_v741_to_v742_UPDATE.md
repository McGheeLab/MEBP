# MEBP v7.4.1 → v7.4.2 Update Plan

## Objective

Hardware Setup → Device sub-page becomes a complete initial-machine-setup workspace:

1. **Connect Hardware** — buttons + live status badges for XY / ZP stages
2. **Per-axis Stage Jog + Record Min/Max** — three preset step sizes per axis (Fine / Med / Coarse) with "Set as Min" / "Set as Max" buttons that copy the current position straight into the Safety Limits spinboxes
3. **Axis Mapping** — four dropdowns map logical axes (Z, P1, P2, P3) to physical Marlin axes (X, Y, Z, E). End-to-end backend support: ZPStageManager, StageController, PrintManager, VelocityExecutor all honor the configured mapping
4. **Stepper Calibration** — command-a-known-distance + measure + calculate + send M92 workflow, per logical axis. Per-axis `steps_per_mm` dict replaces the single shared int

Branch: `Version-7.4.2` (branched from `Version-7.4.1`).

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/ZPStage.py` | `AXIS_MAP` now instance-level (`self.axis_map`); `steps_per_mm` accepts int or dict; new `set_axis_map()`, `set_steps_per_mm()`, `_build_m92_command()` methods; `_setup_printer()` uses new builder. Back-compat preserved — module-level `AXIS_MAP` still exists as default. |
| `SupportClasses/StageController.py` | New `_axis_letter()` helper resolves logical → physical via `zp_stage.axis_map` (falls back to module default). All 3 `AXIS_MAP[...]` call sites converted. New `apply_device_settings()` method caches axis_map + steps until ZP connects, then pushes them. |
| `SupportClasses/PrintManager.py` | 3 `AXIS_MAP.get()` sites updated to read from `ctrl.zp_stage.axis_map` (with module default as fallback). |
| `SupportClasses/VelocityExecutor.py` | 2 `AXIS_MAP.get()` sites updated to read from `ctrl.zp_stage.axis_map`. |
| `SupportClasses/Settings.py` | `device_profile` DEFAULTS gains `axis_map` and `steps_per_mm` dict. |
| `gui/pages/hardware/device_profile.py` | `DeviceProfile` dataclass gains `axis_map` + `steps_per_mm` fields; round-trip + `from_settings` / `apply_to_settings` updated. |
| `gui/pages/hardware/stage_panel.py` | 4 new groups: Connect Hardware, Axis Mapping, Stepper Calibration, Per-Axis Jog + Record Limits. New handler methods: `_connect_xy/zp`, `_disconnect_xy/zp`, `_jog`, `_record_limit`, `_cal_command_move`, `_cal_apply`, `_refresh_steps_grid`, `_refresh_jog_positions`, `_sync_connection_badges`, `on_status_update`. |
| `gui/pages/hardware_setup.py` | `on_status_update()` now forwards to `_stage_panel.on_status_update()` so badges + position readouts refresh while user is on the Device sub-page. |
| `gui/app.py` | After HW page wiring, calls `controller.apply_device_settings()` with the saved axis_map + steps_per_mm so they're ready when the ZP connects. Version bumped to v7.4.2. |
| `config/hardware/devices/Standard.json` | Adds `axis_map` and `steps_per_mm` sections. |
| `config/hardware/devices/Conservative.json` | Adds `axis_map` and `steps_per_mm` sections. |

## Implementation Steps

- [x] Branch from `Version-7.4.1` → `Version-7.4.2`
- [x] Backend: `ZPStageManager` configurable axis_map + per-axis steps_per_mm
- [x] Backend: `StageController._axis_letter()` helper + 3 call sites converted; `apply_device_settings()` injection point
- [x] Backend: `PrintManager` 3 sites + `VelocityExecutor` 2 sites converted to honor `zp_stage.axis_map`
- [x] Settings: `device_profile.axis_map` + `device_profile.steps_per_mm` defaults
- [x] DeviceProfile: carry both new fields; update JSON round-trip + Settings bridge
- [x] Bundled JSON profiles: add `axis_map` and `steps_per_mm`
- [x] UI: Connect Hardware group (buttons + StatusBadges)
- [x] UI: Per-axis Jog + Record Limits (3 step sizes per axis, Set as Min / Set as Max)
- [x] UI: Axis Mapping (4 combos)
- [x] UI: Stepper Calibration (commanded move → measured → calculate + send M92)
- [x] Wire `MainWindow` to push saved axis_map + steps_per_mm into controller via `apply_device_settings()`
- [x] Wire `_apply` to push axis_map into live zp_stage on Apply Settings
- [x] Wire `on_status_update` so jog readouts + connection badges refresh while on Device sub-page
- [x] Smoke-test: settings round-trip, connect path applies pending settings, M92 command builds correctly with custom mapping
- [x] Bump version to v7.4.2
- [x] Run test suite
- [x] Architecture doc + README archive per CLAUDE.md checklist

## Testing Notes

- **Smoke-test passes (verified):**
  - Loading custom axis_map `{Z:Z, P1:X, P2:Y, P3:E}` via Settings → after `apply_device_settings()` + `connect_stages()`, `zp_stage.axis_map` reflects the custom mapping
  - `_build_m92_command()` with custom map + per-axis steps produces `M92 Z5000.00 X5069.00 Y5100.00 E5200.00` — correct physical letters and per-axis values
  - All 4 new UI groups instantiate cleanly with the right widget set
- **Manual test plan:**
  1. Launch app: `python3 main.py` (simulation)
  2. Go to Hardware Setup → Device sub-page
  3. Click **Connect XY** + **Connect ZP** — badges turn green
  4. Click `+10µm` X jog button several times — position readout updates
  5. Click **Set as Max** next to X — confirm `XY X max` safety spinbox now shows the jogged-to position
  6. Change Axis Mapping: P1 dropdown from Y to X → Apply Settings → confirm `settings.json` shows the new mapping
  7. In Stepper Calibration: pick Z, commanded 1mm, click Command Move → enter measured 0.95mm → click Calculate & Send M92 → confirm steps/mm display updates (e.g. 5069 → 5333) and `settings.json` reflects the new value
- **Per-axis calibration verified end-to-end:** different `steps_per_mm` values for each logical axis are correctly translated to physical Marlin axes via the configured `axis_map`.

## Issues & Decisions

- **Per-axis steps_per_mm dict instead of single int.** The user explicitly asked for "save this to the correct axis." Backend `ZPStageManager.steps_per_mm` is now `dict[str, int]`, keyed by logical axis (Z/P1/P2/P3). Sign indicates direction; negative inverts (matches the legacy `Z-{spm}` behavior for P2). Constructor accepts both int (back-compat: collapsed to dict with legacy default signs) and dict.
- **Axis map honored across all execution paths.** Not just the manual jog. `PrintManager` (the print execution engine) and `VelocityExecutor` (the high-rate motion controller) were also reading the module-level `AXIS_MAP` directly. All call sites now consult `ctrl.zp_stage.axis_map` first and fall back to the module default. Without this fix, a user with a custom mapping would have manual jog work correctly while print execution silently sent commands to the wrong physical axes.
- **Pending settings cached on StageController.** Settings can be configured before the ZP stage is connected. `StageController._pending_axis_map` / `_pending_steps_per_mm` hold the values until `connect_stages()` instantiates the `ZPStageManager`, at which point they're pushed via `set_axis_map()` + `set_steps_per_mm(persist=True)` (the latter sends M92 to Marlin).
- **Three preset step sizes, not configurable spinbox.** Per the user's choice: Fine / Med / Coarse for each axis. XY: 10µm / 100µm / 1000µm. Z/P: 0.01mm / 0.1mm / 1mm. Discoverable and fast for the common record-limits workflow; if more granular control is needed, the existing Jog Control page still has the configurable-step path.
- **Calibration formula.** `new_steps = current_steps × (commanded / measured)`. Preserves the sign of the current calibration so direction inversion isn't accidentally clobbered.
- **Record Min/Max writes to safety spinboxes, not directly to settings.** The user can still adjust the value before clicking Apply Settings — useful if you want to round a jog-recorded number to a clean limit (e.g. 129843 → 130000).
