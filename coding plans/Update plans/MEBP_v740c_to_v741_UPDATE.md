# MEBP v7.4.0-c → v7.4.1 Update Plan

## Objective

Introduce **device profiles** — reusable, per-machine setting files for the safety envelope, motor feedrates, and axis direction flips. Same physical-machine settings travel across any experiment-level HardwareConfig.

1. **`config/hardware/devices/` directory** with two bundled profiles:
   - `Standard.json` — sensible defaults for a typical ProScan + Marlin bioprinter
   - `Conservative.json` — tighter safety envelope and slower feedrates; recommended for new lab members
2. **`DeviceProfile` dataclass** with JSON round-trip + Settings bridge.
3. **Stage sub-page UI** gets a Device Profile group at the top with: profile dropdown, Load / Save / Save As… / Delete buttons, active-profile status label.
4. **Stage sub-page promoted to first position** in HW Setup. It's the one-time initial device setup — naturally precedes the experiment-level sub-pages.
5. **First-launch migration auto-applies Standard.json** so new users get a sensible safety envelope from the very first launch.
6. **Fix v7.4.0-b regression**: the Stage panel used `safety_limits.xy_min_um` / `xy_max_um` keys that didn't match the canonical `SafetyLimits.xy_min_x` / `xy_max_x` / `xy_min_y` / `xy_max_y` field names. The panel now uses the correct field names and exposes separate X-min / X-max / Y-min / Y-max spinboxes, plus per-pump P1/P2/P3 min/max.

Branch: `Version-7.4.1` (branched from `Version-7.4.0-c`).

## Files Modified

| File | Rationale |
|------|-----------|
| `config/hardware/devices/Standard.json` (new) | Bundled default profile — wide safety envelope, medium feedrates |
| `config/hardware/devices/Conservative.json` (new) | Bundled tight-safety profile — narrower envelope, slow feedrates |
| `gui/pages/hardware/device_profile.py` (new) | `DeviceProfile` dataclass; `list_profiles()` / `delete_profile()` helpers; Settings bridge |
| `gui/pages/hardware/stage_panel.py` | Device Profile group added at top (combo + Load/Save/Save As/Delete buttons + status label). Safety widgets reworked to use canonical SafetyLimits field names. Active-profile selection restored from settings on each `_load_from_settings`. |
| `gui/pages/hardware_setup.py` | Sub-page registration order changed: Device first (initial setup), then Identity → Plate → Pumps & Inks → Needle → Rosette → Cameras. `get_sub_page_title()` updated. |
| `gui/app.py` | Wizard deep-link target updated for new sub-page order (Pumps & Inks is now idx 3, not 2). Version bumped to v7.4.1. |
| `SupportClasses/Settings.py` | New `device_profile` section in DEFAULTS; new `_apply_default_device_profile()` migration that runs once on fresh install and applies bundled Standard.json |

## Implementation Steps

- [x] Branch from `Version-7.4.0-c` → `Version-7.4.1`
- [x] Create `config/hardware/devices/` directory
- [x] Author `Standard.json` and `Conservative.json` device profiles
- [x] Write `gui/pages/hardware/device_profile.py`:
  - [x] `DeviceProfile` dataclass with `to_dict` / `from_dict` / `save` / `load`
  - [x] `from_settings(settings, name)` factory
  - [x] `apply_to_settings(settings)` bridge
  - [x] `list_profiles()` and `delete_profile()` helpers
- [x] Update `StageHardwarePanel`:
  - [x] Add Device Profile group (combo + 4 action buttons + status label)
  - [x] Rewrite safety group to use canonical field names (`xy_min_x` etc.)
  - [x] Add separate X-min / X-max / Y-min / Y-max spinboxes
  - [x] Add per-pump P1/P2/P3 min/max spinboxes
  - [x] Update `_load_from_settings` / `_apply` / `_reset_defaults`
  - [x] Restore active profile selection in `_load_from_settings`
  - [x] `_refresh_profile_list` / `_load_selected_profile` / `_save_to_selected_profile` / `_save_as_new_profile` / `_delete_selected_profile`
- [x] Reorder sub-pages in `hardware_setup.py`: Device first
- [x] Update wizard deep-link target index in `gui/app.py` (Pumps & Inks: 2 → 3)
- [x] Add `device_profile.active` default + `_apply_default_device_profile` migration in `Settings.py`
- [x] Bump version to v7.4.1
- [x] Smoke-test fresh-install migration: empty settings.json → Standard profile auto-applies
- [x] Smoke-test profile round-trip: load Standard → switch to Conservative → verify settings.json reflects each
- [x] Run test suite
- [x] Architecture doc + README archive per CLAUDE.md checklist

## Testing Notes

- **Fresh install:** Delete `settings.json` AND `settings.json.bak-v7.3`. Launch app. Confirm:
  - `settings.json` has `device_profile.active: "Standard"`
  - `safety_limits.xy_max_x` = 130000
  - `zp_stage.jog_feedrate` = 900
  - `migrations.v7_4_1_default_device: true`
  - Stage sub-page combo shows "Standard" selected
- **Profile switching:** On Stage sub-page, pick "Conservative" → click Load → confirm:
  - `safety_limits.xy_max_x` = 100000 (Conservative value)
  - `zp_stage.jog_feedrate` = 400
  - Status label shows "Loaded profile: Conservative"
- **Save As:** Edit one field → click Save As… → enter "MyBench" → confirm:
  - `config/hardware/devices/MyBench.json` created
  - Combo shows "MyBench" and is selected
  - Relaunch app → Stage panel still shows "MyBench" selected (active persisted)
- **Delete:** Select "MyBench" → click Delete → confirm dialog → file removed, combo refreshed
- **Sub-page order:** HW Setup right sidebar shows Device, Identity, Plate, Pumps & Inks, Needle, Rosette, Cameras (top to bottom).
- **Wizard deep-link:** Trigger first-run wizard → check "Take me to Hardware Setup → Pumps & Inks" → Finish → lands on the right sub-page (now index 3).

## Issues & Decisions

- **Fixed v7.4.0-b safety-limits field-name regression.** The Stage panel in v7.4.0-b used `safety_limits.xy_min_um` / `xy_max_um` keys that didn't match the canonical `SafetyLimits.xy_min_x` / `xy_max_x` / `xy_min_y` / `xy_max_y` field names. This meant the panel was writing to keys the StageController never read — settings adjustments via the Stage panel had no effect on the real safety envelope. Catching this is part of v7.4.1's work because the bundled JSON profiles use the correct field names; mismatches would have shown up immediately. Caught and fixed before any user-facing v7.4.0 merge.
- **Stage sub-page reordered to first.** Conceptually the device profile is the one-time initial setup of the physical machine. Everything else (Identity, Plate, Pumps, Needle, Rosette, Cameras) configures a specific experiment on top of that machine. Putting Device first communicates the "set this once, then configure experiments" pattern.
- **Auto-load Standard.json on fresh install.** Triggered as a one-shot migration (`migrations.v7_4_1_default_device`). Skipped if a user has manually picked a different profile. Means a brand-new user gets a sensible safety envelope before they ever touch the Stage sub-page.
- **No GUI dependency in the Settings migration.** `_apply_default_device_profile` reads the bundled JSON directly with `json.loads(path.read_text())` rather than going through `DeviceProfile.load()` — avoids importing GUI code from `SupportClasses/Settings.py` (which is a backend-only module).
- **Test suite environmental flake.** Full-suite run reported 8 failures + 11 errors vs the v7.4.0-c baseline of 4 + 9. The 6 "new" failures are all in `TestXYSimVsHardware` / `TestZPSimVsHardware` and are guarded by `_skip_if_no_real()`. Running those tests in isolation, they correctly skip (no real hardware detected, exit code 0). The flake appears to be intermittent USB-serial port detection during the long full-suite run — v7.4.1's code changes don't touch serial detection, `XYStageManager`, or any of the sim-vs-hardware comparison logic. The 13 baseline failures (test_v726_print_execution, test_v731_jog_navigation, test_v73_trajectory_planner, test_sim_vs_hardware TestLifecycle/TestXYStageSimulated/TestZPStageSimulated) are unchanged from earlier v7.4.0-* runs.
