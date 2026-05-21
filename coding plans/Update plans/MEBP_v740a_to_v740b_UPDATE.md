# MEBP v7.4.0-a → v7.4.0-b Update Plan

## Objective

Workflow restructure for the v7.4.0 UX-focused release.

1. **Decompose Hardware Setup** from a single 1,800-line scrolling form into a `ModePage` with seven sub-pages (Identity, Plate, Pumps & Inks, Needle, Rosette, Cameras, Stage) so the user can find any one knob without scanning the entire page.
2. **Slim Settings** to user/system preferences only. ZP feedrates, safety limits, and axis direction flips move to a new **Stage** sub-page inside Hardware Setup — they describe the physical machine, not user preferences.
3. **Settings.json migration** with `.bak-v7.3` snapshot so users can roll back if anything goes wrong.
4. **Cross-page invalidation**: `MainWindow.hw_config_invalidated` signal + `InvalidationBanner` widget so downstream pages (Print Setup, Calibration) can detect stale derived data when the HW config changes. Page subscriptions are deferred to v7.4.0-c.

Branch: `Version-7.4.0-b` (branched from `Version-7.4.0-a`).

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/hardware_setup.py` | `HardwareSetupPage` now subclasses `ModePage`; `_setup_ui` partitions the 9 existing sections into 7 sub-pages (Identity, Plate, Pumps & Inks, Needle, Rosette, Cameras, Stage). All existing widget construction code preserved — only the parent layout changes. Stage sub-page hosts a `StageHardwarePanel` for safety/feedrate/axis-flip controls. New `set_settings()` injector. `set_controller()` extended to delegate to Stage panel. New `get_sub_page_title()` override. |
| `gui/pages/hardware/__init__.py` (new) | Hardware sub-page package marker |
| `gui/pages/hardware/stage_panel.py` (new) | `StageHardwarePanel` — safety limits, ZP stage feedrates, axis direction flips. Reads/writes the same `safety_limits.*`, `zp_stage.*`, `axis_flip.*` keys that StageController/SafetyLimits read — no namespace migration. |
| `gui/pages/settings_page.py` | `_setup_ui` no longer calls `_build_safety_card`, `_build_zp_stage_card`, `_build_axis_flip_card`. A new `_build_moved_notice_card` points users at the new home. `_load_from_controller`, `_apply_settings`, `_ctx_safety_toggled`, and the context-widget builder are guarded with `hasattr()` so missing widgets don't crash. |
| `SupportClasses/Settings.py` | New `migrations` section in `DEFAULTS`; `load()` calls `_run_migrations()`; first-run writes a `.bak-v7.3` snapshot of the loaded `settings.json`. |
| `gui/widgets/invalidation_banner.py` (new) | `InvalidationBanner` — yellow strip with ⚠ icon, message, and refresh button. Pages mount it and connect `refresh_clicked` to their reload path. |
| `gui/app.py` | New `MainWindow.hw_config_invalidated = Signal(dict)`. Wired Stage panel injection (`hw_page.set_settings(self.settings)`). Version bumped to v7.4.0-b. |

## Implementation Steps

- [x] Branch from `Version-7.4.0-a` → `Version-7.4.0-b`
- [x] `HardwareSetupPage` subclasses `ModePage`
  - [x] Add `from gui.pages.mode_page import ModePage` import
  - [x] Add `set_settings()` injector; extend `set_controller()` to delegate
  - [x] Replace `_setup_ui` outer-scroll setup with per-sub-page scaffolds in `self._sub_layouts` dict
  - [x] Reroute each section's `self._content_layout.addWidget(...)` to the appropriate sub-page layout
  - [x] Replace finalize block with `add_sub_page()` registrations
  - [x] Add `_make_subpage_scaffold(bg)` helper
  - [x] Add `get_sub_page_title()` override
- [x] Create `gui/pages/hardware/` package + `stage_panel.py` with `StageHardwarePanel`
- [x] Wire `hw_page.set_settings(self.settings)` from `MainWindow._create_pages`
- [x] Slim `SettingsPage`
  - [x] Remove `_build_safety_card`, `_build_zp_stage_card`, `_build_axis_flip_card` calls from `_setup_ui`
  - [x] Add `_build_moved_notice_card` pointing at HW Setup → Stage
  - [x] Guard `_load_from_controller` with `hasattr()` on removed widgets
  - [x] Guard `_apply_settings` with `hasattr()` on removed widgets
  - [x] Guard `_ctx_safety_toggled` + context-widget init
- [x] Add `Settings._run_migrations()` + `_backup_v73()` in `SupportClasses/Settings.py`
- [x] Create `gui/widgets/invalidation_banner.py`
- [x] Add `MainWindow.hw_config_invalidated = Signal(dict)` + `_emit_invalidation()` from `_on_hardware_config_changed`
- [x] Bump version string in `gui/app.py` (`v7.4.0-a` → `v7.4.0-b`)
- [x] Smoke-test full app boot, every sub-page, slimmed Settings, Stage apply round-trip
- [x] Run test suite
- [x] Architecture doc + README archive per CLAUDE.md checklist

## Testing Notes

- **Sub-page navigation:** `python3 main.py`, open HW Setup, click each of the 7 right-sidebar icons. Verify titles change (`Hardware: Identity`, `Hardware: Plate`, …, `Hardware: Stage`).
- **Stage panel ↔ Settings round-trip:** On the Stage sub-page, toggle `Enable safety limits` off, click **Apply Settings**. Quit, relaunch, navigate back — toggle reflects the saved state. Inspect `settings.json` to confirm `safety_limits.enabled: false`.
- **Settings page slim:** Open Settings (page 6). Confirm only Connection, Controller, Polling, Xbox, Logging cards + the yellow "Moved in v7.4.0-b" notice card are visible. Click **Apply Settings** — no crash.
- **Migration:** Delete `settings.json.bak-v7.3` if present. Launch app once. Confirm `.bak-v7.3` file is created, matches the pre-load `settings.json` content, and `migrations.v7_4_0_b: true` is set in `settings.json`. Relaunch — no second backup.
- **Validity gating:** Hardware Setup still gates pages 1–5 (Dashboard, Jog, Cal, Printing, P&P). Identity sub-page validity indicator still updates.
- **HW Setup signal regression:** Edit a pump's syringe → existing config_changed / config_validated still fire → MainWindow still propagates to all pages.

## Issues & Decisions

- **Why keep camera config on HW Setup for now.** The plan called for moving cameras to Calibration. I chose to keep them as a 6th sub-page (Cameras) on HW Setup for v7.4.0-b because moving cameras requires also moving the `CameraManager` reference and per-camera Live Camera Sources state — that's a separate refactor with its own risk surface. The move can land cleanly in v7.4.0-c alongside the onboarding wizard's camera-handoff step.
- **Why duplicate widgets on the Settings page were not removed entirely.** I deleted the build calls and guarded apply/reset/load with `hasattr()` checks. The build methods themselves still exist (dead code) so future migrations can re-enable them if needed; deleting the bodies is a clean-up for v7.4.0-c.
- **Migration is intentionally minimal.** The plan originally called for moving `safety_limits.*` keys into a `hardware_config.safety_limits` namespace. I kept the keys at top-level because `StageController` and `SafetyLimits` read them there — renaming would require touching the controller, simulator, and several other call sites. The Stage sub-page reads/writes the same keys, so behavior is identical. The migration only adds the `.bak-v7.3` snapshot + flag.
- **Cross-page invalidation is wired but no page subscribes yet.** `MainWindow.hw_config_invalidated` emits when plate format or pump configuration changes. `InvalidationBanner` widget is ready to use. Page subscriptions (Print Setup ink combo refresh, Calibration plate-geometry rebuild) are deferred to v7.4.0-c so we can validate the signal payload first.
- **Context panel handling for the new ModePage.** `MainWindow._update_mode_context` already handles ModePage instances by reading the active sub-page's context. For HW Setup, all sub-pages share one context (saved-config browser + validity badge), which is what users expect — this works because `HardwareSetupPage.get_context_widget()` returns the same widget regardless of which sub-page is active.
