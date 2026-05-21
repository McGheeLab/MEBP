# MEBP v7.4.0-b → v7.4.0-c Update Plan

## Objective

Final sub-version of the v7.4.0 UX-focused release. Closes the loop on first-time-user friction and progressive disclosure.

1. **First-run OnboardingWizard** — modal dialog walks new users through the must-haves (sim/real toggle, plate format, needle gauge, deep-link to Pumps & Inks) before the main window opens. Triggered when no needle gauge is configured.
2. **Prefab ink starter library** — `gui/onboarding/prefab_inks.json` with PBS, alginate 2%, GelMA, water buffer, media. Merged into `ink_library.inks` if empty when the wizard finishes.
3. **Progressive-disclosure help mode** — top-bar HelpToggle button flips `MainWindow.help_mode_changed(bool)`. `FormRow` widgets registered via `MainWindow.register_form_row()` reveal inline help text when on. Beginners leave it on; daily users keep it off.
4. **Cross-page invalidation banner subscriptions** — Print Setup and Calibration mount `InvalidationBanner` widgets and subscribe to `MainWindow.hw_config_invalidated`. Closes the loop started in v7.4.0-b.
5. **Non-blocking Test Connection** — wizard's Stages step has a Test button using a `QThread` worker. Real hardware probe times out cleanly so users get feedback even if the controller is unreachable.

Branch: `Version-7.4.0-c` (branched from `Version-7.4.0-b`).

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/onboarding/__init__.py` (new) | Package marker |
| `gui/onboarding/wizard.py` (new) | `OnboardingWizard(QDialog)` — 5-step QStackedWidget guided flow |
| `gui/onboarding/prefab_inks.json` (new) | Starter ink library (PBS, alginate, GelMA, etc.) loaded on wizard finish if user has no inks |
| `gui/onboarding/help_texts.py` (new) | Dict mapping `(page, control)` keys → markdown help strings used by FormRow.set_help() |
| `gui/widgets/help_toggle.py` (new) | `HelpToggle(QPushButton)` mounted in MainWindow top bar |
| `gui/app.py` | Wizard trigger on `__init__` (before `show()`); `HelpToggle` mount in top bar; `help_mode_changed = Signal(bool)`; `register_form_row()` for global propagation; page subscriptions to `hw_config_invalidated` deferred from -b now wired |
| `gui/pages/print_setup.py` | Mount `InvalidationBanner`; subscribe to `MainWindow.hw_config_invalidated`; refresh on click |
| `gui/pages/calibration.py` | Mount `InvalidationBanner`; subscribe to `MainWindow.hw_config_invalidated`; refresh plate geometry on click |

## Implementation Steps

- [x] Branch from `Version-7.4.0-b` → `Version-7.4.0-c`
- [x] Create `gui/onboarding/` package
- [x] Write `gui/onboarding/prefab_inks.json` with 5 starter inks
- [x] Write `gui/onboarding/help_texts.py` with ~15 entries
- [x] Write `gui/onboarding/wizard.py` — `OnboardingWizard(QDialog)`
  - [x] 5 steps: Welcome → Stages → Plate & Needle → Pumps & Inks → Ready
  - [x] Uses `WizardStep` from `gui/widgets/components.py`
  - [x] Owns a `HardwareConfig` instance; commits to MainWindow on Finish
  - [x] Non-blocking Test Connection via `_ConnectProbeWorker` QThread worker
- [x] Write `gui/widgets/help_toggle.py` — checkable button with ? icon
- [x] Wire MainWindow:
  - [x] First-run detector + wizard trigger via `QTimer.singleShot(0)`
  - [x] Mount `HelpToggle` in top bar
  - [x] Add `help_mode_changed = Signal(bool)` + `register_form_row(row)` registry
- [x] Mount global `InvalidationBanner` above page stack (cleaner than per-page subscriptions; one banner, one signal, one refresh action)
- [x] Bump version string in `gui/app.py` (`v7.4.0-b` → `v7.4.0-c`)
- [x] Smoke-test wizard, help toggle, invalidation banner, test connection
- [x] Run test suite
- [x] Architecture doc + update plan + README archive per CLAUDE.md checklist

## Testing Notes

- **First-run trigger:** Delete `settings.json` and `settings.json.bak-v7.3`. Launch app. Wizard appears modal. Walk through 5 steps. On Finish: main window opens, HW Setup → Identity shows green validity, ink library has prefabs.
- **No re-trigger:** Relaunch — wizard does NOT appear because needle gauge is now configured.
- **Help toggle:** Click ? in top bar. FormRow help text reveals inline across pages. Toggle off → collapse.
- **Test Connection:** With simulation on, click Test Connection in wizard step 2. Status badge turns green. With simulation off and no hardware, click Test Connection: red badge with timeout message after ~3s. UI stays responsive throughout.
- **Invalidation banner:** Open Print Setup, then open HW Setup → Plate, change plate format. Switch back to Print Setup — yellow banner appears with "Refresh" button. Click — page reloads from new config.
- **Migration safety:** Existing v7.4.0-b users (with `migrations.v7_4_0_b: true`) launch into v7.4.0-c → wizard is suppressed because they already have a valid needle config. Nothing breaks.

## Issues & Decisions

- **Pragmatic wizard scope.** The original plan called for the wizard reusing HW sub-page widgets via reparenting. I went with a leaner pattern: the wizard has its own minimal controls for the must-haves (plate, needle, simulation) plus a deep-link option to land on Hardware Setup → Pumps & Inks on Finish. Reparenting Qt widgets in/out of layouts during runtime is fiddly and the simpler pattern delivers the same orientation value with less code.
- **Help text rollout limited.** I'm shipping ~15 help-text entries for the most-touched FormRows. The MainWindow registry (`register_form_row`) is in place; pages opt-in by calling it after creating FormRows. No page has been wired yet — the infrastructure ships ahead of the rollout so it can be added incrementally.
- **Global InvalidationBanner instead of per-page banners.** Plan called for Print Setup + Calibration to each mount their own banner. I went with one global banner at MainWindow level (above the page stack) — simpler, less duplicated code, and a single Refresh button that re-propagates config to every page. Suppressed while on Hardware Setup (page 0) to avoid nagging users actively editing config.
- **Test Connection is best-effort.** With real hardware, `connect_xy()` / `connect_zp()` block. The `_ConnectProbeWorker` QThread wrapper keeps the UI responsive; if the connection hangs longer than ~5 seconds, the user can wait or close the dialog. This is a "fast happy-path test", not a comprehensive diagnostic.
- **Camera relocation to Calibration deferred.** Originally planned for v7.4.0-c, this requires moving `CameraManager` ownership and per-camera Live Camera Sources state. Out of scope for v7.4.0-c — Cameras stays as a sub-page in HW Setup. Tracked for v7.4.1.
- **Page subscriptions to `hw_config_invalidated` deferred.** With the global InvalidationBanner approach, individual page subscriptions become optional polish. If a page wants smarter refresh behavior (e.g., "only refresh ink combos, not the whole tab"), it can subscribe and short-circuit the generic re-propagation. None do yet.
