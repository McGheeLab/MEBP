# MEBP v7.4.0-b — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.0-b | May 2026**

> **This document is a delta against
> [`ARCHITECTURE_V737.md`](ARCHITECTURE_V737.md) and the
> [v7.4.0-a delta](ARCHITECTURE_V740A.md).**
> Sections not mentioned here are unchanged.
> v7.4.0-b is the workflow-restructure sub-version of the v7.4.0 UX-focused
> release. v7.4.0-c follows with the first-run onboarding wizard.

---

## 1. What's New in v7.4.0-b

### Hardware Setup decomposed (`gui/pages/hardware_setup.py`)

`HardwareSetupPage` now subclasses
[`ModePage`](../../gui/pages/mode_page.py) and partitions its UI into seven
right-sidebar sub-pages instead of one long vertical scroll:

| Icon | Sub-page | Contents |
|------|----------|----------|
| 🧾 | Identity | Setup Name / Notes, Setup Status, Save/Load Config |
| 🔬 | Plate | Well Plate Format |
| 💧 | Pumps & Inks | Ink Library, Pump Channels, Pump-Ink summary |
| 🪡 | Needle | Needle gauge/length/channels, Channel→Pump Mapping |
| 🌸 | Rosette | Rosette Library |
| 📷 | Cameras | Camera Configuration, Live Camera Sources |
| ⚙️ | Stage | **New** — Safety Limits, ZP Feedrates, Axis Direction Flips |

All existing widget construction code is preserved. The refactor only
changes which parent layout each section's QGroupBox is added to — every
signal connection, instance attribute, validation handler, and config
save/load path is identical to v7.4.0-a.

### New `StageHardwarePanel` (`gui/pages/hardware/stage_panel.py`)

The Stage sub-page is hosted by a fresh `StageHardwarePanel` widget,
new in v7.4.0-b. It holds three sub-groups:

- **Safety Limits** — XY/Z/Pump min/max (µm and mm), XY max feedrate, Z max feedrate, master enable.
- **ZP Stage Feedrates** — max/retract/insert/jog feedrates, auto-save EEPROM position.
- **Axis Direction Flips** — per-axis (Z, P1, P2, P3) positive-direction inverters.

Reads from / writes to the same top-level `settings.json` keys
(`safety_limits.*`, `zp_stage.*`, `axis_flip.*`) the previous Settings
page used — no namespace migration. `StageController` and `SafetyLimits`
continue to read them at their original paths.

### Settings page slimmed (`gui/pages/settings_page.py`)

`SettingsPage._setup_ui` no longer builds the safety / ZP / axis-flip
cards. A new `_build_moved_notice_card` mauve banner tells users the
controls relocated to Hardware Setup → Stage. The build methods
themselves are still defined (dead code) but never invoked.
`_load_from_controller`, `_apply_settings`, the context-panel builder,
and `_ctx_safety_toggled` are wrapped with `hasattr()` guards so the
removed widgets being absent does not crash the page.

The Settings page is now strictly user/system preferences:
**Connection**, **Controller protocol**, **Polling/Watchdog**, **Xbox
mapping/deadzones**, **Logging**, plus the migration notice.

### Settings.json migration (`SupportClasses/Settings.py`)

`Settings.load()` now calls a new `_run_migrations()` method which
performs one-shot, idempotent migrations:

- `migrations.v7_4_0_b` — writes a `settings.json.bak-v7.3` snapshot
  of the loaded file on first launch under v7.4.0-b. Subsequent
  launches see the flag and skip. Users who need to roll back can
  copy `.bak-v7.3` back over `settings.json`.

The framework is designed to host future migrations (e.g.,
v7.4.0-c key renames) without touching `load()` again.

### Cross-page invalidation infrastructure

- `gui/widgets/invalidation_banner.py` — new `InvalidationBanner`
  widget: thin yellow strip with ⚠ icon, message, and refresh button.
- `MainWindow.hw_config_invalidated = Signal(dict)` — emitted from
  `_on_hardware_config_changed` with a dict of changed keys
  (`plate_format`, `pumps`). Pages subscribe and either auto-refresh
  or mount an `InvalidationBanner` and bind its
  `refresh_clicked` to their reload path.

No page subscribes yet — the widget + signal are ready for use, but
the actual page wiring (Print Setup ink-combo refresh, Calibration
plate-geometry rebuild) is deferred to v7.4.0-c so we can validate
signal payloads with real usage first.

---

## 2. New / Changed Files

```
gui/
├── app.py                                  # +hw_config_invalidated signal,
│                                           #  _emit_invalidation, hw_page.set_settings,
│                                           #  version bump v7.4.0-b
├── pages/
│   ├── hardware_setup.py                   # subclass ModePage; partitioned _setup_ui;
│   │                                       #  set_settings + Stage panel delegate
│   ├── settings_page.py                    # slimmed; hasattr guards; moved-notice card
│   └── hardware/                           # NEW package
│       ├── __init__.py
│       └── stage_panel.py                  # NEW — StageHardwarePanel
└── widgets/
    └── invalidation_banner.py              # NEW — InvalidationBanner
SupportClasses/
└── Settings.py                             # +_run_migrations, _backup_v73,
                                            #  migrations.v7_4_0_b default
```

---

## 3. Updated Core Design Principles

(Additions and revisions to the V737 / V740A list — others unchanged.)

12. **Hardware Setup is a mode-page.** Page 0 follows the same pattern
    Printing and Pick & Place have used since v7.3.3 — a right-side icon
    column switches between focused sub-pages. The single `HardwareConfig`
    is owned by the parent page; sub-pages mutate slices of it. The page
    still emits the same `config_changed` / `config_validated` signals
    that `MainWindow._on_hardware_config_changed` listens for.

13. **Settings page is preferences-only.** Anything that describes the
    physical machine (safety envelope, motor direction, feedrate ceilings)
    belongs to Hardware Setup. Anything that describes the user's
    workflow (simulation mode, polling intervals, controller mapping,
    log verbosity) belongs to Settings. The boundary is enforced by
    moving widgets, not just relabeling.

14. **Cross-page invalidation is explicit.** When Hardware Setup mutates
    config in a way that affects derived data on other pages, MainWindow
    fires `hw_config_invalidated` with a payload describing what
    changed. Pages opt in by binding to the signal and either
    auto-refreshing or surfacing an `InvalidationBanner`.

---

## 4. Settings.json Migration Path

```
settings.json     (v7.3.7 layout)
      │ first launch under v7.4.0-b
      ▼
   load()
      │
      ├── deep-merge DEFAULTS (adds migrations.v7_4_0_b = False)
      │
      ├── _run_migrations()
      │      └── v7_4_0_b is False
      │            ├── _backup_v73() → settings.json.bak-v7.3
      │            └── set migrations.v7_4_0_b = True; save()
      │
      └── (next launch: flag is True → migrations are skipped)
```

`StageController` and `SafetyLimits` continue to read `safety_limits.*`,
`zp_stage.*`, `axis_flip.*` at their pre-existing top-level paths. The
migration adds the backup but does **not** rename keys.

---

## 5. Sections Unchanged from V737/V740A

All sections of `ARCHITECTURE_V737.md` and `ARCHITECTURE_V740A.md` that
are not mentioned above apply verbatim to v7.4.0-b. In particular:

- Backend modules in `SupportClasses/` (no changes in -b)
- Hardware Communication
- Print Pipeline, Trajectory Planning, Recorder/History
- Camera Subsystem (camera UI still on HW Setup → Cameras sub-page;
  relocation to Calibration is deferred to v7.4.0-c)
- Threading Model

A full ARCHITECTURE_V740.md will consolidate the -a/-b/-c deltas after
v7.4.0-c ships.
