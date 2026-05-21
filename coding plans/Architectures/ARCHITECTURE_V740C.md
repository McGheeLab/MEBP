# MEBP v7.4.0-c — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.0-c | May 2026**

> **This document is a delta against the previous sub-version
> [`ARCHITECTURE_V740B.md`](ARCHITECTURE_V740B.md), and through it
> [`ARCHITECTURE_V740A.md`](ARCHITECTURE_V740A.md) and
> [`ARCHITECTURE_V737.md`](ARCHITECTURE_V737.md).**
> Sections not mentioned here are unchanged.
> v7.4.0-c closes the v7.4.0 UX-focused release.

---

## 1. What's New in v7.4.0-c

### First-run OnboardingWizard (`gui/onboarding/wizard.py`)

A modal QDialog that orients new users to MEBP before the main window
becomes usable. Triggered from `MainWindow.__init__` via
`QTimer.singleShot(0, _maybe_show_onboarding)` — the wizard pops up
once the event loop starts.

Trigger condition (`should_show_onboarding`): the user has no
`workspace.needle_gauge` saved AND no `hardware_config.last_config_file`
recorded. After Finish, those keys are stamped so the wizard never
re-fires.

Steps:
1. **Welcome** — overview of what MEBP does and what the wizard covers.
2. **Stages** — Simulation toggles for XY and ZP; non-blocking Test
   Connection buttons that probe each axis on a `QThread`
   (`_ConnectProbeWorker`).
3. **Plate & Needle** — Well plate format combo + needle gauge combo.
   The Next button is disabled until a gauge is selected.
4. **Pumps & Inks** — Explanatory card + a checkbox to load the starter
   ink library, plus a deep-link option to land in
   `Hardware Setup → Pumps & Inks` on Finish.
5. **Ready** — Summary of what will be saved (plate, gauge, simulation
   flags, starter ink count). On Finish the wizard:
   - Loads the prefab ink library into `HardwareConfig.ink_library`
   - Stamps `workspace.needle_gauge` and `workspace.plate_format`
   - Emits `completed(HardwareConfig)` to MainWindow
   - Optionally requests deep-link to HW Setup → Pumps & Inks

### Starter ink library (`gui/onboarding/prefab_inks.json`)

Five common bioprinting materials with reasonable defaults: **PBS**,
**Alginate 2%**, **GelMA**, **Water**, **Cell Media**. Loaded into
`HardwareConfig.ink_library` only when the wizard's
"Load starter ink library" checkbox is on (default: on).

### Progressive-disclosure help mode (`gui/widgets/help_toggle.py` + MainWindow)

- `HelpToggle` (checkable QPushButton, ? Help label) lives in the top
  bar next to the connection dots.
- `MainWindow._help_mode` mirrors the toggle state;
  `MainWindow.help_mode_changed = Signal(bool)` notifies subscribers.
- `MainWindow.register_form_row(row)` registers a
  [`FormRow`](../../gui/widgets/components.py) widget; every registered
  row's help text reveals when help mode is on. (Pages opt in by calling
  `main_window.register_form_row(row)` after creating a FormRow with
  `help_text=...`; no page does this yet — the infrastructure ships
  ahead of the rollout.)

### Help text catalog (`gui/onboarding/help_texts.py`)

A flat dict of `(page.section.field)` → markdown help string. Pages look
up help strings by key and pass them to `FormRow.set_help(text)`. Initial
catalog covers ~15 of the highest-confusion controls (Hardware Identity,
Plate, Needle, Pumps; Stage safety/feedrates/axis-flip; Settings
simulation/polling/Xbox; Calibration µm/px and Safe Z).

### Global InvalidationBanner integration

The `InvalidationBanner` widget added in v7.4.0-b is now mounted
permanently in `MainWindow` above the page stack (below the
`LoadingBanner`). When `MainWindow._on_hardware_config_changed` fires
`hw_config_invalidated`, `_show_invalidation_banner()` surfaces a
context-aware message ("Hardware changed (plate format, pumps) —
refresh pages to re-apply"). Clicking **Refresh** calls
`_propagate_hardware_config(self._hardware_config)` so every page
re-reads its inputs.

The banner is suppressed when the user is on Hardware Setup (page 0)
because they're actively editing config and don't need to be nagged.

---

## 2. New / Changed Files

```
gui/
├── app.py                                  # +help_mode_changed signal,
│                                           #  register_form_row, _help_mode state,
│                                           #  _maybe_show_onboarding wizard trigger,
│                                           #  _on_onboarding_completed handler,
│                                           #  _on_invalidation_refresh,
│                                           #  global InvalidationBanner mount,
│                                           #  HelpToggle in top bar,
│                                           #  version bump v7.4.0-c
├── onboarding/                             # NEW package
│   ├── __init__.py
│   ├── wizard.py                           # OnboardingWizard + _ConnectProbeWorker
│   ├── prefab_inks.json                    # 5 starter inks
│   └── help_texts.py                       # ~15 help string entries
└── widgets/
    └── help_toggle.py                      # NEW — HelpToggle QPushButton
```

---

## 3. Updated Core Design Principles

(Additions to the v7.4.0-a/-b list — others unchanged.)

15. **First-run is opinionated.** New users see a 5-step wizard that
    captures the minimum needed to unlock the rest of the app. The
    wizard does not try to be a complete configuration UI — it ensures
    the user can find their way to Hardware Setup → Pumps & Inks
    afterward (via a deep-link option).

16. **Help is progressive.** Beginners turn the top-bar Help toggle on
    and see inline guidance on every FormRow. Daily users keep it off
    and get a dense interface. There is one global toggle, not a
    per-page or per-control opt-in.

17. **Cross-page invalidation is user-driven.** When hardware changes,
    the global InvalidationBanner appears with a Refresh button. The
    refresh is one explicit action — no automatic background reloads
    that could surprise the user mid-task. Pages re-read their inputs
    via the existing `_propagate_hardware_config` path.

---

## 4. Onboarding Trigger Flow

```
MainWindow.__init__
        │
        ├── (build UI shell, create pages, set up timers)
        │
        ├── _navigate_to(0)               # Land on HW Setup
        │
        ├── QTimer.singleShot(0,          # Defer until event loop starts
        │       _maybe_show_onboarding)
        │
        └── return  (caller calls .show())

[event loop]
        │
        ▼
_maybe_show_onboarding
        │
        ├── should_show_onboarding(settings)?
        │       (workspace.needle_gauge unset AND
        │        hardware_config.last_config_file unset?)
        │
        ├── No  → return silently
        │
        └── Yes → OnboardingWizard.exec() (modal)
                       │
                       ├── User walks 5 steps
                       │
                       └── On Finish → completed(HardwareConfig)
                                       │
                                       ▼
                              _on_onboarding_completed
                                       │
                                       ├── hw_page.set_config(config)
                                       │     (triggers normal config_changed
                                       │      pipeline → propagates to all pages)
                                       │
                                       └── deep_link_target? → switch_to page+sub-page
```

---

## 5. Sections Unchanged

All sections of `ARCHITECTURE_V737.md`, `ARCHITECTURE_V740A.md`, and
`ARCHITECTURE_V740B.md` not mentioned above apply verbatim to v7.4.0-c.

A consolidated **ARCHITECTURE_V740.md** combining all three sub-version
deltas should be written after v7.4.0-c is validated in real use, to
serve as the single canonical reference going forward.
