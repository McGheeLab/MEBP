# MEBP v7.4.1 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.1 | May 2026**

> **Delta against [`ARCHITECTURE_V740C.md`](ARCHITECTURE_V740C.md)**
> and through it the rest of the v7.4.0 release deltas. Sections not
> mentioned here are unchanged.
>
> v7.4.1 is a small focused release introducing **device profiles** —
> reusable, per-machine settings bundles for the safety envelope, motor
> feedrates, and axis direction flips. It also fixes a v7.4.0-b
> regression in the Stage sub-page widget keys.

---

## 1. What's New in v7.4.1

### Device profile system

**Why.** Safety limits, feedrate ceilings, and axis direction flips
describe the *physical machine*, not a particular experiment. They
change only when you set up the device (or significantly modify the
machine). They should travel with the machine, not the experiment.

**How.** A new directory `config/hardware/devices/` holds JSON profile
files. Two are bundled:

| Profile | Notes |
|---------|-------|
| `Standard.json` | Wide safety envelope, medium feedrates. Default. |
| `Conservative.json` | Tight envelope, slow feedrates. For new lab members or brittle samples. |

Each file contains `safety_limits`, `zp_stage`, and `axis_flip`
sections — the same top-level keys the rest of the app reads from
`settings.json`. Loading a profile copies these into `settings.json`
and persists `device_profile.active = "<profile_name>"` so the
selection survives launches.

### `DeviceProfile` dataclass (`gui/pages/hardware/device_profile.py`)

Owns JSON round-trip + Settings bridge. Key methods:

- `DeviceProfile.load(path) -> DeviceProfile` — read from JSON
- `DeviceProfile.from_settings(settings, name) -> DeviceProfile` — snapshot current values
- `profile.save(path=None) -> Path` — defaults to `DEVICES_DIR/<name>.json`
- `profile.apply_to_settings(settings)` — copy values into the live `Settings` instance
- Module-level `list_profiles() -> [(name, path), ...]` for the UI dropdown
- Module-level `delete_profile(path) -> bool`

### Stage sub-page updates (`gui/pages/hardware/stage_panel.py`)

A new **Device Profile** group at the top of the panel: profile combo,
Load / Save / Save As… / Delete buttons, and a status label showing the
active profile or last action. The combo refreshes from
`config/hardware/devices/` at panel construction time and via the ↻
button.

The safety-limits group is also reworked to use the canonical
`SafetyLimits` field names (`xy_min_x` / `xy_max_x` / `xy_min_y` /
`xy_max_y`, per-pump `p1_min` / `p1_max` etc.) — this fixes a v7.4.0-b
regression where the panel wrote to keys nothing else read. Separate
X-min / X-max / Y-min / Y-max spinboxes plus per-pump P1/P2/P3 min/max
spinboxes appear in the panel; the `_load_from_settings` /
`_apply` / `_reset_defaults` methods now address the correct keys.

### Sub-page order: Device first

`hardware_setup.py` registers sub-pages in a new order:

1. **⚙️ Device** — initial machine setup (was last in v7.4.0-b)
2. 🧾 Identity
3. 🔬 Plate
4. 💧 Pumps & Inks
5. 🪡 Needle
6. 🌸 Rosette
7. 📷 Cameras

Conceptually this reads as: set up the device once (Device), then
describe your experiment (everything else).

The onboarding wizard's deep-link target was updated to reflect the
new sub-page indices (Pumps & Inks moved from index 2 to index 3).

### Auto-load Standard profile on first launch

`Settings._run_migrations()` gains a new one-shot migration
(`v7_4_1_default_device`) that loads `Standard.json` and applies its
values to `settings.json` when:

- No active profile is currently set (`device_profile.active` is unset), AND
- The migration flag isn't already stamped.

The migration reads the JSON directly without importing GUI code, so
it's safe for headless contexts. New users get a sensible safety
envelope from the very first launch without touching the Stage panel.

---

## 2. New / Changed Files

```
config/
└── hardware/
    └── devices/                            # NEW directory
        ├── Standard.json                   # NEW — default profile
        └── Conservative.json               # NEW — tight-safety profile

gui/
├── app.py                                  # version bump v7.4.1;
│                                           #  wizard deep-link index update
└── pages/
    ├── hardware_setup.py                   # sub-page reorder; updated
    │                                       #  get_sub_page_title labels
    └── hardware/
        ├── device_profile.py               # NEW — DeviceProfile + helpers
        └── stage_panel.py                  # Device Profile group; safety
                                            #  fields rewritten to canonical
                                            #  SafetyLimits names

SupportClasses/
└── Settings.py                             # device_profile section;
                                            #  _apply_default_device_profile
                                            #  v7_4_1_default_device migration
```

---

## 3. Updated Core Design Principles

(Additions to the v7.4.0-a/-b/-c list — others unchanged.)

18. **Device vs experiment separation.** Settings that describe the
    physical machine (safety envelope, motor feedrates, axis direction)
    are device-scoped — saved in `config/hardware/devices/<name>.json`
    and reusable across any experiment-level HardwareConfig. Settings
    that describe an experiment (plate format, needle gauge, pumps,
    inks, rosettes, cameras) are experiment-scoped — saved in
    `config/hardware/<name>.json` HardwareConfig files. Both live
    under `config/hardware/` so they're easy to find together.

19. **Canonical settings keys are owned by the dataclass.** When a
    GUI panel reads/writes settings, the keys must match the field
    names of the consuming dataclass (`SafetyLimits.xy_min_x`, not
    `safety_limits.xy_min_um`). Bundled JSON files in
    `config/hardware/devices/` are the source of truth for the
    schema — if a panel and a profile disagree on key names, the
    profile is right.

---

## 4. Device Profile Flow

```
Fresh install (settings.json absent or empty)
        │
        ▼
Settings.load() → _run_migrations()
        │
        ├── v7_4_0_b not set? → write .bak-v7.3 backup, stamp flag
        │
        └── v7_4_1_default_device not set? →
                _apply_default_device_profile()
                        │
                        ├── reads config/hardware/devices/Standard.json
                        ├── set_section("safety_limits", ...) etc.
                        ├── set("device_profile.active", "Standard")
                        └── stamp v7_4_1_default_device = True

User changes profile via Stage sub-page
        │
        ├── _load_selected_profile()
        │       └── DeviceProfile.load(path).apply_to_settings(settings)
        │
        ├── _save_as_new_profile()
        │       └── DeviceProfile.from_settings(settings, name).save()
        │
        └── _delete_selected_profile()
                └── delete_profile(path)
```

---

## 5. Sections Unchanged

All sections from earlier architecture docs not mentioned above apply
verbatim to v7.4.1.

---
