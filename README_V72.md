# MEBP v7.2 — Microlitre Pump Control Upgrade

## What's New

v7.2 replaces millimetre-based pump control with microlitre-based operations throughout the application. Users now see and enter all pump values in **µL** and **µL/s**, while the backend handles conversion to mm/mm-min for Marlin firmware.

### Key Features
- **Hardware Setup Page** (Page 0) — Configure needle, syringes, inks, and plate format before printing
- **Page Gating** — Other pages locked until hardware config is valid
- **µL Jog Control** — Pump steps in µL, rates in µL/s, position readout in µL
- **µL Print Commands** — `amount_uL`, `rate_uL_s`, `flow_rate_uL_s` in print files
- **Auto Safety Limits** — Flow rate limits auto-set from needle gauge
- **Print File Migration** — v7.1 files auto-detected and convertible to v7.2

---

## Deployment

### Step 1: Copy New/Replacement Files

These files go directly into your repo, replacing any existing versions:

| Source | Destination | Action |
|--------|-------------|--------|
| `SupportClasses/HardwareConfig.py` | `SupportClasses/HardwareConfig.py` | **NEW** |
| `gui/app.py` | `gui/app.py` | **REPLACE** |
| `gui/pages/hardware_setup.py` | `gui/pages/hardware_setup.py` | **NEW** |
| `gui/pages/jog_control.py` | `gui/pages/jog_control.py` | **REPLACE** |
| `config/hardware/sample_setup.json` | `config/hardware/sample_setup.json` | **NEW** |
| `config/prints/sample_v72_print.json` | `config/prints/sample_v72_print.json` | **NEW** |

### Step 2: Run Patch Scripts

The master runner auto-detects the project root, so you can run it from either location:

```bash
# Option A: From the patches directory
cd patches/v72
python apply_all_v72_patches.py

# Option B: From the project root (if you copied patches there)
python patches/v72/apply_all_v72_patches.py
```

This applies 5 patches in order:
1. `apply_v72_patches.py` — StageController µL methods + __init__ exports + Settings defaults
2. `apply_v72_printmanager_patch.py` — PrintManager µL commands + migration functions
3. `apply_v72_dashboard_patch.py` — Dashboard µL pump display
4. `apply_v72_printsetup_patch.py` — Print Setup µL settings panel
5. `apply_v72_sessions3to6_patch.py` — Calibration, SafetyLimits, Xbox, Monitor, Settings pages

### Step 3: Run Tests

```bash
python -m pytest tests/ -v
# Or individually:
python tests/test_hardware_config.py
python tests/test_printmanager_v72.py
python tests/test_v72_integration.py
```

---

## File Inventory

```
McGheeLab/MEBP/
├── SupportClasses/
│   └── HardwareConfig.py          # NEW: Central µL↔mm conversion + config model
├── gui/
│   ├── app.py                     # REPLACE: Page 0 + gating + config propagation
│   └── pages/
│       ├── hardware_setup.py      # NEW: Needle/syringe/ink/plate config page
│       └── jog_control.py         # REPLACE: µL jog steps/rates/display
├── config/
│   ├── hardware/
│   │   └── sample_setup.json      # NEW: Example hardware config
│   └── prints/
│       └── sample_v72_print.json  # NEW: Example v7.2 print file
├── docs/
│   └── ARCHITECTURE_V72_ADDENDUM.md  # v7.2 architecture documentation
├── tests/
│   ├── test_hardware_config.py    # 15 unit tests
│   ├── test_printmanager_v72.py   # 15 unit tests
│   └── test_v72_integration.py    # 25+ integration tests
└── patches/v72/
    ├── apply_all_v72_patches.py   # Master runner
    ├── apply_v72_patches.py       # Session 1 patches
    ├── apply_v72_printmanager_patch.py
    ├── apply_v72_dashboard_patch.py
    ├── apply_v72_printsetup_patch.py
    ├── apply_v72_sessions3to6_patch.py
    ├── V72_UPGRADE_PLAN.md        # Planning document
    └── reference/                 # Superseded guides (for reference only)
```

## Backward Compatibility

- v7.1 print files load and execute normally (legacy `amount`/`feedrate` in mm)
- When no syringe configured, all displays fall back to mm
- `migrate_print_file_v71_to_v72()` converts files when HardwareConfig available
- settings.json auto-merges new `hardware_config` field on first launch
