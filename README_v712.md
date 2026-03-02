# MEBP v7.1.2 — Bug Fix Delivery

## Files Included

```
MEBP-Version-7.1-v712-bugfixes/
├── SupportClasses/
│   └── XYStageSimulator.py        ← REPLACE (BUG-2: microstep-scale speeds)
├── gui/pages/
│   └── jog_control.py             ← REPLACE (BUG-1: relative moves)
├── tests/
│   ├── __init__.py
│   ├── test_workflow.py            ← NEW (8-layer diagnostic, 45 tests)
│   └── test_integration.py         ← NEW (Phase 2: 9 groups, ~50 tests)
├── apply_v712_fixes.py             ← RUN (patches remaining files)
├── CHANGELOG_v712.md               ← Documentation
└── README_v712.md                  ← This file
```

## Installation Steps

### Step 1: Replace Files
Copy these two files directly into your project, overwriting the originals:

```bash
cp SupportClasses/XYStageSimulator.py  /path/to/MEBP-Version-7.1/SupportClasses/
cp gui/pages/jog_control.py            /path/to/MEBP-Version-7.1/gui/pages/
```

### Step 2: Copy Test Suite
```bash
cp -r tests/ /path/to/MEBP-Version-7.1/tests/
```

### Step 3: Run Patch Script
The patch script modifies 3 additional files with targeted fixes:

```bash
cd /path/to/MEBP-Version-7.1

# Preview changes (dry run)
python apply_v712_fixes.py --check

# Apply changes
python apply_v712_fixes.py
```

Patches applied to:
- `SupportClasses/StageController.py` — adds `move_xy_relative()` method
- `SupportClasses/XYStage.py` — loads protocol in sim mode, fixes `int()` → `round()`
- `config/controllers/proscan_iii.json` — adds simulator speed parameters

### Step 4: Verify
```bash
python -m pytest tests/test_workflow.py -v
python -m pytest tests/test_integration.py -v
```

## Bugs Fixed

| ID | Severity | Root Cause | Fix |
|----|----------|-----------|-----|
| BUG-1 | CRITICAL | XY jog computed absolute targets from 300ms-stale cache | Changed to relative `GR dx,dy` commands |
| BUG-2 | CRITICAL | Simulator `max_speed=100` but positions are microsteps (×10,000) | Updated to `max_speed=100,000` µsteps/s |
| BUG-3 | HIGH | Protocol JSON not loaded when `simulate=True` | Load protocol always when path provided |
| BUG-4 | LOW | `int(x)` truncates toward zero instead of rounding | Changed to `round(x)` |
| BUG-5 | LOW | Safety proximity margin=500 steps undocumented | Added documentation comments |

## Backups

The patch script creates `.v711.bak` backups of every file it modifies. To revert:
```bash
find . -name "*.v711.bak" -exec bash -c 'cp "$1" "${1%.v711.bak}"' _ {} \;
```
