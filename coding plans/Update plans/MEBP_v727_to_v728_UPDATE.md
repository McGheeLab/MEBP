# MEBP v7.2.7 → v7.2.8 Update Plan

## Objective

Hardware communication diagnostics and fixes: XY controller auto-detection, ProScan CR-terminator handling, atomic position reads, Xbox status indicator fix, and simulation defaults correction.

---

## Bug Fixes

### BF-1 — XYStage controller detection fails when controller_json=None

**Files:** `SupportClasses/XYStage.py`

**Symptom:** When no controller JSON is specified, XYStage defaults to ProScan III instead of auto-detecting the connected controller.

**Root cause:** `_load_protocol(None)` assumed ProScan III. Protocol was not loaded in simulation mode, leaving `microsteps_per_micron` and other parameters at wrong defaults.

**Fix:** When `controller_json` is None/auto, try all protocol JSONs in `config/controllers/` directory. Always load protocol in `__init__` (even for simulation) to get parameters. Auto-detect iterates all available protocols.

**Status:** `[x]` done

---

### BF-2 — ProScan II/III detection and response handling (Diagnostic-based)

**Files:** `SupportClasses/XYStage.py`, `SupportClasses/XYStageSimulator.py`, `config/controllers/proscan_ii.json`, `config/controllers/proscan_iii.json`, `config/xy_diagnostic_profile.json`, `main.py`, `SupportClasses/StageController.py`

**Symptom:** ProScan II not detected. Overly-broad "E" token in ProScan III config causes false matches. Slow response reads due to wrong line terminator handling. Simulator timing doesn't match real hardware.

**Root cause:** Multiple interacting issues discovered via hardware diagnostic profiling session.

**Fix (7 sub-fixes):**
- A: XYStageSimulator loads timing from `config/xy_diagnostic_profile.json` (replaces hardcoded)
- B: Created `xy_diagnostic_profile.json` config with measured hardware timings
- C: Fixed ProScan II detection tokens in `proscan_ii.json`
- D: Fixed overly-broad "E" token in `proscan_iii.json` (made more specific)
- E: Added CR-aware `_read_response_cr()` helper to XYStage (ProScan II uses CR not LF)
- F: main.py passes `controller_json` from settings to StageController
- G: StageController error handling + poll interval fix

**Status:** `[x]` done

---

### BF-3 — Position read race condition

**Files:** `SupportClasses/XYStage.py`

**Symptom:** Position display occasionally shows stale or garbled values.

**Root cause:** `get_current_position()` had separate lock acquisitions for write (send "P" command) and read (readline). PositionPoller could interleave between them.

**Fix:** Atomic lock around write+read in `get_current_position()` and `get_firmware_version()`. Single `_serial_lock` acquisition covers the entire send-receive cycle.

**Status:** `[x]` done

---

### BF-4 — Xbox controller status indicator mismatch

**Files:** `SupportClasses/StageController.py`, `gui/app.py`, `gui/pages/dashboard.py`

**Symptom:** Dashboard shows Xbox connected but top-bar shows disconnected (or vice versa).

**Root cause:** `xbox_status` was a method in some code paths and a property in others. `XboxQueuePoller` logged every heartbeat (console spam), making status hard to track.

**Fix:** Ensured `xbox_status` is consistently a `@property`. XboxQueuePoller logs only status *changes*. Normalized `on_status_update` Xbox logic in both app.py and dashboard.py with proper unpolish/polish refresh.

**Status:** `[x]` done

---

### BF-5 — Simulation defaults incorrect

**Files:** `SupportClasses/Settings.py`, `main.py`

**Symptom:** App launches in simulation mode when real hardware is available.

**Root cause:** Default values `simulate_xy=True`, `simulate_zp=True` in Settings.py and main.py.

**Fix:** Changed defaults to `simulate_xy=False`, `simulate_zp=False`. Updated settings.json if still set to True.

**Status:** `[x]` done

---

### BF-6 — Windows CRLF line ending handling in patches

**Files:** `SupportClasses/XYStage.py`

**Symptom:** Patches E and F from BF-2 fail to apply on Windows due to CRLF line endings.

**Fix:** Handle Windows line endings explicitly in `_read_response_cr()` and `_auto_detect_controller()`.

**Status:** `[x]` done

---

## Implementation Steps

- [x] BF-1: Auto-detect controller when controller_json=None
- [x] BF-2: Diagnostic-based ProScan fixes (7 sub-fixes A-G)
- [x] BF-3: Atomic position reads (lock around write+read)
- [x] BF-4: Xbox status indicator consistency
- [x] BF-5: Fix simulation defaults to False
- [x] BF-6: Windows CRLF handling for serial reads

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/XYStage.py` | Auto-detect, _read_response_cr(), atomic position reads, CRLF |
| `SupportClasses/XYStageSimulator.py` | Load timing from diagnostic profile |
| `SupportClasses/StageController.py` | Error handling, poll interval, xbox_status property |
| `SupportClasses/Settings.py` | simulate defaults = False |
| `config/controllers/proscan_ii.json` | Fixed detection tokens |
| `config/controllers/proscan_iii.json` | Fixed overly-broad "E" token |
| `config/xy_diagnostic_profile.json` | **New** — measured hardware timing profile |
| `main.py` | Pass controller_json from settings, simulate=False default |
| `gui/app.py` | Xbox status normalization |
| `gui/pages/dashboard.py` | Xbox status normalization, unpolish/polish refresh |

## Testing Notes

1. Launch with ProScan II connected (no controller_json) → verify auto-detection
2. Launch with ProScan III → verify correct detection (not false "E" match)
3. During jog → verify position display stable (no garbled values)
4. Connect Xbox → verify dashboard and top-bar both show connected
5. Disconnect Xbox → verify both indicators update to disconnected
6. Launch with real hardware → verify simulation mode NOT active by default

## Issues & Decisions

- **Diagnostic profiling approach**: Rather than guessing timing values, ran a hardware diagnostic session to measure actual ProScan response times. Results stored in `xy_diagnostic_profile.json` for simulator calibration.
- **CR vs LF terminator**: ProScan II uses CR (`\r`) termination, not CRLF. Standard `readline()` blocks waiting for LF that never comes. `_read_response_cr()` uses `read_until(b'\r')` with timeout.
- **Atomic position reads**: Single lock acquisition for the entire send-receive cycle prevents interleaving. This is the correct fix — removing locks from individual operations just moves the race condition.
- **Simulation defaults**: Changed to False so the app defaults to real hardware. Users can explicitly enable simulation in settings when needed.
