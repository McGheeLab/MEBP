# MEBP v7.5.x — Prior ProScan III baud fix + multi-baud detection scan

## Objective

Fix "the Prior ProScan III is connected for sure, but the software says it can't
find it" on the ME3B V3 machine (operator, after physically swapping a Ludl MAC
5000 back out for a Prior ProScan III). Root-cause it, fix it on the actual
hardware, and make detection resilient to the same class of failure recurring
(Prior's own manual documents that a changed baud setting can silently revert).

## Root causes (both real, independent, found via live read-only diagnostics)

1. **Baud mismatch.** `proscan_iii.json` assumes 38400 baud. This specific unit
   (identifies as `H117P1N4/F` via `STAGE`) was sitting at **9600** — Prior's own
   manual documents no DIP switch exists for baud (confirmed absent from a full-text
   search of the official manual); it's a firmware/EEPROM setting, and it **reverts
   to the 9600 factory default if the serial port sits idle across two power
   cycles**. This unit had almost certainly hit that fail-safe.
2. **`V` (firmware_version) doesn't return a version string on this unit** — it
   replies with a bare `R` (confirmed via an isolated, generously-timed, zero-motion
   probe at both 9600 and 38400 — not a timing/baud artifact, a genuine firmware
   behavior). `proscan_iii.json`'s detection is built entirely around `V` returning
   `"ProScan"`/a version-number pattern, so it could never match this unit even at
   the right baud. `STAGE`, however, reliably returns a rich, distinctive multi-line
   descriptor (`STAGE = H117P1N4/F\rTYPE = 25\r...`).

## Fix

**Hardware (real, live, done on 2026-07-23):** connected at 9600 (confirmed
working), sent Prior's documented `BAUD,<code>` command (research-confirmed against
Prior's own official ProScan II/III manuals — codes `96`/`19`/`38`/`115` for
9600/19200/38400/115200; ack is `0`), stepped it up **9600 → 38400 → 115200**,
verifying communication after each step before proceeding. **115200 is confirmed
working — the maximum this unit supports** (Rev F / USB-FTDI board, matching the
`/F` in its `STAGE` identifier and Prior's "ProScan III Communication Speed"
technical note gating 115200 to Rev F hardware).

**Software:**
- New `config/controllers/proscan_iii_h117.json` — same Prior III wire command
  syntax as `proscan_iii.json` (verified identical `format_command` output for every
  command), but: `detection.firmware_query`/`wake_command` = `STAGE` (not `V`),
  `identify_tokens: ["STAGE =", "MICROSTEPS"]`, `wake_delay_ms: 300` (300ms alone
  produced a truncated/spurious reply on the first STAGE call — confirmed
  empirically that ~600ms total settle is needed for the full multi-line reply to
  arrive before the wake-step drains it); `communication.baud_rates: [115200, 38400,
  9600]` (highest/preferred first).
- **`ControllerProtocol.baud_rate_candidates`** (new) — reads optional
  `communication.baud_rates`; absent ⇒ `[baud_rate]` (the existing single value),
  so every protocol that doesn't declare it (Prior II, Prior III, Ludl) is
  byte-identical to before. `XYStage._find_with_protocol` now loops over candidate
  bauds **per port** (opening/closing at each in turn) instead of trying only the
  protocol's single `baud_rate` — this is the resilience piece: if this unit's baud
  ever reverts to 9600 again (idle across 2 power cycles, per Prior's documented
  fail-safe), detection still finds it without operator intervention.
- `ME3B V3.json` device profile + `settings.json` `controller.controller_json` →
  `config/controllers/proscan_iii_h117.json` (via the safe load-modify-save
  pattern — see Issues & Decisions below).

## Files Modified / Created

**Modified:** `SupportClasses/ControllerProtocol.py` (`baud_rate_candidates`),
`SupportClasses/XYStage.py` (`_find_with_protocol` per-port baud loop).
**Created:** `config/controllers/proscan_iii_h117.json`,
`tests/test_v75x_prior_multi_baud_detect.py`.
**Data (via proper load-modify-save, not hand-edited):**
`config/hardware/devices/ME3B V3.json` (`xy_controller_json`), `settings.json`
(`controller.controller_json`).

## Testing Notes

- `tests/test_v75x_prior_multi_baud_detect.py` (8 tests): `baud_rate_candidates`
  defaults to `[baud_rate]` for Prior II/III/Ludl (regression guard); the new
  H117 protocol's family/baud-list/detection/wire-syntax; a mocked-serial test
  proving `_find_with_protocol` tries every candidate baud on one port before
  moving to the next port.
- Full suite re-run: 114 tests green (multi-baud + Ludl + jog/speed/axis-map
  regression) — zero change to existing Prior/Ludl behavior.
- **Real hardware, through the actual production code path** (not a raw script):
  `XYStageManager(controller_json="config/controllers/proscan_iii_h117.json",
  exclude_ports=["COM4"])` on the ME3B V3 machine → detected as "Prior ProScan III
  (H117 stage, STAGE-based detect)", connected at 115200 (first candidate, matched
  immediately), live position read (1952.0, -1770.0 µm), speed/accel readback both
  100% (confirms `SMS`/`SAS` bare-query readback works normally on this unit even
  though `V` doesn't — the firmware quirk is specific to `V`).

## Issues & Decisions

- **New JSON file, not a shared-file edit** — per the project's established
  "add a variant file, don't touch the shared protocol" pattern (used identically
  for the Ludl work): `proscan_iii.json` is unmodified, so any OTHER Prior III
  installation whose `V` query works normally is completely unaffected.
- **Named after the stage model (`_h117`), not the baud** — the baud was bumped
  twice during this session (9600 → 38400 → 115200 at the operator's request); a
  filename baked to one of those would have gone stale. The `STAGE`-detection
  quirk is tied to this specific unit's firmware, not the baud, so that's the
  stable identifier.
- **A real mistake, caught and recovered:** an earlier attempt to persist the
  `controller.controller_json` setting via `Settings()` without first calling
  `.load()` wrote near-default values over the live `settings.json`, wiping
  `device_profile.active` and several other sections. Recovered via
  `git restore --source=HEAD -- settings.json` (the file was tracked before
  `.gitignore` added it, per operator's own suggestion to "pull settings from the
  last version in github") — verified byte-identical to HEAD except the one
  intended field afterward. **All settings.json/device-profile writes in this
  session and going forward use the load-modify-save pattern** (`Settings(); s.load();
  s.set(...); s.save()` / `DeviceProfile.load(path)` → mutate → `.save(path)`),
  never a bare constructor.
- **A second, related bug found and fixed in the same investigation:** the XY
  controller combo on Hardware Setup → Device (`_on_xy_controller_changed`)
  persisted to `settings.json` but never called `_persist_active_profile()` (unlike
  "Save XY Calibration", which did) — so the active device profile FILE stayed
  stale, and the next explicit profile Load would silently revert a fresh
  controller choice. Fixed in both `stage_panel.py` and (mirrored, since
  `settings_page.py`'s global dropdown had the identical gap)
  `gui/pages/settings_page.py` (`_sync_active_device_profile_controller`).
- **Fail-safe awareness, not just a fix:** rather than treat "found it at 115200"
  as the end state, `baud_rate_candidates` makes the fix durable against Prior's
  own documented reversion behavior — the exact failure mode that caused this
  session's investigation in the first place.
