# MEBP v7.5.x — ZP Jog Clamp Freeze Fix

## Objective

**Reliability/safety fix.** While continuously jogging a ZP axis (Z or a
pump, via Xbox or hold-buttons) **against a soft limit**, the Marlin board
**froze and the poller dropped the connection**. The log showed a position
clamp ("`Z clamped: … → …`") immediately before the freeze.

## Root Cause

The ZP jog loop (`ZPJogHandler._jog_loop`, `segment_time = 0.12 s` ≈ 8 Hz)
clamps each segment's delta against the cached poller position
(`lambda: self._pos_poller.zp_position`). When an axis is pinned at a soft
limit, the clamp result is

```
clamped_delta = clamp(boundary) + ref − current ≈ a few µm of float noise
```

i.e. a tiny non-zero **residual**. Two compounding bugs in
`ZPStage.move_relative` then turned that residual into a board freeze:

1. **The zero-move gate was `abs(d) > 1e-6`** — far below any real motion —
   so the residual passed, and the jog loop emitted **one micro-move every
   segment, forever**, while the stick was held at the limit. A continuous
   stream of un-acked `G0` commands; the board eventually froze and the
   v7.5.0 poller-liveness watchdog declared it disconnected (~2.5 s of failed
   reads).
2. **Raw `f"{d}"` formatting** rendered those tiny values in **scientific
   notation** (verified: `0.00009 → "Z9e-05"`, `1.5e-05 → "Z1.5e-05"`,
   `-2.7e-05 → "Z-2.7e-05"`), which not every G-code parser accepts —
   malformed commands compounding the stall.

(A sibling freeze path also existed: a feedrate clamped to **0** emits
`G0 F0`, which Marlin treats as an infinite-time move → planner stall. Guarded
at the same chokepoint.)

## Fix

All in `SupportClasses/ZPStage.py`, at the single G-code emission chokepoint:

- `move_relative`:
  - **Drop sub-resolution moves**: gate raised to `abs(d) >= 1e-4` mm
    (0.1 µm ≈ one Marlin step). A fully-clamped jog now sends **nothing** —
    the loop idles instead of flooding the board.
  - **Fixed-decimal formatting** `f"{d:.4f}"` — never scientific notation
    (matches the print-path convention; `PrintManager` uses `:.4f`/`:.5f`).
  - **Feedrate floor** `fr = max(float(fr), 1.0)` — never emit `F0`.
- `move_absolute`: fixed-decimal `f"{p:.4f}"` formatting too.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | `move_relative`: zero-move gate `1e-6 → 1e-4`, fixed-decimal format, feedrate floor (≥1). `move_absolute`: fixed-decimal format. |
| `tests/test_v75x_zp_jog_clamp_freeze.py` | New — 8 tests. |
| `coding plans/Update plans/MEBP_v75x_ZP_JOG_CLAMP_FREEZE.md` | This plan. |

## Implementation Steps

- [x] Localize: ZP jog + position(Z) clamp → freeze (user-confirmed).
- [x] Trace `_jog_loop` → `_clamp_delta` (residual) → `move_relative` (gate + format).
- [x] Reproduce the scientific-notation formatting of tiny residuals.
- [x] `move_relative`: raise gate to 1e-4, `:.4f` format, feedrate floor.
- [x] `move_absolute`: `:.4f` format.
- [x] Add `tests/test_v75x_zp_jog_clamp_freeze.py` (8 tests).
- [x] Regression: `test_v75x_zp_dtr_no_reset` + `..._position_override` + `..._position_restore` + `..._axis_map_jog` (36 pass).
- [ ] **Real-hardware verification**: jog hard into a soft limit and hold — board must stay alive (no freeze/disconnect).

## Testing Notes

```
python -m unittest tests.test_v75x_zp_jog_clamp_freeze -v          # 8 pass
python -m unittest tests.test_v75x_zp_dtr_no_reset \
                   tests.test_v75x_zp_position_override \
                   tests.test_v75x_zp_position_restore \
                   tests.test_v75x_axis_map_jog                     # 36 pass
```

**Real hardware:** continuously jog Z (and a pump) into a soft limit and
**hold** the stick/button. Before: board froze + disconnected. After: the
axis stops at the limit and the board stays responsive (no commands are sent
once the clamp zeroes the motion).

## Issues & Decisions

- **Threshold 1e-4 mm (0.1 µm).** Below one Marlin step and below any
  intentional jog segment (slowest realistic segment ≳ 1e-3 mm), so no real
  motion is dropped; it only suppresses clamp-residual noise.
- **Fixed the chokepoint (`move_relative`), not the jog loop.** All ZP G0
  motion flows through here, so every caller (jog loop + discrete jog
  buttons) is protected, and the change is minimal/low-risk.
- **Residual log spam not addressed.** While held at a limit, `clamp_z` still
  logs a warning each 8 Hz iteration (no longer a freeze — just log noise).
  Rate-limiting the clamp warnings is a possible cosmetic follow-up; left out
  to keep this fix focused and the shared `clamp_*` methods untouched.
- **Feedrate floor included** though the user's trigger was a position clamp:
  it's the same emission chokepoint and closes the `F0` sibling freeze at
  near-zero risk (only affects feedrates < 1 mm/min, which are non-physical).
