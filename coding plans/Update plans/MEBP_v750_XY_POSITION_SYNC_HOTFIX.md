# MEBP v7.5.0 — XY Position Sync Hotfix

## Objective

Fix the bug where the displayed XY stage position lags badly and *slowly counts down*
toward the correct value over several seconds after the stage moves — and where
**Set Zero** does not snap to 0 but instead drains down to it. The position display
should now reflect the current hardware position within one poll cycle (~0.3 s)
regardless of any accumulated junk in the serial RX buffer.

## Root Cause

The Prior ProScan controller acks nearly every command with `R\r` (or a position
string). The background `PositionPoller` (`SupportClasses/StageController.py`) sends
`P` every ~0.3 s and reads exactly **one** CR-terminated line via `_read_response_cr()`,
without flushing first. Any unconsumed ack sits in the FIFO RX buffer, so the poller
reads stale data and drains the backlog one line per cycle — the "slow countdown."

Two sources orphaned acks:

1. `set_home()` (`Z`, used by Set Zero) and `stop_stage()` (`I`) routed through
   `send_command()`, which only *writes* — no serial lock, no ack read. The three
   movement methods were already fixed (v7.2.9 / v7.3.4) but these two were missed.
2. The fixed movement methods drain with a tight `timeout=0.05`, so under rapid jogging
   an ack can arrive after the drain window and still accumulate.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/XYStage.py` | (1) `get_current_position()` now calls `self.spo.reset_input_buffer()` inside the serial lock before sending the position query — the bulletproof cure. (2) `set_home()` and `stop_stage()` rewritten to drain their `R` ack under `_serial_lock` (sim path unchanged). |

## Implementation Steps

- [x] Flush RX buffer before each hardware position read in `get_current_position()`.
- [x] Rewrite `set_home()` to write + drain ack under `_serial_lock` on the hardware path.
- [x] Rewrite `stop_stage()` to write + drain ack under `_serial_lock` on the hardware path.
- [x] Syntax-check module (`ast.parse` OK).

## Testing Notes

- **Simulated** (`python main.py`): regression — sim path is unchanged (sim uses a
  separate non-blocking position read), so jogging and Set Zero behave as before.
- **Real hardware** (ProScan on Hardware Setup):
  - Move/jog the stage (incl. sustained Xbox jogging); displayed X/Y should track within
    ~1 poll cycle with no progressive lag.
  - **Set Zero** (per-axis and both) should snap the display to 0.00 immediately.
  - **Stop** mid-move should leave subsequent position reads accurate (no stale-`R` lag).
  - Optional: `_dbg.log("POLL", raw_rx=...)` should show consistent `x,y,z` triples,
    no lone `R` lines.

## Issues & Decisions

- Chose to flush the RX buffer on every poll (rather than only fixing the two missed
  methods) because it makes the poller self-healing against *any* missed ack, including
  the timing-sensitive jog case. Safe because all senders drain their own ack
  synchronously under the same `_serial_lock`, so anything buffered when the poller
  acquires the lock is stale.
- ZP stage not touched — it already drains via `read_all()`.
- `StageController.zero_axis()` logic unchanged — software offset → 0 and hardware `Z` →
  raw 0 already compose correctly (displayed = 0 − 0 = 0).
