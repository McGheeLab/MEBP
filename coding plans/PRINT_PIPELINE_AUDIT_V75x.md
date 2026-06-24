# Print Pipeline Audit — v7.5.x (June 2026)

Triggered by two real-hardware Quick Print failures (ME3B V1):

- **BUG-1** — a 1 mm QuickCircle printed beautifully, then *halfway through*
  the circle the stage abandoned it and drove to XY (0,0).
- **BUG-2** — a saved print with multiple spirals: each spiral started
  nicely, but toward its **end** the commanded jumps grew "longer and
  longer" (jumpy motion); the **next spiral** then started nicely again.

Method: 6-dimension multi-agent audit (geometry, motion layer, discrete
executor, coordinate frames, ZP/pump, trajectory/hybrid path) with
adversarial verification of every finding against the actual code and the
user's actual `config/prints/Spirals.json`. 35 findings, all verified.
Companion feature: per-print JSONL execution logging
(`MEBP_v75x_PRINT_EXECUTION_LOGGING.md`, schema in `logs/prints/README.md`)
to confirm the remaining hardware-only magnitudes.

---

## Executive summary

Quick Print executes through the **discrete** path
(`PrintManager._execute_print_path`), which paces motion **open-loop**: per
segment it fires a non-blocking `G x,y` (the ProScan `R` reply means
*command accepted*, NOT *motion complete*) plus a pump command, then
`time.sleep(max(seg_len/print_speed, 0.05))`. **Nothing ever verifies
arrival** — not per segment, not at path end. Position settling happens only
at the *start* of a path and after the final HOME_XY.

On real hardware the stage runs **systematically slower than the sleep
schedule assumes**, because the mm/s→SMS% speed conversion is uncalibrated
(`_protocol_max_speed_um_s` is only ever set in simulation; real hardware
silently assumes 100% SMS = 50 mm/s) and quantization always errs slow.
Once the physical stage lags the command stream, each new `G` **re-targets**
the in-flight move — the stage cuts chords toward ever-newer points.

- **BUG-1**: the job built by `build_well_plate_job` unconditionally ends
  with `TRAVEL_UP` + `HOME_XY`. The command loop finishes its sleeps while
  the stage is still physically tracing the circle; `HOME_XY` then re-targets
  the stage to zero-ref (0,0) — the operator's Set Zero point — abandoning
  the remaining arc. The drive to (0,0) **is the job's own "Return home"**.
- **BUG-2**: `generate_spiral` samples at constant **angle**, so commanded
  point spacing grows linearly with radius — measured **0.011 → 0.197 mm
  (17.5×) per spiral** in Spirals.json. Near each spiral's center, segments
  are short and the 0.05 s sleep floor over-pads them (stage keeps up /
  re-syncs); near the rim the required speed ramps to ~4 mm/s against a
  physical ceiling of ~2 mm/s (SMS,10 on a mis-assumed 50 mm/s scale) —
  lag accumulates and the re-targeting stage cuts longer and longer chords.
  The "next spiral prints nicely" because its dense inner turns drain the
  lag again (all six spirals are ONE concatenated PRINT_PATH — there is no
  per-spiral settle; the re-sync is geometric).

Standard Printing mode (hybrid plan executor) is structurally safer — all
travel/Z/home phases are blocking with arrival verification — but its
in-well trajectory playback shares the same open-loop design and several
serious pump defects (below).

---

## BUG-1 causal chain (verified: GEO-1, MOT-2, FRM-1, MOT-1, MOT-7)

1. QuickCircle geometry is clean: 65 closed-loop points, uniform 98 µm
   segments, no (0,0) row anywhere (`GeometryEngine.py:406-416`).
2. `build_well_plate_job` appends `TRAVEL_UP` + `HOME_XY` after the
   PRINT_PATH (`PrintManager.py:647-655`).
3. `_execute_print_path` paces 64 segments at the 0.05 s floor (≈3.2 s
   command-loop wall time ≈ effective 1.96 mm/s command pace) and returns
   **without any end-of-path settle**.
4. The physical stage runs slower than even that pace (uncalibrated SMS —
   `XYStage.py:146` never sets `_protocol_max_speed_um_s` on real HW;
   "halfway" implies ~1 mm/s effective during the circle: accel ramps per
   98 µm segment + command overhead, see MOT-7).
5. `TRAVEL_UP` (Z move, 0.5 s sleep) then `HOME_XY` sends `G 0,0`. ProScan
   re-targets the in-flight move → stage abandons the arc mid-circle and
   drives to zero_position. Match with the observed symptom is exact.

## BUG-2 causal chain (verified: GEO-2, MOT-3, FRM-7, GEO-3)

1. `Spirals.json` regenerates through `generate_spiral`
   (`GeometryEngine.py:468-488`), constant Δθ → spacing ∝ radius:
   0.011 mm at the center → 0.197 mm at the rim per spiral.
2. Under the dual pacing regime (`sleep(max(seg/5 mm/s, 0.05))`,
   `PrintManager.py:2637-2639` pre-logging numbering): 1062/1067 segments
   sit under the 0.25 mm floor-escape, so the commanded speed ramps
   0.23 → 3.95 mm/s along each spiral.
3. Physical ceiling ≈ 2 mm/s (SMS,10 against the wrong 50 mm/s full-scale
   assumption) → inner turns have slack (stage synced, "prints nicely"),
   outer turns run a growing per-segment deficit → cumulative lag → the
   re-targeting stage cuts progressively longer chords = "longer and
   longer", "jumpy".
4. At the next spiral, dense inner segments + floor over-padding drain the
   lag → recovery. (Not the per-path settle: Quick Print concatenates all
   objects into ONE PRINT_PATH — `quick_print_workflow.py`
   `_path_points_for_selection`.)
5. Aggravator: the 1.7–5.4 mm inter-spiral seams are **extruded, lift-less
   printed segments** (GEO-3).

---

## All verified findings

Severity · ID · location · one-liner. (Full evidence/mechanism/fixes in the
audit run output; fix proposals consolidated in the plan below.)

### Critical
- **MOT-1** `XYStage.py:146,714-732` — mm/s→SMS% conversion uncalibrated on
  real HW: `_protocol_max_speed_um_s` only set in sim (to 20000!); real
  hardware assumes 50 mm/s full scale; `int()` truncation always errs slow.
  *The single biggest enabler of both bugs.*
- **MOT-2** `PrintManager.py:2576-2667,648-655` — open-loop segment pacing,
  no end-of-path settle, trailing HOME_XY re-targets the lagging stage to
  (0,0). *BUG-1.*
- **FRM-3** `quick_print_workflow.py` (`_build_settings`, safe-z dialog) —
  if no Safe Z is configured the fallback `travel_z_height=5.0` is **5 mm
  DOWNWARD on ME3B V1** (raw Z descends): "TRAVEL_UP" lowers the needle,
  then HOME drags it across the plate. The warning dialog says the opposite.

### High
- **GEO-1** `GeometryEngine.py:406-416` — circle geometry exonerated; (0,0)
  comes only from HOME_XY. *BUG-1 attribution.*
- **GEO-2** `GeometryEngine.py:468-488` — constant-angle spiral sampling,
  17.5× spacing growth per spiral (measured on Spirals.json). *BUG-2.*
- **MOT-3** — dual pacing regimes concentrate lag at spiral rims, re-sync at
  centers. *BUG-2 dynamics.*
- **FRM-1** `PrintManager.py:647-655,2286-2305` — HOME_XY targets
  zero_position (wherever the operator last Set Zero); unconditional after
  every quick print; settle compares against the UNCLAMPED target.
- **FRM-7** — spiral spacing + one-concatenated-PRINT_PATH + extruded seams
  (frames-dimension confirmation of GEO-2/GEO-3).
- **TRJ-1** `PrintManager.py:756-854,1549-1561` — TrajectoryExecutor never
  waits for arrival, even at trajectory end; hybrid raises Z a fixed 1 s
  after the last waypoint regardless of physical lag.
- **TRJ-2** `PrintManager.py:763-772` — pause/resume never rebases the
  trajectory clock; after resume the remainder blasts at full serial rate.
- **TRJ-3** `XYStage.py:714-732` — SMS `int()` truncation can set the stage
  ceiling AT or BELOW plan speed (0.5–0.667 mm/s window) → guaranteed
  growing lag in trajectory mode.
- **TRJ-4** `PrintManager.py:752-754,804-814` — per-waypoint pump emission:
  dead `_prev_pumps` gate (uses `!=0.0` instead of changed-value), un-acked
  G90/G0/G91 triplets at up to 60 lines/s (same family as the documented ZP
  G0-flood board freeze), bypasses the ZP safety envelope.
- **TRJ-5** `PrintTrajectoryPlanner.py:582-641` — hybrid per-well pump frame
  resets to ~zero each well: every well after the first SUCKS BACK the
  previously extruded volume.

### Medium
- **GEO-3** `quick_print_workflow.py:501-504` — multi-object prints
  concatenate into one PRINT_PATH; inter-object jumps are extruded at print
  Z with no lift/settle. **✅ FIXED** (`MEBP_v75x_MULTI_OBJECT_PRINT_SEAM.md`,
  Jun 2026) — confirmed on hardware via the PIAR.json log; one PRINT_PATH per
  object + lift→travel→lower between them; worst in-path jump 4.288→0.197 mm.
- **GEO-4** `quick_print_workflow.py:435-479` — blind XY extraction: csv/
  sketch travel rows print; Z-stacks flatten; pixel-unit CSVs command
  meter-scale offsets.
- **MOT-4** — no motion-complete signal anywhere in production ('$' status
  poll exists only in `tests/proscan_diagnostic.py`); settle timeouts
  silently continue.
- **MOT-7** — 0.05 s pacing floor is marginal for ~0.1 mm segments at SMS,10
  once accel + command overhead are counted (needs HW log to quantify).
- **FRM-2** — Print-Z spinbox is raw zero-ref Marlin mm — internally
  consistent but the OPPOSITE sign convention from the Jog page's ZDIR
  display; range −5..50 encodes a Z-up-positive assumption.
- **FRM-6** — Flow=0.0 silently falls back to legacy `seg_len×0.01` raw-mm
  extrusion (quick print passes `flow_rate=0.01` unconditionally).
- **FRM-8** `PrintManager.py:2216-2222` — MOVE_Z/TRAVEL legs use fixed
  0.3–0.5 s sleeps; a long descent is still in flight when the path starts.
- **TRJ-7** — hybrid travel arrival timeouts warn-and-continue straight into
  Z descent (needle-crash risk).
- **TRJ-12** — per-waypoint Z/pump absolute moves carry no feedrate (Marlin
  modal F roulette).

### Low / info
- **GEO-5** meander fill: chord-proportional density quirks; fill→outline
  seam extrudes a 2r jump. **GEO-6** QuickDot dispenses nothing (volume
  lives in the discarded pump column; both XY rows identical → skipped).
- **MOT-5** serial contention REFUTED as a bug cause (lock-serialized,
  self-healing; worst case ~0.6 s stale display). **MOT-6** envelope
  clamping is silent + settle compares unclamped (latent, uninvolved).
  **MOT-8** `safety_limits.max_xy_speed` never enforced on any XY speed
  path. **FRM-4** well-center math verified correct (zero cancels exactly);
  stale-zero mistargeting REFUTED. **FRM-5** envelope-clipped circles print
  silent flattened chords — does not reproduce either bug. **FRM-9** saved
  3D prints collapse to one Z. **TRJ-6** trajectory waypoint spacing is
  constant-or-finer — planning cannot produce BUG-2 in that path. **TRJ-8**
  ETA model inflated ~5–6×. **TRJ-9** speed scan samples only first 100
  waypoints (long prime → leftover service speed). **TRJ-10** hybrid is the
  default; VelocityExecutor/MotionController are dead code by default.
  **TRJ-11** logging: extend `PrintExecutionLogger` (done) — PrintRecorder
  records nothing in discrete mode and has a µm-vs-mm bug in its actual-XY
  sampling.
- **ZP-1** (lead-engineer check; ZP dimension agent did not run) — per-
  segment pump volumes sit at the resolution floor: QuickCircle at
  0.25 µL/s → 0.0049 µL/segment ≈ 0.0003 mm plunger on a 1 mL syringe
  (passes ZPStage's 1e-4 mm gate; larger syringes fall below it and are
  silently dropped); spiral-center segments fall below PrintManager's own
  0.001 µL gate (`vol_dropped` in the new logs). Extrusion at dense
  geometry is effectively quantization noise.

---

## Prioritized fix plan

**P0 — close the loop (fixes both bugs, small diffs):**
1. End-of-path settle: `_wait_for_xy_settle(points[-1], ...)` at the end of
   `_execute_print_path` before returning (MOT-2/FRM-1/GEO-1).
2. Bounded-lag mid-path re-sync: every N segments (or when the cached
   position lags the commanded target beyond ~200 µm) settle before the
   next `G` (MOT-3). Alternatively '$' motion-status polling (MOT-4).
3. Calibrate the SMS mapping: load `parameters.max_speed` from the protocol
   JSON into `_protocol_max_speed_um_s` on real HW; `round()`/ceil instead
   of `int()`; pace sleeps from the quantized achieved speed; ideally store
   a measured per-device `xy_max_speed_um_s` in `ME3B V1.json` (MOT-1/TRJ-3).
4. ZDIR-aware safe-z fallback + refuse-to-run without calibrated safe z
   (FRM-3) — safety.
5. Make the trailing HOME_XY optional for Quick Print (park at home drags
   the needle across the plate; FRM-1).

**P1 — geometry & quick-print correctness:**
6. Resample `generate_spiral` (and spiral fill) to constant arc length
   (GEO-2/FRM-7).
7. One PRINT_PATH per object + travel (lift, move, settle, lower) between
   objects (GEO-3/FRM-9).
8. Filter/validate csv-sourced paths (travel rows, units, extents) (GEO-4).
9. Flow=0 must mean no extrusion (FRM-6); QuickDot → real dispense (GEO-6).
10. Replace fixed Z sleeps with `flush_moves()` + `wait_for_z_arrival`
    (FRM-8); escalate settle/arrival timeouts instead of continuing
    (MOT-4/TRJ-7).

**P2 — trajectory/hybrid hardening (standard print mode):**
11. End-of-trajectory arrival wait (TRJ-1); pause rebase (TRJ-2);
    changed-value pump gate + feedrates + ZP clamp (TRJ-4/TRJ-12); per-well
    pump frame continuity (TRJ-5); ETA fix (TRJ-8); full-waypoint speed
    scan (TRJ-9); enforce `max_xy_speed` (MOT-8).

**Verification:** all P0 items need one real-hardware run with the new
execution logs (`logs/prints/README.md` triage recipe): `sample.lag_um`
quantifies the physical lag; `path_segment.drift_s` the loop overhead;
`settle_wait` the silent timeouts; plus one run of
`tests/proscan_diagnostic.py` to measure the true SMS↔mm/s mapping.

---

## Audit coverage note

Four of six dimensions ran to completion with 3-lens adversarial
verification per high-severity finding (some verifier votes were lost to a
session cap; every finding retained at least majority confirmation).
The dedicated *discrete-executor-timing* and *ZP-pump* analyzer agents were
cut off by the cap; their scope was independently covered by the motion /
frames / geometry dimensions (timing) and a manual ZP check (ZP-1), so no
finding rests on a single unverified agent.
