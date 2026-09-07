# MEBP v7.21.5 — the sketch's exact output, extrusion and all, reaches Quick Print

## Objective

> *"the exact output of the print sketch including a calculation of the extrusion
> modifier along the printing path should be sent to the quick print."*

The **toolpath** already transferred faithfully — `save_trajectory_as_print_object`
writes the full Nx7 CSV, pump columns and all. The **extrusion** did not. Quick
Print read that trajectory, kept only the XY of each sub-path (the pump column
was used solely as a travel mask), and re-derived **one** flow for the whole
print from its own `Flow @100%` / `Extrusion ×` / bore area. So:

- a sketch whose shapes declare different line widths printed every one of them
  at the same width — which, after v7.21.4 made an outline a single pass whose
  width comes from flow, meant the declared width had *no* effect on the machine
  at all;
- a section marked **no extrude** still extruded once it went through Quick
  Print;
- and the plan doc for v7.21.4 had to state a limitation — *"the multiplier is
  sketch-wide, so shapes with different line widths cannot all match at once"* —
  which is exactly what this closes.

The chain now, end to end:

```
shape.line_width_mm
  → compile_to_trajectory   per-shape modifier + per-SEGMENT profile
  → baked print params      extrusion_profile / vol_per_mm_profile / ref width
  → Quick Print             split with the sub-paths → µL/mm vs the needle now
  → build_well_plate_job    PRINT_PATH.vol_per_mm_profile
  → the executor            deposition per segment (all four sites)
```

---

## 1 — The sketch computes an extrusion along the path

`compile_to_trajectory` had one scalar `vol_per_mm` for the whole compile. It is
now split into a **base** (the orifice cross-section = deposition at modifier
1.0) and a **per-shape modifier**:

```
modifier(shape) = extrusion_multiplier × line_width_mm / ref_width
ref_width       = the orifice INNER Ø  (fallback: the fill pitch)
```

`ref_width` is the inner Ø deliberately: it is the same reference the Sketch
page's shaded band and its *"this width needs N×"* readout already use, so
**what the page reports is what the compiler applies**.

`move_to` stamps every segment it emits with the modifier in force, filling
`extrusion_profile` (bore-relative) and `vol_per_mm_profile` (absolute µL/mm) in
**lockstep with `waypoints`** — by construction they cannot drift out of
alignment with the toolpath. A travel, a lift and a hold-pressure retrace all
come out `0.0`, which is also what tells a downstream consumer where the bead
stops. `total_volume_uL` is now **summed per segment** instead of
`total_length × one scalar` — with a per-shape modifier those are no longer the
same number.

**`no_print` is enforced in exactly one place** — `move_to`'s `printing` flag.
A first cut also checked it in `_shape_modifier`; a mutation proved that second
check was dead code, so it was deleted (two enforcement points for one fact is
how a real guard gets removed later without any test noticing).

### The migration that had to be explicit

Turning this on for an existing sketch would multiply its deposition, because
before v7.21.5 a shape's width was never an extrusion instruction **and** the
page seeded new shapes from the orifice **OUTER** Ø while 1× is the **INNER** Ø
— od/id ≈ **1.7×** on a hypodermic needle. So:

- `Sketch.width_drives_extrusion` defaults **True** for a new sketch and
  **False for any sketch loaded from a dict without the key** (`from_dict` uses
  `d.get(..., False)`), so re-baking a saved sketch deposits exactly what it
  always did. A checkbox on the Print-parameters card opts one in, and its
  tooltip says the volume can change.
- The page now seeds a new shape's `line_width_mm` from the **1× reference**
  (inner Ø), so a freshly drawn shape is exactly 1× and nothing surprising
  happens in the GUI.

---

## 2 — NEW `SupportClasses/ExtrusionProfile.py`

One place that describes "how much per mm, WHERE", and one place that looks it
up. Pure — stdlib only, no numpy, no Qt, no repo imports — because it is
produced by the Sketch compiler and Quick Print and consumed on the print
thread.

- `ExtrusionProfile.build(points, vol_per_mm)` → `None` when the profile is
  absent, empty, the wrong length or not finite. **Refusing beats guessing:** an
  off-by-one profile applies a shape's flow to the wrong stretch of path —
  silently laying a thick bead where a thin one was designed — whereas falling
  back to the scalar is merely the old behaviour.
- `at_index(i)` for the discrete executor (it walks segments) and
  `at_arclen(s)` for the velocity followers (they advance an arc-length cursor).
  Both come off **one** cumulative-length table so they cannot disagree; a
  position exactly on a waypoint takes the segment it is *entering*, so the
  deposition changes at the boundary rather than one segment late.
- Built against the **RAW** points, so the closed-loop follower — which
  de-duplicates coincident waypoints — can still look it up by arc length
  (dropping zero-length segments cannot change an arc length).
- `modifier_to_vol_per_mm(mods, bore_area, trim)`: **absolute µL/mm is the
  transferable quantity.** A modifier is bore-relative and only means something
  next to the needle it was measured against.
- `rate_for(vol_per_mm, speed, fallback)`: **the pump rate has to track the
  volume.** The discrete executor paces each segment by
  `max(xy_move_time, pump_move_time)`, so holding the rate fixed would just make
  a thicker segment take longer — spreading the extra volume over a longer bead
  and quietly cancelling the width it was meant to produce.

---

## 3 — Quick Print reads it, splits it, applies it

- `_obj_dict_extrusion_modifiers` reads the baked `extrusion_profile` and
  **refuses** a length that does not match the object's own trajectory.
- `_subpaths_from_array(arr, modifiers=None)` slices the profile in **lockstep**
  with the existing travel-move split, publishing it on
  `_last_subpath_modifiers`. Kept as a side channel rather than a changed return
  type because `_path_segments_for_selection` has a dozen callers that want
  plain XY.
- `_segments_and_modifiers_for_selection()` / `_vol_per_mm_profiles()` /
  `_extrusion_profile_summary()`.
- **Recomputed against the needle fitted NOW** (`bore area × modifier × trim`)
  rather than replaying the sketch's stored µL/mm — so the print lays the
  DECLARED WIDTHS with the current needle instead of reproducing a volume that
  was only right for whatever needle was fitted at bake time.
- Quick Print's own **Extrusion ×** becomes a **trim** on top. At ×1 you get
  exactly the sketch. This is stated in the readiness card, because an operator
  whose × "looks ignored" needs to see what actually governs the bead.
- **The flow ceiling is sized by the THICKEST segment.** `_flow_modifier_peak()`
  (the trim alone when there is no profile, so an ordinary print is unchanged)
  feeds `_flow_limited_xy_max_mm_s` and the limit warnings, so the resolved
  speed keeps the widest segment inside the needle's max safe flow. A mean would
  let the peak over-pressure the needle — a pulled glass tip shatters. The peak
  is cached and refreshed in `_refresh_setup_status`, from the SAME geometry
  pass that produces the segments, so neither re-reads the print file.
- The **multi-ink** path gets the same fidelity: `_group_segments` already
  recompiles each sub-sketch, so its profile is in hand there.

`PrintReadiness` gains `extrusion_span` / `extrusion_trim` and an **Extrusion**
check, so this is reported through the one evaluation that already drives the
card, the status line, the button state and the tooltip.

---

## 4 — The job and the executors

`build_well_plate_job(path_extrusion_profiles=...)` — parallel to
`path_segments`, in **µL/mm**. Validated at BUILD time so a mismatch is
reported once rather than silently per print. Omitted → `PRINT_PATH` params are
byte-identical to before (pinned by test).

All **four** deposition sites honour it, via one `_extrusion_profile_for(cmd)`:

| executor | lookup |
|---|---|
| discrete point stream | `at_index(i-1)` — its own segment indices |
| open-loop velocity | `at_arclen(s_prev)` |
| closed-loop velocity follower | `at_arclen(s - ds)` |
| feed-plan section runner | `at_arclen(s_global - ds)` — the plan splits into sections but keeps a GLOBAL arc length, so deposition follows the shape across every section boundary |

Every residual flush and the feed plan's exactness top-up are profile-aware too,
so a path that deposits only through its profile cannot drop its tail volume.

---

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/ExtrusionProfile.py` | **NEW** — the profile, its refusal, both lookups, the two conversions |
| `SupportClasses/SketchTrajectory.py` | `Sketch.width_drives_extrusion` (+ serialization, legacy-off); base × `_shape_modifier`; `move_to` stamps every segment; `CompiledSketch.extrusion_profile` / `vol_per_mm_profile` / `extrusion_ref_width_mm`; summed volume |
| `SupportClasses/PrintManager.py` | `_extrusion_profile_for`; the four deposition sites + residual flushes; `build_well_plate_job(path_extrusion_profiles=)` with build-time validation |
| `SupportClasses/PrintReadiness.py` | `extrusion_span` / `extrusion_trim` + the Extrusion check; the bead check says "up to" when the width varies |
| `gui/pages/print_builder_sketch.py` | bake the profile + reference + planned total; seed new shapes at the 1× reference; the width row reports the resolved modifier; the **Line widths set extrusion** switch |
| `gui/pages/workflows/quick_print_workflow.py` | read / refuse / split / convert the profile; pass it to both job builders (single + multi-ink); `_flow_modifier_peak` for the ceiling; the status report |
| `tests/test_v7215_sketch_extrusion_to_quick_print.py` | **NEW** — 36 tests |
| `tests/test_v7214_…`, `test_v75x_sketch_trajectory`, `test_v75x_sketch_extrusion_thickness`, `test_v75x_extrusion_volume_calc` | the width→volume contract is now the point; fixtures declare the width they mean |

---

## Implementation Steps

- [x] Per-shape modifier + per-segment profile in the compiler; summed volume
- [x] `width_drives_extrusion` with the legacy-OFF migration + the 1×-reference seed
- [x] `ExtrusionProfile` (refusal, index + arc-length lookup, µL/mm, rate)
- [x] Bake the profile, its reference width and the planned total
- [x] Quick Print: read / refuse / split / convert / report; ceiling from the peak
- [x] `path_extrusion_profiles` → `PRINT_PATH.vol_per_mm_profile`
- [x] All four executor deposition sites + residual flushes
- [x] Readiness reporting as a first-class check
- [x] New suite (36) + mutation matrix (15/15)
- [x] Regression + `gui.app` import smoke
- [ ] **HW verification on ME3B V1** (checklist below)

---

## Testing Notes

**New suite** `tests/test_v7215_sketch_extrusion_to_quick_print.py` — **36
tests**, offscreen, through the production compiler, the real `SketchPage` bake,
a real `QuickPrintWorkflowPage`, the real job builder and the **real discrete
executor**:

- the helper: index/arc-length agreement, boundary semantics, end clamping,
  totals, the **refusal matrix** (short, long, empty, None, NaN, negative), and
  that the rate tracks the volume;
- the compiler: one entry per segment, each shape at its own declared width,
  travel/lift/no-extrude at 0.0, `vol_per_mm == modifier × bore area`, the total
  summed (30× area for 10 mm at 1× + 10 mm at 2×, **not** 20×), the global
  multiplier still trimming, **the plunger column agreeing with the profile**,
  and the geometry untouched by an 8× width change;
- the migration: a dict without the key loads OFF and deposits the old uniform
  amount; a new sketch is ON and round-trips;
- the bake: params carry the profile, the reference width and the planned total;
- Quick Print: the modifiers survive the sub-path split with the right lengths,
  become µL/mm against the current needle, the operator's × trims them, **the
  flow ceiling halves when the thickest segment is 2×** (asserted on the
  resolved speed, not on the accessor), and a print with no profile leaves the
  ceiling exactly as before;
- the job: per-`PRINT_PATH` profiles, a wrong-length one dropped, and
  **no profiles at all is param-for-param identical to before**;
- the executor: a thick half deposits twice the thin half, a zero stretch
  deposits nothing, the rate tracks the volume, and both "no profile" and
  "malformed profile" fall back to the scalar;
- one end-to-end test that bakes a two-width sketch and asserts the job Quick
  Print would run carries two distinct µL/mm values in a 2:1 ratio summing to
  the sketch's own planned volume.

**Mutation matrix — 15/15 CAUGHT**, sources restored byte-identically: width
ignored · legacy sketches opted in silently · profile stamped with a constant ·
`no_print` still extrudes · volume back to length × one scalar · profile not
baked · Quick Print ignores it · the split does not slice it · the job builder
drops it · the discrete executor ignores it · a malformed profile ACCEPTED · the
rate no longer tracks the volume · the arc-length lookup off by one · the
ceiling sized by the trim instead of the peak · µL/mm without the bore area.

⚠ **THE HARNESS ITSELF WAS WRONG FIRST, AND IT MANUFACTURED TWO FALSE
"CAUGHT"s.** A 2-minute tool timeout killed the first matrix run *while a
mutation was applied*, leaving it in the tree; every mutation after it then ran
against an already-red suite and was scored CAUGHT. The stranded mutation was
restored and the harness now (a) **asserts the baseline is GREEN before it
starts** and (b) restores in a `finally` so a kill cannot strand one. Re-run
honestly it reported **13/15**, exposing two genuinely weak spots:

- **M4 survived** because `no_print` was checked in *two* places and `move_to`'s
  `printing` flag already did the work — the redundant check was deleted and the
  mutation re-aimed at the real authority;
- **M14 survived** because the ceiling test asserted on `_flow_modifier_peak()`
  (the model) instead of on `_flow_limited_xy_max_mm_s()` (the behaviour) — the
  recurring weakness this project keeps recording. Rewritten to pin a binding
  ceiling and assert the resolved speed halves.

**Regression — green:** every sketch suite together **405** · executors +
extrusion model + readiness + log reader **242** (see the pre-existing failure
below) · Quick Print consumers and hosts (pick-and-place, multi-ink, the v7.19
plate queue, the v7.20 print calibrator, v7.21 section promotion, v7.7 zones,
print library) **421** · the two new suites + suite hygiene **82** · `import
gui.app`.

**One pre-existing failure, PROVED not ours:**
`test_v77_print_log_reader::TestRealHardwareLog::test_it_parses_and_reconstructs_the_section_offsets`.
It globs `logs/prints/*.jsonl` and takes the lexicographically last file, which
is now one of the operator's own multi-stroke prints (`send_it`, 2026-08-19
17:36). That log holds **6 separate `PRINT_PATH`s**, each restarting
`plan_section.index` at 0 with `base_s_mm = 0.0`, so `section_bases` — which
keys by `index` alone — collides them and the running-sum assertion cannot hold.
`SupportClasses/PrintLogReader.py` is **not in this diff** (empty diffstat), and
the failure reproduces with the suite run alone. It is a real (pre-existing)
limitation of the reader for multi-stroke runs, worth its own fix.

---

## Issues & Decisions

- **µL/mm is what ships to the machine; the modifier is what ships for the
  operator.** Both are baked. The consumer recomputes µL/mm from the modifier
  against the needle fitted now (declared widths print), and the stored µL/mm is
  the cross-check that says whether the needle changed since the bake.
- **A malformed profile is refused at three levels** (`ExtrusionProfile.build`,
  Quick Print's reader, the job builder) — each refusal falls back to the scalar
  flow, i.e. the previous behaviour, and logs once.
- **The peak governs the ceiling, the profile governs the bead.** Two different
  questions, so `_extrusion_modifier()` (the trim) was left alone and
  `_flow_modifier_peak()` added, rather than overloading one accessor.
- **The pump column was NOT used as the transport.** It is cumulative plunger mm
  against the syringe present at bake time, so re-deriving a modifier from it
  would mis-scale silently if the syringe changed. The explicit modifier profile
  is syringe-independent.
- ⚠ **Turning `width_drives_extrusion` on for an old sketch changes how much it
  deposits** (od/id ≈ 1.7× if its shapes were seeded from the outer Ø). Off by
  default for anything loaded from disk; the checkbox tooltip says so; no
  automatic back-fill, because the multiplier that would reproduce the old
  deposition is per-shape while the multiplier is sketch-wide.
- **Not changed:** the fill pitch (`line_spacing_mm`) still comes from the
  orifice outer Ø; fills still raster at that pitch; the shaded band still shows
  the predicted width, never the declared one.

---

## Needs HW verification on ME3B V1, IN ORDER

1. **Go/no-go, nothing changed:** print an ordinary (non-sketch) object from
   Quick Print — the readiness card must show **no** Extrusion row and the same
   flow/speed as before.
2. Open an **existing** saved sketch: **Line widths set extrusion** must be
   **off** and the reported volume unchanged from before the upgrade.
3. New sketch, two lines at 1× and 2× the bead width, welded end to end. Bake it
   (**Send to Print Setup**), select it in Quick Print: the readiness card reads
   *"Extrusion — from the print itself, ×1.00–2.00 along the path"*.
4. **Print it and look at the bead**: the second half must be visibly wider than
   the first, with **no pen-up** between them, and the reported dispensed volume
   ≈ 1.5× what a uniform print of the same length would use.
5. Set Quick Print's **Extrusion ×** to 0.5 and confirm the card reads
   ×0.50–1.00 and the whole bead thins — the trim works, the ratio holds.
6. Mark one shape **No extrude** and confirm that stretch deposits nothing while
   the needle still traverses it at print height.
7. Declare a very wide line (e.g. 6×) and confirm the **top speed is
   auto-limited** with the warning naming the needle's flow ceiling — the peak,
   not the average, is what binds.
8. Repeat 3–4 through a **multi-ink** sketch and confirm each ink group carries
   its own widths.
9. Check `logs/prints/<run>.jsonl` for the `extrusion_profile` event (entry
   count, min/max µL/mm, planned µL) and confirm the planned total matches the
   Sketch page's reported volume.
