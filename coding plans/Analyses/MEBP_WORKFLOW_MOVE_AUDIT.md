# MEBP Workflow Move Audit — syringe · Z · XY, end-to-end

**Purpose.** A single, code-grounded reference for *every motion* each workflow makes —
what the pump does (signed µL + why), coupled with the Z and XY moves around it — so we can
(1) **close the loop** (verify each workflow's net pump balance and end-state), (2) **find
issues** (mismatched assumptions, slow spots, latent traps), and (3) **teach** how the machine
actually behaves.

Every formula, default, and sequence below was extracted from the working tree and is cited
`file:line`. Where a number is machine-specific it uses **ME3B V3**'s live config
(`config/hardware/devices/ME3B V3.json`, `settings.json`).

> **How to read the step tables.** Each row is one primitive action.
> `Δ µL` is signed: **`+` = DISPENSE (push out, plunger → empty)**, **`−` = ASPIRATE (draw in,
> plunger → full)** — the canonical convention from `StageController.move_pump_uL` (`StageController.py:4245-4256`).
> "Travel" = a cross-position move that **retracts Z first** (`safe_travel_to`); "dip/lower" =
> descend to a working Z **after** arriving.

---

## Part 0 — Foundations (read this once)

### 0.1 The pump: µL → plunger mm, and the direction sign

`move_pump_uL(pump, volume_uL, rate_uL_s, *, settle, relieve)` is the **single chokepoint** for
all volumetric pump motion (`StageController.py:4238`). Two conversions matter:

| Quantity | Formula | Source |
|---|---|---|
| Plunger travel | `distance_mm = volume_uL × (stroke_length_mm / syringe.volume_uL)` | `PhysicalModels.py:314-316`, `StageController.py:4292` |
| Flow → feedrate | `feedrate_mm_min = rate_uL_s × (stroke_length_mm / syringe.volume_uL) × 60`, then flow-clamped | `HardwareConfig.py:284-289`, `StageController.py:4294-4302` |
| Direction sign | calibrated: `pump_dir_sign = −aspirate_sign`; uncalibrated: legacy `_flip_sign` | `StageController.py:1912-1920`, applied in `move_pump_relative` `:3993` |

- **The barrel inner diameter (`barrel_id_mm`) is *not* used for plunger travel** — only the
  syringe `stroke_length_mm` / `volume_uL` ratio is. Bore only feeds flow-physics elsewhere.
- **Direction is owned by the plunger calibration** ("Set Dispensed / Set Aspirated", the pump
  twin of the Z `z_up_sign` setup). Until a pump is calibrated it falls back to the legacy
  `axis_flip` sign.

**ME3B V3:** P2 = 250 µL Hamilton 1725, 30 mm stroke ⇒ **0.12 mm/µL** (1 µL = 0.12 mm plunger).
P2 is calibrated (raw 40→70 mm, `aspirate_sign=−1` ⇒ `pump_dir_sign=+1`). **P1 and P3 are
*not* plunger-calibrated** — they use the legacy sign + a syringe-stroke soft-limit estimate.

### 0.2 The needle: "1 needle" volume vs. printed bead — two different diameters

There are **two** needle-volume models, and they use **different diameters**:

| Model | Formula | Diameter | Used by |
|---|---|---|---|
| **"1 needle" / dead volume** | `internal_volume_uL = π·(id_mm/2)² × length_mm` | **inner** bore | prep/clean (oil/buffer/waste multiples), Quick Print reserve | (`PhysicalModels.py:262-272`) |
| **Printed bead (auto-flow)** | `vol_per_mm = π·(id_mm/2)² × modifier` | **inner** bore | Quick Print `flow@100% = bore_area × xy_max × modifier` | (`quick_print_workflow.py:1252`) |
| **Printed bead (physics model)** | `vol_per_mm = od_mm × layer_height_mm` | **outer** Ø | `GeometryEngine`/`FlowPhysics` *recommended* flow only | (`FlowPhysics.py:328-353`) |

> ⚠️ **Finding F-1.** Quick Print's actual deposit (inner-bore area) and the
> `GeometryEngine.extrusion_flow_rate` model (outer-Ø × layer-height) are **different bead
> models**. A print object designed assuming the physics model, but run through Quick Print's
> auto-flow, deposits a different volume/mm. See Part 9.

**ME3B V3 (22 G, id 413 µm, od 718 µm, length 2.0″ = 50.8 mm):**
- `cross_section_area_mm² = π·(0.2065)² = 0.13396 mm²`
- **1 needle = 0.13396 × 50.8 = 6.81 µL** (= 0.817 mm of P2 plunger).
- 4 needles = **27.22 µL** — exactly the `move_pump_uL(P2, −27.222 µL)` seen in field logs. ✔

### 0.3 Z: the height frame, polarity, and the travel primitives

- Z is reasoned about in a **height frame**: `height = z_up_sign · (raw − zero_Z)` so "bigger =
  higher" regardless of motor polarity (`z_height_of`, `StageController.py:3606`). Jog inputs go
  through `move_z_user_relative` (height-frame delta, `:3575`).
- **ME3B V3:** `z_up_sign = +1`, `steps_per_mm.Z = −5255` (motor inverted; the height frame
  hides it). Print Z direction is *re-derived* from the plate top↔bottom vector
  (`print_z_dir()`, `:2214`) so layer build-up and hops always go physically up.
- Working heights are expressed as **"mm above the calibrated plate bottom"** →
  `print_height_to_zref(h) = plate_bottom + print_z_dir()·h` (`:2231`).

**Three motion primitives do all the safe travel:**

| Primitive | What it does | Key guarantee |
|---|---|---|
| `safe_travel_to(x_um, y_um, safe_z, target_z)` | raise→M400+confirm→XY@50mm/s→confirm→lower | **Aborts the XY move if Z isn't confirmed at safe height**; refuses if ZP dropped with a needle present (`:3821-3959`) |
| `ensure_retracted_to(safe_z)` | raise-only; no-op if already at/above | **Never descends** (a too-low travel-Z degrades to no-op) (`:3698-3751`) |
| `_retract_z_slow_then_fast(...)` | first ~1 mm slow, then fast, then confirm | Gentle lift-out so the bead doesn't peel; only on a genuine lift (`:3655`) |

Slow-retract defaults: **1.0 mm @ 60 mm/min**, then fast (`StageController.py:1445-1446`).
Travel/insert feedrates: retract = `z_max`, insert = `max(0.6·z_max, 100)` mm/min (`:2437-2439`).

### 0.4 Settle / prime / relief (global pump timing)

| Param | Meaning | Default | ME3B V3 | Source |
|---|---|---|---|---|
| `pump_settle_time_s` | dwell **before *and* after** every *discrete* pump move | 0.0 | **10.0** | `HardwareConfig.py:375` |
| `pump_prime_time_s` | pre-flow lead-in; prime µL = flow × this | 0.25 | **1.0** | `HardwareConfig.py:379` |
| `pump_relief_volume_uL` | post-aspirate dispense-back to bleed vacuum | 0.0 | **0 (off)** | `HardwareConfig.py:386` |

- **Settle** is applied only when a caller passes `settle=True` — i.e. *discrete* actuations
  (prep, pickup, prime, push/pull, deposit). The **streamed print path does *not* settle**
  (it would dwell → blobs), and **manual jog does not settle** (`StageController.py:4311-4338`).
  With `settle=True`, the move also **blocks for completion** (M400 drain via `flush_moves`,
  timeout `clamp(move_s+5, 5, 180)`; else open-loop sleep of the full estimate `move_s =
  |vol|/rate + 0.1`).
- **Relief** fires only on `relieve=True` **and** an aspirate (`volume<0`) **and** relief>0,
  recursing once with `+relief` (`:4340-4353`). Used only at reagent-load aspirates (prep
  oil/buffer, ink pickup). **Disabled on ME3B V3** (relief=0).

> ⚠️ **Finding F-2.** `pump_settle_time_s = 10 s` on ME3B V3 means **every** discrete pump move
> carries **~20 s** of dwell (10 before + 10 after) plus the move itself. A full Quick Print run
> has ~6–7 discrete actuations → **~2–2.5 min of pure settle**. This is almost certainly the
> dominant "why is everything so slow" cost. See Part 9.

### 0.5 XY: well-centre resolution

`resolve_well_xy_mm(name, plate, settings)` (`PrintTrajectoryPlanner.py:873`):
1. **Calibrated first** — `settings.well_positions_mm[name]` (taught/warped, already zero-ref mm)
   used **verbatim, never re-signed**.
2. **Geometric fallback** — plate-local `get_well_position()` × `plate_axis_sign`
   (`(−1,−1)` on ME3B, plate mounted 180°). Malformed sign degrades to `(1,1)`.

All workflows prefer calibrated positions; the sign is applied **only** to the geometric fallback
(double-signing a calibrated position would crash to a mirrored well).

---

## Part 1 — Shared needle prep (`run_prep`)

Runs **once** before the operation loop when `do_prep` is on (`PickAndPlaceManager.py:943`).
`unit = needle_volume_uL` (0.2); `bore = prep_bore`; `rate = prep_rate_uL_s` (default 1.0 µL/s);
each well reached by a **full safe-Z travel** then dip to `service_z_mm`.

| # | Well | Move | Δ µL (formula) | Rate | Z | Why |
|---|---|---|---|---|---|---|
| 1 | waste | travel + dip | — | 50 mm/s XY | service Z | go to waste |
| 1 | waste | **DISPENSE** | `+oil_needles·unit` | 1.0 µL/s | service Z | expel stale oil |
| 2 | oil | travel + dip | — | — | service Z | go to oil |
| 2 | oil | **ASPIRATE** (`relieve`) | `−oil_needles·unit` | 1.0 µL/s | service Z | load fresh oil; relief bleeds vacuum |
| 3 | wash | travel + dip | — | — | service Z | go to wash |
| 3 | wash | **WASH** (`_do_wash`) | — | — | dip (jiggle) | scrub tip exterior |
| 4 | buffer | travel + dip | — | — | service Z | go to buffer |
| 4 | buffer | **ASPIRATE** (`relieve`) | `−buffer_needles·unit` | 1.0 µL/s | service Z | load buffer |

**`_do_wash`** (`:1225`) — *intra-well agitation, no retract between jiggles*. Per cycle
(`wash_cycles`, default 3): lift `+wash_z_amplitude_mm` (default 0.5) → dwell → **return to the
exact dip Z via `move_z_absolute`** (so a clamped lift can't walk the tip down) → dwell →
random XY nudge within **±`wash_xy_amplitude_um`** (default 200 µm, *per-axis square*) → dwell;
then recentre + wait.

**Defaults** (`PickAndPlaceManager.py:507-518`): `oil_needles=1.0`, **`buffer_needles=1.0`**,
`prep_rate_uL_s=1.0`, `wash_cycles=3`, `wash_z_amplitude_mm=0.5`, `wash_xy_amplitude_um=200`,
`wash_dwell_s=0.3`. `needle_volume_uL` defaults **0.0**.

> ⚠️ **Finding F-3.** Two prep defaults are traps:
> (a) **`needle_volume_uL` defaults 0.0** → if the GUI doesn't stamp a real needle volume, the
> `unit>0` guard makes **all prep pump moves silent no-ops** (only the wash motion runs).
> (b) **`buffer_needles` defaults 1.0** here and in `CommonPrintSettings.PROMOTED_DEFAULTS`, yet
> the prep *design* (and field logs: 27.22 µL = 4 needles) assume **4**. Whether 1 or 4 buffer
> needles load depends on which page set the value. See Part 9.

**Loop closure of prep alone:** oil `+u` then `−u` cancels; buffer `−buffer_needles·u` is
**left in the needle** (intentional — the needle ends conditioned + buffer-loaded). Prep is *not*
self-balanced, by design.

---

## Part 2 — Shared post-clean (`run_post_clean`)

Runs **once** after the loop when `do_post_clean` is on and not aborted (`:1007`). Order
**waste → wash → buffer** (no oil step):

| # | Well | Move | Δ µL | Why |
|---|---|---|---|---|
| 1 | waste | travel + **DISPENSE** | `+post_dispense_needles·unit` (default 1.0) | empty residual reagent/cells |
| 2 | wash | travel + **WASH** | — | scrub |
| 3 | buffer | travel + **ASPIRATE** (`relieve`) | `−buffer_needles·unit` | reload buffer, leave conditioned |

---

## Part 3 — Quick Print (the full bioprint loop) ★

The headline workflow: **needle prep → ink pickup → print → needle reset**, hands-free after one
confirm dialog. All refs `gui/pages/workflows/quick_print_workflow.py` unless noted.

### 3.1 The master "Print speed: N % of max" lever

One percentage scales **both** the XY traverse and the pump flow, so deposited volume-per-mm is
**invariant to %** (`_resolved_print_kinematics`, `:1268`):

```
pct   = print_speed_% / 100                 (default 25%, clamped 1–100)
speed = pct × xy_max_mm_s                    ← XY traverse (mm/s)
flow  = pct × flow@100%                      ← pump (µL/s)
prime = flow × preflow_s                     ← pre-flow lead-in volume (µL)

flow@100% = bore_area_mm² × xy_max_mm_s × extrusion_modifier      (:1252)
  ⇒  volume_per_mm = flow/speed = bore_area × modifier            (independent of %)
```

- `xy_max_mm_s` = measured top speed → else `safety_limits.max_xy_speed` → else 10 mm/s (`:1172`).
- `preflow_s` seeded from `pump_prime_time_s` (ME3B V3 = 1.0 s), else 0.25 s (`:601`, `:590`).

**ME3B V3 (22 G, modifier 1.0, `max_xy_speed` 10 mm/s):**
`flow@100% = 0.13396 × 10 × 1.0 = 1.34 µL/s`; at the **25 %** default → speed 2.5 mm/s, flow
0.335 µL/s, **volume_per_mm = 0.134 µL/mm** (a bead ≈ the 0.41 mm bore width).

### 3.2 Ink pickup volume (`_compute_pickup_volume_uL`, `:1285`)

```
pickup_uL = flow·(path_len/speed) + flow·preflow_s + needle_dead_volume_uL + ink_padding_uL
          = printed_volume        + prime          + bore_internal_volume  + padding
```

- **Not** a "× safety factor." The reserve is the **needle bore internal volume** (1 needle, the
  oil/buffer plug behind the ink) so the needle never prints buffer once the ink runs out
  (`_needle_dead_volume_uL`, `:1316`). `ink_padding_uL` default 0 (`:468`).

**ME3B V3 worked example — 100 mm path @ 25 %:**
`printed = 0.335 × (100/2.5) = 13.4 µL`; `prime = 0.335 × 1.0 = 0.34 µL`; `reserve = 6.81 µL`
⇒ **pickup ≈ 20.5 µL** (≈ 2.46 mm of P2 plunger). Net ink left after print = pickup − printed −
prime ≈ 6.8 µL (the reserve plug), cleared by the reset (§3.5).

### 3.3 Ordered run

**Phase 0 — gate + build (GUI thread, no motion):** require XY+ZP connected, well/plate/object
selected; resolve calibrated well centre; build per-object `path_segments`; gate prep (Safe Z,
ink well calibrated, service wells) and reset (waste/wash/oil wells); **capture `oil_baseline =
get_pump_position_uL(pump)`** (`:1895`); run the **syringe-budget pre-flight**
(`_check_syringe_budget`, `:1343`) that simulates the whole signed sequence against the plunger
envelope and offers an oil remedy if infeasible; show the **single "Confirm print setup" dialog**.

**Phase 1 — preflight worker (off GUI thread):**
1. (optional) starting-oil remedy.
2. (optional) **`run_prep()`** — Part 1.
3. (optional) **`aspirate_ink(ink_pos, pickup_uL, bore=pump, z=ink_dip_z)`** (`:1277`): safe-Z
   travel to the ink well, dip to `ink_dip_z` (default 0.5 mm above bottom), **ASPIRATE
   `−pickup_uL`** at `prep_rate_uL_s`, `relieve=True`.
4. **`_preposition_for_print`** (`:1719`): `safe_travel_to(x,y, safe_z, target_z=None)` — parks the
   needle **at safe Z** over the print start (raise→confirm→XY→confirm, **never lowers**).
5. `finally`: `_retract_to_safe_z()` (raise-only).

**Phase 2 — hands-free launch (`_on_prepositioned`, `:2084`):** refuses if the preamble
aborted/errored, ZP dropped, or positioning didn't confirm. Else builds the discrete job
(`return_home=False` → ends where it finished, no drive to 0,0) and starts a `PrintManager`.

**Phase 3 — the print (discrete plan):** per object — `TRAVEL_UP → MOVE_XY (retract-confirm) →
MOVE_Z (confirmed descent) → DISPENSE prime → PRINT_PATH → …`; later objects use a **1 mm hop**
instead of a full retract so the needle never drags through printed material. Ends with a final
`TRAVEL_UP`. (Internals in Part 7.)

**Phase 4 — post-print reset (`run_print_cleanup`, on `COMPLETED` only, `:2238`):**

| # | Well | Move | Δ µL (formula) | Why |
|---|---|---|---|---|
| 1 | waste | travel + **DISPENSE** | `+(leftover + oil_margin)`, `leftover = max(0, pump_dir_sign·(baseline − current_live))`; capped 20·unit else fixed 6·unit | expel unprinted ink+buffer + margin |
| 2 | wash | travel + **WASH** | — | scrub tip |
| 3 | oil | travel + **OIL reset** | `pump_dir_sign·(baseline − current_live)` (signed; ASPIRATE oil) | drive plunger back to the **pre-run oil baseline** |

The live position is read **after** the waste dispense, so the return-to-baseline is exact
(`:1109`). `oil_margin = margin_needles·unit` (default 1 needle, `:535`).

### 3.4 Loop closure (Quick Print)

`pickup(−)` then `prime(+)` + `printed(+)` leave the needle with ≈ the reserve plug; the reset
wastes `leftover + margin` and re-aspirates oil to return the plunger to `oil_baseline`. **Net
result: the syringe ends in its initial pure-oil state.** ✔ (closed by the reset, not by the
print itself).

---

## Part 4 — Spheroid Pick & Place

Per picked→placed pair (`_execute_spheroid_pickup`, `PickAndPlaceManager.py:697`). Optional
`run_prep` before / `run_post_clean` after the loop.

**Volume:** `V = (4/3)·π·(d/2)³ × safety_factor / 1e9 µL` (`:132`). Default d=200 µm, safety 1.5 ⇒
**V ≈ 0.0063 µL (6.3 nL)** — a tiny aspirate sized to the spheroid.

| # | Move | Δ µL | Rate | Z | Why |
|---|---|---|---|---|---|
| 1 | travel → source | — | 50 mm/s | pick_z (0.10 mm ↑ bottom) | over source |
| 2 | **ASPIRATE** | `−V` | `pickup_speed_uL_s` (1.0) | pick_z | draw spheroid in |
| 2b | dwell | — | — | — | `pick_dwell_s` (default 0) settle |
| 3 | travel → dest | — | 50 mm/s | place_z (0.50 mm ↑ bottom) | over dest |
| 4 | **DISPENSE** | `+V` | `release_speed_uL_s` (1.0) | place_z | release spheroid |
| 4b | dwell | — | — | — | `place_dwell_s` (default 0) |

**Loop closure:** `−V` then `+V`, **same bore**, `relieve=False` → **net 0** (volume-balanced; a
balanced capture must keep exactly what it drew). ✔

---

## Part 5 — Cell Targeting & Removal

Trypsinize-in-place then extract. Per region (`_execute_cell_removal`, `:756`); optional
prep/clean bracket the loop.

**Volumes:** `push (P) = needle_area × release_depth_mm` (default 0.10 mm → ME3B V3 ≈ **0.0134 µL**);
`pull (L) = P × extract_multiplier` (default 2×) (`:214-230`).

| # | Move | Δ µL | Rate | Z | Why |
|---|---|---|---|---|---|
| 1 | travel → reagent well | — | 50 mm/s | reagent dip (0.50) | go to reagent |
| 1 | **ASPIRATE** load | `−P` | push_speed (slow 0.5) | dip | load reagent column |
| 2 | travel → removal (x,y) | — | 50 mm/s | removal_z (0.10) | over cells |
| 3 | **DISPENSE** slow push | `+P` | push_speed (0.5) | removal_z | push reagent into cells |
| 4 | dwell (incubate) | — | — | removal_z | `dwell_time_s` (default 60 s) |
| 5 | **ASPIRATE** fast pull | `−L` | pull_speed (fast 5.0) | removal_z | pull up reagent + cells |
| 6 | travel → placement | — | 50 mm/s | place_z (0.50) | over destination |
| 7 | **DISPENSE** gentle | `+L` | push_speed (slow 0.5) | place_z | deposit extracted cells |

**Loop closure:** `−P +P −L +L = 0` ✔ — *provided a placement is paired* (the GUI enforces
pairing before Start). With no placement (not reachable via this GUI) the op ends `−L` in the
needle.

---

## Part 6 — Cell Labeling / staining

Deposit stain → incubate → recover → dump to waste. **No placement.** Per region
(`_execute_cell_labeling`, `:852`).

**Volumes:** `deposit (D) = needle_area × deposit_depth_mm` (default 0.10); `aspirate (A) =
D × aspirate_multiplier` (default 2×). Both deposit and recovery are **slow** by request
(`:288-304`).

| # | Move | Δ µL | Rate | Z | Why |
|---|---|---|---|---|---|
| 1 | travel → reagent (stain) | — | 50 mm/s | reagent dip (0.50) | load stain |
| 1 | **ASPIRATE** load | `−D` | aspirate_speed (slow 0.5) | dip | draw deposit volume |
| 2 | travel → region | — | 50 mm/s | label_z (0.10) | over region |
| 3 | **DISPENSE** slow deposit | `+D` | deposit_speed (slow 0.5) | label_z | lay down stain |
| 4 | dwell (incubate — **headline**) | — | — | label_z | `stain_dwell_time_s` (default **300 s**) |
| 5 | **ASPIRATE** slow recover | `−A` | aspirate_speed (slow 0.5) | label_z | pull stain + excess up |
| 6 | travel → waste | — | 50 mm/s | service Z (0.50) | go to waste |
| 6 | **DISPENSE** to waste | `+A` | deposit_speed (slow 0.5) | service Z | dump recovered stain |

**Loop closure:** `−D +D −A +A = 0` ✔. The waste dump is **always required** (waste well +
service Z set unconditionally, even with prep/clean off) — so the op always closes.

---

## Part 7 — Discrete print execution internals

`PrintManager` runs the `build_well_plate_job` command plan (`PrintManager.py:548`).

### 7.1 The command plan (per object)

```
seg 0 :  TRAVEL_UP → MOVE_XY(retract-confirm) → MOVE_Z(confirmed) → DISPENSE(prime) → PRINT_PATH → [DISPENSE(retract)]
seg ≥1 :  MOVE_XY{hop_z = z + z_up·intra_well_hop_z_mm}(1 mm hop) → MOVE_Z → DISPENSE(prime) → PRINT_PATH → …
end   :  TRAVEL_UP  [+ HOME_XY→(0,0) only if return_home=True]
```

- **Layer Z:** `z = print_z_height + z_up_sign·layer·layer_height` (polarity-safe build-up,
  `:599-600`).
- **Prime** = `+prime_uL` DISPENSE at `pump_rates_uL_s[pump]`, just before the path
  (`settle=True`).

### 7.2 ★ Per-segment ink — the core relation

`_execute_print_path` (`:3039`). For each segment of length `seg_len` (mm):

```
seg_time   = seg_len / print_speed_mm_s
volume_uL  = flow_rate_uL_s × seg_time = flow_rate_uL_s × seg_len / print_speed_mm_s
⇒ volume_per_mm = flow_rate_uL_s / print_speed_mm_s          (segment-count-invariant)
```

dispensed via `move_pump_uL(pump, volume_uL, flow_rate_uL_s)` **without `settle`** (open-loop,
continuous). Pacing per segment = `max(xy_move_time, pump_move_time, 0.05 s)` (`:3169`). A
**M400 barrier every 8 segments** bounds the Marlin planner buffer (`:3202`), and an
**end-of-path XY drain** (`_wait_for_xy_settle`, tol 50 µm) makes the Z retract start in sync
(`:3222`). The poller + watchdog are suspended for the whole path.

### 7.3 Safety rails (all paths)

- **MOVE_Z** is a **confirmed descent** (M400 + `wait_for_z_arrival`, poller suspended); **aborts
  the print** if Z can't be confirmed (`:2606-2662`).
- **MOVE_XY / HOME_XY** call `_retract_for_travel` at the top (raise-only, never lowers, `:2449`).
- Every Z move emits an **explicit feedrate** (never a bare `G0 Z` that could inherit the pump's
  slow modal F — the root cause of the historical ZP crawl/disconnect).
- `_execute_loop`'s **`finally` always `_retract_to_safe_z()`** — completion, error, *and* abort
  all end with the needle at safe Z (`:2392-2407`, `:2502-2545`).

---

## Part 8 — Loop-closure ledger

Net pump change per workflow (signed µL; `+`=dispense, `−`=aspirate):

| Workflow | Signed sequence | Net | Closed? |
|---|---|---|---|
| Prep (alone) | `+u −u (oil) … −b·u (buffer)` | `−b·u` (buffer left in needle) | by design (needle ends conditioned) |
| Post-clean | `+p·u (waste) … −b·u (buffer)` | `−b·u` | by design |
| **Spheroid** | `−V +V` | **0** | ✔ inherent |
| **Cell Targeting** | `−P +P −L +L` | **0** | ✔ (needs paired placement; GUI-enforced) |
| **Cell Labeling** | `−D +D −A +A` | **0** | ✔ (waste dump always present) |
| **Quick Print** | `−pickup +prime +printed` … reset `+waste −oil` | **0 vs. oil baseline** | ✔ (closed by reset, not the print) |

---

## Part 9 — Findings & recommendations

> **Operator decisions (2026-06-30).**
> **F-2** — the 10 s dwell was a residual-pressure workaround; now superseded by the post-move
> pressure relief (F-4), so settle can be cut substantially.
> **F-1** — standardize on **inner-bore area × modifier** everywhere (it *is* the deposited line
> thickness); migrate `GeometryEngine`/`FlowPhysics` off `od × layer_height`.
> **F-3** — **1 needle of buffer is correct** (4 is too much); the `1.0` default stays.
> **F-4** — build a configurable relief: default **1 % of syringe volume**, tunable to 0.001 %,
> applied — each independently toggleable — on **deposit**, **pickup**, and **print quick-moves
> (stop-flow before travel)**. F-1 and F-4 are tracked for implementation in an update plan.

| # | Severity | Finding | Recommendation / status |
|---|---|---|---|
| **F-1** | Med · **decided** | **Two bead models.** Quick Print deposits **inner-bore area × modifier** per mm; `GeometryEngine.extrusion_flow_rate` recommends **od × layer_height**. | **✔ DECISION:** use **inner-bore area × modifier everywhere** (it is the deposited line thickness). Migrate `GeometryEngine`/`FlowPhysics` off `od × layer_height`. Tracked for implementation. |
| **F-2** | High · **resolved** | **`pump_settle_time_s = 10 s`** ⇒ ~20 s per discrete pump move ⇒ ~2–2.5 min of dwell per run. | **✔ RESOLVED:** 10 s was a residual-pressure workaround. Superseded by the post-move pressure relief (F-4) → reduce settle substantially once relief is wired. |
| **F-3** | Med · **resolved** | **Prep defaults.** `needle_volume_uL` defaults **0.0** (→ prep pump moves silently no-op); `buffer_needles` defaults **1.0** vs. a design note of 4. | **✔ RESOLVED:** **1 needle of buffer is correct** (4 too much) — `1.0` stays. (`needle_volume_uL=0.0` trap still stands — keep stamping a real value.) |
| **F-4** | Low → **feature** | **Relief disabled** (`pump_relief_volume_uL = 0`). | **✔ FEATURE:** relief = **1 % of syringe volume** default (per-pump-volume-aware), tunable to 0.001 %, applied — each independently toggleable — on **deposit**, **pickup**, and **print quick-moves (stop-flow before travel)**. = `changes_needed.md` item 5. Tracked for implementation. |
| **F-9** | Med · new | **Max pump flow too high for the needle bore** → over-pressure → air ingestion (can't pull material that fast). `changes_needed.md` item 6. | **NEW:** derive a **max flow rate from needle geometry**, show it in settings, **cap max print speed** to obey it, and expose **pickup/deposit flow** as user settings. Tracked for implementation. |
| **F-5** | Low | **µL→mm uses the syringe spec** (`stroke/volume`), while soft-limits + direction use the **plunger calibration**. They agree on P2 (both 30 mm/250 µL) but could drift apart on a mis-entered syringe. | Cross-check the syringe spec against the captured calibration capacity at calibration time; warn on mismatch. |
| **F-6** | Low | **P1/P3 not plunger-calibrated** on ME3B V3 → legacy sign + estimated soft-limits. A print/op assigned to P1/P3 won't have calibration-owned direction. | Run "Set Dispensed / Set Aspirated" for any pump that will be used. |
| **F-7** | Info | **Wash XY jiggle is a square** (independent ±amp per axis), not a radius — diagonal reach = amp·√2 (283 µm at 200 µm). Could touch the wall in a small well. | Cosmetic; clamp to a radius or to the well bore if wash wells are small. |
| **F-8** | Info | **Module `ZDIR = −1`** is stale vs. ME3B V3 `z_up_sign = +1`. Print Z self-corrects via `print_z_dir()`, but any code reading `ZDIR` directly is a latent trap. | Keep routing Z through `z_up_sign`/`print_z_dir`; treat `ZDIR` as fallback-only (already noted in CLAUDE.md). |

---

## Appendix A — ME3B V3 quick numbers

| Quantity | Value |
|---|---|
| Needle | 22 G, id 0.413 mm, od 0.718 mm, 50.8 mm |
| 1 needle (inner bore) | **6.81 µL** = 0.817 mm P2 plunger |
| Active pump | P2, 250 µL / 30 mm ⇒ **0.12 mm/µL**, calibrated, `pump_dir_sign=+1` |
| Bead volume/mm (Quick Print, mod 1.0) | **0.134 µL/mm** |
| Flow @100% | 0.134 × 10 = **1.34 µL/s**; @25% → 0.335 µL/s @ 2.5 mm/s |
| Pickup, 100 mm path @25% | ≈ **20.5 µL** |
| Settle / prime / relief | **10 s / 1.0 s / 0 µL** |
| XY max / fast travel | 10 mm/s / 50 mm/s |
| Z retract / insert feedrate | z_max / 0.6·z_max mm/min; slow-lift 1 mm @ 60 mm/min |

## Appendix B — File:line index

- Pump chokepoint & conversions: `StageController.py:4238-4353` (settle/relief/wait), `:1912-1920` (dir sign), `PhysicalModels.py:314-325` (µL↔mm).
- Needle volume: `PhysicalModels.py:262-272`. Reagent-prep helpers: `gui/pages/workflows/_reagent_prep.py`.
- Travel primitives: `StageController.py:3698-3959` (`ensure_retracted_to`, `_retract_z_slow_then_fast`, `safe_travel_to`).
- Prep/clean/spheroid/cell: `SupportClasses/PickAndPlaceManager.py` (`run_prep:943`, `_do_wash:1225`, `run_post_clean:1007`, `_execute_spheroid_pickup:697`, `_execute_cell_removal:756`, `_execute_cell_labeling:852`, `execute_queue:562`).
- Quick Print: `gui/pages/workflows/quick_print_workflow.py` (`_resolved_print_kinematics:1268`, `_compute_pickup_volume_uL:1285`, `aspirate_ink` via executor `:1277`, `run_print_cleanup:1098`, `_on_print:1746`).
- Discrete print: `SupportClasses/PrintManager.py` (`build_well_plate_job:548`, `_execute_print_path:3039` [per-seg vol `:3119-3124`], MOVE_Z `:2606`, `_retract_to_safe_z:2502`); resolver `PrintTrajectoryPlanner.py:873`.
- Globals: `SupportClasses/HardwareConfig.py:375-386`; `SupportClasses/CommonPrintSettings.py`.

---

*Generated from a 6-agent code audit of the working tree (Version-7.5.1). Numbers are
ME3B V3's live config. Update this doc when motion code, defaults, or the machine config change.*
