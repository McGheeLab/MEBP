# MEBP v7.5.x — Sketch: closed-loop over-closure + print-sequence panel + single/multi-needle mode

## Objective

Three operator-requested Print Builder → **Sketch** additions:

> "Shapes with closed loops should get an 'overlap toggle' that extends the end
> point past the closure point by the radius of the needle — as the needle moves
> through it pushes ink out of the way and we don't get full closure of the
> shape. Also add a [full] section to the right context panel that shows the full
> list of objects and moves; if a series of objects are connected they are
> grouped into color-coded sections, and the only thing that breaks up these
> sections is a needle quick move or an ink replacement to grab a different ink.
> Currently the print assumes multiple pumps and needles — add an option to
> toggle this."

**(A) Over-closure toggle** — per-shape (closed-loop outlines only)
`SketchShape.overlap_closure`. The compiler continues the printed path PAST the
seam by ~the needle **outer radius** so the loop fully closes.

**(B) Print-sequence panel** — a persistent right-panel card listing the
color-coded continuous **sections** (maximal runs printed as one bead) and the
**moves** that separate them (retract points / quick moves / ink replacements).
Clicking a section selects its shapes.

**(C) Single- vs multi-needle mode** — `Sketch.single_needle` (AUTO-detected from
`needle.num_channels`, overridable). Operator-chosen semantics: **a channel/ink
change breaks the bead only in single-needle mode** (ink replacement); multi
keeps channels welded. The panel and the compiled trajectory agree in both.

## Key hardware facts (grounded via an understanding workflow)

`HardwareConfig.needle` is a **single** `NeedleSpec` (`num_channels`, default 1,
+ `channel_pump_map`); `HardwareConfig.pumps` is a **dict** `{"P1":
PumpChannelConfig, ...}` each with `.inks` (list) + a syringe; `InkSpec.color`
(hex, NOT `.display_color`). So `pump_index` selects which pump/material feeds
the one tip, and the panel labels channels with the real ink name+colour.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SketchTrajectory.py` | `SketchShape.overlap_closure` + `Sketch.single_needle` (tri-state) + `Sketch.is_single_needle(needle)` (conditional serialize → legacy-identical); `extend_closed_path(path, overshoot)`; compiler tracks `prev_print_pump` (set only on a printing pass) and, in single mode, refuses to weld across a channel change (`pump_ok`, skipped for move-only passes); applies the over-closure overshoot (needle od/2, fallback ½ bead) after the weld-flip; `plan_print_sections(sketch, needle, single_needle)`; `optimize_print_order` gained `needle`/`single_needle` (single mode groups same-channel shapes) |
| `gui/widgets/sketch_canvas.py` | `select_indices(indices)` (panel click→select); `optimize(needle=None)` threads the needle |
| `gui/pages/print_builder_sketch.py` | persistent **Print sequence** card (needle-mode checkbox auto-synced + overridable, colour-coded section rows click→select, break rows, summary); overlap-closure checkbox in the shape card (closed loops); `_channel_info` from `config.pumps`; dict-safe pump iteration (fixes a latent syringe-detection bug where `pumps` was iterated as a list); `_refresh_sequence` on every preview |
| `tests/test_v75x_sketch_overlap_sequence_needle_mode.py` | NEW — 31 headless + offscreen tests |
| `CLAUDE.md` | new plan added to the Existing Update Plans table |

## Implementation Steps

- [x] `overlap_closure` + `single_needle` model fields + `is_single_needle` +
  conditional serialization
- [x] `extend_closed_path` — continue a closed ring past its seam by `overshoot`
  mm (≤ one lap); no-op on open/degenerate paths
- [x] Compiler: over-closure overshoot per pass (after weld-flip); single-mode
  channel-weld break via `prev_print_pump` (printing passes only) + `pump_ok`
- [x] `plan_print_sections` — mirrors the compiler grouping (first-pass weld +
  far-end flip; last-pass end + overshoot as the exit; break_before
  move/ink_change/layer); groups by OBJECT
- [x] Pump-aware `optimize_print_order` (single mode prefers same channel)
- [x] Canvas `select_indices` + `optimize(needle)`
- [x] Page: sequence card + needle-mode toggle + overlap toggle + `_channel_info`
- [x] Adversarial review (4 dims → skeptic verify): 5 confirmed, 1 rejected;
  fixed (see below); tests + suites green (288)

## Testing Notes

Repo uses **unittest**:
```
python -m unittest tests.test_v75x_sketch_overlap_sequence_needle_mode -v   # 31 OK
```
Regression (288 OK): all `tests.test_v75x_sketch_*` + print-library /
multi-object-seam / quick-print-travel-split / extrusion-volume /
quick-print-workflow.

Coverage: `extend_closed_path` (extend/one-lap-cap/open-noop/zero-noop); compiler
over-closure (needle od/2, ½-bead fallback, no-op on open+filled); `is_single_needle`
resolution + serialize; single-mode channel-weld break + same-pump-welds + explicit-multi
override; `plan_print_sections` (multi=1 section, single splits with ink_change,
travel item, move break, length/indices); pump-aware optimizer grouping; page
(channel_info from dict, auto-detect checkbox, toggle→explicit, single/multi row
counts, section click→select, overlap checkbox gating, dict syringe detection);
**+ the 3 review-fix regressions** (no_print doesn't block a weld; thick-outline
last-pass exit; overlap-extended exit).

**Needs GUI verification on ME3B V1:** draw connected shapes → the Print-sequence
panel groups them into one colour-coded section; add a retract point / a
different-pump shape (single mode) → it splits with a "quick move" / "ink
replacement" break; a click selects a section's shapes. Toggle a closed loop's
"Overlap closure" → the preview shows the path continuing ~a needle radius past
the seam; confirm on hardware the seam now fully closes. Toggle single/multi
needle → sections regroup and the compiled print welds (multi) or lifts (single)
across a channel change accordingly.

## Issues & Decisions

- **Operator-chosen semantics (via AskUserQuestion):** channel change breaks the
  bead **only in single-needle mode** (multi keeps welding across channels);
  single-needle mode **auto-detects** from `needle.num_channels==1`, overridable.
  So multi-pump sketches keep their legacy welded behaviour; a single-channel rig
  (the ME3B default) now correctly lifts between materials.
- **Panel == compiled trajectory.** `plan_print_sections` re-derives the exact
  same grouping the compiler prints (weld rule, far-end flip, single-mode channel
  break, overlap-extended exit, last-pass end for thick outlines) so the panel
  never lies about the print.
- **Sections group by OBJECT** (layer 0). A thick multi-pass outline lifts
  internally between its concentric passes (they're a full bead apart, by the
  pre-existing weld design) — that is intra-object and NOT shown as a section
  break; and odd layers reverse each pass (serpentine), so their inter-shape
  welds can differ slightly. `count_discontinuities` (physical pen-ups) and the
  panel section count therefore measure different things and need not match.
- **Over-closure overshoot = needle OUTER radius** (od/2; ½ bead when the needle
  Ø is unknown) — the radius that pushes ink aside on re-entry. Default OFF
  (opt-in), per-shape, closed loops only. `extend_closed_path` never re-closes
  the ring (by design — it intentionally ends past the seam) and never walks more
  than one lap.
- **Adversarial review (5 confirmed / 1 rejected):**
  - HIGH — `prev_print_pump` was updated by move-only (`no_print`) passes,
    wrongly blocking a later same-channel weld in single mode → now set only on a
    printing pass, and the channel gate is skipped for move-only passes (they
    deposit nothing → never an ink swap). Mirrored in `plan_print_sections`.
  - HIGH — `plan_print_sections` used the un-extended endpoint; the compiler ends
    past the seam with `overlap_closure` → the plan now applies the same overshoot
    to the exit endpoint.
  - MEDIUM — `plan_print_sections` used `paths[0]`'s end; the compiler ends at the
    LAST pass for thick outlines → the plan now uses the last pass's end.
  - MEDIUM ×2 — thick-outline intra-pass lifts + overlap/odd-layer direction are
    documented as intentional layer-0/per-object approximations (see above).
  - REJECTED — "extend_closed_path doesn't re-close": intentional (it must end
    past the seam).
- **Latent bug fixed:** the Sketch page iterated `config.pumps` as a list (it is
  a dict), so it never found a syringe → volume fell back to `flow_factor`. Now
  iterates the dict values (dict/list-tolerant `_pump_values`).
