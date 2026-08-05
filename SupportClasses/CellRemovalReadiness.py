"""v7.9 — "can this machine run this cell-removal job?" as a testable model.

The Cell Targeting page had the same defect Quick Print had before v7.7, in a
sharper form: every refusal lived as a literal string inside ``_on_start``, so
the ONLY way to discover why the run would not go was to click a button that was
already disabled. Meanwhile the status line said "Idle.", the Setup tab printed a
green ✓ next to configurations Start would refuse, and the most consequential
advisory of all — an unmeasured bore mount offset, which dooms the entire run to
dose every cell and collect none — was appended to the status label *after* the
executor thread had already started.

This module is the evaluation half of the fix, deliberately built on
:mod:`SupportClasses.PrintReadiness` rather than beside it: ``Check``,
``Readiness`` and the four states are IMPORTED, so the two pages cannot drift and
the same ``ReadinessList`` widget renders both.

The two governing rules are inherited verbatim:

1. **Unknown data is never blocking.** A field the page could not resolve maps to
   ``info``/``warn``. A model that can disable Start forever because a getter
   returned ``None`` is worse than no model.
2. **``can_start()`` mirrors the pre-existing gates** — plus exactly two
   promotions, both of which were advisory-only and both of which silently
   destroy a run's biological material:

   * an active bore with **no measured mount offset** (dose every cell 100-500 µm
     off-target, collect nothing);
   * a bore programmed to a pump the mounted assembly does not declare (drives a
     syringe that is not plumbed to this needle).

Everything else stays a visible warning the operator can proceed past.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from SupportClasses.PrintReadiness import (      # noqa: F401  (re-exported)
    BLOCK, INFO, OK, WARN, Check, Readiness,
)

#: Groups this module emits, in display order. They are registered in
#: ``PrintReadiness.GROUPS`` so ``Readiness.by_group()`` orders them correctly.
GROUPS = ("Hardware", "Needle assembly", "Targets", "Reagents", "Calibration",
          "Fluidics", "Routine")


@dataclass
class BoreView:
    """One bore's resolved state, as the GUI sees it.

    Deliberately flat and pre-resolved: this module must not reach for a needle,
    a controller or a store. The page does the getattr-safe reads.
    """
    index: int                       # 0-based; displayed as index + 1
    role: str = "idle"               # BoreRole value
    pump_id: Optional[str] = None    # what the operator programmed
    declared_pump: Optional[str] = None   # what the assembly declares
    offset_measured: Optional[bool] = None   # None = unknown
    offset_um: tuple[float, float] = (0.0, 0.0)
    orifice_area_mm2: Optional[float] = None
    internal_volume_uL: Optional[float] = None
    dose_volume_uL: Optional[float] = None
    dose_rate_uL_s: Optional[float] = None
    flow_ceiling_uL_s: Optional[float] = None
    target_type_id: str = ""
    target_type_missing: bool = False

    @property
    def is_active(self) -> bool:
        return self.role != "idle"

    @property
    def label(self) -> str:
        return f"Bore {self.index + 1}"


@dataclass
class CellRemovalContext:
    """Everything :func:`evaluate` needs, already resolved by the caller.

    ``None`` always means "the page could not determine this" and never blocks.
    Volumes are µL, lengths mm, rates µL/s, times s.
    """
    # ── hardware ──
    xy_connected: bool = False
    zp_connected: bool = False
    stage_busy_reason: str = ""

    # ── needle assembly ──
    needle_configured: bool = False
    bore_count: int = 0
    bores: list[BoreView] = field(default_factory=list)

    # ── targets ──
    n_picks: int = 0
    n_places: int = 0
    n_unshifted_mosaic: int = 0
    mosaic_error_bound_um: Optional[float] = None

    # ── reagents ──
    dosing_enabled: bool = False
    dosing_reagent: Optional[str] = None
    dosing_well_calibrated: Optional[bool] = None
    dose_volume_uL: Optional[float] = None
    release_reagent: Optional[str] = None
    release_well_calibrated: Optional[bool] = None
    column_volume_uL: Optional[float] = None
    prep_enabled: bool = False
    clean_enabled: bool = False
    missing_service_wells: list[str] = field(default_factory=list)
    unconfigured_prep_pumps: list[str] = field(default_factory=list)

    # ── calibration heights ──
    safe_z_mm: Optional[float] = None
    plate_bottom_calibrated: bool = False
    removal_clearance_mm: Optional[float] = None
    place_clearance_mm: Optional[float] = None

    # ── timing ──
    lead_time_s: Optional[float] = None
    incubation_s: Optional[float] = None

    # ── fluidics ──
    pull_volume_uL: Optional[float] = None
    extract_multiplier: Optional[float] = None


#: Below this a metered move is at or under one pump microstep on this class of
#: hardware. There is no ``uL_per_step`` accessor anywhere in the codebase, so the
#: wording stays "likely" rather than asserting.
MIN_MEANINGFUL_UL = 0.001


def _fmt_uL(v: Optional[float]) -> str:
    """µL, switching to nL below 0.01 µL so a real dose is not printed as 0.0000."""
    if v is None:
        return "—"
    v = float(v)
    if v and abs(v) < 0.01:
        return f"{v * 1000.0:.3g} nL"
    return f"{v:.4f} µL"


def evaluate(ctx: CellRemovalContext) -> Readiness:
    """Pure evaluation. No Qt, no controller, no hardware."""
    checks: list[Check] = []

    def add(cid, group, label, state, detail="", fix=""):
        checks.append(Check(id=cid, group=group, label=label, state=state,
                            detail=detail, fix=fix))

    # ── Hardware ─────────────────────────────────────────────────────
    add("xy", "Hardware", "XY stage",
        OK if ctx.xy_connected else BLOCK,
        "connected" if ctx.xy_connected else "not connected",
        "" if ctx.xy_connected else "Hardware Setup → Connect Hardware")
    add("zp", "Hardware", "Z + pump board",
        OK if ctx.zp_connected else BLOCK,
        "connected" if ctx.zp_connected
        else "not connected — needed to retract the needle and run the pumps",
        "" if ctx.zp_connected else "Hardware Setup → Connect Hardware")
    if ctx.stage_busy_reason:
        add("busy", "Hardware", "Stage busy", BLOCK, ctx.stage_busy_reason)

    # ── Needle assembly ──────────────────────────────────────────────
    if not ctx.needle_configured:
        add("needle", "Needle assembly", "Needle", BLOCK,
            "no needle configured",
            "Hardware Setup → Needle")
    else:
        add("needle", "Needle assembly", "Needle", OK,
            f"{ctx.bore_count} bore{'s' if ctx.bore_count != 1 else ''}")

    active = [b for b in ctx.bores if b.is_active]
    aspirators = [b for b in ctx.bores if b.role == "aspirate_target"]
    dosers = [b for b in ctx.bores if b.role == "push_reagent"]

    if ctx.needle_configured and not aspirators:
        add("aspirate_role", "Needle assembly", "Aspirating bore", WARN,
            "no bore is set to aspirate — bore 1 will be used",
            "Plan → give a bore the aspirate role")
    elif len(aspirators) > 1:
        add("aspirate_role", "Needle assembly", "Aspirating bore", WARN,
            f"{len(aspirators)} bores are set to aspirate; the first is used",
            "Plan → set the others to Idle")
    if len(dosers) > 1:
        add("dose_role", "Needle assembly", "Dosing bore", WARN,
            f"{len(dosers)} bores are set to dose; the first is used",
            "Plan → set the others to Idle")

    # PROMOTION 1 — an active non-datum bore with no measured mount offset.
    unmeasured = [b for b in active
                  if b.index != 0 and b.offset_measured is False]
    if unmeasured:
        names = ", ".join(b.label for b in unmeasured)
        add("bore_offsets", "Needle assembly", "Bore mount offsets", BLOCK,
            f"{names} have no measured offset, so the run would dose every "
            f"target and then aspirate 100-500 µm away from it — destroying the "
            f"cells without collecting any",
            "Calibration → Needle Location (re-measure after every needle "
            "change or re-seat)")
    elif any(b.offset_measured for b in active if b.index != 0):
        measured = [b for b in active if b.index != 0 and b.offset_measured]
        add("bore_offsets", "Needle assembly", "Bore mount offsets", OK,
            "; ".join(f"{b.label} {b.offset_um[0]:+.0f}, {b.offset_um[1]:+.0f} µm"
                      for b in measured))

    # PROMOTION 2 — a bore programmed to a pump the assembly does not declare.
    rewired = [b for b in active
               if b.declared_pump and b.pump_id
               and b.pump_id.upper() != b.declared_pump.upper()]
    if rewired:
        add("bore_pumps", "Needle assembly", "Bore → pump wiring", BLOCK,
            "; ".join(f"{b.label} is programmed to {b.pump_id} but the mounted "
                      f"assembly declares {b.declared_pump}" for b in rewired)
            + " — the run would drive a syringe that is not plumbed to this needle",
            "Hardware Setup → Needle, or fix the Pump column on Plan")

    unassigned = [b for b in active if not b.pump_id]
    if unassigned:
        add("bore_unassigned", "Needle assembly", "Bore → pump wiring", BLOCK,
            f"{', '.join(b.label for b in unassigned)} has no pump assigned",
            "Plan → Pump column, or Hardware Setup → Needle")

    # ── Targets ──────────────────────────────────────────────────────
    if ctx.n_picks == 0:
        add("targets", "Targets", "Removal targets", BLOCK,
            "none picked yet",
            "Plan → click each cell to remove on the well view")
    elif ctx.n_picks != ctx.n_places:
        add("targets", "Targets", "Removal targets", BLOCK,
            f"{ctx.n_picks} removal(s) but {ctx.n_places} placement(s) — each "
            f"removal needs a paired placement",
            "Plan → add the missing placements on the well view")
    else:
        add("targets", "Targets", "Removal targets", OK,
            f"{ctx.n_picks} cell(s), each with a placement")

    if ctx.n_unshifted_mosaic:
        bound = (f" by up to ~{ctx.mosaic_error_bound_um:.0f} µm"
                 if ctx.mosaic_error_bound_um else "")
        add("mosaic_shift", "Targets", "Mosaic registration", BLOCK,
            f"{ctx.n_unshifted_mosaic} target(s) came from a mosaic whose "
            f"registration shift was never recorded, so their pixel→stage "
            f"mapping may be off{bound} — larger than a cell",
            "Re-scan the well, or go to each target and click it on the live view")

    # ── Reagents ─────────────────────────────────────────────────────
    if ctx.dosing_enabled:
        if not ctx.dosing_reagent:
            add("dose_reagent", "Reagents", "Dosing reagent", BLOCK,
                "the dosing bore has no reagent selected",
                "Plan → Reagents")
        elif ctx.dosing_well_calibrated is False:
            add("dose_reagent", "Reagents", "Dosing reagent", BLOCK,
                f"{ctx.dosing_reagent}'s well is not calibrated",
                "Calibration → Plate Location")
        else:
            add("dose_reagent", "Reagents", "Dosing reagent", OK,
                f"{ctx.dosing_reagent}")
        if ctx.dose_volume_uL is not None and ctx.dose_volume_uL <= 0:
            add("dose_volume", "Reagents", "Dose volume", BLOCK,
                "resolves to 0 µL — nothing would be delivered",
                "Set an explicit volume, or a dose depth with that bore's "
                "geometry on Hardware Setup → Needle")
        elif ctx.dose_volume_uL is not None:
            state = WARN if ctx.dose_volume_uL < MIN_MEANINGFUL_UL else OK
            detail = _fmt_uL(ctx.dose_volume_uL)
            if state == WARN:
                detail += " — likely below one pump microstep, so the move may "\
                          "do nothing"
            add("dose_volume", "Reagents", "Dose volume", state, detail)
        # The aspirate-side reagent is NOT required when a dosing bore is armed:
        # the dosing bore REPLACES that push (it does not add to it), so demanding
        # one is what used to force the operator into a double-dose configuration.
        add("release_reagent", "Reagents", "Cell-release reagent", INFO,
            "not needed — the dosing bore delivers the reagent, so the "
            "aspirating bore loads nothing and only pulls the cell up")
    else:
        if not ctx.release_reagent:
            add("release_reagent", "Reagents", "Cell-release reagent", BLOCK,
                "none selected",
                "Plan → Reagents")
        elif ctx.release_well_calibrated is False:
            add("release_reagent", "Reagents", "Cell-release reagent", BLOCK,
                f"{ctx.release_reagent}'s well is not calibrated",
                "Calibration → Plate Location")
        else:
            add("release_reagent", "Reagents", "Cell-release reagent", OK,
                f"{ctx.release_reagent}")

    if ctx.column_volume_uL is not None and ctx.column_volume_uL <= 0:
        add("column_volume", "Reagents", "Reagent column", BLOCK,
            "the aspirating bore's column resolves to 0 µL, so the extraction "
            "volume would be 0 — nothing would be collected",
            "Set the aspirating bore's geometry on Hardware Setup → Needle")
    elif ctx.column_volume_uL is not None:
        # Symmetric with the dose_volume check above, and the consequence here is
        # WORSE: the column sizes the extraction pull too (pull = column ×
        # multiplier), so a sub-microstep column means the cell is never picked
        # up either. A 30 µm bore at the historic 0.100 mm default column depth
        # is ~71 pL — 14× under this floor — and nothing warned.
        state = WARN if ctx.column_volume_uL < MIN_MEANINGFUL_UL else OK
        detail = _fmt_uL(ctx.column_volume_uL)
        if state == WARN:
            detail += (" — likely below one pump microstep, so neither the push "
                       "nor the extraction pull would move any fluid")
        add("column_volume", "Reagents", "Reagent column", state, detail,
            "Raise the reagent dose on Run, or use a coarser bore"
            if state == WARN else "")

    if (ctx.prep_enabled or ctx.clean_enabled) and ctx.missing_service_wells:
        add("service_wells", "Reagents", "Service wells", BLOCK,
            f"needed and not assigned/calibrated: "
            f"{', '.join(ctx.missing_service_wells)}",
            "Hardware Setup → Ink (Reagent Locations), or turn prep/clean off")

    if ctx.unconfigured_prep_pumps:
        add("prep_pumps", "Reagents", "Prep pumps", BLOCK,
            f"needle prep would drive a pump that is not set up: "
            f"{', '.join(ctx.unconfigured_prep_pumps)} — prep would fail with "
            f"the needle already dipped in a service well",
            "Hardware Setup → Pump")

    # ── Calibration ──────────────────────────────────────────────────
    add("safe_z", "Calibration", "Safe Z", OK if ctx.safe_z_mm is not None else BLOCK,
        f"{ctx.safe_z_mm:.3f} mm" if ctx.safe_z_mm is not None
        else "not configured — the needle could not be retracted for travel",
        "" if ctx.safe_z_mm is not None else "Calibration → Needle Offset")
    add("plate_bottom", "Calibration", "Plate bottom Z",
        OK if ctx.plate_bottom_calibrated else BLOCK,
        "calibrated" if ctx.plate_bottom_calibrated
        else "not calibrated — the removal and placement heights cannot be resolved",
        "" if ctx.plate_bottom_calibrated else "Calibration → Plate Z Auto-Cal")
    if ctx.removal_clearance_mm is not None:
        state = WARN if ctx.removal_clearance_mm <= 0 else OK
        add("removal_z", "Calibration", "Removal height", state,
            f"{ctx.removal_clearance_mm:.3f} mm above the plate bottom"
            + (" — the needle would touch the glass" if state == WARN else ""))

    # ── Fluidics ─────────────────────────────────────────────────────
    for b in active:
        if b.dose_rate_uL_s and b.flow_ceiling_uL_s \
                and b.dose_rate_uL_s > b.flow_ceiling_uL_s:
            ratio = b.dose_rate_uL_s / b.flow_ceiling_uL_s
            # Only a DOSING bore's flow has a lead-time consequence. The page
            # fills `dose_rate_uL_s` from every active program's rate, so
            # labelling an aspirating bore's rate a "dose flow" — and citing a
            # lead time that does not apply to it — was simply wrong.
            dosing = b.role == "push_reagent"
            add(f"flow_{b.index}", "Fluidics",
                f"{b.label} {'dose' if dosing else 'pump'} flow", WARN,
                f"{b.dose_rate_uL_s:.2f} µL/s exceeds this bore's flow ceiling "
                f"({b.flow_ceiling_uL_s:.3f} µL/s) — it will be auto-limited, so "
                + (f"the dose takes {ratio:.0f}× longer than the lead time assumes"
                   if dosing else
                   f"that step takes {ratio:.0f}× longer than the rate implies"),
                "Lower the flow on Plan")
        if b.dose_volume_uL and b.internal_volume_uL \
                and b.dose_volume_uL > b.internal_volume_uL:
            add(f"holdup_{b.index}", "Fluidics", f"{b.label} dose volume", WARN,
                f"{_fmt_uL(b.dose_volume_uL)} exceeds what this bore holds "
                f"({_fmt_uL(b.internal_volume_uL)}) — the plunger will clamp and "
                f"the volume balance will break",
                "Lower the dose on Plan")

    if ctx.pull_volume_uL is not None:
        add("pull_volume", "Fluidics", "Extraction volume", OK,
            f"{_fmt_uL(ctx.pull_volume_uL)}"
            + (f" ({ctx.extract_multiplier:.2f}× the column)"
               if ctx.extract_multiplier else ""))

    # ── Routine ──────────────────────────────────────────────────────
    if ctx.dosing_enabled and ctx.lead_time_s is not None \
            and ctx.incubation_s is not None:
        add("timing", "Routine", "Dose → aspirate interval", OK,
            f"{ctx.lead_time_s + ctx.incubation_s:.1f} s "
            f"({ctx.lead_time_s:.1f} s lead + {ctx.incubation_s:.1f} s incubation)")

    for b in ctx.bores:
        if b.target_type_missing:
            add(f"tt_{b.index}", "Needle assembly", f"{b.label} target type", WARN,
                f"'{b.target_type_id}' is not on disk — the assignment is kept "
                f"but the type definition is missing",
                "Plan → Target types, or restore the file")

    add("prep", "Routine", "Needle prep",
        OK, "on" if ctx.prep_enabled else "off (the needle is used as-is)")
    add("clean", "Routine", "Post-clean",
        OK, "on" if ctx.clean_enabled else "off (the needle stays loaded)")

    return Readiness(checks=checks)
