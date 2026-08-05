"""v7.7 — "can this machine honour this print?" as a structured, testable model.

Quick Print used to answer that question with ONE word-wrapping label assembled
from up to ten ``"   "``-joined clauses, and to gate its Print button on four
ANDed booleans that could not say which of them was false. Everything else the
software knew — how stale the calibration is, which of three limits is binding,
how much syringe headroom is left, whether the ink's particles can even pass the
needle bore — was either buried in a modeless popout, mentioned only on failure,
or never computed at all.

This module is the evaluation half of the replacement: a **pure** function over
already-resolved numbers. It imports nothing from Qt and nothing from the
controller, so every branch is unit-testable without hardware. The GUI does the
getattr-safe reads (that is where mock tolerance belongs), fills a
:class:`ReadinessContext`, and renders the :class:`Readiness` it gets back.

Two rules govern the design, both learned the hard way:

1. **Unknown data is never blocking.** A field the page could not resolve maps
   to ``info``/``warn``. A readiness model that can disable Print forever
   because a getter returned ``None`` is worse than no model.
2. **``can_print()`` mirrors the pre-existing gates.** Only the four conditions
   Quick Print already required (XY+ZP connected, a well, a plate, an object)
   plus the one physically-impossible case (ink particles larger than the needle
   bore) are ``block``. Everything else stays a visible warning the operator can
   proceed past, exactly as before — the checklist explains it instead of a
   button silently refusing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

# ── states ───────────────────────────────────────────────────────────
OK = "ok"
INFO = "info"
WARN = "warn"
BLOCK = "block"

#: Groups, in display order.
#
#: v7.9 appends "Needle assembly", "Targets" and "Reagents" for
#: :mod:`SupportClasses.CellRemovalReadiness`, which reuses this module's
#: ``Check``/``Readiness`` rather than defining a parallel pair. This tuple only
#: controls DISPLAY ORDER and grouping in ``Readiness.by_group()``; Quick Print
#: emits no checks in the new groups, so its rendering is unchanged.
GROUPS = ("Hardware", "Selection", "Needle assembly", "Targets", "Reagents",
          "Calibration", "Machine", "Print", "Fluidics", "Ink & needle",
          "Routine")


@dataclass(frozen=True)
class Check:
    """One readiness line.

    ``detail`` states the measured fact (a number wherever one exists — "why"
    beats "what"); ``fix`` names where the operator would change it.
    """
    id: str
    group: str
    label: str
    state: str
    detail: str = ""
    fix: str = ""

    @property
    def is_blocking(self) -> bool:
        return self.state == BLOCK


@dataclass
class Readiness:
    checks: list[Check] = field(default_factory=list)

    def blocking(self) -> list[Check]:
        return [c for c in self.checks if c.state == BLOCK]

    def warnings(self) -> list[Check]:
        return [c for c in self.checks if c.state == WARN]

    def can_print(self) -> bool:
        """True when nothing is explicitly blocking. Warnings do not block."""
        return not self.blocking()

    def can_start(self) -> bool:
        """Alias for :meth:`can_print`, for non-printing consumers.

        v7.9: ``CellRemovalReadiness`` reuses this class, and "can_print" reads
        wrong when the operation is aspirating cells. Same semantics — one
        implementation, so the two pages cannot disagree about what blocks.
        """
        return self.can_print()

    def by_group(self) -> list[tuple[str, list[Check]]]:
        out = []
        for g in GROUPS:
            rows = [c for c in self.checks if c.group == g]
            if rows:
                out.append((g, rows))
        # Anything with an unexpected group still gets shown.
        rest = [c for c in self.checks if c.group not in GROUPS]
        if rest:
            out.append(("Other", rest))
        return out

    def get(self, check_id: str) -> Optional[Check]:
        for c in self.checks:
            if c.id == check_id:
                return c
        return None

    def headline(self) -> str:
        """One short line for a collapsed/summary view."""
        blocking = self.blocking()
        if blocking:
            return f"Not ready — {blocking[0].label}: {blocking[0].detail}"
        warns = self.warnings()
        if warns:
            n = len(warns)
            return (f"Ready, with {n} warning{'' if n == 1 else 's'} — "
                    f"{warns[0].label}: {warns[0].detail}")
        return "Ready."


@dataclass
class ReadinessContext:
    """Everything :func:`evaluate` needs, already resolved by the caller.

    Every field is optional: ``None`` means "the page could not determine this",
    which never blocks. Volumes are µL, lengths mm, speeds mm/s, times s.
    """
    # ── hardware ──
    xy_connected: bool = False
    zp_connected: bool = False

    # ── selection ──
    object_selected: bool = False
    object_label: str = ""
    well: Optional[str] = None
    plate_available: bool = False
    #: True when the well centre comes from the taught calibration, False when it
    #: fell back to plate geometry, None when unknown.
    well_center_calibrated: Optional[bool] = None

    # ── calibration heights ──
    safe_z: Optional[float] = None
    plate_bottom_z: Optional[float] = None
    print_z_zref: Optional[float] = None
    travel_z_zref: Optional[float] = None
    #: True when no Safe Z was calibrated and a travel height had to be derived.
    travel_z_synthesised: bool = False

    # ── machine characterisation ──
    char_complete: Optional[bool] = None
    char_missing: tuple[str, ...] = ()
    char_measured_at: Optional[str] = None
    dead_time_s: Optional[float] = None
    #: "measured" | "phase_lag" | "unmeasured"
    dead_time_source: str = ""
    motion_mode: str = ""
    #: The mode actually used once "auto" is resolved (v7.7 Stage 5).
    resolved_motion_mode: str = ""

    # ── calibration staleness ──
    xy_travel_since_cal_mm: Optional[float] = None
    xy_recal_travel_mm: Optional[float] = None
    hours_since_xy_cal: Optional[float] = None
    recal_interval_hours: Optional[float] = None

    # ── print parameters ──
    requested_speed_mm_s: Optional[float] = None
    resolved_speed_mm_s: Optional[float] = None
    stage_max_mm_s: Optional[float] = None
    flow_ceiling_speed_mm_s: Optional[float] = None
    resolution_um: Optional[float] = None
    resolution_floor_um: Optional[float] = None
    est_time_s: Optional[float] = None
    n_corner_stops: Optional[int] = None
    path_length_mm: Optional[float] = None
    n_strokes: Optional[int] = None
    bead_width_um: Optional[float] = None
    flow_uL_s: Optional[float] = None

    # ── geometry fit ──
    object_radius_mm: Optional[float] = None
    well_radius_mm: Optional[float] = None

    # ── fluidics ──
    pump: str = ""
    pump_plunger_calibrated: Optional[bool] = None
    syringe_capacity_uL: Optional[float] = None
    syringe_fill_uL: Optional[float] = None
    budget_span_uL: Optional[float] = None
    budget_peak_fill_uL: Optional[float] = None
    budget_min_fill_uL: Optional[float] = None
    budget_overflow_uL: Optional[float] = None
    budget_underflow_uL: Optional[float] = None
    pickup_uL: Optional[float] = None
    pickup_dispense_uL: Optional[float] = None
    pickup_prime_uL: Optional[float] = None
    pickup_dead_volume_uL: Optional[float] = None
    pickup_padding_uL: Optional[float] = None

    # ── ink & needle ──
    needle_configured: bool = False
    needle_gauge: str = ""
    bore_id_um: Optional[float] = None
    ink_name: Optional[str] = None
    ink_well: Optional[str] = None
    ink_well_calibrated: Optional[bool] = None
    #: ``InkSpec.flow_compatibility_detail(needle)`` verbatim, or None.
    clog: Optional[dict] = None
    ink_particle_um: Optional[float] = None
    wall_shear_pa: Optional[float] = None
    shear_limit_pa: Optional[float] = None
    #: Flow ceiling recomputed with the ink's own viscosity vs the shipped
    #: reference-fluid ceiling (µL/s).
    flow_ceiling_ink_uL_s: Optional[float] = None
    flow_ceiling_ref_uL_s: Optional[float] = None

    # ── prep / cleanup routine ──
    prep_enabled: bool = False
    prep_missing: tuple[str, ...] = ()
    cleanup_enabled: bool = False
    cleanup_missing: tuple[str, ...] = ()


def _num(v) -> Optional[float]:
    """Coerce to float, rejecting bools and non-numerics (a bare MagicMock has a
    ``__float__`` of 1.0, which would silently fabricate a limit)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    f = float(v)
    if f != f:                                     # NaN
        return None
    return f


def evaluate(ctx: ReadinessContext) -> Readiness:
    """Build the readiness checklist. Pure — no I/O, no Qt, no controller."""
    out: list[Check] = []
    add = out.append

    # ── Hardware ────────────────────────────────────────────────────
    add(Check("xy", "Hardware", "XY stage",
              OK if ctx.xy_connected else BLOCK,
              "connected" if ctx.xy_connected else "not connected",
              "" if ctx.xy_connected else "Hardware Setup → Device"))
    add(Check("zp", "Hardware", "Z / pump board",
              OK if ctx.zp_connected else BLOCK,
              "connected" if ctx.zp_connected else "not connected",
              "" if ctx.zp_connected else "Hardware Setup → Device"))

    # ── Selection ───────────────────────────────────────────────────
    add(Check("object", "Selection", "Object",
              OK if ctx.object_selected else BLOCK,
              ctx.object_label or "none chosen",
              "" if ctx.object_selected else "Pick an object above"))
    add(Check("plate", "Selection", "Plate",
              OK if ctx.plate_available else BLOCK,
              "loaded" if ctx.plate_available else "no plate",
              "" if ctx.plate_available else "Hardware Setup → Plate"))
    add(Check("well", "Selection", "Well",
              OK if ctx.well else BLOCK,
              ctx.well or "none chosen",
              "" if ctx.well else "Click a well below"))

    _geometry_fit(ctx, add)
    _calibration(ctx, add)
    _machine(ctx, add)
    _print_params(ctx, add)
    _fluidics(ctx, add)
    _ink_and_needle(ctx, add)
    _routine(ctx, add)
    return Readiness(out)


# ── groups ───────────────────────────────────────────────────────────

def _geometry_fit(ctx: ReadinessContext, add) -> None:
    """Does the object actually fit in the well? Quick Print never checked."""
    obj_r = _num(ctx.object_radius_mm)
    well_r = _num(ctx.well_radius_mm)
    if obj_r is None or well_r is None or well_r <= 0:
        return
    if obj_r <= well_r:
        add(Check("fit", "Selection", "Fits the well", OK,
                  f"object Ø {2 * obj_r:.2f} mm in a {2 * well_r:.2f} mm well "
                  f"({100.0 * obj_r / well_r:.0f} % of the radius)"))
    else:
        add(Check("fit", "Selection", "Fits the well", WARN,
                  f"object Ø {2 * obj_r:.2f} mm exceeds the {2 * well_r:.2f} mm "
                  f"well — the outer path would hit the wall",
                  "Reduce the size, or choose a larger well"))


def _calibration(ctx: ReadinessContext, add) -> None:
    safe = _num(ctx.safe_z)
    if safe is None:
        add(Check("safe_z", "Calibration", "Safe / travel Z", WARN,
                  "not calibrated — the needle will not retract to a known "
                  "safe height before travel",
                  "Calibration → Needle Offset"))
    else:
        add(Check("safe_z", "Calibration", "Safe / travel Z", OK,
                  f"{safe:.3f} mm (zero-ref)"))

    bottom = _num(ctx.plate_bottom_z)
    if bottom is None:
        add(Check("plate_bottom", "Calibration", "Plate bottom Z", WARN,
                  "not calibrated — the print height cannot be resolved from "
                  "the plate",
                  "Calibration → Needle Offset"))
    else:
        add(Check("plate_bottom", "Calibration", "Plate bottom Z", OK,
                  f"{bottom:.3f} mm (zero-ref)"))

    # The absolute heights the needle will actually visit — never shown before.
    pz = _num(ctx.print_z_zref)
    tz = _num(ctx.travel_z_zref)
    if pz is not None or tz is not None:
        bits = []
        if pz is not None:
            bits.append(f"print {pz:.3f} mm")
        if tz is not None:
            bits.append(f"travel {tz:.3f} mm")
        if ctx.travel_z_synthesised:
            add(Check("heights", "Calibration", "Resolved heights", WARN,
                      " · ".join(bits) + " — the travel height was DERIVED "
                      "because no Safe Z is calibrated",
                      "Calibration → Needle Offset"))
        else:
            add(Check("heights", "Calibration", "Resolved heights", INFO,
                      " · ".join(bits) + " (zero-ref)"))

    if ctx.well_center_calibrated is False:
        add(Check("well_cal", "Calibration", "Well centre", WARN,
                  "using plate GEOMETRY, not a taught position — the needle may "
                  "not be centred in the well",
                  "Calibration → Plate Location"))
    elif ctx.well_center_calibrated is True:
        add(Check("well_cal", "Calibration", "Well centre", OK, "taught"))

    _staleness(ctx, add)


def _staleness(ctx: ReadinessContext, add) -> None:
    """How long / how far since the XY calibration — tracked by
    CalibrationStatusStore and never read by Quick Print."""
    travel = _num(ctx.xy_travel_since_cal_mm)
    limit = _num(ctx.xy_recal_travel_mm)
    hours = _num(ctx.hours_since_xy_cal)
    interval = _num(ctx.recal_interval_hours)
    bits, stale = [], False
    if travel is not None:
        bits.append(f"{travel / 1000.0:.2f} m travelled since calibration")
        if limit and limit > 0 and travel > limit:
            stale = True
    if hours is not None:
        bits.append(f"{hours:.0f} h since calibration")
        if interval and interval > 0 and hours > interval:
            stale = True
    if not bits:
        return
    add(Check("stale", "Calibration", "Calibration age",
              WARN if stale else OK, " · ".join(bits),
              "Calibration" if stale else ""))


def _machine(ctx: ReadinessContext, add) -> None:
    mode = ctx.resolved_motion_mode or ctx.motion_mode
    if mode:
        detail = mode
        if ctx.resolved_motion_mode and ctx.motion_mode == "auto":
            detail = (f"{ctx.resolved_motion_mode} (auto-selected: "
                      + ("the stage is characterised"
                         if ctx.char_complete else
                         "the stage is not characterised") + ")")
        add(Check("mode", "Machine", "Motion mode", INFO, detail))

    if ctx.char_complete is False:
        missing = ", ".join(ctx.char_missing) if ctx.char_missing else "unknown"
        add(Check("char", "Machine", "Stage characterisation", WARN,
                  f"incomplete — missing {missing}; feature-aware feed planning "
                  f"and the accuracy prediction are unavailable",
                  "Workflows → Timing Calibration (one-click XY calibration)"))
    elif ctx.char_complete is True:
        when = f", measured {ctx.char_measured_at}" if ctx.char_measured_at else ""
        add(Check("char", "Machine", "Stage characterisation", OK,
                  f"complete{when}"))

    dt = _num(ctx.dead_time_s)
    if dt is not None and ctx.dead_time_source:
        src = ctx.dead_time_source
        if src == "measured":
            add(Check("dead_time", "Machine", "Command dead time", OK,
                      f"{dt * 1000.0:.0f} ms (measured)"))
        elif src == "phase_lag":
            add(Check("dead_time", "Machine", "Command dead time", WARN,
                      f"{dt * 1000.0:.0f} ms inferred from the legacy settle "
                      f"intercept, not measured — it caps the print speed",
                      "Timing Calibration → Measure dead time"))
        else:
            add(Check("dead_time", "Machine", "Command dead time", WARN,
                      "not measured — the speed cap is a guess",
                      "Timing Calibration → Measure dead time"))


def _print_params(ctx: ReadinessContext, add) -> None:
    req = _num(ctx.requested_speed_mm_s)
    res_speed = _num(ctx.resolved_speed_mm_s)
    stage_max = _num(ctx.stage_max_mm_s)
    flow_cap = _num(ctx.flow_ceiling_speed_mm_s)

    if res_speed is not None:
        # Name the binding term ALWAYS — previously only on exceedance, so a
        # normal run gave no sense of headroom.
        binder, headroom = "your top speed", None
        if req is not None and res_speed < req - 1e-9:
            if flow_cap is not None and abs(res_speed - flow_cap) < 1e-6:
                binder = f"the needle's flow ceiling ({flow_cap:.2f} mm/s)"
            elif stage_max is not None and abs(res_speed - stage_max) < 1e-6:
                binder = f"the measured stage max ({stage_max:.2f} mm/s)"
            else:
                binder = "a hardware limit"
            add(Check("speed", "Print", "Top speed", WARN,
                      f"asked {req:.2f} mm/s → limited to {res_speed:.2f} mm/s "
                      f"by {binder}",
                      "Lower the top speed, or use a larger needle"))
        else:
            caps = [c for c in (stage_max, flow_cap) if c is not None and c > 0]
            if caps:
                nearest = min(caps)
                headroom = 100.0 * (1.0 - res_speed / nearest) if nearest else None
            detail = f"{res_speed:.2f} mm/s"
            if headroom is not None:
                detail += (f" — {headroom:.0f} % below the nearest hardware "
                           f"limit ({min(caps):.2f} mm/s)")
            add(Check("speed", "Print", "Top speed", OK, detail))

    res = _num(ctx.resolution_um)
    floor = _num(ctx.resolution_floor_um)
    if res is not None:
        if floor is not None and floor > 0 and res < floor:
            add(Check("resolution", "Print", "Resolution", WARN,
                      f"{res:.0f} µm is finer than this machine can hold "
                      f"(~{floor:.0f} µm floor: stop tolerance + coast + "
                      f"encoder) — corners will exceed it",
                      "Ask for a coarser resolution"))
        else:
            extra = f" (machine floor ~{floor:.0f} µm)" if floor else ""
            add(Check("resolution", "Print", "Resolution", OK,
                      f"{res:.0f} µm{extra}"))

    est = _num(ctx.est_time_s)
    if est is not None and est > 0:
        stops = ctx.n_corner_stops or 0
        add(Check("time", "Print", "Estimated time", INFO,
                  f"{est:.0f} s with {stops} corner stop"
                  f"{'' if stops == 1 else 's'} — finer resolution costs time"))

    length = _num(ctx.path_length_mm)
    if length is not None and length > 0:
        strokes = ctx.n_strokes
        detail = f"{length:.1f} mm"
        if strokes:
            detail += (f" in {strokes} stroke{'' if strokes == 1 else 's'} "
                       f"(pen-up between them)")
        add(Check("path", "Print", "Path", INFO, detail))

    bead = _num(ctx.bead_width_um)
    if bead is not None and bead > 0:
        flow = _num(ctx.flow_uL_s)
        detail = f"~{bead:.0f} µm wide"
        if flow:
            detail += f" at {flow:.3f} µL/s"
        add(Check("bead", "Print", "Deposited line", INFO, detail))


def _fluidics(ctx: ReadinessContext, add) -> None:
    if ctx.pump_plunger_calibrated is False:
        add(Check("plunger", "Fluidics", f"Pump {ctx.pump or ''} plunger".strip(),
                  WARN,
                  "not calibrated — volumes are open-loop and the syringe "
                  "budget cannot be checked",
                  "Hardware Setup → Pump → Pump Plunger Setup"))

    cap = _num(ctx.syringe_capacity_uL)
    fill = _num(ctx.syringe_fill_uL)
    if cap is not None and cap > 0 and fill is not None:
        add(Check("fill", "Fluidics", "Syringe now", INFO,
                  f"{fill:.2f} of {cap:.2f} µL ({100.0 * fill / cap:.0f} % full)"))

    over = _num(ctx.budget_overflow_uL) or 0.0
    under = _num(ctx.budget_underflow_uL) or 0.0
    span = _num(ctx.budget_span_uL)
    if over > 1e-9 or under > 1e-9:
        which = "over-fill" if over > under else "run dry"
        amount = max(over, under)
        add(Check("budget", "Fluidics", "Syringe budget", WARN,
                  f"this run would {which} by {amount:.2f} µL",
                  "Reduce the print size, the pickup, or the prep volumes"))
    elif span is not None and cap is not None and cap > 0:
        # Headroom on SUCCESS — previously the check returned silently, so a
        # 0.2 µL margin looked identical to a 200 µL one.
        peak = _num(ctx.budget_peak_fill_uL)
        low = _num(ctx.budget_min_fill_uL)
        bits = [f"needs {span:.2f} of {cap:.2f} µL"]
        if peak is not None and low is not None:
            bits.append(f"plunger spans {low:.2f}–{peak:.2f} µL; "
                        f"{min(cap - peak, low):.2f} µL spare")
        add(Check("budget", "Fluidics", "Syringe budget", OK, " · ".join(bits)))

    pk = _num(ctx.pickup_uL)
    if pk is not None and pk > 0:
        parts = []
        for label, v in (("path", ctx.pickup_dispense_uL),
                         ("prime", ctx.pickup_prime_uL),
                         ("bore reserve", ctx.pickup_dead_volume_uL),
                         ("your padding", ctx.pickup_padding_uL)):
            f = _num(v)
            if f is not None and f > 0:
                parts.append(f"{label} {f:.3f}")
        detail = f"{pk:.3f} µL"
        if parts:
            detail += " = " + " + ".join(parts) + " µL"
        add(Check("pickup", "Fluidics", "Ink pickup", INFO, detail))


def _ink_and_needle(ctx: ReadinessContext, add) -> None:
    if not ctx.needle_configured:
        add(Check("needle", "Ink & needle", "Needle", WARN,
                  "inner Ø / length not configured — flow, bead width and prep "
                  "volumes cannot be derived",
                  "Hardware Setup → Needle"))
    else:
        bore = _num(ctx.bore_id_um)
        detail = ctx.needle_gauge or "configured"
        if bore:
            detail += f" · bore {bore:.0f} µm"
        add(Check("needle", "Ink & needle", "Needle", OK, detail))

    if ctx.ink_name:
        if ctx.ink_well_calibrated is False:
            add(Check("ink", "Ink & needle", "Ink source", WARN,
                      f"“{ctx.ink_name}” → {ctx.ink_well or 'no well'} is not in "
                      f"the calibrated plate",
                      "Hardware Setup → Ink, then Calibration → Plate Location"))
        else:
            add(Check("ink", "Ink & needle", "Ink source", OK,
                      f"“{ctx.ink_name}” ← {ctx.ink_well}"))
    else:
        add(Check("ink", "Ink & needle", "Ink source", INFO,
                  "no pickup — printing with whatever is already loaded"))

    _clog(ctx, add)
    _shear(ctx, add)
    _viscosity(ctx, add)


def _clog(ctx: ReadinessContext, add) -> None:
    """The one hard block: particles that physically cannot pass the bore.

    ``InkSpec.flow_compatibility_detail`` already produces an operator-ready
    message; Quick Print never called it and would happily print a clogging ink
    in silence. Guarded so an unset particle size can never block work.
    """
    particle = _num(ctx.ink_particle_um)
    bore = _num(ctx.bore_id_um)
    if particle is not None and particle > 0 and bore is not None and bore > 0 \
            and particle >= bore:
        add(Check("clog", "Ink & needle", "Particles vs bore", BLOCK,
                  f"{particle:.0f} µm particles cannot pass a {bore:.0f} µm "
                  f"bore — this would jam the needle",
                  "Use a larger needle, or filter the ink"))
        return

    detail = ctx.clog or {}
    status = str(detail.get("status") or "")
    if not status:
        if particle and bore:
            add(Check("clog", "Ink & needle", "Particles vs bore", OK,
                      f"{particle:.0f} µm in a {bore:.0f} µm bore "
                      f"({bore / particle:.1f}× clearance)"))
        return
    msg = str(detail.get("message") or status)
    sev = str(detail.get("severity") or "").lower()
    state = WARN if (sev in ("warn", "warning", "high", "medium")
                     or status == "risk_clogging") else OK
    add(Check("clog", "Ink & needle", "Particles vs bore", state, msg,
              "Use a larger needle, or filter the ink" if state == WARN else ""))


def _shear(ctx: ReadinessContext, add) -> None:
    """Wall shear vs the cell-viability limit — computed nowhere for a print
    today, though FlowPhysics calls it "critical for cell viability"."""
    shear = _num(ctx.wall_shear_pa)
    limit = _num(ctx.shear_limit_pa)
    if shear is None:
        return
    if limit and limit > 0 and shear > limit:
        add(Check("shear", "Ink & needle", "Wall shear stress", WARN,
                  f"{shear:.2f} Pa exceeds the {limit:.1f} Pa cell-viability "
                  f"limit — cells in this ink may be damaged",
                  "Lower the top speed, or use a larger needle"))
    else:
        extra = f" (limit {limit:.1f} Pa)" if limit else ""
        add(Check("shear", "Ink & needle", "Wall shear stress", OK,
                  f"{shear:.2f} Pa{extra}"))


def _viscosity(ctx: ReadinessContext, add) -> None:
    """The shipped flow ceiling is computed against a REFERENCE fluid, not the
    selected ink. Say so when the ink's own viscosity moves it materially."""
    ink_cap = _num(ctx.flow_ceiling_ink_uL_s)
    ref_cap = _num(ctx.flow_ceiling_ref_uL_s)
    if ink_cap is None or ref_cap is None or ref_cap <= 0:
        return
    ratio = ink_cap / ref_cap
    if ratio < 0.8:
        add(Check("visc", "Ink & needle", "Flow ceiling vs this ink", WARN,
                  f"the enforced ceiling ({ref_cap:.2f} µL/s) assumes a "
                  f"reference fluid; with this ink's viscosity it is "
                  f"{ink_cap:.2f} µL/s — {100.0 * (1 - ratio):.0f} % lower",
                  "Lower the top speed, or use a larger needle"))
    else:
        add(Check("visc", "Ink & needle", "Flow ceiling vs this ink", OK,
                  f"{ink_cap:.2f} µL/s with this ink "
                  f"(enforced {ref_cap:.2f} µL/s)"))


def _routine(ctx: ReadinessContext, add) -> None:
    if ctx.prep_enabled:
        if ctx.prep_missing:
            add(Check("prep", "Routine", "Needle prep", WARN,
                      "needs " + ", ".join(ctx.prep_missing),
                      "Hardware Setup → Ink → Reagent Locations"))
        else:
            add(Check("prep", "Routine", "Needle prep", OK,
                      "waste → oil → wash → buffer before the print"))
    else:
        add(Check("prep", "Routine", "Needle prep", INFO,
                  "off — the needle is used as-is"))

    if ctx.cleanup_enabled:
        if ctx.cleanup_missing:
            add(Check("cleanup", "Routine", "Post-print reset", WARN,
                      "needs " + ", ".join(ctx.cleanup_missing),
                      "Hardware Setup → Ink → Reagent Locations"))
        else:
            add(Check("cleanup", "Routine", "Post-print reset", OK,
                      "waste unprinted ink+buffer → wash → top up oil"))
    else:
        add(Check("cleanup", "Routine", "Post-print reset", INFO,
                  "off — the syringe keeps whatever is left after the print"))
