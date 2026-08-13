"""
Turret position ↔ objective name ↔ µm/px, resolved in ONE place.

Two records of "which objective" have always existed side by side and never been
compared:

* ``MicroscopeConfigStore.objectives`` maps **turret position → operator name**
  ("position 3 is the 20x"). Only the operator knows this; the body reports a
  position, not a name.
* ``ObjectiveCalibration`` maps **camera name → objective name → µm/px**. Its key
  is ``CameraConfig.current_objective_name``, which is set only on the Cameras
  tab.

Nothing wrote one from the other, so rotating the nosepiece left the µm/px
calibration pointing at whatever the operator last selected on a different page.
CLAUDE.md records that link as deliberately deferred, because wiring it fully
means deciding whether rotating the turret re-points the calibration and what
happens mid-print.

**This module makes the narrowest link that unblocks a driven survey:** it
RESOLVES µm/px from a turret position, read-only, for the duration of one
operation. It never writes ``current_objective_name`` — doing so would change the
µm/px every other surface reads, mid-run, from a background thread.

v7.18: the turret-position → name half of that join moved to
``OpticsRegistry.resolve_slots``, which serves BOTH turrets and adds the reverse
``name → slot`` lookup a workflow needs before it can switch. This module keeps
the objective-only half — the µm/px resolution and the ladder's fitness gate — and
is now a consumer of the registry rather than a second implementation of the walk.

THE PRECEDENCE RULE, AND WHY IT REFUSES
---------------------------------------
Exactly one source: ``ObjectiveCalibration[camera][objective_name]``, rescaled to
the live resolution. There is deliberately **no fallback** to the live
``CameraManager`` value or to ``current_objective_name``. That fallback is
precisely how "I literally just calibrated it" happened — a stale value shadowing
the measurement the operator had just made. An uncalibrated rung refuses, with a
sentence naming the objective and the camera.

Pure: no Qt, no hardware. Stores are passed in, never imported as singletons.
"""

from __future__ import annotations

from dataclasses import dataclass

from SupportClasses.OpticsRegistry import (
    OBJECTIVE, normalize_optic_name, optic_at, resolve_slots)


@dataclass(frozen=True, kw_only=True)
class LadderRung:
    """One rung of an objective ladder, fully resolved."""
    turret_position: int
    objective_name: str = ""
    product_code: str = ""

    um_per_px: float | None = None
    um_per_px_resolution: tuple[int, int] | None = None
    rotation_deg: float | None = None

    magnification: float | None = None
    numerical_aperture: float | None = None
    working_distance_mm: float | None = None
    #: v7.18 — refractive index for the DOF diffraction term (1.0 = dry).
    immersion_n: float = 1.0
    #: v7.18 — where the optics came from, and any body-vs-spec disagreement
    #: about working distance (the shorter value is used; see OpticsRegistry).
    optics_source: str = ""
    optics_conflict: str = ""

    calibrated: bool = False
    why_not: str = ""

    @property
    def selectable(self) -> bool:
        return bool(self.objective_name)

    def describe(self) -> str:
        who = self.objective_name or f"position {self.turret_position}"
        if not self.calibrated:
            return f"{who} — not usable"
        return f"{who} @ {self.um_per_px:.4g} µm/px"


def um_per_px_at_resolution(cal: dict | None,
                            live_width: float | None) -> float | None:
    """Rescale a stored µm/px to the resolution the camera is running at.

    µm/px is inversely proportional to frame width, so a calibration recorded at
    1024 px and used at 2048 px is 2× wrong. When the calibration carries no
    resolution stamp the value is returned unchanged — it CANNOT be rescaled, and
    inventing a scale would be worse than using it as-is.
    """
    if not cal:
        return None
    try:
        base = float(cal.get("measured_um_per_px") or 0.0)
    except (TypeError, ValueError):
        return None
    if base <= 0:
        return None
    res = cal.get("resolution")
    if not res or not live_width:
        return base
    try:
        cal_w = float(res[0])
        live_w = float(live_width)
    except (TypeError, ValueError, IndexError):
        return base
    if cal_w <= 0 or live_w <= 0:
        return base
    return base * cal_w / live_w


def _optic_at(state, position: int):
    """Retained name; v7.18 delegates to the shared ``OpticsRegistry.optic_at``.

    Kept so existing callers and tests keep working while there is exactly one
    implementation of the ``mounted_objectives`` walk.
    """
    return optic_at(state, position, OBJECTIVE)


def resolve_ladder(*, scope_state, config_store, objective_store,
                   camera_name: str,
                   live_resolution: tuple[int, int] | None = None,
                   positions=None) -> tuple[LadderRung, ...]:
    """Resolve every nosepiece position into a :class:`LadderRung`.

    ``positions`` restricts the result to a chosen ladder; omitting it resolves
    every slot the body reports, which is what the configuration UI wants.

    v7.18: the position → (name, optics) join is ``OpticsRegistry.resolve_slots``.
    The signature, the ``LadderRung`` shape and every refusal sentence are
    unchanged — this is the same answer from one shared join.
    """
    slots = {s.position: s for s in resolve_slots(
        scope_state=scope_state, config_store=config_store, kind=OBJECTIVE)}

    live_w = None
    if live_resolution:
        try:
            live_w = float(live_resolution[0])
        except (TypeError, ValueError, IndexError):
            live_w = None

    wanted = ([int(p) for p in positions] if positions
              else sorted(slots))

    out: list[LadderRung] = []
    for pos in wanted:
        slot = slots.get(pos)
        name = slot.name if slot else ""
        code = slot.code if slot else ""
        mag = slot.magnification if slot else None
        na = slot.numerical_aperture if slot else None
        wd = slot.working_distance_mm if slot else None
        # v7.18 — carried through so the DOF diffraction term is right for a
        # non-dry objective, and so a body-vs-spec WD disagreement stays visible
        # instead of being resolved silently one layer down.
        extra = dict(
            immersion_n=(slot.immersion_n if slot else 1.0),
            optics_source=(slot.optics_source if slot else ""),
            optics_conflict=(slot.optics_conflict if slot else ""))

        if not name:
            out.append(LadderRung(
                turret_position=pos, product_code=code, magnification=mag,
                numerical_aperture=na, working_distance_mm=wd, **extra,
                calibrated=False,
                why_not=(f"Nosepiece position {pos} has no name. Assign one on "
                         f"Hardware Setup → Microscope — '↓ Read from "
                         f"microscope' fills them in from the body.")))
            continue

        cal = None
        try:
            cal = objective_store.get_calibration(camera_name, name)
        except Exception:
            cal = None
        upp = um_per_px_at_resolution(cal, live_w)
        if upp is None:
            out.append(LadderRung(
                turret_position=pos, objective_name=name, product_code=code,
                magnification=mag, numerical_aperture=na,
                working_distance_mm=wd, **extra, calibrated=False,
                why_not=(f"Nosepiece position {pos} is named '{name}', but there "
                         f"is no µm/px calibration for '{name}' on camera "
                         f"{camera_name or '(unknown)'}. Run Hardware Setup → "
                         f"Cameras → objective calibration for it, or drop it "
                         f"from the ladder — without a scale the focus step and "
                         f"the ROI cannot be sized.")))
            continue

        res = cal.get("resolution")
        res_t = None
        if res:
            try:
                res_t = (int(res[0]), int(res[1]))
            except (TypeError, ValueError, IndexError):
                res_t = None
        rot = cal.get("rotation_deg")
        out.append(LadderRung(
            turret_position=pos, objective_name=name, product_code=code,
            um_per_px=upp, um_per_px_resolution=res_t,
            rotation_deg=(None if rot is None else float(rot)),
            magnification=mag, numerical_aperture=na, working_distance_mm=wd,
            **extra, calibrated=True))
    return tuple(out)


def ladder_gate(rungs, *, current_objective_name: str | None = None,
                live_turret_position: int | None = None,
                aliases: dict | None = None) -> tuple[bool, str]:
    """Is this ladder fit to drive a survey? ``(ok, why)``.

    Also performs the first comparison this codebase has ever made between the
    two independent records of "which objective" — the app's
    ``current_objective_name`` and the body's live turret position. They can
    disagree, and every µm/px in a run comes from whichever one wins.

    ⚠ v7.18: that comparison was an exact, case-sensitive ``!=``, so on a rig
    labelling nosepiece 1 "4X" while the app declared "4x" it refused a survey
    over two names that mean the same objective — and told the operator to "fix
    one of them" when nothing was wrong. It now compares normalized, and honours
    an operator-declared alias, so only a REAL disagreement refuses.
    """
    rungs = tuple(rungs or ())
    if not rungs:
        return (False, "Pick at least one objective for the survey.")
    for r in rungs:
        if not r.calibrated:
            return (False, r.why_not or f"position {r.turret_position} is "
                                        f"not usable")
    if current_objective_name and live_turret_position:
        live = next((r for r in rungs
                     if r.turret_position == int(live_turret_position)), None)
        if live is not None and live.objective_name:
            declared = normalize_optic_name(current_objective_name)
            fitted = normalize_optic_name(live.objective_name)
            alias = normalize_optic_name(
                (aliases or {}).get(str(current_objective_name), ""))
            if declared != fitted and (not alias or alias != fitted):
                return (False,
                        f"The app thinks the objective is "
                        f"'{current_objective_name}', but nosepiece position "
                        f"{live_turret_position} is '{live.objective_name}'. Fix "
                        f"one of them before leveling — every µm/px in this run "
                        f"comes from that name.")
    mags = [r.magnification for r in rungs if r.magnification]
    if len(mags) >= 2 and any(b < a for a, b in zip(mags, mags[1:])):
        return (False,
                "The ladder runs from higher to lower magnification. The survey "
                "coarse-focuses on the widest objective first, so order it with "
                "magnification increasing.")
    return (True, "")


def parfocal_offsets_um(per_objective_peaks, reference: str | None = None
                        ) -> tuple[dict, dict, str]:
    """Median focus offset of each objective from a reference. ``(offsets, spread, ref)``.

    ``per_objective_peaks`` is ``{objective_name: [best_focus_um per site]}``.
    Parfocality is the CONSTANT difference between objectives' best-focus
    positions, so the median across sites is the estimate and the spread is a
    free quality check — a large spread means drift, or that the sites were not
    measuring the same thing.
    """
    names = [k for k, v in (per_objective_peaks or {}).items() if v]
    if not names:
        return ({}, {}, "")
    ref = reference if reference in names else names[0]
    ref_vals = list(per_objective_peaks[ref])
    n = min(len(ref_vals), *(len(per_objective_peaks[k]) for k in names))
    if n < 1:
        return ({}, {}, ref)

    def _median(xs):
        s = sorted(xs)
        m = len(s) // 2
        return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])

    offsets, spread = {}, {}
    for k in names:
        diffs = [float(per_objective_peaks[k][i]) - float(ref_vals[i])
                 for i in range(n)]
        offsets[k] = _median(diffs)
        spread[k] = (max(diffs) - min(diffs)) if len(diffs) > 1 else 0.0
    return (offsets, spread, ref)
