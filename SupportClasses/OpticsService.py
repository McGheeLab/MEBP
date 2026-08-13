"""
OpticsService.py — "make sure optic X is in the light path", idempotently.

v7.18. The doing half of the optics work; ``OpticsRegistry`` is the deciding half.
Shaped after this codebase's existing ``StageController.ensure_retracted_to``
precedent: a no-op when already correct, raise-only in the dangerous direction,
never raising an exception at the caller, and reporting an operator-actionable
reason when it refuses.

WHY EVERY STEP IS HERE RATHER THAN IN EACH WORKFLOW
--------------------------------------------------
Before this, exactly one place in the app drove a turret as part of a workflow
(``plate_level_wizard``), and it had to get five separate things right to be safe:
check ``op.error`` and not just ``op.done``, treat a stale drop as fatal rather
than retrying, hold the exclusivity lease, verify the position by read-back, and
put the body back on every exit. Each new caller would have had to rediscover all
five. They live here once.

THE FOUR RULES THAT ARE NOT NEGOTIABLE
--------------------------------------
1. **``done`` is not success.** The controller sets ``done`` and an ``error`` when
   it drops a stale op or refuses one under someone else's lease. A caller that
   waits on ``done`` and proceeds runs everything afterwards through the previous
   optic — and, as ``plate_level_wizard._scope_op`` puts it, "the resulting plane
   looks entirely well-formed".
2. **A stale drop ABORTS, never retries.** A retry re-queues behind exactly the
   same backlog that caused the drop.
3. **The position is VERIFIED by read-back.** ``.error`` cannot see a driver that
   acknowledges and does not move, and ``NikonTiSdkBackend._set_turret`` documents
   this SDK clamping an out-of-range slot and *reporting success*.
4. **``wait_idle()`` is never a substitute for checking ``.error``** — it returns
   True for a drained queue in which every op failed.

Pure: stdlib only. No Qt, no singletons — the controller and stores are passed in.
Blocks on ``op.done.wait``, so it is for a worker thread; a GUI caller must hand
it to one (``gui/widgets/optics_ensure.py``).
"""

from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass
from typing import Callable, Optional

from SupportClasses.OpticsRegistry import (
    FILTER, KINDS, OBJECTIVE, OpticsSnapshot, find_slot, snapshot)

logger = logging.getLogger(__name__)

#: Ceiling on one queued turret/focus op. Matches the wizard's own OP_TIMEOUT_S
#: and comfortably exceeds a real rotation (~2 s) plus the controller's own
#: STALE_OP_S bookkeeping.
DEFAULT_TIMEOUT_S = 20.0

#: How long to wait for the exclusivity lease before refusing. Zero means "take
#: it if free, otherwise say who has it" — queueing behind a ten-minute survey
#: and silently switching at the end of it would be far worse than refusing.
DEFAULT_LEASE_TIMEOUT_S = 0.0

#: Focus read-back tolerance (µm) when confirming a parfocal correction landed.
FOCUS_CONFIRM_TOL_UM = 5.0


class _Cancelled(RuntimeError):
    pass


@dataclass(frozen=True, kw_only=True)
class EnsureResult:
    """What ``ensure_*`` did, or why it would not. Never raises at the caller."""
    ok: bool
    kind: str
    requested: str = ""

    #: True when the optic was ALREADY in the light path: nothing was commanded.
    already: bool = False
    from_position: Optional[int] = None
    to_position: Optional[int] = None
    #: The slot's own name — what the body calls it, not what was asked for.
    resolved_name: str = ""
    #: Which matching tier resolved the name (see OpticsRegistry.HOW_*).
    how: str = ""

    #: Focus actually applied after an objective change, and why / why not.
    focus_applied_um: Optional[float] = None
    focus_note: str = ""

    #: No body, or a body that cannot do this — the caller falls back to asking.
    degraded: bool = False
    #: The switch happened against the SIMULATED backend. A simulated switch
    #: reported as real is a fabricated fact, so this reaches the operator.
    simulated: bool = False
    cancelled: bool = False
    why_not: str = ""

    def describe(self) -> str:
        word = "cube" if self.kind == FILTER else "objective"
        if not self.ok:
            return self.why_not or f"could not set the {word}"
        who = self.resolved_name or self.requested
        if self.already:
            head = f"{word} already {who} ({self.to_position})"
        else:
            head = (f"{word} {self.from_position} → {self.to_position} ({who})")
        bits = [head]
        if self.focus_applied_um is not None:
            bits.append(f"focus → {self.focus_applied_um:.0f} µm")
        if self.focus_note:
            bits.append(self.focus_note)
        if self.simulated:
            bits.append("SIMULATED")
        bits.append("verified")
        return " · ".join(bits)


def wait_for_op(op, what: str, timeout_s: float = DEFAULT_TIMEOUT_S) -> str:
    """Submit-and-wait for ONE controller op. ``""`` on success, else a refusal.

    Lifted in behaviour from ``plate_level_wizard.PlateLevelSurveyWorker._scope_op``
    with both of its hard rules intact — see rules 1 and 2 in the module
    docstring. Returns text instead of raising so no exception crosses this
    boundary; the wizard keeps its own ``_Abort`` wrapper around it.
    """
    if op is None:
        return f"the microscope refused {what}"
    done = getattr(op, "done", None)
    if done is not None and not done.wait(timeout_s):
        return (f"{what} did not complete within {timeout_s:.0f} s — the "
                f"microscope may be busy or the driver wedged")
    err = getattr(op, "error", None)
    if err:
        if "stale" in str(err).lower():
            # Rule 2. Retrying re-queues behind the same backlog.
            return (f"{what} was DROPPED by the microscope queue ({err}). "
                    f"Nothing was retried, because a retry would queue behind "
                    f"the same backlog — the run must stop rather than continue "
                    f"through the wrong optic.")
        return f"{what} failed: {err}"
    return ""


class OpticsService:
    """Idempotent "put optic X in the light path" for both turrets.

    ``owner`` is the lease identity. Pass the SAME string a surrounding workflow
    already holds (e.g. ``"plate_level"``) so a nested call re-enters that
    workflow's lease instead of deadlocking against it.
    """

    def __init__(self, controller, config_store, objective_store=None, *,
                 owner: str, timeout_s: float = DEFAULT_TIMEOUT_S,
                 lease_timeout_s: float = DEFAULT_LEASE_TIMEOUT_S):
        self._c = controller
        self._cfg = config_store
        self._objs = objective_store
        self._owner = str(owner)
        self._timeout = float(timeout_s)
        self._lease_timeout = float(lease_timeout_s)

    # ── reading ───────────────────────────────────────────────────────

    def snapshot(self) -> OpticsSnapshot:
        return snapshot(controller=self._c, config_store=self._cfg)

    def current(self, kind: str):
        return self.snapshot().current(kind)

    def current_name(self, kind: str) -> str:
        return self.snapshot().current_name(kind)

    # ── the public verbs ──────────────────────────────────────────────

    def ensure_filter(self, name: str, *,
                      cancelled: Optional[Callable[[], bool]] = None
                      ) -> EnsureResult:
        """Put the cube named ``name`` in the light path.

        Needs no collision guard: rotating the cassette moves no objective and
        changes no height. That is exactly why filter automation is the safe half
        of this feature and lands first.
        """
        return self._ensure(FILTER, name, cancelled=cancelled)

    def ensure_objective(self, name: str, *,
                         camera_identity: Optional[str] = None,
                         glass_focus_um: Optional[float] = None,
                         needle_retracted: Optional[bool] = None,
                         apply_parfocal: Optional[bool] = None,
                         cancelled: Optional[Callable[[], bool]] = None
                         ) -> EnsureResult:
        """Rotate the nosepiece to ``name``, retreating the focus FIRST.

        ⚠ Refuses unless it can prove the rotation is safe — see
        ``FocusSweepPlanner.plan_turret_change``. ``glass_focus_um`` is the focus
        height at which the plate glass is sharp; without it the target
        objective's clearance cannot be computed and this refuses rather than
        rotating hopefully.

        ``needle_retracted=False`` refuses outright. This service never commands
        the STAGE — travel stays with ``safe_travel_to``/``ensure_retracted_to``,
        and it never touches ``set_print_floor_active`` (the plate-level
        precedent: arming a refcount it cannot need risks an unbalanced decrement
        against a concurrent print).
        """
        return self._ensure(
            OBJECTIVE, name, cancelled=cancelled,
            camera_identity=camera_identity, glass_focus_um=glass_focus_um,
            needle_retracted=needle_retracted, apply_parfocal=apply_parfocal)

    def ensure_position(self, kind: str, position: int, **kw) -> EnsureResult:
        """Drive one turret to a POSITION rather than a name.

        For the manual panels, whose controls are position-based. Goes through the
        same lease / read-back / collision path so there is not a second, weaker
        implementation of a turret move in the app.
        """
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {KINDS!r}, got {kind!r}")
        snap = self.snapshot()
        slot = snap.slot_at(kind, position)
        name = (slot.label if slot else "")
        if not name:
            # Fall back to the raw position: the panel is allowed to drive an
            # unnamed slot, which is how an operator LOOKS at what is in one.
            return self._ensure(kind, "", explicit_position=int(position), **kw)
        return self._ensure(kind, name, explicit_position=int(position), **kw)

    # ── entry / restore ───────────────────────────────────────────────

    def capture_entry(self) -> dict:
        """Snapshot what to put back. Cheap, and never raises."""
        snap = self.snapshot()
        focus = None
        try:
            focus = getattr(self._c.state(), "focus_um", None)
        except Exception:
            focus = None
        return {"objective": snap.objective_position,
                "filter": snap.filter_position, "focus_um": focus}

    def restore(self, entry: dict) -> str:
        """Put the body back. ``""`` when clean, else what it could NOT restore.

        ⚠ ORDER: focus to the entry height BEFORE the turret rotates back. That
        height was legal for the entry objective (it is where the entry objective
        was), so it is the one position guaranteed safe for the optic being
        rotated in. The reverse order can rotate a short-working-distance
        objective into the plate — see ``plan_turret_change``.

        Exactly ONE focus command: re-issuing it after the rotation would add
        nothing (rotation does not move Z, and the clamp is the same either way).
        The post-rotation step is a read-back CHECK.
        """
        entry = dict(entry or {})
        problems: list[str] = []
        focus = entry.get("focus_um")

        if focus is not None:
            why = wait_for_op(self._c.set_focus_um(float(focus)),
                              "returning the focus to its start height",
                              self._timeout)
            if why:
                problems.append(why)
        for kind, setter in ((OBJECTIVE, "set_objective"), (FILTER, "set_filter")):
            pos = entry.get(kind)
            if not pos:
                continue
            try:
                why = wait_for_op(getattr(self._c, setter)(int(pos)),
                                  f"returning the {kind} to position {pos}",
                                  self._timeout)
            except Exception as exc:      # a wedged driver must not raise here
                why = f"returning the {kind} to position {pos} failed: {exc}"
            if why:
                problems.append(why)
        if focus is not None:
            back = self._focus_now_um()
            if back is not None and abs(back - float(focus)) > FOCUS_CONFIRM_TOL_UM:
                problems.append(
                    f"the focus drive is at {back:.0f} µm and did not return to "
                    f"{float(focus):.0f} µm")
        return "; ".join(problems)

    # ── internals ─────────────────────────────────────────────────────

    def _focus_now_um(self) -> Optional[float]:
        try:
            return getattr(self._c.state(), "focus_um", None)
        except Exception:
            return None

    def _refresh(self) -> str:
        """Re-read the body. The cached state is up to ~1 s stale, and a
        hand-rotated turret is precisely the case this service exists for."""
        try:
            self._c.wait_idle(self._timeout)
        except Exception:
            pass
        try:
            return wait_for_op(self._c.refresh(), "reading the microscope state",
                              self._timeout)
        except Exception as exc:
            return f"reading the microscope state failed: {exc}"

    def _ensure(self, kind: str, name: str, *,
                cancelled: Optional[Callable[[], bool]] = None,
                explicit_position: Optional[int] = None,
                camera_identity: Optional[str] = None,
                glass_focus_um: Optional[float] = None,
                needle_retracted: Optional[bool] = None,
                apply_parfocal: Optional[bool] = None) -> EnsureResult:
        word = "cube" if kind == FILTER else "objective"

        def _no(why, **kw) -> EnsureResult:
            return EnsureResult(ok=False, kind=kind, requested=name,
                               why_not=why, **kw)

        def _stop() -> bool:
            try:
                return bool(cancelled and cancelled())
            except Exception:
                return False

        if _stop():
            return _no("cancelled before anything was commanded", cancelled=True)

        snap = self.snapshot()
        if not snap.connected:
            return _no(
                f"The microscope is not connected, so the {word} cannot be "
                f"changed automatically. Set it by hand, or connect the body on "
                f"Hardware Setup → Microscope.", degraded=True)
        if not snap.slots(kind):
            return _no(f"The microscope reports no {word} turret.", degraded=True)

        # Resolve the name (or accept the caller's explicit position).
        if explicit_position is not None:
            target = int(explicit_position)
            slot = snap.slot_at(kind, target)
            resolved, how = ((slot.label if slot else ""), "position")
            legal = [s.position for s in snap.slots(kind)]
            if target not in legal:
                return _no(f"{word.capitalize()} position {target} does not "
                           f"exist — this turret has {legal}.")
        else:
            aliases = {}
            try:
                aliases = self._cfg.optic_aliases(kind)
            except Exception:
                aliases = {}
            m = find_slot(snap.slots(kind), name, aliases=aliases, kind=kind)
            if not m.ok:
                return _no(m.why_not)
            target, resolved, how = m.position, m.resolved_name, m.how

        # Freshness BEFORE the no-op check, or a hand-rotated turret is missed.
        why = self._refresh()
        if why:
            return _no(why)
        snap = self.snapshot()
        here = snap.position(kind)

        if here is not None and int(here) == int(target):
            return EnsureResult(
                ok=True, kind=kind, requested=name, already=True,
                from_position=int(here), to_position=int(target),
                resolved_name=resolved, how=how, simulated=snap.simulated)

        # From here a MOVE will happen, so the gates apply.
        if kind == OBJECTIVE and needle_retracted is False:
            return _no(
                "The needle is not retracted, so the objective will not be "
                "rotated. Retract to the safe travel height first — a rotation "
                "cannot be aborted once it starts.")

        if _stop():
            return _no("cancelled before anything was commanded", cancelled=True)

        # ⚠ RELEASE ONLY A LEASE THIS CALL ACTUALLY TOOK. `try_acquire` is
        # re-entrant for the same thread and returns True, while `release`
        # unconditionally clears it — so a service nested inside a workflow that
        # already holds the lease (same owner string, by design, so it can nest)
        # would hand the body back mid-run and let any other surface in.
        release_at_end = False
        try:
            try:
                held_by = self._c.lease_owner()
            except Exception:
                held_by = None
            already_mine = (held_by is not None and held_by == self._owner)
            try:
                got = bool(self._c.try_acquire(self._owner, self._lease_timeout))
            except Exception as exc:
                return _no(f"could not reserve the microscope: {exc}")
            if not got:
                who = held_by or "another operation"
                return _no(
                    f"The microscope is reserved by {who}, so the {word} was not "
                    f"changed. Wait for it to finish, or stop it.")
            release_at_end = not already_mine

            plan = None
            if kind == OBJECTIVE:
                plan, why = self._plan_rotation(
                    snap, here, target, glass_focus_um=glass_focus_um,
                    camera_identity=camera_identity,
                    apply_parfocal=apply_parfocal)
                if why:
                    return _no(why)
                if plan is not None and plan.retreat_focus_um is not None:
                    if _stop():
                        return _no("cancelled before the focus moved",
                                   cancelled=True)
                    why = wait_for_op(
                        self._c.set_focus_um(float(plan.retreat_focus_um)),
                        f"retreating the focus to {plan.retreat_focus_um:.0f} µm "
                        f"before rotating to {resolved or target}",
                        self._timeout)
                    if why:
                        return _no(why)

            if _stop():
                return _no("cancelled before the turret moved", cancelled=True)

            setter = "set_filter" if kind == FILTER else "set_objective"
            why = wait_for_op(getattr(self._c, setter)(int(target)),
                              f"switching the {word} to {resolved or target}",
                              self._timeout)
            if why:
                return _no(why)

            # Rule 3 — VERIFY. .error cannot see an ack-without-motion.
            why = self._refresh()
            if why:
                return _no(why)
            after = self.snapshot()
            landed = after.position(kind)
            if landed is None or int(landed) != int(target):
                return _no(
                    f"The {word} did not move: the body reports position "
                    f"{landed} after being told to go to {target}. Nothing "
                    f"after this point can be trusted to be through "
                    f"{resolved or target}.")

            focus_um, focus_note = None, ""
            if kind == OBJECTIVE and plan is not None:
                focus_um, focus_note = self._settle_focus(plan)

            return EnsureResult(
                ok=True, kind=kind, requested=name,
                from_position=(int(here) if here is not None else None),
                to_position=int(target), resolved_name=resolved, how=how,
                focus_applied_um=focus_um, focus_note=focus_note,
                simulated=after.simulated)
        finally:
            if release_at_end:
                try:
                    self._c.release(self._owner)
                except Exception:
                    pass

    def _plan_rotation(self, snap, here, target, *, glass_focus_um,
                       camera_identity, apply_parfocal):
        """``(TurretChangePlan | None, refusal)`` for one nosepiece rotation."""
        from SupportClasses.FocusSweepPlanner import plan_turret_change
        from SupportClasses.ObjectiveOptics import ObjectiveOptics

        to_slot = snap.slot_at(OBJECTIVE, target)
        from_slot = snap.slot_at(OBJECTIVE, here) if here else None
        if to_slot is None:
            return (None, f"nosepiece position {target} is not reported by the body")

        def _optics(slot):
            if slot is None:
                return None
            return ObjectiveOptics(
                label=slot.label, position=slot.position,
                magnification=slot.magnification,
                numerical_aperture=slot.numerical_aperture,
                working_distance_mm=slot.working_distance_mm,
                # v7.18: carried through so an oil/water objective's depth of
                # field is not computed as if it were dry.
                immersion_n=slot.immersion_n)

        delta = None
        note = ""
        if apply_parfocal is None:
            try:
                apply_parfocal = bool(self._cfg.parfocal_auto_apply())
            except Exception:
                apply_parfocal = False
        if apply_parfocal and camera_identity:
            delta, note = self._parfocal_delta(
                camera_identity, from_slot, to_slot)

        focus_now = snap.focus_um
        if focus_now is None:
            return (None,
                    f"Cannot rotate to {to_slot.label} safely: the focus "
                    f"position could not be read from the body.")

        up = True
        limits = None
        try:
            up = bool(self._cfg.focus_up_is_positive())
            limits = self._cfg.focus_soft_limits_um()
        except Exception:
            pass

        plan, why = plan_turret_change(
            to_optics=_optics(to_slot), from_optics=_optics(from_slot),
            current_focus_um=float(focus_now), glass_focus_um=glass_focus_um,
            focus_up_is_positive=up, parfocal_delta_um=delta,
            soft_limits_um=limits)
        if why:
            return (None, why)
        if note and plan is not None:
            plan = dataclasses.replace(plan, reasons=plan.reasons + (note,))
        return (plan, "")

    def _parfocal_delta(self, camera_identity, from_slot, to_slot):
        """``(delta_um, note)`` — the focus shift for this objective change.

        ⚠ Stored offsets are RELATIVE TO A REFERENCE objective, so the shift is
        ``offset[to] − offset[from]``, not ``offset[to]``. Using the absolute
        value is a full-magnitude focus error that still looks plausible.
        """
        try:
            to_off, why_to = self._cfg.parfocal_offset_um(
                camera_identity, to_slot.label, product_code=to_slot.code or None)
        except Exception as exc:
            return (None, f"no parfocal correction applied ({exc})")
        if to_off is None:
            return (None, f"no parfocal correction applied — {why_to}")
        if from_slot is None or not from_slot.label:
            return (None, "no parfocal correction applied — the objective being "
                          "rotated out is unknown, and the stored offsets are "
                          "relative to a reference")
        try:
            from_off, why_from = self._cfg.parfocal_offset_um(
                camera_identity, from_slot.label,
                product_code=from_slot.code or None)
        except Exception as exc:
            return (None, f"no parfocal correction applied ({exc})")
        if from_off is None:
            return (None, f"no parfocal correction applied — {why_from}")
        return (float(to_off) - float(from_off), "")

    def _settle_focus(self, plan) -> tuple:
        """Apply the plan's post-rotation focus, reporting where it LANDED.

        ``set_focus_um`` clamps to the soft limits BEFORE queueing, so a move can
        arrive silently short; the read-back is what makes the number honest.
        """
        note = "; ".join(plan.reasons) if plan.reasons else ""
        if plan.post_focus_um is None:
            return (None, note)
        why = wait_for_op(self._c.set_focus_um(float(plan.post_focus_um)),
                          "applying the parfocal focus offset", self._timeout)
        if why:
            # The optic IS correct; only the focus correction failed. That is a
            # note, not a failure of the switch.
            return (None, (note + "; " if note else "") + why)
        back = self._focus_now_um()
        if back is None:
            return (plan.post_focus_um, note)
        if abs(back - float(plan.post_focus_um)) > FOCUS_CONFIRM_TOL_UM:
            extra = (f"focus landed at {back:.0f} µm, not the requested "
                     f"{plan.post_focus_um:.0f} µm (clamped by the soft limits?)")
            return (back, (note + "; " if note else "") + extra)
        return (back, note)
