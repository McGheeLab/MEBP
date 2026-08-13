"""
OpticsRegistry.py — ONE join for "which optic is in the light path".

v7.18. Three independent records of "which objective" have always existed side by
side (the body's live turret position, ``MicroscopeConfigStore``'s per-slot labels,
and ``CameraConfig.current_objective_name``), plus two vocabularies for "which
filter cube" (``MicroscopeConfigStore.filter_cubes`` and
``FluorescenceMosaicStore.CHANNELS``). Nothing reconciled them, and there was no
reverse lookup at all — no code anywhere could answer *"which slot does the name
'FITC' mean?"*, which is exactly what a workflow needs before it can switch.

This module is that join, for BOTH turrets, and it is deliberately:

* **stdlib only** — no Qt, no repo imports, no singletons. Everything is passed in
  (``scope_state``, ``config_store``), so it can be imported from anywhere,
  including from ``ObjectiveCalibration``, without an import cycle.
* **read-only** — nothing here commands hardware or writes a store. Deciding is
  separate from doing; ``OpticsService`` does the doing.

⚠ **THE MATCHER REFUSES RATHER THAN GUESSES.** ``find_slot`` walks ordered tiers
and each must resolve to exactly ONE slot, else it is ambiguous and refuses. There
is deliberately no substring, prefix, edit-distance, or magnification-digit
matching, and no fall back to a hardcoded channel→slot table.
``NikonTiSdkBackend._set_turret`` already documents this SDK clamping slot 999 → 6
and **reporting success**; a fuzzy name match is that same failure one layer up,
and a TxRed image filed as an mCherry channel is a result nothing downstream can
detect. On this rig that is not hypothetical: the cassette holds
DAPI/FITC/**TxRed**/Cy5 while the app's channel vocabulary says *mCherry*, so
"mCherry" MUST NOT resolve without the operator saying the two are the same.

What normalization DOES safely fix is letter case. This rig labels nosepiece 1
"4X" while its calibration is stored under "4x", and the lookup was an exact,
case-sensitive ``dict.get`` — so the plate-bed-leveling ladder reported
*"no µm/px calibration for '4X'"* for an objective calibrated the day before.
"""

from __future__ import annotations

import logging
import unicodedata
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

#: The two turrets. Used as ``kind`` throughout so one code path serves both.
OBJECTIVE = "objective"
FILTER = "filter"
KINDS = (OBJECTIVE, FILTER)

#: How a name was matched to a slot, in the order the tiers are tried.
HOW_EXACT = "exact"
HOW_NORMALIZED = "normalized"
HOW_ALIAS = "alias"
HOW_NATIVE = "native"
HOW_NONE = "none"

#: Immersion medium → refractive index between front lens and specimen.
#:
#: Lives HERE, in the dependency-free module, rather than in
#: ``ObjectiveCatalogue`` where it is also needed — otherwise this module would
#: have to import that one, and its whole point is that it imports nothing from
#: the repo so ``ObjectiveCalibration`` and ``MicroscopeConfigStore`` can both
#: import IT without a cycle. ``ObjectiveCatalogue`` re-exports these, so there
#: is exactly one table.
#:
#: The index is not cosmetic: it multiplies the diffraction term of
#: ``ObjectiveOptics.depth_of_field_um``, so an oil objective left at 1.0 has its
#: depth of field — and therefore its focus step — wrong by ~1.5x.
IMMERSION_N = {
    "air": 1.0,
    "water": 1.333,
    "glycerol": 1.47,
    "oil": 1.515,
    "silicone": 1.406,
}
DEFAULT_IMMERSION = "air"


def clean_immersion(value) -> str:
    """Coerce to a known immersion medium, defaulting to dry."""
    key = str(value or "").strip().lower()
    return key if key in IMMERSION_N else DEFAULT_IMMERSION


def immersion_n(medium) -> float:
    """Refractive index for a medium name. Unknown → dry (1.0)."""
    return IMMERSION_N.get(clean_immersion(medium), 1.0)


def normalize_optic_name(name) -> str:
    """Fold a slot/objective/cube name for comparison. ``""`` when empty.

    NFKC (so a full-width or composed character compares equal to its plain
    form), casefold (stronger than ``.lower()``), and collapse all whitespace —
    the three differences that are safely *typography*, not identity. "4X" and
    "4x" are the same objective; "TxRed" and "mCherry" are not, and no amount of
    normalization will make them so.
    """
    if name is None:
        return ""
    s = unicodedata.normalize("NFKC", str(name))
    return " ".join(s.casefold().split())


@dataclass(frozen=True, kw_only=True)
class OpticSlot:
    """One turret position, fully resolved. ``kind`` is OBJECTIVE or FILTER."""
    kind: str
    position: int

    #: Operator-assigned label from MicroscopeConfigStore. "" when unnamed.
    name: str = ""
    #: The body's OWN label for what is fitted here. "" when it reports none.
    native_name: str = ""
    #: The body reports something fitted here. An empty Nikon cassette slot
    #: comes back as Code 0 with a "-----" placeholder, i.e. present=False.
    present: bool = False
    code: str = ""

    # Objective-only optics (None on a filter slot, and None when the body
    # cannot report them — never fabricated).
    magnification: Optional[float] = None
    numerical_aperture: Optional[float] = None
    working_distance_mm: Optional[float] = None
    #: Refractive index between front lens and specimen (1.0 = dry). Needed by
    #: the depth-of-field diffraction term; an oil objective left at 1.0 has its
    #: DOF wrong by that factor.
    immersion_n: float = 1.0
    #: Where the optics above came from: "body" (the nosepiece's own report),
    #: "catalogue"/"datasheet"/"measured" (an operator-assigned spec), or "".
    optics_source: str = ""
    #: Set when the stored spec and the body disagree about working distance.
    #: The SHORTER value is kept — see resolve_slots.
    optics_conflict: str = ""

    # Filter-only optics, from MicroscopeConfigStore.filter_optics_for().
    excitation_nm: Optional[float] = None
    emission_nm: Optional[float] = None

    @property
    def label(self) -> str:
        """The best name we have: the operator's, else the body's, else "" ."""
        return self.name or self.native_name

    @property
    def usable(self) -> bool:
        """Can this slot be a match target at all?

        Requires a name to match against AND that the body has not told us the
        slot is empty. Driving an empty slot into the light path would store a
        black frame set as a legitimate channel.
        """
        return bool(self.label) and self.present

    def describe(self) -> str:
        who = self.label or "(unnamed)"
        return f"{who} ({self.position})"


@dataclass(frozen=True, kw_only=True)
class SlotMatch:
    """The result of a name → slot lookup. ``position`` is None on a refusal."""
    kind: str
    requested: str = ""
    position: Optional[int] = None
    #: The slot's own name, NOT the requested string — what the body calls it.
    resolved_name: str = ""
    how: str = HOW_NONE
    why_not: str = ""

    @property
    def ok(self) -> bool:
        return self.position is not None

    def describe(self) -> str:
        if not self.ok:
            return self.why_not or f"{self.requested!r} did not resolve"
        via = "" if self.how == HOW_EXACT else f", via {self.how}"
        return f"{self.resolved_name} ({self.position}){via}"


@dataclass(frozen=True, kw_only=True)
class OpticsSnapshot:
    """Everything a caller needs to decide, taken at one instant."""
    connected: bool = False
    backend: str = "none"
    simulated: bool = False
    busy: bool = False
    error: Optional[str] = None
    lease_owner: Optional[str] = None

    objective_position: Optional[int] = None
    filter_position: Optional[int] = None
    objectives: tuple = ()
    filters: tuple = ()

    #: Live focus height (µm). Part of the optics state because a turret change
    #: is only safe at certain focus heights — see FocusSweepPlanner
    #: .plan_turret_change. None when the body reports no focus drive.
    focus_um: Optional[float] = None

    def slots(self, kind: str) -> tuple:
        return self.objectives if kind == OBJECTIVE else self.filters

    def position(self, kind: str) -> Optional[int]:
        return (self.objective_position if kind == OBJECTIVE
                else self.filter_position)

    def slot_at(self, kind: str, position) -> Optional[OpticSlot]:
        if position is None:
            return None
        for s in self.slots(kind):
            if int(s.position) == int(position):
                return s
        return None

    def current(self, kind: str) -> Optional[OpticSlot]:
        """The slot in the light path right now, or None if unknown."""
        return self.slot_at(kind, self.position(kind))

    def current_name(self, kind: str) -> str:
        s = self.current(kind)
        return s.label if s else ""


def optic_at(state, position, kind: str = OBJECTIVE):
    """The ``MountedOptic`` the body reports at ``position``, or None.

    Promoted from ``ObjectiveLadder._optic_at`` (which now delegates here) and
    generalised to either turret, so there is one implementation of the walk.
    """
    if position is None:
        return None
    attr = "mounted_objectives" if kind == OBJECTIVE else "mounted_filters"
    for o in (getattr(state, attr, ()) or ()):
        try:
            if int(getattr(o, "position", 0) or 0) == int(position):
                return o
        except (TypeError, ValueError):
            continue
    return None


def _f(value) -> Optional[float]:
    """Coerce to float, or None. Never raises, never fabricates a default."""
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v


def _native_names(state, kind: str) -> dict:
    """``{position: name}`` from the body's flat ``native_*_names`` tuple.

    That tuple is positional and 1-based by convention, unlike ``mounted_*``
    which carries explicit positions.
    """
    attr = ("native_objective_names" if kind == OBJECTIVE
            else "native_filter_names")
    out: dict[int, str] = {}
    for i, n in enumerate((getattr(state, attr, ()) or ()), start=1):
        s = str(n or "").strip()
        # The Nikon SDK reports an empty cassette slot as a "-----" placeholder.
        if s and set(s) != {"-"}:
            out[i] = s
    return out


def resolve_slots(*, scope_state, config_store, kind: str) -> tuple:
    """Resolve EVERY position of one turret into an :class:`OpticSlot`.

    Joins the three records that have never been joined: the body's live
    ``mounted_*`` / ``native_*_names``, the operator's per-slot labels from
    ``MicroscopeConfigStore``, and (filters only) the stored per-cube
    excitation/emission wavelengths.

    A missing or malformed source degrades to "unknown" for that field rather
    than raising — this feeds gates and UI that must not crash on a body that
    answered oddly.
    """
    if kind not in KINDS:
        raise ValueError(f"kind must be one of {KINDS!r}, got {kind!r}")

    is_obj = kind == OBJECTIVE

    try:
        labels = dict((config_store.objective_labels() if is_obj
                       else config_store.filter_labels()) or {})
    except Exception:
        labels = {}

    try:
        count = int(getattr(scope_state,
                            "objective_count" if is_obj else "filter_count",
                            0) or 0)
    except (TypeError, ValueError):
        count = 0
    if count <= 0:
        # No body (or a body with no such turret) — fall back to however many
        # slots the operator has configured, so the setup UI still has rows.
        try:
            count = int((config_store.objective_slots() if is_obj
                         else config_store.filter_slots()) or 0)
        except Exception:
            count = 0

    natives = _native_names(scope_state, kind)

    optics: dict = {}
    specs: dict = {}
    if not is_obj:
        try:
            optics = dict(config_store.filter_optics() or {})
        except Exception:
            optics = {}
    else:
        try:
            specs = dict(config_store.objective_specs() or {})
        except Exception:
            specs = {}

    out: list[OpticSlot] = []
    for pos in range(1, max(0, count) + 1):
        name = str(labels.get(pos, "") or "").strip()
        mo = optic_at(scope_state, pos, kind)

        native = str(getattr(mo, "label", "") or "").strip() if mo else ""
        if not native or set(native) == {"-"}:
            native = natives.get(pos, "")

        # `present` is the body's claim. With no body at all we cannot know, and
        # "unknown" must not read as "empty" — an operator-named slot on a
        # disconnected rig stays a legitimate target for the UI to list, while
        # `OpticsService` refuses on `connected` long before it looks at a slot.
        if mo is not None:
            present = bool(getattr(mo, "present", False))
        else:
            present = bool(native) or (not natives and bool(name))

        ex = em = None
        if not is_obj:
            key = name or native
            blk = optics.get(key) if key else None
            if isinstance(blk, dict):
                ex = _f(blk.get("excitation_nm"))
                em = _f(blk.get("emission_nm"))

        mag = _f(getattr(mo, "magnification", None)) if (mo and is_obj) else None
        na = _f(getattr(mo, "numerical_aperture", None)) if (mo and is_obj) else None
        wd = _f(getattr(mo, "working_distance_mm", None)) if (mo and is_obj) else None
        n_med, source, conflict = 1.0, ("body" if (mo and is_obj and na) else ""), ""

        if is_obj:
            mag, na, wd, n_med, source, conflict = _merge_objective_spec(
                specs, name or native, mag, na, wd)

        out.append(OpticSlot(
            kind=kind,
            position=pos,
            name=name,
            native_name=native,
            present=present,
            code=str(getattr(mo, "code", "") or "") if mo else "",
            magnification=mag,
            numerical_aperture=na,
            working_distance_mm=wd,
            immersion_n=n_med,
            optics_source=source,
            optics_conflict=conflict,
            excitation_nm=ex,
            emission_nm=em,
        ))
    return tuple(out)


def _spec_for(specs: dict, name: str) -> dict:
    """The stored spec for ``name``, matched normalized. ``{}`` when absent."""
    want = normalize_optic_name(name)
    if not want:
        return {}
    for key, entry in (specs or {}).items():
        if normalize_optic_name(key) == want and isinstance(entry, dict):
            return entry
    return {}


def _merge_objective_spec(specs: dict, name: str, body_mag, body_na, body_wd):
    """Combine the body's report with an operator-assigned spec.

    Returns ``(mag, na, wd, immersion_n, source, conflict)``.

    **The stored spec wins on NA**, because the body only knows the product code
    programmed into its nosepiece — which is routinely blank, and can name a
    different variant of the same nominal objective. An operator who picked the
    part off a catalogue (or read the engraving) knows better, and NA sizes every
    focus step through the depth of field.

    ⚠ **Working distance takes the SHORTER of the two when they disagree**, which
    is NOT the same rule. WD is the collision bound: ``WD_SWEEP_FRACTION`` of it
    is how far the focus may travel and whether a rotation is allowed at all, so
    the fail-safe direction is the smaller number regardless of which source is
    more trustworthy. The disagreement is reported rather than swallowed, because
    the operator is the only one who can settle which part is actually fitted.

    The ONE exception, and it requires a deliberate act: a spec whose provenance
    is ``measured`` wins outright. That is the strongest claim the provenance
    ladder can carry — the operator measured this objective's clearance on THIS
    instrument — and without an override a body reporting a wrong product code
    would permanently cap a correct objective's travel with no way out. Picking a
    catalogue entry yields ``nominal`` or ``datasheet``, never ``measured``, so
    the fail-safe stays the default.
    """
    spec = _spec_for(specs, name)
    if not spec:
        return (body_mag, body_na, body_wd, 1.0,
                ("body" if body_na else ""), "")

    spec_na = _f(spec.get("numerical_aperture"))
    spec_wd = _f(spec.get("working_distance_mm"))
    spec_mag = _f(spec.get("magnification"))
    n_med = immersion_n(spec.get("immersion"))
    source = str(spec.get("provenance") or "catalogue")

    na = spec_na if spec_na else body_na
    mag = spec_mag if spec_mag else body_mag

    conflict = ""
    if spec_wd and body_wd:
        if abs(spec_wd - body_wd) <= 0.05:     # 50 µm — below any real variance
            wd = min(spec_wd, body_wd)
        elif source == "measured":
            wd = spec_wd
            conflict = (
                f"{name or 'this objective'}: the microscope reports a "
                f"{body_wd:g} mm working distance, but {spec_wd:g} mm was "
                f"MEASURED on this instrument, so the measurement is used.")
        else:
            wd = min(spec_wd, body_wd)
            conflict = (
                f"{name or 'this objective'}: the microscope reports a "
                f"{body_wd:g} mm working distance but the assigned optics say "
                f"{spec_wd:g} mm. Using the shorter ({wd:g} mm) so a rotation "
                f"cannot be authorised on the more generous of two numbers. "
                f"Check which objective is really fitted; if the "
                f"{spec_wd:g} mm figure is right, record it as measured to make "
                f"it win.")
    else:
        wd = spec_wd or body_wd

    return (mag, na, wd, n_med, source, conflict)


def resolve_filters(*, scope_state, config_store) -> tuple:
    """``resolve_slots`` for the filter cassette. Convenience for readability."""
    return resolve_slots(scope_state=scope_state, config_store=config_store,
                         kind=FILTER)


def resolve_objectives(*, scope_state, config_store) -> tuple:
    """``resolve_slots`` for the nosepiece. Convenience for readability."""
    return resolve_slots(scope_state=scope_state, config_store=config_store,
                         kind=OBJECTIVE)


def _kind_word(kind: str) -> str:
    return "filter cube" if kind == FILTER else "objective"


def _where_to_fix() -> str:
    return ("Assign it on Hardware Setup → Microscope — '↓ Read from "
            "microscope' fills the names in from the body.")


def _inventory(slots) -> str:
    """Operator-facing list of what IS configured, for a refusal message."""
    named = [s.describe() for s in slots if s.usable]
    unnamed = [str(s.position) for s in slots if not s.usable]
    parts = []
    parts.append("holds " + ", ".join(named) if named else "has nothing named")
    if unnamed:
        word = "slot" if len(unnamed) == 1 else "slots"
        parts.append(f"{word} {', '.join(unnamed)} "
                     f"{'is' if len(unnamed) == 1 else 'are'} unnamed or empty")
    return "; ".join(parts)


def _unique(cands, kind: str, requested: str, slots, tier: str
            ) -> Optional[SlotMatch]:
    """One candidate → a match. Several → an ambiguity refusal. None → None.

    Ambiguity REFUSES and names both positions rather than picking. If a rig
    genuinely has two slots the operator called the same thing, guessing which
    one they meant is worse than saying so.
    """
    if not cands:
        return None
    if len(cands) == 1:
        s = cands[0]
        return SlotMatch(kind=kind, requested=requested, position=s.position,
                         resolved_name=s.label, how=tier)
    where = " and ".join(s.describe() for s in cands)
    return SlotMatch(
        kind=kind, requested=requested,
        why_not=(f"{_kind_word(kind).capitalize()} {requested!r} is ambiguous — "
                 f"it matches {where}. Rename one of them on Hardware Setup → "
                 f"Microscope so the name means exactly one slot."))


def find_slot(slots, wanted, *, aliases=None, kind: Optional[str] = None
              ) -> SlotMatch:
    """Which slot does the name ``wanted`` mean? The missing reverse lookup.

    Tiers, in order, each of which must resolve to exactly ONE usable slot:

    ==============  ==========================================================
    ``exact``       the operator's label, character for character
    ``normalized``  case/whitespace-folded — this is what fixes "4X" vs "4x"
    ``alias``       an operator-configured equivalence, e.g. mCherry → TxRed
    ``native``      the body's own label, for a slot the operator never named
    ==============  ==========================================================

    Anything else REFUSES. See the module docstring for why there is no fuzzy
    tier. ``aliases`` maps a requested name to a slot name (both compared
    normalized) and comes from ``MicroscopeConfigStore.optic_aliases(kind)``.

    ``kind`` is normally inferred from the slots. Pass it explicitly when the
    sequence may be EMPTY — otherwise a refusal about a missing turret cannot
    know which turret was asked about and would name the wrong one.
    """
    slots = tuple(slots or ())
    requested = str(wanted or "").strip()
    if kind not in KINDS:
        kind = slots[0].kind if slots else OBJECTIVE

    if not requested:
        return SlotMatch(kind=kind, requested=requested,
                         why_not=f"No {_kind_word(kind)} was requested.")
    if not slots:
        return SlotMatch(
            kind=kind, requested=requested,
            why_not=(f"The microscope reports no {_kind_word(kind)} turret, so "
                     f"{requested!r} cannot be resolved to a slot."))

    usable = [s for s in slots if s.usable]

    # Tier 1 — exact operator label.
    m = _unique([s for s in usable if s.name == requested],
                kind, requested, slots, HOW_EXACT)
    if m is not None:
        return m

    # Tier 2 — normalized operator label. Fixes "4X" vs "4x".
    norm = normalize_optic_name(requested)
    m = _unique([s for s in usable
                 if s.name and normalize_optic_name(s.name) == norm],
                kind, requested, slots, HOW_NORMALIZED)
    if m is not None:
        return m

    # Tier 3 — operator-configured alias (mCherry → TxRed). The alias names a
    # SLOT LABEL, so resolve it against the labels, still normalized.
    target = ""
    for a, t in (aliases or {}).items():
        if normalize_optic_name(a) == norm:
            target = normalize_optic_name(t)
            break
    if target:
        m = _unique([s for s in usable
                     if s.name and normalize_optic_name(s.name) == target],
                    kind, requested, slots, HOW_ALIAS)
        if m is not None:
            return m
        return SlotMatch(
            kind=kind, requested=requested,
            why_not=(f"{requested!r} is aliased to a {_kind_word(kind)} that is "
                     f"not in the turret — it {_inventory(slots)}. Fix the "
                     f"alias on Hardware Setup → Microscope."))

    # Tier 4 — the body's own label, for a slot the operator never named.
    m = _unique([s for s in usable
                 if not s.name and s.native_name
                 and normalize_optic_name(s.native_name) == norm],
                kind, requested, slots, HOW_NATIVE)
    if m is not None:
        return m

    # Refuse. Name what IS there and where to fix it — and, when the request
    # only failed because a slot is unnamed/empty, say that instead of implying
    # the turret is bare.
    blocked = [s for s in slots
               if not s.usable
               and normalize_optic_name(s.label) == norm and s.label]
    if blocked:
        s = blocked[0]
        return SlotMatch(
            kind=kind, requested=requested,
            why_not=(f"{_kind_word(kind).capitalize()} {requested!r} is "
                     f"position {s.position}, but the microscope reports "
                     f"nothing fitted there. Fit it, or pick another "
                     f"{_kind_word(kind)}."))
    return SlotMatch(
        kind=kind, requested=requested,
        why_not=(f"No {_kind_word(kind)} named {requested!r} is configured. The "
                 f"turret {_inventory(slots)}. {_where_to_fix()}"))


def snapshot(*, controller, config_store) -> OpticsSnapshot:
    """One consistent view of both turrets, taken from a cached state read.

    Cheap by design — ``controller.state()`` is a cached snapshot, so this issues
    no hardware traffic and is safe to call from a render path. It is therefore
    also up to ~1 s stale: a caller that is about to MOVE something must refresh
    first, which is why ``OpticsService.ensure_*`` does so before its no-op check.
    """
    try:
        st = controller.state()
    except Exception as exc:
        logger.debug(f"OpticsRegistry.snapshot: state() failed: {exc}")
        return OpticsSnapshot(error=f"the microscope state could not be read: {exc}")

    try:
        owner = controller.lease_owner()
    except Exception:
        owner = None

    backend = str(getattr(st, "backend", "none") or "none")
    return OpticsSnapshot(
        connected=bool(getattr(st, "connected", False)),
        backend=backend,
        simulated=(backend == "simulated"),
        busy=bool(getattr(st, "busy", False)),
        error=getattr(st, "error", None),
        lease_owner=owner,
        objective_position=getattr(st, "objective_position", None),
        filter_position=getattr(st, "filter_position", None),
        focus_um=_f(getattr(st, "focus_um", None)),
        objectives=resolve_objectives(scope_state=st, config_store=config_store),
        filters=resolve_filters(scope_state=st, config_store=config_store),
    )
