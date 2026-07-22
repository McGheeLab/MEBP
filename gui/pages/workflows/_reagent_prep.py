"""_reagent_prep.py — shared reagent-location + needle-volume helpers.

v7.5.x: The needle-conditioning prep (waste → oil → wash → buffer) used by both
the Spheroid Pick & Place workflow and Quick Print resolves its service-well
locations the same way — from Hardware Setup → Ink (Reagent Locations), matching
a well to a role by the assigned ink's ``ink_type`` (or, as a fallback, an ink
literally named waste/oil/wash/buffer).

These are the free-function form of the resolution logic so Quick Print can
reuse it without depending on the spheroid page. The Spheroid page keeps its own
(identical) methods, so this module is additive — nothing existing is changed.

All positions returned are ABSOLUTE stage µm (the frame ``PickPlaceExecutor``
and ``safe_travel_to`` consume), taken straight from the calibrated
``well_positions`` map (well name → (x_um, y_um)).
"""

from __future__ import annotations

# The four reagent roles the prep cycles through. ``oil`` has no dedicated
# ``WellRole`` (PhysicalModels.well_role_for_ink_type maps it to INK), so we
# match it by the ink's ``ink_type`` string / ink name, not by WellRole.
SERVICE_ROLES = ("waste", "oil", "wash", "buffer")


def resolve_pickup_well(wells, plate=None):
    """Choose the reagent PICKUP well from an ``ink_locations`` list.

    ``ink_locations`` is append-only, so a well pinned to a plain well that
    later became a rosette leaves a stale, now-flattened PARENT name (e.g.
    ``"A2"``) in the list — usually at index 0. A flattened rosette parent is
    NOT a real pickup well: it is absent from the compiled ``plate.well_names``
    yet the calibrated ``well_positions`` map re-adds it at the sub-well
    CENTROID, so a naive ``wells[0]`` silently dips at the rosette centre
    instead of the intended sub-well.

    Prefer the first well that is a real (leaf) well in ``plate.well_names``
    (which excludes rosette parents — the same check the Well-Setup seed path
    uses). Fall back to the first entry when no plate is available or nothing
    qualifies → byte-identical legacy behaviour on non-rosette plates.
    """
    if not wells:
        return None
    valid = None
    if plate is not None:
        try:
            valid = set(plate.well_names)
        except Exception:
            valid = None
    if valid:
        for w in wells:
            if w in valid:
                return w
    return wells[0]


def needle_volume_uL(hw_config) -> float:
    """One needle's internal bore volume (µL) from the configured needle.

    This is "1 needle's worth" of fluid the prep volumes are multiples of.
    Returns 0.0 when no needle is configured (the caller gates on > 0).
    """
    needle = getattr(hw_config, "needle", None) if hw_config else None
    if needle is None:
        return 0.0
    try:
        return float(getattr(needle, "internal_volume_uL", 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def service_well_names(hw_config, plate=None) -> dict[str, str]:
    """role → well name, read from Hardware Setup reagent locations.

    A well is matched to a role by the assigned ink's ``ink_type`` (or, as a
    fallback, an ink literally named waste/oil/wash/buffer). The first *real*
    well assigned to each role wins — a flattened rosette parent is skipped in
    favour of a sub-well when ``plate`` is supplied (see ``resolve_pickup_well``).
    """
    out: dict[str, str] = {}
    if hw_config is None:
        return out
    ink_locations = getattr(hw_config, "ink_locations", {}) or {}
    ink_library = getattr(hw_config, "ink_library", {}) or {}
    for ink_name, wells in ink_locations.items():
        if not wells:
            continue
        spec = ink_library.get(ink_name)
        itype = ((getattr(spec, "ink_type", "") or "").strip().lower()
                 if spec is not None else "")
        name_l = (ink_name or "").strip().lower()
        role = (itype if itype in SERVICE_ROLES
                else name_l if name_l in SERVICE_ROLES else None)
        if role and role not in out:
            out[role] = resolve_pickup_well(wells, plate)
    return out


def resolve_service_positions(hw_config, well_positions, plate=None):
    """Return ``(positions, missing)``.

    ``positions`` maps each resolvable role → absolute stage µm (from the
    calibrated ``well_positions``); ``missing`` lists the roles whose well is
    either unassigned or not present in the calibrated well map. Pass ``plate``
    (the compiled ``WellPlate``) so a rosette parent resolves to a real sub-well.
    """
    names = service_well_names(hw_config, plate)
    wells = well_positions or {}
    positions: dict[str, tuple[float, float]] = {}
    for role in SERVICE_ROLES:
        wn = names.get(role)
        if wn and wn in wells:
            positions[role] = wells[wn]
    missing = [r for r in SERVICE_ROLES if r not in positions]
    return positions, missing
