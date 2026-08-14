"""v7.19 — a plate-wide queue of INDEPENDENT Quick Print jobs, as a pure model.

Quick Print prints one object into one well: the whole configuration is read live
off the widgets the moment Print is pressed, and the page holds exactly one of
everything. This module is the data half of "lay out a whole plate, then run the
lot" — one :class:`QueuedPrint` per well, each carrying its own object, size,
pump, ink, speed, resolution, print height and extrusion modifier, so two wells
can genuinely print different things.

It imports nothing from Qt and nothing from the controller, so ordering, swap
counting and the (de)serialization tolerance are all unit-testable without a
``QApplication`` or hardware. The GUI does the getattr-safe widget reads (that is
where mock tolerance belongs) and hands finished snapshots in.

Three rules govern the design, each learned from a defect this project has
already paid for:

1. **Store INPUTS, never derived outputs.** No ``segments``, no ``PrintSettings``,
   no ``center``, and above all no ZERO-REF Z. Between queueing a print and
   running it the operator can re-run Plate Location, swap the needle, or
   re-calibrate the plate bottom; a snapshot holding a derived zero-ref Z would
   then print at a stale absolute height with nothing on screen saying so.
   ``print_z_mm`` and ``ink_dip_z_mm`` are heights ABOVE PLATE BOTTOM, so every
   run re-resolves them against live calibration. It also keeps a snapshot
   JSON-serializable, and makes "the print file was deleted since you queued it"
   a detectable condition rather than a crash.

2. **The ink map is keyed by ink NAME, not by abstract-ink id.** Two independent
   reasons. JSON round-trips integer keys as strings, so an int-keyed map
   silently breaks on reload. And ``QuickPrintWorkflowPage._rebuild_ink_mapping_ui``
   hard-resets ``self._ink_map = {}`` and re-derives it from the name-keyed
   ``_ink_map_last``, so a snapshot that assigned ``_ink_map`` directly would
   have its mapping wiped by the very refresh that follows applying it.

3. **The sortable atom is the queued print, never the print unit.** A queued
   print whose object is a multi-ink sketch expands into several units, and their
   order is the SKETCH's deposition order (support before structure). Regrouping
   units globally to save ink swaps would silently reorder within such a print
   and print it wrong. :func:`order_queue` therefore permutes
   :class:`QueuedPrint` objects only.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields

#: Bumped when a stored queue's shape changes incompatibly. Readers accept an
#: absent/unknown value and fall back to per-field tolerance, so a queue written
#: by a newer build degrades to "the fields I recognise" rather than vanishing.
SCHEMA = 1

#: ``order_queue`` modes. "plate" is the default because it makes execution order
#: match reading order; "ink" trades that for fewer ink swaps.
ORDER_PLATE = "plate"
ORDER_INK = "ink"


def _as_float(value, default: float) -> float:
    """Coerce to float, falling back to *default* on anything unusable.

    Deliberately total: a queue restored from disk (or hand-edited) must never
    raise its way out of a page's ``showEvent``.
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    # NaN/inf would propagate into a print height or a speed.
    if out != out or out in (float("inf"), float("-inf")):
        return default
    return out


def _as_str(value, default: str = "") -> str:
    if value is None:
        return default
    if isinstance(value, str):
        return value
    try:
        return str(value)
    except Exception:                                   # pragma: no cover
        return default


@dataclass
class QueuedPrint:
    """One well's print, as a complete set of INPUTS (see rule 1 above).

    ``well`` is the queue's unique key: one print per well, so "click a well to
    load its print back for editing" has exactly one answer. Stacking two prints
    in a well at different heights is a legitimate future feature (multi-layer),
    and keeping the key unique now is what leaves room for it to be designed
    rather than stumbled into.
    """

    well: str
    #: The object combo's userData verbatim — ``"simple:circle"`` or
    #: ``"file:<name>"``. A REFERENCE, not geometry, which is what lets a
    #: deleted print file be reported instead of silently printing something else.
    object_data: str = ""
    #: Display text only. Never used to resolve geometry.
    object_label: str = ""
    size_mm: float = 1.0
    pump: str = "P1"
    #: "" means the operator picked "(none) — needle already loaded".
    ink_name: str = ""
    ink_dip_z_mm: float = 0.50          # ABOVE PLATE BOTTOM
    ink_padding_uL: float = 0.0
    top_speed_mm_s: float = 2.5
    resolution_um: float = 30.0
    print_z_mm: float = 0.20            # ABOVE PLATE BOTTOM
    extrusion_mod: float = 1.0
    motion_mode: str = "velocity"
    #: Abstract ink NAME → configured ink name (see rule 2).
    ink_map_by_name: dict = field(default_factory=dict)
    enabled: bool = True

    # ── serialization ────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "well": self.well,
            "object_data": self.object_data,
            "object_label": self.object_label,
            "size_mm": float(self.size_mm),
            "pump": self.pump,
            "ink_name": self.ink_name,
            "ink_dip_z_mm": float(self.ink_dip_z_mm),
            "ink_padding_uL": float(self.ink_padding_uL),
            "top_speed_mm_s": float(self.top_speed_mm_s),
            "resolution_um": float(self.resolution_um),
            "print_z_mm": float(self.print_z_mm),
            "extrusion_mod": float(self.extrusion_mod),
            "motion_mode": self.motion_mode,
            "ink_map_by_name": dict(self.ink_map_by_name),
            "enabled": bool(self.enabled),
        }

    @classmethod
    def from_dict(cls, data) -> "QueuedPrint":
        """Rebuild from stored JSON, tolerating anything.

        Never raises and never returns ``None``: this runs from a settings-profile
        apply, and a malformed entry must degrade to defaults (which the run-time
        gating then refuses by name) rather than take the page down. Unknown keys
        are filtered rather than splatted — the hazard ``NeedleSpec.from_dict``
        and ``PickPlaceTarget.from_dict`` were both hardened against, where a
        newer build's extra key crashed an older one.
        """
        if not isinstance(data, dict):
            return cls(well="")
        defaults = cls(well="")
        raw_map = data.get("ink_map_by_name")
        ink_map: dict = {}
        if isinstance(raw_map, dict):
            for k, v in raw_map.items():
                name, mapped = _as_str(k), _as_str(v)
                if name and mapped:
                    ink_map[name] = mapped
        return cls(
            well=_as_str(data.get("well")),
            object_data=_as_str(data.get("object_data")),
            object_label=_as_str(data.get("object_label")),
            size_mm=_as_float(data.get("size_mm"), defaults.size_mm),
            pump=_as_str(data.get("pump"), defaults.pump) or defaults.pump,
            ink_name=_as_str(data.get("ink_name")),
            ink_dip_z_mm=_as_float(data.get("ink_dip_z_mm"),
                                   defaults.ink_dip_z_mm),
            ink_padding_uL=_as_float(data.get("ink_padding_uL"),
                                     defaults.ink_padding_uL),
            top_speed_mm_s=_as_float(data.get("top_speed_mm_s"),
                                     defaults.top_speed_mm_s),
            resolution_um=_as_float(data.get("resolution_um"),
                                    defaults.resolution_um),
            print_z_mm=_as_float(data.get("print_z_mm"), defaults.print_z_mm),
            extrusion_mod=_as_float(data.get("extrusion_mod"),
                                    defaults.extrusion_mod),
            motion_mode=_as_str(data.get("motion_mode"),
                                defaults.motion_mode) or defaults.motion_mode,
            ink_map_by_name=ink_map,
            enabled=bool(data.get("enabled", True)),
        )

    def copy(self) -> "QueuedPrint":
        """An INDEPENDENT copy.

        Stamping one print into forty wells must not share a single object
        between them — "each print is still treated as an independent activity"
        stops being true the moment editing A17 edits all forty.
        """
        return QueuedPrint.from_dict(self.to_dict())

    def label(self) -> str:
        """One line for a readout: ``"A1 · spiral · Alginate"``."""
        parts = [self.well or "?"]
        if self.object_label:
            parts.append(self.object_label)
        if self.ink_name:
            parts.append(self.ink_name)
        return " · ".join(parts)


# ── queue-level pure helpers ──────────────────────────────────────────
#
# The queue itself is a plain ``list[QueuedPrint]``: it preserves the order the
# operator authored (which is one of the offered run orders) and matches the
# ``bore_programs`` list precedent already used for persisted sub-configs.


def upsert(queue, qp: QueuedPrint) -> list:
    """Return *queue* with *qp* replacing any existing entry for its well.

    Position is preserved on replace, so re-stamping a well does not shuffle the
    authored order under the operator.
    """
    out = list(queue or [])
    for i, existing in enumerate(out):
        if existing.well == qp.well:
            out[i] = qp
            return out
    out.append(qp)
    return out


def remove(queue, well: str) -> list:
    return [qp for qp in (queue or []) if qp.well != well]


def get(queue, well: str):
    for qp in (queue or []):
        if qp.well == well:
            return qp
    return None


def wells(queue) -> list:
    return [qp.well for qp in (queue or [])]


def queue_to_state(queue) -> dict:
    return {"schema": SCHEMA,
            "prints": [qp.to_dict() for qp in (queue or [])]}


def queue_from_state(state) -> list:
    """Rebuild a queue from stored state, dropping only what is unparseable.

    Tolerates ``None``, a bare list (an older shape), a dict without ``prints``,
    and garbage entries. An entry whose well is empty is dropped — the well IS
    the key, so a keyless entry could never be found, edited or run.
    """
    if isinstance(state, dict):
        items = state.get("prints")
    elif isinstance(state, list):
        items = state
    else:
        return []
    if not isinstance(items, list):
        return []
    out: list = []
    seen: set = set()
    for entry in items:
        qp = QueuedPrint.from_dict(entry)
        if not qp.well or qp.well in seen:
            continue
        seen.add(qp.well)
        out.append(qp)
    return out


def order_queue(queue, mode: str = ORDER_PLATE, plate_index=None) -> list:
    """Deterministic run order for the QUEUED PRINTS (never for units — rule 3).

    ``mode=ORDER_PLATE`` (default) is plate order, so execution order matches how
    the plate reads. ``mode=ORDER_INK`` groups prints sharing an ink so each ink
    is picked up once per contiguous run, breaking ties by plate order — the same
    cut-the-swaps principle ``SketchTrajectory.optimize_print_order`` already
    applies to sketch shapes. Each ink swap is a full waste → wash → buffer
    cycle, so on an alternating two-ink plate the difference is minutes per well.

    ``plate_index(well) -> int`` orders wells as the plate does; an unknown well
    sorts last (stably) rather than raising. With no ``plate_index`` the authored
    order stands in, which keeps this total for a page that has no plate yet.

    Ink ranks are assigned by first appearance in the AUTHORED queue, so the
    result is stable and reproducible instead of depending on set iteration.
    """
    items = list(queue or [])
    if not items:
        return []

    def _pos(qp: QueuedPrint) -> tuple:
        if plate_index is None:
            return (0, items.index(qp))
        try:
            idx = plate_index(qp.well)
        except Exception:
            idx = None
        if idx is None:
            # Unknown well: sort after every known one, but keep it — the run
            # gating reports it by name, which beats dropping it here silently.
            return (1, items.index(qp))
        return (0, int(idx))

    if mode != ORDER_INK:
        return sorted(items, key=_pos)

    rank: dict = {}
    for qp in items:
        rank.setdefault(qp.ink_name, len(rank))
    return sorted(items, key=lambda qp: (rank[qp.ink_name],) + _pos(qp))


def count_ink_swaps(units, *, force_clean_between: bool = False) -> int:
    """Ink swaps a unit sequence will cost.

    A swap happens when a unit's ink differs from the previous unit's. The FIRST
    unit never costs one: after the single up-front prep the needle is clean and
    buffer-loaded, so its pickup needs no wash first. ``force_clean_between``
    mirrors the "Clean between every print" escape hatch, which cleans regardless
    of whether the ink changed.

    Counts units, not queued prints, because a multi-ink sketch that uses one ink
    non-contiguously (``A(1) B(2) C(1)``) genuinely costs two swaps — pretending
    otherwise would make the disclosed number wrong.
    """
    prev = None
    swaps = 0
    for unit in (units or []):
        ink = unit.get("ink_name") if isinstance(unit, dict) else getattr(
            unit, "ink_name", "")
        if prev is not None and (force_clean_between or ink != prev):
            swaps += 1
        prev = ink
    return swaps


def plate_index_from_names(names) -> "callable":
    """Build a ``plate_index`` for :func:`order_queue` from a plate's well-name
    sequence (i.e. ``WellPlate.well_names``), which is already plate order."""
    lookup = {name: i for i, name in enumerate(names or [])}
    return lambda well: lookup.get(well)


#: Field names ``QueuedPrint`` accepts — used by the page to assert its widget
#: capture stays in step with the model.
FIELD_NAMES = tuple(f.name for f in fields(QueuedPrint))
