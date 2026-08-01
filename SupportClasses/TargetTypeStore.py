"""
TargetTypeStore.py — User-defined TARGET TYPES and their authored signature rules.

v7.9: On a multi-bore needle each bore is bound to a **class of object to remove**,
not to an imaging channel. The operator's own framing (2026-08-01):

    "an explicit mapping for a target type that the user defines not exactly per
    channel. it is based on some meta data for the objects being removed. for
    example we will identify a cells flouescent signature say it has some
    brightness in both channels. or cells with brigtness in one channel etc. the
    user decides these rules."

So a ``TargetType`` is a named, coloured class of object plus a **signature rule**
over the fluorescence imaging channels (``FluorescenceMosaicStore.CHANNELS`` —
DAPI / FITC / mCherry / Cy5 / Bright Field, keyed by NAME). "Bright in both
channels" and "bright in one channel" are the same rule shape with different
clauses, which is why the shape is a combinator over per-imaging-channel clauses
rather than a per-channel setting.

⚠⚠ **THE RULE IS AUTHORED AND PERSISTED. NOTHING EVALUATES IT AGAINST AN IMAGE.**
That is a deliberate deferral (v7.9 decision D3), not an oversight — the operator
explicitly scoped it out: *"the scheme for selecting the cells from a flourescent
image will be done later. for now we are focusing on the user defines something
for each pump channel to go and do something."* Choosing the segmentation /
gating scheme is a research decision that has to be made against real stained
images; guessing it now would bake a wrong metric into saved operator data.
:meth:`SignatureRule.is_evaluatable` therefore returns ``False`` (see
``RULE_EVALUATION_IMPLEMENTED``) and :meth:`SignatureRule.evaluate` refuses with a
named ``NotImplementedError`` — a loud, actionable refusal rather than a silent
``False`` that a caller could mistake for "no match".

**The rule is DATA, not code.** A serialized predicate expression (or worse, an
eval'd string) would mean the evaluator's semantics leak into every saved file and
could never be revised. A declarative clause list can be re-interpreted by a
smarter evaluator later — and each clause already carries the ``metric`` naming
what its number means, so adding the evaluator needs **no migration**: the fields
it will read are on disk from the first save.

**An empty rule matches NOTHING, never everything.** :meth:`SignatureRule.is_empty`
is True for a type the operator has not finished authoring, and ``describe()`` says
so. A universal default would, the moment an evaluator lands, select every object
in the well and remove them all — so the fail-safe direction is "unmatched",
mirroring the v7.6 decision to have a missed call site read ONE REAL BORE rather
than a fictional aggregate.

**STAMP, DON'T REFERENCE.** When a run records that a target was of a given type it
stores the id **and** the resolved name + colour (:meth:`TargetType.stamp` /
:func:`stamp_target_type`), and never resolves the id back through this store at
load time. Editing or deleting a target type therefore can never silently mutate
saved run data or relabel a completed removal — the same contract as
``NeedleTypeStore`` / ``WellTypeStore`` / ``PlateTypeStore``.

Persistence (this module has ZERO GUI dependencies — json + dataclasses only):

    config/hardware/target_types/builtin/<id>.json   — bundled examples (read-only)
    config/hardware/target_types/user/<id>.json      — user types + overrides

A user entry with the **same id SHADOWS** the built-in (user wins). Built-ins stay
pristine; ``delete_user`` only removes user files and then reloads so a shadowed
built-in re-surfaces. Set ``MEBP_TARGET_TYPE_DIR`` to a directory containing
``builtin/`` and ``user/`` subdirectories to redirect both halves (test isolation).

This is **per-machine / per-operator** data and deliberately NOT part of
``HardwareConfig``: a print setup file copied from another rig must never carry
another lab's target definitions (the ``CAMERA_CAL_PERSIST_STORE`` lesson).
"""

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_HW_DIR = Path(__file__).resolve().parent.parent / "config" / "hardware"
_DEFAULT_BUILTIN_DIR = _HW_DIR / "target_types" / "builtin"
_DEFAULT_USER_DIR = _HW_DIR / "target_types" / "user"

# ── the authoring vocabulary ───────────────────────────────────────

# What the operator asserts about one imaging channel. Deliberately the
# operator's own words ("has brightness in" / "does not") rather than a numeric
# comparison, because a bare threshold is what the deferred evaluator gets to
# choose; the STATE is what only the operator can know.
STATE_POSITIVE = "positive"     # bright in this imaging channel
STATE_NEGATIVE = "negative"     # dim / absent in this imaging channel
STATE_ANY = "any"               # don't care (recorded, but never gates)
CLAUSE_STATES: tuple[str, ...] = (STATE_POSITIVE, STATE_NEGATIVE, STATE_ANY)

# How to combine the clauses. "bright in BOTH channels" is ALL over two positive
# clauses; "bright in ONE channel" is authored as ALL over a positive clause plus
# a negative clause for the other — which is what makes "only that channel"
# unambiguous, and is exactly how a flow-cytometry gate is specified. There is no
# "exactly one of" combinator: it would have no defined meaning per-threshold.
COMBINATOR_ALL = "all"
COMBINATOR_ANY = "any"
COMBINATORS: tuple[str, ...] = (COMBINATOR_ALL, COMBINATOR_ANY)

# What a clause's ``threshold`` number MEANS. Named so the deferred evaluator can
# be written without re-asking the operator, and aligned with the thresholding
# vocabulary already in SpheroidDetector._threshold (a fixed absolute level, and a
# ``background mean + kσ`` floor).
METRIC_AUTO = "auto"                    # no number given — evaluator picks its own level
METRIC_INTENSITY = "intensity"          # absolute brightness, 0-255 in the mosaic channel
METRIC_BACKGROUND_SIGMA = "background_sigma"   # k · σ above the background mean
METRIC_PERCENTILE = "percentile"        # percentile of the channel's own histogram
METRICS: tuple[str, ...] = (METRIC_AUTO, METRIC_INTENSITY,
                            METRIC_BACKGROUND_SIGMA, METRIC_PERCENTILE)

# ⚠ Flipped to True ONLY when a real evaluator lands (v7.9 decision D3). Every
# caller that would act on a rule must gate on this, not on "the rule looks
# complete" — an authored rule is a statement of intent, not a working detector.
RULE_EVALUATION_IMPLEMENTED = False

_DEFAULT_COLOR = "#cba6f7"
_HEX_RE = re.compile(r"^#[0-9a-fA-F]{6}$")


def safe_id(value) -> str:
    """Filesystem-safe token for a target-type id (used as the JSON filename)."""
    return re.sub(r"[^A-Za-z0-9_.-]", "_", str(value)) or "target_type"


def _safe_color(value) -> str:
    """Normalize a display colour to ``#rrggbb`` (hex, like ``InkSpec.color``).

    A bad colour is a cosmetic problem, so it degrades to the default instead of
    refusing to load a target type the operator spent time authoring.
    """
    text = str(value or "").strip()
    if not text:
        return _DEFAULT_COLOR
    if not text.startswith("#"):
        text = "#" + text
    if len(text) == 4 and _HEX_RE.match("#" + text[1] * 2 + text[2] * 2 + text[3] * 2):
        # #abc → #aabbcc
        text = "#" + text[1] * 2 + text[2] * 2 + text[3] * 2
    return text.lower() if _HEX_RE.match(text) else _DEFAULT_COLOR


# ── one clause ─────────────────────────────────────────────────────

@dataclass
class SignatureClause:
    """One assertion about ONE imaging channel of the object's signature.

    ``imaging_channel`` is a channel NAME from ``FluorescenceMosaicStore.CHANNELS``
    (never an index — the fluorescence store is name-keyed throughout, and a slot
    number would silently re-point if the turret assignment changed).

    ``threshold`` is optional: ``None`` means the operator asserted only the
    STATE ("it's bright in FITC") and left the level to the evaluator. When a
    number IS given, ``metric`` says what it is measured in. The pair is kept
    coherent by ``__post_init__`` — a number with no metric reads as an absolute
    intensity (the plainest interpretation, matching ``_threshold(mode="fixed")``),
    and a metric with no number collapses back to ``auto``.
    """

    imaging_channel: str = ""
    state: str = STATE_POSITIVE
    threshold: Optional[float] = None
    metric: str = METRIC_AUTO

    def __post_init__(self) -> None:
        self.imaging_channel = str(self.imaging_channel or "").strip()
        state = str(self.state or STATE_POSITIVE).strip().lower()
        self.state = state if state in CLAUSE_STATES else STATE_POSITIVE
        if self.threshold is not None:
            try:
                self.threshold = float(self.threshold)
            except (TypeError, ValueError):
                self.threshold = None
        metric = str(self.metric or METRIC_AUTO).strip().lower()
        self.metric = metric if metric in METRICS else METRIC_AUTO
        # Keep the (threshold, metric) pair meaningful in both directions.
        if self.threshold is None:
            self.metric = METRIC_AUTO
        elif self.metric == METRIC_AUTO:
            self.metric = METRIC_INTENSITY

    # ── serialization ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        out: dict = {
            "imaging_channel": self.imaging_channel,
            "state": self.state,
        }
        # Conditional-emit: a clause that only asserts a state stays a two-key
        # object, so re-saving an untouched type is byte-stable.
        if self.threshold is not None:
            out["threshold"] = self.threshold
            out["metric"] = self.metric
        return out

    @classmethod
    def from_dict(cls, data) -> "SignatureClause":
        """Build from JSON, IGNORING unknown keys (forward-compat).

        A future field (say a per-clause ROI or a morphology term) must not stop
        this build from loading the file on an older install.
        """
        if not isinstance(data, dict):
            return cls()
        return cls(
            imaging_channel=data.get("imaging_channel", ""),
            state=data.get("state", STATE_POSITIVE),
            threshold=data.get("threshold"),
            metric=data.get("metric", METRIC_AUTO),
        )

    # ── read ──────────────────────────────────────────────────────

    @property
    def is_gating(self) -> bool:
        """True when this clause actually constrains anything.

        A clause with no channel, or an explicit ``any`` state, is recorded (the
        operator may be mid-edit) but must never narrow or widen a match.
        """
        return bool(self.imaging_channel) and self.state != STATE_ANY

    def describe(self) -> str:
        """Compact human-readable form, e.g. ``FITC+ (≥ 120)`` / ``mCherry−``."""
        name = self.imaging_channel or "(no imaging channel)"
        if self.state == STATE_ANY:
            return f"{name} (any)"
        mark = "+" if self.state == STATE_POSITIVE else "−"   # U+2212 minus
        head = f"{name}{mark}"
        if self.threshold is None:
            return head
        # The comparison direction follows the state: a negative clause is an
        # upper bound, not a lower one.
        rel = "≥" if self.state == STATE_POSITIVE else "<"
        if self.metric == METRIC_BACKGROUND_SIGMA:
            tail = f"{rel} {self.threshold:g}σ over background"
        elif self.metric == METRIC_PERCENTILE:
            tail = f"{rel} {self.threshold:g}th percentile"
        else:
            tail = f"{rel} {self.threshold:g}"
        return f"{head} ({tail})"


# ── the rule ───────────────────────────────────────────────────────

@dataclass
class SignatureRule:
    """A declarative fluorescent-signature rule: clauses + how to combine them.

    AUTHORED ONLY — see the module docstring. Nothing in this class looks at an
    image; :meth:`evaluate` refuses by design.
    """

    combinator: str = COMBINATOR_ALL
    clauses: list = field(default_factory=list)

    def __post_init__(self) -> None:
        comb = str(self.combinator or COMBINATOR_ALL).strip().lower()
        self.combinator = comb if comb in COMBINATORS else COMBINATOR_ALL
        src = self.clauses if isinstance(self.clauses, (list, tuple)) else []
        out: list[SignatureClause] = []
        for entry in src:
            if isinstance(entry, SignatureClause):
                out.append(entry)
            elif isinstance(entry, dict):
                out.append(SignatureClause.from_dict(entry))
            # Anything else (a bare string in a hand-edited file) is dropped
            # rather than raising — a partly-corrupt file must still open.
        self.clauses = out

    # ── serialization ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "combinator": self.combinator,
            "clauses": [c.to_dict() for c in self.clauses],
        }

    @classmethod
    def from_dict(cls, data) -> "SignatureRule":
        """Build from JSON, IGNORING unknown keys (forward-compat)."""
        if not isinstance(data, dict):
            return cls()
        return cls(
            combinator=data.get("combinator", COMBINATOR_ALL),
            clauses=data.get("clauses") or [],
        )

    # ── read ──────────────────────────────────────────────────────

    @property
    def gating_clauses(self) -> list:
        """Only the clauses that constrain something (see ``is_gating``)."""
        return [c for c in self.clauses if c.is_gating]

    def is_empty(self) -> bool:
        """True when nothing is asserted — and therefore NOTHING matches.

        Deliberately not "matches everything": an unfinished rule that selected
        every object in the well would, once an evaluator exists, remove them all.
        """
        return not self.gating_clauses

    def imaging_channels(self) -> list:
        """The distinct imaging-channel names this rule references, in order."""
        seen: list[str] = []
        for c in self.clauses:
            if c.imaging_channel and c.imaging_channel not in seen:
                seen.append(c.imaging_channel)
        return seen

    def unknown_imaging_channels(self, known) -> list:
        """Referenced channel names NOT in ``known`` (e.g. ``CHANNELS``).

        Reported, never repaired: a name the current build doesn't recognise may
        be a cube this rig simply doesn't have mounted today, so the UI warns and
        the operator's authored rule is preserved intact.
        """
        allowed = {str(k) for k in (known or ())}
        return [name for name in self.imaging_channels() if name not in allowed]

    def is_evaluatable(self) -> bool:
        """Whether this rule can actually be run against an image. **False.**

        Gate on this before offering any "select matching objects" action; see
        ``RULE_EVALUATION_IMPLEMENTED`` and v7.9 decision D3.
        """
        return RULE_EVALUATION_IMPLEMENTED

    def evaluate(self, *_args, **_kwargs):
        """Refuse, loudly and by name — rule evaluation is deferred (D3).

        Raising beats returning ``False``: a bool would be indistinguishable from
        a genuine "this object does not match" and would silently skip every
        target the operator asked to remove.
        """
        raise NotImplementedError(
            "Fluorescent-signature rule evaluation is not implemented yet "
            "(v7.9 decision D3: the scheme for selecting cells from a "
            "fluorescence image is deferred). This rule is authored metadata "
            f"only: {self.describe()}")

    def describe(self) -> str:
        """Human-readable summary, e.g. ``FITC+ and mCherry−``."""
        parts = [c.describe() for c in self.gating_clauses]
        if not parts:
            return "no signature rule yet (matches nothing)"
        joiner = " and " if self.combinator == COMBINATOR_ALL else " or "
        return joiner.join(parts)


# ── a target type ──────────────────────────────────────────────────

@dataclass
class TargetType:
    """A user-defined class of object to remove, plus its signature rule."""

    id: str
    name: str = ""
    color: str = _DEFAULT_COLOR          # display colour (hex, like InkSpec.color)
    notes: str = ""
    signature_rule: SignatureRule = field(default_factory=SignatureRule)
    builtin: bool = False

    def __post_init__(self) -> None:
        # Coerce defensively (JSON may carry anything).
        self.id = str(self.id)
        self.name = str(self.name or "")
        self.notes = str(self.notes or "")
        self.color = _safe_color(self.color)
        if not isinstance(self.signature_rule, SignatureRule):
            self.signature_rule = SignatureRule.from_dict(self.signature_rule)

    # ── serialization ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        out = {
            "id": self.id,
            "name": self.name,
            "color": self.color,
            "notes": self.notes,
            "signature_rule": self.signature_rule.to_dict(),
            "builtin": self.builtin,
        }
        return out

    @classmethod
    def from_dict(cls, data: dict, *, builtin: bool = False) -> "TargetType":
        """Build from JSON, IGNORING unknown keys (forward-compat).

        Every field is read with an explicit ``get`` and a default, so a partial
        or hand-edited file yields a usable target type (with an empty rule, which
        matches nothing) instead of raising.
        """
        if not isinstance(data, dict):
            data = {}
        return cls(
            id=data.get("id", ""),
            name=data.get("name", ""),
            color=data.get("color", _DEFAULT_COLOR),
            notes=data.get("notes", ""),
            signature_rule=SignatureRule.from_dict(data.get("signature_rule")),
            builtin=bool(data.get("builtin", builtin)),
        )

    # ── read ──────────────────────────────────────────────────────

    @property
    def label(self) -> str:
        """Human-readable combo label (falls back to the id)."""
        return self.name or self.id

    def describe(self) -> str:
        """``<name> — <rule>`` for a one-line UI readout / log line."""
        return f"{self.label} — {self.signature_rule.describe()}"

    def stamp(self) -> dict:
        """The provenance block a saved run/target embeds for this type.

        STAMP, DON'T REFERENCE: carries the resolved ``name`` and ``color``
        alongside the id, so a later rename, recolour or delete cannot mutate or
        un-label data already recorded for a completed run.
        """
        return {
            "target_type_id": self.id,
            "target_type_name": self.label,
            "target_type_color": self.color,
        }


def stamp_target_type(target_type) -> dict:
    """Duck-typed, None-safe :meth:`TargetType.stamp`.

    Returns ``{}`` when there is no target type (an untyped target is legal —
    a single-bore run needs no classification), and tolerates a stub/MagicMock
    standing in for the real object.
    """
    if target_type is None:
        return {}
    try:
        stamped = target_type.stamp()
    except Exception:
        stamped = None
    # Insist on a real dict. A MagicMock's ``.stamp()`` RETURNS a MagicMock
    # instead of raising, and handing that back would silently drop the
    # provenance at the caller (``record.update(...)`` on a non-mapping adds
    # nothing) and then explode in ``json.dumps`` at save time, far from here —
    # the exact un-labelled-run failure STAMP-DON'T-REFERENCE exists to prevent.
    if isinstance(stamped, dict):
        return stamped
    tid = str(getattr(target_type, "id", "") or "")
    if not tid:
        return {}
    return {
        "target_type_id": tid,
        "target_type_name": str(getattr(target_type, "name", "") or tid),
        "target_type_color": _safe_color(getattr(target_type, "color", None)),
    }


# ── the store ──────────────────────────────────────────────────────

def _resolve_dirs(builtin_dir, user_dir) -> tuple[Path, Path]:
    """Where to read from: explicit args → ``$MEBP_TARGET_TYPE_DIR`` → defaults.

    The env override names ONE directory holding ``builtin/`` and ``user/``, so a
    test can isolate both halves (including shipping its own fake built-ins) with
    a single variable.
    """
    if builtin_dir is not None or user_dir is not None:
        return (Path(builtin_dir) if builtin_dir is not None else _DEFAULT_BUILTIN_DIR,
                Path(user_dir) if user_dir is not None else _DEFAULT_USER_DIR)
    env = os.environ.get("MEBP_TARGET_TYPE_DIR")
    if env:
        root = Path(env)
        return root / "builtin", root / "user"
    return _DEFAULT_BUILTIN_DIR, _DEFAULT_USER_DIR


class TargetTypeStore:
    """Load/save target types: bundled built-ins + user overrides (user wins)."""

    def __init__(self, builtin_dir=None, user_dir=None):
        self._builtin_dir, self._user_dir = _resolve_dirs(builtin_dir, user_dir)
        self._types: dict[str, TargetType] = {}
        self.reload()

    # ── Load ──────────────────────────────────────────────────────

    def _load_dir(self, directory: Path, builtin: bool) -> None:
        if not directory.exists():
            return
        for path in sorted(directory.glob("*.json")):
            try:
                with open(path, encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as exc:  # pragma: no cover - corrupt file
                logger.warning(f"TargetTypeStore: failed to load {path}: {exc}")
                continue
            tt = TargetType.from_dict(data, builtin=builtin)
            if not tt.id:
                logger.warning(f"TargetTypeStore: {path} has no id — skipped")
                continue
            # User entries (loaded second) shadow built-ins of the same id.
            self._types[tt.id] = tt

    def reload(self) -> None:
        """Re-scan both directories (built-ins first, then user overrides)."""
        self._types = {}
        self._load_dir(self._builtin_dir, builtin=True)
        self._load_dir(self._user_dir, builtin=False)

    # ── Read ──────────────────────────────────────────────────────

    def get(self, target_type_id) -> Optional[TargetType]:
        if target_type_id is None:
            return None
        return self._types.get(str(target_type_id))

    def all(self) -> list[TargetType]:
        """Every known target type, sorted by label."""
        return sorted(self._types.values(), key=lambda t: t.label.lower())

    def stamp_for(self, target_type_id) -> dict:
        """Provenance block for an id — ``{}`` when the id is unknown.

        Resolve ONCE at record time; never re-resolve a stamped id at load time
        (that is what STAMP, DON'T REFERENCE means).
        """
        return stamp_target_type(self.get(target_type_id))

    # ── Write (user overrides only) ───────────────────────────────

    def save_user(self, target_type: TargetType) -> bool:
        """Persist a USER target type / override (built-ins stay pristine).

        Always written under the user directory with ``builtin=False``; the same
        id shadows a bundled built-in. Updates the in-memory cache so the live
        list reflects the new type immediately.
        """
        if not target_type.id:
            logger.warning("TargetTypeStore.save_user: missing id — skipped")
            return False
        target_type.builtin = False
        fname = f"{safe_id(target_type.id)}.json"
        try:
            self._user_dir.mkdir(parents=True, exist_ok=True)
            tmp = self._user_dir / f"{fname}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(target_type.to_dict(), f, indent=2)
            os.replace(tmp, self._user_dir / fname)
        except Exception as exc:
            logger.error(f"TargetTypeStore.save_user: write failed: {exc}")
            return False
        self._types[target_type.id] = target_type
        logger.info(
            f"TargetTypeStore: saved user target type '{target_type.id}' "
            f"({target_type.signature_rule.describe()})")
        return True

    def delete_user(self, target_type_id) -> bool:
        """Delete a USER target-type file (built-ins cannot be deleted).

        Removes ``user/<id>.json`` if present, then reloads so a shadowed
        built-in (if any) re-surfaces. Returns True if a user file was removed.
        """
        if not target_type_id:
            return False
        fname = f"{safe_id(target_type_id)}.json"
        path = self._user_dir / fname
        if not path.exists():
            return False
        try:
            os.remove(path)
        except Exception as exc:
            logger.error(f"TargetTypeStore.delete_user: remove failed: {exc}")
            return False
        self.reload()
        logger.info(f"TargetTypeStore: deleted user target type '{target_type_id}'")
        return True


_store_singleton: Optional[TargetTypeStore] = None


def get_store() -> TargetTypeStore:
    """Process-wide singleton (lazy)."""
    global _store_singleton
    if _store_singleton is None:
        _store_singleton = TargetTypeStore()
    return _store_singleton
