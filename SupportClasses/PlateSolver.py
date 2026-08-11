"""Constraint solver for the v7.12 :class:`~SupportClasses.PlateDocument`.

Same shape as ``PlateSketchSolver`` — ``scipy.optimize.least_squares`` over
point coordinates — with four deliberate changes.

**1. Pay-for-play variables.** Only points REFERENCED by an active constraint
enter the vector. An unconstrained 384-well plate reports ``EMPTY`` and the
solver never runs; the old one built a 768-variable problem and SVD'd its
Jacobian on every edit. Unconstrained geometry also can never be nudged by a
solve it had nothing to do with.

**2. Pattern members are not variables.** A member resolves to *its pattern's
anchor plus a constant offset*, so dimensioning one member is satisfied by
translating the whole pattern. Two variables move a 384-well grid, and a
pattern can never shear.

**3. The drag ghost lives here, not on the document.** ``PlateSketchSolver``
appended its transient weight-1000 pin into ``design.constraints``; any save or
undo snapshot taken mid-drag serialized it, and seven reached disk as invisible
hard pins. Holding it in solver state makes that structurally impossible — a
filter on the way out is one forgotten call site away from failing.

**4. Structural validation runs before residuals.** The old
``_residual_count`` counted rows unconditionally while ``_residuals`` skipped
structurally-invalid refs. The over-count could pick ``lm`` for an
under-determined system AND shifted the conflict-attribution cursor, so the
WRONG constraint ids were reported as conflicting. Validating first makes the
predicted row count exact by construction.

Pure Python + numpy + scipy. No Qt.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from SupportClasses.PlateDocument import (
    Constraint, EntityId, GridPattern, Line, PatternFeature, PlateDocument,
    Point, Ref, RingPattern, Well,
)
from SupportClasses.SolveTypes import DOFStatus, SolveReport

logger = logging.getLogger(__name__)

_INCONSISTENT_RESIDUAL_EPS = 1e-4
#: Above this many free variables, skip the rank SVD and report a bound.
_RANK_MAX_VARS = 200
#: Above this, use QR instead of SVD for the rank (same answer, 2-5x faster).
_RANK_QR_VARS = 60

#: Rows contributed per kind. Mirrors `_residuals` exactly — see note 4.
_ROWS: dict[str, int] = {
    "coincident": 2, "concentric": 2, "symmetric": 2, "drag_ghost": 2,
    "distance": 1, "distance_to_datum": 1, "distance_to_line": 1,
    "horizontal": 1, "vertical": 1, "point_on_line": 1, "parallel": 1,
    "perpendicular": 1, "equal_length": 1, "tangent": 1, "angle": 1,
    "radial": 1,
    "fix": 0, "equal_radius": 0,
}

DRAG_GHOST_WEIGHT = 1000.0


class PlateSolver:
    """Short-lived; construct one per interaction."""

    def __init__(self, doc: PlateDocument):
        self.doc = doc
        self._ghost: Optional[Constraint] = None
        self._ghost_target: tuple[float, float] = (0.0, 0.0)

    # ── Drag lifecycle (ghost is solver-side ONLY) ────────────────

    def begin_drag(self, ref: Ref, target_xy: tuple[float, float]
                   ) -> SolveReport:
        self._ghost = Constraint(id=-1, kind="drag_ghost", refs=[ref],
                                 weight=DRAG_GHOST_WEIGHT)
        self._ghost_target = target_xy
        return self.solve()

    def update_drag(self, target_xy: tuple[float, float]) -> SolveReport:
        if self._ghost is None:
            return self.solve()
        self._ghost_target = target_xy
        return self.solve()

    def end_drag(self) -> SolveReport:
        self._ghost = None
        return self.solve()

    @property
    def dragging(self) -> bool:
        return self._ghost is not None

    # ── Reference resolution ──────────────────────────────────────

    def _base_point(self, ref: Ref) -> Optional[tuple[EntityId, float, float]]:
        """``(point_id, dx, dy)`` — the free point a ref moves with, plus a
        constant offset from it. None when the ref does not resolve."""
        eid, member = ref
        ent = self.doc.entities.get(eid)
        if ent is None:
            return None
        if isinstance(ent, Point):
            return (ent.id, 0.0, 0.0)
        if isinstance(ent, Well):
            pt = self.doc.entities.get(ent.center)
            return (pt.id, 0.0, 0.0) if isinstance(pt, Point) else None
        if isinstance(ent, PatternFeature):
            anchor = self.doc.entities.get(ent.anchor)
            if not isinstance(anchor, Point):
                return None
            if not member:
                return (anchor.id, 0.0, 0.0)
            for key, dx, dy in ent.local_offsets():
                if key == member:
                    ov = ent.overrides.get(member)
                    if ov is not None and ov.offset_mm:
                        dx += ov.offset_mm[0]
                        dy += ov.offset_mm[1]
                    return (anchor.id, dx, dy)
            return None
        if isinstance(ent, Line):
            return None          # a line is two points; handled per-kind
        return None

    def _line_points(self, ref: Ref) -> Optional[tuple[Ref, Ref]]:
        ent = self.doc.entities.get(ref[0])
        if not isinstance(ent, Line):
            return None
        if not (isinstance(self.doc.entities.get(ent.p1), Point)
                and isinstance(self.doc.entities.get(ent.p2), Point)):
            return None
        return ((ent.p1, ""), (ent.p2, ""))

    def _radius_of(self, ref: Ref) -> Optional[float]:
        """Well radius for a ref, or None. Constant — diameters are parameters,
        not solver variables."""
        eid, member = ref
        ent = self.doc.entities.get(eid)
        if isinstance(ent, Well):
            return ent.style.diameter_mm / 2.0
        if isinstance(ent, PatternFeature):
            ov = ent.overrides.get(member) if member else None
            d = (ov.diameter_mm if ov is not None and ov.diameter_mm is not None
                 else ent.style.diameter_mm)
            return d / 2.0
        return None

    # ── Structural validity (note 4) ──────────────────────────────

    def _valid(self, c: Constraint) -> bool:
        if c.driven or c.kind in ("fix", "equal_radius"):
            return False                     # no residual rows
        if c.kind not in _ROWS:
            return False
        n = len(c.refs)

        if c.kind in ("coincident", "concentric", "distance", "horizontal",
                      "vertical", "tangent"):
            return n >= 2 and all(self._base_point(r) for r in c.refs[:2])
        if c.kind == "drag_ghost":
            return n >= 1 and self._base_point(c.refs[0]) is not None
        if c.kind == "distance_to_datum":
            return (n >= 1 and self._base_point(c.refs[0]) is not None
                    and self.doc.boundary.datum_value(c.datum) is not None
                    and c.value is not None)
        if c.kind == "radial":
            return (n >= 1 and self._base_point(c.refs[0]) is not None
                    and c.value is not None)
        if c.kind in ("point_on_line", "distance_to_line"):
            return (n >= 2 and self._base_point(c.refs[0]) is not None
                    and self._line_points(c.refs[1]) is not None)
        if c.kind in ("parallel", "perpendicular", "equal_length", "angle"):
            return (n >= 2 and self._line_points(c.refs[0]) is not None
                    and self._line_points(c.refs[1]) is not None)
        if c.kind == "symmetric":
            return (n >= 3 and self._base_point(c.refs[0]) is not None
                    and self._base_point(c.refs[1]) is not None
                    and self._line_points(c.refs[2]) is not None)
        return False

    def _active(self) -> list[Constraint]:
        cs = [c for c in self.doc.constraints if self._valid(c)]
        if self._ghost is not None and self._valid(self._ghost):
            cs.append(self._ghost)
        return cs

    # ── Variable index (note 1) ───────────────────────────────────

    def _fixed_points(self) -> set[EntityId]:
        out = {e.id for e in self.doc.entities.values()
               if isinstance(e, Point) and e.fixed}
        for c in self.doc.constraints:
            if c.kind != "fix":
                continue
            for ref in c.refs:
                base = self._base_point(ref)
                if base:
                    out.add(base[0])
        return out

    def _build_index(self, active: list[Constraint]
                     ) -> tuple[dict[EntityId, int], set[EntityId]]:
        fixed = self._fixed_points()
        touched: set[EntityId] = set()
        for c in active:
            for ref in c.refs:
                base = self._base_point(ref)
                if base:
                    touched.add(base[0])
                    continue
                pair = self._line_points(ref)
                if pair:
                    for sub in pair:
                        b = self._base_point(sub)
                        if b:
                            touched.add(b[0])
        free = sorted(p for p in touched if p not in fixed)
        return ({pid: i for i, pid in enumerate(free)}, fixed)

    # ── Solve ─────────────────────────────────────────────────────

    def solve(self, max_iter: int = 120) -> SolveReport:
        active = self._active()
        index, _fixed = self._build_index(active)
        n_free = len(index)
        n_eqs = sum(_ROWS[c.kind] for c in active)

        self._apply_pre_passes()

        if n_free == 0:
            return SolveReport(status=DOFStatus.EMPTY, n_free_vars=0,
                               n_constraint_eqs=n_eqs,
                               message="No free geometry to solve.")
        if n_eqs == 0:
            return SolveReport(status=DOFStatus.UNDER_DETERMINED,
                               n_free_vars=n_free, n_constraint_eqs=0,
                               dof=n_free,
                               message=f"{n_free} degrees of freedom, "
                                       f"no constraints.")

        order = sorted(index, key=lambda pid: index[pid])
        x0 = np.array([v for pid in order
                       for v in (self.doc.entities[pid].x,
                                 self.doc.entities[pid].y)], dtype=float)

        def residuals(vec: np.ndarray) -> np.ndarray:
            return self._residuals(vec, index, active)

        try:
            from scipy.optimize import least_squares
            method = "lm" if n_eqs >= len(x0) else "trf"
            kwargs = dict(method=method, max_nfev=max_iter * (len(x0) + 1))
            if method == "trf":
                kwargs.update(x_scale="jac", ftol=1e-12, xtol=1e-12,
                              gtol=1e-12)
            result = least_squares(residuals, x0, **kwargs)
        except Exception as exc:                       # pragma: no cover
            logger.warning("Plate solve failed: %s", exc)
            return SolveReport(status=DOFStatus.INCONSISTENT,
                               n_free_vars=n_free, n_constraint_eqs=n_eqs,
                               converged=False, message=str(exc))

        for pid, i in index.items():
            pt = self.doc.entities[pid]
            pt.x = float(result.x[2 * i])
            pt.y = float(result.x[2 * i + 1])

        return self._report(result, n_free, n_eqs, active)

    def _apply_pre_passes(self) -> None:
        """``equal_radius`` propagates a diameter by assignment — a parameter,
        not a variable, so it never becomes a residual."""
        for c in self.doc.constraints:
            if c.kind != "equal_radius" or len(c.refs) < 2 or c.driven:
                continue
            lead = self._radius_of(c.refs[0])
            if lead is None:
                continue
            for ref in c.refs[1:]:
                ent = self.doc.entities.get(ref[0])
                if isinstance(ent, Well):
                    ent.style.diameter_mm = lead * 2.0
                elif isinstance(ent, PatternFeature):
                    if ref[1]:
                        self.doc.override(ent, ref[1]).diameter_mm = lead * 2.0
                    else:
                        ent.style.diameter_mm = lead * 2.0

    # ── Residuals ─────────────────────────────────────────────────

    def _xy(self, vec: np.ndarray, index: dict[EntityId, int],
            ref: Ref) -> tuple[float, float]:
        pid, dx, dy = self._base_point(ref)       # validity checked upstream
        i = index.get(pid)
        if i is None:
            pt = self.doc.entities[pid]
            return (pt.x + dx, pt.y + dy)
        return (vec[2 * i] + dx, vec[2 * i + 1] + dy)

    def _residuals(self, vec: np.ndarray, index: dict[EntityId, int],
                   active: list[Constraint]) -> np.ndarray:
        rows: list[float] = []
        for c in active:
            w = c.weight
            k = c.kind

            if k == "drag_ghost":
                x, y = self._xy(vec, index, c.refs[0])
                rows += [w * (x - self._ghost_target[0]),
                         w * (y - self._ghost_target[1])]
                continue
            if k in ("coincident", "concentric"):
                ax, ay = self._xy(vec, index, c.refs[0])
                bx, by = self._xy(vec, index, c.refs[1])
                rows += [w * (ax - bx), w * (ay - by)]
                continue
            if k == "horizontal":
                ay = self._xy(vec, index, c.refs[0])[1]
                by = self._xy(vec, index, c.refs[1])[1]
                rows.append(w * (ay - by))
                continue
            if k == "vertical":
                ax = self._xy(vec, index, c.refs[0])[0]
                bx = self._xy(vec, index, c.refs[1])[0]
                rows.append(w * (ax - bx))
                continue
            if k == "distance":
                ax, ay = self._xy(vec, index, c.refs[0])
                bx, by = self._xy(vec, index, c.refs[1])
                target = float(c.value or 0.0)
                if c.mode == "edge":
                    ra = self._radius_of(c.refs[0]) or 0.0
                    rb = self._radius_of(c.refs[1]) or 0.0
                    target += ra + rb
                # Squared form: smooth through coincidence.
                rows.append(w * ((ax - bx) ** 2 + (ay - by) ** 2
                                 - target ** 2))
                continue
            if k == "distance_to_datum":
                x, y = self._xy(vec, index, c.refs[0])
                rows.append(w * self._datum_residual(c, x, y))
                continue
            if k == "radial":
                x, y = self._xy(vec, index, c.refs[0])
                target = float(c.value or 0.0)
                if c.mode == "edge":
                    target += self._radius_of(c.refs[0]) or 0.0
                rows.append(w * (x * x + y * y - target ** 2))
                continue
            if k in ("point_on_line", "distance_to_line"):
                px, py = self._xy(vec, index, c.refs[0])
                a, b = self._line_points(c.refs[1])
                ax, ay = self._xy(vec, index, a)
                bx, by = self._xy(vec, index, b)
                ux, uy = bx - ax, by - ay
                cross = ux * (py - ay) - uy * (px - ax)
                if k == "point_on_line":
                    rows.append(w * cross)
                else:
                    length = math.hypot(ux, uy) or 1e-9
                    rows.append(w * (cross / length - float(c.value or 0.0)))
                continue
            if k in ("parallel", "perpendicular", "equal_length", "angle"):
                a1, a2 = self._line_points(c.refs[0])
                b1, b2 = self._line_points(c.refs[1])
                ax1, ay1 = self._xy(vec, index, a1)
                ax2, ay2 = self._xy(vec, index, a2)
                bx1, by1 = self._xy(vec, index, b1)
                bx2, by2 = self._xy(vec, index, b2)
                ux, uy = ax2 - ax1, ay2 - ay1
                vx, vy = bx2 - bx1, by2 - by1
                if k == "parallel":
                    rows.append(w * (ux * vy - uy * vx))
                elif k == "perpendicular":
                    rows.append(w * (ux * vx + uy * vy))
                elif k == "equal_length":
                    rows.append(w * ((ux * ux + uy * uy)
                                     - (vx * vx + vy * vy)))
                else:
                    want = math.radians(float(c.value or 0.0))
                    got = math.atan2(uy, ux) - math.atan2(vy, vx)
                    rows.append(w * math.atan2(math.sin(got - want),
                                               math.cos(got - want)))
                continue
            if k == "tangent":
                ax, ay = self._xy(vec, index, c.refs[0])
                bx, by = self._xy(vec, index, c.refs[1])
                ra = self._radius_of(c.refs[0]) or 0.0
                rb = self._radius_of(c.refs[1]) or 0.0
                want = abs(ra - rb) if c.mode == "internal" else (ra + rb)
                rows.append(w * ((ax - bx) ** 2 + (ay - by) ** 2 - want ** 2))
                continue
            if k == "symmetric":
                ax, ay = self._xy(vec, index, c.refs[0])
                bx, by = self._xy(vec, index, c.refs[1])
                l1, l2 = self._line_points(c.refs[2])
                lx1, ly1 = self._xy(vec, index, l1)
                lx2, ly2 = self._xy(vec, index, l2)
                ux, uy = lx2 - lx1, ly2 - ly1
                mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                # Midpoint on the axis, and AB perpendicular to it.
                rows.append(w * (ux * (my - ly1) - uy * (mx - lx1)))
                rows.append(w * ((bx - ax) * ux + (by - ay) * uy))
                continue
        return np.array(rows, dtype=float) if rows else np.zeros(0)

    def _datum_residual(self, c: Constraint, x: float, y: float) -> float:
        d = self.doc.boundary.datum_value(c.datum)
        target = float(c.value or 0.0)
        r = (self._radius_of(c.refs[0]) or 0.0) if c.mode == "edge" else 0.0
        if c.datum == "edge_left":
            return (x - r) - d - target
        if c.datum == "edge_right":
            return d - (x + r) - target
        if c.datum == "edge_top":
            return (y - r) - d - target
        if c.datum == "edge_bottom":
            return d - (y + r) - target
        if c.datum == "axis_v":
            return x - d - target
        if c.datum == "axis_h":
            return y - d - target
        if c.datum == "bore_wall":
            return d - (math.hypot(x, y) + r) - target
        return math.hypot(x, y) - target        # "origin"

    # ── Reporting ─────────────────────────────────────────────────

    def _report(self, result, n_free: int, n_eqs: int,
                active: list[Constraint]) -> SolveReport:
        res = float(np.linalg.norm(result.fun)) if result.fun.size else 0.0
        rank, skipped = self._rank(result.jac, n_free)
        dof = max(0, n_free - rank)

        if res > _INCONSISTENT_RESIDUAL_EPS * max(1, n_free):
            status = DOFStatus.INCONSISTENT
        elif dof > 0:
            status = DOFStatus.UNDER_DETERMINED
        else:
            status = DOFStatus.WELL_DETERMINED

        conflicts: list[int] = []
        if status == DOFStatus.INCONSISTENT:
            conflicts = self._blame(result.fun, active)

        msg = ""
        if skipped:
            msg = (f"Rank not computed above {_RANK_MAX_VARS} free variables; "
                   f"DOF is an upper bound.")
        return SolveReport(status=status, residual_norm=res,
                           n_free_vars=n_free, n_constraint_eqs=n_eqs,
                           rank=rank, dof=dof, conflicts=conflicts,
                           iterations=int(getattr(result, "nfev", 0)),
                           converged=bool(getattr(result, "success", True)),
                           message=msg)

    @staticmethod
    def _rank(jac, n_free: int) -> tuple[int, bool]:
        try:
            arr = np.asarray(jac, dtype=float)
            if arr.size == 0:
                return (0, False)
            if n_free > _RANK_MAX_VARS:
                return (min(arr.shape), True)
            if n_free > _RANK_QR_VARS:
                r = np.linalg.qr(arr, mode="r")
                diag = np.abs(np.diag(r))
                tol = max(arr.shape) * np.finfo(float).eps * (
                    diag.max() if diag.size else 1.0)
                return (int((diag > max(tol, 1e-8)).sum()), False)
            return (int(np.linalg.matrix_rank(arr, tol=1e-8)), False)
        except Exception:                              # pragma: no cover
            return (0, False)

    @staticmethod
    def _blame(fun, active: list[Constraint]) -> list[int]:
        """Top-3 constraint ids by residual row-norm.

        Correct only because the row cursor matches `_residuals` exactly —
        which is what `_valid()` guarantees, and what the old solver got wrong.
        """
        scored: list[tuple[float, int]] = []
        cursor = 0
        for c in active:
            n = _ROWS[c.kind]
            if n and cursor + n <= len(fun) and c.id > 0:
                scored.append((float(np.linalg.norm(fun[cursor:cursor + n])),
                               c.id))
            cursor += n
        scored.sort(reverse=True)
        return [cid for score, cid in scored[:3] if score > 1e-6]

    # ── Query helpers for the UI ──────────────────────────────────

    def seed_value(self, kind: str, refs: list[Ref], datum: str = "",
                   mode: str = "center") -> Optional[float]:
        """The value a new dimension should take from current geometry, so
        adding it never makes anything jump."""
        try:
            if kind == "distance" and len(refs) >= 2:
                ax, ay = self._static_xy(refs[0])
                bx, by = self._static_xy(refs[1])
                d = math.hypot(ax - bx, ay - by)
                if mode == "edge":
                    d -= (self._radius_of(refs[0]) or 0.0)
                    d -= (self._radius_of(refs[1]) or 0.0)
                return round(d, 4)
            if kind == "distance_to_datum" and refs:
                x, y = self._static_xy(refs[0])
                probe = Constraint(kind=kind, refs=refs, value=0.0,
                                   datum=datum, mode=mode)
                return round(self._datum_residual(probe, x, y), 4)
            if kind == "radial" and refs:
                x, y = self._static_xy(refs[0])
                d = math.hypot(x, y)
                if mode == "edge":
                    d += self._radius_of(refs[0]) or 0.0
                return round(d, 4)
        except Exception:                              # pragma: no cover
            return None
        return None

    def _static_xy(self, ref: Ref) -> tuple[float, float]:
        pid, dx, dy = self._base_point(ref)
        pt = self.doc.entities[pid]
        return (pt.x + dx, pt.y + dy)
