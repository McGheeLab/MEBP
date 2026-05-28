"""
PlateSketchSolver.py — Constraint solver for the parametric plate
designer (v7.4.5).

Approach
========

`scipy.optimize.least_squares` with Levenberg-Marquardt drives a vector
of free variables (the (x, y) of every non-fixed `Point`) toward
satisfying a set of residual equations contributed by the active
constraints.

Variable layout
---------------

Every non-fixed, non-grounded `Point` in the design contributes a pair
of free variables `(x, y)`. Wells reach the solver through their center
points; this keeps the constraint surface a function purely of point
coordinates.

Pre-solve passes
----------------

* `equal_radius` is enforced by direct value propagation, not as a
  residual. The constraint's first ref is the "leader"; subsequent refs
  are set equal to the leader's diameter.
* `concentric` on two Wells is treated identically to `coincident_pp`
  on their center points.

Drag
----

A `drag_ghost` constraint with `refs=[point_id]`, `value=None`, and a
`snapshot=(tx, ty)` plus large `weight` (1000) makes the dragged point
"want" to be at the cursor. Other constraints relax around it.

DOF reporting
-------------

After each solve, we compute `rank(J)` and compare against
`len(free_vars)`:

* `cost ≈ 0` + `rank == n_free` → WELL_DETERMINED
* `cost ≈ 0` + `rank  < n_free` → UNDER_DETERMINED (free vars remain)
* `cost  > 0`                   → INCONSISTENT (constraints conflict)

`SolveReport.conflicts` lists the constraint ids with the largest
residuals when inconsistent.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np
from scipy.optimize import least_squares, OptimizeResult

from SupportClasses.PlateDesign import (
    PlateDesign, Point, Well, Line, Constraint, EntityId,
)

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Solve report
# ═══════════════════════════════════════════════════════════════════

class DOFStatus(str, Enum):
    WELL_DETERMINED = "well_determined"
    UNDER_DETERMINED = "under_determined"
    INCONSISTENT = "inconsistent"
    EMPTY = "empty"   # No free variables → trivially solved


@dataclass
class SolveReport:
    status: DOFStatus = DOFStatus.WELL_DETERMINED
    residual_norm: float = 0.0
    n_free_vars: int = 0
    n_constraint_eqs: int = 0
    rank: int = 0
    dof: int = 0                # n_free_vars - rank
    conflicts: list[int] = field(default_factory=list)   # constraint ids
    iterations: int = 0
    converged: bool = True
    message: str = ""

    def __str__(self) -> str:
        return (
            f"SolveReport(status={self.status.value}, "
            f"res={self.residual_norm:.4g}, "
            f"vars={self.n_free_vars}, eqs={self.n_constraint_eqs}, "
            f"rank={self.rank}, dof={self.dof}, "
            f"conflicts={self.conflicts})"
        )


# Convergence thresholds.
_COST_EPS = 1e-9
_INCONSISTENT_RESIDUAL_EPS = 1e-4   # cost = 0.5 * Σr² thresholded → r ≈ √(2·1e-9) ≈ 4.5e-5


# ═══════════════════════════════════════════════════════════════════
# Solver
# ═══════════════════════════════════════════════════════════════════

class PlateSketchSolver:
    """Constraint solver bound to a single `PlateDesign`.

    A solver instance is short-lived — the canvas constructs one per
    interaction. Call `solve()` after batch edits, or use the
    `begin_drag` / `update_drag` / `end_drag` lifecycle while the user
    is actively dragging.
    """

    DRAG_GHOST_WEIGHT = 1000.0

    def __init__(self, design: PlateDesign):
        self.design = design
        self._drag_ghost: Optional[Constraint] = None

    # ── Static solve ──────────────────────────────────────────────

    def solve(self, max_iter: int = 50) -> SolveReport:
        """Re-run the solver from the design's current entity values."""
        self._apply_pre_passes()
        free_index, fixed_xy = self._build_var_index()
        if not free_index:
            return SolveReport(
                status=DOFStatus.EMPTY,
                residual_norm=0.0,
                n_free_vars=0,
                n_constraint_eqs=0,
                rank=0, dof=0,
                converged=True, iterations=0,
                message="No free variables",
            )

        x0 = self._pack_x0(free_index)
        active = [c for c in self.design.constraints
                  if c.kind not in ("ground", "fix", "equal_radius")]

        # Count residual rows up front so we can short-circuit when the
        # system is trivially under-determined (no constraints → no work
        # for the solver).
        n_eqs_predicted = sum(_residual_count(c) for c in active)
        n_free = len(free_index) * 2

        if n_eqs_predicted == 0:
            # No active constraints → design is fully under-determined
            # but trivially valid (every point sits where the user placed
            # it). Skip the solver entirely.
            return SolveReport(
                status=DOFStatus.UNDER_DETERMINED,
                residual_norm=0.0,
                n_free_vars=n_free,
                n_constraint_eqs=0,
                rank=0,
                dof=n_free,
                converged=True,
                iterations=0,
                message="No active residual constraints",
            )

        def residuals(x: np.ndarray) -> np.ndarray:
            return self._residuals(x, free_index, fixed_xy, active)

        # LM requires #residuals >= #vars. Use trust-region-reflective
        # (handles all shapes, supports bounds) when the system is
        # under-determined.
        method = "lm" if n_eqs_predicted >= n_free else "trf"

        try:
            result: OptimizeResult = least_squares(
                residuals, x0, method=method,
                max_nfev=max_iter * (len(x0) + 1),
            )
        except Exception as e:
            logger.warning(f"least_squares failed: {e}")
            return SolveReport(
                status=DOFStatus.INCONSISTENT,
                residual_norm=float("inf"),
                n_free_vars=n_free,
                n_constraint_eqs=n_eqs_predicted,
                rank=0, dof=n_free,
                converged=False, iterations=0,
                message=str(e),
            )

        # Write the solved positions back into the design.
        self._write_xs(result.x, free_index)

        return self._make_report(result, free_index, fixed_xy, active)

    # ── Drag lifecycle ────────────────────────────────────────────

    def begin_drag(
        self, point_id: EntityId, target_xy: tuple[float, float],
    ) -> SolveReport:
        """Start dragging `point_id`. Returns the immediate solve report."""
        self._drag_ghost = Constraint(
            id=-1,                       # transient, never persisted
            kind="drag_ghost",
            refs=[point_id],
            value=None,
            weight=self.DRAG_GHOST_WEIGHT,
            snapshot=target_xy,
        )
        self.design.constraints.append(self._drag_ghost)
        return self.solve()

    def update_drag(self, target_xy: tuple[float, float]) -> SolveReport:
        """Update the active drag target. Cheap re-solve."""
        if self._drag_ghost is None:
            return self.solve()
        self._drag_ghost.snapshot = target_xy
        return self.solve()

    def end_drag(self) -> SolveReport:
        """Remove the drag-ghost and finalize the drag."""
        if self._drag_ghost is not None:
            try:
                self.design.constraints.remove(self._drag_ghost)
            except ValueError:
                pass
            self._drag_ghost = None
        return self.solve()

    # ── Pre-passes (equal_radius, concentric → coincident) ────────

    def _apply_pre_passes(self) -> None:
        """Apply equal_radius value propagation and rewrite concentric.

        equal_radius: refs = [leader, follower, follower, …]. Followers
        adopt the leader's diameter (Wells) or radius (Circles).

        concentric on Wells: treated identically to coincident_pp on the
        wells' center points — no special handling here; the residual
        function looks up Well.center when given a Well id.
        """
        ents = self.design.entities
        for c in self.design.constraints:
            if c.kind != "equal_radius" or len(c.refs) < 2:
                continue
            leader_id = c.refs[0]
            leader = ents.get(leader_id)
            leader_d = self._entity_radius_x2(leader)
            if leader_d is None:
                continue
            for fid in c.refs[1:]:
                follower = ents.get(fid)
                if isinstance(follower, Well):
                    follower.diameter = leader_d
                # Circle → would set follower.radius; deferred (no Circle
                # entities in v7.4.5 shipping scope).

    @staticmethod
    def _entity_radius_x2(ent) -> Optional[float]:
        """Diameter of a Well (its constrainable scalar)."""
        if isinstance(ent, Well):
            return ent.diameter
        return None

    # ── Free var index + fixed points ─────────────────────────────

    def _build_var_index(
        self,
    ) -> tuple[dict[EntityId, tuple[int, int]], dict[EntityId, tuple[float, float]]]:
        """Walk Points in id-sorted order; non-fixed ones become free vars.

        Returns:
            free_index: point_id -> (x_var_idx, y_var_idx)
            fixed_xy: point_id -> (x, y)  (inlined as numeric literals)
        """
        fixed_xy = self._resolve_fixed_points()
        free_index: dict[EntityId, tuple[int, int]] = {}
        next_idx = 0
        for ent in sorted(self.design.entities.values(), key=lambda e: e.id):
            if not isinstance(ent, Point):
                continue
            if ent.id in fixed_xy:
                continue
            free_index[ent.id] = (next_idx, next_idx + 1)
            next_idx += 2
        return free_index, fixed_xy

    def _resolve_fixed_points(self) -> dict[EntityId, tuple[float, float]]:
        """Collect point ids whose (x, y) are constants.

        A point is fixed if (a) `Point.fixed=True`, or (b) it is the
        target of a `ground` constraint (pinned to (0,0)), or (c) the
        target of a `fix` constraint (pinned to the snapshot, or its
        current x/y if no snapshot was captured).
        """
        fixed: dict[EntityId, tuple[float, float]] = {}
        for ent in self.design.entities.values():
            if isinstance(ent, Point) and ent.fixed:
                fixed[ent.id] = (ent.x, ent.y)
        for c in self.design.constraints:
            if c.kind == "ground":
                for pid in c.refs:
                    fixed[pid] = (0.0, 0.0)
            elif c.kind == "fix":
                for pid in c.refs:
                    p = self.design.entities.get(pid)
                    if isinstance(p, Point):
                        snap = c.snapshot if c.snapshot else (p.x, p.y)
                        fixed[pid] = snap
        return fixed

    def _pack_x0(
        self, free_index: dict[EntityId, tuple[int, int]],
    ) -> np.ndarray:
        """Initial guess vector — pull from current Point positions."""
        x0 = np.zeros(len(free_index) * 2, dtype=float)
        for pid, (ix, iy) in free_index.items():
            p = self.design.entities[pid]
            assert isinstance(p, Point)
            x0[ix] = p.x
            x0[iy] = p.y
        return x0

    def _write_xs(
        self, x: np.ndarray, free_index: dict[EntityId, tuple[int, int]],
    ) -> None:
        for pid, (ix, iy) in free_index.items():
            p = self.design.entities[pid]
            assert isinstance(p, Point)
            p.x = float(x[ix])
            p.y = float(x[iy])

    # ── Residual function ─────────────────────────────────────────

    def _resolve_center_point(
        self, entity_id: EntityId,
    ) -> Optional[EntityId]:
        """Map a Well id to its center Point id; pass Points through."""
        ent = self.design.entities.get(entity_id)
        if isinstance(ent, Point):
            return ent.id
        if isinstance(ent, Well):
            return ent.center
        return None

    def _residuals(
        self,
        x: np.ndarray,
        free_index: dict[EntityId, tuple[int, int]],
        fixed_xy: dict[EntityId, tuple[float, float]],
        active: list[Constraint],
    ) -> np.ndarray:
        def get_xy(any_id: EntityId) -> tuple[float, float]:
            pid = self._resolve_center_point(any_id)
            if pid is None:
                return (0.0, 0.0)
            if pid in fixed_xy:
                return fixed_xy[pid]
            ix, iy = free_index[pid]
            return (float(x[ix]), float(x[iy]))

        out: list[float] = []
        for c in active:
            k = c.kind
            if k == "coincident_pp" or k == "concentric":
                if len(c.refs) < 2:
                    continue
                x1, y1 = get_xy(c.refs[0])
                x2, y2 = get_xy(c.refs[1])
                out.append(c.weight * (x1 - x2))
                out.append(c.weight * (y1 - y2))
            elif k == "distance_pp":
                if len(c.refs) < 2 or c.value is None:
                    continue
                x1, y1 = get_xy(c.refs[0])
                x2, y2 = get_xy(c.refs[1])
                d = c.value
                # Squared form — well-behaved through coincidence.
                out.append(c.weight * ((x1 - x2) ** 2 + (y1 - y2) ** 2 - d * d))
            elif k == "horizontal":
                if len(c.refs) < 2:
                    continue
                _, y1 = get_xy(c.refs[0])
                _, y2 = get_xy(c.refs[1])
                out.append(c.weight * (y1 - y2))
            elif k == "vertical":
                if len(c.refs) < 2:
                    continue
                x1, _ = get_xy(c.refs[0])
                x2, _ = get_xy(c.refs[1])
                out.append(c.weight * (x1 - x2))
            elif k == "drag_ghost":
                if not c.refs or c.snapshot is None:
                    continue
                pid = self._resolve_center_point(c.refs[0])
                if pid is None or pid in fixed_xy:
                    continue
                ix, iy = free_index[pid]
                tx, ty = c.snapshot
                out.append(c.weight * (float(x[ix]) - tx))
                out.append(c.weight * (float(x[iy]) - ty))
            elif k == "point_on_line":
                # refs = [point_or_well_id, line_id]. Residual is the
                # (un-normalized) signed cross product (P-A) × (B-A);
                # zero iff P lies on the infinite line through A,B.
                if len(c.refs) < 2:
                    continue
                line = self.design.entities.get(c.refs[1])
                if not isinstance(line, Line):
                    continue
                ax, ay = get_xy(line.p1)
                bx, by = get_xy(line.p2)
                px, py = get_xy(c.refs[0])
                out.append(
                    c.weight * ((px - ax) * (by - ay)
                                - (py - ay) * (bx - ax)))
            elif k == "parallel":
                # refs = [line_a, line_b]. Cross product of direction
                # vectors; zero iff parallel.
                if len(c.refs) < 2:
                    continue
                la = self.design.entities.get(c.refs[0])
                lb = self.design.entities.get(c.refs[1])
                if not (isinstance(la, Line) and isinstance(lb, Line)):
                    continue
                a1x, a1y = get_xy(la.p1); b1x, b1y = get_xy(la.p2)
                a2x, a2y = get_xy(lb.p1); b2x, b2y = get_xy(lb.p2)
                d1x, d1y = b1x - a1x, b1y - a1y
                d2x, d2y = b2x - a2x, b2y - a2y
                out.append(c.weight * (d1x * d2y - d1y * d2x))
            elif k == "perpendicular":
                # Dot product; zero iff perpendicular.
                if len(c.refs) < 2:
                    continue
                la = self.design.entities.get(c.refs[0])
                lb = self.design.entities.get(c.refs[1])
                if not (isinstance(la, Line) and isinstance(lb, Line)):
                    continue
                a1x, a1y = get_xy(la.p1); b1x, b1y = get_xy(la.p2)
                a2x, a2y = get_xy(lb.p1); b2x, b2y = get_xy(lb.p2)
                d1x, d1y = b1x - a1x, b1y - a1y
                d2x, d2y = b2x - a2x, b2y - a2y
                out.append(c.weight * (d1x * d2x + d1y * d2y))
            elif k == "equal_length":
                # refs = [line_a, line_b]. Squared-length form so the
                # Jacobian stays smooth through coincidence.
                if len(c.refs) < 2:
                    continue
                la = self.design.entities.get(c.refs[0])
                lb = self.design.entities.get(c.refs[1])
                if not (isinstance(la, Line) and isinstance(lb, Line)):
                    continue
                a1x, a1y = get_xy(la.p1); b1x, b1y = get_xy(la.p2)
                a2x, a2y = get_xy(lb.p1); b2x, b2y = get_xy(lb.p2)
                len_a_sq = (b1x - a1x) ** 2 + (b1y - a1y) ** 2
                len_b_sq = (b2x - a2x) ** 2 + (b2y - a2y) ** 2
                out.append(c.weight * (len_a_sq - len_b_sq))
            elif k == "tangent_cc":
                # External tangent for two Wells: distance(centers) =
                # r1 + r2. Squared form.
                if len(c.refs) < 2:
                    continue
                w1 = self.design.entities.get(c.refs[0])
                w2 = self.design.entities.get(c.refs[1])
                if not (isinstance(w1, Well) and isinstance(w2, Well)):
                    continue
                c1x, c1y = get_xy(w1.center)
                c2x, c2y = get_xy(w2.center)
                target = (w1.diameter + w2.diameter) / 2.0
                out.append(
                    c.weight * ((c1x - c2x) ** 2 + (c1y - c2y) ** 2
                                - target * target))
            elif k == "symmetric_pp":
                # refs = [p1_or_well, p2_or_well, axis_line]. Constrains
                # p1 and p2 to be reflections across the axis line.
                # Residuals:
                #   1. midpoint of (p1,p2) lies on the axis line
                #   2. (p1-p2) is perpendicular to the axis line
                if len(c.refs) < 3:
                    continue
                axis = self.design.entities.get(c.refs[2])
                if not isinstance(axis, Line):
                    continue
                p1x, p1y = get_xy(c.refs[0])
                p2x, p2y = get_xy(c.refs[1])
                ax, ay = get_xy(axis.p1)
                bx, by = get_xy(axis.p2)
                mx, my = (p1x + p2x) * 0.5, (p1y + p2y) * 0.5
                # Midpoint on line: cross product = 0
                out.append(
                    c.weight * ((mx - ax) * (by - ay)
                                - (my - ay) * (bx - ax)))
                # (p1-p2) perpendicular to line: dot product = 0
                dpx, dpy = p1x - p2x, p1y - p2y
                dlx, dly = bx - ax, by - ay
                out.append(c.weight * (dpx * dlx + dpy * dly))
            elif k in ("dist_left_edge", "dist_top_edge"):
                # refs = [well_id]; value = distance (mm) from the plate
                # edge. Edges derive from the design footprint with A1 at
                # scene origin (0,0):
                #   left edge x = -a1_offset_x ; top edge y = -a1_offset_y
                if not c.refs or c.value is None:
                    continue
                well = self.design.entities.get(c.refs[0])
                if not isinstance(well, Well):
                    continue
                px, py = get_xy(c.refs[0])    # resolves well → center
                r = well.diameter / 2.0 if c.mode == "edge" else 0.0
                if k == "dist_left_edge":
                    left_x = -self.design.a1_offset_x
                    out.append(c.weight * ((px - r - left_x) - c.value))
                else:  # dist_top_edge
                    top_y = -self.design.a1_offset_y
                    out.append(c.weight * ((py - r - top_y) - c.value))
            # equal_radius, ground, fix: not solver residuals.
        return np.asarray(out, dtype=float)

    # ── Report assembly ───────────────────────────────────────────

    def _make_report(
        self,
        result: OptimizeResult,
        free_index: dict[EntityId, tuple[int, int]],
        fixed_xy: dict[EntityId, tuple[float, float]],
        active: list[Constraint],
    ) -> SolveReport:
        n_free = len(free_index) * 2
        n_eqs = result.fun.size

        # Rank via the Jacobian scipy returns at the optimum.
        jac = result.jac
        if hasattr(jac, "toarray"):
            jac_arr = jac.toarray()
        else:
            jac_arr = np.asarray(jac)
        if jac_arr.size == 0:
            rank = 0
        else:
            rank = int(np.linalg.matrix_rank(jac_arr, tol=1e-8))

        dof = max(0, n_free - rank)
        residual_norm = float(np.linalg.norm(result.fun))

        # Classify
        if n_free == 0:
            status = DOFStatus.EMPTY
        elif residual_norm > _INCONSISTENT_RESIDUAL_EPS * max(1.0, n_free):
            status = DOFStatus.INCONSISTENT
        elif dof > 0:
            status = DOFStatus.UNDER_DETERMINED
        else:
            status = DOFStatus.WELL_DETERMINED

        # Conflict ranking — top-3 constraints by per-row residual norm.
        conflicts: list[int] = []
        if status == DOFStatus.INCONSISTENT:
            row_norms: list[tuple[int, float]] = []
            row = 0
            for c in active:
                # Compute the number of rows this constraint contributed.
                count = _residual_count(c)
                if count == 0:
                    continue
                norm = float(np.linalg.norm(result.fun[row:row + count]))
                row_norms.append((c.id, norm))
                row += count
            row_norms.sort(key=lambda t: -t[1])
            conflicts = [cid for cid, _ in row_norms[:3]]

        return SolveReport(
            status=status,
            residual_norm=residual_norm,
            n_free_vars=n_free,
            n_constraint_eqs=n_eqs,
            rank=rank,
            dof=dof,
            conflicts=conflicts,
            iterations=int(getattr(result, "nfev", 0)),
            converged=bool(result.status > 0),
            message=str(getattr(result, "message", "")),
        )


# ═══════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════

def _residual_count(c: Constraint) -> int:
    """How many scalar residual rows a constraint contributes."""
    k = c.kind
    if k in ("coincident_pp", "concentric", "drag_ghost", "symmetric_pp"):
        return 2
    if k in ("distance_pp", "horizontal", "vertical",
             "point_on_line", "parallel", "perpendicular",
             "equal_length", "tangent_cc",
             "dist_left_edge", "dist_top_edge"):
        return 1
    return 0
