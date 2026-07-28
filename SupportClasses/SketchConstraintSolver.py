"""
SketchConstraintSolver.py — parametric constraint solver for the Print
Builder Sketch tool (v7.5.x).

Mirrors the proven :class:`SupportClasses.PlateSketchSolver.PlateSketchSolver`
architecture (`scipy.optimize.least_squares`, LM when square/over-determined /
TRF when under-determined, residual rows per constraint kind, a transient
high-weight ``drag_ghost`` pin during interactive drags, DOF classification
via the Jacobian rank) — but operates **directly on SketchShape DOFs**
instead of Point entities:

=============  =========================================
shape kind     free variables
=============  =========================================
line/polygon   every vertex (2N)
circle         cx, cy, radius
ellipse        cx, cy, rx, ry
rect           cx, cy, width, height
travel         cx, cy
region         not constrainable (baked raster)
=============  =========================================

Only shapes **referenced by at least one constraint** enter the variable
vector, so unconstrained sketches never touch the solver (pay-for-play) and
solving never disturbs unconstrained geometry.

Anchors resolve to smooth functions of the variables (see
:class:`SupportClasses.SketchTrajectory.SketchConstraint` for the anchor
vocabulary; rect corners are ``(cx ± w/2, cy ± h/2)``).

Unlike the plate solver, the drag ghost is held BY THE SOLVER (never appended
to the model), so an undo snapshot taken mid-drag can never serialize it.
"""

from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np
from scipy.optimize import least_squares, OptimizeResult

from SupportClasses.PlateSketchSolver import SolveReport, DOFStatus
from SupportClasses.SketchTrajectory import Sketch, SketchConstraint, SketchShape

logger = logging.getLogger(__name__)

# Convergence thresholds (match PlateSketchSolver).
_COST_EPS = 1e-9
_INCONSISTENT_RESIDUAL_EPS = 1e-4

# Minimum size the write-back clamps to, matching the canvas' own minimums so
# a degenerate solve can't produce zero/negative geometry.
_MIN_SIZE_MM = 0.05

# Scalar-variable layout per center-based shape kind (after cx, cy).
_SCALAR_FIELDS = {
    "circle": ("radius",),
    "ellipse": ("rx", "ry"),
    "rect": ("width", "height"),
    "travel": (),
}


class SketchConstraintSolver:
    """Constraint solver bound to a single :class:`Sketch`.

    Short-lived — the canvas constructs one per interaction. Call
    :meth:`solve` after batch edits, or drive the
    :meth:`begin_drag` / :meth:`update_drag` / :meth:`end_drag` lifecycle
    while the user drags constrained geometry.
    """

    DRAG_GHOST_WEIGHT = 1000.0

    def __init__(self, sketch: Sketch):
        self.sketch = sketch
        self._drag_ghost: Optional[SketchConstraint] = None
        self._drag_target: tuple[float, float] = (0.0, 0.0)
        self._fixed_cache: set[int] = set()
        self._var_index: dict[tuple, int] = {}

    # ── Public API ────────────────────────────────────────────────

    def solve(self, max_iter: int = 50) -> SolveReport:
        """Re-solve from the sketch's current geometry; write results back."""
        self._fixed_cache = self._fixed_ids()
        # Only structurally-valid constraints reach the solver, so the
        # predicted row count is EXACT (conflict attribution + the LM/TRF
        # method choice depend on it).
        active = [c for c in self._all_constraints()
                  if c.kind != "fix" and self._constraint_valid(c)]
        var_index, layouts = self._build_var_index()
        n_free = sum(len(slots) for slots in layouts.values())

        if n_free == 0:
            return SolveReport(
                status=DOFStatus.EMPTY, residual_norm=0.0,
                n_free_vars=0, n_constraint_eqs=0, rank=0, dof=0,
                converged=True, iterations=0, message="No free variables")

        n_eqs_predicted = sum(_residual_count(c) for c in active)
        if n_eqs_predicted == 0:
            return SolveReport(
                status=DOFStatus.UNDER_DETERMINED, residual_norm=0.0,
                n_free_vars=n_free, n_constraint_eqs=0, rank=0, dof=n_free,
                converged=True, iterations=0,
                message="No active residual constraints")

        x0 = self._pack_x0(layouts)

        def residuals(x: np.ndarray) -> np.ndarray:
            return self._residuals(x, layouts, active)

        method = "lm" if n_eqs_predicted >= n_free else "trf"
        try:
            result: OptimizeResult = least_squares(
                residuals, x0, method=method,
                max_nfev=max_iter * (len(x0) + 1))
        except Exception as e:  # pragma: no cover - scipy failure path
            logger.warning(f"sketch constraint solve failed: {e}")
            return SolveReport(
                status=DOFStatus.INCONSISTENT, residual_norm=float("inf"),
                n_free_vars=n_free, n_constraint_eqs=n_eqs_predicted,
                rank=0, dof=n_free, converged=False, iterations=0,
                message=str(e))

        self._write_xs(result.x, layouts)
        return self._make_report(result, n_free, active)

    # ── Drag lifecycle ────────────────────────────────────────────

    def begin_drag(self, shape_id: int, anchor: str,
                   target_xy: tuple[float, float]) -> SolveReport:
        """Start dragging ``anchor`` of shape ``shape_id`` toward
        ``target_xy``. The ghost is held solver-side (never in the model)."""
        self._drag_ghost = SketchConstraint(
            id=-1, kind="drag_ghost",
            refs=[[int(shape_id), str(anchor)]],
            weight=self.DRAG_GHOST_WEIGHT)
        self._drag_target = (float(target_xy[0]), float(target_xy[1]))
        return self.solve()

    def update_drag(self, target_xy: tuple[float, float]) -> SolveReport:
        if self._drag_ghost is None:
            return self.solve()
        self._drag_target = (float(target_xy[0]), float(target_xy[1]))
        return self.solve()

    def end_drag(self) -> SolveReport:
        self._drag_ghost = None
        return self.solve()

    # ── Variable packing ──────────────────────────────────────────

    def _all_constraints(self) -> list[SketchConstraint]:
        cs = list(self.sketch.constraints)
        if self._drag_ghost is not None:
            cs.append(self._drag_ghost)
        return cs

    def _fixed_ids(self) -> set[int]:
        return {int(sid)
                for c in self.sketch.constraints if c.kind == "fix"
                for sid in c.shape_ids()}

    def _constraint_valid(self, c: SketchConstraint) -> bool:
        """Structural validation — mirrors exactly what ``_residuals`` can
        evaluate, so every validated constraint contributes its predicted
        row count (no silent skips)."""
        k = c.kind
        refs = c.refs
        kinds = [self._shape_kind(int(sid)) for sid, _ in refs]
        if any(kd == "" for kd in kinds):        # dangling shape ref
            return False
        if k in ("coincident", "concentric", "horizontal", "vertical"):
            return len(refs) >= 2
        if k in ("parallel", "perpendicular", "equal_length"):
            return (len(refs) >= 2 and kinds[0] == "line"
                    and kinds[1] == "line")
        if k == "equal_radius":
            return (len(refs) >= 2 and kinds[0] == "circle"
                    and kinds[1] == "circle")
        if k == "distance":
            return len(refs) >= 2 and c.value is not None
        if k == "radius":
            return bool(refs) and kinds[0] == "circle" and c.value is not None
        if k == "tangent":
            if len(refs) < 2:
                return False
            pair = {kinds[0], kinds[1]}
            return pair == {"line", "circle"} or pair == {"circle"}
        if k == "point_on":
            return len(refs) >= 2 and kinds[1] in ("line", "circle")
        if k == "drag_ghost":
            return bool(refs) and int(refs[0][0]) not in self._fixed_cache
        return False

    def _build_var_index(self):
        """Map each referenced, non-fixed, constrainable shape to its slice of
        the variable vector.

        Returns ``(var_index, layouts)`` where ``layouts[shape_id]`` is a list
        of ``(field, sub_index)`` slot descriptors in vector order:
        vertex shapes → ``("pt", k)`` pairs of slots (x then y); center shapes
        → ``("cx",0), ("cy",0)`` then scalar fields. ``var_index`` maps
        ``(shape_id, field, sub)`` → vector position.
        """
        referenced: set[int] = set()
        for c in self._all_constraints():
            referenced |= c.shape_ids()
        fixed = self._fixed_ids()

        var_index: dict[tuple, int] = {}
        layouts: dict[int, list[tuple]] = {}
        nxt = 0
        for sid in sorted(referenced - fixed):
            sh = self.sketch.shape_by_id(sid)
            if sh is None or sh.kind == "region":
                continue
            slots: list[tuple] = []
            if sh.kind in ("line", "polygon"):
                for k in range(len(sh.points)):
                    var_index[(sid, "px", k)] = nxt
                    var_index[(sid, "py", k)] = nxt + 1
                    slots += [("px", k), ("py", k)]
                    nxt += 2
            elif sh.kind in _SCALAR_FIELDS:
                var_index[(sid, "cx", 0)] = nxt
                var_index[(sid, "cy", 0)] = nxt + 1
                slots += [("cx", 0), ("cy", 0)]
                nxt += 2
                for f in _SCALAR_FIELDS[sh.kind]:
                    var_index[(sid, f, 0)] = nxt
                    slots.append((f, 0))
                    nxt += 1
            if slots:
                layouts[sid] = slots
        self._var_index = var_index
        return var_index, layouts

    def _pack_x0(self, layouts: dict[int, list[tuple]]) -> np.ndarray:
        vals: list[float] = []
        for sid, slots in layouts.items():
            sh = self.sketch.shape_by_id(sid)
            for field_name, sub in slots:
                if field_name == "px":
                    vals.append(float(sh.points[sub][0]))
                elif field_name == "py":
                    vals.append(float(sh.points[sub][1]))
                else:
                    vals.append(float(getattr(sh, field_name)))
        return np.asarray(vals, dtype=float)

    def _write_xs(self, x: np.ndarray, layouts: dict[int, list[tuple]]):
        pos = 0
        for sid, slots in layouts.items():
            sh = self.sketch.shape_by_id(sid)
            new_pts = list(sh.points)
            for field_name, sub in slots:
                v = float(x[pos])
                pos += 1
                if field_name == "px":
                    new_pts[sub] = (v, new_pts[sub][1])
                elif field_name == "py":
                    new_pts[sub] = (new_pts[sub][0], v)
                elif field_name in ("radius", "rx", "ry", "width", "height"):
                    setattr(sh, field_name, max(_MIN_SIZE_MM, v))
                else:
                    setattr(sh, field_name, v)
            if sh.kind in ("line", "polygon"):
                sh.points = new_pts

    # ── Anchor / scalar resolution (smooth in the variables) ──────

    def _val(self, x, sid: int, field_name: str, sub: int = 0) -> float:
        """Current value of one DOF — from the vector when free, else the
        model (fixed / unreferenced shapes are constants)."""
        idx = self._var_index.get((sid, field_name, sub))
        if idx is not None:
            return float(x[idx])
        sh = self.sketch.shape_by_id(sid)
        if sh is None:
            return 0.0
        if field_name == "px":
            return float(sh.points[sub][0]) if sub < len(sh.points) else 0.0
        if field_name == "py":
            return float(sh.points[sub][1]) if sub < len(sh.points) else 0.0
        return float(getattr(sh, field_name, 0.0))

    def _anchor_xy(self, x, sid: int, anchor: str) -> tuple[float, float]:
        sid = int(sid)
        sh = self.sketch.shape_by_id(sid)
        if sh is None:
            return (0.0, 0.0)
        a = str(anchor)
        if a.startswith("p") and a[1:].isdigit():
            k = int(a[1:])
            return (self._val(x, sid, "px", k), self._val(x, sid, "py", k))
        if a.startswith("c") and a[1:].isdigit() and sh.kind == "rect":
            k = int(a[1:])                       # 0=TL 1=TR 2=BR 3=BL
            sx = -1.0 if k in (0, 3) else 1.0
            sy = -1.0 if k in (0, 1) else 1.0
            return (self._val(x, sid, "cx") + sx * self._val(x, sid, "width") / 2.0,
                    self._val(x, sid, "cy") + sy * self._val(x, sid, "height") / 2.0)
        if a == "mid" and sh.kind == "line" and len(sh.points) >= 2:
            return ((self._val(x, sid, "px", 0) + self._val(x, sid, "px", 1)) / 2.0,
                    (self._val(x, sid, "py", 0) + self._val(x, sid, "py", 1)) / 2.0)
        if a == "centroid" and sh.kind in ("line", "polygon") and sh.points:
            n = len(sh.points)
            return (sum(self._val(x, sid, "px", k) for k in range(n)) / n,
                    sum(self._val(x, sid, "py", k) for k in range(n)) / n)
        # "center" / "shape" / fallback
        if sh.kind in ("line", "polygon") and sh.points:
            n = len(sh.points)
            return (sum(self._val(x, sid, "px", k) for k in range(n)) / n,
                    sum(self._val(x, sid, "py", k) for k in range(n)) / n)
        return (self._val(x, sid, "cx"), self._val(x, sid, "cy"))

    def _line_dir(self, x, sid: int) -> tuple[float, float]:
        """Direction vector of a line shape (p1 − p0)."""
        return (self._val(x, sid, "px", 1) - self._val(x, sid, "px", 0),
                self._val(x, sid, "py", 1) - self._val(x, sid, "py", 0))

    def _shape_kind(self, sid: int) -> str:
        sh = self.sketch.shape_by_id(sid)
        return getattr(sh, "kind", "") if sh is not None else ""

    # ── Residuals ─────────────────────────────────────────────────

    def _residuals(self, x: np.ndarray, layouts, active) -> np.ndarray:
        out: list[float] = []
        for c in active:
            k = c.kind
            w = float(c.weight)
            refs = c.refs
            if k in ("coincident", "concentric"):
                if len(refs) < 2:
                    continue
                x1, y1 = self._anchor_xy(x, *refs[0])
                x2, y2 = self._anchor_xy(x, *refs[1])
                out.append(w * (x1 - x2))
                out.append(w * (y1 - y2))
            elif k == "horizontal":
                if len(refs) < 2:
                    continue
                _, y1 = self._anchor_xy(x, *refs[0])
                _, y2 = self._anchor_xy(x, *refs[1])
                out.append(w * (y1 - y2))
            elif k == "vertical":
                if len(refs) < 2:
                    continue
                x1, _ = self._anchor_xy(x, *refs[0])
                x2, _ = self._anchor_xy(x, *refs[1])
                out.append(w * (x1 - x2))
            elif k == "parallel":
                if len(refs) < 2:
                    continue
                d1x, d1y = self._line_dir(x, int(refs[0][0]))
                d2x, d2y = self._line_dir(x, int(refs[1][0]))
                out.append(w * (d1x * d2y - d1y * d2x))
            elif k == "perpendicular":
                if len(refs) < 2:
                    continue
                d1x, d1y = self._line_dir(x, int(refs[0][0]))
                d2x, d2y = self._line_dir(x, int(refs[1][0]))
                out.append(w * (d1x * d2x + d1y * d2y))
            elif k == "equal_length":
                if len(refs) < 2:
                    continue
                d1x, d1y = self._line_dir(x, int(refs[0][0]))
                d2x, d2y = self._line_dir(x, int(refs[1][0]))
                out.append(w * ((d1x * d1x + d1y * d1y)
                                - (d2x * d2x + d2y * d2y)))
            elif k == "equal_radius":
                if len(refs) < 2:
                    continue
                r1 = self._val(x, int(refs[0][0]), "radius")
                r2 = self._val(x, int(refs[1][0]), "radius")
                out.append(w * (r1 - r2))
            elif k == "distance":
                if len(refs) < 2 or c.value is None:
                    continue
                x1, y1 = self._anchor_xy(x, *refs[0])
                x2, y2 = self._anchor_xy(x, *refs[1])
                d = float(c.value)
                # Squared form — well-behaved through coincidence.
                out.append(w * ((x1 - x2) ** 2 + (y1 - y2) ** 2 - d * d))
            elif k == "radius":
                if not refs or c.value is None:
                    continue
                r = self._val(x, int(refs[0][0]), "radius")
                out.append(w * (r - float(c.value)))
            elif k == "tangent":
                if len(refs) < 2:
                    continue
                out.extend(self._tangent_rows(x, c))
            elif k == "point_on":
                if len(refs) < 2:
                    continue
                px, py = self._anchor_xy(x, *refs[0])
                curve_sid = int(refs[1][0])
                ck = self._shape_kind(curve_sid)
                if ck == "line":
                    ax = self._val(x, curve_sid, "px", 0)
                    ay = self._val(x, curve_sid, "py", 0)
                    bx = self._val(x, curve_sid, "px", 1)
                    by = self._val(x, curve_sid, "py", 1)
                    out.append(w * ((px - ax) * (by - ay)
                                    - (py - ay) * (bx - ax)))
                elif ck == "circle":
                    cx = self._val(x, curve_sid, "cx")
                    cy = self._val(x, curve_sid, "cy")
                    r = self._val(x, curve_sid, "radius")
                    out.append(w * ((px - cx) ** 2 + (py - cy) ** 2 - r * r))
            elif k == "drag_ghost":
                if not refs:
                    continue
                sid = int(refs[0][0])
                ax, ay = self._anchor_xy(x, sid, refs[0][1])
                tx, ty = self._drag_target
                out.append(w * (ax - tx))
                out.append(w * (ay - ty))
            # "fix": no residual — its DOFs are excluded from the variables.
        return np.asarray(out, dtype=float)

    def _tangent_rows(self, x, c: SketchConstraint) -> list[float]:
        """Tangent residual — line↔circle or circle↔circle (mode-resolved)."""
        w = float(c.weight)
        (sid_a, _), (sid_b, _) = c.refs[0], c.refs[1]
        ka, kb = self._shape_kind(int(sid_a)), self._shape_kind(int(sid_b))
        # Normalize order: line first when mixed.
        if ka == "circle" and kb == "line":
            sid_a, sid_b, ka, kb = sid_b, sid_a, kb, ka
        if ka == "line" and kb == "circle":
            ax = self._val(x, int(sid_a), "px", 0)
            ay = self._val(x, int(sid_a), "py", 0)
            bx = self._val(x, int(sid_a), "px", 1)
            by = self._val(x, int(sid_a), "py", 1)
            cx = self._val(x, int(sid_b), "cx")
            cy = self._val(x, int(sid_b), "cy")
            r = self._val(x, int(sid_b), "radius")
            dx, dy = bx - ax, by - ay
            len_sq = max(dx * dx + dy * dy, 1e-12)
            cross = (cx - ax) * dy - (cy - ay) * dx
            # dist(center, line)² − r², in mm² (smooth; degenerate line guarded)
            return [w * (cross * cross / len_sq - r * r)]
        if ka == "circle" and kb == "circle":
            c1x = self._val(x, int(sid_a), "cx")
            c1y = self._val(x, int(sid_a), "cy")
            r1 = self._val(x, int(sid_a), "radius")
            c2x = self._val(x, int(sid_b), "cx")
            c2y = self._val(x, int(sid_b), "cy")
            r2 = self._val(x, int(sid_b), "radius")
            target = (r1 - r2) if c.mode == "internal" else (r1 + r2)
            return [w * ((c1x - c2x) ** 2 + (c1y - c2y) ** 2
                         - target * target)]
        return []

    # ── Report assembly (mirrors PlateSketchSolver) ───────────────

    def _make_report(self, result: OptimizeResult, n_free: int,
                     active: list[SketchConstraint]) -> SolveReport:
        n_eqs = result.fun.size
        jac = result.jac
        jac_arr = jac.toarray() if hasattr(jac, "toarray") else np.asarray(jac)
        rank = 0 if jac_arr.size == 0 else int(
            np.linalg.matrix_rank(jac_arr, tol=1e-8))
        dof = max(0, n_free - rank)
        residual_norm = float(np.linalg.norm(result.fun))

        if n_free == 0:
            status = DOFStatus.EMPTY
        elif residual_norm > _INCONSISTENT_RESIDUAL_EPS * max(1.0, n_free):
            status = DOFStatus.INCONSISTENT
        elif dof > 0:
            status = DOFStatus.UNDER_DETERMINED
        else:
            status = DOFStatus.WELL_DETERMINED

        conflicts: list[int] = []
        if status == DOFStatus.INCONSISTENT:
            row_norms: list[tuple[int, float]] = []
            row = 0
            for c in active:
                count = _residual_count(c)
                if count == 0:
                    continue
                norm = float(np.linalg.norm(result.fun[row:row + count]))
                row_norms.append((c.id, norm))
                row += count
            row_norms.sort(key=lambda t: -t[1])
            conflicts = [cid for cid, _ in row_norms[:3]]

        return SolveReport(
            status=status, residual_norm=residual_norm,
            n_free_vars=n_free, n_constraint_eqs=n_eqs,
            rank=rank, dof=dof, conflicts=conflicts,
            iterations=int(getattr(result, "nfev", 0)),
            converged=bool(result.status > 0),
            message=str(getattr(result, "message", "")))


# ═══════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════

def _residual_count(c: SketchConstraint) -> int:
    """Scalar residual rows a constraint contributes."""
    k = c.kind
    if k in ("coincident", "concentric", "drag_ghost"):
        return 2
    if k in ("horizontal", "vertical", "parallel", "perpendicular",
             "equal_length", "equal_radius", "distance", "radius",
             "tangent", "point_on"):
        return 1
    return 0


def tangent_mode_for(sh_a: SketchShape, sh_b: SketchShape) -> str:
    """Choose external/internal tangency for two circles from their CURRENT
    geometry (the nearer of the two targets), so adding the constraint keeps
    the arrangement the user drew. Non-circle pairs return ""."""
    if getattr(sh_a, "kind", "") != "circle" or \
            getattr(sh_b, "kind", "") != "circle":
        return ""
    d = math.hypot(sh_a.cx - sh_b.cx, sh_a.cy - sh_b.cy)
    ext = abs(d - (sh_a.radius + sh_b.radius))
    inn = abs(d - abs(sh_a.radius - sh_b.radius))
    return "internal" if inn < ext else "external"
