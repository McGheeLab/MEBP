"""Shared result types for the geometric constraint solvers.

Three solvers report through these: ``PlateSketchSolver`` (the v7.4.x plate
designer), ``SketchConstraintSolver`` (Print Builder → Sketch), and
``PlateSolver`` (the v7.12 plate document). Keeping one definition is what lets
a single properties-panel widget render a DOF badge for any of them.

They lived in ``PlateSketchSolver`` until v7.12; that module re-exports them, so
``from SupportClasses.PlateSketchSolver import SolveReport, DOFStatus`` keeps
working for existing callers.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


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
