"""
Plate-bottom Z plane — the "where is the glass, anywhere on the plate" model.

The plate bottom is a flat but TILTED plane. Before this module the tilt was
measured by the calibration page and then thrown away: prints resolved plate
bottom from a SINGLE scalar and ``StageController.print_height_to_zref`` took no
(x, y), so per-well tilt compensation was structurally impossible.

THE REPRESENTATION IS AN ANCHORED GRADIENT, NOT ``a*x + b*y + c``
-----------------------------------------------------------------
    plate_bottom_z(x, y) = z0_zref_mm
                         + sx_mm_per_mm * (x_um - x0_um) / 1000
                         + sy_mm_per_mm * (y_um - y0_um) / 1000

where ``(x0_um, y0_um, z0_zref_mm)`` **is a real measured touch-off point** — the
same number the operator taught and that ``get_plate_bottom_z()`` returns.

This is a safety property, not a style choice:

* Degrading to the legacy single-scalar behaviour is exact and free: with
  ``sx = sy = 0`` the plane returns ``z0_zref_mm`` everywhere, bit-for-bit.
* ``c`` in the slope-intercept form is the value at the frame ORIGIN, which is
  meaningless as soon as the origin moves (a Set Zero, or plate-local vs stage
  coordinates). ``z0`` is the value at a physical point and is invariant under
  any translation of the frame.
* It makes a whole class of disaster impossible. A real machine on this branch
  had a saved plate-local plane with ``c = 5.600`` while its taught
  ``plate_bottom_z`` was ``0.090`` — adopting that intercept would have driven
  the needle 5.5 mm deeper, into the glass. In anchored form the fit can only
  contribute a *gradient*; it can never move Z at the anchor.
* "Re-anchor after a re-mount" becomes trivial: re-touch one well, replace
  ``z0``, keep ``sx``/``sy``.

FRAME — stated once, enforced by naming
---------------------------------------
XY is **absolute stage µm**; Z is **zero-ref mm**; slopes are **mm per mm**
(dimensionless), so a plausibility gate can be a clean "≤ 0.005 ≈ 0.3°".

Absolute stage XY (not zero-ref) because that is the frame taught data already
lives in — ``SafetyLimits`` documents the XY envelope as absolute and
"unaffected by Set Zero", and CLAUDE.md is explicit that taught/warped positions
are already absolute stage µm and must never be re-signed. Storing XY absolutely
means a later Set Zero cannot silently shift the plane.

Plate-LOCAL mm is deliberately NOT used. Evaluating at an arbitrary stage XY
would require inverting the ``affine_tps`` plate warp — not something a safety
clamp should do — and the focal-readout mode measures at locations that are not
wells at all and therefore have no plate-local coordinate.

Every public name carries its unit. An identifier without one is a bug.

This module is GUI-free and imports numpy lazily (only inside the fitting
functions) so that importing the plane type and evaluator costs nothing.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field, replace
from typing import ClassVar, Iterable, Sequence

logger = logging.getLogger(__name__)


# The canonical frame tag. Persisted with every plane and validated on load, so
# that if this ever changes the old data fails LOUD instead of being read in the
# wrong frame.
PLANE_FRAME = "stage_um/zref_mm"

# Slope sanity. This gate exists to catch a UNIT SLIP, not to adjudicate whether
# a measured tilt is believable — a mm-vs-µm mix-up is a 1000× error, so a limit
# ~2.5× above any realistic tilt catches it by a factor of hundreds while never
# arguing with the hardware. 0.05 mm/mm ≈ 2.9°.
#
# Deliberately NOT tighter: this machine's own measured tilt is 0.0199 mm/mm
# (≈1.14°, ~1.15 mm across a 24-well plate's row span). An earlier 0.005 mm/mm
# (0.3°) limit here rejected that real measurement outright, which would have
# made the whole feature unusable on the rig it was written for.
MAX_PLAUSIBLE_SLOPE_MM_PER_MM = 0.05

# Total plate-bottom variation across the plate. THIS is the physical arbiter:
# it bounds the thing that actually matters (how far the datum moves) rather than
# an angle, and it is what catches a single bad touch-off skewing the fit.
MAX_PLAUSIBLE_SPAN_MM = 2.0

# Residual / hold-out tolerance for a plane we are willing to print against.
DEFAULT_RESIDUAL_TOL_MM = 0.05

# Three touch points that are nearly collinear define a garbage gradient while
# still fitting "perfectly". Require a real triangle.
MIN_TRIANGLE_AREA_MM2 = 25.0


# ── points ────────────────────────────────────────────────────────────────

@dataclass(frozen=True, kw_only=True)
class ZPlanePoint:
    """One measured plate-bottom sample.

    ``x_stage_um`` / ``y_stage_um`` are ABSOLUTE stage µm as read from
    ``StageController.get_xy_position(cached=False)`` at the moment of the
    touch-off — never re-derived from ``plate.get_well_position()``, which is
    plate-local and would drag ``plate_axis_sign`` into the math.

    ``z_zref_mm`` is the live encoder Z at the touch, in the zero-ref frame. It
    is None for a focal-readout point until the plane is anchored.
    """
    label: str = ""
    kind: str = "well"                  # "well" | "free"
    x_stage_um: float = 0.0
    y_stage_um: float = 0.0
    z_zref_mm: float | None = None
    source: str = "needle_touch"        # "needle_touch" | "focal_readout"
    surface: str = "well_glass_bottom"  # | "plate_top" | "other"
    focal_raw: float | None = None
    focal_unit: str | None = None       # "mm" | "um"
    focal_mm: float | None = None       # normalised at record time
    focus_axis_um_at_record: float | None = None
    needle_conf: float | None = None
    focus_score: float | None = None
    recorded_at: str = ""

    def to_dict(self) -> dict:
        d = {
            "label": self.label,
            "kind": self.kind,
            "x_um": self.x_stage_um,
            "y_um": self.y_stage_um,
            "z_zref_mm": self.z_zref_mm,
            "source": self.source,
            "surface": self.surface,
            "recorded_at": self.recorded_at,
        }
        # Emit optional keys only when set, so a plain needle-touch point stays
        # a compact, diff-friendly record.
        for k, v in (("focal_raw", self.focal_raw),
                     ("focal_unit", self.focal_unit),
                     ("focal_mm", self.focal_mm),
                     ("focus_axis_um_at_record", self.focus_axis_um_at_record),
                     ("needle_conf", self.needle_conf),
                     ("focus_score", self.focus_score)):
            if v is not None:
                d[k] = v
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "ZPlanePoint":
        if not isinstance(d, dict):
            raise ValueError("ZPlanePoint.from_dict needs a dict")
        return cls(
            label=str(d.get("label", "")),
            kind=str(d.get("kind", "well")),
            x_stage_um=float(d.get("x_um", 0.0)),
            y_stage_um=float(d.get("y_um", 0.0)),
            z_zref_mm=(None if d.get("z_zref_mm") is None
                       else float(d["z_zref_mm"])),
            source=str(d.get("source", "needle_touch")),
            surface=str(d.get("surface", "well_glass_bottom")),
            focal_raw=_opt_float(d.get("focal_raw")),
            focal_unit=(None if d.get("focal_unit") is None
                        else str(d["focal_unit"])),
            focal_mm=_opt_float(d.get("focal_mm")),
            focus_axis_um_at_record=_opt_float(d.get("focus_axis_um_at_record")),
            needle_conf=_opt_float(d.get("needle_conf")),
            focus_score=_opt_float(d.get("focus_score")),
            recorded_at=str(d.get("recorded_at", "")),
        )


def _opt_float(v) -> float | None:
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def focal_to_mm(focal_raw: float, unit: str) -> float:
    """Normalise a focus-stage readout to mm.

    The operator's readout is in real units but which one is a per-rig fact, so
    both the raw value and its unit are stored alongside the normalised mm — a
    unit mistake then stays diagnosable after the fact instead of silently
    becoming a 1000× tilt.
    """
    u = (unit or "").strip().lower()
    if u in ("mm", "millimeter", "millimetre"):
        return float(focal_raw)
    if u in ("um", "µm", "micron", "micrometer", "micrometre"):
        return float(focal_raw) / 1000.0
    raise ValueError(f"unknown focal unit {unit!r} (expected 'mm' or 'um')")


# ── the plane ─────────────────────────────────────────────────────────────

@dataclass(frozen=True, kw_only=True)
class PlateZPlane:
    """Plate-bottom Z as an anchored gradient. See the module docstring.

    ``status`` is set by the consumer (``StageController.set_plate_z_plane``),
    never by the producer: a fit does not get to declare itself trustworthy.
    """
    FRAME: ClassVar[str] = PLANE_FRAME

    # geometry — the only fields the evaluator reads
    x0_um: float
    y0_um: float
    z0_zref_mm: float
    sx_mm_per_mm: float = 0.0
    sy_mm_per_mm: float = 0.0

    frame: str = PLANE_FRAME
    mode: str = "needle_touch"           # | "focal_readout"

    # quality
    points: tuple[ZPlanePoint, ...] = ()
    num_points: int = 0
    r_squared: float = 0.0
    residual_max_mm: float = 0.0
    holdout_error_mm: float | None = None
    degenerate: bool = False

    # provenance — checked on load; a mismatch invalidates the FIT
    zero_z_mm_at_fit: float = 0.0
    zero_xy_um: tuple[float, float] | None = None
    z_up_sign_at_fit: float | None = None
    print_z_dir_at_fit: float | None = None
    plate_key: str | None = None
    plate_flip_180: bool | None = None
    needle_type: str | None = None
    needle_bore_um: float | None = None
    needle_tip_length_mm: float | None = None
    plate_bottom_z_source: str | None = None   # "taught" | "estimated" | "restored"
    camobj: str | None = None
    needle_template_key: str | None = None
    focal_sign: int = 1
    verify: dict | None = None
    fitted_at: str | None = None

    # trust — owned by the consumer
    status: str = "unvalidated"           # "active" | "rejected" | "unvalidated"
    reject_reason: str = ""

    # ── evaluation ────────────────────────────────────────────────────
    def z_zref_mm_at_stage_um(self, x_stage_um: float,
                              y_stage_um: float) -> float:
        """Plate-bottom Z (zero-ref mm) at an ABSOLUTE stage µm position."""
        return plate_plane_z_zref_mm(self, x_stage_um, y_stage_um)

    def z_zref_mm_at_stage_mm(self, x_stage_mm: float,
                              y_stage_mm: float) -> float:
        """Plate-bottom Z (zero-ref mm) at an ABSOLUTE stage mm position."""
        return plate_plane_z_zref_mm(self, float(x_stage_mm) * 1000.0,
                                     float(y_stage_mm) * 1000.0)

    def delta_from_anchor_mm(self, x_stage_um: float,
                             y_stage_um: float) -> float:
        """How much deeper/shallower than the anchor this position is (mm)."""
        return (self.z_zref_mm_at_stage_um(x_stage_um, y_stage_um)
                - self.z0_zref_mm)

    def is_level(self) -> bool:
        """True when the plane carries no tilt (⇒ identical to the scalar)."""
        return self.sx_mm_per_mm == 0.0 and self.sy_mm_per_mm == 0.0

    def corners_z_zref_mm(self, bbox_um: Sequence[float]) -> list[float]:
        """Plate-bottom Z at the four corners of an absolute-µm bbox."""
        x0, y0, x1, y1 = (float(v) for v in bbox_um)
        return [self.z_zref_mm_at_stage_um(x, y)
                for x in (x0, x1) for y in (y0, y1)]

    def span_mm(self, bbox_um: Sequence[float]) -> float:
        """Total plate-bottom variation across a bbox (mm, always ≥ 0)."""
        zs = self.corners_z_zref_mm(bbox_um)
        return max(zs) - min(zs)

    def extremes_zref(self, bbox_um: Sequence[float],
                      z_up_sign: float) -> tuple[float, float]:
        """``(shallowest, deepest)`` plate-bottom Z over a bbox, zero-ref mm.

        "Shallowest" means highest in the PHYSICAL height frame, so the
        comparison is made on ``z_up_sign * z`` — never on the raw zero-ref
        number, which is inverted on ME3B V1 (``ZDIR = -1``, the needle descends
        as raw Z increases).
        """
        up = float(z_up_sign) or 1.0
        zs = self.corners_z_zref_mm(bbox_um)
        shallowest = max(zs, key=lambda z: up * z)
        deepest = min(zs, key=lambda z: up * z)
        return (shallowest, deepest)

    def is_inside(self, x_stage_um: float, y_stage_um: float,
                  bbox_um: Sequence[float] | None = None,
                  margin_um: float = 5000.0) -> bool:
        """Is this position within the taught region (+ margin)?

        Extrapolating a 3-point tilt far past where it was measured is exactly
        how a tilt fit turns into a crash, so consumers fall back to the scalar
        outside this.
        """
        box = bbox_um if bbox_um is not None else self.points_bbox_um()
        if box is None:
            return False
        x0, y0, x1, y1 = (float(v) for v in box)
        m = abs(float(margin_um))
        return ((min(x0, x1) - m) <= float(x_stage_um) <= (max(x0, x1) + m)
                and (min(y0, y1) - m) <= float(y_stage_um) <= (max(y0, y1) + m))

    def points_bbox_um(self) -> tuple[float, float, float, float] | None:
        """Bounding box of the teach points, absolute stage µm."""
        if not self.points:
            return None
        xs = [p.x_stage_um for p in self.points]
        ys = [p.y_stage_um for p in self.points]
        return (min(xs), min(ys), max(xs), max(ys))

    def re_anchored(self, *, x0_um: float, y0_um: float,
                    z0_zref_mm: float) -> "PlateZPlane":
        """Same tilt, new anchor — the "re-mounted the same plate type" case.

        Holding ``sx``/``sy`` and replacing only the anchor is the whole reason
        the anchored form is worth having: one touch-off re-levels the datum
        without re-measuring the tilt.
        """
        return replace(self, x0_um=float(x0_um), y0_um=float(y0_um),
                       z0_zref_mm=float(z0_zref_mm),
                       status="unvalidated", reject_reason="")

    def describe(self) -> str:
        n = self.num_points or len(self.points)
        s = (f"plate bottom {self.z0_zref_mm:+.4f} mm at "
             f"({self.x0_um:.0f}, {self.y0_um:.0f}) µm · tilt "
             f"{self.sx_mm_per_mm:+.6f}/{self.sy_mm_per_mm:+.6f} mm/mm · "
             f"{n} pts")
        if self.residual_max_mm:
            s += f" · max resid {self.residual_max_mm * 1000.0:.0f} µm"
        if self.holdout_error_mm is not None:
            s += f" · hold-out {self.holdout_error_mm * 1000.0:.0f} µm"
        return s

    # ── serialisation ─────────────────────────────────────────────────
    def to_dict(self) -> dict:
        return {
            "version": 1,
            "frame": self.frame,
            "mode": self.mode,
            "x0_um": self.x0_um,
            "y0_um": self.y0_um,
            "z0_zref_mm": self.z0_zref_mm,
            "sx_mm_per_mm": self.sx_mm_per_mm,
            "sy_mm_per_mm": self.sy_mm_per_mm,
            "points": [p.to_dict() for p in self.points],
            "num_points": self.num_points,
            "r_squared": self.r_squared,
            "residual_max_mm": self.residual_max_mm,
            "holdout_error_mm": self.holdout_error_mm,
            "degenerate": self.degenerate,
            "focal_sign": self.focal_sign,
            "verify": self.verify,
            "provenance": {
                "zero_z_mm_at_fit": self.zero_z_mm_at_fit,
                "zero_xy_um": (list(self.zero_xy_um)
                               if self.zero_xy_um is not None else None),
                "z_up_sign_at_fit": self.z_up_sign_at_fit,
                "print_z_dir_at_fit": self.print_z_dir_at_fit,
                "plate_key": self.plate_key,
                "plate_flip_180": self.plate_flip_180,
                "needle_type": self.needle_type,
                "needle_bore_um": self.needle_bore_um,
                "needle_tip_length_mm": self.needle_tip_length_mm,
                "plate_bottom_z_source": self.plate_bottom_z_source,
                "camobj": self.camobj,
                "needle_template_key": self.needle_template_key,
                "fitted_at": self.fitted_at,
            },
        }

    @classmethod
    def from_dict(cls, d: dict) -> "PlateZPlane | None":
        """Rebuild from a persisted dict, or None if it is not ours.

        An unrecognised ``frame`` is refused outright rather than read in the
        wrong frame — that is the whole point of tagging it. In particular a
        legacy plate-local ``{a, b, c}`` block can never satisfy this, which is
        what stops it from being silently adopted.
        """
        if not isinstance(d, dict):
            return None
        frame = str(d.get("frame", ""))
        if frame != PLANE_FRAME:
            logger.error(
                "PlateZPlane.from_dict: refusing a plane in frame %r "
                "(expected %r) — not loading it", frame, PLANE_FRAME)
            return None
        prov = d.get("provenance") or {}
        try:
            pts = tuple(ZPlanePoint.from_dict(p)
                        for p in (d.get("points") or [])
                        if isinstance(p, dict))
            zxy = prov.get("zero_xy_um")
            return cls(
                x0_um=float(d["x0_um"]),
                y0_um=float(d["y0_um"]),
                z0_zref_mm=float(d["z0_zref_mm"]),
                sx_mm_per_mm=float(d.get("sx_mm_per_mm", 0.0)),
                sy_mm_per_mm=float(d.get("sy_mm_per_mm", 0.0)),
                frame=frame,
                mode=str(d.get("mode", "needle_touch")),
                points=pts,
                num_points=int(d.get("num_points", len(pts))),
                r_squared=float(d.get("r_squared", 0.0)),
                residual_max_mm=float(d.get("residual_max_mm", 0.0)),
                holdout_error_mm=_opt_float(d.get("holdout_error_mm")),
                degenerate=bool(d.get("degenerate", False)),
                focal_sign=int(d.get("focal_sign", 1) or 1),
                verify=(d.get("verify") if isinstance(d.get("verify"), dict)
                        else None),
                zero_z_mm_at_fit=float(prov.get("zero_z_mm_at_fit", 0.0)),
                zero_xy_um=((float(zxy[0]), float(zxy[1]))
                            if isinstance(zxy, (list, tuple)) and len(zxy) >= 2
                            else None),
                z_up_sign_at_fit=_opt_float(prov.get("z_up_sign_at_fit")),
                print_z_dir_at_fit=_opt_float(prov.get("print_z_dir_at_fit")),
                plate_key=(None if prov.get("plate_key") is None
                           else str(prov["plate_key"])),
                plate_flip_180=(None if prov.get("plate_flip_180") is None
                                else bool(prov["plate_flip_180"])),
                needle_type=(None if prov.get("needle_type") is None
                             else str(prov["needle_type"])),
                needle_bore_um=_opt_float(prov.get("needle_bore_um")),
                needle_tip_length_mm=_opt_float(prov.get("needle_tip_length_mm")),
                plate_bottom_z_source=(
                    None if prov.get("plate_bottom_z_source") is None
                    else str(prov["plate_bottom_z_source"])),
                camobj=(None if prov.get("camobj") is None
                        else str(prov["camobj"])),
                needle_template_key=(
                    None if prov.get("needle_template_key") is None
                    else str(prov["needle_template_key"])),
                fitted_at=(None if prov.get("fitted_at") is None
                           else str(prov["fitted_at"])),
                # Trust is never restored from disk — it is re-decided on load.
                status="unvalidated",
                reject_reason="",
            )
        except (KeyError, TypeError, ValueError) as e:
            logger.error("PlateZPlane.from_dict: malformed plane (%s)", e)
            return None


def plate_plane_z_zref_mm(plane, x_stage_um: float,
                          y_stage_um: float) -> float:
    """THE evaluator: absolute stage µm → plate-bottom Z in zero-ref mm.

    Kept as a free function taking a duck-typed ``plane`` so that
    ``StageController`` can evaluate without importing the dataclass and so that
    a plain dict-shaped plane (the job-stamped copy) works too. This is the ONLY
    place the anchored-gradient formula is written down.
    """
    z0 = float(_pget(plane, "z0_zref_mm"))
    x0 = float(_pget(plane, "x0_um"))
    y0 = float(_pget(plane, "y0_um"))
    sx = float(_pget(plane, "sx_mm_per_mm", 0.0) or 0.0)
    sy = float(_pget(plane, "sy_mm_per_mm", 0.0) or 0.0)
    return (z0
            + sx * (float(x_stage_um) - x0) / 1000.0
            + sy * (float(y_stage_um) - y0) / 1000.0)


def job_plane_z_zref_mm(job_plane: dict, x_zref_mm: float,
                        y_zref_mm: float) -> float:
    """Evaluate the JOB-STAMPED plane form: zero-ref mm XY → zero-ref mm Z.

    A print job carries the plane pre-converted to zero-ref mm XY, because that
    is the frame the print path already works in (``well_positions_mm``,
    ``MOVE_XY``, ``print_z_height``). Its keys are deliberately named
    ``x0_mm``/``y0_mm`` — NOT ``x0_um``/``y0_um`` — so that handing a job plane
    to :func:`plate_plane_z_zref_mm` (or vice versa) raises instead of silently
    computing a 1000×-wrong offset.
    """
    z0 = float(job_plane["z0_zref_mm"])
    x0 = float(job_plane["x0_mm"])
    y0 = float(job_plane["y0_mm"])
    sx = float(job_plane.get("sx_mm_per_mm", 0.0) or 0.0)
    sy = float(job_plane.get("sy_mm_per_mm", 0.0) or 0.0)
    return z0 + sx * (float(x_zref_mm) - x0) + sy * (float(y_zref_mm) - y0)


def _pget(plane, name: str, default=None):
    """Read a field from a dataclass-like or dict-like plane."""
    if isinstance(plane, dict):
        if name in plane:
            return plane[name]
        if default is None and name not in plane:
            raise KeyError(name)
        return default
    return getattr(plane, name, default)


# ── geometry helpers ──────────────────────────────────────────────────────

def triangle_max_area_mm2(points_xy_um: Iterable[Sequence[float]]) -> float:
    """Largest triangle area (mm²) spanned by any three of these points.

    A cheap, UI-side degeneracy pre-check: three nearly-collinear touch points
    fit a plane "perfectly" (R² ≡ 1 at n = 3) while defining a garbage gradient,
    so the run is refused before the operator spends time on it.
    """
    pts = [(float(p[0]) / 1000.0, float(p[1]) / 1000.0)
           for p in points_xy_um]
    n = len(pts)
    if n < 3:
        return 0.0
    best = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                (ax, ay), (bx, by), (cx, cy) = pts[i], pts[j], pts[k]
                area = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0
                best = max(best, area)
    return best


def tilt_is_plausible(sx_mm_per_mm: float, sy_mm_per_mm: float,
                      bbox_um: Sequence[float] | None = None,
                      max_slope: float = MAX_PLAUSIBLE_SLOPE_MM_PER_MM,
                      max_span_mm: float = MAX_PLAUSIBLE_SPAN_MM,
                      ) -> tuple[bool, str]:
    """Is this tilt physically believable? ``(ok, reason)``.

    MANDATORY in both calibration modes, not advisory: a mm-vs-µm unit slip
    produces a 1000× slope, and a single bad touch-off produces an implausible
    span. Either would otherwise be adopted as a print datum.
    """
    sx, sy = float(sx_mm_per_mm), float(sy_mm_per_mm)
    if not (math.isfinite(sx) and math.isfinite(sy)):
        return (False, "tilt is not finite")
    if abs(sx) > max_slope or abs(sy) > max_slope:
        deg = math.degrees(math.atan(max(abs(sx), abs(sy))))
        return (False,
                f"implausible tilt {sx:+.5f}/{sy:+.5f} mm/mm (~{deg:.2f}°, "
                f"limit {max_slope} mm/mm) — check the measurement units")
    if bbox_um is not None:
        x0, y0, x1, y1 = (float(v) for v in bbox_um)
        span = (abs(sx) * abs(x1 - x0) + abs(sy) * abs(y1 - y0)) / 1000.0
        if span > max_span_mm:
            return (False,
                    f"plate-bottom span {span:.2f} mm over the plate exceeds "
                    f"{max_span_mm} mm")
    return (True, "")


# ── fitting ───────────────────────────────────────────────────────────────

@dataclass(frozen=True, kw_only=True)
class GradientFit:
    """Result of fitting a gradient through a fixed anchor."""
    sx_mm_per_mm: float
    sy_mm_per_mm: float
    r_squared: float
    rms_residual_mm: float
    residual_max_mm: float
    residuals_mm: tuple[float, ...]
    singular_values: tuple[float, ...]
    rank: int
    degenerate: bool
    reason: str = ""


def fit_gradient_through_anchor(
        anchor_xy_um: Sequence[float],
        anchor_z_zref_mm: float,
        others: Sequence[tuple[float, float, float]],
) -> GradientFit | None:
    """Least-squares gradient constrained to pass through the anchor exactly.

    ``others`` are ``(x_stage_um, y_stage_um, z_zref_mm)``. Solving for the
    gradient of the *differences* from the anchor — rather than fitting a free
    plane and then reading its value at the anchor — is what makes the anchor
    exact by construction. That exactness is the safety property described in
    the module docstring; it is not an optimisation detail.

    Returns None when the gradient is underdetermined (fewer than two
    independent directions away from the anchor).
    """
    import numpy as np                      # local: keep import cost off callers

    ax, ay = float(anchor_xy_um[0]), float(anchor_xy_um[1])
    az = float(anchor_z_zref_mm)

    dx, dy, dz = [], [], []
    for (x, y, z) in others:
        if z is None:
            continue
        dx.append((float(x) - ax) / 1000.0)      # mm
        dy.append((float(y) - ay) / 1000.0)
        dz.append(float(z) - az)
    if len(dz) < 2:
        return None

    A = np.column_stack([np.array(dx), np.array(dy)])
    b = np.array(dz)
    sol, _res, rank, sv = np.linalg.lstsq(A, b, rcond=None)
    sx, sy = float(sol[0]), float(sol[1])

    svt = tuple(float(v) for v in sv)
    # Rank/conditioning is the real degeneracy signal. WellBottomDetector.fit_plane
    # discards it, which is why three collinear points there produce a silently
    # wrong tilt behind an R² of exactly 1.0.
    degenerate = (rank < 2
                  or not svt
                  or svt[-1] < 1e-9 * max(svt[0], 1e-30))

    pred = A @ np.array([sx, sy])
    resid = b - pred
    residuals = tuple(float(r) for r in resid)
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((b - float(np.mean(b))) ** 2))
    r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 1.0
    rms = math.sqrt(ss_res / len(residuals)) if residuals else 0.0

    return GradientFit(
        sx_mm_per_mm=sx,
        sy_mm_per_mm=sy,
        r_squared=float(r_squared),
        rms_residual_mm=float(rms),
        residual_max_mm=(max(abs(r) for r in residuals) if residuals else 0.0),
        residuals_mm=residuals,
        singular_values=svt,
        rank=int(rank),
        degenerate=bool(degenerate),
        reason=("gradient underdetermined — the touch points are collinear "
                "with the anchor" if degenerate else ""),
    )


def holdout_error_mm(points: Sequence[ZPlanePoint],
                     anchor_index: int = 0) -> float | None:
    """Leave-one-out prediction error (mm), or None if there is too little data.

    This — not R² — is the acceptance gate. With exactly three points the
    anchor-constrained fit has two unknowns and two equations, so it is exact
    and R² is identically 1.0: it cannot detect a bad touch-off. (That is
    precisely the ``r_squared: 1.0`` sitting in the machine's saved
    calibration.) Predicting a point the fit never saw can.

    Needs ≥ 4 points: dropping one must still leave two independent directions.
    """
    usable = [p for p in points if p.z_zref_mm is not None]
    if len(usable) < 4 or not (0 <= anchor_index < len(usable)):
        return None
    anchor = usable[anchor_index]
    worst = 0.0
    for i, held in enumerate(usable):
        if i == anchor_index:
            continue                          # the anchor is exact by definition
        rest = [(p.x_stage_um, p.y_stage_um, p.z_zref_mm)
                for j, p in enumerate(usable)
                if j != i and j != anchor_index]
        fit = fit_gradient_through_anchor(
            (anchor.x_stage_um, anchor.y_stage_um), anchor.z_zref_mm, rest)
        if fit is None or fit.degenerate:
            return None
        pred = (anchor.z_zref_mm
                + fit.sx_mm_per_mm * (held.x_stage_um - anchor.x_stage_um) / 1000.0
                + fit.sy_mm_per_mm * (held.y_stage_um - anchor.y_stage_um) / 1000.0)
        worst = max(worst, abs(pred - held.z_zref_mm))
    return worst


def _resolve_anchor_index(points: Sequence[ZPlanePoint],
                          anchor_label: str | None) -> int:
    if anchor_label:
        for i, p in enumerate(points):
            if p.label == anchor_label:
                return i
    return 0


def from_needle_touches(*, points: Sequence[ZPlanePoint],
                        anchor_label: str | None = None,
                        provenance: dict | None = None,
                        ) -> tuple["PlateZPlane | None", str]:
    """Fit a plane from needle touch-offs. ``(plane, reason)``.

    The anchor's measured Z becomes ``z0`` verbatim, so the plane reproduces the
    operator's taught plate-bottom exactly at that well.
    """
    usable = [p for p in points if p.z_zref_mm is not None]
    if len(usable) < 3:
        return (None, f"need ≥3 touch points, have {len(usable)}")

    surfaces = {p.surface for p in usable}
    if len(surfaces) > 1:
        return (None,
                "touch points mix surfaces (" + ", ".join(sorted(surfaces))
                + ") — they would measure different planes")

    area = triangle_max_area_mm2([(p.x_stage_um, p.y_stage_um) for p in usable])
    if area < MIN_TRIANGLE_AREA_MM2:
        return (None,
                f"touch points are nearly collinear (largest triangle "
                f"{area:.1f} mm², need {MIN_TRIANGLE_AREA_MM2:.0f} mm²)")

    ai = _resolve_anchor_index(usable, anchor_label)
    anchor = usable[ai]
    others = [(p.x_stage_um, p.y_stage_um, p.z_zref_mm)
              for j, p in enumerate(usable) if j != ai]
    fit = fit_gradient_through_anchor(
        (anchor.x_stage_um, anchor.y_stage_um), anchor.z_zref_mm, others)
    if fit is None:
        return (None, "gradient underdetermined")
    if fit.degenerate:
        return (None, fit.reason or "gradient underdetermined")

    xs = [p.x_stage_um for p in usable]
    ys = [p.y_stage_um for p in usable]
    ok, why = tilt_is_plausible(fit.sx_mm_per_mm, fit.sy_mm_per_mm,
                                bbox_um=(min(xs), min(ys), max(xs), max(ys)))
    if not ok:
        return (None, why)

    return (_build(anchor=anchor, fit=fit, points=usable, mode="needle_touch",
                   provenance=provenance,
                   holdout=holdout_error_mm(usable, ai)), "")


def from_focal_readings(*, points: Sequence[ZPlanePoint],
                        anchor: ZPlanePoint,
                        focal_sign: int = 1,
                        provenance: dict | None = None,
                        ) -> tuple["PlateZPlane | None", str]:
    """Fit the tilt from focus-stage readouts, anchored by ONE needle touch.

    The focal readings give the plate's *shape*: fitting a gradient over
    ``focal_sign * focal_mm`` recovers the tilt but carries the focus stage's
    own arbitrary offset. The needle touch supplies the datum, so the resulting
    plane passes exactly through it — which is what "the needle touch anchors
    all the locations" means.

    ``focal_sign`` cannot be derived from one touch (one equation, one unknown
    offset), so it must be measured or declared by the caller. The 4th-point
    verification is what catches getting it backwards.
    """
    usable = [p for p in points if p.focal_mm is not None]
    if len(usable) < 3:
        return (None, f"need ≥3 focal readings, have {len(usable)}")
    if anchor is None or anchor.z_zref_mm is None:
        return (None, "no needle-touch anchor — the focal plane has no datum")

    surfaces = {p.surface for p in usable}
    if len(surfaces) > 1:
        return (None,
                "focal readings mix surfaces (" + ", ".join(sorted(surfaces))
                + ") — they would measure different planes")

    area = triangle_max_area_mm2([(p.x_stage_um, p.y_stage_um) for p in usable])
    if area < MIN_TRIANGLE_AREA_MM2:
        return (None,
                f"focal locations are nearly collinear (largest triangle "
                f"{area:.1f} mm², need {MIN_TRIANGLE_AREA_MM2:.0f} mm²)")

    sign = 1 if int(focal_sign or 1) >= 0 else -1
    # Anchor the GRADIENT fit on the first focal reading (any choice gives the
    # same slopes); the datum comes from the needle touch below.
    ref = usable[0]
    others = [(p.x_stage_um, p.y_stage_um, sign * p.focal_mm)
              for p in usable[1:]]
    fit = fit_gradient_through_anchor(
        (ref.x_stage_um, ref.y_stage_um), sign * ref.focal_mm, others)
    if fit is None:
        return (None, "gradient underdetermined")
    if fit.degenerate:
        return (None, fit.reason or "gradient underdetermined")

    xs = [p.x_stage_um for p in usable]
    ys = [p.y_stage_um for p in usable]
    ok, why = tilt_is_plausible(fit.sx_mm_per_mm, fit.sy_mm_per_mm,
                                bbox_um=(min(xs), min(ys), max(xs), max(ys)))
    if not ok:
        return (None, why)

    plane = _build(anchor=anchor, fit=fit,
                   points=list(usable) + [anchor], mode="focal_readout",
                   provenance=provenance, holdout=None)
    return (replace(plane, focal_sign=sign), "")


def _build(*, anchor: ZPlanePoint, fit: GradientFit,
           points: Sequence[ZPlanePoint], mode: str,
           provenance: dict | None, holdout: float | None) -> PlateZPlane:
    prov = dict(provenance or {})
    zxy = prov.get("zero_xy_um")
    return PlateZPlane(
        x0_um=float(anchor.x_stage_um),
        y0_um=float(anchor.y_stage_um),
        z0_zref_mm=float(anchor.z_zref_mm),
        sx_mm_per_mm=fit.sx_mm_per_mm,
        sy_mm_per_mm=fit.sy_mm_per_mm,
        mode=mode,
        points=tuple(points),
        num_points=len(points),
        r_squared=fit.r_squared,
        residual_max_mm=fit.residual_max_mm,
        holdout_error_mm=holdout,
        degenerate=fit.degenerate,
        zero_z_mm_at_fit=float(prov.get("zero_z_mm_at_fit", 0.0) or 0.0),
        zero_xy_um=((float(zxy[0]), float(zxy[1]))
                    if isinstance(zxy, (list, tuple)) and len(zxy) >= 2
                    else None),
        z_up_sign_at_fit=_opt_float(prov.get("z_up_sign_at_fit")),
        print_z_dir_at_fit=_opt_float(prov.get("print_z_dir_at_fit")),
        plate_key=(None if prov.get("plate_key") is None
                   else str(prov["plate_key"])),
        plate_flip_180=(None if prov.get("plate_flip_180") is None
                        else bool(prov["plate_flip_180"])),
        needle_type=(None if prov.get("needle_type") is None
                     else str(prov["needle_type"])),
        needle_bore_um=_opt_float(prov.get("needle_bore_um")),
        needle_tip_length_mm=_opt_float(prov.get("needle_tip_length_mm")),
        plate_bottom_z_source=(None if prov.get("plate_bottom_z_source") is None
                               else str(prov["plate_bottom_z_source"])),
        camobj=(None if prov.get("camobj") is None else str(prov["camobj"])),
        needle_template_key=(None if prov.get("needle_template_key") is None
                             else str(prov["needle_template_key"])),
        fitted_at=(None if prov.get("fitted_at") is None
                   else str(prov["fitted_at"])),
    )


def level_plane_at(*, x0_um: float, y0_um: float, z0_zref_mm: float,
                   **kw) -> PlateZPlane:
    """A zero-tilt plane — behaves exactly like the single scalar datum.

    Useful as an explicit "no tilt measured yet" value and in tests that need
    the degrade-to-scalar path to be exercised through the same code.
    """
    return PlateZPlane(x0_um=float(x0_um), y0_um=float(y0_um),
                       z0_zref_mm=float(z0_zref_mm), **kw)
