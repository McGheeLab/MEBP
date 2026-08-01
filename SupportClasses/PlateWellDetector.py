"""
PlateWellDetector.py — model-driven well detection on a full-plate mosaic.

v7.5.x (operator: *"detect 24 wells on the mosaic and do the best effort to
ensure that they are where they should be and they are the size they should
be"*).

WHY A NEW DETECTOR
------------------
``WellDetector.detect_filled_wells`` is a *generic* blob finder: threshold the
bright pixels, keep the large round blobs. On a clear plastic plate imaged in
brightfield the well is NOT a bright blob — it is a hole with a bright moulded
rim, so the interior and the plate top read the same grey and the detector
returns **zero** wells (measured on the operator's own 24-well mosaic). The
Hough fallback does find them, but it is tuned by a generic "expected diameter
± 35 %" band and nothing checks the answer afterwards.

The insight this module is built on: **for a full-plate mosaic we already know
the answer's shape.** The plate definition gives rows × cols, the well pitch and
the well diameter; the mosaic carries its own px/µm. So the well radius in
pixels and the lattice spacing in pixels are *known to ~1 %* before we look at a
single pixel. That turns an open-ended "find circles" problem into three
well-posed ones:

  1. **Where are the rings?**  Correlate the mosaic with an annulus kernel at
     the KNOWN radius (:func:`ring_response`) — a matched filter, polarity
     aware, far more selective than a generic circle search. On the operator's
     plate the 24 true wells score 22.8–45.0 and the next-best spurious peak
     scores 8.8: a 2.6× gap, so thresholding is trivial.
  2. **Which ring is which well?**  Fit the KNOWN rows × cols lattice
     (:func:`fit_lattice`) as rotation + per-axis scale + translation — no
     shear, because a moulded plate is rigid. This both labels the detections
     and *manufactures the missing ones*: a well hidden under a bubble still
     gets a position, so the result always has exactly rows × cols wells.
  3. **How big is each well, really?**  Walk a radial intensity profile out
     from each centre and find the boundary edge per angular sector, then fit a
     circle to those points (:func:`refine_ring`). This measures the well
     instead of assuming the nominal size, and it is what makes a plate whose
     real pitch/diameter differs from the catalogue value come out right.

Finally the per-well measurements are **regularised against the plate**
(:func:`_regularise`): a well whose refined radius is a statistical outlier, or
whose centre wandered off the lattice, falls back to the plate median / the
lattice prediction. One bad well cannot produce a wrong-sized ring, which is
the "best effort to ensure they are where they should be" half of the request.

PER-PLATE-TYPE PARAMETERS
-------------------------
Everything appearance-dependent lives in :class:`WellAppearance`, which a
``PlateType`` can carry in its JSON (``well_detection``). A black-bottom glass
plate whose wells read as dark discs needs a different edge polarity, not
different code. ``edge_polarity="auto"`` (the default) runs the pipeline under
both polarities and keeps whichever locks the lattice better, so a brand-new
plate type works with no tuning at all — and the resolved polarity is reported
so it can be written back into the plate type. As the library grows, those
stored profiles are exactly the labelled training set a classifier would need.

GUI-free: numpy + OpenCV only.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field, replace
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:                                   # pragma: no cover
    cv2 = None
    CV2_AVAILABLE = False


# ── Per-plate-type appearance profile ──────────────────────────────

@dataclass(frozen=True)
class WellAppearance:
    """How one plate product's wells look, and how hard to search for them.

    Defaults are the values validated on the operator's NEST clear-plastic
    24-well mosaic. Every field is a per-plate-type knob; none is a magic
    constant buried in the algorithm.
    """

    key: str = "auto"
    #: Radial intensity step at the well boundary, scanning outward from the
    #: centre. "rising" = darker inside than out (a clear well ringed by a
    #: bright moulded rim, or a dark well on a bright plate). "falling" = the
    #: inverse (bright contents / illuminated well on a dark background).
    #: "auto" tries both and keeps whichever locks the lattice better.
    edge_polarity: str = "auto"
    #: Fractional search band around the catalogue well radius. The refinement
    #: will not report a radius outside nominal x (1 +/- this).
    #: Measured across the three real plate mosaics on this rig: a WIDE band is
    #: actively harmful — at +/-30 % the radial edge walk on a low-contrast
    #: plate latched onto a shoulder well outside the well and reported
    #: 18.4 mm for a 15.6 mm well, while +/-10 % measured 15.58 mm on the same
    #: image. We know the catalogue diameter to ~1 %, so a tight band is both
    #: honest and better conditioned. Widen it only for a plate whose
    #: catalogue diameter is itself a guess.
    diameter_tol: float = 0.10
    #: Annulus kernel half-width as a fraction of the well radius. 0.06 chosen
    #: over 0.05/0.08/0.10 by the same sweep (best worst-case across plates).
    ring_band_frac: float = 0.06
    #: Mosaic downsample for the matched filter (speed; the refinement runs at
    #: full resolution, so this costs no accuracy in the final numbers).
    downsample: int = 4
    #: Keep peaks scoring at least this fraction of the best peak.
    candidate_keep_frac: float = 0.10
    #: A detection may sit this far (in pitches) from its lattice node.
    lattice_tol_frac: float = 0.25
    #: Refuse the fit if the measured pitch differs from nominal by more.
    pitch_tol_frac: float = 0.25
    #: Angular sectors used by the radial edge walk.
    refine_sectors: int = 180
    refine_iters: int = 3
    #: Below this, a refined well is not trusted and falls back to the plate.
    min_quality: float = 0.25
    #: Radius outlier gate, in robust sigmas (MAD) about the plate median.
    radius_mad_k: float = 3.0
    #: A refined centre further than this (in pitches) from the lattice is
    #: rejected in favour of the lattice prediction.
    max_centre_shift_frac: float = 0.15
    #: Acceptance gates. Fraction of wells that must have produced a matched
    #: -filter ring snapped to a lattice node — the discriminator for "wrong
    #: plate type": a 96-well grid fitted to a 24-well mosaic still "measures"
    #: 79 % of its nodes (the refinement finds *some* edge inside those big
    #: wells) but only 11 % of them are real rings.
    #: 0.60 rather than 0.50 because a SQUARE grid also admits a 45-degree
    #: lattice at sqrt(2)x the pitch, which lands on exactly half the wells and
    #: so scores 50 %. The three real plate mosaics score 100/100/75 %.
    min_lattice_frac: float = 0.60
    #: Fraction of wells whose edge could actually be measured — the
    #: discriminator for "wrong appearance profile": forcing the wrong edge
    #: polarity still snaps half the lattice but measures nothing.
    min_measured_frac: float = 0.25

    def to_dict(self) -> dict:
        """Only the fields that differ from the defaults (so a plate type's
        JSON stays minimal and a future default change still reaches it)."""
        base = WellAppearance()
        out = {}
        for f in self.__dataclass_fields__:                # noqa: SLF001
            v = getattr(self, f)
            if v != getattr(base, f):
                out[f] = v
        return out

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> "WellAppearance":
        if not isinstance(data, dict):
            return cls()
        preset = PRESETS.get(str(data.get("key", "") or "").lower())
        base = preset if preset is not None else cls()
        kwargs = {}
        for f in cls.__dataclass_fields__:                 # noqa: SLF001
            if f in data:
                kwargs[f] = data[f]
        try:
            return replace(base, **kwargs)
        except Exception:                                  # unknown/bad value
            return base

    @classmethod
    def for_plate_type(cls, plate_type) -> "WellAppearance":
        """Read the profile off a ``PlateType`` (or anything exposing
        ``well_detection``). Missing/blank → the auto-sensing default."""
        return cls.from_dict(getattr(plate_type, "well_detection", None))


#: Named starting points. A new plate product should start at ``auto``; once a
#: scan confirms which polarity wins, store that preset on the plate type so the
#: detection is deterministic (and so the library doubles as training data).
PRESETS: dict[str, WellAppearance] = {
    "auto": WellAppearance(key="auto", edge_polarity="auto"),
    # Clear/plastic plate, brightfield from above: the well is a hole and the
    # moulded rim around it is bright, so intensity RISES at the boundary.
    # Validated on NEST plastic-bottom 24-well.
    "clear_plastic_rim": WellAppearance(key="clear_plastic_rim",
                                        edge_polarity="rising"),
    # Wells brighter than their surroundings (fluorescent fill, illuminated
    # glass bottom): intensity FALLS crossing the boundary outward.
    "bright_disc": WellAppearance(key="bright_disc", edge_polarity="falling"),
    # Wells darker than a bright plate body (black-bottom plates seen from
    # above): intensity RISES crossing outward, like the rim case.
    "dark_disc": WellAppearance(key="dark_disc", edge_polarity="rising"),
}


# ── Results ────────────────────────────────────────────────────────

@dataclass
class WellDetection:
    """One well on the mosaic, in mosaic pixels."""

    row: int
    col: int
    center_px: tuple[float, float]
    radius_px: float
    #: "measured" = the radial edge fit was trusted; "lattice" = the position
    #: came from the fitted plate lattice (well not found / fit rejected).
    center_source: str = "measured"
    #: "measured" | "plate_median" (this well's fit was an outlier) | "nominal".
    radius_source: str = "measured"
    quality: float = 0.0
    #: Distance (px) between the refined centre and the lattice prediction.
    lattice_residual_px: float = 0.0


@dataclass
class PlateDetectionResult:
    wells: list[WellDetection] = field(default_factory=list)
    ok: bool = False
    refuse_reason: str = ""
    warnings: list[str] = field(default_factory=list)
    #: Resolved polarity actually used (useful to store on the plate type).
    polarity: str = ""
    rotation_deg: float = 0.0
    pitch_x_px: float = 0.0
    pitch_y_px: float = 0.0
    n_candidates: int = 0
    n_lattice_inliers: int = 0
    n_measured: int = 0
    median_radius_px: float = 0.0
    #: Measured geometry in plate units, for the operator-facing report.
    measured_pitch_x_um: float = 0.0
    measured_pitch_y_um: float = 0.0
    measured_diameter_um: float = 0.0
    nominal_diameter_um: float = 0.0
    nominal_pitch_um: float = 0.0
    lattice_rms_px: float = 0.0

    def summary(self) -> str:
        if not self.ok:
            return f"Well detection failed: {self.refuse_reason}"
        d_mm = self.measured_diameter_um / 1000.0
        dn_mm = self.nominal_diameter_um / 1000.0
        return (f"{self.n_measured}/{len(self.wells)} wells measured "
                f"(Ø {d_mm:.2f} mm vs {dn_mm:.2f} nominal, "
                f"rotation {self.rotation_deg:+.2f}°, "
                f"fit RMS {self.lattice_rms_px:.1f} px)")


# ── Stage 1: matched filter for rings of a KNOWN radius ────────────

def ring_response(gray: np.ndarray, radius_px: float, *, downsample: int = 4,
                  band_frac: float = 0.10,
                  polarity: str = "rising") -> np.ndarray:
    """Correlate with an annulus kernel: −1 just inside ``radius_px``, +1 just
    outside (``polarity="rising"``), or the inverse for "falling".

    The two lobes are each normalised to unit total weight, so the response is
    a difference of local means and is therefore invariant to the mosaic's
    overall brightness — which matters because a stitched mosaic has visible
    tile-to-tile illumination steps.
    """
    if not CV2_AVAILABLE:
        raise RuntimeError("OpenCV is required for well detection")
    ds = max(1, int(downsample))
    small = gray if ds == 1 else cv2.resize(
        gray, None, fx=1.0 / ds, fy=1.0 / ds, interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small.astype(np.float32), (0, 0), 1.0)
    r = float(radius_px) / ds
    w = max(1.5, float(band_frac) * r)
    n = int(math.ceil(r + w)) * 2 + 1
    c = n // 2
    yy, xx = np.mgrid[0:n, 0:n]
    rr = np.hypot(xx - c, yy - c)
    k = np.zeros((n, n), np.float32)
    k[(rr >= r - w) & (rr < r)] = -1.0
    k[(rr >= r) & (rr < r + w)] = +1.0
    if polarity == "falling":
        k = -k
    npos, nneg = int((k > 0).sum()), int((k < 0).sum())
    if npos:
        k[k > 0] /= npos
    if nneg:
        k[k < 0] /= nneg
    return cv2.filter2D(small, cv2.CV_32F, k,
                        borderType=cv2.BORDER_REPLICATE)


def find_candidates(gray: np.ndarray, radius_px: float, pitch_px: float,
                    n_wells: int, app: WellAppearance,
                    polarity: str) -> list[tuple[float, float, float]]:
    """Non-maximum-suppressed peaks of the ring response, strongest first.

    Returns ``(x_px, y_px, score)`` at FULL mosaic resolution. Up to 2x the
    well count is returned: extra candidates cost the lattice fit nothing and
    protect against a genuine well being outscored by a bright artefact.
    """
    ds = max(1, int(app.downsample))
    resp = ring_response(gray, radius_px, downsample=ds,
                         band_frac=app.ring_band_frac, polarity=polarity)
    mind = max(3, int(round(pitch_px * 0.6 / ds))) | 1
    dil = cv2.dilate(resp, np.ones((mind, mind), np.uint8))
    mask = (resp >= dil - 1e-6) & (resp > 0)
    pk = np.argwhere(mask)
    if not len(pk):
        return []
    vals = resp[pk[:, 0], pk[:, 1]]
    order = np.argsort(-vals)[:max(8, n_wells * 2)]
    best = float(vals[order[0]])
    keep = [i for i in order
            if float(vals[i]) >= app.candidate_keep_frac * best]
    return [(float(pk[i][1] * ds), float(pk[i][0] * ds), float(vals[i]))
            for i in keep]


# ── Stage 2: fit the KNOWN rows x cols lattice ─────────────────────

def _assign(P: np.ndarray, pred: np.ndarray, tol: float) -> dict:
    """Greedy nearest-first one-to-one matching of candidates to nodes."""
    dm = np.linalg.norm(P[:, None, :] - pred[None, :, :], axis=2)
    assign: dict[int, int] = {}
    used: set[int] = set()
    for k in np.argsort(dm, axis=None):
        pi, ni = divmod(int(k), pred.shape[0])
        if dm[pi, ni] > tol:
            break
        if pi in used or ni in assign:
            continue
        used.add(pi)
        assign[ni] = pi
    return assign


def _residual(nodes, P, assign, A, t) -> float:
    idx = sorted(assign)
    if not idx:
        return float("inf")
    pred = nodes @ A.T + t
    return float(np.mean(np.linalg.norm(
        P[[assign[i] for i in idx]] - pred[idx], axis=1)))


def _solve_scale_t(nodes, P, assign, theta):
    """Closed-form per-axis scale + translation for a GIVEN rotation.

    The model is ``p = R(θ)·diag(sx, sy)·[col, row] + t`` — rotation, a scale
    per lattice axis, and a shift. Deliberately NOT a general affine: a moulded
    plate cannot shear, and allowing shear lets the fit absorb a wrong
    assignment into a skew that still "explains" every point.
    """
    idx = sorted(assign)
    if len(idx) < 2:
        return None
    N = nodes[idx]
    Q = P[[assign[i] for i in idx]]
    c, s = math.cos(-theta), math.sin(-theta)
    q = Q @ np.array([[c, -s], [s, c]]).T          # into lattice axes
    def _axis(n_vals, q_vals):
        if len(set(np.round(n_vals, 6))) > 1:
            m, b = np.polyfit(n_vals, q_vals, 1)
            return float(m), float(b)
        return 0.0, float(np.mean(q_vals))
    sx, ax = _axis(N[:, 0], q[:, 0])
    sy, ay = _axis(N[:, 1], q[:, 1])
    c2, s2 = math.cos(theta), math.sin(theta)
    Rf = np.array([[c2, -s2], [s2, c2]])
    return Rf @ np.diag([sx, sy]), Rf @ np.array([ax, ay])


def fit_lattice(cands, rows: int, cols: int, pitch_x_px: float,
                pitch_y_px: float, app: WellAppearance):
    """Fit the plate lattice to the candidate ring centres.

    Returns ``((A, t), assign, reason)``; ``A`` maps ``[col, row] → px``.
    ``assign`` maps ``node_index → candidate_index``.
    """
    P = np.array([[c[0], c[1]] for c in cands], float)
    if len(P) < 3:
        return None, {}, "fewer than 3 ring candidates"
    nodes = np.array([[j, i] for i in range(rows) for j in range(cols)], float)
    pitch = 0.5 * (pitch_x_px + pitch_y_px)
    tol = app.lattice_tol_frac * pitch

    # Rotation seeds: histogram peak of the adjacent-pair angles folded into a
    # 90-degree wedge, all four quadrant aliases, plus a coarse safety sweep so
    # a plate at any orientation still locks.
    d = P[:, None, :] - P[None, :, :]
    L = np.linalg.norm(d, axis=2)
    sel = (L > 0.7 * pitch) & (L < 1.3 * pitch)
    seeds: list[float] = []
    if int(sel.sum()) >= 2:
        vec = d[sel]
        ang = np.degrees(np.arctan2(vec[:, 1], vec[:, 0])) % 90.0
        hist, edges = np.histogram(ang, bins=180, range=(0, 90))
        pk = float(edges[int(np.argmax(hist))]) + 0.25
        near = np.abs(((ang - pk + 45.0) % 90.0) - 45.0) < 3.0
        if int(near.sum()):
            pk += float(np.mean(((ang[near] - pk + 45.0) % 90.0) - 45.0))
        seeds = [pk + q for q in (0.0, 90.0, 180.0, 270.0)]
    seeds += list(np.arange(0.0, 360.0, 5.0))

    best = (None, None, None)          # (key, theta, assign)
    for th_deg in seeds:
        th = math.radians(th_deg)
        c, s = math.cos(th), math.sin(th)
        base = nodes @ (np.array([[c, -s], [s, c]])
                        @ np.diag([pitch_x_px, pitch_y_px])).T
        # Translation by voting: every (candidate, node) pairing implies one
        # offset; the correct one is voted for by every true well at once.
        offs = (P[:, None, :] - base[None, :, :]).reshape(-1, 2)
        cell = max(2.0, pitch * 0.12)
        key = np.round(offs / cell).astype(np.int64)
        _uk, inv, cnt = np.unique(key, axis=0, return_inverse=True,
                                  return_counts=True)
        for ci in np.argsort(-cnt)[:4]:
            t0 = offs[inv == ci].mean(axis=0)
            a = _assign(P, base + t0, tol)
            if not a:
                continue
            k = (len(a), -_residual(nodes, P, a, base_A(th, pitch_x_px,
                                                        pitch_y_px), t0))
            if best[0] is None or k > best[0]:
                best = (k, th, a)
    if best[1] is None:
        return None, {}, "no lattice hypothesis matched the candidates"

    # Refine: pick theta by MINIMISING the inlier residual. Scoring by inlier
    # COUNT does not work here — the count saturates at "all wells" for a whole
    # family of wrong fits (a 5-degree tilt still lands every well inside a
    # 0.25-pitch gate), so counting alone lets the fit drift off true.
    th, assign = best[1], best[2]
    A = t = None
    for it in range(6):
        gate = tol * (1.0 if it == 0 else 0.5 if it == 1 else 0.25)
        span = 5.0 if it == 0 else 1.0
        cb = (float("inf"), None, None, None)
        for dth in np.radians(np.arange(-span, span + 1e-9, span / 50.0)):
            sol = _solve_scale_t(nodes, P, assign, th + dth)
            if sol is None:
                continue
            A2, t2 = sol
            res = _residual(nodes, P, assign, A2, t2)
            if res < cb[0]:
                cb = (res, th + dth, A2, t2)
        if cb[2] is None:
            break
        _res, th, A, t = cb
        a2 = _assign(P, nodes @ A.T + t, max(gate, 0.05 * pitch))
        if len(a2) >= 3:
            assign = a2
    if A is None:
        return None, {}, "lattice refinement failed"

    # Canonical orientation. A rows x cols lattice fits its own 180-degree
    # rotation equally well — the plate is symmetric, so nothing in the IMAGE
    # says which corner is A1 — and the two solutions put row/col labels on
    # opposite corners. Pick the one nearer 0 degrees so the answer is
    # deterministic (and matches "the plate sits roughly square in its
    # holder"); which corner is really A1 stays the caller's decision, made
    # from the plate-orientation convention, not from pixels.
    if abs(math.degrees(math.atan2(A[1, 0], A[0, 0]))) > 90.0:
        t = A @ np.array([cols - 1.0, rows - 1.0]) + t
        A = -A
        assign = {(rows - 1 - ni // cols) * cols + (cols - 1 - ni % cols): pi
                  for ni, pi in assign.items()}

    sx = float(np.linalg.norm(A[:, 0]))
    sy = float(np.linalg.norm(A[:, 1]))
    lo, hi = 1.0 - app.pitch_tol_frac, 1.0 + app.pitch_tol_frac
    if not (lo <= sx / max(pitch_x_px, 1e-6) <= hi
            and lo <= sy / max(pitch_y_px, 1e-6) <= hi):
        return None, {}, (
            f"the fitted well spacing ({sx:.0f}×{sy:.0f} px) is implausible "
            f"for this plate ({pitch_x_px:.0f}×{pitch_y_px:.0f} px expected) — "
            f"wrong plate type selected, or the mosaic scale is wrong")
    return (A, t), assign, ""


def base_A(theta: float, pitch_x: float, pitch_y: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]]) @ np.diag([pitch_x, pitch_y])


# ── Stage 3: measure each well ─────────────────────────────────────

def refine_ring(gray: np.ndarray, cx: float, cy: float, radius_px: float,
                app: WellAppearance, polarity: str):
    """Measure one well: per-sector radial edge → robust circle fit.

    Walks an intensity profile outward along ``refine_sectors`` rays, takes the
    strongest correctly-signed radial step on each (sub-pixel, by parabolic
    interpolation), drops sectors whose radius disagrees with the rest (label
    text, a bubble, a neighbouring well's rim), and fits a circle to what
    survives. Returns ``(cx, cy, r, quality)`` — ``quality`` combines the
    fraction of sectors that agreed with how tightly they fit.
    """
    g = gray.astype(np.float32)
    H, W = g.shape[:2]
    lo = radius_px * (1.0 - app.diameter_tol)
    hi = radius_px * (1.0 + app.diameter_tol)
    rs = np.arange(lo, hi, 0.5, dtype=np.float32)
    if len(rs) < 4:
        return cx, cy, radius_px, 0.0
    n_ang = max(24, int(app.refine_sectors))
    ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False).astype(np.float32)
    ca, sa = np.cos(ang), np.sin(ang)
    r_out, quality = float(radius_px), 0.0
    for _ in range(max(1, int(app.refine_iters))):
        xs = np.ascontiguousarray(cx + rs[None, :] * ca[:, None], np.float32)
        ys = np.ascontiguousarray(cy + rs[None, :] * sa[:, None], np.float32)
        prof = cv2.remap(g, xs, ys, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        valid = ((xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)).all(axis=1)
        prof = cv2.GaussianBlur(prof, (9, 1), 0)
        dp = np.gradient(prof, axis=1)
        if polarity == "falling":
            dp = -dp
        k = np.argmax(dp, axis=1)
        ar = np.arange(len(k))
        good = valid & (dp[ar, k] > 0)
        if int(good.sum()) < 8:
            return cx, cy, r_out, 0.0
        kk = np.clip(k, 1, len(rs) - 2)
        y0, y1, y2 = dp[ar, kk - 1], dp[ar, kk], dp[ar, kk + 1]
        den = y0 - 2.0 * y1 + y2
        off = np.where(np.abs(den) > 1e-9,
                       0.5 * (y0 - y2) / np.where(den == 0, 1, den), 0.0)
        rr = rs[kk] + np.clip(off, -1.0, 1.0) * 0.5
        med = float(np.median(rr[good]))
        mad = float(np.median(np.abs(rr[good] - med))) + 1e-6
        keep = good & (np.abs(rr - med) < max(4.0, 3.0 * 1.4826 * mad))
        if int(keep.sum()) < 8:
            keep = good
        px = cx + rr[keep] * ca[keep]
        py = cy + rr[keep] * sa[keep]
        M = np.column_stack([px, py, np.ones(int(keep.sum()))])
        try:
            sol, *_ = np.linalg.lstsq(M, px ** 2 + py ** 2, rcond=None)
        except np.linalg.LinAlgError:
            return cx, cy, r_out, 0.0
        ncx, ncy = float(sol[0]) / 2.0, float(sol[1]) / 2.0
        val = float(sol[2]) + ncx ** 2 + ncy ** 2
        if not np.isfinite([ncx, ncy, val]).all() or val <= 0:
            return cx, cy, r_out, 0.0
        cx, cy, r_out = ncx, ncy, math.sqrt(val)
        resid = np.abs(np.hypot(px - cx, py - cy) - r_out)
        quality = (float(keep.sum()) / n_ang) * math.exp(
            -float(np.mean(resid)) / 3.0)
    return cx, cy, r_out, quality


# ── Stage 4: regularise against the plate ──────────────────────────

def _regularise(wells: list[WellDetection], nominal_r: float, pitch: float,
                app: WellAppearance) -> tuple[float, list[str]]:
    """Replace untrustworthy per-well measurements with the plate consensus.

    A moulded plate's wells are identical to well under a percent, so the
    median of 24 measurements is a far better estimate of any single well's
    size than a lone fit that disagreed with all the others.
    """
    notes: list[str] = []
    trusted = [w.radius_px for w in wells
               if w.quality >= app.min_quality and w.center_source == "measured"]
    if len(trusted) >= max(3, len(wells) // 4):
        med = float(np.median(trusted))
        mad = float(np.median(np.abs(np.array(trusted) - med))) + 1e-9
    else:
        med, mad = float(nominal_r), 0.0
        notes.append("too few wells measured cleanly — sizes fall back to the "
                     "plate's nominal well diameter")
    gate = max(2.0, app.radius_mad_k * 1.4826 * mad) if mad > 0 else 0.0
    for w in wells:
        bad_q = w.quality < app.min_quality
        bad_r = gate > 0 and abs(w.radius_px - med) > gate
        out_of_band = not (nominal_r * (1 - app.diameter_tol) <= w.radius_px
                           <= nominal_r * (1 + app.diameter_tol))
        if bad_q or bad_r or out_of_band:
            w.radius_px = med
            w.radius_source = ("plate_median" if trusted else "nominal")
        if w.center_source == "measured" and (
                bad_q or w.lattice_residual_px
                > app.max_centre_shift_frac * pitch):
            w.center_source = "lattice"
    return med, notes


# ── Public entry point ─────────────────────────────────────────────

def detect_plate_wells(image: np.ndarray, *, rows: int, cols: int,
                       px_per_um: float, diameter_um: float,
                       pitch_x_um: float, pitch_y_um: float = 0.0,
                       appearance: Optional[WellAppearance] = None
                       ) -> PlateDetectionResult:
    """Detect every well of a rows × cols plate on a stitched mosaic.

    Args:
        image: the mosaic (BGR or grayscale).
        rows, cols: the plate's grid, from the plate definition.
        px_per_um: the mosaic's scale (``MosaicBuilder._mosaic_scale``).
        diameter_um / pitch_x_um / pitch_y_um: catalogue plate geometry.
        appearance: per-plate-type profile; ``None`` = auto-sensing default.

    The result always carries exactly ``rows × cols`` wells when ``ok`` — any
    well the image did not yield is filled in from the fitted lattice and
    flagged ``center_source="lattice"``.
    """
    app = appearance or WellAppearance()
    res = PlateDetectionResult(nominal_diameter_um=float(diameter_um),
                               nominal_pitch_um=float(pitch_x_um))
    if not CV2_AVAILABLE:
        res.refuse_reason = "OpenCV is not available"
        return res
    if image is None or getattr(image, "size", 0) == 0:
        res.refuse_reason = "no mosaic image"
        return res
    rows, cols = int(rows), int(cols)
    if rows < 1 or cols < 1:
        res.refuse_reason = "the plate has no row/column grid"
        return res
    if px_per_um <= 0 or diameter_um <= 0 or pitch_x_um <= 0:
        res.refuse_reason = "the plate geometry or the mosaic scale is unknown"
        return res
    if not pitch_y_um:
        pitch_y_um = pitch_x_um

    # Brightness = max over channels, not BGR2GRAY: a fluorescence mosaic can
    # be almost pure blue, which the luma weights (B = 0.114) all but throw
    # away — on the real blue-channel plate mosaic that dropped the working
    # contrast from std 47.8 to 5.6. Max-over-channels is colour-agnostic and
    # matches WellDetector.detect_filled_wells.
    gray = image.max(axis=2) if image.ndim == 3 else image
    r_nom = diameter_um / 2.0 * px_per_um
    px_x = pitch_x_um * px_per_um
    px_y = pitch_y_um * px_per_um
    pitch = 0.5 * (px_x + px_y)
    n_wells = rows * cols

    if r_nom < 6.0:
        res.refuse_reason = (
            f"a {diameter_um / 1000.0:.1f} mm well is only "
            f"{2 * r_nom:.0f} px across in this mosaic — too small to measure; "
            f"re-scan at a higher mosaic resolution")
        return res
    if min(gray.shape[:2]) < pitch:
        res.refuse_reason = ("the mosaic is smaller than one well pitch — it "
                             "does not cover the plate")
        return res

    # Polarity: try both when the plate type has not committed to one, and keep
    # whichever locks the lattice better. This is what lets a brand-new plate
    # product work before anyone has characterised it.
    pol_list = (["rising", "falling"] if app.edge_polarity == "auto"
                else [app.edge_polarity])
    attempts = []
    for pol in pol_list:
        cands = find_candidates(gray, r_nom, pitch, n_wells, app, pol)
        fit, assign, why = fit_lattice(cands, rows, cols, px_x, px_y, app)
        if fit is None:
            attempts.append((-1, 0.0, pol, cands, None, {}, why))
            continue
        A, t = fit
        nodes = np.array([[j, i] for i in range(rows) for j in range(cols)],
                         float)
        P = np.array([[c[0], c[1]] for c in cands], float)
        attempts.append((len(assign), -_residual(nodes, P, assign, A, t),
                         pol, cands, fit, assign, ""))
    attempts.sort(key=lambda a: (a[0], a[1]), reverse=True)
    n_in, _score, polarity, cands, fit, assign, why = attempts[0]
    res.polarity = polarity
    res.n_candidates = len(cands)
    if fit is None:
        res.refuse_reason = why or "could not lock the plate grid"
        return res
    if len(pol_list) > 1:
        logger.info(f"Well detection: edge polarity '{polarity}' won "
                    f"({n_in}/{n_wells} wells matched)")

    A, t = fit
    res.n_lattice_inliers = len(assign)
    nodes = np.array([[j, i] for i in range(rows) for j in range(cols)], float)
    pred = nodes @ A.T + t
    res.pitch_x_px = float(np.linalg.norm(A[:, 0]))
    res.pitch_y_px = float(np.linalg.norm(A[:, 1]))
    res.rotation_deg = float(math.degrees(math.atan2(A[1, 0], A[0, 0])))
    res.measured_pitch_x_um = res.pitch_x_px / px_per_um
    res.measured_pitch_y_um = res.pitch_y_px / px_per_um

    # Measure every well at its lattice position (found or not) — a well the
    # matched filter missed is often still measurable once we know where to look.
    wells: list[WellDetection] = []
    for ni in range(len(nodes)):
        j, i = int(nodes[ni][0]), int(nodes[ni][1])
        lx, ly = float(pred[ni][0]), float(pred[ni][1])
        cx, cy, r, q = refine_ring(gray, lx, ly, r_nom, app, polarity)
        shift = math.hypot(cx - lx, cy - ly)
        wells.append(WellDetection(
            row=i, col=j, center_px=(cx, cy), radius_px=float(r),
            quality=float(q), lattice_residual_px=float(shift)))

    med_r, notes = _regularise(wells, r_nom, pitch, app)
    res.warnings.extend(notes)
    # Wells rejected by the regulariser fall back to their lattice node.
    for w, p in zip(wells, pred):
        if w.center_source == "lattice":
            w.center_px = (float(p[0]), float(p[1]))
    res.wells = wells
    res.n_measured = sum(1 for w in wells if w.center_source == "measured")
    res.median_radius_px = med_r
    res.measured_diameter_um = 2.0 * med_r / px_per_um
    resids = [w.lattice_residual_px for w in wells
              if w.center_source == "measured"]
    res.lattice_rms_px = (float(np.sqrt(np.mean(np.square(resids))))
                          if resids else 0.0)

    # Acceptance gates. Placing a full grid of wrong markers is worse than
    # saying "I couldn't find it" — the caller's fallback (teach 3 corners by
    # hand) is a fine outcome, a silently wrong 96-well grid is not.
    if res.n_lattice_inliers < app.min_lattice_frac * n_wells:
        res.refuse_reason = (
            f"only {res.n_lattice_inliers} of {n_wells} wells produced a ring "
            f"of the expected size at a grid position — this mosaic does not "
            f"look like a {rows}×{cols} plate with "
            f"{diameter_um / 1000.0:.1f} mm wells "
            f"{pitch_x_um / 1000.0:.1f} mm apart. Check the selected plate "
            f"type and the mosaic's µm/px — or the mosaic may not cover the "
            f"whole plate.")
        return res
    if res.n_measured < app.min_measured_frac * n_wells:
        res.refuse_reason = (
            f"the grid matched but only {res.n_measured} of {n_wells} well "
            f"edges could be measured — this plate's appearance profile is "
            f"probably wrong (tried edge polarity '{polarity}'). Set a "
            f"different well-detection profile on the plate type.")
        return res
    res.ok = True

    # Operator-facing cross-checks. The plate is a manufactured ruler, so a
    # systematic pitch error is evidence about the MOSAIC's scale, not the plate.
    pitch_err = res.measured_pitch_x_um / pitch_x_um - 1.0
    if abs(pitch_err) > 0.02:
        res.warnings.append(
            f"measured well spacing {res.measured_pitch_x_um / 1000.0:.2f} mm "
            f"vs {pitch_x_um / 1000.0:.2f} mm nominal ({pitch_err:+.1%}) — the "
            f"mosaic µm/px is probably off by about that much")
    d_err = res.measured_diameter_um / diameter_um - 1.0
    if abs(d_err) > 0.10:
        res.warnings.append(
            f"measured well Ø {res.measured_diameter_um / 1000.0:.2f} mm vs "
            f"{diameter_um / 1000.0:.2f} mm nominal ({d_err:+.1%}) — check the "
            f"selected plate type")
    if res.n_measured < n_wells:
        res.warnings.append(
            f"{n_wells - res.n_measured} well(s) could not be measured and were "
            f"placed from the plate grid")
    return res
