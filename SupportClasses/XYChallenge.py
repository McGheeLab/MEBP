"""XYChallenge.py — path-following challenge shapes + error metric (v7.5.x).

Pure geometry + math for the "XY Printing Challenge" calibration: a set of test
paths that stress XY path-following in different ways (sharp corners, curves,
reversals, dense loops), plus a metric that compares the stage's ACTUAL path
(recorded from the encoder) to the IDEAL commanded path.

No Qt, no hardware, no heavy imports — safe to unit-test in isolation.

Convention: every shape is returned as a polyline of ``(x, y)`` points in mm,
centred on ``(0, 0)`` and resampled to ~uniform segment length (so the segment
count / spacing resembles a real sketch toolpath). The caller offsets the whole
path to the start location (zero-ref mm) before driving it.
"""

from __future__ import annotations

import math

# Ordered so the UI combo reads corners → curves → reversals → dense, then the
# three diagnostic shapes added by the v7.5.x challenge upgrade.
CHALLENGE_SHAPES = ["Square", "Circle", "Star", "Zigzag", "Spiral",
                    "Line-Reversal", "Comb", "Dwell-Stitch"]

# What each shape is designed to expose (shown as a hint in the UI).
SHAPE_HINTS = {
    "Square": "4 sharp 90° corners — accel/decel + overshoot at corners.",
    "Circle": "constant curvature — steady tracking + how round the loop stays.",
    "Star": "5 sharp reversals (points) — the hardest corners + direction flips.",
    "Zigzag": "back-and-forth sharp turns — repeated accel reversals (backlash).",
    "Spiral": "dense concentric loops — sustained curvature + loop spacing.",
    "Line-Reversal": "one axis, straight out and back — isolates BACKLASH and the "
                     "reversal spike with zero curvature to confound it.",
    "Comb": "parallel strokes at 2×, 1.5×, 1× and 0.75× the resolution element — "
            "does the machine actually resolve its own claimed element?",
    "Dwell-Stitch": "a line with commanded stops — exposes stitch artefacts and "
                    "unintended dwell (a pause deposits a blob).",
}

#: Per-shape sub-parameters, previously hidden as private defaults. The dialog
#: builds its spin boxes from this, so adding a shape needs no GUI edit.
SHAPE_PARAMS = {
    "Star": {"points": 5},
    "Zigzag": {"rows": 6},
    "Spiral": {"turns": 3.0},
    "Line-Reversal": {"passes": 4},
    "Comb": {"teeth": 4, "pitch_mm": 0.3},
    "Dwell-Stitch": {"stops": 5},
}


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def resample_polyline(pts, step_mm: float):
    """Resample a polyline to ~uniform ``step_mm`` spacing, preserving the
    original vertices (so corners stay crisp) and the endpoints. A point is
    emitted at every vertex AND every ``step_mm`` of arc length along each
    segment."""
    step = max(1e-3, float(step_mm))
    if len(pts) < 2:
        return [(float(p[0]), float(p[1])) for p in pts]
    out = [(float(pts[0][0]), float(pts[0][1]))]
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        seg = _dist(a, b)
        if seg < 1e-9:
            continue
        n = int(seg // step)
        for k in range(1, n + 1):
            t = (k * step) / seg
            if t >= 1.0:
                break
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        out.append((float(b[0]), float(b[1])))   # always land the vertex
    return out


def _square(size, step):
    h = size / 2.0
    corners = [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]
    return resample_polyline(corners, step)


def _circle(size, step):
    r = size / 2.0
    # angular step so the chord ≈ step_mm
    n = max(24, int(math.ceil(2 * math.pi * r / max(step, 1e-3))))
    return [(r * math.cos(2 * math.pi * k / n),
             r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


def _star(size, step, points=5):
    r_out = size / 2.0
    r_in = r_out * 0.4
    verts = []
    for k in range(2 * points + 1):
        ang = math.pi / 2 + math.pi * k / points     # start at top
        r = r_out if k % 2 == 0 else r_in
        verts.append((r * math.cos(ang), r * math.sin(ang)))
    return resample_polyline(verts, step)


def _zigzag(size, step, rows=6):
    h = size / 2.0
    dx = size / rows
    verts = []
    x = -h
    top = True
    verts.append((x, -h if top else h))
    for _ in range(rows):
        # vertical stroke then step across → sharp 180°-ish reversals
        verts.append((x, h if top else -h))
        x += dx
        verts.append((x, h if top else -h))
        top = not top
    return resample_polyline(verts, step)


def _spiral(size, step, turns=3.0):
    r_max = size / 2.0
    r_min = max(0.15 * r_max, 0.2)
    total_ang = 2 * math.pi * turns
    # sample fine enough that the chord ≈ step at the OUTER radius
    n = max(60, int(math.ceil(total_ang * r_max / max(step, 1e-3))))
    out = []
    for k in range(n + 1):
        f = k / n
        ang = total_ang * f
        r = r_max + (r_min - r_max) * f
        out.append((r * math.cos(ang), r * math.sin(ang)))
    return out


def _line_reversal(size, step, passes=4):
    """Straight out-and-back along X, ``passes`` legs. Pure 180° reversals with
    no curvature — the sharpest possible test of backlash and reversal spike."""
    h = size / 2.0
    pts = [(-h, 0.0)]
    for k in range(max(1, int(passes))):
        pts.append((h, 0.0) if k % 2 == 0 else (-h, 0.0))
    return resample_polyline(pts, step)


def _comb(size, step, teeth=4, pitch_mm=0.3):
    """Parallel strokes at decreasing pitch (2×, 1.5×, 1×, 0.75× ``pitch_mm``).

    Turns "does the machine resolve its claimed resolution element?" into a
    direct observation: the finest pair is deliberately below it.
    """
    n = max(2, int(teeth))
    h = size / 2.0
    factors = [2.0, 1.5, 1.0, 0.75]
    pts = [(-h, 0.0)]
    y = 0.0
    for k in range(n):
        pts.append((h, y))                                  # stroke across
        gap = pitch_mm * factors[k % len(factors)]
        y += gap
        pts.append((h, y))                                  # step over
        pts.append((-h, y))                                 # stroke back
        gap = pitch_mm * factors[(k + 1) % len(factors)]
        y += gap
        pts.append((-h, y))
    # Re-centre on the origin.
    cy = 0.5 * (min(p[1] for p in pts) + max(p[1] for p in pts))
    pts = [(x, yy - cy) for x, yy in pts]
    return resample_polyline(pts, step)


def _dwell_stitch(size, step, stops=5):
    """A straight line whose commanded stop points are DUPLICATED vertices.

    The duplicates mark where a stop was asked for; the arc-length positions come
    back from :func:`shape_meta`. Harmless to the print path, which de-duplicates
    coincident points.
    """
    n = max(1, int(stops))
    h = size / 2.0
    pts = [(-h, 0.0)]
    for k in range(1, n + 1):
        x = -h + (size * k) / (n + 1)
        pts.append((x, 0.0))
        pts.append((x, 0.0))                                # the stop
    pts.append((h, 0.0))
    return resample_polyline(pts, step)


def make_shape(name: str, size_mm: float, step_mm: float = 0.5, **params):
    """Ideal path (polyline, mm, centred at origin) for a challenge shape.

    ``**params`` exposes the per-shape sub-parameters listed in
    :data:`SHAPE_PARAMS` (previously buried as private defaults). The positional
    call signature is unchanged, so every existing caller keeps working.
    """
    size = max(0.5, float(size_mm))
    step = max(0.05, float(step_mm))
    key = (name or "").strip().lower()
    if key == "square":
        return _square(size, step)
    if key == "circle":
        return _circle(size, step)
    if key == "star":
        return _star(size, step, points=int(params.get("points", 5)))
    if key == "zigzag":
        return _zigzag(size, step, rows=int(params.get("rows", 6)))
    if key == "spiral":
        return _spiral(size, step, turns=float(params.get("turns", 3.0)))
    if key == "line-reversal":
        return _line_reversal(size, step, passes=int(params.get("passes", 4)))
    if key == "comb":
        return _comb(size, step, teeth=int(params.get("teeth", 4)),
                     pitch_mm=float(params.get("pitch_mm", 0.3)))
    if key == "dwell-stitch":
        return _dwell_stitch(size, step, stops=int(params.get("stops", 5)))
    raise ValueError(f"unknown challenge shape: {name!r}")


def shape_meta(name: str, size_mm: float, step_mm: float = 0.5, **params) -> dict:
    """Structural annotations for a shape: where its corners are, where a stop was
    commanded, and (for Comb) the finest feature pitch.

    Separate from :func:`make_shape` so that function's return type stays a plain
    polyline for every existing caller.
    """
    pts = make_shape(name, size_mm, step_mm, **params)
    cum = _cum(pts)
    corners = [cum[i] for i in range(1, len(pts) - 1)
               if _turn_angle_deg(pts, i) > 30.0]
    meta = {"corner_s_mm": corners, "dwell_s_mm": [],
            "feature_pitch_mm": None, "total_mm": cum[-1]}
    key = (name or "").strip().lower()
    if key == "dwell-stitch":
        # The duplicated vertices that mark the stops do not survive
        # resample_polyline (it drops zero-length segments, correctly), so derive
        # the stop positions from the geometry instead of the emitted polyline.
        n = max(1, int(params.get("stops", 5)))
        size = max(0.5, float(size_mm))
        meta["dwell_s_mm"] = [size * k / (n + 1) for k in range(1, n + 1)]
    if key == "comb":
        meta["feature_pitch_mm"] = float(params.get("pitch_mm", 0.3)) * 0.75
    return meta


def offset_path(pts, cx: float, cy: float):
    """Translate a centred path to a start location (mm)."""
    return [(p[0] + cx, p[1] + cy) for p in pts]


def _point_to_segment_um(p, a, b) -> float:
    """Perpendicular distance (µm) from point ``p`` to segment ``a``–``b`` (all
    in mm)."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:
        return _dist(p, a) * 1000.0
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2
    t = max(0.0, min(1.0, t))
    projx, projy = ax + t * dx, ay + t * dy
    return math.hypot(p[0] - projx, p[1] - projy) * 1000.0


def _path_error_bruteforce(samples_xy, ideal_pts) -> dict:
    """O(N·M) reference implementation of :func:`path_error`.

    Kept so the fast spatially-indexed version can be proved exactly equal to it
    in the tests rather than merely "close".
    """
    if not samples_xy or len(ideal_pts) < 2:
        return {"rms_um": 0.0, "max_um": 0.0, "mean_um": 0.0, "n": 0}
    devs = []
    for p in samples_xy:
        best = float("inf")
        for i in range(1, len(ideal_pts)):
            d = _point_to_segment_um(p, ideal_pts[i - 1], ideal_pts[i])
            if d < best:
                best = d
                if best <= 0.0:
                    break
        devs.append(best)
    n = len(devs)
    rms = math.sqrt(sum(d * d for d in devs) / n)
    return {"rms_um": rms, "max_um": max(devs),
            "mean_um": sum(devs) / n, "n": n}


def path_error(samples_xy, ideal_pts) -> dict:
    """Compare recorded actual positions to the ideal polyline.

    ``samples_xy`` — list of ``(x, y)`` ACTUAL stage positions (mm), in order.
      Also accepts :class:`Sample` / ``(x, y, t)`` triples (extra fields ignored).
    ``ideal_pts``  — the ideal polyline (mm).

    Returns ``{"rms_um", "max_um", "mean_um", "n"}`` — the cross-track deviation
    (min distance from each actual sample to the ideal path) in µm.

    ⚠ THIS METRIC IS ONE-SIDED, UNSIGNED AND UNTIMED, and it only looks at
    samples the stage actually visited. It therefore CANNOT see:
      • a run that stopped early (it is scored only on the prefix it reached);
      • back-and-forth retracing (an exact retrace scores ~ZERO error);
      • a skipped feature or a cut corner (no ideal→actual coverage term);
      • whether the stage went inside or outside a corner (no sign);
      • how long it took (a crawl scores better than a correct-speed run).
    It is retained because historical bench numbers are quoted in it. Use
    :func:`path_report` + :func:`composite_score` for anything that decides
    something — especially as an auto-tune objective.
    """
    return _path_error_fast(samples_xy, ideal_pts)


# ── Samples: timestamped, but tuple-compatible ────────────────────────

class Sample(tuple):
    """One recorded position: ``(x, y, t)`` with ``t`` optional (seconds since
    the run start, or ``None`` for legacy traces).

    Deliberately a tuple subclass rather than a dataclass so every existing
    consumer keeps working untouched — indexing (``p[0]``), iteration, and
    ``for x, y in pts`` unpacking of 2-tuples all behave as before, so the
    overlay painter, ``offset_path`` and ``_point_to_segment_um`` need no
    branching.
    """
    __slots__ = ()

    def __new__(cls, x, y, t=None):
        return super().__new__(cls, (float(x), float(y)) if t is None
                               else (float(x), float(y), float(t)))

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def t(self):
        return self[2] if len(self) > 2 else None


def as_samples(seq, *, dt=None):
    """Normalise a mixed sequence into :class:`Sample` objects.

    Accepts ``(x, y)``, ``(x, y, t)`` and ``Sample``. When ``dt`` is given, any
    sample without a timestamp is assigned a synthetic one at ``i·dt`` (useful
    for replaying a legacy trace through the timed metrics).
    """
    out = []
    for i, p in enumerate(seq or []):
        t = p[2] if len(p) > 2 else (i * dt if dt else None)
        out.append(Sample(p[0], p[1], t))
    return out


# ── Exact nearest-distance via a uniform spatial index ────────────────

class _SegGrid:
    """Uniform-grid index over the ideal polyline's segments.

    Replaces the O(N·M) scan without approximating: the search expands ring by
    ring and stops only once the nearest possible unexamined cell is farther
    than the best distance found, so the answer is EXACT.

    A global nearest search is used deliberately — not the follower's
    forward-windowed projection — because a trace that legitimately revisits or
    strays must be scored honestly, and that is precisely the failure mode being
    measured.
    """

    def __init__(self, pts, cell_mm=None):
        self.pts = pts
        self.n = len(pts)
        if self.n < 2:
            self.cell = 1.0
            self.grid = {}
            return
        seg_lens = [_dist(pts[i - 1], pts[i]) for i in range(1, self.n)]
        mean_seg = (sum(seg_lens) / len(seg_lens)) if seg_lens else 1.0
        self.cell = float(cell_mm) if cell_mm else max(1e-3, 2.0 * mean_seg)
        self.grid = {}
        for i in range(1, self.n):
            a, b = pts[i - 1], pts[i]
            # Register the segment in every cell its bounding box touches.
            cx0, cy0 = self._cell_of(min(a[0], b[0]), min(a[1], b[1]))
            cx1, cy1 = self._cell_of(max(a[0], b[0]), max(a[1], b[1]))
            for cx in range(cx0, cx1 + 1):
                for cy in range(cy0, cy1 + 1):
                    self.grid.setdefault((cx, cy), []).append(i)

    def _cell_of(self, x, y):
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    def nearest_um(self, p) -> float:
        """Exact minimum distance (µm) from ``p`` to the polyline."""
        if self.n < 2:
            return 0.0
        cx, cy = self._cell_of(p[0], p[1])
        best = float("inf")
        seen = set()
        ring = 0
        max_ring = 64
        while ring <= max_ring:
            # Every cell at this ring distance is at least (ring-1)·cell away, so
            # once best beats that bound no farther ring can improve it.
            if best < float("inf") and (ring - 1) * self.cell * 1000.0 > best:
                break
            found_any = False
            for dx in range(-ring, ring + 1):
                for dy in range(-ring, ring + 1):
                    if ring and max(abs(dx), abs(dy)) != ring:
                        continue                     # only the ring's shell
                    for i in self.grid.get((cx + dx, cy + dy), ()):
                        if i in seen:
                            continue
                        seen.add(i)
                        found_any = True
                        d = _point_to_segment_um(p, self.pts[i - 1], self.pts[i])
                        if d < best:
                            best = d
            ring += 1
            if not found_any and best == float("inf") and ring > max_ring:
                break
        if best == float("inf"):                     # pathological: fall back
            return _path_error_bruteforce([p], self.pts)["max_um"]
        return best


def _path_error_fast(samples_xy, ideal_pts) -> dict:
    if not samples_xy or len(ideal_pts) < 2:
        return {"rms_um": 0.0, "max_um": 0.0, "mean_um": 0.0, "n": 0}
    grid = _SegGrid(ideal_pts)
    devs = [grid.nearest_um(p) for p in samples_xy]
    n = len(devs)
    rms = math.sqrt(sum(d * d for d in devs) / n)
    return {"rms_um": rms, "max_um": max(devs),
            "mean_um": sum(devs) / n, "n": n}


# ── Arc-length helpers used by the richer metrics ─────────────────────

def _cum(pts):
    out = [0.0]
    for i in range(1, len(pts)):
        out.append(out[-1] + _dist(pts[i - 1], pts[i]))
    return out


def point_at(pts, cum, s):
    """``(x, y)`` at arc length ``s`` (clamped to the ends)."""
    if not pts:
        return (0.0, 0.0)
    total = cum[-1]
    if s <= 0.0:
        return (pts[0][0], pts[0][1])
    if s >= total:
        return (pts[-1][0], pts[-1][1])
    lo, hi = 0, len(cum) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if cum[mid] < s:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    seg = cum[i] - cum[i - 1]
    t = (s - cum[i - 1]) / seg if seg > 1e-12 else 0.0
    return (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
            pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))


def _tangent_at(pts, cum, s):
    """Unit tangent of the segment containing arc length ``s``."""
    if len(pts) < 2:
        return (1.0, 0.0)
    s = max(0.0, min(s, cum[-1]))
    i = 1
    lo, hi = 1, len(pts) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if cum[mid] < s:
            lo = mid + 1
        else:
            i = mid
            hi = mid - 1
    a, b = pts[i - 1], pts[i]
    d = _dist(a, b)
    if d < 1e-12:
        return (1.0, 0.0)
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def _project(pts, cum, p):
    """Global nearest projection → ``(s, signed_cross_mm, unsigned_cross_mm)``.

    The sign is positive to the LEFT of travel, so an inside-cut and an
    outside-overshoot at a corner are distinguishable — they are not in
    ``path_error``, which is unsigned.
    """
    best_s, best_d, best_i = 0.0, float("inf"), 0
    for i in range(1, len(pts)):
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 <= 1e-18:
            continue
        t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2
        t = max(0.0, min(1.0, t))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(p[0] - px, p[1] - py)
        if d < best_d:
            best_s, best_d, best_i = (cum[i - 1] + t * math.sqrt(seg2), d, i)
    if best_i == 0:
        return (0.0, 0.0, 0.0)
    tx, ty = _tangent_at(pts, cum, best_s)
    pp_x, pp_y = point_at(pts, cum, best_s)
    signed = tx * (p[1] - pp_y) - ty * (p[0] - pp_x)
    return (best_s, signed, best_d)


def _pct(sorted_vals, q):
    """Linear-interpolated percentile of an ALREADY-SORTED list."""
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = q * (len(sorted_vals) - 1)
    lo = int(math.floor(idx))
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = idx - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def _turn_angle_deg(pts, i):
    if i <= 0 or i >= len(pts) - 1:
        return 0.0
    ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
    bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    cosang = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
    return math.degrees(math.acos(cosang))


# ── Metrics that see what the one-sided RMS cannot ────────────────────
#
# Each of these exists because a real ME3B V1 failure was invisible to
# path_error. Evidence: logs/prints/print_20260727_195339_*.jsonl — the follower
# commanded ~1 mm/s for 37 s while arc-length progress sat frozen at
# 0.121 → 0.509 mm, travelling 12.4 mm to net 1.0 mm. path_error scored that run
# on the short prefix it reached and reported a small, flattering number.

def signed_cross_track(samples, ideal_pts):
    """Signed cross-track stats (µm) + the per-sample series.

    ``inside_um`` / ``outside_um`` are the worst excursions to each side, so a
    corner that is CUT reads differently from one that is OVERSHOT.
    """
    if not samples or len(ideal_pts) < 2:
        return {"p50": 0.0, "p95": 0.0, "max": 0.0,
                "inside_um": 0.0, "outside_um": 0.0, "series": []}
    cum = _cum(ideal_pts)
    series = [_project(ideal_pts, cum, p)[1] * 1000.0 for p in samples]
    mags = sorted(abs(v) for v in series)
    return {
        "p50": _pct(mags, 0.50), "p95": _pct(mags, 0.95), "max": mags[-1],
        "inside_um": abs(min(series)) if min(series) < 0 else 0.0,
        "outside_um": max(series) if max(series) > 0 else 0.0,
        "series": series,
    }


def two_sided_deviation(samples, ideal_pts):
    """Both directions of the deviation — the fix for the coverage blind spot.

    ``a2i`` (actual→ideal) is what ``path_error`` measures. ``i2a``
    (ideal→actual) asks the question it never asks: is every part of the ideal
    path near something the stage actually did? A run that skipped a feature, cut
    a corner, or stopped early has a small ``a2i`` and a large ``i2a``.
    """
    if not samples or len(ideal_pts) < 2:
        return {"a2i_p95_um": 0.0, "a2i_max_um": 0.0, "i2a_p95_um": 0.0,
                "i2a_max_um": 0.0, "hausdorff_um": 0.0, "chamfer_um": 0.0}
    a2i = sorted(_SegGrid(ideal_pts).nearest_um(p) for p in samples)
    actual_poly = [(p[0], p[1]) for p in samples]
    if len(actual_poly) >= 2:
        grid_a = _SegGrid(actual_poly)
        i2a = sorted(grid_a.nearest_um(q) for q in ideal_pts)
    else:
        i2a = sorted(_dist(q, actual_poly[0]) * 1000.0 for q in ideal_pts)
    return {
        "a2i_p95_um": _pct(a2i, 0.95), "a2i_max_um": a2i[-1],
        "i2a_p95_um": _pct(i2a, 0.95), "i2a_max_um": i2a[-1],
        "hausdorff_um": max(a2i[-1], i2a[-1]),
        "chamfer_um": 0.5 * (sum(a2i) / len(a2i) + sum(i2a) / len(i2a)),
    }


def coverage(samples, ideal_pts, tol_um):
    """Fraction of the IDEAL path arc length that came within ``tol_um`` of some
    recorded sample."""
    if not samples or len(ideal_pts) < 2:
        return 0.0
    actual_poly = [(p[0], p[1]) for p in samples]
    if len(actual_poly) < 2:
        near = [_dist(q, actual_poly[0]) * 1000.0 <= tol_um for q in ideal_pts]
    else:
        grid_a = _SegGrid(actual_poly)
        near = [grid_a.nearest_um(q) <= tol_um for q in ideal_pts]
    cum = _cum(ideal_pts)
    total = cum[-1]
    if total <= 1e-12:
        return 1.0 if all(near) else 0.0
    covered = sum(cum[i] - cum[i - 1] for i in range(1, len(ideal_pts))
                  if near[i - 1] and near[i])
    return covered / total


def _monotone_progress(samples, pts, cum, max_jump_mm=None):
    """Forward-windowed monotone arc-length progress for each sample.

    A GLOBAL nearest projection cannot measure progress on a closed or
    self-touching path: on a closed square the final point is spatially identical
    to the start, so it projects to ``s = 0`` and a perfect run reads as 98.8 %
    complete. Walking a forward-only cursor (the same idea the follower uses)
    fixes that, while the window keeps a spiral from snapping onto a later loop.

    Returns the list of committed ``s`` values (non-decreasing).
    """
    total = cum[-1]
    if total <= 1e-12 or len(pts) < 2:
        return [0.0 for _ in samples]
    window = float(max_jump_mm) if max_jump_mm else max(2.0, 0.10 * total)
    out = []
    s_cur = 0.0
    seg_lo = 1
    for p in samples:
        best_s, best_d = s_cur, float("inf")
        i = seg_lo
        while i < len(pts):
            if cum[i - 1] > s_cur + window:
                break
            ax, ay = pts[i - 1]
            bx, by = pts[i]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            if seg2 > 1e-18:
                t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2
                t = max(0.0, min(1.0, t))
                px, py = ax + t * dx, ay + t * dy
                d = math.hypot(p[0] - px, p[1] - py)
                if d < best_d:
                    best_d = d
                    best_s = cum[i - 1] + t * math.sqrt(seg2)
            i += 1
        s_cur = max(s_cur, min(best_s, s_cur + window))
        # Advance the segment cursor to keep the scan O(1) amortised.
        while (seg_lo + 1 < len(pts)) and cum[seg_lo] < s_cur - 1e-9:
            seg_lo += 1
        out.append(s_cur)
    return out


def completion_fraction(samples, ideal_pts):
    """How much of the path was traversed: monotone arc-length progress over the
    total. The stalled run reached 0.509 of 59.3 mm ≈ 0.009 — and ``path_error``
    reported a small, flattering number for it."""
    if not samples or len(ideal_pts) < 2:
        return 0.0
    cum = _cum(ideal_pts)
    total = cum[-1]
    if total <= 1e-12:
        return 1.0
    prog = _monotone_progress(samples, ideal_pts, cum)
    return min(1.0, (prog[-1] if prog else 0.0) / total)


def path_length_ratio(samples, ideal_pts):
    """THE DITHER DETECTOR.

    ``travel_mm`` is how far the stage actually moved; ``progress_mm`` is how far
    along the path it got. The ratio is ~1 for a clean run and explodes when the
    stage oscillates in place — ≈12 for the logged deadlock, which ``path_error``
    scored as near-zero because every dithering sample sat on top of the line.
    """
    if not samples:
        return {"travel_mm": 0.0, "ideal_mm": 0.0, "progress_mm": 0.0,
                "ratio_total": 1.0, "dither_ratio": 1.0}
    travel = sum(_dist(samples[i - 1], samples[i])
                 for i in range(1, len(samples)))
    cum = _cum(ideal_pts) if len(ideal_pts) >= 2 else [0.0]
    ideal_len = cum[-1]
    progress = completion_fraction(samples, ideal_pts) * ideal_len
    if progress > 1e-6:
        dither = travel / progress
    else:
        dither = float("inf") if travel > 1e-6 else 1.0
    return {"travel_mm": travel, "ideal_mm": ideal_len,
            "progress_mm": progress,
            "ratio_total": (travel / ideal_len) if ideal_len > 1e-9 else 1.0,
            "dither_ratio": dither}


def reversal_count(samples, ideal_pts, hyst_um=20.0):
    """Along-track direction flips — "back-and-forth over the same line" made
    countable. The hysteresis band stops encoder noise counting."""
    if len(samples) < 3 or len(ideal_pts) < 2:
        return {"n": 0, "per_mm": 0.0}
    cum = _cum(ideal_pts)
    ss = [_project(ideal_pts, cum, p)[0] for p in samples]
    hyst = hyst_um / 1000.0
    n = 0
    direction = 0
    anchor = ss[0]
    for s in ss[1:]:
        d = s - anchor
        if abs(d) < hyst:
            continue
        new_dir = 1 if d > 0 else -1
        if direction and new_dir != direction:
            n += 1
        direction = new_dir
        anchor = s
    total = cum[-1]
    return {"n": n, "per_mm": (n / total) if total > 1e-9 else 0.0}


def corner_overshoot(samples, ideal_pts, corner_angle_deg=30.0, window_mm=1.0):
    """Worst deviation near each corner — what corner tuning is for, and what a
    whole-path RMS averages away."""
    if len(samples) < 2 or len(ideal_pts) < 3:
        return {"per_corner": [], "p95_um": 0.0, "max_um": 0.0, "n": 0}
    cum = _cum(ideal_pts)
    corners = [(cum[i], _turn_angle_deg(ideal_pts, i))
               for i in range(1, len(ideal_pts) - 1)
               if _turn_angle_deg(ideal_pts, i) > corner_angle_deg]
    if not corners:
        return {"per_corner": [], "p95_um": 0.0, "max_um": 0.0, "n": 0}
    proj = [_project(ideal_pts, cum, p) for p in samples]
    per = []
    for sc, ang in corners:
        worst = 0.0
        for s, signed, _d in proj:
            if abs(s - sc) <= window_mm:
                worst = max(worst, abs(signed) * 1000.0)
        per.append({"s_mm": sc, "angle_deg": ang, "overshoot_um": worst})
    vals = sorted(c["overshoot_um"] for c in per)
    return {"per_corner": per, "p95_um": _pct(vals, 0.95),
            "max_um": vals[-1], "n": len(per)}


def along_track_lag(samples, ideal_pts, commanded_speed_mm_s):
    """How far BEHIND the commanded schedule the stage ran. Needs timestamps and
    returns ``None`` for an untimed trace rather than guessing."""
    if not samples or len(ideal_pts) < 2 or not commanded_speed_mm_s:
        return {"mean_mm": None, "p95_mm": None, "mean_s": None}
    timed = [p for p in samples if len(p) > 2 and p[2] is not None]
    if len(timed) < 2:
        return {"mean_mm": None, "p95_mm": None, "mean_s": None}
    cum = _cum(ideal_pts)
    t0 = timed[0][2]
    lags = []
    for p in timed:
        s = _project(ideal_pts, cum, p)[0]
        expected = commanded_speed_mm_s * (p[2] - t0)
        lags.append(max(0.0, min(expected, cum[-1]) - s))
    mean_mm = sum(lags) / len(lags)
    return {"mean_mm": mean_mm, "p95_mm": _pct(sorted(lags), 0.95),
            "mean_s": mean_mm / commanded_speed_mm_s}


def throughput(samples, ideal_pts, commanded_speed_mm_s=None, wall_s=None):
    """Achieved speed vs commanded — the term that stops an optimiser "winning"
    by going slower, which is exactly what the lookahead sweep did."""
    cum = _cum(ideal_pts) if len(ideal_pts) >= 2 else [0.0]
    ideal_len = cum[-1]
    nominal = (ideal_len / commanded_speed_mm_s) if commanded_speed_mm_s else None
    if wall_s is None:
        timed = [p for p in samples if len(p) > 2 and p[2] is not None]
        wall_s = (timed[-1][2] - timed[0][2]) if len(timed) >= 2 else None
    if not wall_s or wall_s <= 0:
        return {"achieved_mm_s": None, "commanded_mm_s": commanded_speed_mm_s,
                "efficiency": None, "wall_s": wall_s, "nominal_s": nominal,
                "time_ratio": None}
    achieved = ideal_len * completion_fraction(samples, ideal_pts) / wall_s
    return {
        "achieved_mm_s": achieved, "commanded_mm_s": commanded_speed_mm_s,
        "efficiency": (achieved / commanded_speed_mm_s)
        if commanded_speed_mm_s else None,
        "wall_s": wall_s, "nominal_s": nominal,
        "time_ratio": (wall_s / nominal) if nominal and nominal > 0 else None,
    }


def dwell_events(samples, min_dwell_s=0.25, move_tol_um=15.0):
    """Unintended stops — a stage that pauses mid-path deposits a blob."""
    timed = [p for p in samples if len(p) > 2 and p[2] is not None]
    if len(timed) < 3:
        return {"events": [], "n": 0, "total_s": 0.0}
    events = []
    tol_mm = move_tol_um / 1000.0
    i = 0
    while i < len(timed) - 1:
        j = i + 1
        while j < len(timed) and _dist(timed[i], timed[j]) <= tol_mm:
            j += 1
        dur = timed[j - 1][2] - timed[i][2]
        if dur >= min_dwell_s and (j - i) >= 3:
            events.append({"t": timed[i][2], "dur_s": dur,
                           "x": timed[i][0], "y": timed[i][1]})
            i = j
        else:
            i += 1
    return {"events": events, "n": len(events),
            "total_s": sum(e["dur_s"] for e in events)}


# ── One report + one score ────────────────────────────────────────────

def path_report(samples, ideal_pts, *, commanded_speed_mm_s=None,
                resolution_um=30.0, corner_angle_deg=30.0, status="ok",
                wall_s=None) -> dict:
    """Every metric for one run, in one dict — the single entry point callers use.

    Keeps ``legacy`` = :func:`path_error` inside it so historical RMS numbers stay
    directly comparable after the switch.
    """
    samples = as_samples(samples)
    rep = {
        "n": len(samples),
        "status": status,
        "resolution_um": float(resolution_um),
        "legacy": path_error(samples, ideal_pts),
        "signed": signed_cross_track(samples, ideal_pts),
        "two_sided": two_sided_deviation(samples, ideal_pts),
        "coverage": coverage(samples, ideal_pts, resolution_um * 3.0),
        "completion": completion_fraction(samples, ideal_pts),
        "length": path_length_ratio(samples, ideal_pts),
        "reversals": reversal_count(samples, ideal_pts),
        "corners": corner_overshoot(samples, ideal_pts,
                                    corner_angle_deg=corner_angle_deg),
        "lag": along_track_lag(samples, ideal_pts, commanded_speed_mm_s),
        "throughput": throughput(samples, ideal_pts, commanded_speed_mm_s,
                                 wall_s=wall_s),
    }
    # Flattened headline values, for logs and table columns.
    rep["rms_um"] = rep["legacy"]["rms_um"]
    rep["max_um"] = rep["legacy"]["max_um"]
    rep["p95_um"] = max(rep["two_sided"]["a2i_p95_um"],
                        rep["two_sided"]["i2a_p95_um"])
    rep["hausdorff_um"] = rep["two_sided"]["hausdorff_um"]
    rep["dither_ratio"] = rep["length"]["dither_ratio"]
    rep["completion_frac"] = rep["completion"]
    rep["corner_p95_um"] = rep["corners"]["p95_um"]
    rep["wall_s"] = rep["throughput"]["wall_s"]
    rep["time_ratio"] = rep["throughput"]["time_ratio"]
    return rep


#: Default objective weights. Recorded with every run so an old score can be
#: recomputed offline under different weights without re-running hardware.
OBJECTIVE_DEFAULTS = {
    "w_dev": 1.0,        # p95 two-sided deviation, in resolution elements
    "w_max": 0.5,        # worst-case deviation
    "w_time": 0.25,      # slower-than-commanded penalty (stops "win by crawling")
    "w_corner": 0.5,     # corner overshoot above the element
    "completion_min": 0.99,
    "dither_max": 1.2,
    "runaway_mult": 20.0,
}


def composite_score(report, *, resolution_um=None, weights=None) -> dict:
    """Turn a :func:`path_report` into ONE number an optimiser can minimise.

    Hard failures score ``1000 + a graded penalty`` rather than ``inf``:
    coordinate descent compares with ``<``, so a flat ``inf`` across an
    all-failing grid leaves the search unable to move at all — and the tuning
    this machine currently ships with IS in a failing region. A graded penalty
    lets the search climb out.

    The ``w_time`` term is what stops the optimiser trading speed for accuracy
    (the observed "the sweep just brings the velocity down"), and ``completion``
    is weighted as a hard failure rather than a trade because a run that stopped
    early must never win on deviation.

    Returns ``{"score", "pass", "fail_reasons", "dev", "time", "corner"}``.
    """
    w = dict(OBJECTIVE_DEFAULTS)
    if weights:
        w.update(weights)
    res = float(resolution_um or report.get("resolution_um") or 30.0)
    res = max(1e-6, res)

    completion = float(report.get("completion_frac") or 0.0)
    dither = report.get("dither_ratio")
    dither = float(dither) if dither not in (None, float("inf")) else 1e9
    haus = float(report.get("hausdorff_um") or 0.0)
    status = report.get("status") or "ok"

    fails = []
    penalty = 0.0
    if completion < w["completion_min"]:
        fails.append("incomplete")
        penalty += 100.0 * (w["completion_min"] - completion)
    if dither > w["dither_max"]:
        fails.append("dither")
        penalty += 20.0 * min(50.0, dither - w["dither_max"])
    if haus > w["runaway_mult"] * res:
        fails.append("runaway")
        penalty += 10.0 * (haus / res - w["runaway_mult"])
    if status != "ok":
        fails.append(f"status:{status}")
        penalty += 200.0

    dev = float(report.get("p95_um") or 0.0) / res
    mx = float(report.get("max_um") or 0.0) / res
    tr = report.get("time_ratio")
    time_pen = max(0.0, (float(tr) - 1.0)) if tr else 0.0
    corner = max(0.0, float(report.get("corner_p95_um") or 0.0) / res - 1.0)
    soft = (w["w_dev"] * dev + w["w_max"] * mx
            + w["w_time"] * time_pen + w["w_corner"] * corner)

    if fails:
        return {"score": 1000.0 + penalty + soft, "pass": False,
                "fail_reasons": fails, "dev": dev, "time": time_pen,
                "corner": corner}
    ok = (float(report.get("p95_um") or 0.0) <= res
          and float(report.get("max_um") or 0.0) <= 3.0 * res)
    return {"score": soft, "pass": bool(ok), "fail_reasons": [],
            "dev": dev, "time": time_pen, "corner": corner}


def verdict_for(report_or_metrics, resolution_um) -> str:
    """``"pass"`` / ``"marginal"`` / ``"fail"`` for the robustness matrix.

    Requires BOTH ``rms <= element`` AND ``max <= 2× element``: RMS alone lets a
    single large corner blow-out pass, and a corner blow-out is exactly what a
    robustness sweep exists to catch.
    """
    res = max(1e-6, float(resolution_um or 30.0))
    m = report_or_metrics or {}
    rms = float(m.get("rms_um") or 0.0)
    mx = float(m.get("max_um") or 0.0)
    if not m.get("n"):
        return "fail"
    comp = m.get("completion_frac")
    if comp is not None and float(comp) < 0.99:
        return "fail"
    if rms <= res and mx <= 2.0 * res:
        return "pass"
    if rms <= 2.0 * res and mx <= 4.0 * res:
        return "marginal"
    return "fail"
