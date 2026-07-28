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

# Ordered so the UI combo reads corners → curves → reversals → dense.
CHALLENGE_SHAPES = ["Square", "Circle", "Star", "Zigzag", "Spiral"]

# What each shape is designed to expose (shown as a hint in the UI).
SHAPE_HINTS = {
    "Square": "4 sharp 90° corners — accel/decel + overshoot at corners.",
    "Circle": "constant curvature — steady tracking + how round the loop stays.",
    "Star": "5 sharp reversals (points) — the hardest corners + direction flips.",
    "Zigzag": "back-and-forth sharp turns — repeated accel reversals (backlash).",
    "Spiral": "dense concentric loops — sustained curvature + loop spacing.",
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


def make_shape(name: str, size_mm: float, step_mm: float = 0.5):
    """Ideal path (polyline, mm, centred at origin) for a challenge shape."""
    size = max(0.5, float(size_mm))
    step = max(0.05, float(step_mm))
    key = (name or "").strip().lower()
    if key == "square":
        return _square(size, step)
    if key == "circle":
        return _circle(size, step)
    if key == "star":
        return _star(size, step)
    if key == "zigzag":
        return _zigzag(size, step)
    if key == "spiral":
        return _spiral(size, step)
    raise ValueError(f"unknown challenge shape: {name!r}")


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


def path_error(samples_xy, ideal_pts) -> dict:
    """Compare recorded actual positions to the ideal polyline.

    ``samples_xy`` — list of ``(x, y)`` ACTUAL stage positions (mm), in order.
    ``ideal_pts``  — the ideal polyline (mm).

    Returns ``{"rms_um", "max_um", "mean_um", "n"}`` — the cross-track deviation
    (min distance from each actual sample to the ideal path) in µm. This is the
    path-following fidelity: lower = the stage traced the shape more faithfully.
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
