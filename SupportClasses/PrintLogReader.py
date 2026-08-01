"""v7.7 — read back a print's JSONL execution log.

``PrintExecutionLogger`` has written one JSONL per run to ``logs/prints/`` since
v7.5.x, and until now **nothing in the repo ever read one**: Quick Print printed
the filename into a grey label and that was the whole post-print story. This
module is the missing half — a pure, Qt-free reader plus an adapter that turns a
log into the exact sample shape the four ``gui/pages/print_results.py`` widgets
already consume, so the report reuses those widgets rather than growing a new
charting stack.

Design notes worth keeping in mind:

* **Tolerant by construction.** A log may be truncated (crash mid-write) or still
  be open (the report may be built while the last flush is in flight). Unparsable
  lines are counted, not raised.
* **``sample.lag_um`` is NOT tracking error.** It is measured against the last
  commanded XY *target*, and in velocity / feed-plan mode nothing updates that
  target inside the control loop — so it degrades to "distance from the path
  start". The honest per-tick error is ``vel_sample.cross_um``, and that is what
  this module reports. Nothing here surfaces ``lag_um`` as accuracy.
* **``planned_x/y`` is the ideal point at the achieved arc length**, not the
  pursuit carrot. The carrot sits a full lookahead ahead of the stage, so an
  error measured against it would mostly measure the lookahead. Projecting onto
  the ideal at ``s_mm`` reproduces the executor's own perpendicular ``cross_um``
  (asserted by the tests).
* **Units match ``PrintRecorder``**: positions and errors in **mm** (the widgets
  multiply by 1000 for display — see ``print_results._error_to_color``).
"""

from __future__ import annotations

import json
import logging
import math
import os
from dataclasses import dataclass, field
from typing import Any, Optional

logger = logging.getLogger(__name__)

#: Events whose absence simply means "this executor doesn't emit them".
_KNOWN = (
    "job_start", "job_end", "command_start", "command_end", "path_start",
    "path_end", "path_segment", "path_barrier", "plan_section", "vel_sample",
    "openvel_sample", "vel_stall", "feed_plan_fallback", "sample", "xy_cmd",
    "xy_arrival", "settle_wait", "z_move", "extrude", "pump_relief",
    "speed_set", "traj_start", "traj_wp", "traj_end", "plan_step",
    "abort_requested", "abort_motion_killed", "zp_disconnected", "error",
)

#: A `sample` row is matched to a `vel_sample` only within this much time (s).
_MERGE_MAX_DT_S = 0.6


@dataclass
class PrintLog:
    """A parsed execution log."""
    path: str = ""
    events: dict[str, list[dict]] = field(default_factory=dict)
    n_lines: int = 0
    n_bad_lines: int = 0
    truncated: bool = False

    # ── convenience views ──
    def of(self, ev: str) -> list[dict]:
        return self.events.get(ev, [])

    @property
    def manifest(self) -> dict:
        rows = self.of("job_start")
        return rows[0] if rows else {}

    @property
    def settings(self) -> dict:
        return self.manifest.get("settings") or {}

    @property
    def zero_position(self) -> dict:
        return self.manifest.get("zero_position") or {}

    @property
    def job_name(self) -> str:
        job = self.manifest.get("job") or {}
        return str(job.get("name") or self.manifest.get("job_name") or "print")

    @property
    def sections(self) -> list[dict]:
        return self.of("plan_section")

    @property
    def vel_samples(self) -> list[dict]:
        return self.of("vel_sample")

    @property
    def samples(self) -> list[dict]:
        return self.of("sample")

    @property
    def path_start(self) -> dict:
        rows = self.of("path_start")
        return rows[-1] if rows else {}

    @property
    def path_end(self) -> dict:
        rows = self.of("path_end")
        return rows[-1] if rows else {}

    @property
    def status(self) -> str:
        end = self.of("job_end")
        if end:
            return str(end[-1].get("status") or "unknown")
        if self.of("error"):
            return "error"
        # No job_end → the log was cut off (crash, kill, or still running).
        return "incomplete"

    @property
    def duration_s(self) -> float:
        end = self.of("job_end")
        if end and end[-1].get("wall_s") is not None:
            try:
                return float(end[-1]["wall_s"])
            except (TypeError, ValueError):
                pass
        return float(self._last_t())

    @property
    def mode(self) -> str:
        """The executor that ran the print path."""
        ps = self.path_start
        if ps.get("feed_plan"):
            return "feed_plan"
        m = str(ps.get("mode") or "")
        if m:
            return m
        if self.of("path_segment"):
            return "discrete"
        return str(self.manifest.get("exec_mode") or "unknown")

    @property
    def errors(self) -> list[dict]:
        return self.of("error")

    def commands(self) -> list[dict]:
        """Pair `command_start` with `command_end` → one row per command with a
        duration and the wall-clock window it occupied."""
        ends = {int(e.get("i", -1)): e for e in self.of("command_end")}
        out = []
        starts = self.of("command_start")
        for k, st in enumerate(starts):
            i = int(st.get("i", k + 1))
            en = ends.get(i, {})
            t0 = _f(st.get("t"), 0.0)
            # A command with no `command_end` (aborted) runs to the next start.
            if en:
                dur = _f(en.get("duration_s"), 0.0)
                t1 = t0 + dur
            else:
                nxt = starts[k + 1] if k + 1 < len(starts) else None
                t1 = _f(nxt.get("t"), self._last_t()) if nxt else self._last_t()
                dur = max(0.0, t1 - t0)
            out.append({"i": i, "type": str(st.get("type") or ""),
                        "label": str(st.get("label") or ""),
                        "t0": t0, "t1": t1, "duration_s": dur,
                        "completed": bool(en)})
        return out

    def ideal_points(self) -> Optional[list[tuple[float, float]]]:
        """The printed path in zero-ref mm, when the log recorded it.

        v7.7 logs store (possibly decimated) PRINT_PATH points on `path_start`;
        older logs stored only ``n_points``, so scoring a historical run against
        its ideal is impossible and callers must degrade gracefully.
        """
        pts = self.path_start.get("points")
        if not isinstance(pts, list) or len(pts) < 2:
            return None
        out = []
        for p in pts:
            try:
                out.append((float(p[0]), float(p[1])))
            except (TypeError, ValueError, IndexError):
                return None
        return out

    def comm_health(self) -> dict:
        """ZP link health across the run, from the 5 Hz sampler's counters
        (monotonic, so the delta over the run is what matters)."""
        rows = [r for r in self.samples if r.get("zp_cmd") is not None]
        if not rows:
            return {}
        first, last = rows[0], rows[-1]

        def d(key):
            return max(0, int(_f(last.get(key), 0)) - int(_f(first.get(key), 0)))

        dropped = sum(1 for r in self.samples if r.get("zp_conn") is False)
        return {"commands": d("zp_cmd"), "ok": d("zp_ok"),
                "ok_failures": d("zp_ok_fail"), "board_resets": d("zp_reset"),
                "disconnected_samples": dropped, "n_samples": len(rows)}

    def _last_t(self) -> float:
        t = 0.0
        for rows in self.events.values():
            if rows:
                t = max(t, _f(rows[-1].get("t"), 0.0))
        return t


def as_float(v, default=0.0) -> float:
    """Tolerant float coercion for log fields (public — the report uses it)."""
    try:
        if v is None or isinstance(v, bool):
            return default
        return float(v)
    except (TypeError, ValueError):
        return default


#: Internal shorthand.
_f = as_float


def read_log(path: str | os.PathLike) -> PrintLog:
    """Parse a JSONL execution log. Never raises on malformed content."""
    log = PrintLog(path=str(path))
    events: dict[str, list[dict]] = {}
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        logger.warning("could not read print log %s: %s", path, exc)
        return log

    for k, raw in enumerate(lines):
        line = raw.strip()
        if not line:
            continue
        log.n_lines += 1
        try:
            rec = json.loads(line)
        except (ValueError, TypeError):
            log.n_bad_lines += 1
            # A bad LAST line is the signature of a truncated write, not
            # corruption — the logger flushes per line.
            if k == len(lines) - 1:
                log.truncated = True
            continue
        if not isinstance(rec, dict):
            log.n_bad_lines += 1
            continue
        ev = str(rec.get("ev") or "")
        if not ev:
            log.n_bad_lines += 1
            continue
        events.setdefault(ev, []).append(rec)
    log.events = events
    return log


# ── geometry helpers (mirrors of the executor's own projection) ───────

def _cumulative(pts: list[tuple[float, float]]) -> list[float]:
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return cum


def point_at_arclength(pts: list[tuple[float, float]], cum: list[float],
                       s: float) -> tuple[float, float]:
    """The point on the polyline at arc length ``s`` (clamped to the ends)."""
    if not pts:
        return (0.0, 0.0)
    if s <= 0 or len(pts) == 1:
        return pts[0]
    total = cum[-1]
    if s >= total:
        return pts[-1]
    # cum is sorted; find the segment containing s.
    lo, hi = 0, len(cum) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    seg = cum[lo + 1] - cum[lo]
    if seg <= 1e-12:
        return pts[lo]
    f = (s - cum[lo]) / seg
    ax, ay = pts[lo]
    bx, by = pts[lo + 1]
    return (ax + f * (bx - ax), ay + f * (by - ay))


def section_bases(log: PrintLog) -> dict[int, float]:
    """``section index → arc length of all preceding sections`` (mm).

    Needed because a feed-plan ``vel_sample`` records **section-local** ``s_mm``
    (with ``tot_mm`` = that section's length), while the ideal path it must be
    projected onto is the WHOLE toolpath. Without this offset every section after
    the first would be compared against the wrong part of the path. The legacy
    velocity executor emits no ``plan_section`` records and its ``s_mm`` is
    already global, so the map is empty and the offset is 0.
    """
    out: dict[int, float] = {}
    for sec in log.sections:
        try:
            out[int(sec["index"])] = float(sec.get("base_s_mm") or 0.0)
        except (KeyError, TypeError, ValueError):
            continue
    return out


def _global_s(row: dict, bases: dict[int, float]) -> float:
    s = _f(row.get("s_mm"))
    sec = row.get("sec")
    if sec is None:
        return s
    try:
        return bases.get(int(sec), 0.0) + s
    except (TypeError, ValueError):
        return s


def _nearest_sample_by_t(rows: list[dict], t: float,
                         cursor: list[int]) -> Optional[dict]:
    """Nearest-in-time row, walking a monotone cursor (both streams are ~5 Hz
    and time-ordered, so this is O(n) overall)."""
    if not rows:
        return None
    i = cursor[0]
    n = len(rows)
    while i + 1 < n and abs(_f(rows[i + 1].get("t")) - t) <= \
            abs(_f(rows[i].get("t")) - t):
        i += 1
    cursor[0] = i
    if abs(_f(rows[i].get("t")) - t) > _MERGE_MAX_DT_S:
        return None
    return rows[i]


# ── the adapter to the print_results widgets ─────────────────────────

def to_recorder_samples(log: PrintLog,
                        ideal_pts: Optional[list[tuple[float, float]]] = None
                        ) -> tuple[list[dict], dict]:
    """Build ``(samples, meta)`` in the shape ``PrintRecorder`` produces and the
    four report widgets consume.

    Sample keys: ``t, planned_x/y/z, actual_x/y/z, tracking_error_xy,
    tracking_error_z, segment_id, is_travel, is_retract`` — positions and errors
    in **mm**.

    ``ideal_pts`` (zero-ref mm) supplies ``planned_x/y``; without it (an older log
    that did not record the path) the planned channel is left equal to the actual
    one and the errors are 0.0, so the widgets still render the executed trace
    honestly instead of inventing a deviation. Callers should check
    ``meta["has_planned"]``.
    """
    ideal = ideal_pts if ideal_pts is not None else log.ideal_points()
    cum = _cumulative(ideal) if ideal else []
    has_planned = bool(ideal) and len(ideal) >= 2

    zp = log.zero_position
    z0x, z0y = _f(zp.get("x"), 0.0), _f(zp.get("y"), 0.0)

    cmds = log.commands()

    def _phase(t: float) -> tuple[bool, bool, int]:
        """(is_travel, is_retract, command index) at time ``t``."""
        for c in cmds:
            if c["t0"] <= t <= c["t1"]:
                ty = c["type"]
                return (ty in ("move_xy", "home_xy", "travel_xy"),
                        ty in ("travel_up", "move_z", "travel_down"),
                        c["i"])
        return (False, False, -1)

    zrows = log.samples
    zcur = [0]
    out: list[dict] = []

    vel = log.vel_samples
    bases = section_bases(log)
    if vel:
        for r in vel:
            t = _f(r.get("t"))
            ax, ay = _f(r.get("x_mm")), _f(r.get("y_mm"))
            s_global = _global_s(r, bases)
            if has_planned:
                px, py = point_at_arclength(ideal, cum, s_global)
                # cross_um is the executor's own perpendicular error; prefer it
                # (it is what the control loop acted on) and fall back to the
                # geometric distance when it is absent.
                cross_um = r.get("cross_um")
                err = (_f(cross_um) / 1000.0 if cross_um is not None
                       else math.hypot(px - ax, py - ay))
            else:
                px, py, err = ax, ay, 0.0
            zr = _nearest_sample_by_t(zrows, t, zcur) or {}
            az = _f(zr.get("z"), 0.0)
            travel, retract, seg_cmd = _phase(t)
            sec = r.get("sec")
            out.append({
                "t": t,
                "planned_x": px, "planned_y": py, "planned_z": az,
                "actual_x": ax, "actual_y": ay, "actual_z": az,
                "tracking_error_xy": err, "tracking_error_z": 0.0,
                "segment_id": int(sec) if sec is not None else seg_cmd,
                "is_travel": travel, "is_retract": retract,
                # extras (ignored by the widgets, used by the report)
                "s_mm": s_global, "sec_s_mm": _f(r.get("s_mm")),
                "tot_mm": _f(r.get("tot_mm")),
                "vx": _f(r.get("vx")), "vy": _f(r.get("vy")),
                "v_meas_mm_s": (None if r.get("v_meas_mm_s") is None
                                else _f(r.get("v_meas_mm_s"))),
                "deposited_uL": (None if r.get("deposited_uL") is None
                                 else _f(r.get("deposited_uL"))),
            })
    else:
        # No closed-loop record. The 5 Hz sampler still logged absolute
        # positions, so the executed trace can be drawn — but with no measured
        # deviation (open-loop mode reads no position during the path at all).
        for r in zrows:
            t = _f(r.get("t"))
            ax = (_f(r.get("x_um")) - z0x) / 1000.0
            ay = (_f(r.get("y_um")) - z0y) / 1000.0
            az = _f(r.get("z"), 0.0)
            travel, retract, seg_cmd = _phase(t)
            out.append({
                "t": t,
                "planned_x": ax, "planned_y": ay, "planned_z": az,
                "actual_x": ax, "actual_y": ay, "actual_z": az,
                "tracking_error_xy": 0.0, "tracking_error_z": 0.0,
                "segment_id": seg_cmd,
                "is_travel": travel, "is_retract": retract,
            })
        has_planned = False

    meta = {
        "job_name": log.job_name,
        "status": log.status,
        "timestamp_str": str(log.manifest.get("ts") or ""),
        "duration_s": log.duration_s,
        "mode": log.mode,
        "has_planned": has_planned,
        "has_position_feedback": bool(vel),
        "log_path": log.path,
        "truncated": log.truncated,
        "n_bad_lines": log.n_bad_lines,
        "summary": _summary(log, out),
        "workspace": {},
    }
    return out, meta


def _summary(log: PrintLog, samples: list[dict]) -> dict:
    """The ``meta["summary"]`` dict ``StatisticsPanel`` reads, plus report extras."""
    errs = [s["tracking_error_xy"] for s in samples if not s["is_travel"]]
    out: dict[str, Any] = {
        "total_samples": len(samples),
        "print_samples": sum(1 for s in samples if not s["is_travel"]),
        "travel_samples": sum(1 for s in samples if s["is_travel"]),
        "retract_samples": sum(1 for s in samples if s["is_retract"]),
        "num_segments": len({s["segment_id"] for s in samples}),
    }
    if errs:
        out["tracking_error_xy_mean_mm"] = sum(errs) / len(errs)
        out["tracking_error_xy_max_mm"] = max(errs)
    pe = log.path_end
    if pe:
        out["path_status"] = str(pe.get("status") or "")
        out["path_s_mm"] = _f(pe.get("s_mm"))
        out["path_total_mm"] = _f(pe.get("tot_mm"))
        out["path_wall_s"] = _f(pe.get("wall_s"))
    ps = log.path_start
    if ps:
        out["plan_est_s"] = _f(ps.get("plan_est_s"))
        out["n_sections"] = int(_f(ps.get("n_sections")))
        out["n_stops"] = int(_f(ps.get("n_stops")))
        out["element_um"] = _f(ps.get("element_um"))
        out["commanded_speed_mm_s"] = _f(ps.get("speed_mm_s"))
        out["flow_rate_uL_s"] = _f(ps.get("flow_rate_uL_s"))
    return out


# ── report-side analyses ─────────────────────────────────────────────

def section_stats(log: PrintLog) -> list[dict]:
    """Per feed-plan section: the plan's own numbers PLUS the deviation actually
    measured inside it, so the operator can see which feature cost accuracy."""
    by_sec: dict[int, list[float]] = {}
    for r in log.vel_samples:
        sec = r.get("sec")
        if sec is None:
            continue
        by_sec.setdefault(int(sec), []).append(abs(_f(r.get("cross_um"))))
    out = []
    for sec in log.sections:
        idx = int(_f(sec.get("index"), -1))
        errs = sorted(by_sec.get(idx, []))
        row = {
            "index": idx,
            "skipped": bool(sec.get("skipped")),
            "length_mm": _f(sec.get("length_mm")),
            "speed_mm_s": _f(sec.get("speed_mm_s")),
            "lookahead_mm": _f(sec.get("lookahead_mm")),
            "arrive_mm": _f(sec.get("arrive_mm")),
            "min_radius_mm": sec.get("min_radius_mm"),
            "reason": str(sec.get("reason") or ""),
            "n_samples": len(errs),
        }
        if errs:
            row["p95_um"] = errs[min(len(errs) - 1, int(0.95 * (len(errs) - 1)))]
            row["max_um"] = errs[-1]
            row["mean_um"] = sum(errs) / len(errs)
        out.append(row)
    return out


def restart_decomposition(log: PrintLog, *, window_mm: float = 0.5) -> dict:
    """Split the measured error into "near a section boundary" vs "mid-section".

    This is the mechanism the v7.6 hardware run exposed: with corner stops the
    vertex itself is HIT, and what remains is the transient as the next section
    spins up. Reporting the split tells the operator whether to spend effort on
    restarts or on the straight-line floor.
    """
    rows = log.vel_samples
    if not rows:
        return {}
    bounds, acc = [0.0], 0.0
    for sec in log.sections:
        acc += _f(sec.get("length_mm"))
        bounds.append(acc)
    if len(bounds) <= 2:
        return {}
    bases = section_bases(log)
    near, mid = [], []
    for r in rows:
        s_global = _global_s(r, bases)
        e = abs(_f(r.get("cross_um")))
        if any(abs(s_global - b) <= window_mm for b in bounds[1:-1]):
            near.append(e)
        else:
            mid.append(e)

    def _p95(v):
        if not v:
            return None
        v = sorted(v)
        return v[min(len(v) - 1, int(0.95 * (len(v) - 1)))]

    return {"window_mm": window_mm,
            "near_p95_um": _p95(near), "near_n": len(near),
            "mid_p95_um": _p95(mid), "mid_n": len(mid)}


def volume_reconciliation(log: PrintLog) -> dict:
    """What the pump was asked to do vs what the plan implied.

    ``extrude`` events are the discrete actuations (prime, retract, service);
    the velocity path deposits continuously, so its total comes from
    ``flow / speed × arc length`` — and from ``vel_sample.deposited_uL`` when the
    log carries it (v7.7+).
    """
    ps = log.path_start
    flow = _f(ps.get("flow_rate_uL_s"))
    speed = _f(ps.get("speed_mm_s"))
    total_mm = _f(log.path_end.get("tot_mm")) or _f(ps.get("path_len_mm"))
    planned = (flow / speed * total_mm) if (flow > 0 and speed > 0) else 0.0

    discrete = [r for r in log.of("extrude")]
    prime = sum(_f(r.get("vol_uL")) for r in discrete
                if _f(r.get("vol_uL")) > 0)
    aspirated = sum(-_f(r.get("vol_uL")) for r in discrete
                    if _f(r.get("vol_uL")) < 0)
    deposited = None
    for r in reversed(log.vel_samples):
        if r.get("deposited_uL") is not None:
            deposited = _f(r.get("deposited_uL"))
            break
    relief = sum(_f(r.get("uL")) for r in log.of("pump_relief"))
    return {"planned_path_uL": planned, "deposited_path_uL": deposited,
            "discrete_dispensed_uL": prime, "discrete_aspirated_uL": aspirated,
            "pressure_relief_uL": relief,
            "vol_per_mm_uL": (flow / speed) if speed > 0 else 0.0,
            "path_length_mm": total_mm}


def problems(log: PrintLog) -> list[dict]:
    """Everything that went wrong, in time order — for the report's Problems
    list. Includes the fluidic consequence of an abort, which used to be written
    only to the application log."""
    out: list[dict] = []
    for r in log.errors:
        out.append({"t": _f(r.get("t")), "kind": "error",
                    "text": str(r.get("message") or "unknown error"),
                    "traceback": str(r.get("traceback") or "")})
    for r in log.of("zp_disconnected"):
        out.append({"t": _f(r.get("t")), "kind": "zp_disconnect",
                    "text": "the Z/pump board dropped off USB mid-print"})
    for r in log.of("abort_requested"):
        out.append({"t": _f(r.get("t")), "kind": "abort",
                    "text": "abort requested — the dispensed volume is "
                            "indeterminate (a pump move may have been cut "
                            "mid-stroke); re-check the syringe fill"})
    for r in log.of("vel_stall"):
        out.append({"t": _f(r.get("t")), "kind": "stall",
                    "text": f"the follower stalled at s={_f(r.get('s_mm')):.2f} "
                            f"mm for {_f(r.get('held_s')):.1f} s"})
    for r in log.of("feed_plan_fallback"):
        out.append({"t": _f(r.get("t")), "kind": "fallback",
                    "text": f"feed planning unavailable "
                            f"({r.get('reason')}) — the legacy follower ran"})
    for r in log.of("settle_wait"):
        if r.get("ok") is False:
            out.append({"t": _f(r.get("t")), "kind": "settle",
                        "text": f"XY did not settle ({r.get('reason')}) — "
                                f"{_f(r.get('final_err_um')):.0f} µm short "
                                f"after {_f(r.get('duration_s')):.1f} s"})
    for r in log.of("z_move"):
        if r.get("ok") is False:
            out.append({"t": _f(r.get("t")), "kind": "z_move",
                        "text": f"Z move not confirmed "
                                f"({r.get('context')}) → {_f(r.get('z_mm')):.3f} mm"})
    out.sort(key=lambda d: d["t"])
    return out


def list_logs(log_dir: str | os.PathLike, limit: int = 50) -> list[dict]:
    """Newest-first listing of the logs in a directory (name, path, mtime, size)."""
    try:
        names = [n for n in os.listdir(log_dir) if n.endswith(".jsonl")]
    except OSError:
        return []
    rows = []
    for n in names:
        p = os.path.join(str(log_dir), n)
        try:
            st = os.stat(p)
        except OSError:
            continue
        rows.append({"name": n, "path": p, "mtime": st.st_mtime,
                     "size": st.st_size})
    rows.sort(key=lambda r: r["mtime"], reverse=True)
    return rows[:limit]
