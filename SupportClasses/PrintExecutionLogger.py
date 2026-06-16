"""
PrintExecutionLogger — machine-readable execution log for every print.

v7.5.x: Records one JSONL file per print run (any mode: discrete Quick
Print, hybrid plan execution, trajectory playback) so that post-hoc
analysis — human or Claude — can reconstruct exactly what the software
commanded versus what the hardware actually did, with timestamps.

Design goals:
- Zero extra serial traffic: the background sampler reads only the
  PositionPoller's cached positions (``cached=True``).
- Crash-resistant: every event is one JSON line, flushed immediately.
- Frame-complete: the job manifest records zero_position, safety limits
  and settings so absolute/zero-ref conversions can be replayed offline.
- Never break a print: every public method swallows its own exceptions.

File location: ``logs/prints/print_YYYYMMDD_HHMMSS_<job>.jsonl``
(``logs/prints/README.md`` documents the event schema).

Event lines share two fields:
    t   — seconds since job start (monotonic clock)
    ev  — event type string

Key events (see README for the full schema):
    job_start      — manifest: settings, command plan, frames, limits
    command_start  — each discrete PrintCommand as it begins
    path_start / path_segment / path_end — PRINT_PATH internals
    xy_cmd         — every XY move command sent (with clamp check)
    settle_wait    — closed-loop wait results (ok / timeout, error µm)
    traj_start / traj_wp / traj_end — trajectory-mode internals
    sample         — periodic actual position (cached) + lag vs target
    state / abort_requested / error / job_end — lifecycle
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEF_LOG_DIR = Path(__file__).resolve().parent.parent / "logs" / "prints"


def _jsonable(value):
    """Best-effort conversion of arbitrary objects to JSON-safe values."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "tolist"):  # numpy
        try:
            return value.tolist()
        except Exception:
            pass
    return repr(value)


class PrintExecutionLogger:
    """Append-only JSONL execution log for a single print run.

    Usage (PrintManager wires this automatically)::

        lg = PrintExecutionLogger(job_name="QuickCircle", mode="discrete")
        lg.start(controller, manifest_dict)
        lg.log("xy_cmd", x_mm=1.0, y_mm=2.0)
        ...
        lg.stop("completed")
    """

    #: Global kill switch (set env MEBP_PRINT_LOG=0 to disable).
    enabled: bool = os.environ.get("MEBP_PRINT_LOG", "1") != "0"

    def __init__(self, job_name: str = "print", mode: str = "discrete",
                 log_dir: Optional[str] = None, sample_hz: float = 5.0):
        self.job_name = job_name
        self.mode = mode
        self.sample_hz = max(0.5, float(sample_hz))
        self._dir = Path(log_dir) if log_dir else _DEF_LOG_DIR
        self.path: Optional[Path] = None

        self._file = None
        self._lock = threading.Lock()
        self._t0 = 0.0
        self._controller = None
        self._sampler: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._target_um: Optional[tuple[float, float]] = None
        self._stopped = False

    # ── Lifecycle ────────────────────────────────────────────────

    @property
    def active(self) -> bool:
        return self._file is not None and not self._stopped

    def start(self, controller, manifest: dict) -> Optional[str]:
        """Open the log file, write the manifest, start the sampler."""
        if not PrintExecutionLogger.enabled:
            return None
        try:
            self._dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe = "".join(c if c.isalnum() or c in "-_ " else "_"
                           for c in self.job_name)[:60].strip() or "print"
            self.path = self._dir / f"print_{stamp}_{safe}.jsonl"
            self._file = open(self.path, "w", encoding="utf-8")
            self._t0 = time.monotonic()
            self._controller = controller
            self._stopped = False

            self.log("job_start",
                     ts=datetime.now().isoformat(timespec="milliseconds"),
                     job_name=self.job_name, mode=self.mode,
                     **_jsonable(manifest))

            self._stop_evt.clear()
            self._sampler = threading.Thread(
                target=self._sample_loop, daemon=True,
                name="PrintExecLogSampler")
            self._sampler.start()
            logger.info(f"Print execution log: {self.path}")
            return str(self.path)
        except Exception as e:
            logger.warning(f"PrintExecutionLogger start failed: {e}")
            self._file = None
            return None

    def stop(self, status: str = "unknown") -> None:
        """Write the final event and close the file (idempotent)."""
        if self._file is None or self._stopped:
            return
        self._stopped = True
        try:
            self._stop_evt.set()
            if self._sampler is not None:
                self._sampler.join(timeout=2.0)
            self.log_force("job_end", status=status,
                           wall_s=round(time.monotonic() - self._t0, 3))
            with self._lock:
                self._file.close()
                self._file = None
        except Exception as e:
            logger.warning(f"PrintExecutionLogger stop failed: {e}")
            self._file = None

    # ── Event recording ──────────────────────────────────────────

    def log(self, ev: str, **fields) -> None:
        """Append one event line. Safe to call from any thread."""
        if not self.active:
            return
        self.log_force(ev, **fields)

    def log_force(self, ev: str, **fields) -> None:
        """Append even during stop() (used for the final job_end line)."""
        if self._file is None:
            return
        try:
            rec = {"t": round(time.monotonic() - self._t0, 4), "ev": ev}
            rec.update({k: _jsonable(v) for k, v in fields.items()})
            line = json.dumps(rec, ensure_ascii=False, default=repr)
            with self._lock:
                if self._file is not None:
                    self._file.write(line + "\n")
                    self._file.flush()
        except Exception as e:
            logger.debug(f"PrintExecutionLogger write failed: {e}")

    def log_error(self, message: str, exc: BaseException | None = None) -> None:
        tb = ""
        try:
            if exc is not None:
                tb = "".join(traceback.format_exception(
                    type(exc), exc, exc.__traceback__))[-4000:]
        except Exception:
            tb = ""
        self.log("error", message=str(message), traceback=tb)

    # ── XY command helper (clamp check + target tracking) ────────

    def note_xy_target(self, x_um: float, y_um: float) -> None:
        """Remember the most recent commanded XY target (absolute µm)
        so the sampler can compute physical lag."""
        self._target_um = (float(x_um), float(y_um))

    def xy_cmd_fields(self, controller, x_mm: float, y_mm: float) -> dict:
        """Compute absolute target + safety-clamp delta for a zero-ref mm
        XY command, and remember it as the lag target. Pure computation —
        does not move anything."""
        out: dict = {"x_mm": round(x_mm, 4), "y_mm": round(y_mm, 4)}
        try:
            zero = getattr(controller, "zero_position", {}) or {}
            ax = x_mm * 1000.0 + float(zero.get("x", 0.0))
            ay = y_mm * 1000.0 + float(zero.get("y", 0.0))
            out["abs_x_um"] = round(ax, 1)
            out["abs_y_um"] = round(ay, 1)
            limits = getattr(controller, "safety_limits", None)
            if limits is not None and getattr(limits, "enabled", False) \
                    and hasattr(limits, "clamp_xy"):
                cx, cy = limits.clamp_xy(ax, ay)
                if abs(cx - ax) > 0.5 or abs(cy - ay) > 0.5:
                    out["clamped"] = True
                    out["clamp_dx_um"] = round(cx - ax, 1)
                    out["clamp_dy_um"] = round(cy - ay, 1)
                self.note_xy_target(cx, cy)
            else:
                self.note_xy_target(ax, ay)
        except Exception:
            pass
        return out

    # ── Background actual-position sampler ───────────────────────

    def _sample_loop(self) -> None:
        period = 1.0 / self.sample_hz
        ctrl = self._controller
        while not self._stop_evt.wait(period):
            if ctrl is None or not self.active:
                continue
            try:
                rec: dict = {}
                try:
                    pos = ctrl.get_xy_position(cached=True)
                    if pos and pos[0] is not None:
                        rec["x_um"] = round(float(pos[0]), 1)
                        rec["y_um"] = round(float(pos[1]), 1)
                        if self._target_um is not None:
                            dx = self._target_um[0] - float(pos[0])
                            dy = self._target_um[1] - float(pos[1])
                            rec["lag_um"] = round(math.hypot(dx, dy), 1)
                except Exception:
                    pass
                try:
                    if hasattr(ctrl, "get_zp_position_logical_tuple"):
                        zp = ctrl.get_zp_position_logical_tuple(cached=True)
                        if zp and zp[0] is not None:
                            rec["z"] = round(float(zp[0]), 4)
                            rec["p1"] = round(float(zp[1]), 4)
                            rec["p2"] = round(float(zp[2]), 4)
                            rec["p3"] = round(float(zp[3]), 4)
                except Exception:
                    pass
                if rec:
                    self.log("sample", **rec)
            except Exception:
                pass

    # ── Manifest builder ─────────────────────────────────────────

    @staticmethod
    def manifest_for_job(job, controller, mode: str = "discrete") -> dict:
        """Build the job_start manifest: settings, frames, limits, and a
        compact command plan (full XY targets so an offline analyzer can
        diff planned vs actual motion)."""
        m: dict = {"exec_mode": mode}
        try:
            m["job"] = {
                "name": getattr(job, "name", "?"),
                "description": getattr(job, "description", ""),
                "source_file": getattr(job, "source_file", ""),
                "num_commands": len(getattr(job, "commands", []) or []),
            }
        except Exception:
            pass

        # Settings dataclass → dict (includes per-pump dicts)
        try:
            st = getattr(job, "settings", None)
            if st is not None and hasattr(st, "__dataclass_fields__"):
                m["settings"] = {
                    f: _jsonable(getattr(st, f))
                    for f in st.__dataclass_fields__}
        except Exception:
            pass

        # Frames + limits snapshot
        try:
            m["zero_position"] = _jsonable(
                dict(getattr(controller, "zero_position", {}) or {}))
        except Exception:
            pass
        try:
            limits = getattr(controller, "safety_limits", None)
            if limits is not None:
                if hasattr(limits, "to_dict"):
                    m["safety_limits"] = _jsonable(limits.to_dict())
                else:
                    m["safety_limits"] = {
                        k: _jsonable(getattr(limits, k))
                        for k in ("enabled", "xy_min_x", "xy_max_x",
                                  "xy_min_y", "xy_max_y", "z_min", "z_max",
                                  "max_xy_speed", "max_z_feedrate",
                                  "max_pump_feedrate")
                        if hasattr(limits, k)}
        except Exception:
            pass
        try:
            m["connection"] = {
                "xy": bool(getattr(controller, "is_xy_connected", False)),
                "zp": bool(getattr(controller, "is_zp_connected", False)),
                "simulated": bool(getattr(controller, "simulate", False)),
            }
        except Exception:
            pass

        # Compact command plan
        try:
            plan = []
            cmds = getattr(job, "commands", []) or []
            for i, cmd in enumerate(cmds[:500]):
                entry: dict = {"i": i, "type": cmd.type.value}
                if cmd.label:
                    entry["label"] = cmd.label
                p = cmd.params or {}
                if "x" in p or "y" in p:
                    entry["x"] = p.get("x")
                    entry["y"] = p.get("y")
                if "z" in p:
                    entry["z"] = p.get("z")
                pts = p.get("points")
                if pts:
                    entry["n_points"] = len(pts)
                    entry.update(PrintExecutionLogger._path_stats(pts))
                    entry["pump"] = p.get("pump")
                    entry["flow_rate_uL_s"] = p.get("flow_rate_uL_s")
                plan.append(entry)
            if len(cmds) > 500:
                plan.append({"i": 500, "type": "…",
                             "label": f"{len(cmds) - 500} more commands"})
            m["command_plan"] = plan
        except Exception:
            pass
        return m

    @staticmethod
    def _path_stats(points) -> dict:
        """Bounding box + segment-length stats for a path point list."""
        try:
            xs = [float(p[0]) for p in points]
            ys = [float(p[1]) for p in points]
            segs = [math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                    for i in range(1, len(xs))]
            out = {
                "bbox_mm": [round(min(xs), 3), round(min(ys), 3),
                            round(max(xs), 3), round(max(ys), 3)],
                "first_pt": [round(xs[0], 3), round(ys[0], 3)],
                "last_pt": [round(xs[-1], 3), round(ys[-1], 3)],
            }
            if segs:
                out["path_len_mm"] = round(sum(segs), 3)
                out["seg_mm_min"] = round(min(segs), 4)
                out["seg_mm_max"] = round(max(segs), 4)
                out["seg_mm_mean"] = round(sum(segs) / len(segs), 4)
            return out
        except Exception:
            return {}
