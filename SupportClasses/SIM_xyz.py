# SIM_xyz.py
from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Dict, Optional, Tuple, List
import math
import json
import threading
import numpy as np
import random


@dataclass
class XYPlantParams:
    # Controller behavior
    refresh_s: float = 1.0           # how often XY controller accepts new VS (≈1 Hz)
    comm_delay_s: float = 0.08       # serial/controller latency before VS takes effect
    refresh_jitter_s: float = 0.02   # small +/- jitter

    # Servo/mechanics (first-order velocity servo)
    tau_v: float = 0.35              # v' = (u - v)/tau_v
    vmax: float = 100000.0           # max |velocity| 100 mm/s
    amax: float = 100000.0             # max |acceleration|
    jmax: float = 8             # max |jerk| (0 disables jerk capping)

    # Friction & dead-zone
    v_dead: float = 0             # dead-zone around zero
    coulomb: float = 0            # “static friction” subtraction on command
    viscous: float = 0           # viscous drag coef (unitless)

    # Metrology
    pos_quant: float = 1.0           # position quantization (stage units)
    pos_noise_std: float = 0.0       # added noise on measurement

    @staticmethod
    def from_json(path: str) -> "XYPlantParams":
        with open(path, "r") as f:
            return XYPlantParams(**json.load(f))

    def to_json(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)


class SIM_XYZ:
    """
    One simulator for the entire printer rig:
      - XY velocity-servo plant with refresh/hold & non-idealities
      - Z/P pumps executed as concatenated relative G0 segments at a feedrate
    Designed to be driven by a control loop at dt_ctrl (no wall-clock sleeps).
    """

    def __init__(self, xy_params: XYPlantParams = XYPlantParams(), zp_update_hz: int = 200):
        self.xy = xy_params

        # --- XY state ---
        self._x = 0.0; self._y = 0.0
        self._vx = 0.0; self._vy = 0.0
        self._ax = 0.0; self._ay = 0.0
        self._ux_hold = 0.0; self._uy_hold = 0.0  # last “accepted” commanded velocity
        self._pending_xy_cmd: Optional[Tuple[float, float]] = None
        self._next_refresh_time = self.xy.comm_delay_s + self._rand_jitter()

        # --- Z/P/E state (printer) ---
        self.pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        self.vel = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        self._active_seg: Optional[Dict] = None  # {'vel':{axis:v}, 'until':t, 'target':{axis:pos}}
        self._zp_dt = 1.0 / float(zp_update_hz)

        # timebase (simulation time, not wall clock)
        self.t = 0.0

        self._lock = threading.Lock()

    # ---------- helpers ----------

    def _rand_jitter(self) -> float:
        return random.uniform(-self.xy.refresh_jitter_s, self.xy.refresh_jitter_s)

    @staticmethod
    def _hypot2(ax: Dict[str, float]) -> float:
        s = 0.0
        for v in ax.values():
            s += float(v) * float(v)
        return math.sqrt(s)

    # ---------- XY API (mirrors XYStageManager minimal surface) ----------

    def get_xy_position(self) -> Tuple[float, float, float]:
        with self._lock:
            # quantized/noisy measurement
            q = self.xy.pos_quant
            nx = round(self._x / q) * q + (np.random.randn()*self.xy.pos_noise_std if self.xy.pos_noise_std>0 else 0.0)
            ny = round(self._y / q) * q + (np.random.randn()*self.xy.pos_noise_std if self.xy.pos_noise_std>0 else 0.0)
            return nx, ny, 0.0

    def move_stage_at_velocity(self, vx: float, vy: float) -> None:
        # this does not immediately change plant velocity; it *queues* a requested VS
        with self._lock:
            self._pending_xy_cmd = (float(vx), float(vy))

    # ---------- Z/P API (mirrors ZPStageManager minimal surface) ----------

    def move_relative(self, axes: Dict[str, float], feedrate: Optional[float] = None) -> None:
        """
        Single concatenated G0 move: we plan a *constant velocity* segment that hits target exactly
        at duration = dist / (feed_mm_per_s). Only axes present in dict are moved.
        """
        axes = {k: float(v) for k, v in axes.items() if abs(float(v)) > 0.0}
        if not axes:
            return

        feed_mm_per_min = float(feedrate) if (feedrate is not None and feedrate > 0) else 200.0
        feed_mm_per_s = feed_mm_per_min / 60.0

        dist = self._hypot2(axes)
        dur = max(1e-6, dist / feed_mm_per_s)

        with self._lock:
            # close any existing segment at the current instant (snap)
            if self._active_seg is not None:
                # snap to target of the old segment before starting a new one
                for a, v in self._active_seg['target'].items():
                    self.pos[a] = v
                self._active_seg = None
                for a in self.vel:
                    self.vel[a] = 0.0

            # target (relative -> absolute)
            target = {a: self.pos.get(a, 0.0) + axes.get(a, 0.0) for a in set(self.pos) | set(axes)}
            vel = {a: (target[a] - self.pos.get(a, 0.0)) / dur for a in axes.keys()}

            self._active_seg = {'vel': vel, 'until': self.t + dur, 'target': target}

    def get_zp_position(self) -> Tuple[float, float, float, float]:
        with self._lock:
            return (self.pos['X'], self.pos['Y'], self.pos['Z'], self.pos['E'])

    # ---------- integrator ----------

    def _apply_refresh_gate(self):
        if self._pending_xy_cmd is None:
            return
        if self.t < self._next_refresh_time:
            return

        vx_cmd, vy_cmd = self._pending_xy_cmd
        mag = max(1.0, math.hypot(vx_cmd, vy_cmd) / self.xy.vmax)
        self._ux_hold = vx_cmd / mag
        self._uy_hold = vy_cmd / mag

        self._pending_xy_cmd = None
        self._next_refresh_time = self.t + self.xy.refresh_s + self._rand_jitter()

    def _friction(self, u: float, v: float) -> float:
        # dead-zone + Coulomb + viscous
        if abs(u) < self.xy.v_dead:
            u_eff = 0.0
        else:
            u_eff = u - math.copysign(self.xy.coulomb, u)
            if math.copysign(1.0, u_eff) != math.copysign(1.0, u):
                u_eff = 0.0
        u_eff -= self.xy.viscous * v
        return u_eff

    def step(self, dt: float) -> None:
        """
        Advance the simulator by dt seconds (no wall-clock sleep).
        Call this in a tight loop for “fast” simulation or at your control cadence for real-time-ish.
        """
        dt = float(dt)
        if dt <= 0:
            return

        with self._lock:
            # 1) XY refresh gate (may accept a queued command)
            self._apply_refresh_gate()

            # 2) XY plant (first-order v servo + jerk/accel caps)
            ux = self._friction(self._ux_hold, self._vx)
            uy = self._friction(self._uy_hold, self._vy)

            dvx = (ux - self._vx) / max(self.xy.tau_v, 1e-6)
            dvy = (uy - self._vy) / max(self.xy.tau_v, 1e-6)

            if self.xy.jmax and self.xy.jmax > 0:
                dax = np.clip(dvx - self._ax, -self.xy.jmax * dt, self.xy.jmax * dt)
                day = np.clip(dvy - self._ay, -self.xy.jmax * dt, self.xy.jmax * dt)
                self._ax += dax
                self._ay += day
                dvx = self._ax
                dvy = self._ay

            dvx = float(np.clip(dvx, -self.xy.amax * dt, self.xy.amax * dt))
            dvy = float(np.clip(dvy, -self.xy.amax * dt, self.xy.amax * dt))

            self._vx = float(np.clip(self._vx + dvx, -self.xy.vmax, self.xy.vmax))
            self._vy = float(np.clip(self._vy + dvy, -self.xy.vmax, self.xy.vmax))

            self._x += self._vx * dt
            self._y += self._vy * dt

            # 3) Z/P segment execution
            if self._active_seg is not None:
                seg = self._active_seg
                remaining = seg['until'] - self.t
                if remaining <= 0:
                    # snap to target, stop
                    for a, v in seg['target'].items():
                        self.pos[a] = v
                    for a in self.vel:
                        self.vel[a] = 0.0
                    self._active_seg = None
                else:
                    # march at constant vel
                    for a, v in seg['vel'].items():
                        self.pos[a] += v * dt
                        self.vel[a] = v

            self.t += dt

    # ---------- tiny adapters to look like your existing managers ----------

    class XYAdapter:
        def __init__(self, sim: "SIM_XYZ"):
            self.sim = sim
        def get_current_position(self):
            return self.sim.get_xy_position()
        def move_stage_at_velocity(self, vx: float, vy: float):
            self.sim.move_stage_at_velocity(vx, vy)

    class ZPAdapter:
        def __init__(self, sim: "SIM_XYZ"):
            self.sim = sim
        def move_relative(self, axes: Dict[str, float], feedrate: Optional[float] = None):
            self.sim.move_relative(axes, feedrate=feedrate)
        def get_current_position(self):
            return self.sim.get_zp_position()

    def as_xy_manager(self) -> "SIM_XYZ.XYAdapter":
        return SIM_XYZ.XYAdapter(self)

    def as_zp_manager(self) -> "SIM_XYZ.ZPAdapter":
        return SIM_XYZ.ZPAdapter(self)
