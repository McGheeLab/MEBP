# calibrate_xy.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Optional
import time
import json
import numpy as np

from SIM_xyz import XYPlantParams  # reuse the dataclass


@dataclass
class CalibrateResult:
    delay_s: float
    tau_v_s: float


class CalibrateXY:
    """
    Interrogates the real XY controller to estimate delay and time constant.
    Requires a driver with:
      - move_stage_at_velocity(vx, vy)
      - get_current_position() -> (x, y, z)
    """

    def __init__(self, dt_ctrl: float = 0.05):
        self.dt_ctrl = float(dt_ctrl)

    @staticmethod
    def _finite_diff(t: np.ndarray, x: np.ndarray) -> np.ndarray:
        dt = np.diff(t)
        dt[dt == 0] = 1e-6
        v = np.diff(x) / dt
        return np.r_[v[0], v]

    def _estimate_delay_tau(self, t: np.ndarray, v: np.ndarray, u_level: float) -> Tuple[float, float]:
        # crude FOPDT fit: 5% crossing for delay, 63.2% rise for tau
        vf = np.median(v[-max(5, len(v)//10):])
        if abs(vf) < 1e-6:
            return 0.2, 0.3
        sign = 1 if u_level >= 0 else -1
        e = sign * v
        thr5 = 0.05 * abs(vf)
        thr63 = 0.632 * abs(vf)
        i5 = int(np.argmax(e > thr5))
        t5 = t[i5] if i5 > 0 else t[0]
        post = e[i5:]
        if post.size == 0:
            return 0.2, 0.3
        i63 = int(np.argmax(post > thr63))
        t63 = t[i5 + i63] if i63 > 0 else t5 + 0.3
        delay = max(0.0, t5 - t[0])
        tau = max(0.05, t63 - t5)
        return delay, tau

    def run(self, xy_driver, step_vel: float = 60.0, dwell_s: float = 4.0) -> CalibrateResult:
        # settle
        xy_driver.move_stage_at_velocity(0.0, 0.0)
        time.sleep(1.0)

        t0 = time.time()
        ts = []; xs = []

        # baseline
        while time.time() - t0 < 0.5:
            x, y, _ = xy_driver.get_current_position()
            ts.append(time.time() - t0); xs.append(x)
            time.sleep(self.dt_ctrl)

        # step
        xy_driver.move_stage_at_velocity(step_vel, 0.0)
        t_step = time.time()
        while time.time() - t_step < dwell_s:
            x, y, _ = xy_driver.get_current_position()
            ts.append(time.time() - t0); xs.append(x)
            time.sleep(self.dt_ctrl)

        # stop
        xy_driver.move_stage_at_velocity(0.0, 0.0)
        time.sleep(0.5)

        t = np.array(ts)
        x = np.array(xs)
        v = self._finite_diff(t, x)
        delay, tau = self._estimate_delay_tau(t, v, step_vel)
        return CalibrateResult(delay_s=delay, tau_v_s=tau)

    def to_xy_params(self,
                     base: Optional[XYPlantParams],
                     result: CalibrateResult) -> XYPlantParams:
        P = base or XYPlantParams()
        P.comm_delay_s = float(result.delay_s)
        P.tau_v = float(result.tau_v_s)
        return P

    @staticmethod
    def save_xy_params(params: XYPlantParams, path: str = "xy_plant.json") -> None:
        params.to_json(path)
