#!/usr/bin/env python3
# PIDPrinter.py (rewrite)
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple, Optional
import threading
import time
import numpy as np
import pandas as pd

# Hardware drivers
from DeviceInterface import XYStageManager, ZPStageManager

# Simulator & calibration
from SIM_xyz import SIM_XYZ, XYPlantParams
from calibrate_xy import CalibrateXY


@dataclass
class AxisMap:
    """Map logical {z,p1,p2,p3} to printer axes {X,Y,Z,E}."""
    z: str = "Z"
    p1: str = "E"
    p2: str = "X"
    p3: str = "Y"


class StageController:
    def __init__(self,
                 csv_path: str,
                 simulate: bool = False,
                 fast_sim: bool = False,
                 axis_map: AxisMap = AxisMap(),
                 # cadences
                 dt_xy: float = 1.0,       # outer setpoint update for XY (≈ XY refresh)
                 dt_zp: float = 0.33,      # Z+P tick
                 dt_ctrl: float = 0.1,     # PID loop interval
                 # PID gains
                 Kp: float = 0.3, Ki: float = 0.01, Kd: float = 0.005,
                 # simulator plant (used only when simulate=True)
                 xy_params: XYPlantParams = XYPlantParams()):
        """
        csv_path: CSV with columns x,y,z,p1,p2,p3,t (or 'time')
        simulate: True uses SIM_XYZ for both controllers
        fast_sim: if True, run the simulation with no sleeps (very fast)
        """
        self.axis_map = axis_map
        self.simulate = simulate
        self.fast_sim = fast_sim

        self.df = self._load_trajectory(csv_path)

        # time steps
        self.dt_xy = float(dt_xy)
        self.dt_zp = float(dt_zp)
        self.dt_ctrl = float(dt_ctrl)

        # PID state
        self.Kp = float(Kp); self.Ki = float(Ki); self.Kd = float(Kd)
        self.err_sum_x = 0.0; self.err_sum_y = 0.0
        self.last_err_x = 0.0; self.last_err_y = 0.0

        # interpolants
        self.xy_times = None; self.xy_x = None; self.xy_y = None
        self.zp_times = None; self.z_arr = None; self.p1_arr = None; self.p2_arr = None; self.p3_arr = None

        # logs
        self.ideal_xy = []   # (t_rel, x_target, y_target)
        self.actual_xy = []  # (t_rel, x_actual, y_actual)

        # devices
        if simulate:
            self.sim = SIM_XYZ(xy_params=xy_params)
            self.xy_mgr = self.sim.as_xy_manager()
            self.z_mgr = self.sim.as_zp_manager()
        else:
            self.sim = None
            self.xy_mgr = XYStageManager(simulate=False)
            self.z_mgr = ZPStageManager(simulate=False)

    # ---------- trajectory ----------

    def _load_trajectory(self, csv_path: str) -> pd.DataFrame:
        df = pd.read_csv(csv_path)
        if 't' not in df.columns and 'time' in df.columns:
            df = df.rename(columns={'time': 't'})
        req = ['x','y','z','p1','p2','p3','t']
        for c in req:
            if c not in df.columns:
                df[c] = 0.0
        return df[req].sort_values('t').drop_duplicates(subset='t').reset_index(drop=True)

    def interpolate_trajectories(self):
        t0, t1 = float(self.df['t'].iloc[0]), float(self.df['t'].iloc[-1])

        self.xy_times = np.arange(t0, t1 + self.dt_xy/2, self.dt_xy)
        self.xy_x = np.interp(self.xy_times, self.df['t'], self.df['x'])
        self.xy_y = np.interp(self.xy_times, self.df['t'], self.df['y'])

        self.zp_times = np.arange(t0, t1 + self.dt_zp/2, self.dt_zp)
        self.z_arr  = np.interp(self.zp_times, self.df['t'], self.df['z'])
        self.p1_arr = np.interp(self.zp_times, self.df['t'], self.df['p1'])
        self.p2_arr = np.interp(self.zp_times, self.df['t'], self.df['p2'])
        self.p3_arr = np.interp(self.zp_times, self.df['t'], self.df['p3'])

    # ---------- pid ----------

    def _pid_velocity(self, err_x: float, err_y: float, dt: float) -> Tuple[float, float]:
        # P
        P_x, P_y = self.Kp*err_x, self.Kp*err_y
        # I
        self.err_sum_x += err_x * dt
        self.err_sum_y += err_y * dt
        I_x, I_y = self.Ki*self.err_sum_x, self.Ki*self.err_sum_y
        # D
        D_x = self.Kd*((err_x - self.last_err_x)/dt if dt>0 else 0.0)
        D_y = self.Kd*((err_y - self.last_err_y)/dt if dt>0 else 0.0)
        self.last_err_x, self.last_err_y = err_x, err_y
        return P_x + I_x + D_x, P_y + I_y + D_y

    # ---------- run modes ----------

    def run(self):
        if self.fast_sim and self.sim is not None:
            return self._run_fast_sim()
        return self._run_realtime()

    def _run_realtime(self):
        assert self.xy_times is not None and self.zp_times is not None
        start_wall = time.time()
        t0 = float(self.xy_times[0])

        x0_init, y0_init, _ = self.xy_mgr.get_current_position()

        def xy_loop():
            next_refresh_wall = time.time()  # mirrors slow HW “accept”
            last_cmd_v = (0.0, 0.0)

            for i in range(len(self.xy_times) - 1):
                t_set = float(self.xy_times[i+1])
                x_rel, y_rel = float(self.xy_x[i+1]), float(self.xy_y[i+1])
                x_target = x0_init + x_rel
                y_target = y0_init + y_rel
                target_wall = start_wall + (t_set - t0)

                while True:
                    now = time.time()
                    if now >= target_wall:
                        break

                    x_act, y_act, _ = self.xy_mgr.get_current_position()
                    err_x = x_target - x_act
                    err_y = y_target - y_act
                    vx, vy = self._pid_velocity(err_x, err_y, self.dt_ctrl)

                    # emulate controller refresh: only send at ~1 Hz
                    if now >= next_refresh_wall:
                        last_cmd_v = (vx, vy)
                        self.xy_mgr.move_stage_at_velocity(vx, vy)
                        next_refresh_wall = now + max(self.dt_xy, 1.0)  # keep ~1 s unless you change dt_xy
                    else:
                        # repeat last command; HW would be holding it
                        self.xy_mgr.move_stage_at_velocity(*last_cmd_v)

                    t_rel = now - start_wall
                    self.ideal_xy.append((t_rel, x_target, y_target))
                    self.actual_xy.append((t_rel, x_act, y_act))

                    time.sleep(self.dt_ctrl)

            self.xy_mgr.move_stage_at_velocity(0.0, 0.0)

        def zp_loop():
            amap = self.axis_map
            for i in range(len(self.zp_times) - 1):
                t_curr = float(self.zp_times[i])
                t_next = float(self.zp_times[i+1])
                dt = t_next - t_curr

                dz  = float(self.z_arr[i+1]  - self.z_arr[i])
                dp1 = float(self.p1_arr[i+1] - self.p1_arr[i])
                dp2 = float(self.p2_arr[i+1] - self.p2_arr[i])
                dp3 = float(self.p3_arr[i+1] - self.p3_arr[i])

                axes = {getattr(amap,'z'): dz,
                        getattr(amap,'p1'): dp1,
                        getattr(amap,'p2'): dp2,
                        getattr(amap,'p3'): dp3}
                axes = {k:v for k,v in axes.items() if abs(v) > 0.0}

                dist = float(np.sqrt(dz*dz + dp1*dp1 + dp2*dp2 + dp3*dp3))
                feed = (dist / max(dt, 1e-6)) * 60.0

                if axes:
                    self.z_mgr.move_relative(axes, feedrate=feed)

                target = start_wall + (t_curr + dt - t0)
                wait = target - time.time()
                if wait > 0:
                    time.sleep(wait)

        t_xy = threading.Thread(target=xy_loop, name="XY-PID")
        t_zp = threading.Thread(target=zp_loop, name="ZP-Motion")
        t_xy.start(); t_zp.start()
        t_xy.join();  t_zp.join()
        print("Run complete (hardware mode).")
        return True

    def _run_fast_sim(self):
        """
        Single fast loop that steps the simulator without sleeps.
        """
        assert self.sim is not None, "fast_sim requires simulate=True"
        assert self.xy_times is not None and self.zp_times is not None

        t0 = float(self.df['t'].iloc[0])
        t1 = float(self.df['t'].iloc[-1])
        sim_t = 0.0

        # Build control timeline
        ctrl_t = np.arange(t0, t1 + self.dt_ctrl/2, self.dt_ctrl)
        x_sp = np.interp(ctrl_t, self.df['t'], self.df['x'])
        y_sp = np.interp(ctrl_t, self.df['t'], self.df['y'])

        # Build Z/P deltas at zp ticks
        zp_idx = 0

        # Anchor at current XY
        x0_init, y0_init, _ = self.xy_mgr.get_current_position()

        k_ctrl = 0
        while k_ctrl < len(ctrl_t)-1:
            # 1) XY PID @ dt_ctrl
            x_act, y_act, _ = self.xy_mgr.get_current_position()
            x_target = x0_init + x_sp[k_ctrl+1]
            y_target = y0_init + y_sp[k_ctrl+1]

            err_x = x_target - x_act
            err_y = y_target - y_act
            vx, vy = self._pid_velocity(err_x, err_y, self.dt_ctrl)
            self.xy_mgr.move_stage_at_velocity(vx, vy)

            # 2) Z/P tick(s) that fall within this ctrl step
            while zp_idx < len(self.zp_times)-1 and self.zp_times[zp_idx+1] <= ctrl_t[k_ctrl+1]:
                dz  = float(self.z_arr[zp_idx+1]  - self.z_arr[zp_idx])
                dp1 = float(self.p1_arr[zp_idx+1] - self.p1_arr[zp_idx])
                dp2 = float(self.p2_arr[zp_idx+1] - self.p2_arr[zp_idx])
                dp3 = float(self.p3_arr[zp_idx+1] - self.p3_arr[zp_idx])
                amap = self.axis_map
                axes = {getattr(amap,'z'): dz,
                        getattr(amap,'p1'): dp1,
                        getattr(amap,'p2'): dp2,
                        getattr(amap,'p3'): dp3}
                axes = {k:v for k,v in axes.items() if abs(v) > 0.0}
                dt_tick = float(self.zp_times[zp_idx+1] - self.zp_times[zp_idx])
                dist = float(np.sqrt(dz*dz + dp1*dp1 + dp2*dp2 + dp3*dp3))
                feed = (dist / max(dt_tick, 1e-6)) * 60.0
                if axes:
                    self.z_mgr.move_relative(axes, feedrate=feed)
                zp_idx += 1

            # 3) advance simulator time by dt_ctrl
            if self.sim is not None:
                self.sim.step(self.dt_ctrl)
                sim_t += self.dt_ctrl

            # 4) log
            self.ideal_xy.append((sim_t, x_target, y_target))
            self.actual_xy.append((sim_t, x_act, y_act))

            k_ctrl += 1

        # stop motion
        self.xy_mgr.move_stage_at_velocity(0.0, 0.0)
        print("Run complete (fast simulator).")
        return True


# -----------------------------
# Example usage
# -----------------------------
if __name__ == "__main__":
    # 1) build or load a trajectory (must have x,y,z,p1,p2,p3,t)
    #    (You can generate your own; here we synthesize a quick one)
    T = 120.0
    t = np.linspace(0, T, int(T/0.1)+1)
    r = 1000.0 * (t / T)
    th = 2*np.pi*2*(t/T)
    x = r*np.cos(th); y = r*np.sin(th)
    z = 2.0*np.sin(2*np.pi*t/T)
    p1 = 0.2*np.sin(2*np.pi*t/5.0)
    p2 = 0.1*np.cos(2*np.pi*t/7.0)
    p3 = 0.05*(t/T)

    pd.DataFrame(dict(x=x,y=y,z=z,p1=p1,p2=p2,p3=p3,t=t)).to_csv("traj.csv", index=False)

    # 2) optional: calibrate on hardware (commented by default)
    # cal = CalibrateXY(dt_ctrl=0.05)
    # xy_hw = XYStageManager(simulate=False)
    # res = cal.run(xy_hw, step_vel=60.0, dwell_s=4.0)
    # print("Calibration:", res)
    # params = cal.to_xy_params(None, res)
    # CalibrateXY.save_xy_params(params, "xy_plant.json")

    # 3) load params (if you saved them) or use defaults
    try:
        params = XYPlantParams.from_json("xy_plant.json")
    except Exception:
        params = XYPlantParams()

    # 4) run in fast simulator mode
    ctrl = StageController(csv_path="traj.csv",
                           simulate=True,
                           fast_sim=True,
                           axis_map=AxisMap(z="Z", p1="E", p2="X", p3="Y"),
                           dt_xy=1.0, dt_zp=0.33, dt_ctrl=0.05,
                           Kp=0.8, Ki=0.01, Kd=0.005,
                           xy_params=params)
    ctrl.interpolate_trajectories()
    ctrl.run()

    # quick visual if desired
    try:
        import matplotlib.pyplot as plt
        ideal = np.array(ctrl.ideal_xy); actual = np.array(ctrl.actual_xy)
        plt.figure(figsize=(7,6))
        plt.plot(ideal[:,1], ideal[:,2], label="Ideal")
        plt.plot(actual[:,1], actual[:,2], "--", label="Simulated")
        plt.axis("equal"); plt.legend(); plt.grid(True); plt.title("XY Path")
        plt.show()
    except Exception:
        pass
