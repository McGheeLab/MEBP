#!/usr/bin/env python3
import threading
import time
from dataclasses import dataclass

import numpy as np
import pandas as pd

from DeviceInterface import XYStageManager, ZPStageManager

from dataclasses import dataclass, asdict
import json
import math
import numpy as np

@dataclass
class AxisMap:
    """
    Map logical channels to ZP controller axes.
    Must be a *bijection* across {'X','Y','Z','E'} for the ones you use.
    Unused letters are fine to leave unmapped.
    """
    z:  str = "Z"
    p1: str = "E"
    p2: str = "X"
    p3: str = "Y"

@dataclass
class XYPlantParams:
    # Controller behavior
    refresh_s: float = 1.0        # stage only accepts new VS command about every ~1 s
    comm_delay_s: float = 0.08    # serial + controller latency (tweak after calibration)
    refresh_jitter_s: float = 0.02  # small random jitter in refresh

    # Servo/Mechanics (velocity servo responding to commanded velocity)
    tau_v: float = 0.35           # first-order time-constant (s) vdot = (u_eff - v)/tau_v
    vmax: float = 100.0           # max velocity magnitude (stage units/s)
    amax: float = 1000.0          # max acceleration (units/s^2)
    jmax: float = 8000.0          # (optional) jerk cap (units/s^3); set 0 to disable

    # Friction/Dead-zone & viscous damping
    v_dead: float = 0.5           # static dead zone around zero (units/s)
    coulomb: float = 3.0          # equivalent “static friction” (units/s of cmd eaten)
    viscous: float = 0.05         # proportional drag on velocity (unitless)

    # Metrology
    pos_quant: float = 1.0        # encoder / report quantization step (stage units)
    pos_noise_std: float = 0.0    # additive noise on measured position (units)



class StageController:
    def __init__(
        self,
        csv_path,
        simulate=False,
        # Intervals
        dt_xy=1.0,          # XY outer setpoint update cadence (s)  ~ 1 s matches HW refresh
        dt_zp=0.33,         # Z+Pumps update cadence (s)
        dt_ctrl=0.1,        # XY PID inner loop poll (s)
        # PID
        Kp=0.3, Ki=0.01, Kd=0.005,
        # Mapping
        axis_map: AxisMap = AxisMap(),
        # XY stage physical limits for simulation/autotune
        xy_refresh_s=1.0,   # controller accepts new vel cmds ~1 Hz
        xy_vmax=100.0,      # stage max speed (XY manager units / s)
        xy_amax=1000.0,     # stage max accel (units / s^2)
    ):
        """
        csv_path : path to trajectory with columns x,y,z,p1,p2,p3,t (or ...,'time')
        simulate : pass-through to DeviceInterface managers (True uses simulators)
        """
        self.simulate = simulate
        self.df = self._load_trajectory(csv_path)  # ensures columns present

        # managers
        self.xy_mgr = XYStageManager(simulate=simulate)
        self.zp_mgr = ZPStageManager(simulate=simulate)

        # time steps
        self.dt_xy = float(dt_xy)
        self.dt_zp = float(dt_zp)
        self.dt_ctrl = float(dt_ctrl)

        # PID
        self.Kp, self.Ki, self.Kd = float(Kp), float(Ki), float(Kd)
        self.err_sum_x = 0.0
        self.err_sum_y = 0.0
        self.last_err_x = 0.0
        self.last_err_y = 0.0

        # mappings/limits
        self.axis_map = axis_map
        self.xy_refresh_s = float(xy_refresh_s)
        self.xy_vmax = float(xy_vmax)
        self.xy_amax = float(xy_amax)

        # interpolated trajectories (filled by interpolate_trajectories)
        self.xy_times = None
        self.xy_x = None
        self.xy_y = None

        self.zp_times = None
        self.z_arr = None
        self.p1_arr = None
        self.p2_arr = None
        self.p3_arr = None

        # logs for PID/plots
        self.ideal_xy = []   # (t_rel, x_target, y_target)
        self.actual_xy = []  # (t_rel, x_act, y_act)

    # ---------- I/O / Trajectory ----------

    def _load_trajectory(self, csv_path: str) -> pd.DataFrame:
        df = pd.read_csv(csv_path)
        # Be tolerant to either 't' or 'time'
        if 't' not in df.columns and 'time' in df.columns:
            df = df.rename(columns={'time': 't'})

        # Ensure all columns exist; fill missing with zeros
        required = ['x', 'y', 'z', 'p1', 'p2', 'p3', 't']
        for c in required:
            if c not in df.columns:
                df[c] = 0.0

        # Sort by time and drop dupes for safety
        df = df.sort_values('t').drop_duplicates(subset='t').reset_index(drop=True)
        return df[required]

    @staticmethod
    def makespiral(csv_out='spiral_circle_trajectory.csv'):
        """
        Example generator now emits x,y,z,p1,p2,p3,t
        Units are whatever your stages expect (xy are "stage units"; zp = mm/E steps)
        """
        duration = 40.0
        dt = 0.1
        turns = 3
        max_radius = 100000.0         # um (matches your XY defaults)
        z_diameter_mm = 10.0
        z_amp = z_diameter_mm / 2.0

        t = np.arange(0.0, duration + 1e-9, dt)

        r = max_radius * (t / duration)
        th = 2 * np.pi * turns * (t / duration)
        x = r * np.cos(th)
        y = r * np.sin(th)

        z = z_amp * np.sin(2 * np.pi * t / duration)   # mm

        # three example pump profiles (feel free to overwrite with your real ones)
        p1 = 0.5 * np.sin(2 * np.pi * t / 20.0)         # relative "E" units
        p2 = 0.25 * np.cos(2 * np.pi * t / 15.0)
        p3 = np.clip(t / duration, 0, 1) * 0.2          # slow ramp

        pd.DataFrame(dict(x=x, y=y, z=z, p1=p1, p2=p2, p3=p3, t=t)).to_csv(csv_out, index=False)
        print(f"Wrote {csv_out} with {len(t)} points.")

    def interpolate_trajectories(self):
        """Linear interpolation onto uniform grids for XY and Z+Pumps."""
        t0, t1 = float(self.df['t'].iloc[0]), float(self.df['t'].iloc[-1])

        self.xy_times = np.arange(t0, t1 + self.dt_xy/2, self.dt_xy)
        self.xy_x = np.interp(self.xy_times, self.df['t'], self.df['x'])
        self.xy_y = np.interp(self.xy_times, self.df['t'], self.df['y'])

        self.zp_times = np.arange(t0, t1 + self.dt_zp/2, self.dt_zp)
        self.z_arr  = np.interp(self.zp_times, self.df['t'], self.df['z'])
        self.p1_arr = np.interp(self.zp_times, self.df['t'], self.df['p1'])
        self.p2_arr = np.interp(self.zp_times, self.df['t'], self.df['p2'])
        self.p3_arr = np.interp(self.zp_times, self.df['t'], self.df['p3'])

    # ---------- PID ----------

    def pid_velocity(self, err_x, err_y, dt):
        # P
        P_x, P_y = self.Kp * err_x, self.Kp * err_y
        # I
        self.err_sum_x += err_x * dt
        self.err_sum_y += err_y * dt
        I_x, I_y = self.Ki * self.err_sum_x, self.Ki * self.err_sum_y
        # D
        D_x = self.Kd * ((err_x - self.last_err_x) / dt if dt > 0 else 0.0)
        D_y = self.Kd * ((err_y - self.last_err_y) / dt if dt > 0 else 0.0)
        self.last_err_x, self.last_err_y = err_x, err_y
        return P_x + I_x + D_x, P_y + I_y + D_y

    # ---------- Runtime (hardware threads) ----------

    def run(self):
        """Run XY (PID) and Z+Pumps (open-loop relative) in parallel."""
        assert self.xy_times is not None and self.zp_times is not None, "Call interpolate_trajectories() first."

        start_wall = time.time()
        t0 = float(self.xy_times[0])

        # anchoring XY absolute target to current position
        x0_init, y0_init, _ = self.xy_mgr.get_current_position()

        # XY thread: PID with hardware-like refresh (you can still poll PID faster than HW refresh)
        def xy_loop():
            next_refresh_wall = time.time()  # simulate ~1 Hz acceptance
            last_cmd_vx = 0.0
            last_cmd_vy = 0.0

            for i in range(len(self.xy_times) - 1):
                t_set = float(self.xy_times[i + 1])
                x_rel, y_rel = float(self.xy_x[i + 1]), float(self.xy_y[i + 1])
                x_target = x0_init + x_rel
                y_target = y0_init + y_rel
                target_wall = start_wall + (t_set - t0)

                while True:
                    now = time.time()
                    if now >= target_wall:
                        break

                    x_act, y_act, _ = self.xy_mgr.get_current_position()

                    # PID velocities
                    err_x = x_target - x_act
                    err_y = y_target - y_act
                    vx, vy = self.pid_velocity(err_x, err_y, self.dt_ctrl)

                    # Emulate controller “refresh”: only update VS every ~xy_refresh_s
                    if now >= next_refresh_wall:
                        # clamp by vmax
                        mag = max(1.0, np.hypot(vx, vy) / self.xy_vmax)
                        vx_cmd = vx / mag
                        vy_cmd = vy / mag
                        last_cmd_vx, last_cmd_vy = vx_cmd, vy_cmd
                        self.xy_mgr.move_stage_at_velocity(vx_cmd, vy_cmd)
                        next_refresh_wall = now + self.xy_refresh_s
                    else:
                        # keep last command (hardware would be holding)
                        self.xy_mgr.move_stage_at_velocity(last_cmd_vx, last_cmd_vy)

                    # log
                    t_rel = now - start_wall
                    self.ideal_xy.append((t_rel, x_target, y_target))
                    self.actual_xy.append((t_rel, x_act, y_act))

                    time.sleep(self.dt_ctrl)

            # stop at end
            self.xy_mgr.move_stage_at_velocity(0.0, 0.0)

        # Z+Pumps thread: single concatenated G0 per tick
        def zp_loop():
            amap = self.axis_map  # short alias
            for i in range(len(self.zp_times) - 1):
                t_curr = float(self.zp_times[i])
                t_next = float(self.zp_times[i + 1])
                dt = t_next - t_curr

                dz  = float(self.z_arr[i + 1]  - self.z_arr[i])
                dp1 = float(self.p1_arr[i + 1] - self.p1_arr[i])
                dp2 = float(self.p2_arr[i + 1] - self.p2_arr[i])
                dp3 = float(self.p3_arr[i + 1] - self.p3_arr[i])

                # Build one *combined* relative move
                axes = {
                    getattr(amap, 'z'):  dz,
                    getattr(amap, 'p1'): dp1,
                    getattr(amap, 'p2'): dp2,
                    getattr(amap, 'p3'): dp3,
                }
                # remove zeros to keep G-code clean
                axes = {k: v for k, v in axes.items() if abs(v) > 0.0}

                # Feedrate: Euclidean length in (Z,P1,P2,P3) space => mm/min (or your units/min)
                dist = np.sqrt(dz*dz + dp1*dp1 + dp2*dp2 + dp3*dp3)
                feed = (dist / max(dt, 1e-6)) * 60.0  # per-minute

                if axes:
                    self.zp_mgr.move_relative(axes, feedrate=feed)

                # align to timeline
                target = start_wall + (t_curr + dt - t0)
                wait = target - time.time()
                if wait > 0:
                    time.sleep(wait)

        t_xy = threading.Thread(target=xy_loop, name="XY-PID-Loop")
        t_zp = threading.Thread(target=zp_loop, name="ZP-Open-Loop")
        t_xy.start()
        t_zp.start()
        t_xy.join()
        t_zp.join()

        print("Run complete.")
        print(f"Logged {len(self.actual_xy)} PID control points.")

    # ---------- Ultra-fast simulation for tuning (no sleeps) ----------

    def simulate_xy_fast(self, Kp=None, Ki=None, Kd=None):
        """
        Ultra-fast discrete simulation of XY tracking with realistic non-idealities:
        - command refresh/hold with latency + jitter
        - first-order velocity servo (tau_v)
        - accel/vel (and optional jerk) limits
        - Coulomb + viscous friction and a dead-zone
        - measurement quantization/noise
        Uses self.df (x,y vs t) as the reference.
        Returns dict with RMS error and trace arrays.
        """
        P = self.xy_params
        Kp = self.Kp if Kp is None else float(Kp)
        Ki = self.Ki if Ki is None else float(Ki)
        Kd = self.Kd if Kd is None else float(Kd)

        # Control timeline: use the same dt_ctrl you use online (no sleeping here)
        t0 = float(self.df['t'].iloc[0])
        t1 = float(self.df['t'].iloc[-1])
        dt = float(self.dt_ctrl)
        t = np.arange(t0, t1 + 1e-9, dt)

        # Interpolate references
        x_ref = np.interp(t, self.df['t'], self.df['x'])
        y_ref = np.interp(t, self.df['t'], self.df['y'])

        # States
        x = 0.0; y = 0.0
        vx = 0.0; vy = 0.0
        ux_hold = 0.0; uy_hold = 0.0        # velocity command the plant is tracking (held between refreshes)
        ex_i = 0.0; ey_i = 0.0
        ex_prev = 0.0; ey_prev = 0.0
        ax = 0.0; ay = 0.0                   # for jerk limiting

        # Schedule next refresh accounting for comms delay & jitter
        next_refresh = t0 + P.comm_delay_s + np.random.uniform(-P.refresh_jitter_s, P.refresh_jitter_s)

        xs=[]; ys=[]; xms=[]; yms=[]; xrs=[]; yrs=[]

        def deadzone_and_friction(u, v):
            # subtract a Coulomb-like term and apply dead-zone; add viscous drag proportional to velocity
            if abs(u) < P.v_dead:
                u_eff = 0.0
            else:
                u_eff = u - math.copysign(P.coulomb, u)
                # If Coulomb subtraction overshoots the sign, clamp to zero
                if math.copysign(1.0, u_eff) != math.copysign(1.0, u):
                    u_eff = 0.0
            # viscous drag
            u_eff -= P.viscous * v
            return u_eff

        for k in range(len(t)-1):
            # Measured positions (quantized + noise) -> what PID "sees"
            xm = round(x / P.pos_quant) * P.pos_quant + (np.random.randn()*P.pos_noise_std if P.pos_noise_std>0 else 0.0)
            ym = round(y / P.pos_quant) * P.pos_quant + (np.random.randn()*P.pos_noise_std if P.pos_noise_std>0 else 0.0)

            # PID on measured error
            ex = x_ref[k] - xm
            ey = y_ref[k] - ym
            ex_i += ex * dt
            ey_i += ey * dt
            dx = (ex - ex_prev) / dt
            dy = (ey - ey_prev) / dt
            ex_prev, ey_prev = ex, ey
            ux_pid = Kp*ex + Ki*ex_i + Kd*dx
            uy_pid = Kp*ey + Ki*ey_i + Kd*dy

            # Refresh/hold of VS command (zero-order hold at ~1 Hz + latency)
            if t[k] >= next_refresh:
                # clamp magnitude to vmax
                scale = max(1.0, np.hypot(ux_pid, uy_pid) / P.vmax)
                ux_hold = ux_pid / scale
                uy_hold = uy_pid / scale
                next_refresh += P.refresh_s
                # add jitter to next one
                next_refresh += np.random.uniform(-P.refresh_jitter_s, P.refresh_jitter_s)

            # Plant: first-order v servo with accel, jerk limits
            uxe = deadzone_and_friction(ux_hold, vx)
            uye = deadzone_and_friction(uy_hold, vy)

            # ideal velocity derivative before rate limits
            dvx = (uxe - vx) / max(P.tau_v, 1e-6)
            dvy = (uye - vy) / max(P.tau_v, 1e-6)

            # jerk limiting (optional)
            if P.jmax and P.jmax > 0:
                dax = np.clip(dvx - ax, -P.jmax*dt, P.jmax*dt)
                day = np.clip(dvy - ay, -P.jmax*dt, P.jmax*dt)
                ax += dax
                ay += day
                dvx = ax
                dvy = ay

            # accel limits
            dvx = np.clip(dvx, -P.amax*dt, P.amax*dt)
            dvy = np.clip(dvy, -P.amax*dt, P.amax*dt)

            vx = np.clip(vx + dvx, -P.vmax, P.vmax)
            vy = np.clip(vy + dvy, -P.vmax, P.vmax)

            # integrate position
            x += vx * dt
            y += vy * dt

            xs.append(x); ys.append(y); xms.append(xm); yms.append(ym); xrs.append(x_ref[k]); yrs.append(y_ref[k])

        err = np.sqrt((np.array(xs)-np.array(xrs))**2 + (np.array(ys)-np.array(yrs))**2)
        rms = float(np.sqrt(np.mean(err**2)))
        return {
            "rms_error": rms,
            "trace": {"t": t[:-1], "x": np.array(xs), "y": np.array(ys),
                    "x_ref": np.array(xrs), "y_ref": np.array(yrs),
                    "x_meas": np.array(xms), "y_meas": np.array(yms)}
        }

    def autotune_pid(self, K_init=(0.3, 0.01, 0.005), span=(0.5, 0.1, 0.05),
                    iters=40, seed=0):
        """
        Random-restart local search around K_init with realism-aware cost.
        'span' is the +/- range around each K. Cost = RMS + penalties.
        """
        rng = np.random.default_rng(seed)
        best = None

        def cost_of(kp, ki, kd):
            sim = self.simulate_xy_fast(kp, ki, kd)
            tr = sim["trace"]
            # Base error
            rms = sim["rms_error"]

            # Penalties:
            # 1) oscillation: high var in error derivative
            ex = tr["x_ref"] - tr["x"]
            ey = tr["y_ref"] - tr["y"]
            e = np.sqrt(ex**2 + ey**2)
            de = np.diff(e) / max(self.dt_ctrl, 1e-6)
            osc = np.std(de)

            # 2) saturation pressure: how often we're at vmax
            vx = np.diff(tr["x"]) / max(self.dt_ctrl, 1e-6)
            vy = np.diff(tr["y"]) / max(self.dt_ctrl, 1e-6)
            speed = np.hypot(vx, vy)
            sat = np.mean(speed > 0.95*self.xy_params.vmax)

            # 3) integral wind-up proxy: large mean absolute integral term
            # (we don't log it directly; infer by big low-freq drift in error)
            wind = np.mean(np.abs(pd.Series(e).rolling(10, min_periods=1).mean()))

            J = rms + 0.15*osc + 0.5*sat + 0.05*wind
            return J, sim

        Kp0, Ki0, Kd0 = K_init
        for _ in range(iters):
            Kp = max(0.0, Kp0 + rng.uniform(-span[0], span[0]))
            Ki = max(0.0, Ki0 + rng.uniform(-span[1], span[1]))
            Kd = max(0.0, Kd0 + rng.uniform(-span[2], span[2]))
            J, sim = cost_of(Kp, Ki, Kd)
            if (best is None) or (J < best[0]):
                best = (J, (Kp, Ki, Kd), sim)

        (J_best, (Kp,Ki,Kd), sim_best) = best
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        print(f"[AUTOTUNE] Kp={Kp:.4f} Ki={Ki:.5f} Kd={Kd:.5f}  J={J_best:.4f}  RMS={sim_best['rms_error']:.4f}")
        return Kp, Ki, Kd, sim_best["rms_error"]


if __name__ == "__main__":
    # --- Generate a sample CSV with p1/p2/p3 (optional) ---
    StageController.makespiral("spiral_circle_trajectory.csv")

    # --- Configure mapping of z/p1/p2/p3 to printer axes ---
    # Example: Z->Z, P1->E, P2->X, P3->Y
    mapping = AxisMap(z="Z", p1="E", p2="X", p3="Y")

    ctrl = StageController(
        csv_path="spiral_circle_trajectory.csv",
        simulate=True,       # set False to use hardware
        dt_xy=1.0,
        dt_zp=0.33,
        dt_ctrl=0.1,
        Kp=0.3, Ki=0.01, Kd=0.005,
        axis_map=mapping,
        xy_refresh_s=1.0,
        xy_vmax=100.0,
        xy_amax=1000.0
    )

    ctrl.interpolate_trajectories()

    # --- Optional: PID Auto-tune using the 10,000× fast simulator ---
    bestKp, bestKi, bestKd, bestRMS = ctrl.autotune_pid(
        K_init=(ctrl.Kp, ctrl.Ki, ctrl.Kd),
        d_init=(0.2, 0.01, 0.01),
        max_iter=20,
        tol=1e-3
    )
    print(f"[AUTOTUNE] Kp={bestKp:.4f} Ki={bestKi:.5f} Kd={bestKd:.5f}  RMS={bestRMS:.3f}")

    # --- Real/Sim run with (possibly tuned) gains ---
    ctrl.run()

    # --- Quick plot of ideal vs actual (only when simulating XY) ---
    try:
        import matplotlib.pyplot as plt
        ideal = np.array(ctrl.ideal_xy)
        actual = np.array(ctrl.actual_xy)
        plt.figure(figsize=(8, 6))
        if len(ideal) > 0 and len(actual) > 0:
            plt.plot(ideal[:, 1], ideal[:, 2], label="Ideal Path")
            plt.plot(actual[:, 1], actual[:, 2], '--', label="Actual Path")
            plt.xlabel("X Position")
            plt.ylabel("Y Position")
            plt.title("Ideal vs Actual XY Path")
            plt.axis('equal')
            plt.legend()
            plt.grid(True)
            plt.show()
    except Exception as e:
        print("Plot skipped:", e)
