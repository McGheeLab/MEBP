#!/usr/bin/env python3
import threading
import time

import numpy as np
import pandas as pd

from DeviceInterface import XYStageManager, ZPStageManager


class StageController:
    def __init__(self, csv_path, simulate=False,
                 dt_xy=0.5, dt_z=0.33, dt_ctrl=0.1,
                 Kp=0.01, Ki=0, Kd=0):
        """
        csv_path: path to raw trajectory CSV with columns [time,x,y,z]
        simulate: pass-through to your DeviceInterface managers
        dt_xy / dt_z: how often to update the open-loop setpoints
        dt_ctrl: PID control loop polling interval (seconds)
        Kp, Ki, Kd: PID gains for XY velocity control
        """
        
        self.makespiral()  
        
        # load the raw trajectory
        self.df = pd.read_csv(csv_path)
        self.simulate = simulate

        # managers
        self.xy_mgr = XYStageManager(simulate=simulate)
        self.z_mgr  = ZPStageManager(simulate=simulate)

        # interpolation time steps
        self.dt_xy   = dt_xy
        self.dt_z    = dt_z
        self.dt_ctrl = dt_ctrl

        # PID state & gains
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.err_sum_x = 0.0
        self.err_sum_y = 0.0
        self.last_err_x = 0.0
        self.last_err_y = 0.0

        # placeholders for interpolated trajectories
        self.xy_times = None
        self.xy_x     = None
        self.xy_y     = None
        self.z_times  = None
        self.z_z      = None

        # logs of actual vs. ideal
        self.ideal_xy  = []   # list of (t_rel, x_target, y_target)
        self.actual_xy = []   # list of (t_rel, x_act, y_act)

    def makespiral(self):
                # Trajectory parameters
        duration    = 120.0       # total time [s]
        dt          = 0.1         # time step [s]
        turns       = 3           # number of spiral revolutions
        max_radius  = 1000.0        # final spiral radius [um]
        z_diameter  = 10.0         # circle diameter in Z [mm]
        z_amp       = z_diameter / 2.0

        # build time vector
        times = np.arange(0.0, duration + dt/2, dt)

        # spiral in XY: r grows linearly, theta runs through 'turns' revolutions
        r     = max_radius * (times / duration)
        theta = 2 * np.pi * turns * (times / duration)
        x     = r * np.cos(theta)
        y     = r * np.sin(theta)

        # circle in Z: simple sinusoid for a full period over 'duration'
        z     = z_amp * np.sin(2 * np.pi * times / duration)

        # assemble and save
        df = pd.DataFrame({
            'time': times,
            'x':    x,
            'y':    y,
            'z':    z
        })
        df.to_csv('spiral_circle_trajectory.csv', index=False)
        print("Wrote spiral_circle_trajectory.csv with", len(df), "points.")
    
    
    def interpolate_trajectories(self):
        """Build uniform time axes & simple linear interp for x,y,z."""
        t0, t1 = self.df['time'].iloc[0], self.df['time'].iloc[-1]
        self.xy_times = np.arange(t0, t1 + self.dt_xy, self.dt_xy)
        self.z_times  = np.arange(t0, t1 + self.dt_z,  self.dt_z)

        self.xy_x = np.interp(self.xy_times, self.df['time'], self.df['x'])
        self.xy_y = np.interp(self.xy_times, self.df['time'], self.df['y'])
        self.z_z  = np.interp(self.z_times,  self.df['time'], self.df['z'])

    def pid_velocity(self, err_x, err_y, dt):
        """Compute PID output velocities given current errors and dt."""
        # Proportional
        P_x, P_y = self.Kp * err_x, self.Kp * err_y

        # Integral
        self.err_sum_x += err_x * dt
        self.err_sum_y += err_y * dt
        I_x, I_y = self.Ki * self.err_sum_x, self.Ki * self.err_sum_y

        # Derivative
        D_x = self.Kd * ((err_x - self.last_err_x) / dt if dt > 0 else 0)
        D_y = self.Kd * ((err_y - self.last_err_y) / dt if dt > 0 else 0)

        # cache for next time
        self.last_err_x, self.last_err_y = err_x, err_y

        return P_x + I_x + D_x, P_y + I_y + D_y

    def run(self):
        """Fire off XY and Z loops in parallel and wait for both to finish."""
        # record wall-clock origin
        start_wall = time.time()
        t0 = self.xy_times[0]

        # get the initial actual stage origin for XY
        x0_init, y0_init,z = self.xy_mgr.get_current_position()

        # XY control loop
        def xy_loop():
            for i in range(len(self.xy_times) - 1):
                # next setpoint relative to CSV origin
                t_set = self.xy_times[i+1]
                x_rel, y_rel = self.xy_x[i+1], self.xy_y[i+1]
                # absolute target = initial + relative
                x_target = x0_init + x_rel
                y_target = y0_init + y_rel
                target_wall = start_wall + (t_set - t0)

                # run PID until it's time for next outer setpoint
                while True:
                    now = time.time()
                    if now >= target_wall:
                        break

                    # fetch actual position
                    x_act, y_act,z = self.xy_mgr.get_current_position()

                    # compute PID commanded velocity
                    err_x = x_target - x_act
                    err_y = y_target - y_act
                    vx, vy = self.pid_velocity(err_x, err_y, self.dt_ctrl)
                    
                    # send velocity to stage
                    self.xy_mgr.move_stage_at_velocity(vx, vy)
                    
                    # print current position and target and velocities
                    print(f"Time: {now - start_wall:.2f}s | "
                          f"Target: ({x_target:.2f}, {y_target:.2f}) | "
                            f"Actual: ({x_act:.2f}, {y_act:.2f}) | "
                            f"Velocity: ({vx:.2f}, {vy:.2f})")
                    
                    # log timestamps and positions
                    t_rel = now - start_wall
                    self.ideal_xy.append((t_rel, x_target, y_target))
                    self.actual_xy.append((t_rel, x_act, y_act))

                    time.sleep(self.dt_ctrl)

            # stop XY motion at end
            self.xy_mgr.move_stage_at_velocity(0.0, 0.0)

        # Z open-loop update (unchanged)
        def z_loop():
            for i in range(len(self.z_times) - 1):
                t_curr, t_next = self.z_times[i], self.z_times[i+1]
                dx = self.z_z[i+1] - self.z_z[i]
                dt = t_next - t_curr

                feed = abs(dx) / dt * 60.0
                self.z_mgr.move_relative({'X': dx}, feedrate=feed)

                target = start_wall + (t_curr + dt - t0)
                wait = target - time.time()
                if wait > 0:
                    time.sleep(wait)

        # launch both loops
        t_xy = threading.Thread(target=xy_loop, name="XY-PID-Loop")
        t_z  = threading.Thread(target=z_loop,  name="Z-Open-Loop")
        t_xy.start()
        t_z.start()
        t_xy.join()
        t_z.join()

        print("Run complete.")
        print(f"Logged {len(self.actual_xy)} PID control points.")

if __name__ == "__main__":

    ctrl = StageController(
        csv_path="spiral_circle_trajectory.csv",
        simulate=False,  # Set to True for simulation mode
        dt_xy=1.0,
        dt_z=0.33,
        dt_ctrl=0.1,
        Kp=0.3, Ki=0.01, Kd=0.005
    )
    ctrl.interpolate_trajectories()
    ctrl.run()

    # After run, plot ideal vs. actual accounting for initial offset
    import matplotlib.pyplot as plt
    ideal = np.array(ctrl.ideal_xy)
    actual = np.array(ctrl.actual_xy)

    plt.figure(figsize=(8,6))
    plt.plot(ideal[:,1], ideal[:,2], label="Ideal Path")
    plt.plot(actual[:,1], actual[:,2], '--', label="Actual Path")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Ideal vs Actual XY Path (offset accounted)")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()
