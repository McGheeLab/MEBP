#!/usr/bin/env python3
import threading
import time
import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from DeviceInterface import XYStageManager, ZPStageManager 

class StageController:
    def __init__(self, csv_path, simulate=False):
        # Generate the spiral+circle CSV every time
        self.makespiral()
        
        # load raw trajectory
        self.df = pd.read_csv(csv_path)
        self.simulate = simulate

        # instantiate both managers
        self.xy_mgr = XYStageManager(simulate=simulate)
        self.z_mgr  = ZPStageManager(simulate=simulate)

        # placeholders for the interpolated trajectories
        self.xy_times = None
        self.xy_x     = None
        self.xy_y     = None
        self.z_times  = None
        self.z_z      = None

        # lists to record ideal vs. actual XY
        self.ideal_xy  = []  # each item: (t, x_des, y_des)
        self.actual_xy = []  # each item: (t, x_act, y_act)

    def makespiral(self):
        """Generate a 2D spiral in XY plus a sinusoidal Z, and save to CSV."""
        duration    = 120.0    # total time [s]
        dt          = 0.1      # time step [s]
        turns       = 3        # spiral revolutions
        max_radius  = 1000.0   # final spiral radius [um]
        z_diameter  = 10.0     # circle diameter in Z [mm]
        z_amp       = z_diameter / 2.0

        times = np.arange(0.0, duration + dt/2, dt)
        # spiral in XY
        r     = max_radius * (times / duration)
        theta = 2 * np.pi * turns * (times / duration)
        x     = r * np.cos(theta)
        y     = r * np.sin(theta)
        # circle in Z
        z     = z_amp * np.sin(2 * np.pi * times / duration)

        df = pd.DataFrame({
            'time': times,
            'x':    x,
            'y':    y,
            'z':    z
        })
        df.to_csv('spiral_circle_trajectory.csv', index=False)
        print(f"Wrote spiral_circle_trajectory.csv with {len(df)} points.")

    def interpolate_trajectories(self, dt_xy=1.0, dt_z=0.33):
        """Create uniform-time interpolated trajectories for XY and Z."""
        t0 = self.df['time'].iloc[0]
        t1 = self.df['time'].iloc[-1]

        self.xy_times = np.arange(t0, t1 + dt_xy, dt_xy)
        self.z_times  = np.arange(t0, t1 + dt_z,  dt_z)

        self.xy_x = np.interp(self.xy_times, self.df['time'], self.df['x'])
        self.xy_y = np.interp(self.xy_times, self.df['time'], self.df['y'])
        self.z_z  = np.interp(self.z_times,  self.df['time'], self.df['z'])

    def run(self):
        """Run the XY & Z routines in parallel, logging ideal vs. actual XY."""
        start_wall = time.time()
        t0 = self.xy_times[0]

        def xy_loop():
            for i in range(len(self.xy_times) - 1):
                t_curr, t_next = self.xy_times[i], self.xy_times[i+1]
                dt = t_next - t_curr

                # log the ideal point
                x_des, y_des = self.xy_x[i], self.xy_y[i]
                self.ideal_xy.append((t_curr, x_des, y_des))

                # compute and send velocity
                vx = (self.xy_x[i+1] - x_des) / dt
                vy = (self.xy_y[i+1] - y_des) / dt
                self.xy_mgr.move_stage_at_velocity(vx, vy)

                # query & log actual position
                x_act,y_act,z = self.xy_mgr.get_current_position() 
                self.actual_xy.append((t_curr, x_act, y_act))

                print(f"Time: {t_curr - start_wall:.2f}s | "
                          f"Target: ({x_des:.2f}, {y_des:.2f}) | "
                            f"Actual: ({x_act:.2f}, {y_act:.2f}) | "
                            f"Velocity: ({vx:.2f}, {vy:.2f})")
                
                # wait until next step
                target = start_wall + (t_next - t0)
                sleep = target - time.time()
                if sleep > 0:
                    time.sleep(sleep)

            # stop at end
            self.xy_mgr.move_stage_at_velocity(0, 0)

        def z_loop():
            for i in range(len(self.z_times) - 1):
                t_curr, t_next = self.z_times[i], self.z_times[i+1]
                dt = t_next - t_curr
                dz = self.z_z[i+1] - self.z_z[i]

                feed_mm_per_min = abs(dz) / dt * 60.0
                self.z_mgr.move_relative({'X': dz}, feedrate=feed_mm_per_min)

                target = start_wall + (t_curr + dt - t0)
                sleep = target - time.time()
                if sleep > 0:
                    time.sleep(sleep)

        t_xy = threading.Thread(target=xy_loop, name="XY-Loop")
        t_z  = threading.Thread(target=z_loop,  name="Z-Loop")
        t_xy.start()
        t_z.start()
        t_xy.join()
        t_z.join()

    def plot_paths(self):
        """Plot ideal vs. actual (offset-corrected) XY paths."""
        ideal  = np.array(self.ideal_xy)   # shape (N, 3)
        actual = np.array(self.actual_xy)  # shape (N, 3)

        # compute initial offset and correct actual trace
        offset = actual[0, 1:] - ideal[0, 1:]
        actual[:, 1:] -= offset

        plt.figure(figsize=(8,6))
        plt.plot(ideal[:,1],  ideal[:,2],  label="Ideal Path")
        plt.plot(actual[:,1], actual[:,2], '--', label="Actual Path (corrected)")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Ideal vs Actual XY Path")
        plt.axis('equal')
        plt.legend()
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    ctrl = StageController(
        csv_path='spiral_circle_trajectory.csv',
        simulate=False,
    )
    ctrl.interpolate_trajectories(dt_xy=1.15, dt_z=0.33)
    ctrl.run()
    ctrl.plot_paths()
