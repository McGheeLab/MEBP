import threading
import csv
import time
import uuid
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import matplotlib.pyplot as plt

from PySide6.QtCore import QPointF, QSizeF

class PrintFile:
    def __init__(self, name="", color="#FF0000", csv_file=None):
        self.uid = str(uuid.uuid4())
        self.name = name
        self.color = color
        self.csv_file = csv_file
        self.well_id = None          # e.g., "A1"
        self.offset = (0, 0, 0)      # (x, y, z) offset from well center/floor
        self.floor_offset = None
        self.waypoints = []          # List of dicts with keys: x, y, z, p1, p2, p3, t

        # UI properties
        self.bbox_offset = QPointF(0, 0)
        self.bbox_size = QSizeF(50, 50)

        # Print time (set when print starts)
        self.print_start_time = None

        # Loaded and interpolated data holders
        self.xy_times = None
        self.zp_times = None
        self.xy_x = None
        self.xy_y = None
        self.zp_z  = None

        # Load raw waypoints from CSV
        self.load_csv()

    def load_csv(self):
        self.waypoints = []
        if self.csv_file is None:
            return self.waypoints
        try:
            with open(self.csv_file, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if not row or row[0].strip().lower().startswith('x'):
                        continue
                    if len(row) == 7:
                        try:
                            x, y, z, p1, p2, p3, t = map(float, row)
                            self.waypoints.append({'x': x, 'y': y, 'z': z,
                                                   'p1': p1, 'p2': p2, 'p3': p3, 't': t})
                        except ValueError:
                            print(f"Skipping invalid row (could not parse floats): {row}")
                    else:
                        print(f"Skipping invalid row length: {row}")
        except FileNotFoundError:
            print(f"CSV file not found: {self.csv_file}")
        except Exception as e:
            print(f"Error loading CSV: {e}")
        return self.waypoints

    def prepare_trajectories(self, dt_xy, dt_zp):
        """
        Resample raw waypoints onto uniform time grids for XY and Z motion.
        After calling, attributes xy_times, xy_x, xy_y, z_times, z_z are available.
        """
        if not self.waypoints:
            raise RuntimeError("No waypoints available for interpolation.")

        times = [wp['t'] for wp in self.waypoints]
        t0, t1 = times[0], times[-1]
        self.xy_times = np.arange(t0, t1 + dt_xy, dt_xy)
        self.zp_times  = np.arange(t0, t1 + dt_zp, dt_zp)

        x_vals = [wp['x'] for wp in self.waypoints]
        y_vals = [wp['y'] for wp in self.waypoints]
        z_vals = [wp['z'] for wp in self.waypoints]

        # Linear interpolation onto uniform grids
        self.xy_x = np.interp(self.xy_times, times, x_vals)
        self.xy_y = np.interp(self.xy_times, times, y_vals)
        self.zp_z  = np.interp(self.zp_times,  times, z_vals)

    def set_offset_xy(self, offset):
        self.offset = (offset[0], offset[1], self.offset[2])
        
    def set_offset_z(self, offset):
        self.offset = (self.offset[0], self.offset[1], offset)
 
class Syringe:
    def __init__(self, axis):
        self.axis = axis
        self.diameter = None
        self.length = None
        self.area = None
        self.max_volume = None
        self.current_volume = None
        self.cell_type = None
        self.ink_volume = None
        self.color = "#FF0000"  # default color

    def set_syringe_properties(self, diameter, length):
        self.diameter = diameter
        self.length = length
        self.area = self.calculate_syringe_area(diameter)
        self.max_volume = self.calculate_max_volume(self.area, length)

    def calculate_displacement(self, volume) -> float | None:
        if self.area is None or volume <= 0:
            print("Error: Syringe diameter is not set or invalid.")
            return None
        return volume / self.area

    def calculate_syringe_area(self, diameter):
        return np.pi * (diameter / 2) ** 2

    def calculate_max_volume(self, area, length):
        return area * length

    def assign_cell_type(self, cell_type):
        self.cell_type = cell_type

    def dispense_ink(self, ink_volume_to_dispense):
        if self.ink_volume is None or self.ink_volume < ink_volume_to_dispense:
            print("Error: Not enough ink")
            return False
        self.ink_volume -= ink_volume_to_dispense
        return True

    def load_ink(self, ink_volume):
        if self.ink_volume is None:
            print("Error: Ink volume is not set.")
            return False
        if self.current_volume + ink_volume > self.max_volume:
            print("Error: Not enough volume in the syringe to load the ink")
            return False
        self.ink_volume += ink_volume
        return True

    def update_syringe_color(self, color):
        self.color = color

class InkWell:
    def __init__(self):
        self.well_location = ""
        self.cell_type = "DefaultCell"
        self.chilled = True
        self.volume = 0.0
        self.color = "#868eff"  # default color

    def __repr__(self):
        return (f"InkWell(location={self.well_location}, cell_type='{self.cell_type}', "
                f"chilled={self.chilled}, volume={self.volume}, color='{self.color}')")

    def set_cell_type(self, cell_type):
        self.cell_type = cell_type

    def set_volume(self, volume):
        self.volume = volume

    def set_chilled(self, chilled):
        self.chilled = chilled

    def set_location(self, location):
        self.well_location = location

    def set_color(self, color):
        self.color = color

class WellPlate:
    """
    Holds all information about the current well-plate layout and provides
    utility methods for computing well positions.
    """
    def __init__(self, properties=None):
        # default plate properties
        defaults = {
            "fastz": 0, "floorz": 0, "topz": 0, "ink_topz": 0, "ink_floorz": 0,
            "Well_A1_x": 0, "Well_A1_y": 0, "well_dx": 0, "well_dy": 0,
            "well_rows": 0, "well_cols": 0, "well_diameter": 0, "plate_mode": "Well Plate"
        }
        self.props = defaults if properties is None else {**defaults, **properties}
        self.ink_wells = {}  # e.g. {"A1": InkWell(...), ...}
        

    def get_well_position(self, well_id):
        """
        Given a well string like "B3", return the (x, y) target for that well.
        """
        row_letter = well_id[0].upper()
        col_num = int(well_id[1:])
        row = ord(row_letter) - ord("A")
        col = col_num - 1

        x0 = self.props["Well_A1_x"] + col * self.props["well_dx"]
        y0 = self.props["Well_A1_y"] + row * self.props["well_dy"]
        return x0, y0

    def update_property(self, key, value):
        self.props[key] = value

class Print:
    def __init__(self, app_controller, stage_handler, pf: PrintFile):
        self.app      = app_controller
        self.stage    = stage_handler
        self.processor = app_controller.processor
        self.pf       = pf

        # Update intervals
        self.xy_interval = stage_handler.XYUPDATE_INTERVAL
        self.zp_interval = stage_handler.ZUPDATE_INTERVAL

        # Control flags and bookkeeping
        self.stop_flag       = False
        self.pause_flag      = False
        self.time_offset     = 0.0
        self.last_pause_time = None

        # Recordings: lists of (t, x, y) and (t, z)
        self.ideal_xy  = []
        self.actual_xy = []
        self.ideal_zp   = []
        self.actual_zp  = []

        # Delegate trajectory interpolation to PrintFile
        self.pf.prepare_trajectories(dt_xy=self.xy_interval,dt_zp=self.zp_interval)

    def run(self):
        
        # move to initial position
        self.stage.move_abs_xy(0.0, 0.0)
        time.sleep(5)  # allow time for stage to settle
        
        self._reset_control_flags()
        start_wall = time.time()
        t0 = 0.0

        t_xy = threading.Thread(target=self._xy_loop, args=(start_wall, t0), name="XY-Loop")
        t_z  = threading.Thread(target=self._zp_loop, args=(start_wall, t0),  name="Z-Loop")
        t_xy.start()
        t_z.start()
        t_xy.join()
        t_z.join()

    def pause(self):
        self.pause_flag = True

    def resume(self):
        self.pause_flag = False

    def stop(self):
        self.stop_flag = True

    # ─── Internal helpers ───────────────────────────────────────────────────────
    def _velocityCalc(self, dx, dy, dt):
        """
        Calculate the velocity needed to reach the target (dx, dy) in time dt.
        Returns (vx, vy) as a tuple.
        """
        vx = dx / dt if dt > 0 else 0.0
        vy = dy / dt if dt > 0 else 0.0
        return vx, vy
    
    def _reset_control_flags(self):
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0
        self.last_pause_time = None

        self.ideal_xy.clear()
        self.actual_xy.clear()
        self.ideal_zp.clear()
        self.actual_zp.clear()

        # for PID
        self._err_sum_x = 0.0
        self._err_sum_y = 0.0
        self._last_err_x = 0.0
        self._last_err_y = 0.0

    def _xy_loop(self, start_wall, t0):
        """Drive XY at velocities, record ideal vs. actual."""
        for i in range(len(self.pf.xy_times)-1):
            if self.stop_flag:
                break

            # pause handling
            now = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = now
                    self.stage.jog_xy(0, 0)  # stop XY motion
                    print("Paused XY motion.")
                time.sleep(0.01)
                continue
            elif self.last_pause_time:
                self.time_offset += now - self.last_pause_time
                self.last_pause_time = None

            # timing
            t_curr, t_next = self.pf.xy_times[i], self.pf.xy_times[i+1]
            dt = t_next - t_curr

            # desired
            x_des, y_des = self.pf.xy_x[i], self.pf.xy_y[i]
            self.ideal_xy.append((t_curr, x_des, y_des))

            # actual
            x_act, y_act, _ = self.stage.get_XY_positions()
            self.actual_xy.append((t_curr, x_act, y_act))

            # calculate velocity
            vx, vy = self._velocityCalc(x_des - x_act, y_des - y_act, dt)
            self.stage.jog_xy(vx, vy)

            # wait until real‐time catch up
            target = start_wall + (t_curr - t0)
            sleep = target - time.time()
            if sleep>0:
                time.sleep(sleep)

        # final stop
        self.stage.jog_xy(0, 0)  # stop XY motion

    def _zp_loop(self, start_wall, t0):
        for i in range(len(self.pf.zp_times)-1):
            if self.stop_flag:
                break

            # pause handling
            now = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = now
                    self.stage.move_rel_z(0, 0)
                time.sleep(0.01)
                continue
            elif self.last_pause_time:
                self.time_offset += now - self.last_pause_time
                self.last_pause_time = None

            # timing & motion
            t_curr, t_next = self.pf.zp_times[i], self.pf.zp_times[i+1]
            dz = self.pf.zp_z[i+1] - self.pf.zp_z[i]
            dt = t_next - t_curr

            feed = abs(dz) / dt * 60.0  # mm/min
            self.stage.move_rel_z(dz, feed)
            self.ideal_zp.append((t_curr, self.pf.zp_z[i]))
            # no actual‐Z logging here, but you could query if desired

            # actual
            z_act, _ , _ , _ = self.stage.get_ZP_positions()
            self.actual_zp.append((t_curr, z_act))
            
            target = start_wall + (t_curr - t0)
            sleep = target - time.time()
            if sleep>0:
                time.sleep(sleep)

        # final stop
        self.stage.move_rel_z(0, 0)

class PrintManager:
    def __init__(self, app_controller):
        self.app = app_controller
        self.processor = app_controller.processor

        # keep a single WellPlate around
        self.wellplate = WellPlate()
        self.syringes = {p: Syringe(p) for p in ("p1", "p2", "p3")}
        
        # queued PrintFile objects
        self.well_queue = []
        self.prints = {}     # uid → PrintFile
        self.results = {}    # uid → {"ideal":…, "actual":…}

        # state for the currently active print
        self.active_uid = None
        self.print_status = "idle"
        
        
    def queue_a_printfile(self, well_id, pf_tocopy, **kwargs):
        pf = PrintFile(name=well_id, csv_file=None)
        pf.uid = pf_tocopy.uid or str(uuid.uuid4())
        pf.offset = pf_tocopy.offset
        pf.floor_offset = pf_tocopy.floor_offset
        pf.name = pf_tocopy.name
        pf.color = pf_tocopy.color
        pf.csv_file = pf_tocopy.csv_file
        pf.waypoints = pf_tocopy.waypoints

        self.well_queue.append(pf)
        self.prints[pf.uid] = pf
        print(f"Queued PrintFile '{pf.name}' (uid={pf.uid})")

        if self.active_uid is None:
            self.active_uid = pf.uid
            self.print_status = "waiting"

    def process_print_queue(self):
        while self.well_queue:
            pf = self.well_queue.pop(0)
            self.active_uid = pf.uid
            self.print_status = "started"
            print(f"Starting print for well '{pf.name}'")

            # delegate all motion & recording to Print
            printer = Print(self.app, self.app.stage_handler, pf)
            res = printer.run()
            self.results[pf.uid] = res

            print(f"Print '{pf.name}' complete.")
            self.print_status = "finished"
            time.sleep(0.5)

        self.print_status = "idle"
        print("All prints done.")
