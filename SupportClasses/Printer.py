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
        self.offset = (0, 0, 0)        # (x, y, z) offset from well center/floor
        self.floor_offset = None
        self.waypoints = []          # List of dicts with keys: x, y, z, p1, p2, p3, t
        
        # Other UI properties:
        self.bbox_offset = QPointF(0, 0)
        self.bbox_size = QSizeF(50, 50)
        # Print time for this file (should be set once when print starts)
        self.print_start_time = None  
        # Load waypoints from CSV.
        self.load_csv()

    def load_csv(self):
        self.waypoints = []
        try:
            if self.csv_file is not None:
                with open(self.csv_file, mode='r') as file:
                    csv_reader = csv.reader(file)
                    for row in csv_reader:
                        if row[0].strip().lower().startswith('x'):
                            continue
                        elif len(row) == 7:
                            try:
                                x, y, z, p1, p2, p3, t = map(float, row)
                                waypoint = {'x': x, 'y': y, 'z': z, 'p1': p1, 'p2': p2, 'p3': p3, 't': t}
                                self.waypoints.append(waypoint)
                            except Exception as e:
                                print(f"Error parsing row {row}: {e}")
                        else:
                            print(f"Invalid row length: {row}")
        except FileNotFoundError:
            print(f"Error: File not found at {self.csv_file}")
        except Exception as e:
            print(f"Error reading CSV file: {e}")
        return self.waypoints

    def set_offset_xy(self, offset):
        self.offset = (offset[0], offset[1], self.offset[2])
        
    def set_offset_z(self, offset):
        self.offset = (self.offset[0], self.offset[1], offset)
 
    def interpolate_waypoints(self, elapsed_time, x0=0, y0=0, z0=0, p1=0, p2=0, p3=0, interpolation_type="linear"):
        if not self.waypoints:
            return None
        # If elapsed time exceeds the last waypoint’s t, return None
        if elapsed_time > self.waypoints[-1]['t']:
            return None
        times = [wp['t'] for wp in self.waypoints]
        data_keys = ['x', 'y', 'z', 'p1', 'p2', 'p3']
        interpolated_values = {}
        for key in data_keys:
            values = [wp[key] for wp in self.waypoints]
            if interpolation_type == "linear":
                interp_func = interp1d(times, values, kind='linear', fill_value="extrapolate")
            elif interpolation_type == "polynomial":
                degree = min(3, len(self.waypoints) - 1)
                interp_func = np.poly1d(np.polyfit(times, values, degree))
            elif interpolation_type == "spline":
                interp_func = CubicSpline(times, values)
            else:
                raise ValueError(f"Unsupported interpolation type: {interpolation_type}")
            interpolated_values[key] = interp_func(elapsed_time)
        interpolated_values['x'] += x0
        interpolated_values['y'] += y0
        interpolated_values['z'] += z0
        interpolated_values['p1'] += p1
        interpolated_values['p2'] += p2
        interpolated_values['p3'] += p3
        return interpolated_values

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

class PrintManager:
    def __init__(self, app_controller):
        self.app_controller = app_controller
        self.processor = app_controller.processor

        # PID parameters
        self.Kp, self.Ki, self.Kd = 0.5, 0.0, 0.0
        self.error_sum_x = self.error_sum_y = 0.0
        self.last_error_x = self.last_error_y = 0.0

        self.well_properties = {
            "fastz": 0, "floorz": 0, "topz": 0, "ink_topz": 0, "ink_floorz": 0,
            "Well_A1_x": 0, "Well_A1_y": 0, "well_dx": 0, "well_dy": 0,
            "well_rows": 0, "well_cols": 0, "well_diameter": 0, "plate_mode": "Well Plate"
        }
        self.well_properties["plate_mode"] = "Well Plate"
        self.ink_wells = {}  # Dictionary of InkWell objects
        self.syringes = {"p1": Syringe("p1"), "p2": Syringe("p2"), "p3": Syringe("p3")}
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0  
        self.last_pause_time = None


        # print queue state
        self.well_queue = []
        self.activeprint = None
        self.print_status = None
        self.prints = {}     # uid -> PrintFile
        self.results = {}    # uid -> {"ideal":{…}, "actual":{…}}

    def post__init__(self):
        if self.app_controller.stage_handler is None:
            print("StageHandler not running. Starting stage devices.")
            return False
        self.stage_handler = self.app_controller.stage_handler
        # intervals for updates
        self.xy_interval = self.stage_handler.XYUPDATE_INTERVAL
        self.zp_interval = self.stage_handler.ZUPDATE_INTERVAL

        return True
    
    def queue_a_printfile(self, well_id, pf_tocopy, **kwargs):
        pf = PrintFile(name=well_id, csv_file=None)
        pf.uid = pf_tocopy.uid or str(uuid.uuid4())
        pf.offset = pf_tocopy.offset
        pf.floor_offset = pf_tocopy.floor_offset
        pf.name = pf_tocopy.name
        pf.color = pf_tocopy.color
        pf.csv_file = pf_tocopy.csv_file
        pf.waypoints = pf_tocopy.waypoints
        pf.print_start_time = None

        self.well_queue.append(pf)
        self.prints[pf.uid] = pf
        print(f"Queued PrintFile '{pf.name}' (uid={pf.uid})")

        if self.activeprint is None:
            self.activeprint = pf
            self.print_status = "waiting"

    def process_print_queue(self, interpolation_type="linear"):
        while self.well_queue:
            pf = self.well_queue.pop(0)
            self.activeprint = pf
            self.print_status = "started"
            print(f"Starting print for well '{pf.name}'")
            self.start_print(f"Print: {pf.name}", interpolation_type, pf)
            time.sleep(0.5)
        self.print_status = "idle"
        print("All prints done.")

    def start_print(self, print_title, interpolation_type, pf):
        # reset control flags
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0
        self.last_pause_time = None

        # shared buffers
        self.ideal_path = {"x": [], "y": [], "z": []}
        self.actual_path = {"x": [], "y": [], "z": []}

        t0 = time.time()
        xy_thread = threading.Thread(
            target=self.xy_print_thread,
            args=(t0, interpolation_type, pf),
            daemon=True
        )
        zp_thread = threading.Thread(
            target=self.zp_print_thread,
            args=(t0, interpolation_type, pf),
            daemon=True
        )

        xy_thread.start()
        zp_thread.start()
        xy_thread.join()
        zp_thread.join()

        # stash the results for later plotting
        self.results[pf.uid] = {
            "ideal": {k: list(v) for k, v in self.ideal_path.items()},
            "actual": {k: list(v) for k, v in self.actual_path.items()}
        }

        print(f"{print_title} complete.")
        self.print_status = "finished"

    def xy_print_thread(self, t0, interpolation_type, pf):
        info = self.app_controller.get_stage_info().get("XY", {})
        x0 = info.get("x", {}).get("position", 0.0)
        y0 = info.get("y", {}).get("position", 0.0)
        next_t = t0

        while not self.stop_flag:
            now = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = now
                    self.processor.add_command("move_stage_at_velocity", average=(0, 0))
                time.sleep(0.05)
                continue
            elif self.last_pause_time:
                self.time_offset += now - self.last_pause_time
                self.last_pause_time = None

            if now >= next_t:
                elapsed = now - t0 - self.time_offset
                data = pf.interpolate_waypoints(
                    elapsed, x0=x0, y0=y0, z0=0, p1=0, p2=0, p3=0,
                    interpolation_type=interpolation_type
                )
                if data is None:
                    break

                tx, ty = data["x"], data["y"]
                cur = self.app_controller.get_stage_info()["XY"]
                cx = cur["x"]["position"]
                cy = cur["y"]["position"]

                print(f"XY target: ({tx}, {ty}), current: ({cx}, {cy})")
                
                # PID
                err_x, err_y = tx - cx, ty - cy
                print(f"XY error: ({err_x}, {err_y})")
                vx, vy = self.calculate_velocity_with_pid(err_x, err_y, self.xy_interval)
                print(f"XY velocity: ({vx}, {vy})")
                mag = np.hypot(vx, vy)
                                
                if mag > self.stage_handler.maxxyspeed:
                    print(f"Velocity exceeds max speed: {mag} > {self.stage_handler.maxxyspeed}.")
                    # pause print
                    self.pause_flag = True
                    
                self.stage_handler.update_xy_velocity(average=(vx, vy))

                # record
                self.ideal_path["x"].append(tx)
                self.ideal_path["y"].append(ty)
                self.actual_path["x"].append(cx)
                self.actual_path["y"].append(cy)

                next_t += self.xy_interval
            time.sleep(0.01)

        self.processor.add_command("move_stage_at_velocity", average=(0, 0))
        print("XY thread done.")

    def zp_print_thread(self, t0, interpolation_type, pf):
        z0 = self.stage_handler.get_stage_info().get("ZP", {}).get("Z", {}).get("position", 0.0)
        next_t = t0

        while not self.stop_flag:
            now = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = now
                    for ax in ("z", "p1", "p2", "p3"):
                        getattr(self.stage_handler, f"update_{ax}_velocity")(average=(0, 0))
                time.sleep(0.05)
                continue
            elif self.last_pause_time:
                self.time_offset += now - self.last_pause_time
                self.last_pause_time = None

            if now >= next_t:
                cur_z = self.stage_handler.get_stage_info()["ZP"]["Z"]["position"]
                self.actual_path["z"].append(cur_z)

                elapsed = now - t0 - self.time_offset
                data = pf.interpolate_waypoints(
                    elapsed, x0=0, y0=0, z0=z0,
                    p1=0, p2=0, p3=0,
                    interpolation_type=interpolation_type
                )
                if data is None:
                    break

                tz = data["z"]
                self.ideal_path["z"].append(tz)

                v_z = (tz - cur_z) / self.zp_interval
                self.stage_handler.update_z_velocity(average=(v_z, 0))
                next_t += self.zp_interval
            time.sleep(0.01)

        self.stage_handler.update_z_velocity(average=(0, 0))
        print("ZP thread done.")

    def calculate_velocity_with_pid(self, ex, ey, dt):
        Px, Py = self.Kp*ex, self.Kp*ey
        self.error_sum_x = ex*dt
        self.error_sum_y = ey*dt
        Ix, Iy = self.Ki*self.error_sum_x, self.Ki*self.error_sum_y
        Dx = self.Kd*((ex - self.last_error_x)/dt) if dt>0 else 0
        Dy = self.Kd*((ey - self.last_error_y)/dt) if dt>0 else 0
        self.last_error_x, self.last_error_y = ex, ey
        return Px + Ix + Dx, Py + Iy + Dy

    def slowmovez(self, zpos):
        self.stage_handler.move_abs_z_zero_reference(zpos)
        time.sleep(0.1)

    # stub for control from GUI
    def handle_control_print(self, cmd):
        if cmd == "pause":
            self.pause_flag = True
            self.print_status = "paused"
        elif cmd == "resume":
            self.pause_flag = False
            self.print_status = "started"
        elif cmd == "stop":
            self.stop_flag = True
            self.print_status = "stopped"

class PrintManager_2:
    def __init__(self, app_controller):
        self.app_controller = app_controller
        self.processor = app_controller.processor
        
        self.well_properties = {
            "fastz": 0, "floorz": 0, "topz": 0, "ink_topz": 0, "ink_floorz": 0,
            "Well_A1_x": 0, "Well_A1_y": 0, "well_dx": 0, "well_dy": 0,
            "well_rows": 0, "well_cols": 0, "well_diameter": 0, "plate_mode": "Well Plate"
        }
        self.well_properties["plate_mode"] = "Well Plate"
        self.ink_wells = {}  # Dictionary of InkWell objects
        self.syringes = {"p1": Syringe("p1"), "p2": Syringe("p2"), "p3": Syringe("p3")}
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0  
        self.last_pause_time = None
        # PID gains
        self.Kp, self.Ki, self.Kd = 0.5, 0.1, 0.0
        self.error_sum_x = self.error_sum_y = 0.0
        self.last_error_x = self.last_error_y = 0.0

        # print queue state
        self.well_queue = []
        self.activeprint = None
        self.print_status = None
        
    def post__init__(self):
        if self.app_controller.stage_handler is None:
            print("StageHandler not running. Starting stage devices.")
            return False
        self.stage_handler = self.app_controller.stage_handler
        # intervals for updates
        self.xy_interval = self.stage_handler.XYUPDATE_INTERVAL
        self.zp_interval = self.stage_handler.ZUPDATE_INTERVAL

        return True

    def queue_a_printfile(self, well_id, pf_tocopy, **kwargs):
        pf = PrintFile(name=well_id, csv_file=None)
        pf.well_id = well_id
        pf.offset = pf_tocopy.offset
        pf.floor_offset = pf_tocopy.floor_offset
        pf.name = pf_tocopy.name
        pf.color = pf_tocopy.color
        pf.csv_file = pf_tocopy.csv_file
        pf.waypoints = pf_tocopy.waypoints
        pf.print_start_time = None

        self.well_queue.append(pf)
        print(f"Queued PrintFile for well '{well_id}' with offset {pf.offset}.")

        if self.activeprint is None:
            self.activeprint = pf
            self.print_status = "waiting"

    def process_print_queue(self, interpolation_type="linear", plot=True):
        while self.well_queue:
            pf = self.well_queue.pop(0)
            print(f"Starting print for well '{pf.well_id}' with offset {pf.offset}.")
            self.start_print(
                print_title=f"Print for well {pf.well_id}",
                interpolation_type=interpolation_type,
                plot=plot,
                pf=pf
            )
            self.slowmovez(self.well_properties["topz"])
            time.sleep(0.5)
        print("Well queue processing complete.")

    def start_print(self, print_title, interpolation_type="linear", plot=False, pf=None):
        if pf is None:
            print("No print file provided.")
            return

        # reset flags & timers
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0
        self.last_pause_time = None

        # shared buffers on self
        self.ideal_path = {"x": [], "y": [], "z": []}
        self.actual_path = {"x": [], "y": [], "z": []}

        start_time = time.time()

        xy_thread = threading.Thread(
            target=self.xy_print_thread,
            args=(start_time, interpolation_type, pf)
        )
        zp_thread = threading.Thread(
            target=self.zp_print_thread,
            args=(start_time, interpolation_type, pf)
        )

        xy_thread.start()
        zp_thread.start()
        xy_thread.join()
        zp_thread.join()

        print(f"{print_title} complete.")

        if plot:
            self.plot_paths()

    def xy_print_thread(self, start_time, interpolation_type, pf):
        info = self.app_controller.get_stage_info().get("XY", {})
        x0 = info.get("x", {}).get("position", 0.0)
        y0 = info.get("y", {}).get("position", 0.0)

        next_t = start_time
        while not self.stop_flag:
            now = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = now
                    self.processor.add_command("move_stage_at_velocity", average=(0, 0))
                time.sleep(0.05)
                continue
            elif self.last_pause_time is not None:
                self.time_offset += now - self.last_pause_time
                self.last_pause_time = None

            if now >= next_t:
                elapsed = now - start_time - self.time_offset
                data = pf.interpolate_waypoints(
                    elapsed, x0=x0, y0=y0, z0=0, p1=0, p2=0, p3=0,
                    interpolation_type=interpolation_type
                )
                if data is None:
                    break

                tx, ty = data["x"], data["y"]
                cur = self.app_controller.get_stage_info()["XY"]
                cx = cur["x"]["position"]
                cy = cur["y"]["position"]

                # PID velocity
                vx, vy = self.calculate_velocity_with_pid(
                    tx - cx, ty - cy, self.xy_interval
                )
                mag = np.hypot(vx, vy)
                if mag > self.stage_handler.maxxyspeed:
                    scale = self.stage_handler.maxxyspeed / mag
                    vx, vy = vx*scale, vy*scale
                    self.error_sum_x = self.error_sum_y = 0.0

                self.stage_handler.update_xy_velocity(average=(vx, vy))

                # record
                self.ideal_path["x"].append(tx)
                self.ideal_path["y"].append(ty)
                self.actual_path["x"].append(cx)
                self.actual_path["y"].append(cy)

                next_t += self.xy_interval

            time.sleep(0.01)

        self.processor.add_command("move_stage_at_velocity", average=(0, 0))
        print("XY thread done.")

    def zp_print_thread(self, start_time, interpolation_type, pf):
        zp_info = self.stage_handler.get_stage_info().get("ZP", {}).get("Z", {})
        z0 = zp_info.get("position", 0.0)

        next_t = start_time
        while not self.stop_flag:
            now = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = now
                    self.stage_handler.update_z_velocity(average=(0, 0))
                    self.stage_handler.update_p1_velocity(average=(0, 0))
                    self.stage_handler.update_p2_velocity(average=(0, 0))
                    self.stage_handler.update_p3_velocity(average=(0, 0))
                time.sleep(0.05)
                continue
            elif self.last_pause_time is not None:
                self.time_offset += now - self.last_pause_time
                self.last_pause_time = None

            if now >= next_t:
                cur_z = self.stage_handler.get_stage_info()["ZP"]["Z"]["position"]
                self.actual_path["z"].append(cur_z)

                elapsed = now - start_time - self.time_offset
                data = pf.interpolate_waypoints(
                    elapsed, x0=0, y0=0, z0=z0,
                    p1=0, p2=0, p3=0,
                    interpolation_type=interpolation_type
                )
                if data is None:
                    break

                tz = data["z"]
                self.ideal_path["z"].append(tz)

                # simple linear velocity for Z
                v_z = (tz - cur_z) / self.zp_interval
                self.stage_handler.update_z_velocity(average=(v_z, 0))

                next_t += self.zp_interval

            time.sleep(0.01)

        # stop Z motion
        self.stage_handler.update_z_velocity(average=(0, 0))
        print("ZP thread done.")

    def calculate_velocity_with_pid(self, err_x, err_y, dt):
        P_x, P_y = self.Kp*err_x, self.Kp*err_y
        self.error_sum_x += err_x * dt
        self.error_sum_y += err_y * dt
        I_x, I_y = self.Ki*self.error_sum_x, self.Ki*self.error_sum_y
        D_x = self.Kd*((err_x - self.last_error_x)/dt) if dt>0 else 0
        D_y = self.Kd*((err_y - self.last_error_y)/dt) if dt>0 else 0
        self.last_error_x, self.last_error_y = err_x, err_y
        return P_x + I_x + D_x, P_y + I_y + D_y

    def plot_paths(self):
        # XY path
        fig, ax = plt.subplots()
        ax.plot(self.ideal_path["x"], self.ideal_path["y"],   label="Ideal XY")
        ax.plot(self.actual_path["x"], self.actual_path["y"], linestyle="--", label="Actual XY")
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_title("XY Stage: Ideal vs. Actual")
        ax.legend()
        plt.show()

        # Z path
        steps = range(len(self.ideal_path["z"]))
        fig, ax = plt.subplots()
        ax.plot(steps, self.ideal_path["z"],    label="Ideal Z")
        ax.plot(steps, self.actual_path["z"],   linestyle="--", label="Actual Z")
        ax.set_xlabel("Step")
        ax.set_ylabel("Z position")
        ax.set_title("Z Stage: Ideal vs. Actual")
        ax.legend()
        plt.show()

    def slowmovez(self, zpos):
        """Example helper to move Z slowly—replace with your real impl."""
        self.stage_handler.move_z_absolute(zpos)
        time.sleep(0.1)

class PrintManager_old:
    def __init__(self, app_controller):
        self.app_controller = app_controller
        self.processor = app_controller.processor
        if app_controller.stage_handler is None:
            print("StageHandler not running. Starting stage devices.")
            self.app_controller.start_stage_devices()
        self.stage_handler = self.app_controller.stage_handler

        self.xy_interval = self.stage_handler.XYUPDATE_INTERVAL
        self.zp_interval = self.stage_handler.ZUPDATE_INTERVAL

        # well_queue is managed internally, but the monitor does not pull from it.
        self.well_queue = []
        # The active print file is maintained by the PrintManager.
        self.activeprint = None
        # Print status can be "waiting", "started", "paused", "stopped", etc.
        self.print_status = "waiting"

        self.Kp = 0.5
        self.Ki = 0.1
        self.Kd = 0.0
        self.error_sum_x = 0.0
        self.error_sum_y = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0

        self.well_properties = {
            "fastz": 0, "floorz": 0, "topz": 0, "ink_topz": 0, "ink_floorz": 0,
            "Well_A1_x": 0, "Well_A1_y": 0, "well_dx": 0, "well_dy": 0,
            "well_rows": 0, "well_cols": 0, "well_diameter": 0, "plate_mode": "Well Plate"
        }

        self.ink_wells = {}  # Dictionary of InkWell objects
        self.syringes = {"p1": Syringe("p1"), "p2": Syringe("p2"), "p3": Syringe("p3")}
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0  
        self.last_pause_time = None

        self.processor.register_handler("queue_waypoint", self.queue_a_printfile)
        self.processor.register_handler("get_waypoints", self.handle_get_waypoints)
        self.processor.register_handler("control_print", self.handle_control_print)

    def queue_a_printfile(self, well_id, pf_tocopy, **kwargs):
        pf = PrintFile(name=well_id, csv_file=None)
        pf.well_id = well_id
        pf.offset = pf_tocopy.offset
        pf.floor_offset = pf_tocopy.floor_offset
        pf.name = pf_tocopy.name
        pf.color = pf_tocopy.color
        pf.csv_file = pf_tocopy.csv_file
        pf.waypoints = pf_tocopy.waypoints
        
        # Print time for this file (should be set once when print starts)
        pf.print_start_time = None 
        
        self.well_queue.append(pf)
        print(f"Queued PrintFile for well '{well_id}' with offset {pf.offset}.")
        # The PrintManager (not the UI) determines when to set activeprint.
        if self.activeprint is None:
            self.activeprint = pf
            self.print_status = "waiting"

    def handle_get_waypoints(self, **kwargs):
        print("Current well queue:")
        for pf in self.well_queue:
            print(f"PrintFile for well '{pf.well_id}', offset {pf.offset}, CSV: {pf.csv_file}")

    def handle_control_print(self, action, **kwargs):
        if action == "pause":
            self.pause_flag = True
            self.print_status = "paused"
            print("Print paused.")
        elif action == "resume":
            self.pause_flag = False
            if self.activeprint:
                # If starting, record the start time on the active print file.
                if self.activeprint.print_start_time is None:
                    self.activeprint.print_start_time = time.time()
                self.print_status = "started"
            print("Print resumed.")
        elif action == "stop":
            self.stop_flag = True
            self.print_status = "stopped"
            self.processor.add_command("move_stage_at_velocity", average=(0, 0))
            self.processor.add_command("move_z_at_velocity", average=(0, 0))
            self.processor.add_command("move_p1_at_velocity", average=(0, 0))
            self.processor.add_command("move_p2_at_velocity", average=(0, 0))
            self.processor.add_command("move_p3_at_velocity", average=(0, 0))
            print("Print stopped.")
        else:
            print(f"Unknown print control action: {action}")

    def calculate_velocity_with_pid(self, error_x, error_y, delta_time):
        P_x = self.Kp * error_x
        P_y = self.Kp * error_y
        self.error_sum_x += error_x * delta_time
        self.error_sum_y += error_y * delta_time
        I_x = self.Ki * self.error_sum_x
        I_y = self.Ki * self.error_sum_y
        D_x = self.Kd * ((error_x - self.last_error_x) / delta_time) if delta_time > 0 else 0
        D_y = self.Kd * ((error_y - self.last_error_y) / delta_time) if delta_time > 0 else 0
        vx = P_x + I_x + D_x
        vy = P_y + I_y + D_y
        self.last_error_x = error_x
        self.last_error_y = error_y
        return vx, vy

    def xy_print_thread(self, start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, pf):
        stage_info = self.app_controller.get_stage_info()
        initial_x = stage_info.get("XY", {}).get("x", {}).get("position", 0.0)
        initial_y = stage_info.get("XY", {}).get("y", {}).get("position", 0.0)
        if initial_x is None or initial_y is None:
            print("Failed to retrieve initial XY position. Stopping XY updates.")
            return

        next_update_time = start_time
        while not self.stop_flag:
            current_time = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = current_time
                    self.processor.add_command("move_stage_at_velocity", average=(0, 0))
                time.sleep(0.05)
                continue
            else:
                if self.last_pause_time is not None:
                    paused_duration = current_time - self.last_pause_time
                    self.time_offset += paused_duration
                    self.last_pause_time = None

            if current_time >= next_update_time:
                effective_elapsed = current_time - start_time - self.time_offset
                if pf is None:
                    print("No print file provided. Stopping XY print thread.")
                    break

                interpolated = pf.interpolate_waypoints(
                    effective_elapsed, x0=initial_x, y0=initial_y, z0=0, p1=0, p2=0, p3=0, interpolation_type=interpolation_type
                )
                if interpolated is None:
                    break

                target_x = interpolated['x']
                target_y = interpolated['y']

                stage_info = self.app_controller.get_stage_info()
                current_x = stage_info.get("XY", {}).get("x", {}).get("position", 0.0)
                current_y = stage_info.get("XY", {}).get("y", {}).get("position", 0.0)

                error_x = target_x - current_x
                error_y = target_y - current_y
                vx, vy = self.calculate_velocity_with_pid(error_x, error_y, self.xy_interval)
                if np.hypot(vx, vy) > self.stage_handler.maxxyspeed:
                    scaling = self.stage_handler.maxxyspeed / np.hypot(vx, vy)
                    vx *= scaling
                    vy *= scaling
                    self.error_sum_x = 0.0
                    self.error_sum_y = 0.0

                self.stage_handler.update_xy_velocity(average=(vx, vy))
                ideal_path_x.append(target_x)
                ideal_path_y.append(target_y)
                actual_path_x.append(current_x)
                actual_path_y.append(current_y)
                next_update_time += self.xy_interval
            time.sleep(0.01)

        self.processor.add_command("move_stage_at_velocity", average=(0, 0))
        print("XY print updates complete.")

    def zp_print_thread(self, start_time, interpolation_type,
                        ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                        actual_path_z, actual_p1, actual_path_p2, actual_path_p3, pf):
        stage_info = self.stage_handler.get_stage_info()
        initial_z = stage_info.get("ZP", {}).get("Z", {}).get("position", 0.0)
        initial_p1 = stage_info.get("ZP", {}).get("P1", {}).get("position", 0.0)
        initial_p2 = stage_info.get("ZP", {}).get("P2", {}).get("position", 0.0)
        initial_p3 = stage_info.get("ZP", {}).get("P3", {}).get("position", 0.0)
        if initial_z is None:
            initial_z = initial_p1 = initial_p2 = initial_p3 = 0
            print("Failed to get ZP initial position, defaulting to 0.")

        next_update_time = start_time
        while not self.stop_flag:
            current_time = time.time()
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = current_time
                    self.stage_handler.update_z_velocity(average=(0.0, 0.0))
                    self.stage_handler.update_p1_velocity(average=(0.0, 0.0))
                    self.stage_handler.update_p2_velocity(average=(0.0, 0.0))
                    self.stage_handler.update_p3_velocity(average=(0.0, 0.0))
                time.sleep(0.05)
                continue
            else:
                if self.last_pause_time is not None:
                    paused_duration = current_time - self.last_pause_time
                    self.time_offset += paused_duration
                    self.last_pause_time = None

            if current_time >= next_update_time:
                stage_info = self.app_controller.get_stage_info()
                current_z = stage_info.get("ZP", {}).get("Z", {}).get("position", 0.0)
                current_p1 = stage_info.get("ZP", {}).get("P1", {}).get("position", 0.0)
                current_p2 = stage_info.get("ZP", {}).get("P2", {}).get("position", 0.0)
                current_p3 = stage_info.get("ZP", {}).get("P3", {}).get("position", 0.0)

                actual_path_z.append(current_z)
                actual_p1.append(current_p1)
                actual_path_p2.append(current_p2)
                actual_path_p3.append(current_p3)

                effective_elapsed = current_time - start_time - self.time_offset
                if pf is None:
                    print("No print file provided. Stopping ZP print thread.")
                    break

                interpolated = pf.interpolate_waypoints(
                    effective_elapsed, x0=0, y0=0, z0=initial_z, p1=initial_p1, p2=initial_p2, p3=initial_p3, interpolation_type=interpolation_type
                )
                if interpolated is None:
                    break

                target_z = interpolated['z']
                target_p1 = interpolated['p1']
                target_p2 = interpolated['p2']
                target_p3 = interpolated['p3']

                dt = self.zp_interval
                v_z = float((target_z - current_z) / dt)
                v_p1 = float((target_p1 - current_p1) / dt)
                v_p2 = float((target_p2 - current_p2) / dt)
                v_p3 = float((target_p3 - current_p3) / dt)

                
                self.stage_handler.update_z_velocity(average=(v_z, 0.0))
                self.stage_handler.update_p1_velocity(average=(v_p1, 0.0))
                self.stage_handler.update_p2_velocity(average=(v_p2, 0.0))
                self.stage_handler.update_p3_velocity(average=(v_p3, 0.0))

                ideal_path_z.append(target_z)
                ideal_p1.append(target_p1)
                ideal_p2.append(target_p2)
                ideal_p3.append(target_p3)
                next_update_time += self.zp_interval
            time.sleep(0.01)

        self.stage_handler.update_z_velocity(average=(0.0, 0.0))
        self.stage_handler.update_p1_velocity(average=(0.0, 0.0))
        self.stage_handler.update_p2_velocity(average=(0.0, 0.0))
        self.stage_handler.update_p3_velocity(average=(0.0, 0.0))
        print("ZP print updates complete.")

    def start_print(self, print_title, interpolation_type="linear", plot=False, pf=None):
        if pf is None:
            print("No print file provided to start_print().")
            return

        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0
        self.last_pause_time = None

        actual_path_x, ideal_path_x = [], []
        actual_path_y, ideal_path_y = [], []
        actual_path_z, ideal_path_z = [], []
        actual_p1, ideal_p1 = [], []
        actual_path_p2, ideal_p2 = [], []
        actual_path_p3, ideal_p3 = [], []

        start_time = time.time()

        xy_thread = threading.Thread(
            target=self.xy_print_thread,
            args=(start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, pf)
        )
        zp_thread = threading.Thread(
            target=self.zp_print_thread,
            args=(start_time, interpolation_type, ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                  actual_path_z, actual_p1, actual_path_p2, actual_path_p3, pf)
        )

        xy_thread.start()
        zp_thread.start()
        xy_thread.join()
        zp_thread.join()

        print(f"{print_title} complete.")

    def process_print_queue(self, interpolation_type="linear", plot=True):
        while self.well_queue:
            pf = self.well_queue.pop(0)
            well_id = pf.well_id
            offset = pf.offset

            print(f"Starting print for well '{well_id}' with offset {offset}.")

            #self.prepare_print(pf)
            self.start_print(print_title=f"Print for well {well_id}", interpolation_type=interpolation_type, plot=plot, pf=pf)
            self.slowmovez(self.well_properties["topz"])
            time.sleep(0.5)
        print("Well queue processing complete.")

    def prepare_print(self, pf):
        # Fast move to z for fast XY moves.
        print(f"Fast moving to Z: {self.well_properties['fastz']}")
        self.fastmovez(self.well_properties["fastz"])
        
        # Load inks based on the print file’s waypoints.
        self.load_ink(pf)
        print(f"Loaded ink for well '{pf.well_id}' with offset {pf.offset}.")
        # Fast move to z for fast XY moves.
        self.fastmovez(self.well_properties["fastz"])
        print(f"Fast moving to Z: {self.well_properties['fastz']}")
        # Move to the well (with XY offset).
        well_xy = self.find_well_xy(pf.well_id)
        self.fastmovexy(well_xy[0] + pf.offset[0], well_xy[1] + pf.offset[1])
        print(f"Fast moving to XY: {well_xy[0] + pf.offset[0]}, {well_xy[1] + pf.offset[1]}")
        # Move to well top and then to floor (with Z offset).
        self.fastmovez(self.well_properties["ink_topz"])
        print(f"Fast moving to Z: {self.well_properties['ink_topz']}")
        self.slowmovez(self.well_properties["floorz"] - pf.offset[2])
        print(f"Slow moving to Z: {self.well_properties['floorz'] - pf.offset[2]}")

    # Helper functions for polling motion completion
    def wait_for_z(self, target_z, tolerance=0.05):
        while True:
            current_z = self.app_controller.stage_handler.get_stage_info()["ZP"]["Z"]["position"]
            if abs(current_z - target_z) <= tolerance:
                break
            time.sleep(0.01)

    def wait_for_xy(self, target_x, target_y,speed, tolerance=0.05):
        while True:
            stage_info = self.app_controller.stage_handler.get_stage_info()["XY"]
            current_x = stage_info["x"]["position"]
            current_y = stage_info["y"]["position"]
            dx = target_x - current_x
            dy = target_y - current_y
            distance = np.hypot(dx, dy)
            if distance <= tolerance:
                break
            # Calculate normalized direction and multiply by a chosen speed factor.
            vx = dx / distance
            vy = dy / distance
            self.processor.add_command("move_stage_at_velocity", average=(vx * speed, vy * speed))
            time.sleep(0.01)

    def wait_for_p(self, p_key, target, tolerance=0.05):
        while True:
            current = self.app_controller.stage_handler.get_stage_info()["ZP"][p_key]["position"]
            if abs(current - target) <= tolerance:
                break
            time.sleep(0.01)

    def fastmovez(self, z):
        self.app_controller.stage_handler.move_abs_z_zero_reference(z, True)
        self.wait_for_z(z)

    def fastmovexy(self, x, y):
        speed = 10000
        self.app_controller.stage_handler.move_abs_xy_well_reference(x, y, True)
        self.wait_for_xy(x, y,speed)

    def slowmovez(self, z):
        self.app_controller.stage_handler.move_abs_z_zero_reference(z, False)
        self.wait_for_z(z)

    def movep(self, p, amount):
        if amount is None or amount <= 0:
            print("Invalid amount for movep.")
        else:
            if p == "p1":
                distance = self.syringes[p].calculate_displacement(amount)
                current = self.app_controller.stage_handler.get_stage_info()["ZP"]["P1"]["position"]
                target = current + distance
                self.stage_handler.move_rel_p('P1', distance)
                self.wait_for_p('P1', target)
            elif p == "p2":
                distance = self.syringes[p].calculate_displacement(amount)
                current = self.app_controller.stage_handler.get_stage_info()["ZP"]["P2"]["position"]
                target = current + distance
                self.stage_handler.move_rel_p('P2', distance)
                self.wait_for_p('P2', target)
            elif p == "p3":
                distance = self.syringes[p].calculate_displacement(amount)
                current = self.app_controller.stage_handler.get_stage_info()["ZP"]["P3"]["position"]
                target = current + distance
                self.stage_handler.move_rel_p('P3', distance)
                self.wait_for_p('P3', target)

    def load_ink(self, pf):
        """
        Updated load_ink uses the new InkWell objects.
        It looks up the InkWell (based on its cell_type) corresponding to each syringe.
        Then it uses the InkWell’s well_location to determine XY coordinates.
        """
        p1, p2, p3 = self.how_much_ink(pf)
        # Find the InkWell object for each syringe by matching cell_type.
        p1_inkwell = next((inkwell for inkwell in self.ink_wells.values()
                           if inkwell.cell_type == self.syringes["p1"].cell_type), None)
        p2_inkwell = next((inkwell for inkwell in self.ink_wells.values()
                           if inkwell.cell_type == self.syringes["p2"].cell_type), None)
        p3_inkwell = next((inkwell for inkwell in self.ink_wells.values()
                           if inkwell.cell_type == self.syringes["p3"].cell_type), None)

        for inkwell, ink_amount, syringe_key in zip(
                [p1_inkwell, p2_inkwell, p3_inkwell],
                [p1, p2, p3],
                ["p1", "p2", "p3"]):
            if inkwell is not None and inkwell.well_location:
                well_xy = self.find_well_xy(inkwell.well_location)
                if well_xy is None:
                    continue
                self.fastmovez(self.well_properties["fastz"])
                self.fastmovexy(well_xy[0], well_xy[1])
                self.fastmovez(self.well_properties["ink_topz"])
                self.slowmovez(self.well_properties["ink_floorz"])
                # Use the syringe key to load ink.
                self.syringes[syringe_key].load_ink(ink_amount)
                self.movep(syringe_key, ink_amount)
            else:
                print(f"Could not find a matching inkwell for syringe {syringe_key} or missing well location.")

    def how_much_ink(self, pf):
        """
        Returns ink amounts (p1, p2, p3) from the last waypoint of the print file.
        """
        waypoint = pf.waypoints
        if waypoint and all(key in waypoint[-1] for key in ["p1", "p2", "p3"]):
            p1 = waypoint[-1]["p1"]
            p2 = waypoint[-1]["p2"]
            p3 = waypoint[-1]["p3"]
            return p1, p2, p3
        else:
            print("Error: Last row of waypoint does not contain expected data for p1, p2, and p3.")
            return None, None, None

    def find_well_xy(self, well_id):
        """
        Expects well_id to be a string like "A1". Uses well_properties to compute XY coordinates.
        """
        try:
            dx = float(self.well_properties["well_dx"])
            dy = float(self.well_properties["well_dy"])
            x0 = float(self.well_properties["Well_A1_x"])
            y0 = float(self.well_properties["Well_A1_y"])
            cols = int(self.well_properties["well_cols"])
            rows = int(self.well_properties["well_rows"])
        except (KeyError, ValueError, TypeError) as e:
            print(f"Missing or invalid well properties: {e}")
            return None

        if not well_id or len(well_id) < 2:
            print(f"Invalid well_id format: '{well_id}'. Expected format like 'A1'.")
            return None

        row_char = well_id[0].upper()
        try:
            row = ord(row_char) - ord('A')
        except Exception as e:
            print(f"Error parsing row from well_id '{well_id}': {e}")
            return None

        try:
            col = int(well_id[1:]) - 1
        except ValueError:
            print(f"Invalid column value in well_id '{well_id}'.")
            return None

        if not (0 <= row < rows) or not (0 <= col < cols):
            print(f"Well ID '{well_id}' is out of range for this plate (rows: {rows}, cols: {cols}).")
            return None

        x = x0 + col * dx
        y = y0 + row * dy
        return x, y

    def stop(self):
        self.stop_flag = True

    def __del__(self):
        self.stop()
