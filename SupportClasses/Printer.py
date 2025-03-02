import threading
import csv
import time
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import matplotlib.pyplot as plt

class PrintManager:
    def __init__(self, app_controller):
        """
        Initialize PrintManager with an AppController (which already holds a running StageHandler
        and Processor). The well queue is initially empty.
        """
        self.app_controller = app_controller
        self.processor = app_controller.processor

        # Ensure stage devices are running.
        if app_controller.stage_handler is None:
            print("StageHandler not running. Starting stage devices.")
            self.app_controller.start_stage_devices()
        self.stage_handler = self.app_controller.stage_handler

        # Use update intervals from StageHandler.
        self.xy_interval = self.stage_handler.XYUPDATE_INTERVAL  # e.g., 1.0 sec
        self.zp_interval = self.stage_handler.ZUPDATE_INTERVAL    # e.g., 0.5 sec

        # Well queue: each entry is a dict with keys: "well_id", "offset", and "waypoint" (a Waypoint instance)
        self.well_queue = []

        # PID parameters for XY motion.
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.error_sum_x = 0.0
        self.error_sum_y = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.max_velocity = 1000  # Maximum allowable XY velocity

        # dict of well properties
        self.well_properties = {"fastz": 0, "floorz": 0, "topz": 0, "ink_topz":0, "ink_floorz":0 , "Well_A1_x": 0, "Well_A1_y": 0,
                                "well_dx": 0, "well_dy": 0, "well_rows": 0, "well_cols": 0, "well_diameter": 0, "plate_mode": "Well Plate"}
        
        # Initialize ink wells as a dictionary of InkWell objects.
        self.ink_wells = {}
        
        # Syringe instances for each pump.
        self.syringes = {"p1": Syringe("p1") , "p2": Syringe("p2") , "p3": Syringe("p3") }
        
        # Flags for stopping/pausing.
        self.stop_flag = False
        self.pause_flag = False
        
        # For freezing elapsed time during pause.
        self.time_offset = 0.0  
        self.last_pause_time = None

        # Register processor commands for well queue management and print control.
        self.processor.register_handler("queue_waypoint", self.queue_a_waypoint)
        self.processor.register_handler("get_waypoints", self.handle_get_waypoints)
        self.processor.register_handler("control_print", self.handle_control_print)

    def set_well_properties(self, well_properties):
        """Set the well properties for the plate."""
        print("Well properties updated from:", self.well_properties)
        self.well_properties.update(well_properties)
        print("Well properties updated to:", self.well_properties)
    
    def set_ink_wells(self, ink_wells):
        """Set the ink wells for the plate."""
        self.ink_wells = ink_wells
        print("Ink wells updated to:", self.ink_wells)
        
    def calculate_velocity_with_pid(self, error_x, error_y, delta_time):
        """Calculate XY velocity using a PID controller."""
        # Proportional term
        P_x = self.Kp * error_x
        P_y = self.Kp * error_y

        # Integral term
        self.error_sum_x += error_x * delta_time
        self.error_sum_y += error_y * delta_time
        I_x = self.Ki * self.error_sum_x
        I_y = self.Ki * self.error_sum_y

        # Derivative term
        D_x = self.Kd * ((error_x - self.last_error_x) / delta_time) if delta_time > 0 else 0
        D_y = self.Kd * ((error_y - self.last_error_y) / delta_time) if delta_time > 0 else 0

        vx = P_x + I_x + D_x
        vy = P_y + I_y + D_y

        # Update last errors.
        self.last_error_x = error_x
        self.last_error_y = error_y

        return vx, vy

    def xy_print_thread(self, start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, wp_obj):
        """Thread to update XY axes via processor commands using PID control."""
        # Get initial XY positions from StageHandler.
        stage_info = self.app_controller.get_stage_info()
        initial_x = stage_info.get("XY", {}).get("x", {}).get("position", 0.0)
        initial_y = stage_info.get("XY", {}).get("y", {}).get("position", 0.0)
        if initial_x is None or initial_y is None:
            print("Failed to retrieve initial XY position. Stopping XY updates.")
            return

        next_update_time = start_time
        while not self.stop_flag:
            current_time = time.time()

            # Handle pause/resume logic.
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
                if wp_obj is None:
                    print("No waypoint provided. Stopping XY print thread.")
                    break

                interpolated = wp_obj.interpolate_waypoints(
                    effective_elapsed, x0=initial_x, y0=initial_y, z0=0, p10=0, p20=0, p30=0, interpolation_type=interpolation_type
                )
                if interpolated is None:
                    break

                target_x = interpolated['x']
                target_y = interpolated['y']

                # Get current positions.
                stage_info = self.app_controller.get_stage_info()
                current_x = stage_info.get("XY", {}).get("x", {}).get("position", 0.0)
                current_y = stage_info.get("XY", {}).get("y", {}).get("position", 0.0)

                error_x = target_x - current_x
                error_y = target_y - current_y
                vx, vy = self.calculate_velocity_with_pid(error_x, error_y, self.xy_interval)
                if np.hypot(vx, vy) > self.max_velocity:
                    scaling = self.max_velocity / np.hypot(vx, vy)
                    vx *= scaling
                    vy *= scaling
                    self.error_sum_x = 0.0
                    self.error_sum_y = 0.0

                # Issue move command via processor.
                self.processor.add_command("move_stage_at_velocity", average=(vx, vy))
                ideal_path_x.append(target_x)
                ideal_path_y.append(target_y)
                actual_path_x.append(current_x)
                actual_path_y.append(current_y)

                next_update_time += self.xy_interval
            time.sleep(0.001)

        # When finished, send a stop command.
        self.processor.add_command("move_stage_at_velocity", average=(0, 0))
        print("XY print updates complete.")

    def zp_print_thread(self, start_time, interpolation_type,
                        ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                        actual_path_z, actual_p1, actual_p2, actual_p3, wp_obj):
        """Thread to update Z and extruder axes (p1, p2, p3) via processor commands."""
        stage_info = self.app_controller.get_stage_info()
        initial_z = stage_info.get("ZP", {}).get("Z", {}).get("position", 0.0)
        initial_p1 = stage_info.get("ZP", {}).get("P1", {}).get("position", 0.0)
        initial_p2 = stage_info.get("ZP", {}).get("P2", {}).get("position", 0.0)
        initial_p3 = stage_info.get("ZP", {}).get("P3", {}).get("position", 0.0)
        if initial_z is None:
            initial_z = initial_p1 = initial_p2 = initial_p3 = 0
            print("Failed to get ZP initial position, defaulting to 0.")

        next_update_time = start_time
        timefactor = -0.15 * self.zp_interval

        while not self.stop_flag:
            current_time = time.time()

            # Pause handling.
            if self.pause_flag:
                if self.last_pause_time is None:
                    self.last_pause_time = current_time
                    self.processor.add_command("move_z_at_velocity", average=(0, 0))
                    self.processor.add_command("move_p1_at_velocity", average=(0, 0))
                    self.processor.add_command("move_p2_at_velocity", average=(0, 0))
                    self.processor.add_command("move_p3_at_velocity", average=(0, 0))
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
                actual_path_z.append(current_z)  # (if desired, duplicate for clarity)
                actual_p2.append(current_p2)
                actual_p3.append(current_p3)

                effective_elapsed = current_time - start_time - self.time_offset
                if wp_obj is None:
                    print("No waypoint provided. Stopping ZP print thread.")
                    break

                interpolated = wp_obj.interpolate_waypoints(
                    effective_elapsed, x0=0, y0=0, z0=initial_z, p10=initial_p1, p20=initial_p2, p30=initial_p3, interpolation_type=interpolation_type
                )
                if interpolated is None:
                    break

                target_z = interpolated['z']
                target_p1 = interpolated['p1']
                target_p2 = interpolated['p2']
                target_p3 = interpolated['p3']

                dt = self.zp_interval
                v_z = (target_z - current_z) / dt
                v_p1 = (target_p1 - current_p1) / dt
                v_p2 = (target_p2 - current_p2) / dt
                v_p3 = (target_p3 - current_p3) / dt

                self.processor.add_command("move_z_at_velocity", average=(v_z, 0))
                self.processor.add_command("move_p1_at_velocity", average=(v_p1, 0))
                self.processor.add_command("move_p2_at_velocity", average=(v_p2, 0))
                self.processor.add_command("move_p3_at_velocity", average=(v_p3, 0))

                ideal_path_z.append(target_z)
                ideal_p1.append(target_p1)
                ideal_p2.append(target_p2)
                ideal_p3.append(target_p3)

                next_update_time += self.zp_interval
            time.sleep(0.001)

        self.processor.add_command("move_z_at_velocity", average=(0, 0))
        self.processor.add_command("move_p1_at_velocity", average=(0, 0))
        self.processor.add_command("move_p2_at_velocity", average=(0, 0))
        self.processor.add_command("move_p3_at_velocity", average=(0, 0))
        print("ZP print updates complete.")

    def plot_results_3d(self, ideal_x, ideal_y, ideal_z,
                        actual_x, actual_y, actual_z,
                        ideal_p1, ideal_p2, ideal_p3,
                        actual_p1, actual_p2, actual_p3,
                        plot_title):
        """Plot the ideal versus actual paths in 3D."""
        max_length = max(
            len(ideal_x), len(ideal_y), len(ideal_z),
            len(actual_x), len(actual_y), len(actual_z),
            len(ideal_p1), len(ideal_p2), len(ideal_p3),
            len(actual_p1), len(actual_p2), len(actual_p3)
        )
        common_time = np.linspace(0, 1, max_length)

        def interpolate_array(data_array):
            original_time = np.linspace(0, 1, len(data_array))
            interp_func = interp1d(original_time, data_array, kind='linear', fill_value="extrapolate")
            return interp_func(common_time)

        ideal_x = interpolate_array(ideal_x)
        ideal_y = interpolate_array(ideal_y)
        ideal_z = interpolate_array(ideal_z)
        actual_x = interpolate_array(actual_x)
        actual_y = interpolate_array(actual_y)
        actual_z = interpolate_array(actual_z)
        ideal_p1 = interpolate_array(ideal_p1)
        ideal_p2 = interpolate_array(ideal_p2)
        ideal_p3 = interpolate_array(ideal_p3)
        actual_p1 = interpolate_array(actual_p1)
        actual_p2 = interpolate_array(actual_p2)
        actual_p3 = interpolate_array(actual_p3)

        positions_p1, positions_p2, positions_p3 = [], [], []
        prev_p1 = prev_p2 = prev_p3 = None
        tol = 1e-2
        for i in range(len(actual_p1)):
            cp1, cp2, cp3 = actual_p1[i], actual_p2[i], actual_p3[i]
            x_pos, y_pos, z_pos = actual_x[i], actual_y[i], actual_z[i]
            if prev_p1 is not None and abs(cp1 - prev_p1) > tol:
                positions_p1.append((x_pos, y_pos, z_pos))
            if prev_p2 is not None and abs(cp2 - prev_p2) > tol:
                positions_p2.append((x_pos*1.02, y_pos*1.02, z_pos))
            if prev_p3 is not None and abs(cp3 - prev_p3) > tol:
                positions_p3.append((x_pos*0.98, y_pos*0.98, z_pos))
            prev_p1, prev_p2, prev_p3 = cp1, cp2, cp3

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(ideal_x, ideal_y, ideal_z, label='Ideal XY Path', linestyle='--', color='blue')
        ax.plot(actual_x, actual_y, actual_z, label='Actual XY Path', linestyle='-', color='red')
        if positions_p1:
            x_p1, y_p1, z_p1 = zip(*positions_p1)
            ax.scatter(x_p1, y_p1, z_p1, c='green', marker='^', label='p1 Change')
        if positions_p2:
            x_p2, y_p2, z_p2 = zip(*positions_p2)
            ax.scatter(x_p2, y_p2, z_p2, c='magenta', marker='s', label='p2 Change')
        if positions_p3:
            x_p3, y_p3, z_p3 = zip(*positions_p3)
            ax.scatter(x_p3, y_p3, z_p3, c='cyan', marker='o', label='p3 Change')
        ax.set_xlabel('X Position (microns)')
        ax.set_ylabel('Y Position (microns)')
        ax.set_zlabel('Z Position (microns)')
        ax.set_title(f'Print Movement: {plot_title}')
        ax.legend()
        plt.show()

    def start_print(self, print_title, interpolation_type="linear", plot=False, wp_obj=None):
        """
        Begin the print job using the given waypoint object.
        If wp_obj is None, the method does nothing.
        """
        if wp_obj is None:
            print("No waypoint provided to start_print().")
            return

        # Reset time offset and flags.
        self.stop_flag = False
        self.pause_flag = False
        self.time_offset = 0.0
        self.last_pause_time = None

        # Storage for path data (for optional plotting)
        actual_path_x, ideal_path_x = [], []
        actual_path_y, ideal_path_y = [], []
        actual_path_z, ideal_path_z = [], []
        actual_p1, ideal_p1 = [], []
        actual_path_p2, ideal_p2 = [], []
        actual_path_p3, ideal_p3 = [], []

        start_time = time.time()

        # Start the XY and ZP threads with the given waypoint object.
        xy_thread = threading.Thread(
            target=self.xy_print_thread,
            args=(start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, wp_obj)
        )
        zp_thread = threading.Thread(
            target=self.zp_print_thread,
            args=(start_time, interpolation_type, ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                  actual_path_z, actual_p1, actual_path_p2, actual_path_p3, wp_obj)
        )

        xy_thread.start()
        zp_thread.start()
        xy_thread.join()
        zp_thread.join()

        print(f"{print_title} complete.")

        if plot:
            self.plot_results_3d(
                ideal_path_x, ideal_path_y, ideal_path_z,
                actual_path_x, actual_path_y, actual_path_z,
                ideal_p1, ideal_p2, ideal_p3,
                actual_p1, actual_path_p2, actual_path_p3,
                print_title
            )

    def process_print_queue(self, interpolation_type="linear", plot=True):
        """
        Process the well queue one entry at a time.
        For each waypoint dict, start a print job (with plotting), then move on.
        """
        while self.well_queue:
            next_entry = self.well_queue.pop(0)
            wp_obj = next_entry["waypoint"]
            well_id = wp_obj.well_id
            offset = wp_obj.offset
            
            print(f"Starting print for well '{well_id}' with offset {offset}.")
            
            # prepare for print
            self.prepare_print(offset, well_id, offset)
            
            # start print
            self.start_print(print_title=f"Print for well {well_id}", interpolation_type=interpolation_type, plot=plot, wp_obj=wp_obj)
            
            # slow move to z top
            self.slowmovez(self.well_properties["topz"])
                    
            # Optionally, add a short delay between jobs.
            time.sleep(0.5)
        print("Well queue processing complete.")

    def prepare_print(self, wp_obj):
        """ 
        Prepare for printing by moving to the ink well to pickup ink and then to the well to print.
        input:
            well_id: str, the well id of the well to print to i.e. 'A1'
            offset: tuple (x,y,z), the offset from the well center and floor to start the print
            wp_obj: Waypoint object, the waypoints to print
        """
        
        # fast move to z so we can do fast xy moves
        self.fastmovez(self.well_properties["fastz"])
            
        # load inks based on the total movement of the syringes in the waypoint file
        self.load_ink(wp_obj)
        
        # fast move to xy well + xy offset
        well_xy = self.find_well_xy(wp_obj.well_id)
        self.fastmovexy(well_xy[0] + wp_obj.offset[0], well_xy[1] + wp_obj.offset[1])
        
        # fastmove to well top
        self.fastmovez(self.well_properties["topz"])
        
        # slow move to z floor - z offset
        self.slowmovez(self.well_properties["floorz"] - wp_obj.offset[2])  
    
    def fastmovez(self, z):
        self.app_controller.stage_handler.zp_stage.move_abs_z_zero_reference(z,True)    
        
    def fastmovexy(self, x, y):
        self.app_controller.stage_handler.xy_stage.move_stage_to_position(x, y)
    
    def slowmovez(self, z):
        self.app_controller.stage_handler.zp_stage.move_abs_z_zero_reference(z,False)
    
    def movep(self, p, amount):
        """
        Move the syringe in the p axis by the amount specified
        input:
            p: str, the syringe to move p1, p2, or p3
            amount: float, the volume the syringe should pickup
        """
        if p == "p1":
            # lookup the syringe object for p1 and ask it to calculate displacement
            distance = self.syringes[p].calculate_displacement(amount) 
            self.stage_handler.move_rel_p('P1', distance)
        elif p == "p2":
            distance = self.syringes[p].calculate_displacement(amount) 
            self.stage_handler.move_rel_p('P2', distance)
        elif p == "p3":
            distance = self.syringes[p].calculate_displacement(amount) 
            self.stage_handler.move_rel_p('P3', distance)
    
    def load_ink(self, wp_obj):
        """
        run through the ink needs and load the ink in the corresponding wells
        """
        # get the amount of ink needed for each p1, p2, p3
        p1, p2, p3 = self.how_much_ink(wp_obj)
              
        # lookup the corresponding ink well for each p1, p2, p3 cell needs based on the objects celltype property
        p1_well = next((well_id for well_id, well in self.ink_wells.items() if well.cell_type == self.syringes["p1"].cell_type), None)
        p2_well = next((well_id for well_id, well in self.ink_wells.items() if well.cell_type == self.syringes["p2"].cell_type), None)
        p3_well = next((well_id for well_id, well in self.ink_wells.items() if well.cell_type == self.syringes["p3"].cell_type), None)
        
        # move to each of the ink wells and load the proper volume of ink
        for well_id, ink_amount, syringe in zip([p1_well, p2_well, p3_well], [p1, p2, p3], ["p1", "p2", "p3"]):
            if well_id is not None:
                well_xy = self.find_well_xy(well_id)
                self.fastmovez(self.well_properties["fastz"])
                self.fastmovexy(well_xy[0], well_xy[1])
                self.fastmovez(self.well_properties["ink_topz"])
                self.slowmovez(self.well_properties["ink_floorz"])
                self.syringes[well_id].load_ink(ink_amount)
                self.movep(syringe, ink_amount)
        
    # --- Utility Functions ---
    def how_much_ink(self, wp_obj):
        """
        Calculate how much ink is needed for the print.
        input:
            wp_obj: Waypoint object, the waypoints to print in the format x,y,z,p1,p2,p3,t
            where the total movement of each p1, p2, p3 is the amount of ink needed
        """
        # load ink well xy locations and store them in a dictionary
        for key, value in self.ink_wells.items():
            self.ink_wells[key] = self.find_well_xy(value)  # find the xy location of the well
        # load in the waypoint .csv file with x,y,z,p1,p2,p3,t on each row
        waypoint = wp_obj.waypoints
        # the total movement of each p1, p2, p3 is in the last row of each waypoint dict
        if waypoint and all(key in waypoint[-1] for key in ["p1", "p2", "p3"]):
            p1 = waypoint[-1]["p1"]
            p2 = waypoint[-1]["p2"]
            p3 = waypoint[-1]["p3"]
        else:
            print("Error: Last row of waypoint does not contain expected data for p1, p2, and p3.")
            return None, None, None
        return p1, p2, p3
     
    def find_well_xy(self, well_id):
        """
        Calculate the XY position for a given well ID.
        Expects well_id in the format 'A1' where the letter indicates the row and the number the column.
        
        Returns:
            A tuple (x, y) if well_id is valid, otherwise None.
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

        # check well_id format
        if not well_id or len(well_id) < 3:
            print(f"Invalid well_id format: '{well_id}'. Expected format like 'A1'.")
            return None

        row_char = well_id[0].upper()
        try:
            # Convert row letter to index (0-based)
            row = ord(row_char) - ord('A')
        except Exception as e:
            print(f"Error parsing row from well_id '{well_id}': {e}")
            return None

        # Check column number
        try:
            # Convert column number to index (0-based)
            col = int(well_id[1:]) - 1
        except ValueError:
            print(f"Invalid column value in well_id '{well_id}'.")
            return None

        # Check if the well is within the plate boundaries.
        if not (0 <= row < rows) or not (0 <= col < cols):
            print(f"Well ID '{well_id}' is out of range for this plate (rows: {rows}, cols: {cols}).")
            return None

        x = x0 + col * dx
        y = y0 + row * dy
        return x, y
        
    # --- Processor Command Handlers ---
    def queue_a_waypoint(self, well_id, offset, csv_file, **kwargs):
        """
        Add a new waypoint to the well queue.
        Parameters:
            well_id (str): Identifier for the well.
            target (tuple): (target_x, target_y, target_z)
            csv_file (str): Path to the CSV file for this waypoint.
        """
        wp_obj = Waypoint(csv_file)
        wp_obj.well_id = well_id
        wp_obj.offset = offset
        entry = {"well_id": well_id, "target": offset, "waypoint": wp_obj}
        self.well_queue.append(entry)
        print(f"Queued waypoint for well '{well_id}' with target {offset}.")

    def handle_get_waypoints(self, **kwargs):
        """Print the current well queue (could be extended to return this list to a GUI)."""
        print("Current well queue:")
        for entry in self.well_queue:
            print(entry)

    def handle_control_print(self, action, **kwargs):
        """
        Control the print operation.
          - action = "pause": Pause the print.
          - action = "resume": Resume movement.
          - action = "stop": Stop the print and clear the well queue.
        """
        if action == "pause":
            self.pause_flag = True
            print("Print paused.")
        elif action == "resume":
            self.pause_flag = False
            print("Print resumed.")
        elif action == "stop":
            self.stop_flag = True
            self.well_queue.clear()
            self.processor.add_command("move_stage_at_velocity", average=(0, 0))
            self.processor.add_command("move_z_at_velocity", average=(0, 0))
            self.processor.add_command("move_p1_at_velocity", average=(0, 0))
            self.processor.add_command("move_p2_at_velocity", average=(0, 0))
            self.processor.add_command("move_p3_at_velocity", average=(0, 0))
            print("Print stopped and well queue cleared.")
        else:
            print(f"Unknown print control action: {action}")

    def stop(self):
        """Signal the print threads to stop."""
        self.stop_flag = True

    def __del__(self):
        self.stop()

class Waypoint: # this object loads in a waypoint file and interpolates between waypoints in time
    def __init__(self, csv_file_path='waypoints.csv'):
        self.csv_file_path = csv_file_path
        self.waypoints = []
        self.import_waypoints_from_csv()
        self.well_id = None
        self.offset = None

    def import_waypoints_from_csv(self):
        """Import waypoints from a CSV file."""
        self.waypoints = []
        try:
            with open(self.csv_file_path, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    # Skip header row
                    if row[0].startswith('x'):
                        continue
                    elif len(row) == 7:
                        x, y, z, p1, p2, p3, t = map(float, row)
                        waypoint = {
                            'x': x,
                            'y': y,
                            'z': z,
                            'p1': p1,
                            'p2': p2,
                            'p3': p3,
                            't': t
                        }
                        self.waypoints.append(waypoint)
                    else:
                        print(f"Invalid row length: {row}")
        except FileNotFoundError:
            print(f"Error: File not found at {self.csv_file_path}")
        except Exception as e:
            print(f"Error reading CSV file: {e}")
        return self.waypoints

    def interpolate_waypoints(self, elapsed_time, x0=0, y0=0, z0=0, p1=0, p2=0, p3=0, interpolation_type="linear"):
        """Interpolate between waypoints to get the target position at the given elapsed time."""
        waypoints = self.waypoints

        if not waypoints:
            return None

        if elapsed_time > waypoints[-1]['t']:
            return None

        times = [wp['t'] for wp in waypoints]
        data_keys = ['x', 'y', 'z', 'p1', 'p2', 'p3']
        interpolated_values = {}

        for key in data_keys:
            values = [wp[key] for wp in waypoints]
            if interpolation_type == "linear":
                interp_func = interp1d(times, values, kind='linear', fill_value="extrapolate")
            elif interpolation_type == "polynomial":
                degree = min(3, len(waypoints) - 1)
                interp_func = np.poly1d(np.polyfit(times, values, degree))
            elif interpolation_type == "spline":
                interp_func = CubicSpline(times, values)
            else:
                raise ValueError(f"Unsupported interpolation type: {interpolation_type}")
            interpolated_values[key] = interp_func(elapsed_time)

        # Add initial positions if necessary (assuming positions are relative)
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
        self.max_volume = None # if the plunger is pulled all the way back
        self.current_volume = None # the amount of oil + ink in the syringe
        self.cell_type = None
        self.ink_volume = None # the amount of ink in the syringe
    
    ### Utility Functions ###
    def set_syringe_properties(self, diameter, length):
        self.diameter = diameter
        self.length = length
        self.area = self.calculate_syringe_area(diameter)
        self.max_volume = self.calculate_max_volume(self.area, length)
    
    def calculate_displacement(self, volume) -> float | None:
        """
        calculate the displacement needed to load a given volume in μL to mm displacement
        input:
            volume: float, the volume of ink to load in μL
        """
        # make sure diameter is not None and greater than zero
        if self.area <= 0 or volume <= 0:
            print("Error: Syringe diameter is not set or invalid.")
            return None
            
        # volume in μL divided by the area of the syringe in mm^2 gives the displacement in mm
        displacement = volume / self.area
        return displacement    
    
    def calculate_syringe_area(self,diameter):
        # Precompute the area if not already done
        if self.area is None:
            area = np.pi * (diameter / 2) ** 2
            return area
        else:
            return self.area
        
    def calculate_max_volume(self,area,length):
        """
        Calculate the maximum volume that can be loaded into the syringe.
        input:
            are: float, the area of the syringe in mm2
            length: float, the length of the syringe in mm
        """

        return area * length
    
    ### define ink types ### 
    def assign_cell_type(self, cell_type):
        """
        assign ink to the syringe
        """
        self.cell_type = cell_type
    
    ### Track syringe volume ###
    
    def dispense_ink(self, ink_volume_to_dispense):
        """
        dispense ink from the syringe
        input:
            ink_volume_to_dispense: float, the volume of ink to dispense in μL
        return:
            bool, True if the ink was dispensed, False if there was not enough ink to dispense
        """
        # check to see if all properties are set ink type, ink volume > 0, and syringe volume
        if self.ink_type is None or self.ink_volume is None or self.ink_volume <= 0:
            print("Error: Ink type or ink volume is not set.")
            return False
        
        
        # check if the syringe has enough ink to dispense
        if self.ink_volume < ink_volume_to_dispense:
            print("Error: Not enough ink")
            return False
        else:
            self.ink_volume -= ink_volume_to_dispense
            return True
    
    def load_ink(self, ink_volume): 
        """
        load ink into the syringe
        input:
            ink_volume: float, the volume of ink to load in μL
        return:
            bool, True if the ink was loaded, False if there was not enough volume in the syringe to load the ink
        """
        # check to see if all properties are set ink type, and syringe volume
        if self.ink_type is None or self.ink_volume is None:
            print("Error: Ink type or ink volume is not set.")
            return False
        
        # check if the syringe has enough volume to load the ink
        if self.current_volume + ink_volume > self.max_volume:
            print("Error: Not enough volume in the syringe to load the ink")
            return False
        else:
            self.ink_volume += ink_volume
            return True

class InkWell:
    def __init__(self):
        self.well_location = None        # e.g., "A1"
        self.cell_type = None            # e.g., "cell_type_1"
        self.chilled = True              # boolean indicating if the well is chilled
        self.volume = None               # volume in μL or appropriate unit

    # use __repr__ to define the string representation of the object
    def __repr__(self):
        return (f"InkWell(location={self.well_location}, cell_type='{self.cell_type}', "
                f"chilled={self.chilled}, volume={self.volume})")
        
    def set_cell_type(self, cell_type):
        self.cell_type = cell_type
        
    def set_volume(self, volume):
        self.volume = volume
    
    def set_chilled(self, chilled):
        self.chilled = chilled
    
    def set_location(self, location):
        self.well_location = location
    
    