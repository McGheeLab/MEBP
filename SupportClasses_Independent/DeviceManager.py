import time
import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import threading
from SupportClasses.DeviceInterface import XYStageManager, ZPStageManager

class Stages:
    UPDATE_INTERVAL_XY = 1  # Time between updates in seconds (for XY stage)
    UPDATE_INTERVAL_ZP = 0.5  # Time between updates for ZP stage
    
    # PID controller gains for Prior III XY stage
    Kp = 0.5 #0.33
    Ki = 0 #0.001
    Kd = 0 #0.02
    
    def __init__(self, waypoints, simulate=False):
        self.simulate = simulate
        self.xy_manager = XYStageManager(simulate=self.simulate)
        self.zp_manager = ZPStageManager(simulate=self.simulate)

        self.current_positions_zp = {'x': 0, 'y': 0, 'z': 0, 'e': 0}
        self.waypoints = waypoints



        # Initialize PID controller variables
        self.error_sum_x = 0.0
        self.error_sum_y = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0

        self.max_velocity = 1000  # Define a maximum allowable velocity for XY

        # Flags to signal threads to stop
        self.stop_flag = False

    def __del__(self):
        """Ensure the simulator stops when the XYStageManager instance is destroyed."""
        if self.simulate:
            if self.xy_manager.spo.running:
                self.xy_manager.spo.stop()
            if self.zp_manager.serial.running:
                self.zp_manager.serial.stop()

    def calculate_velocity_with_pid(self, error_x, error_y, delta_time):
        """Calculate velocity using PID control based on error."""
        # Proportional term: multiply error by Kp
        P_x = self.Kp * error_x
        P_y = self.Kp * error_y

        # Integral term: accumulate error over time
        self.error_sum_x += error_x * delta_time
        self.error_sum_y += error_y * delta_time
        I_x = self.Ki * self.error_sum_x
        I_y = self.Ki * self.error_sum_y

        # Derivative term: rate of change of error
        D_x = self.Kd * (error_x - self.last_error_x) / delta_time if delta_time > 0 else 0
        D_y = self.Kd * (error_y - self.last_error_y) / delta_time if delta_time > 0 else 0

        # PID output: sum of P, I, D components
        vx = P_x + I_x + D_x
        vy = P_y + I_y + D_y

        # Store current error for the next loop
        self.last_error_x = error_x
        self.last_error_y = error_y

        return vx, vy

    def xy_update_thread(self, start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y):
        """Thread function to update XY stage independently."""
        # Retrieve the initial XY position
        x0, y0, _ = self.xy_manager.get_current_position()
        
        if x0 is None or y0 is None:
            # Stop if we can't get initial position
            print("Failed to retrieve initial XY position. Stopping XY updates.")
            return

        # Schedule next update
        next_update_time_xy = start_time
        
        while not self.stop_flag:
            # Check current time
            current_time = time.time()
            
            if current_time >= next_update_time_xy:
                # Compute elapsed time since movement started
                elapsed_time_xy = current_time - start_time
                
                # Interpolate target XY positions from waypoints
                interpolated_values_xy = self.waypoints.interpolate_waypoints(
                    elapsed_time_xy, x0, y0, 0, 0, 0, 0, interpolation_type
                )

                # If no waypoints left, stop
                if interpolated_values_xy is None:
                    break

                # Extract desired XY positions
                target_x = interpolated_values_xy['x']
                target_y = interpolated_values_xy['y']

                # Get current XY position
                current_x, current_y, _ = self.xy_manager.get_current_position()
                if current_x is None or current_y is None:
                    print("Failed to retrieve XY position. Stopping XY updates.")
                    break

                # Calculate position errors
                error_x = target_x - current_x
                error_y = target_y - current_y

                # Calculate velocities using PID
                vx, vy = self.calculate_velocity_with_pid(error_x, error_y, self.UPDATE_INTERVAL_XY)
                
                # Check if velocity exceeds maximum
                velocity_magnitude = np.hypot(vx, vy)
                if velocity_magnitude > self.max_velocity:
                    # Scale down velocity and reset integrator
                    scaling_factor = self.max_velocity / velocity_magnitude
                    vx *= scaling_factor
                    vy *= scaling_factor
                    self.error_sum_x = 0.0
                    self.error_sum_y = 0.0

                # Move stage at computed velocity
                self.xy_manager.move_stage_at_velocity(vx, vy)
                
                # Record ideal and actual positions
                ideal_path_x.append(target_x)
                ideal_path_y.append(target_y)
                actual_path_x.append(current_x)
                actual_path_y.append(current_y)

                # Schedule the next update
                next_update_time_xy += self.UPDATE_INTERVAL_XY

            # Brief pause
            time.sleep(0.001)

        # Stop XY stage movement after completion
        self.xy_manager.move_stage_at_velocity(0, 0)
        print("XY updates complete.")

    def zp_update_thread(self, start_time, interpolation_type,
                         ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                         actual_path_z, actual_p1_list, actual_p2_list, actual_p3_list):
        # Thread to update ZP stage at regular intervals
        z0, p10, p20, p30 = self.zp_manager.get_position()
        
        # Default to zero if reading initial ZP position fails
        if z0 is None:
            z0, p10, p20, p30 = 0, 0, 0, 0
            print("Failed to get ZP initial position, defaulting to 0.")

        # Switch G-code to absolute positioning
        self.zp_manager.send_data("G90")
        
        # Adjust timings for smoother movement
        timefactor = -0.15 * self.UPDATE_INTERVAL_ZP
        next_update_time_zp = start_time

        # Continuously update while not signaled to stop
        while not self.stop_flag:
            current_time = time.time()
            if current_time >= next_update_time_zp:
                # Attempt to read current ZP position; default if failure
                z, p1, p2, p3 = self.zp_manager.get_position()
                if z is None:
                    z, p1, p2, p3 = 0, 0, 0, 0

                # Store actual positions
                actual_path_z.append(z)
                actual_p1_list.append(p1)
                actual_p2_list.append(p2)
                actual_p3_list.append(p3)

                # Determine how far along we are
                elapsed_time_zp = current_time - start_time
                
                # Get target positions from waypoint interpolation
                interpolated_values_zp = self.waypoints.interpolate_waypoints(
                    elapsed_time_zp, 0, 0, z0, p10, p20, p30, interpolation_type
                )

                # Stop if no more waypoints
                if interpolated_values_zp is None:
                    break

                # Extract ZP targets from interpolation
                target_z = interpolated_values_zp['z']
                target_p1 = interpolated_values_zp['p1']
                target_p2 = interpolated_values_zp['p2']
                target_p3 = interpolated_values_zp['p3']

                # Build axes dict for movement
                zp_axes = {
                    'X': target_z,
                    'Y': target_p1,
                    'Z': target_p2,
                    'E': target_p3
                }

                # Calculate feedrate based on largest move distance
                dist_z = abs(target_z - z)
                dist_p1 = abs(target_p1 - p1)
                dist_p2 = abs(target_p2 - p2)
                dist_p3 = abs(target_p3 - p3)
                max_distance = max(dist_z, dist_p1, dist_p2, dist_p3)
                feedrate = (max_distance / (self.UPDATE_INTERVAL_ZP + timefactor)) * 60 if max_distance > 0 else 100.0

                # Send movement command to ZP manager
                self.zp_manager.movecommand(zp_axes, feedrate=feedrate)

                # Store ideal (interpolated) positions
                ideal_path_z.append(interpolated_values_zp['z'])
                ideal_p1.append(interpolated_values_zp['p1'])
                ideal_p2.append(interpolated_values_zp['p2'])
                ideal_p3.append(interpolated_values_zp['p3'])

                # Schedule next update
                next_update_time_zp += self.UPDATE_INTERVAL_ZP

            # Small delay until next check
            time.sleep(0.001)

        # Switch back to relative positioning after movement finishes
        self.zp_manager.send_data("G91")
        print("ZP updates complete.")

    def move(self, plot_title, interpolation_type="linear", plot=False):
        # Prepare storage lists
        actual_path_x = []
        actual_path_y = []
        actual_path_z = []

        ideal_path_x = []
        ideal_path_y = []
        ideal_path_z = []

        actual_p1 = []
        actual_p2 = []
        actual_p3 = []

        ideal_p1 = []
        ideal_p2 = []
        ideal_p3 = []

        start_time = time.time()

        # Create threads for XY and ZP updates
        xy_thread = threading.Thread(
            target=self.xy_update_thread,
            args=(start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y)
        )
        zp_thread = threading.Thread(
            target=self.zp_update_thread,
            args=(start_time, interpolation_type,
                  ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                  actual_path_z, actual_p1, actual_p2, actual_p3)
        )

        # Start threads
        xy_thread.start()
        zp_thread.start()

        # Wait for both threads to finish
        xy_thread.join()
        zp_thread.join()

        print(f"{plot_title} complete.")

        if plot:
            self.plot_results_3d(
                ideal_path_x, ideal_path_y, ideal_path_z,
                actual_path_x, actual_path_y, actual_path_z,
                ideal_p1, ideal_p2, ideal_p3,
                actual_p1, actual_p2, actual_p3,
                plot_title
            )

    def plot_results_3d(
        self, ideal_x, ideal_y, ideal_z,
        actual_x, actual_y, actual_z,
        ideal_p1, ideal_p2, ideal_p3,
        actual_p1, actual_p2, actual_p3,
        plot_title
    ):
        """Plot the results of the stage movement in 3D."""

        # Determine the maximum length among all arrays
        max_length = max(
            len(ideal_x), len(ideal_y), len(ideal_z),
            len(actual_x), len(actual_y), len(actual_z),
            len(ideal_p1), len(ideal_p2), len(ideal_p3),
            len(actual_p1), len(actual_p2), len(actual_p3)
        )

        # Create a common time array for interpolation
        common_time = np.linspace(0, 1, max_length)

        # Function to interpolate data arrays to the common time array
        def interpolate_array(data_array):
            original_time = np.linspace(0, 1, len(data_array))
            interpolation_function = interp1d(
                original_time, data_array, kind='linear', fill_value="extrapolate"
            )
            return interpolation_function(common_time)

        # Interpolate all data arrays
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

        # Detect changes in p1, p2, p3
        positions_p1 = []
        positions_p2 = []
        positions_p3 = []

        prev_p1 = None
        prev_p2 = None
        prev_p3 = None

        # Use a small tolerance for floating-point comparisons
        tolerance = 1e-2

        for i in range(len(actual_p1)):
            current_p1 = actual_p1[i]
            current_p2 = actual_p2[i]
            current_p3 = actual_p3[i]

            # Use the positions from the actual path
            x_pos = actual_x[i]
            y_pos = actual_y[i]
            z_pos = actual_z[i]

            # Check for significant changes in p1
            if prev_p1 is not None and abs(current_p1 - prev_p1) > tolerance:
                positions_p1.append((x_pos, y_pos, z_pos))
            # Check for significant changes in p2
            if prev_p2 is not None and abs(current_p2 - prev_p2) > tolerance:
                positions_p2.append((x_pos*1.02, y_pos*1.02, z_pos))
            # Check for significant changes in p3
            if prev_p3 is not None and abs(current_p3 - prev_p3) > tolerance:
                positions_p3.append((x_pos*0.98, y_pos*0.98, z_pos))

            # Update previous values
            prev_p1 = current_p1
            prev_p2 = current_p2
            prev_p3 = current_p3

        # Proceed with plotting
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Plot ideal XY path
        ax.plot(ideal_x, ideal_y, ideal_z, label='Ideal XY Path', linestyle='--', color='blue')
        # Plot actual XY path
        ax.plot(actual_x, actual_y, actual_z, label='Actual XY Path', linestyle='-', color='red')

        # Plot markers where p1, p2, p3 changed
        if positions_p1:
            x_p1, y_p1, z_p1 = zip(*positions_p1)
            ax.scatter(x_p1, y_p1, z_p1, c='green', marker='^', label='p1 Change')
        if positions_p2:
            x_p2, y_p2, z_p2 = zip(*positions_p2)
            ax.scatter(x_p2, y_p2, z_p2, c='magenta', marker='s', label='p2 Change')
        if positions_p3:
            x_p3, y_p3, z_p3 = zip(*positions_p3)
            ax.scatter(x_p3, y_p3, z_p3, c='cyan', marker='o', label='p3 Change')

        # Customize the plot
        ax.set_xlabel('X Position (microns)')
        ax.set_ylabel('Y Position (microns)')
        ax.set_zlabel('Z Position (microns)')
        ax.set_title(f'Stage Movement: {plot_title}')
        ax.legend()
        plt.show()


class Waypoint:
    def __init__(self, csv_file_path='waypoints.csv'):
        # Store the path to the CSV file containing waypoint data
        self.csv_file_path = csv_file_path
        
        # Initialize an empty list to hold waypoint dictionaries
        self.waypoints = []
        
        # Automatically load waypoints from the CSV file upon creation
        self.import_waypoints_from_csv()

    def import_waypoints_from_csv(self):
        """
        Loads waypoint data (x, y, z, p1, p2, p3, t) from a CSV file.
        Each row should match the required format:
            x, y, z, p1, p2, p3, t
        """
        self.waypoints = []
        try:
            with open(self.csv_file_path, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    # Skip a header row if it starts with 'x'
                    if row[0].startswith('x'):
                        continue
                    # Ensure row length is correct
                    elif len(row) == 7:
                        x, y, z, p1, p2, p3, t = map(float, row)
                        # Store the numeric data in a dict
                        waypoint = {
                            'x': x,
                            'y': y,
                            'z': z,
                            'p1': p1,
                            'p2': p2,
                            'p3': p3,
                            't': t
                        }
                        # Append the constructed waypoint to the list
                        self.waypoints.append(waypoint)
                    else:
                        # Notify if row is invalid
                        print(f"Invalid row length: {row}")
        except FileNotFoundError:
            # Error if the specified file is missing
            print(f"Error: File not found at {self.csv_file_path}")
        except Exception as e:
            # Catch-all for unexpected issues
            print(f"Error reading CSV file: {e}")
        
        # Return the list of waypoints that were loaded
        return self.waypoints

    def interpolate_waypoints(
        self, 
        elapsed_time, 
        x0=0, 
        y0=0, 
        z0=0, 
        p10=0, 
        p20=0, 
        p30=0, 
        interpolation_type="linear"
    ):
        """
        Given an elapsed time, compute the interpolated position 
        by referencing the loaded waypoints. 
        Positions (x, y, z, p1, p2, p3) are offset by provided initial values.
        
        interpolation_type can be:
          "linear"     - linear interpolation
          "polynomial" - polynomial fit (up to degree 3)
          "spline"     - cubic spline interpolation
        """
        # Local reference to the list of loaded waypoints
        waypoints = self.waypoints

        # If no waypoints exist, interpolation cannot be performed
        if not waypoints:
            return None

        # If the elapsed time is beyond the final waypoint, return None
        if elapsed_time > waypoints[-1]['t']:
            return None

        # Extract just the time values from each waypoint for interpolation
        times = [wp['t'] for wp in waypoints]
        
        # Set which keys in each waypoint we will interpolate
        data_keys = ['x', 'y', 'z', 'p1', 'p2', 'p3']
        
        # Prepare a dictionary to store interpolated values for each key
        interpolated_values = {}

        # Loop over each key (x, y, z, p1, p2, p3) to create and use interpolation functions
        for key in data_keys:
            # Extract only the relevant coordinate from each waypoint
            values = [wp[key] for wp in waypoints]
            
            # Choose interpolation strategy based on user input
            if interpolation_type == "linear":
                interp_func = interp1d(times, values, kind='linear', fill_value="extrapolate")
            elif interpolation_type == "polynomial":
                # Polyfit degree is limited to 3 or the number of waypoints - 1 (whichever is smaller)
                degree = min(3, len(waypoints) - 1)
                interp_func = np.poly1d(np.polyfit(times, values, degree))
            elif interpolation_type == "spline":
                # Use a cubic spline for smoother interpolations
                interp_func = CubicSpline(times, values)
            else:
                # If user requests unsupported method, raise an error
                raise ValueError(f"Unsupported interpolation type: {interpolation_type}")
            
            # Generate an interpolated value for the specific key at elapsed_time
            interpolated_values[key] = interp_func(elapsed_time)

        # Adjust the interpolated positions by adding the starting offsets (x0, y0, etc.)
        interpolated_values['x'] += x0
        interpolated_values['y'] += y0
        interpolated_values['z'] += z0
        interpolated_values['p1'] += p10
        interpolated_values['p2'] += p20  # Add p2 offset
        interpolated_values['p3'] += p30  # Add p3 offset

        # Return the final computed positions in a dictionary
        return interpolated_values
