import time
import matplotlib.pyplot as plt
import csv
import sys
import platform
import socket
import serial
import serial.tools.list_ports
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import threading
import re
import queue

class XYStageManager:
    def __init__(self, simulate=False):
        self.simulate = simulate
        if self.simulate:
            # Use the simulator instead of the actual serial port
            self.spo = XYStageSimulator()
            self.spo.start()  # Start the simulator's thread
        else:
            self.spo = self.initialize_serial_port()

    def __del__(self):
        if self.simulate and self.spo.running:
            self.spo.stop()

    def initialize_serial_port(self):
        hostname = socket.gethostname()
        print('platform.system(): ', platform.system())
        print('hostname: ', hostname)

        try:
            spo = self.find_proscan_controller()
            if spo is None:
                raise serial.SerialException("ProScan controller not found.")
        except Exception as e:
            print("Error. An exception was raised by the call to serial.Serial().")
            print("  - Do you have two programs trying to access the serial port maybe?")
            print(f"Exception: {e}")
            sys.exit(1)

        return spo

   
    def find_proscan_controller(self):
        ports = serial.tools.list_ports.comports()
    
      
        for port in ports:
            if "USB" in port.device:  # Check if the device is a USB port
                print(f"Trying port {port.device}")
                
                try:
                    # Open the serial port with the correct settings
                    spo = serial.Serial(
                        port=port.device,
                        baudrate=9600,  # Default baud rate for ProScan III
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=2  # Increased timeout for device response
                    )
                    
                    # Send the "VE" command to verify communication
                    spo.write(b"VE\r")  # VE<CR> requests version information
                    
                    time.sleep(0.2)  # Allow some time for the device to respond
                    
                    # Read the response from the device
                    response = spo.readline().decode('ascii').strip()
                    
                    print(f"Received response: {response}")
                    
                    # Check if the response contains the expected firmware version string
                    if response.startswith("R"):
                        print(f"ProScan III controller found on {port.device}")
                        return spo  # Return the connected serial port object
                    
                    spo.close()  # Close the port if not the desired device
                except Exception as e:
                    print(f"Error communicating with port {port.device}: {e}")
                    if 'spo' in locals() and spo.is_open:
                        spo.close()  # Ensure the port is closed on error

        print("No ProScan III controller found.")
        return None

    def send_command(self, command):
        """Send a command to the ProScan III controller or simulator."""
        if not self.spo:
            print("Serial port not initialized.")
            return

        if self.simulate:
            response = self.spo.send_command(command)
            return response
        else:
            # Use actual serial port
            try:
                command = f"{command}\r\n".encode('ascii')
                print(f"Sending command: {command}")
                self.spo.write(command)
            except serial.SerialException as e:
                print(f"Error sending command: {e}")

    def get_current_position(self):
        """Query the stage for its current position with error handling."""
        response = self.send_command("P")
        if self.simulate:
            try:
                # Simulated response is already in string format
                print(f"Received response: {response}")

                # Clean and split the response to extract x, y, z
                values = response.split(",")
                if len(values) != 3:
                    raise ValueError(f"Unexpected response format: {response}")

                # Convert to floats and return the current position
                x, y, z = map(float, values)
                return x, y, z

            except (ValueError, UnicodeDecodeError) as e:
                print(f"Error parsing response: {e}")
                return None, None, None
        else:
            try:
                response = self.spo.readline().decode("ascii").strip()
                print(f"Received response: {response}")

                # Clean and split the response to extract x, y, z
                values = response.split(",")
                if len(values) != 3:
                    raise ValueError(f"Unexpected response format: {response}")

                # Convert to floats and return the current position
                x, y, z = map(lambda v: float(v.strip().replace('\r', '').strip('R')), values)
                return x, y, z

            except (ValueError, UnicodeDecodeError) as e:
                print(f"Error parsing response: {e}")
                return None, None, None

    def move_stage_at_velocity(self, vx, vy):
        """Move the stage at the specified velocity (vx, vy)."""
        command = f"VS,{vx},{vy}"
        self.send_command(command)

    def move_stage_to_position(self, x, y):
        """Move the stage to the specified position (x, y)."""
        command = f"PA,{x},{y}"
        self.send_command(command)

class ZPStageManager:
    # Class to send and receive data with the 3D printer board
    def __init__(self, simulate=False):
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.e_pos = 0.0
        self.x_cnt = 0.0
        self.y_cnt = 0.0
        self.z_cnt = 0.0

        self.verbose = False
        self.baudrate = 115200
        self.simulate = simulate
        self.printer_found = False  # Flag to indicate whether a 3D printer board was found
        self.COM = None

        if simulate:
            # Use the simulator instead of the actual serial port
            self.serial = ZPStageSimulator()
            self.serial.start()  # Start the simulator's thread
            self.setup()
        else:
            try:
                available_ports = self.get_available_com_ports()
                for port in available_ports:
                    if self.is_3d_printer(port):
                        self.COM = port
                        self.printer_found = True
                        print(f"3D printer board found on port {port}")
                        break
                if not self.printer_found:
                    print("No 3D printer boards found.")
                # Open serial connection to the 3D printer
                self.serial = serial.Serial(self.COM, baudrate=self.baudrate, timeout=1)
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.setup()
            except Exception as e:
                print("ZPStageManager __init__:", e)

    def __del__(self):
        if self.simulate:
            self.serial.stop()
        else:
            self.serial.close()

    def send_data(self, data):
        print(f"Sending data: {data}")
        data = data.encode("utf-8") + b"\n"
        self.serial.write(data)
        self.serial.flush()

    def receive_data(self):
        time.sleep(0.01)
        received_data = self.serial.read_all().decode().strip()
        return received_data

    def movecommand(self, axes, feedrate=None):
        # Filter out any axes with a distance of 0
        filtered_axes = {
            axis: distance for axis, distance in axes.items() if distance != 0
        }
        # Join the non-zero axes and distances into a single string
        axis_str = " ".join(
            f"{axis}{distance}" for axis, distance in filtered_axes.items()
        )

        # Send the move command with the feedrate if it is provided
        if feedrate is not None:
            self.send_data(f"G0 F{feedrate} {axis_str}")
        else:
            self.send_data(f"G0 {axis_str}")

    def get_position(self):
        # M114 get all position values
        self.send_data("M114")
        position_data = self.receive_data()
        self.extract_position_data(position_data)

        return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

    def extract_position_data(self, response):
        # Split the response into lines
        lines = response.split('\n')

        for line in lines:
            line = line.strip()
            # Skip empty lines or lines that contain only "ok"
            if not line or line == "ok":
                continue

            # Attempt to match the position data in this line
            match = re.search(r"X:([+-]?\d+\.\d+)\s+"
                                r"Y:([+-]?\d+\.\d+)\s+"
                                r"Z:([+-]?\d+\.\d+)\s+"
                                r"E:([+-]?\d+\.\d+)\s+"
                                r"Count\s+X:([+-]?\d+)\s+Y:([+-]?\d+)\s+Z:([+-]?\d+)", line
                            )
            if match:
                # Extract the position values from the matched line
                self.x_pos = float(match.group(1))
                self.y_pos = float(match.group(2))
                self.z_pos = float(match.group(3))
                self.e_pos = float(match.group(4))
                self.x_cnt = float(match.group(5))
                self.y_cnt = float(match.group(6))
                self.z_cnt = float(match.group(7))
                return  # Successfully extracted position data
            else:
                print(f"Failed to match line: {line}")

    def get_all_data(self):
        # M503 request all settings from the printer
        self.send_data("M503")
        all_data = self.receive_data()
        return all_data

    def request_data(self):
        # M114 get all position values
        self.send_data("M114")
        position_data = self.receive_data()
        position_values = self.extract_position_data(position_data)

        if position_values and self.verbose:
            x_pos, y_pos, z_pos, e_pos = position_values
            print(f"X Position: {x_pos}")
            print(f"Y Position: {y_pos}")
            print(f"Z Position: {z_pos}")
            print(f"E Position: {e_pos}")

    def resetprinter(self):
        self.send_data("M112")
        print("Printer reset")

    def setup(self):
        step_per_mm = 78040
        max_feedrate = 90 / 10 * 60  # mm/min

        # Ignore the cold extrude condition
        self.send_data("M302 S0")

        # Set the extruder to relative motion
        self.send_data("M83")

        # Set the X,Y,Z motion to relative
        self.send_data("G91")

        # This command sets the maximum feed rate for each of the extruder (E), X, Y, and Z axes.
        # These values determine the maximum speed at which the printer can move each axis.
        self.send_data("M203 E10000 X10000 Y10000 Z10000")

        # M92 set steps per unit E is extruder, X, Y, Z are the axes
        self.send_data("M92 X5069.00 Y5069.00 Z-5069.00 E5069.00")

    def change_max_feeds(self, X, Y, Z, E):
        command = f"M203 E{E} X{X} Y{Y} Z{Z}"
        self.send_data(command)

    def set_feedrate(self, value):
        # Set feedrate
        command = f"F{value} "
        self.send_data(command)

    def get_available_com_ports(self):
        try:
            ports = list(serial.tools.list_ports.comports())
            return [port.device for port in ports]
        except Exception as e:
            print("ZPStageManager.get_available_com_ports:", e)

    def is_3d_printer(self, port):
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(b"\nM115\n")  # Send M115 command to get firmware information
                response = ser.read_until(b"\n").decode("utf-8")

                if "FIRMWARE_NAME" in response:
                    return True
        except serial.SerialException as e:
            print("ZPStageManager.is_3d_printer:", e)

        return False

    def save_settings(self):
        self.send_data("M500")

class Stages:
    UPDATE_INTERVAL_XY = 1  # Time between updates in seconds (for XY stage)
    UPDATE_INTERVAL_ZP = 0.5  # Time between updates for ZP stage

    def __init__(self, waypoints, simulate=False, Kp=1.0, Ki=0.0, Kd=0.0):
        self.simulate = simulate
        self.xy_manager = XYStageManager(simulate=False)
        self.zp_manager = ZPStageManager(simulate=self.simulate)

        self.current_positions_zp = {'x': 0, 'y': 0, 'z': 0, 'e': 0}
        self.waypoints = waypoints

        # PID controller parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

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
        # Proportional term
        P_x = self.Kp * error_x
        P_y = self.Kp * error_y

        # Integral term
        self.error_sum_x += error_x * delta_time
        self.error_sum_y += error_y * delta_time
        I_x = self.Ki * self.error_sum_x
        I_y = self.Ki * self.error_sum_y

        # Derivative term
        D_x = self.Kd * (error_x - self.last_error_x) / delta_time if delta_time > 0 else 0
        D_y = self.Kd * (error_y - self.last_error_y) / delta_time if delta_time > 0 else 0

        # PID output
        vx = P_x + I_x + D_x
        vy = P_y + I_y + D_y

        # Update last error
        self.last_error_x = error_x
        self.last_error_y = error_y

        return vx, vy

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

    def xy_update_thread(self, start_time, interpolation_type, ideal_path_x, ideal_path_y, actual_path_x, actual_path_y):
        """Thread function to update XY stage independently."""
        x0, y0, _ = self.xy_manager.get_current_position()
        if x0 is None or y0 is None:
            print("Failed to retrieve initial XY position. Stopping XY updates.")
            return

        next_update_time_xy = start_time
        while not self.stop_flag:
            current_time = time.time()
            if current_time >= next_update_time_xy:
                elapsed_time_xy = current_time - start_time
                interpolated_values_xy = self.waypoints.interpolate_waypoints(
                    elapsed_time_xy, x0, y0, 0, 0, 0, 0, interpolation_type
                )

                if interpolated_values_xy is None:
                    # No more waypoints - stop thread
                    break

                target_x = interpolated_values_xy['x']
                target_y = interpolated_values_xy['y']

                current_x, current_y, _ = self.xy_manager.get_current_position()
                if current_x is None or current_y is None:
                    print("Failed to retrieve XY position. Stopping XY updates.")
                    break

                error_x = target_x - current_x
                error_y = target_y - current_y

                vx, vy = self.calculate_velocity_with_pid(error_x, error_y, self.UPDATE_INTERVAL_XY)
                velocity_magnitude = np.hypot(vx, vy)
                if velocity_magnitude > self.max_velocity:
                    scaling_factor = self.max_velocity / velocity_magnitude
                    vx *= scaling_factor
                    vy *= scaling_factor
                    self.error_sum_x = 0.0
                    self.error_sum_y = 0.0

                self.xy_manager.move_stage_at_velocity(vx, vy)
                ideal_path_x.append(target_x)
                ideal_path_y.append(target_y)
                actual_path_x.append(current_x)
                actual_path_y.append(current_y)

                next_update_time_xy += self.UPDATE_INTERVAL_XY
            time.sleep(0.001)

        # Stop stage movement at end
        self.xy_manager.move_stage_at_velocity(0, 0)
        print("XY updates complete.")

    def zp_update_thread(self, start_time, interpolation_type,
                         ideal_path_z, ideal_p1, ideal_p2, ideal_p3,
                         actual_path_z, actual_p1_list, actual_p2_list, actual_p3_list):
        """Thread function to update ZP stage independently."""
        z0, p10, p20, p30 = self.zp_manager.get_position()
        if z0 is None:
            z0, p10, p20, p30 = 0, 0, 0, 0
            print("Failed to get ZP initial position, defaulting to 0.")

        self.zp_manager.send_data("G90")  # Absolute positioning
        timefactor = -0.15 * self.UPDATE_INTERVAL_ZP

        next_update_time_zp = start_time

        while not self.stop_flag:
            current_time = time.time()
            if current_time >= next_update_time_zp:
                z, p1, p2, p3 = self.zp_manager.get_position()
                if z is None:  # If reading fails, attempt defaults or break
                    z, p1, p2, p3 = 0, 0, 0, 0

                actual_path_z.append(z)
                actual_p1_list.append(p1)
                actual_p2_list.append(p2)
                actual_p3_list.append(p3)

                elapsed_time_zp = current_time - start_time

                interpolated_values_zp = self.waypoints.interpolate_waypoints(
                    elapsed_time_zp, 0, 0, z0, p10, p20, p30, interpolation_type
                )

                if interpolated_values_zp is None:
                    # No more waypoints - stop thread
                    break

                target_z = interpolated_values_zp['z']
                target_p1 = interpolated_values_zp['p1']
                target_p2 = interpolated_values_zp['p2']
                target_p3 = interpolated_values_zp['p3']

                zp_axes = {
                    'X': target_z,
                    'Y': target_p1,
                    'Z': target_p2,
                    'E': target_p3
                }

                # Calculate a feedrate
                dist_z = abs(target_z - z)
                dist_p1 = abs(target_p1 - p1)
                dist_p2 = abs(target_p2 - p2)
                dist_p3 = abs(target_p3 - p3)
                max_distance = max(dist_z, dist_p1, dist_p2, dist_p3)
                feedrate = (max_distance / (self.UPDATE_INTERVAL_ZP + timefactor)) * 60 if max_distance > 0 else 100.0

                self.zp_manager.movecommand(zp_axes, feedrate=feedrate)

                # Collect ideal positions
                ideal_path_z.append(interpolated_values_zp['z'])
                ideal_p1.append(interpolated_values_zp['p1'])
                ideal_p2.append(interpolated_values_zp['p2'])
                ideal_p3.append(interpolated_values_zp['p3'])

                next_update_time_zp += self.UPDATE_INTERVAL_ZP
            time.sleep(0.001)

        self.zp_manager.send_data("G91")  # Back to relative
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


class Waypoint:
    def __init__(self, csv_file_path='waypoints.csv'):
        self.csv_file_path = csv_file_path
        self.waypoints = []
        self.import_waypoints_from_csv()

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

    def interpolate_waypoints(self, elapsed_time, x0=0, y0=0, z0=0, p10=0, p20=0, p30=0, interpolation_type="linear"):
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
        interpolated_values['p1'] += p10
        interpolated_values['p2'] += p20  # Corrected line
        interpolated_values['p3'] += p30  # Corrected line

        return interpolated_values


class XYStageSimulator:
    def __init__(self, update_rate_hz=100, acceleration_rate=100, communication_delay=0.0):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.last_update_time = time.time()

        self.acceleration_rate = acceleration_rate  # Max velocity change per second (microns/s^2)
        self.update_rate_hz = update_rate_hz  # Updates per second
        self.update_interval = 1.0 / update_rate_hz
        self.communication_delay = communication_delay  # Simulated delay in seconds

        self.running = False
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self.update_loop)
        self.thread.daemon = True

    def start(self):
        """Start the simulator thread."""
        self.running = True
        self.thread.start()

    def stop(self):
        """Stop the simulator thread."""
        self.running = False
        self.thread.join()

    def send_command(self, command):
        time.sleep(self.communication_delay)  # Simulated delay
        """Simulate sending a command to the stage controller."""
        print(f"Simulated command: {command.strip()}")
        if command.startswith("VS"):
            # Parse target velocity from command
            parts = command.split(',')
            if len(parts) == 3:
                try:
                    vx = float(parts[1])
                    vy = float(parts[2])
                    with self.lock:
                        self.target_vx = vx
                        self.target_vy = vy
                    return "R"
                except ValueError:
                    return "Invalid velocity values."
            return "Invalid command format."
        elif command.strip() == "P":
            # Simulate position query
            x, y, z = self.get_current_position()
            return f"{x:.2f},{y:.2f},{z:.2f}"
        return "Unknown command."

    def get_current_position(self):
        """Return the current position."""
        with self.lock:
            return self.current_x, self.current_y, 0.0

    def move_stage_at_velocity(self, vx, vy):
        """Move the stage at the specified velocity (vx, vy)."""
        command = f"VS,{vx},{vy}"
        self.send_command(command)

    def update_velocity(self, current, target, dt):
        """Gradually adjust velocity towards the target with a linear ramp."""
        if current < target:
            return min(current + self.acceleration_rate * dt, target)
        elif current > target:
            return max(current - self.acceleration_rate * dt, target)
        return current

    def update_loop(self):
        """Update position and velocity in a loop."""
        while self.running:
            start_time = time.time()
            with self.lock:
                # Compute elapsed time
                now = time.time()
                dt = now - self.last_update_time
                self.last_update_time = now

                # Gradually adjust velocity
                self.current_vx = self.update_velocity(self.current_vx, self.target_vx, dt)
                self.current_vy = self.update_velocity(self.current_vy, self.target_vy, dt)

                # Update position
                self.current_x += self.current_vx * dt
                self.current_y += self.current_vy * dt

            # Sleep to maintain the update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, self.update_interval - elapsed)
            time.sleep(sleep_time)

class ZPStageSimulator:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.process_commands)
        self.thread.daemon = True

        self.position = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        self.counts = {'X': 0, 'Y': 0, 'Z': 0}

        self.communication_delay = 0.03  # Simulate delay
        self.processing_time_per_command = 0.01  # Simulate time to process each command

        self.buffer = b''

    def start(self):
        """Start the simulator thread."""
        self.running = True
        self.thread.start()

    def stop(self):
        """Stop the simulator thread."""
        self.running = False
        self.thread.join()

    def write(self, data):
        """Simulate writing data to the serial port."""
        with self.lock:
            self.buffer += data

    def flush(self):
        """Simulate flushing the serial port."""
        time.sleep(self.communication_delay)  # Simulated communication delay
        with self.lock:
            lines = self.buffer.split(b'\n')
            self.buffer = lines[-1] if self.buffer[-1:] != b'\n' else b''
            for line in lines[:-1]:
                command = line.decode('utf-8').strip()
                self.command_queue.put(command)
            self.buffer = b''

    def read_all(self):
        """Simulate reading all data from the serial port."""
        responses = []
        while not self.response_queue.empty():
            responses.append(self.response_queue.get())
        return '\n'.join(responses).encode('utf-8')

    def close(self):
        self.stop()

    def process_commands(self):
        """Process commands from the queue."""
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                self.process_command(command)
                self.command_queue.task_done()
            except queue.Empty:
                continue

    def process_command(self, command):
        """Process a single G-code command."""
        print(f"Processing command: {command}")
        time.sleep(self.processing_time_per_command)  # Simulated processing time

        if command.startswith('G0'):
            # Movement command
            axes = re.findall(r'([XYZEF])([-\d\.]+)', command)
            for axis, value in axes:
                value = float(value)
                self.position[axis] += value
            response = 'ok'
        elif command.strip() == 'M114':
            # Get current position
            response = f"X:{self.position['X']} Y:{self.position['Y']} Z:{self.position['Z']} E:{self.position['E']} Count X:{self.counts['X']} Y:{self.counts['Y']} Z:{self.counts['Z']}"
        elif command.strip() == 'M503':
            # Return settings
            response = 'Settings: M203 E10000 X10000 Y10000 Z10000'
        elif command.strip() == 'M500':
            # Save settings
            response = 'Settings saved'
        elif command.startswith('M92'):
            # Set steps per unit
            response = 'Steps per unit set'
        elif command.startswith('M203'):
            # Set maximum feedrates
            response = 'Maximum feedrates set'
        elif command.strip() == 'M302 S0':
            # Allow cold extrusion
            response = 'Cold extrusion allowed'
        elif command.strip() == 'M83':
            # Set extruder to relative mode
            response = 'Extruder set to relative mode'
        elif command.strip() == 'G91':
            # Set positioning to relative
            response = 'Relative positioning enabled'
        elif command.strip() == 'M112':
            # Emergency stop
            response = 'Emergency stop activated'
        else:
            response = 'Unknown command'

        self.response_queue.put(response)

if __name__ == "__main__":
    # Choose whether to use the real stage or the simulator
    use_simulator = True

    # PID controller gains
    Kp = 0.5#0.33
    Ki = 0#0.001
    Kd = 0#0.02

    waypoints = Waypoint('multi_layer_toolpath.csv')
    waypoint_list = waypoints.import_waypoints_from_csv()

    stage = Stages(waypoints, simulate=use_simulator, Kp=Kp, Ki=Ki, Kd=Kd)

    if waypoint_list:
        stage.move("Waypoint Following with PID Control", interpolation_type="linear", plot=True)
    else:
        print("No waypoints available.")
