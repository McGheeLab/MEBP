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

class SerialPortManager:
    #def __init__(self):
    #    self.spo = self.initialize_serial_port()

    def __init__(self):
        self.spo = serial.Serial(
      port='COM3', baudrate=9600, bytesize=8,
      timeout=1, stopbits=serial.STOPBITS_ONE
      )
    
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
            try:
                spo = serial.Serial(
                    port.device, baudrate=9600, bytesize=8,
                    timeout=1, stopbits=serial.STOPBITS_ONE
                )
                spo.write(b"V\r\n")  # Example command to check if it's the ProScan III controller
                response = spo.readline().decode('ascii').strip()
                if "ProScan" in response:  # Adjust this condition based on the expected response
                    print(f"ProScan III controller found on {port.device}")
                    return spo
                spo.close()
            except (serial.SerialException, UnicodeDecodeError):
                continue
        print("No ProScan III controller found.")
        return None


class XYStage:
    UPDATE_INTERVAL = 0.1  # Time between updates (300 ms)

    def __init__(self, waypoints, simulate=False, Kp=1.0, Ki=0.0, Kd=0.0):
        self.simulate = simulate
        if self.simulate:
            self.spo = XYStageSimulator()
        else:
            serial_manager = SerialPortManager()
            self.spo = serial_manager.spo

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

    def send_command(self, command):
        """Send a command to the ProScan III controller or simulator."""
        if not self.spo:
            print("Serial port not initialized.")
            return

        if self.simulate:
            # Use simulator
            response = self.spo.send_command(command)
            print(f"Sent command: {command.strip()} - Response: {response}")
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
        self.send_command("P")
        try:
            if self.simulate:
                x, y, z = self.spo.get_current_position()
                return x, y, z
            else:
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
        D_x = self.Kd * (error_x - self.last_error_x) / delta_time
        D_y = self.Kd * (error_y - self.last_error_y) / delta_time

        # PID output
        vx = P_x + I_x + D_x
        vy = P_y + I_y + D_y

        # Update last error
        self.last_error_x = error_x
        self.last_error_y = error_y

        return vx, vy

    def move(self, plot_title, interpolation_type="linear", plot=False):
        # Temporary variables for initial position; replace later
        z0 = 0
        p10 = 0
        p20 = 0
        p30 = 0

        x0, y0, _ = self.get_current_position()
        if x0 is None or y0 is None:
            print("Failed to retrieve initial position. Exiting.")
            return

        start_time = time.time()

        actual_path_x = []
        actual_path_y = []
        ideal_path_x = []
        ideal_path_y = []

        delta_t = self.UPDATE_INTERVAL  # Time interval
        max_velocity = 1000  # Define maximum allowable velocity

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            # Desired target position at current time
            interpolated_values = self.waypoints.interpolate_waypoints(
                elapsed_time, x0, y0, z0, p10, p20, p30, interpolation_type
            )

            if interpolated_values is None:
                break

            target_x = interpolated_values['x']
            target_y = interpolated_values['y']

            # Get current position
            current_x, current_y, _ = self.get_current_position()
            print(f"Current position: ({current_x}, {current_y})")
            if current_x is None or current_y is None:
                print("Skipping this step due to position retrieval failure.")
                time.sleep(self.UPDATE_INTERVAL)
                continue

            # Calculate error between desired and actual position
            error_x = target_x - current_x
            error_y = target_y - current_y

            # Calculate velocity using PID controller
            vx, vy = self.calculate_velocity_with_pid(error_x, error_y, delta_t)

            # Limit velocities to maximum allowed values
            velocity_magnitude = np.hypot(vx, vy)
            if velocity_magnitude > max_velocity:
                scaling_factor = max_velocity / velocity_magnitude
                vx *= scaling_factor
                vy *= scaling_factor

                # Anti-windup: Reset integral sum if velocity is at maximum limit
                self.error_sum_x = 0.0
                self.error_sum_y = 0.0

            # Move stage
            self.move_stage_at_velocity(vx, vy)

            # Collect data for plotting
            ideal_path_x.append(target_x)
            ideal_path_y.append(target_y)
            actual_path_x.append(current_x)
            actual_path_y.append(current_y)

            # Maintain consistent control loop timing
            next_time = start_time + (len(ideal_path_x)) * self.UPDATE_INTERVAL
            sleep_duration = max(0, next_time - time.time())
            time.sleep(sleep_duration)

        # Stop the stage after movement
        self.move_stage_at_velocity(0, 0)
        print(f"{plot_title} complete.")

        if plot:
            self.plot_results(ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, plot_title)

    @staticmethod
    def calculate_velocity_to_follow_path(current_x, current_y, future_target_x, future_target_y, delta_t, max_velocity):
        """Calculate the velocity needed to move from current position to future target position over delta_t."""
        dx = future_target_x - current_x
        dy = future_target_y - current_y

        # Compute required velocities
        required_vx = dx / delta_t
        required_vy = dy / delta_t

        # Limit velocities to maximum allowed values
        velocity_magnitude = np.hypot(required_vx, required_vy)
        if velocity_magnitude > max_velocity:
            scaling_factor = max_velocity / velocity_magnitude
            required_vx *= scaling_factor
            required_vy *= scaling_factor

        return required_vx, required_vy

    @staticmethod
    def plot_results(ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, plot_title):
        """Plot the results of the stage movement."""
        plt.figure(figsize=(10, 8))
        plt.plot(ideal_path_x, ideal_path_y, label='Ideal Path', linestyle='--', color='blue')
        plt.plot(actual_path_x, actual_path_y, label='Actual Path', linestyle='-', color='red')
        plt.xlabel('X Position (microns)')
        plt.ylabel('Y Position (microns)')
        plt.title(f'Stage Movement: {plot_title}')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.show()

class XYStageSimulator:
    def __init__(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_update_time = time.time()

    def send_command(self, command):
        """Simulate sending a command to the ProScan III controller."""
        print(f"Simulated command: {command.strip()}")
        if command.startswith("VS"):
            # Extract velocity from command
            parts = command.split(',')
            if len(parts) == 3:
                try:
                    self.velocity_x = float(parts[1])
                    self.velocity_y = float(parts[2])
                    response = "R"
                except ValueError:
                    response = "Invalid velocity values."
            else:
                response = "Invalid command format."
        elif command.strip() == "P":
            # Simulate position query
            x, y, z = self.get_current_position()
            response = f"{x:.2f},{y:.2f},{z:.2f}"
        else:
            response = "Unknown command."
        return response

    def get_current_position(self):
        """Simulate querying the stage for its current position."""
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update position based on velocity and elapsed time
        self.current_x += self.velocity_x * elapsed_time
        self.current_y += self.velocity_y * elapsed_time

        # For simulation purposes, z is always 0.0
        return self.current_x, self.current_y, 0.0

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

    
    
# Class to send and recieve data with the 3D printer board
class ZPStage_manager:
    def __init__(self):
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.e_pos = 0.0
        self.x_cnt = 0.0
        self.y_cnt = 0.0
        self.z_cnt = 0.0
        self.bed_temp = None
        self.set_bed_temp = None
        self.hotend_temp = None
        self.set_hotend_temp = None

        self.verbose = False

        self.printer_found = (
            False  # Flag to indicate whether a 3D printer board was found
        )
        self.COM = None
        self.baudrate = 115200
        try:
            available_ports = self.get_available_com_ports()

            for port in available_ports:
                if self.is_3d_printer(port):
                    self.COM = port
                    self.printer_found = True
                    print(f"3D printer board found on port {port}")
                    break
                else:
                    pass

            if not self.printer_found:
                print("No 3D printer boards found.")

            # Open serial connection to BTT SKR MINI E3 V2.0
            self.serial = serial.Serial(self.COM, baudrate=self.baudrate, timeout=1)

            # Reset the input and output buffers to ensure clean communication
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # run the printer board initiil commands to our presets
            self.setup()

        except Exception as e:
            print("StepperBoard __init__:", e)

    def __del__(self):
        self.serial.close()

    def send_data(self, data):
        data = data.encode("utf-8") + b"\n"
        self.serial.write(data)
        self.serial.flush()

    def receive_data(self):
        time.sleep(0.03)
        received_data = self.serial.read_all().decode().strip()
        return received_data

    def save_settings(self):
        self.send_data("M500")

    # Function to extract position data from the response string
    def extract_position_data(self, response):
        # Use a regular expression to match the position values
        match = re.search(
            r"X:(\d+\.\d+)\s+Y:(\d+\.\d+)\s+Z:(\d+\.\d+)\s+E:(\d+\.\d+)\s+Count\s+X:(\d+)\s+Y:(\d+)\s+Z:(\d+)",
            response,
        )

        if match:
            # Extract the position values from the match object
            self.x_pos = float(match.group(1))
            self.y_pos = float(match.group(2))
            self.z_pos = float(match.group(3))
            self.e_pos = float(match.group(4))
            self.x_cnt = float(match.group(5))
            self.y_cnt = float(match.group(6))
            self.z_cnt = float(match.group(7))
            # Return a tuple of the position values
            return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

            # If no match was found, return None
        return None

    def get_position(self):
        # M114 get all position values
        self.send_data("M114")
        position_data = self.receive_data()
        position_values = self.extract_position_data(position_data)

        return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

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

    def get_available_com_ports(self):
        try:
            ports = list(serial.tools.list_ports.comports())
            return [port.device for port in ports]
        except Exception as e:
            print("StepperBoard.get_available_com_ports:", e)

    def resetprinter(self):
        self.send_data("M112")
        print("Printer reset")

    def setup(self):

        step_per_mm = 78040
        max_feedrate = 90/10 *60 # mm/min
        
        # ignore the cold extrude condition
        self.send_data("M302 S0")

        # set the extruder to relative motion
        self.send_data("M83")

        # set the X,Y,Z motion to relative
        self.send_data("G91")

        # This command sets the maximum feed rate for each of the extruder (E), X, Y, and Z axes.
        # These values determine the maximum speed at which the printer can move each axis.
        self.send_data("M203 E10000 X10000 Y10000 Z10000")

        # M92 set steps per unit E is extruder, X, Y, Z are the axis
        self.send_data("M92 X5069.00 Y5069.00 Z-5069.00 E5069.00")
        

    def change_max_feeds(self, X, Y, Z, E):
        command = f"M203 E{E} X{X} Y{Y} Z{Z}"
        self.send_data(command)

    def set_feedrate(self, value):
        # set feedrate
        command = f"F{value} "
        self.send_data(command)

    def is_3d_printer(self, port):
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(b"\nM115\n")  # Send M115 command to get firmware information
                response = ser.read_until(b"\n").decode("utf-8")

                if "FIRMWARE_NAME" in response:
                    return True
        except serial.SerialException as e:
            print("StepperBoard.is_3d_printer:", e)

        return False

    def jog(self, axes, feedrate=None):
        filtered_axes = {
            axis: distance for axis, distance in axes.items() if distance != 0
        }
        axis_str = " ".join(
            f"{axis}{distance}" for axis, distance in filtered_axes.items()
        )

        if feedrate is not None:
            self.send_data(f"G0 F{feedrate} {axis_str}")
        else:
            self.send_data(f"G0 {axis_str}")

class ZPStage:
    UPDATE_INTERVAL = 0.1  # Time between updates (100 ms)

    def __init__(self, interpolation_type="linear"):
        self.waypoints = Waypoint()
        self.zp_manager = ZPStage_manager()
        self.current_positions = {'x': 0, 'y': 0, 'z': 0, 'e': 0}
        self.interpolation_type = interpolation_type

    def move(self):
        # Get initial positions
        x0, y0, z0, e0 = self.zp_manager.get_position()
        if x0 is None or y0 is None or z0 is None or e0 is None:
            print("Failed to retrieve initial position. Exiting.")
            return

        self.current_positions['x'] = x0
        self.current_positions['y'] = y0
        self.current_positions['z'] = z0
        self.current_positions['e'] = e0

        start_time = time.time()
        next_elapsed_time = 0

        # Pre-fill the command queue with two commands
        for _ in range(2):
            next_elapsed_time += self.UPDATE_INTERVAL
            interpolated_values = self.waypoints.interpolate_waypoints(
                next_elapsed_time,
                x0, y0, z0, e0,
                self.interpolation_type
            )
            if interpolated_values is None:
                break

            self.queue_move(interpolated_values)

        # Movement loop
        while True:
            # Wait for update interval
            time.sleep(self.UPDATE_INTERVAL)

            # Each iteration, send the next move command
            next_elapsed_time += self.UPDATE_INTERVAL
            interpolated_values = self.waypoints.interpolate_waypoints(
                next_elapsed_time,
                x0, y0, z0, e0,
                self.interpolation_type
            )
            if interpolated_values is None:
                break

            self.queue_move(interpolated_values)

        # Movement complete
        print("Movement complete.")

    def queue_move(self, interpolated_values):
        # Calculate relative movements
        dx = interpolated_values['x'] - self.current_positions['x']
        dy = interpolated_values['y'] - self.current_positions['y']
        dz = interpolated_values['z'] - self.current_positions['z']
        de = interpolated_values['e'] - self.current_positions['e']
        axes = {'X': dx, 'Y': dy, 'Z': dz, 'E': de}

        # Send the jog command
        self.zp_manager.jog(axes)

        # Update current positions
        self.current_positions['x'] = interpolated_values['x']
        self.current_positions['y'] = interpolated_values['y']
        self.current_positions['z'] = interpolated_values['z']
        self.current_positions['e'] = interpolated_values['e']



if __name__ == "__main__":
    # Choose whether to use the real stage or the simulator
    use_simulator = False

    # PID controller gains
    Kp = 0.4
    Ki = 0
    Kd = 0.01

    waypoints = Waypoint('waypoints.csv')
    waypoint_list = waypoints.import_waypoints_from_csv()
    
    stage = XYStage(waypoints,simulate=use_simulator, Kp=Kp, Ki=Ki, Kd=Kd)
   
    if waypoint_list:
        stage.move( "Waypoint Following with PID Control", interpolation_type="linear", plot=True)
    else:
        print("No waypoints available.")