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
    UPDATE_INTERVAL = 0.3  # Time between updates (300 ms)

    def __init__(self):
        serial_manager = SerialPortManager()
        self.spo = serial_manager.spo

    def send_command(self, command):
        """Send a command to the ProScan III controller."""
        if not self.spo:
            print("Serial port not initialized.")
            return

        try:
            command = f"{command}\r\n".encode('ascii')
            self.spo.write(command)
        except serial.SerialException as e:
            print(f"Error sending command: {e}")

    def get_current_position(self):
        """Query the stage for its current position with error handling."""
        self.send_command("P")
        try:
            response = self.spo.readline().decode("ascii").strip()
            print(f"Received response: {response}")

            # Clean and split the response to extract x, y, z
            values = response.split(",")
            if len(values) != 3:
                raise ValueError(f"Unexpected response format: {response}")

            # Convert to integers and return the current position
            x, y, z = map(lambda v: float(v.strip().replace('\r', '').strip('R')), values)
            return x, y, z

        except (ValueError, UnicodeDecodeError) as e:
            print(f"Error parsing response: {e}")
            return None, None, None

    def move_stage_at_velocity(self, vx, vy):
        """Move the stage at the specified velocity (vx, vy)."""
        command = f"VS,{vx},{vy}\r\n"
        self.spo.write(command.encode())
        print(f"Sent velocity command: {command.strip()}")

    def move(self, waypoints, plot_title, interpolation_type="linear", plot=False):
        """Move the stage along interpolated waypoints and optionally plot the results."""
        x0, y0, _ = self.get_current_position()
        if x0 is None or y0 is None:
            print("Failed to retrieve initial position. Exiting.")
            return

        start_time = time.time()

        actual_path_x = []
        actual_path_y = []
        ideal_path_x = []
        ideal_path_y = []

        while True:
            elapsed_time = time.time() - start_time
            target_x, target_y = self.interpolate_waypoints(elapsed_time + self.UPDATE_INTERVAL, waypoints, x0, y0, interpolation_type)
            if target_x is None and target_y is None:
                break

            ideal_path_x.append(target_x)
            ideal_path_y.append(target_y)

            current_x, current_y, _ = self.get_current_position()
            if current_x is None or current_y is None:
                print("Skipping this step due to position retrieval failure.")
                time.sleep(self.UPDATE_INTERVAL)
                continue

            actual_path_x.append(current_x)
            actual_path_y.append(current_y)

            base_vx, base_vy = self.calculate_dynamic_velocity(target_x, target_y, current_x, current_y, elapsed_time)
            self.move_stage_at_velocity(base_vx, base_vy)

            time.sleep(self.UPDATE_INTERVAL)

        self.move_stage_at_velocity(0, 0)
        print(f"{plot_title} complete.")

        if plot:
            self.plot_results(ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, plot_title)

    @staticmethod
    def interpolate_waypoints(time, waypoints, x0, y0, interpolation_type="linear"):
        """Interpolate between waypoints to get the target position at the given elapsed time."""
        if time > waypoints[-1][2]:
            return None, None

        times = [waypoint[2] for waypoint in waypoints]
        xs = [waypoint[0] for waypoint in waypoints]
        ys = [waypoint[1] for waypoint in waypoints]

        if interpolation_type == "linear":
            interp_func_x = interp1d(times, xs, kind='linear', fill_value="extrapolate")
            interp_func_y = interp1d(times, ys, kind='linear', fill_value="extrapolate")
        elif interpolation_type == "polynomial":
            degree = min(3, len(waypoints) - 1)  # Choose degree based on number of waypoints
            interp_func_x = np.poly1d(np.polyfit(times, xs, degree))
            interp_func_y = np.poly1d(np.polyfit(times, ys, degree))
        elif interpolation_type == "spline":
            interp_func_x = CubicSpline(times, xs)
            interp_func_y = CubicSpline(times, ys)
        else:
            raise ValueError(f"Unsupported interpolation type: {interpolation_type}")

        target_x = interp_func_x(time) + x0
        target_y = interp_func_y(time) + y0
        return target_x, target_y

    @staticmethod
    def calculate_dynamic_velocity(target_x, target_y, current_x, current_y, elapsed_time):
        """Calculate the velocity for moving towards the target position dynamically."""
        # Measure the difference between current and target positions
        dx = target_x - current_x
        dy = target_y - current_y

        # Estimate the time needed for communication and movement
        # Adjust velocity based on the time required to reach the target in the desired timeframe
        distance = np.sqrt(dx**2 + dy**2)
        estimated_time = elapsed_time + XYStage.UPDATE_INTERVAL  # Considering time for updates and communication
        velocity_scale = distance / estimated_time if estimated_time > 0 else 1

        # Calculate velocity based on the distance and the time to reach target
        vx = dx * velocity_scale
        vy = dy * velocity_scale

        return vx, vy

    @staticmethod
    def import_waypoints_from_csv(file_path):
        """Import waypoints from a CSV file."""
        waypoints = []
        try:
            with open(file_path, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if len(row) == 3:
                        x, y, t = map(float, row)
                        waypoints.append((x, y, t))
        except FileNotFoundError:
            print(f"Error: File not found at {file_path}")
        except Exception as e:
            print(f"Error reading CSV file: {e}")
        return waypoints

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
                except ValueError:
                    print("Invalid velocity values.")
        elif command.strip() == "P":
            # Simulate position query
            return self.get_current_position()

    def get_current_position(self):
        """Simulate querying the stage for its current position."""
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update position based on velocity and elapsed time
        self.current_x += self.velocity_x * elapsed_time
        self.current_y += self.velocity_y * elapsed_time

        response = f"{self.current_x:.2f},{self.current_y:.2f},0.00"
        print(f"Simulated response: {response}")
        return self.current_x, self.current_y, 0.0


if __name__ == "__main__":
    # Choose whether to use the real stage or the simulator
    use_simulator = False

    if use_simulator:
        stage = XYStageSimulator()
    else:
        stage = XYStage()

    csv_file_path = 'waypoints.csv'
    waypoints = XYStage.import_waypoints_from_csv(csv_file_path)

    if waypoints:
        if use_simulator:
            # Manually simulate movement and plot results
            stage.last_update_time = time.time()
            ideal_path_x = []
            ideal_path_y = []
            actual_path_x = []
            actual_path_y = []
            start_time = time.time()

            while True:
                elapsed_time = time.time() - start_time
                target_x, target_y = XYStage.interpolate_waypoints(elapsed_time + XYStage.UPDATE_INTERVAL, waypoints, 0, 0)
                if target_x is None and target_y is None:
                    break

                ideal_path_x.append(target_x)
                ideal_path_y.append(target_y)

                current_x, current_y, _ = stage.get_current_position()
                actual_path_x.append(current_x)
                actual_path_y.append(current_y)

                base_vx, base_vy = XYStage.calculate_dynamic_velocity(target_x, target_y, current_x, current_y, elapsed_time)
                stage.send_command(f"VS,{base_vx},{base_vy}")
                time.sleep(XYStage.UPDATE_INTERVAL)

            # Plot results after simulation
            XYStage.plot_results(ideal_path_x, ideal_path_y, actual_path_x, actual_path_y, "Simulated Waypoint Following")
        else:
            stage.move(waypoints, "Waypoint Following", interpolation_type="spline", plot=True)
    else:
        print("No waypoints available.")
