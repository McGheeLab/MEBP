import time
import math
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
import csv
import threading
import sys
import platform
import socket
import serial

class SerialPortManager:
    def __init__(self):
        self.spo = self.initialize_serial_port()

    def initialize_serial_port(self):
        hostname = socket.gethostname()
        print('platform.system(): ', platform.system())
        print('hostname: ', hostname)

        try:
            spo = self.find_proscan_controller()
        except Exception as e:
            print("Error. An exception was raised by the call to serial.Serial().")
            print("  - Do you have two programs trying to access the serial port maybe?")
            print(f"Exception: {e}")
            sys.exit(1)


        return spo
        import serial.tools.list_ports

    def find_proscan_controller():
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
    RADIUS = 1000  # 5 mm in microns
    DURATION = 30  # Total time for the circle (seconds)
    UPDATE_INTERVAL = 0.3  # Time between updates (300 ms)
    NUM_STEPS = int(DURATION / UPDATE_INTERVAL)  # Total updates
    Kp = 0.001  # Proportional gain for velocity correction (tune this value)
    MAX_CORRECTION = 50  # Max correction velocity in microns/sec

    def __init__(self):
        self.kf = self.initialize_kalman_filter()
        serial_manager = SerialPortManager()
        self.spo = serial_manager.spo

    def send_command(self, command):
        """Send a command to the ProScan III controller."""
        command = f"{command}\r\n".encode('ascii')
        self.spo.write(command)

    def get_current_position(self):
        """Query the stage for its current position with error handling."""
        self.send_command("P")
        try:
            response = spo.readline().decode("ascii").strip()
            print(f"Received response: {response}")

            values = response.split(",")
            if len(values) != 3:
                raise ValueError(f"Unexpected response format: {response}")

            x, y, z = map(lambda v: int(v.strip().replace('\r', '').strip('R')), values)
            return x, y, z

        except (ValueError, UnicodeDecodeError) as e:
            print(f"Error parsing response: {e}")
            return None, None, None

    def move_stage_at_velocity(self, vx, vy):
        """Move the stage at the specified velocity (vx, vy)."""
        command = f"VS,{vx},{vy}\r\n"
        spo.write(command.encode())
        print(f"Sent velocity command: {command.strip()}")

    def initialize_kalman_filter(self):
        """Initialize the Kalman filter for velocity prediction."""
        kf = KalmanFilter(dim_x=4, dim_z=2)
        kf.x = np.array([0., 0., 0., 0.])  # Initial state (x, y, vx, vy)
        kf.F = np.array([[1, 0, self.UPDATE_INTERVAL, 0],
                         [0, 1, 0, self.UPDATE_INTERVAL],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])  # State transition matrix
        kf.H = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])  # Measurement function
        kf.P *= 1000.  # Covariance matrix
        kf.R = 5  # Measurement noise
        kf.Q = np.eye(4) * 0.1  # Process noise
        return kf

    def calculate_velocity(self, base_vx, base_vy, current_x, current_y, target_x, target_y):
        """Calculate the adjusted velocity based on prediction and error correction."""
        z = np.array([current_x, current_y])  # Current position as measurement
        self.kf.predict()  # Predict next state
        self.kf.update(z)  # Update with current measurement
        predicted_vx = self.kf.x[2]
        predicted_vy = self.kf.x[3]

        combined_vx = 0.5 * (base_vx + predicted_vx)
        combined_vy = 0.5 * (base_vy + predicted_vy)

        error_x = target_x - current_x
        error_y = target_y - current_y

        correction_vx = self.Kp * error_x
        correction_vy = self.Kp * error_y

        correction_vx = max(min(correction_vx, self.MAX_CORRECTION), -self.MAX_CORRECTION)
        correction_vy = max(min(correction_vy, self.MAX_CORRECTION), -self.MAX_CORRECTION)

        adjusted_vx = combined_vx + correction_vx
        adjusted_vy = combined_vy + correction_vy

        return adjusted_vx, adjusted_vy

    def move_and_plot(self, trajectory_func, plot_title):
        """Move the stage along a given trajectory function and optionally plot the results."""
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
            target_x, target_y = trajectory_func(elapsed_time, x0, y0)
            if target_x is None and target_y is None:
                break

            ideal_path_x.append(target_x)
            ideal_path_y.append(target_y)

            current_x, current_y, _ = self.get_current_position()
            if current_x is None or current_y is None:
                print("Skipping this step due to position retrieval failure.")
                threading.Event().wait(self.UPDATE_INTERVAL)
                continue

            actual_path_x.append(current_x)
            actual_path_y.append(current_y)

            base_vx, base_vy = self.calculate_base_velocity(trajectory_func, elapsed_time, x0, y0)

            adjusted_vx, adjusted_vy = self.calculate_velocity(base_vx, base_vy, current_x, current_y, target_x, target_y)

            self.move_stage_at_velocity(adjusted_vx, adjusted_vy)

            threading.Event().wait(self.UPDATE_INTERVAL)

        self.move_stage_at_velocity(0, 0)
        print(f"{plot_title} complete.")

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

    @staticmethod
    def circular_trajectory(t, x0, y0):
        """Calculate the (x, y) coordinates for time `t` in a circular path."""
        if t > XYStage.DURATION:
            return None, None
        angle = (2 * math.pi * t) / XYStage.DURATION
        x = int(XYStage.RADIUS * math.cos(angle) + x0)
        y = int(XYStage.RADIUS * math.sin(angle) + y0)
        return x, y

    @staticmethod
    def waypoint_trajectory(elapsed_time, x0, y0, waypoints):
        """Calculate the target waypoint position at a given elapsed time."""
        for waypoint in waypoints:
            target_x, target_y, target_time = waypoint
            if elapsed_time <= target_time:
                return target_x + x0, target_y + y0
        return None, None

    @staticmethod
    def calculate_base_velocity(trajectory_func, elapsed_time, x0, y0):
        """Calculate the base velocity for the given trajectory."""
        if trajectory_func == XYStage.circular_trajectory:
            omega = (2 * math.pi) / XYStage.DURATION
            base_vx = -XYStage.RADIUS * omega * math.sin((2 * math.pi * elapsed_time) / XYStage.DURATION)
            base_vy = XYStage.RADIUS * omega * math.cos((2 * math.pi * elapsed_time) / XYStage.DURATION)
        else:
            base_vx = base_vy = 0  # Update as needed
        return base_vx, base_vy

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

   
if __name__ == "__main__":
    stage = XYStage()
    csv_file_path = 'waypoints.csv'
    waypoints = stage.import_waypoints_from_csv(csv_file_path)
    
    if waypoints:
        stage.move_and_plot(lambda t, x0, y0: XYStage.waypoint_trajectory(t, x0, y0, waypoints), "Waypoint Following")
    else:
        stage.move_and_plot(XYStage.circular_trajectory, "Circular Motion")
