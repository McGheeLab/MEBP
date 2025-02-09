import time
import sys
import platform
import socket
import serial
import serial.tools.list_ports
import threading
import re
import queue

############################### Communication Classes ########################################
# These classes manage the serial communication with the XY and ZP stages
class XYStageManager:
    def __init__(self, simulate=False):
        # Store the simulation flag to decide whether to use real hardware or a simulator
        self.simulate = simulate

        # If simulation is enabled, instantiate and start the XYStageSimulator
        if self.simulate:
            self.spo = XYStageSimulator()
            self.spo.start()  # Launch simulator thread
        else:
            # Otherwise, try to find and open a real serial port for the ProScan controller
            self.spo = self.initialize_serial_port()

    def __del__(self):
        # On object destruction, stop the simulator if it's running
        if self.simulate and self.spo.running:
            self.spo.stop()

    ####################### Serial Communication Functions ##################################
    def initialize_serial_port(self):
        # Display some system info for debugging
        hostname = socket.gethostname()
        print('platform.system(): ', platform.system())
        print('hostname: ', hostname)

        try:
            # Attempt to find and open the ProScan controller
            spo = self.find_proscan_controller()
            # If not found, raise an exception
            if spo is None:
                raise serial.SerialException("ProScan controller not found.")
        except Exception as e:
            # Handle possible errors related to serial port usage
            print("Error. An exception was raised by the call to serial.Serial().")
            print("  - Do you have two programs trying to access the serial port maybe?")
            print(f"Exception: {e}")
            sys.exit(1)

        # Return the initialized serial object if successful
        return spo

    def find_proscan_controller(self):
        # Check all available COM ports for a ProScan III controller
        ports = serial.tools.list_ports.comports()
        for port in ports:
            try:
                # Attempt to open the port at 115200 baud
                spo = serial.Serial(
                    port.device, baudrate=115200, bytesize=8,
                    timeout=1, stopbits=serial.STOPBITS_ONE
                )
                # Write a command ('V') to check for the correct device
                spo.write(b"V\r\n")
                # Read the response and strip extra whitespace
                response = spo.readline().decode('ascii').strip()

                # If response indicates a ProScan III controller, return this serial object
                if "R" in response:
                    print(f"ProScan III controller found on {port.device}")
                    return spo

                # Otherwise, close the port and continue checking
                spo.close()
            except (serial.SerialException, UnicodeDecodeError):
                # If there's an issue reading or decoding data, just move on to the next port
                continue

        # If no ProScan III controller is found, print a message
        print("No ProScan III controller found.")
        return None

    def send_command(self, command):
        """
        Send a command to the ProScan III controller or to the simulator.
        Accepts a string command like 'P' or 'VS,100,200'.
        """
        if not self.spo:
            # If the serial port or simulator isn't initialized, notify the user
            print("Serial port not initialized.")
            return

        # In simulation, calls the simulator's command handler
        if self.simulate:
            response = self.spo.send_command(command)
            return response
        else:
            # Otherwise, send the command over the actual serial port
            try:
                command = f"{command}\r\n".encode('ascii')
                # print(f"Sending command: {command}")
                self.spo.write(command)
            except serial.SerialException as e:
                # Handle serial port errors
                print(f"Error sending command: {e}")

    ####################### Stage Query Functions ##################################
    
    def get_current_position(self):
        """
        Query the stage for its current position.
        Returns a tuple (x, y, z) or (None, None, None) if parsing fails.
        """
        # Send the position query command 'P'
        response = self.send_command("P")

        # If running in simulation, parse the simulator's string response directly
        if self.simulate:
            try:
                # print(f"Received response: {response}")
                values = response.split(",")
                if len(values) != 3:
                    raise ValueError(f"Unexpected response format: {response}")
                # Convert to floats and return the current position
                x, y, z = map(float, values)
                return x, y, z
            except (ValueError, UnicodeDecodeError) as e:
                # On parsing error, notify and return None for each axis
                print(f"Error parsing response: {e}")
                return None, None, None

        else:
            # For real hardware, read the next line from the serial port
            try:
                # Read the next line from the serial port and decode it
                response = self.spo.readline().decode("ascii").strip()
                # print(f"Received response: {response}")
                values = response.split(",")
                if len(values) != 3:
                    raise ValueError(f"Unexpected response format: {response}")
                # Convert each value to float after removing any trailing 'R' or extra chars
                x, y, z = map(lambda v: float(v.strip().replace('\r', '').strip('R')), values)
                return x, y, z
            except (ValueError, UnicodeDecodeError) as e:
                # On error, print a message and return None for each axis
                print(f"Error parsing response: {e}")
                return None, None, None

    ####################### Stage Movement Functions ##################################
    
    def move_stage_at_velocity(self, vx, vy):
        """
        Move stage at a specified velocity.
        vx, vy are velocity components in the X and Y axes respectively.
        """
        command = f"VS,{vx},{vy}"
        self.send_command(command)

    def move_stage_to_position(self, x, y):
        """
        Move stage to a specific position given by (x, y).
        This uses an absolute positioning approach (PA command).
        """
        command = f"PA,{x},{y}"
        self.send_command(command)

class ZPStageManager:
    # This class manages communication with a 3D printer or a simulator for testing
    def __init__(self, simulate=False):
        # Store current position and count values
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.e_pos = 0.0
        self.x_cnt = 0.0
        self.y_cnt = 0.0
        self.z_cnt = 0.0

        # General settings for communication and state
        self.verbose = False
        self.baudrate = 115200
        self.simulate = simulate
        self.printer_found = False
        self.COM = None

        # If in simulate mode, use the ZPStageSimulator instead of real hardware
        if simulate:
            self.serial = ZPStageSimulator()
            self.serial.start()
            self.setup()
        else:
            try:
                # Try to find a real 3D printer on available COM ports
                available_ports = self.get_available_com_ports()
                for port in available_ports:
                    if self.is_3d_printer(port):
                        self.COM = port
                        self.printer_found = True
                        print(f"3D printer board found on port {port}")
                        break
                # If no printer is found, show a message
                if not self.printer_found:
                    print("No 3D printer boards found.")
                # Open the serial connection to the board
                self.serial = serial.Serial(self.COM, baudrate=self.baudrate, timeout=1)
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.setup()
            except Exception as e:
                print("ZPStageManager __init__:", e)

    def __del__(self):
        # Stop simulator thread or close hardware connection
        if self.simulate:
            self.serial.stop()
        else:
            self.serial.close()
            
    # Setup the printer for operation
    def setup(self):
        # Prepare printer for normal operation
        step_per_mm = 78040
        max_feedrate = 90 / 10 * 60  # mm/min, example calculation

        self.send_data("M302 S0")  # Allow cold extrusion
        self.send_data("M83")      # Set extruder to relative mode
        self.send_data("G91")      # Set XYZ to relative positioning
        self.send_data("M203 E10000 X10000 Y10000 Z10000")  # Set max feedrates
        self.send_data("M92 X5069.00 Y5069.00 Z-5069.00 E5069.00")  # Configure steps per unit

    ################################# Communication Functions ########################################
    
    def send_data(self, data):
        # Send G-code or commands to the printer or simulator
        # print(f"Sending data: {data}")
        data = data.encode("utf-8") + b"\n"  # Convert to bytes and add newline
        self.serial.write(data)
        self.serial.flush()  # Make sure the data is sent immediately

    def receive_data(self):
        # Short pause to simulate delay, then read all incoming data
        time.sleep(0.01)
        received_data = self.serial.read_all().decode().strip()
        return received_data

    def get_available_com_ports(self):
        # List available serial ports
        try:
            ports = list(serial.tools.list_ports.comports())
            return [port.device for port in ports]
        except Exception as e:
            print("ZPStageManager.get_available_com_ports:", e)

    def is_3d_printer(self, port):
        # Check if a serial port belongs to a 3D printer by asking for firmware info
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(b"\nM115\n")  # M115 asks for printer firmware name
                response = ser.read_until(b"\n").decode("utf-8")
                if "FIRMWARE_NAME" in response:
                    return True
        except serial.SerialException as e:
            print("ZPStageManager.is_3d_printer:", e)
        return False

    ################################# Printer Control Functions ########################################
    
    def movecommand(self, axes, feedrate=None):
        # Build a G0 command string for axes that have a non-zero distance
        filtered_axes = {
            axis: distance for axis, distance in axes.items() if distance != 0
        }
        axis_str = " ".join(f"{axis}{distance}" for axis, distance in filtered_axes.items())

        # If a feed rate is specified, include it. Otherwise just move.
        if feedrate is not None:
            self.send_data(f"G0 F{feedrate} {axis_str}")
            print(f"G0 F{feedrate} {axis_str}")
        else:
            self.send_data(f"G0 {axis_str}")
            print(f"G0 {axis_str}")

    def set_feedrate(self, value):
        # Change feedrate for subsequent moves
        command = f"F{value} "
        self.send_data(command)
        
    ################################# Printer Request Functions ########################################
        
    def get_current_position(self):
        # Send M114 to request current position from the printer or simulator
        self.send_data("M114")
        position_data = self.receive_data()
        self._extract_position_data(position_data)
        # Return the four position values as a tuple
        return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

    def _extract_position_data(self, response):
        # Parse the lines in the response to find the position values
        lines = response.split('\n')
        for line in lines:
            line = line.strip()
            # Skip empty lines or lines that just say "ok"
            if not line or line == "ok":
                continue
            # Try to match the position pattern (X, Y, Z, E, and counts)
            match = re.search(
                r"X:([+-]?\d+\.\d+)\s+"
                r"Y:([+-]?\d+\.\d+)\s+"
                r"Z:([+-]?\d+\.\d+)\s+"
                r"E:([+-]?\d+\.\d+)\s+"
                r"Count\s+X:([+-]?\d+)\s+Y:([+-]?\d+)\s+Z:([+-]?\d+)",
                line
            )
            if match:
                # Update position and count values based on the match
                self.x_pos = float(match.group(1))
                self.y_pos = float(match.group(2))
                self.z_pos = float(match.group(3))
                self.e_pos = float(match.group(4))
                self.x_cnt = float(match.group(5))
                self.y_cnt = float(match.group(6))
                self.z_cnt = float(match.group(7))
                return
            else:
                print(f"Failed to match line: {line}")

    ################################# Printer Settings Functions ########################################
    
    def resetprinter(self):
        # Send emergency stop command
        self.send_data("M112")
        print("Printer reset")

    def change_max_feeds(self, X, Y, Z, E):
        # Adjust maximum speeds on the fly
        command = f"M203 E{E} X{X} Y{Y} Z{Z}"
        self.send_data(command)

    def save_settings(self):
        # Save current configuration to printer memory
        self.send_data("M500")

############################### Communication Simulators ########################################
# Simulators for the XY and ZP stages just spoof the serial communication with the actual devices.
# All functions that control the stages are implemented in the the normal classes.
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
        # print(f"Simulated command: {command.strip()}")
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
        # print(f"Processing command: {command}")
        time.sleep(self.processing_time_per_command)  # Simulated processing time

        if command.startswith('G0'):
            # Movement command
            axes = re.findall(r'([XYZE])([-\d\.]+)', command)
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
