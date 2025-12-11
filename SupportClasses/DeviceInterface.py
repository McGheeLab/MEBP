import time
import sys
import platform
import socket
import serial
import serial.tools.list_ports
import threading
import re
import queue
import json
import numpy as np
import pandas as pd
from dataclasses import dataclass, asdict

############################### Communication Classes ########################################
# These classes manage the serial communication with the XY and ZP stages
class XYStageManager:
    """currently this class is built to support PriorII XY stage
        Common commands for the priorII stage are:
        V - query firmware version
        Z - sets the home position as 0,0,0
        P - query current position
        PA,x,y - move to absolute position x,y
        VS,x,y - move at velocity x,y
        AC,x - set acceleration to x
        VE,x - set velocity to x
     """
    def __init__(self, simulate=False):
        # Store the simulation flag to decide whether to use real hardware or a simulator
        self.simulate = simulate
        
        self.maxSpeed = 100
        self.minJerk = 1
        self.maxJerk = 100
        self.minAcceleration = 1
        self.maxAcceleration = 100
        self.xRange = [-100000, 100000]
        self.yRange = [-100000, 100000]
        self.defaultAcceleration = 1000
        self.defaultVelocity = 50
       
        # If simulation is enabled, instantiate and start the XYStageSimulator
        if self.simulate:
            self.spo = XYStageSimulator()
            self.spo.start()  # Launch simulator thread
        else:
            # Otherwise, try to find and open a real serial port for the ProScan controller
            self.spo = self.initialize_serial_port()

    def __del__(self):
        # On object destruction, stop the simulator if it's running
        if self.simulate:
            self.spo.stop()
        else:
            self.spo.close()

    def stop(self): 
        if self.simulate:
            self.spo.stop()
        else:
            self.spo.close()
    
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
        
        # List of baud rates to try (as recommended by ProScan documentation)
        baud_rates_to_try = [38400]
        
        for port in ports:
            for baud_rate in baud_rates_to_try:
                try:
                    print(f"Trying {port.device} at {baud_rate} baud...")
                    # Attempt to open the port at current baud rate
                    spo = serial.Serial(
                        port.device, baudrate=baud_rate, bytesize=8,
                        timeout=1, stopbits=serial.STOPBITS_ONE
                    )

                    spo.write(b"STAGE\r\n")  # Wake up the controller
                    time.sleep(0.1) # Wait for the response to be ready
                    response = spo.readline().decode('ascii').strip()
                    print(f"Initial response from {port.device} at {baud_rate}: {response}")
                    spo.reset_input_buffer()  # Clear any stale data
                    spo.reset_output_buffer() # Clear any stale data

                    # Write a command ('V') to check for the correct device
                    spo.write(b"V\r\n")
                    time.sleep(0.1) # Wait for the response to be ready
                    # Read the response and strip extra whitespace
                    response = spo.readline().decode('ascii').strip()
                    print(f"Response from {port.device} at {baud_rate}: {response}")
                    
                    # If response indicates a ProScan III controller, return this serial object
                    if "E" in response or "R" in response or "ProScan" in response:
                        print(f"ProScan III controller found on {port.device} at {baud_rate} baud")
                        return spo

                    # Otherwise, close the port and try next baud rate
                    spo.close()
                    
                except (serial.SerialException, UnicodeDecodeError):
                    # If there's an issue reading or decoding data, just move on
                    print(f"Error reading from port {port.device} at {baud_rate} baud.")
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
            
    def set_home(self): 
        """Set the current position as home/reference position for the stage"""
        response = self.send_command("Z")
        if response:
            print(f"HOME command sent, response: {response}")
        else:
            print("HOME command sent")

    ####################### Stage Movement Functions ##################################
    
    def move_stage_at_velocity(self, vx, vy, timefactor=1):
        """
        Move stage at a specified velocity.
        vx, vy are velocity components in the X and Y axes respectively.
        """

        command = f"VS,{vx},{vy}"
        self.send_command(command)


    def move_stage_to_position(self, x, y, fast=False):
        """
        Move stage to absolute position (x, y) using ProScan III ASCII RS-232 protocol.
        """

        # 2. (Optional) if fast=True, you might adjust speed or step size here
        #    e.g. self.send_command("SS 1\r")   # set stage scale to micro-steps
        # self.send_command("VS,100\r") # set stage speed to 100%
        #    – see “scale stage” (SS) and virtual joystick speed (VS) in §5.1.

        # 3. Build and send the absolute move command:
        #    'G x,y<CR>' moves to absolute (x,y)  :contentReference[oaicite:2]{index=2}
        cmd = f"G {int(x)},{int(y)}\r"
        self.send_command(cmd)
        print(f"Sent absolute move command: {cmd!r}")

        #wait for completion
        #read until an 'R' or 'END' is returned.
        
        #    e.g.: self.wait_for_response("R")

    def move_stage_relative(self, dx, dy, fast=False):
        """
        Move stage by a relative offset (dx, dy) using ProScan III ASCII RS-232 protocol.
        """
        # 2. (Optional) if fast=True, you might adjust speed or step size here
        #    e.g. self.send_command("SS 1\r")   # set stage scale to micro-steps
        # self.send_command("VS,100\r") # set stage speed to 100%
        #    – see “scale stage” (SS) and virtual joystick speed (VS) in §5.1.

        # 3. Build and send the relative move command:
        #    'G x,y<CR>' moves to absolute (x,y)  :contentReference[oaicite:2]{index=2}
        cmd = f"GR {int(dx)},{int(dy)}\r"
        self.send_command(cmd)
        print(f"Sent relative move command: {cmd!r}")

        #wait for completion
        #read until an 'R' or 'END' is returned.
        
        #    e.g.: self.wait_for_response("R")
        
    
    def check_stage_limits(self, x, y):
        """
        Check if the given position (x, y) or the resulting velocity (x,y) times 1 second will be within the valid range.
        Returns True if the position is valid, False otherwise.
        """
        # Check if the resulting position is within the valid range
        if x < self.xRange[0] or x > self.xRange[1] or y < self.yRange[0] or y > self.yRange[1]:
            return False
        return True
    
    def set_fast_mode(self):
        """
        Set the stage to move at a fast velocity.
        """
        self.set_velocity(self.maxSpeed)
    
    def set_slow_mode(self):
        """
        Set the stage to move at a slow velocity.
        """
        self.set_velocity(self.defaultVelocity)
    
    ####################### Stage Settings Functions ##################################
    def load_stage_settings(self):
        """
        Load the stage settings from a local .json file.
        """
        try:
            with open('stage_settings.json', 'r') as f:
                settings = json.load(f)
                stage_settings = settings.get("PriorIII_StageSettings", {})
                self.maxSpeed = stage_settings.get("maxSpeed")
                self.minJerk = stage_settings.get("minJerk")
                self.maxJerk = stage_settings.get("maxJerk")
                self.minAcceleration = stage_settings.get("minAcceleration")
                self.maxAcceleration = stage_settings.get("maxAcceleration")
                self.xRange = stage_settings.get("xRange")
                self.yRange = stage_settings.get("yRange")
                self.defaultAcceleration = stage_settings.get("defaultAcceleration")
                self.defaultVelocity = stage_settings.get("defaultVelocity")
                print("Stage settings loaded successfully.")
        except Exception as e:
            print(f"Error loading stage settings: {e}")
    
    def set_jerk(self, jerk):
        """
        Set the stage jerk to the specified value.
        """
        # ensure the jerk value is within the valid range
        if jerk > self.maxJerk or jerk < self.minJerk:
            print("Error: Jerk value is out of bounds.")
            return
        jerk = int(jerk)
        command = f"SCS,{jerk}"
        self.send_command(command)

    def set_acceleration(self, acceleration):
        """
        Set the stage acceleration to the specified value.
        """
        # ensure the acceleration value is within the valid range
        if acceleration > self.maxAcceleration or acceleration < self.minAcceleration:
            print("Error: Acceleration value is out of bounds.")
            return
        acceleration = int(acceleration)
        command = f"SAS,{acceleration}"
        self.send_command(command)

    def set_velocity(self, velocity):
        """
        Set the stage velocity to the specified value.
        """
        # ensure the velocity value is within the valid range
        if velocity > self.maxSpeed or velocity < 0:
            print("Error: Velocity value is out of bounds.")
            return
        velocity = int(velocity)
        command = f"SMS,{velocity}"
        self.send_command(command)
    


    def set_baudrate(self, baudrate):
        """
        Set the stage communication baudrate to the specified value.
        BAUD command: Sets the baud rate of the port issuing the command to the value
        specified by b. As a protection measure, if no command is sent to
        the port while the controller is switched on, the baud rate will
        revert to 9600 after switching off and back on again twice.
        Allowable values for baud rate are 9600 (argument 96), 19200
        (argument 19) and 38400 (argument 38)
        """
        # ProScan III baud rate mapping: actual rate -> command argument
        baudrate_mapping = {9600: 96, 19200: 19, 38400: 38}

        if baudrate not in baudrate_mapping:
            print(f"Error: Invalid baudrate. Choose from {list(baudrate_mapping.keys())}.")
            return False

        # Send the BAUD command with the mapped argument
        command = f"BAUD {baudrate_mapping[baudrate]}"
        
        if self.simulate:
            # In simulation, just return success
            response = self.send_command(command)
            print(f"Simulated: Baudrate command sent: {command}")
            return True
        else:
            try:
                # Send the command
                self.send_command(command)
                time.sleep(0.1)  # Wait for the command to be processed
                
                # Read the response - should be "0" for success
                response = self.spo.readline().decode('ascii').strip()
                print(f"BAUD command response: {response}")
                
                if response == "0":
                    # Success - now change the serial port baud rate
                    print(f"Baudrate successfully changed to {baudrate}")
                    self.spo.baudrate = baudrate
                    
                    # Clear buffers after baud rate change
                    self.spo.reset_input_buffer()
                    self.spo.reset_output_buffer()
                    
                    # Test communication at new baud rate
                    self.spo.write(b"V\r\n")
                    time.sleep(0.1)
                    test_response = self.spo.readline().decode('ascii').strip()
                    if test_response:
                        print(f"Communication test successful at {baudrate} baud: {test_response}")
                        return True
                    else:
                        print("Warning: No response received after baud rate change")
                        return False
                else:
                    print(f"Error: BAUD command failed with response: {response}")
                    return False
                    
            except Exception as e:
                print(f"Error executing BAUD command: {e}")
                return False

    def test_baud_rates(self):
        """
        Test communication at different baud rates to verify BAUD command functionality.
        This is useful for troubleshooting communication issues.
        """
        if self.simulate:
            print("Testing BAUD command in simulation mode...")
            # Test all supported baud rates in simulation
            for baud_rate in [9600, 19200, 38400]:
                result = self.set_baudrate(baud_rate)
                print(f"BAUD {baud_rate}: {'Success' if result else 'Failed'}")
            return True
        
        print("Testing BAUD command with real hardware...")
        print("Current baud rate:", self.spo.baudrate if self.spo else "Unknown")
        
        # Test setting to different baud rates and back
        original_baud = self.spo.baudrate if self.spo else 9600
        test_rates = [9600, 19200, 38400]
        
        for baud_rate in test_rates:
            if baud_rate != original_baud:
                print(f"\nTesting baud rate change to {baud_rate}...")
                if self.set_baudrate(baud_rate):
                    # Test communication at new baud rate
                    pos = self.get_current_position()
                    if pos[0] is not None:
                        print(f"Communication successful at {baud_rate} baud")
                        print(f"Current position: X={pos[0]}, Y={pos[1]}, Z={pos[2]}")
                    else:
                        print(f"Communication failed at {baud_rate} baud")
                else:
                    print(f"Failed to change to {baud_rate} baud")
        
        # Return to original baud rate
        print(f"\nReturning to original baud rate {original_baud}...")
        return self.set_baudrate(original_baud)
            
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
        self.feedrate = 200  # Default feedrate in mm/min

        # General settings for communication and state
        self.verbose = False
        self.baudrate = 38400
        self.simulate = simulate
        self.printer_found = False
        self.COM = None

        # If in simulate mode, use the ZPStageSimulator instead of real hardware
        if self.simulate:
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
    
    def stop(self):
        # Stop the simulator thread or close the hardware connection
        if self.simulate:
            self.serial.stop()
        else:
            self.serial.close()
     
    def set_max_feedrate(self, feedrate):
        self.feedrate = feedrate
        self.send_data(f"M203 E{feedrate} Y{feedrate} X{feedrate} Z{feedrate}")
        print(f"Set max feedrate to {feedrate} mm/min for all axes")

    # Setup the printer for operation
    def setup(self):
        # Prepare printer for normal operation
        step_per_mm = 78040
        # The original feedrate is 200 mm/min, but we use self.feedrate
        self.send_data("M302 S0")  # Allow cold extrusion
        self.send_data("M83")      # Set extruder to relative mode
        self.send_data("G91")      # Set XYZ to relative positioning
        self.send_data(f"M203 E{self.feedrate} Y{self.feedrate} X{self.feedrate} Z{self.feedrate}")  # Set max feedrates
        self.send_data("M92 X5069.00 Y5069.00 Z-5069.00 E5069.00")  # Configure steps per unit
        # set feedrate to max
        self.send_data(f"G0 F{self.feedrate}")  # Set feedrate to max
        self.send_data("M220 S100")  # Set feedrate to 100%

    # (Removed duplicate set_max_feedrate)

    ################################# Communication Functions ########################################
    
    def send_data(self, data):
        # Send G-code or commands to the printer or simulator
        # Convert the command to bytes and append a newline
        data = data.encode("utf-8") + b"\n"
        # Try to wait briefly until the port is open
        retry_count = 5
        wait_time = 0.01  # 100 ms
        tries = 0
        while (not self.serial or (hasattr(self.serial, "is_open") and not self.serial.is_open)) and tries < retry_count:
            print("Warning: Serial port not open. Waiting...")
            time.sleep(wait_time)
            tries += 1

        if not self.serial or (hasattr(self.serial, "is_open") and not self.serial.is_open):
            print("Error: Serial port is still not open. Command not sent.")
            return

        try:
            print(f"Sending data: {data.decode().strip()}")
            self.serial.write(data)
            self.serial.flush()  # Ensure the data is sent immediately
        except serial.SerialException as e:
            print(f"Error sending data: {e}")

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
    
    def move_relative(self, axes, feedrate=None):
        
        # Build a G0 command string for axes that have a non-zero distance
        filtered_axes = {
            axis: distance for axis, distance in axes.items() if distance != 0
        }
        axis_str = " ".join(f"{axis}{distance}" for axis, distance in filtered_axes.items())

        # If a feed rate is specified, include it. Otherwise just move.
        if feedrate is None:
            feedrate = self.feedrate
        # ...existing code...
        self.send_data(f"G0 F{feedrate} {axis_str}")
        print(f"G0 F{feedrate} {axis_str}")
    
    def move_absolute(self, axes, fast=False):
        
        # Build a G0 command string for axes to move to a prescribed location
        filtered_axes = {
            axis: position for axis, position in axes.items() if position != 0
        }
        axis_str = " ".join(f"{axis}{position}" for axis, position in filtered_axes.items())
        
        # change to absolute positioning mode
        self.set_absolute_mode()
        
        if fast:
            self.set_fast_mode()
        else:
            self.set_slow_mode()
                
        # Move to an absolute position
        self.send_data(f"G0 {axis_str}")
        
        # change back to relative positioning mode by defult
        self.set_relative_mode()
        # change back to slow feedrate by default
        self.set_slow_mode()
    
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
    def set_absolute_mode(self):
        # Set the printer to absolute positioning mode
        self.send_data("G90")
        print("Absolute positioning enabled")
    
    def set_relative_mode(self):
        # Set the printer to relative positioning mode
        self.send_data("G91")
        print("Relative positioning enabled")
    
    def set_fast_mode(self):
        # Set the feedrate to fast mode
        pass
        
    def set_slow_mode(self):
        # Set the feedrate to fast mode
        pass
     
    def resetprinter(self):
        # Send emergency stop command
        self.send_data("M112")
        print("Printer reset")

    def change_max_feeds(self, X, Y, Z, E):
        # Adjust maximum speeds
        command = f"M203 E{E} X{X} Y{Y} Z{Z}"
        self.send_data(command)

    def save_settings(self):
        # Save current configuration to printer memory
        self.send_data("M500")



###############################
# Communication Simulators
###############################
# These classes simulate the serial communication with the XY and ZP stages.
# All original functionality is preserved (serial buffering, command responses, etc.)
# but movement commands now result in a continuous, physically plausible motion
# (either via a constant velocity “VS” command or an absolute move “PA”/"G0" using proportional control).

# -------------------------
# XYStageSimulator
# -------------------------
class XYStageSimulator:
    def __init__(self, update_rate_hz=100, acceleration_rate=100, communication_delay=0.0, max_speed=100):
        # Position and velocity (units as used by stage manager)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        # For velocity moves, these are the target velocities.
        self.target_vx = 0.0
        self.target_vy = 0.0
        # For absolute moves, we store target positions.
        self.target_x = self.current_x
        self.target_y = self.current_y
        # Mode: "velocity" (default) or "absolute"
        self.mode = "velocity"
        # Proportional gain for absolute moves
        self.Kp = 2.0

        self.last_update_time = time.time()
        self.acceleration_rate = acceleration_rate  # Max change in velocity per second
        self.update_rate_hz = update_rate_hz          # Updates per second
        self.update_interval = 1.0 / update_rate_hz
        self.communication_delay = communication_delay  # Simulated communication delay
        self.max_speed = max_speed

        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.update_loop)
        self.thread.daemon = True

    def start(self):
        """Start the simulator update thread."""
        self.running = True
        self.thread.start()

    def stop(self):
        """Stop the simulator update thread."""
        self.running = False
        self.thread.join()

    def send_command(self, command):
        time.sleep(self.communication_delay)  # Simulated delay
        command = command.strip()
        # Velocity command remains unchanged.
        if command.startswith("VS"):
            parts = command.split(',')
            if len(parts) == 3:
                try:
                    vx = float(parts[1])
                    vy = float(parts[2])
                    with self.lock:
                        self.mode = "velocity"
                        self.target_vx = vx
                        self.target_vy = vy
                    return "R"
                except ValueError:
                    return "Invalid velocity values."
            return "Invalid command format."
        # New absolute move command. (e.g. "PA,50,50")
        elif command.startswith("PA"):
            parts = command.split(',')
            if len(parts) == 3:
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    with self.lock:
                        self.mode = "absolute"
                        self.target_x = x
                        self.target_y = y
                    return "R"
                except ValueError:
                    return "Invalid position values."
            return "Invalid command format."
        # Query command remains unchanged.
        elif command == "P":
            with self.lock:
                # Return current X, Y and dummy Z (always 0.0)
                return f"{self.current_x:.2f},{self.current_y:.2f},0.00"
        return "Unknown command."

    def get_current_position(self):
        """Return the current position as a tuple (x, y, 0.0)."""
        with self.lock:
            return self.current_x, self.current_y, 0.0

    def move_stage_at_velocity(self, vx, vy):
        """Wrapper to send a velocity command."""
        command = f"VS,{vx},{vy}"
        if vx is not 0.0 or vy is not 0.0:
            print(f"Sending XY command: {command}")
        self.send_command(command)

    def update_velocity(self, current, target, dt):
        """Gradually adjust velocity toward the target with a linear ramp."""
        if current < target:
            return min(current + self.acceleration_rate * dt, target)
        elif current > target:
            return max(current - self.acceleration_rate * dt, target)
        return current

    def update_loop(self):
        """Continuously update the stage position based on the commanded move."""
        while self.running:
            start_time = time.time()
            with self.lock:
                now = time.time()
                dt = now - self.last_update_time
                self.last_update_time = now

                if self.mode == "absolute":
                    # Compute error between target and current positions.
                    error_x = self.target_x - self.current_x
                    error_y = self.target_y - self.current_y
                    # Compute desired velocity via proportional control.
                    desired_vx = max(min(self.Kp * error_x, self.max_speed), -self.max_speed)
                    desired_vy = max(min(self.Kp * error_y, self.max_speed), -self.max_speed)
                else:  # "velocity" mode
                    desired_vx = self.target_vx
                    desired_vy = self.target_vy

                # Ramp current velocities toward desired velocities.
                self.current_vx = self.update_velocity(self.current_vx, desired_vx, dt)
                self.current_vy = self.update_velocity(self.current_vy, desired_vy, dt)

                # Update positions based on current velocities.
                self.current_x += self.current_vx * dt
                self.current_y += self.current_vy * dt

            elapsed = time.time() - start_time
            sleep_time = max(0, self.update_interval - elapsed)
            time.sleep(sleep_time)

# -------------------------
# ZPStageSimulator
# -------------------------
class ZPStageSimulator:
    def __init__(self):
        # Command/response queues and serial buffering (unchanged)
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.running = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.process_commands)
        self.thread.daemon = True

        # For gradual motion simulation, we add an update loop.
        self.update_thread = threading.Thread(target=self.update_loop)
        self.update_thread.daemon = True

        # Current positions and counts (for axes: X, Y, Z, E)
        self.position = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        self.counts = {'X': 0, 'Y': 0, 'Z': 0}

        # For gradual moves, maintain current velocities per axis.
        self.current_velocity = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        # Target positions for absolute moves.
        self.target_position = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        # Default to absolute positioning.
        self.absolute_mode = True

        self.communication_delay = 0.03  # Simulated communication delay
        self.processing_time_per_command = 0.01  # Simulated processing time per command

        self.buffer = b''

        # Parameters for gradual motion simulation.
        self.acceleration_rate = 100  # Max velocity change per second
        self.max_speed = 100
        self.Kp = 2.0  # Proportional gain for absolute moves

    def start(self):
        """Start both the command processor and the update loop."""
        self.running = True
        self.thread.start()
        self.update_thread.start()

    def stop(self):
        """Stop the simulator threads."""
        self.running = False
        self.thread.join()
        self.update_thread.join()

    def write(self, data):
        """Simulate writing data to the serial port."""
        with self.lock:
            self.buffer += data

    def flush(self):
        """Simulate flushing the serial port."""
        time.sleep(self.communication_delay)  # Simulated delay
        with self.lock:
            lines = self.buffer.split(b'\n')
            if lines:
                # Keep any incomplete line in the buffer.
                self.buffer = lines[-1] if self.buffer[-1:] != b'\n' else b''
                for line in lines[:-1]:
                    command = line.decode('utf-8').strip()
                    self.command_queue.put(command)
            else:
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
        """Process commands from the command queue (simulating a serial processor)."""
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                self.process_command(command)
                self.command_queue.task_done()
            except queue.Empty:
                continue

    def process_command(self, command):
        """Process a single G-code command while preserving original responses."""
        time.sleep(self.processing_time_per_command)  # Simulated processing time
        with self.lock:
            if command.startswith('G0'):
                # Movement command: instead of instantaneous update,
                # set target positions for each axis.
                axes = re.findall(r'([XYZE])([-\d\.]+)', command)
                if axes:
                    for axis, value in axes:
                        try:
                            val = float(value)
                        except ValueError:
                            continue
                        if self.absolute_mode:
                            self.target_position[axis] = val
                        else:
                            self.target_position[axis] += val
                    response = 'ok'
                else:
                    response = 'Invalid G0 command'
            elif command.strip() == 'M114':
                # Return current position and counts.
                response = (f"X:{self.position['X']} Y:{self.position['Y']} Z:{self.position['Z']} "
                            f"E:{self.position['E']} Count X:{self.counts['X']} Y:{self.counts['Y']} "
                            f"Z:{self.counts['Z']}")
            elif command.strip() == 'M503':
                response = 'Settings: M203 E10000 X10000 Y10000 Z10000'
            elif command.strip() == 'M500':
                response = 'Settings saved'
            elif command.startswith('M92'):
                response = 'Steps per unit set'
            elif command.startswith('M203'):
                response = 'Maximum feedrates set'
            elif command.strip() == 'M302 S0':
                response = 'Cold extrusion allowed'
            elif command.strip() == 'M83':
                response = 'Extruder set to relative mode'
            elif command.strip() == 'G91':
                self.absolute_mode = False
                response = 'Relative positioning enabled'
            elif command.strip() == 'G90':
                self.absolute_mode = True
                response = 'Absolute positioning enabled'
            elif command.strip() == 'M112':
                response = 'Emergency stop activated'
            else:
                response = 'Unknown command'
            self.response_queue.put(response)

    def update_velocity(self, current, target, dt):
        """Ramp current velocity toward the target value with acceleration limit."""
        if current < target:
            return min(current + self.acceleration_rate * dt, target)
        elif current > target:
            return max(current - self.acceleration_rate * dt, target)
        return current

    def update_loop(self):
        """Continuously update the stage position toward the target positions."""
        last_time = time.time()
        while self.running:
            start_time = time.time()
            with self.lock:
                dt = start_time - last_time
                last_time = start_time
                # For each axis, compute desired velocity using a proportional controller.
                for axis in self.position.keys():
                    error = self.target_position[axis] - self.position[axis]
                    desired_velocity = self.Kp * error
                    # Clamp the desired velocity.
                    desired_velocity = max(min(desired_velocity, self.max_speed), -self.max_speed)
                    self.current_velocity[axis] = self.update_velocity(self.current_velocity[axis],
                                                                         desired_velocity, dt)
                    # Update the position gradually.
                    self.position[axis] += self.current_velocity[axis] * dt
            elapsed = time.time() - start_time
            sleep_time = max(0, (1.0 / 100) - elapsed)  # Using 100 Hz update rate as default
            time.sleep(sleep_time)


if __name__ == "__main__":
    # Test the XYStageManager and ZPStageManager classes
    xy = XYStageManager(simulate=False)
    zp = ZPStageManager(simulate=False)

    # Test the XY stage movement functions
    xy.move_stage_at_velocity(100, 50)
    time.sleep(1)
    xy.move_stage_at_velocity(0, 0)
    time.sleep(1)
    xy.move_stage_to_position(100, 100)
    time.sleep(1)
    print("Current position:", xy.get_current_position())

    # Test the ZP stage movement functions
    zp.movecommand({'X': 10, 'Y': 20, 'Z': 30, 'E': 40})
    time.sleep(1)
    zp.movecommand({'X': 0, 'Y': 0, 'Z': 0, 'E': 0})
    time.sleep(1)
    print("Current position:", zp.get_current_position())

    # Clean up
    xy.stop()
    zp.stop()