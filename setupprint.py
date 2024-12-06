import sys
import re
import time
import platform
import socket
import serial
import serial.tools.list_ports
from new6 import XYStageManager, ZPStageManager

class PlateDefinition:
    def __init__(self):
        # Default parameters for a plate, can be set by SET commands
        self.plate_type = None
        self.xp = None
        self.yp = None
        self.zp = None
        self.dx = None
        self.dy = None
        self.rows = 0
        self.cols = 0
        self.well_map = {}  # A1 -> (x, y, z), etc.

    def set_plate_type(self, plate_type):
        # Set plate parameters based on type
        # For simplicity, define a few known plate types
        if plate_type == "96WP":
            self.plate_type = "96WP"
            self.rows = 8
            self.cols = 12
            # Standard spacing for a 96 well plate is typically 9mm apart
            self.dx = 9.0
            self.dy = 9.0
        elif plate_type == "24WP":
            self.plate_type = "24WP"
            self.rows = 4
            self.cols = 6
            self.dx = 19.3
            self.dy = 19.3
        elif plate_type == "12WP":
            self.plate_type = "12WP"
            self.rows = 3
            self.cols = 4
            self.dx = 26.0
            self.dy = 26.0
        else:
            raise ValueError(f"Unknown plate type: {plate_type}")

    def set_top_left_position(self, xp, yp, zp):
        self.xp = xp
        self.yp = yp
        self.zp = zp

    def get_well_position(self, well_id):
        # well_id is something like "A1", "B2", etc.
        # Convert from well label (A-H for 96 well, etc.) to row/col indices
        # For a 96 well plate: Rows: A-H, Cols: 1-12
        # For generality, we assume row is alpha and col is numeric
        row_char = well_id[0].upper()
        col_num = int(well_id[1:])
        row_index = ord(row_char) - ord('A')  # 0-based
        col_index = col_num - 1  # 0-based

        # Calculate absolute position
        x_well = self.xp + col_index * self.dx
        y_well = self.yp + row_index * self.dy
        z_well = self.zp  # Typically printing at plate surface
        return (x_well, y_well, z_well)


class ReservoirDefinition:
    def __init__(self):
        # Dictionary: R1 -> { "type": "BioInk type 1", "x": ..., "y": ..., "z_pickup": ..., "z_retract": ...}
        self.reservoirs = {}

    def set_reservoir(self, name, ink_type, x, y, z_pickup, z_retract):
        self.reservoirs[name] = {
            "type": ink_type,
            "x": x,
            "y": y,
            "z_pickup": z_pickup,
            "z_retract": z_retract
        }

    def get_reservoir(self, name):
        return self.reservoirs.get(name, None)


class CalibrationManager:
    def __init__(self):
        self.x0 = None
        self.y0 = None
        self.z0 = None

    def set_calibration_point(self, x0, y0, z0):
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0

    def get_calibration_point(self):
        return (self.x0, self.y0, self.z0)


class PrintSetupManager:
    def __init__(self, xy_stage: XYStageManager, zp_stage: ZPStageManager):
        self.xy_stage = xy_stage
        self.zp_stage = zp_stage
        self.plate = PlateDefinition()
        self.reservoirs = ReservoirDefinition()
        self.calibration = CalibrationManager()
        self.current_bioink = None

    def parse_and_execute(self, command_file_path):
        with open(command_file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                self.execute_line(line)

    def execute_line(self, line):
        # Examples of commands:
        # SET: 96WP
        # R1: "BioInk type 1"
        # GOTO: Calibrate
        # GOTO: X100 Y100 Z10 Pa10 Pb2 Pc0
        # GOTO: Well A1
        # PRINT: "printfile1.csv"
        if line.startswith("SET:"):
            # Format: SET: 96WP or SET: 24WP etc.
            plate_type = line.split("SET:")[-1].strip()
            self.plate.set_plate_type(plate_type)
            print(f"Set plate type: {plate_type}")

        elif re.match(r"R\d+:", line):
            # Reservoir definition line
            # R1: "BioInk type 1"
            # The line might only set the ink type, but we should know X,Y,Z positions beforehand
            # or we could hardcode them or set them elsewhere.
            # For now, we assume ink type assignment. Positions could be configured separately or in code.
            parts = line.split(":")
            res_name = parts[0].strip()  # e.g. R1
            ink_type = parts[1].strip().strip('"')
            # Placeholder: you might define actual reservoir coordinates and pickup positions in code or another file
            # For demonstration, we assign some arbitrary values:
            x_res, y_res = 50.0, 50.0  # Example positions
            z_pickup = 10.0
            z_retract = 50.0
            self.reservoirs.set_reservoir(res_name, ink_type, x_res, y_res, z_pickup, z_retract)
            print(f"Set {res_name} with ink: {ink_type}")

        elif line.startswith("GOTO:"):
            # Could be "GOTO: Calibrate"
            # Or "GOTO: X100 Y100 Z10 Pa10 Pb2 Pc0"
            # Or "GOTO: Well A1"
            command_str = line.split("GOTO:")[-1].strip()
            self.handle_goto_command(command_str)

        elif line.startswith("PRINT:"):
            # PRINT: "printfile1.csv"
            filename = line.split("PRINT:")[-1].strip().strip('"')
            self.handle_print_command(filename)
            print(f"Printing from file: {filename}")

        else:
            print(f"Unknown command: {line}")

    def handle_goto_command(self, command_str):
        # Possible forms:
        # "Calibrate"
        # "X100 Y100 Z10 Pa10 Pb2 Pc0"
        # "Well A1"
        if command_str.Upper() == "calibrate":
            self.goto_calibration()
        elif command_str.Upper().startswith("well"):
            # GOTO: Well A1
            well_id = command_str.split("WELL")[-1].strip()
            self.goto_well(well_id)
        else:
            # Parse coordinates
            # Format might be: X100 Y100 Z10 Pa10 Pb2 Pc0
            # We'll parse this generically
            coords = self.parse_coordinates(command_str)
            self.goto_position(coords)

    def parse_coordinates(self, command_str):
        # Parse something like: X100 Y100 Z10 Pa10 Pb2 Pc0
        # We'll return a dict with keys: X, Y, Z, Pa, Pb, Pc
        coords = {}
        tokens = command_str.split()
        for tok in tokens:
            axis = tok[0]
            val = tok[1:]
            if axis in ['X', 'Y', 'Z', 'P']:
                # If P, then we need to distinguish which pump
                # Let's say Pa, Pb, Pc are p1, p2, p3
                if axis == 'P':
                    # This could be Pa10 meaning pump a move 10 units
                    subaxis = tok[1]  # a, b, c
                    val = tok[2:]  # the rest after Pa, Pb, Pc
                    coords[subaxis.upper()] = float(val)  # Store pump moves as 'A', 'B', 'C'
                else:
                    coords[axis] = float(val)
        return coords

    def goto_calibration(self):
        x0, y0, z0 = self.calibration.get_calibration_point()
        if x0 is None or y0 is None or z0 is None:
            print("Calibration point not set!")
            return
        self.xy_stage.move_stage_to_position(x0, y0)
        # Move Z to z0 using ZPStage
        self.zp_stage.movecommand({'Z': z0})
        print(f"Moved to calibration point: ({x0}, {y0}, {z0})")

    def goto_well(self, well_id):
        xw, yw, zw = self.plate.get_well_position(well_id)
        self.xy_stage.move_stage_to_position(xw, yw)
        self.zp_stage.movecommand({'Z': zw})
        print(f"Moved to well {well_id}: ({xw}, {yw}, {zw})")

    def goto_position(self, coords):
        # coords could have X, Y, Z, A, B, C (A,B,C for pumps)
        # Move XY first
        if 'X' in coords and 'Y' in coords:
            self.xy_stage.move_stage_to_position(coords['X'], coords['Y'])
        elif 'X' in coords:
            # Move X only
            # There's no direct command given for single-axis moves, but presumably same command is used
            # Usually XY stage commands move both axes at once. If Y not provided, we can just query current Y.
            x_cur, y_cur, z_cur = self.xy_stage.get_current_position()
            self.xy_stage.move_stage_to_position(coords['X'], y_cur)
        elif 'Y' in coords:
            # Move Y only
            x_cur, y_cur, z_cur = self.xy_stage.get_current_position()
            self.xy_stage.move_stage_to_position(x_cur, coords['Y'])

        # Move Z or pumps
        move_axes = {}
        if 'Z' in coords:
            move_axes['Z'] = coords['Z']
        # If we have pumps A, B, C
        # Let's assume that 'A', 'B', 'C' correspond to extruders/pumps (like E axis moves)
        # You may need to define how pump moves are handled. 
        # For demonstration, assume 'A', 'B', 'C' are additional axes or extruder steps:
        if 'A' in coords:
            move_axes['E'] = coords['A']  # map to extruder moves
        if 'B' in coords:
            # Possibly another extruder or a second pump axis if supported
            # If multiple pumps are independent axes, we might need separate commands per pump.
            # For now, assume sequential moves or a different G-code for pumps.
            # This will depend on how your ZP stage is controlled.
            pass
        if 'C' in coords:
            # Another pump axis if available
            pass

        if move_axes:
            self.zp_stage.movecommand(move_axes)

        print(f"Moved to position with coords: {coords}")

    def handle_print_command(self, filename):
        # This would handle the printing routine. You may need to:
        # - Read the print file (e.g. CSV with coordinates)
        # - Move stage and print accordingly
        # For demonstration, we just print a message.
        print(f"Start printing from file: {filename}")
        # Here you would parse printfile1.csv and send G-code moves using zp_stage and xy_stage.


# Example usage (not run):
# xy_manager = XYStageManager(simulate=True)
# zp_manager = ZPStageManager(simulate=True)
# psm = PrintSetupManager(xy_manager, zp_manager)
# psm.calibration.set_calibration_point(0, 0, 0)
# psm.plate.set_top_left_position(10, 10, 0)
# psm.parse_and_execute("commands.txt")

