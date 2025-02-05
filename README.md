# MEBP
Microscope Enabled Bioprinter


Version 0.7
Last updated 01/29/2025
Author Alex McGhee

- [MEBP](#mebp)
- [XYStageManager](#xystagemanager)
  - [Overview](#overview)
  - [Attributes](#attributes)
  - [Methods](#methods)
- [ZPStageManager](#zpstagemanager)
  - [Overview](#overview-1)
  - [Attributes](#attributes-1)
  - [Methods](#methods-1)
- [Stages](#stages)
  - [Overview](#overview-2)
  - [Attributes](#attributes-2)
  - [Methods](#methods-2)
- [Waypoint](#waypoint)
  - [Overview](#overview-3)
  - [Attributes](#attributes-3)
  - [Methods](#methods-3)
- [XYStageSimulator](#xystagesimulator)
  - [Overview](#overview-4)
  - [Attributes](#attributes-4)
  - [Methods](#methods-4)
- [ZPStageSimulator](#zpstagesimulator)
  - [Overview](#overview-5)
  - [Attributes](#attributes-5)
  - [Methods](#methods-5)
- [Main Execution](#main-execution)

# XYStageManager

## Overview
The `XYStageManager` class manages the XY stage movement. It can operate in simulation mode or connect to an actual ProScan III controller.

## Attributes
- `simulate`: Determines if the class operates in simulation mode.
- `spo`: Serial port object, either real or simulated.

## Methods
- `__init__(simulate=False)`: Initializes the XY stage manager.
- `__del__()`: Ensures cleanup by stopping the simulator if running.
- `initialize_serial_port()`: Finds and connects to the ProScan III controller.
- `find_proscan_controller()`: Searches available COM ports for the controller.
- `send_command(command)`: Sends a command to the stage.
- `get_current_position()`: Queries the current position of the stage.
- `move_stage_at_velocity(vx, vy)`: Moves the stage at the specified velocity.
- `move_stage_to_position(x, y)`: Moves the stage to a specific position.

---

# ZPStageManager

## Overview
The `ZPStageManager` class manages communication with a 3D printer board for Z-axis control.

## Attributes
- Position attributes: `x_pos`, `y_pos`, `z_pos`, `e_pos`
- Connection attributes: `COM`, `baudrate`, `serial`
- `simulate`: Enables simulation mode.
- `printer_found`: Indicates whether a 3D printer was detected.

## Methods
- `__init__(simulate=False)`: Initializes the ZPStageManager and searches for a 3D printer.
- `__del__()`: Ensures cleanup by closing the serial connection.
- `send_data(data)`: Sends commands to the printer.
- `receive_data()`: Reads response data.
- `movecommand(axes, feedrate=None)`: Moves the ZP stage using G-code commands.
- `get_position()`: Retrieves the current position of the Z and P stages.
- `extract_position_data(response)`: Parses position data from the printer response.
- `get_all_data()`: Requests all stored settings from the printer.
- `request_data()`: Requests and prints the current position.
- `resetprinter()`: Sends an emergency stop command.
- `setup()`: Configures printer settings for movement.
- `change_max_feeds(X, Y, Z, E)`: Modifies maximum feed rates.
- `set_feedrate(value)`: Sets a specific feed rate.
- `get_available_com_ports()`: Lists available COM ports.
- `is_3d_printer(port)`: Checks if a given port belongs to a 3D printer.
- `save_settings()`: Saves current printer settings.

---

# Stages

## Overview
The `Stages` class orchestrates movements for both the XY and ZP stages, using PID control for smooth motion.

## Attributes
- `xy_manager`: Manages XY movements.
- `zp_manager`: Manages ZP movements.
- `waypoints`: Stores movement waypoints.
- `Kp, Ki, Kd`: PID controller parameters.
- Various tracking variables for error correction and velocity management.

## Methods
- `__init__(waypoints, simulate=False, Kp=1.0, Ki=0.0, Kd=0.0)`: Initializes stage control with waypoints.
- `__del__()`: Ensures cleanup of simulators.
- `calculate_velocity_with_pid(error_x, error_y, delta_time)`: Implements PID control for movement.
- `plot_results_3d(...)`: Visualizes movement results in 3D.
- `xy_update_thread(...)`: Controls XY stage in a separate thread.
- `zp_update_thread(...)`: Controls ZP stage in a separate thread.
- `move(plot_title, interpolation_type='linear', plot=False)`: Executes movement based on waypoints.

---

# Waypoint

## Overview
The `Waypoint` class manages waypoint data for the movement process.

## Attributes
- `csv_file_path`: Path to the CSV file containing waypoints.
- `waypoints`: Stores parsed waypoints.

## Methods
- `__init__(csv_file_path)`: Initializes waypoint handling.
- `import_waypoints_from_csv()`: Reads waypoints from a CSV file.
- `interpolate_waypoints(elapsed_time, ...)`: Interpolates movement between waypoints.

---

# XYStageSimulator

## Overview
Simulates the XY stage for testing purposes.

## Attributes
- `current_x, current_y`: Current position.
- `current_vx, current_vy`: Current velocity.
- `acceleration_rate`: Rate of velocity change.
- `update_rate_hz`: Frequency of updates.
- `communication_delay`: Simulated delay.
- `running`: Controls simulation state.

## Methods
- `start()`: Starts the simulation thread.
- `stop()`: Stops the simulation thread.
- `send_command(command)`: Simulates command execution.
- `get_current_position()`: Retrieves simulated position.
- `move_stage_at_velocity(vx, vy)`: Simulates stage movement.
- `update_loop()`: Main loop for updating movement.

---

# ZPStageSimulator

## Overview
Simulates ZP stage operations for testing.

## Attributes
- `command_queue`: Queue for incoming commands.
- `response_queue`: Queue for responses.
- `position`: Stores axis positions.
- `counts`: Stores axis counts.
- `communication_delay`: Simulated communication delay.

## Methods
- `start()`: Starts the simulator thread.
- `stop()`: Stops the simulator thread.
- `write(data)`: Simulates data transmission.
- `flush()`: Simulates buffer flushing.
- `read_all()`: Reads simulated responses.
- `process_commands()`: Processes queued commands.
- `process_command(command)`: Handles a single G-code command.

---

# Main Execution
- Reads waypoints from `multi_layer_toolpath.csv`.
- Creates an instance of `Stages`.
- Executes movement if waypoints are available, with optional 3D plotting.
