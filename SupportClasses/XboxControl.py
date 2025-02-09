
from qt_core import *

# xbox_polling_worker.py
from multiprocessing import Queue
import pygame
import time
import json

def xbox_polling_worker(queue: Queue, mapping_file="button_mapping.json", avg_interval=0.5, deadzone=0.2):
    # Initialize Pygame and its joystick module
    pygame.init()
    pygame.joystick.init()
    
    # Attempt to load the button/axis/DPad mapping from a JSON file
    try:
        with open(mapping_file, "r") as f:
            mapping = json.load(f)
    except Exception as e:
        # If the mapping file cannot be loaded, use an empty mapping and notify via the queue
        mapping = {"buttons": {}, "axes": {}, "dpad": {}}
        queue.put({"debug": f"Mapping file error: {e}"})
    
    # Get the number of connected joysticks/controllers
    count = pygame.joystick.get_count()
    queue.put({"debug": f"Found {count} joystick(s)."})
    if count > 0:
        # Initialize the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        # Send controller details (name and available features) for debugging
        queue.put({
            "debug": f"Controller connected: {joystick.get_name()}",
            "joystick_info": {
                "numbuttons": joystick.get_numbuttons(),
                "numaxes": joystick.get_numaxes(),
                "numhats": joystick.get_numhats()
            }
        })
    else:
        # No controllers found, log the information and exit the function
        queue.put({"debug": "No controller connected."})
        return

    # Initialize accumulators for axis readings over time
    num_axes = joystick.get_numaxes()
    axis_accum = {axis: 0.0 for axis in range(num_axes)}  # Sum of axis values
    axis_count = {axis: 0 for axis in range(num_axes)}      # Counts of readings per axis
    last_axis_time = time.time()  # Timestamp for averaging intervals
    last_hat = (0, 0)             # Last recorded position of the DPad (hat)
    last_sent = {}                # Stores last sent axis values to avoid redundant messages

    # Main loop to continually poll and process joystick events
    while True:
        # Process internal Pygame events to update joystick state
        pygame.event.pump()

        # --- Process Button Presses ---
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):  # Check if button i is pressed
                # Get the command mapped to this button (if any)
                mapped_func = mapping.get("buttons", {}).get(str(i))
                if mapped_func and mapped_func != "None":
                    # Send the button press event and mapped command via the queue
                    queue.put({"button": i, "command": mapped_func})
                    time.sleep(0.2)  # Debounce delay to prevent multiple rapid triggers

        # --- Accumulate Axis Readings ---
        for axis in range(num_axes):
            val = joystick.get_axis(axis)
            axis_accum[axis] += val     # Sum the values for this axis
            axis_count[axis] += 1       # Count the number of readings

        current_time = time.time()
        # Process averaged axis values only after the specified averaging interval has elapsed
        if current_time - last_axis_time >= avg_interval:
            # Define groups of axes for combined processing
            groups = [
                {"name": "0-1", "axes": [0, 1], "type": "axis"},    # Group for axes 0 and 1
                {"name": "2-3", "axes": [2, 3], "type": "axis"},    # Group for axes 2 and 3
                {"name": "4",   "axes": [4],    "type": "trigger"}, # Group for trigger (axis 4)
                {"name": "5",   "axes": [5],    "type": "trigger"}  # Group for trigger (axis 5)
            ]
            # Iterate over each axis group for averaging and command dispatch
            for group in groups:
                averages = []  # Will store average values for all axes in this group
                for axis in group["axes"]:
                    # Compute the average if at least one reading was accumulated
                    if axis_count[axis]:
                        avg_val = axis_accum[axis] / axis_count[axis]
                    else:
                        avg_val = 0.0
                    # For trigger axes, adjust the reading from [-1, 1] to [0, 2]
                    if group["type"] == "trigger":
                        avg_val = avg_val + 1
                        # Invert the left trigger (if needed) for axis 4
                        if group["axes"][0] == 4:
                            avg_val = -avg_val
                    averages.append(avg_val)
                send = False  # Flag to determine if the axis movement is significant enough to send
                if len(averages) == 1:
                    # Single axis: check if the absolute value exceeds the deadzone threshold
                    if abs(averages[0]) > deadzone:
                        send = True
                else:
                    # For grouped axes: check if any axis exceeds the deadzone threshold
                    if any(abs(v) > deadzone for v in averages):
                        send = True
                # Get the mapped command for the current axis group
                mapped_func = mapping.get("axes", {}).get(group["name"])
                if mapped_func and mapped_func != "None":
                    # Prepare the value(s) to send: a single value or a tuple of values
                    if len(averages) == 1:
                        current_value = averages[0]
                    else:
                        current_value = tuple(round(v, 2) for v in averages)
                    # Define a zero value for resetting the state
                    zero_value = 0 if len(group["axes"]) == 1 else tuple(0 for _ in group["axes"])
                    if send:
                        # If the axis movement is significant, send the averaged value and command
                        queue.put({"axis": group["name"], "average": current_value, "command": mapped_func})
                        last_sent[group["name"]] = current_value
                    else:
                        # If the axis has returned to neutral and a non-zero value was previously sent, send a zero value
                        if group["name"] in last_sent and last_sent[group["name"]] != zero_value:
                            queue.put({"axis": group["name"], "average": zero_value, "command": mapped_func})
                            last_sent[group["name"]] = zero_value
                        elif group["name"] not in last_sent:
                            last_sent[group["name"]] = zero_value
                # Reset axis accumulators and counters for the next averaging period
                for axis in group["axes"]:
                    axis_accum[axis] = 0.0
                    axis_count[axis] = 0
            # Update the last axis processing timestamp
            last_axis_time = current_time

        # --- Process DPad (Hat) Input ---
        if joystick.get_numhats() > 0:
            # Get the current position of the DPad (hat number 0)
            current_hat = joystick.get_hat(0)
            # Check if the DPad state has changed since last iteration
            if current_hat != last_hat:
                # Retrieve the mapping for DPad directions
                dpad_map = mapping.get("dpad", {})
                # Process upward movement
                if current_hat[1] == 1:
                    func = dpad_map.get("up")
                    if func and func != "None":
                        queue.put({"dpad": "up", "command": func})
                # Process downward movement
                elif current_hat[1] == -1:
                    func = dpad_map.get("down")
                    if func and func != "None":
                        queue.put({"dpad": "down", "command": func})
                # Process rightward movement
                if current_hat[0] == 1:
                    func = dpad_map.get("right")
                    if func and func != "None":
                        queue.put({"dpad": "right", "command": func})
                # Process leftward movement
                elif current_hat[0] == -1:
                    func = dpad_map.get("left")
                    if func and func != "None":
                        queue.put({"dpad": "left", "command": func})
                # Update the last hat state to the current state
                last_hat = current_hat

        # Brief sleep to prevent high CPU usage
        time.sleep(0.02)
