import threading
import time
import json
import pygame

class XboxControllerInterface(threading.Thread):
    """
    Polls the Xbox controller using pygame.
    
    Reads:
      - Buttons (digital): sends an immediate command when pressed.
      - Axes: samples every 0.05 sec and averages over avg_interval seconds.
         • Axes 0 and 1 are linked (sent as a tuple).
         • Axes 2 and 3 are linked (sent as a tuple).
         • Axes 4 and 5 (triggers) are processed independently (remapped from [-1,1] to [0,1]).
      - DPad (hat): sends a command when the state changes.
    
    Mapping is loaded from a JSON file with structure:
      {
         "buttons": { "0": "ButtonCommand", ... },
         "axes":    { "0-1": "AxisCommand", "2-3": "AxisCommand", "4": "TriggerCommand", "5": "TriggerCommand" },
         "dpad":    { "up": "DPadCommand", "down": "DPadCommand", "left": "DPadCommand", "right": "DPadCommand" }
      }
    
    Instead of writing messages to a queue, this class now uses a central Processor instance.
    """
    def __init__(self, processor, deadzone=0.2, mapping_file="button_mapping.json", avg_interval=0.5):
        super().__init__()
        self.processor = processor
        self.deadzone = deadzone
        self.mapping_file = mapping_file
        self.avg_interval = avg_interval  # averaging period in seconds
        self.running = False

        # Load mapping from file (or default to empty sections)
        self.load_mapping()

        # Initialize pygame joystick support.
        pygame.init()
        pygame.joystick.init()
        self.joystick = None

    def load_mapping(self):
        try:
            with open(self.mapping_file, "r") as f:
                mapping = json.load(f)
            # Ensure the expected sections exist.
            if "buttons" not in mapping:
                mapping["buttons"] = {}
            if "axes" not in mapping:
                mapping["axes"] = {}
            if "dpad" not in mapping:
                mapping["dpad"] = {}
            self.mapping = mapping
        except Exception as e:
            self.processor.add_command("debug", message=f"Mapping file error: {e}")
            self.mapping = {"buttons": {}, "axes": {}, "dpad": {}}

    def save_mapping(self):
        with open(self.mapping_file, "w") as f:
            json.dump(self.mapping, f)

    def update_mapping(self, new_mapping):
        """Update the mapping (from the UI) and save it to the default file."""
        self.mapping = new_mapping
        self.save_mapping()

    def connect(self):
        """Connect to the first available joystick."""
        count = pygame.joystick.get_count()
        self.processor.add_command("debug", message=f"Found {count} joystick(s).")
        if count > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.processor.add_command("debug", message=f"Controller connected: {self.joystick.get_name()}")
            self.processor.add_command(
                "debug",
                message=f"Joystick has {self.joystick.get_numbuttons()} buttons, "
                        f"{self.joystick.get_numaxes()} axes, {self.joystick.get_numhats()} hat(s)."
            )
        else:
            self.processor.add_command("debug", message="No controller connected.")

    def run(self):
        # Set the running flag to True and establish connection with a joystick.
        self.running = True
        self.connect()

        # Check if a joystick has been found. Exit early if not.
        if not self.joystick:
            self.processor.add_command("debug", message="No joystick available. Exiting thread.")
            return

        # Initialize accumulators for each axis.
        # axis_accum stores the sum of values read from each axis.
        # axis_count counts how many readings have been accumulated per axis.
        num_axes = self.joystick.get_numaxes()
        axis_accum = {axis: 0.0 for axis in range(num_axes)}
        axis_count = {axis: 0 for axis in range(num_axes)}
        last_axis_time = time.time()  # Timestamp marking the start of the interval for averaging.

        # Initialize the previous hat (DPad) state for detecting changes.
        last_hat = (0, 0)

        # Dictionary to track the last sent command values for each axis group.
        last_sent = {}  # Key: group name, Value: last average value sent (could be a scalar or tuple)

        # Main loop that runs while the controller is active.
        while self.running:
            # Process pending Pygame events (refresh joystick state).
            pygame.event.pump()

            # --- Process Button Presses ---
            # Loop over each button and check if pressed.
            for i in range(self.joystick.get_numbuttons()):
                if self.joystick.get_button(i):
                    # Find the mapping for this button.
                    mapped_func = self.mapping.get("buttons", {}).get(str(i), None)
                    if mapped_func is not None and mapped_func != "None":
                        # Send a command to the processor with the button pressed.
                        self.processor.add_command(mapped_func, button=i)
                        time.sleep(0.2)  # Short delay for debouncing to avoid multiple triggers.

            # --- Accumulate Axis Readings ---
            # For each axis, gather the current reading.
            for axis in range(num_axes):
                val = self.joystick.get_axis(axis)
                axis_accum[axis] += val
                axis_count[axis] += 1

            # Check if it's time to process the average axis value.
            current_time = time.time()
            if current_time - last_axis_time >= self.avg_interval:
                # Define groups of axes. Each group might represent 
                # different controls, e.g., left stick (axes 0-1), right stick (axes 2-3), triggers, etc.
                groups = [
                    {"name": "0-1", "axes": [0, 1], "type": "axis"},
                    {"name": "2-3", "axes": [2, 3], "type": "axis"},
                    {"name": "4", "axes": [4], "type": "trigger"},
                    {"name": "5", "axes": [5], "type": "trigger"}
                ]
                for group in groups:
                    averages = []
                    #########################################################################
                    ########## Compute average value for each axis in the group.#############
                    #########################################################################
                    for axis in group["axes"]:
                    
                        ################## Compute the average value for the axis.###############                        
                        if axis_count[axis]:
                            avg_val = axis_accum[axis] / axis_count[axis]
                        else:
                            avg_val = 0.0
    
                        ################## Remap the value for triggers.#########################
                        
                        if group["type"] == "trigger": 
                            # Remap the value from [-1, 1] to [0, 1].
                            avg_val = (avg_val + 1) 
                            # Invert the value for the left trigger (axis 4)
                            if group["axes"][0] == 4:
                                avg_val = -avg_val
                        averages.append(avg_val)
                    
                    #########################################################################
                    ### Check if the average value is outside the deadzone.##################
                    #########################################################################
                    send = False # Flag to determine if a command should be sent.
                    if len(averages) == 1:
                        if abs(averages[0]) > self.deadzone:
                            send = True
                    else:
                        if any(abs(v) > self.deadzone for v in averages):
                            send = True
                    # Retrieve the mapped function for this group of axes.
                    mapped_func = self.mapping.get("axes", {}).get(group["name"], None)
                    
                    #########################################################################
                    ################## Send the command to the processor ####################
                    #########################################################################
                    if mapped_func is not None and mapped_func != "None":
                        
                        ############# Round the computed averages ####################
                        if len(averages) == 1:
                            current_value = averages[0]
                        else:
                            current_value = tuple(round(v, 2) for v in averages)

                        ############# create a zero value for the group.############
                        if len(group["axes"]) == 1:
                            zero_value = 0
                        else:
                            zero_value = tuple(0 for _ in group["axes"])

                        ############# Send the command to the processor.############
                        if send:
                            # If outside the deadzone, send the averaged axis value.
                            self.processor.add_command(mapped_func, axis=group["name"], average=current_value)
                            last_sent[group["name"]] = current_value
                        else:
                            # If within deadzone but a nonzero command was previously sent, 
                            # send a zero command to stop the motion.
                            if group["name"] in last_sent and last_sent[group["name"]] != zero_value:
                                self.processor.add_command(mapped_func, axis=group["name"], average=zero_value)
                                last_sent[group["name"]] = zero_value
                                
                            # Otherwise, initialize the last_sent record in case no command was ever sent.
                            elif group["name"] not in last_sent:
                                last_sent[group["name"]] = zero_value

                    # Reset the accumulators for the axes in this group for future readings.
                    for axis in group["axes"]:
                        axis_accum[axis] = 0.0
                        axis_count[axis] = 0
                # Update the timestamp for the next averaging period.
                last_axis_time = current_time

            # --- Process DPad (Hat) Input ---
            # If the joystick has a DPad (hat), process its input.
            if self.joystick.get_numhats() > 0:
                current_hat = self.joystick.get_hat(0)  # Obtain the current DPad state as a tuple (x, y)
                if current_hat != last_hat:
                    # Retrieve mapping for DPad directions.
                    dpad_map = self.mapping.get("dpad", {})
                    if current_hat[1] == 1:
                        # DPad moved up.
                        func = dpad_map.get("up", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="up")
                    elif current_hat[1] == -1:
                        # DPad moved down.
                        func = dpad_map.get("down", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="down")
                    if current_hat[0] == 1:
                        # DPad moved right.
                        func = dpad_map.get("right", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="right")
                    elif current_hat[0] == -1:
                        # DPad moved left.
                        func = dpad_map.get("left", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="left")
                    # Update the last recorded hat state after processing.
                    last_hat = current_hat
                    
    def stop(self):
        self.running = False
