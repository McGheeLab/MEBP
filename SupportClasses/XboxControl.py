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
        self.running = True
        self.connect()

        if not self.joystick:
            self.processor.add_command("debug", message="No joystick available. Exiting thread.")
            return

        # Initialize per-axis accumulators.
        num_axes = self.joystick.get_numaxes()
        axis_accum = {axis: 0.0 for axis in range(num_axes)}
        axis_count = {axis: 0 for axis in range(num_axes)}
        last_axis_time = time.time()

        # For DPad (hat) state.
        last_hat = (0, 0)

        while self.running:
            pygame.event.pump()

            # --- Process Button Presses ---
            for i in range(self.joystick.get_numbuttons()):
                if self.joystick.get_button(i):
                    mapped_func = self.mapping.get("buttons", {}).get(str(i), None)
                    if mapped_func is not None and mapped_func != "None":
                        # Publish the button command with additional details if needed.
                        self.processor.add_command(mapped_func, button=i)
                        time.sleep(0.2)  # debouncing

            # --- Accumulate Axis Readings ---
            for axis in range(num_axes):
                val = self.joystick.get_axis(axis)
                axis_accum[axis] += val
                axis_count[axis] += 1

            current_time = time.time()
            if current_time - last_axis_time >= self.avg_interval:
                # Process axes in groups.
                groups = [
                    {"name": "0-1", "axes": [0, 1], "type": "axis"},
                    {"name": "2-3", "axes": [2, 3], "type": "axis"},
                    {"name": "4", "axes": [4], "type": "trigger"},
                    {"name": "5", "axes": [5], "type": "trigger"}
                ]
                for group in groups:
                    averages = []
                    for axis in group["axes"]:
                        if axis_count[axis]:
                            avg_val = axis_accum[axis] / axis_count[axis]
                        else:
                            avg_val = 0.0
                        if group["type"] == "trigger":
                            avg_val = (avg_val + 1) / 2  # remap trigger value from [-1,1] to [0,1]
                        averages.append(avg_val)
                    
                    # Decide whether to send a command based on deadzone.
                    send = False
                    if len(averages) == 1:
                        if abs(averages[0]) > self.deadzone:
                            send = True
                    else:
                        if any(abs(v) > self.deadzone for v in averages):
                            send = True
                    mapped_func = self.mapping.get("axes", {}).get(group["name"], None)
                    
                    if mapped_func is not None and mapped_func != "None" and send:
                        if len(averages) == 1:
                            average=averages[0]
                        else:
                            average=tuple(round(v, 2) for v in averages)
                            
                        self.processor.add_command(mapped_func, axis=group["name"], average=average)
                    
                    # Reset accumulators for this group.
                    for axis in group["axes"]:
                        axis_accum[axis] = 0.0
                        axis_count[axis] = 0
                last_axis_time = current_time

            # --- Process DPad (Hat) Input ---
            if self.joystick.get_numhats() > 0:
                current_hat = self.joystick.get_hat(0)  # (x, y)
                if current_hat != last_hat:
                    dpad_map = self.mapping.get("dpad", {})
                    if current_hat[1] == 1:
                        func = dpad_map.get("up", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="up")
                    elif current_hat[1] == -1:
                        func = dpad_map.get("down", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="down")
                    if current_hat[0] == 1:
                        func = dpad_map.get("right", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="right")
                    elif current_hat[0] == -1:
                        func = dpad_map.get("left", None)
                        if func is not None and func != "None":
                            self.processor.add_command(func, direction="left")
                    last_hat = current_hat

            time.sleep(0.05)

        pygame.quit()

    def stop(self):
        self.running = False
