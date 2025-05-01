from multiprocessing import Process, Queue
import threading
import queue
import math
import time

from SupportClasses.DeviceInterface import ZPStageManager, XYStageManager

from qt_core import *


# manages command processing in a separate thread usable by any device.
class Processor:
    def __init__(self):
        # Thread-safe queue for commands; each command is a tuple: (command_name, args, kwargs)
        self._queue = queue.Queue()
        # Dictionary mapping event names to lists of subscriber callbacks
        self._subscribers = {}
        self._running = True
        # Start the processing thread
        self._thread = threading.Thread(target=self._process_loop, name="ProcessorThread", daemon=True)
        self._thread.start()

    def register_handler(self, command_name, handler):
        """
        Register a callback for a specific command.
        :param command_name: str - the name of the command to subscribe to.
        :param handler: callable - a function to call when the command is processed.
        """
        if command_name not in self._subscribers:
            self._subscribers[command_name] = []
        self._subscribers[command_name].append(handler)

    def unregister_handler(self, command_name, handler):
        """
        Unregister a previously registered handler.
        """
        if command_name in self._subscribers:
            self._subscribers[command_name].remove(handler)
            if not self._subscribers[command_name]:
                del self._subscribers[command_name]

    def add_command(self, command_name, *args, **kwargs):
        """
        Add a new command to the processor's queue.
        :param command_name: str - the identifier for the command.
        :param args: positional arguments for the handler.
        :param kwargs: keyword arguments for the handler.
        """
        self._queue.put((command_name, args, kwargs))

    def _process_loop(self):
        """
        Internal method run in a separate thread. Processes commands as they arrive.
        """
        while self._running:
            try:
                # Wait for a command; timeout allows checking self._running periodically.
                command_name, args, kwargs = self._queue.get(timeout=1)
            except queue.Empty:
                continue

            # Dispatch command to all subscribers registered for that command name.
            handlers = self._subscribers.get(command_name, [])
            if not handlers:
                # Optionally, log or handle unregistered commands.
                print(f"[Processor] No handler registered for command: {command_name}")
            else:
                for handler in handlers:
                    try:
                        handler(*args, **kwargs)
                    except Exception as e:
                        # Exception handling for a misbehaving handler.
                        print(f"[Processor] Error in handler {handler} for command '{command_name}': {e}")
            self._queue.task_done()

    def stop(self):
        """
        Signal the processor to stop and wait for the thread to finish.
        """
        self._running = False
        self._thread.join()

class StageHandler:
    def __init__(self, processor, zp_stage, xy_stage):
        self.XYUPDATE_INTERVAL = 1.15
        self.ZUPDATE_INTERVAL = 0.3333
        self.POS_UPDATE_INTERVAL = 0.5623  # Polling interval for updating positions

        self.z_fast_mode_velocity = 1000 # mm/min
        self.z_slow_mode_velocity = 100 # mm/min
        self.xy_fast_mode_velocity = 1000 # mm/min
        self.p_load_velocity = 100 # mm/min
        
        self.processor = processor
        self.zp_stage = zp_stage
        self.xy_stage = xy_stage
        self._running = True
        
        # Stages are off by default.
        self._zp_running = False
        self._xy_running = False

        self.zspeed = 1.0
        self.pspeed = 1.0
        self.xyspeed = 1.0
        
        self.zspeed_range = (0.01, 10.0)
        self.pspeed_range = (0.01, 10.0)
        self.xyspeed_range = (1.0, 10000.0)
        
        self.maxzspeed = 250
        self.maxpspeed = 250
        self.maxxyspeed = 5000
        
        # State dictionaries storing full information for each axis.
        self.zp_state = {
            "Z": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "P1": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "P2": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "P3": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
        }
        
        self.axes_mapping = {
            "Z": "X",
            "P1": "Y",
            "P2": "Z",
            "P3": "E"
        }
        
        self.xy_state = {
            "x": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "y": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "f": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
        }
        
        self.zero_position = {
            "x": 0.0,
            "y": 0.0,
            "f": 0.0,
            "Z": 0.0,
            "P1": 0.0,
            "P2": 0.0,
            "P3": 0.0
        }

        # Register command handlers.
        self.processor.register_handler("move_axis_1mm", self.handle_move_axis_1mm)
        self.processor.register_handler("set_axis_deactivation", self.handle_set_axis_deactivation)
        self.processor.register_handler("move_z_at_velocity", self.update_z_velocity)
        self.processor.register_handler("move_p1_at_velocity", self.update_p1_velocity)
        self.processor.register_handler("move_p2_at_velocity", self.update_p2_velocity)
        self.processor.register_handler("move_p3_at_velocity", self.update_p3_velocity)
        self.processor.register_handler("move_stage_at_velocity", self.update_xy_velocity)
        self.processor.register_handler("control_stage", self.handle_control_stage)
        self.processor.register_handler("zero_needle_pos", self.calibrate_needle_position)
        self.processor.register_handler("increment_zspeed_up", self.increment_zspeed_up)
        self.processor.register_handler("increment_zspeed_down", self.increment_zspeed_down)
        self.processor.register_handler("increment_pspeed_up", self.increment_pspeed_up)
        self.processor.register_handler("increment_pspeed_down", self.increment_pspeed_down)
        self.processor.register_handler("increment_xyspeed_up", self.increment_xyspeed_up)
        self.processor.register_handler("increment_xyspeed_down", self.increment_xyspeed_down)
            
        self._update_XY_positions()
        self._update_ZP_positions()    
            
    # ----- basic move to location commands for both stages -----
    def move_abs_z_zero_reference(self, z_value, fastmode=False):
        # calculate the distance to move from the zero position and the z_value given
        position = z_value - self.zero_position["Z"]
        mappedZ = self.axes_mapping.get("Z")
          
        # move the stage to the new position
        self.zp_stage.move_absolute(axes={ mappedZ: position}, fast=fastmode)
    
    def move_abs_xy_zero_reference(self, x_value, y_value,fastmode=False):
        # calculate the distance to move from the zero position and the x_value and y_value given
        x_position = x_value - self.zero_position["x"]
        y_position = y_value - self.zero_position["y"]
        
        # move the stage to the new position
        self.xy_stage.move_stage_to_position(x_position, y_position, fast=fastmode)
    
    def move_abs_xy_well_reference(self, x_value, y_value, fastmode=False):
        # calculate the distance to move from the zero position and the x_value and y_value given
        x_position = x_value
        y_position = y_value
        
        # move the stage to the new position
        self.xy_stage.move_stage_to_position(x_position, y_position, fast=fastmode)
    
    def move_abs_xy(self, x_value, y_value):
        self.xy_stage.move_stage_to_position(x_value, y_value)
    
    def move_rel_z(self, z_value, feedrate):
        # move the stage to a relative position from the current position
        mappedZ = self.axes_mapping.get("Z")
        self.zp_stage.move_relative(axes={mappedZ: z_value}, feedrate=feedrate)
    
    def move_rel_xy(self, x_value, y_value):
        # calculate the distance to move from the current position and the x_value and y_value given
        x = self.xy_state["x"]["position"] + x_value
        y = self.xy_state["y"]["position"] + y_value
        
        # move the stage to a relative position from the current position
        self.xy_stage.move_stage_to_position(x, y)  
    
    def move_rel_p(self, pump, p_value):
        # move the stage to a relative position from the current position
        mappedP = self.axes_mapping.get(pump)
        self.zp_stage.move_relative(axes={mappedP: p_value})
        
    # ----- Velocity factor for each stage -----
    def increment_zspeed_up(self, *args, **kwargs):
        # if speed is within the speed range multiply the speed by 10
        if self.zspeed < self.zspeed_range[1]:
            self.zspeed *= 10
        print(f"ZSpeed: {self.zspeed}")
    
    def increment_zspeed_down(self, *args, **kwargs):
        # if speed is within the speed range divide the speed by 10
        if self.zspeed > self.zspeed_range[0]:
            self.zspeed /= 10
        print(f"ZSpeed: {self.zspeed}")
        
    def increment_pspeed_up(self, *args, **kwargs):
        # if speed is within the speed range multiply the speed by 10
        if self.pspeed < self.pspeed_range[1]:
            self.pspeed *= 10
        print(f"PSpeed: {self.pspeed}")
    
    def increment_pspeed_down(self, *args, **kwargs):
        # if speed is within the speed range divide the speed by 10
        if self.pspeed > self.pspeed_range[0]:
            self.pspeed /= 10
        print(f"PSpeed: {self.pspeed}")
            
    def increment_xyspeed_up(self, *args, **kwargs):
        # if speed is within the speed range multiply the speed by 10
        if self.xyspeed < self.xyspeed_range[1]:
            self.xyspeed *= 10
        print(f"XY speed: {self.xyspeed}")
    
    def increment_xyspeed_down(self, *args, **kwargs):
        # if speed is within the speed range divide the speed by 10
        if self.xyspeed > self.xyspeed_range[0]:
            self.xyspeed /= 10
        print(f"XY speed: {self.xyspeed}")

    
    # ----- Velocity Update Methods for ZP Stage -----
    def update_z_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["Z"]["velocity"] = velocity
        # check if the velocity is not faster than the max
        if self.zp_state["Z"]["velocity"] > self.maxzspeed:
            self.zp_state["Z"]["velocity"] = self.maxzspeed
            print(f"speed limit reached: {self.zp_state['Z']['velocity']}")
            
        self.zp_state["Z"]["active"] = (velocity != 0)

    def update_p1_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["P1"]["velocity"] = velocity*self.pspeed
        # check if the velocity is not faster than the max
        if self.zp_state["P1"]["velocity"] > self.maxpspeed:
            self.zp_state["P1"]["velocity"] = self.maxpspeed
            print(f"speed limit reached: {self.zp_state['P1']['velocity']}")
        self.zp_state["P1"]["active"] = (velocity != 0)

    def update_p2_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["P2"]["velocity"] = velocity*self.pspeed
        # check if the velocity is not faster than the max
        if self.zp_state["P2"]["velocity"] > self.maxpspeed:
            self.zp_state["P2"]["velocity"] = self.maxpspeed
            print(f"speed limit reached: {self.zp_state['P2']['velocity']}")    
        self.zp_state["P2"]["active"] = (velocity != 0)

    def update_p3_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["P3"]["velocity"] = velocity*self.pspeed
        # check if the velocity is not faster than the max
        if self.zp_state["P3"]["velocity"] > self.maxpspeed:
            self.zp_state["P3"]["velocity"] = self.maxpspeed
            print(f"speed limit reached: {self.zp_state['P3']['velocity']}")
        self.zp_state["P3"]["active"] = (velocity != 0)

    # ----- Velocity Update for XY Stage -----
    def update_xy_velocity(self, *args, **kwargs):
        vx, vy = self._extract_velocity(*args, **kwargs)
        
        self.xy_state["x"]["velocity"] = vx 
        self.xy_state["y"]["velocity"] = vy
        # check if the velocity is not faster than the max
        if self.xy_state["x"]["velocity"] > self.maxxyspeed:
            self.xy_state["x"]["velocity"] = self.maxxyspeed
            print(f"speed limit reached: {self.xy_state['x']['velocity']}")
        if self.xy_state["y"]["velocity"] > self.maxxyspeed:
            self.xy_state["y"]["velocity"] = self.maxxyspeed
            print(f"speed limit reached: {self.xy_state['y']['velocity']}")
        self.xy_state["x"]["active"] = (vx != 0)
        self.xy_state["y"]["active"] = (vy != 0)
    
    def _extract_velocity(self, *args, **kwargs):
        # Check if average is passed as a keyword argument.
        if "average" in kwargs:
            velocity = kwargs["average"]
        else:
            velocity = 0.0

        # check if velocity is a tuple or single value
        if isinstance(velocity, (list, tuple)):
            v1, v2 = velocity[0], velocity[1]
        else:
            v1, v2 = velocity, 0.0
        return v1, v2

    def send_zp_move_command(self):
        print("Sending ZP move command")
        dt = self.ZUPDATE_INTERVAL
        vz = self.zp_state["Z"]["velocity"]
        vp1 = self.zp_state["P1"]["velocity"]
        vp2 = self.zp_state["P2"]["velocity"]
        vp3 = self.zp_state["P3"]["velocity"]
        dz = vz * dt
        dp1 = vp1 * dt
        dp2 = vp2 * dt
        dp3 = vp3 * dt
        distance = math.sqrt(dz**2 + dp1**2 + dp2**2 + dp3**2)
        if distance == 0:
            return
        feedrate = (distance / dt) * 60
        axes = {'X': dz, 'Y': dp1, 'Z': dp2, 'E': dp3} # Mapping to contoller axes
        print(f"axes {axes} at feedrate {feedrate}")
        self.zp_stage.move_relative(axes, feedrate)

    # ----- Stage Movement Commands -----
    def jog_xy(self, vx, vy):
        # check if the velocity is not faster than the max and update the state
        #self.update_xy_velocity(vx, vy)
        
        # get stage velocity from the state
        #vx = self.xy_state["x"]["velocity"]
        #vy = self.xy_state["y"]["velocity"]
        
        self.xy_stage.move_stage_at_velocity(vx, vy)
        
    
    # ----- Position Polling and State Update -----
    def _update_ZP_positions(self):
        zp_positions = self.zp_stage.get_current_position()  # e.g., (z, p1, p2, p3)
        if zp_positions and len(zp_positions) >= 4:
            self.zp_state["Z"]["position"] = zp_positions[0]
            self.zp_state["P1"]["position"] = zp_positions[1]
            self.zp_state["P2"]["position"] = zp_positions[2]
            self.zp_state["P3"]["position"] = zp_positions[3]
    
    def _update_XY_positions(self):
        xy_positions = self.xy_stage.get_current_position()  # e.g., (x, y, f)
        print(f"XY positions: {xy_positions}")
        if xy_positions and len(xy_positions) >= 3 and xy_positions[0] is not None and xy_positions[1] is not None and xy_positions[2] is not None:
            self.xy_state["x"]["position"] = xy_positions[0]
            self.xy_state["y"]["position"] = xy_positions[1]
            self.xy_state["f"]["position"] = xy_positions[2]

    def get_XY_positions(self):
        self._update_XY_positions()
        x,y,f = self.xy_state["x"]["position"], self.xy_state["y"]["position"], self.xy_state["f"]["position"]
        return x,y,f
    
    def get_ZP_positions(self):
        self._update_ZP_positions() 
        z,p1,p2,p3 = self.zp_state["Z"]["position"], self.zp_state["P1"]["position"], self.zp_state["P2"]["position"], self.zp_state["P3"]["position"]
        return z,p1,p2,p3
    
    def calibrate_needle_position(self, *args, **kwargs):
        # get current position

        self.zero_position["Z"] = self.zp_state["Z"]["position"]
        self.zero_position["P1"] = self.zp_state["P1"]["position"]
        self.zero_position["P2"] = self.zp_state["P2"]["position"]
        self.zero_position["P3"] = self.zp_state["P3"]["position"]
        self.zero_position["x"] = self.xy_state["x"]["position"]
        self.zero_position["y"] = self.xy_state["y"]["position"]
        self.zero_position["f"] = self.xy_state["f"]["position"]

        # show in command console all of the zero positions
        print("Zero positions:")
        for axis, pos in self.zero_position.items():
            print(f"{axis}: {pos}")
        print("Needle position calibrated" )

    # ----- Command Handlers for GUI Commands -----
    def handle_move_axis_1mm(self, **kwargs):
        stage = kwargs.get("stage")
        axis = kwargs.get("axis")
        distance = kwargs.get("distance", 1)
        if stage == "ZP":
            if axis in self.zp_state:
                mapped_axis = self.axes_mapping.get(axis)
                axes = {mapped_axis: distance}
                self.zp_stage.movecommand(axes, feedrate=60)
                print(f"Moving {axis} axis of ZP stage by {distance}mm")
        elif stage == "XY":
            if axis in self.xy_state:
                if axis == "x":
                    self.xy_stage.move_stage_at_velocity(distance, 0)
                elif axis == "y":
                    self.xy_stage.move_stage_at_velocity(0, distance)
                elif axis == "f":
                    print("f axis move not supported yet for XY stage")
                print(f"Moving {axis} axis of XY stage by {distance}mm")
        else:
            print(f"Unknown stage: {stage}")

    def handle_set_axis_deactivation(self, **kwargs):
        stage = kwargs.get("stage")
        axis = kwargs.get("axis")
        deactivated = kwargs.get("deactivated", False)
        if stage == "ZP":
            if axis in self.zp_state:
                self.zp_state[axis]["user_deactivated"] = deactivated
                print(f"ZP stage {axis} deactivated: {deactivated}")
        elif stage == "XY":
            if axis in self.xy_state:
                self.xy_state[axis]["user_deactivated"] = deactivated
                print(f"XY stage {axis} deactivated: {deactivated}")

    def handle_control_stage(self, **kwargs):
        stage = kwargs.get("stage")
        action = kwargs.get("action")
        if stage == "ZP":
            if action == "start":
                self._zp_running = True
                print("ZP stage started")
            elif action == "stop":
                self._zp_running = False
                print("ZP stage stopped")
        elif stage == "XY":
            if action == "start":
                self._xy_running = True
                print("XY stage started")
            elif action == "stop":
                self._xy_running = False
                print("XY stage stopped")
        else:
            print(f"Unknown stage for control: {stage}")

    def get_stage_info(self):
        return {"ZP": self.zp_state.copy(), "XY": self.xy_state.copy()}

    def set_stage_info(self, stage, axis, info):
        if stage == "ZP" and axis in self.zp_state:
            self.zp_state[axis].update(info)
        elif stage == "XY" and axis in self.xy_state:
            self.xy_state[axis].update(info)
        
    def getAxisMap(self, axis):
        return self.axes_mapping.get(axis, None)

class AppController:
    def __init__(self,simulatexy=True, simulatezp=True):
        
        self.simulatexy = simulatexy  # Set to True to simulate devices.
        self.simulatezp = simulatezp  # Set to True to simulate devices.
        
        self.processor = Processor()
        # Initialize device attributes as None.
        self.xbox_interface = None  # Not used now because we use a process.
        self.zp_stage = None
        self.xy_stage = None
        self.stage_handler = None
        
        # Create a Queue for Xbox polling messages.
        self.xbox_queue = Queue()
        self.xbox_process = None
        self.xbox_timer = None  # QTimer to poll the queue.
        
        
        self.processor.register_handler("control_xbox", self.handle_control_xbox)# Register control handler for Xbox commands.
        self.processor.register_handler("control_stage", self.handle_control_stages)# Register a handle to start the stage devices.
        self.processor.register_handler("debug", self.debug_handler)# Register a simple debug handler that prints debug messages.
     
    def stop(self):
        print("AppController: Shutting down application.")
        if self.xbox_interface:
            self.stop_xbox_interface()
        if self.stage_handler or self.zp_stage or self.xy_stage:
            self.stop_stage_devices()
        self.processor.stop()
        
    def debug_handler(self, *args, **kwargs):
        message = kwargs.get("message", "")
        print("DEBUG:", message)
    
    # --- Xbox Process Control ---
    def start_xbox_interface(self):
        if self.xbox_process is None:
            # Start the Xbox polling process.
            from SupportClasses.XboxControl import xbox_polling_worker  # import here if needed
            self.xbox_process = Process(target=xbox_polling_worker, args=(self.xbox_queue,))
            self.xbox_process.start()
            print("Xbox polling process started.")
        else:
            print("Xbox polling process already running.")
    
    def stop_xbox_interface(self):
        if self.xbox_process:
            self.xbox_process.terminate()
            self.xbox_process.join()
            self.xbox_process = None
            print("Xbox polling process stopped.")
        else:
            print("Xbox polling process not running.")
    
    # --- Stage Devices Control ---
    def start_stage_devices(self):
        if self.zp_stage is None:
            self.zp_stage = ZPStageManager(simulate=self.simulatezp)
            print("ZP stage started.")
        else:
            print("ZP stage already running.")
        if self.xy_stage is None:
            self.xy_stage = XYStageManager(simulate=self.simulatexy)
            print("XY stage started.")
        else:
            print("XY stage already running.")
        if self.stage_handler is None:
            self.stage_handler = StageHandler(self.processor, self.zp_stage, self.xy_stage)
            print("StageHandler started.")
        else:
            print("StageHandler already running.")
    
    def stop_stage_devices(self):
        if self.stage_handler:
            try:
                self.stage_handler.stop()
                print("StageHandler stopped.")
            except Exception as e:
                print("Error stopping StageHandler:", e)
            finally:
                self.stage_handler = None
        else:
            print("StageHandler not running.")
        if self.zp_stage:
            try:
                self.zp_stage.stop()
                print("ZP stage stopped.")
            except Exception as e:
                print("Error stopping ZP stage:", e)
            finally:
                self.zp_stage = None
        else:
            print("ZP stage not running.")
        if self.xy_stage:
            try:
                self.xy_stage.stop()
                print("XY stage stopped.")
            except Exception as e:
                print("Error stopping XY stage:", e)
            finally:
                self.xy_stage = None
        else:
            print("XY stage not running.")
            
    # --- handle start stop for external controllers ---
    def handle_control_xbox(self, **kwargs):
        action = kwargs.get("action")
        if action == "start":
            self.start_xbox_interface()
        elif action == "stop":
            self.stop_xbox_interface()
        else:
            print(f"Unknown action for Xbox control: {action}")
    
    def handle_control_stages(self, **kwargs):
        action = kwargs.get("action")
        if action == "start":
            self.start_stage_devices()
        elif action == "stop":
            self.stop_stage_devices()
        else:
            print(f"Unknown action for Stage control: {action}")

    # --- requests ---
    def get_stage_info(self):
        if self.stage_handler is not None:
            return self.stage_handler.get_stage_info()
        else:
            print("StageHandler is not running.")
            return {"ZP": {}, "XY": {}}


