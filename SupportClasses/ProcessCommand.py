from multiprocessing import Process, Queue
import threading
import queue
import sys
import time
import math

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
        self.XYUPDATE_INTERVAL = 1.0
        self.ZUPDATE_INTERVAL = 0.5
        self.POS_UPDATE_INTERVAL = 0.5623  # Polling interval for updating positions

        self.processor = processor
        self.zp_stage = zp_stage
        self.xy_stage = xy_stage
        self._running = True
        # Stages are off by default.
        self._zp_running = False
        self._xy_running = False

        # State dictionaries storing full information for each axis.
        self.zp_state = {
            "X": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "Y": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "Z": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "E": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
        }
        self.xy_state = {
            "x": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "y": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "z": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
        }

        # Register command handlers.
        self.processor.register_handler("move_axis_1mm", self.handle_move_axis_1mm)
        self.processor.register_handler("set_axis_deactivation", self.handle_set_axis_deactivation)
        self.processor.register_handler("move_x_at_velocity", self.update_x_velocity)
        self.processor.register_handler("move_y_at_velocity", self.update_y_velocity)
        self.processor.register_handler("move_z_at_velocity", self.update_z_velocity)
        self.processor.register_handler("move_e_at_velocity", self.update_e_velocity)
        self.processor.register_handler("move_stage_at_velocity", self.update_xy_velocity)
        self.processor.register_handler("control_stage", self.handle_control_stage)

        # Start background threads.
        self.zp_thread = threading.Thread(target=self._zp_stage_loop, name="ZPStageHandlerThread", daemon=True)
        self.xy_thread = threading.Thread(target=self._xy_stage_loop, name="XYStageHandlerThread", daemon=True)
        self.pos_thread = threading.Thread(target=self._update_positions_loop, name="StagePositionUpdateThread", daemon=True)
        self.zp_thread.start()
        self.xy_thread.start()
        self.pos_thread.start()

    # ----- Velocity Update Methods for ZP Stage -----
    def update_x_velocity(self, *args, **kwargs):
        velocity = self._extract_velocity(*args, **kwargs)
        self.zp_state["X"]["velocity"] = velocity
        self.zp_state["X"]["active"] = (velocity != 0)

    def update_y_velocity(self, *args, **kwargs):
        velocity = self._extract_velocity(*args, **kwargs)
        self.zp_state["Y"]["velocity"] = velocity
        self.zp_state["Y"]["active"] = (velocity != 0)

    def update_z_velocity(self, *args, **kwargs):
        velocity = self._extract_velocity(*args, **kwargs)
        self.zp_state["Z"]["velocity"] = velocity
        self.zp_state["Z"]["active"] = (velocity != 0)

    def update_e_velocity(self, *args, **kwargs):
        velocity = self._extract_velocity(*args, **kwargs)
        self.zp_state["E"]["velocity"] = velocity
        self.zp_state["E"]["active"] = (velocity != 0)

    # ----- Velocity Update for XY Stage -----
    def update_xy_velocity(self, *args, **kwargs):
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = tuple(args[0])
            else:
                velocity = tuple(args)
        else:
            return
        if len(velocity) >= 3:
            vx, vy, vz = velocity[0], velocity[1], velocity[2]
        else:
            vx, vy, vz = velocity[0], velocity[1], 0.0
        self.xy_state["x"]["velocity"] = vx
        self.xy_state["y"]["velocity"] = vy
        self.xy_state["z"]["velocity"] = vz
        self.xy_state["x"]["active"] = (vx != 0)
        self.xy_state["y"]["active"] = (vy != 0)
        self.xy_state["z"]["active"] = (vz != 0)

    def _extract_velocity(self, *args, **kwargs):
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = args[0]
            else:
                velocity = args
        else:
            return 0.0
        try:
            return float(velocity)
        except Exception:
            return 0.0

    # ----- Position Polling and State Update -----
    def _update_positions_loop(self):
        while self._running:
            zp_positions = self.zp_stage.get_current_position()  # e.g., (x, y, z, e)
            if zp_positions and len(zp_positions) >= 4:
                self.zp_state["X"]["position"] = zp_positions[0]
                self.zp_state["Y"]["position"] = zp_positions[1]
                self.zp_state["Z"]["position"] = zp_positions[2]
                self.zp_state["E"]["position"] = zp_positions[3]
            xy_positions = self.xy_stage.get_current_position()  # e.g., (x, y, z)
            if xy_positions and len(xy_positions) >= 3:
                self.xy_state["x"]["position"] = xy_positions[0]
                self.xy_state["y"]["position"] = xy_positions[1]
                self.xy_state["z"]["position"] = xy_positions[2]
            time.sleep(self.POS_UPDATE_INTERVAL)

    # ----- Stage Move Loops -----
    def _zp_stage_loop(self):
        while self._running:
            if self._zp_running:
                if (self.zp_state["X"]["velocity"] != 0 or 
                    self.zp_state["Y"]["velocity"] != 0 or 
                    self.zp_state["Z"]["velocity"] != 0 or 
                    self.zp_state["E"]["velocity"] != 0):
                    self.send_zp_move_command()
            time.sleep(self.ZUPDATE_INTERVAL)

    def _xy_stage_loop(self):
        last_xy_velocity = (None, None, None)
        while self._running:
            if self._xy_running:
                vx = self.xy_state["x"]["velocity"]
                vy = self.xy_state["y"]["velocity"]
                vz = self.xy_state["z"]["velocity"]
                current_xy_velocity = (vx, vy, vz)
                if current_xy_velocity != last_xy_velocity:
                    self.xy_stage.move_stage_at_velocity(vx, vy)
                    last_xy_velocity = current_xy_velocity
            time.sleep(self.XYUPDATE_INTERVAL)

    def send_zp_move_command(self):
        print("Sending ZP move command")
        dt = self.ZUPDATE_INTERVAL
        vx = self.zp_state["X"]["velocity"]
        vy = self.zp_state["Y"]["velocity"]
        vz = self.zp_state["Z"]["velocity"]
        ve = self.zp_state["E"]["velocity"]
        dx = vx * dt
        dy = vy * dt
        dz = vz * dt
        de = ve * dt
        distance = math.sqrt(dx**2 + dy**2 + dz**2 + de**2)
        if distance == 0:
            return
        feedrate = (distance / dt) * 60
        axes = {'X': dx, 'Y': dy, 'Z': dz, 'E': de}
        self.zp_stage.movecommand(axes, feedrate)

    # ----- Command Handlers for GUI Commands -----
    def handle_move_axis_1mm(self, **kwargs):
        stage = kwargs.get("stage")
        axis = kwargs.get("axis")
        distance = kwargs.get("distance", 1)
        if stage == "ZP":
            if axis in self.zp_state:
                axes = {axis: distance}
                self.zp_stage.movecommand(axes, feedrate=60)
                print(f"Moving {axis} axis of ZP stage by {distance}mm")
        elif stage == "XY":
            if axis in self.xy_state:
                if axis == "x":
                    self.xy_stage.move_stage_at_velocity(distance, 0)
                elif axis == "y":
                    self.xy_stage.move_stage_at_velocity(0, distance)
                elif axis == "z":
                    print("Z axis move not supported for XY stage")
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

    def stop(self):
        self._running = False
        self.zp_thread.join()
        self.xy_thread.join()
        self.pos_thread.join()


class AppController:
    def __init__(self):
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
        
        # Register control handler for Xbox commands.
        self.processor.register_handler("control_xbox", self.handle_control_xbox)
        
        # Register a handle to start the stage devices.
        self.processor.register_handler("control_stage", self.handle_control_stages)

        # Register a simple debug handler that prints debug messages.
        self.processor.register_handler("debug", self.debug_handler)
        
        
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
    # --- Stage Devices Control (unchanged from previous example) ---
    def start_stage_devices(self):
        if self.zp_stage is None:
            self.zp_stage = ZPStageManager(simulate=True)
            print("ZP stage started.")
        else:
            print("ZP stage already running.")
        if self.xy_stage is None:
            self.xy_stage = XYStageManager(simulate=True)
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
    
    def restart_stage_devices(self):
        self.stop_stage_devices()
        self.start_stage_devices()
        print("Stage devices restarted successfully.")
        
    def get_stage_info(self):
        if self.stage_handler is not None:
            return self.stage_handler.get_stage_info()
        else:
            print("StageHandler is not running.")
            return {"ZP": {}, "XY": {}}
    
    # --- Control functions for stages (as before) ---
    def handle_control_xbox(self, **kwargs):
        action = kwargs.get("action")
        if action == "start":
            self.start_xbox_interface()
        elif action == "stop":
            self.stop_xbox_interface()
        else:
            print(f"Unknown action for Xbox control: {action}")
    
    # --- Stage Devices Control ---
    def handle_control_stages(self, **kwargs):
        action = kwargs.get("action")
        if action == "start":
            self.start_stage_devices()
        elif action == "stop":
            self.stop_stage_devices()
        else:
            print(f"Unknown action for Stage control: {action}")
    
    def start_stage_devices(self):
        if self.zp_stage is None:
            self.zp_stage = ZPStageManager(simulate=True)
            print("ZP stage started.")
        else:
            print("ZP stage already running.")
        if self.xy_stage is None:
            self.xy_stage = XYStageManager(simulate=True)
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

    # --- requests ---
    def get_stage_info(self):
        if self.stage_handler is not None:
            return self.stage_handler.get_stage_info()
        else:
            print("StageHandler is not running.")
            return {"ZP": {}, "XY": {}}

    # --- Application Lifecycle ---
    def start(self):
        self.app = QApplication(sys.argv)
        # Do not automatically start Xbox or stage devices since toggles are off.
        exit_code = self.app.exec()
        self.shutdown()
        sys.exit(exit_code)

    def shutdown(self):
        print("AppController: Shutting down application.")
        if self.xbox_interface:
            self.stop_xbox_interface()
        if self.stage_handler or self.zp_stage or self.xy_stage:
            self.stop_stage_devices()
        self.processor.stop()



