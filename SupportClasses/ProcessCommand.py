import threading
import queue
import sys
from PyQt5 import QtWidgets
from SupportClasses.DeviceInterface import ZPStageManager, XYStageManager
from SupportClasses.XboxControl import XboxControllerInterface

import threading
import time
import math

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

# manage the application and its components.
class AppController:
    def __init__(self):
        # Instantiate the communication processor.
        self.processor = Processor()

        # Instantiate device managers.
        self.zp_stage = ZPStageManager(simulate=True)
        self.xy_stage = XYStageManager(simulate=True)

        # Instantiate the StageHandler, which registers its own command handlers.
        self.stage_handler = StageHandler(self.processor, self.zp_stage, self.xy_stage)

        # Instantiate the controller interface (e.g., Xbox controller) passing the processor.
        self.xbox_interface = XboxControllerInterface(self.processor)

    def start(self):
        """
        Set up and run the Qt application.
        """
        self.app = QtWidgets.QApplication(sys.argv)
        self.xbox_interface.run()
        exit_code = self.app.exec_()
        self.shutdown()
        sys.exit(exit_code)

    def shutdown(self):
        """
        Clean up and shutdown the processor and stage handler.
        """
        print("AppController: Shutting down application.")
        self.xbox_interface.stop()
        self.stage_handler.stop()
        self.processor.stop()

# handle continuous communication with the ZP and XY stages.
class StageHandler:
    """
    Handles continuous communication with both the ZP and XY stages.
    
    For the ZP stage:
      - Every ZUPDATE_INTERVAL seconds, sends a move command using the current velocities,
        but only if movement is required.
        
    For the XY stage:
      - Every XYUPDATE_INTERVAL seconds, if a new velocity (tuple (vx, vy)) is received that differs 
        from the last command, sends a move command.
        
    The StageHandler registers two command handlers with the Processor:
      - "move_z_at_velocity" for Z stage commands.
      - "move_stage_at_velocity" for XY stage commands.
    """
    def __init__(self, processor, zp_stage, xy_stage):
        self.XYUPDATE_INTERVAL = 1.0
        self.ZUPDATE_INTERVAL = 0.5

        self.processor = processor
        self.zp_stage = zp_stage
        self.xy_stage = xy_stage
        
        self._running = True

        # Store the current commanded velocities.
        # For Z stage, we expect separate velocities for X, Y, Z, and E.
        self.current_x_velocity = 0.0
        self.current_y_velocity = 0.0
        self.current_z_velocity = 0.0
        self.current_e_velocity = 0.0
        # This variable is no longer used directly because feedrate is computed automatically.
        self.current_zp_feedrate = 0.0
        
        # For XY stage, we expect a tuple (vx, vy).
        self.current_xy_velocity = None
        self.last_xy_velocity = None  # Track the last sent velocity to avoid duplicates

        # Register command handlers with the Processor.
        self.processor.register_handler("move_x_at_velocity", self.update_x_velocity)
        self.processor.register_handler("move_y_at_velocity", self.update_y_velocity)
        self.processor.register_handler("move_z_at_velocity", self.update_z_velocity)
        self.processor.register_handler("move_e_at_velocity", self.update_e_velocity)
        self.processor.register_handler("move_stage_at_velocity", self.update_xy_velocity)

        # Start dedicated threads for each stage.
        self.zp_thread = threading.Thread(target=self._zp_stage_loop, name="ZPStageHandlerThread", daemon=True)
        self.xy_thread = threading.Thread(target=self._xy_stage_loop, name="XYStageHandlerThread", daemon=True)
        self.zp_thread.start()
        self.xy_thread.start()

    ################## Functions Updating Stage Parameters ##################
    
    def update_x_velocity(self, *args, **kwargs):
        """
        Processor command handler for updating the Z stage velocity.
        
        This method now accepts keyword arguments so that when a command is sent
        with an 'average' keyword (for example from the XboxControllerInterface), 
        it can extract the new velocity.
        
        Expected behavior:
          - If the 'average' keyword is provided, use its value as the new velocity.
          - Otherwise, if positional arguments are provided, treat them as the velocity.
        """
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = tuple(args[0])
            else:
                velocity = tuple(args)
        else:
            # If no velocity is provided, do nothing.
            return

        self.current_x_velocity = velocity

    def update_y_velocity(self, *args, **kwargs):
        """
        Processor command handler for updating the Y stage velocity.
        
        This method now accepts keyword arguments so that when a command is sent
        with an 'average' keyword (for example from the XboxControllerInterface), 
        it can extract the new velocity.
        
        Expected behavior:
          - If the 'average' keyword is provided, use its value as the new velocity.
          - Otherwise, if positional arguments are provided, treat them as the velocity.
        """
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = tuple(args[0])
            else:
                velocity = tuple(args)
        else:
            # If no velocity is provided, do nothing.
            return

        self.current_y_velocity = velocity

    def update_z_velocity(self, *args, **kwargs):
        """
        Processor command handler for updating the Z stage velocity.
        
        This method now accepts keyword arguments so that when a command is sent
        with an 'average' keyword (for example from the XboxControllerInterface), 
        it can extract the new velocity.
        
        Expected behavior:
          - If the 'average' keyword is provided, use its value as the new velocity.
          - Otherwise, if positional arguments are provided, treat them as the velocity.
        """
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = tuple(args[0])
            else:
                velocity = tuple(args)
        else:
            # If no velocity is provided, do nothing.
            return

        self.current_z_velocity = velocity

    def update_e_velocity(self, *args, **kwargs):
        """
        Processor command handler for updating the E stage velocity.
        
        This method now accepts keyword arguments so that when a command is sent
        with an 'average' keyword (for example from the XboxControllerInterface), 
        it can extract the new velocity.
        
        Expected behavior:
          - If the 'average' keyword is provided, use its value as the new velocity.
          - Otherwise, if positional arguments are provided, treat them as the velocity.
        """
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = tuple(args[0])
            else:
                velocity = tuple(args)
        else:
            # If no velocity is provided, do nothing.
            return

        self.current_e_velocity = velocity

    def update_xy_velocity(self, *args, **kwargs):
        """
        Processor command handler for updating the XY stage velocity.
        
        This method now accepts keyword arguments so that when a command is sent
        with an 'average' keyword (for example from the XboxControllerInterface), 
        it can extract the new velocity.
        
        Expected behavior:
          - If the 'average' keyword is provided, use its value as the new velocity.
          - Otherwise, if positional arguments are provided, treat them as the velocity.
        """
        if "average" in kwargs:
            velocity = kwargs["average"]
        elif args:
            if len(args) == 1 and isinstance(args[0], (tuple, list)):
                velocity = tuple(args[0])
            else:
                velocity = tuple(args)
        else:
            # If no velocity is provided, do nothing.
            return

        self.current_xy_velocity = velocity

    ################## Stage control loops ##################

    def send_zp_move_command(self):
        """
        Computes the move distances for the ZP stage based on the current velocities
        and the ZUPDATE_INTERVAL, calculates an appropriate feedrate such that the G-code
        move completes in exactly ZUPDATE_INTERVAL seconds, and then sends the move command.

        The feedrate is computed using the formula:
          feedrate = (distance / ZUPDATE_INTERVAL) * 60
        where 'distance' is the Euclidean norm of the movement distances for the axes.
        """
        dt = self.ZUPDATE_INTERVAL

        # Safely convert each velocity to a float.
        vx = self.safe_float(self.current_x_velocity)
        vy = self.safe_float(self.current_y_velocity)
        vz = self.safe_float(self.current_z_velocity)
        ve = self.safe_float(self.current_e_velocity)
        
        # Calculate commanded distances for each axis (distance = velocity * time)
        dx = vx * dt
        dy = vy * dt
        dz = vz * dt
        de = ve * dt

        # Compute the Euclidean distance of the move.
        distance = math.sqrt(dx**2 + dy**2 + dz**2 + de**2)

        # If there is no movement, skip sending a command.
        if distance == 0:
            return

        # Compute the feedrate so that the move completes in dt seconds.
        # (distance / dt) gives mm/sec; multiplying by 60 converts to mm/min.
        feedrate = (distance / dt) * 60

        # Build the dictionary of axes with their computed move distances.
        axes = {
            'X': dx,
            'Y': dy,
            'Z': dz,
            'E': de
        }

        # Send the move command using the ZPStageManager's movecommand function.
        self.zp_stage.movecommand(axes, feedrate)

    def _zp_stage_loop(self):
        """
        Continuously sends move commands to the ZP stage every ZUPDATE_INTERVAL seconds,
        if there is any movement required.
        """
        while self._running:
            # Optionally check if any of the velocities are nonzero.
            if (self.current_x_velocity != 0 or 
                self.current_y_velocity != 0 or 
                self.current_z_velocity != 0 or 
                self.current_e_velocity != 0):
                self.send_zp_move_command()
            time.sleep(self.ZUPDATE_INTERVAL)

    def _xy_stage_loop(self):
        """
        Continuously checks for XY velocity updates every XYUPDATE_INTERVAL seconds and sends a move command
        only if the new velocity differs from the last sent value.
        """
        while self._running:
            if (self.current_xy_velocity is not None and 
                self.current_xy_velocity != self.last_xy_velocity):
                vx, vy = self.current_xy_velocity
                self.xy_stage.move_stage_at_velocity(vx, vy)
                self.last_xy_velocity = self.current_xy_velocity
            time.sleep(self.XYUPDATE_INTERVAL)

    ################## Utility functions ##################
    
    def safe_float(self,value):
        """
        Convert value to float.
        - If value is an int or float, return it as float.
        - If value is a non-empty list or tuple, return the float of its second element.
        - Otherwise, return 0.0.
        """
        if isinstance(value, (int, float)):
            return float(value)
        elif isinstance(value, (list, tuple)) and len(value) > 0:
            return float(value[1]) # Return the second element of the list or tuple up down on stick
        else:
            return 0.0
    
    def stop(self):
        """
        Signals the StageHandler threads to stop and waits for them to exit.
        """
        self._running = False
        self.zp_thread.join()
        self.xy_thread.join()

