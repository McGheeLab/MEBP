import threading
import queue
import sys
from PyQt5 import QtWidgets
from DeviceInterface import ZPStageManager, XYStageManager
from XboxControl import XboxControllerInterface

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


class AppController:
    def __init__(self):
        # Instantiate the communication processor.
        self.processor = Processor()

        # Instantiate device managers.
        self.zp_stage = ZPStageManager()
        self.xy_stage = XYStageManager()

        # Register command handler(s).
        self.processor.register_handler("move_stage_at_velocity", self.handle_move_stage)

        # Instantiate the controller interface (e.g., Xbox controller) passing the processor.
        self.xbox_interface = XboxControllerInterface(self.processor)

    def handle_move_stage(self, velocity):
        """
        Command handler for moving the stage.
        Called when a 'move_stage_at_velocity' command is received.
        """
        print(f"AppController: Handling move_stage_at_velocity with velocity {velocity}")
        self.xy_stage.move_stage_at_velocity(velocity)

    def handle_move_z(self, velocity):
        """
        Command handler for moving the Z stage.
        Called when a 'move_z_at_velocity' command is received.
        """
        print(f"AppController: Handling move_z_at_velocity with velocity {velocity}")
        self.zp_stage.move_stage_at_velocity(velocity)
    
    def start(self):
        """
        Set up and run the Qt application.
        """
        self.app = QtWidgets.QApplication(sys.argv)
        self.xbox_interface.show()
        exit_code = self.app.exec_()
        self.shutdown()
        sys.exit(exit_code)

    def shutdown(self):
        """
        Clean up and shutdown the processor and any other resources.
        """
        print("AppController: Shutting down application.")
        self.processor.stop()


