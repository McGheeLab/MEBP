"""
Main controller script for Xbox-controlled XY and ZP stages.
Ensures smooth motion by processing only the latest commands (no backlog).
"""

from multiprocessing import Process, Queue
import threading
import time
import sys

from SupportClasses.XboxControl import xbox_polling_worker
from SupportClasses.DeviceInterface import XYStageManager, ZPStageManager
from SupportClasses.ProcessCommand import Processor, StageHandler

# Try to import Qt components for the timer-based polling
try:
    from qt_core import QApplication, QTimer, QObject
    HAS_QT = True
except ImportError:
    HAS_QT = False
    print("Qt not available, using threaded polling instead")


class SmoothXboxPoller:
    """
    Polls Xbox command queue and dispatches only the latest commands.
    Prevents command backlog by draining the queue and keeping only
    the most recent command per type before dispatching.
    """
    def __init__(self, queue, processor, poll_interval_ms=50):
        self.queue = queue
        self.processor = processor
        self.poll_interval = poll_interval_ms / 1000.0
        self._running = False
        self._thread = None
    
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        print("SmoothXboxPoller started")
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        print("SmoothXboxPoller stopped")
    
    def _poll_loop(self):
        while self._running:
            self._process_queue()
            time.sleep(self.poll_interval)
    
    def _process_queue(self):
        """
        Drain the queue and keep only the latest command per command type.
        This prevents backlog - if the user moved the joystick 10 times since
        last poll, we only care about the final position.
        """
        # Collect all pending messages, keeping only the latest per command type
        latest_commands = {}
        
        while not self.queue.empty():
            try:
                msg = self.queue.get_nowait()
            except:
                break
            
            # Handle debug messages immediately
            if "debug" in msg:
                print(f"[Xbox Debug] {msg['debug']}")
                continue
            
            # For other commands, keep only the latest per command type
            cmd = msg.get("command")
            if cmd:
                latest_commands[cmd] = msg
        
        # Now dispatch only the latest command for each type
        for cmd, msg in latest_commands.items():
            if "button" in msg:
                self.processor.add_command(cmd, button=msg["button"])
            elif "axis" in msg:
                self.processor.add_command(cmd, axis=msg["axis"], average=msg["average"])
            elif "dpad" in msg:
                self.processor.add_command(cmd, direction=msg["dpad"])


class XboxStageController:
    """
    Main controller that coordinates Xbox input with stage movement.
    """
    def __init__(self, simulate_xy=True, simulate_zp=True):
        self.simulate_xy = simulate_xy
        self.simulate_zp = simulate_zp
        
        # Core components
        self.processor = Processor()
        self.xbox_queue = Queue()
        self.xbox_process = None
        self.xbox_poller = None
        
        # Stage managers
        self.xy_stage = None
        self.zp_stage = None
        self.stage_handler = None
        
        # Register debug handler
        self.processor.register_handler("debug", self._debug_handler)
        
        # ZP velocity update timer for smooth motion
        self._zp_timer_running = False
        self._zp_timer_thread = None
    
    def _debug_handler(self, *args, **kwargs):
        msg = kwargs.get("message", "")
        print(f"[Debug] {msg}")
    
    def start(self):
        """Initialize and start all components."""
        print("=" * 50)
        print("Starting Xbox Stage Controller")
        print("=" * 50)
        
        # Initialize stages
        print("\nInitializing stages...")
        self.xy_stage = XYStageManager(simulate=self.simulate_xy)
        print(f"  XY Stage: {'SIMULATED' if self.simulate_xy else 'HARDWARE'}")
        
        self.zp_stage = ZPStageManager(simulate=self.simulate_zp)
        print(f"  ZP Stage: {'SIMULATED' if self.simulate_zp else 'HARDWARE'}")
        
        # Initialize stage handler
        self.stage_handler = StageHandler(self.processor, self.zp_stage, self.xy_stage)
        print("  StageHandler initialized")
        
        # Start ZP velocity update loop for smooth continuous motion
        self._start_zp_velocity_loop()
        
        # Start Xbox polling process
        print("\nStarting Xbox controller...")
        self.xbox_process = Process(target=xbox_polling_worker, args=(self.xbox_queue,))
        self.xbox_process.start()
        
        # Start the smooth poller (processes queue without backlog)
        self.xbox_poller = SmoothXboxPoller(self.xbox_queue, self.processor, poll_interval_ms=50)
        self.xbox_poller.start()
        
        print("\n" + "=" * 50)
        print("Controller ready! Use Xbox controller to move stages.")
        print("Press Ctrl+C to exit.")
        print("=" * 50 + "\n")
    
    def _start_zp_velocity_loop(self):
        """
        Start a background thread that periodically sends ZP move commands
        based on current velocity state. This ensures smooth continuous motion.
        """
        self._zp_timer_running = True
        self._zp_timer_thread = threading.Thread(target=self._zp_velocity_loop, daemon=True)
        self._zp_timer_thread.start()
    
    def _zp_velocity_loop(self):
        """
        Periodically send ZP movement commands based on current velocity.
        This is the 'just-in-time' approach - we send commands at regular
        intervals rather than queueing them up.
        """
        interval = self.stage_handler.ZUPDATE_INTERVAL if self.stage_handler else 0.333
        
        while self._zp_timer_running:
            if self.stage_handler:
                # Check if any ZP axis is active
                zp_active = any(
                    self.stage_handler.zp_state[axis]["active"] 
                    for axis in ["Z", "P1", "P2", "P3"]
                )
                if zp_active:
                    self.stage_handler.send_zp_move_command()
            
            time.sleep(interval)
    
    def stop(self):
        """Clean shutdown of all components."""
        print("\nShutting down...")
        
        # Stop ZP velocity loop
        self._zp_timer_running = False
        if self._zp_timer_thread:
            self._zp_timer_thread.join(timeout=1.0)
        
        # Stop Xbox poller
        if self.xbox_poller:
            self.xbox_poller.stop()
        
        # Stop Xbox process
        if self.xbox_process:
            self.xbox_process.terminate()
            self.xbox_process.join(timeout=2.0)
            print("  Xbox process stopped")
        
        # Stop stages (send zero velocity first)
        if self.xy_stage:
            try:
                self.xy_stage.move_stage_at_velocity(0, 0)
                time.sleep(0.1)
                self.xy_stage.stop()
                print("  XY stage stopped")
            except Exception as e:
                print(f"  Error stopping XY stage: {e}")
        
        if self.zp_stage:
            try:
                self.zp_stage.stop()
                print("  ZP stage stopped")
            except Exception as e:
                print(f"  Error stopping ZP stage: {e}")
        
        # Stop processor
        self.processor.stop()
        print("  Processor stopped")
        
        print("Shutdown complete.")
    
    def run(self):
        """Main run loop - keeps the program alive and shows status."""
        try:
            self.start()
            
            # Main loop - print status periodically
            while True:
                time.sleep(1.0)
                self._print_status()
                
        except KeyboardInterrupt:
            print("\n\nInterrupt received...")
        finally:
            self.stop()
    
    def _print_status(self):
        """Print current stage positions and velocities."""
        if not self.stage_handler:
            return
        
        # Get current positions
        x, y, f = self.stage_handler.get_XY_positions()
        z, p1, p2, p3 = self.stage_handler.get_ZP_positions()
        
        # Get current velocities from state
        vx = self.stage_handler.xy_state["x"]["velocity"]
        vy = self.stage_handler.xy_state["y"]["velocity"]
        vz = self.stage_handler.zp_state["Z"]["velocity"]
        vp1 = self.stage_handler.zp_state["P1"]["velocity"]
        
        # Only print if there's activity or periodically
        if any([vx, vy, vz, vp1]):
            print(f"XY: ({x:.1f}, {y:.1f}) vel=({vx:.1f}, {vy:.1f}) | "
                  f"Z: {z:.2f} vel={vz:.2f} | P1: {p1:.2f} vel={vp1:.2f}")


def main():
    """
    Main entry point. Configure simulation mode here.
    Set simulate_xy=False and simulate_zp=False for real hardware.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Xbox-controlled stage system")
    parser.add_argument("--real-xy", action="store_true", help="Use real XY stage hardware")
    parser.add_argument("--real-zp", action="store_true", help="Use real ZP stage hardware")
    parser.add_argument("--real", action="store_true", help="Use all real hardware")
    args = parser.parse_args()
    
    simulate_xy = not (args.real_xy or args.real)
    simulate_zp = not (args.real_zp or args.real)
    
    controller = XboxStageController(simulate_xy=simulate_xy, simulate_zp=simulate_zp)
    controller.run()


if __name__ == "__main__":
    main()