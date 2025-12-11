"""
Main controller using existing project architecture:
- XboxControl.py: Xbox polling in separate process
- ProcessCommand.py: Command routing via Processor
- DeviceInterface.py: Stage communication
"""

from multiprocessing import Process, Queue
import threading
import time
import math

from SupportClasses.XboxControl import xbox_polling_worker
from SupportClasses.ProcessCommand import Processor
from SupportClasses.DeviceInterface import XYStageManager, ZPStageManager


class XboxQueuePoller:
    """
    Polls the Xbox queue in a thread and dispatches to Processor.
    (Replaces Qt-based XboxPoller since we're not running Qt event loop)
    """
    def __init__(self, queue, processor):
        self.queue = queue
        self.processor = processor
        self._running = False
        self._thread = None
    
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def _poll_loop(self):
        while self._running:
            while not self.queue.empty():
                try:
                    msg = self.queue.get_nowait()
                except:
                    break
                
                if "debug" in msg:
                    # Only print connection messages, not polling spam
                    debug_msg = msg["debug"]
                    if "connect" in debug_msg.lower() or "found" in debug_msg.lower():
                        print(f"[Xbox] {debug_msg}")
                elif "button" in msg:
                    self.processor.add_command(msg["command"], button=msg["button"])
                elif "axis" in msg:
                    self.processor.add_command(msg["command"], axis=msg["axis"], average=msg["average"])
                elif "dpad" in msg:
                    self.processor.add_command(msg["command"], direction=msg["dpad"])
            
            time.sleep(0.02)


class ZPJogHandler:
    """
    Handles ZP stage jogging in its own thread.
    """
    def __init__(self, processor, zp_stage):
        self.processor = processor
        self.stage = zp_stage
        
        self.vel_z = 0.0
        self.vel_p1 = 0.0
        self.vel_p2 = 0.0
        self.vel_p3 = 0.0
        self.lock = threading.Lock()
        
        self.segment_time = 0.12
        self.z_speed = 0.5
        self.p_speed = 0.5
        self.max_speed = 1.0
        
        self._was_moving = False
        self._running = False
        self._thread = None
        
        # Register handlers
        self.processor.register_handler("move_z_at_velocity", self._handle_z_velocity)
        self.processor.register_handler("move_p1_at_velocity", self._handle_p1_velocity)
        self.processor.register_handler("move_p2_at_velocity", self._handle_p2_velocity)
        self.processor.register_handler("move_p3_at_velocity", self._handle_p3_velocity)
        self.processor.register_handler("increment_zspeed_up", self._increment_z_up)
        self.processor.register_handler("increment_zspeed_down", self._increment_z_down)
        self.processor.register_handler("increment_pspeed_up", self._increment_p_up)
        self.processor.register_handler("increment_pspeed_down", self._increment_p_down)
    
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._jog_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def _extract_velocity(self, *args, **kwargs):
        if "average" in kwargs:
            val = kwargs["average"]
            if isinstance(val, (list, tuple)):
                return val[1] if len(val) > 1 else val[0]
            return val
        return 0.0
    
    def _handle_z_velocity(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self.lock:
            self.vel_z = max(-self.max_speed, min(self.max_speed, raw * self.z_speed))
    
    def _handle_p1_velocity(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self.lock:
            self.vel_p1 = max(-self.max_speed, min(self.max_speed, raw * self.p_speed))
    
    def _handle_p2_velocity(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self.lock:
            self.vel_p2 = max(-self.max_speed, min(self.max_speed, raw * self.p_speed))
    
    def _handle_p3_velocity(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self.lock:
            self.vel_p3 = max(-self.max_speed, min(self.max_speed, raw * self.p_speed))
    
    def _increment_z_up(self, *args, **kwargs):
        self.z_speed = min(self.z_speed * 2, 100)
        print(f"Z speed: {self.z_speed}")
    
    def _increment_z_down(self, *args, **kwargs):
        self.z_speed = max(self.z_speed / 2, 0.1)
        print(f"Z speed: {self.z_speed}")
    
    def _increment_p_up(self, *args, **kwargs):
        self.p_speed = min(self.p_speed * 2, 100)
        print(f"P speed: {self.p_speed}")
    
    def _increment_p_down(self, *args, **kwargs):
        self.p_speed = max(self.p_speed / 2, 0.1)
        print(f"P speed: {self.p_speed}")
    
    def _jog_loop(self):
        while self._running:
            with self.lock:
                vz, vp1, vp2, vp3 = self.vel_z, self.vel_p1, self.vel_p2, self.vel_p3
            
            is_moving = any(abs(v) > 0.001 for v in [vz, vp1, vp2, vp3])
            
            if is_moving and not self._was_moving:
                print(f"[ZP] Start: z={vz:.2f} p1={vp1:.2f} p2={vp2:.2f} p3={vp3:.2f}")
            elif not is_moving and self._was_moving:
                print("[ZP] Stop")
            self._was_moving = is_moving
            
            if not is_moving:
                time.sleep(0.01)
                continue
            
            dz = -vz * self.segment_time
            dp1 = vp1 * self.segment_time
            dp2 = vp2 * self.segment_time
            dp3 = vp3 * self.segment_time
            
            combined = math.sqrt(vz**2 + vp1**2 + vp2**2 + vp3**2)
            feedrate = max(combined * 60, 1)
            
            axes = {'X': dz, 'Y': dp1, 'Z': dp2, 'E': dp3}
            self.stage.move_relative(axes, feedrate)
            
            time.sleep(self.segment_time)


class XYJogHandler:
    """
    Handles XY stage jogging in its own thread.
    """
    def __init__(self, processor, xy_stage):
        self.processor = processor
        self.stage = xy_stage
        
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.lock = threading.Lock()
        
        self.xy_speed = 100.0
        self.max_speed = 5000.0
        self.update_interval = 0.1
        
        self._was_moving = False
        self._running = False
        self._thread = None
        
        self.processor.register_handler("move_stage_at_velocity", self._handle_xy_velocity)
        self.processor.register_handler("increment_xyspeed_up", self._increment_up)
        self.processor.register_handler("increment_xyspeed_down", self._increment_down)
    
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._jog_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        self.stage.move_stage_at_velocity(0, 0)
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def _handle_xy_velocity(self, *args, **kwargs):
        if "average" in kwargs:
            val = kwargs["average"]
            if isinstance(val, (list, tuple)) and len(val) >= 2:
                vx, vy = val[0], val[1]
            else:
                vx, vy = 0, 0
        else:
            vx, vy = 0, 0
        
        with self.lock:
            self.vel_x = max(-self.max_speed, min(self.max_speed, vx * self.xy_speed))
            self.vel_y = max(-self.max_speed, min(self.max_speed, vy * self.xy_speed))
    
    def _increment_up(self, *args, **kwargs):
        self.xy_speed = min(self.xy_speed * 2, 10000)
        print(f"XY speed: {self.xy_speed}")
    
    def _increment_down(self, *args, **kwargs):
        self.xy_speed = max(self.xy_speed / 2, 1)
        print(f"XY speed: {self.xy_speed}")
    
    def _jog_loop(self):
        while self._running:
            with self.lock:
                vx, vy = self.vel_x, self.vel_y
            
            is_moving = abs(vx) > 0.001 or abs(vy) > 0.001
            
            if is_moving and not self._was_moving:
                print(f"[XY] Start: x={vx:.1f} y={vy:.1f}")
            elif not is_moving and self._was_moving:
                print("[XY] Stop")
            self._was_moving = is_moving
            
            self.stage.move_stage_at_velocity(vx, vy)
            time.sleep(self.update_interval)


def main():
    # ========== CONFIGURATION ==========
    SIMULATE_XY = False
    SIMULATE_ZP = False
    # ===================================
    
    print("=" * 50)
    print("Xbox Stage Controller")
    print("=" * 50)
    print(f"XY: {'SIM' if SIMULATE_XY else 'REAL'}")
    print(f"ZP: {'SIM' if SIMULATE_ZP else 'REAL'}")
    print("=" * 50)
    
    # Create processor
    processor = Processor()
    
    # Create stages
    xy_stage = XYStageManager(simulate=SIMULATE_XY)
    zp_stage = ZPStageManager(simulate=SIMULATE_ZP)
    
    # Create and start jog handlers
    zp_jog = ZPJogHandler(processor, zp_stage)
    xy_jog = XYJogHandler(processor, xy_stage)
    zp_jog.start()
    xy_jog.start()
    
    # Create Xbox queue and process
    xbox_queue = Queue()
    xbox_process = Process(target=xbox_polling_worker, args=(xbox_queue,))
    xbox_process.start()
    
    # Create and start queue poller
    xbox_poller = XboxQueuePoller(xbox_queue, processor)
    xbox_poller.start()
    
    print("Ready. Press Ctrl+C to exit.\n")
    
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        xbox_poller.stop()
        xbox_process.terminate()
        xbox_process.join(timeout=1.0)
        zp_jog.stop()
        xy_jog.stop()
        xy_stage.stop()
        zp_stage.stop()
        processor.stop()
        print("Done.")


if __name__ == "__main__":
    main()