"""
Just-in-time Xbox Stage Controller.
Single-threaded pygame access to avoid threading issues.
"""

import pygame
import threading
import time
import json
import math

from SupportClasses.DeviceInterface import XYStageManager, ZPStageManager


class JsonMapping:
    """Loads and caches button mapping from JSON file."""
    def __init__(self, path="current_button_mapping.json"):
        self.path = path
        self.data = {"buttons": {}, "axes": {}, "dpad": {}}
        self.last_load = 0
        self.load()
    
    def load(self):
        try:
            with open(self.path, "r") as f:
                self.data = json.load(f)
            print(f"Loaded mapping: {self.path}")
        except Exception as e:
            print(f"Could not load mapping: {e}")
        self.last_load = time.time()
    
    def maybe_reload(self):
        if time.time() - self.last_load > 5:
            self.load()
    
    def get_button_cmd(self, idx):
        cmd = self.data.get("buttons", {}).get(str(idx))
        return cmd if cmd and cmd != "None" else None
    
    def get_dpad_cmd(self, direction):
        cmd = self.data.get("dpad", {}).get(direction)
        return cmd if cmd and cmd != "None" else None


class StageController:
    """Handles stage movement commands."""
    def __init__(self, simulate_xy=True, simulate_zp=True):
        self.xy_stage = XYStageManager(simulate=simulate_xy)
        self.zp_stage = ZPStageManager(simulate=simulate_zp)
        
        # Set higher max feedrate on ZP stage to allow smooth motion
        # Default is often too low which causes moves to complete too fast
        self.zp_stage.set_max_feedrate(500)  # mm/min for all axes
        
        # Speed multipliers (user input * speed = velocity in mm/s)
        self.xy_speed = 100.0
        self.z_speed = 5.0    # mm/s at full stick deflection
        self.p_speed = 5.0    # mm/s at full trigger
        
        self.max_xy_speed = 5000
        self.max_z_speed = 50   # mm/s max
        self.max_p_speed = 50   # mm/s max
        
        self.xy_speed_range = (1.0, 10000.0)
        self.z_speed_range = (0.1, 100.0)
        self.p_speed_range = (0.1, 100.0)
        
        self.xy_pos = (0.0, 0.0, 0.0)
        self.zp_pos = (0.0, 0.0, 0.0, 0.0)
    
    def update_xy(self, vx, vy):
        vx *= self.xy_speed
        vy *= self.xy_speed
        vx = max(-self.max_xy_speed, min(self.max_xy_speed, vx))
        vy = max(-self.max_xy_speed, min(self.max_xy_speed, vy))
        self.xy_stage.move_stage_at_velocity(vx, vy)
        self.xy_pos = self.xy_stage.get_current_position()
    
    def update_zp(self, vz, vp1, vp2, dt):
        """
        Update ZP stage with smooth motion by keeping printer buffer full.
        
        Strategy: Send small moves at high frequency. The printer board
        buffers commands, so by sending faster than execution, we keep
        2+ moves in the queue for seamless transitions.
        """
        # Calculate desired velocities in mm/s
        vel_z = vz * self.z_speed
        vel_p1 = vp1 * self.p_speed
        vel_p2 = vp2 * self.p_speed
        
        # Clamp velocities
        vel_z = max(-self.max_z_speed, min(self.max_z_speed, vel_z))
        vel_p1 = max(-self.max_p_speed, min(self.max_p_speed, vel_p1))
        vel_p2 = max(-self.max_p_speed, min(self.max_p_speed, vel_p2))
        
        # Calculate small distances for this short interval
        dz = vel_z * dt
        dp1 = vel_p1 * dt
        dp2 = vel_p2 * dt
        
        # Always send command (even zero) to keep buffer fed
        # Calculate feedrate from velocity
        combined_vel = math.sqrt(vel_z**2 + vel_p1**2 + vel_p2**2)
        
        if combined_vel > 0.001:
            feedrate = combined_vel * 60  # mm/s to mm/min
            feedrate = max(feedrate, 1)   # minimum feedrate
        else:
            feedrate = 60  # default feedrate for zero moves
        
        axes = {'X': dz, 'Y': dp1, 'Z': dp2, 'E': 0}
        self.zp_stage.move_relative(axes, feedrate)
        
        # Don't update position every cycle - too slow
        # self.zp_pos = self.zp_stage.get_current_position()
    
    def run_command(self, cmd):
        if cmd == "increment_zspeed_up":
            if self.z_speed < self.z_speed_range[1]:
                self.z_speed *= 2
            print(f"Z Speed = {self.z_speed}")
        elif cmd == "increment_zspeed_down":
            if self.z_speed > self.z_speed_range[0]:
                self.z_speed /= 2
            print(f"Z Speed = {self.z_speed}")
        elif cmd == "increment_pspeed_up":
            if self.p_speed < self.p_speed_range[1]:
                self.p_speed *= 2
            print(f"P Speed = {self.p_speed}")
        elif cmd == "increment_pspeed_down":
            if self.p_speed > self.p_speed_range[0]:
                self.p_speed /= 2
            print(f"P Speed = {self.p_speed}")
        elif cmd == "increment_xyspeed_up":
            if self.xy_speed < self.xy_speed_range[1]:
                self.xy_speed *= 2
            print(f"XY Speed = {self.xy_speed}")
        elif cmd == "increment_xyspeed_down":
            if self.xy_speed > self.xy_speed_range[0]:
                self.xy_speed /= 2
            print(f"XY Speed = {self.xy_speed}")
        elif cmd == "zero_needle_pos":
            print(f"Zeroed at XY={self.xy_pos} ZP={self.zp_pos}")
    
    def stop(self):
        self.xy_stage.move_stage_at_velocity(0, 0)
        self.xy_stage.stop()
        self.zp_stage.stop()


def main_loop(simulate_xy=True, simulate_zp=True):
    """
    Single-threaded main loop.
    All pygame calls happen in the main thread.
    Stage updates happen at defined intervals.
    """
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()
    
    count = pygame.joystick.get_count()
    print(f"Found {count} joystick(s)")
    
    if count == 0:
        print("No controller found! Exiting.")
        return
    
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")
    print(f"  Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}, Hats: {js.get_numhats()}")
    
    # Initialize components
    mapping = JsonMapping()
    stages = StageController(simulate_xy=simulate_xy, simulate_zp=simulate_zp)
    
    # Timing intervals
    xy_interval = 0.333
    zp_interval = 0.05  # 20Hz - fast updates to keep printer buffer full
    last_xy_update = 0
    last_zp_update = 0
    last_status = 0
    
    # Prime the ZP buffer with initial commands
    zp_buffer_primed = False
    
    # State tracking for edge detection
    prev_buttons = [False] * js.get_numbuttons()
    prev_dpad = (0, 0)
    
    deadzone = 0.15
    
    print("\nControls:")
    print("  Left Stick    -> XY movement")
    print("  Right Stick Y -> Z movement")
    print("  Left Trigger  -> P1 (withdraw)")
    print("  Right Trigger -> P2 (dispense)")
    print("\nPress Ctrl+C to exit.\n")
    
    try:
        while True:
            now = time.time()
            
            # Pump pygame events (MUST be in main thread)
            pygame.event.pump()
            
            # Reload mapping periodically
            mapping.maybe_reload()
            
            # --- Read controller state ---
            
            # Axes with deadzone
            axes = []
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                if abs(val) < deadzone:
                    val = 0.0
                axes.append(val)
            
            # Buttons with edge detection
            for i in range(js.get_numbuttons()):
                pressed = js.get_button(i)
                if pressed and not prev_buttons[i]:
                    # Button just pressed
                    cmd = mapping.get_button_cmd(i)
                    if cmd:
                        print(f"[BUTTON {i}] -> {cmd}")
                        stages.run_command(cmd)
                    else:
                        print(f"[BUTTON {i}] (unmapped)")
                prev_buttons[i] = pressed
            
            # Dpad with edge detection
            if js.get_numhats() > 0:
                dpad = js.get_hat(0)
                
                # Up
                if dpad[1] == 1 and prev_dpad[1] != 1:
                    cmd = mapping.get_dpad_cmd("up")
                    if cmd:
                        print(f"[DPAD UP] -> {cmd}")
                        stages.run_command(cmd)
                # Down
                if dpad[1] == -1 and prev_dpad[1] != -1:
                    cmd = mapping.get_dpad_cmd("down")
                    if cmd:
                        print(f"[DPAD DOWN] -> {cmd}")
                        stages.run_command(cmd)
                # Right
                if dpad[0] == 1 and prev_dpad[0] != 1:
                    cmd = mapping.get_dpad_cmd("right")
                    if cmd:
                        print(f"[DPAD RIGHT] -> {cmd}")
                        stages.run_command(cmd)
                # Left
                if dpad[0] == -1 and prev_dpad[0] != -1:
                    cmd = mapping.get_dpad_cmd("left")
                    if cmd:
                        print(f"[DPAD LEFT] -> {cmd}")
                        stages.run_command(cmd)
                
                prev_dpad = dpad
            
            # --- Update stages at their intervals ---
            
            # XY stage update
            if now - last_xy_update >= xy_interval:
                vx = axes[0] if len(axes) > 0 else 0
                vy = axes[1] if len(axes) > 1 else 0
                stages.update_xy(vx, vy)
                last_xy_update = now
            
            # ZP stage update - high frequency to keep buffer full
            if now - last_zp_update >= zp_interval:
                # Right stick Y for Z (axis 3)
                vz = axes[3] if len(axes) > 3 else 0
                
                # Triggers for pumps (axes 4,5 go from -1 to 1)
                lt = axes[4] if len(axes) > 4 else -1
                rt = axes[5] if len(axes) > 5 else -1
                
                # Normalize triggers from [-1,1] to [0,1]
                lt = (lt + 1) / 2
                rt = (rt + 1) / 2
                
                vp1 = -lt  # Left trigger withdraws
                vp2 = rt   # Right trigger dispenses
                
                # Only send if there's actual input (don't spam zero commands)
                if any(abs(v) > 0.01 for v in [vz, vp1, vp2]):
                    stages.update_zp(vz, vp1, vp2, zp_interval)
                
                last_zp_update = now
            
            # Update ZP position less frequently
            if now - last_status >= 1.0:
                stages.zp_pos = stages.zp_stage.get_current_position()
            
            # Status print
            if now - last_status >= 3.0:
                x, y, _ = stages.xy_pos or (0, 0, 0)
                z, p1, p2, _ = stages.zp_pos or (0, 0, 0, 0)
                print(f"[STATUS] XY=({x:.1f}, {y:.1f}) Z={z:.2f} P1={p1:.2f} P2={p2:.2f} | "
                      f"Speed: xy={stages.xy_speed} z={stages.z_speed} p={stages.p_speed}")
                last_status = now
            
            # Small sleep to prevent CPU spin
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        stages.stop()
        pygame.quit()
        print("Done.")


def test_controller():
    """Raw controller test mode."""
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No controller found!")
        return
    
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")
    print(f"Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}, Hats: {js.get_numhats()}")
    print("\nPress Ctrl+C to exit.\n")
    
    prev_buttons = [False] * js.get_numbuttons()
    prev_dpad = (0, 0)
    
    try:
        while True:
            pygame.event.pump()
            
            # Buttons
            for i in range(js.get_numbuttons()):
                pressed = js.get_button(i)
                if pressed and not prev_buttons[i]:
                    print(f"BUTTON {i} PRESSED")
                prev_buttons[i] = pressed
            
            # Axes
            active = []
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                if abs(val) > 0.2:
                    active.append(f"Axis{i}={val:.2f}")
            if active:
                print(f"AXES: {' '.join(active)}")
            
            # Dpad
            if js.get_numhats() > 0:
                dpad = js.get_hat(0)
                if dpad != prev_dpad and dpad != (0, 0):
                    dirs = []
                    if dpad[1] == 1: dirs.append("UP")
                    if dpad[1] == -1: dirs.append("DOWN")
                    if dpad[0] == 1: dirs.append("RIGHT")
                    if dpad[0] == -1: dirs.append("LEFT")
                    print(f"DPAD: {'+'.join(dirs)}")
                prev_dpad = dpad
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        pygame.quit()


def main():
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--test-controller", action="store_true")
    parser.add_argument("--real-xy", action="store_true")
    parser.add_argument("--real-zp", action="store_true")
    parser.add_argument("--real", action="store_true")
    args = parser.parse_args()
    
    if args.test_controller:
        test_controller()
        return
    
    simulate_xy =  (args.real_xy or args.real)
    simulate_zp =  (args.real_zp or args.real)
    
    print("=" * 60)
    print("Xbox Stage Controller (Single-Threaded)")
    print("=" * 60)
    print(f"XY Stage: {'SIMULATED' if simulate_xy else 'HARDWARE'}")
    print(f"ZP Stage: {'SIMULATED' if simulate_zp else 'HARDWARE'}")
    print("=" * 60)
    
    main_loop(simulate_xy=simulate_xy, simulate_zp=simulate_zp)


if __name__ == "__main__":
    main()