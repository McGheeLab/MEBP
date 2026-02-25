"""
Stage Controller - Unified control layer for XY and ZP stages.

This module contains:
- XboxQueuePoller: Reads Xbox events from multiprocessing queue
- XYJogHandler: Continuous velocity jogging for XY stage (own thread)
- ZPJogHandler: Segmented relative moves for ZP stage (own thread)
- PositionPoller: Background thread for cached position reads
- StageController: Top-level orchestrator that ties everything together

The jog handlers are taken directly from the proven XBOXCONTROLLED code.

Session 4 additions:
- SafetyLimits integration (software endstops for all axes)
- PositionLogger integration (timestamped position recording)
"""

import math
import time
import threading
import logging
from multiprocessing import Process, Queue

from SupportClasses.Processor import Processor
from SupportClasses.XYStage import XYStageManager
from SupportClasses.ZPStage import ZPStageManager, AXIS_MAP
from SupportClasses.XboxController import xbox_polling_worker
from SupportClasses.SerialUtils import ConnectionWatchdog, check_port_health
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.PositionLogger import PositionLogger

logger = logging.getLogger(__name__)


class XboxQueuePoller:
    """
    Polls the Xbox multiprocessing queue in a thread and dispatches to Processor.
    This replaces the Qt-timer-based polling for non-GUI / thread-safe operation.
    """

    def __init__(self, queue: Queue, processor: Processor):
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
                except Exception:
                    break

                if "debug" in msg:
                    debug_msg = msg["debug"]
                    if "connect" in debug_msg.lower() or "found" in debug_msg.lower():
                        logger.info(f"[Xbox] {debug_msg}")
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
    Converts velocity commands into segmented relative moves.
    
    Proven working from XBOXCONTROLLED code.
    
    Session 4: Now accepts optional safety_limits for boundary checking.
    """

    def __init__(self, processor: Processor, zp_stage: ZPStageManager,
                 safety_limits: SafetyLimits = None, get_zp_position=None):
        self.processor = processor
        self.stage = zp_stage
        self.safety_limits = safety_limits
        self._get_zp_position = get_zp_position  # callable → (z, p1, p2, p3)

        self.vel_z = 0.0
        self.vel_p1 = 0.0
        self.vel_p2 = 0.0
        self.vel_p3 = 0.0
        self.lock = threading.Lock()

        # Tuning parameters
        self.segment_time = 0.12  # seconds per move segment
        self.z_speed = 0.5        # speed multiplier for Z
        self.p_speed = 0.5        # speed multiplier for pumps
        self.max_speed = 1.0

        self._was_moving = False
        self._running = False
        self._thread = None

        # Register handlers with processor
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

    @property
    def speeds(self) -> dict:
        """Current speed multipliers."""
        return {"z": self.z_speed, "p": self.p_speed}

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
        logger.info(f"Z speed: {self.z_speed}")

    def _increment_z_down(self, *args, **kwargs):
        self.z_speed = max(self.z_speed / 2, 0.1)
        logger.info(f"Z speed: {self.z_speed}")

    def _increment_p_up(self, *args, **kwargs):
        self.p_speed = min(self.p_speed * 2, 100)
        logger.info(f"P speed: {self.p_speed}")

    def _increment_p_down(self, *args, **kwargs):
        self.p_speed = max(self.p_speed / 2, 0.1)
        logger.info(f"P speed: {self.p_speed}")

    def _jog_loop(self):
        while self._running:
            with self.lock:
                vz, vp1, vp2, vp3 = self.vel_z, self.vel_p1, self.vel_p2, self.vel_p3

            is_moving = any(abs(v) > 0.001 for v in [vz, vp1, vp2, vp3])

            if is_moving and not self._was_moving:
                logger.debug(f"[ZP] Start: z={vz:.2f} p1={vp1:.2f} p2={vp2:.2f} p3={vp3:.2f}")
            elif not is_moving and self._was_moving:
                logger.debug("[ZP] Stop")
            self._was_moving = is_moving

            if not is_moving:
                time.sleep(0.01)
                continue

            dz = -vz * self.segment_time
            dp1 = vp1 * self.segment_time
            dp2 = vp2 * self.segment_time
            dp3 = vp3 * self.segment_time

            # Safety limits check for jog moves
            if self.safety_limits and self.safety_limits.enabled and self._get_zp_position:
                try:
                    pos = self._get_zp_position()
                    if pos[0] is not None:
                        cur_z, cur_p1, cur_p2, cur_p3 = pos
                        new_z = self.safety_limits.clamp_z(cur_z + dz)
                        new_p1 = self.safety_limits.clamp_pump(cur_p1 + dp1, "P1")
                        new_p2 = self.safety_limits.clamp_pump(cur_p2 + dp2, "P2")
                        new_p3 = self.safety_limits.clamp_pump(cur_p3 + dp3, "P3")
                        dz = new_z - cur_z
                        dp1 = new_p1 - cur_p1
                        dp2 = new_p2 - cur_p2
                        dp3 = new_p3 - cur_p3
                except Exception:
                    pass  # Don't block jogging if position query fails

            combined = math.sqrt(vz**2 + vp1**2 + vp2**2 + vp3**2)
            feedrate = max(combined * 60, 1)

            # Clamp feedrate
            if self.safety_limits and self.safety_limits.enabled:
                feedrate = min(feedrate, self.safety_limits.max_z_feedrate)

            axes = {'X': dz, 'Y': dp1, 'Z': dp2, 'E': dp3}
            self.stage.move_relative(axes, feedrate)

            time.sleep(self.segment_time)


class XYJogHandler:
    """
    Handles XY stage jogging in its own thread.
    Sends continuous velocity commands.
    
    Proven working from XBOXCONTROLLED code.
    
    Session 4: Now accepts optional safety_limits for boundary checking.
    """

    def __init__(self, processor: Processor, xy_stage: XYStageManager,
                 safety_limits: SafetyLimits = None, get_xy_position=None):
        self.processor = processor
        self.stage = xy_stage
        self.safety_limits = safety_limits
        self._get_xy_position = get_xy_position  # callable → (x, y, f)

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.lock = threading.Lock()

        # Tuning parameters
        self.xy_speed = 100.0     # speed multiplier
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

    @property
    def speed(self) -> float:
        """Current speed multiplier."""
        return self.xy_speed

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
        logger.info(f"XY speed: {self.xy_speed}")

    def _increment_down(self, *args, **kwargs):
        self.xy_speed = max(self.xy_speed / 2, 1)
        logger.info(f"XY speed: {self.xy_speed}")

    def _jog_loop(self):
        while self._running:
            with self.lock:
                vx, vy = self.vel_x, self.vel_y

            is_moving = abs(vx) > 0.001 or abs(vy) > 0.001

            if is_moving and not self._was_moving:
                logger.debug(f"[XY] Start: x={vx:.1f} y={vy:.1f}")
            elif not is_moving and self._was_moving:
                logger.debug("[XY] Stop")
            self._was_moving = is_moving

            # Safety limits: dampen velocity near boundaries
            if is_moving and self.safety_limits and self.safety_limits.enabled and self._get_xy_position:
                try:
                    pos = self._get_xy_position()
                    if pos[0] is not None:
                        near = self.safety_limits.check_xy_near_limit(pos[0], pos[1], margin=500)
                        if near["x_near_min"] and vx < 0:
                            vx *= 0.3
                        if near["x_near_max"] and vx > 0:
                            vx *= 0.3
                        if near["y_near_min"] and vy < 0:
                            vy *= 0.3
                        if near["y_near_max"] and vy > 0:
                            vy *= 0.3
                except Exception:
                    pass

            self.stage.move_stage_at_velocity(vx, vy)
            time.sleep(self.update_interval)


class PositionPoller:
    """
    Background thread that periodically polls stage positions and caches results.
    
    This prevents the UI timer from blocking on serial I/O. The GUI reads
    cached positions which are updated at a configurable interval.
    """

    def __init__(self, poll_interval: float = 0.3):
        self.poll_interval = poll_interval
        self._xy_stage = None
        self._zp_stage = None
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        # Cached positions
        self._xy_pos = (None, None, None)
        self._zp_pos = (None, None, None, None)

    def set_stages(self, xy_stage=None, zp_stage=None):
        """Update which stages to poll."""
        with self._lock:
            self._xy_stage = xy_stage
            self._zp_stage = zp_stage
            if xy_stage is None:
                self._xy_pos = (None, None, None)
            if zp_stage is None:
                self._zp_pos = (None, None, None, None)

    @property
    def xy_position(self) -> tuple:
        with self._lock:
            return self._xy_pos

    @property
    def zp_position(self) -> tuple:
        with self._lock:
            return self._zp_pos

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="PositionPoller")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _poll_loop(self):
        while self._running:
            with self._lock:
                xy = self._xy_stage
                zp = self._zp_stage

            if xy is not None:
                try:
                    pos = xy.get_current_position()
                    with self._lock:
                        self._xy_pos = pos
                except Exception as e:
                    logger.debug(f"XY poll error: {e}")

            if zp is not None:
                try:
                    pos = zp.get_current_position()
                    with self._lock:
                        self._zp_pos = pos
                except Exception as e:
                    logger.debug(f"ZP poll error: {e}")

            time.sleep(self.poll_interval)


class StageController:
    """
    Top-level orchestrator that manages all hardware and control threads.
    
    This is the main object that the GUI (or headless mode) interacts with.
    It owns the Processor, stages, jog handlers, and Xbox controller.
    
    Session 4 additions:
    - SafetyLimits: Software endstops for all axes (Task 3)
    - PositionLogger: Timestamped position recording (Task 2)
    """

    def __init__(self, simulate_xy=True, simulate_zp=True):
        self.simulate_xy = simulate_xy
        self.simulate_zp = simulate_zp

        # Core command processor
        self.processor = Processor()

        # Stages (created on connect)
        self.xy_stage: XYStageManager | None = None
        self.zp_stage: ZPStageManager | None = None

        # Jog handlers (created after stages)
        self.xy_jog: XYJogHandler | None = None
        self.zp_jog: ZPJogHandler | None = None

        # Xbox controller
        self.xbox_queue: Queue | None = None
        self.xbox_process: Process | None = None
        self.xbox_poller: XboxQueuePoller | None = None

        # Zero reference positions (set during calibration)
        self.zero_position = {
            "x": 0.0, "y": 0.0, "f": 0.0,
            "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0,
        }

        # Session 4: Safety Limits
        self.safety_limits = SafetyLimits()

        # Session 4: Position Logger
        self.position_logger = PositionLogger()

        # Connection watchdog for real hardware
        self._watchdog = ConnectionWatchdog(check_interval=3.0)
        self._watchdog.start()

        # Background position poller (non-blocking for UI)
        self._pos_poller = PositionPoller(poll_interval=0.3)
        self._pos_poller.start()

        # Disconnect callback (GUI can set this to update UI)
        self.on_disconnect: callable | None = None

        # Register calibration command
        self.processor.register_handler("zero_needle_pos", self._calibrate_zero)

    # ── Connection Management ──────────────────────────────────────

    def connect_stages(self):
        """Initialize and connect both stages."""
        if self.xy_stage is None:
            self.xy_stage = XYStageManager(simulate=self.simulate_xy)
            self.xy_jog = XYJogHandler(
                self.processor, self.xy_stage,
                safety_limits=self.safety_limits,
                get_xy_position=lambda: self._pos_poller.xy_position,
            )
            self.xy_jog.start()
            # Watch for disconnects (real hardware only)
            if not self.simulate_xy:
                self._watchdog.watch(
                    "XY",
                    lambda: getattr(self.xy_stage, 'spo', None),
                    lambda: self._handle_disconnect("XY"),
                )
            logger.info("XY stage connected")

        if self.zp_stage is None:
            self.zp_stage = ZPStageManager(simulate=self.simulate_zp)
            self.zp_jog = ZPJogHandler(
                self.processor, self.zp_stage,
                safety_limits=self.safety_limits,
                get_zp_position=lambda: self._pos_poller.zp_position,
            )
            self.zp_jog.start()
            if not self.simulate_zp:
                self._watchdog.watch(
                    "ZP",
                    lambda: getattr(self.zp_stage, 'serial', None),
                    lambda: self._handle_disconnect("ZP"),
                )
            logger.info("ZP stage connected")

        # Update position poller
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)

    def _handle_disconnect(self, stage_name: str):
        """Called by watchdog when a serial disconnect is detected."""
        logger.error(f"{stage_name} stage disconnected!")
        if stage_name == "XY":
            self.disconnect_xy()
        elif stage_name == "ZP":
            self.disconnect_zp()
        if self.on_disconnect:
            self.on_disconnect(stage_name)

    def disconnect_xy(self):
        """Disconnect and clean up XY stage."""
        if self.xy_jog:
            self.xy_jog.stop()
            self.xy_jog = None
        if self.xy_stage:
            try:
                self.xy_stage.stop()
            except Exception:
                pass
            self.xy_stage = None
        self._pos_poller.set_stages(None, self.zp_stage)
        self._watchdog.unwatch("XY")
        logger.info("XY stage disconnected")

    def disconnect_zp(self):
        """Disconnect and clean up ZP stage."""
        if self.zp_jog:
            self.zp_jog.stop()
            self.zp_jog = None
        if self.zp_stage:
            try:
                self.zp_stage.stop()
            except Exception:
                pass
            self.zp_stage = None
        self._pos_poller.set_stages(self.xy_stage, None)
        self._watchdog.unwatch("ZP")
        logger.info("ZP stage disconnected")

    def disconnect_stages(self):
        """Disconnect both stages."""
        self.disconnect_xy()
        self.disconnect_zp()

    # ── Xbox Controller ────────────────────────────────────────────

    def connect_xbox(self, mapping_file: str = "current_button_mapping.json"):
        """Start Xbox controller in separate process."""
        if self.xbox_process and self.xbox_process.is_alive():
            logger.warning("Xbox already connected")
            return

        self.xbox_queue = Queue()
        self.xbox_process = Process(
            target=xbox_polling_worker,
            args=(self.xbox_queue,),
            kwargs={"mapping_file": mapping_file},
            daemon=True,
        )
        self.xbox_process.start()
        self.xbox_poller = XboxQueuePoller(self.xbox_queue, self.processor)
        self.xbox_poller.start()
        logger.info("Xbox controller connected")

    def disconnect_xbox(self):
        """Stop Xbox controller."""
        if self.xbox_poller:
            self.xbox_poller.stop()
            self.xbox_poller = None
        if self.xbox_process and self.xbox_process.is_alive():
            self.xbox_process.terminate()
            self.xbox_process.join(timeout=2.0)
            self.xbox_process = None
        self.xbox_queue = None
        logger.info("Xbox controller disconnected")

    # ── Position Queries ───────────────────────────────────────────

    def get_xy_position(self, cached=True) -> tuple:
        """
        Get current XY position.
        Returns (x, y, f) or (None, None, None).
        
        Args:
            cached: If True, returns the most recent polled value (non-blocking).
                    If False, queries the stage directly (may block on serial I/O).
        """
        if cached:
            return self._pos_poller.xy_position

        if self.xy_stage:
            try:
                return self.xy_stage.get_current_position()
            except Exception as e:
                logger.debug(f"XY position query error: {e}")
        return (None, None, None)

    def get_zp_position(self, cached=True) -> tuple:
        """
        Get current ZP position.
        Returns (z, p1, p2, p3) or (None,)*4.
        
        Args:
            cached: If True, returns the most recent polled value (non-blocking).
                    If False, queries the stage directly (may block on serial I/O).
        """
        if cached:
            return self._pos_poller.zp_position

        if self.zp_stage:
            try:
                return self.zp_stage.get_current_position()
            except Exception as e:
                logger.debug(f"ZP position query error: {e}")
        return (None, None, None, None)

    def get_speed_info(self) -> dict:
        """Get current speed multipliers for all axes."""
        return {
            "xy": self.xy_jog.speed if self.xy_jog else 0,
            "z": self.zp_jog.speeds["z"] if self.zp_jog else 0,
            "p": self.zp_jog.speeds["p"] if self.zp_jog else 0,
        }

    @property
    def is_xy_connected(self) -> bool:
        return self.xy_stage is not None

    @property
    def is_zp_connected(self) -> bool:
        return self.zp_stage is not None

    @property
    def is_xbox_connected(self) -> bool:
        return self.xbox_process is not None and self.xbox_process.is_alive()

    # ── Calibration ────────────────────────────────────────────────

    def _calibrate_zero(self, *args, **kwargs):
        """Set current position as the zero reference."""
        if self.xy_stage:
            pos = self.xy_stage.get_current_position()
            if pos[0] is not None:
                self.zero_position["x"] = pos[0]
                self.zero_position["y"] = pos[1]
                self.zero_position["f"] = pos[2] if pos[2] else 0

        if self.zp_stage:
            pos = self.zp_stage.get_current_position()
            if pos[0] is not None:
                self.zero_position["Z"] = pos[0]
                self.zero_position["P1"] = pos[1]
                self.zero_position["P2"] = pos[2]
                self.zero_position["P3"] = pos[3]

        logger.info(f"Zero position calibrated: {self.zero_position}")

        # Session 4: Log calibration event
        self.position_logger.record(
            "calibrate_zero",
            xy_pos=self.get_xy_position(cached=False),
            zp_pos=self.get_zp_position(cached=False),
            metadata={"zero_position": dict(self.zero_position)},
        )

    # ── Movement (for GUI / print commands) ────────────────────────

    def move_xy_absolute(self, x, y, from_zero_ref=True, fast=False):
        """
        Move XY stage to absolute position.
        
        Session 4: Now applies safety limit clamping before sending command.
        """
        if not self.xy_stage:
            return

        # Apply safety limits (clamp in zero-ref space before converting)
        if self.safety_limits.enabled and from_zero_ref:
            x, y = self.safety_limits.clamp_xy(x, y)

        if from_zero_ref:
            x = x + self.zero_position["x"]
            y = y + self.zero_position["y"]
        self.xy_stage.move_stage_to_position(x, y, fast)

    def move_z_absolute(self, z_value, from_zero_ref=True, fast=False):
        """
        Move Z axis to absolute position.
        
        Session 4: Now applies safety limit clamping.
        """
        if not self.zp_stage:
            return

        # Apply safety limits (clamp in zero-ref space)
        if self.safety_limits.enabled and from_zero_ref:
            z_value = self.safety_limits.clamp_z(z_value)

        position = z_value
        if from_zero_ref:
            position = z_value + self.zero_position["Z"]
        mapped = AXIS_MAP["Z"]
        self.zp_stage.move_absolute({mapped: position}, fast)

    def move_z_relative(self, distance, feedrate=None):
        """
        Move Z axis by relative distance.
        
        Session 4: Clamps feedrate and checks resulting position.
        """
        if not self.zp_stage:
            return

        # Clamp feedrate
        if feedrate and self.safety_limits.enabled:
            feedrate = self.safety_limits.clamp_z_feedrate(feedrate)

        # Check resulting position against limits
        if self.safety_limits.enabled:
            try:
                pos = self.get_zp_position(cached=True)
                if pos[0] is not None:
                    new_z = pos[0] + distance
                    clamped_z = self.safety_limits.clamp_z(
                        new_z - self.zero_position["Z"]
                    )
                    distance = (clamped_z + self.zero_position["Z"]) - pos[0]
            except Exception:
                pass

        mapped = AXIS_MAP["Z"]
        self.zp_stage.move_relative({mapped: distance}, feedrate)

    def move_pump_relative(self, pump: str, distance: float, feedrate=None):
        """
        Move a pump (P1/P2/P3) by relative distance.
        
        Session 4: Clamps feedrate and checks resulting position.
        """
        if not self.zp_stage:
            return

        # Clamp feedrate
        if feedrate and self.safety_limits.enabled:
            feedrate = self.safety_limits.clamp_pump_feedrate(feedrate)

        # Check resulting position against limits
        if self.safety_limits.enabled:
            try:
                pos = self.get_zp_position(cached=True)
                if pos[0] is not None:
                    # Map pump to position index: P1→1, P2→2, P3→3
                    pump_idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
                    cur_pos = pos[pump_idx]
                    new_pos = cur_pos + distance
                    zero_ref = self.zero_position.get(pump, 0)
                    clamped = self.safety_limits.clamp_pump(
                        new_pos - zero_ref, pump
                    )
                    distance = (clamped + zero_ref) - cur_pos
            except Exception:
                pass

        mapped = AXIS_MAP.get(pump)
        if mapped:
            self.zp_stage.move_relative({mapped: distance}, feedrate)

    # ── Shutdown ───────────────────────────────────────────────────

    def shutdown(self):
        """Clean shutdown of all components."""
        logger.info("Shutting down StageController...")
        self._watchdog.stop()
        self._pos_poller.stop()
        self.disconnect_xbox()
        self.disconnect_stages()
        self.processor.stop()
        logger.info("Shutdown complete")
