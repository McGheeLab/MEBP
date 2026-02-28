"""
Stage Controller — Unified control layer for XY and ZP stages.

Contains:
    - XboxQueuePoller:  Reads Xbox events from multiprocessing queue
    - XYJogHandler:     Continuous velocity jogging for XY stage
    - ZPJogHandler:     Segmented relative moves for ZP stage
    - PositionPoller:   Background cached position reads
    - StageController:  Top-level orchestrator tying everything together

The jog handlers preserve the proven XBOXCONTROLLED workflow.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from multiprocessing import Process, Queue
from typing import Callable, Optional

from SupportClasses.Processor import Processor
from SupportClasses.XYStage import XYStageManager
from SupportClasses.ZPStage import ZPStageManager, AXIS_MAP
from SupportClasses.XboxController import xbox_polling_worker
from SupportClasses.SerialUtils import ConnectionWatchdog, check_port_health
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.PositionLogger import PositionLogger

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Xbox Queue Poller
# ═══════════════════════════════════════════════════════════════════

class XboxQueuePoller:
    """
    Polls the Xbox multiprocessing queue in a thread and dispatches
    events to the Processor command bus.
    """

    def __init__(self, queue: Queue, processor: Processor):
        self.queue = queue
        self.processor = processor
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="XboxPoller"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _poll_loop(self) -> None:
        while self._running:
            while not self.queue.empty():
                try:
                    msg = self.queue.get_nowait()
                except Exception:
                    break

                if "debug" in msg:
                    text = msg["debug"]
                    if "connect" in text.lower() or "found" in text.lower():
                        logger.info(f"[Xbox] {text}")
                elif "button" in msg:
                    self.processor.add_command(msg["command"], button=msg["button"])
                elif "axis" in msg:
                    self.processor.add_command(
                        msg["command"], axis=msg["axis"], average=msg["average"]
                    )
                elif "dpad" in msg:
                    self.processor.add_command(msg["command"], direction=msg["dpad"])

            time.sleep(0.02)


# ═══════════════════════════════════════════════════════════════════
# ZP Jog Handler
# ═══════════════════════════════════════════════════════════════════

class ZPJogHandler:
    """
    Converts velocity commands into segmented relative moves for
    the ZP stage.  Runs its own daemon thread.

    Proven working from XBOXCONTROLLED code — core logic unchanged.
    """

    def __init__(
        self,
        processor: Processor,
        zp_stage: ZPStageManager,
        safety_limits: SafetyLimits | None = None,
        get_zp_position: Callable | None = None,
    ):
        self.processor = processor
        self.stage = zp_stage
        self.safety_limits = safety_limits
        self._get_zp_position = get_zp_position

        # Velocity state (updated by command handlers)
        self.vel_z: float = 0.0
        self.vel_p1: float = 0.0
        self.vel_p2: float = 0.0
        self.vel_p3: float = 0.0
        self._lock = threading.Lock()

        # Tuning parameters
        self.segment_time: float = 0.12   # seconds per move segment
        self.z_speed: float = 0.5         # Z speed multiplier
        self.p_speed: float = 0.5         # Pump speed multiplier
        self.max_speed: float = 1.0

        self._was_moving = False
        self._running = False
        self._thread: threading.Thread | None = None

        # Register processor handlers
        self.processor.register_handler("move_z_at_velocity", self._handle_z_vel)
        self.processor.register_handler("move_p1_at_velocity", self._handle_p1_vel)
        self.processor.register_handler("move_p2_at_velocity", self._handle_p2_vel)
        self.processor.register_handler("move_p3_at_velocity", self._handle_p3_vel)
        self.processor.register_handler("increment_zspeed_up", self._incr_z_up)
        self.processor.register_handler("increment_zspeed_down", self._incr_z_down)
        self.processor.register_handler("increment_pspeed_up", self._incr_p_up)
        self.processor.register_handler("increment_pspeed_down", self._incr_p_down)

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._jog_loop, daemon=True, name="ZPJog"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    @property
    def speeds(self) -> dict[str, float]:
        return {"z": self.z_speed, "p": self.p_speed}

    # ── Velocity Extraction ───────────────────────────────────────

    @staticmethod
    def _extract_velocity(*args, **kwargs) -> float:
        """Pull a scalar velocity from Xbox axis kwargs."""
        val = kwargs.get("average", 0.0)
        if isinstance(val, (list, tuple)):
            return val[1] if len(val) > 1 else val[0]
        return float(val)

    def _clamp_vel(self, raw: float, multiplier: float) -> float:
        return max(-self.max_speed, min(self.max_speed, raw * multiplier))

    # ── Command Handlers ──────────────────────────────────────────

    def _handle_z_vel(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self._lock:
            self.vel_z = self._clamp_vel(raw, self.z_speed)

    def _handle_p1_vel(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self._lock:
            self.vel_p1 = self._clamp_vel(raw, self.p_speed)

    def _handle_p2_vel(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self._lock:
            self.vel_p2 = self._clamp_vel(raw, self.p_speed)

    def _handle_p3_vel(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self._lock:
            self.vel_p3 = self._clamp_vel(raw, self.p_speed)

    def _incr_z_up(self, *a, **kw):
        self.z_speed = min(self.z_speed * 2, 100)
        logger.info(f"Z speed: {self.z_speed}")

    def _incr_z_down(self, *a, **kw):
        self.z_speed = max(self.z_speed / 2, 0.1)
        logger.info(f"Z speed: {self.z_speed}")

    def _incr_p_up(self, *a, **kw):
        self.p_speed = min(self.p_speed * 2, 100)
        logger.info(f"P speed: {self.p_speed}")

    def _incr_p_down(self, *a, **kw):
        self.p_speed = max(self.p_speed / 2, 0.1)
        logger.info(f"P speed: {self.p_speed}")

    # ── Jog Loop ──────────────────────────────────────────────────

    def _jog_loop(self) -> None:
        while self._running:
            with self._lock:
                vz, vp1, vp2, vp3 = self.vel_z, self.vel_p1, self.vel_p2, self.vel_p3

            is_moving = any(abs(v) > 0.001 for v in (vz, vp1, vp2, vp3))

            if is_moving and not self._was_moving:
                logger.debug(f"[ZP] Jog start: z={vz:.2f} p1={vp1:.2f}")
            elif not is_moving and self._was_moving:
                logger.debug("[ZP] Jog stop")
            self._was_moving = is_moving

            if not is_moving:
                time.sleep(0.01)
                continue

            dz = -vz * self.segment_time
            dp1 = vp1 * self.segment_time
            dp2 = vp2 * self.segment_time
            dp3 = vp3 * self.segment_time

            # Apply safety limits
            if self.safety_limits and self.safety_limits.enabled and self._get_zp_position:
                try:
                    pos = self._get_zp_position()
                    if pos[0] is not None:
                        cz, cp1, cp2, cp3 = pos
                        dz = self.safety_limits.clamp_z(cz + dz) - cz
                        dp1 = self.safety_limits.clamp_pump(cp1 + dp1, "P1") - cp1
                        dp2 = self.safety_limits.clamp_pump(cp2 + dp2, "P2") - cp2
                        dp3 = self.safety_limits.clamp_pump(cp3 + dp3, "P3") - cp3
                except Exception:
                    pass

            combined = math.sqrt(vz**2 + vp1**2 + vp2**2 + vp3**2)
            feedrate = max(combined * 60, 1)

            if self.safety_limits and self.safety_limits.enabled:
                feedrate = min(feedrate, self.safety_limits.max_z_feedrate)

            self.stage.move_relative(
                {"X": dz, "Y": dp1, "Z": dp2, "E": dp3}, feedrate
            )
            time.sleep(self.segment_time)


# ═══════════════════════════════════════════════════════════════════
# XY Jog Handler
# ═══════════════════════════════════════════════════════════════════

class XYJogHandler:
    """
    Sends continuous velocity commands to the XY stage.
    Runs its own daemon thread.

    Proven working from XBOXCONTROLLED code — core logic unchanged.
    """

    def __init__(
        self,
        processor: Processor,
        xy_stage: XYStageManager,
        safety_limits: SafetyLimits | None = None,
        get_xy_position: Callable | None = None,
    ):
        self.processor = processor
        self.stage = xy_stage
        self.safety_limits = safety_limits
        self._get_xy_position = get_xy_position

        self.vel_x: float = 0.0
        self.vel_y: float = 0.0
        self._lock = threading.Lock()

        # Tuning
        self.xy_speed: float = 100.0
        self.max_speed: float = 5000.0
        self.update_interval: float = 0.1

        self._was_moving = False
        self._running = False
        self._thread: threading.Thread | None = None

        self.processor.register_handler("move_stage_at_velocity", self._handle_vel)
        self.processor.register_handler("increment_xyspeed_up", self._incr_up)
        self.processor.register_handler("increment_xyspeed_down", self._incr_down)

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._jog_loop, daemon=True, name="XYJog"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self.stage.move_stage_at_velocity(0, 0)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)

    @property
    def speed(self) -> float:
        return self.xy_speed

    def _handle_vel(self, *args, **kwargs):
        val = kwargs.get("average", (0, 0))
        if isinstance(val, (list, tuple)) and len(val) >= 2:
            vx, vy = val[0], val[1]
        else:
            vx, vy = 0.0, 0.0
        with self._lock:
            self.vel_x = max(-self.max_speed, min(self.max_speed, vx * self.xy_speed))
            self.vel_y = max(-self.max_speed, min(self.max_speed, vy * self.xy_speed))

    def _incr_up(self, *a, **kw):
        self.xy_speed = min(self.xy_speed * 2, 10000)
        logger.info(f"XY speed: {self.xy_speed}")

    def _incr_down(self, *a, **kw):
        self.xy_speed = max(self.xy_speed / 2, 1)
        logger.info(f"XY speed: {self.xy_speed}")

    def _jog_loop(self) -> None:
        while self._running:
            with self._lock:
                vx, vy = self.vel_x, self.vel_y

            is_moving = abs(vx) > 0.001 or abs(vy) > 0.001

            if is_moving and not self._was_moving:
                logger.debug(f"[XY] Jog start: x={vx:.1f} y={vy:.1f}")
            elif not is_moving and self._was_moving:
                logger.debug("[XY] Jog stop")
            self._was_moving = is_moving

            # Dampen near limits
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


# ═══════════════════════════════════════════════════════════════════
# Position Poller
# ═══════════════════════════════════════════════════════════════════

class PositionPoller:
    """
    Background thread that caches stage positions at regular intervals.
    Prevents the GUI from blocking on serial I/O.
    """

    def __init__(self, poll_interval: float = 0.3):
        self.poll_interval = poll_interval
        self._xy_stage: XYStageManager | None = None
        self._zp_stage: ZPStageManager | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._xy_pos: tuple = (None, None, None)
        self._zp_pos: tuple = (None, None, None, None)

    def set_stages(
        self,
        xy_stage: XYStageManager | None = None,
        zp_stage: ZPStageManager | None = None,
    ) -> None:
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

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="PositionPoller"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _poll_loop(self) -> None:
        while self._running:
            with self._lock:
                xy, zp = self._xy_stage, self._zp_stage

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


# ═══════════════════════════════════════════════════════════════════
# Stage Controller (Top-Level Orchestrator)
# ═══════════════════════════════════════════════════════════════════

class StageController:
    """
    Main entry point that the GUI (or headless mode) interacts with.

    Owns the Processor, stages, jog handlers, Xbox controller,
    safety limits, and position logger.
    """

    def __init__(
        self,
        simulate_xy: bool = True,
        simulate_zp: bool = True,
        controller_json: str | None = None,
    ):
        self.simulate_xy = simulate_xy
        self.simulate_zp = simulate_zp

        # v7.1 P8.27: Controller JSON path (passed through to XYStageManager)
        # "auto" = auto-detect, None = default ProScan III, or explicit path
        self.controller_json = controller_json

        # Core command processor
        self.processor = Processor()

        # Hardware stages (created on connect)
        self.xy_stage: XYStageManager | None = None
        self.zp_stage: ZPStageManager | None = None

        # Jog handlers (created after stages)
        self.xy_jog: XYJogHandler | None = None
        self.zp_jog: ZPJogHandler | None = None

        # Xbox controller
        self.xbox_queue: Queue | None = None
        self.xbox_process: Process | None = None
        self.xbox_poller: XboxQueuePoller | None = None

        # Zero reference (set during calibration)
        self.zero_position: dict[str, float] = {
            "x": 0.0, "y": 0.0, "f": 0.0,
            "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0,
        }

        # Safety limits
        self.safety_limits = SafetyLimits()

        # Position logger
        self.position_logger = PositionLogger()

        # Watchdog for real hardware disconnects
        self._watchdog = ConnectionWatchdog(check_interval=3.0)
        self._watchdog.start()

        # Background position cache
        self._pos_poller = PositionPoller(poll_interval=0.3)
        self._pos_poller.start()

        # Disconnect callback (GUI can set this)
        self.on_disconnect: Callable | None = None

        # Register calibration handler
        self.processor.register_handler("zero_needle_pos", self._calibrate_zero)

    # ── Connection Management ─────────────────────────────────────

    def connect_stages(self, xy: bool = True, zp: bool = True) -> None:
        """Initialise and connect stages.

        Args:
            xy: If True, connect the XY stage (default True).
            zp: If True, connect the ZP stage (default True).
        """
        if xy and self.xy_stage is None:
            self.xy_stage = XYStageManager(
                simulate=self.simulate_xy,
                controller_json=self.controller_json,
            )
            self.xy_jog = XYJogHandler(
                self.processor, self.xy_stage,
                safety_limits=self.safety_limits,
                get_xy_position=lambda: self._pos_poller.xy_position,
            )
            self.xy_jog.start()
            if not self.simulate_xy:
                self._watchdog.watch(
                    "XY",
                    lambda: getattr(self.xy_stage, "spo", None),
                    lambda: self._handle_disconnect("XY"),
                )
            logger.info("XY stage connected")

        if zp and self.zp_stage is None:
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
                    lambda: getattr(self.zp_stage, "serial", None),
                    lambda: self._handle_disconnect("ZP"),
                )
            logger.info("ZP stage connected")

        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)

    def _handle_disconnect(self, stage_name: str) -> None:
        logger.error(f"{stage_name} stage disconnected!")
        if stage_name == "XY":
            self.disconnect_xy()
        elif stage_name == "ZP":
            self.disconnect_zp()
        if self.on_disconnect:
            self.on_disconnect(stage_name)

    def disconnect_xy(self) -> None:
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

    def disconnect_zp(self) -> None:
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

    def disconnect_stages(self) -> None:
        self.disconnect_xy()
        self.disconnect_zp()

    # ── Xbox Controller ───────────────────────────────────────────

    def connect_xbox(self, mapping_file: str = "current_button_mapping.json") -> None:
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

    def disconnect_xbox(self) -> None:
        if self.xbox_poller:
            self.xbox_poller.stop()
            self.xbox_poller = None
        if self.xbox_process and self.xbox_process.is_alive():
            self.xbox_process.terminate()
            self.xbox_process.join(timeout=2.0)
            self.xbox_process = None
        self.xbox_queue = None
        logger.info("Xbox controller disconnected")

    # ── Position Queries ──────────────────────────────────────────

    def get_xy_position(self, cached: bool = True) -> tuple:
        """Get XY position. cached=True returns polled value (non-blocking)."""
        if cached:
            return self._pos_poller.xy_position
        if self.xy_stage:
            try:
                return self.xy_stage.get_current_position()
            except Exception as e:
                logger.debug(f"XY direct query error: {e}")
        return (None, None, None)

    def get_zp_position(self, cached: bool = True) -> tuple:
        """Get ZP position. cached=True returns polled value (non-blocking)."""
        if cached:
            return self._pos_poller.zp_position
        if self.zp_stage:
            try:
                return self.zp_stage.get_current_position()
            except Exception as e:
                logger.debug(f"ZP direct query error: {e}")
        return (None, None, None, None)

    def get_speed_info(self) -> dict:
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

    # ── Calibration ───────────────────────────────────────────────

    def _calibrate_zero(self, *args, **kwargs) -> None:
        """Set current position as zero reference."""
        if self.xy_stage:
            pos = self.xy_stage.get_current_position()
            if pos[0] is not None:
                self.zero_position["x"] = pos[0]
                self.zero_position["y"] = pos[1]
                self.zero_position["f"] = pos[2] if pos[2] else 0.0

        if self.zp_stage:
            pos = self.zp_stage.get_current_position()
            if pos[0] is not None:
                self.zero_position["Z"] = pos[0]
                self.zero_position["P1"] = pos[1]
                self.zero_position["P2"] = pos[2]
                self.zero_position["P3"] = pos[3]

        logger.info(f"Zero position calibrated: {self.zero_position}")

        self.position_logger.record(
            "calibrate_zero",
            xy_pos=self.get_xy_position(cached=False),
            zp_pos=self.get_zp_position(cached=False),
            metadata={"zero_position": dict(self.zero_position)},
        )

    # ── Movement (for GUI / Print commands) ───────────────────────

    def move_xy_absolute(
        self, x: float, y: float, from_zero_ref: bool = True, fast: bool = False
    ) -> None:
        """Move XY to absolute position, optionally relative to zero ref."""
        if not self.xy_stage:
            return
        if self.safety_limits.enabled and from_zero_ref:
            x, y = self.safety_limits.clamp_xy(x, y)
        if from_zero_ref:
            x += self.zero_position["x"]
            y += self.zero_position["y"]
        self.xy_stage.move_stage_to_position(x, y, fast)

    def move_z_absolute(
        self, z_value: float, from_zero_ref: bool = True, fast: bool = False
    ) -> None:
        """Move Z needle to absolute position."""
        if not self.zp_stage:
            return
        if self.safety_limits.enabled and from_zero_ref:
            z_value = self.safety_limits.clamp_z(z_value)
        position = z_value
        if from_zero_ref:
            position += self.zero_position["Z"]
        self.zp_stage.move_absolute({AXIS_MAP["Z"]: position}, fast)

    def move_z_relative(self, distance: float, feedrate: float | None = None) -> None:
        """Move Z by relative distance."""
        if not self.zp_stage:
            return
        if feedrate and self.safety_limits.enabled:
            feedrate = self.safety_limits.clamp_z_feedrate(feedrate)
        if self.safety_limits.enabled:
            try:
                pos = self.get_zp_position(cached=True)
                if pos[0] is not None:
                    new_z = pos[0] + distance
                    clamped = self.safety_limits.clamp_z(new_z - self.zero_position["Z"])
                    distance = (clamped + self.zero_position["Z"]) - pos[0]
            except Exception:
                pass
        self.zp_stage.move_relative({AXIS_MAP["Z"]: distance}, feedrate)

    def move_pump_relative(
        self, pump: str, distance: float, feedrate: float | None = None
    ) -> None:
        """Move a pump (P1/P2/P3) by relative distance."""
        if not self.zp_stage:
            return
        if feedrate and self.safety_limits.enabled:
            feedrate = self.safety_limits.clamp_pump_feedrate(feedrate)
        if self.safety_limits.enabled:
            try:
                pos = self.get_zp_position(cached=True)
                if pos[0] is not None:
                    idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
                    cur = pos[idx]
                    zero_ref = self.zero_position.get(pump, 0)
                    clamped = self.safety_limits.clamp_pump(cur + distance - zero_ref, pump)
                    distance = (clamped + zero_ref) - cur
            except Exception:
                pass
        mapped = AXIS_MAP.get(pump)
        if mapped:
            self.zp_stage.move_relative({mapped: distance}, feedrate)

    # ── v7.1: Velocity & Timestamped Position API ──────────────────

    def send_velocity_xy(self, vx: float, vy: float) -> None:
        """
        P8.24: Send continuous velocity command to XY stage.

        Used by the MotionController for trajectory tracking. Velocity
        units match the stage's native format (typically µsteps/s for Prior).

        Args:
            vx: X velocity
            vy: Y velocity
        """
        if not self.xy_stage:
            return
        self.xy_stage.move_stage_at_velocity(vx, vy)

    def get_position_with_timestamp(self) -> dict:
        """
        P8.25: Get position with monotonic timestamp for Kalman filter.

        Returns a dict with timestamped XY and ZP positions. Uses
        direct (non-cached) reads for accuracy.

        Returns:
            {
                "t": float (time.monotonic()),
                "xy": (x, y, z) or (None, None, None),
                "zp": (z, p1, p2, p3) or (None, None, None, None),
            }
        """
        t_before = time.monotonic()
        xy = self.get_xy_position(cached=False)
        zp = self.get_zp_position(cached=False)
        t_after = time.monotonic()

        return {
            "t": (t_before + t_after) / 2.0,  # Midpoint estimate
            "latency_s": t_after - t_before,
            "xy": xy,
            "zp": zp,
        }

    def test_command_rate(self, num_tests: int = 20) -> dict:
        """
        P8.26: Test the XY stage command/response rate.

        Sends a series of position queries to measure communication latency.
        Safe to call at any time — only reads position, no movement.

        ⚠ SAFETY: This method does NOT move the stage. It only queries position.

        Args:
            num_tests: Number of position queries to time (default 20)

        Returns:
            {
                "avg_round_trip_ms": float,
                "min_round_trip_ms": float,
                "max_round_trip_ms": float,
                "max_command_hz": float,
                "num_tests": int,
                "controller": str,
            }
        """
        if not self.xy_stage:
            return {
                "error": "XY stage not connected",
                "avg_round_trip_ms": 0,
                "max_command_hz": 0,
            }

        latencies = []
        for _ in range(num_tests):
            t0 = time.monotonic()
            self.xy_stage.get_current_position()
            t1 = time.monotonic()
            latencies.append((t1 - t0) * 1000)  # ms
            time.sleep(0.01)  # Small gap to avoid flooding

        avg_ms = sum(latencies) / len(latencies)
        min_ms = min(latencies)
        max_ms = max(latencies)

        controller_name = "unknown"
        if hasattr(self.xy_stage, '_protocol') and self.xy_stage._protocol:
            controller_name = self.xy_stage._protocol.controller_name

        result = {
            "avg_round_trip_ms": round(avg_ms, 2),
            "min_round_trip_ms": round(min_ms, 2),
            "max_round_trip_ms": round(max_ms, 2),
            "max_command_hz": round(1000.0 / avg_ms, 1) if avg_ms > 0 else 0,
            "num_tests": num_tests,
            "controller": controller_name,
        }

        logger.info(
            f"Command rate test: avg={avg_ms:.1f}ms, "
            f"max_hz={result['max_command_hz']}, "
            f"controller={controller_name}"
        )
        return result

    # ── Shutdown ──────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Clean shutdown of all components."""
        logger.info("Shutting down StageController…")
        self._watchdog.stop()
        self._pos_poller.stop()
        self.disconnect_xbox()
        self.disconnect_stages()
        self.processor.stop()
        logger.info("Shutdown complete")
