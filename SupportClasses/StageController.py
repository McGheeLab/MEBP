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
from SupportClasses.HardwareConfig import HardwareConfig
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
        # v7.2.6: S4-D status handler — track controller state from worker
        self._xbox_status: str = "disconnected"
        self._last_heartbeat_time: float = time.time()


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

                if "status" in msg:
                    self._last_heartbeat_time = time.time()
                    # v7.2.8: quiet heartbeat — only log on change
                    _prev = self._xbox_status
                    self._xbox_status = msg["status"]
                    if self._xbox_status != _prev:
                        logger.info(f"[Xbox] Status: {self._xbox_status}")
                elif "debug" in msg:
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

            # Heartbeat staleness: worker sends every 3s, stale after 10s
            if self._xbox_status in ("connected", "alive"):
                if time.time() - self._last_heartbeat_time > 10.0:
                    self._xbox_status = "disconnected"
                    logger.warning("[Xbox] Heartbeat stale — marking disconnected")

            time.sleep(0.02)

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
        zero_position: dict | None = None,  # v7.2.6: zero ref for clamping
    ):
        self.processor = processor
        self.stage = zp_stage
        self.safety_limits = safety_limits
        self._get_zp_position = get_zp_position
        self._zero_position = zero_position or {}  # v7.2.6: zero ref for jog clamping

        # Velocity state (updated by command handlers)
        self.vel_z: float = 0.0
        self.vel_p1: float = 0.0
        self.vel_p2: float = 0.0
        self.vel_p3: float = 0.0
        self._lock = threading.Lock()

        # Tuning parameters
        self.segment_time: float = 0.12   # seconds per move segment
        self.z_speed: float = 1.0         # Z speed multiplier
        self.p_speed: float = 0.5         # Pump speed: µL/s when hw config, mm/s otherwise
        self.max_speed: float = 1.0
        self._hardware_config = None      # Set via set_hardware_config()

        self._was_moving = False
        self._running = False
        self._thread: threading.Thread | None = None
        self._pump_disabled_warned: set = set()  # debounce warnings

        # Register processor handlers
        self.processor.register_handler("move_z_at_velocity", self._handle_z_vel)
        self.processor.register_handler("move_p1_at_velocity", self._handle_p1_vel)
        self.processor.register_handler("move_p2_at_velocity", self._handle_p2_vel)
        self.processor.register_handler("move_p3_at_velocity", self._handle_p3_vel)
        self.processor.register_handler("increment_zspeed_up", self._incr_z_up)
        self.processor.register_handler("increment_zspeed_down", self._incr_z_down)
        self.processor.register_handler("increment_pspeed_up", self._incr_p_up)
        self.processor.register_handler("increment_pspeed_down", self._incr_p_down)

        # v7.2.6: ZP registered handlers — track for unregister on stop()
        self._registered_handlers: list = [
            ("move_z_at_velocity",       self._handle_z_vel),
            ("move_p1_at_velocity",      self._handle_p1_vel),
            ("move_p2_at_velocity",      self._handle_p2_vel),
            ("move_p3_at_velocity",      self._handle_p3_vel),
            ("increment_zspeed_up",      self._incr_z_up),
            ("increment_zspeed_down",    self._incr_z_down),
            ("increment_pspeed_up",      self._incr_p_up),
            ("increment_pspeed_down",    self._incr_p_down),
        ]

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._jog_loop, daemon=True, name="ZPJog"
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: ZP unregister — prevents stale callbacks on reconnect.
        """
        self._running = False
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_relative({}, None)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)


    def set_hardware_config(self, config):
        """Enable µL-based pump jog. p_speed becomes µL/s."""
        self._hardware_config = config
        self._pump_disabled_warned.clear()  # re-evaluate on config change
        if config and config.configured_pump_ids:
            self.p_speed = 1.0  # sensible default: 1 µL/s
            logger.info("ZPJog: µL mode enabled, p_speed = 1.0 µL/s")

    @property
    def p_speed_is_uL(self) -> bool:
        """True when p_speed represents µL/s (HardwareConfig available)."""
        hw = self._hardware_config
        return hw is not None and bool(hw.configured_pump_ids)

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

    def _pump_vel_mm_s(self, raw: float, pump_id: str) -> float:
        """Convert raw Xbox axis to pump velocity in mm/s.

        When HardwareConfig is available, p_speed is in µL/s and we
        convert per-pump using the syringe geometry.  Otherwise p_speed
        is a raw mm/s multiplier (legacy).
        """
        hw = self._hardware_config
        if hw:
            pump_cfg = hw.pumps.get(pump_id)
            if pump_cfg and pump_cfg.is_configured:
                target_uL_s = raw * self.p_speed  # µL/s
                vel_mm_s = pump_cfg.uL_to_mm(abs(target_uL_s))
                vel_mm_s = min(vel_mm_s, self.max_speed)
                return vel_mm_s if target_uL_s >= 0 else -vel_mm_s
        # Fallback: mm/s mode
        return self._clamp_vel(raw, self.p_speed)

    # ── Command Handlers ──────────────────────────────────────────

    def _is_pump_enabled(self, pump_id: str) -> bool:
        """Check if pump is enabled in hardware config. True if no config."""
        hw = self._hardware_config
        if hw is None:
            return True
        pump_cfg = hw.pumps.get(pump_id)
        if pump_cfg is None:
            return True
        return pump_cfg.is_configured

    def _handle_z_vel(self, *args, **kwargs):
        raw = self._extract_velocity(*args, **kwargs)
        with self._lock:
            self.vel_z = self._clamp_vel(raw, self.z_speed)

    def _clamp_pump_flow(self, vel, pump_id):
        """Clamp pump velocity (mm/s) to max safe flow rate."""
        hw = self._hardware_config
        if hw is None:
            return vel
        sl = self.safety_limits
        if sl is None:
            return vel
        try:
            max_rate = sl.get_max_flow_rate(pump_id)
            if max_rate is not None and max_rate > 0:
                pump_cfg = hw.pumps.get(pump_id)
                if pump_cfg and pump_cfg.is_configured:
                    rate = abs(pump_cfg.mm_to_uL(abs(vel)))
                    if rate > max_rate:
                        clamped_mm = pump_cfg.uL_to_mm(max_rate)
                        vel = clamped_mm if vel > 0 else -clamped_mm
        except Exception:
            pass
        return vel

    def _handle_pump_vel(self, pump_id: str, vel_attr: str, *args, **kwargs):
        """Common handler for pump velocity commands."""
        raw = self._extract_velocity(*args, **kwargs)
        if not self._is_pump_enabled(pump_id):
            # Block movement — warn once per pump
            if pump_id not in self._pump_disabled_warned and abs(raw) > 0.05:
                self._pump_disabled_warned.add(pump_id)
                logger.warning(
                    f"[Xbox] {pump_id} is disabled — ignoring jog. "
                    f"Enable {pump_id} in Hardware Setup or change your Xbox mapping."
                )
            with self._lock:
                setattr(self, vel_attr, 0.0)
            return
        # Clear warning debounce when pump becomes enabled again
        self._pump_disabled_warned.discard(pump_id)
        vel = self._pump_vel_mm_s(raw, pump_id)
        vel = self._clamp_pump_flow(vel, pump_id)
        with self._lock:
            setattr(self, vel_attr, vel)

    def _handle_p1_vel(self, *args, **kwargs):
        self._handle_pump_vel("P1", "vel_p1", *args, **kwargs)

    def _handle_p2_vel(self, *args, **kwargs):
        self._handle_pump_vel("P2", "vel_p2", *args, **kwargs)

    def _handle_p3_vel(self, *args, **kwargs):
        self._handle_pump_vel("P3", "vel_p3", *args, **kwargs)

    def _incr_z_up(self, *a, **kw):
        self.z_speed = min(self.z_speed * 10, 100)
        logger.info(f"Z speed: {self.z_speed}")

    def _incr_z_down(self, *a, **kw):
        self.z_speed = max(self.z_speed / 10, 0.01)
        logger.info(f"Z speed: {self.z_speed}")

    def _incr_p_up(self, *a, **kw):
        self.p_speed = min(self.p_speed * 10, 100)
        unit = "µL/s" if self.p_speed_is_uL else "mm/s"
        logger.info(f"P speed: {self.p_speed} {unit}")

    def _incr_p_down(self, *a, **kw):
        self.p_speed = max(self.p_speed / 10, 0.01)
        unit = "µL/s" if self.p_speed_is_uL else "mm/s"
        logger.info(f"P speed: {self.p_speed} {unit}")


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
                        # v7.2.6: Subtract zero_position before clamping
                        # clamp_z/clamp_pump expect zero-referenced values
                        _zp = getattr(self, '_zero_position', {})
                        z0 = _zp.get('Z', 0.0)
                        # target_zr = (abs_pos + delta) - zero; clamp; new_delta = clamped_abs - abs_pos
                        dz = self.safety_limits.clamp_z((cz + dz) - z0) + z0 - cz
                        p10 = _zp.get('P1', 0.0)
                        dp1 = self.safety_limits.clamp_pump((cp1 + dp1) - p10, "P1") + p10 - cp1
                        p20 = _zp.get('P2', 0.0)
                        dp2 = self.safety_limits.clamp_pump((cp2 + dp2) - p20, "P2") + p20 - cp2
                        p30 = _zp.get('P3', 0.0)
                        dp3 = self.safety_limits.clamp_pump((cp3 + dp3) - p30, "P3") + p30 - cp3
                except Exception:
                    pass

            combined = math.sqrt(vz**2 + vp1**2 + vp2**2 + vp3**2)
            feedrate = max(combined * 60, 1)

            if self.safety_limits and self.safety_limits.enabled:
                feedrate = min(feedrate, self.safety_limits.max_z_feedrate)

            # v7.2.6: ZP jog serial guard — protect against disconnected stage
            try:
                self.stage.move_relative(
                    {"X": dz, "Y": dp1, "Z": dp2, "E": dp3}, feedrate
                )
            except Exception as e:
                logger.warning(f"[ZP] Jog move failed: {e}")
                with self._lock:
                    self.vel_z = self.vel_p1 = self.vel_p2 = self.vel_p3 = 0.0
                break

            time.sleep(self.segment_time)

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

        # v7.2.6: XY registered handlers
        self._registered_handlers: list = [
            ("move_stage_at_velocity", self._handle_vel),
            ("increment_xyspeed_up",   self._incr_up),
            ("increment_xyspeed_down", self._incr_down),
        ]


    # v7.2: Hardware config for µL conversion
    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Enable µL-based pump jog when hardware config available."""
        self._hardware_config = config

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._jog_loop, daemon=True, name="XYJog"
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: XY unregister — prevents stale callbacks on reconnect.
        """
        self._running = False
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_stage_at_velocity(0, 0)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)


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

    def _incr_up(self, *a, **kw):  # v7.2.7: decade speed
        self.xy_speed = min(self.xy_speed * 10, 10000)
        logger.info(f"XY speed: {self.xy_speed}")


    def _incr_down(self, *a, **kw):  # v7.2.7: decade speed
        self.xy_speed = max(self.xy_speed / 10, 1)
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

            if not is_moving:
                time.sleep(0.01)
                continue

            # Dampen near limits
            if is_moving and self.safety_limits and self.safety_limits.enabled and self._get_xy_position:
                try:
                    pos = self._get_xy_position()
                    if pos[0] is not None:
                        # BUG-5: margin=500 microsteps ≈ 50µm at 10 steps/µm
                        # This slows jog speed when within 50µm of a limit
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

            # v7.2.6: XY jog serial guard
            try:
                self.stage.move_stage_at_velocity(vx, vy)
            except Exception as e:
                logger.warning(f"[XY] Jog move failed: {e}")
                with self._lock:
                    self.vel_x = self.vel_y = 0.0
                break
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

        # v7.2.7: init _hardware_config
        self._hardware_config = None

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
        self._pos_poller = PositionPoller(poll_interval=0.3)  # v7.2.8: poll interval restored  # v7.2.6: poll interval 1.0s
        self._pos_poller.start()

        # Disconnect callback (GUI can set this)
        self.on_disconnect: Callable | None = None

        # v7.2: Hardware configuration (set by GUI when hardware setup completes)
        self._hardware_config: HardwareConfig | None = None

        # Register calibration handler
        self.processor.register_handler("zero_needle_pos", self._calibrate_zero)
        # v7.2.6: debug handler — ensures Xbox debug messages always dispatch
        self.processor.register_handler("debug", self._handle_debug)

    # ── Connection Management ─────────────────────────────────────

    def _handle_debug(self, *args, **kwargs) -> None:
        """v7.2.6: debug handler — logs Xbox worker debug messages."""
        msg = kwargs.get("message", "") or (args[0] if args else "")
        logger.info(f"[Debug] {msg}")

    def connect_stages(self, xy: bool = True, zp: bool = True) -> None:
        """Initialise and connect stages.

        Args:
            xy: If True, connect the XY stage (default True).
            zp: If True, connect the ZP stage (default True).
        """
        if xy and self.xy_stage is None:
            # v7.2.8: connection error handling
            try:
                self.xy_stage = XYStageManager(
                    simulate=self.simulate_xy,
                    controller_json=self.controller_json,
                )
            except (ConnectionError, ImportError, OSError) as e:
                logger.error(f"XY stage connection failed: {e}")
                self.xy_stage = None
                if self.on_disconnect:
                    self.on_disconnect("XY")
                return
            # v7.2.6: stop old jog handlers before creating new ones
            if self.xy_jog is not None:
                self.xy_jog.stop()
                self.xy_jog = None
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
            # v7.2.8: ZP connection error handling
            try:
                self.zp_stage = ZPStageManager(simulate=self.simulate_zp)
            except (ConnectionError, ImportError, OSError) as e:
                logger.error(f"ZP stage connection failed: {e}")
                self.zp_stage = None
                if self.on_disconnect:
                    self.on_disconnect("ZP")
                return
            # v7.2.6: stop old ZP jog handler
            if self.zp_jog is not None:
                self.zp_jog.stop()
                self.zp_jog = None
            self.zp_jog = ZPJogHandler(
                self.processor, self.zp_stage,
                safety_limits=self.safety_limits,
                get_zp_position=lambda: self._pos_poller.zp_position,
                zero_position=self.zero_position,
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

    # ── Convenience connection methods (used by Dashboard) ────────

    def connect_xy(self) -> None:
        """Connect only the XY stage."""
        self.connect_stages(xy=True, zp=False)

    def connect_zp(self) -> None:
        """Connect only the ZP stage."""
        self.connect_stages(xy=False, zp=True)

    def disconnect_xy(self) -> None:
        """Disconnect only the XY stage."""
        if self.xy_jog:
            self.xy_jog.stop()
            self.xy_jog = None
        if self.xy_stage:
            self.xy_stage.stop()
            self.xy_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        logger.info("XY stage disconnected")

    def disconnect_zp(self) -> None:
        """Disconnect only the ZP stage."""
        if self.zp_jog:
            self.zp_jog.stop()
            self.zp_jog = None
        if self.zp_stage:
            self.zp_stage.stop()
            self.zp_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        logger.info("ZP stage disconnected")

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

    def connect_xbox(self, mapping_file: str = "current_button_mapping.json",
                     use_thread: bool = False,
                     reconnect_timeout: float = 30.0) -> None:
        """Connect Xbox controller. v7.2.7: thread fallback

        Args:
            mapping_file: Path to button mapping JSON.
            use_thread: If True, run worker in a thread instead of process.
                        Use this for macOS Bluetooth controllers that are
                        invisible to spawned subprocesses.
            reconnect_timeout: Seconds to attempt reconnection after loss.
        """
        if self.xbox_process and self.xbox_process.is_alive():
            logger.warning("Xbox already connected")
            return
        # Also check thread-based worker
        if getattr(self, "_xbox_thread", None) and self._xbox_thread.is_alive():
            logger.warning("Xbox already connected (thread mode)")
            return

        from pathlib import Path as _Path
        self._mapping_file = str(_Path(mapping_file).resolve())

        if self.xy_stage is None and self.zp_stage is None:
            logger.warning(
                "Xbox started but no stages are connected -- "
                "controller input will have no effect until stages connect"
            )

        self.xbox_queue = Queue()

        _worker_kwargs = {
            "mapping_file": self._mapping_file,
            "reconnect_timeout": reconnect_timeout,
        }

        if use_thread:
            import threading
            self._xbox_thread = threading.Thread(
                target=xbox_polling_worker,
                args=(self.xbox_queue,),
                kwargs=_worker_kwargs,
                daemon=True,
                name="XboxWorkerThread",
            )
            self._xbox_thread.start()
            self.xbox_process = None  # Not using process mode
            logger.info(f"Xbox worker started (THREAD mode, mapping: {self._mapping_file})")
        else:
            self.xbox_process = Process(
                target=xbox_polling_worker,
                args=(self.xbox_queue,),
                kwargs=_worker_kwargs,
                daemon=True,
            )
            self.xbox_process.start()
            logger.info(f"Xbox worker started (PROCESS mode, mapping: {self._mapping_file})")

        self.xbox_poller = XboxQueuePoller(self.xbox_queue, self.processor)
        self.xbox_poller.start()
        self.xbox_poller._xbox_status = "waiting"  # v7.2.7: force initial status


    def disconnect_xbox(self) -> None:
        """Disconnect Xbox controller. v7.2.7: thread cleanup"""
        if self.xbox_poller:
            self.xbox_poller.stop()
            self.xbox_poller = None
        if self.xbox_process and self.xbox_process.is_alive():
            self.xbox_process.terminate()
            self.xbox_process.join(timeout=2.0)
            self.xbox_process = None
        # v7.2.7: thread mode cleanup
        _xt = getattr(self, "_xbox_thread", None)
        if _xt and _xt.is_alive():
            # Thread mode — worker checks queue; we signal via _running
            # but it's a daemon thread, so it dies when main exits.
            # We can't cleanly stop it, but we nil the poller so no more dispatch.
            pass
        self._xbox_thread = None
        self.xbox_queue = None
        logger.info("Xbox controller disconnected")


    @property  # v7.2.8: xbox_status property
    def xbox_status(self) -> str:
        """Return Xbox connection status string.

        Returns: 'disconnected', 'waiting', 'connected', or 'alive'.
        'waiting'   = worker running, searching for controller
        'connected' = controller just found
        'alive'     = controller confirmed active (heartbeat)
        """
        if self.xbox_poller is None:
            return "disconnected"
        return getattr(self.xbox_poller, "_xbox_status", "unknown")

    @property
    def is_xbox_connected(self) -> bool:
        """Backward-compatible bool: True when controller is active."""
        # v7.2.7: is_xbox_connected thread
        status = self.xbox_status
        if status in ("connected", "alive"):
            return True
        # Fallback: check if process or thread is alive
        if self.xbox_process and self.xbox_process.is_alive():
            return False  # Process alive but controller not found yet
        _xt = getattr(self, "_xbox_thread", None)
        if _xt and _xt.is_alive():
            return False  # Thread alive but controller not found yet
        return False


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

    def get_xy_position_mm(self, cached: bool = True) -> tuple:
        """Get XY position in mm (relative to zero reference).

        v7.3: Convenience method for callers that need mm.
        Returns (x_mm, y_mm) or (None, None) if not connected.
        """
        pos = self.get_xy_position(cached)
        if pos[0] is not None:
            zx = self.zero_position.get("x", 0)
            zy = self.zero_position.get("y", 0)
            return ((pos[0] - zx) / 1000.0, (pos[1] - zy) / 1000.0)
        return (None, None)


    def wait_for_xy_arrival(
        self, target_x_mm: float, target_y_mm: float,
        tolerance_mm: float = 0.1, timeout_s: float = 10.0,
    ) -> bool:
        """Block until XY stage reaches target position (zero-ref mm).

        v7.2.9: Safety method — call before Z descent to confirm XY is at
        the correct well position, preventing needle breakage.

        Returns True if position reached within tolerance, False on timeout.
        """
        import time
        import math

        if not self.is_xy_connected:
            return True  # No stage to wait for

        deadline = time.monotonic() + timeout_s
        poll_interval = 0.15  # 150ms between polls

        while time.monotonic() < deadline:
            pos = self.get_xy_position_mm(cached=False)
            if pos[0] is not None and pos[1] is not None:
                dx = pos[0] - target_x_mm
                dy = pos[1] - target_y_mm
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= tolerance_mm:
                    return True
            time.sleep(poll_interval)

        # Timeout — log warning but don't block forever
        pos = self.get_xy_position_mm(cached=False)
        logger.warning(
            f"wait_for_xy_arrival timeout ({timeout_s}s): "
            f"target=({target_x_mm:.2f}, {target_y_mm:.2f}), "
            f"actual={pos}")
        return False

    def wait_for_z_arrival(
        self, target_z_mm: float,
        tolerance_mm: float = 0.05, timeout_s: float = 10.0,
    ) -> bool:
        """Block until Z axis reaches target position (zero-ref mm).

        v7.2.9: Companion to wait_for_xy_arrival for hybrid execution.
        Returns True if position reached within tolerance, False on timeout.
        """
        import time
        import math

        if not self.is_zp_connected:
            return True

        deadline = time.monotonic() + timeout_s
        poll_interval = 0.15

        while time.monotonic() < deadline:
            pos = self.get_zp_position(cached=False)
            if pos[0] is not None:
                z_mm = (pos[0] - self.zero_position.get("Z", 0))
                if abs(z_mm - target_z_mm) <= tolerance_mm:
                    return True
            time.sleep(poll_interval)

        pos = self.get_zp_position(cached=False)
        logger.warning(
            f"wait_for_z_arrival timeout ({timeout_s}s): "
            f"target={target_z_mm:.2f}, actual={pos}")
        return False

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
        """Return current jog speeds as numeric values.

        v7.2.8: Reads .xy_speed/.z_speed/.p_speed directly
        to avoid broken @property decorators.
        """
        return {
            "xy": getattr(self.xy_jog, "xy_speed", 0) if self.xy_jog else 0,
            "z": getattr(self.zp_jog, "z_speed", 0) if self.zp_jog else 0,
            "p": getattr(self.zp_jog, "p_speed", 0) if self.zp_jog else 0,
        }
    @property
    def is_xy_connected(self) -> bool:
        return self.xy_stage is not None

    @property
    def is_zp_connected(self) -> bool:
        return self.zp_stage is not None

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

    def reset_pump_zero(self, pump: str) -> None:
        """
        v7.2.6: Reset zero reference for a single pump axis.

        Sets the current absolute position as the new zero point
        for the specified pump, without affecting other axes.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
        """
        if pump not in ("P1", "P2", "P3"):
            logger.warning(f"Invalid pump ID for zero reset: {pump}")
            return

        pos = self.get_zp_position(cached=True)
        if pos[0] is None:
            logger.warning(f"Cannot reset {pump} zero — ZP stage not connected")
            return

        idx = {"P1": 1, "P2": 2, "P3": 3}[pump]
        if idx < len(pos) and pos[idx] is not None:
            self.zero_position[pump] = pos[idx]
            logger.info(f"{pump} zero set to {pos[idx]:.3f} mm (absolute)")

            # Log the event
            self.position_logger.record(
                f"zero_reset_{pump.lower()}",
                zp_pos=pos,
                metadata={"pump": pump, "new_zero": pos[idx]},
            )

    def reset_z_zero(self) -> None:
        """
        v7.2.6: Reset zero reference for Z axis only.

        Sets the current absolute position as the new zero point
        for Z, without affecting XY or pump axes.
        """
        pos = self.get_zp_position(cached=True)
        if pos[0] is None:
            logger.warning("Cannot reset Z zero — ZP stage not connected")
            return

        self.zero_position["Z"] = pos[0]
        logger.info(f"Z zero set to {pos[0]:.3f} mm (absolute)")

        self.position_logger.record(
            "zero_reset_z",
            zp_pos=pos,
            metadata={"new_zero": pos[0]},
        )


    def move_xy_absolute(
        self, x: float, y: float, from_zero_ref: bool = True, fast: bool = False
    ) -> None:
        """Move XY to absolute position, optionally relative to zero ref."""
        """v7.3: Accept mm, convert to µm internally.

        When from_zero_ref=True, x/y are in mm (from WellPlate/trajectory).
        Converts to µm, applies safety limits, adds zero reference (µm),
        then sends to stage which expects µm (Prior manual page 36).
        """
        if not self.xy_stage:
            return

        if from_zero_ref:
            # Convert mm → µm, then add zero reference (which is in µm)
            x_um = x * 1000.0
            y_um = y * 1000.0
            if self.safety_limits.enabled:
                x_um, y_um = self.safety_limits.clamp_xy(x_um, y_um)
            x_um += self.zero_position["x"]
            y_um += self.zero_position["y"]
        else:
            # Legacy: raw values passed directly (assumed µm already)
            x_um = x
            y_um = y

        self.xy_stage.move_stage_to_position(x_um, y_um, fast)
    def move_xy_relative(self, dx: float, dy: float) -> None:
        """
        Move XY stage by a relative offset in µm.

        BUG-1 FIX: Sends relative moves directly to hardware, eliminating
        dependency on stale cached position data.

        Safety limits are projected from cached position (minor boundary
        imprecision is acceptable vs. the gross errors from the old approach).
        """
        if not self.xy_stage:
            return

        # Safety: project cached position + delta, clamp if needed
        if self.safety_limits.enabled:
            pos = self.get_xy_position(cached=True)
            if pos[0] is not None:
                zero_x = self.zero_position["x"]
                zero_y = self.zero_position["y"]
                cur_zr_x = pos[0] - zero_x
                cur_zr_y = pos[1] - zero_y
                new_zr_x = cur_zr_x + dx
                new_zr_y = cur_zr_y + dy
                clamped_x, clamped_y = self.safety_limits.clamp_xy(new_zr_x, new_zr_y)
                dx = clamped_x - cur_zr_x
                dy = clamped_y - cur_zr_y

        # v7.2.4: Verification logging — exact microstep values
        logger.debug(f"move_xy_relative: sending dx={round(dx)} dy={round(dy)} "
                     f"microsteps (raw: dx={dx:.2f} dy={dy:.2f})")
        self.xy_stage.move_stage_relative(dx, dy)

    def move_xy_relative_um(self, dx_um: float, dy_um: float) -> None:
        """
        Move XY stage by a relative offset in MICRONS.

        v7.2.5: The Prior ProScan controller accepts movement commands
        in microns directly. This method sends micron values without
        any microstep conversion.

        Safety limits are projected from cached position.
        """
        if not self.xy_stage:
            return

        # Safety: project cached position + delta, clamp if needed
        # Note: safety limits and positions are all in the same units (microns)
        if self.safety_limits.enabled:
            pos = self.get_xy_position(cached=True)
            if pos[0] is not None:
                zero_x = self.zero_position["x"]
                zero_y = self.zero_position["y"]
                cur_zr_x = pos[0] - zero_x
                cur_zr_y = pos[1] - zero_y
                new_zr_x = cur_zr_x + dx_um
                new_zr_y = cur_zr_y + dy_um
                clamped_x, clamped_y = self.safety_limits.clamp_xy(new_zr_x, new_zr_y)
                dx_um = clamped_x - cur_zr_x
                dy_um = clamped_y - cur_zr_y

        logger.debug(f"move_xy_relative_um: sending dx={dx_um:.1f} dy={dy_um:.1f} µm")
        self.xy_stage.move_stage_relative(dx_um, dy_um)



    def move_z_absolute(
        self, z_value: float, from_zero_ref: bool = True, fast: bool = False,
        feedrate_mm_min: float | None = None,
    ) -> None:
        """Move Z needle to absolute position.

        Args:
            z_value: Target Z in mm (zero-ref or raw).
            from_zero_ref: If True, add zero reference offset.
            fast: If True, use maximum feedrate.
            feedrate_mm_min: Optional per-move feedrate (mm/min).
        """
        if not self.zp_stage:
            return
        if self.safety_limits.enabled and from_zero_ref:
            z_value = self.safety_limits.clamp_z(z_value)
        position = z_value
        if from_zero_ref:
            position += self.zero_position["Z"]
        self.zp_stage.move_absolute(
            {AXIS_MAP["Z"]: position}, fast,
            feedrate_mm_min=feedrate_mm_min)

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

    def safe_travel_to(
        self,
        target_x_um: float,
        target_y_um: float,
        safe_z_mm: float,
        target_z_mm: float | None = None,
        fast_xy_speed_mm_s: float = 50.0,
    ) -> None:
        """v7.3.1: Safe 3-step travel: raise Z → fast XY → lower Z.

        Args:
            target_x_um: Absolute target X in µm.
            target_y_um: Absolute target Y in µm.
            safe_z_mm: Safe travel height in mm (zero-referenced).
            target_z_mm: Optional target Z after XY move (zero-referenced).
                         If None, stays at safe_z.
            fast_xy_speed_mm_s: XY travel speed in mm/s.
        """
        # Step 1: Raise Z to safe height
        if self.is_zp_connected:
            self.move_z_absolute(safe_z_mm, from_zero_ref=True)

        # Step 2: Fast XY travel
        if self.is_xy_connected:
            if hasattr(self, 'xy_stage') and self.xy_stage:
                if hasattr(self.xy_stage, 'set_speed_mm_s'):
                    self.xy_stage.set_speed_mm_s(fast_xy_speed_mm_s)
                else:
                    self.xy_stage.set_velocity(100)
            self.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)

        # Step 3: Lower Z to target
        if self.is_zp_connected and target_z_mm is not None:
            self.move_z_absolute(target_z_mm, from_zero_ref=True)

        logger.info(f"Safe travel to ({target_x_um:.0f}, {target_y_um:.0f}) µm, "
                    f"safe_z={safe_z_mm:.2f} mm")

    def is_pump_enabled(self, pump: str) -> bool:
        """Check if a pump is enabled in the hardware config."""
        hw = self._hardware_config
        if hw is None:
            return True  # No config → allow all (legacy behavior)
        pump_cfg = hw.pumps.get(pump)
        if pump_cfg is None:
            return True  # Unknown pump → allow
        return pump_cfg.is_configured

    def move_pump_relative(
        self, pump: str, distance: float, feedrate: float | None = None
    ) -> None:
        """Move a pump (P1/P2/P3) by relative distance."""
        if not self.zp_stage:
            return
        if not self.is_pump_enabled(pump):
            logger.warning(f"Pump {pump} is disabled — move blocked")
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
        units match the stage's native format (µm/s for Prior (VS command default unit per manual)).

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


    # ══════════════════════════════════════════════════════════════
    #  v7.2: µL-Based Pump Control
    # ══════════════════════════════════════════════════════════════

    def set_hardware_config(self, config: HardwareConfig) -> None:
        """
        Set the hardware configuration. Called by MainWindow when
        hardware setup is completed or modified.

        Enables µL-based pump methods and updates safety limits.
        """
        self._hardware_config = config
        if config:
            for pid in ["P1", "P2", "P3"]:
                pump_cfg = config.pumps.get(pid)
                if pump_cfg and pump_cfg.is_configured:
                    logger.debug(f"{pid}: syringe={pump_cfg.syringe.volume_uL}µL")
        logger.info("StageController: hardware config updated")

        # Propagate to ZP jog handler for µL pump jog
        if self.zp_jog and hasattr(self.zp_jog, 'set_hardware_config'):
            self.zp_jog.set_hardware_config(config)

        # Update safety limits from hardware config
        if self.safety_limits:
            self.safety_limits.update_from_hardware_config(config)

    @property
    def hardware_config(self) -> HardwareConfig | None:
        """Get the current hardware configuration."""
        return self._hardware_config

    def move_pump_uL(
        self, pump: str, volume_uL: float, rate_uL_s: float | None = None
    ) -> None:
        """
        Move a pump by a specified volume in µL.

        This is the primary pump movement method for v7.2+.
        Converts µL → mm using the syringe spec, and µL/s → mm/min
        for the feedrate.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
            volume_uL: Volume to dispense (+) or aspirate (-) in µL
            rate_uL_s: Flow rate in µL/s. If None, uses default feedrate.

        Raises:
            ValueError: If pump has no syringe configured
        """
        if not self._hardware_config:
            raise ValueError("No hardware config — complete Hardware Setup first")

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            raise ValueError(f"{pump}: No syringe configured")

        # Convert µL → mm
        distance_mm = pump_cfg.uL_to_mm(volume_uL)

        # Convert rate µL/s → mm/min
        feedrate_mm_min = None
        if rate_uL_s is not None:
            feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(rate_uL_s))
            # Clamp flow rate via safety limits
            if self.safety_limits.enabled:
                clamped = self.safety_limits.clamp_flow_rate(rate_uL_s, pump)
                if abs(clamped) != abs(rate_uL_s):
                    feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(clamped))

        logger.debug(
            f"move_pump_uL({pump}, {volume_uL:+.3f} µL"
            + (f", {rate_uL_s:.3f} µL/s" if rate_uL_s else "")
            + f") → {distance_mm:+.5f} mm"
            + (f", {feedrate_mm_min:.1f} mm/min" if feedrate_mm_min else "")
        )
        self.move_pump_relative(pump, distance_mm, feedrate_mm_min)

    def get_pump_position_uL(self, pump: str) -> float | None:
        """
        Get the current pump position in µL (relative to zero reference).

        Returns None if unavailable or no syringe configured.
        """
        if not self._hardware_config:
            return None

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            return None

        pos = self.get_zp_position(cached=True)
        if pos is None or pos[0] is None:
            return None

        idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
        if idx >= len(pos):
            return None

        pos_mm = pos[idx]
        zero_ref = self.zero_position.get(pump, 0)
        relative_mm = pos_mm - zero_ref

        try:
            return pump_cfg.mm_to_uL(relative_mm)
        except ValueError:
            return None

    def get_all_pump_positions_uL(self) -> dict[str, float | None]:
        """Get all pump positions in µL as a dict."""
        return {pid: self.get_pump_position_uL(pid) for pid in ["P1", "P2", "P3"]}

    def extrude_uL(
        self, pump: str, volume_uL: float, rate_uL_s: float | None = None
    ) -> bool:
        """
        Extrude (dispense) a specific volume with fluid column tracking.

        Positive volume_uL = dispense, negative = aspirate.
        Updates FluidColumn in HardwareConfig if available.

        Returns True if executed successfully.
        """
        if not self._hardware_config:
            logger.error("No hardware config — cannot extrude")
            return False

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            logger.error(f"{pump}: not configured — cannot extrude")
            return False

        # Track fluid column
        if volume_uL > 0:
            if not pump_cfg.fluid_column.can_dispense(volume_uL):
                logger.warning(
                    f"{pump}: Requested {volume_uL:.2f} µL but only "
                    f"{pump_cfg.fluid_column.ink_volume_uL:.2f} µL available"
                )
            pump_cfg.fluid_column.dispense(volume_uL)
        elif volume_uL < 0 and pump_cfg.ink:
            pump_cfg.fluid_column.aspirate_ink(abs(volume_uL), pump_cfg.ink)

        self.move_pump_uL(pump, volume_uL, rate_uL_s)
        return True