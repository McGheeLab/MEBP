"""
XY Stage Simulator v6 — Diagnostic-profile-driven timing — Prior ProScan II with realistic serial timing.

Implements both interfaces:
  1. send_command(cmd) → response  (direct, for simulate=True mode)
  2. write/flush/read_all          (pyserial-like, for serial interface testing)

Timing loaded from config/xy_diagnostic_profile.json at import time.
Diagnostic results (38400 baud, 2026-03-09):
  ALL commands: ~4.2ms round-trip, ~240 Hz burst, ~41 Hz sustained (with pacing)
  Processing (excl baud delay): ~0ms — baud delay accounts for entire round-trip
  VS is NOT slower than P — the old 800ms value was incorrect
  Sine-wave tracking: clean to 1.0 Hz (1885 µm/s peak) at 50 Hz VS rate

All positions in µm per Prior manual page 36.
"""

from __future__ import annotations

import logging
import math
import queue
import threading
import json
import time
from pathlib import Path

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════
#  Physical constants
# ═══════════════════════════════════════════════════════════════════

MAX_SPEED_UM_S = 20_000.0
MAX_ACCEL_UM_S2 = 400_000.0  # v7.2.8-cal: real settles 300µm in 59ms
DEFAULT_SPEED_PCT = 50
DEFAULT_ACCEL_PCT = 50
MICROSTEPS_PER_MICRON = 10
DEFAULT_KP = 50.0  # v7.2.8-cal: faster convergence to match real settle time
SETTLE_THRESHOLD_UM = 0.5
PHYSICS_HZ = 200

# Serial timing
DEFAULT_BAUD = 38400
BITS_PER_BYTE = 10          # 8N1: start + 8 data + stop
# v7.2.8: load timing from diagnostic profile
# Hardcoded fallback — used only if config/xy_diagnostic_profile.json is missing.
# These defaults are from the 2026-03-09 diagnostic run at 38400 baud.
# v7.2.8-cal: Per-command processing measured from sim_calibration_report.json.
# Real ProScan II @ 38400 baud: position query = 4.16ms total round-trip.
# Controller processing is ~0ms. The 4.16ms is entirely baud-rate TX/RX delay.
# Sustained poll rate: ~41 Hz (with 20ms inter-command pacing).
# These near-zero values let the baud-rate model handle all timing.
# v7.2.8-cal: Tuned from sim_calibration_report.json
# Real ProScan II: 4.16ms total round-trip at 38400 baud.
# TX delay (0.52ms) + RX delay (3.6ms) = 4.12ms.
# Controller processing = 4.16 - 4.12 = ~0ms.
# Set processing times to near-zero; baud model handles the rest.
PROCESSING_TIMES = {
    "position":  0.0001,   # 12ms — P query (measured avg)
    "move":      0.0001,   # 12ms — G/GR: responds immediately, stage moves async
    "velocity":  0.0001,   # 12ms — VS: responds immediately in COMP,0 mode
    "setting":   0.0001,   # 12ms — SMS/SAS/SCS register write
    "stop":      0.0001,   # 12ms — I/K
    "default":   0.0001,   # 12ms — everything else
}
PROCESSING_TIME_S = 0.0001  # v7.2.8-cal: tuned from real hardware  # backward compat

# Diagnostic profile path (relative to project root)
_DIAGNOSTIC_PROFILE_PATH = "config/xy_diagnostic_profile.json"


def _load_diagnostic_profile():
    """Load timing constants from xy_diagnostic_profile.json.

    Searches for the profile JSON relative to this file's location,
    walking up to find the project root (directory containing config/).

    Returns:
        (processing_times_dict, baud_rate, stage_info_dict) or defaults.
    """
    # Walk up from this file to find project root
    here = Path(__file__).resolve().parent
    for ancestor in [here] + list(here.parents):
        candidate = ancestor / _DIAGNOSTIC_PROFILE_PATH
        if candidate.exists():
            try:
                with open(candidate) as f:
                    profile = json.load(f)

                # Extract processing times
                derived = profile.get("simulator_derived_constants", {})
                proc_times = {
                    "position": derived.get("processing_time_position_s", 0.008),
                    "move":     derived.get("processing_time_move_s", 0.008),
                    "velocity": derived.get("processing_time_velocity_s", 0.008),
                    "setting":  derived.get("processing_time_setting_s", 0.008),
                    "stop":     derived.get("processing_time_stop_s", 0.005),
                    "default":  derived.get("processing_time_default_s", 0.008),
                }

                # Extract baud rate
                comm = profile.get("communication", {})
                baud = comm.get("baud_rate", DEFAULT_BAUD)

                # Extract stage info
                stage_info = profile.get("stage_info", {})

                logger.info(
                    f"Loaded XY diagnostic profile from {candidate} "
                    f"(baud={baud}, VS={proc_times['velocity']*1000:.0f}ms)"
                )
                return proc_times, baud, stage_info

            except Exception as e:
                logger.warning(f"Failed to load diagnostic profile {candidate}: {e}")
                break

    logger.info("No diagnostic profile found — using default timing constants")
    return dict(PROCESSING_TIMES), DEFAULT_BAUD, {}


# Load at module import time so all instances share the same profile
PROCESSING_TIMES, _PROFILE_BAUD, _PROFILE_STAGE_INFO = _load_diagnostic_profile()
PROCESSING_TIME_S = PROCESSING_TIMES.get("default", 0.008)


class XYStageSimulator:
    """
    Prior ProScan II simulator with baud-rate-accurate serial timing.

    Two interfaces:
      Direct:  response = sim.send_command("G 19300,0")
      Serial:  sim.write(b"G 19300,0\\r"); sim.flush(); data = sim.read_all()

    Both produce identical results. The serial interface adds realistic
    baud-rate delays so position polling and command throughput match
    real hardware behavior.
    """

    def __init__(
        self,
        microsteps_per_micron: float = MICROSTEPS_PER_MICRON,
        update_rate_hz: int = PHYSICS_HZ,
        baud_rate: int = _PROFILE_BAUD,
    ):
        self._usteps_per_um = microsteps_per_micron
        self._baud = baud_rate
        self._bytes_per_second = baud_rate / BITS_PER_BYTE

        # ── Position state (µm) ───────────────────────────────────
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_vx: float = 0.0
        self.current_vy: float = 0.0

        # ── Mode & targets ────────────────────────────────────────
        self.mode: str = "idle"
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_vx: float = 0.0
        self.target_vy: float = 0.0

        # ── Settings (1-100 percentage) ───────────────────────────
        self._speed_pct: int = DEFAULT_SPEED_PCT
        self._accel_pct: int = DEFAULT_ACCEL_PCT
        self._scurve_pct: int = 50
        self._max_speed = (self._speed_pct / 100.0) * MAX_SPEED_UM_S
        self._max_accel = (self._accel_pct / 100.0) * MAX_ACCEL_UM_S2
        self._kp = DEFAULT_KP
        self._settle = SETTLE_THRESHOLD_UM
        self._step_x: float = 100.0
        self._step_y: float = 100.0

        # Compat property
        self.max_speed = self._max_speed

        # ── Serial buffers ────────────────────────────────────────
        self._rx_buffer: bytes = b""              # incoming from "host"
        self._tx_queue: queue.Queue[str] = queue.Queue()  # responses waiting
        self._serial_lock = threading.Lock()

        # ── Stage info (v7.2.8: from diagnostic profile) ──────────
        self._stage_name = _PROFILE_STAGE_INFO.get("stage_name", "H101 Simulator")
        self._size_x_mm = _PROFILE_STAGE_INFO.get("size_x_mm", 108)
        self._size_y_mm = _PROFILE_STAGE_INFO.get("size_y_mm", 71)

        # ── Physics timing ────────────────────────────────────────
        self.update_rate_hz = update_rate_hz
        self._update_interval = 1.0 / update_rate_hz
        self._last_update_time = time.time()

        # ── Threading ─────────────────────────────────────────────
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._last_update_time = time.time()
        self._thread = threading.Thread(
            target=self._update_loop, daemon=True, name="XYSimulator")
        self._thread.start()
        logger.info(
            f"XY stage simulator started "
            f"(baud={self._baud}, SMS={self._speed_pct}, "
            f"max={self._max_speed:.0f} µm/s)")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("XY stage stopped")

    def close(self) -> None:
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_open(self) -> bool:
        """Pyserial compatibility."""
        return self._running

    @property
    def in_waiting(self) -> int:
        """Pyserial compatibility — bytes available to read."""
        return self._tx_queue.qsize() * 5  # approximate

    def configure_from_protocol(self, max_speed=None, acceleration=None,
                                kp=None) -> None:
        with self._lock:
            if kp is not None and kp > 0:
                self._kp = kp

    # ══════════════════════════════════════════════════════════════
    #  SERIAL INTERFACE (pyserial-compatible)
    # ══════════════════════════════════════════════════════════════

    def write(self, data: bytes) -> int:
        """Buffer incoming bytes. Simulates TX time at baud rate."""
        tx_time = len(data) / self._bytes_per_second
        time.sleep(tx_time)
        with self._serial_lock:
            self._rx_buffer += data
        return len(data)

    def flush(self) -> None:
        """Parse complete lines from buffer and process commands."""
        with self._serial_lock:
            buf = self._rx_buffer
            self._rx_buffer = b""

        text = buf.decode("utf-8", errors="replace")
        # Split on CR, LF, or CRLF
        lines = text.replace("\r\n", "\r").replace("\n", "\r").split("\r")
        for line in lines:
            line = line.strip()
            if not line:
                continue
            # Process command with per-type delay
            response = self._process_command(line)
            delay = self._get_processing_time(line)
            time.sleep(delay)
            if response is not None:
                self._tx_queue.put(response + "\r")

    def read_all(self) -> bytes:
        """Read all queued responses. Simulates RX time at baud rate."""
        responses = []
        while not self._tx_queue.empty():
            try:
                responses.append(self._tx_queue.get_nowait())
            except queue.Empty:
                break
        result = "".join(responses)
        if result:
            rx_time = len(result) / self._bytes_per_second
            time.sleep(rx_time)
        return result.encode("utf-8")

    def readline(self) -> bytes:
        """Read one line from response queue."""
        try:
            resp = self._tx_queue.get(timeout=0.5)
            rx_time = len(resp) / self._bytes_per_second
            time.sleep(rx_time)
            return resp.encode("utf-8")
        except queue.Empty:
            return b""

    def reset_input_buffer(self) -> None:
        while not self._tx_queue.empty():
            try: self._tx_queue.get_nowait()
            except queue.Empty: break

    def reset_output_buffer(self) -> None:
        with self._serial_lock:
            self._rx_buffer = b""

    # ══════════════════════════════════════════════════════════════
    #  DIRECT INTERFACE (for simulate=True send_command path)
    # ══════════════════════════════════════════════════════════════

    def send_command(self, command: str) -> str:
        """v7.2.7-block-v2: Delegate to raw handler, then block for absolute moves.

        Real Prior hardware: G/GR commands block until stage arrives.
        This wrapper replicates that behavior for the simulator.
        """
        response = self._send_command_raw(command)
        # Block if the command triggered an absolute move
        with self._lock:
            is_moving = (self.mode == "absolute")
        if is_moving:
            self._wait_for_idle()
        return response

    def _send_command_raw(self, command: str) -> str:
        """Process command with baud-rate-accurate timing.

        Simulates the full round-trip:
          TX command bytes → processing delay → RX response bytes
        """
        cmd = command.strip().replace("\r", "").replace("\n", "")
        if not cmd:
            cmd = "P"  # empty CR = position query

        # Simulate TX time
        tx_bytes = len(cmd) + 1  # +1 for CR
        time.sleep(tx_bytes / self._bytes_per_second)

        # Execute command
        response = self._process_command(cmd)

        # Per-command-type processing delay
        delay = self._get_processing_time(cmd)
        time.sleep(delay)

        # Simulate RX time
        rx_bytes = len(response) + 1  # +1 for CR
        time.sleep(rx_bytes / self._bytes_per_second)

        return response

    # ══════════════════════════════════════════════════════════════
    #  COMMAND PROCESSING (shared by both interfaces)
    # ══════════════════════════════════════════════════════════════

    def _get_processing_time(self, cmd: str) -> float:
        """Get realistic processing delay for this command type."""
        upper = cmd.upper().strip()
        if upper in ("P", "PS") or upper.startswith("P ") or upper.startswith("P,"):
            return PROCESSING_TIMES["position"]
        if upper.startswith("VS"):
            return PROCESSING_TIMES["velocity"]
        if upper.startswith("G") and not upper.startswith("GR"):
            return PROCESSING_TIMES["move"]
        if upper.startswith("GR"):
            return PROCESSING_TIMES["move"]
        if upper.startswith("SMS") or upper.startswith("SAS") or upper.startswith("SCS"):
            return PROCESSING_TIMES["setting"]
        if upper in ("I", "K"):
            return PROCESSING_TIMES["stop"]
        return PROCESSING_TIMES["default"]

    def _process_command(self, cmd: str) -> str:
        """Parse and execute a ProScan II command. Returns response string."""
        upper = cmd.upper().strip()
        if not upper:
            return self._cmd_P()

        # Position
        if upper == "P":
            return self._cmd_P()
        if upper == "PS":
            with self._lock:
                return f"{self.current_x:.1f},{self.current_y:.1f}"
        if upper.startswith("P ") or upper.startswith("P,"):
            return self._cmd_P_set(cmd)

        # Movement
        if upper.startswith("GR"):
            return self._cmd_GR(cmd)
        if upper.startswith("GX"):
            return self._cmd_GX(cmd)
        if upper.startswith("GY"):
            return self._cmd_GY(cmd)
        if upper.startswith("G ") or upper.startswith("G,"):
            return self._cmd_G(cmd)

        # Velocity
        if upper.startswith("VS"):
            return self._cmd_VS(cmd)

        # Settings
        if upper.startswith("SMS"):
            return self._cmd_SMS(cmd)
        if upper.startswith("SAS"):
            return self._cmd_SAS(cmd)
        if upper.startswith("SCS"):
            return self._cmd_SCS(cmd)

        # Home/stop
        if upper == "Z":
            return self._cmd_Z()
        if upper == "I":
            return self._cmd_I()
        if upper == "K":
            return self._cmd_K()
        if upper == "M":
            return self._cmd_M()

        # Joystick
        if upper == "H": return "0"
        if upper == "J": return "0"

        # Step size
        if upper == "X":
            return f"{self._step_x:.0f},{self._step_y:.0f}"

        # Directional
        for d in ("L", "R", "F", "B"):
            if upper == d or (upper.startswith(d) and len(upper) > 1 and
                              upper[1:].strip().lstrip(",").replace("-","").replace(".","").isdigit()):
                return self._cmd_dir(cmd, d)

        # Info
        if upper == "V":
            return "Prior ProScan II Simulator v7.3"
        if upper == "STAGE":
            return self._cmd_STAGE()
        if upper.startswith("$"):
            with self._lock:
                moving = abs(self.current_vx) > 0.1 or abs(self.current_vy) > 0.1
            return "1" if moving else "0"

        # Misc
        if upper.startswith("COMP"): return "0"
        if upper.startswith("SS"): return "0"
        if upper == "O": return str(self._speed_pct)
        return "0"

    # ── Command implementations ───────────────────────────────────

    @staticmethod
    def _parse_args(cmd: str, skip: int = 0) -> list[str]:
        import re
        raw = cmd[skip:] if skip else cmd
        return [p for p in re.split(r'[,\s\t=;:]+', raw.strip()) if p]

    def _cmd_P(self) -> str:
        with self._lock:
            return f"{self.current_x:.1f},{self.current_y:.1f},0.0"

    def _cmd_P_set(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 1)
        if len(parts) >= 2:
            try:
                with self._lock:
                    self.current_x = float(parts[0])
                    self.current_y = float(parts[1])
                    self.target_x = self.current_x
                    self.target_y = self.current_y
                return "0"
            except ValueError: pass
        return "E"

    def _cmd_G(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 1)
        if len(parts) < 2: return "E"
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError: return "E"
        with self._lock:
            self.mode = "absolute"
            self.target_x = x; self.target_y = y
        return "R"

    def _cmd_GR(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 2)
        if len(parts) < 2: return "E"
        try:
            dx, dy = float(parts[0]), float(parts[1])
        except ValueError: return "E"
        with self._lock:
            self.mode = "absolute"
            self.target_x = self.current_x + dx
            self.target_y = self.current_y + dy
        return "R"

    def _cmd_GX(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 2)
        if not parts: return "E"
        try: x = float(parts[0])
        except ValueError: return "E"
        with self._lock:
            self.mode = "absolute"; self.target_x = x
        return "R"

    def _cmd_GY(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 2)
        if not parts: return "E"
        try: y = float(parts[0])
        except ValueError: return "E"
        with self._lock:
            self.mode = "absolute"; self.target_y = y
        return "R"

    def _cmd_VS(self, cmd: str) -> str:
        """VS x,y[,u] — velocity. Default µm/s, with ,p = µsteps/s."""
        parts = self._parse_args(cmd, 2)
        if len(parts) < 2: return "E"
        try:
            vx_raw, vy_raw = float(parts[0]), float(parts[1])
        except ValueError: return "E"
        use_usteps = len(parts) >= 3 and parts[2].lower() == "p"
        with self._lock:
            if use_usteps:
                self.target_vx = vx_raw / self._usteps_per_um
                self.target_vy = vy_raw / self._usteps_per_um
            else:
                self.target_vx = vx_raw
                self.target_vy = vy_raw
            if abs(self.target_vx) < 0.01 and abs(self.target_vy) < 0.01:
                self.mode = "idle"
                self.target_vx = self.target_vy = 0.0
            else:
                self.mode = "velocity"
        return "R"

    def _cmd_SMS(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 3)
        if parts:
            try:
                m = max(1, min(100, int(float(parts[0]))))
                with self._lock:
                    self._speed_pct = m
                    self._max_speed = (m / 100.0) * MAX_SPEED_UM_S
                    self.max_speed = self._max_speed
                return "0"
            except ValueError: return "E"
        return str(self._speed_pct)

    def _cmd_SAS(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 3)
        if parts:
            try:
                a = max(1, min(100, int(float(parts[0]))))
                with self._lock:
                    self._accel_pct = a
                    self._max_accel = (a / 100.0) * MAX_ACCEL_UM_S2
                return "0"
            except ValueError: return "E"
        return str(self._accel_pct)

    def _cmd_SCS(self, cmd: str) -> str:
        parts = self._parse_args(cmd, 3)
        if parts:
            try:
                self._scurve_pct = max(1, min(100, int(float(parts[0]))))
                return "0"
            except ValueError: return "E"
        return str(self._scurve_pct)

    def _cmd_dir(self, cmd: str, direction: str) -> str:
        parts = self._parse_args(cmd, 1)
        amount = float(parts[0]) if parts else self._step_x
        with self._lock:
            self.mode = "absolute"
            if direction == "L": self.target_x = self.current_x - amount
            elif direction == "R": self.target_x = self.current_x + amount
            elif direction == "F": self.target_y = self.current_y + amount
            elif direction == "B": self.target_y = self.current_y - amount
        # v7.2.7: blocking move
        self._wait_for_idle()

        return "R"

    def _cmd_Z(self) -> str:
        with self._lock:
            self.current_x = self.current_y = 0.0
            self.target_x = self.target_y = 0.0
            self.current_vx = self.current_vy = 0.0
            self.target_vx = self.target_vy = 0.0
            self.mode = "idle"
        return "0"

    def _cmd_I(self) -> str:
        with self._lock:
            self.mode = "idle"
            self.target_vx = self.target_vy = 0.0
        return "R"

    def _cmd_K(self) -> str:
        with self._lock:
            self.mode = "idle"
            self.current_vx = self.current_vy = 0.0
            self.target_vx = self.target_vy = 0.0
        return "R"

    def _cmd_M(self) -> str:
        with self._lock:
            self.mode = "absolute"
            self.target_x = self.target_y = 0.0

        # v7.2.7: blocking move
        self._wait_for_idle()
        return "R"

    def _cmd_STAGE(self) -> str:
        return (
            f"STAGE = {self._stage_name}\n"
            f"TYPE = 1\n"
            f"SIZE_X = {self._size_x_mm} MM\n"
            f"SIZE_Y = {self._size_y_mm} MM\n"
            f"MICROSTEPS/MICRON = {int(self._usteps_per_um)}\n"
            f"LIMITS = NORMALLY CLOSED\n"
            f"END")

    # ── Direct position access ────────────────────────────────────

    def _wait_for_idle(self, timeout_s: float = 60.0):
        """v7.2.7: blocking move — wait for physics loop to reach target.

        Real Prior G command blocks until the stage arrives and returns R.
        This makes the simulator behave identically.
        """
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._lock:
                if self.mode == "idle":
                    return True
            time.sleep(0.005)  # 5ms poll — fast enough, low CPU
        logger.warning(f"Simulator settle timeout after {timeout_s}s")
        return False

    def get_current_position(self) -> tuple[float, float, float]:
        with self._lock:
            return self.current_x, self.current_y, 0.0

    # ── Physics Loop ──────────────────────────────────────────────

    def _ramp(self, current: float, target: float, dt: float) -> float:
        max_d = self._max_accel * dt
        diff = target - current
        return target if abs(diff) <= max_d else current + math.copysign(max_d, diff)

    def _update_loop(self) -> None:
        while self._running:
            t0 = time.time()
            with self._lock:
                dt = min(t0 - self._last_update_time, 0.05)
                self._last_update_time = t0

                if self.mode == "absolute":
                    ex = self.target_x - self.current_x
                    ey = self.target_y - self.current_y
                    dist = math.sqrt(ex*ex + ey*ey)
                    if dist < self._settle:
                        self.current_x = self.target_x
                        self.current_y = self.target_y
                        self.current_vx = self.current_vy = 0.0
                        self.mode = "idle"
                    else:
                        decel_speed = math.sqrt(2.0 * self._max_accel * dist)
                        speed_lim = min(self._max_speed, decel_speed)
                        nx, ny = ex / dist, ey / dist
                        self.current_vx = self._ramp(self.current_vx, nx * speed_lim, dt)
                        self.current_vy = self._ramp(self.current_vy, ny * speed_lim, dt)
                        self.current_x += self.current_vx * dt
                        self.current_y += self.current_vy * dt

                elif self.mode == "velocity":
                    tvx, tvy = self.target_vx, self.target_vy
                    mag = math.sqrt(tvx*tvx + tvy*tvy)
                    if mag > self._max_speed and mag > 0:
                        s = self._max_speed / mag
                        tvx *= s; tvy *= s
                    self.current_vx = self._ramp(self.current_vx, tvx, dt)
                    self.current_vy = self._ramp(self.current_vy, tvy, dt)
                    self.current_x += self.current_vx * dt
                    self.current_y += self.current_vy * dt

                elif self.mode == "idle":
                    if abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01:
                        self.current_vx = self._ramp(self.current_vx, 0.0, dt)
                        self.current_vy = self._ramp(self.current_vy, 0.0, dt)
                        self.current_x += self.current_vx * dt
                        self.current_y += self.current_vy * dt
                    else:
                        self.current_vx = self.current_vy = 0.0

            time.sleep(max(0.0, self._update_interval - (time.time() - t0)))
