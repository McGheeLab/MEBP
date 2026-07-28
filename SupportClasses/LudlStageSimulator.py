"""LudlStageSimulator — software simulator for a Ludl LEP MAC 5000 XY stage.

Sibling of :class:`XYStageSimulator` (which emulates a Prior ProScan II). Both
expose the SAME dual interface so :class:`XYStageManager` and the position poller
treat them identically:

  1. ``send_command(cmd) -> response``   (direct, for ``simulate=True``)
  2. ``write`` / ``flush`` / ``read_all`` / ``readline`` (pyserial-like)

Deliberately implemented STANDALONE (physics loop copied, not shared) so the
known-good Prior simulator is untouched — zero regression risk (see the update
plan, ``MEBP_v75x_LUDL_MAC5000_XY_CONTROLLER.md``).

Wire semantics (Ludl High-Level ASCII commands):
  - Positions/moves are in **counts** on the wire; the sim keeps µm internally
    and converts using ``position_scale`` (counts per µm, default 10 ⇒ 0.1
    µm/count) — so ``XYStageManager``'s µm→count send and count→µm parse are
    exercised end-to-end.
  - Acks: ``:A`` (ok) / ``:N -<code>`` (error).
  - ``MOVE X=<c> Y=<c>`` / ``MOVREL X=<c> Y=<c>`` → ``:A`` ; ``WHERE X Y`` →
    ``:A <cx> <cy>`` ; ``HERE X=0 Y=0`` (set position) → ``:A`` ; ``SPEED``
    (counts/s) / ``ACCEL`` (1-255) → ``:A`` ; ``HALT`` → ``:A`` ; ``VER`` →
    ``:A <version>`` ; ``/`` → ``B`` (busy) / ``N`` (idle).

All positions returned by :meth:`get_current_position` are in **µm** (like the
Prior sim), so nothing above ``XYStageManager`` sees counts.
"""

from __future__ import annotations

import logging
import math
import queue
import re
import threading
import json
import time
from pathlib import Path

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_LUDL_STATE_PATH = _PROJECT_ROOT / "config" / "sim_xy_state_ludl.json"

# Physical defaults (overridable via configure_from_protocol).
MAX_SPEED_UM_S = 50_000.0
MAX_ACCEL_UM_S2 = 400_000.0
DEFAULT_POSITION_SCALE = 10.0   # counts per micron (0.1 µm/count)
DEFAULT_BAUD = 9600
BITS_PER_BYTE = 10              # 8-N-1 framing for the baud timing model
PHYSICS_HZ = 200
SETTLE_THRESHOLD_UM = 0.5
_ACCEL_INDEX_MAX = 255.0       # Ludl ACCEL ramp index range top

_AXIS_VALUE_RE = re.compile(r"([XYZB])\s*=\s*(-?\d+(?:\.\d+)?)", re.IGNORECASE)
_AXIS_QUERY_RE = re.compile(r"([XYZB])\s*\?", re.IGNORECASE)


class LudlStageSimulator:
    """Ludl LEP MAC 5000 (High-Level ASCII) XY-stage simulator."""

    def __init__(
        self,
        position_scale: float = DEFAULT_POSITION_SCALE,
        update_rate_hz: int = PHYSICS_HZ,
        baud_rate: int = DEFAULT_BAUD,
        state_file: Path | str | None = None,
    ):
        self._position_scale = float(position_scale) or 1.0
        self._baud = baud_rate
        self._bytes_per_second = baud_rate / BITS_PER_BYTE
        self._state_file = Path(state_file) if state_file else _DEFAULT_LUDL_STATE_PATH

        # Position state (µm)
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_vx: float = 0.0
        self.current_vy: float = 0.0

        # Mode & targets
        self.mode: str = "idle"
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_vx: float = 0.0
        self.target_vy: float = 0.0

        # Motion limits (µm/s, µm/s²)
        self._max_speed = MAX_SPEED_UM_S
        self._max_accel = MAX_ACCEL_UM_S2
        self._settle = SETTLE_THRESHOLD_UM
        self.max_speed = self._max_speed  # compat

        # v7.5.x: per-axis RAW wire values as last SET, echoed back on a '?'
        # query — real MAC 5000 hardware can genuinely diverge between axes
        # (confirmed: a malformed command corrupted only X's SPEED register),
        # so the sim tracks each axis independently rather than one shared
        # value, exercising the exact read-back/divergence scenario.
        self._speed_raw: dict = {}   # {'X': counts/s, 'Y': counts/s}
        self._accel_raw: dict = {}   # {'X': ramp index 1-255, 'Y': ...}

        # Serial buffers
        self._rx_buffer: bytes = b""
        self._tx_queue: "queue.Queue[str]" = queue.Queue()
        self._serial_lock = threading.Lock()

        # Physics timing
        self.update_rate_hz = update_rate_hz
        self._update_interval = 1.0 / update_rate_hz
        self._last_update_time = time.time()

        # Threading
        self._lock = threading.Lock()
        self._running = False
        self._thread: "threading.Thread | None" = None

        self._load_state()

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._last_update_time = time.time()
        self._thread = threading.Thread(
            target=self._update_loop, daemon=True, name="LudlSimulator")
        self._thread.start()
        logger.info("Ludl MAC 5000 simulator started (baud=%s, scale=%.3f counts/µm, "
                    "max=%.0f µm/s)", self._baud, self._position_scale, self._max_speed)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._save_state()
        logger.info("Ludl MAC 5000 simulator stopped")

    def close(self) -> None:
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_open(self) -> bool:
        return self._running

    @property
    def in_waiting(self) -> int:
        return self._tx_queue.qsize() * 5

    def configure_from_protocol(self, max_speed=None, acceleration=None,
                                kp=None, position_scale=None) -> None:
        with self._lock:
            if max_speed is not None and max_speed > 0:
                self._max_speed = float(max_speed)
                self.max_speed = self._max_speed
            if acceleration is not None and acceleration > 0:
                # Ludl ACCEL is a 1-255 ramp index at the protocol layer, but the
                # 'acceleration' param here is the sim's physics ceiling (µm/s²).
                self._max_accel = float(acceleration) if acceleration > 1000 else MAX_ACCEL_UM_S2
            if position_scale:
                self._position_scale = float(position_scale) or 1.0

    # ── Serial interface (pyserial-compatible) ────────────────────

    def write(self, data: bytes) -> int:
        tx_time = len(data) / self._bytes_per_second
        time.sleep(tx_time)
        with self._serial_lock:
            self._rx_buffer += data
        return len(data)

    def flush(self) -> None:
        with self._serial_lock:
            buf = self._rx_buffer
            self._rx_buffer = b""
        text = buf.decode("ascii", errors="replace")
        lines = text.replace("\r\n", "\r").replace("\n", "\r").split("\r")
        for line in lines:
            line = line.strip()
            if not line:
                continue
            response = self._process_command(line)
            if response is not None:
                self._tx_queue.put(response + "\r")

    def read_all(self) -> bytes:
        responses = []
        while not self._tx_queue.empty():
            try:
                responses.append(self._tx_queue.get_nowait())
            except queue.Empty:
                break
        result = "".join(responses)
        if result:
            time.sleep(len(result) / self._bytes_per_second)
        return result.encode("ascii", errors="replace")

    def readline(self) -> bytes:
        try:
            resp = self._tx_queue.get(timeout=0.5)
            time.sleep(len(resp) / self._bytes_per_second)
            return resp.encode("ascii", errors="replace")
        except queue.Empty:
            return b""

    def reset_input_buffer(self) -> None:
        while not self._tx_queue.empty():
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break

    def reset_output_buffer(self) -> None:
        with self._serial_lock:
            self._rx_buffer = b""

    # ── Direct interface (simulate=True send_command path) ────────

    def send_command(self, command: str) -> str:
        """Process a command; block on absolute moves (like the Prior sim).

        Real MAC 5000 MOVE is async (``:A`` on accept, poll ``/`` for done), but
        blocking here gives deterministic move→position for callers/tests, and
        ``StageController`` still detects arrival by polling position either way.
        """
        response = self._process_command(command)
        with self._lock:
            is_moving = (self.mode == "absolute")
        if is_moving:
            self._wait_for_idle()
        return response

    # ── Command processing ────────────────────────────────────────

    @staticmethod
    def _axis_values(cmd: str) -> dict:
        """Extract {'x':.., 'y':.., 'z':..} from 'MOVE X=1 Y=2'-style commands."""
        out = {}
        for m in _AXIS_VALUE_RE.finditer(cmd):
            out[m.group(1).lower()] = float(m.group(2))
        return out

    @staticmethod
    def _axis_queries(cmd: str) -> list:
        """Extract axis letters from a 'SPEED X?' / 'ACCEL X? Y?'-style query."""
        return [m.group(1).upper() for m in _AXIS_QUERY_RE.finditer(cmd)]

    def _process_command(self, cmd: str) -> "str | None":
        raw = cmd.strip()
        if not raw:
            return None
        # Command-mode init bytes (0xFF 0x41) would arrive as replacement chars;
        # any leading non-alphanumeric junk before a keyword is ignored.
        upper = raw.upper()

        # Quick motion-status poll: '/' → 'B' (busy) / 'N' (idle)
        if raw == "/":
            with self._lock:
                busy = (self.mode != "idle") or abs(self.current_vx) > 0.1 or abs(self.current_vy) > 0.1
            return "B" if busy else "N"

        if upper.startswith("WHERE") or upper.startswith("W "):
            with self._lock:
                cx = round(self.current_x * self._position_scale)
                cy = round(self.current_y * self._position_scale)
            return f":A {cx} {cy}"

        if upper.startswith("MOVREL") or upper.startswith("MOVRL"):
            vals = self._axis_values(raw)
            with self._lock:
                self.mode = "absolute"
                if "x" in vals:
                    self.target_x = self.current_x + vals["x"] / self._position_scale
                if "y" in vals:
                    self.target_y = self.current_y + vals["y"] / self._position_scale
            return ":A"

        if upper.startswith("MOVE") or (upper.startswith("M") and "=" in raw):
            vals = self._axis_values(raw)
            if not vals:
                return ":N -3"  # missing parameter
            with self._lock:
                self.mode = "absolute"
                if "x" in vals:
                    self.target_x = vals["x"] / self._position_scale
                if "y" in vals:
                    self.target_y = vals["y"] / self._position_scale
            return ":A"

        if upper.startswith("HERE") or upper.startswith("H "):
            vals = self._axis_values(raw)
            with self._lock:
                if "x" in vals:
                    self.current_x = self.target_x = vals["x"] / self._position_scale
                if "y" in vals:
                    self.current_y = self.target_y = vals["y"] / self._position_scale
                self.current_vx = self.current_vy = 0.0
                self.mode = "idle"
            return ":A"

        if upper == "ZERO" or upper == "Z":
            with self._lock:
                self.current_x = self.current_y = 0.0
                self.target_x = self.target_y = 0.0
                self.current_vx = self.current_vy = 0.0
                self.mode = "idle"
            return ":A"

        if upper.startswith("SPEED") or upper.startswith("S "):
            # v7.5.x: '?' query form — echo back the RAW per-axis wire value
            # last SET (or a default before any SET), same as real hardware.
            queried = self._axis_queries(raw)
            if queried:
                with self._lock:
                    val = self._speed_raw.get(
                        queried[0], round(self._max_speed * self._position_scale))
                return f":A {int(val)}"
            vals = self._axis_values(raw)
            if vals:
                # counts/s → µm/s ceiling
                v = max(vals.values())
                with self._lock:
                    for ax, val in vals.items():
                        self._speed_raw[ax.upper()] = int(val)
                    self._max_speed = max(1.0, v / self._position_scale)
                    self.max_speed = self._max_speed
            return ":A"

        if upper.startswith("ACCEL") or upper.startswith("AC"):
            queried = self._axis_queries(raw)
            if queried:
                with self._lock:
                    val = self._accel_raw.get(queried[0], 1)
                return f":A {int(val)}"
            vals = self._axis_values(raw)
            if vals:
                idx = max(1.0, min(_ACCEL_INDEX_MAX, max(vals.values())))
                with self._lock:
                    for ax, val in vals.items():
                        self._accel_raw[ax.upper()] = int(val)
                    self._max_accel = (idx / _ACCEL_INDEX_MAX) * MAX_ACCEL_UM_S2
            return ":A"

        if upper.startswith("STSPEED"):
            return ":A"

        if upper in ("HALT",) or raw == "\\":
            with self._lock:
                self.mode = "idle"
                self.current_vx = self.current_vy = 0.0
                self.target_vx = self.target_vy = 0.0
            return ":A"

        if upper.startswith("STATUS") or upper.startswith("RDSTAT") or upper.startswith("RS"):
            with self._lock:
                busy = (self.mode != "idle")
            return "B" if busy else "N"

        if upper.startswith("VER") or upper == "V" or upper.startswith("VERSION"):
            return ":A MAC5000 SIM v1.0"

        if upper.startswith("BUILD") or upper.startswith("BU"):
            return ":A MAC5000-SIM BUILD"

        if upper.startswith("REMRES"):
            return ":A"

        # Unknown command → Ludl "unknown command" error.
        return ":N -1"

    # ── Physics ───────────────────────────────────────────────────

    def _wait_for_idle(self, timeout_s: float = 60.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._lock:
                if self.mode == "idle":
                    return True
            time.sleep(0.005)
        logger.warning("Ludl simulator settle timeout after %ss", timeout_s)
        return False

    def get_current_position(self) -> tuple[float, float, float]:
        with self._lock:
            return self.current_x, self.current_y, 0.0

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
                    dist = math.sqrt(ex * ex + ey * ey)
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

                elif self.mode == "idle":
                    if abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01:
                        self.current_vx = self._ramp(self.current_vx, 0.0, dt)
                        self.current_vy = self._ramp(self.current_vy, 0.0, dt)
                        self.current_x += self.current_vx * dt
                        self.current_y += self.current_vy * dt
                    else:
                        self.current_vx = self.current_vy = 0.0

            time.sleep(max(0.0, self._update_interval - (time.time() - t0)))

    # ── State persistence ─────────────────────────────────────────

    def _save_state(self) -> None:
        from datetime import datetime, timezone
        state = {
            "_description": "Simulated Ludl MAC 5000 state — saved on stop()",
            "_last_saved": datetime.now(timezone.utc).isoformat(),
            "position": {"x": self.current_x, "y": self.current_y},
            "settings": {
                "max_speed_um_s": self._max_speed,
                "position_scale": self._position_scale,
            },
        }
        try:
            self._state_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self._state_file, "w") as f:
                json.dump(state, f, indent=4)
        except Exception as e:
            logger.warning("LudlSim: failed to save state: %s", e)

    def _load_state(self) -> None:
        if not self._state_file.exists():
            return
        try:
            with open(self._state_file, "r") as f:
                state = json.load(f)
            if "position" in state:
                self.current_x = float(state["position"].get("x", 0.0))
                self.current_y = float(state["position"].get("y", 0.0))
                self.target_x = self.current_x
                self.target_y = self.current_y
            if "settings" in state:
                s = state["settings"]
                if "max_speed_um_s" in s:
                    self._max_speed = float(s["max_speed_um_s"])
                    self.max_speed = self._max_speed
        except Exception as e:
            logger.warning("LudlSim: failed to load state: %s", e)
