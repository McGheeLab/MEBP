"""
ZP Stage Manager — Interface to the Marlin-based Z/Pump stage.

Manages communication with a 3D printer board that controls:
    - Z axis (vertical needle) → mapped to printer X axis
    - P1 syringe pump         → mapped to printer Y axis
    - P2 syringe pump         → mapped to printer Z axis
    - P3 syringe pump         → mapped to printer E axis

When ``simulate=True``, delegates to :class:`ZPStageSimulator`.
"""

from __future__ import annotations

import logging
import threading
import re
import time
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore[assignment]
    logger.warning("pyserial not installed — hardware mode unavailable")

from SupportClasses.ZPStageSimulator import ZPStageSimulator
from SupportClasses.ZPSerialTrace import tracer as _zp_tracer


# ═══════════════════════════════════════════════════════════════════
# Logical → Printer Axis Mapping
# ═══════════════════════════════════════════════════════════════════
#
# v7.4.2: AXIS_MAP and steps_per_mm are now instance-level — each
# ZPStageManager can carry its own per-machine mapping. The module-
# level constants below are kept as defaults for backwards compatibility
# and as the canonical starting point for new device profiles.

AXIS_MAP: dict[str, str] = {
    "Z": "X",    # Vertical needle → printer X
    "P1": "Y",   # Syringe pump 1  → printer Y
    "P2": "Z",   # Syringe pump 2  → printer Z
    "P3": "E",   # Syringe pump 3  → printer E
}

AXIS_MAP_REVERSE: dict[str, str] = {v: k for k, v in AXIS_MAP.items()}


class ZPStageManager:
    """
    Manager for the Marlin-based ZP stage (or simulator).

    Parameters:
        simulate:        If True, use the software simulator.
        baudrate:        Serial baud rate for real hardware.
        default_feedrate: Default movement feedrate (mm/min).
        steps_per_mm:    Steps-per-mm for the stepper calibration.
                         v7.4.2: accepts int (back-compat — applied to
                         every logical axis) or dict[str, int] keyed by
                         logical axis (Z, P1, P2, P3). Sign indicates
                         direction; negative inverts.
        axis_map:        v7.4.2: dict mapping logical axes (Z, P1, P2,
                         P3) to physical Marlin axes (X, Y, Z, E).
                         Defaults to the module-level AXIS_MAP.
    """

    DEFAULT_BAUDRATE = 38400
    DEFAULT_FEEDRATE = 200      # mm/min
    DEFAULT_STEPS_PER_MM = 5069

    # v7.5.x (flow-control Phase 1): max seconds to wait for Marlin's 'ok'
    # acknowledgement of a command. 'ok' normally returns in milliseconds (it
    # means "accepted into the planner buffer", NOT "motion complete"); the only
    # slow case is planner-buffer backpressure on a dense burst, which paces us
    # to the board's capacity — exactly the flow control we want. A 'busy'
    # keep-alive resets this window, so a genuinely long move keeps the link
    # alive; only true silence past this budget is treated as a failed command.
    DEFAULT_OK_TIMEOUT_S = 6.0

    # v7.5.x: once the board has stopped answering (a prior command's 'ok'
    # already timed out, or the poller's M114 failed), don't keep paying the
    # full DEFAULT_OK_TIMEOUT_S on EVERY subsequent command — that compounds
    # into multi-second hangs across a whole print/travel (and starves the XY
    # position reads via the shared poller). Drop to this short budget until a
    # command is acked again (which restores the full timeout). Healthy boards
    # never see this — 'ok' returns in milliseconds.
    SILENT_OK_TIMEOUT_S = 0.5

    # v7.5.x: absolute wall-clock ceiling for the per-command 'ok' handshake.
    # A Marlin 'busy' keep-alive RESETS the per-line wait window (so a genuinely
    # long move keeps the link alive), but with no upper bound a command whose
    # 'ok' NEVER comes — a stuck axis, or the documented bare-G0/F1.8 modal-
    # feedrate crawl that makes a ~21 mm Z retract take ~12 min — keeps
    # _read_until_ok (holding _serial_lock) alive forever, hanging the caller and
    # starving every other thread that needs the ZP port. This cap makes such a
    # move FAIL after a generous ceiling instead of hanging indefinitely. It is
    # deliberately large: an 'ok' means "admitted to the planner buffer" (ms on a
    # healthy board), so even a saturated planner admits the next command in far
    # less than this — only a truly stuck board reaches it. flush_moves (M400) is
    # already bounded separately.
    READ_OK_HARD_CAP_S = 180.0

    # Lines that mean "Marlin just (re)booted" — seeing one mid-session is a
    # board reset (position counter lost).
    _RESET_MARKERS = ("start", "firmware_name", "marlin")

    # v7.4.2: Per-axis default. Note P2 inverts direction (negative) —
    # matches the original hard-coded `Z-{spm}.00` in the M92 command
    # for the original single-machine config.
    DEFAULT_STEPS_PER_MM_DICT = {
        "Z": 5069, "P1": 5069, "P2": -5069, "P3": 5069,
    }

    # Position pattern for M114 response parsing
    _M114_PATTERN = re.compile(
        r"X:([+-]?\d+\.?\d*)\s+"
        r"Y:([+-]?\d+\.?\d*)\s+"
        r"Z:([+-]?\d+\.?\d*)\s+"
        r"E:([+-]?\d+\.?\d*)\s+"
        r"Count\s+X:([+-]?\d+)\s+Y:([+-]?\d+)\s+Z:([+-]?\d+)"
    )

    def __init__(
        self,
        simulate: bool = False,
        baudrate: int = DEFAULT_BAUDRATE,
        default_feedrate: float = DEFAULT_FEEDRATE,
        steps_per_mm: int | dict[str, int] = DEFAULT_STEPS_PER_MM,
        axis_map: dict[str, str] | None = None,
        preferred_port: str | None = None,
    ):
        # v7.2.6: lock before init
        self._serial_lock = threading.RLock()  # v7.2.6: ZP serial lock
        self.simulate = simulate
        # v7.2.6: ZP serial lock — thread-safe serial access
        self.baudrate = baudrate
        self.feedrate = default_feedrate
        # v7.4.2 hotfix: cached preferred port from last successful
        # connect — tried first to skip the rediscovery scan.
        self.preferred_port = preferred_port
        # Set after a successful connect so the caller can persist it.
        self.connected_port: str | None = None
        # v7.4.2: per-axis steps_per_mm + configurable axis map
        if isinstance(steps_per_mm, dict):
            self.steps_per_mm = dict(steps_per_mm)
        else:
            # Back-compat: int collapses to dict; preserve P2's
            # legacy negative sign so existing call sites continue to work.
            self.steps_per_mm = dict(self.DEFAULT_STEPS_PER_MM_DICT)
            for k in self.steps_per_mm:
                sign = -1 if self.steps_per_mm[k] < 0 else 1
                self.steps_per_mm[k] = sign * abs(int(steps_per_mm))
        self.axis_map = dict(axis_map) if axis_map else dict(AXIS_MAP)
        # v7.4.2 hotfix: per-logical-axis max accel (M201). Defaults
        # are sensible Marlin defaults; callers can override via
        # set_axis_accelerations().
        self.max_accel: dict[str, float] = {
            "Z": 100.0, "P1": 1000.0, "P2": 1000.0, "P3": 1000.0,
        }
        # v7.4.2 hotfix: per-logical-axis max feedrate (M203). Starts
        # as broadcast of default_feedrate; overridden by callers via
        # set_per_axis_max_feedrate(). Used at _setup_printer so M203
        # matches the software per-axis ceilings from connect-time.
        self.per_axis_max_feedrate: dict[str, float] = {
            "Z": default_feedrate, "P1": default_feedrate,
            "P2": default_feedrate, "P3": default_feedrate,
        }

        # Cached position values (updated on get_current_position)
        self.x_pos: float = 0.0
        self.y_pos: float = 0.0
        self.z_pos: float = 0.0
        self.e_pos: float = 0.0

        # v7.5.x ZP reconnect hotfix: True when the most recent M114 query
        # parsed a valid position. get_current_position() returns the cached
        # (possibly stale) floats on a failed read, so callers that need to
        # know whether the board actually answered (e.g. the PositionPoller's
        # liveness check) read this flag instead of inspecting the tuple.
        self._last_position_read_ok: bool = True

        # v7.5.x (flow-control Phase 1): set True if a mid-session board RESET
        # banner ("start" / "Marlin" / "FIRMWARE_NAME") is seen while awaiting
        # an 'ok'. A reset means Marlin lost its position counter — callers /
        # the reconnect+restore flow must re-declare position before trusting it.
        self._board_reset_detected: bool = False

        # v7.5.x flow-control telemetry — the Stress Test workflow reads these
        # to validate the 'ok' handshake under sustained load (commands issued
        # vs cleanly acknowledged, plus failures and mid-session resets). Best-
        # effort counters; snapshot via get_comm_counters(), zero via
        # reset_comm_counters().
        self.cmd_count: int = 0       # commands sent expecting an 'ok'
        self.ok_count: int = 0        # acknowledged with 'ok'
        self.ok_fail_count: int = 0   # no clean 'ok' (timeout / Error / reset)
        self.reset_count: int = 0     # mid-session board resets observed

        # Initialise communication backend
        if simulate:
            self.serial = ZPStageSimulator()
            self.serial.start()
            self._setup_printer()
            logger.info("ZP stage simulator started")
        else:
            self.serial = self._initialise_serial()
            if self.serial is not None:
                self._setup_printer()

    # ── Lifecycle ─────────────────────────────────────────────────

    def stop(self) -> None:
        """Disconnect and release resources."""
        if self.serial is None:
            return
        try:
            if self.simulate:
                self.serial.stop()
            else:
                # v7.5.x close-time reset mitigation (safety): de-assert
                # DTR/RTS *before* closing so the OS close doesn't pulse the
                # Marlin RESET line (a reset de-energizes the steppers → the
                # axes run to their extents). This is the ONLY software lever
                # for the close-time reset: the open path must keep its reset
                # (the ME3B V1 board needs it to connect — see
                # _try_open_marlin), so we can't pin the lines low for the
                # whole session. Whether this de-assert actually avoids the
                # reset is board/driver-polarity dependent; if it doesn't, the
                # reliable cure is the hardware auto-reset disable (cut RST-EN
                # / 10 µF RESET→GND). Connectivity is unaffected either way —
                # this runs only at shutdown, after the session is done.
                # v7.5.x CRASH FIX: serialize close() against any in-flight
                # serial I/O under _serial_lock. stop() is called from the
                # watchdog / poller thread on a mid-print disconnect, while the
                # retract thread may be BLOCKED inside self.serial.readline()
                # (flush_moves' M400 wait / the 'ok' handshake) holding the
                # lock. Calling CloseHandle on a USB-serial port that another
                # thread is mid-read on is a hard crash on Windows. Acquiring
                # the lock first makes the close wait for the in-flight read to
                # finish (each read self-times-out in ~2 s, so the lock frees);
                # the bounded timeout guarantees shutdown can't deadlock even if
                # a read is wedged — we then close best-effort regardless. The
                # timeout MUST exceed the longest continuous lock-hold so we
                # reliably wait the read out instead of giving up and closing
                # mid-read: that is flush_moves' M400 wait (up to ~15 s on a
                # genuinely long move), longer than _read_until_ok's
                # DEFAULT_OK_TIMEOUT_S. Use 17 s to cover it with margin. (In the
                # common disconnect case the board is already silent, so both
                # waits are capped to ~2 s and this never actually blocks long.)
                lock = getattr(self, "_serial_lock", None)
                got = (lock.acquire(timeout=17.0)
                       if lock is not None else False)
                try:
                    try:
                        self.serial.dtr = False
                        self.serial.rts = False
                    except Exception:
                        pass
                    self.serial.close()
                finally:
                    if got and lock is not None:
                        lock.release()
        except Exception as e:
            logger.warning(f"Error closing ZP stage: {e}")
        logger.info("ZP stage stopped")

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    # ── Serial Initialisation ─────────────────────────────────────
    #
    # v7.4.2 hotfix: speed up + correct connect path.
    #
    # Before: every port was probed twice — once at hardcoded 115200
    # to look for "FIRMWARE_NAME", then closed and reopened at
    # self.baudrate (38400) for the real session. Each open triggers
    # an Arduino-class DTR reset (Marlin board reboots ~2s), so the
    # naive loop took ~5-10s on a typical Mac with 3-4 random tty
    # ports (Bluetooth, debug-console, etc.).
    #
    # After:
    #   1. Rank candidate ports — hwid/description hints first
    #      (MARLIN, STM32, ATMEGA, VID:PID hints), then known-good
    #      naming patterns (usbmodem, usbserial, COM), then skip
    #      obvious junk (Bluetooth-*, debug-console, headphones).
    #   2. Try the previously-good port first (preferred_port).
    #   3. Single open per port at self.baudrate — if Marlin
    #      responds to M115, that same handle is the session.
    #   4. Drain Marlin's boot banner before checking the M115
    #      response, so a "start"/"echo:..." line doesn't look
    #      like a failed probe.

    # Platform-agnostic skip patterns. Lowercase substring match
    # against port.device.
    _SKIP_DEVICE_PATTERNS = (
        "bluetooth", "debug-console", "qc35", "headphone", "speaker",
        "incoming-port", "wireless",
    )
    # Lowercase substring match against port.device. Bumps score.
    _LIKELY_DEVICE_PATTERNS = (
        "usbmodem", "usbserial", "ttyusb", "ttyacm",
    )
    # Substring match (case-insensitive) against port.hwid + description.
    # Strong Marlin signal.
    _MARLIN_HWID_HINTS = (
        "MARLIN", "STM32", "ATMEGA", "ARDUINO",
        # Common Marlin board VID:PID — extend as needed
        "0483:5740",   # STM32 CDC
        "1A86:7523",   # CH340G (common Marlin clone boards)
        "0403:6001",   # FT232 (FTDI) — common adapter
        "10C4:EA60",   # CP210x — Prusa/etc.
        "2341:",       # Arduino VID
    )

    def _initialise_serial(self) -> Optional["serial.Serial"]:
        """Find and open the Marlin board (v7.4.2 hotfix: ranked + cached)."""
        if serial is None:
            raise ImportError("pyserial is required for hardware mode")

        try:
            port_infos = list(serial.tools.list_ports.comports())
        except Exception as e:
            logger.error(f"Error listing COM ports: {e}")
            return None

        ranked = self._rank_ports(port_infos, preferred=self.preferred_port)
        if not ranked:
            logger.warning("No serial ports detected")
            return None

        logger.info(
            "ZP probe order: " +
            ", ".join(f"{p.device}({p.description or 'n/a'})" for p in ranked))

        for port_info in ranked:
            device = port_info.device
            device_lc = (device or "").lower()
            if any(s in device_lc for s in self._SKIP_DEVICE_PATTERNS):
                logger.debug(f"Skipping non-candidate port {device}")
                continue

            ser = self._try_open_marlin(device)
            if ser is not None:
                logger.info(f"Marlin board connected on {device}")
                self.connected_port = device
                return ser

        logger.warning("No Marlin board found on any candidate port")
        return None

    @classmethod
    def _rank_ports(cls, port_infos, preferred: str | None = None):
        """Sort serial ports by Marlin-likeness, preferred port first.

        Ranking signals (higher = tried earlier):
          * preferred (last-known-good) port → 10000
          * description/hwid contains a Marlin/STM32/Arduino hint → +500
          * device name matches usbmodem/usbserial/ttyACM* → +100
          * device name matches a known-junk pattern → -1000 (de-facto skipped)
        """
        def score(p) -> int:
            device = (p.device or "").lower()
            hwid = (p.hwid or "").upper()
            desc = (p.description or "").upper()
            s = 0
            if preferred and p.device == preferred:
                s += 10000
            for hint in cls._MARLIN_HWID_HINTS:
                if hint.upper() in hwid or hint.upper() in desc:
                    s += 500
                    break
            for pat in cls._LIKELY_DEVICE_PATTERNS:
                if pat in device:
                    s += 100
                    break
            for skip in cls._SKIP_DEVICE_PATTERNS:
                if skip in device:
                    s -= 1000
                    break
            return s
        return sorted(port_infos, key=score, reverse=True)

    @staticmethod
    def _get_available_ports() -> list[str]:
        """List available serial port device names. v7.2 compat shim."""
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception as e:
            logger.error(f"Error listing COM ports: {e}")
            return []

    def _try_open_marlin(self, port: str,
                         probe_timeout: float = 2.0
                         ) -> Optional["serial.Serial"]:
        """Open ``port`` at self.baudrate, send M115, return the handle
        if Marlin responds with a FIRMWARE_NAME line.

        Single open — the same handle becomes the session if Marlin
        is found. No close-reopen → only one Arduino DTR reset.

        v7.5.x note: an attempt to suppress the open-time DTR/RTS reset
        (to also kill the close-time reset that slams the axes — see
        ``stop()``) by opening with the control lines pinned low BROKE
        connection on the ME3B V1 board — this board needs the DTR reset
        edge on open to start talking. The open path is therefore left
        at the default (resetting) behavior. The close-time reset is
        mitigated only in ``stop()``; the reliable cure is the hardware
        auto-reset disable (cut RST-EN jumper / 10 µF RESET→GND).
        """
        try:
            ser = serial.Serial(port, self.baudrate, timeout=probe_timeout)
        except (serial.SerialException, OSError) as e:
            # v7.5.x ZP reconnect hotfix: INFO (was debug) so a failed
            # reconnect shows the OS reason in the log. "Access is denied" /
            # "PermissionError" here means the port is still held open (the
            # previous handle wasn't released) — the signature of the
            # "dead until app restart" bug.
            logger.info(f"ZP probe: open {port} failed: {e}")
            return None
        try:
            # Brief settle — gives boards that DTR-reset on open a moment
            # to wake up enough to accept bytes. Bigger waits don't help
            # because Marlin's M115 response is what we actually look for.
            time.sleep(0.1)
            # Drain anything in the input buffer (the "start" banner,
            # bootloader chatter, etc.) so it doesn't precede our M115 reply.
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            ser.write(b"\nM115\n")
            ser.flush()
            # Marlin's M115 response is multiline and terminates with "ok".
            # Read up to a few KB or until we see FIRMWARE_NAME / ok.
            deadline = time.monotonic() + probe_timeout
            accumulated = ""
            while time.monotonic() < deadline:
                chunk = ser.read(512)
                if chunk:
                    try:
                        accumulated += chunk.decode("utf-8", errors="replace")
                    except Exception:
                        accumulated += str(chunk)
                if "FIRMWARE_NAME" in accumulated:
                    logger.info(f"ZP probe: {port} answered M115 (Marlin OK)")
                    try:
                        ser.reset_output_buffer()
                    except Exception:
                        pass
                    return ser
                if "ok" in accumulated.lower() and len(accumulated) > 80:
                    # End-of-response without FIRMWARE_NAME → not Marlin
                    break
                if not chunk:
                    # Nothing came back — brief pause before next loop
                    time.sleep(0.05)
            # v7.5.x ZP reconnect hotfix: INFO (was debug). An empty response
            # here means the port opened but nothing answered M115 within the
            # probe window — e.g. a board still rebooting from the DTR reset
            # that opening the port triggers, or a non-Marlin port.
            logger.info(
                f"ZP probe: {port} did not answer M115 "
                f"(response: {accumulated[:80]!r})")
            ser.close()
            return None
        except Exception as e:
            logger.info(f"ZP probe: {port} errored during M115 probe: {e}")
            try:
                ser.close()
            except Exception:
                pass
            return None

    @staticmethod
    def _is_marlin_printer(port: str) -> bool:
        """v7.2 compat shim. Kept so existing callers still link.

        v7.4.2 hotfix: ZPStageManager itself no longer goes through
        this method — see ``_try_open_marlin`` which uses
        ``self.baudrate`` consistently instead of the legacy
        hardcoded 115200.
        """
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(b"\nM115\n")
                response = ser.read_until(b"\n").decode("utf-8", errors="replace")
                return "FIRMWARE_NAME" in response
        except (serial.SerialException, OSError) as e:
            logger.debug(f"Probe failed on {port}: {e}")
            return False

    # ── Printer Setup ─────────────────────────────────────────────

    def _setup_printer(self) -> None:
        """Configure the printer for bioprinting operation.

        v7.4.2: M92 is now built per-logical-axis from
        ``self.steps_per_mm`` and routed to physical axes via
        ``self.axis_map``. Sign of each value is preserved (negative
        = direction-inverted).
        """
        commands = [
            "M302 S0",                          # Allow cold extrusion
            "M83",                               # Extruder relative mode
            "G91",                               # Relative positioning
            self._build_m203_command(),
            self._build_m92_command(),
            f"G0 F{self.feedrate}",              # Set initial feedrate
            "M220 S100",                         # Speed factor 100%
        ]
        for cmd in commands:
            self.send_data(cmd)
        logger.info(
            f"ZP stage configured (feedrate={self.feedrate}, "
            f"steps={self.steps_per_mm}, axis_map={self.axis_map})")

    def _build_m92_command(self) -> str:
        """v7.4.2: Build M92 G-code from current steps_per_mm + axis_map.

        Returns e.g. ``"M92 X5069.00 Y5069.00 Z-5069.00 E5069.00"``.
        """
        parts = ["M92"]
        for logical, physical in self.axis_map.items():
            steps = self.steps_per_mm.get(logical, self.DEFAULT_STEPS_PER_MM)
            parts.append(f"{physical}{steps:.2f}")
        return " ".join(parts)

    def set_axis_map(self, axis_map: dict[str, str]) -> None:
        """v7.4.2: Update the logical→physical axis mapping.

        Does NOT re-send M92 — caller should call
        :meth:`set_steps_per_mm(..., persist=True)` afterward if the
        mapping has changed and Marlin needs to be reprogrammed.
        """
        self.axis_map = dict(axis_map)
        logger.info(f"ZP axis_map updated: {self.axis_map}")

    def set_steps_per_mm(self, steps: dict[str, int],
                         persist: bool = True) -> None:
        """v7.4.2: Update per-axis steps_per_mm and optionally send M92.

        Args:
            steps:   dict keyed by logical axis (Z, P1, P2, P3).
            persist: If True (default), send M92 to Marlin so the new
                     calibration takes effect immediately. If False,
                     just updates local state.
        """
        self.steps_per_mm = dict(steps)
        if persist:
            cmd = self._build_m92_command()
            self.send_data(cmd)
            logger.info(f"ZP M92 sent: {cmd}")

    def set_zero(self, logical_axis: str) -> bool:
        """v7.4.2 hotfix: zero the Marlin physical axis mapped to
        ``logical_axis`` by sending a G92.

        Returns True if the command was queued, False if the logical
        axis isn't mapped. After this call, the next M114 query for
        that axis returns 0.
        """
        physical = self.axis_map.get(logical_axis)
        if not physical:
            logger.warning(
                f"set_zero({logical_axis}): no mapping in axis_map={self.axis_map}")
            return False
        self.send_data(f"G92 {physical}0")
        logger.info(f"ZP G92 {physical}0 sent (logical {logical_axis})")
        return True

    def set_position(self, logical_axis: str, value_mm: float) -> bool:
        """v7.5.x: override the Marlin physical axis mapped to
        ``logical_axis`` to ``value_mm`` by sending a G92.

        The arbitrary-value generalization of :meth:`set_zero` (which is
        just ``set_position(axis, 0.0)``). G92 redefines the firmware's
        current position counter for that axis *without moving* — after
        this call the next M114 query reports ``value_mm``.

        Used to re-sync the firmware position counter to the real
        physical position after a board power cycle: Marlin has no
        absolute encoder and powers up reporting 0, so only the operator
        can declare where the axis actually is.

        Returns True if the command was queued, False if ``logical_axis``
        isn't mapped.
        """
        physical = self.axis_map.get(logical_axis)
        if not physical:
            logger.warning(
                f"set_position({logical_axis}): no mapping in "
                f"axis_map={self.axis_map}")
            return False
        self.send_data(f"G92 {physical}{value_mm:.4f}")
        logger.info(
            f"ZP G92 {physical}{value_mm:.4f} sent (logical {logical_axis})")
        return True

    def _build_m203_command(self) -> str:
        """v7.4.2 hotfix: Build M203 G-code from per_axis_max_feedrate +
        axis_map. Returns e.g. ``"M203 X3000.00 Y3000.00 Z600.00 E200.00"``.

        Falls back to broadcasting ``self.feedrate`` to every mapped
        physical axis if per_axis_max_feedrate is empty.
        """
        parts = ["M203"]
        if self.per_axis_max_feedrate:
            for logical, physical in self.axis_map.items():
                feed = self.per_axis_max_feedrate.get(logical, self.feedrate)
                parts.append(f"{physical}{float(feed):.2f}")
        else:
            for physical in self.axis_map.values():
                parts.append(f"{physical}{float(self.feedrate):.2f}")
        return " ".join(parts)

    def set_per_axis_max_feedrate(self, feedrates: dict[str, float],
                                  persist: bool = True) -> None:
        """v7.4.2 hotfix: set per-logical-axis maximum feedrate via M203.

        ``feedrates`` keys are logical axes (Z, P1, P2, P3). The M203
        command is built from the current axis_map so each logical axis
        is routed to its physical Marlin letter.

        This is the per-axis replacement for :meth:`set_max_feedrate`,
        which sent the same value to every Marlin axis and so could
        never match a per-axis software ceiling.
        """
        self.per_axis_max_feedrate = dict(feedrates)
        if persist:
            cmd = self._build_m203_command()
            self.send_data(cmd)
            logger.info(f"ZP {cmd} sent")

    def set_axis_accelerations(self, accels: dict[str, float],
                               persist: bool = True) -> None:
        """v7.4.2 hotfix: set per-logical-axis maximum acceleration via M201.

        ``accels`` keys are logical axes (Z, P1, P2, P3). The M201
        command is built from the current axis_map (each logical axis
        is routed to its physical Marlin letter).

        Marlin's M201 expects mm/s²; the command applies to G0/G1
        moves until overridden.
        """
        self.max_accel = dict(accels)
        if persist:
            parts = ["M201"]
            for logical, accel in accels.items():
                physical = self.axis_map.get(logical)
                if physical:
                    parts.append(f"{physical}{float(accel):.2f}")
            cmd = " ".join(parts)
            self.send_data(cmd)
            logger.info(f"ZP {cmd} sent")

    def query_settings(self, timeout: float = 2.0) -> dict:
        """v7.4.2 hotfix: send M503 and parse Marlin's echoed settings.

        Returns a permissive dict with whatever was successfully parsed.
        Example::

            {
                "steps_per_mm": {"X": 5069.0, "Y": 5069.0, "Z": 5070.0, "E": 5069.0},
                "max_feedrate": {"X": 3000.0, "Y": 3000.0, "Z": 600.0, "E": 200.0},
                "max_accel":    {"X": 1000.0, "Y": 1000.0, "Z": 100.0, "E": 1000.0},
            }

        Per-build Marlin output varies; the parser is best-effort —
        unparseable lines are silently ignored.
        """
        result = {"steps_per_mm": {}, "max_feedrate": {}, "max_accel": {}}
        if self.serial is None:
            return result
        try:
            with self._serial_lock:
                try:
                    self.serial.reset_input_buffer()
                except Exception:
                    pass
                self.serial.write(b"M503\n")
                self.serial.flush()
            deadline = time.monotonic() + timeout
            accumulated = ""
            while time.monotonic() < deadline:
                with self._serial_lock:
                    try:
                        chunk = self.serial.read(1024)
                    except Exception:
                        break
                if chunk:
                    try:
                        accumulated += chunk.decode("utf-8", errors="replace")
                    except Exception:
                        accumulated += str(chunk)
                # Stop early if we've seen a long enough echo
                if "ok" in accumulated.lower() and len(accumulated) > 200:
                    break
                if not chunk:
                    time.sleep(0.05)
            self._parse_m503(accumulated, result)
        except Exception as e:
            logger.warning(f"query_settings failed: {e}")
        return result

    _M503_FIELD_RE = re.compile(
        r"\b(?P<letter>[XYZE])(?P<value>[+-]?\d+\.?\d*)")

    def _parse_m503(self, text: str, result: dict) -> None:
        """Parse M503 echo into result dict. Permissive; ignores noise."""
        for line in text.splitlines():
            up = line.strip().upper()
            # Marlin typically prefixes echoes with "echo:" or "echo: "
            up = up.replace("ECHO:", "").strip()
            if up.startswith("M92"):
                bucket = "steps_per_mm"
            elif up.startswith("M203"):
                bucket = "max_feedrate"
            elif up.startswith("M201"):
                bucket = "max_accel"
            else:
                continue
            for m in self._M503_FIELD_RE.finditer(up[3:]):
                letter = m.group("letter")
                try:
                    val = float(m.group("value"))
                except ValueError:
                    continue
                result[bucket][letter] = val

    # ── Communication ─────────────────────────────────────────────

    def send_data(self, data: str, wait_ok: bool = True) -> bool:
        """Send a G-code line to Marlin.

        v7.5.x (flow-control Phase 1): on **real hardware** this now BLOCKS
        until Marlin acknowledges the command with ``ok`` — the standard
        request/response handshake every reliable printer host (OctoPrint,
        Pronterface, …) uses, and the reason Marlin runs print jobs for hours
        without dropping. Firing commands blind (the old behavior) outran
        Marlin's small serial/planner buffers under load; the buffer overflowed,
        bytes were dropped, the board desynced and went silent — which surfaced
        to us as the mid-print "ZP disconnected". Waiting for ``ok`` means we
        never send a command Marlin hasn't accepted, and we know immediately if
        one wasn't accepted.

        ``ok`` means "admitted to the planner buffer", not "move finished" — it
        returns in milliseconds, so this does not slow motion; it only paces us
        to the board's buffer when a dense burst would otherwise overflow it.
        Use :meth:`flush_moves` (M400) to wait for motion completion.

        Args:
            data:    The G-code line (no trailing newline).
            wait_ok: When True (default), block for ``ok`` on real hardware.
                     Pass False for the rare case a caller reads the response
                     itself.

        Returns:
            True if Marlin acknowledged (or ``wait_ok=False`` / simulate / the
            command was written). False on write error, ``ok`` timeout, a
            Marlin ``Error:`` reply, or a detected board reset. Most callers
            ignore the return; on failure the poller-liveness flag is also
            cleared so a silent board escalates to a disconnect.
        """
        if self.serial is None:
            logger.error("ZP serial not initialised -- command ignored")
            return False
        # Brief retry window if the port is momentarily not open (reconnect).
        for attempt in range(5):
            if hasattr(self.serial, "is_open") and not self.serial.is_open:
                if attempt < 4:
                    time.sleep(0.01)
                    continue
                logger.error("ZP serial still not open after retries")
                return False
            break

        # Simulator: keep the existing fire-and-forget behavior. The simulator
        # does not model RX-buffer backpressure (it answers every command), so
        # the synchronous handshake adds nothing there — and the test suite
        # relies on the non-blocking write semantics.
        if self.simulate or not wait_ok:
            with self._serial_lock:
                try:
                    self.serial.write(data.encode("utf-8") + b"\n")
                    self.serial.flush()
                except Exception as e:
                    logger.error(f"ZP send_data error: {e}")
                    return False
            return True

        # Adaptive timeout: full budget on a healthy board (last command/read
        # acked), short budget once it's gone silent so we fail fast instead of
        # paying 6 s on every command and dragging the whole print/travel down.
        ok_timeout = (self.DEFAULT_OK_TIMEOUT_S
                      if getattr(self, "_last_position_read_ok", True)
                      else self.SILENT_OK_TIMEOUT_S)
        ok, _ = self._txn(data, ok_timeout=ok_timeout, collect=False)
        # Flow-control telemetry (getattr-safe for __new__-constructed test
        # stand-ins that bypass __init__).
        self.cmd_count = getattr(self, "cmd_count", 0) + 1
        if ok:
            self.ok_count = getattr(self, "ok_count", 0) + 1
            # An ack proves the board is alive — restore the full timeout and
            # feed the liveness flag (the poller reads it too).
            self._last_position_read_ok = True
        else:
            self.ok_fail_count = getattr(self, "ok_fail_count", 0) + 1
            # A command that never got 'ok' is the earliest signal of a silent
            # board — surface it to the PositionPoller liveness watchdog (which
            # reads _last_position_read_ok) so the disconnect is caught fast.
            self._last_position_read_ok = False
        return ok

    def get_comm_counters(self) -> dict:
        """Snapshot the flow-control telemetry counters (see __init__)."""
        return {
            "cmd": getattr(self, "cmd_count", 0),
            "ok": getattr(self, "ok_count", 0),
            "ok_fail": getattr(self, "ok_fail_count", 0),
            "reset": getattr(self, "reset_count", 0),
        }

    def reset_comm_counters(self) -> None:
        """Zero the flow-control telemetry counters (Stress Test 'start')."""
        self.cmd_count = self.ok_count = self.ok_fail_count = 0
        self.reset_count = 0
        self._board_reset_detected = False

    def _txn(self, line: str, *, ok_timeout: float,
             collect: bool) -> tuple[bool, str]:
        """Write one command and synchronously wait for Marlin's ``ok``.

        Real-hardware only (callers short-circuit ``simulate``). The whole
        write→read-until-``ok`` round trip is serialized under ``_serial_lock``
        so a concurrent reader (e.g. the position poller) cannot steal the
        ``ok`` or interleave mid-transaction.

        Returns ``(ok, collected_text)`` — ``collected_text`` holds the non-ok
        response lines (e.g. an M114 position line) when ``collect=True``.
        """
        _t0 = time.monotonic()
        with self._serial_lock:
            try:
                self.serial.write(line.encode("utf-8") + b"\n")
                self.serial.flush()
            except Exception as e:
                logger.error(f"ZP write error ({line!r}): {e}")
                _zp_tracer().txn(line, outcome="write_error",
                                 latency_ms=(time.monotonic() - _t0) * 1000.0,
                                 note=str(e)[:60])
                return (False, "")
            _zp_tracer().tx(line)
            ok, text, stats = self._read_until_ok(ok_timeout, collect)
        _zp_tracer().txn(line, outcome=stats.get("outcome", "?"),
                         latency_ms=(time.monotonic() - _t0) * 1000.0,
                         rx_count=stats.get("rx", 0),
                         busy_count=stats.get("busy", 0))
        return (ok, text)

    def _read_until_ok(self, ok_timeout: float,
                       collect: bool) -> tuple[bool, str, dict]:
        """Read serial lines until Marlin acknowledges with ``ok``.

        MUST be called holding ``_serial_lock``. Returns
        ``(ok, text, stats)`` where ``stats`` = ``{outcome, rx, busy}`` for the
        serial tracer (outcome ∈ ok/timeout/error/reset/read_error).

        Line classification:
          * ``ok`` / ``ok ...``        → success.
          * ``error...``               → command rejected; stop, ok=False.
          * ``busy`` / ``echo:busy``   → host keep-alive: board alive and still
                                         processing a long move → reset the wait
                                         window and keep going.
          * reset banner (see
            ``_RESET_MARKERS``)        → board RESET mid-session → flag it,
                                         ok=False (position counter is now lost).
          * any other non-empty line   → board alive; accumulate (when
                                         ``collect``) and keep waiting.
          * silence past the deadline  → board not answering → ok=False.
        """
        deadline = time.monotonic() + ok_timeout
        # v7.5.x: absolute ceiling so 'busy' keep-alives (which reset `deadline`)
        # can never extend the wait forever on a never-completing move. Honor a
        # caller that intentionally passes an ok_timeout larger than the cap.
        hard_deadline = time.monotonic() + max(float(ok_timeout),
                                               self.READ_OK_HARD_CAP_S)
        parts: list[str] = []
        rx_count = 0
        busy_count = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= hard_deadline:
                logger.error(
                    f"ZP _read_until_ok: no 'ok' within the "
                    f"{self.READ_OK_HARD_CAP_S:.0f}s hard cap "
                    f"(rx_lines={rx_count}, busy={busy_count}) — board appears "
                    f"stuck (move never completing?); failing the command")
                return (False, "\n".join(parts),
                        {"outcome": "hard_timeout", "rx": rx_count,
                         "busy": busy_count})
            try:
                raw = self.serial.readline()
            except Exception as e:
                logger.warning(f"ZP read error while awaiting 'ok': {e}")
                return (False, "\n".join(parts),
                        {"outcome": "read_error", "rx": rx_count,
                         "busy": busy_count})
            if not raw:
                continue  # readline timed out with no data; re-check deadline
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            rx_count += 1
            _zp_tracer().rx(line)
            low = line.lower()
            if low == "ok" or low.startswith("ok "):
                return (True, "\n".join(parts),
                        {"outcome": "ok", "rx": rx_count, "busy": busy_count})
            if low.startswith("error"):
                logger.error(f"ZP Marlin reported an error: {line}")
                return (False, "\n".join(parts),
                        {"outcome": "error", "rx": rx_count,
                         "busy": busy_count})
            if "busy" in low:
                # Still processing (long move) — board is alive, extend window.
                # The absolute hard cap (checked at the top of the loop) still
                # bounds the total wait, so a never-ending 'busy' stream can't
                # extend this forever.
                busy_count += 1
                deadline = time.monotonic() + ok_timeout
                continue
            if any(m in low for m in self._RESET_MARKERS):
                logger.error(
                    f"ZP board RESET detected mid-session: {line!r} — "
                    f"position counter is lost until re-declared")
                self._board_reset_detected = True
                self.reset_count = getattr(self, "reset_count", 0) + 1
                _zp_tracer().event("board_reset", line=repr(line))
                return (False, "\n".join(parts),
                        {"outcome": "reset", "rx": rx_count,
                         "busy": busy_count})
            # Some other line (M114 position echo, info) → board is alive.
            if collect:
                parts.append(line)
            # A live line is progress: don't let a chatty-but-slow board be cut
            # off right at the deadline. Left UNCAPPED (like the busy branch) so
            # the top-of-loop hard-cap check is the single exit for a stuck-but-
            # chatty board — it then reports 'hard_timeout' uniformly rather than
            # a plain 'timeout' (the absolute wait is still bounded by the cap).
            deadline = max(deadline, time.monotonic() + 0.5)
        return (False, "\n".join(parts),
                {"outcome": "timeout", "rx": rx_count, "busy": busy_count})


    def receive_data(self) -> str:
        """Read all available response data.
        v7.2.6: ZP serial lock protects read_all.
        """
        import time as _t; _t.sleep(0.01)
        with self._serial_lock:
            try:
                return self.serial.read_all().decode("utf-8", errors="replace").strip()
            except Exception as e:
                logger.debug(f"ZP receive_data error: {e}")
                return ""


    def move_relative(self, axes: dict[str, float], feedrate: Optional[float] = None) -> None:
        """
        Move axes by relative distances (in relative mode).

        Args:
            axes:     Dict of {printer_axis: distance}, e.g. {"X": 1.5, "Y": -0.5}
            feedrate: Optional feedrate override (mm/min).
        """
        # v7.5.x freeze fix: drop SUB-RESOLUTION moves (≥ 1e-4 mm = 0.1 µm).
        # The old `> 1e-6` gate let through the tiny residual a soft-limit
        # clamp leaves when an axis is pinned at the limit (clamped_delta =
        # boundary − current ≈ a few µm of float noise). The continuous jog
        # loop then emitted one such micro-move *every segment, forever*,
        # while the stick was held against the limit — a stream of un-acked
        # commands that freezes the board (→ the poller then drops it). Below
        # one Marlin step (~0.1 µm) there is nothing real to move, so skip.
        active = {a: d for a, d in axes.items() if abs(d) >= 1e-4}
        if not active:
            return

        # v7.5.x freeze fix: fixed-decimal format. Raw f"{d}" emitted
        # scientific notation for tiny values (e.g. "Z9e-05"), which not all
        # G-code parsers accept. Matches the print-path convention
        # (PrintManager uses :.4f / :.5f). 4 dp = 0.1 µm, below any real move.
        axis_str = " ".join(f"{a}{d:.4f}" for a, d in active.items())
        fr = feedrate if feedrate is not None else self.feedrate
        # Never emit a non-positive feedrate: Marlin treats "F0" as an
        # infinite-time move and stalls the planner (another freeze path).
        try:
            fr = max(float(fr), 1.0)
        except (TypeError, ValueError):
            fr = self.feedrate
        self.send_data(f"G0 F{fr} {axis_str}")

    def move_absolute(self, axes: dict[str, float], fast: bool = False,
                      feedrate_mm_min: float | None = None) -> None:
        """
        Move axes to absolute positions.

        Temporarily switches to absolute mode, executes the move,
        then returns to relative mode.

        Args:
            axes: Dict of {printer_axis: position}
            fast: If True, use maximum feedrate.
            feedrate_mm_min: Optional feedrate for this move (mm/min).
        """
        active = {a: p for a, p in axes.items() if True}  # include all
        if not active:
            return

        # v7.5.x freeze fix: fixed-decimal format (never scientific notation).
        axis_str = " ".join(f"{a}{p:.4f}" for a, p in active.items())
        # v7.5.x ROOT-CAUSE FIX: ALWAYS emit an explicit F. An absolute move
        # with no feedrate used to omit F and inherit Marlin's last modal
        # feedrate — which the slow per-segment pump moves leave at ~F1.8
        # mm/min, turning the next bare Z move into a ~12-min crawl that hangs
        # the board (M400 sits "busy" → timeout → Z overheats → USB drop).
        # Mirror move_relative: fall back to the configured default feedrate,
        # NEVER to whatever the previous (pump) command happened to set.
        fr = feedrate_mm_min if feedrate_mm_min else self.feedrate
        try:
            fr = max(float(fr), 1.0)
        except (TypeError, ValueError):
            fr = self.feedrate
        feed_str = f" F{fr:.0f}"

        self.send_data("G90")  # Absolute mode
        self.send_data(f"G0 {axis_str}{feed_str}")
        self.send_data("G91")  # Back to relative

    # ── Position Query ────────────────────────────────────────────

    def get_current_position(self) -> tuple[float, float, float, float]:
        """
        Query and return current position of all four axes.

        Returns:
            (x, y, z, e) — printer axis positions.
            Maps to (Z-needle, P1, P2, P3) via AXIS_MAP.
        """
        if self.serial is None:
            self._last_position_read_ok = False
            return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

        if self.simulate:
            # Simulator path unchanged: the sim answers M114 via its response
            # queue (no synchronous 'ok' handshake) — bounded drain + parse.
            self.send_data("M114")
            deadline = time.monotonic() + 0.25
            response = ""
            while True:
                chunk = self.receive_data()
                if chunk:
                    response = f"{response}\n{chunk}" if response else chunk
                    self._parse_position(response)
                    if self._last_position_read_ok:
                        break
                if time.monotonic() >= deadline:
                    if not response:
                        self._parse_position(response)
                    break
            return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

        # Real hardware: a single synchronous M114 transaction — write, then
        # read until 'ok', parsing the position line along the way. Consuming
        # the 'ok' leaves the RX buffer clean for the next command, so the old
        # stale-ack drain (needed only because we used to fire commands without
        # reading their 'ok') is no longer required. ok_timeout is kept short:
        # M114 answers immediately, and the poller calls this every ~0.3 s.
        ok, text = self._txn("M114", ok_timeout=0.5, collect=True)
        if text:
            self._parse_position(text)  # sets _last_position_read_ok
        else:
            # No reply at all (or only 'ok' with no position) → failed read so
            # the poller-liveness watchdog still sees a silent board.
            self._last_position_read_ok = False
        return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

    def flush_moves(self, timeout_s: float = 15.0) -> bool:
        """Block until Marlin confirms all queued moves are physically complete.

        Sends M400 (Wait for Moves to Finish). Marlin only responds with 'ok'
        after every buffered move has executed. This is more reliable than
        polling M114, which may return commanded (planned) position rather than
        the actual stepper position during motion.

        v7.5.x CRITICAL: the ENTIRE write→read-until-'ok' round trip is held
        under ``_serial_lock`` (atomic), exactly like the synchronous command
        handshake. Previously the lock was released between each ``readline()``,
        so a *concurrent* ZP reader — a straggler poller M114 (``suspend()`` only
        sets a flag; a poll already in flight does one more read), the Xbox jog
        handler, etc. — could slip in and CONSUME the M400 'ok'. flush_moves then
        waited out the full timeout for an 'ok' that was already eaten and
        reported a bogus "M400 timed out → ZP disconnected", even on a
        zero-distance move with a perfectly alive board. Holding the lock makes
        any other reader WAIT until M400 completes, so the 'ok' can't be stolen.

        v7.3.5 BF-1: Drains the serial RX buffer before sending M400 so a stale
        "ok" can't be mistaken for the M400 completion.

        Returns:
            True  — Marlin confirmed completion within timeout.
            False — Timed out (caller should log a warning and decide how to proceed).
        """
        if self.serial is None or self.simulate:
            return True  # Simulated or disconnected — treat as immediate success

        # v7.5.x: if the board has already gone silent, don't wait the full
        # 15 s for an M400 'ok' that will never come — cap it so the
        # end-of-print / safe-travel retract fails fast instead of hanging.
        if not getattr(self, "_last_position_read_ok", True):
            timeout_s = min(timeout_s, 2.0)

        _t0 = time.monotonic()
        deadline = _t0 + timeout_s
        rx_count = 0
        busy_count = 0
        # Atomic: drain → write M400 → read until 'ok', all under one lock hold
        # so no other thread can interleave a read and steal the 'ok'.
        with self._serial_lock:
            try:
                self.serial.reset_input_buffer()
                self.serial.write(b"M400\n")
                self.serial.flush()
                _zp_tracer().tx("M400")
            except Exception as e:
                logger.warning(f"flush_moves send error: {e}")
                _zp_tracer().txn("M400", outcome="write_error",
                                 latency_ms=(time.monotonic() - _t0) * 1000.0,
                                 note=str(e)[:60])
                return False
            while time.monotonic() < deadline:
                try:
                    line = self.serial.readline().decode(
                        "utf-8", errors="replace").strip()
                except Exception as e:
                    logger.warning(f"flush_moves read error: {e}")
                    _zp_tracer().txn(
                        "M400", outcome="read_error", rx_count=rx_count,
                        busy_count=busy_count,
                        latency_ms=(time.monotonic() - _t0) * 1000.0)
                    return False
                if not line:
                    continue
                rx_count += 1
                _zp_tracer().rx(line)
                if line == "ok" or line.lower().startswith("ok "):
                    _zp_tracer().txn(
                        "M400", outcome="ok", rx_count=rx_count,
                        busy_count=busy_count,
                        latency_ms=(time.monotonic() - _t0) * 1000.0)
                    return True
                if "busy" in line.lower():
                    busy_count += 1
                # Discard other lines (position echo, busy keep-alive, etc.).

        # The rx_count distinguishes a SILENT board (rx=0 → no bytes at all,
        # genuinely not answering) from a STOLEN 'ok' / slow move (rx>0 → bytes
        # came back but no 'ok' for us) — the single most useful flush_moves
        # diagnostic, now captured.
        logger.warning(
            f"flush_moves: M400 timed out after {timeout_s:.1f}s "
            f"(rx_lines={rx_count}, busy={busy_count})")
        _zp_tracer().txn("M400", outcome="timeout", rx_count=rx_count,
                         busy_count=busy_count,
                         latency_ms=(time.monotonic() - _t0) * 1000.0,
                         note=("SILENT" if rx_count == 0 else "no-ok"))
        return False

    def _parse_position(self, response: str) -> None:
        """Parse M114 response and update cached positions."""
        for line in response.split("\n"):
            line = line.strip()
            if not line or line == "ok":
                continue

            match = self._M114_PATTERN.search(line)
            if match:
                self.x_pos = float(match.group(1))
                self.y_pos = float(match.group(2))
                self.z_pos = float(match.group(3))
                self.e_pos = float(match.group(4))
                self._last_position_read_ok = True  # v7.5.x
                return

        # v7.5.x: no parseable position in the response — the board did not
        # answer M114 (dead/powered-off serial). The poller uses this to
        # escalate to a disconnect (the tuple itself stays at stale floats).
        self._last_position_read_ok = False
        logger.debug(f"ZP position parse failed: {response[:100]}")

    # ── Settings ──────────────────────────────────────────────────

    def set_max_feedrate(self, feedrate: float) -> None:
        """Set maximum feedrate. Broadcasts ``feedrate`` to every logical
        axis (Z, P1, P2, P3). Prefer :meth:`set_per_axis_max_feedrate`
        for the per-axis case.
        """
        self.feedrate = feedrate
        broadcast = {logical: float(feedrate)
                     for logical in (self.axis_map or AXIS_MAP).keys()}
        self.set_per_axis_max_feedrate(broadcast, persist=True)
        logger.debug(f"ZP max feedrate broadcast to {feedrate} mm/min")

    def set_absolute_mode(self) -> None:
        """Switch to absolute positioning."""
        self.send_data("G90")

    def set_relative_mode(self) -> None:
        """Switch to relative positioning."""
        self.send_data("G91")

    def emergency_stop(self) -> None:
        """Send emergency stop command (M112)."""
        self.send_data("M112")
        logger.warning("ZP emergency stop sent")

    def save_settings(self) -> None:
        """Save current settings to printer EEPROM."""
        self.send_data("M500")

    def __repr__(self) -> str:
        mode = "SIM" if self.simulate else "HW"
        return f"ZPStageManager(mode={mode}, feedrate={self.feedrate})"
