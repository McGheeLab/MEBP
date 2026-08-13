"""
controller.py — the incubator facade. No Qt, no widgets.

Everything the GUI can do, it does through here, which keeps the whole control
surface testable headlessly (see :mod:`.selftest`).

Threading contract
------------------
* Public methods are safe to call from ANY thread (including the GUI thread) and
  return immediately. Work is queued to a single command worker thread, which
  serialises access to the serial port — the same guarantee ``_serial_lock``
  gives ``ZPStageManager``, without needing the lock.
* The link's reader thread pushes inbound lines straight into the callbacks, so
  temperature and autotune progress keep flowing while a command is in flight.
* Callbacks fire on background threads. The GUI adapts them to Qt signals.

Two design decisions worth stating outright
-------------------------------------------
1. **We do not use M190/M109 by default.** On this rig a board-side wait can
   block for the best part of an hour, and while it blocks the command queue a
   subsequent "heater off" cannot get through. Instead we send the non-blocking
   ``M140``/``M104`` and decide "arrived / stable" on the host from the
   autoreport stream (:mod:`.stability`). Board-side waits remain available as an
   explicit opt-in, cancellable with ``M108``.
2. **Emergency commands bypass the queue.** ``M112``/``M108`` go out via
   :meth:`MarlinLink.send_priority`, which is correct because Marlin's
   ``EMERGENCY_PARSER`` plucks them from the input buffer without waiting for the
   queue to drain.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from .calibration import CalibrationStore
from .marlin_gcode import (
    LineKind,
    PidValues,
    build_autoreport,
    parse_autotune_failure,
    parse_autotune_progress,
    parse_autotune_result,
    parse_fault,
    parse_pid_dump,
    parse_temp_line,
)
from .marlin_link import MarlinLink
from .probe import FirmwareProbe, FirmwareReport
from .safety import (
    FaultLatch,
    MAX_SETPOINT_C,
    SensorDivergenceMonitor,
    SetpointCheck,
    check_setpoint,
)
from .sensors import MARLIN_SOURCE_ID, MarlinSensorSource, SensorHub
from .stability import StabilityReport, StabilityTracker
from .telemetry import TelemetryLogger
from .zones import ALL_ZONES, ZoneSpec, zone_by_id, zone_for_heater_id

logger = logging.getLogger(__name__)

#: How often the host samples/publishes derived state.
SAMPLE_PERIOD_S = 1.0

#: Autoreport interval requested from the firmware (seconds).
AUTOREPORT_INTERVAL_S = 1

#: Fallback poll period when the firmware has no autoreport support.
POLL_PERIOD_S = 2.0


@dataclass
class ZoneRuntime:
    """Live per-zone state owned by the controller."""

    spec: ZoneSpec
    tracker: StabilityTracker
    #: Real-world setpoint the operator asked for (before calibration inversion).
    requested_c: float = 0.0
    #: Integer value actually commanded to the board.
    commanded_c: int = 0
    pid: PidValues | None = None
    pid_available: bool = False
    autotune_running: bool = False
    #: False when the zone's sensor reads implausibly (unplugged/shorted).
    #: Heating is refused in that state.
    sensor_ok: bool = True
    sensor_fault: str = ""
    #: v7.18: the firmware REFUSED this zone's last heater command —
    #: "``<cmd>`` → ``<the board's Error: line>``". While latched the host
    #: stops re-asserting the target (ramp + dither), because silently
    #: retrying a refused command every period is how a heater that "won't
    #: get hot" hid for 48 minutes on the bench (2026-08-12: ``M104 S37``
    #: sent to a hotend output with no usable heater, refused every time,
    #: nothing shown anywhere). Cleared by the next operator action on the
    #: zone, or by any accepted command.
    refused: str = ""
    #: v7.18: how many times the BOARD was found to have forgotten this zone's
    #: target (a Marlin reset clears every setpoint to 0) and the host put it
    #: back. Non-zero means the board is resetting underneath the hold.
    reasserts: int = 0
    #: Consecutive samples whose board-reported target disagreed with what we
    #: commanded. Debounce: one disagreeing sample is normal (a sample taken
    #: between the command and its effect legitimately lags).
    _target_mismatch: int = 0
    #: Host-side dither state for fractional holds.
    dither_enabled: bool = False
    dither_target_c: float = 0.0
    _dither_next_flip: float = 0.0
    _dither_high: bool = False

    @property
    def zone_id(self) -> str:
        return self.spec.zone_id


class IncubatorController:
    """Framework-agnostic control of the two-zone incubator."""

    MAX_SETPOINT_C = MAX_SETPOINT_C

    def __init__(self, *, calibration_path=None):
        self.hub = SensorHub()
        self.calibration = CalibrationStore(calibration_path)
        self.fault_latch = FaultLatch()
        self.divergence = SensorDivergenceMonitor()
        self.telemetry = TelemetryLogger()
        self.report: FirmwareReport | None = None

        self._link: MarlinLink | None = None
        self._port_obj = None
        self._sim = False
        self._connected = False
        #: Port/baud actually in use once connected (may differ from what was
        #: requested, if auto-detect had to find the board).
        self.active_port: str = ""
        self.active_baud: int = 0
        #: v7.18: which transport carries the session —
        #: "" (disconnected) | "serial" | "shared" | "simulated".
        #: "shared" rides the app's live ZP connection and REFUSES operations
        #: that would hold the motion board's serial lock for minutes
        #: (autotune, board-side waits) — see :mod:`.zp_shared_link`.
        self._transport: str = ""
        #: v7.18: optional zero-arg callable returning ports this controller
        #: must never open or offer (the app's live ZP / XY connections —
        #: opening a port DTR-resets the board behind it).
        self.exclude_ports_provider = None

        self._source = MarlinSensorSource(
            power_for={z.temp_key: z.power_key for z in ALL_ZONES},
            labels={z.temp_key: z.title for z in ALL_ZONES},
            calibrator=self._calibrate_reading,
        )
        self.hub.add_source(self._source)

        self._zones: dict[str, ZoneRuntime] = {
            z.zone_id: ZoneRuntime(spec=z, tracker=StabilityTracker(z.zone_id))
            for z in ALL_ZONES
        }

        # Command worker
        self._cmd_q: "queue.Queue[tuple[Callable, tuple, dict] | None]" = queue.Queue()
        self._worker: threading.Thread | None = None
        self._sampler: threading.Thread | None = None
        self._stop = threading.Event()

        # Autotune accumulation
        self._at_lock = threading.RLock()
        self._at_zone: str | None = None
        self._at_result: dict[str, float] = {}
        self._at_started: float = 0.0

        self._use_polling = False
        self._stale_warned = False
        #: Per-zone staircase ramps (see .ramp for why they exist).
        self._ramps: dict = {}

        # ── callbacks (plain lists; the GUI wraps them in Qt signals) ──
        self._cb_channels: list[Callable] = []
        self._cb_zone: list[Callable] = []
        self._cb_line: list[Callable] = []
        self._cb_pid: list[Callable] = []
        self._cb_at_progress: list[Callable] = []
        self._cb_at_done: list[Callable] = []
        self._cb_fault: list[Callable] = []
        self._cb_divergence: list[Callable] = []
        self._cb_conn: list[Callable] = []
        self._cb_probe: list[Callable] = []
        self._cb_status: list[Callable] = []
        self._cb_ramp: list[Callable] = []

    # ═══════════════════════════════════════════════════════════════
    # Callback registration
    # ═══════════════════════════════════════════════════════════════

    def on_channels(self, cb): self._cb_channels.append(cb)
    def on_zone_state(self, cb): self._cb_zone.append(cb)
    def on_raw_line(self, cb): self._cb_line.append(cb)
    def on_pid(self, cb): self._cb_pid.append(cb)
    def on_autotune_progress(self, cb): self._cb_at_progress.append(cb)
    def on_autotune_done(self, cb): self._cb_at_done.append(cb)
    def on_fault(self, cb): self._cb_fault.append(cb)
    def on_divergence(self, cb): self._cb_divergence.append(cb)
    def on_connection_changed(self, cb): self._cb_conn.append(cb)
    def on_probe(self, cb): self._cb_probe.append(cb)
    def on_status(self, cb): self._cb_status.append(cb)
    def on_ramp(self, cb): self._cb_ramp.append(cb)

    @staticmethod
    def _fire(cbs: list[Callable], *args) -> None:
        for cb in list(cbs):
            try:
                cb(*args)
            except Exception:
                logger.debug("controller callback raised", exc_info=True)

    def _status(self, text: str) -> None:
        self._fire(self._cb_status, text)

    # ═══════════════════════════════════════════════════════════════
    # Connection
    # ═══════════════════════════════════════════════════════════════

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def simulated(self) -> bool:
        return self._sim

    @property
    def transport(self) -> str:
        """"" | "serial" | "shared" | "simulated" (see ``_transport``)."""
        return self._transport

    @property
    def shared_transport(self) -> bool:
        return self._transport == "shared"

    def set_max_setpoint_c(self, value) -> None:
        """v7.18: per-machine ceiling from the config store.

        Shadows the class attribute on THIS instance, clamped so a config
        file can only LOWER the code-level hard maximum, never raise it.
        """
        try:
            v = float(value)
        except (TypeError, ValueError):
            return
        self.MAX_SETPOINT_C = max(1.0, min(MAX_SETPOINT_C, v))

    def set_zone_labels(self, labels: dict) -> None:
        """v7.18: operator display names per zone id, from the config store."""
        from .zones import zone_by_id as _zone_by_id
        mapped: dict[str, str] = {}
        for zid, label in (labels or {}).items():
            text = str(label or "").strip()
            if not text:
                continue
            try:
                spec = _zone_by_id(zid)
            except KeyError:
                continue
            mapped[spec.temp_key] = text
        if mapped:
            self._source.set_labels(mapped)

    def _excluded_ports(self) -> list[str]:
        """Ports reserved for other devices (never opened, never offered)."""
        provider = self.exclude_ports_provider
        if provider is None:
            return []
        try:
            return [str(p) for p in (provider() or []) if p]
        except Exception:
            logger.debug("exclude_ports_provider raised", exc_info=True)
            return []

    def detect_board(self):
        """
        Find the Marlin board by probing candidate ports with ``M115``.

        Read-only and safe — EXCEPT that merely opening a port DTR-resets the
        board behind it, which is why the app's live ZP / XY ports are
        excluded from the scan entirely (v7.18).

        Returns a :class:`~.device_config.DetectedBoard` or ``None``.
        """
        from . import device_config
        self._status("Scanning serial ports for a Marlin board…")
        found = device_config.detect_marlin(
            on_progress=self._status,
            exclude_ports=self._excluded_ports())
        if found is None:
            stale = device_config.stale_hints()
            extra = ""
            if stale:
                extra = (
                    f" (the saved port {', '.join(stale)} is no longer present, "
                    f"so it was skipped)"
                )
            self._status(
                "No Marlin board found on any serial port" + extra +
                ". Check the USB cable, and make sure the main MEBP app is not "
                "holding the port."
            )
        else:
            self._status(
                f"Found {found.firmware} on {found.port} at {found.baud} baud."
            )
        return found

    def connect(
        self,
        port: str = "",
        baud: int = 38400,
        *,
        simulate: bool = False,
        sim_time_scale: float = 1.0,
        sim_kwargs: dict | None = None,
        auto_detect: bool = True,
    ) -> bool:
        """
        Open the link, probe the firmware, and start streaming.

        With ``auto_detect`` (default) an empty or unopenable port falls back to
        scanning for a Marlin board. That matters in practice: the cached port
        hint in the app's device profiles goes stale whenever Windows renumbers
        the COM port, and a stale hint is otherwise indistinguishable from
        "the tool is broken".

        Blocks for the duration of the probe, so call it from a worker thread if
        the GUI must stay responsive. Returns True on success.
        """
        if self._connected:
            return True

        self.fault_latch.acknowledge()
        self._use_polling = False

        # v7.18: never open a port another device owns. The ZP board's port
        # is the common case on this rig — the heaters are wired to THAT
        # board, and the right way to reach them while the app holds the port
        # is the shared transport (connect_shared), not a second open.
        if port:
            excl = {p.upper() for p in self._excluded_ports()}
            if not simulate and port.strip().upper() in excl:
                self._status(
                    f"{port} belongs to a device the app is already connected "
                    f"to (ZP board or XY stage). If the heaters are on the ZP "
                    f"board, use the shared transport instead of a dedicated "
                    f"port — it rides the existing connection."
                )
                return False

        try:
            if simulate:
                from .fake_marlin import FakeMarlinLink
                self._port_obj = FakeMarlinLink(
                    time_scale=sim_time_scale, **(sim_kwargs or {})
                )
                self._sim = True
            else:
                self._port_obj = None
                if port:
                    try:
                        self._port_obj = self._open_serial(port, baud)
                    except Exception as e:
                        if not auto_detect:
                            raise
                        self._status(
                            f"{port} could not be opened ({self._friendly(e)}) — "
                            f"scanning for the board instead."
                        )
                if self._port_obj is None:
                    if not auto_detect:
                        raise ValueError("no serial port selected")
                    found = self.detect_board()
                    if found is None:
                        return False
                    port, baud = found.port, found.baud
                    self._port_obj = self._open_serial(port, baud)
                self._sim = False
                self.active_port, self.active_baud = port, baud
        except Exception as e:
            self._status(f"Connect failed: {self._friendly(e)}")
            return False

        self._link = MarlinLink(
            self._port_obj,
            on_line=self._on_line,
            on_write=lambda cmd: self._fire(self._cb_line, "tx", cmd),
            on_disconnect=self._on_link_lost,
        )
        self._link.start()
        time.sleep(0.35 if not simulate else 0.2)  # let a boot banner land

        self._transport = "simulated" if simulate else "serial"
        return self._start_session()

    def connect_shared(self, zp_getter, *, poll_gate=None) -> bool:
        """v7.18: open the incubator session over the app's LIVE ZP link.

        The heaters on this rig are wired to the same SKR Mini E3 V3 that
        drives Z and the pumps, and the app holds that COM port for the whole
        session — so heater traffic must ride the existing connection. See
        :mod:`.zp_shared_link` for the rules that keep the motion side
        healthy (no autoreport, short transactions only, polling yields to
        prints).

        Args:
            zp_getter: zero-arg callable returning the current
                ``ZPStageManager`` (or None). Resolved per call.
            poll_gate: optional zero-arg callable; False = skip this poll
                round (a print owns the channel).

        Blocks for the duration of the firmware probe — call from a worker
        thread. Returns True on success.
        """
        if self._connected:
            return True

        from .zp_shared_link import ZPSharedLink

        self.fault_latch.acknowledge()
        # Shared mode NEVER uses autoreport: M155 pushes unsolicited lines
        # that would land mid-M114/M400 on the motion side.
        self._use_polling = True

        link = ZPSharedLink(
            zp_getter,
            on_line=self._on_line,
            on_write=lambda cmd: self._fire(self._cb_line, "tx", cmd),
            on_disconnect=self._on_link_lost,
            poll_gate=poll_gate,
        )
        if not link.is_open():
            zp = None
            try:
                zp = zp_getter()
            except Exception:
                pass
            if zp is not None and getattr(zp, "simulate", False):
                self._status(
                    "The ZP board is SIMULATED — its simulator models motion, "
                    "not heaters. Use the incubator's own simulator instead."
                )
            else:
                self._status(
                    "The ZP board is not connected. Connect it on Hardware "
                    "Setup → Device (or the Connect Hardware card) first — "
                    "the incubator heaters share that board."
                )
            return False

        self._link = link
        self._port_obj = None
        self._sim = False
        self._transport = "shared"
        try:
            zp = zp_getter()
            self.active_port = (
                f"{getattr(zp, 'connected_port', '') or '?'} (shared ZP link)"
            )
        except Exception:
            self.active_port = "(shared ZP link)"
        self.active_baud = 0
        return self._start_session(pre_commands=("M155 S0",))

    def _start_session(self, *, pre_commands: tuple = ()) -> bool:
        """Common session start once ``self._link`` carries a live transport:
        worker thread, firmware probe, capability seeding, sampler."""
        self._stop.clear()
        self._worker = threading.Thread(
            target=self._worker_loop, daemon=True, name="incubator-cmds"
        )
        self._worker.start()

        # e.g. "M155 S0" in shared mode: a previous DEDICATED session may have
        # left autoreport enabled, and unsolicited pushes would interleave
        # with the motion side's M114/M400 parsing. Best-effort — a firmware
        # without M155 just answers with an error we ignore.
        for cmd in pre_commands:
            try:
                self._link.send_and_wait(cmd, timeout_s=3.0)
            except Exception:
                logger.debug("pre-command %r failed", cmd, exc_info=True)

        # Probe synchronously — everything downstream depends on the answers.
        try:
            self.report = FirmwareProbe(self._link).run()
        except Exception as e:
            logger.warning("probe failed: %s", e)
            self.report = FirmwareReport(errors=[f"probe failed: {e}"])

        for spec in ALL_ZONES:
            cap = self.report.zone(spec.zone_id)
            rt = self._zones[spec.zone_id]
            rt.pid_available = cap.pid_available
            rt.pid = cap.pid
            rt.sensor_ok = cap.sensor_ok
            rt.sensor_fault = cap.sensor_fault

        self._connected = True
        self._fire(self._cb_conn, True)
        self._fire(self._cb_probe, self.report)
        self._fire(self._cb_pid, self._pid_snapshot())

        # Prefer firmware autoreport; fall back to host polling. Shared mode
        # arrives here with _use_polling already forced True.
        if self._use_polling or self.report.autoreport_supported is False:
            self._use_polling = True
            if self._transport == "shared":
                self._status("Polling M105 over the shared ZP link.")
            else:
                self._status("Firmware has no autoreport; polling M105 instead.")
        else:
            self.submit(self._do_autoreport, AUTOREPORT_INTERVAL_S)

        self._sampler = threading.Thread(
            target=self._sample_loop, daemon=True, name="incubator-sampler"
        )
        self._sampler.start()

        name = self.report.firmware_name or "unknown firmware"
        suffix = {"simulated": " (simulated)",
                  "shared": " (shared ZP link)"}.get(self._transport, "")
        self._status(f"Connected to {name}{suffix}.")
        return True

    @staticmethod
    def _open_serial(port: str, baud: int):
        import serial
        if not port:
            raise ValueError("no serial port selected")
        return serial.Serial(
            port=port,
            baudrate=int(baud),
            timeout=0.25,
            write_timeout=2.0,
        )

    @staticmethod
    def _friendly(err: Exception) -> str:
        from .serial_helpers import friendly_error_message
        try:
            msg = friendly_error_message(err)
        except Exception:
            msg = str(err)
        low = str(err).lower()
        if "access is denied" in low or "permission" in low:
            msg += (
                " Only one program can hold a COM port — if the main MEBP app is "
                "connected to the ZP board, disconnect it there first."
            )
        return msg

    def disconnect(self, *, heaters_off: bool = True) -> None:
        """
        Tear down. By default commands both heaters off first.

        Note this is a courtesy, not a safety guarantee: if the USB link is
        already gone, the firmware keeps holding its last setpoint. That is by
        design (the firmware owns the loop) and the UI states it.
        """
        if not self._connected and self._link is None:
            return

        if heaters_off and self._connected and not self.fault_latch.active:
            for spec in ALL_ZONES:
                try:
                    self._link.send_and_wait(spec.heater_off(), timeout_s=3.0)
                except Exception:
                    pass

        for _zid in list(self._ramps):
            self.stop_ramp(_zid, reason="disconnecting")

        self._stop.set()
        self._cmd_q.put(None)
        for t in (self._worker, self._sampler):
            if t is not None and t.is_alive():
                t.join(timeout=2.0)
        self._worker = self._sampler = None

        if self._link is not None:
            try:
                self._link.close()
            except Exception:
                pass
        self._link = None
        self._port_obj = None

        self.telemetry.stop("disconnected")
        was = self._connected
        self._connected = False
        self._transport = ""
        self.hub.clear()
        for rt in self._zones.values():
            rt.tracker.reset()
        if was:
            self._fire(self._cb_conn, False)
            self._status("Disconnected.")

    def _on_link_lost(self) -> None:
        if not self._connected:
            return
        self._connected = False
        self._fire(self._cb_conn, False)
        self._status(
            "Serial link lost. NOTE the board keeps running its own control loop "
            "and will hold the last setpoint — power it down if that is not what "
            "you want."
        )

    # ═══════════════════════════════════════════════════════════════
    # Command worker
    # ═══════════════════════════════════════════════════════════════

    def submit(self, fn: Callable, *args, **kwargs) -> None:
        """Queue work for the serial worker thread. Returns immediately."""
        self._cmd_q.put((fn, args, kwargs))

    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            item = self._cmd_q.get()
            if item is None:
                return
            fn, args, kwargs = item
            try:
                fn(*args, **kwargs)
            except Exception:
                logger.warning("queued command failed", exc_info=True)

    def _send(self, command: str, *, timeout_s: float = 8.0,
              hard_ceiling_s: float | None = None):
        if self._link is None:
            return None
        if self.fault_latch.active:
            self._status(self.fault_latch.blocking_reason())
            return None
        return self._link.send_and_wait(
            command, timeout_s=timeout_s, hard_ceiling_s=hard_ceiling_s
        )

    def _send_zone_cmd(self, zone_id: str, command: str, *,
                       timeout_s: float = 8.0,
                       hard_ceiling_s: float | None = None):
        """
        v7.18: send one HEATER command for a zone and CHECK THE ANSWER.

        Every setpoint used to go through the bare fire-and-forget ``_send``,
        so a firmware-REFUSED command produced no status, no log and no card
        state — and the hold/dither loop kept re-asserting it every period.
        Bench 2026-08-12: ``M104 S37`` (a hotend output with no usable heater
        on this rig) was refused by the firmware every ~minute for 48 minutes
        while the UI looked healthy and nothing was heating. A refusal now
        latches on the zone (:attr:`ZoneRuntime.refused`), stops the automatic
        re-assertion (ramp + dither), and is logged WITH the board's reply
        text so the evidence exists after the fact.

        Only an ANSWERED "no" (``txn.rejected``) latches — a timeout or a
        board reset is a link problem, which the stale-data warning and the
        reset detection already own.
        """
        txn = self._send(command, timeout_s=timeout_s,
                         hard_ceiling_s=hard_ceiling_s)
        rt = self._zones.get(zone_id)
        if rt is None or txn is None:
            return txn
        if getattr(txn, "ok", False):
            # A later ACCEPTED command clears the latch — the zone works again.
            rt.refused = ""
            return txn
        if not getattr(txn, "rejected", False):
            # Not acknowledged rather than refused (timeout / reset / serial
            # fault). Log it, but leave the latch to the paths that own those
            # failure modes.
            logger.warning(
                "incubator %s: %r was not acknowledged: %s",
                zone_id, command, getattr(txn, "error_text", "") or "no reply")
            return txn

        reply = (getattr(txn, "error_text", "") or "").strip() or \
            "command rejected"
        spec = rt.spec
        first = not rt.refused
        rt.refused = f"{command} → {reply}"
        rt.dither_enabled = False
        self.stop_ramp(zone_id, reason="the firmware refused the command")
        logger.warning("incubator %s: firmware REFUSED %r: %s",
                       zone_id, command, reply)
        self.telemetry.log_event("command_refused", zone=zone_id,
                                 cmd=command, reply=reply)
        if first:
            self._status(
                f"{spec.title}: the firmware REFUSED '{command}' — {reply}. "
                f"This zone is NOT heating, and the automatic hold has been "
                f"stopped so the refused command is not silently retried. "
                f"Check that this zone's heater is really wired to "
                f"{spec.heater_connector} before trying again."
            )
        return txn

    # ═══════════════════════════════════════════════════════════════
    # Inbound line handling (reader thread)
    # ═══════════════════════════════════════════════════════════════

    def _on_line(self, text: str, kind: LineKind) -> None:
        self._fire(self._cb_line, "rx", text)

        frame = parse_temp_line(text)
        if frame is not None:
            self._source.ingest_frame(frame)

        if kind is LineKind.ERROR:
            self._handle_fault(text)
            return

        if self._at_zone is not None:
            self._handle_autotune_line(text)

    def _handle_fault(self, text: str) -> None:
        fault = parse_fault(text)
        if fault is None:
            return
        zone = zone_for_heater_id(fault.heater_id)
        zone_id = zone.zone_id if zone else None
        if self.fault_latch.record(fault, zone_id):
            self.telemetry.log_event(
                "fault", kind=fault.kind, heater=fault.heater_id, text=text
            )
            self._fire(self._cb_fault, self.fault_latch.current)
            for _zid in list(self._ramps):
                self.stop_ramp(_zid, reason="heater fault")
            for rt in self._zones.values():
                rt.dither_enabled = False
                rt.autotune_running = False
            with self._at_lock:
                self._at_zone = None

    def _handle_autotune_line(self, text: str) -> None:
        fail = parse_autotune_failure(text)
        if fail is not None:
            with self._at_lock:
                zone_id, self._at_zone = self._at_zone, None
                self._at_result = {}
            if zone_id:
                self._zones[zone_id].autotune_running = False
            hint = ""
            low = fail.lower()
            # 2.0.x: "Bad extruder number".  2.1.x: "Bad heater id".
            if "extruder" in low or "heater id" in low:
                spec = zone_by_id(zone_id) if zone_id else None
                sym = spec.pid_config_symbol if spec else "PIDTEMPBED"
                hint = (
                    f"Marlin rejected the autotune because this heater has no PID "
                    f"compiled in — enable {sym} in Configuration.h."
                )
            elif "timeout" in low:
                hint = (
                    "The autotune cycle exceeded Marlin's per-cycle time limit. "
                    "That is expected on a large water thermal mass with a "
                    "low-power heater; try fewer cycles, or set the PID manually."
                )
            self.telemetry.log_event("autotune_failed", zone=zone_id, reason=fail)
            self._fire(self._cb_at_done, zone_id, None, fail, hint)
            self._status(f"PID autotune failed: {fail}. {hint}")
            return

        res = parse_autotune_result(text)
        if res is not None:
            _heater, which, value = res
            with self._at_lock:
                self._at_result[which] = value
                have = dict(self._at_result)
                zone_id = self._at_zone
            if {"p", "i", "d"} <= set(have) and zone_id:
                pid = PidValues(kp=have["p"], ki=have["i"], kd=have["d"])
                with self._at_lock:
                    self._at_zone = None
                    self._at_result = {}
                self._zones[zone_id].autotune_running = False
                self.telemetry.log_event(
                    "autotune_done", zone=zone_id,
                    kp=pid.kp, ki=pid.ki, kd=pid.kd,
                )
                self._fire(self._cb_at_done, zone_id, pid, "", "")
                self._status(
                    f"PID autotune finished: Kp={pid.kp:.2f} Ki={pid.ki:.2f} "
                    f"Kd={pid.kd:.2f}. Save to EEPROM to keep it."
                )
            return

        prog = parse_autotune_progress(text)
        if prog is not None:
            with self._at_lock:
                zone_id = self._at_zone
                elapsed = time.monotonic() - self._at_started
            self._fire(self._cb_at_progress, zone_id, prog, elapsed)

    # ═══════════════════════════════════════════════════════════════
    # Sampling / derived state
    # ═══════════════════════════════════════════════════════════════

    def _sample_loop(self) -> None:
        last_poll = 0.0
        while not self._stop.is_set():
            time.sleep(SAMPLE_PERIOD_S)
            if self._stop.is_set():
                break

            now = time.monotonic()
            if self._use_polling and (now - last_poll) >= POLL_PERIOD_S:
                # v7.18: on the shared transport the link yields the channel
                # while a print owns it (the position poller is suspended for
                # PRINT_PATH bursts) — skip the round instead of adding
                # lock-contention jitter to print pacing. Marlin holds the
                # setpoint on its own the whole time.
                if self._poll_permitted():
                    last_poll = now
                    self.submit(self._do_poll_once)

            try:
                self._publish_sample()
            except Exception:
                logger.debug("sample publish failed", exc_info=True)

    def _publish_sample(self) -> None:
        channels = self.hub.all_channels()
        if channels:
            self._fire(self._cb_channels, channels)
        self._check_data_flowing(channels)

        zone_reports: dict[str, StabilityReport] = {}
        for spec in ALL_ZONES:
            rt = self._zones[spec.zone_id]
            ch = self.hub.marlin_channel(spec.temp_key)
            if ch is None:
                continue
            rt.tracker.set_target(rt.requested_c if rt.requested_c > 0 else None)
            rt.tracker.add(ch.value_c, ch.power_pct)
            rep = rt.tracker.report()
            zone_reports[spec.zone_id] = rep
            self._fire(self._cb_zone, spec.zone_id, rep, rt)

        self._service_dither()
        self._service_setpoint_keeper()

        vals = {c.uid: c.value_c for c in channels}
        ev = self.divergence.check(vals)
        if ev is not None:
            self.telemetry.log_event("divergence", **{
                "a": ev.channel_a, "b": ev.channel_b, "delta": ev.delta_c,
            })
            self._fire(self._cb_divergence, ev)
            self._status(f"SENSOR DIVERGENCE — {ev.message}")
            if self.divergence.action == "alarm_and_off":
                self._status("Divergence action: commanding both heaters OFF.")
                self.all_heaters_off()

        if self.telemetry.active:
            self.telemetry.log_sample(
                channels,
                {
                    zid: {
                        "temp": r.temp_c, "target": r.target_c, "duty": r.duty_pct,
                        "rate": r.rate_c_per_min, "ripple": r.ripple_c,
                        "steady_duty": r.steady_duty_pct, "settled": r.settled,
                    }
                    for zid, r in zone_reports.items()
                },
            )

    #: How far the board's own reported target may differ from what we
    #: commanded before it counts as a disagreement. Marlin stores whole
    #: degrees, so anything at or above 1 °C is a real difference.
    TARGET_MATCH_BAND_C = 0.6

    #: Consecutive disagreeing samples before the host puts the target back.
    #: Two, never one: a sample taken between issuing a command and the board
    #: applying it legitimately still shows the old target.
    REASSERT_AFTER_SAMPLES = 2

    def _service_setpoint_keeper(self) -> None:
        """
        Put back a setpoint the BOARD has forgotten.

        A Marlin reset clears every heater target to 0, and this board resets
        routinely: opening its serial port pulses DTR, so EVERY ZP
        auto-reconnect reboots it. ``app.log`` says so in as many words —
        *"ZP auto-reconnect succeeded — re-declare the Z position (Marlin
        reset on reopen)"*. The Z position is re-declared after that reset.
        The heater target was not, by anyone.

        Nothing re-asserted it because the ramp deliberately finishes the
        moment it has commanded the final target — its own comment reads
        *"from this point the firmware owns the hold"*, which is true right
        up until the firmware forgets. The result is the fault reported on
        2026-08-12/13: the page shows a 37 °C hold, the board is sitting at
        target 0, and nothing heats. Bench-proved on ME3B V1 that same
        session — a direct ``M140 S37`` held 36.7-38.0 °C for three minutes
        with no drift, so the hardware was never the problem.

        M105 reports the board's OWN target, so the disagreement is directly
        observable rather than inferred. Every re-assert is logged, because a
        board that keeps forgetting is itself a fault worth seeing.
        """
        if not self._connected or self.fault_latch.active:
            return
        for spec in ALL_ZONES:
            rt = self._zones[spec.zone_id]
            # A refused command is a verdict, not a target to retry — that is
            # the 48-minute bug this file already records.
            if rt.refused or rt.requested_c <= 0 or rt.commanded_c <= 0:
                rt._target_mismatch = 0
                continue
            ch = self.hub.marlin_channel(spec.temp_key)
            if ch is None or ch.stale or ch.target_c is None:
                # Cannot judge without a fresh reading from the board.
                rt._target_mismatch = 0
                continue
            if abs(float(ch.target_c) - float(rt.commanded_c)) \
                    < self.TARGET_MATCH_BAND_C:
                rt._target_mismatch = 0
                continue

            rt._target_mismatch += 1
            if rt._target_mismatch < self.REASSERT_AFTER_SAMPLES:
                continue
            rt._target_mismatch = 0
            rt.reasserts += 1
            board_target = float(ch.target_c)
            logger.warning(
                "incubator %s: the board's target is %.1f C but %d C was "
                "commanded - re-asserting (board reset #%d)",
                spec.zone_id, board_target, rt.commanded_c, rt.reasserts)
            self.telemetry.log_event(
                "setpoint_reasserted", zone=spec.zone_id,
                board_target=board_target, commanded=rt.commanded_c,
                count=rt.reasserts)
            # Say it the first few times. A board reset-looping would
            # otherwise bury every other message.
            if rt.reasserts <= 3:
                self._status(
                    f"{spec.title}: the board had forgotten its setpoint "
                    f"(it reported {board_target:.0f} °C, not "
                    f"{rt.commanded_c} °C) — putting it back. A Marlin reset "
                    f"clears every heater target, and re-opening the board's "
                    f"port resets it, so this happens on every ZP reconnect. "
                    f"If it keeps repeating, the board is rebooting under you."
                )
            self.submit(self._send_zone_cmd, spec.zone_id,
                        spec.set_target(rt.commanded_c))

    def _poll_permitted(self) -> bool:
        """True unless the shared link is yielding the channel to a print."""
        link = self._link
        if link is None:
            return True
        gate = getattr(link, "poll_allowed", None)
        if gate is None:
            return True
        try:
            return bool(gate())
        except Exception:
            return True

    def _check_data_flowing(self, channels) -> None:
        """
        Warn when the board goes quiet without the port actually failing.

        The reader thread notices an UNPLUGGED device (reads raise), but a board
        that has hung, been reset, or stopped autoreporting keeps a perfectly
        valid port handle while sending nothing — reads just time out. That looks
        identical to "idle" unless we watch data age, so the operator would sit
        in front of a frozen readout with no indication anything was wrong.
        """
        if not self._connected or self.fault_latch.active:
            self._stale_warned = False
            return

        # v7.18: while polling is deliberately paused (a print owns the shared
        # channel) stale data is EXPECTED — warning would cry wolf, and the
        # queued extra poll below would defeat the yield.
        if not self._poll_permitted():
            self._stale_warned = False
            return

        marlin = [c for c in channels if c.source_id == MARLIN_SOURCE_ID]
        all_stale = bool(marlin) and all(c.stale for c in marlin)

        if all_stale and not getattr(self, "_stale_warned", False):
            self._stale_warned = True
            self._status(
                f"No temperature data for over {self.hub.STALE_AFTER_S:.0f} s — the "
                f"board has gone quiet (hung, reset, or autoreport stopped) even "
                f"though the port is still open. Try Refresh; if that fails, "
                f"power-cycle the board. It may still be heating."
            )
            self.submit(self._do_poll_once)
        elif not all_stale:
            self._stale_warned = False

    def _calibrate_reading(self, key: str, raw_c: float) -> tuple[float, bool]:
        return self.calibration.apply(f"{MARLIN_SOURCE_ID}:{key}", raw_c)

    # ═══════════════════════════════════════════════════════════════
    # Setpoints
    # ═══════════════════════════════════════════════════════════════

    def zone_runtime(self, zone_id: str) -> ZoneRuntime:
        return self._zones[zone_id]

    def preview_setpoint(self, zone_id: str, real_c: float) -> dict:
        """
        Work out exactly what will be sent, without sending it.

        Returned dict is what the UI shows so the operator can see the whole
        chain: requested -> calibration-inverted -> integer-quantised -> the real
        temperature that should result. On this firmware the quantisation step is
        real (targets are ``int16_t``), so hiding it would be a lie.
        """
        spec = zone_by_id(zone_id)
        chk = check_setpoint(
            real_c,
            current_target_c=self._zones[zone_id].requested_c,
            max_c=self.MAX_SETPOINT_C,
        )
        uid = f"{MARLIN_SOURCE_ID}:{spec.temp_key}"
        raw_wanted, was_cal = self.calibration.invert(uid, chk.allowed_c)
        commanded = int(round(raw_wanted))
        cal = self.calibration.get(uid)
        predicted_real = cal.apply(commanded) if cal.active else float(commanded)
        return {
            "check": chk,
            "requested_c": chk.allowed_c,
            "raw_wanted_c": raw_wanted,
            "commanded_c": commanded,
            "predicted_real_c": predicted_real,
            "calibrated": was_cal,
            "quantisation_error_c": predicted_real - chk.allowed_c,
        }

    def set_target(self, zone_id: str, real_c: float, *,
                   board_side_wait: bool = False) -> SetpointCheck:
        """
        Command a setpoint. Non-blocking by default (see the module docstring on
        why we avoid ``M190``/``M109``).

        Returns the :class:`SetpointCheck` so the caller can see whether the value
        was clamped or wants confirmation; the command is queued regardless, so
        UIs should call :meth:`preview_setpoint` and confirm BEFORE calling this.
        """
        spec = zone_by_id(zone_id)
        plan = self.preview_setpoint(zone_id, real_c)
        rt = self._zones[zone_id]

        # Refuse to heat a zone whose sensor is not reporting a real temperature.
        # The firmware would trip MINTEMP the instant the heater came on, but
        # failing loudly here explains WHY instead of presenting a dead board.
        if not rt.sensor_ok and plan["requested_c"] > 0:
            self._status(
                f"{spec.title}: refusing to heat — {rt.sensor_fault} "
                f"Fix the sensor wiring first."
            )
            return plan["check"]

        # A direct setpoint supersedes a running ramp; otherwise the ramp's
        # next rung would silently overwrite what the operator just asked for.
        self.stop_ramp(zone_id, reason="superseded by a direct setpoint")

        # v7.18: a board-side wait (M190/M109) answers its 'ok' when the
        # temperature ARRIVES — on this rig that can be most of an hour, and
        # on the shared transport the transaction would hold the motion
        # board's serial lock the whole time, starving the position poller
        # into a false "ZP disconnected". Degrade to the non-blocking form:
        # the outcome is identical (the host already decides arrived/stable
        # from the temperature stream — that IS the default design).
        if board_side_wait and self._transport == "shared":
            board_side_wait = False
            self._status(
                f"{spec.title}: board-side waits are unavailable on the "
                f"shared ZP link (they would block the motion channel for "
                f"the whole heat-up) — using the normal host-side hold, "
                f"which behaves the same."
            )

        rt.requested_c = plan["requested_c"]
        rt.commanded_c = plan["commanded_c"]
        rt.dither_enabled = False
        rt.refused = ""  # a fresh operator action gets a fresh verdict
        rt.tracker.set_target(rt.requested_c if rt.requested_c > 0 else None)

        cmd = (
            spec.wait_for_target(plan["commanded_c"])
            if board_side_wait
            else spec.set_target(plan["commanded_c"])
        )
        self.telemetry.log_event(
            "setpoint", zone=zone_id, requested=plan["requested_c"],
            commanded=plan["commanded_c"], wait=board_side_wait,
        )
        # A board-side wait can legitimately run for a very long time on this
        # rig, hence the generous ceiling.
        self.submit(
            self._send_zone_cmd, zone_id, cmd,
            timeout_s=20.0 if board_side_wait else 8.0,
            hard_ceiling_s=5400.0 if board_side_wait else None,
        )
        return plan["check"]

    # ── watchdog-safe staircase ramp ────────────────────────────────

    def start_ramp(self, zone_id: str, real_c: float,
                   *, step_c: float | None = None) -> bool:
        """
        Reach ``real_c`` by walking the setpoint up in small steps.

        This exists because Marlin's heat-up watchdog false-trips on this rig: a
        low-power heater against a large water mass cannot rise 2 °C in 60 s, so
        the firmware concludes the heater has failed and halts the board. Each
        step here stays below the threshold at which that watchdog even arms, so
        the check is never scheduled. See :mod:`.ramp` for the source-verified
        reasoning. Runaway and MIN/MAXTEMP protection are unaffected.
        """
        from .ramp import DEFAULT_STEP_C, SetpointRamp

        spec = zone_by_id(zone_id)
        rt = self._zones[zone_id]
        plan = self.preview_setpoint(zone_id, real_c)
        target = plan["requested_c"]

        if not rt.sensor_ok and target > 0:
            self._status(
                f"{spec.title}: refusing to heat — {rt.sensor_fault} "
                f"Fix the sensor wiring first."
            )
            return False

        existing = self._ramps.get(zone_id)
        if existing is not None and existing.active:
            existing.stop("superseded")

        ramp = SetpointRamp(
            zone_id,
            read_temp=lambda k=spec.temp_key: (
                (ch.value_c if (ch := self.hub.marlin_channel(k)) else None)
            ),
            command_target=lambda c, z=zone_id: self._ramp_command(z, c),
            on_state=lambda st, z=zone_id: self._fire(self._cb_ramp, z, st),
            on_status=self._status,
            step_c=DEFAULT_STEP_C if step_c is None else step_c,
        )
        self._ramps[zone_id] = ramp

        rt.dither_enabled = False
        rt.refused = ""  # a fresh operator action gets a fresh verdict
        rt.requested_c = target
        rt.tracker.set_target(target if target > 0 else None)
        self.telemetry.log_event(
            "ramp_start", zone=zone_id, target=target, step=ramp.step_c
        )
        return ramp.start(target)

    def _ramp_command(self, zone_id: str, celsius: float) -> None:
        """Command one rung of a ramp (called from the ramp's own thread)."""
        spec = zone_by_id(zone_id)
        rt = self._zones[zone_id]
        rt.commanded_c = int(round(celsius))
        self.telemetry.log_event("ramp_step", zone=zone_id, commanded=rt.commanded_c)
        self.submit(self._send_zone_cmd, zone_id,
                    spec.set_target(rt.commanded_c))

    def stop_ramp(self, zone_id: str, *, reason: str = "cancelled") -> None:
        ramp = self._ramps.get(zone_id)
        if ramp is not None and ramp.active:
            ramp.stop(reason)
            self.telemetry.log_event("ramp_stop", zone=zone_id, reason=reason)

    def ramp_state(self, zone_id: str):
        ramp = self._ramps.get(zone_id)
        return None if ramp is None else ramp.snapshot()

    def ramp_active(self, zone_id: str) -> bool:
        ramp = self._ramps.get(zone_id)
        return bool(ramp is not None and ramp.active)

    def heater_off(self, zone_id: str) -> None:
        spec = zone_by_id(zone_id)
        rt = self._zones[zone_id]
        self.stop_ramp(zone_id, reason="heater turned off")
        rt.requested_c = 0.0
        rt.commanded_c = 0
        rt.dither_enabled = False
        rt.refused = ""  # a fresh operator action gets a fresh verdict
        rt.tracker.set_target(None)
        self.telemetry.log_event("heater_off", zone=zone_id)
        # A refused OFF matters even more than a refused setpoint — the
        # operator believes the heater is off when it is not.
        self.submit(self._send_zone_cmd, zone_id, spec.heater_off())

    def all_heaters_off(self) -> None:
        for spec in ALL_ZONES:
            self.heater_off(spec.zone_id)

    def emergency_stop(self) -> None:
        """
        M112. This HALTS the board — it is not a soft heater-off, and the board
        will ignore everything until it is power-cycled. Sent out-of-band so it
        works even mid-wait or mid-autotune.
        """
        self.telemetry.log_event("emergency_stop")
        self._status(
            "M112 EMERGENCY STOP sent. The board is halted and must be "
            "POWER-CYCLED before it will respond again."
        )
        if self._link is not None:
            self._link.send_priority("M112")

    def cancel_wait(self) -> None:
        """M108 — break out of a board-side wait or a running autotune."""
        self.telemetry.log_event("cancel_wait")
        if self._link is not None:
            self._link.send_priority("M108")
        note = ""
        if self.report is not None and self.report.emergency_parser is False:
            note = (
                " NOTE: EMERGENCY_PARSER is disabled in this firmware, so the "
                "cancel will only take effect once the current command finishes."
            )
        self._status("Sent M108 to interrupt the current wait/autotune." + note)

    # ── fine setpoint via host dither ───────────────────────────────

    def set_fine_target(self, zone_id: str, real_c: float,
                        *, period_s: float = 60.0) -> None:
        """
        Hold a FRACTIONAL temperature despite integer-only firmware setpoints.

        Marlin cannot be told 37.4 C. But the block's thermal time constant is
        enormous compared with a minute, so alternating the integer setpoint
        between 37 and 38 with the right duty ratio produces a smooth average of
        37.4 — the mass itself does the low-pass filtering. With a fast, low-mass
        load this would be a bad idea; here it is well matched to the physics.
        """
        spec = zone_by_id(zone_id)
        chk = check_setpoint(real_c, max_c=self.MAX_SETPOINT_C)
        rt = self._zones[zone_id]
        rt.requested_c = chk.allowed_c
        rt.dither_target_c = chk.allowed_c
        rt.refused = ""  # a fresh operator action gets a fresh verdict
        rt.tracker.set_target(chk.allowed_c)

        frac = chk.allowed_c - int(chk.allowed_c)
        if frac < 0.02 or frac > 0.98:
            rt.dither_enabled = False
            self.set_target(zone_id, chk.allowed_c)
            return

        rt.dither_enabled = True
        rt._dither_next_flip = 0.0
        rt._dither_high = False
        rt._dither_period_s = max(10.0, float(period_s))  # type: ignore[attr-defined]
        self.telemetry.log_event(
            "fine_setpoint", zone=zone_id, target=chk.allowed_c, period_s=period_s
        )
        self._status(
            f"{spec.title}: holding {chk.allowed_c:.2f} °C by alternating the "
            f"integer setpoint between {int(chk.allowed_c)} and "
            f"{int(chk.allowed_c) + 1} °C."
        )

    def _service_dither(self) -> None:
        now = time.monotonic()
        for rt in self._zones.values():
            if not rt.dither_enabled or rt.dither_target_c <= 0:
                continue
            if now < rt._dither_next_flip:
                continue
            period = float(getattr(rt, "_dither_period_s", 60.0))
            lo = int(rt.dither_target_c)
            frac = rt.dither_target_c - lo
            high_s = max(1.0, period * frac)
            low_s = max(1.0, period - high_s)

            rt._dither_high = not rt._dither_high
            value = lo + 1 if rt._dither_high else lo
            rt._dither_next_flip = now + (high_s if rt._dither_high else low_s)
            rt.commanded_c = value
            self.submit(self._send_zone_cmd, rt.zone_id,
                        rt.spec.set_target(value))

    # ═══════════════════════════════════════════════════════════════
    # PID
    # ═══════════════════════════════════════════════════════════════

    def _pid_snapshot(self) -> dict[str, PidValues | None]:
        return {zid: rt.pid for zid, rt in self._zones.items()}

    def query_pid(self) -> None:
        self.submit(self._do_query_pid)

    def _do_query_pid(self) -> None:
        txn = self._send("M503", timeout_s=12.0)
        if txn is None:
            return
        for spec in ALL_ZONES:
            rt = self._zones[spec.zone_id]
            pid = parse_pid_dump(txn.lines, spec.pid_cmd)
            rt.pid = pid
            rt.pid_available = pid is not None
        self._fire(self._cb_pid, self._pid_snapshot())

    def set_pid(self, zone_id: str, kp: float, ki: float, kd: float) -> None:
        spec = zone_by_id(zone_id)
        rt = self._zones[zone_id]
        if not rt.pid_available:
            self._status(
                f"{spec.title}: cannot set PID — {spec.pid_config_symbol} is not "
                f"enabled in this firmware (the zone runs bang-bang)."
            )
            return
        self.telemetry.log_event("set_pid", zone=zone_id, kp=kp, ki=ki, kd=kd)
        self.submit(self._do_set_pid, zone_id, kp, ki, kd)

    def _do_set_pid(self, zone_id: str, kp: float, ki: float, kd: float) -> None:
        spec = zone_by_id(zone_id)
        txn = self._send(spec.set_pid(kp, ki, kd))
        if txn is not None and txn.ok:
            self._zones[zone_id].pid = PidValues(kp=kp, ki=ki, kd=kd)
            self._fire(self._cb_pid, self._pid_snapshot())
            self._status(
                f"{spec.title}: PID applied in RAM. Save to EEPROM to persist it."
            )

    def save_eeprom(self) -> None:
        self.telemetry.log_event("eeprom_save")
        self.submit(self._eeprom_cmd, "M500", "Settings saved to EEPROM.")

    def load_eeprom(self) -> None:
        self.telemetry.log_event("eeprom_load")
        self.submit(self._eeprom_cmd, "M501",
                    "Settings reloaded from EEPROM (unsaved changes discarded).")
        self.submit(self._do_query_pid)

    def factory_reset(self) -> None:
        self.telemetry.log_event("eeprom_factory_reset")
        self.submit(self._eeprom_cmd, "M502",
                    "Factory defaults loaded into RAM (not yet saved).")
        self.submit(self._do_query_pid)

    def _eeprom_cmd(self, cmd: str, ok_text: str) -> None:
        txn = self._send(cmd, timeout_s=12.0)
        if txn is not None and txn.ok:
            self._status(ok_text)
        elif txn is not None:
            self._status(f"{cmd} was not acknowledged: {txn.error_text}")

    # ═══════════════════════════════════════════════════════════════
    # Autotune
    # ═══════════════════════════════════════════════════════════════

    def start_autotune(self, zone_id: str, target_c: float, cycles: int = 5,
                       *, apply_result: bool = True) -> bool:
        spec = zone_by_id(zone_id)
        rt = self._zones[zone_id]
        # v7.18: an M303 answers its 'ok' only when the WHOLE tune finishes —
        # minutes to hours on this thermal mass. On the shared transport that
        # would hold the motion board's serial lock and starve the position
        # poller into a false "ZP disconnected", so it is refused outright
        # with the alternatives named (refusing beats a silent hang).
        if self._transport == "shared":
            self._status(
                f"{spec.title}: PID autotune is unavailable over the shared "
                f"ZP link — the tune would monopolise the motion board's "
                f"serial channel for its whole duration. Either set the PID "
                f"manually (PID tab), or disconnect the ZP board in the app "
                f"and run the autotune over a dedicated connection."
            )
            return False
        if rt.autotune_running:
            self._status(f"{spec.title}: autotune already running.")
            return False
        if not rt.pid_available:
            self._status(
                f"{spec.title}: autotune needs PID support — enable "
                f"{spec.pid_config_symbol} in Configuration.h first."
            )
            return False

        chk = check_setpoint(target_c, max_c=self.MAX_SETPOINT_C)
        with self._at_lock:
            self._at_zone = zone_id
            self._at_result = {}
            self._at_started = time.monotonic()
        rt.autotune_running = True

        self.telemetry.log_event(
            "autotune_start", zone=zone_id, target=chk.allowed_c, cycles=cycles
        )
        self._status(
            f"{spec.title}: PID autotune started at {chk.allowed_c:.0f} °C, "
            f"{cycles} cycles. On a large water mass this can take a long time — "
            f"Cancel sends M108."
        )
        self.submit(self._do_autotune, zone_id, chk.allowed_c, cycles, apply_result)
        return True

    def _do_autotune(self, zone_id: str, target_c: float, cycles: int,
                     apply_result: bool) -> None:
        spec = zone_by_id(zone_id)
        # Rolling deadline handles the long quiet stretches; the ceiling stops a
        # runaway. Marlin caps a cycle at ~20 min, so allow generously beyond it.
        txn = self._send(
            spec.autotune(target_c, cycles, apply_result),
            timeout_s=180.0,
            hard_ceiling_s=max(1800.0, cycles * 1500.0),
        )
        if txn is not None and txn.timed_out:
            with self._at_lock:
                self._at_zone = None
            self._zones[zone_id].autotune_running = False
            self._fire(self._cb_at_done, zone_id, None, "host timeout",
                       "The host stopped waiting. The BOARD may still be tuning.")
            self._status(
                f"{spec.title}: autotune timed out on the host side. The board may "
                f"still be running it — check the console."
            )

    def cancel_autotune(self) -> None:
        """
        Best-effort cancel via M108.

        Honest caveat surfaced to the UI: without ``EMERGENCY_PARSER`` the
        command queues, and Marlin may finish the tune regardless.
        """
        with self._at_lock:
            zone_id = self._at_zone
        self.cancel_wait()
        if zone_id:
            self._status(
                f"Cancel requested for {zone_by_id(zone_id).title} autotune. If "
                f"the board does not stop, wait for it to finish — it cannot be "
                f"aborted any harder without a reset."
            )

    # ═══════════════════════════════════════════════════════════════
    # Misc commands
    # ═══════════════════════════════════════════════════════════════

    def _do_autoreport(self, interval_s: int) -> None:
        txn = self._send(build_autoreport(interval_s))
        if txn is None or not txn.ok:
            self._use_polling = True

    def _do_poll_once(self) -> None:
        self._send("M105", timeout_s=5.0)

    def send_raw(self, command: str) -> None:
        """Send an arbitrary G-code line, for full manual control."""
        cmd = (command or "").strip()
        if not cmd:
            return
        head = cmd.split()[0].upper()
        if head in ("M112", "M108", "M410"):
            if self._link is not None:
                self._link.send_priority(cmd)
            return
        self.telemetry.log_event("raw", cmd=cmd)
        self.submit(self._send, cmd, timeout_s=15.0)

    def refresh_now(self) -> None:
        self.submit(self._do_poll_once)

    # ── sensor re-check ─────────────────────────────────────────────

    def rescan_sensors(self) -> None:
        """
        Re-read the sensors and update the cached capability verdict.

        The connect-time probe decides once whether each zone's sensor is usable,
        and that verdict gates heating. Plug a thermistor in afterwards and the
        board reports it immediately — but the tool would keep refusing to heat
        until reconnected, which looks like a bug. This is the fix: cheap, safe,
        touches no heater, and can be pressed at any time.
        """
        if not self._connected:
            self._status("Not connected — nothing to re-check.")
            return
        if self.fault_latch.active:
            # Worth being blunt: every fault we latch on is one that makes Marlin
            # call kill(), after which it ignores all G-code. Re-reading sensors
            # cannot recover that, and implying otherwise would waste the
            # operator's time on a board that is already halted.
            self._status(
                "The board is halted by a latched fault, so it will not answer a "
                "sensor re-read. Fix the wiring, POWER-CYCLE the board, then "
                "reconnect."
            )
            return
        self.submit(self._do_rescan_sensors)

    def _do_rescan_sensors(self) -> None:
        from .probe import rescan_sensors as _rescan

        if self._link is None:
            return
        if self.report is None:
            self.report = FirmwareReport()

        try:
            res = _rescan(self._link, self.report)
        except Exception as e:
            logger.warning("sensor rescan failed", exc_info=True)
            self._status(f"Sensor re-check failed: {e}")
            return

        titles = {z.zone_id: z.title for z in ALL_ZONES}

        if res.ok:
            for spec in ALL_ZONES:
                cap = self.report.zone(spec.zone_id)
                rt = self._zones[spec.zone_id]
                rt.sensor_ok = cap.sensor_ok
                rt.sensor_fault = cap.sensor_fault

            # A recovered zone's history spans the jump from an open-circuit
            # reading to a real one. Left in place that discontinuity would be
            # reported as a wild rate of change and a huge ripple, so the derived
            # statistics start fresh from the good data.
            for zone_id in res.recovered:
                self._zones[zone_id].tracker.reset()

        self.telemetry.log_event(
            "sensor_rescan",
            ok=res.ok,
            recovered=res.recovered,
            lost=res.lost,
            still_bad=res.still_bad,
            not_configured=res.not_configured,
        )
        # Re-publishing the report is what actually refreshes the zone cards,
        # the firmware tab and the banner — they all render from it.
        self._fire(self._cb_probe, self.report)
        self._status(res.summary(titles))

    def acknowledge_fault(self) -> None:
        self.fault_latch.acknowledge()
        self._status(
            "Fault acknowledged. If the board was halted it still needs a "
            "power-cycle before it will respond."
        )

    # ── calibration helpers ─────────────────────────────────────────

    def calibrate_single_point(self, zone_id: str, reference_c: float) -> bool:
        """Set a one-point offset for a zone from a reference thermometer."""
        spec = zone_by_id(zone_id)
        ch = self.hub.marlin_channel(spec.temp_key)
        if ch is None:
            self._status(f"{spec.title}: no reading yet to calibrate against.")
            return False
        cal = self.calibration.get(ch.uid)
        cal.set_single_point(ch.raw_c, float(reference_c))
        self.calibration.save()
        self.telemetry.log_event(
            "calibrate", zone=zone_id, raw=ch.raw_c, ref=reference_c,
            mode=cal.mode,
        )
        self._status(
            f"{spec.title}: calibrated — board read {ch.raw_c:.2f} °C, reference "
            f"{reference_c:.2f} °C, {cal.describe()}."
        )
        return True

    def clear_calibration(self, zone_id: str) -> None:
        spec = zone_by_id(zone_id)
        self.calibration.clear(f"{MARLIN_SOURCE_ID}:{spec.temp_key}")
        self.calibration.save()
        self._status(f"{spec.title}: calibration cleared.")

    # ── telemetry ───────────────────────────────────────────────────

    def start_logging(self, label: str = "hold"):
        manifest = {
            "firmware": (self.report.firmware_name if self.report else None),
            "simulated": self._sim,
            "zones": {
                z.zone_id: {
                    "title": z.title,
                    "temp_key": z.temp_key,
                    "pid_available": self._zones[z.zone_id].pid_available,
                }
                for z in ALL_ZONES
            },
            "max_setpoint_c": self.MAX_SETPOINT_C,
        }
        path = self.telemetry.start(label, manifest)
        self._status(f"Logging to {path}" if path else "Logging is disabled.")
        return path

    def stop_logging(self) -> None:
        self.telemetry.stop("user stopped")
        self._status("Logging stopped.")
