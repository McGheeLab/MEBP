"""
probe.py — discover what this firmware can actually do, at connect time.

The alternative to probing is guessing, and guessing about firmware build
options produces the worst class of bug on a rig like this: a control that looks
functional, silently does nothing, and leaves the operator wondering whether the
hardware is broken. Every question below has a concrete answer available over
the wire, so we ask.

What gets determined, and why it matters:

``firmware_name``
    Confirms we are talking to Marlin at all, and which version.

``capabilities`` (``Cap:`` lines from M115)
    Only present when ``EXTENDED_CAPABILITIES_REPORT`` is compiled in. A MISSING
    capability line means "unknown", never "off" — so we probe behaviourally
    where it matters instead of trusting absence.

``sensor_fields``
    Which ``M105`` fields this board actually emits. Drives the sensor table and
    catches, for example, a board with no hotend thermistor configured (no
    ``T:``) — in which case Zone B cannot be controlled at all.

``bed_pid`` / ``hotend_pid``
    Whether ``M503`` dumps an ``M304`` / ``M301`` line. Stock Marlin ships with
    ``PIDTEMPBED`` **disabled**, so the bed usually runs bang-bang and ``M304`` /
    ``M303 E-1`` are rejected. This has to be reported honestly rather than shown
    as Kp=0.

``setpoint_resolution_c``
    Marlin stores targets as ``celsius_t`` (``int16_t``), so a fractional target
    is truncated. We verify empirically by commanding a ``.5`` value and reading
    the target field back — then restore whatever was set before. Confirms the
    quantisation instead of assuming it from a version number.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from .marlin_gcode import (
    parse_capabilities,
    parse_firmware_name,
    parse_pid_dump,
    parse_temp_line,
    PidValues,
)
from .zones import ALL_ZONES, BED_ZONE, HOTEND_ZONE, ZoneSpec

logger = logging.getLogger(__name__)


#: A thermistor that is unplugged reads as an open circuit, which Marlin's
#: conversion tables render as a large negative number (e.g. -15 C). A shorted
#: sensor or the wrong table reads implausibly high. Neither is a temperature.
SENSOR_PLAUSIBLE_MIN_C = 0.0
SENSOR_PLAUSIBLE_MAX_C = 200.0

#: The resolution probe needs a reading at least this warm, because it works by
#: commanding a target BELOW the current temperature (so the heater never fires).
RESOLUTION_PROBE_MIN_C = 10.0


@dataclass
class ZoneCapability:
    """What we learned about one zone."""

    zone_id: str
    sensor_present: bool = False
    pid_available: bool = False
    pid: PidValues | None = None
    #: Last raw reading seen for this zone's sensor, if any.
    reading_c: float | None = None
    #: Non-empty when the sensor reports an implausible value. Heating a zone
    #: whose sensor is broken is how thermal runaway happens, so this blocks
    #: control rather than just annotating it.
    sensor_fault: str = ""
    #: Populated when something is missing, in operator-actionable terms.
    note: str = ""

    @property
    def sensor_ok(self) -> bool:
        return self.sensor_present and not self.sensor_fault

    @property
    def control_mode(self) -> str:
        if not self.sensor_present:
            return "no sensor"
        if self.sensor_fault:
            return "sensor fault"
        return "PID" if self.pid_available else "bang-bang"


@dataclass
class SensorRescan:
    """
    Outcome of re-reading the sensors on an already-connected board.

    Exists because a thermistor is very often plugged in *after* the tool is
    running, and the connect-time probe result would otherwise keep heating
    blocked until the operator thought to disconnect and reconnect.
    """

    ok: bool = False
    #: Zones that went from unusable to usable — heating is now permitted.
    recovered: list[str] = field(default_factory=list)
    #: Zones that went from usable to faulted (a sensor was unplugged).
    lost: list[str] = field(default_factory=list)
    #: Zones still faulted, for the same reason as before.
    still_bad: list[str] = field(default_factory=list)
    #: Zones the FIRMWARE has no sensor configured for. Plugging a thermistor in
    #: cannot fix these — they need a rebuild — so they are reported separately
    #: rather than lumped in with wiring faults.
    not_configured: list[str] = field(default_factory=list)
    #: Fresh readings by zone id, for the status message.
    readings: dict[str, float | None] = field(default_factory=dict)
    error: str = ""

    @property
    def changed(self) -> bool:
        return bool(self.recovered or self.lost)

    def summary(self, titles: dict[str, str] | None = None) -> str:
        """One line for the status bar, saying what actually changed."""
        def name(zid: str) -> str:
            return (titles or {}).get(zid, zid)

        if not self.ok:
            return self.error or "Sensor re-check failed."

        bits: list[str] = []
        for zid in self.recovered:
            r = self.readings.get(zid)
            bits.append(
                f"{name(zid)} sensor is now reading"
                + (f" {r:.2f} °C" if r is not None else "")
                + " — heating unblocked"
            )
        for zid in self.lost:
            bits.append(f"{name(zid)} sensor has FAILED — heating blocked")
        for zid in self.still_bad:
            r = self.readings.get(zid)
            bits.append(
                f"{name(zid)} sensor still bad"
                + (f" (reads {r:.2f} °C)" if r is not None else "")
            )
        for zid in self.not_configured:
            bits.append(
                f"{name(zid)} has no sensor in this FIRMWARE build — wiring one "
                f"in changes nothing until TEMP_SENSOR is set and reflashed"
            )
        if not bits:
            return "Sensors re-checked — all healthy, nothing changed."
        return "Sensors re-checked: " + "; ".join(bits) + "."


def read_temp_frame(link, *, timeout_s: float = 6.0):
    """Send one ``M105`` and return the first parsable frame, or ``None``."""
    txn = link.send_and_wait("M105", timeout_s=timeout_s)
    for line in txn.lines:
        frame = parse_temp_line(line)
        if frame is not None:
            return frame
    return None


def evaluate_zone_sensor(spec: ZoneSpec, frame) -> ZoneCapability:
    """
    Decide from one temperature frame whether this zone's sensor is usable.

    Split out of the probe so a later re-read applies **exactly** the same test.
    Two copies of a plausibility threshold is precisely how a rescan ends up
    disagreeing with the connect probe about whether it is safe to heat.

    Only the sensor fields are filled in; PID fields are left at their defaults
    for the caller to merge, because PID availability is a compile-time property
    that a sensor re-read tells us nothing about.
    """
    cap = ZoneCapability(zone_id=spec.zone_id)
    if not frame:
        return cap

    key = spec.temp_key
    cap.sensor_present = key in frame.fields
    # Marlin reports hotend 0 as 'T' but some builds emit 'T0'.
    if not cap.sensor_present and spec is HOTEND_ZONE and "T0" in frame.fields:
        cap.sensor_present = True
        key = "T0"
    if not cap.sensor_present:
        return cap

    reading = frame.temp(key)
    cap.reading_c = reading
    if reading is None:
        return cap

    if reading < SENSOR_PLAUSIBLE_MIN_C:
        cap.sensor_fault = (
            f"reads {reading:.1f} °C, which is not a real temperature — "
            f"an unplugged or broken thermistor reads as an open circuit. "
            f"Check the sensor on {spec.sensor_connector}."
        )
    elif reading > SENSOR_PLAUSIBLE_MAX_C:
        cap.sensor_fault = (
            f"reads {reading:.1f} °C — implausibly high. Check for a "
            f"shorted sensor on {spec.sensor_connector} or the wrong "
            f"TEMP_SENSOR table."
        )
    return cap


@dataclass
class FirmwareReport:
    """Everything the probe determined. Rendered directly by the UI."""

    ok: bool = False
    firmware_name: str | None = None
    capabilities: dict[str, bool] = field(default_factory=dict)
    sensor_fields: list[str] = field(default_factory=list)
    zones: dict[str, ZoneCapability] = field(default_factory=dict)
    setpoint_resolution_c: float = 1.0
    autoreport_supported: bool | None = None   # None = unknown
    eeprom_supported: bool | None = None
    emergency_parser: bool | None = None
    raw_m115: list[str] = field(default_factory=list)
    raw_m503: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    elapsed_s: float = 0.0

    # ── convenience for the UI ──────────────────────────────────────

    def zone(self, zone_id: str) -> ZoneCapability:
        return self.zones.get(zone_id) or ZoneCapability(zone_id=zone_id)

    @property
    def integer_setpoints(self) -> bool:
        return self.setpoint_resolution_c >= 0.999

    def warnings(self) -> list[str]:
        """
        Operator-facing list of things that will limit this rig, each naming the
        firmware option to change. Ordered most-consequential first.
        """
        out: list[str] = []

        for spec in ALL_ZONES:
            cap = self.zone(spec.zone_id)
            if not cap.sensor_present:
                out.append(
                    f"{spec.title}: no '{spec.temp_key}:' field in M105 — the "
                    f"firmware has no sensor configured on {spec.sensor_connector}. "
                    f"Set TEMP_SENSOR_{'BED' if spec is BED_ZONE else '0'} to your "
                    f"thermistor type. This zone cannot be controlled until then."
                )
            elif cap.sensor_fault:
                out.append(
                    f"{spec.title}: SENSOR FAULT — {cap.sensor_fault} Heating is "
                    f"blocked for this zone: driving a heater with a broken sensor "
                    f"is exactly how thermal runaway happens, and the firmware "
                    f"would trip MINTEMP the moment it was switched on."
                )
            elif not cap.pid_available:
                out.append(
                    f"{spec.title}: running BANG-BANG, not PID (no {spec.pid_cmd} "
                    f"line in M503). Enable {spec.pid_config_symbol} in "
                    f"Configuration.h for stable holding; expect ±1-2 °C swings "
                    f"until then."
                )

        if self.emergency_parser is False:
            out.append(
                "EMERGENCY_PARSER is disabled — M108/M112 will queue behind the "
                "current command instead of interrupting it, so 'Cancel' during a "
                "wait or autotune will be late. Enable it in Configuration_adv.h."
            )
        if self.eeprom_supported is False:
            out.append(
                "EEPROM is unavailable — PID values cannot be saved and will be "
                "lost on reset. Enable EEPROM_SETTINGS in Configuration.h."
            )
        if self.autoreport_supported is False:
            out.append(
                "AUTO_REPORT_TEMPERATURES is unavailable — falling back to host "
                "polling of M105, which is fine but slightly noisier."
            )
        if self.integer_setpoints:
            out.append(
                "Setpoints are whole degrees only (Marlin stores targets as "
                "int16_t). Use the fine-setpoint dither option if you need to "
                "hold a fractional temperature such as 37.5 °C."
            )
        return out


class FirmwareProbe:
    """Runs the probe sequence over a :class:`~.marlin_link.MarlinLink`."""

    def __init__(self, link, *, verbose: bool = True):
        self._link = link
        self._verbose = verbose

    def run(self) -> FirmwareReport:
        started = time.monotonic()
        rep = FirmwareReport()

        self._probe_identity(rep)
        frame = self._probe_sensors(rep)
        self._probe_pid(rep)
        self._probe_setpoint_resolution(rep, frame)

        rep.ok = bool(rep.firmware_name) or bool(rep.sensor_fields)
        rep.elapsed_s = time.monotonic() - started
        if self._verbose:
            logger.info(
                "[probe] fw=%s fields=%s bed=%s hotend=%s res=%.2fC in %.2fs",
                rep.firmware_name, rep.sensor_fields,
                rep.zone("bed").control_mode, rep.zone("hotend").control_mode,
                rep.setpoint_resolution_c, rep.elapsed_s,
            )
        return rep

    # ── steps ───────────────────────────────────────────────────────

    def _probe_identity(self, rep: FirmwareReport) -> None:
        txn = self._link.send_and_wait("M115", timeout_s=6.0)
        rep.raw_m115 = list(txn.lines)
        if txn.failed and not txn.lines:
            rep.errors.append(
                "M115 got no reply — is this a Marlin board, and is the baud rate "
                "right? (The ZP board uses 38400.)"
            )
            return
        rep.firmware_name = parse_firmware_name(txn.lines)
        rep.capabilities = parse_capabilities(txn.lines)

        # Only trust a capability that was explicitly reported.
        for key, attr in (
            ("AUTOREPORT_TEMP", "autoreport_supported"),
            ("EEPROM", "eeprom_supported"),
            ("EMERGENCY_PARSER", "emergency_parser"),
        ):
            if key in rep.capabilities:
                setattr(rep, attr, rep.capabilities[key])

    def _probe_sensors(self, rep: FirmwareReport):
        frame = read_temp_frame(self._link)
        if frame is None:
            rep.errors.append(
                "M105 returned no temperature fields — the firmware reports no "
                "sensors at all. Check TEMP_SENSOR_BED / TEMP_SENSOR_0."
            )
            return None

        rep.sensor_fields = frame.temperature_keys()
        for spec in ALL_ZONES:
            cap = rep.zones.setdefault(
                spec.zone_id, ZoneCapability(zone_id=spec.zone_id)
            )
            fresh = evaluate_zone_sensor(spec, frame)
            cap.sensor_present = fresh.sensor_present
            cap.reading_c = fresh.reading_c
            cap.sensor_fault = fresh.sensor_fault
        return frame

    def _probe_pid(self, rep: FirmwareReport) -> None:
        txn = self._link.send_and_wait("M503", timeout_s=10.0)
        rep.raw_m503 = list(txn.lines)
        for spec in ALL_ZONES:
            cap = rep.zones.setdefault(
                spec.zone_id, ZoneCapability(zone_id=spec.zone_id)
            )
            pid = parse_pid_dump(txn.lines, spec.pid_cmd)
            cap.pid = pid
            cap.pid_available = pid is not None
            if pid is None:
                cap.note = (
                    f"No {spec.pid_cmd} line in the M503 dump, which means "
                    f"{spec.pid_config_symbol} is not enabled in this build. "
                    f"This zone is running bang-bang and its PID cannot be "
                    f"read or set."
                )

    def _probe_setpoint_resolution(self, rep: FirmwareReport, frame) -> None:
        """
        Determine setpoint granularity empirically WITHOUT switching a heater on.

        The only way to learn the granularity is to command a fractional target
        and read it back. The trick that makes that safe is to choose a target
        *below* the current temperature: Marlin stores and reports the target, but
        its control loop has nothing to do, so the heater never fires. An earlier
        version commanded 30.5 °C, which briefly energised the bed during what the
        operator thinks is a read-only connect — not acceptable.

        Requires a zone with a healthy, reasonably warm sensor. If none qualifies
        we leave the assumed 1.0 °C, which is correct for Marlin anyway.
        """
        spec: ZoneSpec | None = None
        prior_target = 0.0
        best_reading = None
        for cand in ALL_ZONES:
            cap = rep.zone(cand.zone_id)
            if not cap.sensor_ok or cap.reading_c is None:
                continue
            if cap.reading_c < RESOLUTION_PROBE_MIN_C:
                continue
            tgt = (frame.target(cand.temp_key) if frame else None) or 0.0
            # Prefer an idle zone so we are not disturbing an active hold.
            if spec is None or (tgt == 0.0 and prior_target != 0.0):
                spec, prior_target, best_reading = cand, float(tgt), cap.reading_c
        if spec is None or best_reading is None:
            rep.setpoint_resolution_c = 1.0
            return
        if prior_target > 0:
            # Something is actively holding; do not perturb it just to measure
            # granularity. Marlin is integer-only, so assume that.
            rep.setpoint_resolution_c = 1.0
            return

        # Comfortably below the current temperature => zero heater demand.
        probe_c = float(int(best_reading)) - 5.0 + 0.5
        if probe_c <= 0.0:
            rep.setpoint_resolution_c = 1.0
            return
        try:
            t = self._link.send_and_wait(spec.set_target_raw(probe_c), timeout_s=6.0)
            if t.failed:
                rep.errors.append(
                    f"Could not probe setpoint resolution ({spec.set_cmd} was not "
                    f"acknowledged); assuming whole degrees."
                )
                return

            reported = None
            for _ in range(3):
                q = self._link.send_and_wait("M105", timeout_s=6.0)
                for line in q.lines:
                    f = parse_temp_line(line)
                    if f is not None and f.target(spec.temp_key) is not None:
                        reported = f.target(spec.temp_key)
                        break
                if reported is not None:
                    break
                time.sleep(0.1)

            if reported is None:
                return
            # The .5 surviving the round trip means fractional targets are kept.
            rep.setpoint_resolution_c = (
                0.1 if abs(reported - probe_c) < 0.05 else 1.0
            )
        finally:
            # Always put the target back where we found it.
            try:
                self._link.send_and_wait(
                    spec.set_target(prior_target), timeout_s=6.0
                )
            except Exception:
                logger.warning(
                    "failed to restore %s target after resolution probe",
                    spec.zone_id,
                )


def rescan_sensors(link, report: FirmwareReport) -> SensorRescan:
    """
    Re-read the sensors on a live board and update ``report`` in place.

    Why this is a legitimate, cheap operation rather than a reconnect: Marlin
    samples every configured thermistor ADC continuously and reports whatever it
    reads. Plug a thermistor in mid-session and the very next ``M105`` carries a
    real temperature — the *firmware* needs no reset, only our cached
    connect-time verdict does. So one ``M105`` is genuinely all it takes.

    Deliberately narrow: no heater is touched, no setpoint is written, and the
    setpoint-resolution probe is NOT repeated (that one commands a target, and a
    button labelled "re-check sensors" has no business doing that). PID
    availability is also left alone, being compile-time.

    Note the one case this cannot fix, which it reports separately: if the
    firmware was built with ``TEMP_SENSOR_x 0`` the field never appears in
    ``M105`` at all, and no amount of wiring will change that without a rebuild.
    """
    res = SensorRescan()

    frame = read_temp_frame(link)
    if frame is None:
        res.error = (
            "M105 returned nothing parsable. The board may be halted after a "
            "fault (in which case it must be power-cycled) or the link is down."
        )
        return res

    res.ok = True
    report.sensor_fields = frame.temperature_keys()

    for spec in ALL_ZONES:
        cap = report.zones.setdefault(
            spec.zone_id, ZoneCapability(zone_id=spec.zone_id)
        )
        was_ok = cap.sensor_ok
        fresh = evaluate_zone_sensor(spec, frame)

        # Merge sensor findings only; PID state is not ours to touch here.
        cap.sensor_present = fresh.sensor_present
        cap.reading_c = fresh.reading_c
        cap.sensor_fault = fresh.sensor_fault
        res.readings[spec.zone_id] = fresh.reading_c

        if not cap.sensor_present:
            res.not_configured.append(spec.zone_id)
        elif cap.sensor_ok and not was_ok:
            res.recovered.append(spec.zone_id)
        elif was_ok and not cap.sensor_ok:
            res.lost.append(spec.zone_id)
        elif not cap.sensor_ok:
            res.still_bad.append(spec.zone_id)

    logger.info(
        "[rescan] fields=%s recovered=%s lost=%s still_bad=%s unconfigured=%s",
        report.sensor_fields, res.recovered, res.lost,
        res.still_bad, res.not_configured,
    )
    return res
