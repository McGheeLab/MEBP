"""
zones.py — the two heated zones and their Marlin command families.

This is the ONE place the differences between "bed" and "hotend" live. Every
other module is zone-generic and asks a :class:`ZoneSpec` how to phrase a
command, so adding a third zone (or renaming one) touches only this file.

    Zone A "bed"    film heater on HB  + thermistor on THB
                    M140 / M190, PID M304, autotune M303 E-1, M105 field 'B'
    Zone B "hotend" heater      on HE0 + thermistor on THO
                    M104 / M109, PID M301, autotune M303 E0,  M105 field 'T'

Naming caution: Marlin calls these "bed" and "hotend" because that is what a
3D printer uses them for. Physically they are just two independent
PID-controlled heater channels, which is exactly what an incubator needs. The
``title``/``blurb`` fields carry the incubator-facing names for the UI.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ZoneSpec:
    """Static description of one heated zone."""

    zone_id: str            # "bed" | "hotend" — stable key for configs/logs
    title: str              # operator-facing name
    blurb: str              # one-line description of what it heats
    temp_key: str           # M105 field carrying this zone's temperature
    power_key: str          # M105 field carrying this zone's PWM duty
    set_cmd: str            # non-blocking set-target command word
    wait_cmd: str           # blocking set-and-wait command word
    pid_cmd: str            # runtime PID set/readback command word
    autotune_extruder: int  # M303 E value (-1 = bed)
    heater_connector: str   # silkscreen label, for the UI + docs
    sensor_connector: str   # silkscreen label, for the UI + docs
    #: Marlin config symbol that must be enabled for this zone to have PID.
    pid_config_symbol: str
    #: Prefix of the autotune result lines for this zone
    #: ("bed" -> DEFAULT_bedKp, "hotend" -> DEFAULT_Kp).
    autotune_result_heater: str

    # ── command builders ────────────────────────────────────────────

    def set_target(self, celsius: float) -> str:
        """
        Non-blocking set-target. NOTE Marlin truncates to whole degrees
        (targets are stored as ``celsius_t`` = ``int16_t``), so we format as an
        integer to make that explicit rather than sending a lie like S37.5.
        """
        return f"{self.set_cmd} S{int(round(celsius))}"

    def set_target_raw(self, celsius: float) -> str:
        """
        Set-target preserving a fractional value. Only used by the
        setpoint-resolution probe, which deliberately sends e.g. S37.5 to
        discover whether the firmware quantizes it.
        """
        return f"{self.set_cmd} S{celsius:g}"

    def wait_for_target(self, celsius: float) -> str:
        """Blocking set-and-wait (board-side). Cancel with M108."""
        return f"{self.wait_cmd} S{int(round(celsius))}"

    def heater_off(self) -> str:
        return f"{self.set_cmd} S0"

    def set_pid(self, kp: float, ki: float, kd: float) -> str:
        return f"{self.pid_cmd} P{kp:.2f} I{ki:.2f} D{kd:.2f}"

    def autotune(self, target_c: float, cycles: int, apply_result: bool) -> str:
        from .marlin_gcode import build_autotune
        return build_autotune(
            extruder=self.autotune_extruder,
            target_c=target_c,
            cycles=cycles,
            apply_result=apply_result,
        )


BED_ZONE = ZoneSpec(
    zone_id="bed",
    title="Zone A — water block",
    blurb="Film heater bonded to the water-filled aluminium block",
    temp_key="B",
    power_key="B@",
    set_cmd="M140",
    wait_cmd="M190",
    pid_cmd="M304",
    autotune_extruder=-1,
    heater_connector="HB",
    sensor_connector="THB",
    pid_config_symbol="PIDTEMPBED",
    autotune_result_heater="bed",
)

HOTEND_ZONE = ZoneSpec(
    zone_id="hotend",
    title="Zone B — stage area",
    blurb="Second heater elsewhere on the incubator stage",
    temp_key="T",
    power_key="@",
    set_cmd="M104",
    wait_cmd="M109",
    pid_cmd="M301",
    autotune_extruder=0,
    heater_connector="HE0",
    sensor_connector="THO",
    pid_config_symbol="PIDTEMP",
    autotune_result_heater="hotend",
)

#: Canonical ordering used by the UI and by iteration everywhere.
ALL_ZONES: tuple[ZoneSpec, ...] = (BED_ZONE, HOTEND_ZONE)

_BY_ID = {z.zone_id: z for z in ALL_ZONES}


def zone_by_id(zone_id: str) -> ZoneSpec:
    """Look up a zone by its stable id, raising a clear error if unknown."""
    try:
        return _BY_ID[zone_id]
    except KeyError:
        raise KeyError(
            f"unknown zone {zone_id!r}; known zones: {sorted(_BY_ID)}"
        ) from None


def zone_for_heater_id(heater_id: str | None) -> ZoneSpec | None:
    """
    Map a Marlin ``Heater_ID`` from a fault line onto a zone.

    Marlin reports the bed as ``Bed`` (or ``-1``) and hotend 0 as ``E0``
    (or ``0``). Returns ``None`` when it cannot be attributed.
    """
    if not heater_id:
        return None
    h = str(heater_id).strip().lower()
    if h in ("bed", "b", "-1"):
        return BED_ZONE
    if h in ("e0", "0", "e", "hotend", "h0"):
        return HOTEND_ZONE
    return None
