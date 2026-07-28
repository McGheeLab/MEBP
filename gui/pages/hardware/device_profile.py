"""
device_profile.py — Device profile load/save for the Stage sub-page (v7.4.1).

A "device profile" holds settings that describe the *physical machine* —
safety envelope, motor feedrates, axis direction flips. Each lab machine
gets its own profile (`config/hardware/devices/<name>.json`) so the
same settings travel with the hardware regardless of which experiment-
level HardwareConfig is loaded.

This module owns:
  * The :class:`DeviceProfile` dataclass (round-trippable to JSON).
  * Helpers to load / save / list profiles in
    ``config/hardware/devices/``.
  * Bridge methods to / from a :class:`Settings` instance, since the
    rest of the app reads these values from settings.json at known
    top-level keys (``safety_limits.*``, ``zp_stage.*``, ``axis_flip.*``).
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# Standard location for device profile JSONs. Ships with two presets
# (Standard.json, Conservative.json); users add more via the Stage UI.
DEVICES_DIR = (
    Path(__file__).resolve().parent.parent.parent.parent
    / "config" / "hardware" / "devices"
)


@dataclass
class DeviceProfile:
    """A physical-machine settings bundle.

    Round-trips to JSON in ``config/hardware/devices/<profile_name>.json``.
    Active profile name is persisted via ``settings.set("device_profile.active", ...)``
    so the same profile re-loads across launches.

    v7.4.2: Carries ``axis_map`` (logical→physical axis mapping) and
    per-axis ``steps_per_mm`` (stepper calibration). These define how
    MEBP G-code translates to Marlin axes for a specific machine.
    """

    profile_name: str = "Untitled Device"
    notes: str = ""
    safety_limits: dict = field(default_factory=dict)
    zp_stage: dict = field(default_factory=dict)
    axis_flip: dict = field(default_factory=dict)
    # v7.4.2: per-machine wiring of logical axes (Z, P1, P2, P3) to
    # physical Marlin axes (X, Y, Z, E), plus per-axis stepper calibration.
    axis_map: dict = field(default_factory=dict)
    steps_per_mm: dict = field(default_factory=dict)
    # v7.4.2: per-axis max feedrate discovered via Stepper Calibration
    # feedrate experimentation. Reference value only — global safety
    # clamping still uses safety_limits.max_z_feedrate / max_pump_feedrate.
    per_axis_max_feedrate: dict = field(default_factory=dict)
    # v7.4.2 hotfix: per-logical-axis max acceleration (mm/s²) sent
    # to Marlin via M201; XY stage motion calibration (velocity %,
    # acceleration 1–100 SAS, optional jerk 1–100 SCS).
    per_axis_max_accel: dict = field(default_factory=dict)
    xy_velocity_pct: Optional[int] = None
    xy_acceleration: Optional[int] = None
    xy_jerk: Optional[int] = None
    # v7.5.x: which XY controller this machine uses. A controller-protocol JSON
    # path (e.g. "config/controllers/mac5000.json"), "auto" (detect), or None
    # (inherit the global controller.controller_json setting). Persisted into the
    # SAME global key on apply, so selecting a machine profile picks its stage
    # (precedence: device profile → global setting → auto-detect at connect).
    xy_controller_json: Optional[str] = None
    # v7.5.x: measured/known true XY top speed (µm/s), family-neutral. Home for
    # the "Measure top speed" result; re-applied at connect via
    # StageController → XYStage.set_max_speed_um_s so mm/s conversions are
    # correct for both Prior (SMS %) and Ludl (absolute) stages.
    xy_max_speed_um_s: Optional[float] = None
    # v7.5.x: unified Z convention (see MEBP_v75x_Z_AXIS_CONVENTION_RETHINK).
    # The Z setup derives the user-facing up-direction from the captured
    # bottom/top extremes; +1 = user_Z grows with raw Z, -1 = user_Z grows as
    # raw Z shrinks (ME3B V1). None ⇒ fall back to the module ZDIR.
    z_up_sign: Optional[float] = None
    # v7.5.x: per-machine well-plate orientation. True = plate is mounted 180°
    # to the stage axes (ME3B V1: stage 0,0 bottom-right, well A1 top-left), so
    # plate displays flip 180° and the plate-local→stage geometry sign is
    # (-1, -1). None ⇒ fall back to StageController.DEFAULT_PLATE_FLIP_180.
    plate_flip_180: Optional[bool] = None
    # v7.5.x: needle-tip-camera Z fiducial (user-frame mm = height above the
    # bottom datum), captured during XY needle calibration. Stable across
    # power cycles as long as the bottom datum is re-established by the Z setup.
    needle_cam_z: Optional[float] = None
    # v7.5.x: standard mechanical offsets (mm BELOW the needle-cam fiducial) to
    # the plate features — used to PRE-FILL plate Z reference guesses. Keys:
    # "top", "bottom", "safe". Max/Replace Z stay manual (not here).
    plate_z_offsets: dict = field(default_factory=dict)
    # v7.5.x: approximate needle location — the centered needle position in the
    # ABSOLUTE Prior stage frame ([x_um, y_um]). The side cameras are bolted to
    # the frame, so this is a stable per-machine datum; the Needle Location tab's
    # "Go to needle location" quick-move drives here. None ⇒ not captured.
    needle_loc_xy_um: Optional[list] = None
    # v7.5.x: per-pump plunger calibration (mirror of the Z setup). Maps pump
    # ("P1"/"P2"/"P3") → {"raw_dispensed", "raw_aspirated", "aspirate_sign",
    # "capacity_uL"} captured by the "Set Dispensed / Set Aspirated" flow.
    # ZERO = plunger all the way IN (empty); MAX = all the way OUT (full). The
    # dispense/aspirate DIRECTION is derived from these extremes and owned by the
    # calibration. The soft-limit envelope itself lives in safety_limits.p*.
    pump_setup: dict = field(default_factory=dict)
    # v7.5.x: per-pump compliance / "pressure relief" value in µL (mirror of
    # pump_setup). Maps pump ("P1"/"P2"/"P3") → µL = half the aspirate-back
    # volume measured by the Needle Location compliance calibration. Used by the
    # backlash-compensation engine (take-up on reversal + unload on stop). 0 /
    # absent ⇒ no compensation for that pump.
    pump_compliance_uL: dict = field(default_factory=dict)
    # v7.5.x: global enable for backlash / pressure compensation at every pump
    # start/stop. Toggled from the pump jog panel. None ⇒ default False (off).
    backlash_comp_enabled: Optional[bool] = None

    # ── JSON I/O ─────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "_format_version": "v7.4.2",
            "_description": (
                "Device profile — physical-machine settings (safety envelope, "
                "motor feedrates, axis direction, axis mapping, stepper "
                "calibration). Edit on Hardware Setup → Device."
            ),
            "profile_name": self.profile_name,
            "notes": self.notes,
            "safety_limits": self.safety_limits,
            "zp_stage": self.zp_stage,
            "axis_flip": self.axis_flip,
            "axis_map": self.axis_map,
            "steps_per_mm": self.steps_per_mm,
            "per_axis_max_feedrate": self.per_axis_max_feedrate,
            "per_axis_max_accel": self.per_axis_max_accel,
            "xy_velocity_pct": self.xy_velocity_pct,
            "xy_acceleration": self.xy_acceleration,
            "xy_jerk": self.xy_jerk,
            "xy_controller_json": self.xy_controller_json,
            "xy_max_speed_um_s": self.xy_max_speed_um_s,
            "z_up_sign": self.z_up_sign,
            "plate_flip_180": self.plate_flip_180,
            "needle_cam_z": self.needle_cam_z,
            "plate_z_offsets": self.plate_z_offsets,
            "needle_loc_xy_um": self.needle_loc_xy_um,
            "pump_setup": self.pump_setup,
            "pump_compliance_uL": self.pump_compliance_uL,
            "backlash_comp_enabled": self.backlash_comp_enabled,
        }

    @classmethod
    def from_dict(cls, data: dict) -> DeviceProfile:
        return cls(
            profile_name=data.get("profile_name", "Untitled Device"),
            notes=data.get("notes", ""),
            safety_limits=data.get("safety_limits", {}) or {},
            zp_stage=data.get("zp_stage", {}) or {},
            axis_flip=data.get("axis_flip", {}) or {},
            axis_map=data.get("axis_map", {}) or {},
            steps_per_mm=data.get("steps_per_mm", {}) or {},
            per_axis_max_feedrate=data.get("per_axis_max_feedrate", {}) or {},
            per_axis_max_accel=data.get("per_axis_max_accel", {}) or {},
            xy_velocity_pct=data.get("xy_velocity_pct"),
            xy_acceleration=data.get("xy_acceleration"),
            xy_jerk=data.get("xy_jerk"),
            xy_controller_json=data.get("xy_controller_json"),
            xy_max_speed_um_s=data.get("xy_max_speed_um_s"),
            z_up_sign=data.get("z_up_sign"),
            plate_flip_180=data.get("plate_flip_180"),
            needle_cam_z=data.get("needle_cam_z"),
            plate_z_offsets=data.get("plate_z_offsets", {}) or {},
            needle_loc_xy_um=data.get("needle_loc_xy_um"),
            pump_setup=data.get("pump_setup", {}) or {},
            pump_compliance_uL=data.get("pump_compliance_uL", {}) or {},
            backlash_comp_enabled=data.get("backlash_comp_enabled"),
        )

    def save(self, path: Path | None = None) -> Path:
        """Save profile to JSON. Default location is DEVICES_DIR/<name>.json."""
        if path is None:
            DEVICES_DIR.mkdir(parents=True, exist_ok=True)
            safe_name = self.profile_name.strip() or "Untitled"
            # Strip filename-unfriendly characters
            for ch in "/\\:*?\"<>|":
                safe_name = safe_name.replace(ch, "_")
            path = DEVICES_DIR / f"{safe_name}.json"
        path.write_text(json.dumps(self.to_dict(), indent=2))
        logger.info(f"Device profile saved: {path}")
        return path

    @classmethod
    def load(cls, path: Path) -> DeviceProfile:
        data = json.loads(path.read_text())
        return cls.from_dict(data)

    # ── Settings bridge ──────────────────────────────────────────

    @classmethod
    def from_settings(cls, settings, name: str = "Current") -> DeviceProfile:
        """Build a DeviceProfile from the current Settings instance."""
        return cls(
            profile_name=name,
            notes="",
            safety_limits=settings.get_section("safety_limits") or {},
            zp_stage=settings.get_section("zp_stage") or {},
            axis_flip=settings.get_section("axis_flip") or {},
            axis_map=settings.get("device_profile.axis_map") or {},
            steps_per_mm=settings.get("device_profile.steps_per_mm") or {},
            per_axis_max_feedrate=settings.get(
                "device_profile.per_axis_max_feedrate") or {},
            per_axis_max_accel=settings.get(
                "device_profile.per_axis_max_accel") or {},
            xy_velocity_pct=settings.get("device_profile.xy_velocity_pct"),
            xy_acceleration=settings.get("device_profile.xy_acceleration"),
            xy_jerk=settings.get("device_profile.xy_jerk"),
            xy_controller_json=settings.get("controller.controller_json"),
            xy_max_speed_um_s=settings.get("device_profile.xy_max_speed_um_s"),
            z_up_sign=settings.get("device_profile.z_up_sign"),
            plate_flip_180=settings.get("device_profile.plate_flip_180"),
            needle_cam_z=settings.get("device_profile.needle_cam_z"),
            plate_z_offsets=settings.get("device_profile.plate_z_offsets") or {},
            needle_loc_xy_um=settings.get("device_profile.needle_loc_xy_um"),
            pump_setup=settings.get("device_profile.pump_setup") or {},
            pump_compliance_uL=settings.get(
                "device_profile.pump_compliance_uL") or {},
            backlash_comp_enabled=settings.get(
                "device_profile.backlash_comp_enabled"),
        )

    def apply_to_settings(self, settings) -> None:
        """Copy this profile's values into the live Settings instance.

        Writes to the same top-level keys the rest of the app reads
        (``safety_limits.*``, ``zp_stage.*``, ``axis_flip.*``,
        ``device_profile.axis_map``, ``device_profile.steps_per_mm``).
        Caller is responsible for triggering any UI refresh +
        ``settings.save()``.
        """
        if self.safety_limits:
            settings.set_section("safety_limits", self.safety_limits)
        if self.zp_stage:
            settings.set_section("zp_stage", self.zp_stage)
        if self.axis_flip:
            settings.set_section("axis_flip", self.axis_flip)
        if self.axis_map:
            settings.set("device_profile.axis_map", self.axis_map)
        if self.steps_per_mm:
            settings.set("device_profile.steps_per_mm", self.steps_per_mm)
        if self.per_axis_max_feedrate:
            settings.set("device_profile.per_axis_max_feedrate",
                         self.per_axis_max_feedrate)
        if self.per_axis_max_accel:
            settings.set("device_profile.per_axis_max_accel",
                         self.per_axis_max_accel)
        if self.xy_velocity_pct is not None:
            settings.set("device_profile.xy_velocity_pct", self.xy_velocity_pct)
        if self.xy_acceleration is not None:
            settings.set("device_profile.xy_acceleration", self.xy_acceleration)
        if self.xy_jerk is not None:
            settings.set("device_profile.xy_jerk", self.xy_jerk)
        # v7.5.x: write the per-machine XY controller into the SAME global key
        # the Settings-page dropdown uses, so a loaded profile overrides the
        # global default. None ⇒ leave the global setting untouched (inherit).
        if self.xy_controller_json is not None:
            settings.set("controller.controller_json", self.xy_controller_json)
        if self.xy_max_speed_um_s is not None:
            settings.set("device_profile.xy_max_speed_um_s", self.xy_max_speed_um_s)
        if self.z_up_sign is not None:
            settings.set("device_profile.z_up_sign", self.z_up_sign)
        if self.plate_flip_180 is not None:
            settings.set("device_profile.plate_flip_180", self.plate_flip_180)
        if self.needle_cam_z is not None:
            settings.set("device_profile.needle_cam_z", self.needle_cam_z)
        if self.plate_z_offsets:
            settings.set("device_profile.plate_z_offsets", self.plate_z_offsets)
        if self.needle_loc_xy_um is not None:
            settings.set("device_profile.needle_loc_xy_um",
                         self.needle_loc_xy_um)
        if self.pump_setup:
            settings.set("device_profile.pump_setup", self.pump_setup)
        if self.pump_compliance_uL:
            settings.set("device_profile.pump_compliance_uL",
                         self.pump_compliance_uL)
        if self.backlash_comp_enabled is not None:
            settings.set("device_profile.backlash_comp_enabled",
                         self.backlash_comp_enabled)


# ── Module-level helpers ─────────────────────────────────────────

def list_profiles() -> list[tuple[str, Path]]:
    """Return [(profile_name, path), ...] for every JSON in DEVICES_DIR.

    Sorted alphabetically by profile_name; falls back to filename stem
    if the JSON doesn't declare profile_name.
    """
    if not DEVICES_DIR.is_dir():
        return []
    results: list[tuple[str, Path]] = []
    for jf in sorted(DEVICES_DIR.glob("*.json")):
        try:
            data = json.loads(jf.read_text())
            name = data.get("profile_name", jf.stem)
        except Exception as e:
            logger.debug(f"Skipping malformed device profile {jf.name}: {e}")
            continue
        results.append((name, jf))
    return results


def delete_profile(path: Path) -> bool:
    """Delete a device profile file. Returns True on success."""
    try:
        path.unlink()
        logger.info(f"Device profile deleted: {path}")
        return True
    except Exception as e:
        logger.warning(f"Failed to delete device profile {path}: {e}")
        return False
