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
    """

    profile_name: str = "Untitled Device"
    notes: str = ""
    safety_limits: dict = field(default_factory=dict)
    zp_stage: dict = field(default_factory=dict)
    axis_flip: dict = field(default_factory=dict)

    # ── JSON I/O ─────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "_format_version": "v7.4.1",
            "_description": (
                "Device profile — physical-machine settings (safety envelope, "
                "motor feedrates, axis direction). Edit on Hardware Setup → Stage."
            ),
            "profile_name": self.profile_name,
            "notes": self.notes,
            "safety_limits": self.safety_limits,
            "zp_stage": self.zp_stage,
            "axis_flip": self.axis_flip,
        }

    @classmethod
    def from_dict(cls, data: dict) -> DeviceProfile:
        return cls(
            profile_name=data.get("profile_name", "Untitled Device"),
            notes=data.get("notes", ""),
            safety_limits=data.get("safety_limits", {}) or {},
            zp_stage=data.get("zp_stage", {}) or {},
            axis_flip=data.get("axis_flip", {}) or {},
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
        )

    def apply_to_settings(self, settings) -> None:
        """Copy this profile's values into the live Settings instance.

        Writes to the same top-level keys the rest of the app reads
        (``safety_limits.*``, ``zp_stage.*``, ``axis_flip.*``). Caller
        is responsible for triggering any UI refresh + ``settings.save()``.
        """
        if self.safety_limits:
            settings.set_section("safety_limits", self.safety_limits)
        if self.zp_stage:
            settings.set_section("zp_stage", self.zp_stage)
        if self.axis_flip:
            settings.set_section("axis_flip", self.axis_flip)


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
