"""CommonPrintSettings.py — shared settings common to every workflow.

v7.5.x: A central collection (surfaced on the Workflows-mode "Common Print
Settings" page) of the settings that are common to all workflows. Two classes:

  * **Global params** — ``pump_settle_time_s`` (dwell after every discrete
    syringe move) and ``pump_prime_time_s`` (pre-flow prime). These are
    canonically owned by :class:`HardwareConfig` (the single source of truth the
    controller reads), so this model PROXIES them — ``get``/``set`` of a global
    key read/write the live HardwareConfig attribute. There is one value; no
    per-workflow override. (v7.5.x: the old ``pump_relief_percent`` global was
    retired — pressure relief / compliance is now an absolute µL PER PUMP, edited
    as a table on the Common Print Settings page and stored on the controller /
    device profile, not proxied here.)

  * **Promoted prep defaults** — the needle-prep knobs every prep workflow
    duplicates (service dip Z, prep flow rate, oil/buffer needle counts, wash
    cycles + amplitudes). These live HERE as shared DEFAULTS. Each workflow
    INHERITS the common default but can OVERRIDE it locally (the override lives
    in the workflow's own saved settings; this model only holds the default).

Pure Python (no Qt / GUI) per the SupportClasses convention — change
notification is a plain listener-callback list. ``gui/app.py`` owns the single
instance, registers a listener that persists + propagates, and fans it out to
the workflow pages.
"""

from __future__ import annotations

import logging
from typing import Callable

logger = logging.getLogger(__name__)


# Global keys — canonically stored on HardwareConfig (single source of truth).
GLOBAL_KEYS: tuple[str, ...] = (
    "pump_settle_time_s",
    "pump_prime_time_s",
    # v7.5.x: gentle-Z near the plate — one distance + one speed drive BOTH the
    # slow first-mm LIFT out of a print and the slow last-mm DESCENT back into
    # position, for every workflow (via safe_travel_to / ensure_retracted_to /
    # the discrete MOVE_Z handler). dist 0 disables (single-speed / legacy).
    "gentle_z_slow_dist_mm",
    "gentle_z_slow_speed_mm_s",
)
GLOBAL_DEFAULTS: dict[str, float] = {
    "pump_settle_time_s": 0.0,
    "pump_prime_time_s": 0.25,
    "gentle_z_slow_dist_mm": 1.0,
    "gentle_z_slow_speed_mm_s": 1.0,
}

# Promoted per-workflow prep knobs — shared DEFAULTS owned here. Keys match the
# field keys used by the workflow settings popouts (so a workflow's common_key
# == its own field key). Values are the canonical defaults (consistent across
# the spheroid / cell-targeting / cell-labeling / quick-print prep sections).
PROMOTED_DEFAULTS: dict[str, float] = {
    "service_z": 0.50,        # service dip Z above plate bottom (mm)
    "prep_rate": 1.0,         # prep/clean aspirate-dispense flow (µL/s)
    "oil_needles": 1.0,       # needles of oil dispensed-to-waste / aspirated
    "buffer_needles": 1.0,    # needles of buffer aspirated after the wash
    "wash_cycles": 3,         # dip-jiggle cycles at the wash well (int)
    "wash_z_amp": 0.5,        # wash Z jiggle amplitude (mm)
    "wash_xy_amp": 200.0,     # wash XY jiggle radius (µm)
    "wash_dwell": 0.3,        # settle between wash jiggles (s)
}
# Keys that are integers (everything else is a float).
_INT_KEYS = frozenset({"wash_cycles"})


class CommonPrintSettings:
    """The single common-settings model. Not thread-affine / not a QObject."""

    def __init__(self):
        self._promoted: dict[str, float] = dict(PROMOTED_DEFAULTS)
        self._hw_config = None
        self._listeners: list[Callable[["CommonPrintSettings", str], None]] = []

    # ── Hardware config (holds the global params) ──────────────────

    def set_hardware_config(self, hw_config) -> None:
        """Point the model at the live HardwareConfig (globals live there)."""
        self._hw_config = hw_config

    # ── Keys / introspection ───────────────────────────────────────

    @staticmethod
    def keys() -> list[str]:
        return list(GLOBAL_KEYS) + list(PROMOTED_DEFAULTS.keys())

    @staticmethod
    def is_global(key: str) -> bool:
        return key in GLOBAL_KEYS

    @staticmethod
    def default(key: str):
        if key in GLOBAL_KEYS:
            return GLOBAL_DEFAULTS[key]
        return PROMOTED_DEFAULTS.get(key)

    @staticmethod
    def _coerce(key: str, value):
        try:
            v = int(round(float(value))) if key in _INT_KEYS else float(value)
        except (TypeError, ValueError):
            return CommonPrintSettings.default(key)
        # All common settings are non-negative.
        return max(v, 0)

    # ── Get / set ──────────────────────────────────────────────────

    def get(self, key: str):
        if key in GLOBAL_KEYS:
            hw = self._hw_config
            if hw is None:
                return GLOBAL_DEFAULTS[key]
            try:
                raw = getattr(hw, key, GLOBAL_DEFAULTS[key])
                return max(0.0, float(raw if raw is not None else 0.0))
            except (TypeError, ValueError):
                return GLOBAL_DEFAULTS[key]
        return self._promoted.get(key, PROMOTED_DEFAULTS.get(key))

    def set(self, key: str, value, *, notify: bool = True) -> None:
        if key not in GLOBAL_KEYS and key not in PROMOTED_DEFAULTS:
            logger.warning("CommonPrintSettings.set: unknown key '%s'", key)
            return
        coerced = self._coerce(key, value)
        if key in GLOBAL_KEYS:
            if self._hw_config is not None:
                setattr(self._hw_config, key, float(coerced))
        else:
            self._promoted[key] = coerced
        if notify:
            self._notify(key)

    def values(self) -> dict:
        return {k: self.get(k) for k in self.keys()}

    # ── Persistence (promoted defaults only — globals persist via HW config) ──

    def promoted_dict(self) -> dict:
        return dict(self._promoted)

    def load_promoted(self, data: dict | None) -> None:
        if not isinstance(data, dict):
            return
        for key in PROMOTED_DEFAULTS:
            if key in data:
                self._promoted[key] = self._coerce(key, data[key])

    # ── Listeners ──────────────────────────────────────────────────

    def add_listener(
            self, cb: Callable[["CommonPrintSettings", str], None]) -> None:
        if cb not in self._listeners:
            self._listeners.append(cb)

    def _notify(self, key: str) -> None:
        for cb in list(self._listeners):
            try:
                cb(self, key)
            except Exception as exc:        # a bad listener must not break a set
                logger.debug("CommonPrintSettings listener failed: %s", exc)
