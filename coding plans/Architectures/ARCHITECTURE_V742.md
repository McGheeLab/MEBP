# MEBP v7.4.2 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.2 | May 2026**

> **Delta against [`ARCHITECTURE_V741.md`](ARCHITECTURE_V741.md)** and
> through it the rest of the v7.4.0 release deltas + `ARCHITECTURE_V737.md`.
> Sections not mentioned here are unchanged.

---

## 1. What's New in v7.4.2

The Device sub-page becomes the complete initial-machine-setup
workspace. Four new groups join the existing Device Profile / Safety
Limits / ZP Feedrates / Axis Flips:

### Connect Hardware

Buttons to bring up the XY stage and the ZP + pumps stage, plus
Disconnect buttons and live `StatusBadge` indicators that reflect the
controller's actual connection state. Connection badges update every
poll tick via `on_status_update()` so they stay accurate even if the
hardware drops mid-session.

### Per-axis Jog + Record Limits

One row per axis (X, Y, Z, P1, P2, P3). Each row has a position
readout, six step buttons (−Coarse / −Med / −Fine / +Fine / +Med /
+Coarse) and two record buttons: **Set as Min** and **Set as Max**.
Recording copies the current position straight into the appropriate
Safety Limits spinbox above — turning "jog to where the mechanical
endstop kicks" into a single click.

Step sizes:

| Axis | Fine | Medium | Coarse |
|------|------|--------|--------|
| X, Y | 10 µm | 100 µm | 1000 µm |
| Z, P1, P2, P3 | 0.01 mm | 0.1 mm | 1 mm |

### Axis Mapping

Four dropdowns: each of the four logical axes (Z, P1, P2, P3) can be
remapped to any physical Marlin axis (X, Y, Z, E). The previous
hard-coded mapping `{Z:X, P1:Y, P2:Z, P3:E}` is now just the default —
users with non-standard wiring (e.g. X→P1, Y→P2, Z→Z, E→P3) can set
their actual configuration on this page.

### Stepper Calibration

Per-logical-axis `steps_per_mm`. Workflow:

1. Pick the axis (Z / P1 / P2 / P3)
2. Enter the commanded distance in mm
3. Click **Command Move** — the controller sends the move
4. Measure the actual physical displacement with a caliper or scale
5. Enter the measured distance
6. Click **Calculate & Send M92** — `new = current × (commanded / measured)`,
   M92 is sent to Marlin to take effect immediately, and the value is
   saved into the device profile

The display strip above the workflow shows the current per-axis
steps/mm so you can see all four values at once.

---

## 2. Backend Changes

### `SupportClasses/ZPStage.py`

```python
class ZPStageManager:
    DEFAULT_STEPS_PER_MM_DICT = {"Z": 5069, "P1": 5069, "P2": -5069, "P3": 5069}

    def __init__(self, ..., steps_per_mm=DEFAULT_STEPS_PER_MM,
                 axis_map=None):
        # steps_per_mm: int (back-compat) OR dict[str, int]
        # axis_map: dict[str, str] OR None (uses module AXIS_MAP)
        ...

    def set_axis_map(self, axis_map: dict[str, str]) -> None: ...
    def set_steps_per_mm(self, steps: dict[str, int],
                         persist: bool = True) -> None: ...
    def _build_m92_command(self) -> str: ...
```

The module-level `AXIS_MAP` constant remains as the default but is no
longer the single source of truth — each `ZPStageManager` instance
holds its own mapping in `self.axis_map`.

### `SupportClasses/StageController.py`

```python
def _axis_letter(zp_stage, logical: str) -> str | None:
    """Resolve logical → physical via zp_stage.axis_map, falling
    back to module default."""

class StageController:
    def apply_device_settings(self, axis_map=None,
                              steps_per_mm=None,
                              persist_steps=False) -> None:
        """Cache values; push live if zp_stage is connected."""
```

Three internal `AXIS_MAP["Z"]` / `AXIS_MAP.get(pump)` call sites are
routed through `_axis_letter()`. Pending settings are applied
automatically when the ZP stage connects.

### `SupportClasses/PrintManager.py` + `SupportClasses/VelocityExecutor.py`

Five remaining `AXIS_MAP.get()` sites were reading the module-level
constant. All converted to read from `ctrl.zp_stage.axis_map` first,
falling back to the module default. Without this, the user's custom
mapping would have affected only the jog UI — print execution would
silently send commands to the wrong physical axes.

---

## 3. Device Profile + Settings Schema Additions

```jsonc
// settings.json (defaults)
"device_profile": {
    "active": null,
    "axis_map": {                // v7.4.2
        "Z": "X", "P1": "Y", "P2": "Z", "P3": "E"
    },
    "steps_per_mm": {            // v7.4.2 — dict, not int
        "Z": 5069, "P1": 5069, "P2": -5069, "P3": 5069
    }
}
```

```jsonc
// config/hardware/devices/Standard.json
{
    ...
    "axis_map": { "Z": "X", "P1": "Y", "P2": "Z", "P3": "E" },
    "steps_per_mm": { "Z": 5069, "P1": 5069, "P2": -5069, "P3": 5069 }
}
```

`DeviceProfile.from_settings()` reads these new fields, and
`apply_to_settings()` writes them back so Load Profile / Save Profile
both carry the mapping and calibration along with the safety / feedrate
/ axis-flip sections.

---

## 4. Connection / Apply Flow

```
App startup
    │
    ├── MainWindow.__init__
    │       │
    │       ├── controller.apply_device_settings(axis_map, steps_per_mm)
    │       │       └── cached on controller as _pending_*
    │       │
    │       └── Stage panel set_settings() loads UI
    │
    └── User: Connect Hardware → Connect ZP
            │
            └── controller.connect_zp() → ZPStageManager instantiated
                    │
                    └── Pending axis_map + steps_per_mm pushed
                            │
                            ├── zp_stage.set_axis_map(...)
                            └── zp_stage.set_steps_per_mm(..., persist=True)
                                    └── M92 sent to Marlin

User: edits Axis Mapping or runs Stepper Calibration → clicks Apply
    │
    └── StageHardwarePanel._apply()
            │
            ├── settings.set("device_profile.axis_map", new_map)
            ├── settings.set("device_profile.steps_per_mm", new_steps)  (from Cal workflow)
            ├── controller.zp_stage.set_axis_map(new_map)
            └── controller.zp_stage.set_steps_per_mm(new_steps, persist=True)
                    └── M92 sent immediately
```

---

## 5. Updated Core Design Principles

(Additions to the v7.4.1 list.)

20. **Logical axes are user-configurable, not hardcoded.** The Z, P1,
    P2, P3 → X, Y, Z, E mapping is per-machine. The module-level
    `AXIS_MAP` is now a default value, not a single source of truth.
    Any code path that needs to translate logical → physical must read
    from `controller.zp_stage.axis_map` (with the module default as
    fallback). This was retrofitted into `StageController`,
    `PrintManager`, and `VelocityExecutor` in v7.4.2 — new code should
    follow the same pattern.

21. **Per-axis stepper calibration.** `steps_per_mm` is `dict[str, int]`
    keyed by logical axis. Sign indicates direction (negative inverts).
    Calibration via the Device sub-page sends M92 to Marlin immediately
    and persists the value in the device profile.

---

## 6. Sections Unchanged

All earlier architecture sections not mentioned above apply verbatim.
