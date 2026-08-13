"""
SupportClasses.incubator — two-zone incubator heater control (v7.18).

Integrated from the standalone bring-up tool (formerly ``tools/incubator/``).
The heaters are driven by a Marlin board — on this rig the SAME SKR Mini E3 V3
that runs Z + pumps (Zone A = bed outputs HB/THB on the water block, Zone B =
hotend HE0/THO on the stage area) — so the default transport RIDES the app's
live ZP connection (:mod:`.zp_shared_link`); a dedicated serial port and a
thermal simulator (:mod:`.fake_marlin`) are the other two transports.

Module map (framework-agnostic, no Qt anywhere in this package):

    zones.py           the two zones and their Marlin command families
    marlin_gcode.py    pure line parsing / command building, no I/O
    sensors.py         multi-channel sensor model (SensorHub)
    marlin_link.py     dedicated-port serial transport + ok handshake
    zp_shared_link.py  transport riding the live ZPStageManager (v7.18)
    fake_marlin.py     simulated board + first-order thermal models
    probe.py           connect-time firmware capability detection
    controller.py      the facade every GUI surface drives
    stability.py       derived metrics (rate/ETA/ripple/steady duty/tau)
    ramp.py            watchdog-safe staircase setpoint ramp
    calibration.py     host-side sensor correction (per-machine store)
    safety.py          setpoint ceiling, fault latch, divergence check
    telemetry.py       JSONL run logger (logs/incubator/)
    device_config.py   port hints + Marlin auto-detect (exclusion-aware)
    config_store.py    per-machine settings (config/hardware/incubator.json)
    service.py         get_incubator()/peek_incubator()/shutdown_incubator()

Deliberately NO eager imports here: the GUI lazy-imports what it needs, and
the stage-connect port-exclusion logic must be able to ``peek`` without
constructing anything.
"""
