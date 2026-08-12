"""
tools.incubator — standalone two-zone incubator heater bring-up tool.

This package is DELIBERATELY self-contained and is NOT part of the MEBP
application. It opens its own serial connection to the ZP stage board
(BigTreeTech SKR Mini E3 V3 running Marlin) and drives the two heater
outputs as an incubator:

    Zone A ("bed")     film heater on HB  + thermistor on THB
    Zone B ("hotend")  heater      on HE0 + thermistor on THO

Nothing here imports ``SupportClasses.ZPStage`` or any GUI page from the
main app. The only shared code is read-only reuse of hardware-agnostic
helpers (``SupportClasses.SerialUtils``) and the theme/scaling modules
(``gui.styles``, ``gui.scaling``).

Run it with::

    python tools/incubator/run.py

See ``README.md`` for wiring cautions and ``FIRMWARE_NOTES.md`` for the
Marlin configuration this rig requires.
"""

__all__ = []
