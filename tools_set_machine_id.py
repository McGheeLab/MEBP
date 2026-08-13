"""
tools_set_machine_id.py — set THIS physical rig's machine identity.

Per-machine config (calibration, camera cal, mosaics, needle-bore cal, ...)
lives under ``config/hardware/<machine-id>/`` (see
``SupportClasses/MachineConfig.py``) so it can never collide with another
rig's data when this repo is shared across several machines via git.

The identity IS the active device profile's name, held in ``settings.json``
under ``device_profile.active``. This script sets that key.

Run from the project root:

    python tools_set_machine_id.py ME3B_01
    python tools_set_machine_id.py --show          # print the current id

⚠ Normally you do this in the GUI instead — Hardware Setup → Device, which
also writes the profile itself. This script is for headless/CI use. Do not
run it while the app is open: the in-memory Settings would overwrite it on
the next save.
"""

import argparse
import sys

from SupportClasses import MachineConfig


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("machine_id", nargs="?",
                         help="e.g. ME3B_01, ME3B_02")
    parser.add_argument("--show", action="store_true",
                         help="print the currently configured id and exit")
    args = parser.parse_args()

    if args.show or not args.machine_id:
        current = MachineConfig.machine_id()
        configured = MachineConfig.machine_id_is_configured()
        print(f"Current machine id: {current}"
              f"{'' if configured else ' (no device profile — fallback bucket)'}")
        print(f"Active device profile: "
              f"{MachineConfig.active_profile_name() or '(none)'}")
        print(f"Profiles directory: {MachineConfig.devices_dir()}")
        return 0

    try:
        MachineConfig.set_machine_id(args.machine_id)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Machine id set to '{MachineConfig.machine_id()}'.")
    print(f"Per-machine config folder: {MachineConfig.machine_config_dir()}")
    if not (MachineConfig.devices_dir() / f"{args.machine_id}.json").exists():
        print(f"NOTE: no device profile named '{args.machine_id}' exists yet in "
              f"{MachineConfig.devices_dir()} — create it on Hardware Setup → "
              f"Device, or the next Save there will overwrite this setting.",
              file=sys.stderr)
    print("Run tools_migrate_machine_config.py to move any existing "
          "config/hardware/*.json files into it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
