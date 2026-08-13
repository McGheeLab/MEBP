"""
tools_migrate_machine_config.py — one-shot move of flat config/hardware/*
files into their new per-machine / shared homes.

Background: config/hardware/ used to be one flat directory holding a mix of
per-machine calibration (camera cal, taught plate positions, mosaics, ...) and
shared/portable catalogs (needle/plate/well/target types, plate/rosette
designs, swappable hardware Setup*.json files), separated only by a
hand-maintained .gitignore list. That list fell behind, so several per-machine
stores were being tracked in git — the actual cause of the device-specific
pull conflicts between rigs. See SupportClasses/MachineConfig.py.

Every store now resolves its own path through
SupportClasses.MachineConfig.resolve_machine_path()/resolve_shared_path(),
which auto-migrates its own file/folder the first time it's touched — so this
script isn't strictly required for correctness (just importing each store
module, or simply running the app, does the same thing incrementally). What
it buys is a DETERMINISTIC one-shot move: run it once per physical machine,
then `git status` shows every old tracked path as deleted in one pass instead
of trickling in as different pages happen to get visited, so `git add -A` /
`git commit` cleanly removes them from tracking in one commit.

Run from the project root, after setting this machine's id:

    python tools_set_machine_id.py ME3B_01
    python tools_migrate_machine_config.py

Safe to run more than once (every migration is idempotent — nothing left at
the old flat location the second time, so nothing moves).
"""

import sys

from SupportClasses import MachineConfig


# Importing each of these triggers its own resolve_machine_path/
# resolve_shared_path call at module load time, migrating that store's file
# or folder out of the legacy flat config/hardware/ root.
_PER_MACHINE_MODULES = [
    "SupportClasses.CalibrationStatusStore",
    "SupportClasses.CameraCalibrationStore",
    "SupportClasses.MicroscopeConfigStore",
    "SupportClasses.FluorescenceMosaicStore",
    "SupportClasses.CalibrationSnapshotStore",
    "SupportClasses.MosaicStore",
    "SupportClasses.MosaicAlignmentStore",
    "SupportClasses.ObjectiveCalibration",
    "SupportClasses.ReanchorFeatureStore",
    "SupportClasses.PrintTimingCalibrationStore",
    "SupportClasses.NeedleBoreCalibrationStore",
    "SupportClasses.NeedleFocusTemplateStore",
    "SupportClasses.PlateFocusDatumStore",
    "SupportClasses.PlateLevelSiteStore",
    "SupportClasses.SpheroidSinkCalibrationStore",
    "SupportClasses.SpheroidTrainingStore",
    "SupportClasses.WellTrainingStore",
    "SupportClasses.PlateTemplateStore",
    "SupportClasses.LabLinkConfigStore",
    "gui.pages.hardware.device_profile",
]

_SHARED_MODULES = [
    "SupportClasses.PhysicalModels",
    "SupportClasses.NeedleTypeStore",
    "SupportClasses.PlateTypeStore",
    "SupportClasses.WellTypeStore",
    "SupportClasses.TargetTypeStore",
    "SupportClasses.PlateDocumentStore",
    "SupportClasses.WellPlate",
    "SupportClasses.PlateDesign",
]


def _import_all(names: list) -> list:
    """Import each module (triggering its own path migration). Returns the
    names that FAILED — the caller must not sweep leftovers if any did, since
    an unclaimed per-machine file would then be misfiled as shared."""
    import importlib
    failed = []
    for name in names:
        try:
            importlib.import_module(name)
        except Exception as exc:   # pragma: no cover - diagnostic only
            print(f"  ! could not import {name}: {exc}", file=sys.stderr)
            failed.append(name)
    return failed


def main() -> int:
    machine = MachineConfig.machine_id()
    configured = MachineConfig.machine_id_is_configured()
    print(f"Machine id: {machine}"
          f"{'' if configured else ' (NOT configured — run tools_set_machine_id.py first)'}")
    if not configured:
        print("Refusing to migrate into the 'unassigned' bucket — "
              "set a machine id first: python tools_set_machine_id.py <ID>",
              file=sys.stderr)
        return 1

    print("Migrating per-machine stores...")
    failed = _import_all(_PER_MACHINE_MODULES)
    # Tucsen SDK per-camera XMLs: _sdk_config_dir() only migrates when
    # actually called (e.g. by opening a Tucsen camera), so trigger it here.
    MachineConfig.resolve_machine_path("tucsen")

    print("Migrating shared/portable catalogs...")
    failed += _import_all(_SHARED_MODULES)

    if failed:
        print(f"\n⚠ {len(failed)} module(s) failed to import, so their config "
              "files were not claimed. Reporting leftovers WITHOUT moving "
              "them — a per-machine file swept into the shared folder would "
              "be published to every rig.", file=sys.stderr)
    print("Sweeping any remaining flat config/hardware/*.json "
          "(swappable Setup files) into the shared folder...")
    moved = MachineConfig.sweep_remaining_flat_files(allow_sweep=not failed)
    for path in moved:
        print(f"  moved -> {path}")

    remaining = sorted(MachineConfig.HARDWARE_ROOT.glob("*.json"))
    if remaining:
        print("WARNING: files still sitting flat at config/hardware/ — either "
              "a name collision with the destination, or a per-machine file "
              "whose store did not load (see above):")
        for p in remaining:
            print(f"  {p}")

    print()
    print(f"Per-machine folder: {MachineConfig.machine_config_dir()}")
    print(f"Shared folder:      {MachineConfig.shared_config_dir()}")
    print()
    if failed:
        print("INCOMPLETE — fix the import failures above and re-run before "
              "committing.", file=sys.stderr)
        return 1
    print("Done. Run `git status` to confirm the old tracked paths show as "
          "deleted, then `git add -A && git commit` to finish removing them "
          "from tracking.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
