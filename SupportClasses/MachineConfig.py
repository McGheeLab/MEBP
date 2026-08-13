"""MachineConfig.py — per-machine vs. shared config folder resolution.

MEBP runs on several physical rigs (e.g. ``ME3B_01``, ``ME3B_02``) that all pull
from one shared git repo. Per-machine state (camera calibration, taught plate
positions, mosaics, needle-bore calibration, ...) must never collide between
rigs, while a set of shared/portable catalogs (needle/plate/well-type
libraries, plate/rosette designs, print-adjacent configs) is meant to sync
across every rig.

Previously this split was enforced file-by-file in ``.gitignore`` — a list that
had to be kept in sync by hand every time a new per-machine store was added,
and had fallen behind (``config/hardware/well_training/`` and several other
per-machine stores were being tracked in git, which is exactly what caused
device-specific pull conflicts). This module makes the split STRUCTURAL
instead: every per-machine store lives under ``config/hardware/<machine-id>/``
and every shared store lives under ``config/hardware/ME3B_general/`` — so
``.gitignore`` only needs one rule ("everything under config/hardware/*/ is
per-machine except ME3B_general/") and a brand-new per-machine store added
later is ignored automatically with no further edits, as long as it resolves
its path through ``resolve_machine_path``/``resolve_shared_path`` below.

**The machine identity IS the active device profile's name.** The profile
already names the rig, is chosen by the operator on Hardware Setup → Device
(the mandatory Page 0), and is recorded in the per-machine, gitignored
``settings.json``. So naming a rig ``ME3B_01`` gives it
``config/hardware/ME3B_01/``, and there is exactly one home for "which
machine is this". Device profiles themselves live in the machine-INDEPENDENT
``config/hardware/devices/`` (they name the folder, so they cannot live
inside it) and are tracked in git: each rig's file has a distinct name, so
they cannot conflict, and every rig's envelope/steps-per-mm/axis map gets a
backup a rebuilt machine can restore from.

Machine identity resolution order:

  1. ``MEBP_MACHINE_ID`` environment variable (test/CI override — matches the
     existing ``MEBP_<STORE>_DIR`` convention used by several stores here).
  2. ``settings.json`` → ``device_profile.active``.
  3. The literal string ``"unassigned"`` — a real, functioning bucket that can
     never collide with an actual rig's data, used so a fresh/unconfigured
     checkout never hard-crashes at startup. ``machine_id_is_configured()``
     lets a caller detect this case; while unconfigured NOTHING is ever moved
     (see ``resolve_machine_path``).

Zero GUI dependencies (stdlib only), mirroring every other ``SupportClasses``
store in this repo.
"""

from __future__ import annotations

import json
import logging
import os
import re
import shutil
from pathlib import Path
from typing import Optional, Union

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent
HARDWARE_ROOT = REPO_ROOT / "config" / "hardware"
GENERAL_DIRNAME = "ME3B_general"
DEVICES_DIRNAME = "devices"
UNASSIGNED_ID = "unassigned"

#: THE machine identity is the ACTIVE DEVICE PROFILE's name. The profile
#: already names the rig ("this is ME3B_01"), is already chosen by the
#: operator on Hardware Setup → Device (the mandatory Page 0), and is already
#: recorded in the per-machine, gitignored settings.json. Deriving the config
#: folder from it means there is exactly ONE home for "which machine is this"
#: — an earlier cut of this module kept a separate config/machine_id.txt
#: beside it, which is precisely the two-homes-for-one-fact trap this
#: codebase keeps being bitten by.
#:
#: Read from the FILE rather than through ``Settings``: this module must stay
#: stdlib-only (no repo imports, no import cycle), and the value is needed at
#: module-import time, before any Settings object necessarily exists.
_settings_path = REPO_ROOT / "settings.json"

#: Folder names the per-machine bucket may never take — they already mean
#: something else under config/hardware/. A rig whose profile is called
#: "ME3B_general" would put its private calibration into the SHARED, TRACKED
#: folder and publish it to every other rig: the exact failure this module
#: exists to prevent.
RESERVED_IDS = frozenset({GENERAL_DIRNAME.casefold(), DEVICES_DIRNAME.casefold()})

#: Spaces and () + are allowed because real profiles are named like "ME3B V1".
#: What is refused is anything that changes the MEANING of the path
#: (separators, drive colons, dot-only names). Deliberately NOT a many-to-one
#: sanitize: quietly mapping two distinct profile names onto one folder is the
#: collision trap recorded for the six filesystem stores in CLAUDE.md, and here
#: it would silently merge two rigs' calibration.
_ID_RE = re.compile(r"^[A-Za-z0-9 _.()+-]+$")

# Cached so repeated calls in one process don't re-read settings.json; a test
# that wants to change identity mid-run should call reset_cache() (mirrors the
# reset hooks other stores expose for tests).
_cached_id: Optional[str] = None
_warned_unassigned = False
_warned_unconfigured = False


def _sanitize(value: str) -> str:
    """Validate a machine id (= device profile name). Never rewrites it."""
    value = (value or "").strip()
    if not value or not _ID_RE.match(value):
        raise ValueError(
            f"Invalid machine name {value!r} — letters, digits, spaces and "
            "_ . - ( ) + only (no / \\ : * ? \" < > |)")
    # '.' and '..' satisfy the charset but are path traversal: the id becomes a
    # directory name, so '..' would resolve the per-machine folder to config/
    # itself and scatter this rig's files over the shared tree.
    if set(value) == {"."}:
        raise ValueError(f"Invalid machine name {value!r} — reserved path name")
    if value.casefold() in RESERVED_IDS:
        raise ValueError(
            f"'{value}' is reserved — config/hardware/{value} already holds "
            "shared config, so a rig by that name would publish its private "
            "calibration to every other rig. Pick another name (e.g. ME3B_01).")
    return value


def is_valid_machine_name(value: str) -> tuple[bool, str]:
    """``(ok, reason)`` — for UI validation without catching exceptions."""
    try:
        _sanitize(value)
        return True, ""
    except ValueError as exc:
        return False, str(exc)


def set_settings_path(path) -> None:
    """Point the identity lookup at a non-default settings.json.

    ``main.py --settings other.json`` must not leave this module reading the
    default file, or the app would run one rig's settings against another
    rig's config folder.
    """
    global _settings_path
    _settings_path = Path(path)
    reset_cache()


def active_profile_name() -> str:
    """The active device profile's name from settings.json, or ''."""
    try:
        with open(_settings_path, encoding="utf-8") as fh:
            data = json.load(fh)
        name = (data.get("device_profile") or {}).get("active") or ""
        return str(name).strip()
    except (OSError, ValueError, AttributeError):
        return ""


def reset_cache() -> None:
    """Forget the cached machine id (env var and marker file are re-read).

    ⚠ Does NOT retarget module-level constants that already resolved their
    path at import time (``_DEFAULT_PATH`` and friends). A test that needs a
    different machine id must set ``MEBP_MACHINE_ID`` before importing the
    store, or reload the module.
    """
    global _cached_id, _warned_unassigned, _warned_unconfigured
    _cached_id = None
    _warned_unassigned = False
    _warned_unconfigured = False


def machine_id() -> str:
    """This rig's identity: env override > active device profile > fallback."""
    global _cached_id, _warned_unassigned

    env = os.environ.get("MEBP_MACHINE_ID")
    if env:
        try:
            return _sanitize(env)
        except ValueError as exc:
            logger.warning("MEBP_MACHINE_ID=%r ignored: %s", env, exc)

    if _cached_id is not None:
        return _cached_id

    name = active_profile_name()
    if name:
        try:
            _cached_id = _sanitize(name)
            return _cached_id
        except ValueError as exc:
            logger.warning(
                "Active device profile %r cannot name a config folder: %s "
                "Rename it on Hardware Setup → Device.", name, exc)

    if not _warned_unassigned:
        logger.warning(
            "No device profile selected — using the '%s' config bucket. Load "
            "or create a device profile on Hardware Setup → Device to give "
            "this machine its own calibration folder.", UNASSIGNED_ID)
        _warned_unassigned = True
    _cached_id = UNASSIGNED_ID
    return _cached_id


def machine_id_is_configured() -> bool:
    """True once a real (non-fallback) machine identity is available.

    Never raises — an unreadable/absent settings.json reports "not
    configured", the conservative answer (callers then move nothing).
    """
    if os.environ.get("MEBP_MACHINE_ID"):
        try:
            _sanitize(os.environ["MEBP_MACHINE_ID"])
            return True
        except ValueError:
            return False
    name = active_profile_name()
    if not name:
        return False
    try:
        _sanitize(name)
        return True
    except ValueError:
        return False


def _warn_unconfigured_once() -> None:
    """Say once, loudly, that per-machine config is not being separated yet."""
    global _warned_unconfigured
    if not _warned_unconfigured:
        _warned_unconfigured = True
        logger.warning(
            "No device profile selected — per-machine config is being read "
            "from its legacy location and NOTHING will be moved. Load or "
            "create a device profile on Hardware Setup → Device (it names "
            "this machine) and restart to separate this rig from the others.")


def set_machine_id(value: str) -> None:
    """Set the active device profile name in settings.json.

    ⚠ Load-modify-save on the real file, never a bare ``Settings()`` — an
    errant constructor once wiped settings.json wholesale (recorded in
    CLAUDE.md). The GUI writes the same key through ``Settings``; this exists
    for headless/CI use, so do not call it while the app is running or the
    in-memory Settings will overwrite it on the next save.
    """
    global _cached_id, _warned_unassigned, _warned_unconfigured
    clean = _sanitize(value)
    try:
        with open(_settings_path, encoding="utf-8") as fh:
            data = json.load(fh)
        if not isinstance(data, dict):
            raise ValueError("settings.json is not an object")
    except FileNotFoundError:
        data = {}
    section = data.get("device_profile")
    if not isinstance(section, dict):
        section = {}
    section["active"] = clean
    data["device_profile"] = section
    tmp = Path(str(_settings_path) + ".tmp")
    tmp.parent.mkdir(parents=True, exist_ok=True)
    tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
    os.replace(tmp, _settings_path)
    _cached_id = clean
    _warned_unassigned = False
    _warned_unconfigured = False
    logger.info("Active device profile set to %r (%s)", clean, _settings_path)


def devices_dir() -> Path:
    """Where device profiles live — machine-INDEPENDENT, and tracked in git.

    ⚠ This must NOT sit under :func:`machine_config_dir`: the profile is what
    NAMES that folder, so keeping profiles inside it is a chicken-and-egg.
    Each rig's profile has a distinct filename, so tracking them cannot
    produce a merge conflict, and it gives every rig's safety envelope /
    steps-per-mm / axis map a backup that a rebuilt machine can restore from.
    """
    d = HARDWARE_ROOT / DEVICES_DIRNAME
    d.mkdir(parents=True, exist_ok=True)
    return d


def machine_config_dir() -> Path:
    """Root directory for THIS machine's per-rig config (created if absent)."""
    d = HARDWARE_ROOT / machine_id()
    d.mkdir(parents=True, exist_ok=True)
    return d


def rename_machine_folder(old_name: str, new_name: str) -> bool:
    """Move ``config/hardware/<old>/`` to ``<new>/`` when a profile is renamed.

    A rename means "same rig, new label", so its calibration must follow —
    otherwise the operator silently lands on an empty folder and every taught
    position appears lost. Returns True if a folder was actually moved.
    Refuses rather than merges when the destination already exists.
    """
    try:
        old_clean, new_clean = _sanitize(old_name), _sanitize(new_name)
    except ValueError as exc:
        logger.warning("Not renaming machine folder: %s", exc)
        return False
    if old_clean == new_clean:
        return False
    src, dst = HARDWARE_ROOT / old_clean, HARDWARE_ROOT / new_clean
    if not src.is_dir():
        return False
    if dst.exists():
        logger.warning(
            "Not moving '%s' -> '%s': the destination already exists. Merge "
            "them by hand; picking one silently would lose calibration.",
            src, dst)
        return False
    shutil.move(str(src), str(dst))
    logger.info("Machine config folder renamed '%s' -> '%s'", src, dst)
    return True


def shared_config_dir() -> Path:
    """Root directory for config meant to sync across every rig."""
    d = HARDWARE_ROOT / GENERAL_DIRNAME
    d.mkdir(parents=True, exist_ok=True)
    return d


def _migrate_once(target: Path, legacy: Path) -> Path:
    """Move `legacy` -> `target` the first time `target` is asked for.

    Safe to call every time a store resolves its path: cheap once migrated
    (both existence checks are no-ops), and idempotent (a second call with
    nothing left at `legacy` just returns `target`). Works for both a single
    file and a whole directory tree (``shutil.move`` handles both).
    """
    if not target.exists() and legacy.exists() and legacy.resolve() != target.resolve():
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            shutil.move(str(legacy), str(target))
            logger.info("Migrated config '%s' -> '%s'", legacy, target)
        except OSError as exc:
            logger.warning("Could not migrate '%s' -> '%s': %s", legacy, target, exc)
    return target


def resolve_machine_path(relative: Union[str, Path]) -> Path:
    """Path for a per-machine file/dir, auto-migrating it from the legacy
    flat ``config/hardware/<relative>`` location the first time it's asked
    for. `relative` may name a file (e.g. 'objectives.json') or a directory
    (e.g. 'well_training').

    ⚠ **Nothing is ever MOVED while the machine id is unconfigured.** These
    paths resolve at module-import time, and a store can be imported before
    the operator has had any chance to say which rig this is (importing
    ``gui.app`` alone pulls in five of them). Migrating then would file real
    calibration under the ``unassigned`` fallback, and once the operator
    answers the prompt the app would look in ``<their-id>/`` and find an
    empty folder — the calibration is still on disk but invisible, which is
    exactly the silently-wrong state this module exists to prevent. So while
    unconfigured we READ THE LEGACY LOCATION IN PLACE: the app behaves
    exactly as it did before the split, and the real migration happens on the
    next run, once the id is known.
    """
    legacy = HARDWARE_ROOT / relative
    if not machine_id_is_configured():
        _warn_unconfigured_once()
        # Read in place if it is there; otherwise point at the fallback
        # bucket so a WRITE still lands somewhere real and self-consistent.
        return legacy if legacy.exists() else HARDWARE_ROOT / UNASSIGNED_ID / relative
    return _migrate_once(machine_config_dir() / relative, legacy)


def resolve_shared_path(relative: Union[str, Path]) -> Path:
    """Path for a shared/portable file/dir, auto-migrating it from the
    legacy flat ``config/hardware/<relative>`` location the first time it's
    asked for."""
    target = shared_config_dir() / relative
    legacy = HARDWARE_ROOT / relative
    return _migrate_once(target, legacy)


#: Filenames that are per-machine no matter what. The sweep below files
#: leftovers as SHARED, i.e. straight into git — so if a store ever fails to
#: claim its own file (a bad import, a missing optional dependency), that
#: file must not be swept. Publishing one rig's calibration to every other rig
#: is precisely the failure this module exists to prevent, so the sweep is
#: denied by name here rather than trusting that every store loaded.
PER_MACHINE_FILENAMES = frozenset({
    "calibration_status.json",
    "camera_calibrations.json",
    "microscope.json",
    "fluorescence_mosaics.json",
    "last_calibration.json",
    "plate_mosaics.json",
    "mosaic_alignment.json",
    "objectives.json",
    "reanchor_features.json",
    "print_timing_calibration.json",
    "needle_bore_calibration.json",
    "needle_focus_templates.json",
    "plate_focus_datum.json",
    "plate_level_sites.json",
    "spheroid_sink_calibration.json",
    "spheroid_training.json",
    "plate_templates.json",
    "lablink.json",
    "lablink_results.json",
})


def sweep_remaining_flat_files(*, allow_sweep: bool = True) -> list:
    """Move any ``*.json`` file still sitting directly in the legacy flat
    ``config/hardware/`` root into the shared folder.

    By the time this runs, every known per-machine/shared store has already
    claimed its own file or subtree via ``resolve_machine_path``/
    ``resolve_shared_path``. Whatever is still flat is a swappable
    ``*Setup*.json`` hardware config (arbitrarily named by the operator, so
    no store resolves it by a fixed name) or an unclassified stray — either
    way it belongs in the shared bucket, not left behind where nothing looks
    for it any more. NOT run automatically on import (only explicitly, by
    ``tools_migrate_machine_config.py``) since it touches files with no
    single well-known name.

    ``allow_sweep=False`` reports what WOULD move without touching anything —
    used when a store failed to import, since the classification can no
    longer be trusted.
    """
    moved: list = []
    if not HARDWARE_ROOT.is_dir():
        return moved
    dest_dir = shared_config_dir()
    for f in sorted(HARDWARE_ROOT.glob("*.json")):
        if f.name in PER_MACHINE_FILENAMES:
            logger.warning(
                "'%s' is per-machine but was not claimed by its store — NOT "
                "sweeping it into the shared folder. Its store probably "
                "failed to import; fix that and re-run.", f)
            continue
        target = dest_dir / f.name
        if target.exists():
            logger.warning(
                "Leftover '%s' left in place — a file already exists at '%s'",
                f, target)
            continue
        if not allow_sweep:
            logger.warning("Would move '%s' -> '%s' (skipped)", f, target)
            continue
        shutil.move(str(f), str(target))
        logger.info("Migrated leftover config '%s' -> '%s'", f, target)
        moved.append(target)
    return moved
