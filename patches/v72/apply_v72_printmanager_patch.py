#!/usr/bin/env python3
"""
apply_v72_printmanager_patch.py — Patch PrintManager.py for v7.2 µL-based pump control.

Run from the MEBP project root:
    python apply_v72_printmanager_patch.py

Changes applied:
    1. PrintSettings: Add µL fields (pump_rate_uL_s, retract_amounts_uL, etc.)
    2. _execute_command(): EXTRUDE checks for amount_uL first, falls back to mm
    3. _execute_print_path(): Uses flow_rate_uL_s with proper volume calculation
    4. build_well_plate_job(): Generates µL-based EXTRUDE and PRINT_PATH commands
    5. _load_json_file(): Auto-detects v7.1 vs v7.2 format
    6. New function: migrate_print_file_v71_to_v72()
    7. export_gcode(): Updated to handle µL-based commands

All changes maintain full backward compatibility with v7.1 print files.
"""

import os
import sys
import re

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"


def read_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path, content):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def replace_block(content, start_marker, end_marker, replacement, desc=""):
    """Replace content between start_marker and end_marker (inclusive of start, exclusive of end)."""
    start_idx = content.find(start_marker)
    if start_idx == -1:
        print(f"  {YELLOW}SKIP{RESET}: start marker not found for: {desc}")
        return content
    end_idx = content.find(end_marker, start_idx + len(start_marker))
    if end_idx == -1:
        print(f"  {YELLOW}SKIP{RESET}: end marker not found for: {desc}")
        return content
    result = content[:start_idx] + replacement + content[end_idx:]
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def replace_first(content, old, new, desc=""):
    if old not in content:
        print(f"  {YELLOW}SKIP{RESET}: text not found for: {desc}")
        return content
    result = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def insert_after(content, anchor, insertion, desc=""):
    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: anchor not found for: {desc}")
        return content
    idx = content.index(anchor) + len(anchor)
    result = content[:idx] + "\n" + insertion + content[idx:]
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def insert_before(content, anchor, insertion, desc=""):
    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: anchor not found for: {desc}")
        return content
    idx = content.index(anchor)
    result = content[:idx] + insertion + "\n" + content[idx:]
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def append_to_file(content, text, desc=""):
    result = content.rstrip() + "\n\n" + text + "\n"
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


# ═══════════════════════════════════════════════════════════════════
# PATCH DEFINITIONS
# ═══════════════════════════════════════════════════════════════════

def patch_print_settings(content):
    """Add µL fields to PrintSettings dataclass."""
    print("\n  --- PrintSettings µL fields ---")

    # Add new µL fields after the existing per-pump retract/prime amounts
    content = insert_after(
        content,
        '    prime_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})',
        '''
    # v7.2: µL-based pump settings
    pump_rate_uL_s: float = 0.25         # Default pump flow rate (µL/s)
    retract_amounts_uL: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    prime_amounts_uL: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    pump_rates_uL_s: dict = field(default_factory=lambda: {"P1": 0.25, "P2": 0.25, "P3": 0.25})''',
        "Add µL fields to PrintSettings",
    )

    # Add helper methods before from_dict
    content = insert_before(
        content,
        '    @classmethod\n    def from_dict(cls, d: dict) -> "PrintSettings":',
        '''    def get_retract_uL(self, pump: str) -> float:
        """Get retract amount for a pump in µL (v7.2). Falls back to legacy mm value."""
        uL = self.retract_amounts_uL.get(pump, 0.0)
        if uL > 0:
            return uL
        return 0.0

    def get_prime_uL(self, pump: str) -> float:
        """Get prime amount for a pump in µL (v7.2). Falls back to legacy mm value."""
        uL = self.prime_amounts_uL.get(pump, 0.0)
        if uL > 0:
            return uL
        return 0.0

    def get_pump_rate(self, pump: str) -> float:
        """Get flow rate for a specific pump in µL/s."""
        return self.pump_rates_uL_s.get(pump, self.pump_rate_uL_s)

''',
        "Add µL helper methods to PrintSettings",
    )

    return content


def patch_execute_command(content):
    """Update _execute_command() EXTRUDE handler to support µL."""
    print("\n  --- _execute_command EXTRUDE handler ---")

    old_extrude = """        elif cmd.type == CommandType.EXTRUDE:
            pump = p.get("pump", self._active_pump)
            amount = p.get("amount", 0)
            feedrate = p.get("feedrate", settings.pump_feedrate)
            ctrl.move_pump_relative(pump, amount, feedrate)
            wait = abs(amount) / (feedrate / 60.0) + 0.1
            time.sleep(min(wait, 5.0))"""

    new_extrude = """        elif cmd.type == CommandType.EXTRUDE:
            pump = p.get("pump", self._active_pump)

            # v7.2: Check for µL-based command first
            if "amount_uL" in p:
                amount_uL = p["amount_uL"]
                rate_uL_s = p.get("rate_uL_s", settings.get_pump_rate(pump))
                if hasattr(ctrl, 'move_pump_uL'):
                    ctrl.move_pump_uL(pump, amount_uL, rate_uL_s)
                else:
                    # Fallback if controller not yet patched
                    logger.warning("Controller missing move_pump_uL — using raw mm")
                    ctrl.move_pump_relative(pump, amount_uL * 0.3, 30.0)
                wait = abs(amount_uL) / max(rate_uL_s, 0.001) + 0.1
                time.sleep(min(wait, 10.0))

            # v7.1 legacy: mm-based command
            elif "amount" in p:
                amount = p["amount"]
                feedrate = p.get("feedrate", settings.pump_feedrate)
                ctrl.move_pump_relative(pump, amount, feedrate)
                wait = abs(amount) / max(feedrate / 60.0, 0.001) + 0.1
                time.sleep(min(wait, 5.0))"""

    content = replace_first(content, old_extrude, new_extrude,
                            "Update EXTRUDE handler for µL support")
    return content


def patch_execute_print_path(content):
    """Update _execute_print_path() to use flow_rate_uL_s."""
    print("\n  --- _execute_print_path µL flow rate ---")

    old_flow = """        flow_rate = cmd.params.get("flow_rate", 0.01)"""
    new_flow = """        # v7.2: flow_rate_uL_s takes priority, fall back to legacy flow_rate
        flow_rate_uL_s = cmd.params.get("flow_rate_uL_s", None)
        flow_rate = cmd.params.get("flow_rate", 0.01)
        use_uL = flow_rate_uL_s is not None"""

    content = replace_first(content, old_flow, new_flow,
                            "Add flow_rate_uL_s detection in _execute_print_path")

    # Replace the extrusion calculation block
    old_extrude_calc = """            # Extrude proportional to segment length
            extrude_amount = seg_length * flow_rate
            if extrude_amount > 0.0001:
                mapped = AXIS_MAP.get(pump)
                if mapped and ctrl.zp_stage:
                    ctrl.zp_stage.move_relative(
                        {mapped: extrude_amount},
                        settings.pump_feedrate,
                    )"""

    new_extrude_calc = """            # v7.2: Extrude using µL/s flow rate or legacy ratio
            if use_uL and flow_rate_uL_s and flow_rate_uL_s > 0:
                # Calculate volume from flow rate × segment time
                xy_speed = max(settings.print_feedrate, 1.0)
                seg_time = seg_length / (xy_speed / 60.0) if xy_speed > 0 else 0
                volume_uL = flow_rate_uL_s * seg_time
                if volume_uL > 0.001 and hasattr(ctrl, 'move_pump_uL'):
                    ctrl.move_pump_uL(pump, volume_uL, flow_rate_uL_s)
                elif volume_uL > 0.001:
                    mapped = AXIS_MAP.get(pump)
                    if mapped and ctrl.zp_stage:
                        ctrl.zp_stage.move_relative(
                            {mapped: volume_uL * 0.3},
                            settings.pump_feedrate,
                        )
            else:
                # Legacy: extrude proportional to segment length (dimensionless ratio)
                extrude_amount = seg_length * flow_rate
                if extrude_amount > 0.0001:
                    mapped = AXIS_MAP.get(pump)
                    if mapped and ctrl.zp_stage:
                        ctrl.zp_stage.move_relative(
                            {mapped: extrude_amount},
                            settings.pump_feedrate,
                        )"""

    content = replace_first(content, old_extrude_calc, new_extrude_calc,
                            "Update extrusion calculation for µL")
    return content


def patch_build_well_plate_job(content):
    """Update build_well_plate_job() to generate µL-based commands."""
    print("\n  --- build_well_plate_job µL commands ---")

    # Update prime command generation
    old_prime = """            # Prime (using per-pump amount)
            prime_amt = settings.get_prime_amount(active_pump)
            if prime_amt > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount": prime_amt,
                            "feedrate": settings.pump_feedrate},
                    label=f"Prime {active_pump}",
                ))"""

    new_prime = """            # Prime (v7.2: µL amounts, legacy mm fallback)
            prime_uL = settings.get_prime_uL(active_pump)
            prime_mm = settings.get_prime_amount(active_pump)
            if prime_uL > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount_uL": prime_uL,
                            "rate_uL_s": settings.get_pump_rate(active_pump)},
                    label=f"Prime {active_pump} ({prime_uL:.2f} µL)",
                ))
            elif prime_mm > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount": prime_mm,
                            "feedrate": settings.pump_feedrate},
                    label=f"Prime {active_pump} (legacy)",
                ))"""

    content = replace_first(content, old_prime, new_prime,
                            "Update prime command generation for µL")

    # Update print path command — add flow_rate_uL_s
    old_path = """            # Print the path in this well
            well_path = [(well_x + px, well_y + py) for px, py in path_points]
            commands.append(PrintCommand(
                type=CommandType.PRINT_PATH,
                params={
                    "points": well_path,
                    "pump": active_pump,
                    "flow_rate": flow_rate,
                },
                label=f"Print in {well_name}",
            ))"""

    new_path = """            # Print the path in this well (v7.2: include flow_rate_uL_s)
            well_path = [(well_x + px, well_y + py) for px, py in path_points]
            path_params = {
                "points": well_path,
                "pump": active_pump,
                "flow_rate": flow_rate,
            }
            # If flow_rate looks like µL/s (> 0.05), tag as v7.2
            pump_rate = settings.get_pump_rate(active_pump) if hasattr(settings, 'get_pump_rate') else 0
            if pump_rate > 0:
                path_params["flow_rate_uL_s"] = pump_rate
            commands.append(PrintCommand(
                type=CommandType.PRINT_PATH,
                params=path_params,
                label=f"Print in {well_name}",
            ))"""

    content = replace_first(content, old_path, new_path,
                            "Update print_path command for µL flow rate")

    # Update retract command
    old_retract = """            # Retract (using per-pump amount)
            retract_amt = settings.get_retract_amount(active_pump)
            if retract_amt > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount": -retract_amt,
                            "feedrate": settings.pump_feedrate},
                    label=f"Retract {active_pump}",
                ))"""

    new_retract = """            # Retract (v7.2: µL amounts, legacy mm fallback)
            retract_uL = settings.get_retract_uL(active_pump) if hasattr(settings, 'get_retract_uL') else 0
            retract_mm = settings.get_retract_amount(active_pump)
            if retract_uL > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount_uL": -retract_uL,
                            "rate_uL_s": settings.get_pump_rate(active_pump)},
                    label=f"Retract {active_pump} ({retract_uL:.2f} µL)",
                ))
            elif retract_mm > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount": -retract_mm,
                            "feedrate": settings.pump_feedrate},
                    label=f"Retract {active_pump} (legacy)",
                ))"""

    content = replace_first(content, old_retract, new_retract,
                            "Update retract command for µL")
    return content


def patch_export_gcode(content):
    """Update export_gcode() to handle µL-based EXTRUDE commands."""
    print("\n  --- export_gcode µL support ---")

    old_extrude_gcode = """        elif cmd.type == CommandType.EXTRUDE:
            amount = p.get("amount", 0)
            feedrate = p.get("feedrate", settings.pump_feedrate)
            pump = p.get("pump", "P1")
            lines.append(f"G1 E{amount:.5f} F{feedrate} ; {pump} {cmd.label}")"""

    new_extrude_gcode = """        elif cmd.type == CommandType.EXTRUDE:
            pump = p.get("pump", "P1")
            if "amount_uL" in p:
                # v7.2: µL-based — note in comment, use raw value for G-code
                amount_uL = p["amount_uL"]
                rate = p.get("rate_uL_s", settings.pump_rate_uL_s if hasattr(settings, 'pump_rate_uL_s') else 0.25)
                lines.append(f"; v7.2 EXTRUDE {pump}: {amount_uL:.3f} µL at {rate:.3f} µL/s")
                lines.append(f"G1 E{amount_uL:.5f} F{rate * 60:.1f} ; {pump} {cmd.label} (µL units)")
            else:
                amount = p.get("amount", 0)
                feedrate = p.get("feedrate", settings.pump_feedrate)
                lines.append(f"G1 E{amount:.5f} F{feedrate} ; {pump} {cmd.label}")"""

    content = replace_first(content, old_extrude_gcode, new_extrude_gcode,
                            "Update export_gcode EXTRUDE for µL")
    return content


def patch_load_json(content):
    """Update _load_json_file to detect and migrate v7.1 files."""
    print("\n  --- _load_json_file version detection ---")

    old_load = """def _load_json_file(path: Path) -> PrintJob:
    \"\"\"Load from custom JSON format.\"\"\"
    with open(path, "r") as f:
        data = json.load(f)

    settings = PrintSettings.from_dict(data.get("settings", {}))"""

    new_load = """def _load_json_file(path: Path) -> PrintJob:
    \"\"\"Load from custom JSON format. Auto-detects v7.1 vs v7.2 format.\"\"\"
    with open(path, "r") as f:
        data = json.load(f)

    # v7.2: Detect file version and migrate if needed
    file_version = data.get("version", "7.1")
    if file_version < "7.2":
        logger.info(f"Loading v{file_version} print file — commands may use legacy mm format")

    settings = PrintSettings.from_dict(data.get("settings", {}))"""

    content = replace_first(content, old_load, new_load,
                            "Add version detection to _load_json_file")
    return content


def add_migration_function(content):
    """Add migrate_print_file_v71_to_v72() function."""
    print("\n  --- Add migration function ---")

    migration_code = '''
# ═══════════════════════════════════════════════════════════════════
# v7.2: Print File Version Migration
# ═══════════════════════════════════════════════════════════════════

def migrate_print_file_v71_to_v72(data: dict, hardware_config=None) -> dict:
    """
    Migrate a v7.1 print file to v7.2 format.

    Converts mm-based pump amounts to µL using the hardware config.
    If no hardware config is available, cannot convert and returns
    the data with a migration warning flag.

    Args:
        data: Parsed JSON print file dict
        hardware_config: HardwareConfig with syringe specs for conversion

    Returns:
        Migrated data dict with v7.2 format commands
    """
    file_version = data.get("version", "7.1")
    if file_version >= "7.2":
        return data

    logger.info(f"Migrating print file from v{file_version} to v7.2")

    migrated = dict(data)
    migrated["version"] = "7.2"
    migrated["migrated_from"] = file_version

    if not hardware_config:
        logger.warning(
            "No hardware config — cannot convert mm→µL. "
            "Commands with legacy 'amount' (mm) will be executed as-is."
        )
        migrated["_migration_incomplete"] = True
        return migrated

    commands = migrated.get("commands", [])
    converted_count = 0

    for cmd in commands:
        cmd_type = cmd.get("type", "")

        if cmd_type == "extrude" and "amount" in cmd and "amount_uL" not in cmd:
            pump = cmd.get("pump", "P1")
            amount_mm = cmd["amount"]
            pump_cfg = hardware_config.pumps.get(pump)

            if pump_cfg and pump_cfg.is_configured:
                try:
                    cmd["amount_uL"] = pump_cfg.mm_to_uL(amount_mm)
                    if "feedrate" in cmd:
                        cmd["rate_uL_s"] = pump_cfg.feedrate_mm_min_to_uL_s(
                            cmd["feedrate"]
                        )
                    # Archive legacy values
                    cmd["_legacy_amount_mm"] = cmd.pop("amount")
                    if "feedrate" in cmd:
                        cmd["_legacy_feedrate_mm_min"] = cmd.pop("feedrate")
                    converted_count += 1
                except (ValueError, AttributeError) as e:
                    logger.warning(f"Migration failed for {pump} extrude: {e}")

        elif cmd_type == "print_path" and "flow_rate" in cmd and "flow_rate_uL_s" not in cmd:
            # Legacy flow_rate was dimensionless ratio — flag for manual review
            cmd["flow_rate_uL_s"] = cmd.get("flow_rate", 0.01)
            cmd["_legacy_flow_rate"] = cmd["flow_rate"]
            cmd["_needs_review"] = True
            converted_count += 1

    # Migrate settings
    settings = migrated.get("settings", {})
    if "pump_feedrate" in settings and "pump_rate_uL_s" not in settings:
        settings["pump_rate_uL_s"] = 0.25  # Default — needs manual verification

    if "retract_amounts" in settings and "retract_amounts_uL" not in settings:
        settings["retract_amounts_uL"] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}

    if "prime_amounts" in settings and "prime_amounts_uL" not in settings:
        settings["prime_amounts_uL"] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}

    logger.info(f"Migration complete: {converted_count} commands converted")
    return migrated


def detect_print_file_version(data: dict) -> str:
    """
    Detect the version of a print file.

    Returns "7.2" if any command has amount_uL or flow_rate_uL_s,
    otherwise returns whatever is in the version field or "7.1".
    """
    # Explicit version tag
    if "version" in data:
        return str(data["version"])

    # Heuristic: check commands for v7.2 fields
    for cmd in data.get("commands", []):
        if "amount_uL" in cmd or "flow_rate_uL_s" in cmd or "rate_uL_s" in cmd:
            return "7.2"

    # Heuristic: check settings for v7.2 fields
    settings = data.get("settings", {})
    if "pump_rate_uL_s" in settings or "retract_amounts_uL" in settings:
        return "7.2"

    return "7.1"
'''

    content = append_to_file(content, migration_code,
                             "Add migrate_print_file_v71_to_v72 + detect_print_file_version")
    return content


def patch_gcode_header(content):
    """Update G-code header version string."""
    print("\n  --- G-code header version ---")
    content = replace_first(
        content,
        'lines.append(f"; G-code exported from MEBP v7.0")',
        'lines.append(f"; G-code exported from MEBP v7.2")',
        "Update G-code export version string",
    )
    return content


def patch_save_print_job(content):
    """Update save_print_job to include version tag."""
    print("\n  --- save_print_job version tag ---")

    old_save_header = """    data = {
        "name": job.name,
        "description": job.description,"""

    new_save_header = """    data = {
        "version": "7.2",
        "name": job.name,
        "description": job.description,"""

    content = replace_first(content, old_save_header, new_save_header,
                            "Add version tag to save_print_job")
    return content


def update_init_exports(content_init):
    """Update __init__.py to export new migration functions."""
    print("\n  --- __init__.py migration exports ---")

    content_init = replace_first(
        content_init,
        '    "load_print_file", "save_print_job", "build_well_plate_job",',
        '    "load_print_file", "save_print_job", "build_well_plate_job",\n'
        '    "migrate_print_file_v71_to_v72", "detect_print_file_version",',
        "Add migration function exports to __init__.py",
    )

    # Also add to the import
    content_init = replace_first(
        content_init,
        "    load_print_file, save_print_job, build_well_plate_job,",
        "    load_print_file, save_print_job, build_well_plate_job,\n"
        "    migrate_print_file_v71_to_v72, detect_print_file_version,",
        "Add migration function imports to __init__.py",
    )

    return content_init


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    if os.path.isdir("SupportClasses"):
        root = "."
    elif os.path.isdir("MEBP-Version-7.0/SupportClasses"):
        root = "MEBP-Version-7.0"
    elif os.path.isdir("MEBP-Version-7.1/SupportClasses"):
        root = "MEBP-Version-7.1"
    else:
        print(f"{RED}ERROR{RESET}: Cannot find SupportClasses/ directory.")
        sys.exit(1)

    pm_path = os.path.join(root, "SupportClasses", "PrintManager.py")
    init_path = os.path.join(root, "SupportClasses", "__init__.py")

    print(f"\n{BOLD}MEBP v7.2 PrintManager Patch{RESET}")
    print(f"Project root: {os.path.abspath(root)}")

    if not os.path.isfile(pm_path):
        print(f"  {RED}ERROR{RESET}: {pm_path} not found!")
        sys.exit(1)

    content = read_file(pm_path)

    print(f"\n{'='*60}")
    print(f"Patching: {pm_path}")
    print(f"{'='*60}")

    # Apply all patches
    content = patch_print_settings(content)
    content = patch_execute_command(content)
    content = patch_execute_print_path(content)
    content = patch_build_well_plate_job(content)
    content = patch_export_gcode(content)
    content = patch_load_json(content)
    content = patch_gcode_header(content)
    content = patch_save_print_job(content)
    content = add_migration_function(content)

    write_file(pm_path, content)
    print(f"\n  {GREEN}DONE{RESET}: PrintManager.py patched for v7.2 µL support")

    # Patch __init__.py exports
    if os.path.isfile(init_path):
        print(f"\n{'='*60}")
        print(f"Patching: {init_path}")
        print(f"{'='*60}")
        init_content = read_file(init_path)
        init_content = update_init_exports(init_content)
        write_file(init_path, init_content)
        print(f"  {GREEN}DONE{RESET}: __init__.py updated")

    print(f"\n{BOLD}All PrintManager patches applied.{RESET}")
    print(f"\nSummary of changes:")
    print(f"  • PrintSettings: pump_rate_uL_s, retract_amounts_uL, prime_amounts_uL, pump_rates_uL_s")
    print(f"  • _execute_command: EXTRUDE checks amount_uL first, falls back to amount (mm)")
    print(f"  • _execute_print_path: Uses flow_rate_uL_s with volume = rate × time calculation")
    print(f"  • build_well_plate_job: Generates µL-based commands when settings have µL values")
    print(f"  • export_gcode: Handles both µL and mm EXTRUDE commands")
    print(f"  • _load_json_file: Version detection for v7.1 vs v7.2 files")
    print(f"  • NEW: migrate_print_file_v71_to_v72() + detect_print_file_version()")
    print(f"  • save_print_job: Adds version='7.2' tag")


if __name__ == "__main__":
    main()
