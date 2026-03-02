#!/usr/bin/env python3
"""
fix_print_objects_syringe.py — Wire HardwareConfig into print_objects.py.

The GeometryEngine warning "No syringe for pump P1" occurs because
print_objects.py only reads syringes from WorkspaceConfig, which is
empty. The actual syringe assignments are in HardwareConfig.

This patch:
1. Adds set_hardware_config() to PrintObjectsTab
2. Updates _extract_needle_syringe() to check HardwareConfig
3. Wires propagation in print_setup.py

Run from MEBP project root:
    python fix_print_objects_syringe.py
"""

import os, sys

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"

count = 0


# ── Fix 1: print_objects.py — add HardwareConfig awareness ───────
print(f"\n{BOLD}Fix 1: gui/pages/print_objects.py{RESET}")

path = "gui/pages/print_objects.py"
if not os.path.isfile(path):
    print(f"  {RED}SKIP{RESET}: {path} not found")
else:
    with open(path) as f:
        content = f.read()

    changed = False

    # 1a. Add _hw_config field to __init__
    old_init = "        # Workspace (set by Tab 1)\n        self._workspace = WorkspaceConfig() if HAS_MODELS else None"
    new_init = ("        # Workspace (set by Tab 1)\n"
                "        self._workspace = WorkspaceConfig() if HAS_MODELS else None\n\n"
                "        # v7.2: Hardware config for syringe/needle info\n"
                "        self._hw_config = None")

    if "self._hw_config = None" not in content and old_init in content:
        content = content.replace(old_init, new_init, 1)
        print(f"  {GREEN}OK{RESET}: Added _hw_config field to __init__")
        changed = True
    elif "self._hw_config" in content:
        print(f"  {YELLOW}SKIP{RESET}: _hw_config already in __init__")
    else:
        print(f"  {YELLOW}SKIP{RESET}: init anchor not found")

    # 1b. Add set_hardware_config method after set_workspace
    if "def set_hardware_config" not in content:
        anchor = '''    def set_workspace(self, workspace) -> None:
        """Update workspace config (called when Tab 1 changes)."""
        self._workspace = workspace
        self._refresh_ink_combos()
        self._update_well_diameter()'''

        replacement = '''    def set_workspace(self, workspace) -> None:
        """Update workspace config (called when Tab 1 changes)."""
        self._workspace = workspace
        self._refresh_ink_combos()
        self._update_well_diameter()

    def set_hardware_config(self, config) -> None:
        """v7.2: Receive hardware config for syringe/needle info."""
        self._hw_config = config'''

        if anchor in content:
            content = content.replace(anchor, replacement, 1)
            print(f"  {GREEN}OK{RESET}: Added set_hardware_config() method")
            changed = True
        else:
            print(f"  {YELLOW}SKIP{RESET}: set_workspace anchor not found")
    else:
        print(f"  {YELLOW}SKIP{RESET}: set_hardware_config already exists")

    # 1c. Update _extract_needle_syringe to check HardwareConfig
    old_extract = '''    def _extract_needle_syringe(self) -> tuple:
        """Extract (needle, syringe_map, settings) from workspace."""
        needle = None
        syringe_map = {}
        speed = 5.0
        layer_h = 0.2

        if self._workspace:
            needle = getattr(self._workspace, 'needle', None)
            for pid, pump in getattr(self._workspace, 'pumps', {}).items():
                if hasattr(pump, 'syringe') and pump.syringe is not None:
                    syringe_map[pid] = pump.syringe
            ps = getattr(self._workspace, 'print_settings', {})
            speed = ps.get('print_speed_mm_s', 5.0)
            layer_h = ps.get('layer_height_mm', 0.2)'''

    new_extract = '''    def _extract_needle_syringe(self) -> tuple:
        """Extract (needle, syringe_map, settings) from workspace + hardware config."""
        needle = None
        syringe_map = {}
        speed = 5.0
        layer_h = 0.2

        # Try workspace first
        if self._workspace:
            needle = getattr(self._workspace, 'needle', None)
            for pid, pump in getattr(self._workspace, 'pumps', {}).items():
                if hasattr(pump, 'syringe') and pump.syringe is not None:
                    syringe_map[pid] = pump.syringe
            ps = getattr(self._workspace, 'print_settings', {})
            speed = ps.get('print_speed_mm_s', 5.0)
            layer_h = ps.get('layer_height_mm', 0.2)

        # v7.2: Fall back to HardwareConfig for needle and syringes
        if self._hw_config is not None:
            if needle is None and hasattr(self._hw_config, 'needle'):
                needle = self._hw_config.needle
            if not syringe_map and hasattr(self._hw_config, 'pumps'):
                for pid, pump_cfg in self._hw_config.pumps.items():
                    if hasattr(pump_cfg, 'syringe') and pump_cfg.syringe is not None:
                        syringe_map[pid] = pump_cfg.syringe'''

    if old_extract in content:
        content = content.replace(old_extract, new_extract, 1)
        print(f"  {GREEN}OK{RESET}: Updated _extract_needle_syringe for HardwareConfig fallback")
        changed = True
    elif "self._hw_config" in content and "_extract_needle_syringe" in content:
        print(f"  {YELLOW}SKIP{RESET}: _extract_needle_syringe may already be patched")
    else:
        print(f"  {YELLOW}SKIP{RESET}: _extract_needle_syringe anchor not found")

    if changed:
        with open(path, "w") as f:
            f.write(content)
        count += 1


# ── Fix 2: print_setup.py — propagate HardwareConfig to print_objects ──
print(f"\n{BOLD}Fix 2: gui/pages/print_setup.py — propagate to print_objects{RESET}")

path2 = "gui/pages/print_setup.py"
if not os.path.isfile(path2):
    print(f"  {RED}SKIP{RESET}: {path2} not found")
else:
    with open(path2) as f:
        content2 = f.read()

    # Check if set_hardware_config exists and add forwarding to tab_objects
    if "def set_hardware_config" in content2:
        if "tab_objects" in content2 and "set_hardware_config" in content2:
            # Check if it already forwards to tab_objects
            if "self.tab_objects.set_hardware_config" not in content2:
                # Find set_hardware_config method and add forwarding
                old_shc = "        self._hardware_config = config"
                new_shc = ("        self._hardware_config = config\n"
                          "        # v7.2: Forward to Print Objects tab\n"
                          "        if hasattr(self, 'tab_objects') and hasattr(self.tab_objects, 'set_hardware_config'):\n"
                          "            self.tab_objects.set_hardware_config(config)")

                if old_shc in content2:
                    content2 = content2.replace(old_shc, new_shc, 1)
                    with open(path2, "w") as f:
                        f.write(content2)
                    print(f"  {GREEN}OK{RESET}: Added tab_objects forwarding in set_hardware_config")
                    count += 1
                else:
                    print(f"  {YELLOW}SKIP{RESET}: Could not find insertion point")
            else:
                print(f"  {YELLOW}SKIP{RESET}: Already forwards to tab_objects")
        else:
            print(f"  {YELLOW}SKIP{RESET}: Missing expected attributes")
    else:
        print(f"  {YELLOW}SKIP{RESET}: set_hardware_config not found in print_setup.py")


print(f"\n{BOLD}Done: {count} file(s) modified{RESET}")
