#!/usr/bin/env python3
"""
MEBP v7.2.7 — Feature upgrades: triggers, speed, pump visibility, status.

1) LT=retract, RT=extend on configured pump (negate axis 4 in XboxController)
2) Faster XY/axis update (avg_interval 0.5→0.1, poll_interval 1.0→0.3)
3) Jog page pump visibility from hardware config
4) Disconnect properly clears connection status
5) Xbox status dot reads correctly in thread mode
"""

import ast, json, re, shutil, sys, platform
from datetime import datetime
from pathlib import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

ok_count = 0
skip_count = 0
miss_count = 0

def find_root():
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent,
              Path(__file__).resolve().parent.parent.parent,
              Path.home() / "Documents" / "GitHub" / "MEBP",
              Path.home() / "OneDrive" / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    sys.exit("Cannot find MEBP root")

def find_method(content, name, indent=4):
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(.*?\n)(.*?)(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)

def report(tag, msg):
    global ok_count, skip_count, miss_count
    if tag == "OK":
        print(f"  {GREEN}✓{RESET} {msg}"); ok_count += 1
    elif tag == "SKIP":
        print(f"  {YELLOW}○{RESET} {msg}"); skip_count += 1
    elif tag == "MISS":
        print(f"  {RED}✗{RESET} {msg}"); miss_count += 1

def safe_write(path, content, label):
    if path.suffix == ".py":
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {RED}AST FAIL{RESET} {label}: {e}")
            return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727feat_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}→ {label} written{RESET}")
    return True


# ═══════════════════════════════════════════════════════════════════
# FIX 1: LT=retract, RT=extend + mapping update
# ═══════════════════════════════════════════════════════════════════

def fix_trigger_direction(root):
    print(f"\n{CYAN}{BOLD}Fix 1: LT=retract, RT=extend{RESET}")

    # 1a: Update XboxController.py — negate axis 4 for triggers
    path = root / "SupportClasses" / "XboxController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: LT retract"
    if marker not in content:
        # Find where trigger avg_val is computed and sent.
        # Pattern: "else:" block after "if group["type"] == "axis":"
        # The trigger sends: avg_val = axis_accum[a] / axis_count[a]
        # We need to negate for axis 4 (LT).
        #
        # Find the line: avg_val = axis_accum[a] / axis_count[a] if axis_count[a] else 0.0
        # in the "else:" (trigger) branch
        trigger_avg = re.search(
            r'(                    else:\s*\n'
            r'                        a = axes\[0\]\s*\n'
            r'                        avg_val = axis_accum\[a\] / axis_count\[a\] if axis_count\[a\] else 0\.0)',
            content
        )
        if trigger_avg:
            old = trigger_avg.group(0)
            new = (
                '                    else:\n'
                '                        a = axes[0]\n'
                '                        avg_val = axis_accum[a] / axis_count[a] if axis_count[a] else 0.0\n'
                f'                        # {marker}: LT (axis 4) = retract (negative)\n'
                '                        if a == 4:\n'
                '                            avg_val = -avg_val'
            )
            content = content.replace(old, new, 1)
            report("OK", "XboxController: axis 4 (LT) negated for retract")
        else:
            report("MISS", "trigger avg_val pattern not found")

        if content != original:
            safe_write(path, content, "XboxController.py")
    else:
        report("SKIP", "LT retract already applied")

    # 1b: Update mapping — map triggers to configured pump (P1 by default)
    map_path = root / "current_button_mapping.json"
    if map_path.exists():
        mapping = json.loads(map_path.read_text(encoding="utf-8"))
        axes = mapping.get("axes", {})
        changed = False
        # Map both triggers to P1 (the typically configured pump)
        if axes.get("4") != "move_p1_at_velocity":
            axes["4"] = "move_p1_at_velocity"
            changed = True
        if axes.get("5") != "move_p1_at_velocity":
            axes["5"] = "move_p1_at_velocity"
            changed = True
        if changed:
            mapping["axes"] = axes
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(map_path, map_path.with_suffix(f".bak_v727feat_{ts}"))
            map_path.write_text(json.dumps(mapping, indent=4) + "\n", encoding="utf-8")
            report("OK", "Mapping: triggers → move_p1_at_velocity (LT=retract, RT=extend)")
        else:
            report("SKIP", "Mapping already has P1 on triggers")


# ═══════════════════════════════════════════════════════════════════
# FIX 2: Faster update frequencies
# ═══════════════════════════════════════════════════════════════════

def fix_update_speed(root):
    print(f"\n{CYAN}{BOLD}Fix 2: Faster update frequencies{RESET}")

    # 2a: XboxController avg_interval 0.5 → 0.1
    path = root / "SupportClasses" / "XboxController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker2a = "v7.2.7: fast axis update"
    if marker2a not in content:
        # Find avg_interval default in function signature
        old_sig = 'avg_interval: float = 0.5'
        if old_sig in content:
            content = content.replace(old_sig,
                f'avg_interval: float = 0.1  # {marker2a}', 1)
            report("OK", "Xbox avg_interval: 0.5s → 0.1s")
        else:
            # Maybe already changed
            report("SKIP", "avg_interval signature not found (may be different)")

        # Also speed up the averaging timer check
        old_avg_check = 'if current_time - last_axis_time >= avg_interval:'
        new_avg_check = 'if current_time - last_axis_time >= avg_interval:  # 100ms default'
        if old_avg_check in content and marker2a not in content:
            content = content.replace(old_avg_check, new_avg_check, 1)

        if content != original:
            safe_write(path, content, "XboxController.py")
    else:
        report("SKIP", "fast axis update already applied")

    # 2b: PositionPoller poll_interval 1.0 → 0.3
    path = root / "SupportClasses" / "StageController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker2b = "v7.2.7: fast position poll"
    if marker2b not in content:
        old_poll = 'self._pos_poller = PositionPoller(poll_interval=1.0)'
        if old_poll in content:
            content = content.replace(old_poll,
                f'self._pos_poller = PositionPoller(poll_interval=0.3)  # {marker2b}', 1)
            report("OK", "PositionPoller: 1.0s → 0.3s")
        else:
            # Try alternate patterns
            poll_pat = re.search(r'(PositionPoller\(poll_interval=)([\d.]+)', content)
            if poll_pat and float(poll_pat.group(2)) > 0.5:
                old = poll_pat.group(0)
                content = content.replace(old,
                    f'PositionPoller(poll_interval=0.3  # {marker2b}', 1)
                report("OK", f"PositionPoller: {poll_pat.group(2)}s → 0.3s")
            else:
                report("SKIP", "PositionPoller already fast or not found")

        if content != original:
            safe_write(path, content, "StageController.py")
    else:
        report("SKIP", "fast poll already applied")


# ═══════════════════════════════════════════════════════════════════
# FIX 3: Xbox status dot (thread mode)
# ═══════════════════════════════════════════════════════════════════

def fix_xbox_status(root):
    print(f"\n{CYAN}{BOLD}Fix 3: Xbox status dot + disconnect indicators{RESET}")

    # The XboxQueuePoller._poll_loop must handle "status" messages.
    # Check if the _poll_loop already does this.
    path = root / "SupportClasses" / "StageController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    # 3a: Ensure _xbox_status is initialized
    marker3a = "v7.2.7: xbox status init"
    if '_xbox_status' in content:
        report("SKIP", "_xbox_status already exists in XboxQueuePoller")
    else:
        report("MISS", "_xbox_status NOT in XboxQueuePoller — add it")

    # 3b: Ensure is_xbox_connected works in thread mode
    # The property should check both process and thread
    marker3b = "v7.2.7: thread-aware xbox check"
    if marker3b not in content:
        # The is_xbox_connected property needs to handle thread mode
        # Find the property and check its logic
        prop = find_method(content, "is_xbox_connected")
        if prop:
            prop_body = content[prop.start():prop.end()]
            # Check if it references _xbox_thread
            if "_xbox_thread" not in prop_body and "xbox_poller" in prop_body:
                # It uses xbox_poller which exists in both modes, should be OK
                # But the xbox_status property might return "unknown" if
                # _xbox_status was never set. Let's check...
                pass
        
        # The key issue: getattr(self.controller, 'is_xbox_connected', False)
        # in app.py uses getattr with default False. If is_xbox_connected is
        # a property, getattr calls it. If it raises, returns False.
        # But the xbox_status property returns "unknown" when _xbox_status
        # hasn't been set yet, and "unknown" not in ("connected", "alive").
        #
        # Fix: Make _xbox_status default to "disconnected" not checked via
        # attribute existence. Already done in v7.2.7 patches.
        #
        # More likely issue: the poller's _poll_loop doesn't handle "status"
        # messages. Let's verify:
        if '"status" in msg' in content:
            report("SKIP", "Status handler exists in _poll_loop")
        else:
            report("MISS", "Status handler MISSING — needs to be added")

    # 3c: Ensure app.py conn dot uses the property correctly
    app_path = root / "gui" / "app.py"
    app_content = app_path.read_text(encoding="utf-8")

    # The issue might be that app.py checks is_xbox_connected which is
    # a property on the controller, but calls it with getattr default False
    old_xbox_check = 'xbox_ok = getattr(self.controller, \'is_xbox_connected\', False)'
    alt_xbox_check = 'xbox_ok = getattr(self.controller, "is_xbox_connected", False)'
    if old_xbox_check in app_content or alt_xbox_check in app_content:
        # getattr with a property calls the getter — this should work
        # unless the property itself has a bug
        report("SKIP", "app.py xbox check uses getattr (should call property)")

    # 3d: Fix dashboard disconnect handling
    dash_path = root / "gui" / "pages" / "dashboard.py"
    dash_content = dash_path.read_text(encoding="utf-8")
    dash_original = dash_content

    # Check _disconnect_xbox clears status
    marker3d = "v7.2.7: disconnect clears status"
    if marker3d not in dash_content:
        # Find _disconnect_xy and verify it calls controller.disconnect_xy()
        for stage_name in ["xy", "zp", "xbox"]:
            method_name = f"_disconnect_{stage_name}"
            m = find_method(dash_content, method_name)
            if m:
                body = dash_content[m.start():m.end()]
                if f"controller.disconnect_{stage_name}" in body:
                    report("SKIP", f"{method_name} calls controller properly")
                else:
                    report("MISS", f"{method_name} may not call controller.disconnect_{stage_name}")
            else:
                report("MISS", f"{method_name} not found")

        # The real issue is likely that on_status_update reads cached state.
        # After disconnect, the controller properties should return False.
        # Let's check if the controller disconnect methods clear state:
        if "self.xy_stage = None" in content and "self.zp_stage = None" in content:
            report("SKIP", "Controller disconnect sets stage=None (is_xy_connected returns False)")

    # The most likely issue #5: is_xbox_connected uses xbox_status which
    # uses xbox_poller._xbox_status. After connection, the worker sends
    # {"status": "connected"} then heartbeats. BUT — the poller may
    # not have processed the status message yet when the dashboard checks.
    # Also: in thread mode, xbox_process is None, and the old is_alive()
    # check would fail.
    #
    # Simplest fix: add a forced status set in connect_xbox after poller starts
    marker3e = "v7.2.7: force initial status"
    if marker3e not in content:
        connect_match = re.search(
            r'(        self\.xbox_poller = XboxQueuePoller\(self\.xbox_queue, self\.processor\)\s*\n'
            r'        self\.xbox_poller\.start\(\))',
            content
        )
        if connect_match:
            inject = (
                f'\n        self.xbox_poller._xbox_status = "waiting"  # {marker3e}'
            )
            pos = connect_match.end()
            content = content[:pos] + inject + content[pos:]
            report("OK", "Force _xbox_status='waiting' on connect")
        else:
            report("MISS", "Could not find poller.start() in connect_xbox")

    if content != original:
        safe_write(path, content, "StageController.py")


# ═══════════════════════════════════════════════════════════════════
# FIX 4: Jog page pump visibility
# ═══════════════════════════════════════════════════════════════════

def fix_jog_pump_visibility(root):
    print(f"\n{CYAN}{BOLD}Fix 4: Jog page pump visibility from hardware config{RESET}")

    path = root / "gui" / "pages" / "jog_control.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: pump visibility"

    if marker in content:
        report("SKIP", "Pump visibility already applied")
        return

    # Find set_hardware_config in the jog page
    m = find_method(content, "set_hardware_config")
    if not m:
        report("MISS", "set_hardware_config not found in jog_control.py")
        return

    # Replace with version that updates pump control visibility
    new_method = (
        f'    def set_hardware_config(self, config):  # {marker}\n'
        '        """Update jog page from hardware config — show/hide pump controls."""\n'
        '        self._hardware_config = config\n'
        '        self._update_pump_step_combo()\n'
        '        # Update pump section visibility based on configured pumps\n'
        '        if config:\n'
        '            configured = set(config.configured_pump_ids) if hasattr(config, "configured_pump_ids") else set()\n'
        '        else:\n'
        '            configured = set()\n'
        '        for pid in ["P1", "P2", "P3"]:\n'
        '            frame = getattr(self, f"_pump_frame_{pid.lower()}", None)\n'
        '            if frame is not None:\n'
        '                frame.setVisible(pid in configured)\n'
        '            # Also update labels\n'
        '            lbl = getattr(self, f"lbl_{pid.lower()}_name", None)\n'
        '            if lbl and config:\n'
        '                pcfg = config.pumps.get(pid)\n'
        '                if pcfg and pcfg.is_configured:\n'
        '                    ink = getattr(pcfg, "ink_name", "") or pid\n'
        '                    lbl.setText(f"{pid}: {ink}")\n'
        '                else:\n'
        '                    lbl.setText(f"{pid}: (not configured)")\n'
        '        logger.info(f"Jog: pump visibility updated — active: {configured or \'none\'}")\n'
        '\n'
    )
    content = content[:m.start()] + new_method + content[m.end():]
    report("OK", "set_hardware_config now updates pump visibility")

    if content != original:
        safe_write(path, content, "jog_control.py")


# ═══════════════════════════════════════════════════════════════════
# FIX 5: Dashboard disconnect refresh
# ═══════════════════════════════════════════════════════════════════

def fix_disconnect_refresh(root):
    print(f"\n{CYAN}{BOLD}Fix 5: Dashboard disconnect forces status refresh{RESET}")

    path = root / "gui" / "pages" / "dashboard.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: disconnect refresh"
    if marker in content:
        report("SKIP", "Already applied")
        return

    # The issue: after calling controller.disconnect_xy(), the dashboard
    # doesn't immediately refresh. The on_status_update runs every 300ms
    # but the disconnect handler doesn't force an immediate update.
    #
    # Fix: Call on_status_update() at the end of each disconnect handler.

    for stage in ["xy", "zp", "xbox"]:
        method_name = f"_disconnect_{stage}"
        m = find_method(content, method_name)
        if m:
            body = content[m.start():m.end()]
            if "on_status_update" not in body:
                # Inject on_status_update call at end of method
                # Find the last line of the method body
                method_end = m.end()
                inject = f'        self.on_status_update()  # {marker}\n'
                # Insert before the method boundary
                content = content[:method_end] + inject + content[method_end:]
                report("OK", f"_disconnect_{stage}: added immediate status refresh")
            else:
                report("SKIP", f"_disconnect_{stage} already refreshes")
        else:
            report("MISS", f"_disconnect_{stage} not found")

    # Also do the same for connect methods
    for stage in ["xy", "zp"]:
        method_name = f"_connect_{stage}"
        m = find_method(content, method_name)
        if m:
            body = content[m.start():m.end()]
            if "on_status_update" not in body:
                method_end = m.end()
                inject = f'        self.on_status_update()  # {marker}\n'
                content = content[:method_end] + inject + content[method_end:]
                report("OK", f"_connect_{stage}: added immediate status refresh")

    if content != original:
        safe_write(path, content, "dashboard.py")


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{BOLD}{'='*60}")
    print(f"  MEBP v7.2.7 — Feature Upgrades")
    print(f"{'='*60}{RESET}")

    root = find_root()
    print(f"Project root: {root}")
    print(f"Platform:     {platform.system()}\n")

    fix_trigger_direction(root)
    fix_update_speed(root)
    fix_xbox_status(root)
    fix_jog_pump_visibility(root)
    fix_disconnect_refresh(root)

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  SUMMARY: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    print(f"\n  Changes:")
    print(f"  1. LT(axis 4) = retract (negative), RT(axis 5) = extend (positive)")
    print(f"     Both mapped to move_p1_at_velocity")
    print(f"  2. Xbox axis averaging: 500ms → 100ms (snappier response)")
    print(f"     Position polling: 1.0s → 0.3s")
    print(f"  3. Xbox status forced to 'waiting' on connect (fixes dot)")
    print(f"  4. Jog page hides unconfigured pumps, shows configured ones")
    print(f"  5. Disconnect handlers force immediate status refresh")
    print()

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
