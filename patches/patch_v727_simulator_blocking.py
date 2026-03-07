#!/usr/bin/env python3
"""Fix XYStageSimulator: G and GR commands block until stage arrives,
   matching real Prior hardware behavior. Also revert _safe_navigate_to
   back to simple blocking version."""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

def find_root():
    for c in [Path(__file__).resolve().parent.parent.parent, Path.cwd(),
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir(): return c
    sys.exit("Cannot find MEBP root")

def find_method(content, name):
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE)
    return pat.search(content)

def main():
    print("  Simulator blocking moves + clean _safe_navigate_to")
    root = find_root()

    # ═══════════════════════════════════════════════════════════════
    # Part 1: XYStageSimulator — add _wait_for_settle helper,
    #         make G and GR block until arrival
    # ═══════════════════════════════════════════════════════════════
    print(f"\n  [1] XYStageSimulator — blocking G/GR")
    sim_path = root / "SupportClasses" / "XYStageSimulator.py"
    sim = sim_path.read_text(encoding="utf-8")

    marker = "v7.2.7: blocking move"
    if marker in sim:
        print("    ○ Already patched")
    else:
        # Add _wait_for_idle method before get_current_position
        get_pos_pat = re.compile(
            r'^(    def get_current_position\(self\))',
            re.MULTILINE
        )
        m_pos = get_pos_pat.search(sim)
        if m_pos:
            wait_method = '''    def _wait_for_idle(self, timeout_s: float = 60.0):
        """v7.2.7: blocking move — wait for physics loop to reach target.

        Real Prior G command blocks until the stage arrives and returns R.
        This makes the simulator behave identically.
        """
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._lock:
                if self.mode == "idle":
                    return True
            time.sleep(0.005)  # 5ms poll — fast enough, low CPU
        logger.warning(f"Simulator settle timeout after {timeout_s}s")
        return False

'''
            sim = sim[:m_pos.start()] + wait_method + sim[m_pos.start():]
            print("    ✓ Added _wait_for_idle method")
        else:
            print("    ✗ Could not find get_current_position")
            return 1

        # Now find _cmd_G (absolute move) and add blocking wait after setting target
        # Pattern: the G command handler sets mode="absolute" and returns "R"
        # We need to find where it returns "R" after setting absolute mode
        #
        # The G command is parsed in send_command or _execute_command.
        # Let's find the exact pattern. Looking at the code:
        #   with self._lock:
        #       self.mode = "absolute"
        #       ...target_x/y set...
        #   return "R"
        #
        # We need to insert _wait_for_idle() between the lock release and return "R"
        
        # Find: self.mode = "absolute"\n ... self.target_x = ... self.target_y = ...
        # followed by: return "R"
        # This appears in _cmd_G and _cmd_GR and _cmd_dir
        
        # Strategy: Add blocking to send_command for G and GR commands
        # Find the send_command method and add post-processing
        
        # Actually, simplest: find where "absolute" mode is set in the G command
        # handler. The simulator processes commands via send_command() which
        # calls into various _cmd_* methods. Let's find send_command.
        
        m_send = find_method(sim, "send_command")
        if m_send:
            body = m_send.group(0)
            # Find the return at the end of send_command
            # The method returns the response from _execute_command or similar
            # We need to add: if mode was set to "absolute", wait before returning
            
            # Better approach: add a wrapper at the end of send_command
            # Find the last "return" in the method
            
            # Actually, let's look at how commands flow.
            # In the simulator, send_command parses and returns immediately.
            # The cleanest approach: modify send_command to check if the
            # command was a move (G or GR) and if so, wait for settle.
            pass

        # Most robust approach: modify the actual response generation
        # Find where "G " commands are handled and inject wait before return
        
        # Look for the pattern in send_command that handles G:
        # The simulator's send_command dispatches to _execute_command or
        # handles inline. Let me search for the G command dispatch.
        
        # From project knowledge, the _cmd_G method likely exists or
        # the G command is handled in send_command directly
        
        # Search for "G " command handling
        g_handler = re.search(
            r'((?:el)?if\s+cmd_upper\.startswith\(["\']G\s*["\']\)|'
            r'(?:el)?if\s+.*?startswith\(["\']G ["\']\))',
            sim
        )
        
        if not g_handler:
            # Try alternate: look for where target is set for absolute mode
            # with a "G" context
            g_handler = re.search(r'# G — absolute move', sim)
        
        # Simplest guaranteed approach: patch the method that's called
        # after setting mode to absolute. Add wait in the physics response path.
        # 
        # OR: Just override send_command to add post-move blocking.
        # Find the end of send_command and wrap it.
        
        # Let me find the actual send_command structure
        send_pat = re.compile(
            r'^    def send_command\(self, command:.*?\n(.*?)(?=\n    def )',
            re.DOTALL | re.MULTILINE
        )
        m_sc = send_pat.search(sim)
        if m_sc:
            # Find the last line of send_command (the final return or end)
            # Insert a check: if mode is "absolute" after command, wait for idle
            
            # Strategy: replace send_command entirely with a version that
            # delegates to _send_command_inner and then waits
            
            # Actually, simplest: find where G/GR returns "R" and add wait
            # In the current code, commands go through _execute_command
            # which returns the response string, then send_command returns it.
            # 
            # Let's just add the wait at the end of send_command for move cmds
            pass

        # CLEANEST APPROACH: Just add the wait in the existing command
        # dispatch. Since the exact structure varies, let's add a generic
        # hook at the end of send_command.
        
        # Find send_command and add a post-hook
        if "def send_command" in sim:
            # Add a _post_command_hook that blocks for absolute moves
            # Find the first "return" after a G command sets mode
            
            # Actually, the simplest patch: modify _cmd_dir, and the G handler
            # to call _wait_for_idle before returning.
            
            # Let's look for ALL places that set mode = "absolute" and
            # add _wait_for_idle() call right before the return
            
            # Pattern: within a method, find lines where mode="absolute" is set
            # and there's a "return" nearby
            
            # Replace in _cmd_dir:
            dir_pat = re.compile(
                r'(def _cmd_dir\(self, cmd: str, direction: str\).*?'
                r'self\.mode = "absolute".*?)'
                r'(\s+return "R")',
                re.DOTALL
            )
            m_dir = dir_pat.search(sim)
            if m_dir:
                # Add wait before return
                sim = sim[:m_dir.start(2)] + (
                    "\n        # v7.2.7: blocking move\n"
                    "        self._wait_for_idle()\n"
                ) + sim[m_dir.start(2):]
                print("    ✓ Added blocking to _cmd_dir (L/R/F/B)")

            # For G and GR commands — they're likely in send_command's dispatch
            # Find pattern: sets mode="absolute", target_x/y, returns "R"
            # These may be inline in send_command or in _execute_command
            
            # Search for "G " or "G," command handling that returns "R"
            # with mode="absolute" nearby
            g_abs_pat = re.compile(
                r'(self\.mode\s*=\s*"absolute"\s*\n'
                r'(?:.*?self\.target_[xy]\s*=.*?\n)+)'
                r'(\s+return\s+"R")',
                re.DOTALL
            )
            # Apply to all matches (G and GR handlers)
            offset = 0
            for match in list(g_abs_pat.finditer(sim)):
                pos = match.start(2) + offset
                inject = (
                    "\n        # v7.2.7: blocking move\n"
                    "        self._wait_for_idle()\n"
                )
                sim = sim[:pos] + inject + sim[pos:]
                offset += len(inject)
                print(f"    ✓ Added blocking wait before return \"R\" (pos {match.start()})")

        # AST check
        try:
            ast.parse(sim)
        except SyntaxError as e:
            print(f"    ✗ AST FAIL: line {e.lineno}: {e.msg}")
            lines = sim.split('\n')
            for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
                mk = ">>>" if i==e.lineno-1 else "   "
                print(f"      {mk} {i+1:4d} | {lines[i]}")
            return 1

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(sim_path, sim_path.with_suffix(f".bak_v727blk_{ts}"))
        sim_path.write_text(sim, encoding="utf-8")
        print("    ✓ XYStageSimulator.py written + AST OK")

    # ═══════════════════════════════════════════════════════════════
    # Part 2: Revert _safe_navigate_to to simple blocking version
    # ═══════════════════════════════════════════════════════════════
    print(f"\n  [2] calibration.py — simple _safe_navigate_to")
    cal_path = root / "gui" / "pages" / "calibration.py"
    cal = cal_path.read_text(encoding="utf-8")

    m_nav = find_method(cal, "_safe_navigate_to")
    if m_nav:
        new_nav = '''    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7-simple: blocking absolute moves. Simulator now blocks like real hw."""
        ctrl = self.controller
        safe_z = getattr(self, '_safe_z', None) or 0.0

        # Step 1: Raise Z
        if ctrl.is_zp_connected:
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)

        # Step 2: Fast XY travel (blocks until arrival on both real hw and simulator)
        if ctrl.is_xy_connected:
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):
                    ctrl.xy_stage.set_speed_mm_s(50.0)
                else:
                    ctrl.xy_stage.set_velocity(100)
            ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)

        # Step 3: Lower Z
        if ctrl.is_zp_connected:
            if target_z_mm is not None:
                ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
            elif getattr(self, '_top_z', None) is not None:
                approach = self._top_z + getattr(self, '_z_buffer_mm', 0.5)
                ctrl.move_z_absolute(approach, from_zero_ref=True)

        logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")

'''
        # Remove _reenable_nav_buttons if it was added by threaded patch
        cal = cal[:m_nav.start()] + new_nav + cal[m_nav.end():]

        # Remove _reenable_nav_buttons method if present
        m_reenable = find_method(cal, "_reenable_nav_buttons")
        if m_reenable:
            cal = cal[:m_reenable.start()] + cal[m_reenable.end():]

        try:
            ast.parse(cal)
        except SyntaxError as e:
            print(f"    ✗ AST FAIL: line {e.lineno}: {e.msg}")
            return 1

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(cal_path, cal_path.with_suffix(f".bak_v727blk_{ts}"))
        cal_path.write_text(cal, encoding="utf-8")
        print("    ✓ calibration.py — simple blocking _safe_navigate_to")
    else:
        print("    ✗ _safe_navigate_to not found")

    print(f"\n  ✓ Done")
    return 0

if __name__ == "__main__":
    sys.exit(main())
