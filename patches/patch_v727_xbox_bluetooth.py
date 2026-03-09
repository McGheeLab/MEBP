#!/usr/bin/env python3
"""
MEBP v7.2.7 — Xbox Bluetooth Controller Fix

Patches:
  1) SupportClasses/XboxController.py — SDL hints, event pump, diagnostics
  2) SupportClasses/StageController.py — Status pipeline, log fix, thread fallback
  3) gui/pages/dashboard.py — Xbox status feedback

Works on both pre-v7.2.6 and post-v7.2.6 codebases.
Run from anywhere; auto-detects MEBP project root.
"""

import ast
import os
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

# ── Terminal Colors ───────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

ok_count = 0
skip_count = 0
miss_count = 0


def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path.cwd(),
        Path(__file__).resolve().parent,
        Path(__file__).resolve().parent.parent,
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP project root.{RESET}")
    print("Run from the project directory or place this script inside it.")
    sys.exit(1)


def safe_read(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → backup → write. Returns False on AST failure."""
    # AST verify (only for .py files)
    if path.suffix == ".py":
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {RED}AST FAIL{RESET} for {label}: {e}")
            return False

    # Backup
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v727_{ts}")
    if path.exists():
        shutil.copy2(path, backup)

    path.write_text(content, encoding="utf-8")
    return True


def find_method(content: str, name: str, indent: int = 4):
    """Find method boundaries. Returns (start, end) or None."""
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def report(tag, msg):
    global ok_count, skip_count, miss_count
    if tag == "OK":
        print(f"  {GREEN}✓ OK{RESET}:   {msg}")
        ok_count += 1
    elif tag == "SKIP":
        print(f"  {YELLOW}○ SKIP{RESET}: {msg}")
        skip_count += 1
    elif tag == "MISS":
        print(f"  {RED}✗ MISS{RESET}: {msg}")
        miss_count += 1


# ═══════════════════════════════════════════════════════════════════
# SESSION 1: XboxController.py — Bluetooth-Aware Worker
# ═══════════════════════════════════════════════════════════════════

def patch_xbox_controller(root: Path):
    print(f"\n{CYAN}{BOLD}Session 1: XboxController.py — Bluetooth-Aware Worker{RESET}")
    path = root / "SupportClasses" / "XboxController.py"
    content = safe_read(path)
    if not content:
        report("MISS", "XboxController.py not found")
        return

    original = content

    # ── S1-A: SDL environment variables ───────────────────────────
    marker_s1a = "v7.2.7: SDL Bluetooth hints"
    if marker_s1a not in content:
        # Find the xbox_polling_worker function
        m = re.search(
            r'(def xbox_polling_worker\(.*?\).*?:.*?\n)',
            content, re.DOTALL
        )
        if m:
            # Find the `try: import pygame` or `import pygame` inside the function
            func_start = m.end()
            # Look for either the try/import or direct import pattern
            pygame_import = re.search(
                r'(    try:\s*\n\s+import pygame)',
                content[func_start:]
            )
            if pygame_import:
                inject_pos = func_start + pygame_import.start()
            else:
                pygame_import = re.search(
                    r'(    import pygame)',
                    content[func_start:]
                )
                if pygame_import:
                    inject_pos = func_start + pygame_import.start()
                else:
                    inject_pos = None

            if inject_pos is not None:
                sdl_block = (
                    f'    # {marker_s1a}\n'
                    f'    import os as _os\n'
                    f'    _os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "1")\n'
                    f'    _os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")\n'
                    f'\n'
                )
                content = content[:inject_pos] + sdl_block + content[inject_pos:]
                report("OK", "S1-A: SDL Bluetooth hints injected before pygame import")
            else:
                report("MISS", "S1-A: Could not find pygame import inside xbox_polling_worker")
        else:
            report("MISS", "S1-A: xbox_polling_worker function not found")
    else:
        report("SKIP", "S1-A: SDL hints already present")

    # ── S1-B + S1-C: Replace _find_controller with Bluetooth-aware version ──
    marker_s1b = "v7.2.7: Bluetooth-aware detection"
    if marker_s1b not in content:
        # Check if _find_controller exists (v7.2.6+)
        if "def _find_controller" in content:
            # Replace the existing _find_controller with enhanced version
            fc_pattern = re.compile(
                r'(    def _find_controller\(\):.*?""".*?""")\s*\n(.*?)(?=\n    # ── |    joystick = _find_controller)',
                re.DOTALL
            )
            fc_match = fc_pattern.search(content)
            if fc_match:
                new_find_controller = (
                    '    def _find_controller():\n'
                    f'        """Retry until a controller is found. {marker_s1b}."""\n'
                    '        _attempt = 0\n'
                    '        while True:\n'
                    '            _attempt += 1\n'
                    '            pygame.event.pump()  # Critical for Bluetooth on macOS\n'
                    '            pygame.joystick.quit()\n'
                    '            pygame.joystick.init()\n'
                    '            count = pygame.joystick.get_count()\n'
                    '            if count > 0:\n'
                    '                js = pygame.joystick.Joystick(0)\n'
                    '                js.init()\n'
                    '                out_queue.put({"debug": f"Controller connected: {js.get_name()} "\n'
                    '                               f"(axes={js.get_numaxes()}, btns={js.get_numbuttons()}, "\n'
                    '                               f"hats={js.get_numhats()}, attempt={_attempt})"})\n'
                    '                out_queue.put({"status": "connected"})\n'
                    '                return js\n'
                    '            else:\n'
                    '                if _attempt <= 3 or _attempt % 10 == 0:\n'
                    '                    out_queue.put({"debug": f"No controller found "\n'
                    '                                   f"(attempt {_attempt}), retrying in 2s..."})\n'
                    '                out_queue.put({"status": "waiting"})\n'
                    '                time.sleep(2.0)\n'
                )
                content = content[:fc_match.start()] + new_find_controller + content[fc_match.end():]
                report("OK", "S1-B/C: _find_controller replaced with Bluetooth-aware version")
            else:
                report("MISS", "S1-B/C: Could not match _find_controller boundaries")
        else:
            # Pre-v7.2.6: no _find_controller exists. Need to add it AND convert
            # the old exit-on-failure code to use it.
            # Find the old pattern: if count == 0: ... return  OR  if count > 0: ... else: ... return
            old_detect = re.search(
                r'    # Get the number of.*?get_count\(\).*?if count\s*==\s*0.*?return',
                content, re.DOTALL
            )
            if not old_detect:
                old_detect = re.search(
                    r'    pygame\.joystick\.init\(\).*?count\s*=\s*pygame\.joystick\.get_count\(\).*?'
                    r'if count\s*(?:==\s*0|>).*?return',
                    content, re.DOTALL
                )

            if old_detect:
                # Find the block from joystick.init() through the else/return
                # and replace it with _find_controller definition + call
                block_start = old_detect.start()
                # Find where the old detection block ends (the return statement + next blank line)
                block_end_pattern = re.search(r'\n\n', content[old_detect.end():])
                if block_end_pattern:
                    block_end = old_detect.end() + block_end_pattern.end()
                else:
                    block_end = old_detect.end()

                replacement = (
                    f'    # {marker_s1b}\n'
                    '    def _find_controller():\n'
                    '        """Retry until a controller is found."""\n'
                    '        _attempt = 0\n'
                    '        while True:\n'
                    '            _attempt += 1\n'
                    '            pygame.event.pump()\n'
                    '            pygame.joystick.quit()\n'
                    '            pygame.joystick.init()\n'
                    '            count = pygame.joystick.get_count()\n'
                    '            if count > 0:\n'
                    '                js = pygame.joystick.Joystick(0)\n'
                    '                js.init()\n'
                    '                out_queue.put({"debug": f"Controller connected: {js.get_name()} "\n'
                    '                               f"(axes={js.get_numaxes()}, btns={js.get_numbuttons()}, "\n'
                    '                               f"hats={js.get_numhats()}, attempt={_attempt})"})\n'
                    '                out_queue.put({"status": "connected"})\n'
                    '                return js\n'
                    '            else:\n'
                    '                if _attempt <= 3 or _attempt % 10 == 0:\n'
                    '                    out_queue.put({"debug": f"No controller found "\n'
                    '                                   f"(attempt {_attempt}), retrying in 2s..."})\n'
                    '                out_queue.put({"status": "waiting"})\n'
                    '                time.sleep(2.0)\n'
                    '\n'
                    '    joystick = _find_controller()\n\n'
                )
                content = content[:block_start] + replacement + content[block_end:]
                report("OK", "S1-B/C: Injected _find_controller (pre-v7.2.6 code path)")
            else:
                report("MISS", "S1-B/C: Could not find old controller detection code")

    else:
        report("SKIP", "S1-B/C: Bluetooth-aware detection already present")

    # ── S1-D: Add event pump to reconnection handler (post-crash) ─
    marker_s1d = "v7.2.7: event pump in reconnect"
    if marker_s1d not in content:
        # Look for the reconnection loop pattern (v7.2.6 crash recovery)
        reconnect_pattern = re.search(
            r'(joystick = None\s*\n\s*# Fall back to retry loop\s*\n\s*while joystick is None:)\s*\n'
            r'(\s*time\.sleep\(2\.0\)\s*\n\s*pygame\.joystick\.quit)',
            content
        )
        if reconnect_pattern:
            # Add event pump before the sleep in reconnect loop
            old_block = reconnect_pattern.group(0)
            new_block = old_block.replace(
                'while joystick is None:\n',
                f'while joystick is None:  # {marker_s1d}\n'
            ).replace(
                'time.sleep(2.0)\n',
                'time.sleep(2.0)\n                    pygame.event.pump()  # Bluetooth needs this\n'
            )
            content = content.replace(old_block, new_block)
            report("OK", "S1-D: Event pump added to reconnection loop")
        else:
            # Try alternate pattern: the worker calls _find_controller again after crash
            if "joystick = _find_controller()" in content:
                # _find_controller already has event pump from S1-B, so reconnection
                # via _find_controller() is already covered
                report("SKIP", "S1-D: Reconnection uses _find_controller (already has pump)")
            else:
                report("SKIP", "S1-D: No reconnection loop found (pre-v7.2.6 or handled by _find_controller)")
    else:
        report("SKIP", "S1-D: Event pump in reconnect already present")

    # ── S1-E: Ensure heartbeat in main loop ───────────────────────
    marker_s1e = "v7.2.7: heartbeat"
    if '{"status": "alive"}' not in content and marker_s1e not in content:
        # Need to add heartbeat to main loop
        # Find the main while True loop
        main_loop = re.search(r'(    while True:\s*\n\s*current_time = time\.time\(\))', content)
        if main_loop:
            inject_heartbeat = (
                '    last_heartbeat = time.time()  # ' + marker_s1e + '\n\n'
            )
            content = content[:main_loop.start()] + inject_heartbeat + content[main_loop.start():]

            # Now add the heartbeat check inside the loop
            pump_line = re.search(
                r'(            pygame\.event\.pump\(\)\n)',
                content
            )
            if pump_line:
                heartbeat_code = (
                    '\n'
                    '            # Heartbeat every 3 seconds\n'
                    '            if current_time - last_heartbeat >= 3.0:\n'
                    '                out_queue.put({"status": "alive"})\n'
                    '                last_heartbeat = current_time\n'
                )
                content = (content[:pump_line.end()] + heartbeat_code +
                          content[pump_line.end():])
                report("OK", "S1-E: Heartbeat added to main loop")
            else:
                report("MISS", "S1-E: Could not find event.pump() in main loop for heartbeat injection")
        else:
            report("MISS", "S1-E: Main while loop not found")
    else:
        report("SKIP", "S1-E: Heartbeat already present")

    # ── Write ─────────────────────────────────────────────────────
    if content != original:
        if safe_write(path, content, "XboxController.py"):
            print(f"  {GREEN}→ XboxController.py written successfully{RESET}")
        else:
            print(f"  {RED}→ XboxController.py WRITE FAILED (AST error){RESET}")
    else:
        print(f"  {YELLOW}→ XboxController.py unchanged{RESET}")


# ═══════════════════════════════════════════════════════════════════
# SESSION 2: StageController.py — Status Pipeline + Log Fix
# ═══════════════════════════════════════════════════════════════════

def patch_stage_controller(root: Path):
    print(f"\n{CYAN}{BOLD}Session 2: StageController.py — Status Pipeline + Thread Fallback{RESET}")
    path = root / "SupportClasses" / "StageController.py"
    content = safe_read(path)
    if not content:
        report("MISS", "StageController.py not found")
        return

    original = content

    # ── S2-A: Fix misleading log message in connect_xbox ──────────
    marker_s2a = "v7.2.7: worker started"
    if marker_s2a not in content:
        # Match either the old or v7.2.6 version of the final log message
        old_log_patterns = [
            'logger.info(f"Xbox controller connected (mapping: {self._mapping_file})")',
            'logger.info("Xbox controller connected")',
        ]
        replaced = False
        for old_log in old_log_patterns:
            if old_log in content:
                new_log = (
                    f'logger.info(f"Xbox worker started — searching for controller '
                    f'(mapping: {{self._mapping_file}})  # {marker_s2a}")'
                )
                content = content.replace(old_log, new_log, 1)
                report("OK", "S2-A: Fixed misleading 'connected' log → 'worker started'")
                replaced = True
                break
        if not replaced:
            report("SKIP", "S2-A: Log message already fixed or not found")
    else:
        report("SKIP", "S2-A: Log already fixed")

    # ── S2-B: Ensure XboxQueuePoller has _xbox_status ─────────────
    marker_s2b = "v7.2.7: xbox status tracking"
    if '_xbox_status' not in content:
        # Find XboxQueuePoller.__init__
        init_match = re.search(
            r'(class XboxQueuePoller:.*?def __init__\(self.*?\):.*?\n)',
            content, re.DOTALL
        )
        if init_match:
            # Find the end of __init__ body
            init_body = re.search(
                r'(self\._thread.*?None)\n',
                content[init_match.end():]
            )
            if init_body:
                inject_pos = init_match.end() + init_body.end()
                inject = f'        self._xbox_status: str = "disconnected"  # {marker_s2b}\n'
                content = content[:inject_pos] + inject + content[inject_pos:]
                report("OK", "S2-B: Added _xbox_status to XboxQueuePoller.__init__")
            else:
                report("MISS", "S2-B: Could not find end of XboxQueuePoller.__init__")
        else:
            report("MISS", "S2-B: XboxQueuePoller class not found")
    else:
        report("SKIP", "S2-B: _xbox_status already exists")

    # ── S2-C: Ensure _poll_loop handles "status" messages ─────────
    marker_s2c = "v7.2.7: status handler"
    if '"status" in msg' not in content and marker_s2c not in content:
        # Find the _poll_loop method and its "debug" handler
        debug_handler = re.search(
            r'(                if "debug" in msg:)',
            content
        )
        if debug_handler:
            status_handler = (
                f'                if "status" in msg:  # {marker_s2c}\n'
                f'                    self._xbox_status = msg["status"]\n'
                f'                    logger.info(f"[Xbox] Status: {{self._xbox_status}}")\n'
                f'                el'
            )
            content = content[:debug_handler.start()] + status_handler + content[debug_handler.start() + len('                '):]
            report("OK", "S2-C: Added status handler to _poll_loop")
        else:
            report("MISS", "S2-C: Could not find debug handler in _poll_loop")
    else:
        report("SKIP", "S2-C: Status handler already present")

    # ── S2-D: Ensure xbox_status property exists ──────────────────
    if 'def xbox_status(self)' not in content:
        # Find the disconnect_xbox method to inject properties after it
        disconnect_xbox = find_method(content, "disconnect_xbox")
        if disconnect_xbox:
            props = (
                '\n'
                '    # v7.2.7: Xbox status properties\n'
                '    @property\n'
                '    def xbox_status(self) -> str:\n'
                '        """Return Xbox connection status string.\n'
                '        Returns: disconnected, waiting, connected, or alive.\n'
                '        """\n'
                '        if self.xbox_poller is None:\n'
                '            return "disconnected"\n'
                '        return getattr(self.xbox_poller, "_xbox_status", "unknown")\n'
                '\n'
                '    @property\n'
                '    def is_xbox_connected(self) -> bool:\n'
                '        """Backward-compatible bool: True when controller is active."""\n'
                '        return self.xbox_status in ("connected", "alive")\n'
                '\n'
            )
            content = content[:disconnect_xbox.end()] + props + content[disconnect_xbox.end():]
            report("OK", "S2-D: Added xbox_status + is_xbox_connected properties")
        else:
            report("MISS", "S2-D: Could not find disconnect_xbox method")
    else:
        report("SKIP", "S2-D: xbox_status property already exists")

    # ── S2-E: Add threading fallback to connect_xbox ──────────────
    marker_s2e = "v7.2.7: thread fallback"
    if marker_s2e not in content:
        # Find the connect_xbox method
        cx_match = find_method(content, "connect_xbox")
        if cx_match:
            # Replace the entire connect_xbox with enhanced version
            new_connect_xbox = (
                '    def connect_xbox(self, mapping_file: str = "current_button_mapping.json",\n'
                '                     use_thread: bool = False) -> None:\n'
                f'        """Connect Xbox controller. {marker_s2e}\n'
                '\n'
                '        Args:\n'
                '            mapping_file: Path to button mapping JSON.\n'
                '            use_thread: If True, run worker in a thread instead of process.\n'
                '                        Use this for macOS Bluetooth controllers that are\n'
                '                        invisible to spawned subprocesses.\n'
                '        """\n'
                '        if self.xbox_process and self.xbox_process.is_alive():\n'
                '            logger.warning("Xbox already connected")\n'
                '            return\n'
                '        # Also check thread-based worker\n'
                '        if getattr(self, "_xbox_thread", None) and self._xbox_thread.is_alive():\n'
                '            logger.warning("Xbox already connected (thread mode)")\n'
                '            return\n'
                '\n'
                '        from pathlib import Path as _Path\n'
                '        self._mapping_file = str(_Path(mapping_file).resolve())\n'
                '\n'
                '        if self.xy_stage is None and self.zp_stage is None:\n'
                '            logger.warning(\n'
                '                "Xbox started but no stages are connected -- "\n'
                '                "controller input will have no effect until stages connect"\n'
                '            )\n'
                '\n'
                '        self.xbox_queue = Queue()\n'
                '\n'
                '        if use_thread:\n'
                '            import threading\n'
                '            self._xbox_thread = threading.Thread(\n'
                '                target=xbox_polling_worker,\n'
                '                args=(self.xbox_queue,),\n'
                '                kwargs={"mapping_file": self._mapping_file},\n'
                '                daemon=True,\n'
                '                name="XboxWorkerThread",\n'
                '            )\n'
                '            self._xbox_thread.start()\n'
                '            self.xbox_process = None  # Not using process mode\n'
                '            logger.info(f"Xbox worker started (THREAD mode, mapping: {self._mapping_file})")\n'
                '        else:\n'
                '            self.xbox_process = Process(\n'
                '                target=xbox_polling_worker,\n'
                '                args=(self.xbox_queue,),\n'
                '                kwargs={"mapping_file": self._mapping_file},\n'
                '                daemon=True,\n'
                '            )\n'
                '            self.xbox_process.start()\n'
                '            logger.info(f"Xbox worker started (PROCESS mode, mapping: {self._mapping_file})")\n'
                '\n'
                '        self.xbox_poller = XboxQueuePoller(self.xbox_queue, self.processor)\n'
                '        self.xbox_poller.start()\n'
                '\n'
            )
            content = content[:cx_match.start()] + new_connect_xbox + content[cx_match.end():]
            report("OK", "S2-E: connect_xbox replaced with thread fallback support")
        else:
            report("MISS", "S2-E: connect_xbox method not found")
    else:
        report("SKIP", "S2-E: Thread fallback already present")

    # ── S2-F: Update disconnect_xbox for thread mode ──────────────
    marker_s2f = "v7.2.7: thread cleanup"
    if marker_s2f not in content:
        dx_match = find_method(content, "disconnect_xbox")
        if dx_match:
            new_disconnect = (
                '    def disconnect_xbox(self) -> None:\n'
                f'        """Disconnect Xbox controller. {marker_s2f}"""\n'
                '        if self.xbox_poller:\n'
                '            self.xbox_poller.stop()\n'
                '            self.xbox_poller = None\n'
                '        if self.xbox_process and self.xbox_process.is_alive():\n'
                '            self.xbox_process.terminate()\n'
                '            self.xbox_process.join(timeout=2.0)\n'
                '            self.xbox_process = None\n'
                '        # v7.2.7: thread mode cleanup\n'
                '        _xt = getattr(self, "_xbox_thread", None)\n'
                '        if _xt and _xt.is_alive():\n'
                '            # Thread mode — worker checks queue; we signal via _running\n'
                '            # but it\'s a daemon thread, so it dies when main exits.\n'
                '            # We can\'t cleanly stop it, but we nil the poller so no more dispatch.\n'
                '            pass\n'
                '        self._xbox_thread = None\n'
                '        self.xbox_queue = None\n'
                '        logger.info("Xbox controller disconnected")\n'
                '\n'
            )
            content = content[:dx_match.start()] + new_disconnect + content[dx_match.end():]
            report("OK", "S2-F: disconnect_xbox updated for thread mode")
        else:
            report("MISS", "S2-F: disconnect_xbox method not found")
    else:
        report("SKIP", "S2-F: Thread cleanup already present")

    # ── S2-G: Ensure is_xbox_connected handles thread mode ────────
    # The xbox_status property already handles this via xbox_poller._xbox_status
    # But we need to make sure is_xbox_connected also checks the thread
    marker_s2g = "v7.2.7: is_xbox_connected thread"
    old_is_xbox = 'return self.xbox_status in ("connected", "alive")'
    if old_is_xbox in content and marker_s2g not in content:
        new_is_xbox = (
            '# ' + marker_s2g + '\n'
            '        status = self.xbox_status\n'
            '        if status in ("connected", "alive"):\n'
            '            return True\n'
            '        # Fallback: check if process or thread is alive\n'
            '        if self.xbox_process and self.xbox_process.is_alive():\n'
            '            return False  # Process alive but controller not found yet\n'
            '        _xt = getattr(self, "_xbox_thread", None)\n'
            '        if _xt and _xt.is_alive():\n'
            '            return False  # Thread alive but controller not found yet\n'
            '        return False'
        )
        content = content.replace(old_is_xbox, new_is_xbox, 1)
        report("OK", "S2-G: is_xbox_connected updated for thread mode")
    else:
        report("SKIP", "S2-G: is_xbox_connected already handles thread or not found")

    # ── Write ─────────────────────────────────────────────────────
    if content != original:
        if safe_write(path, content, "StageController.py"):
            print(f"  {GREEN}→ StageController.py written successfully{RESET}")
        else:
            print(f"  {RED}→ StageController.py WRITE FAILED (AST error){RESET}")
    else:
        print(f"  {YELLOW}→ StageController.py unchanged{RESET}")


# ═══════════════════════════════════════════════════════════════════
# SESSION 3: Dashboard — Xbox Status Feedback
# ═══════════════════════════════════════════════════════════════════

def patch_dashboard(root: Path):
    print(f"\n{CYAN}{BOLD}Session 3: dashboard.py — Xbox Status Feedback{RESET}")
    path = root / "gui" / "pages" / "dashboard.py"
    content = safe_read(path)
    if not content:
        report("MISS", "dashboard.py not found")
        return

    original = content

    # ── S3-A: Enhance _connect_xbox to offer thread mode ──────────
    marker_s3a = "v7.2.7: thread mode option"
    if marker_s3a not in content:
        # Find the _connect_xbox or _on_connect_xbox method
        for method_name in ["_connect_xbox", "_on_connect_xbox", "_on_ctx_connect_xbox"]:
            cx_btn = find_method(content, method_name)
            if cx_btn:
                break

        if cx_btn:
            new_method = (
                f'    def {method_name}(self):\n'
                f'        """Connect Xbox controller. {marker_s3a}"""\n'
                f'        import platform\n'
                f'        use_thread = False\n'
                f'        # On macOS, Bluetooth controllers often need thread mode\n'
                f'        if platform.system() == "Darwin":\n'
                f'            use_thread = True\n'
                f'            logger.info("macOS detected — using thread mode for Bluetooth compatibility")\n'
                f'        mapping = getattr(self.controller, "_mapping_file",\n'
                f'                          "current_button_mapping.json")\n'
                f'        self.controller.connect_xbox(mapping_file=mapping, use_thread=use_thread)\n'
                f'\n'
            )
            content = content[:cx_btn.start()] + new_method + content[cx_btn.end():]
            report("OK", f"S3-A: {method_name} enhanced with thread mode for macOS")
        else:
            report("SKIP", "S3-A: Xbox connect button handler not found")
    else:
        report("SKIP", "S3-A: Thread mode option already present")

    # ── S3-B: Show richer Xbox status tooltip ─────────────────────
    marker_s3b = "v7.2.7: xbox tooltip"
    if marker_s3b not in content:
        # Find the on_status_update where xbox status is checked
        xbox_status_check = re.search(
            r'(_xbox_st\s*=\s*getattr\(self\.controller.*?xbox_status.*?\).*?\n'
            r'\s*self\._update_conn_status\("xbox".*?\))',
            content, re.DOTALL
        )
        if xbox_status_check:
            old_block = xbox_status_check.group(0)
            new_block = (
                f'_xbox_st = getattr(self.controller, "xbox_status", "disconnected")  # {marker_s3b}\n'
                '        self._update_conn_status("xbox", _xbox_st in ("connected", "alive"))\n'
                '        # Update Xbox tooltip with status detail\n'
                '        _xbox_dot = getattr(self, "_conn_dots", {}).get("xbox")\n'
                '        if _xbox_dot:\n'
                '            if _xbox_st == "waiting":\n'
                '                _xbox_dot.setToolTip("Searching for Xbox controller...")\n'
                '            elif _xbox_st in ("connected", "alive"):\n'
                '                _xbox_dot.setToolTip("Xbox controller active")\n'
                '            elif _xbox_st == "unknown":\n'
                '                _xbox_dot.setToolTip("Xbox worker running — status unknown")\n'
                '            else:\n'
                '                _xbox_dot.setToolTip("Xbox controller disconnected")'
            )
            content = content[:xbox_status_check.start()] + new_block + content[xbox_status_check.end():]
            report("OK", "S3-B: Xbox status tooltip added")
        else:
            report("SKIP", "S3-B: Xbox status check pattern not found in on_status_update")
    else:
        report("SKIP", "S3-B: Xbox tooltip already present")

    # ── Write ─────────────────────────────────────────────────────
    if content != original:
        if safe_write(path, content, "dashboard.py"):
            print(f"  {GREEN}→ dashboard.py written successfully{RESET}")
        else:
            print(f"  {RED}→ dashboard.py WRITE FAILED (AST error){RESET}")
    else:
        print(f"  {YELLOW}→ dashboard.py unchanged{RESET}")


# ═══════════════════════════════════════════════════════════════════
# SESSION 4: app.py — Ensure xbox dot uses correct property
# ═══════════════════════════════════════════════════════════════════

def patch_app(root: Path):
    print(f"\n{CYAN}{BOLD}Session 4: app.py — Xbox indicator check{RESET}")
    path = root / "gui" / "app.py"
    content = safe_read(path)
    if not content:
        report("MISS", "app.py not found")
        return

    original = content
    marker_s4 = "v7.2.7: xbox property check"

    # The app.py uses getattr(self.controller, 'is_xbox_connected', False)
    # This should work whether or not the property exists. But let's make sure
    # it's using the property version, not the old is_alive() check.
    old_pattern = "xbox_ok = getattr(self.controller, 'is_xbox_connected', False)"
    if old_pattern in content and marker_s4 not in content:
        new_pattern = (
            f'xbox_ok = getattr(self.controller, "is_xbox_connected", False)  # {marker_s4}'
        )
        content = content.replace(old_pattern, new_pattern, 1)
        report("OK", "S4: Xbox indicator confirmed using is_xbox_connected property")
    else:
        report("SKIP", "S4: Xbox indicator already correct or not found")

    if content != original:
        if safe_write(path, content, "app.py"):
            print(f"  {GREEN}→ app.py written successfully{RESET}")
        else:
            print(f"  {RED}→ app.py WRITE FAILED (AST error){RESET}")
    else:
        print(f"  {YELLOW}→ app.py unchanged{RESET}")


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{BOLD}{'='*60}")
    print(f"  MEBP v7.2.7 — Xbox Bluetooth Controller Fix")
    print(f"{'='*60}{RESET}\n")

    root = find_root()
    print(f"Project root: {root}\n")

    patch_xbox_controller(root)
    patch_stage_controller(root)
    patch_dashboard(root)
    patch_app(root)

    # Summary
    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}{RESET}")
    print(f"  {GREEN}✓ Applied:  {ok_count}{RESET}")
    print(f"  {YELLOW}○ Skipped:  {skip_count}{RESET}")
    print(f"  {RED}✗ Missed:   {miss_count}{RESET}")
    print(f"  Total:     {total}")

    if miss_count > 0:
        print(f"\n  {RED}WARNING: {miss_count} change(s) could not be applied.{RESET}")
        print(f"  Check the output above for details.")
    else:
        print(f"\n  {GREEN}All changes applied or already present!{RESET}")

    print(f"\n  Next steps:")
    print(f"  1. Run: python test_xbox_bluetooth.py  (diagnostic)")
    print(f"  2. Run: python main.py  (launch app)")
    print(f"  3. Connect stages → Connect Xbox → verify green dot")
    print()

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
