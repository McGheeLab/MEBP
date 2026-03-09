#!/usr/bin/env python3
"""
patch_v726_s4s5_fix.py — v7.2.6 Sessions 4+5 FIX for StageController.py + dashboard.py
Applies S4-D, S4-E, S5-A, S5-D correctly.

Root causes of previous failures:
  S4-D: DOTALL regex matched self._thread in wrong class (line 52), inserting
        code at module level. Fix: replace XboxQueuePoller.__init__ entirely.
  S5-A: _mapping_file injection used raw string r'\\1\\n...' — \\n is literal
        backslash-n, not newline. Fix: use string slicing instead of subn.
  S5-D: Dashboard uses getattr(self.controller, 'is_xbox_connected', False) —
        old pattern didn't match. Fix: target the exact getattr call.

XboxController.py (S4-A/B/C) and xbox_mapping_editor.py (S5-C)
already applied successfully — not touched here.
dashboard._open_xbox_editor (S5-B) already applied — not touched here.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; YELLOW = "\033[93m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; SKIP = f"{YELLOW}  o{RESET}"; MISS = f"{RED}  x{RESET}"
applied = 0; skipped = 0; failed = 0

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit(f"{RED}ERROR: Cannot find MEBP project root{RESET}")

def safe_read(path):
    if not path.exists():
        print(f"{MISS} File not found: {path}")
        return ""
    return path.read_text(encoding="utf-8")

def safe_write(path, content, label):
    global applied, failed
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"{MISS} AST FAIL in {label}: {e}")
        failed += 1
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v726_{ts}"))
    path.write_text(content, encoding="utf-8")
    applied += 1
    print(f"{OK} Written: {label}")
    return True

def find_class_section(content, class_name, next_class_name):
    m_start = re.search(rf'^class {re.escape(class_name)}\b', content, re.MULTILINE)
    m_end   = re.search(rf'^class {re.escape(next_class_name)}\b', content, re.MULTILINE)
    if m_start and m_end:
        return m_start.start(), m_end.start()
    return None, None

ROOT = find_root()

# ════════════════════════════════════════════════════════════════════
# StageController.py — S4-D, S4-E, S5-A
# ════════════════════════════════════════════════════════════════════
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

# ══ S4-D (FIX): Replace XboxQueuePoller.__init__ entirely ════════
print("\n=== S4-D FIX: XboxQueuePoller.__init__ with _xbox_status ===")

if "# v7.2.6: S4-D status handler" not in content:
    # Find XboxQueuePoller class section
    xqp_s, zp_s = find_class_section(content, "XboxQueuePoller", "ZPJogHandler")
    if xqp_s is None:
        print(f"{MISS} XboxQueuePoller class not found"); failed += 1
    else:
        xqp_section = content[xqp_s:zp_s]
        # Replace the entire __init__ method
        new_init = '''\
    def __init__(self, queue: Queue, processor: Processor):
        self.queue = queue
        self.processor = processor
        self._running = False
        self._thread: threading.Thread | None = None
        # v7.2.6: S4-D status handler — track controller state from worker
        self._xbox_status: str = "disconnected"

'''
        m = re.search(
            r'    def __init__\(self, queue: Queue, processor: Processor\).*?(?=\n    def )',
            xqp_section, re.DOTALL
        )
        if m:
            abs_s = xqp_s + m.start()
            abs_e = xqp_s + m.end()
            content = content[:abs_s] + new_init + content[abs_e:]
            print(f"{OK} Replaced XboxQueuePoller.__init__ with _xbox_status version")
        else:
            print(f"{MISS} XboxQueuePoller.__init__ not found in class section"); failed += 1

    # Also replace _poll_loop to handle 'status' messages
    xqp_s2, zp_s2 = find_class_section(content, "XboxQueuePoller", "ZPJogHandler")
    if xqp_s2 is not None:
        xqp_section2 = content[xqp_s2:zp_s2]
        new_poll = '''\
    def _poll_loop(self) -> None:
        while self._running:
            while not self.queue.empty():
                try:
                    msg = self.queue.get_nowait()
                except Exception:
                    break

                if "status" in msg:
                    # v7.2.6: S4-D — store Xbox worker status for StageController
                    self._xbox_status = msg["status"]
                    logger.info(f"[Xbox] Status: {self._xbox_status}")
                elif "debug" in msg:
                    text = msg["debug"]
                    if "connect" in text.lower() or "found" in text.lower():
                        logger.info(f"[Xbox] {text}")
                elif "button" in msg:
                    self.processor.add_command(msg["command"], button=msg["button"])
                elif "axis" in msg:
                    self.processor.add_command(
                        msg["command"], axis=msg["axis"], average=msg["average"]
                    )
                elif "dpad" in msg:
                    self.processor.add_command(msg["command"], direction=msg["dpad"])

            time.sleep(0.02)

'''
        m2 = re.search(
            r'    def _poll_loop\(self\).*?(?=\n    def |\nclass |\Z)',
            xqp_section2, re.DOTALL | re.MULTILINE
        )
        if m2:
            abs_s2 = xqp_s2 + m2.start()
            abs_e2 = xqp_s2 + m2.end()
            content = content[:abs_s2] + new_poll + content[abs_e2:]
            print(f"{OK} Replaced XboxQueuePoller._poll_loop with status-aware version")
        else:
            print(f"{MISS} XboxQueuePoller._poll_loop not found"); failed += 1
else:
    print(f"{SKIP} S4-D already applied"); skipped += 1


# ══ S4-E: Add xbox_status + is_xbox_connected properties ═════════
print("\n=== S4-E: StageController xbox_status property ===")

if "# v7.2.6: S4-E xbox_status" not in content:
    xbox_props = (
        "\n"
        "    # v7.2.6: S4-E xbox_status property\n"
        "    @property\n"
        "    def xbox_status(self) -> str:\n"
        '        """Return Xbox connection status string.\n'
        "\n"
        "        Returns: 'disconnected', 'waiting', 'connected', or 'alive'.\n"
        "        'waiting'   = worker running, searching for controller\n"
        "        'connected' = controller just found\n"
        "        'alive'     = controller confirmed active (heartbeat)\n"
        '        """\n'
        "        if self.xbox_poller is None:\n"
        '            return "disconnected"\n'
        '        return getattr(self.xbox_poller, "_xbox_status", "unknown")\n'
        "\n"
        "    @property\n"
        "    def is_xbox_connected(self) -> bool:\n"
        '        """Backward-compatible bool: True when controller is active."""\n'
        '        return self.xbox_status in ("connected", "alive")\n'
        "\n"
    )
    # Inject after disconnect_xbox method
    m = re.search(
        r'    def disconnect_xbox\(self\).*?(?=\n    def |\nclass |\Z)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        content = content[:m.end()] + xbox_props + content[m.end():]
        print(f"{OK} Added xbox_status + is_xbox_connected properties")
    else:
        print(f"{MISS} disconnect_xbox not found"); failed += 1
else:
    print(f"{SKIP} S4-E already applied"); skipped += 1


# ══ S5-A (FIX): connect_xbox absolute path ═══════════════════════
print("\n=== S5-A FIX: connect_xbox() absolute mapping path ===")

if "# v7.2.6: S5-A mapping path" not in content:
    new_cx = (
        "    def connect_xbox(self, mapping_file: str = \"current_button_mapping.json\") -> None:\n"
        "        \"\"\"Connect Xbox controller.\n"
        "        v7.2.6: S5-A mapping path — resolves to absolute so subprocess finds same file.\n"
        "        \"\"\"\n"
        "        if self.xbox_process and self.xbox_process.is_alive():\n"
        "            logger.warning(\"Xbox already connected\")\n"
        "            return\n"
        "        # v7.2.6: S5-A mapping path — resolve now so worker and editor use same file\n"
        "        from pathlib import Path as _Path\n"
        "        self._mapping_file = str(_Path(mapping_file).resolve())\n"
        "        # v7.2.6: connect order warning\n"
        "        if self.xy_stage is None and self.zp_stage is None:\n"
        "            logger.warning(\n"
        "                \"Xbox connected but no stages are connected -- \"\n"
        "                \"controller input will have no effect until stages connect\"\n"
        "            )\n"
        "        self.xbox_queue = Queue()\n"
        "        self.xbox_process = Process(\n"
        "            target=xbox_polling_worker,\n"
        "            args=(self.xbox_queue,),\n"
        "            kwargs={\"mapping_file\": self._mapping_file},\n"
        "            daemon=True,\n"
        "        )\n"
        "        self.xbox_process.start()\n"
        "        self.xbox_poller = XboxQueuePoller(self.xbox_queue, self.processor)\n"
        "        self.xbox_poller.start()\n"
        "        logger.info(f\"Xbox controller connected (mapping: {self._mapping_file})\")\n"
        "\n"
    )
    m = re.search(
        r'^    def connect_xbox\(self.*?(?=\n    def |\nclass |\Z)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        content = content[:m.start()] + new_cx + content[m.end():]
        print(f"{OK} Replaced connect_xbox with absolute-path version (S5-A)")
        # Note: this replaces the S3 connect-order warning too — new_cx includes it inline
    else:
        print(f"{MISS} connect_xbox not found"); failed += 1
else:
    print(f"{SKIP} S5-A already applied"); skipped += 1

# Write StageController.py
safe_write(sc_path, content, "StageController.py S4+S5")


# ════════════════════════════════════════════════════════════════════
# dashboard.py — S5-D only (S5-B already applied)
# ════════════════════════════════════════════════════════════════════
print("\n=== S5-D: dashboard on_status_update xbox_status display ===")
dash_path = ROOT / "gui" / "pages" / "dashboard.py"
content_d = safe_read(dash_path)
if not content_d:
    print(f"{MISS} dashboard.py not found"); sys.exit(1)

if "# v7.2.6: S5-D xbox status" not in content_d:
    # The actual pattern in dashboard is:
    # self._update_conn_status("xbox", getattr(self.controller, 'is_xbox_connected', False))
    old_line = "self._update_conn_status(\"xbox\", getattr(self.controller, 'is_xbox_connected', False))"
    new_lines = (
        "# v7.2.6: S5-D xbox status — use rich status property\n"
        "        _xbox_st = getattr(self.controller, \"xbox_status\", \"disconnected\")\n"
        "        self._update_conn_status(\"xbox\", _xbox_st in (\"connected\", \"alive\"))"
    )
    if old_line in content_d:
        content_d = content_d.replace(old_line, new_lines)
        print(f"{OK} Updated on_status_update to use xbox_status property")
        safe_write(dash_path, content_d, "dashboard.py S5-D")
    else:
        print(f"{MISS} Expected xbox line not found in dashboard.py")
        print(f"       Looking for: {old_line[:80]}...")
        # Try the single-quote variant
        old_line2 = "self._update_conn_status(\"xbox\", getattr(self.controller, \"is_xbox_connected\", False))"
        if old_line2 in content_d:
            content_d = content_d.replace(old_line2, new_lines)
            print(f"{OK} Updated on_status_update (double-quote variant)")
            safe_write(dash_path, content_d, "dashboard.py S5-D")
        else:
            print(f"       Also tried double-quote variant — not found")
            print(f"       S5-D is non-fatal: dashboard will use is_xbox_connected bool")
            skipped += 1
else:
    print(f"{SKIP} S5-D already applied"); skipped += 1

print(f"\n{'='*50}")
print(f"S4+S5 Fix Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
