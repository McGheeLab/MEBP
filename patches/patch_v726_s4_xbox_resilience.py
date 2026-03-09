#!/usr/bin/env python3
"""
patch_v726_s4_xbox_resilience.py — v7.2.6 Session 4
Fixes BUGs 7, 8, 9: Xbox worker retry/reconnect/heartbeat.
- xbox_polling_worker: retry loop instead of early exit, crash recovery, heartbeat
- XboxQueuePoller: handles 'status' key, stores _xbox_status
- StageController: xbox_status property (rich) + backward-compat is_xbox_connected bool
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

ROOT = find_root()

# ══ S4-A/B/C: Replace xbox_polling_worker with resilient version ══
print("\n=== S4-A/B/C: xbox_polling_worker retry/reconnect/heartbeat ===")
xc_path = ROOT / "SupportClasses" / "XboxController.py"
content = safe_read(xc_path)
if not content:
    sys.exit(1)

if "# v7.2.6: S4 resilient worker" not in content:
    # Replace the entire xbox_polling_worker function with the resilient version.
    # We find from 'def xbox_polling_worker(' to EOF or next top-level def.
    new_worker = '''
def xbox_polling_worker(
    out_queue: Queue,
    mapping_file: str = "current_button_mapping.json",
    avg_interval: float = 0.5,
    deadzone: float = 0.2,
) -> None:
    """
    Main polling loop — runs in a separate process.
    v7.2.6: S4 resilient worker — retry loop on startup, crash recovery,
    periodic heartbeat. Worker never exits; reconnects automatically.

    Args:
        out_queue:     Multiprocessing queue for outbound messages.
        mapping_file:  Path to the button mapping JSON file.
        avg_interval:  Seconds between averaged axis updates.
        deadzone:      Axis deadzone threshold (0–1).
    """
    try:
        import pygame
    except ImportError:
        out_queue.put({"debug": "pygame not installed -- Xbox controller unavailable"})
        return

    pygame.init()

    # Load initial mapping
    mapping = load_xbox_mapping(mapping_file)

    def _init_accumulators(js):
        """Build fresh axis accumulators for a joystick."""
        n = js.get_numaxes()
        return (
            {i: 0.0 for i in range(n)},
            {i: 0   for i in range(n)},
        )

    def _find_controller():
        """Retry until a controller is found. Sends status messages."""
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                out_queue.put({"debug": f"Controller connected: {js.get_name()}"})
                out_queue.put({"status": "connected"})
                return js
            else:
                out_queue.put({"debug": "No controller found, retrying in 2s..."})
                out_queue.put({"status": "waiting"})
                time.sleep(2.0)

    # ── Initial controller acquisition ────────────────────────────
    joystick = _find_controller()

    axis_accum, axis_count = _init_accumulators(joystick)
    num_axes = joystick.get_numaxes()
    last_axis_time   = time.time()
    last_mapping_time = time.time()
    last_heartbeat   = time.time()
    last_hat = (0, 0)
    last_sent: dict = {}

    axis_groups = [
        {"name": "0-1", "axes": [0, 1], "type": "axis"},
        {"name": "2-3", "axes": [2, 3], "type": "axis"},
        {"name": "4",   "axes": [4],    "type": "trigger"},
        {"name": "5",   "axes": [5],    "type": "trigger"},
    ]

    # ── Main Loop ─────────────────────────────────────────────────
    while True:
        current_time = time.time()

        try:
            pygame.event.pump()

            # Hot-reload mapping every 5 seconds
            if current_time - last_mapping_time >= 5:
                mapping = load_xbox_mapping(mapping_file)
                last_mapping_time = current_time

            # Heartbeat every 3 seconds
            if current_time - last_heartbeat >= 3.0:
                out_queue.put({"status": "alive"})
                last_heartbeat = current_time

            # ── Button Presses ─────────────────────────────────────
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    mapped_func = mapping.get("buttons", {}).get(str(i))
                    if mapped_func and mapped_func != "None":
                        out_queue.put({"button": i, "command": mapped_func})

            # ── Axis Accumulation ──────────────────────────────────
            for i in range(num_axes):
                raw = joystick.get_axis(i)
                if abs(raw) > deadzone:
                    axis_accum[i] += raw
                    axis_count[i] += 1

            if current_time - last_axis_time >= avg_interval:
                for group in axis_groups:
                    axes = group["axes"]
                    cmd = mapping.get("axes", {}).get(group["name"])
                    if not cmd or cmd == "None":
                        for a in axes:
                            axis_accum[a] = 0.0; axis_count[a] = 0
                        continue

                    if group["type"] == "axis":
                        counts = [axis_count[a] for a in axes]
                        total = sum(counts)
                        if total > 0:
                            avgs = [axis_accum[a] / axis_count[a] if axis_count[a] else 0.0
                                    for a in axes]
                        else:
                            avgs = [0.0] * len(axes)
                        avg_val = tuple(avgs)
                    else:
                        a = axes[0]
                        avg_val = axis_accum[a] / axis_count[a] if axis_count[a] else 0.0

                    zero_value = (0.0, 0.0) if group["type"] == "axis" else 0.0
                    prev = last_sent.get(group["name"], zero_value)

                    # Only send if changed or non-zero
                    if avg_val != zero_value or prev != zero_value:
                        out_queue.put({
                            "axis":    group["name"],
                            "average": avg_val,
                            "command": cmd,
                        })
                        last_sent[group["name"]] = avg_val

                    for a in axes:
                        axis_accum[a] = 0.0; axis_count[a] = 0
                last_axis_time = current_time

            # ── D-Pad (Hat) ────────────────────────────────────────
            if joystick.get_numhats() > 0:
                current_hat = joystick.get_hat(0)
                if current_hat != last_hat:
                    dpad_map = mapping.get("dpad", {})
                    if current_hat[1] == 1:
                        cmd = dpad_map.get("up")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "up", "command": cmd})
                    elif current_hat[1] == -1:
                        cmd = dpad_map.get("down")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "down", "command": cmd})
                    if current_hat[0] == 1:
                        cmd = dpad_map.get("right")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "right", "command": cmd})
                    elif current_hat[0] == -1:
                        cmd = dpad_map.get("left")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "left", "command": cmd})
                    last_hat = current_hat

            time.sleep(0.02)

        except Exception as e:
            # v7.2.6: Crash recovery — reconnect instead of dying
            out_queue.put({"debug": f"Controller error: {e}"})
            out_queue.put({"status": "disconnected"})
            joystick = _find_controller()
            # Re-init accumulators for the new joystick
            axis_accum, axis_count = _init_accumulators(joystick)
            num_axes = joystick.get_numaxes()
            last_axis_time   = time.time()
            last_mapping_time = time.time()
            last_heartbeat   = time.time()
            last_hat = (0, 0)
            last_sent = {}

'''
    # Find the existing xbox_polling_worker function
    m = re.search(
        r'^def xbox_polling_worker\(.*?(?=^\ndef |\Z)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        content = content[:m.start()] + new_worker + content[m.end():]
        print(f"{OK} Replaced xbox_polling_worker with resilient version")
        safe_write(xc_path, content, "XboxController.py S4-A/B/C")
    else:
        print(f"{MISS} xbox_polling_worker function not found in XboxController.py")
        failed += 1
else:
    print(f"{SKIP} S4-A/B/C already applied"); skipped += 1


# ══ S4-D: XboxQueuePoller handles 'status' key ════════════════════
print("\n=== S4-D: XboxQueuePoller status message handling ===")
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

if "# v7.2.6: S4-D status handler" not in content:
    # Find XboxQueuePoller class and replace _poll_loop
    # New _poll_loop stores _xbox_status from "status" messages
    new_init_addition = '''\
    def __init__(self, queue: Queue, processor: Processor):
        self.queue = queue
        self.processor = processor
        self._running = False
        self._thread: "threading.Thread | None" = None
        # v7.2.6: S4-D status handler — track controller state
        self._xbox_status: str = "disconnected"

'''
    # Replace XboxQueuePoller.__init__
    m = re.search(
        r'^class XboxQueuePoller\b.*?(?=\n    def start)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        # Just inject the _xbox_status line into existing __init__
        # Find the __init__ method inside XboxQueuePoller
        init_m = re.search(
            r'(    def __init__\(self, queue: Queue, processor: Processor\):.*?)'
            r'(        self\._thread)',
            content, re.DOTALL
        )
        if init_m:
            inject = (
                "        # v7.2.6: S4-D status handler — track controller state\n"
                "        self._xbox_status: str = \"disconnected\"\n"
                "        "
            )
            content = content[:init_m.end(1)] + "\n" + inject + content[init_m.start(2):]
            print(f"{OK} Added _xbox_status to XboxQueuePoller.__init__")
        else:
            print(f"{MISS} XboxQueuePoller.__init__ pattern not matched")
            failed += 1
    else:
        print(f"{MISS} XboxQueuePoller class not found")
        failed += 1

    # Now update _poll_loop to handle "status" messages
    new_poll = '''\
    def _poll_loop(self) -> None:
        while self._running:
            while not self.queue.empty():
                try:
                    msg = self.queue.get_nowait()
                except Exception:
                    break

                if "status" in msg:
                    # v7.2.6: S4-D — store Xbox worker status
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
    # Find XboxQueuePoller class boundaries
    xqp_start = re.search(r'^class XboxQueuePoller\b', content, re.MULTILINE)
    zp_jog_start = re.search(r'^class ZPJogHandler\b', content, re.MULTILINE)
    if xqp_start and zp_jog_start:
        xqp_section = content[xqp_start.start():zp_jog_start.start()]
        poll_m = re.search(
            r'    def _poll_loop\(self\).*?(?=\n    def |\nclass |\Z)',
            xqp_section, re.DOTALL | re.MULTILINE
        )
        if poll_m:
            abs_s = xqp_start.start() + poll_m.start()
            abs_e = xqp_start.start() + poll_m.end()
            content = content[:abs_s] + new_poll + content[abs_e:]
            print(f"{OK} Replaced XboxQueuePoller._poll_loop with status-aware version")
        else:
            print(f"{MISS} _poll_loop not found in XboxQueuePoller")
            failed += 1
    else:
        print(f"{MISS} XboxQueuePoller/ZPJogHandler class boundaries not found")
        failed += 1
else:
    print(f"{SKIP} S4-D already applied"); skipped += 1


# ══ S4-E: StageController xbox_status property ════════════════════
print("\n=== S4-E: StageController xbox_status property ===")

if "# v7.2.6: S4-E xbox_status property" not in content:
    xbox_props = '''\
    # v7.2.6: S4-E xbox_status property — richer than simple bool
    @property
    def xbox_status(self) -> str:
        """Return Xbox connection status string.

        Returns: 'disconnected', 'waiting', 'connected', or 'alive'.
        'waiting'   = worker process running, searching for controller
        'connected' = controller just found
        'alive'     = controller confirmed active (heartbeat)
        """
        if self.xbox_poller is None:
            return "disconnected"
        return getattr(self.xbox_poller, "_xbox_status", "unknown")

    @property
    def is_xbox_connected(self) -> bool:
        """Backward-compatible bool: True when controller is active."""
        return self.xbox_status in ("connected", "alive")

'''
    # Inject before connect_stages or after disconnect_xbox
    # Best: inject after disconnect_xbox method
    m = re.search(
        r'(    def disconnect_xbox\(self\).*?(?=\n    def |\nclass |\Z))',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        insert_at = m.end()
        content = content[:insert_at] + "\n" + xbox_props + content[insert_at:]
        print(f"{OK} Added xbox_status + is_xbox_connected properties to StageController")
    else:
        print(f"{MISS} disconnect_xbox method not found in StageController")
        failed += 1
else:
    print(f"{SKIP} S4-E already applied"); skipped += 1

safe_write(sc_path, content, "StageController.py S4-D/E")

print(f"\n{'='*50}")
print(f"S4 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
