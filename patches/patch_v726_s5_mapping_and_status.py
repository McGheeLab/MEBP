#!/usr/bin/env python3
"""
patch_v726_s5_mapping_and_status.py — v7.2.6 Session 5
Fixes BUG 10: mapping file path resolution.
- StageController.connect_xbox(): resolve to absolute path, store as _mapping_file
- dashboard._open_xbox_editor(): use resolved path
- XboxMappingEditor._load_mapping(): create default file if missing
- dashboard.on_status_update(): use xbox_status for richer dot display
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

# ══ S5-A: StageController.connect_xbox() absolute path ════════════
print("\n=== S5-A: StageController.connect_xbox() mapping path fix ===")
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

if "# v7.2.6: S5-A mapping path" not in content:
    # Replace connect_xbox method body to resolve absolute path
    new_cx = '''\
    def connect_xbox(self, mapping_file: str = "current_button_mapping.json") -> None:
        """Connect Xbox controller.
        v7.2.6: S5-A mapping path — resolves to absolute so subprocess finds same file.
        """
        if self.xbox_process and self.xbox_process.is_alive():
            logger.warning("Xbox already connected")
            return
        # v7.2.6: S5-A mapping path — resolve now so both worker and editor use same file
        from pathlib import Path as _Path
        self._mapping_file = str(_Path(mapping_file).resolve())
        self.xbox_queue = Queue()
        self.xbox_process = Process(
            target=xbox_polling_worker,
            args=(self.xbox_queue,),
            kwargs={"mapping_file": self._mapping_file},
            daemon=True,
        )
        self.xbox_process.start()
        self.xbox_poller = XboxQueuePoller(self.xbox_queue, self.processor)
        self.xbox_poller.start()
        logger.info(f"Xbox controller connected (mapping: {self._mapping_file})")

'''
    m = re.search(
        r'^    def connect_xbox\(self.*?(?=\n    def |\nclass |\Z)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        content = content[:m.start()] + new_cx + content[m.end():]
        print(f"{OK} Replaced connect_xbox with absolute path version")
    else:
        print(f"{MISS} connect_xbox not found in StageController")
        failed += 1

    # Also ensure _mapping_file is initialized in __init__ as None
    if '"_mapping_file"' not in content and "self._mapping_file = None" not in content:
        new_c, n = re.subn(
            r'(        # v7\.2: Hardware configuration.*?self\._hardware_config.*?None)',
            r'\1\n        # v7.2.6: Mapping file path (set on connect_xbox)\n        self._mapping_file: str = "current_button_mapping.json"',
            content, count=1, flags=re.DOTALL
        )
        if n:
            content = new_c
            print(f"{OK} Added _mapping_file default to StageController.__init__")
        # Non-critical if it misses — connect_xbox creates it dynamically

    safe_write(sc_path, content, "StageController.py S5-A")
else:
    print(f"{SKIP} S5-A already applied"); skipped += 1


# ══ S5-B: Dashboard _open_xbox_editor uses resolved path ══════════
print("\n=== S5-B: dashboard._open_xbox_editor() use resolved path ===")
dash_path = ROOT / "gui" / "pages" / "dashboard.py"
content_d = safe_read(dash_path)
if not content_d:
    print(f"{MISS} dashboard.py not found"); sys.exit(1)

if "# v7.2.6: S5-B mapping path" not in content_d:
    new_editor_fn = '''\
    def _open_xbox_editor(self):
        # v7.2.6: S5-B mapping path — use the resolved path from StageController
        from gui.widgets.xbox_mapping_editor import XboxMappingEditor
        mapping_path = getattr(
            self.controller, "_mapping_file", "current_button_mapping.json"
        )
        editor = XboxMappingEditor(mapping_file=mapping_path, parent=self)
        editor.exec()

'''
    m = re.search(
        r'^    def _open_xbox_editor\(self\).*?(?=\n    def |\nclass |\Z)',
        content_d, re.DOTALL | re.MULTILINE
    )
    if m:
        content_d = content_d[:m.start()] + new_editor_fn + content_d[m.end():]
        print(f"{OK} Replaced _open_xbox_editor with resolved-path version")
    else:
        print(f"{MISS} _open_xbox_editor not found in dashboard.py")
        failed += 1
else:
    print(f"{SKIP} S5-B already applied")

# ══ S5-D: Dashboard on_status_update uses xbox_status ═════════════
print("\n=== S5-D: dashboard on_status_update xbox_status display ===")

if "# v7.2.6: S5-D xbox status" not in content_d:
    # Find the section in on_status_update where Xbox connection is checked
    # Current code likely calls: self._update_conn_status("xbox", self.controller.is_xbox_connected)
    # Replace it with the richer status check
    new_c, n = re.subn(
        r'self\._update_conn_status\("xbox",\s*self\.controller\.is_xbox_connected\)',
        (
            '# v7.2.6: S5-D xbox status — use rich status\n'
            '        _xbox_st = getattr(self.controller, "xbox_status", "disconnected")\n'
            '        self._update_conn_status("xbox", _xbox_st in ("connected", "alive"))'
        ),
        content_d
    )
    if n:
        content_d = new_c
        print(f"{OK} Updated on_status_update to use xbox_status property")
    else:
        # Try alternate pattern: is_xbox_connected anywhere in on_status_update
        new_c2, n2 = re.subn(
            r'(self\.controller\.is_xbox_connected)',
            '(getattr(self.controller, "xbox_status", "disconnected") in ("connected", "alive"))  # v7.2.6: S5-D xbox status',
            content_d
        )
        if n2:
            content_d = new_c2
            print(f"{OK} Updated is_xbox_connected references to xbox_status (fallback)")
        else:
            print(f"{MISS} Could not find is_xbox_connected usage in dashboard.py")
            # Non-fatal — dashboard still works, just less rich display

safe_write(dash_path, content_d, "dashboard.py S5-B/D")


# ══ S5-C: XboxMappingEditor._load_mapping creates default file ════
print("\n=== S5-C: XboxMappingEditor creates default mapping file ===")
editor_path = ROOT / "gui" / "widgets" / "xbox_mapping_editor.py"
content_e = safe_read(editor_path)
if not content_e:
    print(f"{MISS} xbox_mapping_editor.py not found"); sys.exit(1)

if "# v7.2.6: S5-C create default" not in content_e:
    new_load = '''\
    def _load_mapping(self) -> dict:
        """Load current mapping from file.
        v7.2.6: S5-C create default — creates file with defaults if missing.
        """
        # v7.2.6: S5-C create default mapping file if it doesn't exist
        path = Path(self.mapping_file)
        if not path.exists():
            defaults = {
                "buttons": {
                    "0": "zero_needle_pos",
                    "1": "None", "2": "None", "3": "None",
                    "4": "increment_zspeed_down",
                    "5": "increment_zspeed_up",
                    "6": "increment_pspeed_down",
                    "7": "increment_pspeed_up",
                    "8": "increment_xyspeed_down",
                    "9": "increment_xyspeed_up",
                    "10": "None", "11": "None",
                },
                "axes": {
                    "0-1": "move_stage_at_velocity",
                    "2-3": "move_z_at_velocity",
                    "4": "move_p3_at_velocity",
                    "5": "move_p3_at_velocity",
                },
                "dpad": {
                    "up": "increment_zspeed_up",
                    "down": "increment_zspeed_down",
                    "left": "increment_pspeed_down",
                    "right": "increment_pspeed_up",
                },
            }
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                with open(path, "w") as f:
                    import json as _json
                    _json.dump(defaults, f, indent=4)
                logger.info(f"Created default mapping file: {path}")
            except Exception as e:
                logger.warning(f"Could not create default mapping: {e}")
            return defaults

        try:
            with open(self.mapping_file, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Failed to load mapping: {e}")
            return {"buttons": {}, "axes": {}, "dpad": {}}

'''
    m = re.search(
        r'^    def _load_mapping\(self\).*?(?=\n    def |\nclass |\Z)',
        content_e, re.DOTALL | re.MULTILINE
    )
    if m:
        content_e = content_e[:m.start()] + new_load + content_e[m.end():]
        print(f"{OK} Replaced _load_mapping with default-creating version")
        safe_write(editor_path, content_e, "xbox_mapping_editor.py S5-C")
    else:
        print(f"{MISS} _load_mapping not found in xbox_mapping_editor.py")
        failed += 1
else:
    print(f"{SKIP} S5-C already applied"); skipped += 1

print(f"\n{'='*50}")
print(f"S5 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
