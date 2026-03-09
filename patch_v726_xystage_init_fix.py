#!/usr/bin/env python3
"""
patch_v726_xystage_init_fix.py — Surgical fix for XYStage.__init__ only.
Previous attempts left an empty else-body causing SyntaxError at line 117.
Strategy: Replace the entire __init__ method with a known-correct version
that has _serial_lock initialised at the top before any serial I/O.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; MISS = f"{RED}  x{RESET}"

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit(f"{RED}ERROR: Cannot find MEBP project root{RESET}")

ROOT = find_root()
xy_path = ROOT / "SupportClasses" / "XYStage.py"
content = xy_path.read_text(encoding="utf-8")

# ── Find the __init__ method boundary ────────────────────────────
# It starts at 'def __init__(' and ends just before 'def _load_protocol('
m_start = re.search(r'^    def __init__\(', content, re.MULTILINE)
m_end   = re.search(r'^    # ── Protocol Loading', content, re.MULTILINE)
if not m_start or not m_end:
    print(f"{MISS} Cannot locate __init__ / Protocol Loading boundary")
    sys.exit(1)

# ── Build the correct __init__ ────────────────────────────────────
new_init = '''\
    def __init__(
        self,
        simulate: bool = False,
        settings: Optional[dict] = None,
        controller_json: Optional[str] = None,
    ):
        self.simulate = simulate
        self._protocol: Optional[ControllerProtocol] = None
        self._detected_controller: Optional[str] = None

        # Stage parameters (may be overridden by protocol or settings)
        self.max_speed: int = self.DEFAULT_MAX_SPEED
        self.min_jerk: int = 1
        self.max_jerk: int = 100
        self.min_acceleration: int = 1
        self.max_acceleration: int = 100
        self.x_range: list[int] = [-100_000, 100_000]
        self.y_range: list[int] = [-100_000, 100_000]
        self.default_acceleration: int = self.DEFAULT_ACCELERATION
        self.default_velocity: int = self.DEFAULT_VELOCITY

        # v7.2.6: serial lock — must be created BEFORE _initialise_serial()
        # because send_command() acquires it during port detection
        self._serial_lock = threading.RLock()

        # P8.17: Load controller protocol JSON
        # BUG-3 FIX (v7.1.2): Always load protocol, even in sim mode,
        # so parameters like microsteps_per_micron are accessible.
        if controller_json is not None:
            self._load_protocol(controller_json)
            self._apply_protocol_parameters()
        elif not simulate:
            # Real hardware without explicit protocol → try default/auto-detect
            self._load_protocol(controller_json)
            self._apply_protocol_parameters()

        # Apply any user-provided settings (override protocol defaults)
        if settings:
            self._apply_settings(settings)

        # Initialise the communication backend
        if self.simulate:
            self.spo = XYStageSimulator()
            # BUG-3 FIX: Configure simulator with protocol-derived parameters
            if self._protocol:
                params = self._protocol._config.get("parameters", {})
                sim_max_speed = params.get("max_speed", 100000)
                sim_accel = params.get("acceleration", 200000)
                if hasattr(self.spo, 'configure_from_protocol'):
                    self.spo.configure_from_protocol(
                        max_speed=float(sim_max_speed),
                        acceleration=float(sim_accel),
                    )
            self.spo.start()
            logger.info("XY stage simulator started")
        else:
            self.spo = self._initialise_serial()

'''

content = content[:m_start.start()] + new_init + content[m_end.start():]

# ── AST verify + write ────────────────────────────────────────────
try:
    ast.parse(content)
except SyntaxError as e:
    print(f"{MISS} AST FAIL: {e}")
    sys.exit(1)

ts = datetime.now().strftime("%Y%m%d_%H%M%S")
shutil.copy2(xy_path, xy_path.with_suffix(f".bak_v726_{ts}"))
xy_path.write_text(content, encoding="utf-8")
print(f"{OK} XYStage.__init__ rewritten cleanly — _serial_lock before _initialise_serial()")
