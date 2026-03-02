#!/usr/bin/env python3
"""
fix_stage_controller_connect.py — Add connect_xy/zp/disconnect_xy/zp convenience methods.

Dashboard calls connect_xy(), connect_zp(), disconnect_xy(), disconnect_zp()
but StageController only has connect_stages(xy, zp) and disconnect_stages().

Run from MEBP project root:
    python fix_stage_controller_connect.py
"""

import os, sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

path = "SupportClasses/StageController.py"
if not os.path.isfile(path):
    print(f"{RED}ERROR{RESET}: {path} not found"); sys.exit(1)

with open(path) as f:
    content = f.read()

# Check if already fixed
if "def connect_xy(self)" in content:
    print(f"{GREEN}Already fixed{RESET}: connect_xy already exists"); sys.exit(0)

# Find the anchor: after connect_stages method
anchor = '        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)'

if anchor not in content:
    print(f"{RED}FAIL{RESET}: Could not find connect_stages anchor")
    sys.exit(1)

# Find the end of connect_stages by looking for the next method after the anchor
idx = content.index(anchor) + len(anchor)
# Skip to end of line
while idx < len(content) and content[idx] != '\n':
    idx += 1

convenience_methods = '''

    # ── Convenience connection methods (used by Dashboard) ────────

    def connect_xy(self) -> None:
        """Connect only the XY stage."""
        self.connect_stages(xy=True, zp=False)

    def connect_zp(self) -> None:
        """Connect only the ZP stage."""
        self.connect_stages(xy=False, zp=True)

    def disconnect_xy(self) -> None:
        """Disconnect only the XY stage."""
        if self.xy_jog:
            self.xy_jog.stop()
            self.xy_jog = None
        if self.xy_stage:
            self.xy_stage.stop()
            self.xy_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        self._watchdog.unwatch("XY")
        logger.info("XY stage disconnected")

    def disconnect_zp(self) -> None:
        """Disconnect only the ZP stage."""
        if self.zp_jog:
            self.zp_jog.stop()
            self.zp_jog = None
        if self.zp_stage:
            self.zp_stage.stop()
            self.zp_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        self._watchdog.unwatch("ZP")
        logger.info("ZP stage disconnected")

    @property
    def is_xy_connected(self) -> bool:
        """True if XY stage is initialized."""
        return self.xy_stage is not None

    @property
    def is_zp_connected(self) -> bool:
        """True if ZP stage is initialized."""
        return self.zp_stage is not None

    @property
    def is_xbox_connected(self) -> bool:
        """True if Xbox controller process is running."""
        return self.xbox_process is not None and self.xbox_process.is_alive()'''

# Check if is_xy_connected already exists (just need connect/disconnect)
if "is_xy_connected" in content:
    # Only add connect/disconnect, not properties
    convenience_methods = '''

    # ── Convenience connection methods (used by Dashboard) ────────

    def connect_xy(self) -> None:
        """Connect only the XY stage."""
        self.connect_stages(xy=True, zp=False)

    def connect_zp(self) -> None:
        """Connect only the ZP stage."""
        self.connect_stages(xy=False, zp=True)

    def disconnect_xy(self) -> None:
        """Disconnect only the XY stage."""
        if self.xy_jog:
            self.xy_jog.stop()
            self.xy_jog = None
        if self.xy_stage:
            self.xy_stage.stop()
            self.xy_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        self._watchdog.unwatch("XY")
        logger.info("XY stage disconnected")

    def disconnect_zp(self) -> None:
        """Disconnect only the ZP stage."""
        if self.zp_jog:
            self.zp_jog.stop()
            self.zp_jog = None
        if self.zp_stage:
            self.zp_stage.stop()
            self.zp_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        self._watchdog.unwatch("ZP")
        logger.info("ZP stage disconnected")'''

content = content[:idx] + convenience_methods + content[idx:]

with open(path, "w") as f:
    f.write(content)

print(f"{GREEN}OK{RESET}: Added connect_xy/zp + disconnect_xy/zp to StageController")
