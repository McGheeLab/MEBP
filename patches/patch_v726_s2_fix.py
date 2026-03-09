#!/usr/bin/env python3
"""
patch_v726_s2_fix.py — v7.2.6 Session 2 (FIX)
Re-applies all S2 changes correctly. Previous attempt failed because:
  - S2-A: ZPJogHandler _registered_handlers anchor used wrong handler name
  - S2-C: ZP try/except injection mangled multi-line move_relative call
Strategy: use correct anchors and full method replacement for _jog_loop.
Safe to run even if S2 was partially applied in memory (never written).
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
    """Return (start, end) indices of a class body."""
    m_start = re.search(rf'^class {re.escape(class_name)}\b', content, re.MULTILINE)
    m_end   = re.search(rf'^class {re.escape(next_class_name)}\b', content, re.MULTILINE)
    if m_start and m_end:
        return m_start.start(), m_end.start()
    return None, None

ROOT = find_root()
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

# ══ S2-A (FIX): ZPJogHandler _registered_handlers ════════════════
print("\n=== S2-A FIX: ZPJogHandler _registered_handlers ===")

if "# v7.2.6: ZP registered handlers" not in content:
    # Anchor on the LAST register_handler call in ZPJogHandler.__init__
    # which is: self.processor.register_handler("increment_pspeed_down", self._incr_p_down)
    inject_code = (
        "\n        # v7.2.6: ZP registered handlers — track for unregister on stop()\n"
        "        self._registered_handlers: list = [\n"
        '            ("move_z_at_velocity",       self._handle_z_vel),\n'
        '            ("move_p1_at_velocity",      self._handle_p1_vel),\n'
        '            ("move_p2_at_velocity",      self._handle_p2_vel),\n'
        '            ("move_p3_at_velocity",      self._handle_p3_vel),\n'
        '            ("increment_zspeed_up",      self._incr_z_up),\n'
        '            ("increment_zspeed_down",    self._incr_z_down),\n'
        '            ("increment_pspeed_up",      self._incr_p_up),\n'
        '            ("increment_pspeed_down",    self._incr_p_down),\n'
        "        ]\n"
    )
    # Find the last register_handler line within ZPJogHandler class section
    zp_start, xy_start = find_class_section(content, "ZPJogHandler", "XYJogHandler")
    if zp_start is not None:
        zp_section = content[zp_start:xy_start]
        # Find the last register_handler in this section
        last_m = None
        for m in re.finditer(
            r'        self\.processor\.register_handler\("increment_pspeed_down",.*?\n',
            zp_section
        ):
            last_m = m
        if last_m:
            abs_end = zp_start + last_m.end()
            content = content[:abs_end] + inject_code + content[abs_end:]
            print(f"{OK} Added _registered_handlers list in ZPJogHandler.__init__")
        else:
            print(f"{MISS} increment_pspeed_down register_handler not found in ZPJogHandler")
            failed += 1
    else:
        print(f"{MISS} ZPJogHandler class boundary not found")
        failed += 1
else:
    print(f"{SKIP} S2-A already applied"); skipped += 1

# ══ S2-A (FIX): Replace ZPJogHandler.stop() with unregister version ══
print("\n=== S2-A FIX: ZPJogHandler.stop() with unregister ===")

if "# v7.2.6: ZP unregister" not in content:
    new_zp_stop = '''\
    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: ZP unregister — prevents stale callbacks on reconnect.
        """
        self._running = False
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_relative({}, None)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)

'''
    # Find stop() within ZPJogHandler section only
    zp_start, xy_start = find_class_section(content, "ZPJogHandler", "XYJogHandler")
    if zp_start is not None:
        zp_section = content[zp_start:xy_start]
        m = re.search(
            r'    def stop\(self\).*?(?=\n    def |\nclass |\Z)',
            zp_section, re.DOTALL | re.MULTILINE
        )
        if m:
            abs_s = zp_start + m.start()
            abs_e = zp_start + m.end()
            content = content[:abs_s] + new_zp_stop + content[abs_e:]
            print(f"{OK} Replaced ZPJogHandler.stop() with unregister version")
        else:
            print(f"{MISS} ZPJogHandler.stop() not found")
            failed += 1
    else:
        print(f"{MISS} ZPJogHandler boundary not found for stop()"); failed += 1
else:
    print(f"{SKIP} ZPJogHandler.stop() unregister already applied"); skipped += 1

# ══ S2-B: XYJogHandler _registered_handlers ══════════════════════
print("\n=== S2-B: XYJogHandler _registered_handlers ===")

if "# v7.2.6: XY registered handlers" not in content:
    xy_start, pos_start = find_class_section(content, "XYJogHandler", "PositionPoller")
    if xy_start is not None:
        xy_section = content[xy_start:pos_start]
        # Anchor on the last register_handler: increment_xyspeed_down
        last_m = None
        for m in re.finditer(
            r'        self\.processor\.register_handler\("increment_xyspeed_down",.*?\n',
            xy_section
        ):
            last_m = m
        if last_m:
            inject_xy = (
                "\n        # v7.2.6: XY registered handlers\n"
                "        self._registered_handlers: list = [\n"
                '            ("move_stage_at_velocity", self._handle_vel),\n'
                '            ("increment_xyspeed_up",   self._incr_up),\n'
                '            ("increment_xyspeed_down", self._incr_down),\n'
                "        ]\n"
            )
            abs_end = xy_start + last_m.end()
            content = content[:abs_end] + inject_xy + content[abs_end:]
            print(f"{OK} Added _registered_handlers list in XYJogHandler.__init__")
        else:
            print(f"{MISS} increment_xyspeed_down register_handler not found in XYJogHandler")
            failed += 1
    else:
        print(f"{MISS} XYJogHandler/PositionPoller boundary not found"); failed += 1
else:
    print(f"{SKIP} S2-B _registered_handlers already applied"); skipped += 1

# ══ S2-B: Replace XYJogHandler.stop() with unregister version ════
print("\n=== S2-B: XYJogHandler.stop() with unregister ===")

if "# v7.2.6: XY unregister" not in content:
    new_xy_stop = '''\
    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: XY unregister — prevents stale callbacks on reconnect.
        """
        self._running = False
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_stage_at_velocity(0, 0)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)

'''
    xy_start, pos_start = find_class_section(content, "XYJogHandler", "PositionPoller")
    if xy_start is not None:
        xy_section = content[xy_start:pos_start]
        m = re.search(
            r'    def stop\(self\).*?(?=\n    def |\nclass |\Z)',
            xy_section, re.DOTALL | re.MULTILINE
        )
        if m:
            abs_s = xy_start + m.start()
            abs_e = xy_start + m.end()
            content = content[:abs_s] + new_xy_stop + content[abs_e:]
            print(f"{OK} Replaced XYJogHandler.stop() with unregister version")
        else:
            print(f"{MISS} XYJogHandler.stop() not found")
            failed += 1
    else:
        print(f"{MISS} XYJogHandler boundary not found for stop()"); failed += 1
else:
    print(f"{SKIP} XYJogHandler.stop() unregister already applied"); skipped += 1

# ══ S2-C: XY jog loop try/except (single-line call) ══════════════
print("\n=== S2-C: XYJogHandler._jog_loop try/except ===")

if "# v7.2.6: XY jog serial guard" not in content:
    # The XY call is single-line: self.stage.move_stage_at_velocity(vx, vy)
    new_c, n = re.subn(
        r'^( +)(self\.stage\.move_stage_at_velocity\(vx, vy\))\s*$',
        lambda mo: (
            mo.group(1) + "# v7.2.6: XY jog serial guard\n" +
            mo.group(1) + "try:\n" +
            mo.group(1) + "    " + mo.group(2) + "\n" +
            mo.group(1) + "except Exception as e:\n" +
            mo.group(1) + '    logger.warning(f"[XY] Jog move failed: {e}")\n' +
            mo.group(1) + "    with self._lock:\n" +
            mo.group(1) + "        self.vel_x = self.vel_y = 0.0\n" +
            mo.group(1) + "    break"
        ),
        content, flags=re.MULTILINE
    )
    if n:
        content = new_c
        print(f"{OK} Added try/except around XY jog move_stage_at_velocity")
    else:
        print(f"{MISS} move_stage_at_velocity(vx, vy) not found in XY jog loop")
        failed += 1
else:
    print(f"{SKIP} S2-C XY guard already applied"); skipped += 1

# ══ S2-C (FIX): ZP _jog_loop — full method replacement ═══════════
# Instead of splicing inside the multi-line move_relative call,
# replace the entire _jog_loop method with a correctly structured version.
print("\n=== S2-C FIX: ZPJogHandler._jog_loop full replacement ===")

if "# v7.2.6: ZP jog serial guard" not in content:
    new_zp_jog_loop = '''\
    def _jog_loop(self) -> None:
        while self._running:
            with self._lock:
                vz, vp1, vp2, vp3 = self.vel_z, self.vel_p1, self.vel_p2, self.vel_p3

            is_moving = any(abs(v) > 0.001 for v in (vz, vp1, vp2, vp3))

            if is_moving and not self._was_moving:
                logger.debug(f"[ZP] Jog start: z={vz:.2f} p1={vp1:.2f}")
            elif not is_moving and self._was_moving:
                logger.debug("[ZP] Jog stop")
            self._was_moving = is_moving

            if not is_moving:
                time.sleep(0.01)
                continue

            dz = -vz * self.segment_time
            dp1 = vp1 * self.segment_time
            dp2 = vp2 * self.segment_time
            dp3 = vp3 * self.segment_time

            # Apply safety limits
            if self.safety_limits and self.safety_limits.enabled and self._get_zp_position:
                try:
                    pos = self._get_zp_position()
                    if pos[0] is not None:
                        cz, cp1, cp2, cp3 = pos
                        dz = self.safety_limits.clamp_z(cz + dz) - cz
                        dp1 = self.safety_limits.clamp_pump(cp1 + dp1, "P1") - cp1
                        dp2 = self.safety_limits.clamp_pump(cp2 + dp2, "P2") - cp2
                        dp3 = self.safety_limits.clamp_pump(cp3 + dp3, "P3") - cp3
                except Exception:
                    pass

            combined = math.sqrt(vz**2 + vp1**2 + vp2**2 + vp3**2)
            feedrate = max(combined * 60, 1)

            if self.safety_limits and self.safety_limits.enabled:
                feedrate = min(feedrate, self.safety_limits.max_z_feedrate)

            # v7.2.6: ZP jog serial guard — protect against disconnected stage
            try:
                self.stage.move_relative(
                    {"X": dz, "Y": dp1, "Z": dp2, "E": dp3}, feedrate
                )
            except Exception as e:
                logger.warning(f"[ZP] Jog move failed: {e}")
                with self._lock:
                    self.vel_z = self.vel_p1 = self.vel_p2 = self.vel_p3 = 0.0
                break

            time.sleep(self.segment_time)

'''
    zp_start, xy_start = find_class_section(content, "ZPJogHandler", "XYJogHandler")
    if zp_start is not None:
        zp_section = content[zp_start:xy_start]
        m = re.search(
            r'    def _jog_loop\(self\).*?(?=\n    def |\nclass |\Z)',
            zp_section, re.DOTALL | re.MULTILINE
        )
        if m:
            abs_s = zp_start + m.start()
            abs_e = zp_start + m.end()
            content = content[:abs_s] + new_zp_jog_loop + content[abs_e:]
            print(f"{OK} Replaced ZPJogHandler._jog_loop with try/except version")
        else:
            print(f"{MISS} ZPJogHandler._jog_loop not found"); failed += 1
    else:
        print(f"{MISS} ZPJogHandler boundary not found for _jog_loop"); failed += 1
else:
    print(f"{SKIP} S2-C ZP guard already applied"); skipped += 1

# ══ S2-D: connect_stages() duplicate handler guard ════════════════
print("\n=== S2-D: connect_stages() duplicate handler guard ===")

if "# v7.2.6: stop old jog handlers" not in content:
    new_c, n = re.subn(
        r'^( +)(self\.xy_jog = XYJogHandler\()',
        lambda mo: (
            mo.group(1) + "# v7.2.6: stop old jog handlers before creating new ones\n" +
            mo.group(1) + "if self.xy_jog is not None:\n" +
            mo.group(1) + "    self.xy_jog.stop()\n" +
            mo.group(1) + "    self.xy_jog = None\n" +
            mo.group(1) + mo.group(2)
        ),
        content, flags=re.MULTILINE
    )
    if n:
        content = new_c
        print(f"{OK} Added xy_jog cleanup guard in connect_stages()")
    else:
        print(f"{MISS} 'self.xy_jog = XYJogHandler(' not found"); failed += 1

    new_c2, n2 = re.subn(
        r'^( +)(self\.zp_jog = ZPJogHandler\()',
        lambda mo: (
            mo.group(1) + "# v7.2.6: stop old ZP jog handler\n" +
            mo.group(1) + "if self.zp_jog is not None:\n" +
            mo.group(1) + "    self.zp_jog.stop()\n" +
            mo.group(1) + "    self.zp_jog = None\n" +
            mo.group(1) + mo.group(2)
        ),
        content, flags=re.MULTILINE
    )
    if n2:
        content = new_c2
        print(f"{OK} Added zp_jog cleanup guard in connect_stages()")
    else:
        print(f"{MISS} 'self.zp_jog = ZPJogHandler(' not found"); failed += 1
else:
    print(f"{SKIP} S2-D already applied"); skipped += 1

# ── Final write ────────────────────────────────────────────────────
safe_write(sc_path, content, "StageController.py S2-fix")

print(f"\n{'='*50}")
print(f"S2-fix Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
