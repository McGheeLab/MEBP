#!/usr/bin/env python3
"""
patch_v726_s7_pump_uL_safety.py — v7.2.6 Session 7
Fixes BUGs 14, 15, 16: pump jog in µL/s with flow rate safety clamping.

Changes in SupportClasses/StageController.py:
  S7-A: ZPJogHandler._handle_p1_vel/_handle_p2_vel/_handle_p3_vel
        — clamp incoming velocity to max safe flow rate when hw config available

Changes in gui/pages/jog_control.py:
  S7-B: _on_p_speed — interpret slider as µL/s when hw config available
  S7-C: _sync_speed_sliders — display pump speed as µL/s when hw config available
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
# S7-A: ZPJogHandler pump velocity handlers with flow clamping
# ════════════════════════════════════════════════════════════════════
print("\n=== S7-A: ZPJogHandler pump handler flow rate clamping ===")
sc_path = ROOT / "SupportClasses" / "StageController.py"
content_sc = safe_read(sc_path)
if not content_sc:
    sys.exit(1)

if "# v7.2.6: S7-A flow clamp" not in content_sc:
    # Replace all three pump velocity handlers with clamped versions
    # Strategy: replace the block containing all three _handle_pX_vel methods
    # They all follow the same pattern, so we replace them as a group.

    new_pump_handlers = (
        "    def _handle_p1_vel(self, *args, **kwargs):\n"
        "        raw = self._extract_velocity(*args, **kwargs)\n"
        "        vel = self._clamp_vel(raw, self.p_speed)\n"
        "        vel = self._clamp_pump_flow(vel, \"P1\")  # v7.2.6: S7-A flow clamp\n"
        "        with self._lock:\n"
        "            self.vel_p1 = vel\n"
        "\n"
        "    def _handle_p2_vel(self, *args, **kwargs):\n"
        "        raw = self._extract_velocity(*args, **kwargs)\n"
        "        vel = self._clamp_vel(raw, self.p_speed)\n"
        "        vel = self._clamp_pump_flow(vel, \"P2\")  # v7.2.6: S7-A flow clamp\n"
        "        with self._lock:\n"
        "            self.vel_p2 = vel\n"
        "\n"
        "    def _handle_p3_vel(self, *args, **kwargs):\n"
        "        raw = self._extract_velocity(*args, **kwargs)\n"
        "        vel = self._clamp_vel(raw, self.p_speed)\n"
        "        vel = self._clamp_pump_flow(vel, \"P3\")  # v7.2.6: S7-A flow clamp\n"
        "        with self._lock:\n"
        "            self.vel_p3 = vel\n"
        "\n"
    )

    # Add _clamp_pump_flow helper method
    clamp_helper = (
        "    def _clamp_pump_flow(self, vel_dimensionless: float, pump_id: str) -> float:\n"
        "        \"\"\"Clamp pump velocity to max safe flow rate.\n"
        "\n"
        "        v7.2.6: S7-A — converts dimensionless vel → µL/s, clamps, converts back.\n"
        "        Falls through unchanged when hardware config or safety limits not set.\n"
        "\n"
        "        Args:\n"
        "            vel_dimensionless: Velocity in handler units (segment_time × p_speed)\n"
        "            pump_id: 'P1', 'P2', or 'P3'\n"
        "        Returns:\n"
        "            Clamped velocity in same units.\n"
        "        \"\"\"\n"
        "        hw = self._hardware_config\n"
        "        sl = self.safety_limits\n"
        "        if hw is None or sl is None or not sl.enabled:\n"
        "            return vel_dimensionless\n"
        "        pump_cfg = hw.pumps.get(pump_id)\n"
        "        if pump_cfg is None or not pump_cfg.is_configured:\n"
        "            return vel_dimensionless\n"
        "        max_rate = sl.get_max_flow_rate(pump_id)\n"
        "        if max_rate <= 0:\n"
        "            return vel_dimensionless\n"
        "        try:\n"
        "            # vel_dimensionless ~ mm/s of plunger travel\n"
        "            rate_uL_s = abs(pump_cfg.mm_to_uL(abs(vel_dimensionless)))\n"
        "            if rate_uL_s > max_rate:\n"
        "                scale = max_rate / rate_uL_s\n"
        "                logger.debug(\n"
        "                    f\"{pump_id} jog flow clamped: {rate_uL_s:.3f} → \"\n"
        "                    f\"{max_rate:.3f} µL/s (scale={scale:.3f})\"\n"
        "                )\n"
        "                return vel_dimensionless * scale\n"
        "        except (ValueError, AttributeError):\n"
        "            pass\n"
        "        return vel_dimensionless\n"
        "\n"
    )

    zp_s, xy_s = find_class_section(content_sc, "ZPJogHandler", "XYJogHandler")
    if zp_s is None:
        print(f"{MISS} ZPJogHandler class not found"); failed += 1
    else:
        zp_section = content_sc[zp_s:xy_s]

        # Find the block from _handle_p1_vel through end of _handle_p3_vel
        m = re.search(
            r'    def _handle_p1_vel\(self.*?(?=\n    def _incr_z_up|\n    def _jog_loop|\Z)',
            zp_section, re.DOTALL | re.MULTILINE
        )
        if m:
            abs_s = zp_s + m.start()
            abs_e = zp_s + m.end()
            content_sc = content_sc[:abs_s] + new_pump_handlers + content_sc[abs_e:]
            print(f"{OK} Replaced _handle_p1/p2/p3_vel with flow-clamped versions")

            # Now inject _clamp_pump_flow helper before _handle_p1_vel
            # Re-find after replacement
            zp_s2, xy_s2 = find_class_section(content_sc, "ZPJogHandler", "XYJogHandler")
            m2 = re.search(r'^    def _handle_p1_vel\(', content_sc[zp_s2:xy_s2], re.MULTILINE)
            if m2:
                abs_inject = zp_s2 + m2.start()
                content_sc = content_sc[:abs_inject] + clamp_helper + content_sc[abs_inject:]
                print(f"{OK} Added _clamp_pump_flow helper to ZPJogHandler")
            else:
                print(f"{MISS} Could not inject _clamp_pump_flow (helper method)"); failed += 1
        else:
            print(f"{MISS} _handle_p1_vel block not found in ZPJogHandler"); failed += 1

    safe_write(sc_path, content_sc, "StageController.py S7-A")
else:
    print(f"{SKIP} S7-A already applied"); skipped += 1

# ════════════════════════════════════════════════════════════════════
# S7-B + S7-C: jog_control.py — µL/s display and slider
# ════════════════════════════════════════════════════════════════════
jc_path = ROOT / "gui" / "pages" / "jog_control.py"
content_jc = safe_read(jc_path)
if not content_jc:
    sys.exit(1)

# ── S7-B: _on_p_speed µL/s mode ──────────────────────────────────
print("\n=== S7-B: _on_p_speed µL/s mode when hw config available ===")

if "# v7.2.6: S7-B uL/s mode" not in content_jc:
    new_on_p = (
        "    def _on_p_speed(self, value):\n"
        "        # v7.2.6: S6-A p_speed fix + S7-B uL/s mode\n"
        "        if (self._hardware_config and\n"
        "                self._hardware_config.configured_pump_ids):\n"
        "            # Slider 1-500 → 0.01-5.00 µL/s\n"
        "            rate_uL_s = value / 100.0\n"
        "            self.lbl_p_speed.setText(f\"{rate_uL_s:.2f} µL/s\")\n"
        "            if getattr(self.controller, 'zp_jog', None):\n"
        "                pid = self._hardware_config.configured_pump_ids[0]\n"
        "                try:\n"
        "                    mm_per_s = self._hardware_config.uL_to_mm(pid, rate_uL_s)\n"
        "                    self.controller.zp_jog.p_speed = mm_per_s\n"
        "                except (ValueError, AttributeError):\n"
        "                    self.controller.zp_jog.p_speed = rate_uL_s\n"
        "        else:\n"
        "            # Fallback: raw multiplier (v7.2.6: S6-A fix — p_speed not pump_speed)\n"
        "            speed = value / 100.0\n"
        "            self.lbl_p_speed.setText(f\"{speed:.2f}\")\n"
        "            if getattr(self.controller, 'zp_jog', None):\n"
        "                self.controller.zp_jog.p_speed = speed\n"
        "\n"
    )
    m = re.search(
        r'^    def _on_p_speed\(self.*?(?=\n    def |\nclass |\Z)',
        content_jc, re.DOTALL | re.MULTILINE
    )
    if m:
        content_jc = content_jc[:m.start()] + new_on_p + content_jc[m.end():]
        print(f"{OK} Replaced _on_p_speed with µL/s-aware version")
    else:
        print(f"{MISS} _on_p_speed not found in jog_control.py"); failed += 1
else:
    print(f"{SKIP} S7-B already applied"); skipped += 1

# ── S7-C: _sync_speed_sliders pump µL/s display ──────────────────
print("\n=== S7-C: _sync_speed_sliders pump µL/s display ===")

if "# v7.2.6: S7-C uL/s display" not in content_jc:
    # Replace _sync_speed_sliders with version that shows µL/s for pump
    new_sync = (
        "    def _sync_speed_sliders(self):\n"
        "        \"\"\"Read current speeds from jog handlers and update sliders + labels.\n"
        "\n"
        "        v7.2.6: S6-E + S7-C — Xbox speed changes reflected on jog page;\n"
        "        reconnect resets handled; pump shows µL/s when hw config available.\n"
        "        \"\"\"\n"
        "        ctrl = self.controller\n"
        "\n"
        "        # XY speed\n"
        "        if getattr(ctrl, 'xy_jog', None):\n"
        "            actual_xy = ctrl.xy_jog.xy_speed\n"
        "            sld = getattr(self, '_sld_xy_speed', None)\n"
        "            if sld is not None and abs(sld.value() - actual_xy) > 0.5:\n"
        "                sld.blockSignals(True)\n"
        "                sld.setValue(max(sld.minimum(), min(sld.maximum(), int(actual_xy))))\n"
        "                sld.blockSignals(False)\n"
        "                self.lbl_xy_speed.setText(f\"{int(actual_xy)}\")\n"
        "\n"
        "        # Z speed\n"
        "        if getattr(ctrl, 'zp_jog', None):\n"
        "            actual_z = ctrl.zp_jog.z_speed\n"
        "            sld_z = getattr(self, '_sld_z_speed', None)\n"
        "            if sld_z is not None:\n"
        "                slider_z = sld_z.value() / 100.0\n"
        "                if abs(slider_z - actual_z) > 0.005:\n"
        "                    sld_z.blockSignals(True)\n"
        "                    sld_z.setValue(max(sld_z.minimum(),\n"
        "                                      min(sld_z.maximum(), int(actual_z * 100))))\n"
        "                    sld_z.blockSignals(False)\n"
        "                    self.lbl_z_speed.setText(f\"{actual_z:.2f}\")\n"
        "\n"
        "            # Pump speed — show µL/s when hw config available (v7.2.6: S7-C uL/s display)\n"
        "            actual_p = ctrl.zp_jog.p_speed\n"
        "            sld_p = getattr(self, '_sld_p_speed', None)\n"
        "            if sld_p is not None:\n"
        "                hw = self._hardware_config\n"
        "                if hw and hw.configured_pump_ids:\n"
        "                    pid = hw.configured_pump_ids[0]\n"
        "                    try:\n"
        "                        rate_uL_s = abs(hw.mm_to_uL(pid, abs(actual_p)))\n"
        "                        slider_target = int(rate_uL_s * 100)\n"
        "                        if abs(sld_p.value() - slider_target) > 1:\n"
        "                            sld_p.blockSignals(True)\n"
        "                            sld_p.setValue(max(sld_p.minimum(),\n"
        "                                              min(sld_p.maximum(), slider_target)))\n"
        "                            sld_p.blockSignals(False)\n"
        "                        self.lbl_p_speed.setText(f\"{rate_uL_s:.2f} \u00b5L/s\")\n"
        "                    except (ValueError, AttributeError):\n"
        "                        self.lbl_p_speed.setText(f\"{actual_p:.2f}\")\n"
        "                else:\n"
        "                    slider_p = sld_p.value() / 100.0\n"
        "                    if abs(slider_p - actual_p) > 0.005:\n"
        "                        sld_p.blockSignals(True)\n"
        "                        sld_p.setValue(max(sld_p.minimum(),\n"
        "                                          min(sld_p.maximum(), int(actual_p * 100))))\n"
        "                        sld_p.blockSignals(False)\n"
        "                        self.lbl_p_speed.setText(f\"{actual_p:.2f}\")\n"
        "\n"
    )
    m = re.search(
        r'^    def _sync_speed_sliders\(self\).*?(?=\n    def |\nclass |\Z)',
        content_jc, re.DOTALL | re.MULTILINE
    )
    if m:
        content_jc = content_jc[:m.start()] + new_sync + content_jc[m.end():]
        print(f"{OK} Replaced _sync_speed_sliders with µL/s-aware version")
    else:
        print(f"{MISS} _sync_speed_sliders not found (run S6 first)"); failed += 1
else:
    print(f"{SKIP} S7-C already applied"); skipped += 1

safe_write(jc_path, content_jc, "jog_control.py S7-B/C")

print(f"\n{'='*50}")
print(f"S7 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
