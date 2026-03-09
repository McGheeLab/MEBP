#!/usr/bin/env python3
"""
patch_v726_s6_speed_sync.py — v7.2.6 Session 6
Fixes BUGs 11, 12, 13: pump_speed name, speed readback sync, slider refs.

Changes in gui/pages/jog_control.py:
  S6-A: _on_p_speed — pump_speed → p_speed (BUG 11 critical fix)
  S6-B: _make_ctx_slider — add optional store_as param to save slider ref
  S6-C: 3 call sites — pass store_as to capture _sld_xy_speed, _sld_z_speed, _sld_p_speed
  S6-D: on_status_update — append self._sync_speed_sliders() call
  S6-E: Add _sync_speed_sliders() method
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
jc_path = ROOT / "gui" / "pages" / "jog_control.py"
content = safe_read(jc_path)
if not content:
    sys.exit(1)

# ══ S6-A: Fix pump_speed → p_speed (BUG 11) ══════════════════════
print("\n=== S6-A: Fix _on_p_speed pump_speed → p_speed ===")

if "# v7.2.6: S6-A p_speed fix" not in content:
    new_on_p = (
        "    def _on_p_speed(self, value):\n"
        "        # v7.2.6: S6-A p_speed fix — was 'pump_speed' (dead attr), now 'p_speed'\n"
        "        speed = value / 100.0\n"
        "        self.lbl_p_speed.setText(f\"{speed:.2f}\")\n"
        "        if hasattr(self.controller, 'zp_jog') and self.controller.zp_jog:\n"
        "            self.controller.zp_jog.p_speed = speed\n"
        "\n"
    )
    m = re.search(
        r'^    def _on_p_speed\(self.*?(?=\n    def |\nclass |\Z)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        content = content[:m.start()] + new_on_p + content[m.end():]
        print(f"{OK} Fixed _on_p_speed: pump_speed → p_speed")
    else:
        print(f"{MISS} _on_p_speed not found"); failed += 1
else:
    print(f"{SKIP} S6-A already applied"); skipped += 1

# ══ S6-B: _make_ctx_slider — add store_as param ══════════════════
print("\n=== S6-B: _make_ctx_slider store_as param ===")

if "# v7.2.6: S6-B store_as" not in content:
    new_slider_fn = (
        "    def _make_ctx_slider(self, label_text, value_label, min_val, max_val,\n"
        "                         default, callback, store_as=None):\n"
        "        \"\"\"Create a labeled slider row for the context panel.\n"
        "        v7.2.6: S6-B store_as — when given, stores slider ref as self.<store_as>.\n"
        "        \"\"\"\n"
        "        frame = QFrame()\n"
        "        layout = QHBoxLayout(frame)\n"
        "        layout.setContentsMargins(0, 0, 0, 0)\n"
        "        layout.setSpacing(4)\n"
        "\n"
        "        lbl = QLabel(label_text)\n"
        "        lbl.setMinimumWidth(24)\n"
        "        layout.addWidget(lbl)\n"
        "\n"
        "        slider = QSlider(Qt.Orientation.Horizontal)\n"
        "        slider.setRange(min_val, max_val)\n"
        "        slider.setValue(default)\n"
        "        slider.valueChanged.connect(callback)\n"
        "        layout.addWidget(slider, stretch=1)\n"
        "\n"
        "        layout.addWidget(value_label)\n"
        "\n"
        "        # v7.2.6: S6-B store_as — store slider ref for readback\n"
        "        if store_as is not None:\n"
        "            setattr(self, store_as, slider)\n"
        "\n"
        "        return frame\n"
        "\n"
    )
    m = re.search(
        r'^    def _make_ctx_slider\(self.*?(?=\n    def |\nclass |\Z)',
        content, re.DOTALL | re.MULTILINE
    )
    if m:
        content = content[:m.start()] + new_slider_fn + content[m.end():]
        print(f"{OK} Replaced _make_ctx_slider with store_as version")
    else:
        print(f"{MISS} _make_ctx_slider not found"); failed += 1
else:
    print(f"{SKIP} S6-B already applied"); skipped += 1

# ══ S6-C: Update 3 call sites to pass store_as ═══════════════════
print("\n=== S6-C: Slider call sites — add store_as ===")

if "# v7.2.6: S6-C slider refs" not in content:
    replacements = [
        (
            'layout.addWidget(self._make_ctx_slider(\n'
            '            "XY:", self.lbl_xy_speed, 1, 10000, 100, self._on_xy_speed))',
            '# v7.2.6: S6-C slider refs\n'
            '        layout.addWidget(self._make_ctx_slider(\n'
            '            "XY:", self.lbl_xy_speed, 1, 10000, 100, self._on_xy_speed,\n'
            '            store_as="_sld_xy_speed"))',
        ),
        (
            'layout.addWidget(self._make_ctx_slider(\n'
            '            "Z:", self.lbl_z_speed, 1, 500, 50, self._on_z_speed))',
            'layout.addWidget(self._make_ctx_slider(\n'
            '            "Z:", self.lbl_z_speed, 1, 500, 50, self._on_z_speed,\n'
            '            store_as="_sld_z_speed"))',
        ),
        (
            'layout.addWidget(self._make_ctx_slider(\n'
            '            "P:", self.lbl_p_speed, 1, 500, 50, self._on_p_speed))',
            'layout.addWidget(self._make_ctx_slider(\n'
            '            "P:", self.lbl_p_speed, 1, 500, 50, self._on_p_speed,\n'
            '            store_as="_sld_p_speed"))',
        ),
    ]
    n_replaced = 0
    for old, new in replacements:
        if old in content:
            content = content.replace(old, new, 1)
            n_replaced += 1
        else:
            # Try stripped/normalized version - sometimes whitespace differs
            print(f"{MISS} Call site not found (exact): {old[:50].strip()!r}")
            failed += 1
    if n_replaced == 3:
        print(f"{OK} Updated all 3 slider call sites with store_as")
    elif n_replaced > 0:
        print(f"{YELLOW}  ~ {n_replaced}/3 call sites updated{RESET}")
else:
    print(f"{SKIP} S6-C already applied"); skipped += 1

# ══ S6-D: on_status_update — append _sync_speed_sliders() ════════
print("\n=== S6-D: on_status_update — append _sync_speed_sliders() ===")

if "# v7.2.6: S6-D sync speeds" not in content:
    # Find on_status_update — it ends just before _setup_shortcuts
    # The last line of on_status_update body ends with the step verification block
    # Append the sync call before the closing of the method
    old_end = (
        "        if hasattr(self, 'ctx_lbl_log_count') and hasattr(self.controller, 'position_logger'):\n"
        "            self.ctx_lbl_log_count.setText(f\"Entries: {self.controller.position_logger.count}\")"
    )
    new_end = (
        "        if hasattr(self, 'ctx_lbl_log_count') and hasattr(self.controller, 'position_logger'):\n"
        "            self.ctx_lbl_log_count.setText(f\"Entries: {self.controller.position_logger.count}\")\n"
        "        # v7.2.6: S6-D sync speeds — keep jog page sliders in sync with handlers\n"
        "        self._sync_speed_sliders()"
    )
    if old_end in content:
        content = content.replace(old_end, new_end, 1)
        print(f"{OK} Appended _sync_speed_sliders() to on_status_update (long pattern)")
    else:
        # Try shorter pattern — look for end of on_status_update
        # The method ends before _setup_shortcuts or _jog_xy; just find and append
        m = re.search(
            r'^    def on_status_update\(self\).*?(?=\n    def |\nclass |\Z)',
            content, re.DOTALL | re.MULTILINE
        )
        if m:
            method_body = m.group(0)
            # Append to the end of the method
            new_body = method_body.rstrip() + "\n        # v7.2.6: S6-D sync speeds\n        self._sync_speed_sliders()\n\n"
            content = content[:m.start()] + new_body + content[m.end():]
            print(f"{OK} Appended _sync_speed_sliders() to on_status_update (method replacement)")
        else:
            print(f"{MISS} on_status_update not found in jog_control.py"); failed += 1
else:
    print(f"{SKIP} S6-D already applied"); skipped += 1

# ══ S6-E: Add _sync_speed_sliders() method ════════════════════════
print("\n=== S6-E: Add _sync_speed_sliders() method ===")

if "def _sync_speed_sliders" not in content:
    new_method = (
        "    def _sync_speed_sliders(self):\n"
        "        \"\"\"Read current speeds from jog handlers and update sliders + labels.\n"
        "\n"
        "        v7.2.6: S6-E — ensures Xbox speed changes are reflected on jog page,\n"
        "        and handles stage reconnect which creates handlers with default speeds.\n"
        "        Uses blockSignals(True) to prevent the readback triggering a write-back.\n"
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
        "        # Z and pump speeds\n"
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
        "            actual_p = ctrl.zp_jog.p_speed\n"
        "            sld_p = getattr(self, '_sld_p_speed', None)\n"
        "            if sld_p is not None:\n"
        "                slider_p = sld_p.value() / 100.0\n"
        "                if abs(slider_p - actual_p) > 0.005:\n"
        "                    sld_p.blockSignals(True)\n"
        "                    sld_p.setValue(max(sld_p.minimum(),\n"
        "                                      min(sld_p.maximum(), int(actual_p * 100))))\n"
        "                    sld_p.blockSignals(False)\n"
        "                    self.lbl_p_speed.setText(f\"{actual_p:.2f}\")\n"
        "\n"
    )
    # Inject before _make_ctx_slider (which is after the callback methods)
    m = re.search(r'^    def _make_ctx_slider\(', content, re.MULTILINE)
    if m:
        content = content[:m.start()] + new_method + content[m.start():]
        print(f"{OK} Added _sync_speed_sliders() method")
    else:
        # Fallback: inject before _setup_ui
        m2 = re.search(r'^    def _setup_ui\(', content, re.MULTILINE)
        if m2:
            content = content[:m2.start()] + new_method + content[m2.start():]
            print(f"{OK} Added _sync_speed_sliders() method (before _setup_ui)")
        else:
            print(f"{MISS} Could not find injection point for _sync_speed_sliders"); failed += 1
else:
    print(f"{SKIP} _sync_speed_sliders already exists"); skipped += 1

safe_write(jc_path, content, "jog_control.py S6")

print(f"\n{'='*50}")
print(f"S6 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
