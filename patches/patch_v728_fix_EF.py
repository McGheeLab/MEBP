#!/usr/bin/env python3
"""
MEBP v7.2.8 — Fix patches E and F that failed due to indentation/line endings.

Handles Windows CRLF line endings properly.
"""

import ast
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

applied = 0
skipped = 0
failed  = 0

def ok(msg):
    global applied; applied += 1
    print(f"  {GREEN}✓ OK{RESET}   {msg}")

def skip(msg):
    global skipped; skipped += 1
    print(f"  {YELLOW}○ SKIP{RESET} {msg}")

def miss(msg):
    global failed; failed += 1
    print(f"  {RED}✗ MISS{RESET} {msg}")

def find_root(hint=None):
    if hint:
        p = Path(hint)
        if (p / "SupportClasses").is_dir():
            return p
    for d in [Path(__file__).resolve().parent] + list(Path(__file__).resolve().parents):
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    for loc in [
        Path.home() / "Documents" / "GitHub" / "MEBP",
        Path(r"C:\Users\mcghe\OneDrive\Documents\GitHub\MEBP"),
    ]:
        if loc.is_dir() and (loc / "SupportClasses").is_dir():
            return loc
    print(f"{RED}ERROR: Cannot find MEBP root.{RESET}")
    sys.exit(1)


def read_normalized(path):
    """Read file and normalize to LF line endings for matching."""
    raw = path.read_text(encoding="utf-8")
    return raw.replace("\r\n", "\n")


def write_with_crlf(path, content, label):
    """AST-verify, backup, write with CRLF line endings (Windows)."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        miss(f"{label}: AST FAIL — {e}")
        debug_path = path.with_suffix(".debug_v728.py")
        debug_path.write_text(content, encoding="utf-8")
        print(f"    Debug file: {debug_path}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v728ef_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    # Write with CRLF to match Windows convention
    crlf_content = content.replace("\n", "\r\n")
    path.write_bytes(crlf_content.encode("utf-8"))
    return True


# ═══════════════════════════════════════════════════════════════════
# FIX E: XYStage.py
# ═══════════════════════════════════════════════════════════════════

def fix_E(root):
    print(f"\n{CYAN}[E] XYStage.py — CR-aware detection + response_pattern{RESET}")
    path = root / "SupportClasses" / "XYStage.py"
    content = read_normalized(path)

    marker = "v7.2.8: CR-aware detection read"
    if marker in content:
        skip("Already patched")
        return

    # ── E1: Add `import re` ──
    anchor_e1 = (
        "from SupportClasses.ControllerProtocol import (\n"
        "    ControllerProtocol,\n"
        "    discover_controller_files,\n"
        "    DEFAULT_CONTROLLERS_DIR,\n"
        ")\n"
    )
    if "import re" not in content:
        if anchor_e1 in content:
            content = content.replace(
                anchor_e1,
                anchor_e1 + "import re  # v7.2.8: response_pattern matching\n",
                1
            )
            print("    E1: Added import re")
        else:
            miss("E1: ControllerProtocol import block not found")
            return
    else:
        print("    E1: import re already present")

    # ── E2: Add _read_response_cr helper before class ──
    helper = (
        '\n'
        '# v7.2.8: CR-aware detection read\n'
        'def _read_response_cr(spo, timeout=0.3):\n'
        '    """Read bytes until CR or LF, with short timeout for detection probes.\n'
        '\n'
        '    pyserial\'s readline() reads until LF (\\\\n), but Prior ProScan II\n'
        '    terminates responses with CR (\\\\r) only. This causes readline() to\n'
        '    block until the full timeout (1s+) on every detection probe.\n'
        '    """\n'
        '    old_timeout = spo.timeout\n'
        '    spo.timeout = timeout\n'
        '    buf = b""\n'
        '    try:\n'
        '        while True:\n'
        '            ch = spo.read(1)\n'
        '            if not ch:  # timeout\n'
        '                break\n'
        '            if ch in (b"\\\\r", b"\\\\n"):\n'
        '                if buf:  # got data before terminator\n'
        '                    break\n'
        '                continue  # skip leading CR/LF\n'
        '            buf += ch\n'
        '    except Exception:\n'
        '        pass\n'
        '    finally:\n'
        '        try:\n'
        '            spo.timeout = old_timeout\n'
        '        except Exception:\n'
        '            pass\n'
        '    return buf.decode("ascii", errors="replace").strip()\n'
        '\n'
        '\n'
    )

    class_anchor = "\nclass XYStageManager:\n"
    if "_read_response_cr" not in content:
        if class_anchor in content:
            content = content.replace(class_anchor, helper + "class XYStageManager:\n", 1)
            print("    E2: Added _read_response_cr helper")
        else:
            miss("E2: 'class XYStageManager:' not found")
            return
    else:
        print("    E2: _read_response_cr already present")

    # ── E3: Replace _find_with_protocol method ──
    old_method = (
        '    def _find_with_protocol(self, protocol: ControllerProtocol) -> Optional[serial.Serial]:\n'
        '        """Try to find a controller matching the given protocol."""\n'
        '        detection = protocol.get_detection_info()\n'
        '        wake_cmd = detection.get("wake_command")\n'
        '        wake_delay = detection.get("wake_delay_ms", 100) / 1000.0\n'
        '        fw_query = detection.get("firmware_query", "V")\n'
        '        tokens = detection.get("identify_tokens", [])\n'
        '        baud = protocol.baud_rate\n'
        '\n'
        '        ports = serial.tools.list_ports.comports()\n'
        '        for port_info in ports:\n'
        '            try:\n'
        '                logger.debug(f"Trying {port_info.device} @ {baud} baud ({protocol.controller_name})")\n'
        '                spo = serial.Serial(\n'
        '                    port_info.device,\n'
        '                    baudrate=baud,\n'
        '                    bytesize=protocol.byte_size,\n'
        '                    timeout=protocol.timeout,\n'
        '                    stopbits=serial.STOPBITS_ONE,\n'
        '                )\n'
        '\n'
        '                # P8.18: Use protocol terminators\n'
        '                tx_term = protocol.tx_terminator\n'
        '\n'
        '                # Wake command (if defined)\n'
        '                if wake_cmd:\n'
        '                    spo.write(wake_cmd.encode(protocol.encoding) + tx_term)\n'
        '                    time.sleep(wake_delay)\n'
        '                    spo.readline()  # discard wake-up response\n'
        '                    spo.reset_input_buffer()\n'
        '                    spo.reset_output_buffer()\n'
        '\n'
        '                # Firmware query\n'
        '                spo.write(fw_query.encode(protocol.encoding) + tx_term)\n'
        '                time.sleep(0.1)\n'
        '                response = spo.readline().decode(protocol.encoding, errors="replace").strip()\n'
        '                logger.debug(f"Response from {port_info.device}: {response}")\n'
        '\n'
        '                # Check identification tokens\n'
        '                if tokens and any(tok in response for tok in tokens):\n'
        '                    logger.info(\n'
        '                        f"{protocol.controller_name} found on "\n'
        '                        f"{port_info.device} @ {baud} baud"\n'
        '                    )\n'
        '                    self._detected_controller = protocol.controller_name\n'
        '                    return spo\n'
        '\n'
        '                spo.close()\n'
        '            except (serial.SerialException, UnicodeDecodeError, OSError) as e:\n'
        '                logger.debug(f"Error on {port_info.device} @ {baud}: {e}")\n'
        '                continue\n'
        '\n'
        '        logger.debug(f"{protocol.controller_name} not found on any port")\n'
        '        return None\n'
    )

    new_method = (
        '    def _find_with_protocol(self, protocol: ControllerProtocol) -> "Optional[serial.Serial]":\n'
        '        """Try to find a controller matching the given protocol.\n'
        '        v7.2.8: CR-aware detection read + response_pattern support.\n'
        '        """\n'
        '        detection = protocol.get_detection_info()\n'
        '        wake_cmd = detection.get("wake_command")\n'
        '        wake_delay = detection.get("wake_delay_ms", 100) / 1000.0\n'
        '        fw_query = detection.get("firmware_query", "V")\n'
        '        tokens = detection.get("identify_tokens", [])\n'
        '        resp_pattern = detection.get("response_pattern")\n'
        '        baud = protocol.baud_rate\n'
        '\n'
        '        ports = serial.tools.list_ports.comports()\n'
        '        for port_info in ports:\n'
        '            try:\n'
        '                logger.debug(f"Trying {port_info.device} @ {baud} baud ({protocol.controller_name})")\n'
        '                spo = serial.Serial(\n'
        '                    port_info.device,\n'
        '                    baudrate=baud,\n'
        '                    bytesize=protocol.byte_size,\n'
        '                    timeout=0.5,  # v7.2.8: short timeout for detection\n'
        '                    stopbits=serial.STOPBITS_ONE,\n'
        '                )\n'
        '                time.sleep(0.1)  # let port settle\n'
        '                spo.reset_input_buffer()\n'
        '                spo.reset_output_buffer()\n'
        '\n'
        '                tx_term = protocol.tx_terminator\n'
        '\n'
        '                # Wake command (if defined)\n'
        '                if wake_cmd:\n'
        '                    spo.write(wake_cmd.encode(protocol.encoding) + tx_term)\n'
        '                    time.sleep(wake_delay)\n'
        '                    _read_response_cr(spo, timeout=0.3)  # discard wake response\n'
        '                    spo.reset_input_buffer()\n'
        '\n'
        '                # Detection query\n'
        '                spo.write(fw_query.encode(protocol.encoding) + tx_term)\n'
        '                time.sleep(0.05)\n'
        '                response = _read_response_cr(spo, timeout=0.3)  # v7.2.8: CR-aware\n'
        '                logger.debug(f"Detection response from {port_info.device}: {response!r}")\n'
        '\n'
        '                if not response:\n'
        '                    spo.close()\n'
        '                    continue\n'
        '\n'
        '                # v7.2.8: Check response_pattern first (more specific)\n'
        '                if resp_pattern:\n'
        '                    if re.match(resp_pattern, response):\n'
        '                        logger.info(\n'
        '                            f"{protocol.controller_name} found on "\n'
        '                            f"{port_info.device} @ {baud} baud (pattern match: {response!r})"\n'
        '                        )\n'
        '                        spo.timeout = protocol.timeout\n'
        '                        self._detected_controller = protocol.controller_name\n'
        '                        return spo\n'
        '                    else:\n'
        '                        spo.close()\n'
        '                        continue\n'
        '\n'
        '                # Fallback: token-based check (only if no response_pattern)\n'
        '                if tokens and any(tok in response for tok in tokens):\n'
        '                    logger.info(\n'
        '                        f"{protocol.controller_name} found on "\n'
        '                        f"{port_info.device} @ {baud} baud (token match: {response!r})"\n'
        '                    )\n'
        '                    spo.timeout = protocol.timeout\n'
        '                    self._detected_controller = protocol.controller_name\n'
        '                    return spo\n'
        '\n'
        '                spo.close()\n'
        '            except (serial.SerialException, UnicodeDecodeError, OSError) as e:\n'
        '                logger.debug(f"Error on {port_info.device} @ {baud}: {e}")\n'
        '                continue\n'
        '\n'
        '        logger.debug(f"{protocol.controller_name} not found on any port")\n'
        '        return None\n'
    )

    if old_method in content:
        content = content.replace(old_method, new_method, 1)
        print("    E3: Replaced _find_with_protocol")
    else:
        miss("E3: _find_with_protocol exact text not found")
        return

    # ── E4: Fix get_current_position readline ──
    old_readline = (
        '            response = self.spo.readline().decode(\n'
        '                self._protocol.encoding if self._protocol else "ascii",\n'
        '                errors="replace"\n'
        '            ).strip()\n'
        '            return self._parse_position_response(response)'
    )
    new_readline = (
        '            # v7.2.8: CR-aware read (Prior sends \\r not \\n)\n'
        '            response = _read_response_cr(self.spo, timeout=0.5)\n'
        '            return self._parse_position_response(response)'
    )
    if old_readline in content:
        content = content.replace(old_readline, new_readline, 1)
        print("    E4: Fixed get_current_position readline")

    # ── E5: Fix get_firmware_version readline (same pattern, appears later) ──
    old_fw = (
        '            response = self.spo.readline().decode(\n'
        '                self._protocol.encoding if self._protocol else "ascii",\n'
        '                errors="replace"\n'
        '            ).strip()\n'
        '            return response'
    )
    new_fw = (
        '            # v7.2.8: CR-aware read\n'
        '            response = _read_response_cr(self.spo, timeout=0.5)\n'
        '            return response'
    )
    if old_fw in content:
        content = content.replace(old_fw, new_fw, 1)
        print("    E5: Fixed get_firmware_version readline")

    if write_with_crlf(path, content, "E: XYStage.py"):
        ok("XYStage.py fully patched")


# ═══════════════════════════════════════════════════════════════════
# FIX F: main.py
# ═══════════════════════════════════════════════════════════════════

def fix_F(root):
    print(f"\n{CYAN}[F] main.py — Pass controller_json from settings{RESET}")
    path = root / "main.py"
    content = read_normalized(path)

    marker = "v7.2.8: Pass controller_json"
    if marker in content:
        skip("Already patched")
        return

    old_block = (
        '    controller = StageController(\n'
        '        simulate_xy=simulate_xy,\n'
        '        simulate_zp=simulate_zp,\n'
        '    )'
    )
    new_block = (
        '    # v7.2.8: Pass controller_json from settings for hardware auto-detect\n'
        '    controller_json = settings.get("controller.controller_json", "auto")\n'
        '\n'
        '    controller = StageController(\n'
        '        simulate_xy=simulate_xy,\n'
        '        simulate_zp=simulate_zp,\n'
        '        controller_json=controller_json,\n'
        '    )'
    )

    if old_block in content:
        content = content.replace(old_block, new_block, 1)
    else:
        miss("StageController() constructor not found")
        return

    if write_with_crlf(path, content, "F: main.py"):
        ok("main.py: controller_json='auto' passed from settings")


def main():
    print(f"\n{BOLD}{'═' * 60}{RESET}")
    print(f"{BOLD} MEBP v7.2.8 — Fix patches E and F{RESET}")
    print(f"{BOLD}{'═' * 60}{RESET}")

    hint = sys.argv[1] if len(sys.argv) > 1 else None
    root = find_root(hint)
    print(f"\nProject root: {root}")

    fix_E(root)
    fix_F(root)

    print(f"\n{'═' * 60}")
    print(f"  {GREEN}Applied: {applied}{RESET}  |  "
          f"{YELLOW}Skipped: {skipped}{RESET}  |  "
          f"{RED}Failed: {failed}{RESET}")
    print(f"{'═' * 60}\n")

    if failed > 0:
        print(f"{RED}⚠ Some patches failed.{RESET}")
        sys.exit(1)
    else:
        print(f"{GREEN}✓ Patches E and F applied successfully.{RESET}")


if __name__ == "__main__":
    main()
