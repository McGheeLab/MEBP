#!/usr/bin/env python3
"""
Fix XY stage detection failing with E,5 (not initialized).

Problem: _find_with_protocol() only waits 0.1s after opening a serial port.
         When scanning multiple ports, the rapid open/close cycle on COM3
         (Marlin at wrong baud) leaves the USB subsystem unsettled. COM4
         (ProScan) responds with E,5 ("not initialized") because it needs
         more time after the port is opened.

Fix: Replace the detection loop with one that:
  1. Waits 0.3s after port open (not 0.1s)
  2. Toggles DTR to properly reset the serial adapter
  3. Retries once on ProScan error responses (E,N)

Run: python patches/patch_fix_detection_settle.py
"""

import ast
import re
import sys
from pathlib import Path


def find_root() -> Path:
    here = Path(__file__).resolve().parent
    for p in [here, here.parent, here.parent.parent, Path.cwd()]:
        if (p / "SupportClasses").is_dir():
            return p
    print("ERROR: Could not find MEBP project root")
    sys.exit(1)


NEW_METHOD = '''    def _find_with_protocol(self, protocol: ControllerProtocol) -> "Optional[serial.Serial]":
        """Try to find a controller matching the given protocol.
        v7.2.8s3: Increased settle time + retry on error for reliable detection.
        
        Prior ProScan II returns E,5 ("not initialized") if queried too soon
        after port open, especially when the USB subsystem was churned by
        scanning other ports first. Fix: longer settle + DTR toggle + retry.
        """
        detection = protocol.get_detection_info()
        wake_cmd = detection.get("wake_command")
        wake_delay = detection.get("wake_delay_ms", 100) / 1000.0
        fw_query = detection.get("firmware_query", "V")
        tokens = detection.get("identify_tokens", [])
        resp_pattern = detection.get("response_pattern")
        baud = protocol.baud_rate

        ports = serial.tools.list_ports.comports()
        for port_info in ports:
            try:
                logger.debug(f"Trying {port_info.device} @ {baud} baud ({protocol.controller_name})")
                spo = serial.Serial(
                    port_info.device,
                    baudrate=baud,
                    bytesize=protocol.byte_size,
                    timeout=0.5,
                    stopbits=serial.STOPBITS_ONE,
                )

                # v7.2.8s3: DTR toggle + longer settle for USB-serial adapters.
                # Rapid open/close of other ports can leave the USB subsystem
                # unsettled, causing E,5 (not initialized) on first query.
                spo.dtr = False
                time.sleep(0.05)
                spo.dtr = True
                time.sleep(0.3)  # 300ms settle (was 100ms — too short)
                spo.reset_input_buffer()
                spo.reset_output_buffer()

                tx_term = protocol.tx_terminator

                # Wake command (if defined)
                if wake_cmd:
                    spo.write(wake_cmd.encode(protocol.encoding) + tx_term)
                    time.sleep(wake_delay)
                    _read_response_cr(spo, timeout=0.3)
                    spo.reset_input_buffer()

                # v7.2.8s3: Detection with retry on error.
                # ProScan returns E,N error codes if not ready.
                # Retry once after a longer delay.
                response = ""
                for attempt in range(2):
                    spo.write(fw_query.encode(protocol.encoding) + tx_term)
                    time.sleep(0.05)
                    response = _read_response_cr(spo, timeout=0.3)
                    logger.debug(
                        f"Detection response from {port_info.device} "
                        f"(attempt {attempt+1}): {response!r}"
                    )

                    if not response:
                        break  # no device here

                    # If we got an error response (E,N), retry after delay
                    if response.startswith("E,") and attempt == 0:
                        logger.debug(
                            f"{port_info.device}: got error {response!r}, "
                            f"retrying after 500ms settle..."
                        )
                        spo.reset_input_buffer()
                        time.sleep(0.5)
                        continue

                    break  # got a real response (or empty on retry)

                if not response:
                    spo.close()
                    continue

                # Check response_pattern first (more specific)
                if resp_pattern:
                    if re.match(resp_pattern, response):
                        logger.info(
                            f"{protocol.controller_name} found on "
                            f"{port_info.device} @ {baud} baud "
                            f"(pattern match: {response!r})"
                        )
                        spo.timeout = protocol.timeout
                        self._detected_controller = protocol.controller_name
                        return spo
                    else:
                        spo.close()
                        continue

                # Fallback: token-based check
                if tokens and any(tok in response for tok in tokens):
                    logger.info(
                        f"{protocol.controller_name} found on "
                        f"{port_info.device} @ {baud} baud "
                        f"(token match: {response!r})"
                    )
                    spo.timeout = protocol.timeout
                    self._detected_controller = protocol.controller_name
                    return spo

                spo.close()
            except (serial.SerialException, UnicodeDecodeError, OSError) as e:
                logger.debug(f"Error on {port_info.device} @ {baud}: {e}")
                continue

        logger.debug(f"{protocol.controller_name} not found on any port")
        return None
'''


def main():
    root = find_root()
    xy_path = root / "SupportClasses" / "XYStage.py"

    print("=" * 60)
    print("  Fix: XY detection settle time + retry on E,N errors")
    print("=" * 60)
    print(f"  File: {xy_path}")

    if not xy_path.exists():
        print(f"  ERROR: {xy_path} not found")
        sys.exit(1)

    content = xy_path.read_text(encoding="utf-8")

    # Check if already patched
    if "v7.2.8s3" in content:
        print("\n  ✅ Already patched (v7.2.8s3 marker found)")
        return

    # Find the method using regex
    pattern = re.compile(
        r'^    def _find_with_protocol\(self, protocol.*?\n'
        r'(.*?)'
        r'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )

    match = pattern.search(content)
    if not match:
        print("  ERROR: Could not find _find_with_protocol method")
        sys.exit(1)

    print(f"  Found method at position {match.start()}")

    # Replace
    content = content[:match.start()] + NEW_METHOD + content[match.end():]

    # AST verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ERROR: Replacement produced invalid Python: {e}")
        sys.exit(1)

    xy_path.write_text(content, encoding="utf-8")
    print("\n  Changes:")
    print("    - Port settle: 0.1s → 0.3s + DTR toggle")
    print("    - Added retry: if response is E,N error, wait 500ms and retry once")
    print("    - Idempotency marker: v7.2.8s3")
    print("\n  ✅ Patch applied successfully")
    print("=" * 60)


if __name__ == "__main__":
    main()
