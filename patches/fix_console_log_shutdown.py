#!/usr/bin/env python3
"""
fix_console_log_shutdown.py — Prevent 'Signal source has been deleted' on exit.

The QtLogHandler tries to emit to a Qt signal after the console widget
has been destroyed during shutdown. Wrapping in try/except RuntimeError
silences this harmless race condition.

Run from MEBP project root:
    python fix_console_log_shutdown.py
"""

import os, sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

path = "gui/widgets/console_log.py"
if not os.path.isfile(path):
    print(f"{RED}ERROR{RESET}: {path} not found"); sys.exit(1)

with open(path) as f:
    content = f.read()

old = '''    def emit(self, record: logging.LogRecord):
        try:
            msg = self.format(record)
            self.console.log_record(record, msg)
        except Exception:
            self.handleError(record)'''

new = '''    def emit(self, record: logging.LogRecord):
        try:
            msg = self.format(record)
            self.console.log_record(record, msg)
        except RuntimeError:
            # Qt signal/widget already destroyed during shutdown — ignore
            pass
        except Exception:
            self.handleError(record)'''

if old in content:
    content = content.replace(old, new, 1)
    with open(path, "w") as f:
        f.write(content)
    print(f"{GREEN}OK{RESET}: Added RuntimeError guard to QtLogHandler.emit()")
elif "RuntimeError" in content and "already destroyed" in content:
    print(f"{GREEN}Already fixed{RESET}")
else:
    print(f"{RED}SKIP{RESET}: Pattern not found — may need manual edit")
