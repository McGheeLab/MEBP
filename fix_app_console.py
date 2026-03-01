#!/usr/bin/env python3
"""
fix_app_console.py — Add missing ConsoleLogWidget to app.py.

The v7.2 replacement app.py builds the page stack directly without a
QSplitter + ConsoleLogWidget. This restores the console that main.py
expects at `window.console`.

Run from MEBP project root:
    python fix_app_console.py
"""

import os
import sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


def main():
    path = "gui/app.py"
    if not os.path.isfile(path):
        print(f"{RED}ERROR{RESET}: {path} not found. Run from MEBP project root.")
        sys.exit(1)

    with open(path, "r") as f:
        content = f.read()

    # Check if already fixed
    if "self.console = ConsoleLogWidget()" in content:
        print(f"{GREEN}Already fixed{RESET}: console widget already in app.py")
        return

    # Replace the bare page stack with splitter + console
    old_block = """        # Page stack
        self._page_stack = QStackedWidget()
        self._page_stack.setObjectName("pagesContainer")
        content_layout.addWidget(self._page_stack)

        app_layout.addWidget(content_frame)"""

    new_block = """        # Content splitter (pages + console)
        self._splitter = QSplitter(Qt.Vertical)
        self._splitter.setObjectName("contentBottom")

        # Page stack
        self._page_stack = QStackedWidget()
        self._page_stack.setObjectName("pagesContainer")
        self._splitter.addWidget(self._page_stack)

        # Console log
        self.console = ConsoleLogWidget()
        self._splitter.addWidget(self.console)

        self._splitter.setStretchFactor(0, 5)
        self._splitter.setStretchFactor(1, 1)

        content_layout.addWidget(self._splitter)

        app_layout.addWidget(content_frame)"""

    if old_block in content:
        content = content.replace(old_block, new_block, 1)

        # Make sure QSplitter is imported
        if "QSplitter" not in content:
            content = content.replace(
                "from PySide6.QtWidgets import (",
                "from PySide6.QtWidgets import (\n    QSplitter,",
                1,
            )

        with open(path, "w") as f:
            f.write(content)
        print(f"{GREEN}OK{RESET}: Added QSplitter + ConsoleLogWidget to app.py")
    else:
        print(f"{RED}FAIL{RESET}: Could not find page stack block to replace.")
        print("You may need to manually add after the _page_stack creation:")
        print("    self.console = ConsoleLogWidget()")


if __name__ == "__main__":
    main()
