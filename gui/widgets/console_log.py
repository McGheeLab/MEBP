"""
Console Log Widget + QtLogHandler.

Provides a read-only QTextEdit that displays colour-coded log messages and
a ``QtLogHandler`` that routes Python ``logging`` records into the widget
via a thread-safe Qt signal bridge.

Usage in main.py::

    qt_handler = QtLogHandler(window.console)
    qt_handler.setFormatter(logging.Formatter("%(name)s: %(message)s"))
    logging.getLogger().addHandler(qt_handler)
"""

from __future__ import annotations

import logging
from datetime import datetime

from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton
from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtGui import QTextCursor, QColor, QFont


# ── Colour mapping ────────────────────────────────────────────────

LEVEL_COLOURS = {
    logging.DEBUG:    "#6c7086",   # Overlay0 — dim
    logging.INFO:     "#cdd6f4",   # Text — normal
    logging.WARNING:  "#f9e2af",   # Yellow
    logging.ERROR:    "#f38ba8",   # Red
    logging.CRITICAL: "#f38ba8",   # Red
}

TAG_COLOURS = {
    "success": "#a6e3a1",
    "error":   "#f38ba8",
    "warning": "#f9e2af",
    "info":    "#cdd6f4",
    "debug":   "#6c7086",
}


# ── Thread-safe signal bridge ─────────────────────────────────────

class _LogSignalBridge(QObject):
    """Emits a Qt signal so background threads can safely append text."""
    log_received = Signal(str, str)  # (html_line, raw_text)


# ── Console widget ────────────────────────────────────────────────

class ConsoleLogWidget(QWidget):
    """
    Read-only log viewer with auto-scroll, clear button, and colour coding.
    """

    MAX_LINES = 5000

    # v7.4.2: signal emitted when collapse/expand toggled so the parent
    # splitter can relax its minimum-height constraint to let the widget
    # shrink down to the toolbar height.
    collapse_toggled = Signal(bool)  # True when collapsed

    def __init__(self, parent=None, start_collapsed: bool = False):
        super().__init__(parent)
        self._auto_scroll = True
        self._line_count = 0
        self._collapsed = False
        self._bridge = _LogSignalBridge()
        self._bridge.log_received.connect(self._append)

        self._setup_ui()

        if start_collapsed:
            # Apply collapsed visual state up-front without emitting the
            # signal — the host hasn't connected slots yet, and the
            # splitter is responsible for picking a sensible initial size.
            self._collapsed = True
            self._text.setVisible(False)
            self._btn_collapse.setText("▲ Terminal")
            self._btn_collapse.setToolTip("Expand the terminal")

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # v7.2.6: Prevent resize crash
        self.setMinimumHeight(40)

        # Toolbar
        toolbar = QHBoxLayout()
        toolbar.setSpacing(4)

        # v7.4.2: collapse/expand toggle — first control so it's easy to
        # spot when the terminal is hogging vertical space.
        self._btn_collapse = QPushButton("▼ Terminal")
        self._btn_collapse.setFixedWidth(110)
        self._btn_collapse.setToolTip("Collapse / expand the terminal")
        self._btn_collapse.clicked.connect(self._toggle_collapsed)
        toolbar.addWidget(self._btn_collapse)

        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(60)
        btn_clear.clicked.connect(self.clear)
        toolbar.addWidget(btn_clear)

        self._btn_scroll = QPushButton("Auto-scroll: ON")
        self._btn_scroll.setFixedWidth(120)
        self._btn_scroll.clicked.connect(self._toggle_scroll)
        toolbar.addWidget(self._btn_scroll)

        toolbar.addStretch()
        layout.addLayout(toolbar)

        # Text area
        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont("Consolas", 9))
        self._text.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        layout.addWidget(self._text)

    # ── Public API ────────────────────────────────────────────────

    def log(self, message: str, tag: str = "info"):
        """
        Append a message with a colour tag.  Safe to call from any thread
        (signals are queued across threads automatically).
        """
        colour = TAG_COLOURS.get(tag, "#cdd6f4")
        stamp = datetime.now().strftime("%H:%M:%S")
        html = f'<span style="color:#6c7086">{stamp}</span> <span style="color:{colour}">{_escape(message)}</span>'
        self._bridge.log_received.emit(html, message)

    def log_record(self, record: logging.LogRecord, formatted: str):
        """Append a formatted logging record with level-based colour."""
        colour = LEVEL_COLOURS.get(record.levelno, "#cdd6f4")
        stamp = datetime.now().strftime("%H:%M:%S")
        level = record.levelname[0]  # D/I/W/E/C
        html = (f'<span style="color:#6c7086">{stamp}</span> '
                f'<span style="color:{colour}">[{level}] {_escape(formatted)}</span>')
        self._bridge.log_received.emit(html, formatted)

    def clear(self):
        self._text.clear()
        self._line_count = 0

    # ── Internal ──────────────────────────────────────────────────

    def _append(self, html: str, _raw: str):
        self._text.append(html)
        self._line_count += 1

        # Trim if exceeding max lines
        if self._line_count > self.MAX_LINES:
            cursor = self._text.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            cursor.movePosition(QTextCursor.MoveOperation.Down,
                                QTextCursor.MoveMode.KeepAnchor, 500)
            cursor.removeSelectedText()
            self._line_count -= 500

        if self._auto_scroll:
            self._text.moveCursor(QTextCursor.MoveOperation.End)
            self._text.ensureCursorVisible()

    def _toggle_scroll(self):
        self._auto_scroll = not self._auto_scroll
        self._btn_scroll.setText(f"Auto-scroll: {'ON' if self._auto_scroll else 'OFF'}")

    # v7.4.2: collapse/expand control. Hides the text area and shrinks the
    # widget down to the toolbar so it occupies minimal space. Emits
    # ``collapse_toggled`` so the host splitter can relax/restore its
    # minimum-height constraint (otherwise the splitter floor keeps the
    # console at ~180 px even with the text area hidden).
    def set_collapsed(self, collapsed: bool):
        if collapsed == self._collapsed:
            return
        self._collapsed = collapsed
        self._text.setVisible(not collapsed)
        self._btn_collapse.setText("▲ Terminal" if collapsed else "▼ Terminal")
        self._btn_collapse.setToolTip(
            "Expand the terminal" if collapsed else "Collapse the terminal"
        )
        self.collapse_toggled.emit(collapsed)

    def is_collapsed(self) -> bool:
        return self._collapsed

    def _toggle_collapsed(self):
        self.set_collapsed(not self._collapsed)


# ── Qt log handler ────────────────────────────────────────────────

class QtLogHandler(logging.Handler):
    """
    Routes Python ``logging`` records to a :class:`ConsoleLogWidget`.
    Thread-safe: uses the widget's internal signal bridge.
    """

    def __init__(self, console_widget: ConsoleLogWidget):
        super().__init__()
        self.console = console_widget

    def emit(self, record: logging.LogRecord):
        try:
            msg = self.format(record)
            self.console.log_record(record, msg)
        except RuntimeError:
            # Qt signal/widget already destroyed during shutdown — ignore
            pass
        except Exception:
            self.handleError(record)


# ── Helpers ───────────────────────────────────────────────────────

def _escape(text: str) -> str:
    """Minimal HTML escape for log text."""
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
