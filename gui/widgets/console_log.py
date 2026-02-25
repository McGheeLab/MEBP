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

    def __init__(self, parent=None):
        super().__init__(parent)
        self._auto_scroll = True
        self._line_count = 0
        self._bridge = _LogSignalBridge()
        self._bridge.log_received.connect(self._append)

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Toolbar
        toolbar = QHBoxLayout()
        toolbar.setSpacing(4)

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
        except Exception:
            self.handleError(record)


# ── Helpers ───────────────────────────────────────────────────────

def _escape(text: str) -> str:
    """Minimal HTML escape for log text."""
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
