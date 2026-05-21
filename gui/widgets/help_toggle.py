"""
help_toggle.py — Top-bar Help toggle button (v7.4.0-c).

Checkable QPushButton that flips MainWindow._help_mode. Registered
FormRow widgets receive the change via MainWindow.help_mode_changed
and reveal/hide their inline help text.

Beginners leave it on; daily users keep it off so the UI is dense.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QPushButton

from gui.styles import COLORS
from gui.scaling import s, sf


class HelpToggle(QPushButton):
    """Top-bar Help toggle. Emits :attr:`toggled` (inherited from QPushButton)."""

    def __init__(self, parent=None):
        super().__init__("? Help", parent)
        self.setObjectName("helpToggleBtn")
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedHeight(s(28))
        self.setToolTip(
            "Toggle inline help text for form fields throughout the app.")
        self._apply_style()
        self.toggled.connect(self._apply_style)

    def _apply_style(self, *_args):
        if self.isChecked():
            bg = COLORS['mauve']
            fg = COLORS['base']
            border = COLORS['mauve']
        else:
            bg = COLORS['surface0']
            fg = COLORS['text']
            border = COLORS['surface1']
        self.setStyleSheet(
            f"QPushButton#helpToggleBtn {{"
            f"  background-color: {bg};"
            f"  color: {fg};"
            f"  border: 1px solid {border};"
            f"  border-radius: {s(6)}px;"
            f"  padding: 0 {s(10)}px;"
            f"  font-size: {sf(9.5)}pt;"
            f"  font-weight: 600;"
            f"}}"
            f"QPushButton#helpToggleBtn:hover {{"
            f"  background-color: {COLORS['surface1']};"
            f"  color: {COLORS['text']};"
            f"}}"
        )
