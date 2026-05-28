"""
validation_panel.py — Slide-up validation panel for the Print Setup
wizard.

When the user presses "Validate" the orchestrator runs each step's
``validate()`` method, aggregates the issues, and pushes them into the
``ValidationPanel`` which animates open from the bottom. Each row has
a "Fix →" button that emits ``jump_requested(step_index, target_id)``
so the orchestrator can switch the active step and focus the relevant
sub-widget.

The panel is not modal — the user can interact with the rest of the
wizard while it is open, and dismiss it with the chevron.
"""

from __future__ import annotations

from PySide6.QtCore import (
    Qt, Signal, QPropertyAnimation, QEasingCurve,
)
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QScrollArea, QSizePolicy,
    QVBoxLayout, QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS

from .validation import ValidationIssue, ValidationSeverity


_BADGE_BG = {
    ValidationSeverity.ERROR: COLORS["red"],
    ValidationSeverity.WARN: COLORS["yellow"],
    ValidationSeverity.INFO: COLORS["blue"],
}


class _IssueRow(QFrame):
    """One issue row: severity badge + title + detail + Fix button."""

    fix_clicked = Signal(int, str)   # step_index, target_id

    def __init__(self, issue: ValidationIssue, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("validationIssueRow")
        self.setStyleSheet(
            f"#validationIssueRow {{"
            f"  background: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {_s(6)}px;"
            f"}}"
        )

        row = QHBoxLayout(self)
        row.setContentsMargins(_s(10), _s(8), _s(10), _s(8))
        row.setSpacing(_s(10))

        badge = QLabel(issue.severity.value.upper())
        badge.setStyleSheet(
            f"background: {_BADGE_BG.get(issue.severity, COLORS['overlay0'])};"
            f"color: {COLORS['base']};"
            f"padding: {_s(2)}px {_s(8)}px;"
            f"border-radius: {_s(8)}px;"
            f"font-weight: 700;"
            f"font-size: {_sf(8.5)}pt;"
            f"letter-spacing: 1px;"
        )
        badge.setAlignment(Qt.AlignCenter)
        row.addWidget(badge, 0, Qt.AlignTop)

        text_col = QVBoxLayout()
        text_col.setSpacing(_s(2))
        title = QLabel(issue.title)
        title.setStyleSheet(
            f"color: {COLORS['text']}; "
            f"font-size: {_sf(11)}pt; font-weight: 600;"
        )
        title.setWordWrap(True)
        text_col.addWidget(title)
        if issue.detail:
            detail = QLabel(issue.detail)
            detail.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(10)}pt;"
            )
            detail.setWordWrap(True)
            text_col.addWidget(detail)
        row.addLayout(text_col, 1)

        fix_btn = QPushButton("Fix →")
        fix_btn.setCursor(Qt.PointingHandCursor)
        fix_btn.setStyleSheet(
            f"QPushButton {{"
            f"  background: {COLORS['surface1']};"
            f"  color: {COLORS['mauve']};"
            f"  border: 1px solid {COLORS['surface2']};"
            f"  border-radius: {_s(6)}px;"
            f"  padding: {_s(4)}px {_s(10)}px;"
            f"  font-weight: 600;"
            f"  font-size: {_sf(10)}pt;"
            f"}}"
            f"QPushButton:hover {{ background: {COLORS['surface2']}; }}"
        )
        fix_btn.clicked.connect(
            lambda _, s=issue.step, t=issue.target_id:
            self.fix_clicked.emit(s, t)
        )
        row.addWidget(fix_btn, 0, Qt.AlignTop)


class ValidationPanel(QFrame):
    """Slide-up Card showing aggregated validation issues.

    Public API:
        set_issues(list[ValidationIssue])  — replace the content
        show_animated()                    — slide up
        hide_animated()                    — slide down
        toggle_animated()
        is_open() -> bool
        jump_requested(step, target_id)    — Signal
    """

    PANEL_HEIGHT = 260   # px (pre-scale)

    jump_requested = Signal(int, str)
    closed = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("validationPanel")
        self.setStyleSheet(
            f"#validationPanel {{"
            f"  background: {COLORS['mantle']};"
            f"  border-top: 1px solid {COLORS['surface1']};"
            f"}}"
        )
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMaximumHeight(0)
        self._is_open = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(_s(16), _s(8), _s(16), _s(8))
        outer.setSpacing(_s(6))

        # ── Header ────────────────────────────────────────────────
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        self._summary_lbl = QLabel("No issues")
        self._summary_lbl.setStyleSheet(
            f"color: {COLORS['text']}; "
            f"font-size: {_sf(12)}pt; font-weight: 700;"
        )
        header.addWidget(self._summary_lbl)
        header.addStretch(1)

        close_btn = QPushButton("Hide")
        close_btn.setCursor(Qt.PointingHandCursor)
        close_btn.setStyleSheet(
            f"QPushButton {{"
            f"  background: transparent; color: {COLORS['subtext0']};"
            f"  border: none; padding: {_s(4)}px {_s(8)}px;"
            f"  font-size: {_sf(10)}pt;"
            f"}}"
            f"QPushButton:hover {{ color: {COLORS['text']}; }}"
        )
        close_btn.clicked.connect(self.hide_animated)
        header.addWidget(close_btn)
        outer.addLayout(header)

        # ── Scrollable issue list ─────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(
            f"QScrollArea {{ background: transparent; }}"
        )
        self._list_host = QWidget()
        self._list_layout = QVBoxLayout(self._list_host)
        self._list_layout.setContentsMargins(0, 0, 0, 0)
        self._list_layout.setSpacing(_s(6))
        self._list_layout.addStretch(1)
        scroll.setWidget(self._list_host)
        outer.addWidget(scroll, 1)

        self._animation = QPropertyAnimation(self, b"maximumHeight")
        self._animation.setDuration(160)
        self._animation.setEasingCurve(QEasingCurve.OutCubic)

    # ── Public ────────────────────────────────────────────────────

    def set_issues(self, issues: list[ValidationIssue]) -> None:
        # Clear existing rows (leave the trailing stretch in place)
        while self._list_layout.count() > 1:
            item = self._list_layout.takeAt(0)
            if item is None:
                continue
            w = item.widget()
            if w is not None:
                w.deleteLater()

        errors = sum(1 for i in issues if i.is_error)
        warns = sum(1 for i in issues if i.is_warn)
        if not issues:
            self._summary_lbl.setText("All checks passed")
            self._summary_lbl.setStyleSheet(
                f"color: {COLORS['green']}; "
                f"font-size: {_sf(12)}pt; font-weight: 700;"
            )
        else:
            parts = []
            if errors:
                parts.append(f"{errors} error{'' if errors == 1 else 's'}")
            if warns:
                parts.append(f"{warns} warning{'' if warns == 1 else 's'}")
            info = len(issues) - errors - warns
            if info:
                parts.append(f"{info} info")
            self._summary_lbl.setText(" · ".join(parts) or "No issues")
            self._summary_lbl.setStyleSheet(
                f"color: {COLORS['red'] if errors else COLORS['yellow']}; "
                f"font-size: {_sf(12)}pt; font-weight: 700;"
            )

        for issue in issues:
            row = _IssueRow(issue, self._list_host)
            row.fix_clicked.connect(self.jump_requested.emit)
            self._list_layout.insertWidget(self._list_layout.count() - 1, row)

    def is_open(self) -> bool:
        return self._is_open

    def show_animated(self) -> None:
        self._animation.stop()
        self._animation.setStartValue(self.maximumHeight())
        self._animation.setEndValue(_s(self.PANEL_HEIGHT))
        self._animation.start()
        self._is_open = True

    def hide_animated(self) -> None:
        self._animation.stop()
        self._animation.setStartValue(self.maximumHeight())
        self._animation.setEndValue(0)
        self._animation.start()
        self._is_open = False
        self.closed.emit()

    def toggle_animated(self) -> None:
        if self._is_open:
            self.hide_animated()
        else:
            self.show_animated()
