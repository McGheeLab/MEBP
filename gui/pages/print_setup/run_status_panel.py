"""
run_status_panel.py — Lightweight live "running prints" mirror
(v7.6.0).

Shown in the wizard's right pane on Step 3 (Plan & Run). A read-only
compact mirror of the Print Monitor's live state: a queue list, a
color-coded state badge, and the four progress readouts (job / well /
step / time). The full Print Monitor sub-page remains the detailed
view; this is just an at-a-glance status while the user is still on
the Plan & Run step.

Wiring: the orchestrator pushes summaries via ``add_job`` /
``set_state`` / ``set_progress`` from the same PrintManager callbacks
the monitor consumes — read-only, never re-emitting.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QVBoxLayout, QWidget,
)

from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS


_STATE_COLORS = {
    "IDLE": COLORS["overlay0"],
    "RUNNING": COLORS["green"],
    "PAUSED": COLORS["yellow"],
    "COMPLETED": COLORS["blue"],
    "ABORTED": COLORS["red"],
    "ERROR": COLORS["red"],
}


class RunStatusPanel(QWidget):
    """Compact live status mirror for the Plan & Run step."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("runStatusPanel")
        self.setStyleSheet(
            f"#runStatusPanel {{ background: {COLORS['base']}; }}"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(_s(8), _s(8), _s(8), _s(8))
        lay.setSpacing(_s(8))

        # ── State badge ───────────────────────────────────────────
        state_row = QHBoxLayout()
        state_row.setContentsMargins(0, 0, 0, 0)
        state_lbl = QLabel("State:")
        state_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(10)}pt;")
        state_row.addWidget(state_lbl)
        self._state_badge = QLabel("IDLE")
        self._state_badge.setAlignment(Qt.AlignCenter)
        self._set_state_badge_style("IDLE")
        state_row.addWidget(self._state_badge)
        state_row.addStretch(1)
        lay.addLayout(state_row)

        # ── Progress readouts ─────────────────────────────────────
        self._labels: dict[str, QLabel] = {}
        for key, caption in [
            ("job", "Job"), ("well", "Well"),
            ("step", "Step"), ("time", "Time"),
        ]:
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            cap = QLabel(f"{caption}:")
            cap.setFixedWidth(_s(44))
            cap.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {_sf(9.5)}pt;")
            row.addWidget(cap)
            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-size: {_sf(9.5)}pt;")
            val.setWordWrap(True)
            row.addWidget(val, 1)
            lay.addLayout(row)
            self._labels[key] = val

        # ── Queue list ────────────────────────────────────────────
        q_lbl = QLabel("Queue")
        q_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 700; "
            f"font-size: {_sf(10)}pt; padding-top: {_s(4)}px;")
        lay.addWidget(q_lbl)

        self._queue = QListWidget()
        self._queue.setStyleSheet(
            f"QListWidget {{"
            f"  background: {COLORS['mantle']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {_s(6)}px;"
            f"  font-size: {_sf(9.5)}pt;"
            f"}}"
            f"QListWidget::item {{ padding: {_s(3)}px {_s(6)}px; }}"
        )
        lay.addWidget(self._queue, 1)

        self._empty_hint = QLabel("No prints queued.")
        self._empty_hint.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-style: italic; "
            f"font-size: {_sf(9)}pt;")
        lay.addWidget(self._empty_hint)

    # ── Public update API (read-only mirror) ─────────────────────

    def _set_state_badge_style(self, state: str) -> None:
        color = _STATE_COLORS.get(state.upper(), COLORS["overlay0"])
        self._state_badge.setText(state.upper())
        self._state_badge.setStyleSheet(
            f"background: {color}; color: {COLORS['base']}; "
            f"padding: {_s(2)}px {_s(10)}px; border-radius: {_s(8)}px; "
            f"font-weight: 700; font-size: {_sf(9)}pt; "
            f"letter-spacing: 1px;")

    def set_state(self, state: str) -> None:
        self._set_state_badge_style(str(state))

    def set_progress(self, job: str = "", well: str = "",
                     step: str = "", time: str = "") -> None:
        if job:
            self._labels["job"].setText(job)
        if well:
            self._labels["well"].setText(well)
        if step:
            self._labels["step"].setText(step)
        if time:
            self._labels["time"].setText(time)

    def add_job(self, name: str, detail: str = "") -> None:
        label = name if not detail else f"{name}   ·   {detail}"
        self._queue.addItem(QListWidgetItem(label))
        self._empty_hint.setVisible(self._queue.count() == 0)

    def mark_active(self, name: str) -> None:
        for i in range(self._queue.count()):
            item = self._queue.item(i)
            txt = item.text().lstrip("▶ ").strip()
            if txt.startswith(name):
                item.setText(f"▶ {txt}")
            else:
                item.setText(txt)

    def clear(self) -> None:
        self._queue.clear()
        self._empty_hint.setVisible(True)
        for v in self._labels.values():
            v.setText("—")
        self.set_state("IDLE")
