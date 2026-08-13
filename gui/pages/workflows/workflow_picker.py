"""workflow_picker.py — Workflows mode landing page.

v7.4.3: Grid of clickable tiles, one per workflow. The active workflow's
ID is emitted via `workflow_selected(workflow_id: str)`. The parent
container (`WorkflowsModePage`) maps that ID to a sub-page in its
QStackedWidget.

Workflows:
    spheroid_pickup       — Spheroid Pick & Place (functional)
    cell_targeting        — Cell Targeting & Removal (functional in v7.5.x)
    cell_labeling         — Cell Labeling / staining (functional in v7.5.x)
    quick_print           — Quick Print (functional)
    immuno                — Immuno (stub)
"""

from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel, QPushButton, QSizePolicy,
    QFrame, QHBoxLayout,
)

from gui.styles import COLORS
from gui.scaling import s, sf, sp


@dataclass(frozen=True)
class WorkflowTile:
    workflow_id: str
    icon: str
    title: str
    description: str
    enabled: bool


# Order here drives the grid layout. Keep enabled workflows first.
WORKFLOWS: tuple[WorkflowTile, ...] = (
    WorkflowTile(
        workflow_id="spheroid_pickup",
        icon="🧫",
        title="Spheroid Pick & Place",
        description="Extract spheroids from picked locations and deposit at a chosen place target.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="cell_targeting",
        icon="🎯",
        title="Cell Targeting & Removal",
        description="Load a cell-release reagent, trypsinize cells in place, "
                    "then extract and relocate them.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="fluorescence_mosaic",
        icon="🔬",
        title="Fluorescence Mosaic",
        description="High-resolution multi-channel mosaic of one well "
                    "(DAPI / FITC / mCherry / Cy5). Overlays into other workflows.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="cell_labeling",
        icon="🏷️",
        title="Cell Labeling",
        description="Select regions and a stain, deposit it slowly, incubate "
                    "for a set time, then aspirate it back off to waste.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="quick_print",
        icon="⚡",
        title="Quick Print",
        description="Drop one object in a well and print — no setup. Use it to "
                    "dial in printing conditions before a full-plate run.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="full_print",
        icon="🖨️",
        title="Full Print",
        description="The complete print workflow — set up objects and wells, "
                    "run the print with live monitoring, then review results.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="lablink",
        icon="🔗",
        title="LabLink Processing",
        description="Send scans, mosaics, images and video to a LabLink hub for "
                    "an ND2Studios recipe — deconvolution, segmentation — and "
                    "collect the results.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="immuno",
        icon="🧪",
        title="Immuno",
        description="Multi-step immunostaining wash + incubation cycles.",
        enabled=False,
    ),
    WorkflowTile(
        workflow_id="incubator",
        icon="🌡️",
        title="Incubator",
        description="Hold the two-zone incubator stage at temperature — "
                    "setpoints, watchdog-safe ramps, PID, sensor calibration "
                    "and a live trend.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="stress_test",
        icon="🔁",
        title="ZP Stress Test",
        description="Bench-validate the ZP board: loop prints + jog/travel "
                    "under load and watch for any disconnect or reset.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="timing_calibration",
        icon="⏱️",
        title="XY↔ZP Timing Calibration",
        description="Measure how far the needle (XY) lags the commands over a "
                    "1–5 min run, so the pump stays synced to the needle.",
        enabled=True,
    ),
    WorkflowTile(
        workflow_id="common_print_settings",
        icon="⚙️",
        title="Common Print Settings",
        description="Settings shared by every workflow — pump dwell, pressure "
                    "relief, prime, and the needle-prep defaults workflows "
                    "inherit.",
        enabled=True,
    ),
)


class _TileButton(QFrame):
    """Clickable tile rendering one workflow option."""

    clicked = Signal(str)  # workflow_id

    def __init__(self, tile: WorkflowTile, parent: QWidget | None = None):
        super().__init__(parent)
        self._tile = tile
        self.setObjectName("workflowTile")
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(s(220), s(140))
        self._apply_style(hovered=False)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(14), s(12), s(14), s(12))
        outer.setSpacing(s(6))

        # Icon + title row
        head = QHBoxLayout()
        head.setSpacing(s(8))
        icon = QLabel(tile.icon)
        icon.setStyleSheet(
            f"font-size: {sf(20)}pt;"
            f"font-family: 'Apple Color Emoji', 'Segoe UI Emoji', "
            f"'Noto Color Emoji', sans-serif;"
        )
        head.addWidget(icon)

        title = QLabel(tile.title)
        title.setStyleSheet(
            f"color: {COLORS['text']};"
            f"font-size: {sf(12)}pt;"
            f"font-weight: 600;"
        )
        title.setWordWrap(True)
        head.addWidget(title, stretch=1)
        outer.addLayout(head)

        # Description
        desc = QLabel(tile.description)
        desc.setWordWrap(True)
        desc.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(9)}pt;"
        )
        outer.addWidget(desc, stretch=1)

        # "Coming soon" badge for disabled tiles
        if not tile.enabled:
            badge = QLabel("Coming soon")
            badge.setAlignment(Qt.AlignLeft)
            badge.setStyleSheet(
                f"color: {COLORS['peach']};"
                f"background-color: {COLORS['surface1']};"
                f"border-radius: {sp(6)};"
                f"padding: {sp(2)} {sp(8)};"
                f"font-size: {sf(8)}pt;"
                f"font-weight: 600;"
            )
            badge.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
            outer.addWidget(badge, alignment=Qt.AlignLeft)

    def _apply_style(self, hovered: bool):
        border = COLORS["blue"] if hovered else COLORS["surface1"]
        bg = COLORS["surface1"] if hovered else COLORS["surface0"]
        self.setStyleSheet(
            f"QFrame#workflowTile {{"
            f"  background-color: {bg};"
            f"  border: 1px solid {border};"
            f"  border-radius: {sp(10)};"
            f"}}"
        )

    def enterEvent(self, event):
        self._apply_style(hovered=True)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self._apply_style(hovered=False)
        super().leaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self._tile.workflow_id)
        super().mousePressEvent(event)


class WorkflowPickerPage(QWidget):
    """Grid of workflow tiles. Emits `workflow_selected(workflow_id)`."""

    workflow_selected = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(24), s(20), s(24), s(20))
        outer.setSpacing(s(16))

        # Heading
        heading = QLabel("Choose a workflow")
        heading.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(16)}pt;"
            f"font-weight: 600;"
        )
        outer.addWidget(heading)

        subhead = QLabel(
            "Each workflow combines a set of tools to complete a specific task. "
            "Tools are reused across workflows where it makes sense."
        )
        subhead.setWordWrap(True)
        subhead.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(10)}pt;"
        )
        outer.addWidget(subhead)

        # Tile grid — 3 columns
        grid = QGridLayout()
        grid.setHorizontalSpacing(s(14))
        grid.setVerticalSpacing(s(14))
        cols = 3
        for i, tile in enumerate(WORKFLOWS):
            row, col = divmod(i, cols)
            btn = _TileButton(tile)
            btn.clicked.connect(self.workflow_selected.emit)
            grid.addWidget(btn, row, col)
        # Even column stretch
        for c in range(cols):
            grid.setColumnStretch(c, 1)
        outer.addLayout(grid)
        outer.addStretch(1)
