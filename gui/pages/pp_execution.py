"""
pp_execution.py — Pick & Place Execution monitoring page.

v7.3.3: Runs the operation queue with real-time monitoring, similar
to Print Monitor but for pick-and-place operations.

Layout:
    ┌───────────────────────┬───────────────────────────────┐
    │  Plate Overview       │  Operation Progress           │
    │  (needle pos + marks) │  [Op1 ✓] [Op2 ▶] [Op3 ○]    │
    ├───────────────────────┴───────────────────────────────┤
    │  Progress: ████████░░ 67%  | ETA: 3:45                │
    │  Current: Aspirating 0.0052 µL                        │
    │  [▶ Start] [⏸ Pause] [⏹ Abort]                       │
    └───────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QFrame,
    QPushButton, QLabel, QListWidget, QListWidgetItem,
    QProgressBar, QGroupBox, QSizePolicy, QScrollArea,
)
from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtGui import QColor

from gui.styles import COLORS
from SupportClasses.PickAndPlaceManager import (
    OperationQueue, PickPlaceExecutor, PickPlaceOperation,
    OperationStatus,
)

logger = logging.getLogger(__name__)


# ── Thread-safe signal bridge ────────────────────────────────────

class _PPSignalBridge(QObject):
    """Thread-safe bridge: executor thread → GUI updates."""
    op_started = Signal(object)       # PickPlaceOperation
    op_completed = Signal(object)     # PickPlaceOperation
    op_failed = Signal(object, str)   # PickPlaceOperation, error_msg
    sub_step = Signal(object, str)    # PickPlaceOperation, message
    dwell_tick = Signal(object, float, float)  # op, elapsed, total
    progress = Signal(int, int, str)  # completed, total, message
    execution_done = Signal(bool)     # success


class PPExecutionPage(QWidget):
    """Pick & Place Execution — monitors and controls operation execution."""

    def __init__(self, controller=None, parent=None):
        super().__init__(parent)
        self.controller = controller

        self._queue: Optional[OperationQueue] = None
        self._executor: Optional[PickPlaceExecutor] = None
        self._exec_thread: Optional[threading.Thread] = None
        self._hardware_config = None
        self._running = False

        # Signal bridge for thread safety
        self._bridge = _PPSignalBridge(self)
        self._bridge.op_started.connect(self._on_op_started)
        self._bridge.op_completed.connect(self._on_op_completed)
        self._bridge.op_failed.connect(self._on_op_failed)
        self._bridge.sub_step.connect(self._on_sub_step)
        self._bridge.dwell_tick.connect(self._on_dwell_tick)
        self._bridge.progress.connect(self._on_progress)
        self._bridge.execution_done.connect(self._on_execution_done)

        self._build_ui()

    def _build_ui(self):
        self.setStyleSheet(f"background-color: {COLORS['base']};")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Main content: plate + operations
        splitter = QSplitter(Qt.Horizontal)

        # Left: Plate overview placeholder
        plate_frame = QGroupBox("Plate Overview")
        plate_frame.setStyleSheet(self._group_style())
        plate_layout = QVBoxLayout(plate_frame)
        self._plate_label = QLabel("Plate view will appear here")
        self._plate_label.setAlignment(Qt.AlignCenter)
        self._plate_label.setStyleSheet(f"""
            background-color: {COLORS['surface0']};
            border: 1px solid {COLORS['surface1']};
            border-radius: 4px;
            color: {COLORS['overlay0']};
            min-height: 200px;
        """)
        plate_layout.addWidget(self._plate_label)
        splitter.addWidget(plate_frame)

        # Right: Operation progress list
        ops_frame = QGroupBox("Operations")
        ops_frame.setStyleSheet(self._group_style())
        ops_layout = QVBoxLayout(ops_frame)

        self._ops_list = QListWidget()
        self._ops_list.setStyleSheet(f"""
            QListWidget {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 4px;
                color: {COLORS['text']};
                font-family: Consolas;
            }}
            QListWidget::item {{
                padding: 6px;
                border-bottom: 1px solid {COLORS['surface1']};
            }}
        """)
        ops_layout.addWidget(self._ops_list)
        splitter.addWidget(ops_frame)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)

        # Bottom: Progress + controls
        bottom = QFrame()
        bottom.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px;
            }}
        """)
        bottom_layout = QVBoxLayout(bottom)
        bottom_layout.setContentsMargins(12, 8, 12, 8)

        # Progress bar row
        prog_row = QHBoxLayout()
        self._progress_bar = QProgressBar()
        self._progress_bar.setMinimum(0)
        self._progress_bar.setMaximum(100)
        self._progress_bar.setValue(0)
        self._progress_bar.setMaximumHeight(20)
        prog_row.addWidget(self._progress_bar, 1)

        self._lbl_progress = QLabel("0 / 0")
        self._lbl_progress.setStyleSheet(f"color: {COLORS['text']};")
        prog_row.addWidget(self._lbl_progress)
        bottom_layout.addLayout(prog_row)

        # Status row
        self._lbl_status = QLabel("Idle — send operations from Queue page")
        self._lbl_status.setStyleSheet(f"color: {COLORS['subtext0']};")
        bottom_layout.addWidget(self._lbl_status)

        # Dwell timer row (hidden when not dwelling)
        self._dwell_row = QHBoxLayout()
        self._lbl_dwell = QLabel("")
        self._lbl_dwell.setStyleSheet(f"color: {COLORS['yellow']};")
        self._dwell_bar = QProgressBar()
        self._dwell_bar.setMaximumHeight(14)
        self._dwell_row.addWidget(self._lbl_dwell)
        self._dwell_row.addWidget(self._dwell_bar, 1)
        self._dwell_widget = QWidget()
        self._dwell_widget.setLayout(self._dwell_row)
        self._dwell_widget.setVisible(False)
        bottom_layout.addWidget(self._dwell_widget)

        # Control buttons
        ctrl_row = QHBoxLayout()

        self._btn_start = QPushButton("▶  Start")
        self._btn_start.setStyleSheet(self._green_btn_style())
        self._btn_start.clicked.connect(self._start_execution)
        ctrl_row.addWidget(self._btn_start)

        self._btn_pause = QPushButton("⏸  Pause")
        self._btn_pause.setEnabled(False)
        self._btn_pause.clicked.connect(self._pause_execution)
        ctrl_row.addWidget(self._btn_pause)

        self._btn_resume = QPushButton("▶  Resume")
        self._btn_resume.setEnabled(False)
        self._btn_resume.clicked.connect(self._resume_execution)
        ctrl_row.addWidget(self._btn_resume)

        self._btn_abort = QPushButton("⏹  Abort")
        self._btn_abort.setEnabled(False)
        self._btn_abort.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLORS['red']};
                color: {COLORS['crust']};
                font-weight: bold;
                padding: 8px 16px;
                border-radius: 6px;
            }}
        """)
        self._btn_abort.clicked.connect(self._abort_execution)
        ctrl_row.addWidget(self._btn_abort)

        ctrl_row.addStretch()
        bottom_layout.addLayout(ctrl_row)

        layout.addWidget(bottom)

    # ── Public interface ─────────────────────────────────────────

    def receive_queue(self, queue: OperationQueue):
        """Receive an operation queue (alias for update_queue)."""
        self.update_queue(queue)

    def set_hardware_config(self, config):
        self._hardware_config = config

    def get_page_title(self) -> str:
        return "Execution"

    def get_context_widget(self) -> QWidget:
        """Build context panel showing the live operation queue."""
        if hasattr(self, '_context_widget') and self._context_widget:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Header
        header = QLabel("Operation Queue")
        header.setObjectName("contextSectionLabel")
        layout.addWidget(header)

        # Queue count
        self._ctx_queue_count = QLabel("0 operations")
        self._ctx_queue_count.setStyleSheet(f"color: {COLORS['subtext0']};")
        layout.addWidget(self._ctx_queue_count)

        # Compact queue list
        self._ctx_queue_list = QListWidget()
        self._ctx_queue_list.setStyleSheet(f"""
            QListWidget {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 4px;
                color: {COLORS['text']};
                font-size: 9pt;
            }}
            QListWidget::item {{
                padding: 3px;
                border-bottom: 1px solid {COLORS['surface1']};
            }}
        """)
        layout.addWidget(self._ctx_queue_list, 1)

        layout.addStretch()
        self._context_widget = ctx
        return ctx

    def update_queue(self, queue: OperationQueue):
        """Update the queue display (called continuously from target page)."""
        self._queue = queue
        self._update_ops_list()
        self._update_context_queue()
        n = len(queue) if queue else 0
        if not self._running:
            self._lbl_status.setText(
                f"Ready — {n} operations queued. Press Start.")
            self._lbl_progress.setText(f"0 / {n}")
            self._btn_start.setEnabled(n > 0)

    def set_operation_config(self, op_type, config):
        """Update config on all pending operations when setup changes."""
        if self._queue is None:
            return
        for op in self._queue.operations:
            if op.status.value == "pending":
                op.op_type = op_type
                op.config = config
        self._update_ops_list()
        self._update_context_queue()

    def _update_context_queue(self):
        """Refresh the context panel queue list."""
        ctx_list = getattr(self, '_ctx_queue_list', None)
        ctx_count = getattr(self, '_ctx_queue_count', None)
        if ctx_list is None:
            return
        ctx_list.clear()
        if self._queue is None:
            if ctx_count:
                ctx_count.setText("0 operations")
            return
        for op in self._queue.operations:
            icon = {"spheroid_pickup": "S",
                    "trypsin_cell_pickup": "T",
                    "fluorescent_tagging": "F"}.get(op.op_type.value, "?")
            status = {"pending": "o", "running": ">",
                      "completed": "v", "failed": "x",
                      "skipped": "-"}.get(op.status.value, "?")
            src = op.source_target.target_id
            label = f"[{status}] {icon} {src}"
            if op.dest_target:
                label += f" > {op.dest_target.well_name}"
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, op.op_id)
            ctx_list.addItem(item)
        if ctx_count:
            ctx_count.setText(f"{len(self._queue)} operations")

    def on_status_update(self):
        """Update plate view with current position."""
        pass  # TODO: plate view position update

    # ── Execution control ────────────────────────────────────────

    def _start_execution(self):
        """Start executing the operation queue."""
        if self._queue is None or len(self._queue) == 0:
            logger.warning("No queue to execute")
            return

        if self._running:
            logger.warning("Already running")
            return

        # Create executor
        self._executor = PickPlaceExecutor(
            controller=self.controller,
            hw_config=self._hardware_config,
        )

        # Wire callbacks via signal bridge (thread-safe)
        self._executor.on_op_started = lambda op: self._bridge.op_started.emit(op)
        self._executor.on_op_completed = lambda op: self._bridge.op_completed.emit(op)
        self._executor.on_op_failed = lambda op, msg: self._bridge.op_failed.emit(op, msg)
        self._executor.on_sub_step = lambda op, msg: self._bridge.sub_step.emit(op, msg)
        self._executor.on_dwell_tick = lambda op, e, t: self._bridge.dwell_tick.emit(op, e, t)

        # Set Z parameters from settings/calibration
        # TODO: wire from calibration data
        self._executor.safe_z_mm = 5.0
        self._executor.intra_well_retract_mm = 1.0

        self._running = True
        self._btn_start.setEnabled(False)
        self._btn_pause.setEnabled(True)
        self._btn_abort.setEnabled(True)
        self._lbl_status.setText("Running...")

        # Start in daemon thread
        def _run():
            try:
                success = self._executor.execute_queue(
                    self._queue,
                    on_progress=lambda c, t, m: self._bridge.progress.emit(c, t, m),
                )
                self._bridge.execution_done.emit(success)
            except Exception as e:
                logger.error(f"Execution error: {e}", exc_info=True)
                self._bridge.execution_done.emit(False)

        self._exec_thread = threading.Thread(target=_run, daemon=True)
        self._exec_thread.start()
        logger.info("Execution started")

    def _pause_execution(self):
        if self._executor:
            self._executor.pause()
            self._btn_pause.setEnabled(False)
            self._btn_resume.setEnabled(True)
            self._lbl_status.setText("Paused")

    def _resume_execution(self):
        if self._executor:
            self._executor.resume()
            self._btn_pause.setEnabled(True)
            self._btn_resume.setEnabled(False)
            self._lbl_status.setText("Running...")

    def _abort_execution(self):
        if self._executor:
            self._executor.abort()
            self._lbl_status.setText("Aborting...")

    # ── Signal handlers (main thread) ────────────────────────────

    def _on_op_started(self, op: PickPlaceOperation):
        self._update_ops_list()
        self._lbl_status.setText(
            f"Running: {op.op_type.value} on {op.source_target.target_id}")

    def _on_op_completed(self, op: PickPlaceOperation):
        self._update_ops_list()
        self._dwell_widget.setVisible(False)

    def _on_op_failed(self, op: PickPlaceOperation, msg: str):
        self._update_ops_list()
        self._lbl_status.setText(
            f"Failed: {op.op_id} — {msg}")
        self._dwell_widget.setVisible(False)

    def _on_sub_step(self, op: PickPlaceOperation, msg: str):
        self._lbl_status.setText(msg)
        # Update the running item in the list
        for i in range(self._ops_list.count()):
            item = self._ops_list.item(i)
            if item and item.data(Qt.UserRole) == op.op_id:
                self._ops_list.item(i).setText(
                    f"▶ {op.op_type.value}: {op.source_target.target_id} — {msg}")
                break

    def _on_dwell_tick(self, op: PickPlaceOperation, elapsed: float, total: float):
        self._dwell_widget.setVisible(True)
        remaining = max(0, total - elapsed)
        self._lbl_dwell.setText(f"Dwell: {remaining:.0f}s remaining")
        self._dwell_bar.setMaximum(int(total))
        self._dwell_bar.setValue(int(elapsed))

    def _on_progress(self, completed: int, total: int, msg: str):
        if total > 0:
            pct = int(100 * completed / total)
            self._progress_bar.setValue(pct)
        self._lbl_progress.setText(f"{completed} / {total}")

    def _on_execution_done(self, success: bool):
        self._running = False
        self._btn_start.setEnabled(True)
        self._btn_pause.setEnabled(False)
        self._btn_resume.setEnabled(False)
        self._btn_abort.setEnabled(False)
        self._dwell_widget.setVisible(False)

        if success:
            self._lbl_status.setText("All operations completed successfully")
            self._lbl_status.setStyleSheet(f"color: {COLORS['green']};")
        else:
            self._lbl_status.setText("Execution stopped (aborted or error)")
            self._lbl_status.setStyleSheet(f"color: {COLORS['red']};")

        self._update_ops_list()
        logger.info(f"Execution done, success={success}")

    # ── UI helpers ───────────────────────────────────────────────

    def _update_ops_list(self):
        """Rebuild the operations list with current statuses."""
        self._ops_list.clear()
        if self._queue is None:
            return

        for op in self._queue.operations:
            status_map = {
                OperationStatus.PENDING: ("○", COLORS["subtext0"]),
                OperationStatus.RUNNING: ("▶", COLORS["blue"]),
                OperationStatus.COMPLETED: ("✓", COLORS["green"]),
                OperationStatus.FAILED: ("✗", COLORS["red"]),
                OperationStatus.SKIPPED: ("—", COLORS["overlay0"]),
            }
            icon, color = status_map.get(op.status, ("?", COLORS["text"]))
            src = op.source_target.target_id
            dst = op.dest_target.target_id if op.dest_target else "—"
            label = f"{icon}  {op.op_type.value}: {src} → {dst}"
            if op.sub_step and op.status == OperationStatus.RUNNING:
                label += f"  ({op.sub_step})"
            if op.error_msg:
                label += f"  [{op.error_msg}]"

            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, op.op_id)
            item.setForeground(QColor(color))
            self._ops_list.addItem(item)

    def _group_style(self) -> str:
        return f"""
            QGroupBox {{
                border: 1px solid {COLORS['surface1']};
                border-radius: 8px;
                color: {COLORS['text']};
                font-size: 11pt;
                font-weight: 600;
                margin-top: 12px;
                padding-top: 16px;
            }}
            QGroupBox::title {{
                color: {COLORS['blue']};
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }}
        """

    def _green_btn_style(self) -> str:
        return f"""
            QPushButton {{
                background-color: {COLORS['green']};
                color: {COLORS['crust']};
                font-weight: bold;
                padding: 8px 16px;
                border-radius: 6px;
            }}
            QPushButton:hover {{
                background-color: {COLORS['blue']};
            }}
            QPushButton:disabled {{
                background-color: {COLORS['surface1']};
                color: {COLORS['overlay0']};
            }}
        """
